"""基于 Livox 静态结构关键帧的稀疏平移重定位。

输入点必须来自参赛方定位节点发布的已配准 Livox 点云。本模块只估计平面
平移候选，不发布 ROS 消息，也不修改定位；调用方必须经过静止、质量和定位
节点事务门控后才能应用候选。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class SparseRelocalizationConfig:
    enabled: bool = True
    anchor_radius: float = 12.0
    voxel_size: float = 0.10
    minimum_points: int = 120
    maximum_points: int = 2400
    minimum_correction: float = 0.08
    maximum_correction: float = 1.20
    coarse_step: float = 0.10
    minimum_overlap: float = 0.55
    maximum_residual_p70: float = 0.18
    minimum_uniqueness_margin: float = 0.08
    required_frames: int = 3
    maximum_frame_spread: float = 0.08
    maximum_linear_speed: float = 0.08
    maximum_angular_speed: float = 0.10
    correction_timeout: float = 2.0

    def __post_init__(self) -> None:
        positive = (
            self.anchor_radius,
            self.voxel_size,
            self.minimum_correction,
            self.maximum_correction,
            self.coarse_step,
            self.maximum_residual_p70,
            self.maximum_frame_spread,
            self.maximum_linear_speed,
            self.maximum_angular_speed,
            self.correction_timeout,
        )
        if (
            not all(math.isfinite(value) and value > 0.0 for value in positive)
            or self.minimum_points < 20
            or self.maximum_points < self.minimum_points
            or self.required_frames < 2
            or self.minimum_correction >= self.maximum_correction
            or not 0.0 < self.minimum_overlap <= 1.0
            or not 0.0 <= self.minimum_uniqueness_margin <= 1.0
        ):
            raise ValueError("稀疏地标重定位参数必须为有限正值且范围有效")


@dataclass(frozen=True)
class SparseLandmarkAnchor:
    landmark_id: str
    floor: int
    world_xy: tuple[float, float]
    outward_yaw: float
    width: float
    points: np.ndarray


@dataclass(frozen=True)
class SparseRelocalizationEvaluation:
    status: str
    correction_xy: tuple[float, float] | None = None
    correction_norm: float | None = None
    overlap: float = 0.0
    residual_p70: float | None = None
    uniqueness_margin: float = 0.0
    frame_spread: float | None = None
    point_count: int = 0

    @property
    def accepted(self) -> bool:
        return self.status in {"accepted", "no_correction_needed"}


@dataclass(frozen=True)
class LandmarkRelocalizationOutcome:
    """任务状态机可见的有界纠偏结果。"""

    status: str
    applied: bool = False
    reason: str | None = None


class SparseLandmarkRelocalizer:
    """维护少量冻结关键帧，并以平移搜索复核当前 Livox 静态结构。"""

    def __init__(self, config: SparseRelocalizationConfig):
        self.config = config
        self.anchors: dict[str, SparseLandmarkAnchor] = {}

    def freeze_anchor(
        self,
        landmark_id: str,
        *,
        floor: int,
        world_xy: tuple[float, float],
        outward_yaw: float,
        width: float,
        frames: Iterable[Iterable[tuple[float, float, float]]],
        floor_height: float,
    ) -> bool:
        """冻结地标首次有效关键帧；语义几何刷新不得覆盖。"""
        key = str(landmark_id).strip()
        if not self.config.enabled or not key or key in self.anchors:
            return False
        values = (*world_xy, outward_yaw, width, floor_height)
        if (
            not all(math.isfinite(float(value)) for value in values)
            or float(width) <= 0.0
            or float(floor_height) <= 0.0
        ):
            return False
        selected = [
            self._filter_frame(
                frame,
                floor=int(floor),
                center=world_xy,
                outward_yaw=float(outward_yaw),
                width=float(width),
                floor_height=float(floor_height),
            )
            for frame in frames
        ]
        selected = [frame for frame in selected if len(frame) >= self.config.minimum_points]
        if len(selected) < self.config.required_frames:
            return False
        points = self._voxelize(np.concatenate(selected[-self.config.required_frames :], axis=0))
        if len(points) < self.config.minimum_points:
            return False
        self.anchors[key] = SparseLandmarkAnchor(
            landmark_id=key,
            floor=int(floor),
            world_xy=(float(world_xy[0]), float(world_xy[1])),
            outward_yaw=float(outward_yaw),
            width=float(width),
            points=points,
        )
        return True

    def evaluate(
        self,
        landmark_id: str,
        *,
        frames: Iterable[Iterable[tuple[float, float, float]]],
        floor_height: float,
    ) -> SparseRelocalizationEvaluation:
        if not self.config.enabled:
            return SparseRelocalizationEvaluation("disabled")
        anchor = self.anchors.get(str(landmark_id).strip())
        if anchor is None:
            return SparseRelocalizationEvaluation("anchor_unavailable")
        filtered = [
            self._filter_frame(
                frame,
                floor=anchor.floor,
                center=anchor.world_xy,
                outward_yaw=anchor.outward_yaw,
                width=anchor.width,
                floor_height=float(floor_height),
            )
            for frame in frames
        ]
        filtered = [frame for frame in filtered if len(frame) >= self.config.minimum_points]
        if len(filtered) < self.config.required_frames:
            return SparseRelocalizationEvaluation("insufficient_frames")

        per_frame = [self._match(anchor.points, frame) for frame in filtered[-self.config.required_frames :]]
        usable = [item for item in per_frame if item is not None]
        if len(usable) < self.config.required_frames:
            return SparseRelocalizationEvaluation("match_unavailable")
        corrections = np.asarray([item[0] for item in usable], dtype=float)
        median_correction = np.median(corrections, axis=0)
        spread = float(np.max(np.linalg.norm(corrections - median_correction, axis=1)))

        aggregate = self._voxelize(np.concatenate(filtered[-self.config.required_frames :], axis=0))
        match = self._match(anchor.points, aggregate)
        if match is None:
            return SparseRelocalizationEvaluation("match_unavailable")
        correction, overlap, residual_p70, uniqueness = match
        correction_norm = float(np.linalg.norm(correction))
        common = dict(
            correction_xy=(float(correction[0]), float(correction[1])),
            correction_norm=correction_norm,
            overlap=float(overlap),
            residual_p70=float(residual_p70),
            uniqueness_margin=float(uniqueness),
            frame_spread=spread,
            point_count=int(len(aggregate)),
        )
        if correction_norm > self.config.maximum_correction:
            return SparseRelocalizationEvaluation("correction_exceeded", **common)
        if spread > self.config.maximum_frame_spread:
            return SparseRelocalizationEvaluation("frame_spread_exceeded", **common)
        if overlap < self.config.minimum_overlap:
            return SparseRelocalizationEvaluation("overlap_too_low", **common)
        if residual_p70 > self.config.maximum_residual_p70:
            return SparseRelocalizationEvaluation("residual_too_high", **common)
        # 小于注入死区时不会改变定位；只要重叠和残差已证明仍在原锚点，
        # 无需让粗栅格搜索的相邻平台区把安全的 no-op 误判成歧义。
        if correction_norm < self.config.minimum_correction:
            return SparseRelocalizationEvaluation("no_correction_needed", **common)
        if uniqueness < self.config.minimum_uniqueness_margin:
            return SparseRelocalizationEvaluation("ambiguous", **common)
        return SparseRelocalizationEvaluation("accepted", **common)

    def _filter_frame(
        self,
        points,
        *,
        floor: int,
        center: tuple[float, float],
        outward_yaw: float,
        width: float,
        floor_height: float,
    ) -> np.ndarray:
        array = np.asarray(tuple(points), dtype=float)
        if array.size == 0:
            return np.empty((0, 3), dtype=float)
        if array.ndim != 2 or array.shape[1] != 3:
            return np.empty((0, 3), dtype=float)
        finite = np.all(np.isfinite(array), axis=1)
        floor_z = int(floor) * float(floor_height)
        relative_z = array[:, 2] - floor_z
        dx = array[:, 0] - float(center[0])
        dy = array[:, 1] - float(center[1])
        radial = np.hypot(dx, dy) <= self.config.anchor_radius
        structural_height = (relative_z > 0.22) & (relative_z < 1.90)

        normal = dx * math.cos(outward_yaw) + dy * math.sin(outward_yaw)
        tangent = -dx * math.sin(outward_yaw) + dy * math.cos(outward_yaw)
        dynamic_door = (np.abs(normal) <= 0.70) & (
            np.abs(tangent) <= 0.5 * float(width) + 0.40
        )
        return self._voxelize(array[finite & radial & structural_height & ~dynamic_door])

    def _voxelize(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return np.empty((0, 3), dtype=float)
        keys = np.floor(points / self.config.voxel_size).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        selected = points[np.sort(indices)]
        if len(selected) <= self.config.maximum_points:
            return selected
        stride = int(math.ceil(len(selected) / self.config.maximum_points))
        return selected[::stride][: self.config.maximum_points]

    def _match(
        self,
        anchor_points: np.ndarray,
        current_points: np.ndarray,
    ) -> tuple[np.ndarray, float, float, float] | None:
        if (
            len(anchor_points) < self.config.minimum_points
            or len(current_points) < self.config.minimum_points
        ):
            return None
        voxel = self.config.voxel_size
        anchor_keys = np.floor(anchor_points / voxel).astype(np.int64)
        current_keys = np.floor(current_points / voxel).astype(np.int64)
        maximum_cells = int(math.floor(self.config.maximum_correction / self.config.coarse_step))
        scale = self.config.coarse_step / voxel
        maximum_voxel_shift = int(round(maximum_cells * scale))
        expanded_anchor = np.concatenate(
            [
                anchor_keys + np.asarray((ox, oy, 0), dtype=np.int64)
                for ox in (-1, 0, 1)
                for oy in (-1, 0, 1)
            ],
            axis=0,
        )
        minimum_key = np.minimum(
            np.min(expanded_anchor, axis=0),
            np.min(current_keys, axis=0)
            - np.asarray((maximum_voxel_shift, maximum_voxel_shift, 0)),
        )
        maximum_key = np.maximum(
            np.max(expanded_anchor, axis=0),
            np.max(current_keys, axis=0)
            + np.asarray((maximum_voxel_shift, maximum_voxel_shift, 0)),
        )
        shape = maximum_key - minimum_key + 1
        if np.prod(shape, dtype=np.int64) > 20_000_000:
            return None
        occupancy = np.zeros(tuple(int(value) for value in shape), dtype=bool)
        anchor_indices = expanded_anchor - minimum_key
        occupancy[
            anchor_indices[:, 0], anchor_indices[:, 1], anchor_indices[:, 2]
        ] = True

        scores: list[tuple[float, int, int]] = []
        denominator = max(1, min(len(anchor_keys), len(current_keys)))
        for ix in range(-maximum_cells, maximum_cells + 1):
            shift_x = int(round(ix * scale))
            for iy in range(-maximum_cells, maximum_cells + 1):
                shift_y = int(round(iy * scale))
                indices = current_keys - minimum_key
                indices[:, 0] += shift_x
                indices[:, 1] += shift_y
                matches = int(
                    np.count_nonzero(
                        occupancy[indices[:, 0], indices[:, 1], indices[:, 2]]
                    )
                )
                scores.append((min(1.0, matches / denominator), ix, iy))
        scores.sort(reverse=True)
        best_score, best_x, best_y = scores[0]
        second_score = max(
            (
                score
                for score, ix, iy in scores[1:]
                if math.hypot(ix - best_x, iy - best_y) >= 2.0
            ),
            default=0.0,
        )
        correction = np.asarray(
            [best_x * self.config.coarse_step, best_y * self.config.coarse_step],
            dtype=float,
        )

        # 粗搜索只负责跨越局部极小值，随后用对应点中位残差细化平移。
        for _ in range(5):
            residuals, distances = self._nearest_residuals(
                anchor_points,
                current_points,
                correction,
                maximum_distance=0.35,
            )
            if len(residuals) < self.config.minimum_points:
                break
            step = np.median(residuals, axis=0)
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.config.coarse_step:
                step *= self.config.coarse_step / step_norm
            correction += step
            if float(np.linalg.norm(step)) < 0.005:
                break

        _residuals, distances = self._nearest_residuals(
            anchor_points,
            current_points,
            correction,
            maximum_distance=0.50,
        )
        if len(distances) < self.config.minimum_points:
            return None
        overlap = min(1.0, float(np.count_nonzero(distances <= 0.15)) / denominator)
        residual_p70 = float(np.quantile(distances, 0.70))
        return correction, overlap, residual_p70, float(best_score - second_score)

    @staticmethod
    def _nearest_residuals(
        anchor: np.ndarray,
        current: np.ndarray,
        correction: np.ndarray,
        *,
        maximum_distance: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        shifted = np.array(current, dtype=float, copy=True)
        shifted[:, :2] += correction
        residuals = []
        distances = []
        for start in range(0, len(shifted), 192):
            batch = shifted[start : start + 192]
            delta = anchor[None, :, :] - batch[:, None, :]
            planar_squared = np.sum(delta[:, :, :2] ** 2, axis=2)
            planar_squared[np.abs(delta[:, :, 2]) > 0.16] = np.inf
            nearest = np.argmin(planar_squared, axis=1)
            nearest_squared = planar_squared[np.arange(len(batch)), nearest]
            valid = np.isfinite(nearest_squared) & (
                nearest_squared <= float(maximum_distance) ** 2
            )
            if np.any(valid):
                chosen = delta[np.arange(len(batch)), nearest, :2]
                residuals.extend(chosen[valid])
                distances.extend(np.sqrt(nearest_squared[valid]))
        return np.asarray(residuals, dtype=float), np.asarray(distances, dtype=float)

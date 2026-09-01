"""利用固定主入口几何估计有限的平面平移纠偏候选。

本模块只消费已经由参赛方定位节点注册到世界坐标的 Livox 点云，不读取真值、
场景文件或运行时建筑 metadata。入口合同仅描述比赛中不随楼层布局变化的外墙
缺口：中心在 ``(0, 0)``、外墙法向指向 ``+y``、门宽为 2 m。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class EntranceGeometryConfig:
    """固定入口合同及有界质量门槛。"""

    enabled: bool = True
    center_x: float = 0.0
    wall_y: float = 0.09
    opening_width: float = 2.0
    wall_half_length: float = 5.51
    wall_thickness: float = 0.18
    voxel_size: float = 0.08
    minimum_points: int = 80
    required_frames: int = 3
    maximum_frame_spread: float = 0.08
    minimum_correction: float = 0.08
    maximum_correction: float = 0.50
    coarse_step: float = 0.05
    maximum_residual_p70: float = 0.12
    minimum_uniqueness_margin: float = 0.04
    timeout: float = 3.0
    maximum_frame_age: float = 1.0
    maximum_linear_speed: float = 0.08
    maximum_angular_speed: float = 0.10

    def __post_init__(self) -> None:
        positive = (
            self.opening_width,
            self.wall_half_length,
            self.wall_thickness,
            self.voxel_size,
            self.maximum_frame_spread,
            self.minimum_correction,
            self.maximum_correction,
            self.coarse_step,
            self.maximum_residual_p70,
            self.timeout,
            self.maximum_frame_age,
            self.maximum_linear_speed,
            self.maximum_angular_speed,
        )
        if (
            not all(math.isfinite(float(value)) and float(value) > 0.0 for value in positive)
            or not math.isfinite(float(self.center_x))
            or not math.isfinite(float(self.wall_y))
            or not math.isfinite(float(self.minimum_uniqueness_margin))
            or self.minimum_points < 20
            or self.required_frames < 2
            or self.minimum_correction >= self.maximum_correction
            or self.coarse_step > self.maximum_correction
            or not 0.0 <= self.minimum_uniqueness_margin <= 1.0
        ):
            raise ValueError("主入口几何纠偏参数无效")


@dataclass(frozen=True)
class EntranceGeometryEvaluation:
    """一次多帧入口几何匹配结果。"""

    status: str
    correction_xy: tuple[float, float] | None = None
    correction_norm: float | None = None
    residual_p70: float | None = None
    uniqueness_margin: float = 0.0
    frame_spread: float | None = None
    point_count: int = 0


@dataclass(frozen=True)
class EntranceCorrectionOutcome:
    """主任务可见的有界纠偏状态。"""

    status: str
    applied: bool = False
    reason: str | None = None
    correction_xy: tuple[float, float] | None = None


class EntranceGeometryCorrector:
    """固定入口几何匹配和一次性纠偏事务协调器。"""

    def __init__(self, config: EntranceGeometryConfig):
        self.config = config
        self._phase_started: dict[str, float] = {}
        self._phase_attempted: set[str] = set()
        self._phase_done: set[str] = set()
        self._operations: dict[str, tuple[int, float]] = {}
        self.last_evaluations: dict[str, EntranceGeometryEvaluation] = {}

    def evaluate(
        self,
        frames: Iterable[Iterable[tuple[float, float, float]]],
        *,
        floor_height: float,
        floor: int = 0,
    ) -> EntranceGeometryEvaluation:
        """在当前连续定位段内匹配入口外墙，禁止估计 yaw。"""
        selected = [
            self._filter_frame(frame, floor_height=float(floor_height), floor=int(floor))
            for frame in frames
        ]
        selected = [frame for frame in selected if len(frame) >= self.config.minimum_points]
        if len(selected) < self.config.required_frames:
            return EntranceGeometryEvaluation("insufficient_frames")
        selected = selected[-self.config.required_frames :]
        matches = [self._match(frame) for frame in selected]
        usable = [item for item in matches if item is not None]
        if len(usable) < self.config.required_frames:
            return EntranceGeometryEvaluation("match_unavailable")
        corrections = np.asarray([item[0] for item in usable], dtype=float)
        median_correction = np.median(corrections, axis=0)
        spread = float(np.max(np.linalg.norm(corrections - median_correction, axis=1)))
        residual = float(np.median([item[2] for item in usable]))
        uniqueness = float(min(item[3] for item in usable))
        point_count = int(sum(len(frame) for frame in selected))
        norm = float(np.linalg.norm(median_correction))
        common = dict(
            correction_xy=(float(median_correction[0]), float(median_correction[1])),
            correction_norm=norm,
            residual_p70=residual,
            uniqueness_margin=uniqueness,
            frame_spread=spread,
            point_count=point_count,
        )
        if norm > self.config.maximum_correction:
            return EntranceGeometryEvaluation("correction_exceeded", **common)
        if spread > self.config.maximum_frame_spread:
            return EntranceGeometryEvaluation("frame_spread_exceeded", **common)
        if residual > self.config.maximum_residual_p70:
            return EntranceGeometryEvaluation("residual_too_high", **common)
        if norm < self.config.minimum_correction:
            return EntranceGeometryEvaluation("no_correction_needed", **common)
        if uniqueness < self.config.minimum_uniqueness_margin:
            return EntranceGeometryEvaluation("ambiguous", **common)
        return EntranceGeometryEvaluation("accepted", **common)

    def tick(
        self,
        phase: str,
        *,
        now: float,
        frames: Iterable[Iterable[tuple[float, float, float]]],
        latest_cloud_stamp: float | None,
        floor_height: float,
        pose_provider,
        controller,
        map_runtime,
        robot_pose: tuple[float, float, float],
        robot_floor: int,
    ) -> EntranceCorrectionOutcome:
        """推进一次启动或门前纠偏；超时和请求失败都只降级一次。"""
        key = str(phase)
        if not self.config.enabled or key in self._phase_done:
            return EntranceCorrectionOutcome("done")
        if key not in self._phase_started:
            self._phase_started[key] = float(now)
        if key in self._operations:
            sequence, started = self._operations[key]
            ack = pose_provider.correction_ack(sequence)
            if ack is None:
                if float(now) - started < self.config.timeout:
                    controller.stop(reason=f"entrance_geometry_correction_{key}")
                    return EntranceCorrectionOutcome("running")
                self._operations.pop(key, None)
                self._phase_done.add(key)
                return EntranceCorrectionOutcome("done", reason="correction_ack_timeout")
            self._operations.pop(key, None)
            self._phase_done.add(key)
            correction_runtime = getattr(map_runtime, "apply_planar_correction", None)
            if callable(correction_runtime):
                corrected_xy = (
                    float(robot_pose[0]) + float(ack[0]),
                    float(robot_pose[1]) + float(ack[1]),
                )
                correction_runtime(
                    float(ack[0]),
                    float(ack[1]),
                    now=float(now),
                    robot_floor=int(robot_floor),
                    robot_xy=corrected_xy,
                )
            controller.clear_path(reason=f"entrance_geometry_correction_{key}_applied")
            return EntranceCorrectionOutcome(
                "done",
                applied=True,
                correction_xy=(float(ack[0]), float(ack[1])),
            )

        if key in self._phase_attempted:
            self._phase_done.add(key)
            return EntranceCorrectionOutcome("done", reason="attempt_already_used")
        latest = getattr(pose_provider, "latest", None)
        sample = latest() if callable(latest) else None
        if sample is not None and (
            float(getattr(sample, "planar_speed", math.inf)) > self.config.maximum_linear_speed
            or float(getattr(sample, "angular_speed", math.inf)) > self.config.maximum_angular_speed
        ):
            if float(now) - self._phase_started[key] < self.config.timeout:
                controller.stop(reason=f"entrance_geometry_waiting_for_stationary_{key}")
                return EntranceCorrectionOutcome("running")
            self._phase_attempted.add(key)
            self._phase_done.add(key)
            return EntranceCorrectionOutcome("done", reason="stationary_timeout")
        if latest_cloud_stamp is None or float(now) - float(latest_cloud_stamp) > self.config.maximum_frame_age:
            if float(now) - self._phase_started[key] >= self.config.timeout:
                self._phase_attempted.add(key)
                self._phase_done.add(key)
                return EntranceCorrectionOutcome("done", reason="cloud_stale")
            controller.stop(reason=f"entrance_geometry_waiting_for_cloud_{key}")
            return EntranceCorrectionOutcome("running")
        evaluation = self.evaluate(frames, floor_height=floor_height, floor=robot_floor)
        self.last_evaluations[key] = evaluation
        if evaluation.status == "insufficient_frames" and float(now) - self._phase_started[key] < self.config.timeout:
            controller.stop(reason=f"entrance_geometry_waiting_for_frames_{key}")
            return EntranceCorrectionOutcome("running")
        self._phase_attempted.add(key)
        self._phase_done.add(key)
        if evaluation.status != "accepted" or evaluation.correction_xy is None:
            return EntranceCorrectionOutcome("done", reason=evaluation.status)
        controller.stop(reason=f"entrance_geometry_correction_{key}_request")
        sequence = pose_provider.request_planar_correction(
            evaluation.correction_xy,
            maximum_correction=self.config.maximum_correction,
            maximum_linear_speed=self.config.maximum_linear_speed,
            maximum_angular_speed=self.config.maximum_angular_speed,
        )
        if sequence is None:
            return EntranceCorrectionOutcome("done", reason="correction_request_rejected")
        self._phase_done.discard(key)
        self._operations[key] = (int(sequence), float(now))
        return EntranceCorrectionOutcome("running")

    def _filter_frame(self, points, *, floor_height: float, floor: int) -> np.ndarray:
        array = np.asarray(tuple(points), dtype=float)
        if array.size == 0 or array.ndim != 2 or array.shape[1] != 3:
            return np.empty((0, 2), dtype=float)
        floor_z = int(floor) * float(floor_height)
        relative_z = array[:, 2] - floor_z
        finite = np.all(np.isfinite(array), axis=1)
        x = array[:, 0]
        y = array[:, 1]
        roi = (
            (x >= self.config.center_x - self.config.wall_half_length - self.config.maximum_correction - 0.4)
            & (x <= self.config.center_x + self.config.wall_half_length + self.config.maximum_correction + 0.4)
            & (y >= self.config.wall_y - 0.80 - self.config.maximum_correction)
            & (y <= self.config.wall_y + 0.80 + self.config.maximum_correction)
            & (relative_z >= 0.22)
            & (relative_z <= 1.90)
        )
        result = array[finite & roi, :2]
        return self._voxelize(result)

    def _reference_points(self) -> np.ndarray:
        step = max(0.04, min(0.12, self.config.voxel_size))
        half_gap = self.config.opening_width / 2.0 + 0.10
        xs = np.concatenate(
            (
                np.arange(-self.config.wall_half_length, -half_gap + step * 0.5, step),
                np.arange(half_gap, self.config.wall_half_length + step * 0.5, step),
            )
        )
        ys = self.config.wall_y + np.asarray(
            (-self.config.wall_thickness / 2.0, 0.0, self.config.wall_thickness / 2.0)
        )
        return np.asarray([(x, y) for x in xs for y in ys], dtype=float)

    def _match(self, current: np.ndarray):
        reference = self._reference_points()
        if len(current) < self.config.minimum_points:
            return None
        current_xy = np.asarray(current, dtype=float)
        maximum_dy = int(math.floor(self.config.maximum_correction / self.config.coarse_step))
        candidates: list[tuple[float, float, int, np.ndarray, float, float]] = []
        half_gap = self.config.opening_width / 2.0 + 0.10
        minimum_side_points = max(8, self.config.minimum_points // 5)
        for iy in range(-maximum_dy, maximum_dy + 1):
            dy = iy * self.config.coarse_step
            shifted_y = current_xy[:, 1] + dy
            wall_band = np.abs(shifted_y - self.config.wall_y) <= max(
                0.12, self.config.wall_thickness
            )
            shifted_x = current_xy[:, 0]
            side_limit = self.config.maximum_correction
            left = shifted_x <= self.config.center_x - half_gap + side_limit
            right = shifted_x >= self.config.center_x + half_gap - side_limit
            left_x = shifted_x[wall_band & left]
            right_x = shifted_x[wall_band & right]
            if len(left_x) < minimum_side_points or len(right_x) < minimum_side_points:
                continue
            # 缺口两侧内缘提供 x 方向基准；取分位数可抑制少量门板或杂点。
            left_inner = float(np.quantile(left_x, 0.95))
            right_inner = float(np.quantile(right_x, 0.05))
            dx = self.config.center_x - 0.5 * (left_inner + right_inner)
            correction = np.asarray((dx, dy), dtype=float)
            shifted = current_xy + correction
            delta = shifted[:, None, :] - reference[None, :, :]
            distances = np.sqrt(np.min(np.sum(delta * delta, axis=2), axis=1))
            eligible = distances <= 0.35
            score = float(np.count_nonzero(distances[eligible] <= 0.14)) / max(1, len(current_xy))
            center_error = abs(float(np.median(shifted_y[wall_band])) - self.config.wall_y)
            side_support = float(min(len(left_x), len(right_x))) / max(1, len(current_xy))
            candidates.append(
                (
                    score,
                    center_error,
                    iy,
                    correction,
                    float(np.quantile(distances[eligible], 0.70)),
                    side_support,
                )
            )
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], -item[1]), reverse=True)
        best_score, _center_error, _iy, correction, residual_p70, side_support = candidates[0]
        second_score = max(
            (item[0] for item in candidates[1:] if abs(item[2] - _iy) >= 2),
            default=0.0,
        )
        shifted = current_xy + correction
        delta = shifted[:, None, :] - reference[None, :, :]
        distances = np.sqrt(np.min(np.sum(delta * delta, axis=2), axis=1))
        valid = distances <= 0.35
        if int(np.count_nonzero(valid)) < self.config.minimum_points:
            return None
        uniqueness = max(float(best_score - second_score), float(side_support))
        return correction, best_score, max(residual_p70, float(np.quantile(distances[valid], 0.70))), uniqueness

    def _voxelize(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return np.empty((0, 2), dtype=float)
        keys = np.floor(points / self.config.voxel_size).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(indices)]

"""负障碍支撑证据的 shadow-only 稀疏网格。

本模块只把显式楼面回波和“穿过当前楼面后命中更低表面”的射线记为
诊断证据。没有回波、视场外、遮挡和点数不足都不会被解释为缺失地面。
它不被生产导航导入，也不提供控制输出。
"""
from __future__ import annotations

from collections import Counter, defaultdict, deque
from dataclasses import asdict, dataclass
import math
from typing import Iterable, Sequence

import numpy as np


SUPPORTED = "SUPPORTED"
UNKNOWN_SUPPORT = "UNKNOWN_SUPPORT"
NO_SUPPORT_EVIDENCE = "NO_SUPPORT_EVIDENCE"


@dataclass(frozen=True)
class ShadowSupportConfig:
    """仅供诊断的候选参数；这些值不是生产安全阈值。"""

    resolution: float = 0.1
    origin_x: float = -14.0
    origin_y: float = -6.0
    floor_height: float = 2.6
    cloud_stride: int = 3
    cloud_max_points: int = 18000
    support_band: float = 0.08
    minimum_drop: float = 0.30
    minimum_cell_hits: int = 2
    required_consecutive_frames: int = 2
    minimum_alert_cells: int = 3
    sweep_half_width: float = 0.22
    lookahead_seconds: float = 1.0
    minimum_lookahead: float = 0.50
    maximum_lookahead: float = 0.80
    footprint_front_reach: float = 0.30
    snapshot_period: float = 0.5
    obstacle_min_height: float = 0.15
    obstacle_max_height: float = 1.40
    minimum_frame_points: int = 50
    fov_min_elevation_deg: float = -7.0
    fov_max_elevation_deg: float = 55.0
    threshold_widths: tuple[float, ...] = (0.18, 0.22, 0.28)
    threshold_hit_counts: tuple[int, ...] = (1, 2, 3)
    threshold_consecutive_frames: tuple[int, ...] = (1, 2, 3)

    def diagnostic_payload(self) -> dict[str, object]:
        payload = asdict(self)
        payload.update(
            {
                "shadow_only": True,
                "behavior_effect": False,
                "candidate_thresholds_not_production_defaults": True,
            }
        )
        return payload


@dataclass(frozen=True)
class SweepWidthComponents:
    """由机器人包络与观测误差组成的诊断扫掠半宽。"""

    collision_half_width: float
    foot_lateral_extent: float
    lateral_tracking_error: float
    turning_sweep: float
    cloud_odom_registration_error: float
    grid_quantization_error: float
    safety_margin: float

    def physical_half_width(self) -> float:
        mechanical_envelope = max(
            0.0,
            float(self.collision_half_width),
            float(self.foot_lateral_extent),
        )
        uncertainty = sum(
            max(0.0, float(value))
            for value in (
                self.lateral_tracking_error,
                self.turning_sweep,
                self.cloud_odom_registration_error,
                self.grid_quantization_error,
                self.safety_margin,
            )
        )
        return round(mechanical_envelope + uncertainty, 6)

    def neighbor_widths(self, *, delta: float = 0.02) -> tuple[float, float, float]:
        width = self.physical_half_width()
        offset = max(0.0, float(delta))
        return tuple(round(max(0.0, value), 6) for value in (width - offset, width, width + offset))


@dataclass
class _CellEvidence:
    support_hits: int = 0
    lower_hits: int = 0
    support_frames: int = 0
    lower_frames: int = 0
    lower_streak: int = 0
    state: str = UNKNOWN_SUPPORT
    last_stamp: float | None = None
    last_reason: str = "never_observed"

    def payload(self, cell: tuple[int, int]) -> dict[str, object]:
        return {
            "cell": [int(cell[0]), int(cell[1])],
            "state": self.state,
            "support_hits": self.support_hits,
            "lower_hits": self.lower_hits,
            "support_frames": self.support_frames,
            "lower_frames": self.lower_frames,
            "lower_streak": self.lower_streak,
            "last_stamp": self.last_stamp,
            "reason": self.last_reason,
        }


def _cell_for_xy(
    x: float,
    y: float,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[int, int]:
    return (
        int(math.floor((float(x) - origin_x) / resolution)),
        int(math.floor((float(y) - origin_y) / resolution)),
    )


def _cell_center(
    cell: tuple[int, int],
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    z: float,
) -> np.ndarray:
    return np.asarray(
        [
            origin_x + (cell[0] + 0.5) * resolution,
            origin_y + (cell[1] + 0.5) * resolution,
            z,
        ],
        dtype=float,
    )


def _point_cell_counts(points_xy: np.ndarray, config: ShadowSupportConfig) -> Counter:
    points = np.asarray(points_xy, dtype=float).reshape((-1, 2))
    if points.size == 0:
        return Counter()
    gx = np.floor((points[:, 0] - config.origin_x) / config.resolution).astype(int)
    gy = np.floor((points[:, 1] - config.origin_y) / config.resolution).astype(int)
    return Counter(zip(gx.tolist(), gy.tolist()))


def _project_lower_returns(
    sensor_origin: np.ndarray,
    points: np.ndarray,
    *,
    floor_z: float,
    minimum_drop: float,
) -> np.ndarray:
    origin = np.asarray(sensor_origin, dtype=float).reshape(3)
    cloud = np.asarray(points, dtype=float).reshape((-1, 3))
    if cloud.size == 0:
        return np.empty((0, 3), dtype=float)
    lower = cloud[cloud[:, 2] < floor_z - minimum_drop]
    if lower.size == 0:
        return np.empty((0, 3), dtype=float)
    denominator = lower[:, 2] - origin[2]
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = (floor_z - origin[2]) / denominator
    valid = np.isfinite(scale) & (scale > 0.0) & (scale < 1.0)
    return origin + scale[valid, None] * (lower[valid] - origin)


def _bresenham(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += sx
        if twice_error <= dx:
            error += dx
            y0 += sy


def _largest_cluster(cells: Iterable[tuple[int, int]]) -> int:
    remaining = set(cells)
    largest = 0
    while remaining:
        seed = remaining.pop()
        queue = deque([seed])
        count = 0
        while queue:
            x, y = queue.popleft()
            count += 1
            for neighbor in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
        largest = max(largest, count)
    return largest


def _distance_to_polyline_progress(
    position: np.ndarray,
    path: Sequence[np.ndarray],
) -> tuple[int, float]:
    """返回最近路径段及其段内比例。"""
    best = (math.inf, 0, 0.0)
    point = np.asarray(position, dtype=float).reshape(2)
    for index, (start, end) in enumerate(zip(path, path[1:])):
        start_array = np.asarray(start, dtype=float).reshape(2)
        delta = np.asarray(end, dtype=float).reshape(2) - start_array
        length_squared = float(delta @ delta)
        fraction = (
            max(0.0, min(1.0, float((point - start_array) @ delta / length_squared)))
            if length_squared > 1e-12
            else 0.0
        )
        distance = float(np.linalg.norm(point - (start_array + fraction * delta)))
        if distance < best[0]:
            best = (distance, index, fraction)
    return best[1], best[2]


def _future_polyline(
    robot_xy: np.ndarray,
    robot_yaw: float,
    path: Sequence[np.ndarray],
    distance: float,
) -> list[np.ndarray]:
    position = np.asarray(robot_xy, dtype=float).reshape(2)
    remaining = max(0.0, float(distance))
    normalized = [np.asarray(point, dtype=float).reshape(2) for point in path]
    if len(normalized) < 2:
        direction = np.asarray([math.cos(robot_yaw), math.sin(robot_yaw)])
        return [position, position + direction * remaining]

    segment_index, fraction = _distance_to_polyline_progress(position, normalized)
    segment_end = normalized[segment_index + 1]
    segment_start = normalized[segment_index]
    projection = segment_start + fraction * (segment_end - segment_start)
    result = [position]
    if float(np.linalg.norm(projection - position)) > 1e-6:
        result.append(projection)
    cursor = projection
    for point in normalized[segment_index + 1 :]:
        segment = point - cursor
        length = float(np.linalg.norm(segment))
        if length <= 1e-9:
            cursor = point
            continue
        if length >= remaining:
            result.append(cursor + segment * (remaining / length))
            return result
        result.append(point)
        remaining -= length
        cursor = point
    return result


def _sweep_cells(
    polyline: Sequence[np.ndarray],
    *,
    half_width: float,
    config: ShadowSupportConfig,
) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    radius_cells = int(math.ceil(max(0.0, half_width) / config.resolution))
    for start, end in zip(polyline, polyline[1:]):
        start_array = np.asarray(start, dtype=float)
        end_array = np.asarray(end, dtype=float)
        distance = float(np.linalg.norm(end_array - start_array))
        samples = max(2, int(math.ceil(distance / (config.resolution * 0.4))) + 1)
        for fraction in np.linspace(0.0, 1.0, samples):
            point = start_array + fraction * (end_array - start_array)
            center = _cell_for_xy(
                point[0],
                point[1],
                resolution=config.resolution,
                origin_x=config.origin_x,
                origin_y=config.origin_y,
            )
            for gx in range(center[0] - radius_cells, center[0] + radius_cells + 1):
                for gy in range(center[1] - radius_cells, center[1] + radius_cells + 1):
                    candidate_center = _cell_center(
                        (gx, gy),
                        resolution=config.resolution,
                        origin_x=config.origin_x,
                        origin_y=config.origin_y,
                        z=0.0,
                    )[:2]
                    if float(np.linalg.norm(candidate_center - point)) <= half_width:
                        cells.add((gx, gy))
    return cells


class ShadowSupportGrid:
    """按显式回波维护稀疏支撑证据，输出反事实告警但不控制机器人。"""

    def __init__(self, config: ShadowSupportConfig | None = None):
        self.config = config or ShadowSupportConfig()
        self._cells: dict[int, dict[tuple[int, int], _CellEvidence]] = defaultdict(dict)
        self._lower_streaks: dict[int, dict[int, dict[tuple[int, int], int]]] = defaultdict(
            lambda: defaultdict(dict)
        )
        self._ever_visible: dict[int, set[tuple[int, int]]] = defaultdict(set)
        self._changed: dict[int, set[tuple[int, int]]] = defaultdict(set)
        self._last_cloud_stamp: float | None = None
        self._last_emit_stamp: float | None = None
        self._last_alert_active = False
        self.updated_frame_count = 0
        self.skipped_ambiguous_count = 0
        self.duplicate_cloud_count = 0
        self.total_input_points = 0
        self.total_sampled_points = 0

    def cell_state(self, floor: int, cell: tuple[int, int]) -> str:
        evidence = self._cells.get(int(floor), {}).get(tuple(cell))
        return evidence.state if evidence is not None else UNKNOWN_SUPPORT

    def cell_snapshot(self, floor: int, cell: tuple[int, int]) -> dict[str, object]:
        evidence = self._cells.get(int(floor), {}).get(tuple(cell), _CellEvidence())
        return evidence.payload(tuple(cell))

    def no_support_cell_count(self, floor: int) -> int:
        return sum(
            evidence.state == NO_SUPPORT_EVIDENCE
            for evidence in self._cells.get(int(floor), {}).values()
        )

    def sparse_grid_snapshot(self) -> dict[str, list[dict[str, object]]]:
        return {
            str(floor): [
                evidence.payload(cell)
                for cell, evidence in sorted(cells.items())
            ]
            for floor, cells in sorted(self._cells.items())
        }

    def consume_changed_cells(self, floor: int) -> list[dict[str, object]]:
        floor_index = int(floor)
        changed = self._changed[floor_index]
        payload = [
            self._cells[floor_index][cell].payload(cell)
            for cell in sorted(changed)
        ]
        changed.clear()
        return payload

    def _lookahead(self, command_linear: float) -> float:
        raw = self.config.footprint_front_reach + abs(float(command_linear)) * self.config.lookahead_seconds
        return max(self.config.minimum_lookahead, min(self.config.maximum_lookahead, raw))

    def _update_streaks(
        self,
        floor: int,
        lower_counts: Counter,
        support_counts: Counter,
    ) -> None:
        for hit_threshold in self.config.threshold_hit_counts:
            streaks = self._lower_streaks[int(floor)][int(hit_threshold)]
            active = {
                cell
                for cell, count in lower_counts.items()
                if count >= int(hit_threshold) and support_counts[cell] == 0
            }
            for cell in list(streaks):
                if cell not in active:
                    streaks.pop(cell, None)
            for cell in active:
                streaks[cell] = min(
                    max(self.config.threshold_consecutive_frames),
                    streaks.get(cell, 0) + 1,
                )

    def _update_cells(
        self,
        floor: int,
        stamp: float,
        support_counts: Counter,
        lower_counts: Counter,
    ) -> None:
        floor_cells = self._cells[int(floor)]
        active_default = {
            cell
            for cell, count in lower_counts.items()
            if count >= self.config.minimum_cell_hits and support_counts[cell] == 0
        }
        for cell, evidence in floor_cells.items():
            if evidence.lower_streak and cell not in active_default:
                before = (evidence.lower_streak, evidence.state, evidence.last_reason)
                evidence.lower_streak = 0
                if evidence.state != SUPPORTED:
                    evidence.state = UNKNOWN_SUPPORT
                    evidence.last_reason = "lower_evidence_not_consecutive"
                after = (evidence.lower_streak, evidence.state, evidence.last_reason)
                if after != before:
                    evidence.last_stamp = float(stamp)
                    self._changed[int(floor)].add(cell)

        for cell, count in support_counts.items():
            evidence = floor_cells.setdefault(cell, _CellEvidence())
            before = (
                evidence.support_hits,
                evidence.support_frames,
                evidence.lower_streak,
                evidence.state,
                evidence.last_reason,
            )
            evidence.support_hits = min(
                max(self.config.threshold_hit_counts),
                evidence.support_hits + int(count),
            )
            evidence.support_frames = min(
                max(self.config.threshold_consecutive_frames),
                evidence.support_frames + 1,
            )
            evidence.lower_streak = 0
            if evidence.support_hits >= self.config.minimum_cell_hits:
                evidence.state = SUPPORTED
                evidence.last_reason = "explicit_current_floor_returns"
            else:
                evidence.state = UNKNOWN_SUPPORT
                evidence.last_reason = "insufficient_support_returns"
            after = (
                evidence.support_hits,
                evidence.support_frames,
                evidence.lower_streak,
                evidence.state,
                evidence.last_reason,
            )
            if after != before:
                evidence.last_stamp = float(stamp)
                self._changed[int(floor)].add(cell)

        for cell, count in lower_counts.items():
            evidence = floor_cells.setdefault(cell, _CellEvidence())
            before = (
                evidence.lower_hits,
                evidence.lower_frames,
                evidence.lower_streak,
                evidence.state,
                evidence.last_reason,
            )
            evidence.lower_hits = min(
                max(self.config.threshold_hit_counts)
                * max(self.config.threshold_consecutive_frames),
                evidence.lower_hits + int(count),
            )
            if count >= self.config.minimum_cell_hits:
                evidence.lower_frames = min(
                    max(self.config.threshold_consecutive_frames),
                    evidence.lower_frames + 1,
                )
                if support_counts[cell] == 0:
                    evidence.lower_streak = min(
                        max(self.config.threshold_consecutive_frames),
                        evidence.lower_streak + 1,
                    )
            if evidence.state == SUPPORTED:
                evidence.state = SUPPORTED
                evidence.last_reason = "historical_explicit_support_overrides_conflict"
            elif evidence.lower_streak >= self.config.required_consecutive_frames:
                evidence.state = NO_SUPPORT_EVIDENCE
                evidence.last_reason = "consecutive_lower_surface_crossings"
            else:
                evidence.state = UNKNOWN_SUPPORT
                evidence.last_reason = "insufficient_lower_crossing_evidence"
            after = (
                evidence.lower_hits,
                evidence.lower_frames,
                evidence.lower_streak,
                evidence.state,
                evidence.last_reason,
            )
            if after != before:
                evidence.last_stamp = float(stamp)
                self._changed[int(floor)].add(cell)

    def _inside_fov(
        self,
        cell: tuple[int, int],
        *,
        floor_z: float,
        sensor_origin: np.ndarray,
        sensor_rotation: np.ndarray,
    ) -> bool:
        center = _cell_center(
            cell,
            resolution=self.config.resolution,
            origin_x=self.config.origin_x,
            origin_y=self.config.origin_y,
            z=floor_z,
        )
        sensor_vector = np.asarray(sensor_rotation, dtype=float).T @ (
            center - np.asarray(sensor_origin, dtype=float)
        )
        distance = float(np.linalg.norm(sensor_vector))
        if distance < 0.20:
            return False
        elevation = math.degrees(
            math.atan2(sensor_vector[2], math.hypot(sensor_vector[0], sensor_vector[1]))
        )
        return (
            self.config.fov_min_elevation_deg
            <= elevation
            <= self.config.fov_max_elevation_deg
        )

    def _is_occluded(
        self,
        cell: tuple[int, int],
        *,
        sensor_origin: np.ndarray,
        obstacle_counts: Counter,
    ) -> bool:
        sensor_cell = _cell_for_xy(
            sensor_origin[0],
            sensor_origin[1],
            resolution=self.config.resolution,
            origin_x=self.config.origin_x,
            origin_y=self.config.origin_y,
        )
        return any(
            obstacle_counts[item] >= self.config.minimum_cell_hits
            for item in _bresenham(sensor_cell, cell)[1:-1]
        )

    def _sweep_payload(
        self,
        floor: int,
        sweep_cells: set[tuple[int, int]],
        *,
        floor_z: float,
        sensor_origin: np.ndarray,
        sensor_rotation: np.ndarray,
        obstacle_counts: Counter,
        sampled_point_count: int,
    ) -> dict[str, object]:
        states = Counter()
        reasons = Counter()
        cells = []
        floor_cells = self._cells.get(int(floor), {})
        for cell in sorted(sweep_cells):
            evidence = floor_cells.get(cell)
            if evidence is not None and evidence.state != UNKNOWN_SUPPORT:
                state = evidence.state
                reason = evidence.last_reason
                adequate = True
            elif not self._inside_fov(
                cell,
                floor_z=floor_z,
                sensor_origin=sensor_origin,
                sensor_rotation=sensor_rotation,
            ):
                state = UNKNOWN_SUPPORT
                reason = "outside_fov"
                adequate = False
            elif self._is_occluded(
                cell,
                sensor_origin=sensor_origin,
                obstacle_counts=obstacle_counts,
            ):
                state = UNKNOWN_SUPPORT
                reason = "occluded"
                adequate = False
            elif sampled_point_count < self.config.minimum_frame_points:
                state = UNKNOWN_SUPPORT
                reason = "insufficient_frame_points"
                adequate = False
            elif cell not in self._ever_visible[int(floor)]:
                state = UNKNOWN_SUPPORT
                reason = "never_observed"
                adequate = False
            else:
                state = UNKNOWN_SUPPORT
                reason = "visible_without_explicit_support_or_drop_return"
                adequate = False
            states[state] += 1
            reasons[reason] += 1
            cells.append(
                {
                    "cell": [cell[0], cell[1]],
                    "state": state,
                    "reason": reason,
                    "observation_adequate": adequate,
                }
            )
        no_support = {
            cell
            for cell in sweep_cells
            if floor_cells.get(cell) is not None
            and floor_cells[cell].state == NO_SUPPORT_EVIDENCE
        }
        return {
            "cell_count": len(sweep_cells),
            "state_counts": dict(states),
            "reason_counts": dict(reasons),
            "largest_no_support_cluster": _largest_cluster(no_support),
            "cells": cells,
        }

    def _threshold_sensitivity(
        self,
        floor: int,
        polylines_by_width: dict[float, set[tuple[int, int]]],
    ) -> dict[str, object]:
        values: dict[str, object] = {}
        supported = {
            cell
            for cell, evidence in self._cells.get(int(floor), {}).items()
            if evidence.state == SUPPORTED
        }
        for width, sweep_cells in polylines_by_width.items():
            for hit_threshold in self.config.threshold_hit_counts:
                streaks = self._lower_streaks[int(floor)][int(hit_threshold)]
                for consecutive in self.config.threshold_consecutive_frames:
                    candidates = {
                        cell
                        for cell in sweep_cells
                        if streaks.get(cell, 0) >= int(consecutive) and cell not in supported
                    }
                    largest = _largest_cluster(candidates)
                    key = f"w{width:.2f}_h{int(hit_threshold)}_c{int(consecutive)}"
                    values[key] = {
                        "candidate_cells": len(candidates),
                        "largest_cluster": largest,
                        "active": largest >= self.config.minimum_alert_cells,
                    }
        return values

    def process_frame(
        self,
        *,
        stamp: float,
        floor: int,
        robot_xy: np.ndarray,
        robot_yaw: float,
        sensor_origin: np.ndarray,
        sensor_rotation: np.ndarray,
        points: np.ndarray,
        path: Sequence[np.ndarray],
        command_linear: float,
        ambiguous: bool,
    ) -> dict[str, object]:
        """消费一帧；歧义帧的 stamp 会被消费，但不会更新网格。"""
        stamp = float(stamp)
        if self._last_cloud_stamp is not None and stamp <= self._last_cloud_stamp:
            self.duplicate_cloud_count += 1
            return {
                "schema": "ground_support_shadow_frame_v1",
                "stamp": stamp,
                "floor": int(floor),
                "action": "skip_duplicate_cloud",
                "shadow_only": True,
                "behavior_effect": False,
            }
        self._last_cloud_stamp = stamp
        if ambiguous:
            self.skipped_ambiguous_count += 1
            return {
                "schema": "ground_support_shadow_frame_v1",
                "stamp": stamp,
                "floor": int(floor),
                "action": "skip_floor_transition_ambiguous",
                "shadow_only": True,
                "behavior_effect": False,
            }

        cloud = np.asarray(points, dtype=float).reshape((-1, 3))
        finite = cloud[np.all(np.isfinite(cloud), axis=1)] if cloud.size else cloud
        self.total_input_points += int(finite.shape[0])
        stride = max(1, int(self.config.cloud_stride))
        limit = max(0, int(self.config.cloud_max_points))
        sampled = finite[::stride][:limit] if limit else np.empty((0, 3), dtype=float)
        self.total_sampled_points += int(sampled.shape[0])
        self.updated_frame_count += 1

        floor_index = int(floor)
        floor_z = floor_index * self.config.floor_height
        support_points = sampled[
            np.abs(sampled[:, 2] - floor_z) <= self.config.support_band
        ]
        lower_crossings = _project_lower_returns(
            np.asarray(sensor_origin, dtype=float),
            sampled,
            floor_z=floor_z,
            minimum_drop=self.config.minimum_drop,
        )
        obstacle_points = sampled[
            (sampled[:, 2] >= floor_z + self.config.obstacle_min_height)
            & (sampled[:, 2] <= floor_z + self.config.obstacle_max_height)
        ]
        support_counts = _point_cell_counts(support_points[:, :2], self.config)
        lower_counts = _point_cell_counts(lower_crossings[:, :2], self.config)
        obstacle_counts = _point_cell_counts(obstacle_points[:, :2], self.config)
        self._update_streaks(floor_index, lower_counts, support_counts)
        self._update_cells(floor_index, stamp, support_counts, lower_counts)

        lookahead = self._lookahead(command_linear)
        polyline = _future_polyline(robot_xy, robot_yaw, path, lookahead)
        sweep_by_width = {
            float(width): _sweep_cells(
                polyline,
                half_width=float(width),
                config=self.config,
            )
            for width in self.config.threshold_widths
        }
        default_sweep = _sweep_cells(
            polyline,
            half_width=self.config.sweep_half_width,
            config=self.config,
        )
        for cell in default_sweep:
            if self._inside_fov(
                cell,
                floor_z=floor_z,
                sensor_origin=sensor_origin,
                sensor_rotation=sensor_rotation,
            ):
                self._ever_visible[floor_index].add(cell)

        sweep_payload = self._sweep_payload(
            floor_index,
            default_sweep,
            floor_z=floor_z,
            sensor_origin=np.asarray(sensor_origin, dtype=float),
            sensor_rotation=np.asarray(sensor_rotation, dtype=float),
            obstacle_counts=obstacle_counts,
            sampled_point_count=int(sampled.shape[0]),
        )
        alert_active = (
            sweep_payload["largest_no_support_cluster"]
            >= self.config.minimum_alert_cells
        )
        threshold_payload = self._threshold_sensitivity(floor_index, sweep_by_width)
        should_emit = (
            self._last_emit_stamp is None
            or stamp - self._last_emit_stamp >= self.config.snapshot_period
            or alert_active != self._last_alert_active
        )
        if should_emit:
            self._last_emit_stamp = stamp
        alert_transition = alert_active != self._last_alert_active
        self._last_alert_active = alert_active
        return {
            "schema": "ground_support_shadow_frame_v1",
            "stamp": stamp,
            "floor": floor_index,
            "action": "update",
            "shadow_only": True,
            "behavior_effect": False,
            "input_point_count": int(finite.shape[0]),
            "sampled_point_count": int(sampled.shape[0]),
            "support_return_count": int(support_points.shape[0]),
            "lower_return_count": int(np.count_nonzero(sampled[:, 2] < floor_z - self.config.minimum_drop)),
            "lower_crossing_count": int(lower_crossings.shape[0]),
            "support_cell_count": len(support_counts),
            "lower_crossing_cell_count": len(lower_counts),
            "lookahead_distance": lookahead,
            "path_footprint_sweep": sweep_payload,
            "hypothetical_alert": {
                "active": alert_active,
                "transition": alert_transition,
                "would_block_if_production_gate_existed": alert_active,
                "production_gate_enabled": False,
            },
            "threshold_sensitivity": threshold_payload,
            "should_emit": should_emit,
        }

    def summary(self) -> dict[str, object]:
        state_counts = Counter()
        per_floor = {}
        for floor, cells in self._cells.items():
            counts = Counter(item.state for item in cells.values())
            state_counts.update(counts)
            per_floor[str(floor)] = {
                "observed_cell_count": len(cells),
                "state_counts": dict(counts),
            }
        return {
            "schema": "ground_support_shadow_summary_v1",
            "shadow_only": True,
            "behavior_effect": False,
            "config": self.config.diagnostic_payload(),
            "updated_frame_count": self.updated_frame_count,
            "skipped_ambiguous_count": self.skipped_ambiguous_count,
            "duplicate_cloud_count": self.duplicate_cloud_count,
            "total_input_points": self.total_input_points,
            "total_sampled_points": self.total_sampled_points,
            "state_counts": dict(state_counts),
            "floors": per_floor,
        }

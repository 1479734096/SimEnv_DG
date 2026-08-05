"""返航路径快照规划、历史轨迹回退与候选验证。

本模块只处理调用方冻结后的 ``FloorGrid``、轨迹和失败首段，不读取 ROS、不修改
正式地图，也不切换任务状态。MissionManager 负责创建快照、轮询结果和执行路径。
"""
from __future__ import annotations

import math
from threading import Lock, Thread

import numpy as np

from danger_search.mapper import BLACK, NEGATIVE, OCC, VISITED, FloorGrid
from danger_search.planner import astar_plan


class AsyncReturnPlanner:
    """在冻结输入上执行一次返航规划，主循环只轮询不可变结果。"""

    def __init__(self, *, floor: int, goal_xy: tuple[float, float], compute):
        self.floor = int(floor)
        self.goal_xy = (float(goal_xy[0]), float(goal_xy[1]))
        self._compute = compute
        self._lock = Lock()
        self._done = False
        self._outcome: tuple[str, list[tuple[float, float]]] | None = None
        self._error: str | None = None
        self._thread = Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def poll(self) -> tuple[str, list[tuple[float, float]]] | None:
        with self._lock:
            return self._outcome if self._done else None

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    def _run(self) -> None:
        try:
            outcome = self._compute()
        except Exception as exc:
            outcome = ("error", [])
            error = repr(exc)
        else:
            error = None
        with self._lock:
            self._outcome = outcome
            self._error = error
            self._done = True


def compute_return_home_plan(
    grid: FloorGrid,
    pose: tuple[float, float, float],
    goal_xy: tuple[float, float],
    *,
    trace_snapshot: tuple[tuple[float, float], ...] = (),
    visited_radius: float = 0.25,
    veto_segments: tuple[tuple[float, float, float, float], ...] = (),
    failed_signature: tuple[int, int] | None = None,
    veto_lookahead: float = 0.8,
) -> tuple[str, list[tuple[float, float]]]:
    """在地图快照上完成 A*、历史轨迹和安全直线三级判定。"""
    start_xy = (float(pose[0]), float(pose[1]))
    original_grid = copy_floor_grid(grid) if veto_segments else None
    if veto_segments:
        apply_return_home_vetoes(
            grid,
            veto_segments,
            goal_xy=goal_xy,
            lookahead=veto_lookahead,
        )
    result = astar_plan(grid, start_xy, goal_xy, max_iterations=None)
    if result.reached:
        return validate_return_home_candidate(
            result.path,
            original_grid=original_grid,
            failed_signature=failed_signature,
        )
    if not result.unreachable:
        return "budget_exhausted", []
    trace_path = return_path_from_navigation_trace(
        grid,
        pose,
        goal_xy,
        trace_points=trace_snapshot,
        visited_radius=visited_radius,
    )
    if trace_path:
        return validate_return_home_candidate(
            trace_path,
            original_grid=original_grid,
            failed_signature=failed_signature,
        )
    if not _segment_crosses_blocked_cell(grid, start_xy, goal_xy):
        return validate_return_home_candidate(
            [start_xy, goal_xy],
            original_grid=original_grid,
            failed_signature=failed_signature,
        )
    if veto_segments:
        return "candidate_rejected_unreachable", []
    return "unreachable", []


def validate_return_home_candidate(
    path: list[tuple[float, float]],
    *,
    original_grid: FloorGrid | None,
    failed_signature: tuple[int, int] | None,
) -> tuple[str, list[tuple[float, float]]]:
    if original_grid is None:
        return "reached", path
    if len(path) < 2:
        return "candidate_rejected_invalid", []
    if return_segment_signature(path[1]) == failed_signature:
        return "candidate_rejected_same_segment", []
    if path_crosses_blocked_beyond(original_grid, path, exemption=0.20):
        return "candidate_rejected_blocked", []
    return "reached", path


def return_segment_signature(end_xy: tuple[float, float]) -> tuple[int, int]:
    """以 0.3m 量化首段终点，识别确定性重规划产生的等价路径。"""
    return (
        int(round(float(end_xy[0]) / 0.3)),
        int(round(float(end_xy[1]) / 0.3)),
    )


def apply_return_home_vetoes(
    grid: FloorGrid,
    segments: tuple[tuple[float, float, float, float], ...],
    *,
    goal_xy: tuple[float, float],
    lookahead: float,
) -> None:
    """在返航快照中把已证实不可执行的首段标为 BLACK。"""
    for ax, ay, bx, by in segments:
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        end_distance = min(float(lookahead), length)
        if end_distance <= 0.15:
            continue
        samples = max(
            1,
            int(
                math.ceil(
                    (end_distance - 0.15) / max(grid.resolution * 0.5, 1e-6)
                )
            ),
        )
        for index in range(samples + 1):
            distance = 0.15 + (end_distance - 0.15) * index / samples
            x = ax + dx * distance / length
            y = ay + dy * distance / length
            if math.hypot(x - goal_xy[0], y - goal_xy[1]) <= 0.30:
                continue
            cell = grid.world_to_grid(x, y)
            if cell is None:
                continue
            gx, gy = cell
            if int(grid.data[gy, gx]) != OCC:
                grid.data[gy, gx] = BLACK


def path_crosses_blocked_beyond(
    grid: FloorGrid,
    path: list[tuple[float, float]],
    *,
    exemption: float,
) -> bool:
    """忽略机身已占的短起步区，检查候选后续是否穿越实体或安全禁区。"""
    travelled = 0.0
    for start_xy, end_xy in zip(path, path[1:]):
        length = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
        samples = max(
            1,
            int(math.ceil(length / max(grid.resolution * 0.5, 1e-6))),
        )
        for index in range(1, samples + 1):
            segment_distance = length * index / samples
            if travelled + segment_distance <= exemption:
                continue
            ratio = index / samples
            cell = grid.world_to_grid(
                start_xy[0] + (end_xy[0] - start_xy[0]) * ratio,
                start_xy[1] + (end_xy[1] - start_xy[1]) * ratio,
            )
            if cell is None:
                return True
            gx, gy = cell
            if int(grid.data[gy, gx]) in (BLACK, NEGATIVE, OCC):
                return True
        travelled += length
    return False


def copy_floor_grid(grid: FloorGrid) -> FloorGrid:
    """复制返航所需的栅格状态，后台线程不得持有正式地图引用。"""
    snapshot = FloorGrid(
        floor_index=grid.floor_index,
        resolution=grid.resolution,
        width=grid.width,
        length=grid.length,
        origin_x=grid.origin_x,
        origin_y=grid.origin_y,
    )
    snapshot.data = np.array(grid.data, copy=True)
    snapshot.negative_cells = set(grid.negative_cells)
    return snapshot


def return_path_from_navigation_trace(
    grid: FloorGrid,
    pose: tuple[float, float, float],
    goal_xy: tuple[float, float],
    *,
    trace_points: tuple[tuple[float, float], ...],
    visited_radius: float,
) -> list[tuple[float, float]] | None:
    """在地图副本中恢复实际走过的走廊，不污染正式 FREE/OCC 语义。"""
    trace = list(trace_points)
    if len(trace) < 2:
        return None

    goal_index = min(
        range(len(trace)),
        key=lambda index: math.hypot(
            trace[index][0] - goal_xy[0],
            trace[index][1] - goal_xy[1],
        ),
    )
    if math.hypot(
        trace[goal_index][0] - goal_xy[0],
        trace[goal_index][1] - goal_xy[1],
    ) > 1.5:
        return None

    relevant_trace = trace[goal_index:]
    current_xy = (float(pose[0]), float(pose[1]))
    if math.hypot(
        relevant_trace[-1][0] - current_xy[0],
        relevant_trace[-1][1] - current_xy[1],
    ) > 0.75:
        return None

    trace_grid = copy_floor_grid(grid)
    corridor_radius = min(
        0.30,
        max(float(grid.resolution), float(visited_radius) * 0.5),
    )
    trusted_points = [*relevant_trace, current_xy]
    for start_xy, end_xy in zip(trusted_points, trusted_points[1:]):
        if math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1]) > 0.75:
            continue
        _mark_return_trace_segment(
            trace_grid,
            start_xy,
            end_xy,
            corridor_radius,
        )
    _mark_return_trace_segment(
        trace_grid,
        relevant_trace[0],
        goal_xy,
        corridor_radius,
    )
    result = astar_plan(trace_grid, current_xy, goal_xy, max_iterations=None)
    return result.path if result.reached else None


def initial_entry_reverse_path(
    pose: tuple[float, float, float],
    home_pose: tuple[float, float, float] | None,
    trace_points: tuple[tuple[float, float], ...],
) -> list[tuple[float, float]] | None:
    """构造从门外当前位姿沿已走轨迹回到实际起点的路径。"""
    if home_pose is None or not trace_points:
        return None
    path = [(float(pose[0]), float(pose[1]))]
    for point in reversed(trace_points):
        waypoint = (float(point[0]), float(point[1]))
        if math.hypot(waypoint[0] - path[-1][0], waypoint[1] - path[-1][1]) >= 0.15:
            path.append(waypoint)
    home_xy = (float(home_pose[0]), float(home_pose[1]))
    if math.hypot(home_xy[0] - path[-1][0], home_xy[1] - path[-1][1]) >= 0.05:
        path.append(home_xy)
    return path if len(path) >= 2 else None


def _mark_return_trace_segment(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    radius: float,
) -> None:
    """只在临时地图中恢复机器人中心确实经过的窄走廊，保留 BLACK。"""
    distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    samples = max(
        1,
        int(math.ceil(distance / max(grid.resolution * 0.5, 1e-6))),
    )
    radius_cells = max(0, int(math.ceil(radius / grid.resolution)))
    for index in range(samples + 1):
        ratio = index / samples
        sample_x = start_xy[0] + (end_xy[0] - start_xy[0]) * ratio
        sample_y = start_xy[1] + (end_xy[1] - start_xy[1]) * ratio
        cell = grid.world_to_grid(sample_x, sample_y)
        if cell is None:
            continue
        cx, cy = cell
        for gy in range(cy - radius_cells, cy + radius_cells + 1):
            for gx in range(cx - radius_cells, cx + radius_cells + 1):
                if not (0 <= gx < grid.size_x and 0 <= gy < grid.size_y):
                    continue
                wx, wy = grid.grid_to_world(gx, gy)
                if math.hypot(wx - sample_x, wy - sample_y) > radius:
                    continue
                if int(grid.data[gy, gx]) not in (BLACK, NEGATIVE):
                    grid.data[gy, gx] = VISITED


def _segment_crosses_blocked_cell(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
) -> bool:
    if grid.world_to_grid(*start_xy) is None or grid.world_to_grid(*end_xy) is None:
        return True
    distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    step_count = max(
        1,
        int(math.ceil(distance / max(grid.resolution * 0.5, 1e-6))),
    )
    for index in range(1, step_count + 1):
        ratio = index / step_count
        cell = grid.world_to_grid(
            start_xy[0] + (end_xy[0] - start_xy[0]) * ratio,
            start_xy[1] + (end_xy[1] - start_xy[1]) * ratio,
        )
        if cell is None:
            return True
        gx, gy = cell
        if int(grid.data[gy, gx]) in (BLACK, NEGATIVE, OCC):
            return True
    return False

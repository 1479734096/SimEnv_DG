"""电梯任务复用已走轨迹时的纯导航几何。"""
from __future__ import annotations

import math

from danger_search.exploration_runtime import (
    grid_cell_has_clearance,
    initial_occ_escape_segment_is_safe,
)
from danger_search.mapper import OCC


def elevator_approach_trace_candidates(
    *,
    elevator,
    floor_index: int,
    navigation_trace,
    grid,
    visual_max_distance: float,
    max_candidates: int,
) -> list[tuple[float, float]]:
    """返回大厅侧已走轨迹点，调用方仍须执行标准 A*。"""
    if (
        max_candidates <= 0
        or elevator is None
        or int(floor_index) not in elevator.lobby_poses
    ):
        return []
    trace = list(navigation_trace)
    if not trace:
        return []
    ex, ey, eyaw = elevator.lobby_poses[int(floor_index)]
    outward = (math.cos(eyaw), math.sin(eyaw))
    wait_xy = (ex + 0.8 * outward[0], ey + 0.8 * outward[1])
    maximum_distance = min(3.0, max(1.0, float(visual_max_distance) - 0.5))
    ranked: list[tuple[float, int, tuple[float, float]]] = []
    for index, point in enumerate(trace):
        point_xy = (float(point[0]), float(point[1]))
        distance = math.hypot(point_xy[0] - ex, point_xy[1] - ey)
        hall_progress = (
            (point_xy[0] - ex) * outward[0]
            + (point_xy[1] - ey) * outward[1]
        )
        cell = grid.world_to_grid(*point_xy)
        if (
            not 1.0 <= distance <= maximum_distance
            or hall_progress < 0.30
            or cell is None
            or not grid.is_traversable(*cell)
        ):
            continue
        ranked.append(
            (
                math.hypot(point_xy[0] - wait_xy[0], point_xy[1] - wait_xy[1]),
                -index,
                point_xy,
            )
        )
    ranked.sort(key=lambda item: (item[0], item[1]))
    selected: list[tuple[float, float]] = []
    for _, _, candidate in ranked:
        if any(
            math.hypot(candidate[0] - previous[0], candidate[1] - previous[1])
            < 0.5
            for previous in selected
        ):
            continue
        selected.append(candidate)
        if len(selected) >= max_candidates:
            break
    return selected


def elevator_start_retreat_target(
    *,
    pose: tuple[float, float, float],
    grid,
    navigation_trace,
    rotation_min_clearance: float,
    retreat_clearance_margin: float,
) -> tuple[float, float] | None:
    """起点仅被实体膨胀覆盖时选择已走直线退路。"""
    start_cell = grid.world_to_grid(float(pose[0]), float(pose[1]))
    if start_cell is None or int(grid.data[start_cell[1], start_cell[0]]) != OCC:
        return None
    required_clearance = float(rotation_min_clearance) + float(
        retreat_clearance_margin
    )
    for trace_point in reversed(list(navigation_trace)):
        target = (float(trace_point[0]), float(trace_point[1]))
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        if distance < 0.5 or distance > 1.5:
            continue
        target_cell = grid.world_to_grid(*target)
        if target_cell is None or not grid.is_traversable(*target_cell):
            continue
        if not grid_cell_has_clearance(grid, target_cell, required_clearance):
            continue
        target_direction = math.atan2(target[1] - pose[1], target[0] - pose[0])
        backward_error = math.atan2(
            math.sin(target_direction - pose[2] - math.pi),
            math.cos(target_direction - pose[2] - math.pi),
        )
        if abs(backward_error) > 0.35:
            continue
        if not initial_occ_escape_segment_is_safe(
            grid,
            (float(pose[0]), float(pose[1])),
            target,
            maximum_initial_occ_distance=0.35,
        ):
            continue
        return target
    return None

"""A* 路径规划与碰撞检测。"""
from __future__ import annotations

import heapq
import math
from typing import Iterable

from danger_search.mapper import FloorGrid


def astar_path(grid: FloorGrid, start_xy: tuple[float, float], goal_xy: tuple[float, float]) -> list[tuple[float, float]]:
    start = grid.world_to_grid(start_xy[0], start_xy[1])
    goal = grid.world_to_grid(goal_xy[0], goal_xy[1])
    if start is None or goal is None:
        return []
    if not grid.is_traversable(*start):
        grid.mark_free(*start)
    if not grid.is_traversable(*goal):
        nearest = _nearest_traversable(grid, goal)
        if nearest is None:
            return []
        goal = nearest

    open_heap: list[tuple[float, tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, start))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost_so_far: dict[tuple[int, int], float] = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            return [grid.grid_to_world(gx, gy) for gx, gy in _reconstruct_path(came_from, current)]
        for neighbor in _neighbors8(grid, current):
            step_cost = math.hypot(neighbor[0] - current[0], neighbor[1] - current[1])
            new_cost = cost_so_far[current] + step_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + _heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, neighbor))
                came_from[neighbor] = current
    return []


def path_length(path: list[tuple[float, float]]) -> float:
    if len(path) < 2:
        return 0.0
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def _neighbors8(grid: FloorGrid, cell: tuple[int, int]) -> Iterable[tuple[int, int]]:
    gx, gy = cell
    for ny in range(gy - 1, gy + 2):
        for nx in range(gx - 1, gx + 2):
            if nx == gx and ny == gy:
                continue
            if grid.is_traversable(nx, ny):
                yield (nx, ny)


def _nearest_traversable(grid: FloorGrid, goal: tuple[int, int], max_radius: int = 8) -> tuple[int, int] | None:
    gx, gy = goal
    for radius in range(1, max_radius + 1):
        for ny in range(gy - radius, gy + radius + 1):
            for nx in range(gx - radius, gx + radius + 1):
                if grid.is_traversable(nx, ny):
                    return (nx, ny)
    return None


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _reconstruct_path(
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

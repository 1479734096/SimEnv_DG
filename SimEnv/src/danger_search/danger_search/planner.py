"""A* 路径规划与碰撞检测。"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from danger_search.mapper import FloorGrid

# 默认迭代预算：在当前配置尺寸（28x48m, 0.10m 分辨率）下，
# 不可达目标的全量探索约 0.5s，会阻塞约 5 个 Timer 周期。
# 20000 次迭代约 0.1s（一个周期量级），既限制阻塞时长，
# 又给正常可达目标留足搜索空间。
DEFAULT_MAX_ITERATIONS = 20000

REACHED = "reached"
UNREACHABLE = "unreachable"
BUDGET_EXHAUSTED = "budget_exhausted"


@dataclass
class PlanResult:
    """A* 规划结果，携带状态以区分“确实不可达”与“预算耗尽”。"""
    path: list[tuple[float, float]] = field(default_factory=list)
    status: str = UNREACHABLE

    @property
    def reached(self) -> bool:
        return self.status == REACHED

    @property
    def budget_exhausted(self) -> bool:
        return self.status == BUDGET_EXHAUSTED

    @property
    def unreachable(self) -> bool:
        return self.status == UNREACHABLE


def astar_plan(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    max_iterations: int | None = DEFAULT_MAX_ITERATIONS,
) -> PlanResult:
    """A* 规划，返回 PlanResult。

    - REACHED：到达目标，path 为完整路径。
    - UNREACHABLE：open_heap 耗尽，确认不可达；上层可黑名单目标。
    - BUDGET_EXHAUSTED：迭代达到上限，可能可达但未搜完；上层不应黑名单，
      应等待地图更新后重试，避免把可达目标误判不可达。
    """
    start = grid.world_to_grid(start_xy[0], start_xy[1])
    goal = grid.world_to_grid(goal_xy[0], goal_xy[1])
    if start is None or goal is None:
        return PlanResult(status=UNREACHABLE)
    if not grid.is_traversable(*start):
        grid.mark_free(*start)
    if not grid.is_traversable(*goal):
        nearest = _nearest_traversable(grid, goal)
        if nearest is None:
            return PlanResult(status=UNREACHABLE)
        goal = nearest

    open_heap: list[tuple[float, tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, start))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost_so_far: dict[tuple[int, int], float] = {start: 0.0}
    closed_set: set[tuple[int, int]] = set()

    iterations = 0
    while open_heap:
        if max_iterations is not None and iterations >= max_iterations:
            return PlanResult(status=BUDGET_EXHAUSTED)
        iterations += 1
        _, current = heapq.heappop(open_heap)
        if current in closed_set:
            continue

        closed_set.add(current)
        if current == goal:
            path = [grid.grid_to_world(gx, gy) for gx, gy in _reconstruct_path(came_from, current)]
            return PlanResult(path=path, status=REACHED)

        gx, gy = current
        for dx, dy in ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)):
            nx, ny = gx + dx, gy + dy
            if not grid.is_traversable(nx, ny):
                continue
            # 对角线移动时，检查相邻两个正交格是否同时可通行，防止切角
            if dx != 0 and dy != 0:
                if not grid.is_traversable(gx + dx, gy) or not grid.is_traversable(gx, gy + dy):
                    continue
            neighbor = (nx, ny)
            if neighbor in closed_set:
                continue
            step_cost = 1.414 if dx != 0 and dy != 0 else 1.0
            new_cost = cost_so_far[current] + step_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + _heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, neighbor))
                came_from[neighbor] = current

    return PlanResult(status=UNREACHABLE)


def astar_path(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    max_iterations: int | None = DEFAULT_MAX_ITERATIONS,
) -> list[tuple[float, float]]:
    """兼容包装：仅返回路径，空列表表示未到达目标（不可达或预算耗尽）。

    需要区分“不可达”与“预算耗尽”的调用方应改用 astar_plan()。
    """
    return astar_plan(grid, start_xy, goal_xy, max_iterations=max_iterations).path


def path_length(path: list[tuple[float, float]]) -> float:
    if len(path) < 2:
        return 0.0
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


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

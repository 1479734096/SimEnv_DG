"""前沿探索与栅格 A* 规划。"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import time
from typing import Iterable

import numpy as np

from danger_search.multi_floor_mapper import BLACK, FREE, FloorGrid, OCC, UNKNOWN, VISITED


@dataclass(frozen=True)
class Frontier:
    cells: tuple[tuple[int, int], ...]
    centroid: tuple[float, float]
    information_gain: float
    score: float = 0.0


class FrontierExplorer:
    """负责提取前沿、评分、黑名单和 A* 规划。"""

    def __init__(
        self,
        *,
        min_cluster_size: int = 8,
        blacklist_ttl: float = 60.0,
        fail_limit: int = 3,
        information_gain_weight: float = 2.0,
        path_cost_weight: float = 1.0,
        yaw_cost_weight: float = 0.3,
        revisit_penalty_weight: float = 2.0,
    ):
        self.min_cluster_size = int(min_cluster_size)
        self.blacklist_ttl = float(blacklist_ttl)
        self.fail_limit = int(fail_limit)
        self.information_gain_weight = float(information_gain_weight)
        self.path_cost_weight = float(path_cost_weight)
        self.yaw_cost_weight = float(yaw_cost_weight)
        self.revisit_penalty_weight = float(revisit_penalty_weight)
        self._blacklist: dict[tuple[int, int], float] = {}
        self._fail_counts: dict[tuple[int, int], int] = {}

    def select_frontier(
        self,
        grid: FloorGrid,
        robot_pose: tuple[float, float, float],
    ) -> tuple[Frontier | None, list[tuple[float, float]]]:
        self._expire_blacklist()
        frontiers = self.extract_frontiers(grid)
        best_frontier: Frontier | None = None
        best_path: list[tuple[float, float]] = []
        best_score = -math.inf
        for frontier in frontiers:
            target_xy = _frontier_goal_for_robot(grid, frontier, (robot_pose[0], robot_pose[1]))
            if self._is_blacklisted(grid, target_xy):
                continue
            path = astar_path(grid, (robot_pose[0], robot_pose[1]), target_xy)
            if not path:
                self.record_failure(grid, target_xy)
                continue
            score = self._score_frontier(frontier, path, robot_pose)
            if score > best_score:
                best_score = score
                best_frontier = Frontier(
                    cells=frontier.cells,
                    centroid=target_xy,
                    information_gain=frontier.information_gain,
                    score=score,
                )
                best_path = path
        if best_frontier is None:
            return self._select_free_space_goal(grid, robot_pose)
        return best_frontier, best_path

    def extract_frontiers(self, grid: FloorGrid) -> list[Frontier]:
        frontier_mask = np.zeros_like(grid.data, dtype=bool)
        for gy in range(1, grid.size_y - 1):
            for gx in range(1, grid.size_x - 1):
                if int(grid.data[gy, gx]) not in (FREE, VISITED):
                    continue
                if _has_unknown_neighbor(grid, gx, gy):
                    frontier_mask[gy, gx] = True

        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters: list[list[tuple[int, int]]] = []
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                if not frontier_mask[gy, gx] or visited[gy, gx]:
                    continue
                cluster = _flood_fill(frontier_mask, visited, gx, gy)
                if len(cluster) >= self.min_cluster_size:
                    clusters.append(cluster)

        frontiers: list[Frontier] = []
        for cluster in clusters:
            xs = [cell[0] for cell in cluster]
            ys = [cell[1] for cell in cluster]
            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)
            target_cell = min(
                cluster,
                key=lambda cell: (cell[0] - mean_x) * (cell[0] - mean_x) + (cell[1] - mean_y) * (cell[1] - mean_y),
            )
            centroid = grid.grid_to_world(*target_cell)
            frontiers.append(
                Frontier(
                    cells=tuple(cluster),
                    centroid=centroid,
                    information_gain=float(len(cluster)),
                )
            )
        return frontiers

    def record_failure(self, grid: FloorGrid, world_xy: tuple[float, float]) -> None:
        cell = grid.world_to_grid(world_xy[0], world_xy[1])
        if cell is None:
            return
        self._fail_counts[cell] = self._fail_counts.get(cell, 0) + 1
        if self._fail_counts[cell] >= self.fail_limit:
            self.blacklist_cell(cell)

    def blacklist_cell(self, cell: tuple[int, int]) -> None:
        self._blacklist[cell] = time.time() + self.blacklist_ttl

    def _is_blacklisted(self, grid: FloorGrid, world_xy: tuple[float, float]) -> bool:
        cell = grid.world_to_grid(world_xy[0], world_xy[1])
        return cell in self._blacklist if cell is not None else True

    def _expire_blacklist(self) -> None:
        now = time.time()
        expired = [cell for cell, expiry in self._blacklist.items() if expiry <= now]
        for cell in expired:
            self._blacklist.pop(cell, None)

    def _score_frontier(
        self,
        frontier: Frontier,
        path: list[tuple[float, float]],
        robot_pose: tuple[float, float, float],
    ) -> float:
        path_cost = _path_length(path)
        target_yaw = math.atan2(frontier.centroid[1] - robot_pose[1], frontier.centroid[0] - robot_pose[0])
        yaw_cost = abs(_angle_wrap(target_yaw - robot_pose[2]))
        revisit_penalty = 1.0 if path_cost < 0.8 else 0.0
        return (
            self.information_gain_weight * frontier.information_gain
            - self.path_cost_weight * path_cost
            - self.yaw_cost_weight * yaw_cost
            - self.revisit_penalty_weight * revisit_penalty
        )

    def _select_free_space_goal(
        self,
        grid: FloorGrid,
        robot_pose: tuple[float, float, float],
    ) -> tuple[Frontier | None, list[tuple[float, float]]]:
        robot_xy = (robot_pose[0], robot_pose[1])
        stride = max(1, int(round(0.4 / grid.resolution)))
        candidates: list[tuple[float, tuple[int, int]]] = []
        for gy in range(0, grid.size_y, stride):
            for gx in range(0, grid.size_x, stride):
                if int(grid.data[gy, gx]) != FREE:
                    continue
                wx, wy = grid.grid_to_world(gx, gy)
                distance = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
                if distance < 1.0 or self._is_blacklisted(grid, (wx, wy)):
                    continue
                candidates.append((distance, (gx, gy)))
        candidates.sort(reverse=True)
        for _, cell in candidates[:30]:
            goal = grid.grid_to_world(*cell)
            path = astar_path(grid, robot_xy, goal)
            if path:
                frontier = Frontier(cells=(cell,), centroid=goal, information_gain=0.0, score=-_path_length(path))
                return frontier, path
            self.record_failure(grid, goal)
        return None, []


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


def _has_unknown_neighbor(grid: FloorGrid, gx: int, gy: int) -> bool:
    for ny in range(gy - 1, gy + 2):
        for nx in range(gx - 1, gx + 2):
            if nx == gx and ny == gy:
                continue
            if int(grid.data[ny, nx]) == UNKNOWN:
                return True
    return False


def _frontier_goal_for_robot(
    grid: FloorGrid,
    frontier: Frontier,
    robot_xy: tuple[float, float],
) -> tuple[float, float]:
    def distance_sq(cell: tuple[int, int]) -> float:
        wx, wy = grid.grid_to_world(cell[0], cell[1])
        return (wx - robot_xy[0]) * (wx - robot_xy[0]) + (wy - robot_xy[1]) * (wy - robot_xy[1])

    target_cell = max(frontier.cells, key=distance_sq)
    return grid.grid_to_world(*target_cell)


def _flood_fill(mask: np.ndarray, visited: np.ndarray, start_x: int, start_y: int) -> list[tuple[int, int]]:
    stack = [(start_x, start_y)]
    visited[start_y, start_x] = True
    cluster: list[tuple[int, int]] = []
    while stack:
        gx, gy = stack.pop()
        cluster.append((gx, gy))
        for nx, ny in ((gx + 1, gy), (gx - 1, gy), (gx, gy + 1), (gx, gy - 1)):
            if 0 <= nx < mask.shape[1] and 0 <= ny < mask.shape[0] and mask[ny, nx] and not visited[ny, nx]:
                visited[ny, nx] = True
                stack.append((nx, ny))
    return cluster


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


def _path_length(path: list[tuple[float, float]]) -> float:
    if len(path) < 2:
        return 0.0
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def _angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

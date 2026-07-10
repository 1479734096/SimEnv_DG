"""前沿探索。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time

import numpy as np

try:
    import rospy
except ImportError:
    rospy = None

from danger_search.mapper import BLACK, FREE, FloorGrid, OCC, UNKNOWN, VISITED
from danger_search.planner import astar_path, path_length
from danger_search.time_utils import get_time


@dataclass(frozen=True)
class Frontier:
    cells: tuple[tuple[int, int], ...]
    centroid: tuple[float, float]
    information_gain: float
    score: float = 0.0


@dataclass(frozen=True)
class FrontierSelectionContext:
    forbidden_half_plane_origin: tuple[float, float] | None = None
    forbidden_half_plane_normal: tuple[float, float] | None = None
    forbidden_radius: float = 0.0
    skip_in_forbidden_half_plane: bool = False
    preferred_origin: tuple[float, float] | None = None
    preferred_direction: tuple[float, float] | None = None
    temporary_revisit_penalty: float = 0.0
    current_target: tuple[float, float] | None = None
    current_score: float | None = None
    current_region_radius: float = 0.0
    switch_score_margin: float = 0.0


@dataclass(frozen=True)
class FrontierSelectionDebug:
    candidate_count: int
    reachable_count: int
    blacklisted_count: int
    unreachable_count: int
    context_filtered_count: int
    used_fallback: bool
    final_target: tuple[float, float] | None
    final_score: float
    path_length: float
    retained_by_inertia: bool = False
    inertia_reason: str = ""
    previous_target: tuple[float, float] | None = None
    previous_score: float | None = None
    current_region_score: float | None = None
    switch_score_margin: float = 0.0


class FrontierExplorer:
    """负责提取前沿、评分、黑名单管理。"""

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
        frontier_obstacle_clearance: float = 0.25,
        frontier_unknown_standoff: float = 0.20,
        frontier_min_information_gain: float = 1.0,
        door_front_half_plane_penalty: float = 6.0,
        door_front_half_plane_radius: float = 1.5,
        door_preferred_direction_bonus: float = 1.5,
    ):
        self.min_cluster_size = int(min_cluster_size)
        self.blacklist_ttl = float(blacklist_ttl)
        self.fail_limit = int(fail_limit)
        self.information_gain_weight = float(information_gain_weight)
        self.path_cost_weight = float(path_cost_weight)
        self.yaw_cost_weight = float(yaw_cost_weight)
        self.revisit_penalty_weight = float(revisit_penalty_weight)
        self.frontier_obstacle_clearance = float(frontier_obstacle_clearance)
        self.frontier_unknown_standoff = float(frontier_unknown_standoff)
        self.frontier_min_information_gain = float(frontier_min_information_gain)
        self.door_front_half_plane_penalty = float(door_front_half_plane_penalty)
        self.door_front_half_plane_radius = float(door_front_half_plane_radius)
        self.door_preferred_direction_bonus = float(door_preferred_direction_bonus)
        self._blacklist: dict[tuple[int, int], float] = {}
        self._fail_counts: dict[tuple[int, int], int] = {}
        self.last_selection_debug: FrontierSelectionDebug | None = None

    def select_frontier(
        self,
        grid: FloorGrid,
        robot_pose: tuple[float, float, float],
        selection_context: FrontierSelectionContext | None = None,
    ) -> tuple[Frontier | None, list[tuple[float, float]]]:
        self._expire_blacklist()
        frontiers = self.extract_frontiers(grid)
        best_frontier: Frontier | None = None
        best_path: list[tuple[float, float]] = []
        best_score = -math.inf
        best_current_region_frontier: Frontier | None = None
        best_current_region_path: list[tuple[float, float]] = []
        best_current_region_score = -math.inf
        blacklisted_count = 0
        unreachable_count = 0
        reachable_count = 0
        context_filtered_count = 0
        for frontier in frontiers:
            target_xy, path, blacklisted_only, context_adjustment, context_filtered_only = self._resolve_frontier_target(
                grid,
                frontier,
                robot_pose,
                selection_context=selection_context,
            )
            if target_xy is None or not path:
                if blacklisted_only:
                    blacklisted_count += 1
                elif context_filtered_only:
                    context_filtered_count += 1
                else:
                    unreachable_count += 1
                    self.record_failure(grid, frontier.centroid)
                continue
            reachable_count += 1
            score = self._score_frontier(frontier, path, robot_pose, target_xy=target_xy) + context_adjustment
            if score > best_score:
                best_score = score
                best_frontier = Frontier(
                    cells=frontier.cells,
                    centroid=target_xy,
                    information_gain=frontier.information_gain,
                    score=score,
                )
                best_path = path
            if (
                selection_context is not None
                and selection_context.current_target is not None
                and selection_context.current_region_radius > 0.0
                and _world_distance(target_xy, selection_context.current_target)
                <= selection_context.current_region_radius
                and score > best_current_region_score
            ):
                best_current_region_score = score
                best_current_region_frontier = Frontier(
                    cells=frontier.cells,
                    centroid=target_xy,
                    information_gain=frontier.information_gain,
                    score=score,
                )
                best_current_region_path = path
        if best_frontier is None:
            fallback_frontier, fallback_path = self._select_free_space_goal(
                grid,
                robot_pose,
                selection_context=selection_context,
            )
            self._log_selection_debug(
                candidate_count=len(frontiers),
                reachable_count=reachable_count,
                blacklisted_count=blacklisted_count,
                unreachable_count=unreachable_count,
                context_filtered_count=context_filtered_count,
                used_fallback=True,
                frontier=fallback_frontier,
                path=fallback_path,
                debug=FrontierSelectionDebug(
                    candidate_count=len(frontiers),
                    reachable_count=reachable_count,
                    blacklisted_count=blacklisted_count,
                    unreachable_count=unreachable_count,
                    context_filtered_count=context_filtered_count,
                    used_fallback=True,
                    final_target=fallback_frontier.centroid if fallback_frontier is not None else None,
                    final_score=fallback_frontier.score if fallback_frontier is not None else 0.0,
                    path_length=path_length(fallback_path) if fallback_path else 0.0,
                    previous_target=selection_context.current_target if selection_context is not None else None,
                    previous_score=selection_context.current_score if selection_context is not None else None,
                    switch_score_margin=selection_context.switch_score_margin if selection_context is not None else 0.0,
                ),
            )
            return fallback_frontier, fallback_path
        retained_by_inertia = False
        inertia_reason = ""
        if (
            selection_context is not None
            and best_current_region_frontier is not None
            and best_frontier.centroid != best_current_region_frontier.centroid
        ):
            margin = max(0.0, float(selection_context.switch_score_margin))
            if best_score < best_current_region_score + margin:
                retained_by_inertia = True
                inertia_reason = (
                    f"best_score={best_score:.2f} < current_region_score={best_current_region_score:.2f}"
                    f" + margin={margin:.2f}"
                )
                best_frontier = best_current_region_frontier
                best_path = best_current_region_path
                best_score = best_current_region_score
        self._log_selection_debug(
            candidate_count=len(frontiers),
            reachable_count=reachable_count,
            blacklisted_count=blacklisted_count,
            unreachable_count=unreachable_count,
            context_filtered_count=context_filtered_count,
            used_fallback=False,
            frontier=best_frontier,
            path=best_path,
            debug=FrontierSelectionDebug(
                candidate_count=len(frontiers),
                reachable_count=reachable_count,
                blacklisted_count=blacklisted_count,
                unreachable_count=unreachable_count,
                context_filtered_count=context_filtered_count,
                used_fallback=False,
                final_target=best_frontier.centroid,
                final_score=best_frontier.score,
                path_length=path_length(best_path) if best_path else 0.0,
                retained_by_inertia=retained_by_inertia,
                inertia_reason=inertia_reason,
                previous_target=selection_context.current_target if selection_context is not None else None,
                previous_score=selection_context.current_score if selection_context is not None else None,
                current_region_score=best_current_region_score if best_current_region_frontier is not None else None,
                switch_score_margin=selection_context.switch_score_margin if selection_context is not None else 0.0,
            ),
        )
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
            information_gain = float(len(cluster))
            if information_gain < self.frontier_min_information_gain:
                continue
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
                    information_gain=information_gain,
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
        self._blacklist[cell] = get_time() + self.blacklist_ttl

    def _is_blacklisted(self, grid: FloorGrid, world_xy: tuple[float, float]) -> bool:
        cell = grid.world_to_grid(world_xy[0], world_xy[1])
        return cell in self._blacklist if cell is not None else True

    def _expire_blacklist(self) -> None:
        now = get_time()
        expired = [cell for cell, expiry in self._blacklist.items() if expiry <= now]
        for cell in expired:
            self._blacklist.pop(cell, None)

    def _score_frontier(
        self,
        frontier: Frontier,
        path: list[tuple[float, float]],
        robot_pose: tuple[float, float, float],
        *,
        target_xy: tuple[float, float] | None = None,
    ) -> float:
        cost = path_length(path)
        goal_xy = target_xy if target_xy is not None else frontier.centroid
        target_yaw = math.atan2(goal_xy[1] - robot_pose[1], goal_xy[0] - robot_pose[0])
        yaw_cost = abs(_angle_wrap(target_yaw - robot_pose[2]))
        revisit_penalty = 1.0 if cost < 0.8 else 0.0
        capped_ig = min(float(frontier.information_gain), 25.0)
        return (
            self.information_gain_weight * capped_ig
            - self.path_cost_weight * cost
            - self.yaw_cost_weight * yaw_cost
            - self.revisit_penalty_weight * revisit_penalty
        )

    def _select_free_space_goal(
        self,
        grid: FloorGrid,
        robot_pose: tuple[float, float, float],
        selection_context: FrontierSelectionContext | None = None,
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
        best_choice: tuple[Frontier, list[tuple[float, float]], float] | None = None
        for _, cell in candidates[:30]:
            goal = grid.grid_to_world(*cell)
            context_adjustment, skip_target = self._selection_context_adjustment(goal, selection_context)
            if skip_target:
                continue
            path = astar_path(grid, robot_xy, goal)
            if path:
                score = -path_length(path) + context_adjustment
                frontier = Frontier(cells=(cell,), centroid=goal, information_gain=0.0, score=score)
                if best_choice is None or score > best_choice[2]:
                    best_choice = (frontier, path, score)
            else:
                self.record_failure(grid, goal)
        if best_choice is not None:
            return best_choice[0], best_choice[1]
        return None, []

    def _resolve_frontier_target(
        self,
        grid: FloorGrid,
        frontier: Frontier,
        robot_pose: tuple[float, float, float],
        *,
        selection_context: FrontierSelectionContext | None = None,
    ) -> tuple[tuple[float, float] | None, list[tuple[float, float]], bool, float, bool]:
        candidates = self._candidate_targets_for_frontier(grid, frontier, (robot_pose[0], robot_pose[1]))
        if not candidates:
            return None, [], False, 0.0, False
        saw_non_blacklisted = False
        saw_context_skip = False
        saw_path_attempt = False
        best_choice: tuple[tuple[float, float], list[tuple[float, float]], float, float] | None = None
        for target_xy in candidates:
            if self._is_blacklisted(grid, target_xy):
                continue
            saw_non_blacklisted = True
            context_adjustment, skip_target = self._selection_context_adjustment(target_xy, selection_context)
            if skip_target:
                saw_context_skip = True
                continue
            saw_path_attempt = True
            path = astar_path(grid, (robot_pose[0], robot_pose[1]), target_xy)
            if path:
                if selection_context is None:
                    return target_xy, path, False, 0.0, False
                candidate_score = context_adjustment - path_length(path)
                if best_choice is None or candidate_score > best_choice[3]:
                    best_choice = (target_xy, path, context_adjustment, candidate_score)
        if best_choice is not None:
            return best_choice[0], best_choice[1], False, best_choice[2], False
        return None, [], not saw_non_blacklisted, 0.0, saw_context_skip and not saw_path_attempt

    def _candidate_targets_for_frontier(
        self,
        grid: FloorGrid,
        frontier: Frontier,
        robot_xy: tuple[float, float],
    ) -> list[tuple[float, float]]:
        safe_cells = self._safe_goal_cells_for_frontier(grid, frontier, robot_xy)
        if safe_cells:
            return [grid.grid_to_world(*cell) for cell in safe_cells]
        return [_frontier_goal_for_robot(grid, frontier, robot_xy)]

    def _safe_goal_cells_for_frontier(
        self,
        grid: FloorGrid,
        frontier: Frontier,
        robot_xy: tuple[float, float],
    ) -> list[tuple[int, int]]:
        frontier_cells = set(frontier.cells)
        standoff_steps = 0
        if self.frontier_unknown_standoff > 0.0:
            standoff_steps = max(1, int(math.ceil(self.frontier_unknown_standoff / grid.resolution)))
        clearance_steps = 0
        if self.frontier_obstacle_clearance > 0.0:
            clearance_steps = max(1, int(math.ceil(self.frontier_obstacle_clearance / grid.resolution)))
        max_search_steps = max(standoff_steps + clearance_steps + 6, standoff_steps + 2)

        queue = deque((cell, 0) for cell in frontier.cells)
        visited = set(frontier.cells)
        candidate_cells: list[tuple[int, int]] = []
        best_steps: int | None = None
        while queue:
            cell, steps = queue.popleft()
            if best_steps is not None and steps > best_steps:
                break
            if steps > max_search_steps:
                continue
            if (
                steps >= standoff_steps
                and cell not in frontier_cells
                and self._has_blocked_clearance(grid, cell, clearance_steps)
            ):
                candidate_cells.append(cell)
                best_steps = steps
                continue
            if steps == max_search_steps:
                continue
            for neighbor in _neighbors8_cells(cell):
                if neighbor in visited:
                    continue
                if not grid.is_traversable(*neighbor):
                    continue
                visited.add(neighbor)
                queue.append((neighbor, steps + 1))
        distance_search_steps = max(clearance_steps + 2, 4)
        candidate_cells.sort(
            key=lambda cell: (
                -self._distance_to_blocked(grid, cell, max_search_steps=distance_search_steps),
                -_world_distance_sq(grid, cell, robot_xy),
            )
        )
        return candidate_cells

    def _has_blocked_clearance(
        self,
        grid: FloorGrid,
        cell: tuple[int, int],
        clearance_steps: int,
    ) -> bool:
        if clearance_steps <= 0:
            return True
        gx, gy = cell
        min_y = max(0, gy - clearance_steps)
        max_y = min(grid.size_y, gy + clearance_steps + 1)
        min_x = max(0, gx - clearance_steps)
        max_x = min(grid.size_x, gx + clearance_steps + 1)
        for ny in range(min_y, max_y):
            for nx in range(min_x, max_x):
                if int(grid.data[ny, nx]) not in (OCC, BLACK):
                    continue
                if math.hypot(nx - gx, ny - gy) < float(clearance_steps):
                    return False
        return True

    def _distance_to_blocked(
        self,
        grid: FloorGrid,
        cell: tuple[int, int],
        *,
        max_search_steps: int = 10,
    ) -> float:
        gx, gy = cell
        search_steps = max(1, int(max_search_steps))
        best = float(search_steps)
        min_y = max(0, gy - search_steps)
        max_y = min(grid.size_y, gy + search_steps + 1)
        min_x = max(0, gx - search_steps)
        max_x = min(grid.size_x, gx + search_steps + 1)
        for ny in range(min_y, max_y):
            for nx in range(min_x, max_x):
                if int(grid.data[ny, nx]) not in (OCC, BLACK):
                    continue
                distance = math.hypot(nx - gx, ny - gy)
                if distance < best:
                    best = distance
        return best

    def _selection_context_adjustment(
        self,
        target_xy: tuple[float, float],
        selection_context: FrontierSelectionContext | None,
    ) -> tuple[float, bool]:
        if selection_context is None:
            return 0.0, False
        adjustment = 0.0
        signed_distance = self._signed_half_plane_distance(
            target_xy,
            selection_context.forbidden_half_plane_origin,
            selection_context.forbidden_half_plane_normal,
        )
        if signed_distance is not None and signed_distance < -0.05:
            if selection_context.skip_in_forbidden_half_plane:
                return 0.0, True
            origin = selection_context.forbidden_half_plane_origin
            if origin is not None:
                distance_to_origin = math.hypot(target_xy[0] - origin[0], target_xy[1] - origin[1])
                if distance_to_origin <= max(0.0, selection_context.forbidden_radius):
                    return 0.0, True
            adjustment -= max(0.0, selection_context.temporary_revisit_penalty)
        preferred_progress = self._signed_half_plane_distance(
            target_xy,
            selection_context.preferred_origin,
            selection_context.preferred_direction,
        )
        if preferred_progress is not None and preferred_progress > 0.05:
            adjustment += self.door_preferred_direction_bonus * min(preferred_progress, 1.0)
        return adjustment, False

    @staticmethod
    def _signed_half_plane_distance(
        target_xy: tuple[float, float],
        origin: tuple[float, float] | None,
        direction: tuple[float, float] | None,
    ) -> float | None:
        if origin is None or direction is None:
            return None
        norm = math.hypot(direction[0], direction[1])
        if norm <= 1e-6:
            return None
        dir_x = direction[0] / norm
        dir_y = direction[1] / norm
        return (target_xy[0] - origin[0]) * dir_x + (target_xy[1] - origin[1]) * dir_y

    def _log_selection_debug(
        self,
        *,
        candidate_count: int,
        reachable_count: int,
        blacklisted_count: int,
        unreachable_count: int,
        context_filtered_count: int,
        used_fallback: bool,
        frontier: Frontier | None,
        path: list[tuple[float, float]],
        debug: FrontierSelectionDebug,
    ) -> None:
        self.last_selection_debug = debug
        if rospy is None:
            return
        if frontier is None:
            rospy.loginfo(
                "[frontier_debug] 候选前沿=%d 可达=%d 黑名单=%d 不可达=%d 上下文过滤=%d fallback_far_free=%s final_target=None",
                candidate_count,
                reachable_count,
                blacklisted_count,
                unreachable_count,
                context_filtered_count,
                used_fallback,
            )
            return
        rospy.loginfo(
            "[frontier_debug] 候选前沿=%d 可达=%d 黑名单=%d 不可达=%d 上下文过滤=%d fallback_far_free=%s final_target=(%.2f, %.2f) score=%.2f path_length=%.2f",
            candidate_count,
            reachable_count,
            blacklisted_count,
            unreachable_count,
            context_filtered_count,
            used_fallback,
            frontier.centroid[0],
            frontier.centroid[1],
            frontier.score,
            path_length(path) if path else 0.0,
        )


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


def _neighbors8_cells(cell: tuple[int, int]) -> list[tuple[int, int]]:
    gx, gy = cell
    return [
        (gx + 1, gy),
        (gx - 1, gy),
        (gx, gy + 1),
        (gx, gy - 1),
        (gx + 1, gy + 1),
        (gx + 1, gy - 1),
        (gx - 1, gy + 1),
        (gx - 1, gy - 1),
    ]


def _world_distance_sq(grid: FloorGrid, cell: tuple[int, int], robot_xy: tuple[float, float]) -> float:
    wx, wy = grid.grid_to_world(cell[0], cell[1])
    return (wx - robot_xy[0]) * (wx - robot_xy[0]) + (wy - robot_xy[1]) * (wy - robot_xy[1])


def _world_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


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


def _angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

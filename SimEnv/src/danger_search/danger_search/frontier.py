"""前沿探索。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import json
import math
import time

import numpy as np

try:
    import rospy
except ImportError:
    rospy = None

from danger_search.mapper import BLACK, FREE, FloorGrid, OCC, UNKNOWN, VISITED
from danger_search.planner import UNREACHABLE, astar_plan, path_length
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
    permanent_forbidden_half_plane_origin: tuple[float, float] | None = None
    permanent_forbidden_half_plane_normal: tuple[float, float] | None = None
    permanent_forbidden_radius: float = 3.0
    permanent_forbidden_penalty: float = 0.0
    permanent_forbidden_decay_distance: float = 6.0
    permanent_gate_half_width: float = 0.0
    door_id: str | None = None
    temporary_context_expires_at: float | None = None
    context_time: float | None = None
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
    budget_exhausted_count: int = 0
    inside_tolerance_count: int = 0
    retained_by_inertia: bool = False
    inertia_reason: str = ""
    previous_target: tuple[float, float] | None = None
    previous_score: float | None = None
    current_region_score: float | None = None
    switch_score_margin: float = 0.0
    selection_wall_duration: float = 0.0
    candidates: tuple["FrontierCandidateDebug", ...] = ()


@dataclass(frozen=True)
class FrontierCandidateDebug:
    position: tuple[float, float]
    cluster_size: int
    information_gain: float
    euclidean_distance: float
    path_length: float | None
    final_score: float | None
    rejection_reason: str
    door_context_adjustment: float
    door_context_penalty: float | None
    blacklist_status: str
    reachability_result: str
    source: str = "frontier"
    selected: bool = False


@dataclass(frozen=True)
class _FrontierExtractionRejection:
    position: tuple[float, float]
    cluster_size: int
    information_gain: float
    rejection_reason: str


class FrontierExplorer:
    """负责提取前沿、评分、黑名单管理。"""

    def __init__(
        self,
        *,
        min_cluster_size: int = 8,
        blacklist_ttl: float = 60.0,
        fail_limit: int = 3,
        stuck_blacklist_radius: float = 1.20,
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
        max_planning_candidates: int = 8,
        max_goal_candidates_per_frontier: int = 2,
        max_fallback_planning_candidates: int = 8,
        goal_tolerance: float = 0.35,
    ):
        self.min_cluster_size = int(min_cluster_size)
        self.blacklist_ttl = float(blacklist_ttl)
        self.fail_limit = int(fail_limit)
        self.stuck_blacklist_radius = max(0.0, float(stuck_blacklist_radius))
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
        self.max_planning_candidates = max(1, int(max_planning_candidates))
        self.max_goal_candidates_per_frontier = max(1, int(max_goal_candidates_per_frontier))
        self.max_fallback_planning_candidates = max(1, int(max_fallback_planning_candidates))
        self.goal_tolerance = float(goal_tolerance)
        self._blacklist: dict[tuple[int, int], float] = {}
        self._fail_counts: dict[tuple[int, int], int] = {}
        self.last_selection_debug: FrontierSelectionDebug | None = None
        self._last_extraction_rejections: list[_FrontierExtractionRejection] = []

    def select_frontier(
        self,
        grid: FloorGrid,
        robot_pose: tuple[float, float, float],
        selection_context: FrontierSelectionContext | None = None,
        *,
        frontiers: list[Frontier] | None = None,
    ) -> tuple[Frontier | None, list[tuple[float, float]]]:
        selection_started = time.perf_counter()
        self._expire_blacklist()
        if frontiers is None:
            frontiers = self.extract_frontiers(grid)
        candidate_debug_records = [
            FrontierCandidateDebug(
                position=rejection.position,
                cluster_size=rejection.cluster_size,
                information_gain=rejection.information_gain,
                euclidean_distance=_world_distance(rejection.position, (robot_pose[0], robot_pose[1])),
                path_length=None,
                final_score=None,
                rejection_reason=rejection.rejection_reason,
                door_context_adjustment=0.0,
                door_context_penalty=0.0,
                blacklist_status="not_evaluated",
                reachability_result="not_evaluated",
            )
            for rejection in self._last_extraction_rejections
        ]
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
        budget_exhausted_count = 0
        inside_tolerance_count = 0
        ranked_frontiers = sorted(
            frontiers,
            key=lambda frontier: self._optimistic_score(frontier, robot_pose),
            reverse=True,
        )
        planned_frontiers = ranked_frontiers[: self.max_planning_candidates]
        for frontier in ranked_frontiers[self.max_planning_candidates :]:
            candidate_debug_records.append(
                FrontierCandidateDebug(
                    position=frontier.centroid,
                    cluster_size=len(frontier.cells),
                    information_gain=frontier.information_gain,
                    euclidean_distance=_world_distance(
                        frontier.centroid,
                        (robot_pose[0], robot_pose[1]),
                    ),
                    path_length=None,
                    final_score=None,
                    rejection_reason="planning_candidate_limit",
                    door_context_adjustment=0.0,
                    door_context_penalty=0.0,
                    blacklist_status="not_evaluated",
                    reachability_result="not_evaluated",
                )
            )
        for frontier in planned_frontiers:
            (
                target_xy,
                path,
                blacklisted_only,
                context_adjustment,
                context_filtered_only,
                budget_exhausted,
                inside_tolerance_only,
                candidate_debug,
            ) = self._resolve_frontier_target(
                grid,
                frontier,
                robot_pose,
                selection_context=selection_context,
            )
            if target_xy is None or not path:
                candidate_debug_records.append(candidate_debug)
                if blacklisted_only:
                    blacklisted_count += 1
                elif context_filtered_only:
                    context_filtered_count += 1
                elif budget_exhausted:
                    # 预算耗尽：可能可达，不累计失败，留待下轮地图更新后重试。
                    budget_exhausted_count += 1
                elif inside_tolerance_only:
                    # 落点在 controller goal_tolerance 内：不算失败、不拉黑、不屏蔽 cluster。
                    # 跳过此 frontier，继续尝试其他 frontier。
                    inside_tolerance_count += 1
                else:
                    unreachable_count += 1
                    self.record_failure(grid, frontier.centroid)
                continue
            reachable_count += 1
            score = self._score_frontier(frontier, path, robot_pose, target_xy=target_xy) + context_adjustment
            candidate_debug_records.append(
                replace(
                    candidate_debug,
                    path_length=path_length(path),
                    final_score=score,
                    rejection_reason="",
                    reachability_result="reached",
                )
            )
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
                candidate_debug_records=candidate_debug_records,
            )
            final_target = fallback_frontier.centroid if fallback_frontier is not None else None
            top_candidates = self._top_candidate_debug(candidate_debug_records, final_target)
            self._log_selection_debug(
                candidate_count=len(frontiers),
                reachable_count=reachable_count,
                blacklisted_count=blacklisted_count,
                unreachable_count=unreachable_count,
                context_filtered_count=context_filtered_count,
                    budget_exhausted_count=budget_exhausted_count,
                    inside_tolerance_count=inside_tolerance_count,
                    used_fallback=True,
                frontier=fallback_frontier,
                path=fallback_path,
                selection_context=selection_context,
                debug=FrontierSelectionDebug(
                    candidate_count=len(frontiers),
                    reachable_count=reachable_count,
                    blacklisted_count=blacklisted_count,
                    unreachable_count=unreachable_count,
                    context_filtered_count=context_filtered_count,
                budget_exhausted_count=budget_exhausted_count,
                inside_tolerance_count=inside_tolerance_count,
                used_fallback=True,
                final_target=final_target,
                    final_score=fallback_frontier.score if fallback_frontier is not None else 0.0,
                    path_length=path_length(fallback_path) if fallback_path else 0.0,
                    previous_target=selection_context.current_target if selection_context is not None else None,
                    previous_score=selection_context.current_score if selection_context is not None else None,
                    switch_score_margin=selection_context.switch_score_margin if selection_context is not None else 0.0,
                    selection_wall_duration=time.perf_counter() - selection_started,
                    candidates=top_candidates,
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
        top_candidates = self._top_candidate_debug(candidate_debug_records, best_frontier.centroid)
        self._log_selection_debug(
            candidate_count=len(frontiers),
            reachable_count=reachable_count,
            blacklisted_count=blacklisted_count,
            unreachable_count=unreachable_count,
            context_filtered_count=context_filtered_count,
            budget_exhausted_count=budget_exhausted_count,
            inside_tolerance_count=inside_tolerance_count,
            used_fallback=False,
            frontier=best_frontier,
            path=best_path,
            selection_context=selection_context,
            debug=FrontierSelectionDebug(
                candidate_count=len(frontiers),
                reachable_count=reachable_count,
                blacklisted_count=blacklisted_count,
                unreachable_count=unreachable_count,
                context_filtered_count=context_filtered_count,
                budget_exhausted_count=budget_exhausted_count,
                inside_tolerance_count=inside_tolerance_count,
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
                selection_wall_duration=time.perf_counter() - selection_started,
                candidates=top_candidates,
            ),
        )
        return best_frontier, best_path

    def extract_frontiers(self, grid: FloorGrid) -> list[Frontier]:
        self._last_extraction_rejections = []
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
                clusters.append(cluster)

        frontiers: list[Frontier] = []
        for cluster in clusters:
            information_gain = float(len(cluster))
            xs = [cell[0] for cell in cluster]
            ys = [cell[1] for cell in cluster]
            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)
            target_cell = min(
                cluster,
                key=lambda cell: (cell[0] - mean_x) * (cell[0] - mean_x) + (cell[1] - mean_y) * (cell[1] - mean_y),
            )
            centroid = grid.grid_to_world(*target_cell)
            if len(cluster) < self.min_cluster_size:
                self._last_extraction_rejections.append(
                    _FrontierExtractionRejection(
                        position=centroid,
                        cluster_size=len(cluster),
                        information_gain=information_gain,
                        rejection_reason="cluster_below_min_size",
                    )
                )
                continue
            if information_gain < self.frontier_min_information_gain:
                self._last_extraction_rejections.append(
                    _FrontierExtractionRejection(
                        position=centroid,
                        cluster_size=len(cluster),
                        information_gain=information_gain,
                        rejection_reason="information_gain_below_minimum",
                    )
                )
                continue
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

    def record_stuck_failure(self, grid: FloorGrid, world_xy: tuple[float, float]) -> None:
        """临时排除已确认卡住目标附近区域，避免前沿质心漂移绕过单格黑名单。"""
        center = grid.world_to_grid(world_xy[0], world_xy[1])
        if center is None:
            return
        radius = max(grid.resolution, self.stuck_blacklist_radius)
        radius_cells = int(math.ceil(radius / grid.resolution))
        expiry = get_time() + self.blacklist_ttl
        for gy in range(center[1] - radius_cells, center[1] + radius_cells + 1):
            for gx in range(center[0] - radius_cells, center[0] + radius_cells + 1):
                if not (0 <= gx < grid.size_x and 0 <= gy < grid.size_y):
                    continue
                wx, wy = grid.grid_to_world(gx, gy)
                if math.hypot(wx - world_xy[0], wy - world_xy[1]) <= radius:
                    self._blacklist[(gx, gy)] = expiry

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
        effective_ig = math.sqrt(min(float(frontier.information_gain), 25.0))
        return (
            self.information_gain_weight * effective_ig
            - self.path_cost_weight * cost
            - self.yaw_cost_weight * yaw_cost
            - self.revisit_penalty_weight * revisit_penalty
        )

    def _optimistic_score(
        self,
        frontier: Frontier,
        robot_pose: tuple[float, float, float],
    ) -> float:
        """不运行规划的候选预排序，下界用欧氏距离，最终选择仍使用路径长度。"""
        distance = _world_distance(frontier.centroid, (robot_pose[0], robot_pose[1]))
        target_yaw = math.atan2(
            frontier.centroid[1] - robot_pose[1],
            frontier.centroid[0] - robot_pose[0],
        )
        yaw_cost = abs(_angle_wrap(target_yaw - robot_pose[2]))
        return (
            self.information_gain_weight
            * math.sqrt(min(float(frontier.information_gain), 25.0))
            - self.path_cost_weight * distance
            - self.yaw_cost_weight * yaw_cost
        )

    def _select_free_space_goal(
        self,
        grid: FloorGrid,
        robot_pose: tuple[float, float, float],
        selection_context: FrontierSelectionContext | None = None,
        candidate_debug_records: list[FrontierCandidateDebug] | None = None,
    ) -> tuple[Frontier | None, list[tuple[float, float]]]:
        """无有效 frontier 时，从 FREE 区域中选择探索价值高、路径可达的目标。

        旧逻辑按欧氏距离倒序优先选最远点，易导致舍近求远。新版改用综合评分：
        路径代价为主，未知邻接数为探索价值奖励，已访问邻接数为重复惩罚，
        门上下文照常参与，避免退化成"永远最近点"。
        """
        robot_xy = (robot_pose[0], robot_pose[1])
        stride = max(1, int(round(0.4 / grid.resolution)))

        # 第一阶段：廉价预筛选，收集候选及其局部探索价值指标。
        scored_candidates: list[
            tuple[float, tuple[int, int], float, int, int]
        ] = []
        for gy in range(0, grid.size_y, stride):
            for gx in range(0, grid.size_x, stride):
                if int(grid.data[gy, gx]) != FREE:
                    continue
                wx, wy = grid.grid_to_world(gx, gy)
                distance = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
                if distance <= self.goal_tolerance:
                    # 与 controller 完成半径保持一致：过近 fallback 不是规划失败。
                    continue
                if distance < 1.0:
                    # 保留既有 fallback 近距离预筛选，避免重新开放 0.35~1.0m 候选。
                    continue
                if self._is_blacklisted(grid, (wx, wy)):
                    continue
                unknown_neighbors = self._count_neighbor_cells(grid, gx, gy, UNKNOWN)
                visited_neighbors = self._count_neighbor_cells(grid, gx, gy, VISITED)
                # 启发式分数：优先未知邻接多、访问少、距离适中的点。
                # 距离系数较小，确保稍远但探索价值高的区域仍有机会。
                heuristic_score = (
                    min(unknown_neighbors, 12) * 0.5
                    - visited_neighbors * 0.15
                    - distance * 0.2
                )
                scored_candidates.append(
                    (heuristic_score, (gx, gy), distance, unknown_neighbors, visited_neighbors)
                )

        # 按启发式分数倒序，让高探索价值候选优先进入精确规划。
        scored_candidates.sort(reverse=True, key=lambda item: item[0])

        best_choice: tuple[Frontier, list[tuple[float, float]], float, tuple[float, float]] | None = None
        fallback_log_records: list[dict[str, object]] = []
        evaluated_count = 0
        skipped_count = 0
        unreachable_count = 0

        for (
            heuristic_score,
            cell,
            euclidean_distance,
            unknown_neighbors,
            visited_neighbors,
        ) in scored_candidates[: self.max_fallback_planning_candidates]:
            goal = grid.grid_to_world(*cell)
            context_adjustment, skip_target, context_penalty = self._selection_context_effect(
                goal,
                selection_context,
            )
            if skip_target:
                skipped_count += 1
                if candidate_debug_records is not None:
                    candidate_debug_records.append(
                        FrontierCandidateDebug(
                            position=goal,
                            cluster_size=1,
                            information_gain=0.0,
                            euclidean_distance=euclidean_distance,
                            path_length=None,
                            final_score=None,
                            rejection_reason="door_context_hard_filter",
                            door_context_adjustment=context_adjustment,
                            door_context_penalty=context_penalty,
                            blacklist_status="not_blacklisted",
                            reachability_result="context_filtered",
                            source="free_space_fallback",
                        )
                    )
                fallback_log_records.append(
                    {
                        "position": [round(goal[0], 3), round(goal[1], 3)],
                        "euclidean_distance": round(euclidean_distance, 3),
                        "unknown_neighbors": unknown_neighbors,
                        "visited_neighbors": visited_neighbors,
                        "heuristic_score": round(heuristic_score, 3),
                        "path_length": None,
                        "final_score": None,
                        "selected": False,
                        "rejection_reason": "door_context_hard_filter",
                    }
                )
                continue

            evaluated_count += 1
            result = astar_plan(grid, robot_xy, goal)
            if result.reached:
                if self._path_crosses_permanent_gate(result.path, selection_context):
                    skipped_count += 1
                    if candidate_debug_records is not None:
                        candidate_debug_records.append(
                            FrontierCandidateDebug(
                                position=goal,
                                cluster_size=1,
                                information_gain=0.0,
                                euclidean_distance=euclidean_distance,
                                path_length=path_length(result.path),
                                final_score=None,
                                rejection_reason="door_context_path_gate",
                                door_context_adjustment=context_adjustment,
                                door_context_penalty=context_penalty,
                                blacklist_status="not_blacklisted",
                                reachability_result="context_filtered",
                                source="free_space_fallback",
                            )
                        )
                    fallback_log_records.append(
                        {
                            "position": [round(goal[0], 3), round(goal[1], 3)],
                            "euclidean_distance": round(euclidean_distance, 3),
                            "unknown_neighbors": unknown_neighbors,
                            "visited_neighbors": visited_neighbors,
                            "heuristic_score": round(heuristic_score, 3),
                            "path_length": round(path_length(result.path), 3),
                            "final_score": None,
                            "selected": False,
                            "rejection_reason": "door_context_path_gate",
                        }
                    )
                    continue
                candidate_path_length = path_length(result.path)
                # 精确分数：路径短 + 未知邻接奖励 + 门上下文奖励。
                # 路径系数 1.0 保证可达性是首要约束；奖励项防止只选最近已访问点。
                final_score = (
                    -candidate_path_length
                    + min(unknown_neighbors, 12) * 0.3
                    + context_adjustment
                )
                frontier = Frontier(cells=(cell,), centroid=goal, information_gain=0.0, score=final_score)
                is_best = best_choice is None or final_score > best_choice[2]
                if candidate_debug_records is not None:
                    candidate_debug_records.append(
                        FrontierCandidateDebug(
                            position=goal,
                            cluster_size=1,
                            information_gain=0.0,
                            euclidean_distance=euclidean_distance,
                            path_length=candidate_path_length,
                            final_score=final_score,
                            rejection_reason="",
                            door_context_adjustment=context_adjustment,
                            door_context_penalty=context_penalty,
                            blacklist_status="not_blacklisted",
                            reachability_result="reached",
                            source="free_space_fallback",
                        )
                    )
                fallback_log_records.append(
                    {
                        "position": [round(goal[0], 3), round(goal[1], 3)],
                        "euclidean_distance": round(euclidean_distance, 3),
                        "unknown_neighbors": unknown_neighbors,
                        "visited_neighbors": visited_neighbors,
                        "heuristic_score": round(heuristic_score, 3),
                        "path_length": round(candidate_path_length, 3),
                        "final_score": round(final_score, 3),
                        "selected": is_best,
                        "rejection_reason": "",
                    }
                )
                if is_best:
                    best_choice = (frontier, result.path, final_score, goal)
            else:
                unreachable_count += 1
                if candidate_debug_records is not None:
                    candidate_debug_records.append(
                        FrontierCandidateDebug(
                            position=goal,
                            cluster_size=1,
                            information_gain=0.0,
                            euclidean_distance=euclidean_distance,
                            path_length=None,
                            final_score=None,
                            rejection_reason=result.status,
                            door_context_adjustment=context_adjustment,
                            door_context_penalty=context_penalty,
                            blacklist_status="not_blacklisted",
                            reachability_result=result.status,
                            source="free_space_fallback",
                        )
                    )
                fallback_log_records.append(
                    {
                        "position": [round(goal[0], 3), round(goal[1], 3)],
                        "euclidean_distance": round(euclidean_distance, 3),
                        "unknown_neighbors": unknown_neighbors,
                        "visited_neighbors": visited_neighbors,
                        "heuristic_score": round(heuristic_score, 3),
                        "path_length": None,
                        "final_score": None,
                        "selected": False,
                        "rejection_reason": result.status,
                    }
                )
                if result.unreachable:
                    # 确实不可达才累计失败；预算耗尽留待重试。
                    self.record_failure(grid, goal)

        if rospy is not None and fallback_log_records:
            selected_position = (
                [round(best_choice[3][0], 3), round(best_choice[3][1], 3)]
                if best_choice is not None
                else None
            )
            rospy.loginfo(
                "%s",
                json.dumps(
                    {
                        "event": "free_space_fallback_selection",
                        "candidate_count": len(scored_candidates),
                        "evaluated_count": evaluated_count,
                        "skipped_count": skipped_count,
                        "unreachable_count": unreachable_count,
                        "selected_position": selected_position,
                        "selected_score": round(best_choice[2], 3) if best_choice is not None else None,
                        "candidates": fallback_log_records[: self.max_fallback_planning_candidates],
                    },
                    ensure_ascii=False,
                    sort_keys=True,
                ),
            )

        if best_choice is not None:
            return best_choice[0], best_choice[1]
        return None, []

    @staticmethod
    def _count_neighbor_cells(
        grid: FloorGrid,
        gx: int,
        gy: int,
        target_state: int,
    ) -> int:
        """统计 (gx, gy) 周围 8 邻域中处于 target_state 的格子数。"""
        count = 0
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < grid.size_x and 0 <= ny < grid.size_y:
                    if int(grid.data[ny, nx]) == target_state:
                        count += 1
        return count

    def _resolve_frontier_target(
        self,
        grid: FloorGrid,
        frontier: Frontier,
        robot_pose: tuple[float, float, float],
        *,
        selection_context: FrontierSelectionContext | None = None,
    ) -> tuple[
        tuple[float, float] | None,
        list[tuple[float, float]],
        bool,
        float,
        bool,
        bool,
        bool,
        FrontierCandidateDebug,
    ]:
        robot_xy = (robot_pose[0], robot_pose[1])
        candidates = self._candidate_targets_for_frontier(grid, frontier, robot_xy)
        if selection_context is not None:
            # 在精确规划次数受限时，先规划门上下文更偏好的合法落点。
            candidates.sort(
                key=lambda target: (
                    self._selection_context_effect(target, selection_context)[1],
                    -self._selection_context_effect(target, selection_context)[0],
                )
            )
        if not candidates:
            return (
                None,
                [],
                False,
                0.0,
                False,
                False,
                False,
                FrontierCandidateDebug(
                    position=frontier.centroid,
                    cluster_size=len(frontier.cells),
                    information_gain=frontier.information_gain,
                    euclidean_distance=_world_distance(frontier.centroid, robot_xy),
                    path_length=None,
                    final_score=None,
                    rejection_reason="no_target_candidate",
                    door_context_adjustment=0.0,
                    door_context_penalty=0.0,
                    blacklist_status="not_evaluated",
                    reachability_result="not_evaluated",
                ),
            )
        saw_non_blacklisted = False
        saw_context_skip = False
        saw_context_path_skip = False
        saw_path_attempt = False
        saw_budget_exhausted = False
        saw_unreachable = False
        saw_inside_tolerance = False
        path_attempt_count = 0
        blacklisted_target_count = 0
        debug_position = candidates[0]
        debug_context_adjustment = 0.0
        debug_context_penalty: float | None = 0.0
        debug_path_length: float | None = None
        last_path_position: tuple[float, float] | None = None
        last_path_context_adjustment = 0.0
        last_path_context_penalty: float | None = 0.0
        best_choice: tuple[tuple[float, float], list[tuple[float, float]], float, float, float | None] | None = None
        for target_xy in candidates:
            if self._is_blacklisted(grid, target_xy):
                blacklisted_target_count += 1
                continue
            saw_non_blacklisted = True
            debug_position = target_xy
            context_adjustment, skip_target, context_penalty = self._selection_context_effect(
                target_xy,
                selection_context,
            )
            debug_context_adjustment = context_adjustment
            debug_context_penalty = context_penalty
            if skip_target:
                saw_context_skip = True
                continue
            if _world_distance(target_xy, robot_xy) <= self.goal_tolerance:
                saw_inside_tolerance = True
                continue
            if path_attempt_count >= self.max_goal_candidates_per_frontier:
                # 仍可能存在可达落点，因此按预算耗尽处理，不能累计失败或拉黑。
                saw_budget_exhausted = True
                break
            saw_path_attempt = True
            path_attempt_count += 1
            last_path_position = target_xy
            last_path_context_adjustment = context_adjustment
            last_path_context_penalty = context_penalty
            result = astar_plan(grid, robot_xy, target_xy)
            if result.reached:
                if self._path_crosses_permanent_gate(result.path, selection_context):
                    saw_context_skip = True
                    saw_context_path_skip = True
                    debug_path_length = path_length(result.path)
                    continue
                if selection_context is None:
                    return (
                        target_xy,
                        result.path,
                        False,
                        0.0,
                        False,
                        False,
                        False,
                        FrontierCandidateDebug(
                            position=target_xy,
                            cluster_size=len(frontier.cells),
                            information_gain=frontier.information_gain,
                            euclidean_distance=_world_distance(target_xy, robot_xy),
                            path_length=path_length(result.path),
                            final_score=None,
                            rejection_reason="",
                            door_context_adjustment=0.0,
                            door_context_penalty=0.0,
                            blacklist_status="partial" if blacklisted_target_count else "not_blacklisted",
                            reachability_result="reached",
                        ),
                    )
                candidate_score = context_adjustment - path_length(result.path)
                if best_choice is None or candidate_score > best_choice[3]:
                    best_choice = (
                        target_xy,
                        result.path,
                        context_adjustment,
                        candidate_score,
                        context_penalty,
                    )
            elif result.budget_exhausted:
                # 预算耗尽：可能可达，不能黑名单，标记后跳过本轮。
                saw_budget_exhausted = True
            else:
                saw_unreachable = True
        if best_choice is not None:
            return (
                best_choice[0],
                best_choice[1],
                False,
                best_choice[2],
                False,
                False,
                False,
                FrontierCandidateDebug(
                    position=best_choice[0],
                    cluster_size=len(frontier.cells),
                    information_gain=frontier.information_gain,
                    euclidean_distance=_world_distance(best_choice[0], robot_xy),
                    path_length=path_length(best_choice[1]),
                    final_score=None,
                    rejection_reason="",
                    door_context_adjustment=best_choice[2],
                    door_context_penalty=best_choice[4],
                    blacklist_status="partial" if blacklisted_target_count else "not_blacklisted",
                    reachability_result="reached",
                ),
            )
        # 没有可达候选：若全是预算耗尽，则不视为不可达，避免误黑名单。
        blacklisted_only = not saw_non_blacklisted
        context_filtered_only = saw_context_skip and not saw_budget_exhausted and not saw_unreachable
        inside_tolerance_only = (
            saw_inside_tolerance
            and not blacklisted_only
            and not context_filtered_only
            and not saw_budget_exhausted
            and not saw_unreachable
        )
        if last_path_position is not None:
            debug_position = last_path_position
            debug_context_adjustment = last_path_context_adjustment
            debug_context_penalty = last_path_context_penalty
        if blacklisted_only:
            rejection_reason = "all_targets_blacklisted"
            reachability_result = "not_evaluated"
            blacklist_status = "all_blacklisted"
        elif context_filtered_only:
            rejection_reason = (
                "door_context_path_gate" if saw_context_path_skip else "door_context_hard_filter"
            )
            reachability_result = "context_filtered"
            blacklist_status = "partial" if blacklisted_target_count else "not_blacklisted"
        elif saw_budget_exhausted:
            rejection_reason = "planning_budget_exhausted"
            reachability_result = "budget_exhausted"
            blacklist_status = "partial" if blacklisted_target_count else "not_blacklisted"
        elif inside_tolerance_only:
            rejection_reason = "inside_goal_tolerance"
            reachability_result = "inside_goal_tolerance"
            blacklist_status = "partial" if blacklisted_target_count else "not_blacklisted"
        else:
            rejection_reason = "unreachable"
            reachability_result = UNREACHABLE
            blacklist_status = "partial" if blacklisted_target_count else "not_blacklisted"
        return (
            None,
            [],
            blacklisted_only,
            0.0,
            context_filtered_only,
            saw_budget_exhausted,
            inside_tolerance_only,
            FrontierCandidateDebug(
                position=debug_position,
                cluster_size=len(frontier.cells),
                information_gain=frontier.information_gain,
                euclidean_distance=_world_distance(debug_position, robot_xy),
                path_length=debug_path_length,
                final_score=None,
                rejection_reason=rejection_reason,
                door_context_adjustment=debug_context_adjustment,
                door_context_penalty=debug_context_penalty,
                blacklist_status=blacklist_status,
                reachability_result=reachability_result,
            ),
        )

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
        adjustment, skip_target, _ = self._selection_context_effect(target_xy, selection_context)
        return adjustment, skip_target

    def _selection_context_effect(
        self,
        target_xy: tuple[float, float],
        selection_context: FrontierSelectionContext | None,
    ) -> tuple[float, bool, float | None]:
        if selection_context is None:
            return 0.0, False, 0.0
        adjustment = 0.0
        penalty = 0.0

        # 永久禁区：大门外侧不可返回区域。只在门口附近有限半径内跳过，
        # 远处仅大幅降分，避免用无限半平面永久屏蔽整片可达区域。
        permanent_signed = self._signed_half_plane_distance(
            target_xy,
            selection_context.permanent_forbidden_half_plane_origin,
            selection_context.permanent_forbidden_half_plane_normal,
        )
        if permanent_signed is not None and permanent_signed < -0.05:
            perm_origin = selection_context.permanent_forbidden_half_plane_origin
            if perm_origin is not None:
                perm_distance = math.hypot(target_xy[0] - perm_origin[0], target_xy[1] - perm_origin[1])
                if perm_distance <= max(0.0, selection_context.permanent_forbidden_radius):
                    return 0.0, True, None
                # 远处不 skip，惩罚从硬半径边界开始连续指数衰减。
                decay_distance = max(
                    selection_context.permanent_forbidden_decay_distance,
                    1e-6,
                )
                perm_decay = math.exp(
                    -max(0.0, perm_distance - selection_context.permanent_forbidden_radius)
                    / decay_distance
                )
            else:
                perm_decay = 1.0
            permanent_penalty = max(
                0.0,
                selection_context.permanent_forbidden_penalty,
            ) * perm_decay
            adjustment -= permanent_penalty
            penalty += permanent_penalty

        # 临时禁区：过门后短时间内的回头抑制。
        # 只在门附近有限半径内跳过门前侧前沿，超出半径仅降分，
        # 避免用无限半平面把整片区域的可达前沿全部过滤掉。
        signed_distance = self._signed_half_plane_distance(
            target_xy,
            selection_context.forbidden_half_plane_origin,
            selection_context.forbidden_half_plane_normal,
        )
        if signed_distance is not None and signed_distance < -0.05:
            origin = selection_context.forbidden_half_plane_origin
            if origin is not None:
                distance_to_origin = math.hypot(target_xy[0] - origin[0], target_xy[1] - origin[1])
                if selection_context.skip_in_forbidden_half_plane and distance_to_origin <= max(0.0, selection_context.forbidden_radius):
                    return 0.0, True, None
                # penalty 随距离衰减：门附近全罚，5m 外不罚，避免远处前沿被误压。
                decay = max(0.0, 1.0 - distance_to_origin / 5.0)
            else:
                decay = 1.0
            temporary_penalty = max(0.0, selection_context.temporary_revisit_penalty) * decay
            adjustment -= temporary_penalty
            penalty += temporary_penalty
        preferred_progress = self._signed_half_plane_distance(
            target_xy,
            selection_context.preferred_origin,
            selection_context.preferred_direction,
        )
        if preferred_progress is not None and preferred_progress > 0.05:
            adjustment += self.door_preferred_direction_bonus * min(preferred_progress, 1.0)
        return adjustment, False, penalty

    @classmethod
    def _path_crosses_permanent_gate(
        cls,
        path: list[tuple[float, float]],
        selection_context: FrontierSelectionContext | None,
    ) -> bool:
        """只阻止从门后侧穿过有限门洞返回门前侧，不屏蔽整个半平面。"""
        if selection_context is None or selection_context.permanent_gate_half_width <= 0.0:
            return False
        origin = selection_context.permanent_forbidden_half_plane_origin
        direction = selection_context.permanent_forbidden_half_plane_normal
        if origin is None or direction is None:
            return False
        norm = math.hypot(direction[0], direction[1])
        if norm <= 1e-6:
            return False
        normal = (direction[0] / norm, direction[1] / norm)
        tangent = (-normal[1], normal[0])
        half_width = float(selection_context.permanent_gate_half_width)
        for start, end in zip(path, path[1:]):
            start_signed = (start[0] - origin[0]) * normal[0] + (start[1] - origin[1]) * normal[1]
            end_signed = (end[0] - origin[0]) * normal[0] + (end[1] - origin[1]) * normal[1]
            if start_signed < 0.0 or end_signed >= 0.0:
                continue
            denominator = start_signed - end_signed
            if denominator <= 1e-6:
                continue
            ratio = start_signed / denominator
            cross_x = start[0] + ratio * (end[0] - start[0])
            cross_y = start[1] + ratio * (end[1] - start[1])
            lateral = abs((cross_x - origin[0]) * tangent[0] + (cross_y - origin[1]) * tangent[1])
            if lateral <= half_width:
                return True
        return False

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

    @staticmethod
    def _top_candidate_debug(
        candidates: list[FrontierCandidateDebug],
        final_target: tuple[float, float] | None,
        *,
        limit: int = 8,
    ) -> tuple[FrontierCandidateDebug, ...]:
        enriched: list[FrontierCandidateDebug] = []
        selected_assigned = False
        for candidate in candidates:
            selected = (
                not selected_assigned
                and final_target is not None
                and candidate.reachability_result == "reached"
                and candidate.position == final_target
            )
            if selected:
                selected_assigned = True
            rejection_reason = candidate.rejection_reason
            if not selected and candidate.reachability_result == "reached" and not rejection_reason:
                rejection_reason = "score_not_selected"
            enriched.append(
                replace(
                    candidate,
                    selected=selected,
                    rejection_reason=rejection_reason,
                )
            )
        selected = [candidate for candidate in enriched if candidate.selected]
        reached = [
            candidate
            for candidate in enriched
            if not candidate.selected and candidate.reachability_result == "reached"
        ]
        rejected = [candidate for candidate in enriched if candidate.reachability_result != "reached"]
        reached.sort(
            key=lambda candidate: (
                -(candidate.final_score if candidate.final_score is not None else -math.inf),
                candidate.euclidean_distance,
            )
        )
        rejected.sort(key=lambda candidate: candidate.euclidean_distance)

        result_limit = max(1, int(limit))
        ordered = selected[:result_limit]
        remaining = result_limit - len(ordered)
        rejected_slots = min(len(rejected), max(1, remaining // 3)) if rejected else 0
        reached_slots = min(len(reached), remaining - rejected_slots)
        rejected_slots = min(len(rejected), remaining - reached_slots)
        ordered.extend(reached[:reached_slots])
        ordered.extend(rejected[:rejected_slots])
        remaining = result_limit - len(ordered)
        if remaining > 0:
            ordered.extend(reached[reached_slots : reached_slots + remaining])
        remaining = result_limit - len(ordered)
        if remaining > 0:
            ordered.extend(rejected[rejected_slots : rejected_slots + remaining])
        return tuple(ordered)

    def _log_selection_debug(
        self,
        *,
        candidate_count: int,
        reachable_count: int,
        blacklisted_count: int,
        unreachable_count: int,
        context_filtered_count: int,
        budget_exhausted_count: int,
        inside_tolerance_count: int,
        used_fallback: bool,
        frontier: Frontier | None,
        path: list[tuple[float, float]],
        selection_context: FrontierSelectionContext | None,
        debug: FrontierSelectionDebug,
    ) -> None:
        self.last_selection_debug = debug
        if rospy is None:
            return
        payload = {
            "event": "frontier_selection",
            "candidate_count": candidate_count,
            "reachable_count": reachable_count,
            "blacklisted_count": blacklisted_count,
            "unreachable_count": unreachable_count,
            "context_filtered_count": context_filtered_count,
            "budget_exhausted_count": budget_exhausted_count,
            "used_fallback": used_fallback,
            "final_target": list(debug.final_target) if debug.final_target is not None else None,
            "final_score": debug.final_score,
            "path_length": debug.path_length,
            "retained_by_inertia": debug.retained_by_inertia,
            "inertia_reason": debug.inertia_reason,
            "selection_wall_duration": debug.selection_wall_duration,
            "door_context": self._door_context_payload(selection_context),
            "candidates": [
                self._candidate_debug_payload(candidate, selection_context)
                for candidate in debug.candidates
            ],
        }
        rospy.loginfo("%s", json.dumps(payload, ensure_ascii=False, separators=(",", ":")))

    def _candidate_debug_payload(
        self,
        candidate: FrontierCandidateDebug,
        context: FrontierSelectionContext | None,
    ) -> dict[str, object]:
        origin = None
        normal = None
        if context is not None:
            origin = context.forbidden_half_plane_origin or context.permanent_forbidden_half_plane_origin
            normal = context.forbidden_half_plane_normal or context.permanent_forbidden_half_plane_normal
        signed_distance = self._signed_half_plane_distance(candidate.position, origin, normal)
        distance_to_door = (
            math.hypot(candidate.position[0] - origin[0], candidate.position[1] - origin[1])
            if origin is not None
            else None
        )
        if candidate.rejection_reason == "door_context_hard_filter":
            context_effect = "hard_filter"
        elif candidate.rejection_reason == "door_context_path_gate":
            context_effect = "path_gate_filter"
        elif candidate.door_context_penalty is not None and candidate.door_context_penalty > 0.0:
            context_effect = "penalized"
        else:
            context_effect = "unaffected"
        return {
            "position": list(candidate.position),
            "cluster_size": candidate.cluster_size,
            "information_gain": candidate.information_gain,
            "euclidean_distance": candidate.euclidean_distance,
            "path_length": candidate.path_length,
            "final_score": candidate.final_score,
            "rejection_reason": candidate.rejection_reason,
            "door_context_adjustment": candidate.door_context_adjustment,
            "door_context_penalty": candidate.door_context_penalty,
            "door_context_effect": context_effect,
            "door_id": context.door_id if context is not None else None,
            "distance_to_door": distance_to_door,
            "door_side": (
                "pass_side"
                if signed_distance is not None and signed_distance > 0.05
                else "front_side"
                if signed_distance is not None and signed_distance < -0.05
                else "center_or_unknown"
            ),
            "blacklist_status": candidate.blacklist_status,
            "reachability_result": candidate.reachability_result,
            "source": candidate.source,
            "selected": candidate.selected,
        }

    @staticmethod
    def _door_context_payload(
        context: FrontierSelectionContext | None,
    ) -> dict[str, object] | None:
        if context is None:
            return None
        remaining = None
        if context.temporary_context_expires_at is not None and context.context_time is not None:
            remaining = max(0.0, context.temporary_context_expires_at - context.context_time)
        return {
            "door_id": context.door_id,
            "temporary_remaining": remaining,
            "temporary_hard_radius": context.forbidden_radius,
            "temporary_penalty": context.temporary_revisit_penalty,
            "permanent_hard_radius": context.permanent_forbidden_radius,
            "permanent_penalty": context.permanent_forbidden_penalty,
            "permanent_decay_distance": context.permanent_forbidden_decay_distance,
            "permanent_gate_half_width": context.permanent_gate_half_width,
        }


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

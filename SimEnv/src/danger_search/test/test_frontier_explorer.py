import json
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from danger_search.frontier import FrontierExplorer
from danger_search.frontier import Frontier
from danger_search.frontier import FrontierSelectionContext
from danger_search.frontier import _build_frontier_mask
from danger_search.mapper import BLACK, FREE, NEGATIVE, FloorGrid, OCC, UNKNOWN, VISITED
from danger_search.planner import (
    PlanResult,
    PlanningConstraints,
    SupportVetoRegion,
    REACHED,
    BUDGET_EXHAUSTED,
    UNREACHABLE,
)


class FrontierExplorerTest(unittest.TestCase):
    def test_live_blacklist_query_expires_old_entry(self):
        grid = FloorGrid(0, 1.0, 3.0, 3.0, 0.0, 0.0)
        target = grid.grid_to_world(1, 1)
        explorer = FrontierExplorer(min_cluster_size=1)
        explorer._blacklist[(1, 1)] = 5.0

        with patch("danger_search.frontier.get_time", return_value=4.0):
            self.assertTrue(explorer.is_blacklisted(grid, target))
        with patch("danger_search.frontier.get_time", return_value=6.0):
            self.assertFalse(explorer.is_blacklisted(grid, target))

    def test_formal_reachable_but_support_blocked_is_reported_without_astar_loop(self):
        grid = FloorGrid(
            floor_index=0, resolution=1.0, width=7.0, length=3.0,
            origin_x=0.0, origin_y=0.0,
        )
        for gx in range(1, 6):
            grid.set_base_state(gx, 1, FREE)
        start = grid.grid_to_world(1, 1)
        goal = grid.grid_to_world(5, 1)
        frontier = Frontier(cells=((5, 1),), centroid=goal, information_gain=1.0)
        region = SupportVetoRegion(
            "support:1", frozenset({(3, 1)}), frozenset({(3, 0)})
        )
        constraints = PlanningConstraints.from_regions((region,), revision=7)
        explorer = FrontierExplorer(
            min_cluster_size=1, max_fallback_planning_candidates=1
        )
        explorer._candidate_targets_for_frontier = lambda *_args: [goal]
        explorer._select_free_space_goal = lambda *_args, **_kwargs: (None, [])

        selected, path = explorer.select_frontier(
            grid, (start[0], start[1], 0.0),
            frontiers=[frontier], planning_constraints=constraints,
        )

        self.assertIsNone(selected)
        self.assertEqual(path, [])
        evidence = explorer.support_constraint_blocked_evidence(
            grid,
            (start[0], start[1], 0.0),
            constraints,
            frontiers=[frontier],
        )
        self.assertIsNotNone(evidence)
        assert evidence is not None
        self.assertEqual(evidence.constraint_revision, 7)
        self.assertEqual(evidence.region_ids, ("support:1",))
        self.assertEqual(evidence.formal_reachable_count, 1)
        self.assertEqual(evidence.effective_reachable_count, 0)
        self.assertTrue(evidence.fully_blocked)

        grid.mark_occupied(2, 1)
        self.assertIsNone(
            explorer.support_constraint_blocked_evidence(
                grid,
                (start[0], start[1], 0.0),
                constraints,
                frontiers=[frontier],
            )
        )

    def test_termination_evidence_is_frozen_from_current_map_not_selection_debug(self):
        grid = FloorGrid(
            floor_index=0, resolution=1.0, width=7.0, length=3.0,
            origin_x=0.0, origin_y=0.0,
        )
        for gx in range(1, 6):
            grid.set_base_state(gx, 1, FREE)
        start = grid.grid_to_world(1, 1)
        goal = grid.grid_to_world(5, 1)
        frontier = Frontier(cells=((5, 1),), centroid=goal, information_gain=1.0)
        constraints = PlanningConstraints.from_regions(
            (SupportVetoRegion(
                "support:current", frozenset({(3, 1)}), frozenset({(3, 0)})
            ),),
            revision=9,
        )
        explorer = FrontierExplorer(min_cluster_size=1)
        explorer._candidate_targets_for_frontier = lambda *_args: [goal]
        explorer.last_selection_debug = SimpleNamespace(
            formal_reachable_count=99,
            effective_reachable_count=98,
            constraints_revision=1,
        )

        evidence = explorer.floor_termination_evidence(
            grid,
            (start[0], start[1], 0.0),
            constraints,
            frontiers=[frontier],
        )

        self.assertEqual(evidence.constraint_revision, 9)
        self.assertEqual(evidence.region_ids, ("support:current",))
        self.assertEqual(evidence.formal_reachable_count, 1)
        self.assertEqual(evidence.effective_reachable_count, 0)
        self.assertTrue(evidence.fully_blocked)

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            FrontierExplorer(min_cluster_size=0)
        with self.assertRaises(ValueError):
            FrontierExplorer(blacklist_ttl=math.nan)
        with self.assertRaises(ValueError):
            FrontierExplorer(max_planning_candidates=0)

    @staticmethod
    def _mark_free_rect(grid: FloorGrid, gx0: int, gx1: int, gy0: int, gy1: int) -> None:
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                grid.mark_free(gx, gy)

    def _make_door_transition_grid(self) -> FloorGrid:
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=14.0,
            length=14.0,
            origin_x=-7.0,
            origin_y=-7.0,
        )
        self._mark_free_rect(grid, 6, 7, 2, 11)
        self._mark_free_rect(grid, 4, 9, 1, 6)
        self._mark_free_rect(grid, 5, 8, 7, 11)
        return grid

    def test_vectorized_frontier_mask_matches_original_eight_neighbor_semantics(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=4.0,
            length=5.0,
            origin_x=-2.0,
            origin_y=-2.5,
        )
        rng = np.random.default_rng(42)
        grid.load_legacy_data(rng.choice(
            [UNKNOWN, FREE, VISITED, BLACK, OCC],
            size=grid.data.shape,
            p=[0.35, 0.30, 0.15, 0.05, 0.15],
        ))

        expected = np.zeros_like(grid.data, dtype=bool)
        for gy in range(1, grid.size_y - 1):
            for gx in range(1, grid.size_x - 1):
                if int(grid.data[gy, gx]) not in (FREE, VISITED):
                    continue
                expected[gy, gx] = any(
                    int(grid.data[ny, nx]) == UNKNOWN
                    for ny in range(gy - 1, gy + 2)
                    for nx in range(gx - 1, gx + 2)
                    if (nx, ny) != (gx, gy)
                )

        self.assertTrue(np.array_equal(_build_frontier_mask(grid), expected))

    def test_cluster_target_is_an_actual_frontier_cell(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=9.0,
            length=9.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        for gy in range(3, 6):
            for gx in range(3, 6):
                grid.mark_free(gx, gy)

        explorer = FrontierExplorer(min_cluster_size=1)
        frontiers = explorer.extract_frontiers(grid)

        self.assertEqual(len(frontiers), 1)
        target = frontiers[0].centroid
        target_cell = grid.world_to_grid(target[0], target[1])
        self.assertIn(target_cell, frontiers[0].cells)
        center = grid.grid_to_world(4, 4)
        self.assertGreater(math.hypot(target[0] - center[0], target[1] - center[1]), 0.9)

    def test_selected_frontier_goal_is_not_inside_goal_tolerance(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=3.0,
            length=3.0,
            origin_x=-1.5,
            origin_y=-1.5,
        )
        for gy in range(10, 20):
            for gx in range(10, 20):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(min_cluster_size=1)

        frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertGreater(math.hypot(frontier.centroid[0], frontier.centroid[1]), 0.35)

    def test_selection_emits_single_structured_candidate_log(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=3.0,
            length=3.0,
            origin_x=-1.5,
            origin_y=-1.5,
        )
        for gy in range(10, 20):
            for gx in range(10, 20):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(min_cluster_size=1)
        log_messages: list[str] = []
        fake_rospy = SimpleNamespace(
            loginfo=lambda fmt, *args: log_messages.append(fmt % args),
        )

        with patch("danger_search.frontier.rospy", fake_rospy):
            frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertEqual(len(log_messages), 1)
        payload = json.loads(log_messages[0])
        self.assertEqual(payload["event"], "frontier_selection")
        self.assertGreaterEqual(payload["selection_wall_duration"], 0.0)
        self.assertTrue(payload["candidates"])
        required_fields = {
            "position",
            "cluster_size",
            "frontier_boundary_cells",
            "information_gain",
            "expected_unknown_gain",
            "optimistic_score",
            "euclidean_distance",
            "path_length",
            "final_score",
            "information_gain_reward",
            "path_cost",
            "initial_heading_error",
            "initial_heading_cost",
            "cumulative_turn_angle",
            "path_turn_cost",
            "revisit_penalty",
            "forward_bonus",
            "rejection_reason",
            "door_context_adjustment",
            "door_context_penalty",
            "blacklist_status",
            "reachability_result",
            "elimination_stage",
        }
        self.assertTrue(required_fields.issubset(payload["candidates"][0]))
        self.assertTrue(payload["candidates"][0]["selected"])
        self.assertIsNotNone(payload["candidates"][0]["initial_heading_error"])
        self.assertIsNotNone(payload["candidates"][0]["cumulative_turn_angle"])

    def test_selection_debug_records_small_cluster_rejection(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.mark_free(2, 2)
        explorer = FrontierExplorer(min_cluster_size=4)

        frontier, path = explorer.select_frontier(grid, (2.5, 2.5, 0.0))

        self.assertIsNone(frontier)
        self.assertEqual(path, [])
        debug = explorer.last_selection_debug
        assert debug is not None
        self.assertTrue(debug.candidates)
        self.assertEqual(debug.candidates[0].rejection_reason, "cluster_below_min_size")
        self.assertEqual(debug.candidates[0].cluster_size, 1)

    def test_stuck_failure_blacklists_nearby_shifted_targets(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        explorer = FrontierExplorer(stuck_blacklist_radius=0.75, blacklist_ttl=60.0)

        with patch("danger_search.frontier.get_time", return_value=10.0):
            explorer.record_stuck_failure(grid, (0.0, 0.0))

        self.assertTrue(explorer._is_blacklisted(grid, (0.1, 0.0)))
        self.assertTrue(explorer._is_blacklisted(grid, (0.6, 0.0)))
        self.assertFalse(explorer._is_blacklisted(grid, (0.9, 0.0)))

    def test_selected_frontier_goal_avoids_obstacle_edge(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=14.0,
            length=12.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        self._mark_free_rect(grid, 1, 5, 5, 5)
        self._mark_free_rect(grid, 5, 9, 3, 7)
        grid.mark_occupied(9, 5)
        grid.mark_black(9, 4, owner="test:frontier_clearance")

        explorer = FrontierExplorer(
            min_cluster_size=1,
            frontier_obstacle_clearance=1.5,
            frontier_unknown_standoff=1.0,
        )

        frontier, path = explorer.select_frontier(grid, (1.5, 5.5, 0.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        target_cell = grid.world_to_grid(frontier.centroid[0], frontier.centroid[1])
        self.assertIsNotNone(target_cell)
        self.assertNotIn(target_cell, frontier.cells)
        self.assertNotIn(target_cell, {(8, 5), (8, 4), (8, 6), (9, 4), (9, 5)})
        self.assertNotEqual(grid.data[target_cell[1], target_cell[0]], OCC)
        self.assertNotEqual(grid.data[target_cell[1], target_cell[0]], BLACK)

    def test_blocked_clearance_only_considers_local_window(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=40.0,
            length=40.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        explorer = FrontierExplorer(min_cluster_size=1)
        grid.mark_occupied(30, 30)

        self.assertTrue(explorer._has_blocked_clearance(grid, (5, 5), 3))
        self.assertEqual(explorer._distance_to_blocked(grid, (5, 5), max_search_steps=3), 3.0)

    def test_score_frontier_uses_actual_path_first_segment_not_target_centroid(self):
        """最终评分的朝向项只能由真实路径首段决定。"""
        explorer = FrontierExplorer(min_cluster_size=1)
        frontier = Frontier(cells=((0, 0),), centroid=(-1.0, 0.0), information_gain=5.0)
        path = [(0.0, 0.0), (1.0, 0.0)]

        score_with_safe_target = explorer._score_frontier(
            frontier,
            path,
            (0.0, 0.0, 0.0),
            target_xy=(1.0, 0.0),
        )
        score_with_old_centroid = explorer._score_frontier(
            frontier,
            path,
            (0.0, 0.0, 0.0),
        )

        self.assertAlmostEqual(score_with_safe_target, score_with_old_centroid)

    def test_score_frontier_penalizes_actual_path_turns_not_goal_bearing(self):
        """同一目标的绕行路径必须因首段和中途转弯而降分。"""
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=0.0,
            path_cost_weight=0.0,
            yaw_cost_weight=1.0,
            forward_alignment_weight=1.0,
            revisit_penalty_weight=0.0,
            path_turn_cost_weight=0.5,
        )
        frontier = Frontier(
            cells=((0, 0),),
            centroid=(2.0, 0.0),
            information_gain=1.0,
        )
        robot_pose = (0.0, 0.0, 0.0)

        straight_score = explorer._score_frontier(
            frontier,
            [(0.0, 0.0), (2.0, 0.0)],
            robot_pose,
        )
        turning_score = explorer._score_frontier(
            frontier,
            [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (2.0, 0.0)],
            robot_pose,
        )

        self.assertGreater(straight_score, turning_score)
        terms = explorer._frontier_score_terms(
            frontier,
            [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (2.0, 0.0)],
            robot_pose,
        )
        self.assertAlmostEqual(terms.initial_heading_error, math.pi / 2.0)
        self.assertAlmostEqual(terms.cumulative_turn_angle, 3.0 * math.pi / 4.0)
        self.assertGreater(terms.initial_heading_cost, 0.0)
        self.assertGreater(terms.path_turn_cost, 0.0)

    def test_expected_unknown_gain_counts_four_connected_unknown_inside_three_meter_circle(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=8.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.fill_base(FREE)
        for cell in ((3, 2), (4, 2), (5, 2), (4, 1)):
            grid.set_base_state(*cell, UNKNOWN)
        frontier = Frontier(
            cells=((2, 2),),
            centroid=grid.grid_to_world(2, 2),
            information_gain=1.0,
        )
        explorer = FrontierExplorer(
            min_cluster_size=1,
            expected_gain_radius=3.0,
        )

        gain = explorer._expected_unknown_gain(
            grid,
            frontier,
            grid.grid_to_world(1, 2),
        )

        # 距离圆心 2m 和 3m 的 UNKNOWN 计入；4m 及由它连接的分支不计入。
        self.assertEqual(gain, 2.0)

    def test_expected_unknown_gain_clips_at_map_boundary_without_double_counting(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=4.0,
            length=4.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.fill_base(FREE)
        for cell in ((1, 1), (1, 2), (2, 1), (2, 2)):
            grid.set_base_state(*cell, UNKNOWN)
        frontier = Frontier(
            cells=((0, 1), (0, 2)),
            centroid=grid.grid_to_world(0, 1),
            information_gain=2.0,
        )
        explorer = FrontierExplorer(
            min_cluster_size=1,
            expected_gain_radius=3.0,
        )

        gain = explorer._expected_unknown_gain(
            grid,
            frontier,
            grid.grid_to_world(0, 1),
        )

        self.assertEqual(gain, 4.0)

    def test_expected_unknown_gain_never_traverses_known_or_blocking_states(self):
        for blocking_state in (OCC, BLACK, NEGATIVE, FREE, VISITED):
            with self.subTest(blocking_state=blocking_state):
                grid = FloorGrid(
                    floor_index=0,
                    resolution=1.0,
                    width=7.0,
                    length=3.0,
                    origin_x=0.0,
                    origin_y=0.0,
                )
                snapshot = np.full((grid.size_y, grid.size_x), FREE, dtype=np.int16)
                snapshot[1, 3] = UNKNOWN
                snapshot[1, 4] = blocking_state
                snapshot[1, 5] = UNKNOWN
                grid.load_legacy_data(snapshot)
                frontier = Frontier(
                    cells=((2, 1),),
                    centroid=grid.grid_to_world(2, 1),
                    information_gain=1.0,
                )
                explorer = FrontierExplorer(
                    min_cluster_size=1,
                    expected_gain_radius=5.0,
                )

                gain = explorer._expected_unknown_gain(
                    grid,
                    frontier,
                    grid.grid_to_world(1, 1),
                )

                self.assertEqual(gain, 1.0)

    def test_far_large_unknown_corridor_beats_near_small_unknown_pocket(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=20.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.fill_base(FREE)
        robot_pose = (1.5, 2.5, 0.0)
        near_target = (2.5, 2.5)
        far_target = (13.5, 2.5)
        near = Frontier(cells=((2, 2),), centroid=near_target, information_gain=12.0)
        far = Frontier(cells=((13, 2),), centroid=far_target, information_gain=2.0)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=1.0,
            path_cost_weight=1.0,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
        )

        def candidates(_grid, frontier, _robot_xy):
            return [frontier.centroid]

        def expected_gain(_grid, frontier, _target):
            return 4.0 if frontier is near else 400.0

        def fake_astar(_grid, start, goal, **_kwargs):
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(
            explorer, "_candidate_targets_for_frontier", side_effect=candidates
        ), patch.object(
            explorer, "_expected_unknown_gain", side_effect=expected_gain
        ), patch(
            "danger_search.frontier.astar_plan", side_effect=fake_astar
        ):
            selected, path = explorer.select_frontier(
                grid,
                robot_pose,
                frontiers=[near, far],
            )

        self.assertIsNotNone(selected)
        self.assertTrue(path)
        self.assertEqual(selected.centroid, far_target)
        self.assertEqual(selected.expected_unknown_gain, 400.0)

    def test_long_boundary_with_small_unknown_domain_loses_to_narrow_large_domain(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=12.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.fill_base(FREE)
        robot_pose = (1.5, 2.5, 0.0)
        long_boundary = Frontier(
            cells=tuple((3, gy) for gy in range(5)),
            centroid=(3.5, 2.5),
            information_gain=50.0,
        )
        narrow_entrance = Frontier(
            cells=((4, 2),),
            centroid=(4.5, 2.5),
            information_gain=1.0,
        )
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=1.0,
            path_cost_weight=1.0,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
        )

        def expected_gain(_grid, frontier, _target):
            return 4.0 if frontier is long_boundary else 100.0

        with patch.object(
            explorer,
            "_candidate_targets_for_frontier",
            side_effect=lambda _grid, frontier, _robot: [frontier.centroid],
        ), patch.object(
            explorer, "_expected_unknown_gain", side_effect=expected_gain
        ), patch(
            "danger_search.frontier.astar_plan",
            side_effect=lambda _grid, start, goal, **_kwargs: PlanResult(
                path=[start, goal], status=REACHED
            ),
        ):
            selected, _path = explorer.select_frontier(
                grid,
                robot_pose,
                frontiers=[long_boundary, narrow_entrance],
            )

        self.assertIsNotNone(selected)
        self.assertEqual(selected.centroid, narrow_entrance.centroid)

    def test_far_high_gain_candidate_enters_top_eight_planning_budget(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=20.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.fill_base(FREE)
        robot_pose = (0.5, 2.5, 0.0)
        near_frontiers = [
            Frontier(
                cells=((gx, 2),),
                centroid=grid.grid_to_world(gx, 2),
                information_gain=20.0,
            )
            for gx in range(1, 10)
        ]
        far_target = grid.grid_to_world(12, 2)
        far_frontier = Frontier(
            cells=((12, 2),),
            centroid=far_target,
            information_gain=1.0,
        )
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=1.0,
            path_cost_weight=1.0,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
            max_planning_candidates=8,
        )
        planned_goals: list[tuple[float, float]] = []

        def expected_gain(_grid, frontier, _target):
            return 625.0 if frontier is far_frontier else 1.0

        def fake_astar(_grid, start, goal, **_kwargs):
            planned_goals.append(goal)
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(
            explorer,
            "_candidate_targets_for_frontier",
            side_effect=lambda _grid, frontier, _robot: [frontier.centroid],
        ), patch.object(
            explorer, "_expected_unknown_gain", side_effect=expected_gain
        ), patch(
            "danger_search.frontier.astar_plan", side_effect=fake_astar
        ):
            explorer.select_frontier(
                grid,
                robot_pose,
                frontiers=near_frontiers + [far_frontier],
            )

        self.assertIn(far_target, planned_goals)
        debug = explorer.last_selection_debug
        assert debug is not None
        far_debug = next(
            candidate
            for candidate in debug.candidates
            if candidate.position == far_target
        )
        self.assertNotEqual(far_debug.rejection_reason, "planning_candidate_limit")

    def test_optimistic_and_final_scores_share_expected_unknown_gain(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=8.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.fill_base(FREE)
        robot_pose = (1.5, 2.5, 0.0)
        target = (3.5, 2.5)
        frontier = Frontier(cells=((3, 2),), centroid=target, information_gain=99.0)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=1.0,
            path_cost_weight=1.0,
            yaw_cost_weight=0.0,
            path_turn_cost_weight=0.0,
            revisit_penalty_weight=0.0,
        )

        with patch.object(
            explorer, "_candidate_targets_for_frontier", return_value=[target]
        ), patch.object(
            explorer, "_expected_unknown_gain", return_value=49.0
        ) as gain_mock, patch(
            "danger_search.frontier.astar_plan",
            return_value=PlanResult(
                path=[robot_pose[:2], (1.5, 3.5), target],
                status=REACHED,
            ),
        ):
            selected, _path = explorer.select_frontier(
                grid,
                robot_pose,
                frontiers=[frontier],
            )

        self.assertIsNotNone(selected)
        debug = explorer.last_selection_debug
        assert debug is not None
        candidate = next(item for item in debug.candidates if item.selected)
        self.assertEqual(candidate.expected_unknown_gain, 49.0)
        self.assertEqual(selected.expected_unknown_gain, 49.0)
        self.assertAlmostEqual(candidate.optimistic_score, 5.0)
        self.assertAlmostEqual(candidate.final_score, 7.0 - candidate.path_length)
        gain_mock.assert_called_once_with(grid, frontier, target)

    def test_expected_gain_is_cached_for_at_most_two_valid_landings(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=10.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.fill_base(FREE)
        robot_pose = (0.5, 2.5, 0.0)
        targets = [(2.5, 2.5), (3.5, 2.5), (4.5, 2.5)]
        frontier = Frontier(
            cells=((4, 2),),
            centroid=targets[-1],
            information_gain=3.0,
        )
        explorer = FrontierExplorer(
            min_cluster_size=1,
            max_goal_candidates_per_frontier=2,
        )
        planned_goals: list[tuple[float, float]] = []

        def fake_astar(_grid, start, goal, **_kwargs):
            planned_goals.append(goal)
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(
            explorer,
            "_candidate_targets_for_frontier",
            return_value=targets,
        ), patch.object(
            explorer,
            "_expected_unknown_gain",
            side_effect=(9.0, 16.0),
        ) as gain_mock, patch(
            "danger_search.frontier.astar_plan",
            side_effect=fake_astar,
        ):
            selected, _path = explorer.select_frontier(
                grid,
                robot_pose,
                frontiers=[frontier],
            )

        self.assertIsNotNone(selected)
        self.assertEqual(gain_mock.call_count, 2)
        self.assertEqual(planned_goals, targets[:2])

    def test_extraction_thresholds_keep_using_frontier_boundary_count(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.mark_free(2, 2)

        cluster_filtered = FrontierExplorer(min_cluster_size=2)
        self.assertEqual(cluster_filtered.extract_frontiers(grid), [])
        self.assertEqual(
            cluster_filtered._last_extraction_rejections[0].rejection_reason,
            "cluster_below_min_size",
        )

        gain_filtered = FrontierExplorer(
            min_cluster_size=1,
            frontier_min_information_gain=2.0,
        )
        self.assertEqual(gain_filtered.extract_frontiers(grid), [])
        self.assertEqual(
            gain_filtered._last_extraction_rejections[0].rejection_reason,
            "information_gain_below_minimum",
        )

    def test_balanced_breadth_first_prefers_near_frontier_unless_gain_is_extreme(self):
        """常规收益先探索近处，但极高收益仍可覆盖路径代价。"""
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=0.5,
            path_cost_weight=3.0,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
        )
        near = Frontier(
            cells=((0, 0),),
            centroid=(3.0, 0.0),
            information_gain=400.0,
            expected_unknown_gain=400.0,
        )
        far = Frontier(
            cells=((0, 0),),
            centroid=(8.0, 0.0),
            information_gain=1600.0,
            expected_unknown_gain=1600.0,
        )
        extreme_far = Frontier(
            cells=((0, 0),),
            centroid=(8.0, 0.0),
            information_gain=3600.0,
            expected_unknown_gain=3600.0,
        )

        near_score = explorer._score_frontier(
            near,
            [(0.0, 0.0), (3.0, 0.0)],
            (0.0, 0.0, 0.0),
        )
        far_score = explorer._score_frontier(
            far,
            [(0.0, 0.0), (8.0, 0.0)],
            (0.0, 0.0, 0.0),
        )
        extreme_far_score = explorer._score_frontier(
            extreme_far,
            [(0.0, 0.0), (8.0, 0.0)],
            (0.0, 0.0, 0.0),
        )

        self.assertGreater(near_score, far_score)
        self.assertGreater(extreme_far_score, near_score)

    def test_falls_back_to_far_free_cell_when_no_unknown_frontier_exists(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.2,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        grid.mark_visited_radius(0.0, 0.0, 0.4)
        explorer = FrontierExplorer(min_cluster_size=1)

        frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertGreater(math.hypot(frontier.centroid[0], frontier.centroid[1]), 0.35)

    def test_all_real_frontiers_cooled_still_uses_free_space_fallback(self):
        """真实前沿全部冷却时不能直接失败，仍应走既有自由空间出口。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=7.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        for gy in range(1, 4):
            for gx in range(1, 6):
                grid.mark_free(gx, gy)
        start = grid.grid_to_world(1, 2)
        target = grid.grid_to_world(5, 2)
        real_frontier = Frontier(
            cells=((5, 2),),
            centroid=target,
            information_gain=20.0,
        )
        explorer = FrontierExplorer(min_cluster_size=1)
        explorer._candidate_targets_for_frontier = lambda *_args: [target]
        explorer.blacklist_region(grid, target, radius=1.0)

        selected, path = explorer.select_frontier(
            grid,
            (start[0], start[1], 0.0),
            frontiers=[real_frontier],
        )

        self.assertIsNotNone(selected)
        self.assertTrue(path)
        self.assertEqual(selected.information_gain, 0.0)
        self.assertTrue(explorer.last_selection_debug.used_fallback)
        self.assertIn(
            "all_targets_blacklisted",
            {
                candidate.rejection_reason
                for candidate in explorer.last_selection_debug.candidates
            },
        )

    def test_fallback_prefers_unknown_adjacent_over_far_open_space(self):
        """无有效 frontier 进入 free-space fallback 时，应优先选择靠近未知区域的自由格子，
        而不是机械地选择最远开阔点。
        """
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=5.0,
            length=5.0,
            origin_x=-2.5,
            origin_y=-2.5,
        )
        # 全部 FREE，中间放一块 UNKNOWN 作为探索价值锚点
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        grid.set_base_state(2, 2, UNKNOWN)
        explorer = FrontierExplorer(min_cluster_size=1)

        frontier, path = explorer._select_free_space_goal(grid, (0.0, 0.0, 0.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        # 选中的目标应靠近未知块 (0,0)，而非最远角
        self.assertLess(math.hypot(frontier.centroid[0], frontier.centroid[1]), 1.6)

    def test_successful_free_space_fallback_does_not_record_failure(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.2,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        grid.mark_visited_radius(0.0, 0.0, 0.4)
        explorer = FrontierExplorer(min_cluster_size=1, fail_limit=1)

        frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertEqual(explorer._fail_counts, {})
        self.assertEqual(explorer._blacklist, {})

    def test_falls_back_when_no_safe_frontier_goal_exists(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        self._mark_free_rect(grid, 5, 10, 5, 10)
        grid.mark_visited_radius(-1.25, -1.25, 0.3)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            frontier_unknown_standoff=2.5,
            frontier_obstacle_clearance=1.0,
        )

        frontier, path = explorer.select_frontier(grid, (-1.25, -1.25, 0.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertGreater(math.hypot(frontier.centroid[0] + 1.25, frontier.centroid[1] + 1.25), 0.35)

    def test_transition_context_avoids_front_side_frontier_during_hold(self):
        grid = self._make_door_transition_grid()
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=2.0,
            path_cost_weight=0.2,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
            door_front_half_plane_penalty=8.0,
            door_front_half_plane_radius=1.8,
            door_preferred_direction_bonus=1.0,
        )
        context = FrontierSelectionContext(
            forbidden_half_plane_origin=(0.0, 0.0),
            forbidden_half_plane_normal=(0.0, 1.0),
            forbidden_radius=1.8,
            skip_in_forbidden_half_plane=True,
            preferred_origin=(0.0, 0.0),
            preferred_direction=(0.0, 1.0),
            temporary_revisit_penalty=8.0,
        )

        frontier, path = explorer.select_frontier(grid, (0.0, 1.5, math.pi / 2.0), selection_context=context)

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertGreater(frontier.centroid[1], 0.0)

    def test_transition_context_expiry_allows_front_side_revisit(self):
        grid = self._make_door_transition_grid()
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=2.0,
            path_cost_weight=0.2,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
        )

        frontier, path = explorer.select_frontier(grid, (0.0, 1.5, math.pi / 2.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertLess(frontier.centroid[1], 0.0)

    def test_permanent_context_has_independent_decaying_penalty(self):
        explorer = FrontierExplorer(min_cluster_size=1)
        context = FrontierSelectionContext(
            permanent_forbidden_half_plane_origin=(0.0, 0.0),
            permanent_forbidden_half_plane_normal=(1.0, 0.0),
            permanent_forbidden_radius=1.0,
            permanent_forbidden_penalty=3.0,
            permanent_forbidden_decay_distance=3.0,
        )

        hard_adjustment, hard_skip, hard_penalty = explorer._selection_context_effect(
            (-0.5, 0.0),
            context,
        )
        far_adjustment, far_skip, far_penalty = explorer._selection_context_effect(
            (-4.0, 0.0),
            context,
        )

        self.assertTrue(hard_skip)
        self.assertEqual(hard_adjustment, 0.0)
        self.assertIsNone(hard_penalty)
        self.assertFalse(far_skip)
        self.assertLess(far_adjustment, 0.0)
        self.assertGreater(far_penalty, 0.0)
        self.assertLess(far_penalty, 3.0)

    def test_permanent_gate_only_blocks_return_path_through_door_opening(self):
        explorer = FrontierExplorer(min_cluster_size=1)
        context = FrontierSelectionContext(
            permanent_forbidden_half_plane_origin=(0.0, 0.0),
            permanent_forbidden_half_plane_normal=(0.0, 1.0),
            permanent_forbidden_radius=0.0,
            permanent_gate_half_width=1.0,
        )

        self.assertTrue(
            explorer._path_crosses_permanent_gate([(0.0, 2.0), (0.0, -2.0)], context)
        )
        self.assertFalse(
            explorer._path_crosses_permanent_gate([(2.0, 2.0), (2.0, -2.0)], context)
        )
        self.assertFalse(
            explorer._path_crosses_permanent_gate([(0.0, -2.0), (0.0, 2.0)], context)
        )

    def test_permanent_gate_rejects_planned_return_without_recording_unreachable(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        self._mark_free_rect(grid, 0, 7, 0, 7)
        explorer = FrontierExplorer(min_cluster_size=1, max_goal_candidates_per_frontier=1)
        frontier = Frontier(cells=((4, 1),), centroid=(0.5, -2.5), information_gain=4.0)
        context = FrontierSelectionContext(
            permanent_forbidden_half_plane_origin=(0.0, 0.0),
            permanent_forbidden_half_plane_normal=(0.0, 1.0),
            permanent_forbidden_radius=0.0,
            permanent_gate_half_width=1.0,
        )
        failures: list[tuple[float, float]] = []
        explorer.record_failure = lambda _grid, target: failures.append(target)

        with patch.object(explorer, "_candidate_targets_for_frontier", return_value=[(0.0, -2.0)]), patch(
            "danger_search.frontier.astar_plan",
            return_value=PlanResult(path=[(0.0, 2.0), (0.0, -2.0)], status=REACHED),
        ):
            target, path, _, _, context_filtered, _, _, debug = explorer._resolve_frontier_target(
                grid,
                frontier,
                (0.0, 2.0, -math.pi / 2.0),
                selection_context=context,
            )

        self.assertIsNone(target)
        self.assertEqual(path, [])
        self.assertTrue(context_filtered)
        self.assertEqual(debug.rejection_reason, "door_context_path_gate")
        self.assertEqual(debug.path_length, 4.0)
        self.assertEqual(failures, [])

    def test_select_frontier_with_none_context_matches_default_behavior(self):
        grid = self._make_door_transition_grid()
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=2.0,
            path_cost_weight=0.2,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
        )

        default_frontier, default_path = explorer.select_frontier(grid, (0.0, 1.5, math.pi / 2.0))
        explicit_frontier, explicit_path = explorer.select_frontier(
            grid,
            (0.0, 1.5, math.pi / 2.0),
            selection_context=None,
        )

        self.assertIsNotNone(default_frontier)
        self.assertIsNotNone(explicit_frontier)
        self.assertTrue(default_path)
        self.assertTrue(explicit_path)
        self.assertEqual(default_frontier.centroid, explicit_frontier.centroid)
        self.assertEqual(default_path, explicit_path)

    def test_current_region_inertia_requires_clear_score_improvement_to_switch(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=20.0,
            length=20.0,
            origin_x=-10.0,
            origin_y=-10.0,
        )
        self._mark_free_rect(grid, 2, 5, 2, 5)
        self._mark_free_rect(grid, 13, 17, 13, 17)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=1.0,
            path_cost_weight=0.0,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
            frontier_obstacle_clearance=0.0,
            frontier_unknown_standoff=0.0,
        )
        context = FrontierSelectionContext(
            current_target=(-6.5, -6.5),
            current_score=0.0,
            current_region_radius=4.0,
            switch_score_margin=20.0,
        )

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(path=[start, goal], status=REACHED)

        all_free = frozenset(
            (gx, gy)
            for gy in range(grid.size_y)
            for gx in range(grid.size_x)
            if grid.data[gy, gx] in (FREE, VISITED)
        )
        with patch(
            "danger_search.frontier.astar_plan", side_effect=fake_astar
        ), patch(
            "danger_search.frontier.reachable_traversable_component",
            return_value=all_free,
        ), patch.object(
            explorer,
            "_expected_unknown_gain",
            side_effect=lambda _grid, _frontier, target: (
                16.0 if target[0] > 0.0 else 1.0
            ),
        ):
            frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0), selection_context=context)

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertLess(frontier.centroid[0], 0.0)
        self.assertIsNotNone(explorer.last_selection_debug)
        assert explorer.last_selection_debug is not None
        self.assertTrue(explorer.last_selection_debug.retained_by_inertia)

    def test_budget_exhausted_frontier_does_not_record_failure(self):
        """前沿规划预算耗尽时不应累计失败，避免误黑名单可能可达的目标。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=10.0,
            length=10.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        # 只标记一小块 free，其余保持 unknown，确保能提取到前沿。
        self._mark_free_rect(grid, 0, 2, 0, 2)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=1.0,
            path_cost_weight=0.0,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
        )
        recorded_failures: list[tuple[float, float]] = []
        explorer.record_failure = lambda g, xy: recorded_failures.append(xy)

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(status=BUDGET_EXHAUSTED)

        with patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            frontier, path = explorer.select_frontier(grid, (1.0, 1.0, 0.0))

        # 预算耗尽：不选择该前沿，也不累计失败。
        self.assertEqual(recorded_failures, [])
        debug = explorer.last_selection_debug
        assert debug is not None
        self.assertGreater(debug.budget_exhausted_count, 0)

    def test_budget_exhausted_frontier_can_be_confirmed_once_without_budget(self):
        """完整扫描后应能确认预算未决候选，而不是把它计为不可达。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=10.0,
            length=10.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        self._mark_free_rect(grid, 0, 9, 0, 9)
        explorer = FrontierExplorer(min_cluster_size=1)
        candidate = Frontier(
            cells=((8, 8),),
            centroid=(8.5, 8.5),
            information_gain=20.0,
        )
        iteration_limits = []

        def fake_astar(_grid, start, goal, *args, **kwargs):
            explicit_limit = kwargs.get("max_iterations", "default")
            iteration_limits.append(explicit_limit)
            if explicit_limit is None:
                return PlanResult(path=[start, goal], status=REACHED)
            return PlanResult(status=BUDGET_EXHAUSTED)

        with patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            selected, path = explorer.select_frontier(
                grid,
                (1.5, 1.5, 0.0),
                frontiers=[candidate],
            )
            confirmed, confirmed_path = explorer.confirm_budget_exhausted_frontier(
                grid,
                (1.5, 1.5, 0.0),
            )

        self.assertIsNone(selected)
        self.assertEqual(path, [])
        self.assertIsNotNone(confirmed)
        self.assertTrue(confirmed_path)
        self.assertIn(None, iteration_limits)

    def test_unreachable_frontier_records_failure(self):
        """真正不可达的前沿应累计失败，以便后续换目标。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=10.0,
            length=10.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        self._mark_free_rect(grid, 0, 9, 0, 9)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=1.0,
            path_cost_weight=0.0,
            yaw_cost_weight=0.0,
            revisit_penalty_weight=0.0,
        )
        recorded_failures: list[tuple[float, float]] = []
        explorer.record_failure = lambda g, xy: recorded_failures.append(xy)

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(status=UNREACHABLE)

        with patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            frontier, path = explorer.select_frontier(grid, (1.0, 1.0, 0.0))

        self.assertGreater(len(recorded_failures), 0)

    # ------------------------------------------------------------------
    # 近邻 Frontier 退化循环修复：inside_goal_tolerance 源头过滤测试
    # ------------------------------------------------------------------

    def test_inside_goal_tolerance_rejects_without_failure(self):
        """落点在 controller goal_tolerance 内时，应跳过且不记录失败、不拉黑。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gy in range(5, 35):
            for gx in range(5, 35):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            goal_tolerance=0.35,
            fail_limit=1,
        )
        recorded_failures: list[tuple[float, float]] = []
        explorer.record_failure = lambda g, xy: recorded_failures.append(xy)

        frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        # 不应选中在 tolerance 内的落点
        if frontier is not None and path:
            self.assertGreater(
                math.hypot(frontier.centroid[0], frontier.centroid[1]),
                0.35,
            )
        self.assertEqual(recorded_failures, [])
        self.assertEqual(explorer._blacklist, {})
        self.assertEqual(explorer._fail_counts, {})

    def test_euclidean_not_path_length_for_tolerance_check(self):
        """历史回放：欧氏距离 0.31m <= tolerance 0.35m，即使 path_length 0.38m > 0.35m 也应拒绝。

        证明过滤使用欧氏距离而非路径长度。
        """
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gy in range(5, 35):
            for gx in range(5, 35):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            goal_tolerance=0.35,
        )
        recorded_failures: list[tuple[float, float]] = []
        explorer.record_failure = lambda g, xy: recorded_failures.append(xy)

        # 构造一个 frontier，其安全落点距 robot 欧氏距离 0.31m
        # mock astar 返回一条弯曲路径，path_length 约 0.38m
        target = (0.31, 0.0)
        frontier = Frontier(
            cells=((20, 20),),
            centroid=target,
            information_gain=10.0,
        )

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            # 弯曲路径：0 -> (0.2, 0.18) -> (0.31, 0)，路径长度 > 0.38
            return PlanResult(
                path=[start, (0.2, 0.18), goal],
                status=REACHED,
            )

        with patch.object(
            explorer, "_candidate_targets_for_frontier", return_value=[target]
        ), patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            result = explorer._resolve_frontier_target(
                grid, frontier, (0.0, 0.0, 0.0), selection_context=None
            )

        # target_xy 应为 None：被 inside_goal_tolerance 拒绝
        self.assertIsNone(result[0])
        self.assertEqual(result[1], [])
        # inside_tolerance_only 标志应为 True
        self.assertTrue(result[6])
        debug = result[7]
        self.assertEqual(debug.rejection_reason, "inside_goal_tolerance")
        self.assertEqual(debug.reachability_result, "inside_goal_tolerance")
        # 不记录失败
        self.assertEqual(recorded_failures, [])

    def test_tolerance_skip_does_not_block_other_cluster_candidates(self):
        """同一 cluster 中一个候选在 tolerance 内，另一个在 tolerance 外，应选中后者。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gy in range(5, 55):
            for gx in range(5, 55):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            goal_tolerance=0.35,
            max_goal_candidates_per_frontier=10,
        )

        # 两个候选：一个在 tolerance 内 (0.20, 0.0)，一个在 tolerance 外 (1.50, 0.0)
        near_target = (0.20, 0.0)
        far_target = (1.50, 0.0)
        frontier = Frontier(
            cells=((20, 20),),
            centroid=(0.20, 0.0),
            information_gain=10.0,
        )

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(
            explorer,
            "_candidate_targets_for_frontier",
            return_value=[near_target, far_target],
        ), patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            result = explorer._resolve_frontier_target(
                grid, frontier, (0.0, 0.0, 0.0), selection_context=None
            )

        # 应选中 far_target
        self.assertIsNotNone(result[0])
        self.assertEqual(result[0], far_target)
        self.assertTrue(result[1])

    def test_inside_tolerance_in_free_space_fallback(self):
        """Fallback 候选在 tolerance 内时也应跳过，不选中过近落点。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=0.2,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        grid.mark_visited_radius(0.0, 0.0, 0.3)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            goal_tolerance=0.35,
        )

        frontier, path = explorer._select_free_space_goal(grid, (0.0, 0.0, 0.0))

        # 选中的目标应在 tolerance 外
        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertGreater(
            math.hypot(frontier.centroid[0], frontier.centroid[1]),
            0.35,
        )

    def test_all_frontiers_inside_tolerance_no_failure_recorded(self):
        """所有 frontier 落点都在 tolerance 内时，不应记录失败或拉黑。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=2.0,
            length=2.0,
            origin_x=-1.0,
            origin_y=-1.0,
        )
        # 小地图，robot 在中心，所有 frontier 落点都在 0.35m 内
        for gy in range(5, 15):
            for gx in range(5, 15):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            goal_tolerance=0.35,
            fail_limit=1,
        )
        recorded_failures: list[tuple[float, float]] = []
        explorer.record_failure = lambda g, xy: recorded_failures.append(xy)

        frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        # 可能选中 fallback（如果存在 tolerance 外的 FREE 格子）或 None
        # 无论哪种，都不应记录 failure
        self.assertEqual(recorded_failures, [])
        self.assertEqual(explorer._blacklist, {})
        self.assertEqual(explorer._fail_counts, {})

    def test_centroid_inside_tolerance_but_safe_goal_outside_allowed(self):
        """Frontier centroid 在 tolerance 内，但最终安全落点在 tolerance 外时应允许。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gy in range(5, 55):
            for gx in range(5, 55):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            goal_tolerance=0.35,
        )

        # centroid 在 tolerance 内 (0.20, 0.0)，但安全落点在 tolerance 外 (1.50, 0.0)
        near_centroid = (0.20, 0.0)
        far_safe_goal = (1.50, 0.0)
        frontier = Frontier(
            cells=((20, 20),),
            centroid=near_centroid,
            information_gain=10.0,
        )

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(
            explorer,
            "_candidate_targets_for_frontier",
            return_value=[far_safe_goal],
        ), patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            result = explorer._resolve_frontier_target(
                grid, frontier, (0.0, 0.0, 0.0), selection_context=None
            )

        # 应选中 far_safe_goal（在 tolerance 外）
        self.assertIsNotNone(result[0])
        self.assertEqual(result[0], far_safe_goal)
        self.assertTrue(result[1])

    def test_safe_goal_inside_tolerance_but_centroid_outside_rejected(self):
        """反向：centroid 在 tolerance 外，但最终安全落点在 tolerance 内时应拒绝该落点。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gy in range(5, 55):
            for gx in range(5, 55):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            goal_tolerance=0.35,
        )

        # centroid 在 tolerance 外 (2.0, 0.0)，但安全落点在 tolerance 内 (0.30, 0.0)
        far_centroid = (2.0, 0.0)
        near_safe_goal = (0.30, 0.0)
        frontier = Frontier(
            cells=((20, 20),),
            centroid=far_centroid,
            information_gain=10.0,
        )

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(
            explorer,
            "_candidate_targets_for_frontier",
            return_value=[near_safe_goal],
        ), patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            result = explorer._resolve_frontier_target(
                grid, frontier, (0.0, 0.0, 0.0), selection_context=None
            )

        # 应拒绝：target_xy 为 None，inside_tolerance_only 为 True
        self.assertIsNone(result[0])
        self.assertTrue(result[6])
        debug = result[7]
        self.assertEqual(debug.rejection_reason, "inside_goal_tolerance")

    # ------------------------------------------------------------------
    # 最小补丁补充测试
    # ------------------------------------------------------------------

    def test_distance_equal_to_tolerance_rejected(self):
        """distance == goal_tolerance 应拒绝（与 controller <= 一致）。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=6.0, length=6.0, origin_x=-3.0, origin_y=-3.0,
        )
        for gy in range(5, 55):
            for gx in range(5, 55):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(min_cluster_size=1, goal_tolerance=0.35)
        target = (0.35, 0.0)
        frontier = Frontier(cells=((20, 20),), centroid=target, information_gain=10.0)

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(explorer, "_candidate_targets_for_frontier", return_value=[target]), \
             patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            result = explorer._resolve_frontier_target(grid, frontier, (0.0, 0.0, 0.0), selection_context=None)

        self.assertIsNone(result[0])
        self.assertTrue(result[6])

    def test_distance_just_above_tolerance_allowed(self):
        """distance 刚大于 goal_tolerance 应允许。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=6.0, length=6.0, origin_x=-3.0, origin_y=-3.0,
        )
        for gy in range(5, 55):
            for gx in range(5, 55):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(min_cluster_size=1, goal_tolerance=0.35)
        target = (0.36, 0.0)
        frontier = Frontier(cells=((20, 20),), centroid=target, information_gain=10.0)

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(explorer, "_candidate_targets_for_frontier", return_value=[target]), \
             patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            result = explorer._resolve_frontier_target(grid, frontier, (0.0, 0.0, 0.0), selection_context=None)

        self.assertIsNotNone(result[0])
        self.assertEqual(result[0], target)

    def test_frontier_a_all_inside_tolerance_frontier_b_selected(self):
        """frontier A 全部过近、frontier B 有有效落点：选择 B。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=8.0, length=8.0, origin_x=-4.0, origin_y=-4.0,
        )
        for gy in range(5, 75):
            for gx in range(5, 75):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(min_cluster_size=1, goal_tolerance=0.35)

        frontier_a = Frontier(cells=((10, 10),), centroid=(0.20, 0.0), information_gain=5.0)
        frontier_b = Frontier(cells=((60, 60),), centroid=(3.0, 3.0), information_gain=8.0)

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(explorer, "extract_frontiers", return_value=[frontier_a, frontier_b]), \
             patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            # mock _candidate_targets to force specific targets
            def fake_candidates(_grid, f, _robot):
                if f is frontier_a:
                    return [(0.20, 0.0)]
                return [(3.0, 3.0)]
            with patch.object(explorer, "_candidate_targets_for_frontier", side_effect=fake_candidates):
                selected, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        self.assertIsNotNone(selected)
        self.assertEqual(selected.centroid, (3.0, 3.0))

    def test_same_cluster_first_two_inside_third_still_checked(self):
        """同 cluster 前两个落点在 tolerance 内，第三个在 tolerance 外且可达，必须选中第三个。

        证明 inside_goal_tolerance 不消耗 max_goal_candidates_per_frontier 预算。
        """
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=8.0, length=8.0, origin_x=-4.0, origin_y=-4.0,
        )
        for gy in range(5, 75):
            for gx in range(5, 75):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(
            min_cluster_size=1, goal_tolerance=0.35, max_goal_candidates_per_frontier=2,
        )

        t1 = (0.20, 0.0)  # inside tolerance
        t2 = (0.30, 0.0)  # inside tolerance
        t3 = (2.0, 0.0)   # outside tolerance, reachable
        frontier = Frontier(cells=((20, 20),), centroid=(0.20, 0.0), information_gain=10.0)

        astar_calls = []
        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            astar_calls.append(goal)
            return PlanResult(path=[start, goal], status=REACHED)

        with patch.object(explorer, "_candidate_targets_for_frontier", return_value=[t1, t2, t3]), \
             patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            result = explorer._resolve_frontier_target(grid, frontier, (0.0, 0.0, 0.0), selection_context=None)

        # 必须选中第三个
        self.assertIsNotNone(result[0])
        self.assertEqual(result[0], t3)
        # A* 只对 t3 调用过（t1, t2 在 A* 前被 tolerance 拒绝）
        self.assertEqual(len(astar_calls), 1)
        self.assertEqual(astar_calls[0], t3)

    def test_fallback_inside_tolerance_branch_executes(self):
        """fallback 的 inside_goal_tolerance 预过滤真实执行：不选 tolerance 内候选。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=3.0, length=3.0, origin_x=-1.5, origin_y=-1.5,
        )
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        grid.mark_visited_radius(0.0, 0.0, 0.3)
        explorer = FrontierExplorer(min_cluster_size=1, goal_tolerance=0.35)

        frontier, path = explorer._select_free_space_goal(grid, (0.0, 0.0, 0.0))

        # 选中的目标应在 tolerance 外
        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertGreater(math.hypot(frontier.centroid[0], frontier.centroid[1]), 0.35)

    def test_fallback_preserves_legacy_one_meter_prefilter(self):
        """fallback 仅允许 1m 及更远候选进入 A*，不能重新开放 0.35~1m 区间。"""
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=1.0,
            length=0.1,
            origin_x=0.15,
            origin_y=-0.05,
        )
        # fallback stride 为 4 格，三个真实采样点分别位于 0.2m、0.6m、1.0m。
        for gx in (0, 4, 8):
            grid.mark_free(gx, 0)
        explorer = FrontierExplorer(min_cluster_size=1, goal_tolerance=0.35)
        astar_goals = []
        recorded_failures = []
        explorer.record_failure = lambda _grid, goal: recorded_failures.append(goal)

        def fake_astar(_grid, start, goal, max_iterations=None, **_kwargs):
            astar_goals.append(goal)
            return PlanResult(path=[start, goal], status=REACHED)

        with patch("danger_search.frontier.astar_plan", side_effect=fake_astar):
            frontier, path = explorer._select_free_space_goal(grid, (0.0, 0.0, 0.0))

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertEqual(astar_goals, [(1.0, 0.0)])
        self.assertEqual(recorded_failures, [])

    def test_forward_alignment_favors_straight_corridor_over_perpendicular_branch(self):
        """验证启用 forward_alignment_weight 后，同等收益下正前方走廊目标得分高于垂直侧向目标。"""
        explorer = FrontierExplorer(
            min_cluster_size=1,
            information_gain_weight=1.0,
            path_cost_weight=1.0,
            yaw_cost_weight=0.0,
            forward_alignment_weight=1.0,
            revisit_penalty_weight=0.0,
        )
        robot_pose = (0.0, 0.0, 0.0)  # 朝向 X+
        straight_frontier = Frontier(cells=((10, 0),), centroid=(5.0, 0.0), information_gain=16.0)
        perpendicular_frontier = Frontier(cells=((0, 10),), centroid=(0.0, 5.0), information_gain=16.0)

        score_straight = explorer._score_frontier(
            straight_frontier,
            [(0.0, 0.0), (5.0, 0.0)],
            robot_pose,
            target_xy=(5.0, 0.0),
            expected_unknown_gain=16.0,
        )
        score_perp = explorer._score_frontier(
            perpendicular_frontier,
            [(0.0, 0.0), (0.0, 5.0)],
            robot_pose,
            target_xy=(0.0, 5.0),
            expected_unknown_gain=16.0,
        )
        # 正前方 alignment cos(0)=1.0, 垂直侧向 alignment cos(pi/2)=0.0
        # 收益与路程相同情况下，正前方得分高出 1.0
        self.assertAlmostEqual(score_straight - score_perp, 1.0)

    def test_far_small_enclosed_pocket_is_pruned(self):
        """验证距离超过 3.5m 且期望收益小于 4 的封闭孤岛前沿被直接剪枝。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=10.0, length=10.0,
            origin_x=0.0, origin_y=0.0,
        )
        robot_pose = (0.0, 0.0, 0.0)
        # 远距离小前沿 (距离 5.0m, gain=3 < 4)
        far_target = (5.0, 0.0)
        far_frontier = Frontier(cells=((50, 0),), centroid=far_target, information_gain=3.0)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            min_far_expected_gain=4.0,
            far_prune_distance=3.5,
        )
        explorer._candidate_targets_for_frontier = lambda *_args: [far_target]
        explorer._expected_unknown_gain = lambda *_args: 3.0

        targets, planning, gains, scores, best_target, best_score = explorer._prepare_frontier_targets(
            grid, far_frontier, robot_pose, None
        )
        self.assertIsNone(best_target)
        self.assertEqual(best_score, -math.inf)
        self.assertEqual(scores, {})

    def test_far_small_pocket_cannot_reenter_astar_after_preplanning(self):
        """同一剪枝判据在 A* 阶段也必须阻止远距小前沿复活。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=10.0, length=10.0,
            origin_x=0.0, origin_y=0.0,
        )
        robot_pose = (0.0, 0.0, 0.0)
        far_target = (5.0, 0.0)
        frontier = Frontier(cells=((50, 0),), centroid=far_target, information_gain=3.0)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            min_far_expected_gain=4.0,
            far_prune_distance=3.5,
        )

        with patch("danger_search.frontier.astar_plan") as astar:
            target, path, *_unused, debug = explorer._resolve_frontier_target(
                grid,
                frontier,
                robot_pose,
                candidate_targets=[far_target],
                expected_gain_cache={far_target: 3.0},
            )

        self.assertIsNone(target)
        self.assertEqual(path, [])
        self.assertEqual(debug.rejection_reason, "far_small_pocket_pruned")
        astar.assert_not_called()

    def test_near_small_enclosed_pocket_is_retained(self):
        """验证距离在 3.5m 以内的小前沿正常保留（近场顺路扫除）。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=10.0, length=10.0,
            origin_x=0.0, origin_y=0.0,
        )
        robot_pose = (0.0, 0.0, 0.0)
        # 近距离小前沿 (距离 2.0m, gain=3 < 4)
        near_target = (2.0, 0.0)
        near_frontier = Frontier(cells=((20, 0),), centroid=near_target, information_gain=3.0)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            min_far_expected_gain=4.0,
            far_prune_distance=3.5,
        )
        explorer._candidate_targets_for_frontier = lambda *_args: [near_target]
        explorer._expected_unknown_gain = lambda *_args: 3.0

        targets, planning, gains, scores, best_target, best_score = explorer._prepare_frontier_targets(
            grid, near_frontier, robot_pose, None
        )
        self.assertEqual(best_target, near_target)
        self.assertGreater(best_score, -math.inf)
        self.assertIn(near_target, scores)

    def test_far_threshold_frontier_is_retained(self):
        """验证距离超过 3.5m 且期望收益等于 4 的前沿仍可进入规划。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=10.0, length=10.0,
            origin_x=0.0, origin_y=0.0,
        )
        robot_pose = (0.0, 0.0, 0.0)
        # 临界收益前沿 (距离 5.0m, gain=4 == 4)
        far_target = (5.0, 0.0)
        far_frontier = Frontier(cells=((50, 0),), centroid=far_target, information_gain=4.0)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            min_far_expected_gain=4.0,
            far_prune_distance=3.5,
        )
        explorer._candidate_targets_for_frontier = lambda *_args: [far_target]
        explorer._expected_unknown_gain = lambda *_args: 4.0

        targets, planning, gains, scores, best_target, best_score = explorer._prepare_frontier_targets(
            grid, far_frontier, robot_pose, None
        )
        self.assertEqual(best_target, far_target)
        self.assertGreater(best_score, -math.inf)
        self.assertIn(far_target, scores)

    def test_select_frontier_prunes_far_small_pocket_end_to_end(self):
        """验证端到端 select_frontier 会彻底跳过远距离小前沿，不作为有效前沿选择。"""
        grid = FloorGrid(
            floor_index=0, resolution=0.1, width=10.0, length=10.0,
            origin_x=0.0, origin_y=0.0,
        )
        for gy in range(1, 90):
            for gx in range(1, 90):
                grid.mark_free(gx, gy)
        robot_pose = (0.5, 0.5, 0.0)
        # 远距离小前沿 (距离 5.0m, 期望收益 3 < 4)
        far_target = (5.5, 0.5)
        far_frontier = Frontier(cells=((55, 5),), centroid=far_target, information_gain=3.0)
        explorer = FrontierExplorer(
            min_cluster_size=1,
            min_far_expected_gain=4.0,
            far_prune_distance=3.5,
        )
        explorer._candidate_targets_for_frontier = lambda *_args: [far_target]
        explorer._expected_unknown_gain = lambda *_args: 3.0

        selected, _path = explorer.select_frontier(
            grid, robot_pose, None, frontiers=[far_frontier]
        )
        # 真实前沿被剪枝跳过，未被作为真实前沿选中
        if selected is not None:
            self.assertEqual(selected.information_gain, 0.0)
            self.assertTrue(explorer.last_selection_debug.used_fallback)
        self.assertIn(
            "far_small_pocket_pruned",
            {
                candidate.rejection_reason
                for candidate in explorer.last_selection_debug.candidates
            },
        )


if __name__ == "__main__":
    unittest.main()

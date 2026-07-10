import math
import unittest
from unittest.mock import patch

from danger_search.frontier import FrontierExplorer
from danger_search.frontier import Frontier
from danger_search.frontier import FrontierSelectionContext
from danger_search.mapper import BLACK, FloorGrid, OCC


class FrontierExplorerTest(unittest.TestCase):
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
        grid.data[4, 9] = BLACK

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

    def test_score_frontier_uses_actual_target_for_yaw_cost(self):
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

        self.assertGreater(score_with_safe_target, score_with_old_centroid)

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
        self.assertGreater(math.hypot(frontier.centroid[0], frontier.centroid[1]), 1.0)

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

        def fake_astar(_grid, start, goal):
            return [start, goal]

        with patch("danger_search.frontier.astar_path", side_effect=fake_astar):
            frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0), selection_context=context)

        self.assertIsNotNone(frontier)
        self.assertTrue(path)
        self.assertLess(frontier.centroid[0], 0.0)
        self.assertIsNotNone(explorer.last_selection_debug)
        assert explorer.last_selection_debug is not None
        self.assertTrue(explorer.last_selection_debug.retained_by_inertia)


if __name__ == "__main__":
    unittest.main()

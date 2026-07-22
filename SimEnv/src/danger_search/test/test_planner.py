"""A* 路径规划相关测试。"""
from __future__ import annotations

import math
import unittest

from danger_search.mapper import FloorGrid
from danger_search.planner import (
    BUDGET_EXHAUSTED,
    REACHED,
    UNREACHABLE,
    astar_path,
    astar_plan,
    path_length,
    simplify_path_line_of_sight,
)


class PlannerTest(unittest.TestCase):
    def _make_long_corridor(self, resolution: float = 0.5) -> FloorGrid:
        grid = FloorGrid(
            floor_index=0,
            resolution=resolution,
            width=20.0,
            length=2.0,
            origin_x=-10.0,
            origin_y=-1.0,
        )
        for gx in range(grid.size_x):
            for gy in range(1, grid.size_y - 1):
                grid.mark_free(gx, gy)
        return grid

    def test_astar_finds_full_path_when_reachable(self):
        grid = self._make_long_corridor()
        start = grid.grid_to_world(2, 2)
        goal = grid.grid_to_world(grid.size_x - 3, 2)

        path = astar_path(grid, start, goal)

        self.assertTrue(path)
        self.assertGreater(path_length(path), 0.0)
        self.assertAlmostEqual(path[0][0], start[0], places=5)
        self.assertAlmostEqual(path[-1][0], goal[0], places=5)

    def test_line_of_sight_simplifies_grid_staircase_to_straight_segment(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        path = [
            grid.grid_to_world(10, 10),
            grid.grid_to_world(11, 11),
            grid.grid_to_world(11, 12),
            grid.grid_to_world(12, 13),
            grid.grid_to_world(12, 14),
        ]

        simplified = simplify_path_line_of_sight(grid, path)

        self.assertEqual(simplified, [path[0], path[-1]])

    def test_line_of_sight_keeps_bend_around_obstacle(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        obstacle = (grid.size_x // 2, grid.size_y // 2)
        grid.mark_occupied(*obstacle)
        start = grid.grid_to_world(obstacle[0] - 3, obstacle[1])
        goal = grid.grid_to_world(obstacle[0] + 3, obstacle[1])

        result = astar_plan(grid, start, goal, max_iterations=None)

        self.assertTrue(result.reached)
        self.assertGreater(len(result.path), 2)

    def test_line_of_sight_does_not_cut_blocked_diagonal_corner(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        grid.mark_occupied(3, 2)
        path = [
            grid.grid_to_world(2, 2),
            grid.grid_to_world(2, 3),
            grid.grid_to_world(3, 3),
        ]

        simplified = simplify_path_line_of_sight(grid, path)

        self.assertEqual(simplified, path)

    def test_astar_returns_empty_when_budget_exhausted_without_progress(self):
        grid = self._make_long_corridor()
        start = grid.grid_to_world(2, 2)
        goal = grid.grid_to_world(grid.size_x - 3, 2)

        # 只让算法展开一步，best_node 仍然是 start，必须返回 [] 而不是零长度路径
        path = astar_path(grid, start, goal, max_iterations=1)

        self.assertEqual(path, [])

    def test_astar_returns_empty_when_budget_exhausted_with_progress(self):
        grid = self._make_long_corridor(resolution=0.2)
        start = grid.grid_to_world(5, 5)
        goal = grid.grid_to_world(grid.size_x - 5, 5)

        # 预算足够走一段但到不了终点，仍必须返回 []，避免上层把部分路径误判为已到达目标。
        path = astar_path(grid, start, goal, max_iterations=50)

        self.assertEqual(path, [])

    def test_astar_returns_empty_for_truly_unreachable(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        # 在中间竖一堵墙
        wall_x = grid.size_x // 2
        for gy in range(grid.size_y):
            grid.mark_occupied(wall_x, gy)

        start = grid.grid_to_world(1, grid.size_y // 2)
        goal = grid.grid_to_world(grid.size_x - 2, grid.size_y // 2)

        path = astar_path(grid, start, goal)

        self.assertEqual(path, [])

    def test_path_length_empty_or_single(self):
        self.assertEqual(path_length([]), 0.0)
        self.assertEqual(path_length([(0.0, 0.0)]), 0.0)

    def test_astar_plan_status_reached_for_reachable_goal(self):
        grid = self._make_long_corridor()
        start = grid.grid_to_world(2, 2)
        goal = grid.grid_to_world(grid.size_x - 3, 2)

        result = astar_plan(grid, start, goal)

        self.assertEqual(result.status, REACHED)
        self.assertTrue(result.reached)
        self.assertGreater(path_length(result.path), 0.0)

    def test_astar_plan_status_budget_exhausted(self):
        grid = self._make_long_corridor(resolution=0.2)
        start = grid.grid_to_world(5, 5)
        goal = grid.grid_to_world(grid.size_x - 5, 5)

        result = astar_plan(grid, start, goal, max_iterations=50)

        self.assertEqual(result.status, BUDGET_EXHAUSTED)
        self.assertTrue(result.budget_exhausted)
        self.assertFalse(result.unreachable)
        self.assertEqual(result.path, [])

    def test_astar_plan_status_unreachable(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        wall_x = grid.size_x // 2
        for gy in range(grid.size_y):
            grid.mark_occupied(wall_x, gy)

        start = grid.grid_to_world(1, grid.size_y // 2)
        goal = grid.grid_to_world(grid.size_x - 2, grid.size_y // 2)

        result = astar_plan(grid, start, goal, max_iterations=None)

        self.assertEqual(result.status, UNREACHABLE)
        self.assertTrue(result.unreachable)
        self.assertFalse(result.budget_exhausted)

    def test_astar_rejects_occupied_start_cell(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        start_cell = (2, 2)
        grid.mark_occupied(*start_cell)

        result = astar_plan(
            grid,
            grid.grid_to_world(*start_cell),
            grid.grid_to_world(6, 6),
            max_iterations=None,
        )

        self.assertEqual(result.status, UNREACHABLE)
        self.assertEqual(result.path, [])

    def test_blocked_goal_uses_geometrically_nearest_free_cell(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        goal_cell = (2, 2)
        grid.mark_occupied(*goal_cell)

        result = astar_plan(
            grid,
            grid.grid_to_world(2, 0),
            grid.grid_to_world(*goal_cell),
            max_iterations=None,
        )

        self.assertTrue(result.reached)
        self.assertEqual(result.path[-1], grid.grid_to_world(2, 1))

    def test_astar_rejects_invalid_iteration_budget(self):
        grid = self._make_long_corridor()
        start = grid.grid_to_world(2, 2)
        goal = grid.grid_to_world(grid.size_x - 3, 2)

        with self.assertRaises(ValueError):
            astar_plan(grid, start, goal, max_iterations=0)
        with self.assertRaises(ValueError):
            astar_plan(grid, start, goal, max_iterations=math.nan)


if __name__ == "__main__":
    unittest.main()

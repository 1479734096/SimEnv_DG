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


if __name__ == "__main__":
    unittest.main()

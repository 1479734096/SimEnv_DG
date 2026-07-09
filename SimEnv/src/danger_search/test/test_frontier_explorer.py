import math
import unittest

from danger_search.frontier import FrontierExplorer
from danger_search.mapper import FloorGrid


class FrontierExplorerTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()

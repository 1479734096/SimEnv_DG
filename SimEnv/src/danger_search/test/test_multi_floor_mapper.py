import unittest

from danger_search.frontier_explorer import astar_path
from danger_search.multi_floor_mapper import MultiFloorGridMap, OCC


class MultiFloorMapperTest(unittest.TestCase):
    def test_obstacle_points_are_inflated(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=3.0,
            length=3.0,
            origin_x=-1.5,
            origin_y=-1.5,
            obstacle_inflation_radius=0.2,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[(0.5, 0.0, 0.6)],
        )
        grid = mapper.floor(0)
        center = grid.world_to_grid(0.5, 0.0)
        neighbor = grid.world_to_grid(0.6, 0.0)

        self.assertIsNotNone(center)
        self.assertIsNotNone(neighbor)
        self.assertEqual(grid.data[center[1], center[0]], OCC)
        self.assertEqual(grid.data[neighbor[1], neighbor[0]], OCC)
        self.assertFalse(grid.is_traversable(*center))
        self.assertFalse(grid.is_traversable(*neighbor))

    def test_astar_does_not_cross_inflated_obstacle(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=3.0,
            length=3.0,
            origin_x=-1.5,
            origin_y=-1.5,
            obstacle_inflation_radius=0.2,
        )
        grid = mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        grid.mark_occupied_radius(0.0, 0.0, 0.25)

        path = astar_path(grid, (-1.0, 0.0), (1.0, 0.0))
        blocked_cells = {
            grid.world_to_grid(0.0, 0.0),
            grid.world_to_grid(0.1, 0.0),
            grid.world_to_grid(-0.1, 0.0),
        }
        path_cells = {grid.world_to_grid(x, y) for x, y in path}

        self.assertTrue(path)
        self.assertTrue(blocked_cells.isdisjoint(path_cells))


if __name__ == "__main__":
    unittest.main()

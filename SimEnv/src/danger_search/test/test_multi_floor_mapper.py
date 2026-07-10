import math
import unittest
from typing import Tuple

from danger_search.planner import astar_path
from danger_search.mapper import BLACK, FREE, MultiFloorGridMap, OCC, UNKNOWN, VISITED


class MultiFloorMapperTest(unittest.TestCase):
    @staticmethod
    def _point_from_polar(distance: float, angle: float, z: float = 0.6) -> Tuple[float, float, float]:
        return (distance * math.cos(angle), distance * math.sin(angle), z)

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

    def test_area_and_known_bbox_metrics(self):
        mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid = mapper.floor(0)
        grid.data[1, 1] = FREE
        grid.data[1, 2] = VISITED
        grid.data[3, 4] = OCC

        self.assertAlmostEqual(grid.known_area(), 0.75)
        self.assertAlmostEqual(grid.visited_area(), 0.25)
        self.assertEqual(grid.known_bbox_extent(), (2.0, 1.5))

    def test_free_clear_marks_visible_sector_as_free(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=True,
            free_clear_max_range=1.5,
            free_clear_angle_step=0.3,
            free_clear_obstacle_margin=0.1,
            free_clear_min_points_per_bin=2,
        )

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[
                self._point_from_polar(1.8, -0.12),
                self._point_from_polar(1.8, 0.12),
            ],
        )

        grid = mapper.floor(0)
        clear_cell = grid.world_to_grid(1.0, 0.0)

        self.assertIsNotNone(clear_cell)
        self.assertEqual(grid.data[clear_cell[1], clear_cell[0]], FREE)

    def test_free_clear_ignores_non_obstacle_nearby_points_for_range(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=True,
            free_clear_max_range=1.5,
            free_clear_angle_step=0.3,
            free_clear_obstacle_margin=0.1,
            free_clear_min_points_per_bin=3,
            obstacle_min_z=0.15,
            obstacle_max_z=1.40,
        )

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[
                self._point_from_polar(0.55, 0.02, z=0.0),
                self._point_from_polar(1.8, -0.12, z=0.6),
                self._point_from_polar(1.8, 0.12, z=0.6),
            ],
        )

        grid = mapper.floor(0)
        far_clear_cell = grid.world_to_grid(1.0, 0.0)

        self.assertIsNotNone(far_clear_cell)
        self.assertEqual(grid.data[far_clear_cell[1], far_clear_cell[0]], FREE)

    def test_free_clear_stops_before_known_occupied_cell(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=True,
            free_clear_max_range=1.5,
            free_clear_angle_step=0.3,
            free_clear_obstacle_margin=0.1,
            free_clear_min_points_per_bin=2,
        )
        grid = mapper.floor(0)
        wall_cell = grid.world_to_grid(0.8, 0.0)
        before_wall = grid.world_to_grid(0.6, 0.0)
        after_wall = grid.world_to_grid(1.2, 0.0)

        self.assertIsNotNone(wall_cell)
        self.assertIsNotNone(before_wall)
        self.assertIsNotNone(after_wall)
        grid.mark_occupied(*wall_cell)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[
                self._point_from_polar(1.8, -0.12),
                self._point_from_polar(1.8, 0.12),
            ],
        )

        self.assertEqual(grid.data[before_wall[1], before_wall[0]], FREE)
        self.assertEqual(grid.data[wall_cell[1], wall_cell[0]], OCC)
        self.assertEqual(grid.data[after_wall[1], after_wall[0]], UNKNOWN)

    def test_raycast_free_stops_before_known_occupied_cell(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=False,
        )
        grid = mapper.floor(0)
        wall_cell = grid.world_to_grid(0.8, 0.0)
        after_wall = grid.world_to_grid(1.2, 0.0)

        self.assertIsNotNone(wall_cell)
        self.assertIsNotNone(after_wall)
        grid.mark_occupied(*wall_cell)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[(1.6, 0.0, 0.0)],
        )

        self.assertEqual(grid.data[wall_cell[1], wall_cell[0]], OCC)
        self.assertEqual(grid.data[after_wall[1], after_wall[0]], UNKNOWN)

    def test_dynamic_door_strip_clears_only_door_area_obstacles(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        grid = mapper.floor(0)
        inside_cell = grid.world_to_grid(0.2, 0.3)
        outside_cell = grid.world_to_grid(0.2, 0.7)
        black_cell = grid.world_to_grid(-0.2, 0.0)

        self.assertIsNotNone(inside_cell)
        self.assertIsNotNone(outside_cell)
        self.assertIsNotNone(black_cell)
        grid.mark_occupied(*inside_cell)
        grid.mark_occupied(*outside_cell)
        grid.data[black_cell[1], black_cell[0]] = BLACK

        mapper.clear_dynamic_door_strip(0, 0.0, 0.0, 0.0, width=1.0, depth=0.8)

        self.assertEqual(grid.data[inside_cell[1], inside_cell[0]], FREE)
        self.assertEqual(grid.data[outside_cell[1], outside_cell[0]], OCC)
        self.assertEqual(grid.data[black_cell[1], black_cell[0]], BLACK)

    def test_free_clear_skips_sector_when_points_are_insufficient(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=True,
            free_clear_max_range=1.5,
            free_clear_angle_step=0.3,
            free_clear_obstacle_margin=0.1,
            free_clear_min_points_per_bin=2,
        )

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[self._point_from_polar(1.8, 0.12)],
        )

        grid = mapper.floor(0)
        skipped_cell = grid.world_to_grid(1.0, 0.0)

        self.assertIsNotNone(skipped_cell)
        self.assertEqual(grid.data[skipped_cell[1], skipped_cell[0]], UNKNOWN)


if __name__ == "__main__":
    unittest.main()

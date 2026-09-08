import math
import unittest
from typing import Tuple

from danger_search.planner import astar_path
from danger_search.mapper import (
    BLACK,
    FREE,
    FloorGrid,
    NEGATIVE,
    NegativeClusterRecord,
    MultiFloorGridMap,
    OCC,
    UNKNOWN,
    VISITED,
)


class MultiFloorMapperTest(unittest.TestCase):
    @staticmethod
    def _negative_record(mapper, cell, *, cluster_id="drop:1", generation=1):
        return NegativeClusterRecord(
            cluster_id=cluster_id,
            floor=0,
            frame_id="odom",
            map_epoch=1,
            localization_epoch=1,
            stamp=float(generation),
            generation=generation,
            grid_origin=(mapper.origin_x, mapper.origin_y),
            grid_resolution=mapper.resolution,
            source_cells=frozenset({cell}),
        )
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

    def test_semantic_occupied_owner_survives_free_ray_door_clear_and_epoch(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.25,
        )
        grid = mapper.floor(0)
        grid.fill_base(FREE)
        owner = "danger:7"
        grid.replace_semantic_occupied_radius(
            0.0,
            0.0,
            0.40,
            owner=owner,
        )
        center = grid.world_to_grid(0.0, 0.0)
        self.assertIsNotNone(center)
        self.assertEqual(int(grid.data[center[1], center[0]]), OCC)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(-1.0, 0.0, 0.3),
            points=[(1.0, 0.0, 0.0)],
            cloud_generation=2,
            map_epoch=1,
            localization_epoch=1,
        )
        mapper.clear_dynamic_door_strip(0, 0.0, 0.0, 0.0, 1.2, 0.5)
        grid.activate_evidence_scope(map_epoch=2, localization_epoch=1)

        self.assertEqual(int(grid.data[center[1], center[0]]), OCC)
        self.assertIn(owner, grid.semantic_occupied_owners[center])
        self.assertEqual(grid.get_constraint_kind(*center), "danger_obstacle")

    @staticmethod
    def _trail_protection_mapper(**overrides):
        params = {
            "resolution": 0.1,
            "width": 10.0,
            "length": 6.0,
            "origin_x": -5.0,
            "origin_y": -3.0,
            "visited_radius": 0.0,
            "obstacle_inflation_radius": 0.2,
            "free_clear_enabled": False,
            "evidence_lifecycle_enabled": True,
            "trail_protection_enabled": True,
            "trail_protection_radius": 0.25,
            "trail_protection_near_range": 2.0,
            "trail_protection_confirm_generations": 3,
            "trail_protection_confirm_positions": 2,
            "counter_pose_separation": 0.15,
        }
        params.update(overrides)
        return MultiFloorGridMap(**params)

    def test_far_obstacle_cannot_block_protected_trail_or_its_inflation(self):
        mapper = self._trail_protection_mapper()
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        trail_cell = grid.world_to_grid(0.0, 0.0)
        inflated_trail_cell = grid.world_to_grid(0.15, 0.0)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(3.0, 0.0, 0.6),
            points=[(0.0, 0.0, 0.6)],
            cloud_generation=2,
        )

        self.assertEqual(int(grid.data[trail_cell[1], trail_cell[0]]), VISITED)
        self.assertEqual(
            int(grid.data[inflated_trail_cell[1], inflated_trail_cell[0]]),
            VISITED,
        )
        self.assertIn(trail_cell, grid.occ_cores)

    def test_disabled_trail_protection_preserves_original_obstacle_priority(self):
        mapper = self._trail_protection_mapper(trail_protection_enabled=False)
        grid = mapper.floor(0)
        trail_cell = grid.world_to_grid(0.0, 0.0)
        grid.set_base_state(*trail_cell, VISITED)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(3.0, 0.0, 0.6),
            points=[(0.0, 0.0, 0.6)],
            cloud_generation=1,
        )

        self.assertEqual(int(grid.data[trail_cell[1], trail_cell[0]]), OCC)
        self.assertFalse(grid.protected_trail_cells)

    def test_far_hidden_obstacle_still_occludes_same_frame_free_ray(self):
        mapper = self._trail_protection_mapper(
            obstacle_inflation_radius=0.0,
            point_free_clear_max_range=5.0,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        hidden_obstacle = grid.world_to_grid(0.0, 0.0)
        behind = grid.world_to_grid(-0.5, 0.0)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(3.0, 0.0, 0.6),
            points=[(0.0, 0.0, 0.6), (-1.0, 0.0, 0.0)],
            cloud_generation=2,
        )

        self.assertEqual(
            int(grid.data[hidden_obstacle[1], hidden_obstacle[0]]), VISITED
        )
        self.assertEqual(int(grid.data[behind[1], behind[0]]), UNKNOWN)

    def test_two_meter_hit_is_near_but_over_two_meter_hit_is_not(self):
        mapper = self._trail_protection_mapper(
            obstacle_inflation_radius=0.0,
            trail_protection_confirm_generations=1,
            trail_protection_confirm_positions=1,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        protected = grid.world_to_grid(0.0, 0.0)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(2.0, 0.0, 0.6),
            points=[(0.0, 0.0, 0.6)],
            cloud_generation=2,
        )
        self.assertEqual(int(grid.data[protected[1], protected[0]]), OCC)

        mapper = self._trail_protection_mapper(
            obstacle_inflation_radius=0.0,
            trail_protection_confirm_generations=1,
            trail_protection_confirm_positions=1,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        protected = grid.world_to_grid(0.0, 0.0)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(2.01, 0.0, 0.6),
            points=[(0.0, 0.0, 0.6)],
            cloud_generation=2,
        )
        self.assertEqual(int(grid.data[protected[1], protected[0]]), VISITED)

    def test_near_obstacle_requires_three_generations_and_two_positions(self):
        mapper = self._trail_protection_mapper(obstacle_inflation_radius=0.0)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        protected = grid.world_to_grid(0.0, 0.0)

        for generation in (2, 3, 4):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(1.8, 0.0, 0.6),
                points=[(0.0, 0.0, 0.6)],
                cloud_generation=generation,
            )
        self.assertEqual(int(grid.data[protected[1], protected[0]]), VISITED)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(1.78, 0.20, 0.6),
            points=[(0.0, 0.0, 0.6)],
            cloud_generation=5,
        )
        self.assertEqual(int(grid.data[protected[1], protected[0]]), OCC)

    def test_confirmed_protected_obstacle_stays_latched_across_hit_window(self):
        mapper = self._trail_protection_mapper(obstacle_inflation_radius=0.0)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        protected = grid.world_to_grid(0.0, 0.0)
        for generation, robot_xy in (
            (2, (1.8, 0.0)),
            (3, (1.8, 0.0)),
            (4, (1.78, 0.2)),
        ):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(*robot_xy, 0.6),
                points=[(0.0, 0.0, 0.6)],
                cloud_generation=generation,
            )
        self.assertEqual(int(grid.data[protected[1], protected[0]]), OCC)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(1.8, 0.0, 0.6),
            points=[(0.0, 0.0, 0.6)],
            cloud_generation=100,
        )

        self.assertTrue(grid.occ_cores[protected].protected_trail_confirmed)
        self.assertEqual(int(grid.data[protected[1], protected[0]]), OCC)

    def test_unconfirmed_protected_pose_samples_expire_with_hit_window(self):
        mapper = self._trail_protection_mapper(obstacle_inflation_radius=0.0)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        protected = grid.world_to_grid(0.0, 0.0)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(1.78, 0.2, 0.6),
            points=[(0.0, 0.0, 0.6)],
            cloud_generation=2,
        )
        for generation in (100, 101, 102):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(1.8, 0.0, 0.6),
                points=[(0.0, 0.0, 0.6)],
                cloud_generation=generation,
            )

        self.assertFalse(grid.occ_cores[protected].protected_trail_confirmed)
        self.assertEqual(int(grid.data[protected[1], protected[0]]), VISITED)

    def test_black_and_negative_override_protected_trail(self):
        mapper = self._trail_protection_mapper(obstacle_inflation_radius=0.0)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        black_cell = grid.world_to_grid(0.0, 0.0)
        negative_cell = grid.world_to_grid(0.2, 0.0)

        grid.mark_black(*black_cell, owner="test:trail")
        commit = mapper.persist_negative_clusters_atomic(
            (self._negative_record(mapper, negative_cell),)
        )

        self.assertTrue(commit.committed)
        self.assertEqual(int(grid.data[black_cell[1], black_cell[0]]), BLACK)
        self.assertEqual(
            int(grid.data[negative_cell[1], negative_cell[0]]), NEGATIVE
        )

    def test_counterevidence_revokes_protected_trail_obstacle_confirmation(self):
        mapper = self._trail_protection_mapper(
            obstacle_inflation_radius=0.0,
            point_free_clear_max_range=4.0,
            trail_protection_confirm_generations=2,
            trail_protection_confirm_positions=2,
            low_candidate_counter_generations=2,
            low_candidate_counter_positions=2,
            low_confirmed_counter_generations=2,
            low_confirmed_counter_positions=2,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        protected = grid.world_to_grid(0.0, 0.0)
        for generation, robot_xy in ((2, (1.8, 0.0)), (3, (1.78, 0.2))):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(*robot_xy, 0.6),
                points=[(0.0, 0.0, 0.6)],
                cloud_generation=generation,
            )
        self.assertEqual(int(grid.data[protected[1], protected[0]]), OCC)

        counter_observations = (
            (4, (1.8, 0.0), (-1.0, 0.0, 0.0)),
            (5, (1.8, 0.2), (-1.0, -0.05, 0.0)),
        )
        for generation, robot_xy, free_point in counter_observations:
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(*robot_xy, 0.6),
                points=[free_point],
                cloud_generation=generation,
            )

        self.assertIn(protected, grid.occ_cores)
        self.assertEqual(int(grid.data[protected[1], protected[0]]), VISITED)

    def test_reobserved_obstacle_resets_protected_counterevidence(self):
        mapper = self._trail_protection_mapper(
            obstacle_inflation_radius=0.0,
            point_free_clear_max_range=4.0,
            trail_protection_confirm_generations=2,
            trail_protection_confirm_positions=2,
            low_candidate_counter_generations=2,
            low_candidate_counter_positions=2,
            low_confirmed_counter_generations=2,
            low_confirmed_counter_positions=2,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        protected = grid.world_to_grid(0.0, 0.0)
        for generation, robot_xy in ((2, (1.8, 0.0)), (3, (1.78, 0.2))):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(*robot_xy, 0.6),
                points=[(0.0, 0.0, 0.6)],
                cloud_generation=generation,
            )
        self.assertEqual(int(grid.data[protected[1], protected[0]]), OCC)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(1.8, 0.0, 0.6),
            points=[(-1.0, 0.0, 0.0)],
            cloud_generation=4,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(1.8, 0.2, 0.6),
            points=[(0.0, 0.0, 0.6)],
            cloud_generation=5,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(1.8, 0.4, 0.6),
            points=[(-1.0, -0.1, 0.0)],
            cloud_generation=6,
        )

        self.assertEqual(int(grid.data[protected[1], protected[0]]), OCC)

    def test_dynamic_door_clear_removes_confirmed_protected_obstacle(self):
        mapper = self._trail_protection_mapper(
            obstacle_inflation_radius=0.0,
            trail_protection_confirm_generations=2,
            trail_protection_confirm_positions=2,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        grid = mapper.floor(0)
        protected = grid.world_to_grid(0.0, 0.0)
        for generation, robot_xy in ((2, (1.8, 0.0)), (3, (1.78, 0.2))):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(*robot_xy, 0.6),
                points=[(0.0, 0.0, 0.6)],
                cloud_generation=generation,
            )
        self.assertEqual(int(grid.data[protected[1], protected[0]]), OCC)

        mapper.clear_dynamic_door_strip(
            0, 0.0, 0.0, 0.0, width=0.4, depth=0.4
        )

        self.assertNotIn(protected, grid.occ_cores)
        self.assertIn(protected, grid.protected_trail_cells)
        self.assertEqual(int(grid.data[protected[1], protected[0]]), VISITED)

    def test_trail_protection_is_per_floor_and_survives_copy(self):
        mapper = self._trail_protection_mapper()
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        mapper.update_from_cloud(
            floor_index=1,
            robot_xyz=(1.0, 0.0, 3.2),
            points=[],
            cloud_generation=2,
        )
        floor_zero = mapper.floor(0)
        floor_one = mapper.floor(1)
        zero_cell = floor_zero.world_to_grid(0.0, 0.0)
        one_cell = floor_one.world_to_grid(1.0, 0.0)
        copied = floor_zero.copy()

        self.assertIn(zero_cell, floor_zero.protected_trail_cells)
        self.assertNotIn(one_cell, floor_zero.protected_trail_cells)
        self.assertIn(one_cell, floor_one.protected_trail_cells)
        self.assertEqual(copied.protected_trail_cells, floor_zero.protected_trail_cells)
        self.assertIsNot(copied.protected_trail_cells, floor_zero.protected_trail_cells)

    def test_trail_protection_fills_between_consecutive_poses(self):
        mapper = self._trail_protection_mapper(trail_protection_radius=0.25)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.4, 0.0, 0.6),
            points=[],
            cloud_generation=2,
        )
        grid = mapper.floor(0)

        for x in (0.0, 0.1, 0.2, 0.3, 0.4):
            cell = grid.world_to_grid(x, 0.0)
            self.assertIn(cell, grid.protected_trail_cells)
            self.assertEqual(int(grid.data[cell[1], cell[0]]), VISITED)

    def test_trail_protection_does_not_connect_large_pose_jump(self):
        mapper = self._trail_protection_mapper(trail_protection_radius=0.25)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
            cloud_generation=1,
        )
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(2.0, 0.0, 0.6),
            points=[],
            cloud_generation=2,
        )
        grid = mapper.floor(0)
        midpoint = grid.world_to_grid(1.0, 0.0)

        self.assertNotIn(midpoint, grid.protected_trail_cells)
        self.assertEqual(int(grid.data[midpoint[1], midpoint[0]]), UNKNOWN)

    def test_low_candidate_does_not_block_visited_until_distinct_pose_confirmation(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            visited_radius=0.0,
            obstacle_inflation_radius=0.15,
            free_clear_enabled=False,
            evidence_lifecycle_enabled=True,
            low_confirm_generations=3,
            low_confirm_window_generations=10,
            counter_pose_separation=0.15,
        )
        grid = mapper.floor(0)
        core = grid.world_to_grid(0.5, 0.0)
        assert core is not None
        grid.set_base_state(*core, VISITED)

        for generation, robot_x in ((1, -0.4), (2, -0.4)):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(robot_x, 0.0, 0.4),
                points=[(0.5, 0.0, 0.18)],
                cloud_generation=generation,
                map_epoch=1,
                localization_epoch=1,
            )
        self.assertEqual(grid.occ_cores[core].kind, "LOW_CANDIDATE")
        self.assertEqual(int(grid.data[core[1], core[0]]), VISITED)

        for generation, robot_x in ((3, -0.4),):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(robot_x, 0.0, 0.4),
                points=[(0.5, 0.0, 0.18)],
                cloud_generation=generation,
                map_epoch=1,
                localization_epoch=1,
            )
        self.assertEqual(grid.occ_cores[core].kind, "LOW_CONFIRMED")
        self.assertEqual(int(grid.data[core[1], core[0]]), OCC)

    def test_strict_counterevidence_removes_low_core_and_restores_base(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            visited_radius=0.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=False,
            evidence_lifecycle_enabled=True,
            low_confirm_generations=3,
            low_confirm_window_generations=10,
            low_candidate_counter_generations=3,
            low_candidate_counter_positions=2,
            counter_pose_separation=0.15,
        )
        grid = mapper.floor(0)
        core = grid.world_to_grid(0.5, 0.0)
        assert core is not None
        grid.set_base_state(*core, VISITED)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(-0.4, 0.0, 0.4),
            points=[(0.5, 0.0, 0.18)],
            cloud_generation=1,
            map_epoch=1,
            localization_epoch=1,
        )
        for generation, robot_x in ((2, -0.3), (3, -0.1), (4, 0.1)):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(robot_x, 0.0, 0.4),
                points=[(1.0, 0.0, 0.0)],
                cloud_generation=generation,
                map_epoch=1,
                localization_epoch=1,
            )
        self.assertNotIn(core, grid.occ_cores)
        self.assertEqual(int(grid.data[core[1], core[0]]), VISITED)

    def test_overlapping_inflation_survives_one_core_removal(self):
        grid = FloorGrid(0, 0.1, 4.0, 4.0, -2.0, -2.0)
        first = grid.world_to_grid(0.0, 0.0)
        second = grid.world_to_grid(0.1, 0.0)
        assert first is not None and second is not None
        for generation, owner in enumerate((first, second), start=1):
            grid.record_obstacle_observation(
                *grid.grid_to_world(*owner),
                0.2,
                kind="HIGH_HARD",
                cloud_generation=generation,
                robot_xy=(-0.5, 0.0),
                map_epoch=1,
                localization_epoch=1,
                low_confirm_generations=3,
                low_confirm_window_generations=10,
                pose_separation=0.15,
            )
        overlap = next(
            iter(
                grid.occ_cores[first].inflated_cells
                & grid.occ_cores[second].inflated_cells
            )
        )
        self.assertEqual(grid.inflation_owners[overlap], {first, second})
        self.assertTrue(grid.remove_obstacle_core(first))
        self.assertEqual(grid.inflation_owners[overlap], {second})
        self.assertEqual(int(grid.data[overlap[1], overlap[0]]), OCC)

    def test_visited_mark_does_not_erase_black_safety_cell(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=2.0,
            length=2.0,
            origin_x=-1.0,
            origin_y=-1.0,
        )
        grid = mapper.floor(0)
        cell = grid.world_to_grid(0.0, 0.0)
        self.assertIsNotNone(cell)
        center = grid.grid_to_world(*cell)
        grid.mark_black(*cell, owner="test:safety")

        grid.mark_visited_radius(center[0], center[1], 0.05)

        self.assertEqual(grid.data[cell[1], cell[0]], BLACK)

    def test_invalid_geometry_is_rejected_explicitly(self):
        with self.assertRaises(ValueError):
            MultiFloorGridMap(resolution=0.0)
        with self.assertRaises(ValueError):
            MultiFloorGridMap(floor_height=math.nan)
        with self.assertRaises(ValueError):
            MultiFloorGridMap(point_free_clear_max_range=math.nan)

    def test_non_finite_cloud_point_is_ignored(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=3.0,
            length=3.0,
            origin_x=-1.5,
            origin_y=-1.5,
            obstacle_inflation_radius=0.0,
        )

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.3),
            points=[(math.nan, 0.0, 0.6), (0.5, 0.0, 0.6)],
        )

        cell = mapper.floor(0).world_to_grid(0.5, 0.0)
        self.assertIsNotNone(cell)
        self.assertEqual(mapper.floor(0).data[cell[1], cell[0]], OCC)

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
        grid.set_base_state(1, 1, FREE)
        grid.set_base_state(2, 1, VISITED)
        grid.mark_occupied(4, 3)

        self.assertAlmostEqual(grid.known_area(), 0.75)
        self.assertAlmostEqual(grid.visited_area(), 0.25)
        self.assertEqual(grid.known_bbox_extent(), (2.0, 1.5))
        self.assertAlmostEqual(grid.known_bbox_fill_ratio(), 3.0 / 12.0)
        self.assertAlmostEqual(grid.traversable_known_ratio(), 2.0 / 3.0)
        self.assertAlmostEqual(grid.coverage_ratio(), 3.0 / 12.0)
        self.assertAlmostEqual(grid.visited_traversable_ratio(), 1.0 / 2.0)

    def test_debug_snapshot_records_ray_stop_and_nearest_obstacle(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=False,
        )

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[
                (0.5, 0.0, 0.6),
                (1.0, 0.0, 0.0),
            ],
        )

        snapshot = mapper.latest_debug_snapshot()
        assert snapshot is not None
        self.assertEqual(snapshot.point_ray_count, 2)
        self.assertEqual(snapshot.sector_ray_count, 0)
        self.assertEqual(snapshot.blocked_stop_count, 1)
        self.assertGreater(snapshot.suspicious_clearing_prevented_count, 0)
        self.assertAlmostEqual(snapshot.nearest_obstacle_distance, 0.5)
        self.assertGreater(snapshot.max_clearing_length, 0.0)
        self.assertEqual(snapshot.occupied_to_free_count, 0)

    def test_debug_snapshot_counts_capped_no_obstacle_sector(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=10.0,
            length=10.0,
            origin_x=-5.0,
            origin_y=-5.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=True,
            free_clear_max_range=8.0,
            free_clear_angle_step=0.3,
            free_clear_min_points_per_bin=2,
        )

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[
                self._point_from_polar(2.0, -0.1, z=0.0),
                self._point_from_polar(2.0, 0.1, z=0.0),
            ],
        )

        snapshot = mapper.latest_debug_snapshot()
        assert snapshot is not None
        self.assertEqual(snapshot.point_ray_count, 2)
        self.assertEqual(snapshot.sector_ray_count, 1)
        self.assertIsNone(snapshot.nearest_obstacle_distance)
        self.assertGreaterEqual(snapshot.suspicious_clearing_prevented_count, 1)

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

    def test_same_frame_obstacle_blocks_earlier_miss_ray(self):
        """同帧 miss 即使排在墙命中前，也不能先把墙后清成 free。"""
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=False,
            point_free_clear_max_range=1.5,
        )
        grid = mapper.floor(0)
        wall_cell = grid.world_to_grid(0.8, 0.0)
        behind_wall = grid.world_to_grid(1.2, 0.0)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[
                (1.4, 0.0, 0.0),
                (0.8, 0.0, 0.6),
            ],
        )

        self.assertEqual(grid.data[wall_cell[1], wall_cell[0]], OCC)
        self.assertEqual(grid.data[behind_wall[1], behind_wall[0]], UNKNOWN)

    def test_non_obstacle_point_ray_is_capped_to_near_field(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=False,
            point_free_clear_max_range=1.5,
        )
        grid = mapper.floor(0)
        far_cell = grid.world_to_grid(2.0, 0.0)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[(3.0, 0.0, 0.0)],
        )

        self.assertEqual(grid.data[far_cell[1], far_cell[0]], UNKNOWN)
        snapshot = mapper.latest_debug_snapshot()
        assert snapshot is not None
        self.assertGreater(snapshot.suspicious_clearing_prevented_count, 0)

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
        grid.mark_black(*black_cell, owner="test:door_safety")

        mapper.clear_dynamic_door_strip(0, 0.0, 0.0, 0.0, width=1.0, depth=0.8)

        self.assertEqual(grid.data[inside_cell[1], inside_cell[0]], FREE)
        self.assertEqual(grid.data[outside_cell[1], outside_cell[0]], OCC)
        self.assertEqual(grid.data[black_cell[1], black_cell[0]], BLACK)

        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[],
        )
        snapshot = mapper.latest_debug_snapshot()
        assert snapshot is not None
        self.assertEqual(snapshot.dynamic_door_cleared_count, 1)
        self.assertEqual(snapshot.occupied_to_free_count, 1)

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

    def test_lower_return_does_not_create_free_ray_or_visible_sector(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            visited_radius=0.0,
            free_clear_enabled=True,
            free_clear_min_points_per_bin=1,
        )

        mapper.update_from_cloud(
            floor_index=1,
            robot_xyz=(0.0, 0.0, 2.9),
            points=[(1.0, 0.0, 0.0)],
        )

        grid = mapper.floor(1)
        middle = grid.world_to_grid(0.5, 0.0)
        target = grid.world_to_grid(1.0, 0.0)
        self.assertEqual(grid.data[middle[1], middle[0]], UNKNOWN)
        self.assertEqual(grid.data[target[1], target[0]], UNKNOWN)
        self.assertEqual(mapper.latest_debug_snapshot().point_ray_count, 0)

    def test_dynamic_negative_is_blocked_and_only_direct_support_clears_it(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            visited_radius=0.0,
        )
        grid = mapper.floor(0)
        cell = grid.world_to_grid(0.5, 0.0)
        grid.mark_free(*cell)

        commit = mapper.persist_negative_clusters_atomic(
            (self._negative_record(mapper, cell),)
        )
        self.assertTrue(commit.committed)
        self.assertEqual(grid.data[cell[1], cell[0]], NEGATIVE)
        self.assertFalse(grid.is_traversable(*cell))
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.3),
            points=[(1.0, 0.0, 0.0)],
        )
        self.assertEqual(grid.data[cell[1], cell[0]], NEGATIVE)
        self.assertEqual(
            mapper.clear_negative_clusters_with_support(
                floor=0, cells=[cell], map_epoch=1,
                localization_epoch=1, generation=2,
                grid_origin=(mapper.origin_x, mapper.origin_y),
                grid_resolution=mapper.resolution,
            ),
            ("drop:1",),
        )
        self.assertEqual(grid.data[cell[1], cell[0]], FREE)

    def test_negative_cluster_batch_rejects_mixed_scope_without_partial_commit(self):
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        cells = (mapper.floor(0).world_to_grid(0.2, 0.0),
                 mapper.floor(0).world_to_grid(0.4, 0.0))
        first = self._negative_record(mapper, cells[0], cluster_id="drop:a")
        second = NegativeClusterRecord(
            **{**first.__dict__, "cluster_id": "drop:b", "map_epoch": 2,
               "source_cells": frozenset({cells[1]})}
        )

        commit = mapper.persist_negative_clusters_atomic((first, second))

        self.assertFalse(commit.committed)
        self.assertEqual(commit.reason_code, "drop_cluster_scope_invalid")
        self.assertEqual(mapper.floor(0).negative_cluster_records, {})
        self.assertFalse(mapper.floor(0).negative_cell_owners)

    def test_negative_cluster_rejects_non_integer_cells_and_nonfinite_geometry(self):
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        base = self._negative_record(mapper, (20, 20))
        malformed_cell = NegativeClusterRecord(
            **{**base.__dict__, "source_cells": frozenset({(20.5, 20)})}
        )
        malformed_geometry = NegativeClusterRecord(
            **{**base.__dict__, "cluster_id": "drop:nan",
               "grid_resolution": math.nan}
        )
        malformed_floor = NegativeClusterRecord(
            **{**base.__dict__, "cluster_id": "drop:floor", "floor": True}
        )
        malformed_cluster_id = NegativeClusterRecord(
            **{**base.__dict__, "cluster_id": 7}
        )
        floors_before = dict(mapper._floors)

        self.assertFalse(
            mapper.persist_negative_clusters_atomic((malformed_cell,)).committed
        )
        self.assertFalse(
            mapper.persist_negative_clusters_atomic((malformed_geometry,)).committed
        )
        self.assertFalse(
            mapper.persist_negative_clusters_atomic((malformed_floor,)).committed
        )
        self.assertFalse(
            mapper.persist_negative_clusters_atomic(
                (malformed_cluster_id,)
            ).committed
        )
        self.assertEqual(mapper._floors, floors_before)
        self.assertEqual(mapper.floor(0).negative_cluster_records, {})

    def test_new_floor_bad_geometry_or_bounds_does_not_create_floor(self):
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        bad_geometry = NegativeClusterRecord(
            cluster_id="drop:bad-geometry", floor=7, frame_id="odom",
            map_epoch=1, localization_epoch=1, stamp=1.0, generation=1,
            grid_origin=(99.0, 99.0), grid_resolution=0.1,
            source_cells=frozenset({(1, 1)}),
        )
        out_of_bounds = NegativeClusterRecord(
            cluster_id="drop:out-of-bounds", floor=8, frame_id="odom",
            map_epoch=1, localization_epoch=1, stamp=1.0, generation=1,
            grid_origin=(-2.0, -2.0), grid_resolution=0.1,
            source_cells=frozenset({(999, 1)}),
        )

        self.assertFalse(
            mapper.persist_negative_clusters_atomic((bad_geometry,)).committed
        )
        self.assertFalse(
            mapper.persist_negative_clusters_atomic((out_of_bounds,)).committed
        )
        self.assertEqual(mapper._floors, {})

    def test_confirmation_for_unknown_floor_does_not_create_floor(self):
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )

        removed = mapper.clear_negative_clusters_with_support(
            floor=9, cells=[(1, 1)], map_epoch=1,
            localization_epoch=1, generation=2,
            grid_origin=(-2.0, -2.0), grid_resolution=0.1,
        )

        self.assertEqual(removed, ())
        self.assertEqual(mapper._floors, {})

    def test_idempotent_cluster_batch_does_not_advance_revision(self):
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        cell = mapper.floor(0).world_to_grid(0.2, 0.0)
        record = self._negative_record(mapper, cell)
        first = mapper.persist_negative_clusters_atomic((record,))
        replay = mapper.persist_negative_clusters_atomic((record,))

        self.assertTrue(first.committed)
        self.assertTrue(replay.committed)
        self.assertEqual(replay.reason_code, "already_committed")
        self.assertEqual(replay.revision, first.revision)

    def test_negative_cluster_support_accumulates_across_newer_batches(self):
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        grid = mapper.floor(0)
        first_cell = grid.world_to_grid(0.2, 0.0)
        cells = frozenset({first_cell, (first_cell[0] + 1, first_cell[1])})
        record = NegativeClusterRecord(
            **{**self._negative_record(mapper, next(iter(cells))).__dict__,
               "source_cells": cells, "generation": 10}
        )
        mapper.persist_negative_clusters_atomic((record,))

        self.assertEqual(
            mapper.clear_negative_clusters_with_support(
                floor=0, cells=[next(iter(cells))], map_epoch=1,
                localization_epoch=1, generation=11,
                grid_origin=(mapper.origin_x, mapper.origin_y),
                grid_resolution=mapper.resolution,
            ),
            (),
        )
        remaining = cells - {next(iter(cells))}
        self.assertEqual(
            mapper.clear_negative_clusters_with_support(
                floor=0, cells=remaining, map_epoch=1,
                localization_epoch=1, generation=12,
                grid_origin=(mapper.origin_x, mapper.origin_y),
                grid_resolution=mapper.resolution,
            ),
            ("drop:1",),
        )

    def test_negative_underlay_is_removed_when_occ_hides_revoked_cluster(self):
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        grid = mapper.floor(0)
        cell = grid.world_to_grid(0.2, 0.0)
        mapper.persist_negative_clusters_atomic((self._negative_record(mapper, cell),))
        grid.mark_occupied(*cell)

        mapper.clear_negative_clusters_with_support(
            floor=0, cells=[cell], map_epoch=1,
            localization_epoch=1, generation=2,
            grid_origin=(mapper.origin_x, mapper.origin_y),
            grid_resolution=mapper.resolution,
        )

        self.assertEqual(grid.data[cell[1], cell[0]], OCC)
        self.assertNotIn(cell, grid.negative_first_state)

    def test_stale_or_wrong_geometry_support_does_not_accumulate(self):
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        grid = mapper.floor(0)
        first = grid.world_to_grid(0.2, 0.0)
        second = (first[0] + 1, first[1])
        record = NegativeClusterRecord(
            **{
                **self._negative_record(mapper, first).__dict__,
                "generation": 10,
                "source_cells": frozenset({first, second}),
            }
        )
        mapper.persist_negative_clusters_atomic((record,))

        self.assertEqual(mapper.clear_negative_clusters_with_support(
            floor=0, cells=[first], map_epoch=1, localization_epoch=1,
            generation=10, grid_origin=(mapper.origin_x, mapper.origin_y),
            grid_resolution=mapper.resolution,
        ), ())
        self.assertEqual(mapper.clear_negative_clusters_with_support(
            floor=0, cells=[first], map_epoch=1, localization_epoch=1,
            generation=11, grid_origin=(99.0, 99.0),
            grid_resolution=mapper.resolution,
        ), ())
        self.assertEqual(mapper.clear_negative_clusters_with_support(
            floor=0, cells=[second], map_epoch=1, localization_epoch=1,
            generation=11, grid_origin=(mapper.origin_x, mapper.origin_y),
            grid_resolution=mapper.resolution,
        ), ())
        self.assertIn("drop:1", grid.negative_cluster_records)
        self.assertEqual(mapper.clear_negative_clusters_with_support(
            floor=0, cells=[first], map_epoch=1, localization_epoch=1,
            generation=12, grid_origin=(mapper.origin_x, mapper.origin_y),
            grid_resolution=mapper.resolution,
        ), ("drop:1",))

    def test_direct_support_never_clears_black_or_occupied_priority(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        grid = mapper.floor(0)
        black = grid.world_to_grid(0.2, 0.0)
        occupied = grid.world_to_grid(0.4, 0.0)
        mapper.persist_negative_clusters_atomic((
            self._negative_record(mapper, black, cluster_id="drop:black"),
            self._negative_record(mapper, occupied, cluster_id="drop:occ"),
        ))
        grid.mark_black(*black, owner="test:direct_support")
        grid.mark_occupied(*occupied)

        mapper.clear_negative_clusters_with_support(
            floor=0, cells=[black, occupied], map_epoch=1,
            localization_epoch=1, generation=2,
            grid_origin=(mapper.origin_x, mapper.origin_y),
            grid_resolution=mapper.resolution,
        )

        self.assertEqual(grid.data[black[1], black[0]], BLACK)
        self.assertEqual(grid.data[occupied[1], occupied[0]], OCC)

    def test_open_door_restores_negative_source_hidden_by_door_obstacle(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        grid = mapper.floor(0)
        cell = grid.world_to_grid(0.2, 0.0)
        mapper.persist_negative_clusters_atomic((
            self._negative_record(mapper, cell),
        ))
        grid.mark_occupied(*cell)

        mapper.clear_dynamic_door_strip(
            0, 0.0, 0.0, 0.0, width=1.0, depth=0.8
        )

        self.assertEqual(grid.data[cell[1], cell[0]], NEGATIVE)

    def test_epoch_change_discards_old_obstacle_and_drop_evidence(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        grid = mapper.floor(0)
        grid.fill_base(FREE)
        obstacle = grid.world_to_grid(0.2, 0.0)
        negative = grid.world_to_grid(-0.2, 0.0)
        grid.mark_occupied(*obstacle)
        mapper.persist_negative_clusters_atomic((
            self._negative_record(
                mapper, negative, cluster_id="drop:old"
            ),
        ))

        grid.activate_evidence_scope(map_epoch=2, localization_epoch=1)

        self.assertEqual(grid.data[obstacle[1], obstacle[0]], FREE)
        self.assertEqual(grid.data[negative[1], negative[0]], FREE)
        self.assertEqual(grid.occ_cores, {})
        self.assertEqual(grid.negative_cluster_records, {})

    def test_high_hit_beyond_clear_range_is_low_candidate_and_within_is_high_hard(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=30.0,
            length=30.0,
            origin_x=-15.0,
            origin_y=-15.0,
            point_free_clear_max_range=8.0,
            raycast_max_range=15.0,
            obstacle_inflation_radius=0.0,
        )
        grid = mapper.floor(0)
        # 近距 (5m <= 8m): 高回波 (Z_rel=0.6m) 记为 HIGH_HARD
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[(5.0, 0.0, 0.6)],
            cloud_generation=1,
        )
        near_cell = grid.world_to_grid(5.0, 0.0)
        self.assertEqual(grid.occ_cores[near_cell].kind, "HIGH_HARD")
        self.assertEqual(grid.data[near_cell[1], near_cell[0]], OCC)

        # 远距 (10m > 8m): 高回波 (Z_rel=0.6m) 记为 LOW_CANDIDATE
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.6),
            points=[(10.0, 0.0, 0.6)],
            cloud_generation=2,
        )
        far_cell = grid.world_to_grid(10.0, 0.0)
        self.assertEqual(grid.occ_cores[far_cell].kind, "LOW_CANDIDATE")

    def test_planar_transform_moves_observation_and_obstacle_evidence(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
            obstacle_inflation_radius=0.0,
        )
        grid = mapper.floor(0)
        grid.fill_base(FREE)
        source = grid.world_to_grid(0.5, 0.0)
        grid.mark_occupied(*source)

        mapper.transform_planar(0.0, 0.0, math.pi / 2.0)

        target = grid.world_to_grid(-0.05, 0.55)
        self.assertEqual(grid.data[target[1], target[0]], OCC)
        self.assertIn(target, grid.occ_cores)

    def test_counter_ray_3d_height_gate_protects_low_core_from_high_rays(self):
        """高空光束严禁反证撤销地表低矮核心；只有贴近地面的反证射线才生效。"""
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
            obstacle_inflation_radius=0.0,
            free_clear_enabled=False,
            evidence_lifecycle_enabled=True,
            low_height_max_z=0.26,
            obstacle_min_z=0.20,
            low_candidate_counter_generations=2,
            low_candidate_counter_positions=2,
            counter_pose_separation=0.15,
        )
        grid = mapper.floor(0)
        core_cell = grid.world_to_grid(1.0, 0.0)
        assert core_cell is not None

        # 1. 建立地表矮障碍核心 (Z_rel = 0.22m < 0.26m)
        mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(0.0, 0.0, 0.22),
            points=[(1.0, 0.0, 0.22)],
            cloud_generation=1,
            map_epoch=1,
            localization_epoch=1,
        )
        self.assertIn(core_cell, grid.occ_cores)
        self.assertEqual(grid.occ_cores[core_cell].kind, "LOW_CANDIDATE")

        # 2. 高空射线 (Z=1.2m > 0.26m) 穿过该栅格，严禁撤销地表低矮核心
        for gen, rx in ((2, -0.2), (3, 0.2)):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(rx, 0.0, 1.2),
                points=[(2.0, 0.0, 1.5)],  # 高处非障碍点 (1.5m > obstacle_max_z 1.4m)
                cloud_generation=gen,
                map_epoch=1,
                localization_epoch=1,
            )
        self.assertIn(core_cell, grid.occ_cores)

        # 3. 贴近地表射线 (Z=0.22m <= 0.26m) 穿过该栅格，正常反证并撤销
        for gen, rx in ((4, -0.2), (5, 0.2)):
            mapper.update_from_cloud(
                floor_index=0,
                robot_xyz=(rx, 0.0, 0.22),
                points=[(2.0, 0.0, 0.10)],  # 低于 obstacle_min_z 0.20m 的自由点
                cloud_generation=gen,
                map_epoch=1,
                localization_epoch=1,
            )
        self.assertNotIn(core_cell, grid.occ_cores)


if __name__ == "__main__":
    unittest.main()


import math
import unittest
from danger_search.elevator_crossing_runtime import (
    DONE,
    FAILED,
    RUNNING,
    ElevatorCrossingConfig,
    ElevatorCrossingRuntime,
    crossing_corridor_obstacle,
)
from danger_search.local_semantic_detector import LocalSemanticDetector


def _opening_points(center=(0.57, 0.8), yaw=math.pi / 2.0, width=0.90):
    heading = (math.cos(yaw), math.sin(yaw))
    lateral = (-heading[1], heading[0])
    points = []
    for side in (-0.5 * width, 0.5 * width):
        for height in (0.20, 0.50, 0.80, 1.10, 1.40):
            points.append(
                (
                    center[0] + side * lateral[0],
                    center[1] + side * lateral[1],
                    height,
                )
            )
    return points


class ElevatorCrossingRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.runtime = ElevatorCrossingRuntime()
        self.detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        self.config = ElevatorCrossingConfig()

    def observe(
        self,
        now,
        stamp,
        points=None,
        mode="enter",
        old_door_center=(0.0, 0.8),
    ):
        return self.runtime.observe(
            now=now,
            door_id="lift_0",
            mode=mode,
            expected_crossing_yaw=math.pi / 2.0,
            old_door_center=old_door_center,
            pose=(0.57, 0.0, math.pi / 2.0),
            cloud_stamp=stamp,
            cloud_points=_opening_points() if points is None else points,
            floor_index=0,
            floor_height=2.6,
            local_detector=self.detector,
            config=self.config,
        )

    def test_requires_three_distinct_consistent_cloud_generations(self):
        first = self.observe(10.0, 1.0)
        repeated = self.observe(10.1, 1.0)
        second = self.observe(10.2, 2.0)
        third = self.observe(10.4, 3.0)

        self.assertEqual((first.status, repeated.status, second.status), (RUNNING,) * 3)
        self.assertEqual(third.status, DONE)
        self.assertEqual(third.geometry.generation_count, 3)

    def test_current_opening_is_used_even_when_old_center_shift_is_057m(self):
        self.observe(10.0, 1.0)
        self.observe(10.2, 2.0)
        result = self.observe(10.4, 3.0)

        self.assertEqual(result.status, DONE)
        self.assertAlmostEqual(result.geometry.door_center[0], 0.57, places=2)
        self.assertAlmostEqual(result.geometry.old_center_shift, 0.57, places=2)
        self.assertAlmostEqual(result.geometry.cabin_center[1], 1.75, places=2)

    def test_exit_ignores_stable_near_structure_and_uses_distant_door(self):
        points = _opening_points(center=(0.57, 0.70))
        points.extend(_opening_points(center=(0.57, 0.95)))

        kwargs = {
            "points": points,
            "mode": "exit",
            "old_door_center": (0.57, 0.95),
        }
        self.observe(10.0, 1.0, **kwargs)
        self.observe(10.2, 2.0, **kwargs)
        result = self.observe(10.4, 3.0, **kwargs)

        self.assertEqual(result.status, DONE)
        self.assertAlmostEqual(result.geometry.door_center[1], 0.95, places=2)
        self.assertAlmostEqual(result.geometry.cabin_center[1], 0.0, places=2)

    def test_confirmation_resets_after_gap_or_inconsistent_center(self):
        self.observe(10.0, 1.0)
        gap = self.observe(10.7, 2.0)
        shifted = self.observe(10.9, 3.0, _opening_points(center=(0.30, 0.8)))

        self.assertEqual(gap.diagnostic["confirmations"], 1)
        self.assertEqual(shifted.diagnostic["confirmations"], 1)

    def test_timeout_fails_closed_without_stale_geometry_fallback(self):
        self.observe(10.0, 1.0, points=[])
        result = self.observe(14.1, 2.0, points=[])

        self.assertEqual(result.status, FAILED)
        self.assertEqual(result.failure, "elevator_open_crossing_geometry_unavailable")
        self.assertIsNone(result.geometry)

    def test_exit_empty_observations_fall_back_after_frame_and_time_thresholds(self):
        self.config = ElevatorCrossingConfig(
            exit_empty_fallback_timeout=1.0,
            exit_empty_fallback_frames=10,
        )
        results = [
            self.observe(
                10.0 + index * 0.1,
                float(index + 1),
                points=[],
                mode="exit",
            )
            for index in range(10)
        ]
        self.assertTrue(all(result.status == RUNNING for result in results))

        fallback = self.observe(11.0, 11.0, points=[], mode="exit")

        self.assertEqual(fallback.status, FAILED)
        self.assertEqual(
            fallback.failure,
            "elevator_open_crossing_geometry_not_observed",
        )
        self.assertEqual(
            fallback.diagnostic["event"],
            "elevator_local_crossing_geometry_early_fallback",
        )
        self.assertEqual(fallback.diagnostic["empty_frame_count"], 11)

    def test_entry_does_not_use_exit_empty_observation_early_fallback(self):
        self.config = ElevatorCrossingConfig(
            exit_empty_fallback_timeout=1.0,
            exit_empty_fallback_frames=10,
        )
        result = None
        for index in range(11):
            result = self.observe(
                10.0 + index * 0.1,
                float(index + 1),
                points=[],
                mode="enter",
            )

        self.assertEqual(result.status, RUNNING)

    def test_exit_candidate_resets_consecutive_empty_observation_count(self):
        self.config = ElevatorCrossingConfig(
            exit_empty_fallback_timeout=1.0,
            exit_empty_fallback_frames=6,
        )
        for index in range(5):
            self.observe(
                10.0 + index * 0.1,
                float(index + 1),
                points=[],
                mode="exit",
            )
        candidate = self.observe(10.5, 6.0, mode="exit")
        self.assertEqual(candidate.status, RUNNING)
        self.assertEqual(candidate.diagnostic["confirmations"], 1)

        result = None
        for index in range(5):
            result = self.observe(
                11.0 + index * 0.1,
                float(index + 7),
                points=[],
                mode="exit",
            )

        self.assertEqual(result.status, RUNNING)

    def test_context_is_scoped_by_door_and_mode(self):
        self.observe(10.0, 1.0)
        result = self.runtime.observe(
            now=10.2,
            door_id="lift_1",
            mode="exit",
            expected_crossing_yaw=math.pi / 2.0,
            old_door_center=(0.0, 0.8),
            pose=(0.57, 0.0, 0.0),
            cloud_stamp=2.0,
            cloud_points=_opening_points(),
            floor_index=1,
            floor_height=0.0,
            local_detector=self.detector,
            config=self.config,
        )

        self.assertEqual(result.status, RUNNING)
        self.assertEqual(result.diagnostic["confirmations"], 1)


class CrossingCorridorObstacleTest(unittest.TestCase):
    def observe(self, points):
        return crossing_corridor_obstacle(
            points,
            start_xy=(0.0, 0.0),
            end_xy=(0.0, 1.75),
            floor_index=0,
            floor_height=2.6,
            obstacle_min_height=0.15,
            obstacle_max_height=1.40,
            sweep_half_width=0.435393,
            door_center=(0.0, 0.8),
            door_yaw=-math.pi / 2.0,
            door_width=0.90,
        )

    def test_live_obstacle_inside_crossing_corridor_blocks(self):
        result = self.observe([(0.20, 1.35, 0.75)])

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.forward, 1.35)

    def test_side_wall_and_open_door_plane_residue_are_ignored(self):
        result = self.observe(
            [
                (0.55, 1.35, 0.75),
                (0.20, 0.80, 0.75),
            ]
        )

        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()

import math
import unittest
from types import SimpleNamespace

from danger_search.building_control_client import DoorInfo
from danger_search.local_semantic_detector import LocalSemanticDetector


class LocalSemanticDetectorTest(unittest.TestCase):
    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            LocalSemanticDetector(max_distance=math.nan)
        with self.assertRaises(ValueError):
            LocalSemanticDetector(opening_min_width=2.0, opening_max_width=1.0)
    @staticmethod
    def _opening_points(
        robot_pose=(1.0, 2.0, math.pi / 2.0),
        *,
        forward=0.8,
        half_gap=0.45,
        include_center=False,
    ):
        heading = (math.cos(robot_pose[2]), math.sin(robot_pose[2]))
        lateral = (-heading[1], heading[0])
        points = []
        lateral_samples = (-0.75, -0.60, -half_gap, half_gap, 0.60, 0.75)
        if include_center:
            lateral_samples = (*lateral_samples, -0.10, 0.0, 0.10)
        for local_y in lateral_samples:
            for height in (0.20, 0.50, 0.80, 1.10, 1.40):
                points.append(
                    (
                        robot_pose[0] + forward * heading[0] + local_y * lateral[0],
                        robot_pose[1] + forward * heading[1] + local_y * lateral[1],
                        height,
                    )
                )
        return points

    def test_detects_opening_ahead_from_two_sensor_observed_edges(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        robot_pose = (1.0, 2.0, math.pi / 2.0)

        opening = detector.detect_opening_ahead(
            robot_pose=robot_pose,
            floor_index=0,
            cloud_points=self._opening_points(robot_pose),
            floor_height=2.6,
        )

        self.assertIsNotNone(opening)
        self.assertAlmostEqual(opening.world_xy[0], 1.0, places=2)
        self.assertAlmostEqual(opening.world_xy[1], 2.8, places=2)
        self.assertAlmostEqual(opening.width, 0.90, places=2)
        self.assertAlmostEqual(opening.yaw, math.pi / 2.0, places=2)

    def test_opening_distance_window_skips_near_internal_structure(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        robot_pose = (0.0, 0.0, 0.0)
        points = self._opening_points(robot_pose, forward=0.35)
        points.extend(self._opening_points(robot_pose, forward=0.95))

        opening = detector.detect_opening_ahead(
            robot_pose=robot_pose,
            floor_index=0,
            cloud_points=points,
            floor_height=2.6,
            minimum_distance=0.60,
            maximum_distance=1.30,
        )

        self.assertIsNotNone(opening)
        self.assertAlmostEqual(opening.distance, 0.95, places=2)

    def test_reports_all_openings_for_memory_based_selection(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        robot_pose = (0.0, 0.0, 0.0)
        points = self._opening_points(robot_pose, forward=0.70)
        points.extend(self._opening_points(robot_pose, forward=0.95))

        openings = detector.detect_openings_ahead(
            robot_pose=robot_pose,
            floor_index=0,
            cloud_points=points,
            floor_height=2.6,
        )

        self.assertEqual(
            [round(opening.distance, 2) for opening in openings],
            [0.70, 0.95],
        )

    def test_opening_ahead_rejects_structure_across_center_lane(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)

        opening = detector.detect_opening_ahead(
            robot_pose=(0.0, 0.0, 0.0),
            floor_index=0,
            cloud_points=self._opening_points(
                (0.0, 0.0, 0.0),
                include_center=True,
            ),
            floor_height=2.6,
        )

        self.assertIsNone(opening)

    def test_opening_ahead_ignores_low_center_floor_noise_without_vertical_span(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        points = self._opening_points((0.0, 0.0, 0.0))
        points.extend((0.8, lateral, 0.18) for lateral in (-0.10, 0.0, 0.10) for _ in range(8))

        opening = detector.detect_opening_ahead(
            robot_pose=(0.0, 0.0, 0.0),
            floor_index=0,
            cloud_points=points,
            floor_height=2.6,
        )

        self.assertIsNotNone(opening)
        self.assertAlmostEqual(opening.width, 0.90, places=2)

    def test_elevator_opening_requires_observed_side_and_back_walls(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        points = self._opening_points((0.0, 0.0, 0.0))
        heights = (0.25, 0.55, 0.85, 1.15)
        for depth in (0.25, 0.45, 0.65, 0.85, 1.05, 1.20):
            for height in heights:
                points.append((0.8 + depth, -0.45, height))
                points.append((0.8 + depth, 0.45, height))
        for lateral in (-0.45, -0.30, -0.15, 0.0, 0.15, 0.30, 0.45):
            for height in heights:
                points.append((2.0, lateral, height))

        observation = detector.detect_elevator_opening_ahead(
            robot_pose=(0.0, 0.0, 0.0),
            floor_index=0,
            cloud_points=points,
            floor_height=2.6,
        )

        self.assertIsNotNone(observation)
        self.assertAlmostEqual(observation.world_xy[0], 0.8, places=2)
        self.assertAlmostEqual(observation.width, 0.90, places=2)

    def test_ordinary_opening_is_not_registered_as_elevator(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)

        observation = detector.detect_elevator_opening_ahead(
            robot_pose=(0.0, 0.0, 0.0),
            floor_index=0,
            cloud_points=self._opening_points((0.0, 0.0, 0.0)),
            floor_height=2.6,
        )

        self.assertIsNone(observation)

    def test_nearby_elevator_detection_does_not_require_robot_heading(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        points = self._opening_points((0.0, 0.0, 0.0))
        heights = (0.25, 0.55, 0.85, 1.15)
        for depth in (0.25, 0.45, 0.65, 0.85, 1.05, 1.20):
            for height in heights:
                points.append((0.8 + depth, -0.45, height))
                points.append((0.8 + depth, 0.45, height))
        for lateral in (-0.45, -0.30, -0.15, 0.0, 0.15, 0.30, 0.45):
            for height in heights:
                points.append((2.0, lateral, height))

        observation = detector.detect_elevator_opening_nearby(
            robot_pose=(0.0, 0.0, math.pi / 2.0),
            floor_index=0,
            cloud_points=points,
            floor_height=2.6,
        )

        self.assertIsNotNone(observation)
        self.assertAlmostEqual(observation.world_xy[0], 0.8, places=2)
        self.assertAlmostEqual(observation.world_xy[1], 0.0, places=2)

    def test_detects_visible_dynamic_door_with_pointcloud_evidence(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                DoorInfo("main_entrance", "main_entrance", 0, 2.0, 0.0, math.pi / 2.0)
            ]
        )
        points = [(2.0, 0.0, 0.4), (2.1, 0.1, 0.8), (1.9, -0.1, 1.2)]

        candidate = detector.detect_controllable_door(
            robot_pose=(0.0, 0.0, 0.0),
            floor_index=0,
            cloud_points=points,
            building_client=client,
            floor_height=2.6,
        )

        self.assertIsNotNone(candidate)
        self.assertEqual(candidate.door_id, "main_entrance")
        self.assertEqual(candidate.kind, "dynamic_door")
        self.assertEqual(candidate.source, "local_pointcloud")

    def test_rejects_truth_proximity_without_sensor_evidence_by_default(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=3)
        client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                DoorInfo("main_entrance", "main_entrance", 0, 2.0, 0.0, math.pi / 2.0)
            ]
        )

        candidate = detector.detect_controllable_door(
            robot_pose=(0.0, 0.0, 0.0),
            floor_index=0,
            cloud_points=[],
            building_client=client,
            floor_height=2.6,
        )

        self.assertIsNone(candidate)

    def test_truth_proximity_debug_fallback_is_explicit(self):
        detector = LocalSemanticDetector(
            max_distance=3.0,
            min_evidence_points=3,
            use_truth_proximity_detector=True,
        )
        client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                DoorInfo("main_entrance", "main_entrance", 0, 2.0, 0.0, math.pi / 2.0)
            ]
        )

        candidate = detector.detect_controllable_door(
            robot_pose=(0.0, 0.0, 0.0),
            floor_index=0,
            cloud_points=[],
            building_client=client,
            floor_height=2.6,
        )

        self.assertIsNotNone(candidate)
        self.assertEqual(candidate.source, "truth_proximity_debug")

    def test_does_not_return_elevator_door_during_explore(self):
        detector = LocalSemanticDetector(max_distance=3.0, min_evidence_points=1)
        client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                DoorInfo("elevator_floor_0", "elevator", 0, 2.0, 0.0, math.pi)
            ]
        )

        candidate = detector.detect_controllable_door(
            robot_pose=(0.0, 0.0, 0.0),
            floor_index=0,
            cloud_points=[(2.0, 0.0, 0.6)],
            building_client=client,
            floor_height=2.6,
            allow_elevator=False,
        )

        self.assertIsNone(candidate)


if __name__ == "__main__":
    unittest.main()

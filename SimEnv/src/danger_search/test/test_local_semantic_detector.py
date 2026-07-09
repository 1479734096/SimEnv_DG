import math
import unittest
from types import SimpleNamespace

from danger_search.building_control_client import DoorInfo
from danger_search.local_semantic_detector import LocalSemanticDetector


class LocalSemanticDetectorTest(unittest.TestCase):
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

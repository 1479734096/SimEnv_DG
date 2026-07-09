import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.local_semantic_detector import DoorCandidate
from danger_search.mapper import MultiFloorGridMap
import danger_search.mission_manager as mission_manager_module
from danger_search.mission_manager import EXPLORE_FLOOR
from danger_search.mission_manager import MissionManager
from danger_search.mission_manager import probe_target_for_attempt


class MissionManagerHelperTest(unittest.TestCase):
    def test_probe_targets_sweep_lanes_before_extending_forward(self):
        base_pose = (0.0, 0.0, 0.0)

        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 0), (1.2, 0.0))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 1), (1.2, -1.2))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 2), (1.2, 1.2))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 5), (2.4, 0.0))

    def test_probe_targets_follow_home_yaw(self):
        target = probe_target_for_attempt((2.0, 3.0, math.pi / 2.0), 1.0, 2)

        self.assertAlmostEqual(target[0], 1.0, places=6)
        self.assertAlmostEqual(target[1], 4.0, places=6)

    def test_probe_distance_has_lower_bound(self):
        target = probe_target_for_attempt((0.0, 0.0, 0.0), 0.0, 0)

        self.assertEqual(target, (0.05, 0.0))

    def test_should_open_door_accepts_visible_dynamic_candidate(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="main_entrance",
            distance=1.2,
            visible=True,
            kind="dynamic_door",
            floor_index=0,
            world_xy=(0.0, 0.0),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertTrue(manager._should_open_door(candidate))

    def test_should_open_door_rejects_elevator_candidate(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: True,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="elevator_floor_0",
            distance=1.2,
            visible=True,
            kind="elevator",
            floor_index=0,
            world_xy=(1.0, 2.6),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertFalse(manager._should_open_door(candidate))

    def test_should_open_door_does_not_interrupt_current_task(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_task = object()
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="main_entrance",
            distance=1.2,
            visible=True,
            kind="dynamic_door",
            floor_index=0,
            world_xy=(0.0, 0.0),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertFalse(manager._should_open_door(candidate))

    def test_probe_unknown_space_starts_from_current_pose_after_passing_door(self):
        recorded_paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.home_pose = (0.0, -2.2, math.pi / 2.0)
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._probe_attempt = 0
        manager.current_frontier = object()
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.controller = SimpleNamespace(set_path=lambda path: recorded_paths.append(path))

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(loginfo=lambda *args, **kwargs: None)):
            self.assertTrue(manager._try_probe_unknown_space((0.0, 0.8, math.pi / 2.0)))

        self.assertEqual(len(recorded_paths), 1)
        self.assertAlmostEqual(recorded_paths[0][-1][0], 0.0, places=6)
        self.assertAlmostEqual(recorded_paths[0][-1][1], 2.0, places=6)

    def test_probe_unknown_space_skips_targets_behind_known_wall(self):
        recorded_paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.home_pose = (0.0, -2.2, math.pi / 2.0)
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._probe_attempt = 0
        manager.current_frontier = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        grid = manager.mapper.floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.0, 1.4))
        manager.controller = SimpleNamespace(set_path=lambda path: recorded_paths.append(path))

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(loginfo=lambda *args, **kwargs: None)):
            self.assertTrue(manager._try_probe_unknown_space((0.0, 0.8, math.pi / 2.0)))

        self.assertEqual(len(recorded_paths), 1)
        self.assertNotAlmostEqual(recorded_paths[0][-1][0], 0.0, places=6)


if __name__ == "__main__":
    unittest.main()

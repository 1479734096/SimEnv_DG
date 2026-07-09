import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.building_control_client import DoorInfo
from danger_search.mapper import MultiFloorGridMap
import danger_search.mission_manager as mission_manager_module
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

    def test_setup_floor_doors_only_tracks_non_elevator_doors_on_current_floor(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.unvisited_doors = set()
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": DoorInfo("main_entrance", "main_entrance", 0, 0.0, 0.0, math.pi / 2.0),
                "elevator_floor_0": DoorInfo("elevator_floor_0", "elevator", 0, 1.65, 2.6, math.pi),
                "floor_1_gate": DoorInfo("floor_1_gate", "main_entrance", 1, 0.0, 0.0, math.pi / 2.0),
            }
        )

        manager._setup_floor_doors()

        self.assertEqual(manager.unvisited_doors, {"main_entrance"})

    def test_nearest_unvisited_door_chooses_closest_metadata_pose(self):
        manager = MissionManager.__new__(MissionManager)
        manager.unvisited_doors = {"far_gate", "near_gate"}
        manager.building_client = SimpleNamespace(
            doors={
                "far_gate": DoorInfo("far_gate", "main_entrance", 0, 5.0, 0.0, 0.0),
                "near_gate": DoorInfo("near_gate", "main_entrance", 0, 1.0, 0.0, 0.0),
            }
        )

        self.assertEqual(manager._nearest_unvisited_door((0.0, 0.0, 0.0)), "near_gate")

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

import math
import unittest

from danger_search.building_control_client import BuildingControlClient, DoorInfo


class BuildingControlClientTest(unittest.TestCase):
    def test_constructor_does_not_create_spatial_targets_from_scene_files(self):
        client = BuildingControlClient()

        self.assertEqual(client.doors, {})
        self.assertEqual(client.elevators, {})
        self.assertFalse(client.has_controllable_doors())

    def test_public_service_calls_fail_closed_before_features_are_registered(self):
        client = BuildingControlClient()

        with self.assertRaises(ValueError):
            client.open_door("main_entrance")
        with self.assertRaises(ValueError):
            client.close_door("main_entrance")
        with self.assertRaises(ValueError):
            client.call_elevator("elevator_main", 0)

    def test_public_service_calls_reject_unknown_ids_and_floors(self):
        client = BuildingControlClient()
        client.set_public_features(
            door_ids={"main_entrance"},
            elevators={"elevator_main": {0, 1}},
        )

        with self.assertRaises(ValueError):
            client.open_door("unknown")
        with self.assertRaises(ValueError):
            client.call_elevator("unknown", 0)
        with self.assertRaises(ValueError):
            client.call_elevator("elevator_main", 2)

    def test_elevator_door_id_requires_one_public_id_per_floor(self):
        client = BuildingControlClient()
        client.set_public_features(
            door_ids={"lift_ground", "lift_upper_a", "lift_upper_b"},
            elevators={"lift": {0, 1, 2}},
            elevator_door_ids_by_floor={
                0: ["lift_ground"],
                1: ["lift_upper_a", "lift_upper_b"],
            },
        )

        self.assertEqual(client.elevator_door_id(0), "lift_ground")
        with self.assertRaises(ValueError):
            client.elevator_door_id(1)
        with self.assertRaises(ValueError):
            client.elevator_door_id(2)

    def test_door_points_choose_current_side_as_wait_point(self):
        client = BuildingControlClient()
        client.doors["main_entrance"] = DoorInfo(
            "main_entrance", "main_entrance", 0, 0.0, 0.0, math.pi / 2.0,
        )

        wait_point, pass_point = client.get_door_points(
            "main_entrance", (0.0, -1.0), offset=0.8,
        )

        self.assertAlmostEqual(wait_point[0], 0.0, places=6)
        self.assertAlmostEqual(wait_point[1], -0.8, places=6)
        self.assertAlmostEqual(pass_point[0], 0.0, places=6)
        self.assertAlmostEqual(pass_point[1], 0.8, places=6)

    def test_door_path_points_extend_deeper_into_pass_side(self):
        client = BuildingControlClient()
        client.doors["main_entrance"] = DoorInfo(
            "main_entrance", "main_entrance", 0, 0.0, 0.0, math.pi / 2.0,
        )

        wait_point, pass_point, commit_point = client.get_door_path_points(
            "main_entrance",
            (0.0, -1.0),
            pass_offset=0.8,
            commit_distance=1.8,
        )

        self.assertAlmostEqual(wait_point[1], -0.8, places=6)
        self.assertAlmostEqual(pass_point[1], 0.8, places=6)
        self.assertAlmostEqual(commit_point[0], 0.0, places=6)
        self.assertAlmostEqual(commit_point[1], 1.8, places=6)


if __name__ == "__main__":
    unittest.main()

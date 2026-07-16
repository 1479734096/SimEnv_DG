import math
import unittest

from danger_search.building_control_client import BuildingControlClient, DoorInfo


class BuildingControlClientTest(unittest.TestCase):
    def test_constructor_does_not_create_spatial_targets_from_scene_files(self):
        client = BuildingControlClient()

        self.assertEqual(client.doors, {})
        self.assertEqual(client.elevators, {})
        self.assertFalse(client.has_controllable_doors())

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

import json
import math
import tempfile
import unittest
from pathlib import Path

from danger_search.building_control_client import BuildingControlClient


class BuildingControlClientTest(unittest.TestCase):
    def test_loads_dynamic_doors_and_elevator_metadata(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            metadata_path = Path(tmpdir) / "layout_metadata.json"
            metadata_path.write_text(
                json.dumps(
                    {
                        "door_specs": [
                            {
                                "id": "main_entrance",
                                "kind": "main_entrance",
                                "floor_index": 0,
                                "dynamic": True,
                                "pose": [0.0, 0.0, 1.2, 0.0, 0.0, math.pi / 2.0],
                            },
                            {
                                "id": "static_room_door",
                                "kind": "room",
                                "floor_index": 0,
                                "dynamic": False,
                                "pose": [1.0, 2.0, 1.2, 0.0, 0.0, 0.0],
                            },
                            {
                                "id": "elevator_floor_0",
                                "kind": "elevator",
                                "floor_index": 0,
                                "dynamic": True,
                                "pose": [1.65, 2.6, 1.2, 0.0, 0.0, math.pi],
                            },
                        ],
                        "elevator_specs": [
                            {
                                "id": "elevator_main",
                                "lobby_positions": {
                                    "0": [1.65, 2.6, 1.2, 0.0, 0.0, math.pi],
                                },
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            client = BuildingControlClient(metadata_path=str(metadata_path))

        self.assertIn("main_entrance", client.doors)
        self.assertNotIn("static_room_door", client.doors)
        self.assertIn("elevator_floor_0", client.doors)
        self.assertIn("elevator_main", client.elevators)
        self.assertTrue(client.has_door_targets())
        self.assertEqual(client.doors["main_entrance"].floor_index, 0)
        self.assertAlmostEqual(client.elevators["elevator_main"].lobby_poses[0][2], math.pi)

    def test_missing_metadata_keeps_empty_targets(self):
        client = BuildingControlClient(metadata_path="/tmp/not_existing_layout_metadata.json")

        self.assertEqual(client.doors, {})
        self.assertEqual(client.elevators, {})
        self.assertFalse(client.has_door_targets())

    def test_door_points_choose_current_side_as_wait_point(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            metadata_path = Path(tmpdir) / "layout_metadata.json"
            metadata_path.write_text(
                json.dumps(
                    {
                        "door_specs": [
                            {
                                "id": "main_entrance",
                                "kind": "main_entrance",
                                "floor_index": 0,
                                "dynamic": True,
                                "pose": [0.0, 0.0, 1.2, 0.0, 0.0, math.pi / 2.0],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            client = BuildingControlClient(metadata_path=str(metadata_path))
            wait_point, pass_point = client.get_door_points("main_entrance", (0.0, -1.0), offset=0.8)

        self.assertAlmostEqual(wait_point[0], 0.0, places=6)
        self.assertAlmostEqual(wait_point[1], -0.8, places=6)
        self.assertAlmostEqual(pass_point[0], 0.0, places=6)
        self.assertAlmostEqual(pass_point[1], 0.8, places=6)


if __name__ == "__main__":
    unittest.main()

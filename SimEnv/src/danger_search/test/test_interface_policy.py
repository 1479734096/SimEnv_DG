"""正式比赛接口准入规则测试。"""
from pathlib import Path
import unittest

from danger_search.interface_policy import (
    require_allowed_service,
    require_allowed_topic,
    require_public_scene_info,
)
from danger_search.building_feature_provider import BuildingFeatureProvider


class InterfacePolicyTest(unittest.TestCase):
    def test_initial_documented_topics_are_allowed(self):
        for topic in ("/scan", "/trunk_imu", "/real_sense/depth/points", "/cmd_vel"):
            with self.subTest(topic=topic):
                self.assertEqual(topic, require_allowed_topic(topic, purpose="测试"))

    def test_truth_and_undocumented_topics_are_rejected(self):
        for topic in ("/Odometry_gazebo", "/ground_truth/base_w", "/tf", "/nav/goal"):
            with self.subTest(topic=topic):
                with self.assertRaises(ValueError):
                    require_allowed_topic(topic, purpose="测试")

    def test_only_documented_services_are_allowed(self):
        self.assertEqual(
            "/set_door_state",
            require_allowed_service("/set_door_state", purpose="测试"),
        )
        with self.assertRaises(ValueError):
            require_allowed_service("/gazebo/set_model_state", purpose="测试")

    def test_only_public_scene_info_file_is_allowed(self):
        self.assertEqual(
            Path("generated_building/team_scene_info.json"),
            require_public_scene_info("generated_building/team_scene_info.json", purpose="测试"),
        )
        for path in (
            "generated_building/layout_metadata.json",
            "generated_building/scene_manifest.json",
            "results/danger_truth.json",
        ):
            with self.subTest(path=path):
                with self.assertRaises(ValueError):
                    require_public_scene_info(path, purpose="测试")

    def test_public_scene_provider_does_not_expose_geometry_truth(self):
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "team_scene_info.json"
            path.write_text(
                json.dumps(
                    {
                        "schema": "team_scene_info_v1",
                        "robot_start": {"x": 1, "y": 2, "z": 0, "yaw": 0},
                        "public_scene": {
                            "door_ids": [{"id": "main_entrance"}],
                            "elevators": [{"id": "elevator_main", "served_floors": [0, 1]}],
                        },
                        "allowed_interfaces": {
                            "topics": ["/scan", "/cmd_vel"],
                            "services": ["/set_door_state", "/call_elevator"],
                            "result_file": "results/detected_danger.json",
                        },
                    }
                ),
                encoding="utf-8",
            )
            provider = BuildingFeatureProvider.load(path, purpose="测试")
            self.assertEqual(provider.robot_start, (1.0, 2.0, 0.0, 0.0))
            self.assertEqual(provider.elevators["elevator_main"], frozenset({0, 1}))
            self.assertFalse(hasattr(provider, "door_positions"))


if __name__ == "__main__":
    unittest.main()

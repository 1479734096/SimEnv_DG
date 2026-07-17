"""正式比赛接口准入规则测试。"""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from danger_search.interface_policy import (
    ALLOWED_SERVICES,
    ALLOWED_TOPICS,
    official_scene_info_path,
    require_allowed_service,
    require_allowed_topic,
    require_public_scene_info,
)
from danger_search.building_feature_provider import BuildingFeatureProvider


def _make_official_scene_info(directory: Path):
    """在临时目录中创建合规的唯一正式场景文件。"""
    generated = directory / "generated_building"
    generated.mkdir(parents=True, exist_ok=True)
    path = generated / "team_scene_info.json"
    path.write_text(
        json.dumps(
            {
                "schema": "team_scene_info_v1",
                "coordinate_frame": "world",
                "robot_start": {"x": 1, "y": 2, "z": 0, "yaw": 0},
                "public_scene": {
                    "door_ids": [{
                        "id": "main_entrance",
                        "kind": "main_entrance",
                        "floor_index": 0,
                        "initial_open": True,
                    }],
                    "elevators": [{
                        "id": "elevator_main",
                        "current_floor": 0,
                        "served_floors": [0],
                    }],
                },
                "allowed_interfaces": {
                    "topics": list(ALLOWED_TOPICS),
                    "services": list(ALLOWED_SERVICES),
                    "result_file": str((directory / "results" / "detected_danger.json").resolve()),
                },
            }
        ),
        encoding="utf-8",
    )
    return path


class InterfacePolicyTest(unittest.TestCase):
    def test_official_scene_info_path_uses_rospkg_lexical_workspace(self):
        """ROS 包路径本身是工作区别名时，Python/C++ 应使用同一词法根。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            workspace = root / "real" / "SimEnv"
            package = workspace / "src" / "danger_search"
            package.mkdir(parents=True)
            official = _make_official_scene_info(workspace)
            alias = root / "workspace_alias"
            try:
                alias.symlink_to(workspace, target_is_directory=True)
            except OSError:
                self.skipTest("当前环境无法创建目录符号链接")
            alias_package = alias / "src" / "danger_search"
            alias_official = alias / "generated_building" / "team_scene_info.json"

            with patch("rospkg.RosPack.get_path", return_value=str(alias_package)):
                self.assertEqual(official_scene_info_path(), alias_official)
                self.assertEqual(
                    require_public_scene_info(alias_official, purpose="测试"),
                    official.resolve(),
                )

    def test_tilde_alias_is_rejected_to_match_cpp_policy(self):
        """正式路径必须由 launch 明确传入，Python 不额外展开 shell 缩写。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            official = _make_official_scene_info(root)
            with patch.dict(os.environ, {"HOME": str(root)}), patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=official,
            ):
                with self.assertRaises(ValueError):
                    require_public_scene_info(
                        "~/generated_building/team_scene_info.json",
                        purpose="测试",
                    )

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

    def test_official_absolute_path_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            official = _make_official_scene_info(Path(directory))
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=official,
            ):
                self.assertEqual(
                    require_public_scene_info(str(official), purpose="测试"),
                    official.resolve(),
                )

    def test_lexical_traversal_to_official_path_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            official = _make_official_scene_info(Path(directory))
            supplied = official.parent / ".." / "generated_building" / "team_scene_info.json"
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=official,
            ):
                self.assertEqual(
                    require_public_scene_info(str(supplied), purpose="测试"),
                    official.resolve(),
                )

    def test_forbidden_scene_files_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            official = _make_official_scene_info(Path(directory))
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=official,
            ):
                for path in (
                    "generated_building/layout_metadata.json",
                    "generated_building/scene_manifest.json",
                    "results/danger_truth.json",
                ):
                    with self.subTest(path=path):
                        with self.assertRaises(ValueError):
                            require_public_scene_info(path, purpose="测试")

    def test_same_name_in_tmp_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            official = _make_official_scene_info(Path(directory))
            with tempfile.TemporaryDirectory() as tmp_dir:
                fake_path = Path(tmp_dir) / "team_scene_info.json"
                fake_path.write_text("{}", encoding="utf-8")
                with patch(
                    "danger_search.interface_policy.official_scene_info_path",
                    return_value=official,
                ):
                    with self.assertRaises(ValueError):
                        require_public_scene_info(str(fake_path), purpose="测试")

    def test_symlink_to_official_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target_root = root / "target"
            target = _make_official_scene_info(target_root)
            generated = root / "generated_building"
            generated.mkdir()
            official = generated / "team_scene_info.json"
            try:
                official.symlink_to(target)
            except OSError:
                self.skipTest("当前环境无法创建符号链接")
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=official,
            ):
                with self.assertRaises(ValueError):
                    require_public_scene_info(str(official), purpose="测试")

    def test_symlinked_generated_building_directory_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "src" / "danger_search"
            package.mkdir(parents=True)
            target_root = root / "target"
            target = _make_official_scene_info(target_root).parent
            generated = root / "generated_building"
            try:
                generated.symlink_to(target, target_is_directory=True)
            except OSError:
                self.skipTest("当前环境无法创建目录符号链接")
            official = generated / "team_scene_info.json"
            with patch("rospkg.RosPack.get_path", return_value=str(package)):
                with self.assertRaises(ValueError):
                    require_public_scene_info(official, purpose="测试")

    def test_public_scene_provider_does_not_expose_geometry_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _make_official_scene_info(Path(directory))
            (Path(directory) / "results").mkdir(parents=True, exist_ok=True)
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=path,
            ):
                provider = BuildingFeatureProvider.load(path, purpose="测试")
            self.assertEqual(provider.robot_start, (1.0, 2.0, 0.0, 0.0))
            self.assertEqual(provider.elevators["elevator_main"], frozenset({0}))
            self.assertEqual(provider.doors["main_entrance"].kind, "main_entrance")
            self.assertTrue(provider.doors["main_entrance"].initial_open)
            self.assertEqual(provider.elevator_features["elevator_main"].current_floor, 0)
            self.assertFalse(hasattr(provider, "door_positions"))

    def test_public_scene_provider_rejects_result_file_outside_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _make_official_scene_info(Path(directory))
            data = json.loads(path.read_text(encoding="utf-8"))
            data["allowed_interfaces"]["result_file"] = "/tmp/detected_danger.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=path,
            ):
                with self.assertRaises(ValueError):
                    BuildingFeatureProvider.load(path, purpose="测试")

    def test_public_scene_provider_rejects_invalid_schema_and_coordinate_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _make_official_scene_info(Path(directory))
            data = json.loads(path.read_text(encoding="utf-8"))
            data["schema"] = "team_scene_info_v2"
            path.write_text(json.dumps(data), encoding="utf-8")
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=path,
            ):
                with self.assertRaises(ValueError):
                    BuildingFeatureProvider.load(path, purpose="测试")

        with tempfile.TemporaryDirectory() as directory:
            path = _make_official_scene_info(Path(directory))
            data = json.loads(path.read_text(encoding="utf-8"))
            data["coordinate_frame"] = "map"
            path.write_text(json.dumps(data), encoding="utf-8")
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=path,
            ):
                with self.assertRaises(ValueError):
                    BuildingFeatureProvider.load(path, purpose="测试")

    def test_public_scene_provider_rejects_duplicate_and_negative_interface_data(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _make_official_scene_info(Path(directory))
            data = json.loads(path.read_text(encoding="utf-8"))
            data["public_scene"]["door_ids"].append({"id": "main_entrance"})
            path.write_text(json.dumps(data), encoding="utf-8")
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=path,
            ):
                with self.assertRaises(ValueError):
                    BuildingFeatureProvider.load(path, purpose="测试")

    def test_public_scene_provider_rejects_incomplete_or_coerced_public_features(self):
        invalid_mutations = (
            lambda data: data["robot_start"].update({"x": True}),
            lambda data: data["robot_start"].update({"yaw": "0.0"}),
            lambda data: data["public_scene"]["door_ids"][0].pop("kind"),
            lambda data: data["public_scene"]["door_ids"][0].update({"floor_index": True}),
            lambda data: data["public_scene"]["door_ids"][0].update({"initial_open": 1}),
            lambda data: data["public_scene"]["elevators"][0].update({"served_floors": ["0"]}),
            lambda data: data["public_scene"]["elevators"][0].update({"current_floor": 1}),
            lambda data: data.update({"public_scene": []}),
            lambda data: data.update({"allowed_interfaces": []}),
        )
        for mutate in invalid_mutations:
            with self.subTest(mutate=mutate):
                with tempfile.TemporaryDirectory() as directory:
                    path = _make_official_scene_info(Path(directory))
                    data = json.loads(path.read_text(encoding="utf-8"))
                    mutate(data)
                    path.write_text(json.dumps(data), encoding="utf-8")
                    with patch(
                        "danger_search.interface_policy.official_scene_info_path",
                        return_value=path,
                    ):
                        with self.assertRaises(ValueError):
                            BuildingFeatureProvider.load(path, purpose="测试")

        with tempfile.TemporaryDirectory() as directory:
            path = _make_official_scene_info(Path(directory))
            data = json.loads(path.read_text(encoding="utf-8"))
            data["public_scene"]["elevators"][0]["served_floors"] = [-1]
            path.write_text(json.dumps(data), encoding="utf-8")
            with patch(
                "danger_search.interface_policy.official_scene_info_path",
                return_value=path,
            ):
                with self.assertRaises(ValueError):
                    BuildingFeatureProvider.load(path, purpose="测试")


if __name__ == "__main__":
    unittest.main()

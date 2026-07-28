"""比赛接口文档一致性测试。"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


SIMENV_ROOT = Path(__file__).resolve().parents[3]
REFERENCE_PATH = SIMENV_ROOT / "docs" / "reference.md"
FORMAL_LAUNCH_PATH = SIMENV_ROOT / "src" / "danger_search" / "launch" / "danger_search.launch"
FORMAL_CONFIG_PATH = SIMENV_ROOT / "src" / "danger_search" / "config" / "danger_search.yaml"
LOCALIZATION_LAUNCH_PATH = (
    SIMENV_ROOT
    / "src"
    / "danger_search_localization"
    / "launch"
    / "scan_localization.launch"
)
README_PATH = SIMENV_ROOT / "README.md"
ALGORITHM_INTERFACES_PATH = SIMENV_ROOT / "docs" / "algorithm-interfaces.md"
TRUTH_RESTRICTION = (
    "`/Odometry_gazebo`、`/ground_truth/base_w`、`/ground_truth/base_trunk` 和 "
    "`/ground_truth/*_foot` 是 Gazebo 真值通道，不作为正式比赛算法输入。"
    "即使本地调试时能够看到这些话题，参赛算法也不得订阅或读取。"
)


class InterfaceDocumentationTest(unittest.TestCase):
    def test_formal_launch_explicitly_passes_public_scene_path_to_localization(self):
        root = ElementTree.parse(FORMAL_LAUNCH_PATH).getroot()
        self.assertFalse(
            any(arg.get("name") == "team_scene_info_path" for arg in root.findall("arg"))
        )
        self.assertFalse(any(arg.get("name") == "max_floor" for arg in root.findall("arg")))
        localization_includes = [
            include
            for include in root.findall("include")
            if "scan_localization.launch" in str(include.get("file"))
        ]
        self.assertEqual(len(localization_includes), 1)
        arguments = {
            arg.get("name"): arg.get("value")
            for arg in localization_includes[0].findall("arg")
        }
        self.assertEqual(
            arguments.get("team_scene_info_path"),
            "$(find danger_search)/../../generated_building/team_scene_info.json",
        )
        launch_arguments = {
            arg.get("name"): arg.get("default") for arg in root.findall("arg")
        }
        self.assertEqual(
            launch_arguments.get("localization_backend"), "adaptive_shared_map"
        )
        self.assertEqual(
            arguments.get("localization_backend"), "$(arg localization_backend)"
        )
        formal_node = next(
            node for node in root.findall("node") if node.get("name") == "danger_search"
        )
        self.assertNotIn(
            "mission/max_floor",
            {param.get("name") for param in formal_node.findall("param")},
        )
        node_parameters = {
            param.get("name"): param.get("value")
            for param in formal_node.findall("param")
        }
        self.assertEqual(
            node_parameters.get("wait_for_localization_ready"),
            "true",
        )
        localization_arguments = {
            arg.get("name"): arg.get("value")
            for arg in localization_includes[0].findall("arg")
        }
        self.assertEqual(
            localization_arguments.get("stance_health_enabled"),
            "true",
        )

    def test_localization_extrinsics_are_ros_parameter_arrays(self):
        root = ElementTree.parse(LOCALIZATION_LAUNCH_PATH).getroot()
        node = root.find("node")
        self.assertIsNotNone(node)
        parameters = {
            item.get("param"): item.text.strip()
            for item in node.findall("rosparam")
            if item.get("param")
        }
        self.assertEqual(parameters.get("base_to_livox_xyz"), "[0.2, 0.0, 0.08]")
        self.assertEqual(parameters.get("base_to_livox_rpy"), "[0.0, 0.785, 0.0]")
        self.assertEqual(parameters.get("base_to_depth_xyz"), "[0.28, 0.0, 0.043]")
        self.assertEqual(
            parameters.get("base_to_depth_rpy"),
            "[-1.57079632679, 0.0, -1.57079632679]",
        )
        string_parameters = {item.get("name") for item in node.findall("param")}
        arguments = {item.get("name"): item.get("default") for item in root.findall("arg")}
        self.assertEqual(arguments.get("localization_backend"), "kiss_icp")
        self.assertEqual(arguments.get("shared_map_depth_max_age"), "0.35")
        self.assertEqual(
            arguments.get("shared_map_max_source_divergence"), "0.05"
        )
        self.assertEqual(
            arguments.get("shared_map_max_map_commit_difference"), "0.05"
        )
        self.assertEqual(
            next(
                item.get("value")
                for item in node.findall("param")
                if item.get("name") == "localization_backend"
            ),
            "$(arg localization_backend)",
        )
        self.assertNotIn("base_to_livox_xyz", string_parameters)
        self.assertNotIn("base_to_livox_rpy", string_parameters)

    def test_truth_restriction_is_listed_in_all_interface_documents(self):
        for path in (ALGORITHM_INTERFACES_PATH, REFERENCE_PATH):
            with self.subTest(path=path):
                content = path.read_text(encoding="utf-8")
                self.assertIn(TRUTH_RESTRICTION, content)
        readme = README_PATH.read_text(encoding="utf-8")
        self.assertIn("真值文件仅供裁判评估使用", readme)
        self.assertIn("不作为参赛算法输入", readme)

    def test_stance_health_allows_the_normal_angular_command_envelope(self):
        launch_root = ElementTree.parse(LOCALIZATION_LAUNCH_PATH).getroot()
        launch_arguments = {
            arg.get("name"): arg.get("default")
            for arg in launch_root.findall("arg")
        }
        config = yaml.safe_load(FORMAL_CONFIG_PATH.read_text(encoding="utf-8"))

        self.assertGreaterEqual(
            float(launch_arguments["health_max_angular_speed"]),
            float(config["controller"]["max_angular"]),
        )

    def test_reference_marks_truth_topics_as_referee_only(self):
        content = REFERENCE_PATH.read_text(encoding="utf-8")
        self.assertIn("`/ground_truth/base_w`", content)
        self.assertIn("不作为正式比赛算法输入", content)

    def test_reference_marks_referee_odom_switch_as_non_algorithm_interface(self):
        content = REFERENCE_PATH.read_text(encoding="utf-8")
        self.assertIn("ENABLE_REFEREE_ODOM", content)
        self.assertIn("`/Odometry_gazebo`", content)
        self.assertIn("参赛算法也不得订阅或读取", content)

    def test_formal_config_does_not_contain_fixed_elevator_geometry(self):
        config = yaml.safe_load(FORMAL_CONFIG_PATH.read_text(encoding="utf-8"))
        self.assertNotIn("max_floor", config.get("mission", {}))
        floor = config.get("floor", {})
        self.assertNotIn("elevator_lobby_xy", floor)
        self.assertNotIn("elevator_inside_xy", floor)
        self.assertNotIn("elevator_id", floor)

    def test_formal_config_enables_verified_ground_support_envelope(self):
        config = yaml.safe_load(FORMAL_CONFIG_PATH.read_text(encoding="utf-8"))
        controller = config["controller"]
        support = config["ground_support"]
        validated_max_speed = float(support["stop_distance_table"][-1][0])

        self.assertTrue(support["production_gate_enabled"])
        self.assertTrue(config["localization"]["stance_health_required"])
        self.assertLessEqual(float(controller["max_linear"]), validated_max_speed)
        self.assertGreaterEqual(
            float(support["stop_linear_threshold"]),
            float(support["odometry_linear_deadband"]),
        )
        self.assertGreaterEqual(
            float(support["stop_angular_threshold"]),
            float(support["odometry_angular_deadband"]),
        )


if __name__ == "__main__":
    unittest.main()

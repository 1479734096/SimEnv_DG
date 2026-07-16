"""比赛接口文档一致性测试。"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree


SIMENV_ROOT = Path(__file__).resolve().parents[3]
REFERENCE_PATH = SIMENV_ROOT / "docs" / "reference.md"
FORMAL_LAUNCH_PATH = SIMENV_ROOT / "src" / "danger_search" / "launch" / "danger_search.launch"
LOCALIZATION_LAUNCH_PATH = (
    SIMENV_ROOT
    / "src"
    / "danger_search_localization"
    / "launch"
    / "scan_localization.launch"
)
DOCUMENT_PATHS = (
    SIMENV_ROOT / "README.md",
    SIMENV_ROOT / "docs" / "algorithm-interfaces.md",
    REFERENCE_PATH,
)
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
        string_parameters = {item.get("name") for item in node.findall("param")}
        self.assertNotIn("base_to_livox_xyz", string_parameters)
        self.assertNotIn("base_to_livox_rpy", string_parameters)

    def test_truth_restriction_is_listed_in_all_interface_documents(self):
        for path in DOCUMENT_PATHS:
            with self.subTest(path=path):
                content = path.read_text(encoding="utf-8")
                self.assertIn(TRUTH_RESTRICTION, content)

    def test_odometry_is_not_documented_as_an_algorithm_interface(self):
        for path in DOCUMENT_PATHS:
            with self.subTest(path=path):
                content = path.read_text(encoding="utf-8")
                self.assertNotIn("| `/Odometry_gazebo` |", content)

    def test_reference_does_not_document_nonexistent_odom_switch(self):
        content = REFERENCE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("ENABLE_REFEREE_ODOM", content)


if __name__ == "__main__":
    unittest.main()

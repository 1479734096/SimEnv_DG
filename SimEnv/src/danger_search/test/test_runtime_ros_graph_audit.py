#!/usr/bin/env python3
"""完整 ROS graph 审计器的纯函数测试。"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import unittest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[3]
    / "tools"
    / "offline"
    / "check_runtime_ros_graph.py"
)
SPEC = spec_from_file_location("check_runtime_ros_graph", SCRIPT_PATH)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _valid_publishers():
    return [
        ("/cmd_vel", ["/danger_search"]),
        ("/danger_search/localization/odom", ["/scan_localization"]),
        (
            "/danger_search/localization/cloud_registered",
            ["/scan_localization"],
        ),
        ("/danger_search/localization/health", ["/scan_localization"]),
        ("/danger_search/localization/ready", ["/scan_localization"]),
    ]


class RuntimeRosGraphAuditTest(unittest.TestCase):
    def test_valid_complete_graph_passes(self):
        self.assertEqual(
            MODULE.audit_runtime_graph(
                publishers=_valid_publishers(),
                subscribers=[],
            ),
            [],
        )

    def test_truth_subscription_cmd_vel_competition_and_joy_are_reported(self):
        publishers = _valid_publishers()
        publishers.extend(
            [
                ("/cmd_vel", ["/danger_search", "/other_controller"]),
                ("/joy", ["/joy_node"]),
            ]
        )
        errors = MODULE.audit_runtime_graph(
            publishers=publishers,
            subscribers=[
                ("/ground_truth/base_w", ["/junior_ctrl"]),
                ("/Odometry_gazebo", ["/pointcloud_converter"]),
            ],
        )
        joined = "\n".join(errors)
        self.assertIn("/junior_ctrl", joined)
        self.assertIn("/pointcloud_converter", joined)
        self.assertIn("/other_controller", joined)
        self.assertIn("/joy_node", joined)

    def test_internal_topic_requires_exact_producer(self):
        publishers = [
            entry
            for entry in _valid_publishers()
            if entry[0] != "/danger_search/localization/odom"
        ]
        publishers.append(
            (
                "/danger_search/localization/odom",
                ["/ground_truth_relay"],
            )
        )
        errors = MODULE.audit_runtime_graph(
            publishers=publishers,
            subscribers=[],
        )
        self.assertIn("/ground_truth_relay", "\n".join(errors))

    def test_environment_tf_publishers_are_inventory_but_algorithm_tf_use_fails(self):
        publishers = _valid_publishers()
        publishers.append(("/tf", ["/robot_state_publisher"]))
        self.assertEqual(
            MODULE.audit_runtime_graph(
                publishers=publishers,
                subscribers=[("/tf", ["/rviz"])],
            ),
            [],
        )
        errors = MODULE.audit_runtime_graph(
            publishers=publishers,
            subscribers=[("/tf", ["/scan_localization"])],
        )
        self.assertIn("/scan_localization", "\n".join(errors))


if __name__ == "__main__":
    unittest.main()

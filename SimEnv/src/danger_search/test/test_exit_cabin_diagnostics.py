"""EXIT_CABIN 外部诊断工具的离线测试。"""
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


TOOL_PATH = Path(__file__).resolve().parents[1] / "tools" / "capture_exit_cabin_diagnostics.py"
SPEC = importlib.util.spec_from_file_location("capture_exit_cabin_diagnostics", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
diagnostics = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = diagnostics
SPEC.loader.exec_module(diagnostics)


def _pose(x: float, y: float, z: float):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _twist(linear_x: float, angular_z: float = 0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=linear_x, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=angular_z),
    )


class ExitCabinDiagnosticsTest(unittest.TestCase):
    def test_parse_elevator_floors_from_production_task_name(self):
        self.assertEqual(diagnostics.parse_elevator_floors("TakeElevator(0->1)"), (0, 1))
        self.assertEqual(diagnostics.parse_elevator_floors("TakeElevator(2->0)"), (2, 0))
        self.assertIsNone(diagnostics.parse_elevator_floors("NavigateToPose"))

    def test_select_relevant_elevator_and_threshold_poses(self):
        names = [
            "a1_gazebo",
            "dynamic_elevator_main",
            "dynamic_elevator_floor_1::left_panel",
            "competition_building::elevator_threshold_floor_1",
            "floor_1_room_0_desk",
        ]
        poses = [_pose(float(index), 0.0, 0.0) for index in range(len(names))]

        selected = diagnostics.select_relevant_poses(names, poses)

        self.assertEqual(
            set(selected),
            {
                "a1_gazebo",
                "dynamic_elevator_main",
                "dynamic_elevator_floor_1::left_panel",
                "competition_building::elevator_threshold_floor_1",
            },
        )
        self.assertEqual(selected["dynamic_elevator_main"]["x"], 1.0)

    def test_select_link_states_keeps_model_scoped_a1_links_and_velocity(self):
        names = [
            "a1_gazebo::trunk",
            "a1_gazebo::FR_foot",
            "a1_gazebo::FL_calf",
            "a1_gazebo::RR_thigh",
            "generated_building::elevator_threshold_floor_1",
            "generated_building::floor_1_wall",
        ]
        poses = [_pose(float(index), 0.0, 2.6) for index in range(len(names))]
        twists = [_twist(-0.1 * index, 0.2 * index) for index in range(len(names))]

        selected = diagnostics.select_relevant_link_states(names, poses, twists)

        self.assertEqual(
            set(selected),
            {
                "a1_gazebo::trunk",
                "a1_gazebo::FR_foot",
                "a1_gazebo::FL_calf",
                "a1_gazebo::RR_thigh",
                "generated_building::elevator_threshold_floor_1",
            },
        )
        self.assertAlmostEqual(selected["a1_gazebo::FR_foot"]["linear_x"], -0.1)
        self.assertAlmostEqual(selected["a1_gazebo::RR_thigh"]["angular_z"], 0.6)

    def test_select_link_states_preserves_duplicate_gazebo_link_names(self):
        names = ["a1_gazebo::FR_foot", "a1_gazebo::FR_foot"]
        poses = [_pose(1.2, 2.6, 2.62), _pose(14.0, -15.0, 0.02)]
        twists = [_twist(-0.2), _twist(0.0)]

        selected = diagnostics.select_relevant_link_states(names, poses, twists)

        self.assertEqual(
            set(selected),
            {"a1_gazebo::FR_foot", "a1_gazebo::FR_foot#2"},
        )
        self.assertEqual(selected["a1_gazebo::FR_foot"]["x"], 1.2)
        self.assertEqual(selected["a1_gazebo::FR_foot#2"]["x"], 14.0)

    def test_forward_cloud_summary_uses_robot_frame_and_reports_step_height(self):
        # 机器人朝 +X；前方近区同时包含楼板 z=2.60 和门槛 z=2.66。
        points = [
            (1.2, 2.6, 2.60),
            (1.4, 2.6, 2.60),
            (1.6, 2.6, 2.66),
            (1.8, 2.6, 2.66),
            (1.6, 3.3, 3.20),  # 超出前方足迹宽度，不应纳入。
            (0.7, 2.6, 2.60),  # 位于机器人后方，不应纳入。
        ]

        summary = diagnostics.summarize_forward_cloud(
            points,
            robot_xyz_yaw=(1.0, 2.6, 2.91, 0.0),
            max_forward=1.0,
            half_width=0.45,
        )

        self.assertEqual(summary["point_count"], 4)
        self.assertAlmostEqual(summary["world_z_min"], 2.60)
        self.assertAlmostEqual(summary["world_z_max"], 2.66)
        self.assertAlmostEqual(summary["world_z_span"], 0.06)
        self.assertEqual(len(summary["forward_bins"]), 4)

    def test_depth_summary_normalizes_uint16_millimetres_and_ignores_invalid(self):
        depth_mm = np.array(
            [
                [0, 1000, 2000, 65535],
                [500, 1500, 2500, 0],
            ],
            dtype=np.uint16,
        )

        summary = diagnostics.summarize_depth_array(depth_mm, "16UC1")

        self.assertEqual(summary["valid_count"], 5)
        self.assertAlmostEqual(summary["min_m"], 0.5)
        self.assertAlmostEqual(summary["median_m"], 1.5)
        self.assertAlmostEqual(summary["max_m"], 2.5)

    def test_exit_outcome_requires_observed_exit_phase(self):
        tracker = diagnostics.ExitOutcomeTracker()
        tracker.observe_status(
            {"current_task": "TakeElevator(0->1)", "task_phase": "ENTER_CABIN"}
        )
        tracker.observe_log("[TakeElevator(0->1)] 离开轿厢失败。")
        self.assertIsNone(tracker.outcome)

        tracker.observe_status(
            {"current_task": "TakeElevator(0->1)", "task_phase": "EXIT_CABIN"}
        )
        tracker.observe_log("[TakeElevator(0->1)] 离开轿厢失败。")
        self.assertEqual(tracker.outcome, "exit_cabin_failed")

    def test_exit_outcome_detects_successful_phase_advance(self):
        tracker = diagnostics.ExitOutcomeTracker()
        tracker.observe_status(
            {"current_task": "TakeElevator(0->1)", "task_phase": "EXIT_CABIN"}
        )
        tracker.observe_status(
            {
                "current_task": "TakeElevator(0->1)",
                "task_phase": "CLOSE_TARGET_FLOOR_DOOR",
            }
        )

        self.assertEqual(tracker.outcome, "exit_cabin_succeeded")

    def test_exit_outcome_detects_first_stuck_from_status_counter(self):
        tracker = diagnostics.ExitOutcomeTracker()
        tracker.observe_status(
            {
                "current_task": "TakeElevator(0->1)",
                "task_phase": "EXIT_CABIN",
                "controller": {"stuck_count": 2},
            }
        )
        tracker.observe_status(
            {
                "current_task": "TakeElevator(0->1)",
                "task_phase": "EXIT_CABIN",
                "controller": {"stuck_count": 3},
            }
        )

        self.assertEqual(tracker.outcome, "exit_cabin_first_stuck")

    def test_exit_outcome_classifies_failure_logs(self):
        cases = {
            "[TakeElevator(0->1)] 离开轿厢首次尝试卡住，实验配置禁用倒车恢复。": (
                "exit_cabin_first_stuck"
            ),
            "[TakeElevator(0->1)] 离开轿厢倒车恢复失败：无有效位移。": (
                "exit_cabin_backoff_failed"
            ),
            "[TakeElevator(0->1)] 离开轿厢失败。": "exit_cabin_failed",
        }
        for message, expected in cases.items():
            with self.subTest(message=message):
                tracker = diagnostics.ExitOutcomeTracker()
                tracker.observe_status(
                    {
                        "current_task": "TakeElevator(0->1)",
                        "task_phase": "EXIT_CABIN",
                    }
                )
                tracker.observe_log(message)
                self.assertEqual(tracker.outcome, expected)


if __name__ == "__main__":
    unittest.main()

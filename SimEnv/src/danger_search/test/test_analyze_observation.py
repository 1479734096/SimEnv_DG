"""观察数据离线分析器的单元测试。"""
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


TOOL_PATH = Path(__file__).resolve().parents[1] / "tools" / "analyze_observation.py"
SPEC = importlib.util.spec_from_file_location("analyze_observation", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
analyzer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analyzer
SPEC.loader.exec_module(analyzer)


class AnalyzeObservationTest(unittest.TestCase):
    def test_quaternion_to_rpy_reports_roll_pitch_yaw(self):
        quaternion = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        self.assertEqual(analyzer.rpy_from_quaternion(quaternion), (0.0, 0.0, 0.0))

    def test_flatten_exit_sample_keeps_robot_elevator_door_and_contact_evidence(self):
        sample = {
            "sim_time": 68.5,
            "status": {
                "current_task": "TakeElevator(0->1)",
                "task_phase": "EXIT_CABIN",
                "floor": 1,
            },
            "elevator_floors": [0, 1],
            "odom": {"x": 1.4, "y": 2.6, "z": 2.91, "yaw": 3.14, "linear_x": 0.01},
            "cmd_vel": {"linear_x": 0.35, "angular_z": -0.04},
            "model_poses": {"dynamic_elevator_main": {"z": 3.63}},
            "link_poses": {
                "dynamic_elevator_floor_1::right_panel": {"x": 2.0, "y": 1.4, "z": 2.6},
                "dynamic_elevator_floor_1::left_panel": {"x": 2.0, "y": 3.8, "z": 2.6},
            },
            "foot_contacts": {"FR": {"force_norm": 32.0, "force": [1.0, 2.0, 31.0]}},
            "forward_cloud": {"point_count": 40, "world_z_span": 0.06},
            "depth_roi": {"valid_count": 100, "median_m": 0.8},
            "exit_outcome": None,
        }

        row = analyzer.flatten_exit_sample(sample)

        self.assertEqual(row["robot_z"], 2.91)
        self.assertEqual(row["elevator_car_z"], 3.63)
        self.assertEqual(row["target_door_panel_count"], 2)
        self.assertEqual(row["target_left_panel_y"], 3.8)
        self.assertEqual(row["target_right_panel_y"], 1.4)
        self.assertEqual(row["FR_force_norm"], 32.0)
        self.assertEqual(row["forward_world_z_span"], 0.06)
        self.assertEqual(row["depth_median_m"], 0.8)

    def test_flatten_exit_sample_reports_link_geometry_target_and_recover_phase(self):
        sample = {
            "sim_time": 82.0,
            "status": {
                "current_task": "TakeElevator(0->1)",
                "task_phase": "EXIT_CABIN",
                "floor": 1,
                "controller": {
                    "recover_state": "settle",
                    "recover_reason": "exit_cabin_stuck",
                    "recover_remaining": 0.3,
                    "stuck_count": 1,
                    "recover_count": 1,
                },
            },
            "elevator_floors": [0, 1],
            "odom": {"x": 1.4, "y": 2.6, "z": 2.91, "yaw": 3.14},
            "cmd_vel": {"linear_x": 0.42, "angular_z": 0.1},
            "followed_path": {"goal_xy": [1.05, 2.65]},
            "local_target": {"x": 1.1, "y": 2.64},
            "link_states": {
                "generated_building::elevator_threshold_floor_1": {
                    "x": 1.54,
                    "y": 2.6,
                    "z": 2.63,
                },
                "a1_gazebo::trunk": {
                    "x": 1.4,
                    "y": 2.6,
                    "z": 3.0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 3.14,
                    "linear_x": -0.2,
                    "linear_y": 0.0,
                    "linear_z": 0.0,
                    "angular_x": 0.1,
                    "angular_y": -0.2,
                    "angular_z": 0.3,
                },
                "a1_gazebo::FR_foot": {
                    "x": 1.5,
                    "y": 2.5,
                    "z": 2.68,
                    "linear_x": -0.3,
                    "linear_y": 0.0,
                    "linear_z": 0.0,
                },
            },
        }

        row = analyzer.flatten_exit_sample(sample)

        self.assertAlmostEqual(row["threshold_top_z"], 2.66)
        self.assertAlmostEqual(row["trunk_bottom_relative_threshold_top"], 0.283)
        self.assertAlmostEqual(row["FR_foot_bottom_relative_threshold_top"], 0.0)
        self.assertEqual(row["exit_target_x"], 1.05)
        self.assertEqual(row["local_target_x"], 1.1)
        self.assertEqual(row["recover_state"], "settle")
        self.assertEqual(row["trunk_linear_x"], -0.2)
        self.assertEqual(row["trunk_angular_y"], -0.2)

    def test_add_exit_derivatives_uses_link_velocity_and_sim_time(self):
        rows = [
            {"sim_time": 10.0, "trunk_linear_x": -0.2},
            {"sim_time": 10.1, "trunk_linear_x": -0.4},
        ]

        analyzer.add_exit_derivatives(rows)

        self.assertEqual(rows[0]["trunk_linear_ax"], "")
        self.assertAlmostEqual(rows[1]["trunk_linear_ax"], -2.0)

    def test_flatten_exit_sample_uses_base_when_gazebo_lumps_trunk_link(self):
        sample = {
            "sim_time": 82.0,
            "status": {"current_task": "TakeElevator(0->1)", "task_phase": "EXIT_CABIN"},
            "elevator_floors": [0, 1],
            "link_states": {
                "generated_building::elevator_threshold_floor_1": {"z": 2.63},
                "a1_gazebo::base": {
                    "x": 1.4,
                    "y": 2.6,
                    "z": 3.0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 3.14,
                    "linear_x": -0.2,
                },
            },
        }

        row = analyzer.flatten_exit_sample(sample)

        self.assertEqual(row["body_link_name"], "a1_gazebo::base")
        self.assertEqual(row["trunk_x"], 1.4)
        self.assertEqual(row["trunk_linear_x"], -0.2)

    def test_flatten_rejects_implausible_duplicate_foot_and_derives_from_calf(self):
        sample = {
            "sim_time": 82.0,
            "status": {"current_task": "TakeElevator(0->1)", "task_phase": "EXIT_CABIN"},
            "elevator_floors": [0, 1],
            "link_states": {
                "generated_building::elevator_threshold_floor_1": {"z": 2.63},
                "a1_gazebo::FR_calf": {
                    "x": 1.6,
                    "y": 2.6,
                    "z": 2.88,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "linear_x": -0.2,
                    "linear_y": 0.0,
                    "linear_z": 0.0,
                    "angular_x": 0.0,
                    "angular_y": 0.0,
                    "angular_z": 0.0,
                },
                "a1_gazebo::FR_foot": {"x": 14.0, "y": -15.0, "z": 0.02},
            },
        }

        row = analyzer.flatten_exit_sample(sample)

        self.assertEqual(row["FR_foot_position_source"], "calf_kinematic_fallback")
        self.assertAlmostEqual(row["FR_foot_x"], 1.6)
        self.assertAlmostEqual(row["FR_foot_z"], 2.68)
        self.assertAlmostEqual(row["FR_foot_bottom_relative_threshold_top"], 0.0)

    def test_time_breakdown_reports_overlapping_motion_and_elevator_categories(self):
        trajectory = [
            {
                "timestamp": 0.0,
                "task": "TakeElevator(0->1)",
                "task_phase": "OPEN_TARGET_FLOOR_DOOR",
                "actual_linear_speed": 0.0,
                "actual_angular_speed": 0.0,
                "cmd_linear_x": 0.0,
                "cmd_linear_y": 0.0,
                "cmd_angular_z": 0.0,
                "recover_state": "idle",
            },
            {
                "timestamp": 1.0,
                "task": "TakeElevator(0->1)",
                "task_phase": "EXIT_CABIN",
                "actual_linear_speed": 0.0,
                "actual_angular_speed": 0.0,
                "cmd_linear_x": 0.4,
                "cmd_linear_y": 0.0,
                "cmd_angular_z": 0.1,
                "recover_state": "idle",
            },
            {
                "timestamp": 2.0,
                "task": "TakeElevator(0->1)",
                "task_phase": "EXIT_BACKOFF_REVERSE",
                "actual_linear_speed": 0.2,
                "actual_angular_speed": 0.1,
                "cmd_linear_x": 0.4,
                "cmd_linear_y": 0.0,
                "cmd_angular_z": 0.1,
                "recover_state": "turn",
            },
            {
                "timestamp": 3.0,
                "task": "TakeElevator(0->1)",
                "task_phase": "EXIT_CABIN",
                "actual_linear_speed": 0.0,
                "actual_angular_speed": 0.4,
                "cmd_linear_x": 0.0,
                "cmd_linear_y": 0.0,
                "cmd_angular_z": 0.5,
                "recover_state": "turn",
            },
            {
                "timestamp": 4.0,
                "task": "",
                "task_phase": "",
                "actual_linear_speed": 0.2,
                "actual_angular_speed": 0.0,
                "cmd_linear_x": 0.3,
                "cmd_linear_y": 0.0,
                "cmd_angular_z": 0.0,
                "recover_state": "idle",
            },
        ]
        events = [{"event": "frontier_selection", "selection_wall_duration": 0.12}]

        breakdown = analyzer.summarize_time_breakdown(trajectory, events)

        self.assertAlmostEqual(breakdown["exit_cabin_duration"], 3.0)
        self.assertAlmostEqual(breakdown["elevator_door_wait_duration"], 1.0)
        self.assertAlmostEqual(breakdown["commanded_no_motion_duration"], 1.0)
        self.assertAlmostEqual(breakdown["effective_translation_duration"], 1.0)
        self.assertAlmostEqual(breakdown["recover_duration"], 2.0)
        self.assertAlmostEqual(breakdown["rotate_in_place_duration"], 1.0)
        self.assertAlmostEqual(breakdown["planning_wall_duration"], 0.12)

    def test_parse_rl_status_reports_command_received_by_policy(self):
        parsed = analyzer.parse_rl_status_command(
            "in_rl=1 policy_loaded=1 cmd_timed_out=0 cmd_clamped=0 cmd=(0.25,-0.01,0.2)"
        )

        self.assertEqual(parsed["linear_x"], 0.25)
        self.assertEqual(parsed["linear_y"], -0.01)
        self.assertEqual(parsed["angular_z"], 0.2)
        self.assertFalse(parsed["timed_out"])
        self.assertFalse(parsed["clamped"])

    def test_exit_ab_metrics_cover_threshold_stall_posture_and_foot_crossing(self):
        rows = [
            {
                "sim_time": 10.0,
                "phase": "EXIT_CABIN",
                "robot_x": 1.60,
                "robot_y": 2.60,
                "robot_roll": 0.05,
                "robot_pitch": -0.04,
                "odom_linear_x": -0.10,
                "odom_linear_y": 0.0,
                "cmd_linear_x": 0.25,
                "exit_target_x": 1.05,
                "threshold_x": 1.54,
                "FR_foot_x": 1.50,
                "FL_foot_x": 1.50,
                "RR_foot_x": 1.75,
                "RL_foot_x": 1.75,
            },
            {
                "sim_time": 11.0,
                "phase": "EXIT_CABIN",
                "robot_x": 1.45,
                "robot_y": 2.62,
                "robot_roll": 0.10,
                "robot_pitch": -0.08,
                "odom_linear_x": -0.01,
                "odom_linear_y": 0.01,
                "cmd_linear_x": 0.25,
                "exit_target_x": 1.05,
                "threshold_x": 1.54,
                "FR_foot_x": 1.40,
                "FL_foot_x": 1.39,
                "RR_foot_x": 1.65,
                "RL_foot_x": 1.66,
            },
            {
                "sim_time": 12.0,
                "phase": "EXIT_CABIN",
                "robot_x": 1.38,
                "robot_y": 2.59,
                "robot_roll": -0.12,
                "robot_pitch": 0.09,
                "odom_linear_x": -0.15,
                "odom_linear_y": -0.01,
                "cmd_linear_x": 0.25,
                "exit_target_x": 1.05,
                "threshold_x": 1.54,
                "FR_foot_x": 1.30,
                "FL_foot_x": 1.31,
                "RR_foot_x": 1.42,
                "RL_foot_x": 1.41,
            },
            {
                "sim_time": 13.0,
                "phase": "EXIT_CABIN",
                "robot_x": 1.41,
                "robot_y": 2.60,
                "robot_roll": 0.04,
                "robot_pitch": 0.03,
                "odom_linear_x": 0.03,
                "odom_linear_y": 0.0,
                "cmd_linear_x": 0.25,
                "exit_target_x": 1.05,
                "threshold_x": 1.54,
                "FR_foot_x": 1.33,
                "FL_foot_x": 1.34,
                "RR_foot_x": 1.40,
                "RL_foot_x": 1.39,
                "exit_outcome": "exit_cabin_succeeded",
            },
        ]
        received = [
            {"timestamp": 10.5, "linear_x": 0.25, "linear_y": 0.0, "angular_z": 0.1},
            {"timestamp": 11.5, "linear_x": 0.25, "linear_y": 0.0, "angular_z": 0.1},
        ]

        metrics = analyzer.summarize_exit_ab_metrics(rows, received)

        self.assertAlmostEqual(metrics["exit_first_attempt_duration"], 3.0)
        self.assertAlmostEqual(metrics["threshold_region_duration"], 2.0)
        self.assertAlmostEqual(metrics["commanded_no_x_motion_duration"], 1.0)
        self.assertAlmostEqual(metrics["front_to_rear_crossing_duration"], 1.0)
        self.assertAlmostEqual(metrics["max_abs_roll"], 0.12)
        self.assertAlmostEqual(metrics["max_abs_pitch"], 0.09)
        self.assertAlmostEqual(metrics["rebound_distance"], 0.03)
        self.assertEqual(metrics["rebound_count"], 1)
        self.assertAlmostEqual(metrics["received_cmd_linear_max"], 0.25)


if __name__ == "__main__":
    unittest.main()

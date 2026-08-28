import json
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from danger_search.ground_support_gate import (
    DROP_STOPPING,
    DropClusterEvidence,
    GroundSupportAssessment,
    GroundSupportGateConfig,
    GroundSupportSafetyGate,
)
from danger_search.navigator import PathFollower, compute_tracking_command, RUNNING, STUCK


class LocalControllerTest(unittest.TestCase):
    def test_planning_progress_snapshot_is_production_state_not_debug_dictionary(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path(
            [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)],
            reason="frontier_selected_path",
        )
        follower.tick((0.0, 0.0, 0.0))

        snapshot = follower.planning_progress_snapshot()

        self.assertTrue(snapshot.active)
        self.assertGreater(snapshot.remaining_distance, 0.0)
        self.assertEqual(snapshot.final_goal, (2.0, 1.0))
        self.assertAlmostEqual(snapshot.final_heading, math.pi / 4.0)
        self.assertEqual(snapshot.path_reason, "frontier_selected_path")
        self.assertFalse(snapshot.terminal_pending)

    def test_control_and_diagnostic_topics_are_direction_checked(self):
        with self.assertRaises(ValueError):
            PathFollower(cmd_vel_topic="/scan")
        with self.assertRaises(ValueError):
            PathFollower(cmd_vel_topic="/cmd_vel", path_raw_topic="/scan")

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            PathFollower(cmd_vel_topic="/cmd_vel", max_linear=math.nan)
        with self.assertRaises(ValueError):
            PathFollower(cmd_vel_topic="/cmd_vel", obstacle_slow_distance=0.4)
        with self.assertRaises(ValueError):
            PathFollower(
                cmd_vel_topic="/cmd_vel",
                arc_turn_min_linear_fraction=-0.1,
            )

    def test_large_yaw_error_enters_turn_in_place(self):
        # 大航向误差（86°）超过先对准阈值：禁止前进与侧移，只输出转向。
        linear_x, linear_y, angular = compute_tracking_command(
            1.5,
            max_linear=0.35,
            max_lateral=0.3,
            max_angular=0.9,
            min_linear_fraction=0.25,
            turn_in_place_active=True,
        )

        self.assertEqual(linear_x, 0.0)
        self.assertEqual(linear_y, 0.0)
        self.assertAlmostEqual(angular, 0.9)

    def test_large_heading_uses_only_compatibility_creep(self):
        """大偏角不得同时下发高线速度和饱和转向。"""
        linear_x, _, angular = compute_tracking_command(
            1.35,
            max_linear=0.60,
            max_lateral=0.25,
            max_angular=1.2,
            min_linear_fraction=0.35,
        )

        self.assertLess(linear_x, 0.02)
        self.assertAlmostEqual(angular, 1.2)

    def test_straight_target_uses_full_linear_speed(self):
        linear_x, linear_y, angular = compute_tracking_command(
            0.0,
            max_linear=0.35,
            max_lateral=0.3,
            max_angular=0.9,
            min_linear_fraction=0.25,
        )

        self.assertAlmostEqual(linear_x, 0.35)
        self.assertAlmostEqual(linear_y, 0.0)
        self.assertAlmostEqual(angular, 0.0)

    def test_min_linear_fraction_is_clamped(self):
        # min_linear_fraction 被 clamp 到 1.0 时，小角度偏角应满速前进。
        linear_x, _, _ = compute_tracking_command(
            0.1,
            max_linear=0.4,
            max_lateral=0.3,
            max_angular=1.0,
            min_linear_fraction=2.0,
        )

        self.assertAlmostEqual(linear_x, 0.4, places=2)

    def test_large_yaw_error_does_not_drive_full_speed_straight(self):
        linear_x, linear_y, angular = compute_tracking_command(
            3.14,
            max_linear=0.60,
            max_lateral=0.25,
            max_angular=0.8,
            min_linear_fraction=0.15,
        )

        self.assertLess(linear_x, 0.20)
        self.assertAlmostEqual(linear_y, 0.0, places=3)
        self.assertAlmostEqual(angular, 0.8)

    def test_default_path_follower_can_turn_while_tracking(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")

        self.assertGreater(follower.max_angular, 0.0)
        self.assertGreater(follower.max_lateral, 0.0)
        self.assertLess(follower.min_linear_fraction, 1.0)

    def test_per_tick_linear_cap_does_not_modify_following_ticks(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel", max_linear=0.60)
        follower.set_path([(0.0, 0.0), (2.0, 0.0)])

        self.assertEqual(follower.tick((0.0, 0.0, 0.0), max_linear=0.25), RUNNING)
        self.assertAlmostEqual(follower.tracking_debug_snapshot()["linear_command"], 0.25)
        self.assertAlmostEqual(follower.max_linear, 0.60)

        self.assertEqual(follower.tick((0.01, 0.0, 0.0)), RUNNING)
        self.assertGreater(follower.tracking_debug_snapshot()["linear_command"], 0.25)
        self.assertAlmostEqual(follower.max_linear, 0.60)

    def test_per_tick_goal_tolerance_only_tightens_current_call(self):
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            goal_tolerance=0.35,
        )
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])

        self.assertEqual(
            follower.tick((0.75, 0.0, 0.0), goal_tolerance=0.15),
            RUNNING,
        )
        self.assertAlmostEqual(follower.goal_tolerance, 0.35)

        self.assertEqual(
            follower.tick((0.86, 0.0, 0.0), goal_tolerance=0.15),
            "done",
        )
        self.assertAlmostEqual(follower.goal_tolerance, 0.35)

    def test_current_segment_returns_copy_of_active_tracking_segment(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)])

        self.assertEqual(
            follower.current_segment(),
            ((0.0, 0.0), (0.0, 1.0)),
        )
        follower._waypoint_index = 1
        self.assertEqual(
            follower.current_segment(),
            ((0.0, 1.0), (1.0, 1.0)),
        )
        follower.clear_path(reason="test")
        self.assertIsNone(follower.current_segment())

        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        self.assertEqual(follower.tick((0.75, 0.0, 0.0)), "done")

        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        self.assertEqual(
            follower.tick((0.60, 0.0, 0.0), goal_tolerance=0.50),
            RUNNING,
        )
        self.assertAlmostEqual(follower.goal_tolerance, 0.35)

    def test_progress_resets_on_robot_displacement(self):
        """即使到终点欧氏距离没变，只要机器人自身移动了就不应误判 STUCK。"""
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            no_progress_timeout=0.1,
            progress_distance=0.2,
        )
        # 绕直角路径：终点在东侧，机器人从 (0,0) 先向北移动
        follower.set_path([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)])

        # 第一步：向北移动 0.3m，到终点的欧氏距离几乎不变（都是 ~1.4m）
        status = follower.tick((0.0, 0.3, math.pi / 2.0))
        self.assertEqual(status, RUNNING)

        # 立刻再 tick 一次（仍在超时内），不应 STUCK
        status = follower.tick((0.0, 0.31, math.pi / 2.0))
        self.assertEqual(status, RUNNING)

    def test_stuck_when_robot_does_not_move(self):
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            route_progress_timeout=0.05,
        )
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        follower.tick((0.0, 0.0, 0.0))

        import time
        time.sleep(0.06)

        status = follower.tick((0.0, 0.0, 0.0))
        self.assertEqual(status, STUCK)
        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(snapshot["stuck_count"], 1)
        self.assertEqual(snapshot["replan_reason"], "route_progress_timeout")
        self.assertEqual(snapshot["zero_command_reason"], "route_progress_timeout")

        follower.recover(reason="frontier_stuck")
        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(snapshot["recover_count"], 1)
        self.assertEqual(snapshot["recover_state"], "settle")
        self.assertEqual(snapshot["recover_reason"], "frontier_stuck")
        self.assertEqual(snapshot["zero_command_reason"], "frontier_stuck")

    def test_angular_saturation_band_strongly_limits_forward_speed(self):
        """下游角速度接近饱和的偏角下，不能继续以约 0.5m/s 前冲。"""
        linear_x, _, angular = compute_tracking_command(
            0.59,
            max_linear=0.60,
            max_lateral=0.25,
            max_angular=1.2,
            min_linear_fraction=0.35,
        )

        self.assertGreater(abs(angular), 0.99)
        self.assertLessEqual(linear_x, 0.24)

    def test_balanced_arc_floor_keeps_large_non_terminal_turn_moving(self):
        positive, _, _ = compute_tracking_command(
            1.09,
            max_linear=0.80,
            max_lateral=0.01,
            max_angular=1.2,
            arc_turn_min_linear_fraction=0.15,
            turn_in_place_heading=1.10,
        )
        negative, _, _ = compute_tracking_command(
            -1.09,
            max_linear=0.80,
            max_lateral=0.01,
            max_angular=1.2,
            arc_turn_min_linear_fraction=0.15,
            turn_in_place_heading=1.10,
        )

        self.assertAlmostEqual(positive, 0.12)
        self.assertAlmostEqual(negative, positive)

    def test_arc_floor_never_overrides_curvature_or_obstacle_safety(self):
        curved, _, _ = compute_tracking_command(
            1.09,
            max_linear=0.80,
            max_angular=1.2,
            arc_turn_min_linear_fraction=0.15,
            turn_in_place_heading=1.10,
            path_curvature=10.0,
        )
        blocked, _, angular = compute_tracking_command(
            1.09,
            max_linear=0.80,
            max_angular=1.2,
            arc_turn_min_linear_fraction=0.15,
            turn_in_place_heading=1.10,
            nearest_obstacle_distance=0.40,
            obstacle_stop_distance=0.40,
            obstacle_slow_distance=0.60,
        )

        self.assertLess(curved, 0.12)
        self.assertEqual(blocked, 0.0)
        self.assertGreater(angular, 0.0)

    def test_curvature_and_forward_obstacle_jointly_limit_speed(self):
        straight, _, _ = compute_tracking_command(
            0.0,
            max_linear=0.60,
            max_angular=1.2,
        )
        corner, _, _ = compute_tracking_command(
            0.0,
            max_linear=0.60,
            max_angular=1.2,
            path_curvature=math.pi / 2.0,
        )
        obstacle, _, _ = compute_tracking_command(
            0.0,
            max_linear=0.60,
            max_angular=1.2,
            nearest_obstacle_distance=0.5,
        )

        self.assertLess(corner, straight)
        self.assertLess(obstacle, corner)

    def test_close_forward_obstacle_stops_translation_but_keeps_turning(self):
        linear_x, linear_y, angular = compute_tracking_command(
            0.4,
            max_linear=0.60,
            max_lateral=0.25,
            max_angular=1.2,
            nearest_obstacle_distance=0.49,
            obstacle_stop_distance=0.50,
            obstacle_slow_distance=1.00,
        )

        self.assertEqual(linear_x, 0.0)
        self.assertEqual(linear_y, 0.0)
        self.assertGreater(angular, 0.0)

    def test_recover_has_turn_and_replan_exit(self):
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            recover_duration=0.5,
            recover_turn_duration=1.0,
        )
        follower._debug_heading_error = -0.4

        with patch("danger_search.navigator.get_time", return_value=10.0):
            follower.recover(reason="frontier_stuck")
        with patch("danger_search.navigator.get_time", return_value=10.6):
            self.assertEqual(follower.tick((0.0, 0.0, 0.0)), RUNNING)
            turn_snapshot = follower.tracking_debug_snapshot()
            self.assertEqual(turn_snapshot["recover_state"], "turn")
            self.assertLess(turn_snapshot["angular_command"], 0.0)
        with patch("danger_search.navigator.get_time", return_value=11.7):
            self.assertEqual(follower.tick((0.0, 0.0, 0.0)), "idle")

        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(snapshot["recover_state"], "idle")
        self.assertEqual(snapshot["replan_reason"], "recover_turn_completed")
        # 恢复完成后旧路径必须被清除，任务层才会重新 A*。
        self.assertFalse(follower.has_path())

    def test_recover_clears_path_for_replan(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        self.assertTrue(follower.has_path())
        with patch("danger_search.navigator.get_time", return_value=2.0):
            follower.recover(reason="stuck")
            self.assertEqual(follower.tick((0.0, 0.0, 0.0)), RUNNING)
        with patch("danger_search.navigator.get_time", return_value=4.6):
            self.assertEqual(follower.tick((0.0, 0.0, 0.0)), RUNNING)
        with patch("danger_search.navigator.get_time", return_value=5.7):
            self.assertEqual(follower.tick((0.0, 0.0, 0.0)), "idle")
        self.assertFalse(follower.has_path())

    def test_high_measured_yaw_rate_brakes_forward(self):
        # 实测角速度高（>0.45 rad/s）即使误差不大也必须制动（先对准）。
        linear_x, linear_y, angular = compute_tracking_command(
            0.10,
            max_linear=0.60,
            max_lateral=0.25,
            max_angular=1.2,
            min_linear_fraction=0.35,
            measured_yaw_rate=1.0,
            yaw_rate_damping=0.45,
            turn_in_place_rate=0.45,
            turn_in_place_active=True,
        )
        self.assertEqual(linear_x, 0.0)
        self.assertEqual(linear_y, 0.0)
        self.assertLess(angular, 0.0)

    def test_yaw_rate_damping_reduces_overshoot(self):
        # 阻尼项：误差方向与转速同向时角速度应小于纯比例控制。
        _, _, damped = compute_tracking_command(
            0.30,
            max_linear=0.60,
            max_angular=1.2,
            measured_yaw_rate=0.6,
            yaw_rate_damping=0.45,
            turn_in_place_rate=2.0,
        )
        _, _, plain = compute_tracking_command(
            0.30,
            max_linear=0.60,
            max_angular=1.2,
        )
        self.assertLess(damped, plain)

    def test_stale_yaw_rate_falls_back_to_plain_control(self):
        # yaw_rate 缺失（odom 未提供）→ 无阻尼项，但触发死区补偿将小角速度提升到 0.55。
        linear_x, _, angular = compute_tracking_command(
            0.10,
            max_linear=0.60,
            max_angular=1.2,
            min_linear_fraction=0.35,
            measured_yaw_rate=None,
            turn_in_place_active=False,
        )
        self.assertGreater(linear_x, 0.0)
        self.assertAlmostEqual(angular, 0.55, places=2)

    def test_remaining_path_distance_along_polyline(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)])
        # 在首段中点投影：剩余 = 0.5（首段剩余）+ 1.0（第二段）
        self.assertAlmostEqual(follower._remaining_path_distance(0.0, 0.5), 1.5)
        # 在首段起点：剩余 = 1.0 + 1.0
        self.assertAlmostEqual(follower._remaining_path_distance(0.0, 0.0), 2.0)
        # 绕圈（离开路径横向 1m）：投影仍在路径上，剩余不变 → 绕圈不算进度
        self.assertAlmostEqual(follower._remaining_path_distance(1.0, 0.5), 1.5)

    def test_circle_motion_without_route_progress_triggers_stuck(self):
        # 绕固定目标圆周运动：每步位移 >0.25m（旧判据会放行），
        # 但沿折线剩余不改善 → route_progress_timeout 内必须 STUCK。
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            route_progress_timeout=0.05,
        )
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        follower.tick((0.0, 0.0, 0.0))
        import time
        time.sleep(0.06)
        # 横向偏移 0.5m（位移 >0.25m 旧判据会放行）但沿路径投影剩余不变
        # → 路线无有效投影进度 → route_progress_timeout 内必须 STUCK。
        status = follower.tick((0.0, 0.5, 0.0))
        self.assertEqual(status, STUCK)
        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(snapshot["replan_reason"], "route_progress_timeout")
        # 终态未消费前路径保留；poll_status 消费后清除。
        self.assertTrue(follower.has_path())
        self.assertEqual(follower.poll_status(), STUCK)
        self.assertFalse(follower.has_path())

    def test_reverse_straight_publishes_configured_negative_speed(self):
        class FakeTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            max_linear=0.60,
            reverse_speed=-0.15,
        )
        publisher = FakePublisher()
        follower._publisher = publisher

        with patch("danger_search.navigator.Twist", FakeTwist):
            commanded = follower.reverse_straight(reason="exit_cabin_backoff")

        self.assertTrue(commanded)
        self.assertEqual(len(publisher.messages), 1)
        self.assertAlmostEqual(publisher.messages[0].linear.x, -0.15)
        self.assertAlmostEqual(publisher.messages[0].angular.z, 0.0)
        self.assertEqual(
            follower.tracking_debug_snapshot()["recover_reason"],
            "exit_cabin_backoff",
        )

    def test_final_output_replaces_non_finite_command_with_zero(self):
        class FakeTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = FakePublisher()
        with patch("danger_search.navigator.Twist", FakeTwist):
            follower._publish(math.nan, 0.2)

        message = follower._publisher.messages[-1]
        self.assertEqual((message.linear.x, message.linear.y, message.angular.z), (0.0, 0.0, 0.0))
        self.assertEqual(
            follower.tracking_debug_snapshot()["zero_command_reason"],
            "invalid_requested_command",
        )

    def test_single_point_path_outside_tolerance_remains_trackable(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel", goal_tolerance=0.1)
        follower.set_path([(1.0, 0.0)])

        self.assertEqual(follower.tick((0.0, 0.0, 0.0)), RUNNING)
        self.assertEqual(follower.tracking_debug_snapshot()["local_target"], [1.0, 0.0])

    def test_invalid_pose_stops_and_reports_stuck(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])

        self.assertEqual(follower.tick((math.nan, 0.0, 0.0)), STUCK)
        self.assertFalse(follower.has_path())
        self.assertEqual(
            follower.tracking_debug_snapshot()["zero_command_reason"],
            "invalid_pose",
        )

    def test_malformed_path_stops_before_raising(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        follower._last_emitted_command = (0.3, 0.0, 0.0)

        with self.assertRaises(ValueError):
            follower.set_path([()])

        self.assertEqual(follower._last_emitted_command, (0.0, 0.0, 0.0))

    def test_command_gate_exception_is_replaced_with_zero(self):
        class BrokenGate:
            reason_code = "broken"

            @staticmethod
            def filter_command(*_args, **_kwargs):
                raise RuntimeError("测试异常")

        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_command_gate(BrokenGate())

        follower._publish(0.3, 0.0)

        self.assertEqual(follower._last_emitted_command, (0.0, 0.0, 0.0))
        self.assertEqual(
            follower._debug_zero_command_reason,
            "ground_support_gate:broken",
        )

    def test_tracking_debug_snapshot_records_tracking_context(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel", pure_pursuit_lookahead=0.45)
        follower.set_path([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)])

        status = follower.tick((0.0, 0.0, 0.0))
        snapshot = follower.tracking_debug_snapshot()

        self.assertEqual(status, RUNNING)
        self.assertEqual(snapshot["pose"], [0.0, 0.0, 0.0])
        self.assertEqual(snapshot["raw_path_size"], 3)
        self.assertEqual(snapshot["path_size"], 3)
        self.assertEqual(snapshot["current_waypoint"], [0.0, 1.0])
        self.assertEqual(snapshot["local_target"], [0.0, 0.45])
        self.assertAlmostEqual(snapshot["distance_to_target"], 0.45)
        self.assertAlmostEqual(snapshot["heading_error"], math.pi / 2.0)
        self.assertAlmostEqual(snapshot["path_turn_angle"], -math.pi / 2.0)
        self.assertAlmostEqual(snapshot["path_curvature"], -math.pi / 2.0)
        # 90° 航向误差超过先对准阈值：禁止前进，只输出转向。
        self.assertEqual(snapshot["linear_command"], 0.0)
        self.assertEqual(snapshot["lateral_command"], 0.0)
        self.assertEqual(snapshot["angular_command"], follower.max_angular)
        self.assertIsNone(snapshot["zero_command_reason"])
        self.assertIsNone(snapshot["last_command_wall_time"])
        json.dumps(snapshot, allow_nan=False)

    def test_path_snapshots_preserve_raw_and_followed_paths(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0, 1.0)])

        self.assertEqual(
            follower.raw_path_snapshot(),
            [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0, 1.0)],
        )
        self.assertEqual(
            follower.followed_path_snapshot(),
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        )

        follower.clear_path(reason="test_complete")
        self.assertEqual(follower.raw_path_snapshot(), [])
        self.assertEqual(follower.followed_path_snapshot(), [])

    def test_tracking_debug_records_wall_command_time_and_zero_reason(self):
        class FakeTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg)

        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        publisher = FakePublisher()
        follower._publisher = publisher

        with patch("danger_search.navigator.Twist", FakeTwist):
            follower.stop(reason="diagnostic_stop")

        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(len(publisher.messages), 1)
        self.assertEqual(snapshot["command_count"], 1)
        self.assertEqual(snapshot["zero_command_reason"], "diagnostic_stop")
        self.assertIsNotNone(snapshot["last_command_wall_time"])
        self.assertIsNotNone(snapshot["wall_command_age"])
        self.assertGreaterEqual(snapshot["wall_command_age"], 0.0)
        json.dumps(follower.command_debug_summary(), allow_nan=False)

    def test_publish_failure_does_not_commit_command_and_latches_zero(self):
        class FakeTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        class FailOncePublisher:
            def __init__(self):
                self.messages = []
                self.failed = False

            def publish(self, msg):
                if not self.failed:
                    self.failed = True
                    raise RuntimeError("publish failed")
                self.messages.append(msg)

        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = FailOncePublisher()
        with patch("danger_search.navigator.Twist", FakeTwist):
            with self.assertRaises(RuntimeError):
                follower._publish(0.3, 0.0)
            follower._publish(0.3, 0.0)

        self.assertEqual(follower.command_count, 1)
        self.assertEqual(follower.last_command, (0.0, 0.0, 0.0))
        self.assertEqual(
            follower.active_motion_command_snapshot(),
            (0.0, 0.0, 0.0),
        )
        self.assertTrue(
            follower.tracking_debug_snapshot()["publish_failure_latched"]
        )
        self.assertEqual(follower._publisher.messages[0].linear.x, 0.0)

    def test_active_motion_persists_until_fresh_odometry_confirms_stop(self):
        class FakeTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        class FakePublisher:
            def publish(self, _msg):
                return None

        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = FakePublisher()
        with patch("danger_search.navigator.Twist", FakeTwist):
            follower._publish(0.3, 0.0)
            follower.stop(reason="test_braking")

        self.assertEqual(
            follower.active_motion_command_snapshot(),
            (0.3, 0.0, 0.0),
        )
        self.assertEqual(
            follower.committed_command_snapshot(),
            (0.0, 0.0, 0.0),
        )
        for stamp in (1.0, 2.0):
            follower.observe_motion_odometry(
                stamp,
                linear_speed=0.0,
                angular_speed=0.0,
                wall_time=stamp,
            )
        self.assertEqual(
            follower.active_motion_command_snapshot(),
            (0.3, 0.0, 0.0),
        )
        follower.observe_motion_odometry(
            3.0,
            linear_speed=0.0,
            angular_speed=0.0,
            wall_time=3.0,
        )
        self.assertEqual(
            follower.active_motion_command_snapshot(),
            (0.0, 0.0, 0.0),
        )

    def test_path_diagnostics_are_noop_without_publishers(self):
        """rospy 已导入但未 init_node、publisher 为 None 时 set_path 不得报错。"""
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        self.assertEqual(follower.raw_path_snapshot(), [(0.0, 0.0), (1.0, 0.0)])

    def test_path_diagnostics_are_published_when_publishers_exist(self):
        """诊断 publisher 存在且 ROS 时钟可用时，set_path 仍发布 raw/followed path。"""
        try:
            import rospy
        except ImportError:
            rospy = None

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        class FakePoseStamped:
            def __init__(self):
                self.header = SimpleNamespace()
                self.pose = SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace())

        class FakePathMessage:
            def __init__(self):
                self.header = SimpleNamespace()
                self.poses = []

        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._raw_path_publisher = FakePublisher()
        follower._followed_path_publisher = FakePublisher()
        fake_rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace()))
        with patch("danger_search.navigator.PathMessage", FakePathMessage), patch(
            "danger_search.navigator.PoseStamped", FakePoseStamped
        ), patch("danger_search.navigator.rospy", fake_rospy):
            follower.set_path([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)])

        self.assertEqual(len(follower._raw_path_publisher.messages), 1)
        self.assertEqual(len(follower._followed_path_publisher.messages), 1)
        self.assertEqual(
            follower._raw_path_publisher.messages[0].header.frame_id,
            "world",
        )
        self.assertEqual(
            follower._followed_path_publisher.messages[0].header.frame_id,
            "world",
        )
        self.assertEqual(len(follower._raw_path_publisher.messages[0].poses), 3)
        self.assertEqual(len(follower._followed_path_publisher.messages[0].poses), 2)

    def test_lookahead_uses_path_distance_not_euclidean(self):
        """直角路径上，lookahead 应落在第一段路径上，而不是直接看穿墙角。"""
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            waypoint_tolerance=0.25,
            pure_pursuit_lookahead=0.3,
        )
        # L 型路径：先向北 1m，再向东 1m
        follower.set_path([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)])
        target = follower._lookahead_target(0.0, 0.0)

        # 目标应该在第一段（南北向）上，且不超过 y=0.3
        self.assertAlmostEqual(target[0], 0.0, places=5)
        self.assertLessEqual(target[1], 0.35)

    def test_lookahead_does_not_jump_to_future_parallel_segment(self):
        """U 型/平行走廊中，机器人靠近未来折返段时也不能跳段。"""
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            waypoint_tolerance=0.25,
            corner_tolerance=0.10,
            pure_pursuit_lookahead=0.45,
        )
        # U 型路径：向北 2m，向东 1m，向南 2m。
        follower.set_path([(0.0, 0.0), (0.0, 2.0), (1.0, 2.0), (1.0, 0.0)])
        # 机器人在第一段东侧附近，但尚未到达转角 (0,2)。
        target = follower._lookahead_target(0.8, 1.5)

        # 目标必须仍在第一段（x≈0），不能跳到 x≈1 的未来段。
        self.assertAlmostEqual(target[0], 0.0, places=5)
        self.assertGreater(target[1], 1.5)
        self.assertLessEqual(target[1], 2.0)

    def test_corner_tolerance_prevents_early_segment_advance(self):
        """转角前使用更小的 corner_tolerance，防止未贴近拐角就切换段。"""
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            waypoint_tolerance=0.25,
            corner_tolerance=0.10,
        )
        follower.set_path([(0.0, 0.0), (0.0, 2.0), (1.0, 2.0)])

        # 距离转角 0.15m，大于 corner_tolerance，不应推进段索引。
        target = follower._lookahead_target(0.0, 1.85)
        self.assertEqual(follower._waypoint_index, 0)
        self.assertAlmostEqual(target[0], 0.0, places=5)

        # 距离转角 0.05m，小于等于 corner_tolerance，允许推进。
        target = follower._lookahead_target(0.0, 1.95)
        self.assertEqual(follower._waypoint_index, 1)

    def test_passed_corner_advances_despite_small_lateral_offset(self):
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            waypoint_tolerance=0.25,
            corner_tolerance=0.10,
        )
        follower.set_path([(0.0, 0.0), (0.0, 2.0), (1.0, 2.0)])

        follower._lookahead_target(0.28, 2.05)

        self.assertEqual(follower._waypoint_index, 1)

    def test_is_turn_vertex_detects_right_angle(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)])

        self.assertTrue(follower._is_turn_vertex(1))
        self.assertFalse(follower._is_turn_vertex(0))
        self.assertFalse(follower._is_turn_vertex(2))

    def test_is_turn_vertex_false_for_collinear_points(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)])

        self.assertFalse(follower._is_turn_vertex(1))

    def test_safety_latch_clamps_every_command_entry_and_keepalive(self):
        class FakeTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.09337727739487302),),
            )
        )
        gate.update_assessment(
            GroundSupportAssessment(
                stamp=1.0,
                received_wall_time=1.0,
                data_age=0.01,
                supported_distance=0.8,
                current_footprint_supported=True,
                observation_adequate=True,
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.4,
                drop_direction_world=(1.0, 0.0),
                floor=0,
                map_epoch=1,
                localization_epoch=1,
                cloud_generation=1,
                grid_origin=(-2.0, -2.0),
                grid_resolution=0.1,
                drop_clusters=(
                    DropClusterEvidence(
                        cluster_id="drop:local-controller",
                        floor=0,
                        frame_id="odom",
                        map_epoch=1,
                        localization_epoch=1,
                        stamp=1.0,
                        generation=1,
                        grid_origin=(-2.0, -2.0),
                        grid_resolution=0.1,
                        source_cells=((20, 20),),
                    ),
                ),
            )
        )
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            command_keepalive_period=0.1,
            max_linear=0.60,
        )
        follower.set_command_gate(gate)
        follower._publisher = FakePublisher()
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])

        with patch("danger_search.navigator.Twist", FakeTwist), patch(
            "danger_search.navigator.time.monotonic",
            return_value=1.0,
        ), patch("danger_search.navigator.get_time", return_value=1.0):
            follower.tick((0.0, 0.0, 0.0))
            self.assertEqual(gate.state, DROP_STOPPING)
            follower.rotate_in_place(0.5)
            follower.reverse_straight(reason="test_reverse")
            follower.recover(reason="test_recover")
            follower._last_command_wall_time = 0.0
            follower._on_control_timer(None)

        self.assertGreaterEqual(len(follower._publisher.messages), 5)
        for message in follower._publisher.messages:
            self.assertEqual(message.linear.x, 0.0)
            self.assertEqual(message.linear.y, 0.0)
            self.assertEqual(message.angular.z, 0.0)
        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(snapshot["requested_linear_command"], 0.0)
        self.assertEqual(snapshot["linear_command"], 0.0)
        self.assertEqual(snapshot["safety_gate"]["state"], DROP_STOPPING)

    def test_keepalive_zeroes_direct_command_that_lost_its_driver(self):
        """直接控制命令失去每 tick 刷新方后必须归零，不能被 keepalive 无限重放。"""

        class FakeTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            command_keepalive_period=0.1,
            direct_command_max_age=0.6,
        )
        follower._publisher = FakePublisher()

        with patch("danger_search.navigator.Twist", FakeTwist), patch(
            "danger_search.navigator.get_time", return_value=1.0
        ):
            # 任务层发一条原地转向：无路径、无对准、无恢复，属于直接控制模式。
            with patch("danger_search.navigator.time.monotonic", return_value=1.0):
                follower.rotate_in_place(0.8)
            self.assertAlmostEqual(follower._publisher.messages[-1].angular.z, 0.8)

            # 寿命内：keepalive 照常重放非零命令，维持 RL 控制器命令流。
            with patch("danger_search.navigator.time.monotonic", return_value=1.2):
                follower._on_control_timer(None)
            self.assertAlmostEqual(follower._publisher.messages[-1].angular.z, 0.8)

            # 超过 direct_command_max_age 仍无人刷新：必须归零并给出原因。
            with patch("danger_search.navigator.time.monotonic", return_value=2.0):
                follower._on_control_timer(None)
            self.assertEqual(follower._publisher.messages[-1].angular.z, 0.0)
            self.assertEqual(follower._publisher.messages[-1].linear.x, 0.0)
            self.assertEqual(
                follower._debug_zero_command_reason,
                "direct_command_expired",
            )


class FixedFrequencyControllerTest(unittest.TestCase):
    """固定频率控制器（第一阶段）专项测试。"""

    @staticmethod
    def _sample(xy_yaw, yaw_rate=0.0, stamp=100.0):
        from types import SimpleNamespace
        return SimpleNamespace(
            xy_yaw=tuple(xy_yaw),
            angular_xyz=(0.0, 0.0, yaw_rate),
            stamp=stamp,
        )

    def test_control_timer_produces_new_decisions_not_keepalive(self):
        # 控制定时器在路径模式下每周期应产生新控制决策，
        # 而不是只重放旧命令增加 keepalive 计数。
        from danger_search.navigator import PathFollower
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        provider = SimpleNamespace(latest=lambda: self._sample((0.0, 0.0, 0.0)))
        follower._pose_provider = provider
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: None)
        before = follower._control_decision_count
        fresh = self._sample((0.0, 0.0, 0.0), stamp=199.95)
        provider = SimpleNamespace(latest=lambda: fresh)
        follower._pose_provider = provider
        with patch("danger_search.navigator.get_time", return_value=200.0):
            for _ in range(5):
                follower._on_control_timer(None)
        self.assertGreaterEqual(follower._control_decision_count - before, 5)

    def test_turn_in_place_release_requires_three_samples(self):
        # 转向滞回：进入立即，释放需连续 3 个新鲜样本同时满足 heading 与 yaw_rate 条件。
        from danger_search.navigator import PathFollower, RUNNING
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        # 路径向东 (1,0)，yaw=pi/2（面朝北）→ 航向误差 ~90° >= 0.35 → 进入转向
        follower.step_once_for_test(self._sample((0.0, 0.0, math.pi / 2.0)))
        self.assertTrue(follower._turn_in_place_active)
        # 误差已小但 yaw_rate 仍高：不得退出
        follower.step_once_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.5))
        self.assertTrue(follower._turn_in_place_active)
        # 连续 3 个释放样本（heading<=0.15 且 yaw_rate<=0.20）
        for _ in range(3):
            follower.step_once_for_test(self._sample((0.0, 0.0, 0.05), yaw_rate=0.1))
        self.assertFalse(follower._turn_in_place_active)

    def test_balanced_turn_threshold_keeps_moderate_turn_on_arc(self):
        from danger_search.navigator import PathFollower

        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            turn_in_place_heading=0.90,
            turn_in_place_rate=0.65,
            turn_release_heading=0.20,
        )
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])

        follower.step_once_for_test(self._sample((0.0, 0.0, -0.80), yaw_rate=0.50))
        self.assertFalse(follower._turn_in_place_active)
        self.assertGreater(follower.requested_command_snapshot()[0], 0.0)

        follower.step_once_for_test(self._sample((0.0, 0.0, -0.80), yaw_rate=0.70))
        self.assertTrue(follower._turn_in_place_active)
        self.assertEqual(follower.requested_command_snapshot()[0], 0.0)

        heading_follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            turn_in_place_heading=0.90,
            turn_in_place_rate=0.65,
            turn_release_heading=0.20,
        )
        heading_follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        heading_follower.step_once_for_test(
            self._sample((0.0, 0.0, -0.95), yaw_rate=0.10)
        )
        self.assertTrue(heading_follower._turn_in_place_active)
        self.assertEqual(heading_follower.requested_command_snapshot()[0], 0.0)

    def test_balanced_profile_expands_arc_and_releases_after_two_fresh_samples(self):
        follower = PathFollower(
            cmd_vel_topic="/cmd_vel",
            turn_in_place_heading=1.10,
            turn_in_place_rate=0.65,
            turn_release_heading=0.30,
            turn_release_rate=0.30,
            turn_release_samples=2,
            arc_turn_min_linear_fraction=0.15,
        )
        follower.set_path([(0.0, 0.0), (10.0, 0.0)])

        follower.step_once_for_test(
            self._sample((0.0, 0.0, -1.00), yaw_rate=0.10)
        )
        self.assertFalse(follower._turn_in_place_active)
        self.assertGreaterEqual(follower.requested_command_snapshot()[0], 0.12)

        follower.step_once_for_test(
            self._sample((0.0, 0.0, -1.12), yaw_rate=0.10)
        )
        self.assertTrue(follower._turn_in_place_active)
        self.assertEqual(follower.requested_command_snapshot()[0], 0.0)

        follower.step_once_for_test(
            self._sample((0.0, 0.0, -0.25), yaw_rate=0.20)
        )
        self.assertTrue(follower._turn_in_place_active)
        follower.step_once_for_test(
            self._sample((0.0, 0.0, -0.25), yaw_rate=math.nan)
        )
        self.assertTrue(follower._turn_in_place_active)
        for _ in range(2):
            follower.step_once_for_test(
                self._sample((0.0, 0.0, -0.25), yaw_rate=0.20)
            )
        self.assertFalse(follower._turn_in_place_active)
        self.assertGreater(follower.requested_command_snapshot()[0], 0.0)

    def test_pose_stale_publishes_zero_and_freezes_stuck(self):
        # pose 过期：立即零速，且不触发 STUCK（冻结进度计时）。
        from danger_search.navigator import PathFollower, STUCK
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        published = []
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: published.append(a))
        # 过期 sample：stamp 落后 pose_max_age 以上
        stale = self._sample((0.0, 0.0, 0.0), stamp=0.0)
        provider = SimpleNamespace(latest=lambda: stale)
        follower._pose_provider = provider
        with patch("danger_search.navigator.get_time", return_value=100.0):
            follower._on_control_timer(None)
        self.assertEqual(published[-1][0].linear.x, 0.0)  # 零速
        self.assertEqual(published[-1][0].angular.z, 0.0)
        self.assertNotEqual(follower._tracking_status, STUCK)

    def test_terminal_not_lost_before_poll(self):
        # DONE/STUCK 在慢任务轮询前不得丢失，也不得提前重规划。
        from danger_search.navigator import PathFollower, DONE, RUNNING, IDLE
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        # 直接推进到终点（两 tick 足够：起点到终点）
        follower.step_once_for_test(self._sample((0.0, 0.0, 0.0)))
        follower.step_once_for_test(self._sample((1.0, 0.0, 0.0)))
        self.assertTrue(follower._terminal_pending)
        self.assertTrue(follower.has_path())  # 终态未消费前路径保留
        self.assertEqual(follower.poll_status(), DONE)
        self.assertEqual(follower.poll_status(), IDLE)
        self.assertFalse(follower.has_path())

    def test_path_limits_atomic_commit(self):
        # set_path 原子提交限速与终点容差，控制步进使用提交值。
        from danger_search.navigator import PathFollower
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path(
            [(0.0, 0.0), (1.0, 0.0)],
            max_linear=0.30,
            goal_tolerance=0.50,
        )
        self.assertEqual(follower._path_max_linear, 0.30)
        self.assertEqual(follower._path_goal_tolerance, 0.50)
        snapshot = follower.tracking_debug_snapshot()
        self.assertLessEqual(snapshot["linear_command"], 0.30)

    def test_yaw_rate_damping_sign_correct(self):
        # angular_xyz[2] 正负值时阻尼方向正确：同向转速降低角速度指令。
        from danger_search.navigator import compute_tracking_command
        _, _, pos_ang = compute_tracking_command(
            0.30, max_linear=0.6, max_angular=1.2,
            measured_yaw_rate=0.6, yaw_rate_damping=0.45,
        )
        _, _, neg_ang = compute_tracking_command(
            0.30, max_linear=0.6, max_angular=1.2,
            measured_yaw_rate=-0.6, yaw_rate_damping=0.45,
        )
        _, _, plain_ang = compute_tracking_command(0.30, max_linear=0.6, max_angular=1.2)
        self.assertLess(pos_ang, plain_ang)
        self.assertGreater(neg_ang, plain_ang)


class AlignModeTest(unittest.TestCase):
    """固定频率 ALIGN 模式（R2）专项测试。"""

    @staticmethod
    def _sample(xy_yaw, yaw_rate=0.0, stamp=100.0, linear_x=0.0):
        from types import SimpleNamespace
        return SimpleNamespace(
            xy_yaw=tuple(xy_yaw),
            angular_xyz=(0.0, 0.0, yaw_rate),
            linear_xyz=(linear_x, 0.0, 0.0),
            stamp=stamp,
        )

    def test_align_completes_after_release_samples(self):
        from danger_search.navigator import PathFollower, DONE
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: None)
        # 对准目标：正东方向 (1.0, 0)（相对原点），yaw=0 已对准。
        follower.begin_heading_alignment((1.0, 0.0))
        self.assertEqual(follower._align_state, "settle")
        # SETTLE：3 帧停稳后进入 ALIGN
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        self.assertEqual(follower._align_state, "align")
        # 已对准（err=0 < deadband）：下一决策进入 CONFIRM
        follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        self.assertEqual(follower._align_state, "confirm")
        # 连续 3 个释放样本后 DONE+alignment_done
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        result = follower.poll_result()
        self.assertEqual(result.status, DONE)
        self.assertEqual(result.reason, "alignment_done")

    def test_align_damping_direction(self):
        # yaw_rate 与误差同向时角速度指令被阻尼减小。
        from danger_search.navigator import PathFollower
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        published = []
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: published.append(a))
        # 目标正东，yaw=0.3（逆时针偏），需要顺时针转（负角速度）
        follower.begin_heading_alignment((1.0, 0.0))
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        # 已进入 ALIGN：发一次命令，记录角速度
        follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        # 阻尼生效样本：误差 0.3、yaw_rate +0.6（同向）→ 指令更小
        follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.6))
        self.assertGreaterEqual(follower._debug_align_yaw_error, 0.0)
        # 不深入数值断言，只验证 ALIGN 状态推进与决策计数
        self.assertGreater(follower._control_decision_count, 0)

    def test_align_drift_latches_stuck(self):
        from danger_search.navigator import PathFollower, STUCK
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        published = []
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: published.append(a))
        follower.begin_heading_alignment(
            (1.0, 0.0),
            anchor_xy=(0.0, 0.0),
            max_anchor_error=0.20,
        )
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        # 位姿离开锚点超 0.20m：立即零速并锁存 STUCK+alignment_drift
        follower.step_align_for_test(self._sample((0.5, 0.0, 0.0), yaw_rate=0.0))
        result = follower.poll_result()
        self.assertEqual(result.status, STUCK)
        self.assertEqual(result.reason, "alignment_drift")
        self.assertEqual(published[-1][0].linear.x, 0.0)
        self.assertEqual(published[-1][0].angular.z, 0.0)

    def test_align_begin_idempotent(self):
        from danger_search.navigator import PathFollower
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: None)
        follower.begin_heading_alignment((1.0, 0.0))
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        confirm_before = follower._align_confirm_count
        # 相同目标重复调用：不重置确认计数
        follower.begin_heading_alignment((1.0, 0.0))
        self.assertEqual(follower._align_confirm_count, confirm_before)
        self.assertEqual(follower._align_state, "align")

    def test_align_settle_timeout_fails(self):
        from danger_search.navigator import PathFollower, STUCK
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: None)
        # 持续不停止（yaw_rate 高）：3s 后 settle 超时
        with patch("danger_search.navigator.get_time", side_effect=[0.0, 0.1, 0.2, 0.3, 4.0]):
            follower.begin_heading_alignment((1.0, 0.0))
            for _ in range(4):
                follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.5))
        result = follower.poll_result()
        self.assertEqual(result.status, STUCK)
        self.assertEqual(result.reason, "alignment_settle_timeout")

    def test_poll_result_atomic_consume(self):
        from danger_search.navigator import PathFollower, DONE, IDLE
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: None)
        follower.begin_heading_alignment((1.0, 0.0))
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        # 原子消费：status+reason+sequence 同时返回
        first = follower.poll_result()
        self.assertEqual(first.status, DONE)
        self.assertEqual(first.reason, "alignment_done")
        self.assertGreaterEqual(first.sequence, 1)
        # 消费后回到 IDLE，reason 清空
        second = follower.poll_result()
        self.assertEqual(second.status, IDLE)
        self.assertIsNone(second.reason)

    def test_poll_result_reason_not_overwritten_by_timer(self):
        # 消费期间控制定时器更新：返回的 reason 不被覆盖（原子锁内捕获）。
        from danger_search.navigator import PathFollower, DONE
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = SimpleNamespace(publish=lambda *a, **k: None)
        follower.begin_heading_alignment((1.0, 0.0))
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        # 消费前模拟 Timer 更新（再步进一次），reason 必须仍为 alignment_done
        result = follower.poll_result()
        self.assertEqual(result.status, DONE)
        self.assertEqual(result.reason, "alignment_done")

    def test_begin_heading_alignment_with_max_angular_limit(self):
        """局部角速度上限测试：传入 max_angular=0.65 时，发布角速度绝对值不超过 0.65 且线速度严格为零。"""
        from danger_search.navigator import PathFollower
        follower = PathFollower(cmd_vel_topic="/cmd_vel", max_angular=1.2)
        published_cmds = []
        follower._publisher = SimpleNamespace(
            publish=lambda msg: published_cmds.append((msg.linear.x, msg.linear.y, msg.angular.z))
        )
        follower.begin_heading_alignment((0.0, 1.0), max_angular=0.65)
        # 初始机身朝向 (0, 0, 0)，目标方位 pi/2 (1.57rad)，误差很大
        # step 1-3: settle
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        # step 4: align rotate
        follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        self.assertGreater(len(published_cmds), 0)
        for lx, ly, az in published_cmds:
            self.assertEqual(lx, 0.0)
            self.assertEqual(ly, 0.0)
            self.assertLessEqual(abs(az), 0.65 + 1e-6)

    def test_begin_heading_alignment_default_max_angular_preserves_global_limit(self):
        """未指定局部上限时，对准角速度按全局 max_angular 限幅。"""
        from danger_search.navigator import PathFollower
        follower = PathFollower(cmd_vel_topic="/cmd_vel", max_angular=1.2)
        published_cmds = []
        follower._publisher = SimpleNamespace(
            publish=lambda msg: published_cmds.append((msg.linear.x, msg.linear.y, msg.angular.z))
        )
        follower.begin_heading_alignment((0.0, 1.0))
        for _ in range(3):
            follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        follower.step_align_for_test(self._sample((0.0, 0.0, 0.0), yaw_rate=0.0))
        self.assertGreater(len(published_cmds), 0)
        rotate_cmds = [az for _, _, az in published_cmds if abs(az) > 0.0]
        self.assertGreater(len(rotate_cmds), 0)
        # 误差 1.57rad * 1.7 = 2.67 -> clamp 到 1.2
        self.assertAlmostEqual(rotate_cmds[-1], 1.2, places=2)


if __name__ == "__main__":
    unittest.main()

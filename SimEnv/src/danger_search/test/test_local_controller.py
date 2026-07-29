import json
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from danger_search.ground_support_gate import (
    DROP_STOPPING,
    GroundSupportAssessment,
    GroundSupportGateConfig,
    GroundSupportSafetyGate,
)
from danger_search.navigator import PathFollower, compute_tracking_command, RUNNING, STUCK


class LocalControllerTest(unittest.TestCase):
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

    def test_large_yaw_error_keeps_forward_component(self):
        linear_x, linear_y, angular = compute_tracking_command(
            1.5,
            max_linear=0.35,
            max_lateral=0.3,
            max_angular=0.9,
            min_linear_fraction=0.25,
        )

        self.assertGreater(linear_x, 0.0)
        self.assertGreater(linear_y, 0.0)
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
            no_progress_timeout=0.05,
            progress_distance=0.2,
        )
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        follower.tick((0.0, 0.0, 0.0))

        import time
        time.sleep(0.06)

        status = follower.tick((0.0, 0.0, 0.0))
        self.assertEqual(status, STUCK)
        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(snapshot["stuck_count"], 1)
        self.assertEqual(snapshot["replan_reason"], "no_progress_timeout")
        self.assertEqual(snapshot["zero_command_reason"], "no_progress_timeout")

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
        self.assertGreater(snapshot["linear_command"], 0.0)
        self.assertGreater(snapshot["lateral_command"], 0.0)
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
        import rospy

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
        with patch("danger_search.navigator.PathMessage", FakePathMessage), patch(
            "danger_search.navigator.PoseStamped", FakePoseStamped
        ), patch.object(rospy.Time, "now", return_value=SimpleNamespace()):
            follower.set_path([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)])

        self.assertEqual(len(follower._raw_path_publisher.messages), 1)
        self.assertEqual(len(follower._followed_path_publisher.messages), 1)
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
            )
        )
        follower = PathFollower(cmd_vel_topic="/cmd_vel", command_keepalive_period=0.1)
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
            follower._on_keepalive_timer(None)

        self.assertGreaterEqual(len(follower._publisher.messages), 5)
        for message in follower._publisher.messages:
            self.assertEqual(message.linear.x, 0.0)
            self.assertEqual(message.linear.y, 0.0)
            self.assertEqual(message.angular.z, 0.0)
        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(snapshot["requested_linear_command"], 0.0)
        self.assertEqual(snapshot["linear_command"], 0.0)
        self.assertEqual(snapshot["safety_gate"]["state"], DROP_STOPPING)


if __name__ == "__main__":
    unittest.main()

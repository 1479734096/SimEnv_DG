import math
import unittest

from danger_search.navigator import PathFollower, compute_tracking_command, RUNNING, STUCK


class LocalControllerTest(unittest.TestCase):
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
        linear_x, _, _ = compute_tracking_command(
            3.14,
            max_linear=0.4,
            max_lateral=0.3,
            max_angular=1.0,
            min_linear_fraction=2.0,
        )

        self.assertAlmostEqual(linear_x, 0.4)

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


if __name__ == "__main__":
    unittest.main()

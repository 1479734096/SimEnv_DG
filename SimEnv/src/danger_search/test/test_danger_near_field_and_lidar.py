import unittest
import numpy as np

from danger_search.detection_tracker import DetectionCandidate, Track


class TestDangerNearFieldAndLidar(unittest.TestCase):
    def test_near_field_truncation_discards_far_outliers(self):
        """验证近场截断机制能彻底剔除远距离门外视角的拉偏点。"""
        # 1. 模拟在门外远眺 (dist=3.5m) 产生的 5 帧粗糙候选，横向偏置 +0.8m
        far_candidates = [
            DetectionCandidate.from_sequence(
                [0.8, 3.5, 0.15],
                confidence=0.85,
                rmse=0.02,
                radius=0.15,
                stamp=10.0 + i * 0.1,
                distance=3.5,
            )
            for i in range(5)
        ]

        # 2. 模拟靠近到近场停靠点后 (dist=0.8m) 采集的 4 帧近场高精度候选，横向无偏置 (0.0)
        near_candidates = [
            DetectionCandidate.from_sequence(
                [0.0, 0.8, 0.15],
                confidence=0.95,
                rmse=0.01,
                radius=0.15,
                stamp=20.0 + i * 0.1,
                distance=0.8,
            )
            for i in range(4)
        ]

        track = Track(
            track_id=1,
            mean_position=np.array([0.8, 3.5, 0.15], dtype=float),
            first_seen=10.0,
            last_seen=10.0,
        )

        for c in far_candidates:
            track.update(c, now=c.stamp)

        # 远距离时，均值在 0.8 附近
        self.assertAlmostEqual(track.mean_position[0], 0.8, delta=0.01)

        # 注入近场样本
        for c in near_candidates:
            track.update(c, now=c.stamp)

        # 验证：近场截断机制生效，远距离 3.5m 粗检点被丢弃，均值完全收敛至近场 0.0 附近
        self.assertAlmostEqual(track.mean_position[0], 0.0, delta=0.02)
        self.assertAlmostEqual(track.mean_position[1], 0.8, delta=0.02)
        # 验证 position_std 没有被 3.5m 与 0.8m 的跨度撑大
        self.assertLess(track.position_std(), 0.05)

    def test_footprint_unsupported_blocks_even_when_unknown_hold_disabled(self):
        """验证机身足端踩空悬空时，即便 unknown_support_hold_enabled 为 False，也绝对强制刹停且超时不误放行。"""
        from danger_search.ground_support_gate import (
            GroundSupportSafetyGate, GroundSupportGateConfig,
            GroundSupportAssessment, OBSERVATION_HOLD, SUPPORT_PATH_UNAVAILABLE
        )
        config = GroundSupportGateConfig(
            enabled=True,
            boundary_hold_margin=0.40,
        )
        gate = GroundSupportSafetyGate(config)
        gate.set_unknown_support_hold_enabled(False)

        # 模拟 1 层天井现场: 自身足端踩空，开阔天井无负高度回波 (drop_confirmed=False)
        assessment = GroundSupportAssessment(
            stamp=100.0,
            received_wall_time=100.0,
            data_age=0.01,
            supported_distance=0.0,
            current_footprint_supported=False,
            observation_adequate=True,
            drop_confirmed=False,
            drop_intersects_sweep=False,
            drop_distance=None,
            drop_direction_world=(1.0, 0.0),
            received_control_time=100.0,
            support_boundary_detected=True,
        )
        gate.update_assessment(assessment)

        # 验证 1: 必须强制输出零速 (0, 0, 0)
        cmd = gate.filter_command((0.65, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0, control_time=100.0)
        self.assertEqual(cmd, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, OBSERVATION_HOLD)

        # 验证 2: 观察期超时后不能错误放行直走，必须切入 SUPPORT_PATH_UNAVAILABLE 触发重规划
        gate.check_control_timeout(control_time=102.1, wall_time=102.1)
        self.assertEqual(gate.state, SUPPORT_PATH_UNAVAILABLE)


if __name__ == "__main__":
    unittest.main()

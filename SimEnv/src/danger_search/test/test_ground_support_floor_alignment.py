"""跨楼层地面防坠落高度对齐与防御校准测试。"""
from __future__ import annotations

import math
from threading import RLock
from types import SimpleNamespace
import unittest

import numpy as np

from danger_search.ground_support_coordinator import (
    GroundSupportCoordinator,
    GroundSupportComponents,
)
from danger_search.ground_support_runtime import GroundSupportFeedbackRuntime
from danger_search.pose_provider import PoseSample, PoseProvider
from danger_search.support_shadow import (
    ShadowSupportConfig,
    ShadowSupportGrid,
    _cell_for_xy,
)


class GroundSupportFloorAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.map_params = {
            "resolution": 0.05,
            "origin_x": -10.0,
            "origin_y": -10.0,
            "floor_height": 2.6,
            "obstacle_min_z": 0.1,
            "obstacle_max_z": 1.0,
        }
        self.coordinator_params = {
            "production_gate_enabled": False,
            "decision_period": 0.1,
            "base_to_livox_xyz": (0.2, 0.0, 0.08),
            "base_to_livox_rpy": (0.0, 0.785, 0.0),
            "sweep_half_width": 0.25,
            "support_band": 0.08,
            "minimum_drop": 0.15,
            "minimum_cell_hits": 2,
            "required_consecutive_frames": 2,
            "minimum_alert_cells": 3,
            "foot_band_center_offset": 0.25,
            "foot_band_half_width": 0.15,
            "minimum_foot_band_coverage": 0.5,
            "maximum_longitudinal_unknown_gap": 0.3,
            "minimum_longitudinal_support_ratio": 0.7,
            "minimum_lookahead": 0.3,
            "maximum_lookahead": 0.8,
            "footprint_front_reach": 0.35,
            "stop_linear_threshold": 0.05,
            "stop_angular_threshold": 0.1,
            "stop_confirmation_samples": 2,
            "localization_speed_anomaly_threshold": 2.0,
            "localization_speed_anomaly_timeout": 0.5,
        }

    def _create_test_coordinator(self, fake_process_frame):
        coordinator = GroundSupportCoordinator(
            params=self.coordinator_params,
            map_params=self.map_params,
            cloud_max_points=500,
            path_goal_tolerance=0.1,
        )
        coordinator.components = GroundSupportComponents(
            gate=SimpleNamespace(
                hole_detection_exempt=False,
                hole_scope_revision=1,
                invalidate_evidence_scope=lambda: None,
                update_assessment=lambda assessment: None,
            ),
            grid=SimpleNamespace(
                config=SimpleNamespace(floor_height=2.6),
                process_frame=fake_process_frame,
                invalidate_evidence=lambda: None,
            ),
        )
        coordinator.activate_floor(1)
        return coordinator

    def test_coordinator_defense_correction_when_raw_pose_provided(self):
        """当外部传入未平移的 0 层高度样本（z=0.268）时，coordinator 能防御性自动补齐 1 层高程。"""
        captured_kwargs = {}

        def fake_process_frame(**kwargs):
            captured_kwargs.update(kwargs)
            return {"action": "test_done"}

        coordinator = self._create_test_coordinator(fake_process_frame)

        pose_provider = PoseProvider.__new__(PoseProvider)
        # 原始未平移位姿，z 只有 0.268
        raw_sample = PoseSample(
            stamp=1.0,
            xyz=(0.0, 0.0, 0.268),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )

        controller = SimpleNamespace(
            followed_path_snapshot=lambda: [(0.0, 0.0), (1.0, 0.0)],
            active_motion_command_snapshot=lambda: (0.2, 0.0, 0.0),
            committed_command_snapshot=lambda: (0.0, 0.0, 0.0),
            last_command=(0.2, 0.0, 0.0),
        )

        coordinator.assess_frame(
            controller=controller,
            pose_provider=pose_provider,
            floor_index=1,
            pose=(0.0, 0.0, 0.0),
            pose_sample=raw_sample,
            points=np.empty((0, 3), dtype=float),
            cloud_stamp=1.0,
            cloud_generation=1,
        )

        self.assertIn("sensor_origin", captured_kwargs)
        sensor_origin = captured_kwargs["sensor_origin"]
        # 雷达绝对 z 必须大于 1 层地面 (2.6m)，实际应为 2.6 + 0.268 + 0.08 = 2.948m
        self.assertAlmostEqual(sensor_origin[2], 2.948, places=3)

    def test_coordinator_no_double_correction_when_logical_pose_provided(self):
        """当外部已传入正确的逻辑高度样本（z=2.868）时，不重复累加高度。"""
        captured_kwargs = {}

        def fake_process_frame(**kwargs):
            captured_kwargs.update(kwargs)
            return {"action": "test_done"}

        coordinator = self._create_test_coordinator(fake_process_frame)

        pose_provider = PoseProvider.__new__(PoseProvider)
        # 已经包含 1 层偏移的逻辑位姿 (2.6 + 0.268 = 2.868)
        logical_sample = PoseSample(
            stamp=1.0,
            xyz=(0.0, 0.0, 2.868),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )

        controller = SimpleNamespace(
            followed_path_snapshot=lambda: [(0.0, 0.0), (1.0, 0.0)],
            active_motion_command_snapshot=lambda: (0.2, 0.0, 0.0),
            committed_command_snapshot=lambda: (0.0, 0.0, 0.0),
            last_command=(0.2, 0.0, 0.0),
        )

        coordinator.assess_frame(
            controller=controller,
            pose_provider=pose_provider,
            floor_index=1,
            pose=(0.0, 0.0, 0.0),
            pose_sample=logical_sample,
            points=np.empty((0, 3), dtype=float),
            cloud_stamp=1.0,
            cloud_generation=1,
        )

        sensor_origin = captured_kwargs["sensor_origin"]
        # 仍然稳定在 2.948m，没有被重复加上 2.6m
        self.assertAlmostEqual(sensor_origin[2], 2.948, places=3)

    def test_blind_zone_geometry_under_corrected_height(self):
        """验证真实高度下，脚前近身盲区不会被误算入雷达 FOV，从而不会引发深坑误报。"""
        config = ShadowSupportConfig(
            resolution=0.05,
            origin_x=-10.0,
            origin_y=-10.0,
            floor_height=2.6,
            cloud_stride=1,
            cloud_max_points=500,
            minimum_cell_hits=2,
            required_consecutive_frames=2,
            minimum_alert_cells=2,
            sweep_half_width=0.25,
            minimum_frame_points=1,
            threshold_widths=(0.25,),
            fov_min_elevation_deg=-7.0,
            fov_max_elevation_deg=55.0,
        )
        grid = ShadowSupportGrid(config)

        # 正确高度下的雷达原点与 45° 下倾姿态
        sensor_origin = np.asarray((0.2, 0.0, 2.948), dtype=float)
        # pitch = 45° (0.785 rad) 下倾
        pitch = 0.785
        cp, sp = math.cos(pitch), math.sin(pitch)
        sensor_rotation = np.array([
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ], dtype=float)

        # 检查狗前方 0.35m 处的地面网格（x=0.35, y=0.0, z=2.60）
        cell = _cell_for_xy(0.35, 0.0, resolution=0.05, origin_x=-10.0, origin_y=-10.0)
        inside_fov = grid._inside_fov(
            cell,
            floor_z=2.60,
            sensor_origin=sensor_origin,
            sensor_rotation=sensor_rotation,
        )
        # 物理上脚前 0.35m 地面相对雷达处于下盲区（仰角约为 -22° < -7°），inside_fov 必须为 False！
        self.assertFalse(
            inside_fov,
            "脚前近身盲区在正确高度下绝不应被误判为 inside_fov！",
        )


if __name__ == "__main__":
    unittest.main()

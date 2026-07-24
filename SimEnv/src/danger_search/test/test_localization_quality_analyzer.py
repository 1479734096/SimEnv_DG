"""定位质量离线分析工具测试。"""

import importlib.util
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import unittest


SIMENV_ROOT = Path(__file__).resolve().parents[3]
ANALYZER_PATH = (
    SIMENV_ROOT / "tools" / "offline" / "analyze_localization_quality_bag.py"
)


def _load_analyzer_module():
    spec = importlib.util.spec_from_file_location(
        "localization_quality_analyzer", ANALYZER_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载定位质量分析脚本: {ANALYZER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class LocalizationQualityAnalyzerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.analyzer = _load_analyzer_module()

    def test_same_stamp_counterfactual_isolates_applied_correction(self):
        harmful = self.analyzer.correction_benefit(
            pose_after=(1.2, 2.0), truth=(1.0, 2.0), applied=(0.1, 0.0)
        )
        necessary = self.analyzer.correction_benefit(
            pose_after=(1.0, 2.0), truth=(1.0, 2.0), applied=(0.1, 0.0)
        )
        self.assertAlmostEqual(harmful[2], -0.1)
        self.assertAlmostEqual(harmful[3], -0.1)
        self.assertAlmostEqual(necessary[2], 0.1)
        self.assertAlmostEqual(necessary[3], 0.1)
        self.assertAlmostEqual(necessary[4], 0.0)

    def test_parses_scan_stamp_and_raw_applied_vectors(self):
        message = (
            "localization_map_registration_frame scan_stamp=12.340000 "
            "source=livox_scan_to_map status=accepted accepted=1 "
            "raw=(0.020000, -0.030000) applied=(0.010000, -0.015000) "
            "clamped=1 source_points=900 matched=700 retained=500 "
            "map_points=4000 iterations=3 residual_p50=0.020000 "
            "residual_p70=0.030000 residual_p90=0.080000 "
            "residual_mean=(0.001000, -0.002000) "
            "residual_median=(0.000500, -0.001000) "
            "residual_mad=(0.010000, 0.020000) "
            "residual_positive=(0.510000, 0.490000) "
            "residual_negative=(0.490000, 0.510000) "
            "covariance_eigen=(0.000100000, 0.000900000) anisotropy=0.111111"
        )
        frame = self.analyzer.parse_map_frame(message)
        self.assertIsNotNone(frame)
        self.assertAlmostEqual(frame.stamp, 12.34)
        self.assertAlmostEqual(frame.raw_x, 0.02)
        self.assertAlmostEqual(frame.applied_y, -0.015)
        self.assertEqual(frame.matched, 700)
        self.assertAlmostEqual(frame.residual_positive_x, 0.51)

    def test_planar_parser_preserves_frame_interval(self):
        message = (
            "localization_planar_registration_frame source=livox_frame_gicp "
            "scan_stamp=8.100000 frame_interval=0.100000 status=accepted "
            "accepted=1 has_delta=1 candidate=(0.01, 0.02) "
            "applied=(-0.030000, 0.040000) candidate_yaw=-0.01200000 "
            "candidate_rotation_angle=0.01500000 fitness=0.01 points=300"
        )
        frame = self.analyzer.parse_planar_frame(message)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.source, "livox_frame_gicp")
        self.assertAlmostEqual(frame.interval, 0.1)
        self.assertAlmostEqual(frame.applied_y, 0.04)
        self.assertTrue(frame.accepted)
        self.assertTrue(frame.has_delta)
        self.assertEqual(frame.status, "accepted")
        self.assertAlmostEqual(frame.candidate_yaw, -0.012)
        self.assertAlmostEqual(frame.candidate_rotation_angle, 0.015)

    def test_planar_parser_accepts_legacy_log_without_rotation(self):
        message = (
            "localization_planar_registration_frame source=rgbd_frame_gicp "
            "scan_stamp=8.100000 frame_interval=0.100000 status=accepted "
            "accepted=1 has_delta=1 candidate=(0.01, 0.02) "
            "applied=(-0.030000, 0.040000) fitness=0.01 points=300"
        )
        frame = self.analyzer.parse_planar_frame(message)
        self.assertIsNotNone(frame)
        self.assertTrue(math.isnan(frame.candidate_yaw))
        self.assertTrue(math.isnan(frame.candidate_rotation_angle))

    def test_planar_parser_accepts_nonfinite_rotation_diagnostic(self):
        message = (
            "localization_planar_registration_frame source=rgbd_frame_gicp "
            "scan_stamp=8.100000 frame_interval=nan status=initial_frame "
            "accepted=1 has_delta=0 candidate=(0.0, 0.0) "
            "applied=(0.0, 0.0) candidate_yaw=nan "
            "candidate_rotation_angle=nan fitness=0.0 points=300"
        )
        frame = self.analyzer.parse_planar_frame(message)
        self.assertIsNotNone(frame)
        self.assertTrue(math.isnan(frame.interval))
        self.assertTrue(math.isnan(frame.candidate_yaw))
        self.assertTrue(math.isnan(frame.candidate_rotation_angle))
        payload = self.analyzer._json_safe({
            "candidate_yaw": frame.candidate_yaw,
            "candidate_rotation_angle": frame.candidate_rotation_angle,
        })
        self.assertEqual(payload, {
            "candidate_yaw": None,
            "candidate_rotation_angle": None,
        })

    def test_pose_interpolation_wraps_yaw_on_shortest_arc(self):
        Pose = self.analyzer.PoseSample
        samples = [
            Pose(0.0, 0.0, 0.0, math.radians(179.0)),
            Pose(1.0, 1.0, 2.0, math.radians(-179.0)),
        ]
        midpoint = self.analyzer.interpolate_pose(samples, 0.5, 0.6)
        self.assertIsNotNone(midpoint)
        self.assertAlmostEqual(midpoint.x, 0.5)
        self.assertAlmostEqual(midpoint.y, 1.0)
        self.assertAlmostEqual(abs(midpoint.yaw), math.pi, places=6)
        self.assertIsNone(self.analyzer.interpolate_pose(samples, 0.5, 0.4))

    def test_motion_window_parser_preserves_axis_relationship(self):
        message = (
            "localization_motion_consistency_summary scan_stamp=10.200000 "
            "duration=2.100000 depth=(0.2, -0.1) livox=(0.3, -0.05) "
            "scan_to_map=(-0.04, 0.02) fused=(0.16, -0.08) "
            "command=(0.18, -0.02) divergence=(0.1, 0.05) "
            "axis_relation=(-1, 1) axis_ratio=(0.4, 0.4) "
            "imu_abs_yaw=0.3 depth_fresh_ratio=0.8 frames=21"
        )
        window = self.analyzer.parse_motion_window(message)
        self.assertIsNotNone(window)
        self.assertAlmostEqual(window.duration, 2.1)
        self.assertEqual(window.relation_x, -1)
        self.assertAlmostEqual(window.scan_to_map_y, 0.02)

    def test_motion_window_uses_same_stamp_counterfactual(self):
        Pose = self.analyzer.PoseSample
        Window = self.analyzer.MotionWindow
        estimates = [Pose(1.0, 0.0, 0.0, 0.0), Pose(3.0, 1.0, 0.0, 0.0)]
        truths = [Pose(1.0, 0.0, 0.0, 0.0), Pose(3.0, 1.0, 0.0, 0.0)]
        window = Window(
            3.0, 2.0, 1.2, 0.0, 1.0, 0.0, 0.2, 0.0,
            1.0, 0.0, 1.0, 0.0, -0.2, 0.0, -1, 0,
            1.0, math.nan, 0.1, 1.0, 20,
        )
        sample = self.analyzer.build_motion_window_samples(
            [window], estimates, truths, 0.1
        )[0]
        self.assertAlmostEqual(sample.error_post, 0.0)
        self.assertAlmostEqual(sample.error_without_map, 0.2)
        self.assertAlmostEqual(sample.map_benefit_x, 0.2)

    def test_local_projection_uses_pose_yaw_instead_of_world_axis(self):
        forward, lateral = self.analyzer.project_to_local(
            (0.0, 1.0), math.pi / 2.0
        )
        self.assertAlmostEqual(forward, 1.0)
        self.assertAlmostEqual(lateral, 0.0, places=12)

    def test_motion_window_builds_local_counterfactual_and_structure_features(self):
        Pose = self.analyzer.PoseSample
        Window = self.analyzer.MotionWindow
        Structure = self.analyzer.StructureFrame
        estimates = [
            Pose(1.0, 0.0, 0.0, math.pi / 2.0),
            Pose(3.0, 0.0, 0.0, math.pi / 2.0),
        ]
        truths = [Pose(1.0, 0.0, 0.0, 0.0), Pose(3.0, 0.0, 0.0, 0.0)]
        window = Window(
            3.0, 2.0, 0.0, 0.2, 0.0, 0.4, 0.0, 0.1,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.2, 0, 1,
            math.nan, 2.0, 0.1, 1.0, 20,
        )
        structure = Structure(3.0, True, 100, 0.1, 0.0, 1.0)
        sample = self.analyzer.build_motion_window_samples(
            [window], estimates, truths, 0.1, [structure]
        )[0]
        self.assertAlmostEqual(sample.correction_forward, 0.1)
        self.assertAlmostEqual(sample.divergence_forward, 0.2)
        self.assertAlmostEqual(sample.benefit_forward, 0.1)
        self.assertEqual(sample.relation_forward, 1)
        self.assertTrue(sample.structure_valid)
        self.assertAlmostEqual(sample.structure_c_weak, 0.0, places=12)

    def test_structure_pca_rejects_isotropic_points_and_fixes_axis_sign(self):
        elongated = self.analyzer.np.asarray([
            (float(index), 0.02 * (-1.0 if index % 2 else 1.0))
            for index in range(-20, 21)
        ])
        structure = self.analyzer.structure_frame_from_points(1.0, elongated)
        self.assertTrue(structure.valid)
        self.assertLess(structure.ratio, 0.01)
        self.assertAlmostEqual(structure.weak_axis_x, 0.0, delta=0.01)
        self.assertGreater(structure.weak_axis_y, 0.99)

        angles = [2.0 * math.pi * index / 40.0 for index in range(40)]
        circle = self.analyzer.np.asarray([
            (math.cos(angle), math.sin(angle)) for angle in angles
        ])
        isotropic = self.analyzer.structure_frame_from_points(2.0, circle)
        self.assertFalse(isotropic.valid)
        self.assertGreater(isotropic.ratio, 0.9)

    def test_transition_bounds_use_mission_state_events(self):
        start = {
            "event": "mission_state_transition",
            "old_state": "EXPLORE_FLOOR",
            "new_state": "CHANGE_FLOOR",
            "current_floor": 0,
            "target_floor": 1,
        }
        done = {
            "event": "mission_state_transition",
            "new_state": "EXPLORE_FLOOR",
            "current_floor": 1,
            "current_task_phase": "DONE",
        }
        bounds = self.analyzer.transition_bounds_from_rosout([
            (5.0, json.dumps(start)), (8.0, json.dumps(done))
        ])
        self.assertEqual(bounds, (5.0, 8.0))

    def test_multiple_floor_transitions_and_post_elevator_phase_are_preserved(self):
        entries = []
        for stamp, source, target in ((5.0, 0, 1), (20.0, 1, 0)):
            entries.append((stamp, json.dumps({
                "event": "mission_state_transition",
                "old_state": "EXPLORE_FLOOR",
                "new_state": "CHANGE_FLOOR",
                "current_floor": source,
                "target_floor": target,
            })))
            entries.append((stamp + 3.0, json.dumps({
                "event": "mission_state_transition",
                "new_state": "EXPLORE_FLOOR",
                "current_floor": target,
                "current_task_phase": "DONE",
            })))
        transitions = self.analyzer.floor_transition_intervals_from_rosout(entries)
        states = self.analyzer.mission_state_samples_from_rosout(entries)
        self.assertEqual(len(transitions), 2)
        self.assertEqual(
            self.analyzer.mission_context_at_stamp(6.0, states, transitions),
            ("elevator_transition_0_to_1", 0),
        )
        self.assertEqual(
            self.analyzer.mission_context_at_stamp(9.0, states, transitions),
            ("post_elevator_floor_1_5s", 1),
        )

    def test_common_windows_compare_both_sensors_against_interpolated_truth(self):
        Pose = self.analyzer.PoseSample
        Planar = self.analyzer.PlanarFrame
        Map = self.analyzer.MapFrame
        poses = [Pose(0.1 * index, 0.1 * index, 0.0, 0.0)
                 for index in range(11)]
        planar = []
        for index in range(1, 11):
            stamp = 0.1 * index
            planar.append(Planar(
                "rgbd_frame_gicp", stamp, 0.1, 0.08, 0.0
            ))
            planar.append(Planar(
                "livox_frame_gicp", stamp, 0.1, 0.10, 0.0
            ))
        map_frames = [Map(
            stamp=0.1 * index,
            raw_x=0.0, raw_y=0.0, applied_x=0.001, applied_y=0.0,
            source_points=100, matched=100, retained=100, map_points=100,
            iterations=1, residual_p50=0.0, residual_p70=0.0,
            residual_p90=0.0, residual_mean_x=0.0, residual_mean_y=0.0,
            residual_median_x=0.0, residual_median_y=0.0,
            residual_mad_x=0.0, residual_mad_y=0.0,
            residual_positive_x=0.5, residual_positive_y=0.5,
            residual_negative_x=0.5, residual_negative_y=0.5,
            covariance_min=0.0, covariance_max=0.0, anisotropy=0.0,
        ) for index in range(1, 11)]
        windows = self.analyzer.build_sensor_delta_windows(
            planar, map_frames, poses, poses, [], window_duration=0.5
        )
        self.assertEqual(len(windows), 2)
        first = windows[0]
        self.assertAlmostEqual(first.truth_distance, 0.5)
        self.assertAlmostEqual(first.rgbd_error_forward, -0.1)
        self.assertAlmostEqual(first.livox_error_forward, 0.0)
        self.assertAlmostEqual(first.scan_to_map_x, 0.004)
        self.assertGreaterEqual(first.rgbd_coverage, 0.99)
        self.assertTrue(math.isnan(first.rgbd_rotation_mean))
        summary = self.analyzer.sensor_delta_summary(windows)
        self.assertLess(
            summary["groups"]["all"]["livox"]["rmse"],
            summary["groups"]["all"]["rgbd"]["rmse"],
        )

    def test_common_windows_aggregate_hidden_rotation_diagnostics(self):
        Pose = self.analyzer.PoseSample
        Planar = self.analyzer.PlanarFrame
        poses = [Pose(0.1 * index, 0.1 * index, 0.0, 0.0)
                 for index in range(16)]
        planar = []
        for index in range(1, 16):
            stamp = 0.1 * index
            planar.extend((
                Planar("rgbd_frame_gicp", stamp, 0.1,
                       0.09 - 0.003 * index, 0.0,
                       candidate_yaw=-0.02 * index,
                       candidate_rotation_angle=0.02 * index),
                Planar("livox_frame_gicp", stamp, 0.1, 0.10, 0.0,
                       candidate_yaw=0.005 * index,
                       candidate_rotation_angle=0.006 * index),
            ))
        windows = self.analyzer.build_sensor_delta_windows(
            planar, [], poses, poses, [], window_duration=0.5
        )
        self.assertEqual(len(windows), 3)
        self.assertGreater(windows[0].rgbd_rotation_mean,
                           windows[0].livox_rotation_mean)
        self.assertAlmostEqual(windows[0].rgbd_yaw_abs_max, 0.10)
        summary = self.analyzer.sensor_delta_summary(windows)
        self.assertTrue(math.isfinite(
            summary["groups"]["all"]["rgbd"]
            ["hidden_rotation_error_spearman"]
        ))
        self.assertTrue(math.isnan(
            summary["groups"]["all"]["livox"]
            ["hidden_rotation_error_spearman"]
        ))
        self.assertIn(
            "hidden_rotation_window_mean_p50",
            summary["groups"]["all"]["rgbd"],
        )

    def test_hidden_rotation_spearman_requires_three_windows(self):
        Window = self.analyzer.SensorDeltaWindow
        base = dict(
            start_stamp=0.0, end_stamp=0.5, duration=0.5,
            phase="EXPLORE_FLOOR", floor=0,
            odom_start_x=0.0, odom_start_y=0.0,
            odom_end_x=0.1, odom_end_y=0.0,
            truth_start_x=0.0, truth_start_y=0.0,
            truth_end_x=0.1, truth_end_y=0.0,
            truth_distance=0.1,
            rgbd_x=0.1, rgbd_y=0.0, livox_x=0.1, livox_y=0.0,
            scan_to_map_x=0.0, scan_to_map_y=0.0,
            rgbd_coverage=1.0, livox_coverage=1.0,
            rgbd_count=5, livox_count=5, yaw=0.0,
            absolute_yaw_change=0.0, depth_fresh_ratio=1.0,
            truth_forward=0.1, truth_lateral=0.0,
            rgbd_error_forward=0.01, rgbd_error_lateral=0.0,
            livox_error_forward=0.0, livox_error_lateral=0.0,
            rgbd_error_norm=0.01, livox_error_norm=0.0,
            rgbd_rotation_mean=0.01, rgbd_rotation_max=0.02,
        )
        windows = [Window(**base), Window(**{
            **base,
            "start_stamp": 0.5,
            "end_stamp": 1.0,
            "rgbd_error_norm": 0.02,
            "rgbd_rotation_mean": 0.02,
        })]
        metric = self.analyzer.sensor_delta_summary(windows)["groups"]["all"]
        self.assertTrue(math.isnan(
            metric["rgbd"]["hidden_rotation_error_spearman"]
        ))

    def test_livox_primary_gate_requires_enough_windows_and_low_turn_safety(self):
        Pose = self.analyzer.PoseSample
        Planar = self.analyzer.PlanarFrame
        poses = [Pose(0.1 * index, 0.1 * index, 0.0, 0.0)
                 for index in range(121)]
        planar = []
        for index in range(1, 121):
            stamp = 0.1 * index
            planar.extend((
                Planar("rgbd_frame_gicp", stamp, 0.1, 0.08, 0.0),
                Planar("livox_frame_gicp", stamp, 0.1, 0.10, 0.0),
            ))
        windows = self.analyzer.build_sensor_delta_windows(
            planar, [], poses, poses, [], window_duration=0.5
        )
        self.assertGreaterEqual(len(windows), 20)
        summary = self.analyzer.sensor_delta_summary(windows)
        self.assertTrue(summary["livox_primary_candidate"])
        degraded = [
            replace(item, livox_error_forward=0.2, livox_error_norm=0.2)
            for item in windows
        ]
        self.assertFalse(
            self.analyzer.sensor_delta_summary(degraded)[
                "livox_primary_candidate"
            ]
        )

    def test_error_growth_details_are_deterministic_and_use_local_axes(self):
        Pose = self.analyzer.PoseSample
        Planar = self.analyzer.PlanarFrame
        truths = [Pose(0.1 * index, 0.0, 0.1 * index, math.pi / 2.0)
                  for index in range(41)]
        estimates = [
            Pose(item.stamp, item.x + 0.02 * max(0.0, item.stamp - 1.0),
                 item.y, item.yaw)
            for item in truths
        ]
        planar = []
        for index in range(1, 41):
            stamp = 0.1 * index
            planar.extend((
                Planar("rgbd_frame_gicp", stamp, 0.1, 0.0, 0.08),
                Planar("livox_frame_gicp", stamp, 0.1, 0.0, 0.10),
            ))
        sensor_windows = self.analyzer.build_sensor_delta_windows(
            planar, [], estimates, truths, [], window_duration=0.5
        )
        first = self.analyzer.describe_error_growth_windows(
            estimates, truths, sensor_windows, [], duration=2.0, count=1
        )
        second = self.analyzer.describe_error_growth_windows(
            estimates, truths, sensor_windows, [], duration=2.0, count=1
        )
        self.assertEqual(first, second)
        self.assertEqual(len(first), 1)
        self.assertIn("odom_start", first[0])
        self.assertIn("diagnostic_hypotheses", first[0])
        self.assertAlmostEqual(first[0]["truth_delta"][0], 0.0, places=6)
        self.assertAlmostEqual(
            first[0]["common_window_duration"],
            0.5 * first[0]["common_window_count"],
        )
        self.assertAlmostEqual(
            first[0]["sensor_truth_delta"][1],
            first[0]["common_window_duration"],
            places=6,
        )

    def test_map_replay_recovers_translation_and_tracks_origin(self):
        np = self.analyzer.np
        world = np.asarray([
            (0.2 * x, 0.2 * y, 0.25 * z)
            for x in range(-6, 7)
            for y in range(-6, 7)
            for z in range(2)
        ], dtype=float)
        replay = self.analyzer.DiagnosticMapReplay()
        initial = replay.process(
            world, np.zeros(3), "source_floor", 1.0, np.zeros(3)
        )
        self.assertEqual(initial["status"], "initial_frame")
        moved_vectors = world - np.asarray((0.1, 0.0, 0.0))
        result = replay.process(
            moved_vectors,
            np.asarray((0.08, 0.0, 0.0)),
            "target_floor",
            2.0,
            np.asarray((0.1, 0.0, 0.0)),
        )
        self.assertTrue(result["accepted"])
        self.assertAlmostEqual(result["correction"][0], 0.02, delta=0.01)
        self.assertGreater(result["origins"].get("source_floor", 0), 80)

        replay.reset()
        reset = replay.process(
            moved_vectors,
            np.asarray((0.08, 0.0, 0.0)),
            "target_floor",
            2.0,
        )
        self.assertEqual(reset["status"], "initial_frame")
        self.assertAlmostEqual(float(np.linalg.norm(reset["correction"])), 0.0)

    def test_auc_and_correlation_keep_continuous_signal(self):
        values = [0.1, 0.2, 0.8, 0.9]
        labels = [False, False, True, True]
        self.assertAlmostEqual(self.analyzer.roc_auc(values, labels), 1.0)
        self.assertAlmostEqual(
            self.analyzer.spearman(values, [-2.0, -1.0, 1.0, 2.0]), 1.0
        )
        self.assertTrue(math.isnan(self.analyzer.roc_auc([1.0], [True])))

    def test_block_bootstrap_auc_is_deterministic(self):
        scores = [0.0, 0.1, 0.2, 0.8, 0.9, 1.0] * 4
        labels = [False, False, False, True, True, True] * 4
        first = self.analyzer.block_bootstrap_auc(
            scores, labels, block_size=5, repetitions=200, seed=7
        )
        second = self.analyzer.block_bootstrap_auc(
            scores, labels, block_size=5, repetitions=200, seed=7
        )
        self.assertEqual(first, second)
        self.assertAlmostEqual(first["auc"], 1.0)
        self.assertEqual(first["valid_repetitions"], 200)
        self.assertLessEqual(first["ci95_low"], first["ci95_high"])

    def test_top_windows_are_non_overlapping(self):
        Pose = self.analyzer.PoseSample
        estimates = [Pose(float(index), float(index), 0.0, 0.0) for index in range(8)]
        truths = [Pose(float(index), 0.5 * float(index), 0.0, 0.0) for index in range(8)]
        windows = self.analyzer.top_error_growth_windows(
            estimates, truths, duration=2.0, count=3
        )
        self.assertGreaterEqual(len(windows), 2)
        for left, right in zip(windows, windows[1:]):
            self.assertTrue(
                left["end_stamp"] <= right["start_stamp"]
                or right["end_stamp"] <= left["start_stamp"]
            )


if __name__ == "__main__":
    unittest.main()

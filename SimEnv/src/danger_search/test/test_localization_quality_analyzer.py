"""定位质量离线分析工具测试。"""

import importlib.util
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
            "applied=(-0.030000, 0.040000) fitness=0.01 points=300"
        )
        frame = self.analyzer.parse_planar_frame(message)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.source, "livox_frame_gicp")
        self.assertAlmostEqual(frame.interval, 0.1)
        self.assertAlmostEqual(frame.applied_y, 0.04)

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

    def test_auc_and_correlation_keep_continuous_signal(self):
        values = [0.1, 0.2, 0.8, 0.9]
        labels = [False, False, True, True]
        self.assertAlmostEqual(self.analyzer.roc_auc(values, labels), 1.0)
        self.assertAlmostEqual(
            self.analyzer.spearman(values, [-2.0, -1.0, 1.0, 2.0]), 1.0
        )
        self.assertTrue(math.isnan(self.analyzer.roc_auc([1.0], [True])))

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

"""逐帧定位分歧反事实分析器测试。"""

import importlib.util
import math
from pathlib import Path
import sys
import unittest


SIMENV_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = (
    SIMENV_ROOT
    / "tools/offline/analyze_localization_frame_counterfactual.py"
)
SPEC = importlib.util.spec_from_file_location(
    "analyze_localization_frame_counterfactual", SCRIPT_PATH
)
frame_cf = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = frame_cf
SPEC.loader.exec_module(frame_cf)


class LocalizationFrameCounterfactualTest(unittest.TestCase):
    @staticmethod
    def _poses(step_count=40):
        estimates = [
            frame_cf.PoseSample(index * 0.1, index * 0.01, 0.0, 0.0)
            for index in range(step_count)
        ]
        truths = list(estimates)
        return estimates, truths

    @staticmethod
    def _frames(rgbd_delta=0.02, livox_delta=0.01):
        frames = []
        for index in range(1, 16):
            stamp = index * 0.2
            frames.append(
                frame_cf.PlanarFrame(
                    "rgbd_frame_gicp",
                    stamp,
                    0.2,
                    rgbd_delta if index == 8 else 0.02,
                    0.0,
                )
            )
            frames.extend(
                (
                    frame_cf.PlanarFrame(
                        "livox_frame_gicp",
                        stamp - 0.1,
                        0.1,
                        0.01,
                        0.0,
                    ),
                    frame_cf.PlanarFrame(
                        "livox_frame_gicp",
                        stamp,
                        0.1,
                        livox_delta if index == 8 else 0.01,
                        0.0,
                    ),
                )
            )
        return frames

    def test_overlap_is_causal_and_uses_fractional_interval(self):
        frames = [
            frame_cf.PlanarFrame(
                "livox_frame_gicp", 0.9, 0.2, 0.04, 0.0
            ),
            frame_cf.PlanarFrame(
                "livox_frame_gicp", 1.0, 0.1, 0.03, 0.0
            ),
            frame_cf.PlanarFrame(
                "livox_frame_gicp", 1.1, 0.1, 99.0, 0.0
            ),
        ]

        x, y, coverage = frame_cf._livox_overlap(0.8, 1.0, frames)

        self.assertAlmostEqual(x, 0.05)
        self.assertEqual(y, 0.0)
        self.assertEqual(coverage, 1.0)

    def test_quality_missing_falls_back_instead_of_becoming_zero_motion(self):
        estimates, truths = self._poses()
        frames = [
            frame_cf.PlanarFrame(
                "rgbd_frame_gicp", 1.0, 0.2, 0.02, 0.0
            ),
            frame_cf.PlanarFrame(
                "livox_frame_gicp", 0.85, 0.05, 0.005, 0.0
            ),
        ]

        comparisons, counters = frame_cf.build_comparisons(
            frames, estimates, truths
        )

        self.assertEqual(comparisons, [])
        self.assertEqual(counters["livox_coverage_unavailable"], 1)

    def test_disagreement_boundary_and_error_labels_are_inclusive(self):
        estimates, truths = self._poses()
        frames = self._frames(rgbd_delta=0.07, livox_delta=0.01)

        comparisons, _ = frame_cf.build_comparisons(
            frames, estimates, truths
        )
        anomalous = comparisons[7]

        self.assertAlmostEqual(
            anomalous.disagreement,
            frame_cf.DISAGREEMENT_THRESHOLD,
        )
        self.assertTrue(anomalous.flagged)
        self.assertGreaterEqual(
            anomalous.rgbd_error,
            frame_cf.HIGH_ERROR_MINIMUM,
        )

    def test_global_regression_blocks_locally_better_action(self):
        estimates, truths = self._poses()
        frames = self._frames(rgbd_delta=0.12, livox_delta=0.01)
        result, comparisons = frame_cf.analyze_run(
            "synthetic",
            "validation",
            Path("synthetic.bag"),
            frames,
            estimates,
            truths,
        )

        substitution = next(
            action
            for action in result["actions"]
            if action["action"] == "livox_substitution"
        )

        self.assertTrue(any(item.flagged for item in comparisons))
        self.assertGreater(
            substitution["local_relative_improvement"],
            frame_cf.MINIMUM_LOCAL_RMSE_IMPROVEMENT,
        )
        self.assertFalse(substitution["passed"])
        self.assertIn(
            "global_rmse_regression",
            substitution["failure_reasons"],
        )

    def test_nonfinite_and_out_of_range_depth_intervals_fall_back(self):
        estimates, truths = self._poses()
        frames = [
            frame_cf.PlanarFrame(
                "rgbd_frame_gicp", 1.0, math.nan, 0.02, 0.0
            ),
            frame_cf.PlanarFrame(
                "rgbd_frame_gicp", 1.2, 0.36, 0.02, 0.0
            ),
        ]

        comparisons, counters = frame_cf.build_comparisons(
            frames, estimates, truths
        )

        self.assertEqual(comparisons, [])
        self.assertEqual(counters["accepted_rgbd"], 1)
        self.assertEqual(counters["invalid_interval"], 1)


if __name__ == "__main__":
    unittest.main()

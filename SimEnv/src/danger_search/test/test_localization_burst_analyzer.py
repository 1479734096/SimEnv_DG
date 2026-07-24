"""定位短窗分歧离线分析器测试。"""

import csv
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest


SIMENV_ROOT = Path(__file__).resolve().parents[3]


def _load_script(name: str, relative_path: str):
    path = SIMENV_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


burst = _load_script(
    "analyze_localization_burst_windows",
    "tools/offline/analyze_localization_burst_windows.py",
)
drift = _load_script(
    "analyze_localization_drift_bag",
    "tools/offline/analyze_localization_drift_bag.py",
)


FIELDS = (
    "truth_distance",
    "rgbd_x",
    "rgbd_y",
    "livox_x",
    "livox_y",
    "rgbd_coverage",
    "livox_coverage",
    "rgbd_error_norm",
    "livox_error_norm",
)


class LocalizationBurstAnalyzerTest(unittest.TestCase):
    def _write_rows(self, root: Path, rows, *, fields=FIELDS) -> Path:
        path = root / "windows.csv"
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        return path

    @staticmethod
    def _row(score: float, rgbd_error: float, livox_error: float):
        return {
            "truth_distance": 0.10,
            "rgbd_x": score,
            "rgbd_y": 0.0,
            "livox_x": 0.0,
            "livox_y": 0.0,
            "rgbd_coverage": 1.0,
            "livox_coverage": 1.0,
            "rgbd_error_norm": rgbd_error,
            "livox_error_norm": livox_error,
        }

    def test_frozen_separation_passes_complete_synthetic_run(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows = [
                self._row(0.01 + index * 0.001, 0.01, 0.01)
                for index in range(8)
            ]
            rows.extend(
                self._row(0.20 + index * 0.001, 0.10, 0.02)
                for index in range(8)
            )
            path = self._write_rows(root, rows)

            result = burst.evaluate(
                [("validation", path)],
                [("confirmation", path)],
            )

        self.assertTrue(result["passed"])
        self.assertEqual(result["runs"][0]["auc"], 1.0)
        self.assertGreaterEqual(
            result["runs"][0]["bootstrap_ci95_low"],
            burst.MINIMUM_BOOTSTRAP_LOWER,
        )

    def test_middle_ineligible_and_boundary_samples_follow_frozen_rules(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows = [
                self._row(0.01, burst.LOW_ERROR_MAXIMUM, 0.01),
                self._row(0.20, burst.HIGH_ERROR_MINIMUM, 0.01),
                self._row(0.10, 0.03, 0.01),
                {
                    **self._row(0.20, 0.10, 0.01),
                    "rgbd_coverage": 0.59,
                },
            ]
            path = self._write_rows(root, rows)

            samples, counters = burst.load_labeled_windows(path)

        self.assertEqual(len(samples), 2)
        self.assertEqual(sum(sample.high_error for sample in samples), 1)
        self.assertEqual(counters["middle_error"], 1)
        self.assertEqual(counters["ineligible"], 1)

    def test_insufficient_or_reversed_classes_fail_without_threshold_change(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows = [
                self._row(0.20, 0.01, 0.01)
                for _ in range(8)
            ]
            rows.extend(
                self._row(0.01, 0.10, 0.02)
                for _ in range(8)
            )
            path = self._write_rows(root, rows)

            result = burst.analyze_run("reversed", path, "validation")

        self.assertFalse(result["passed"])
        self.assertIn("auc", result["failure_reasons"])

    def test_corrupt_csv_is_rejected_explicitly(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = self._write_rows(
                root,
                [{"truth_distance": 0.10}],
                fields=("truth_distance",),
            )
            with self.assertRaisesRegex(ValueError, "缺少字段"):
                burst.load_labeled_windows(path)

    def test_drift_elapsed_uses_first_aligned_estimate_stamp(self):
        Pose = drift.PoseSample
        pairs = [
            (Pose(4.626, 0.0, 0.0, 0.0), Pose(4.626, 0.0, 0.0, 0.0)),
            (Pose(78.530, 1.0, 0.0, 0.0), Pose(78.530, 0.0, 0.0, 0.0)),
        ]

        samples = drift._compute_drift(pairs)

        self.assertAlmostEqual(samples[-1].elapsed, 73.904)
        self.assertAlmostEqual(samples[-1].position_error, 1.0)
        self.assertTrue(math.isfinite(samples[-1].truth_distance))


if __name__ == "__main__":
    unittest.main()

import tempfile
import unittest
from pathlib import Path

from danger_search.evaluation_tools import (
    SUMMARY_FIELDS,
    apply_param_overrides,
    count_duplicate_detections,
    read_summary_csv,
    write_json,
    write_summary_csv,
)


class EvaluationToolsTest(unittest.TestCase):
    def test_param_aliases_match_current_config_keys(self):
        base = {
            "tracker": {"nms_radius": 0.9, "min_hits": 4, "min_confidence": 0.75},
            "mission": {"max_runtime": 590.0},
            "controller": {"max_linear": 0.35},
        }
        merged = apply_param_overrides(
            base,
            {
                "detection.nms_radius": 1.0,
                "detection.confirm_hits": 5,
                "mission.total_timeout": 500.0,
                "controller.max_linear_vel": 0.42,
            },
        )

        self.assertEqual(merged["tracker"]["nms_radius"], 1.0)
        self.assertEqual(merged["tracker"]["min_hits"], 5)
        self.assertEqual(merged["mission"]["max_runtime"], 500.0)
        self.assertEqual(merged["controller"]["max_linear"], 0.42)

    def test_duplicate_detection_count_is_offline_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            truth = root / "danger_truth.json"
            detected = root / "detected_danger.json"
            write_json(
                truth,
                {"danger_sources": [{"position": [0.0, 0.0, 0.0]}, {"position": [5.0, 0.0, 0.0]}]},
            )
            write_json(
                detected,
                {
                    "exploration_time": 10.0,
                    "detected_danger_sources": [
                        {"position": [0.1, 0.0, 0.0]},
                        {"position": [0.2, 0.0, 0.0]},
                        {"position": [9.0, 0.0, 0.0]},
                    ],
                },
            )

            self.assertEqual(count_duplicate_detections(truth, detected, threshold=1.0), 1)

    def test_summary_csv_contains_required_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            runs_dir = root / "runs"
            run_dir = runs_dir / "20260707_120000_seed_42"
            run_dir.mkdir(parents=True)
            write_json(
                run_dir / "mission_summary.json",
                {
                    "seed": 42,
                    "scene_type": "small",
                    "param_version": "params_000",
                    "timeout": 620,
                    "max_floor": 3,
                    "completed": True,
                },
            )
            write_json(
                run_dir / "evaluation.json",
                {
                    "metrics": {
                        "truth_count": 2,
                        "detected_count": 2,
                        "correct": 1,
                        "missed": 1,
                        "false_alarms": 1,
                        "exploration_time": 12.5,
                        "threshold_used": 1.0,
                    },
                    "scores": {
                        "exploration_time_score": 15.0,
                        "recognition_probability_score": 0.0,
                        "false_alarm_rate_score": 0.0,
                        "technical_objective_total": 15.0,
                    },
                },
            )
            output = root / "summary.csv"
            rows = write_summary_csv(runs_dir, output)

            self.assertEqual(len(rows), 1)
            self.assertTrue(output.exists())
            loaded = read_summary_csv(output)
            self.assertEqual(set(SUMMARY_FIELDS), set(loaded[0].keys()))
            self.assertEqual(loaded[0]["run_id"], run_dir.name)
            self.assertEqual(loaded[0]["recognition_rate"], "0.5")
            self.assertEqual(loaded[0]["false_alarm_rate"], "0.5")

    def test_summary_marks_external_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            runs_dir = root / "runs"
            run_dir = runs_dir / "20260707_120000_seed_7"
            run_dir.mkdir(parents=True)
            write_json(
                run_dir / "mission_summary.json",
                {
                    "seed": 7,
                    "timeout": 90,
                    "elapsed_wall_time": 91.2,
                    "completed": False,
                    "external_timeout": True,
                    "failure_reason": [],
                },
            )

            rows = write_summary_csv(runs_dir, root / "summary.csv")

            self.assertEqual(rows[0]["total_time"], 91.2)
            self.assertIn("外部超时", rows[0]["failure_reason"])
            self.assertIn("探索超时", rows[0]["failure_reason"])


if __name__ == "__main__":
    unittest.main()

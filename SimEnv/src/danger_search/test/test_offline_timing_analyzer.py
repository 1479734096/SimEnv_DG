"""任务耗时离线分析工具测试。"""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import yaml


SIMENV_ROOT = Path(__file__).resolve().parents[3]
ANALYZER_PATH = SIMENV_ROOT / "tools" / "offline" / "analyze_mission_timing.py"
FORMAL_CONFIG_PATH = SIMENV_ROOT / "src" / "danger_search" / "config" / "danger_search.yaml"


def _load_analyzer_module():
    spec = importlib.util.spec_from_file_location("mission_timing_analyzer", ANALYZER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载耗时分析脚本: {ANALYZER_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _line(payload):
    return f"1.0 INFO /danger_search [source] {json.dumps(payload)}\n"


class OfflineTimingAnalyzerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.analyzer = _load_analyzer_module()

    def test_loader_keeps_only_latest_run(self):
        payloads = [
            {"event": "mission_runtime", "elapsed": 10.0},
            {
                "event": "mission_state_transition",
                "old_state": "INIT",
                "new_state": "WAIT_CONTROLLER_READY",
            },
            {"event": "mission_runtime", "elapsed": 0.5},
            {"event": "mission_runtime", "elapsed": 1.0},
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rosout.log"
            path.write_text("".join(_line(item) for item in payloads), encoding="utf-8")
            samples = self.analyzer.load_latest_runtime_samples(path)
        self.assertEqual([item["elapsed"] for item in samples], [0.5, 1.0])

    def test_analysis_separates_motion_and_low_displacement(self):
        samples = [
            {
                "event": "mission_runtime",
                "elapsed": 0.0,
                "state": "EXPLORE_FLOOR",
                "pose": [0.0, 0.0, 0.0],
                "current_task_phase": "GO_PASS_POINT",
                "controller": {
                    "linear_command": 0.25,
                    "lateral_command": 0.0,
                    "angular_command": 0.0,
                    "requested_linear_command": 0.25,
                    "requested_lateral_command": 0.0,
                    "recover_state": "idle",
                    "path_active": True,
                    "zero_command_reason": None,
                },
            },
            {
                "event": "mission_runtime",
                "elapsed": 0.5,
                "state": "EXPLORE_FLOOR",
                "pose": [0.005, 0.0, 0.0],
                "controller": {
                    "linear_command": 0.0,
                    "lateral_command": 0.0,
                    "angular_command": 0.5,
                    "requested_linear_command": 0.0,
                    "requested_lateral_command": 0.0,
                    "recover_state": "idle",
                    "path_active": True,
                    "zero_command_reason": "rotate",
                },
            },
            {
                "event": "mission_runtime",
                "elapsed": 1.0,
                "state": "EXPLORE_FLOOR",
                "pose": [0.005, 0.0, 0.0],
                "controller": {},
            },
        ]
        summary = self.analyzer.analyze_runtime_samples(samples)
        self.assertAlmostEqual(summary["motion_seconds"]["前进命令"], 0.5)
        self.assertAlmostEqual(summary["motion_seconds"]["原地转向"], 0.5)
        self.assertAlmostEqual(summary["task_phase_seconds"]["GO_PASS_POINT"], 0.5)
        self.assertAlmostEqual(summary["low_displacement_command_seconds"], 0.5)
        self.assertAlmostEqual(summary["trajectory_distance"], 0.005)
        self.assertAlmostEqual(summary["commanded_motion_seconds"], 0.5)
        self.assertEqual(summary["stuck_episode_count"], 0)

    def test_analysis_reports_coverage_distance_and_recovery_episodes(self):
        samples = []
        for index, (x, recover, stuck) in enumerate(
            [(0.0, "idle", 0), (0.1, "turn", 1), (0.2, "idle", 0)]
        ):
            samples.append(
                {
                    "event": "mission_runtime",
                    "elapsed": 0.5 * index,
                    "state": "EXPLORE_FLOOR",
                    "pose": [x, 0.0, 0.0],
                    "coverage_ratio": 0.2 + 0.1 * index,
                    "controller": {
                        "linear_command": 0.2,
                        "lateral_command": 0.0,
                        "angular_command": 0.0,
                        "requested_linear_command": 0.2,
                        "requested_lateral_command": 0.0,
                        "recover_state": recover,
                        "stuck_count": stuck,
                        "path_active": True,
                    },
                }
            )
        summary = self.analyzer.analyze_runtime_samples(samples)
        self.assertAlmostEqual(summary["trajectory_distance"], 0.2)
        self.assertAlmostEqual(summary["effective_motion_ratio"], 1.0)
        self.assertEqual(summary["stuck_episode_count"], 1)
        self.assertEqual(summary["recovery_episode_count"], 1)
        self.assertAlmostEqual(summary["maximum_coverage_ratio"], 0.4)

    def test_trajectory_excludes_floor_change_and_localization_jump(self):
        samples = [
            {
                "event": "mission_runtime",
                "elapsed": 0.0,
                "state": "EXPLORE_FLOOR",
                "current_floor": 0,
                "pose": [0.0, 0.0, 0.0],
                "controller": {"stuck_count": 0, "recover_count": 0},
            },
            {
                "event": "mission_runtime",
                "elapsed": 0.5,
                "state": "CHANGE_FLOOR",
                "current_floor": 1,
                "pose": [0.8, 0.0, 0.0],
                "controller": {"stuck_count": 1, "recover_count": 1},
            },
            {
                "event": "mission_runtime",
                "elapsed": 1.0,
                "state": "EXPLORE_FLOOR",
                "current_floor": 1,
                "pose": [2.5, 0.0, 0.0],
                "controller": {"stuck_count": 2, "recover_count": 2},
            },
            {
                "event": "mission_runtime",
                "elapsed": 1.5,
                "state": "EXPLORE_FLOOR",
                "current_floor": 1,
                "pose": [2.6, 0.0, 0.0],
                "controller": {"stuck_count": 2, "recover_count": 2},
            },
        ]
        summary = self.analyzer.analyze_runtime_samples(samples)
        self.assertAlmostEqual(summary["trajectory_distance"], 0.1)
        self.assertAlmostEqual(summary["raw_trajectory_distance"], 2.6)
        self.assertEqual(
            summary["excluded_trajectory_segments"],
            {"floor_change": 1, "localization_jump": 1},
        )
        self.assertEqual(
            summary["excluded_trajectory_distance"],
            {"floor_change": 0.8, "localization_jump": 1.7},
        )
        self.assertEqual(
            summary["coverage_by_floor"],
            {},
        )
        self.assertEqual(summary["stuck_episode_count"], 2)
        self.assertEqual(summary["recovery_episode_count"], 2)

    def test_counter_increase_in_last_sample_is_not_lost(self):
        samples = [
            {
                "event": "mission_runtime",
                "elapsed": 0.0,
                "state": "EXPLORE_FLOOR",
                "pose": [0.0, 0.0, 0.0],
                "controller": {
                    "stuck_count": 0,
                    "recover_count": 0,
                    "recover_state": "idle",
                },
            },
            {
                "event": "mission_runtime",
                "elapsed": 0.5,
                "state": "EXPLORE_FLOOR",
                "pose": [0.1, 0.0, 0.0],
                "controller": {
                    "stuck_count": 1,
                    "recover_count": 1,
                    "recover_state": "turn",
                },
            },
        ]
        summary = self.analyzer.analyze_runtime_samples(samples)
        self.assertEqual(summary["stuck_episode_count"], 1)
        self.assertEqual(summary["recovery_episode_count"], 1)

    def test_run_classification_prefers_mission_terminal_reason(self):
        records = [
            {
                "event": "mission_result_write",
                "terminal_outcome": "failed",
                "result_write_reason": "localization_stance_unhealthy",
            }
        ]
        outcome = self.analyzer.classify_run_failure(records, "段错误")
        self.assertEqual(outcome["category"], "定位健康失败")
        self.assertEqual(outcome["reason"], "localization_stance_unhealthy")

    def test_run_classification_groups_floor_change_failure(self):
        outcome = self.analyzer.classify_run_failure(
            [
                {
                    "event": "mission_result_write",
                    "terminal_outcome": "failed",
                    "result_write_reason": "floor_change_failed_return_home",
                }
            ]
        )
        self.assertEqual(outcome["category"], "电梯或换层失败")

    def test_run_classification_recognizes_success(self):
        outcome = self.analyzer.classify_run_failure(
            [
                {
                    "event": "mission_result_write",
                    "terminal_outcome": "success",
                    "result_write_reason": "return_home_completed",
                }
            ]
        )
        self.assertEqual(outcome["category"], "任务成功")
        self.assertEqual(outcome["terminal_outcome"], "success")

    def test_commanded_motion_requires_an_emitted_translation_command(self):
        samples = [
            {
                "event": "mission_runtime",
                "elapsed": 0.0,
                "state": "EXPLORE_FLOOR",
                "pose": [0.0, 0.0, 0.0],
                "controller": {
                    "linear_command": 0.0,
                    "lateral_command": 0.0,
                    "requested_linear_command": 0.2,
                    "requested_lateral_command": 0.0,
                },
            },
            {
                "event": "mission_runtime",
                "elapsed": 0.5,
                "state": "EXPLORE_FLOOR",
                "pose": [0.0, 0.0, 0.0],
                "controller": {},
            },
        ]
        summary = self.analyzer.analyze_runtime_samples(samples)
        self.assertEqual(summary["commanded_motion_seconds"], 0.0)
        self.assertAlmostEqual(summary["low_displacement_command_seconds"], 0.5)

    def test_run_classification_recognizes_controller_crash_without_mission_result(self):
        outcome = self.analyzer.classify_run_failure([], "控制器发生段错误")
        self.assertEqual(outcome["category"], "控制器进程崩溃")
        self.assertEqual(outcome["terminal_outcome"], "environment_failed")

    def test_analyze_run_keeps_early_startup_failure_without_runtime_samples(self):
        summary = self.analyzer.analyze_run([], auto_log_text="控制器发生段错误")
        self.assertEqual(summary["sample_count"], 0)
        self.assertIsNone(summary["sampled_duration"])
        self.assertEqual(summary["run_outcome"]["category"], "控制器进程崩溃")
        self.assertIsNone(summary["trajectory_distance"])

    def test_run_classification_uses_test_summary_for_fixture_and_startup_failure(self):
        fixture = self.analyzer.classify_run_failure(
            [],
            test_summary={"result": "失败", "detail": "FIFO 测试夹具错误"},
        )
        startup = self.analyzer.classify_run_failure(
            [],
            test_summary={
                "result": "失败",
                "detail": "Gazebo 或关节控制链未在 180 秒内就绪",
            },
        )
        self.assertEqual(fixture["category"], "测试夹具错误")
        self.assertEqual(startup["category"], "模型或控制器反馈未就绪")

    def test_analyze_run_counts_events_and_official_results(self):
        runtime = {
            "event": "mission_runtime",
            "state": "EXPLORE_FLOOR",
            "pose": [0.0, 0.0, 0.0],
            "controller": {},
        }
        records = [dict(runtime, elapsed=0.0), dict(runtime, elapsed=0.5)]
        with tempfile.TemporaryDirectory() as directory:
            result_file = Path(directory) / "detected_danger.json"
            result_file.write_text(
                json.dumps(
                    {
                        "exploration_time": 1.0,
                        "detected_danger_sources": [{"position": [0, 0, 0]}],
                    }
                ),
                encoding="utf-8",
            )
            summary = self.analyzer.analyze_run(records, result_file=result_file)
        self.assertEqual(summary["event_counts"]["mission_runtime"], 2)
        self.assertEqual(summary["detected_source_count"], 1)

    def test_analyze_run_reports_safety_events_and_missing_sidecars(self):
        runtime = {
            "event": "mission_runtime",
            "state": "EXPLORE_FLOOR",
            "pose": [0.0, 0.0, 0.0],
            "controller": {},
        }
        records = [
            dict(runtime, elapsed=0.0),
            {"event": "ground_support_gate_transition"},
            {"event": "unsafe_rotation_blocked"},
            dict(runtime, elapsed=0.5),
        ]
        summary = self.analyzer.analyze_run(records)
        self.assertEqual(
            summary["safety_event_counts"],
            {
                "ground_support_gate_transition": 1,
                "unsafe_rotation_blocked": 1,
            },
        )
        self.assertIsNone(summary["detected_source_count"])
        self.assertIsNone(summary["localization_error"])
        self.assertIsNone(summary["evaluation"])

    def test_analyze_run_reads_localization_and_evaluation_sidecars(self):
        runtime = {
            "event": "mission_runtime",
            "state": "EXPLORE_FLOOR",
            "pose": [0.0, 0.0, 0.0],
            "controller": {},
        }
        records = [dict(runtime, elapsed=0.0), dict(runtime, elapsed=0.5)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            localization = root / "localization_drift.json"
            localization.write_text(
                json.dumps(
                    {
                        "sample_count": 10,
                        "rms_position_error": 0.12,
                        "p95_position_error": 0.2,
                        "maximum_position_error": 0.3,
                    }
                ),
                encoding="utf-8",
            )
            evaluation = root / "evaluation_result.json"
            evaluation.write_text(
                json.dumps(
                    {
                        "metrics": {"correct": 4, "detected_count": 5},
                        "scores": {"technical_objective_total": 30.5},
                    }
                ),
                encoding="utf-8",
            )
            summary = self.analyzer.analyze_run(
                records,
                localization_report=localization,
                evaluation_file=evaluation,
            )
        self.assertAlmostEqual(
            summary["localization_error"]["rms_position_error"], 0.12
        )
        self.assertEqual(summary["evaluation"]["metrics"]["correct"], 4.0)
        self.assertAlmostEqual(
            summary["evaluation"]["scores"]["technical_objective_total"], 30.5
        )

    def test_invalid_sidecars_are_reported_as_missing(self):
        runtime = {
            "event": "mission_runtime",
            "state": "EXPLORE_FLOOR",
            "pose": [0.0, 0.0, 0.0],
            "controller": {},
        }
        records = [dict(runtime, elapsed=0.0), dict(runtime, elapsed=0.5)]
        with tempfile.TemporaryDirectory() as directory:
            invalid = Path(directory) / "invalid.json"
            invalid.write_text("{", encoding="utf-8")
            summary = self.analyzer.analyze_run(
                records,
                result_file=invalid,
                localization_report=invalid,
                evaluation_file=invalid,
            )
        self.assertIsNone(summary["detected_source_count"])
        self.assertIsNone(summary["localization_error"])
        self.assertIsNone(summary["evaluation"])

    def test_duration_boundaries_and_per_floor_coverage(self):
        def sample(elapsed, floor, coverage, x):
            return {
                "event": "mission_runtime",
                "elapsed": elapsed,
                "state": "EXPLORE_FLOOR",
                "current_floor": floor,
                "coverage_ratio": coverage,
                "pose": [x, 0.0, 0.0],
                "controller": {
                    "linear_command": 0.2,
                    "lateral_command": 0.0,
                },
            }

        summary = self.analyzer.analyze_runtime_samples(
            [
                sample(0.0, 0, 0.1, 0.0),
                sample(0.0, 0, 0.2, 0.0),
                sample(5.0, 0, 0.3, 0.5),
                sample(10.1, 1, 0.4, 1.0),
            ]
        )
        self.assertAlmostEqual(summary["sampled_duration"], 10.1)
        self.assertAlmostEqual(summary["active_seconds"], 5.0)
        self.assertAlmostEqual(summary["commanded_motion_seconds"], 5.0)
        self.assertAlmostEqual(summary["peak_commanded_speed"], 0.2)
        self.assertEqual(
            summary["coverage_by_floor"],
            {
                "0": {"initial": 0.1, "final": 0.3, "maximum": 0.3},
                "1": {"initial": 0.4, "final": 0.4, "maximum": 0.4},
            },
        )

    def test_analysis_rejects_incomplete_input(self):
        with self.assertRaisesRegex(ValueError, "至少需要两份"):
            self.analyzer.analyze_runtime_samples([{"elapsed": 0.0}])
        with self.assertRaisesRegex(ValueError, "必须为正"):
            self.analyzer.analyze_run(
                [],
                auto_log_text="控制器发生段错误",
                max_trajectory_jump_meters=0.0,
            )

    def test_door_speed_stays_inside_verified_safety_envelope(self):
        config = yaml.safe_load(FORMAL_CONFIG_PATH.read_text(encoding="utf-8"))
        door_speed = float(config["mission"]["door_pass_max_linear"])
        controller_speed = float(config["controller"]["max_linear"])
        verified_speed = float(config["ground_support"]["stop_distance_table"][-1][0])
        self.assertGreaterEqual(door_speed, 0.30)
        self.assertLessEqual(door_speed, controller_speed)
        self.assertLessEqual(door_speed, verified_speed)

    def test_return_budget_keeps_a_positive_measured_margin(self):
        config = yaml.safe_load(FORMAL_CONFIG_PATH.read_text(encoding="utf-8"))
        self.assertGreaterEqual(float(config["mission"]["return_safety_buffer"]), 5.0)


if __name__ == "__main__":
    unittest.main()

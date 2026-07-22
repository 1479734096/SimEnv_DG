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

    def test_analysis_rejects_incomplete_input(self):
        with self.assertRaisesRegex(ValueError, "至少需要两份"):
            self.analyzer.analyze_runtime_samples([{"elapsed": 0.0}])

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
        self.assertGreaterEqual(float(config["mission"]["return_safety_buffer"]), 20.0)


if __name__ == "__main__":
    unittest.main()

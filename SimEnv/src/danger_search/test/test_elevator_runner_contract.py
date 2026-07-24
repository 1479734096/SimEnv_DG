"""电梯专项 runner 的楼层参数契约测试。"""

from pathlib import Path
import os
import subprocess
import unittest


SIMENV_ROOT = Path(__file__).resolve().parents[3]
RUNNER = SIMENV_ROOT / "tools" / "sim" / "run_elevator_gui_test.sh"


class ElevatorRunnerContractTest(unittest.TestCase):
    def test_help_documents_floor_count_without_starting_ros(self):
        result = subprocess.run(
            ["bash", str(RUNNER), "--help"],
            cwd=SIMENV_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 0)
        self.assertIn("ELEVATOR_TEST_FLOOR_COUNT=2", result.stdout)
        self.assertIn(
            "ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS=1",
            result.stdout,
        )
        self.assertIn("到达显式楼层数的最高层", result.stdout)
        self.assertNotIn("完成 0→1 电梯任务", result.stdout)

    def test_invalid_floor_count_fails_before_creating_evidence(self):
        environment = dict(os.environ)
        environment["ELEVATOR_TEST_FLOOR_COUNT"] = "4"
        result = subprocess.run(
            ["bash", str(RUNNER)],
            cwd=SIMENV_ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn("只能是 2 或 3", result.stderr)

    def test_runner_passes_floor_count_and_tracks_highest_floor(self):
        source = RUNNER.read_text(encoding="utf-8")

        self.assertIn('FLOOR_COUNT="$ELEVATOR_TEST_FLOOR_COUNT"', source)
        self.assertIn(
            "highest_floor=$((ELEVATOR_TEST_FLOOR_COUNT - 1))",
            source,
        )
        self.assertIn('grep -F "\\"current_floor\\": $highest_floor"', source)
        self.assertIn("0→1→2 换层", source)
        self.assertIn(
            "landmark_drift_diagnostic_enabled: $LANDMARK_DIAGNOSTIC_VALUE",
            source,
        )
        self.assertIn("analyze_landmark_drift.py", source)
        self.assertIn('terminate_group "$KEEPER_PID" KILL', source)
        self.assertIn('wait "$KEEPER_PID" 2>/dev/null || true', source)

    def test_landmark_diagnostic_requires_isolated_truth_recording(self):
        environment = dict(os.environ)
        environment["ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS"] = "1"
        environment["ELEVATOR_TEST_RECORD_DRIFT"] = "0"
        environment["ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH"] = "0"
        result = subprocess.run(
            ["bash", str(RUNNER)],
            cwd=SIMENV_ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn("必须同时录制定位和独立离线真值", result.stderr)


if __name__ == "__main__":
    unittest.main()

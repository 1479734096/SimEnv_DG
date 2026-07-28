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
        self.assertIn(
            "ELEVATOR_TEST_LOCALIZATION_BACKEND=adaptive_shared_map", result.stdout
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

    def test_invalid_localization_backend_fails_before_starting_ros(self):
        environment = dict(os.environ)
        environment["ELEVATOR_TEST_LOCALIZATION_BACKEND"] = "unknown"
        result = subprocess.run(
            ["bash", str(RUNNER)],
            cwd=SIMENV_ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn(
            "仅支持 kiss_icp、imu_constrained、livox_imu_local_map、"
            "adaptive_shared_map 或 rgbd_livox",
            result.stderr,
        )

    def test_runner_accepts_adaptive_shared_map_without_starting_ros(self):
        environment = dict(os.environ)
        environment["ELEVATOR_TEST_LOCALIZATION_BACKEND"] = "adaptive_shared_map"
        environment["ELEVATOR_TEST_RECORD_DRIFT"] = "2"
        result = subprocess.run(
            ["bash", str(RUNNER)],
            cwd=SIMENV_ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn("诊断开关只能是 0 或 1", result.stderr)
        self.assertNotIn("LOCALIZATION_BACKEND 仅支持", result.stderr)

    def test_runner_passes_localization_backend_to_formal_launch(self):
        source = RUNNER.read_text(encoding="utf-8")

        self.assertIn(
            'ELEVATOR_TEST_LOCALIZATION_BACKEND="${ELEVATOR_TEST_LOCALIZATION_BACKEND:-adaptive_shared_map}"',
            source,
        )
        self.assertIn(
            'localization_backend:="$ELEVATOR_TEST_LOCALIZATION_BACKEND"', source
        )
        self.assertIn(
            "printf 'localization_backend=%s\\n' \"$ELEVATOR_TEST_LOCALIZATION_BACKEND\"",
            source,
        )


if __name__ == "__main__":
    unittest.main()

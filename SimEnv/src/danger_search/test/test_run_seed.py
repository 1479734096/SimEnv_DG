"""run_seed 终止语义的离线测试。"""
from __future__ import annotations

import importlib.util
from pathlib import Path
import signal
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


RUN_SEED_PATH = Path(__file__).resolve().parents[1] / "tools" / "run_seed.py"
SPEC = importlib.util.spec_from_file_location("danger_search_run_seed", RUN_SEED_PATH)
assert SPEC is not None and SPEC.loader is not None
run_seed = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = run_seed
SPEC.loader.exec_module(run_seed)


class RunSeedCompletionTest(unittest.TestCase):
    def test_environment_cleanup_uses_recorded_group_after_shell_exit(self):
        child = SimpleNamespace(
            pid=12345,
            isalive=lambda: False,
            close=lambda force: None,
        )

        with patch.object(run_seed.os, "getpgrp", return_value=99999), patch.object(
            run_seed.os,
            "killpg",
            side_effect=[None, ProcessLookupError(), ProcessLookupError()],
        ) as killpg:
            run_seed.stop_environment(child)

        self.assertEqual(killpg.call_args_list[0].args, (12345, signal.SIGINT))

    def test_environment_output_is_drained_until_timeout(self):
        import pexpect

        child = SimpleNamespace(
            read_nonblocking=unittest.mock.Mock(
                side_effect=["first", "second", pexpect.TIMEOUT("empty")]
            )
        )

        drained = run_seed.drain_environment_output(child)

        self.assertEqual(drained, len("firstsecond"))
        self.assertEqual(child.read_nonblocking.call_count, 3)

    def test_external_shutdown_result_is_not_completed(self):
        summary = {
            "completed": False,
            "terminal_outcome": "failed",
            "terminal_state": "FAILED",
            "terminal_reason": "external_ros_shutdown",
            "return_home_started": False,
            "return_home_completed": False,
            "return_home_success": False,
            "timeout_triggered": False,
        }

        self.assertFalse(run_seed.runtime_completed_successfully(summary))

    def test_legacy_finish_summary_is_not_completed(self):
        self.assertFalse(
            run_seed.runtime_completed_successfully(
                {"state": "FINISH", "return_home_success": False}
            )
        )

    def test_only_successful_return_home_is_completed(self):
        summary = {
            "completed": True,
            "terminal_outcome": "success",
            "terminal_state": "FINISH",
            "terminal_reason": "return_home_completed",
            "return_home_started": True,
            "return_home_completed": True,
            "return_home_success": True,
            "timeout_triggered": False,
        }

        self.assertTrue(run_seed.runtime_completed_successfully(summary))


class StartupPostureGateTest(unittest.TestCase):
    def test_identity_quaternion_is_zero_rpy(self):
        roll, pitch = run_seed._quaternion_to_rpy(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)

    def test_quaternion_half_pi_roll(self):
        import math
        # 绕 x 轴旋转 +90°
        roll, pitch = run_seed._quaternion_to_rpy(
            math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)
        )
        self.assertAlmostEqual(roll, math.pi / 2, places=5)
        self.assertAlmostEqual(pitch, 0.0, places=5)

    def test_normal_standup_passes_gate(self):
        stats = {
            "samples": 500,
            "max_abs_roll": 0.066,
            "max_abs_pitch": 0.041,
            "min_z": 0.18,
        }
        self.assertIsNone(run_seed._posture_breach_detail(stats))

    def test_flipped_robot_fails_gate(self):
        stats = {
            "samples": 100,
            "max_abs_roll": 3.14,
            "max_abs_pitch": 0.32,
            "min_z": 0.15,
        }
        detail = run_seed._posture_breach_detail(stats)
        self.assertIsNotNone(detail)
        self.assertIn("startup_posture_failure", detail)
        self.assertIn("max_abs_roll=3.1400", detail)

    def test_low_z_fails_gate(self):
        stats = {
            "samples": 100,
            "max_abs_roll": 0.05,
            "max_abs_pitch": 0.04,
            "min_z": 0.08,
        }
        detail = run_seed._posture_breach_detail(stats)
        self.assertIsNotNone(detail)
        self.assertIn("min_z=0.0800", detail)

    def test_no_samples_fails_gate(self):
        detail = run_seed._posture_breach_detail(
            {"samples": 0, "max_abs_roll": 0.0, "max_abs_pitch": 0.0, "min_z": None}
        )
        self.assertIsNotNone(detail)
        self.assertIn("samples=0", detail)

    def test_boundary_roll_not_breached(self):
        # 阈值使用严格大于，恰好 0.5 不触发
        stats = {
            "samples": 10,
            "max_abs_roll": run_seed.POSTURE_MAX_ABS_ROLL,
            "max_abs_pitch": 0.0,
            "min_z": 0.20,
        }
        self.assertIsNone(run_seed._posture_breach_detail(stats))


if __name__ == "__main__":
    unittest.main()

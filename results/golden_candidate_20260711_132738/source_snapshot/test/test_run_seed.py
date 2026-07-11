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


if __name__ == "__main__":
    unittest.main()

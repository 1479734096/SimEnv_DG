"""run_seed 终止语义的离线测试。"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


RUN_SEED_PATH = Path(__file__).resolve().parents[1] / "tools" / "run_seed.py"
SPEC = importlib.util.spec_from_file_location("danger_search_run_seed", RUN_SEED_PATH)
assert SPEC is not None and SPEC.loader is not None
run_seed = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = run_seed
SPEC.loader.exec_module(run_seed)


def _make_run_summary(
    *,
    completed: bool = True,
    exploration_completed: bool = True,
    terminal_reason: str = "return_home_completed",
    timeout_triggered: bool = False,
) -> dict:
    return {
        "completed": completed,
        "exploration_completed": exploration_completed,
        "terminal_outcome": "success" if completed else "failed",
        "terminal_state": "FINISH" if completed else "FAILED",
        "terminal_reason": terminal_reason,
        "return_home_started": True,
        "return_home_completed": True,
        "return_home_success": True,
        "timeout_triggered": timeout_triggered,
    }


def _make_fake_env_process():
    return SimpleNamespace(
        pid=12345,
        isalive=lambda: True,
        close=lambda force: None,
        owned_environment_processes=[],
    )


class RunSeedCompletionTest(unittest.TestCase):
    def test_runner_configures_workspace_ros_path_without_prior_source(self):
        workspace = Path("/tmp/simenv-workspace")
        with patch.dict(
            run_seed.os.environ,
            {"ROS_PACKAGE_PATH": "/opt/ros/noetic/share:/existing/packages"},
        ):
            run_seed.configure_workspace_ros_package_path(workspace)
            run_seed.configure_workspace_ros_package_path(workspace)
            paths = run_seed.os.environ["ROS_PACKAGE_PATH"].split(run_seed.os.pathsep)

        self.assertEqual(paths[0], str(workspace / "src"))
        self.assertEqual(paths.count(str(workspace / "src")), 1)
        self.assertIn("/opt/ros/noetic/share", paths)
        self.assertIn("/existing/packages", paths)

    def test_environment_launch_starts_paused_without_auto_unpause(self):
        overrides = run_seed.environment_launch_overrides(
            seed=42,
            gui_enabled=False,
            max_floor=1,
            run_pid_dir=Path("/tmp/run-pids"),
        )

        self.assertEqual(overrides["PAUSED"], "true")
        self.assertEqual(overrides["AUTO_UNPAUSE"], "false")
        self.assertEqual(overrides["SEED"], "42")
        self.assertEqual(overrides["FLOOR_COUNT"], "1")
        self.assertEqual(overrides["POINTCLOUD_USE_GROUND_TRUTH_ODOM"], "false")
        self.assertEqual(overrides["ENABLE_REFEREE_ODOM"], "false")
        self.assertEqual(overrides["ENABLE_GROUND_TRUTH"], "true")

    def test_environment_unpauses_after_livox_before_fixed_stand(self):
        events = []

        class FakeChild:
            pid = 12345

            def expect(self, pattern, timeout):
                events.append(("expect", pattern))

            def send(self, value):
                events.append(("send", value))

            def isalive(self):
                return True

            def close(self, force):
                events.append(("close", force))

        child = FakeChild()

        def fake_unpause(_workspace):
            events.append(("unpause", None))

        def fake_wait_for_publisher(topic, *, timeout):
            events.append(("publisher", topic))

        with patch("pexpect.spawn", return_value=child) as spawn, patch.object(
            run_seed,
            "wait_for_ros_topic_publisher",
            side_effect=fake_wait_for_publisher,
        ), patch.object(
            run_seed, "unpause_environment_physics", side_effect=fake_unpause
        ), patch.object(
            run_seed,
            "wait_for_fixed_stand_posture",
            return_value={"stable": True},
        ), patch.object(run_seed, "check_startup_health"), patch.object(
            run_seed, "capture_environment_processes", return_value=[]
        ), patch.object(run_seed.time, "sleep"):
            returned = run_seed.run_environment(
                42,
                Path("/workspace"),
                log_file=None,
                gui_enabled=False,
                health_log_path=Path("/workspace/health.log"),
                max_floor=1,
            )

        self.assertIs(returned, child)
        spawn_env = spawn.call_args.kwargs["env"]
        self.assertEqual(spawn_env["PAUSED"], "true")
        self.assertEqual(spawn_env["AUTO_UNPAUSE"], "false")
        self.assertLess(events.index(("publisher", "/scan")), events.index(("unpause", None)))
        self.assertLess(events.index(("unpause", None)), events.index(("send", "2")))
        self.assertLess(events.index(("send", "2")), events.index(("send", "6")))

    def test_unpause_uses_gazebo_service_with_timeout(self):
        with patch.object(run_seed.subprocess, "run") as call:
            run_seed.unpause_environment_physics(Path("/workspace"), timeout=7)

        command = call.call_args.args[0]
        self.assertEqual(command[:2], ["bash", "-lc"])
        self.assertIn("rosservice call /gazebo/unpause_physics", command[2])
        self.assertEqual(call.call_args.kwargs["timeout"], 7)
        self.assertTrue(call.call_args.kwargs["check"])

    def test_algorithm_health_failure_stops_owned_launch_process(self):
        process = SimpleNamespace(poll=lambda: None)
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            run_seed.subprocess, "Popen", return_value=process
        ), patch.object(
            run_seed,
            "check_startup_health",
            side_effect=RuntimeError("定位健康检查失败"),
        ), patch.object(run_seed, "stop_owned_process") as stop_process:
            with self.assertRaisesRegex(RuntimeError, "定位健康检查失败"):
                run_seed.run_danger_search(
                    workspace_dir=Path(temp_dir),
                    run_config_path=Path(temp_dir) / "params.yaml",
                    result_file=None,
                    mission_summary_file=None,
                    max_floor=1,
                    timeout=30,
                    log_path=Path(temp_dir) / "danger_search.log",
                    environment_process=_make_fake_env_process(),
                    gui_enabled=False,
                )

        stop_process.assert_called_once_with(process, label="danger_search")

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
        summary = _make_run_summary()

        self.assertTrue(run_seed.runtime_completed_successfully(summary))

    def test_corrupted_summary_is_invalid(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission_runtime_summary.json"
            path.write_text("not json", encoding="utf-8")
            self.assertIsNone(run_seed.read_and_validate_summary(path))

    def test_archive_existing_public_result_only_touches_result_file(self):
        with tempfile.TemporaryDirectory() as directory:
            results_dir = Path(directory) / "results"
            results_dir.mkdir()
            public_result = results_dir / "detected_danger.json"
            public_result.write_text("{}", encoding="utf-8")
            # 模拟一个不应被碰的环境文件
            metadata = Path(directory) / "generated_building" / "layout_metadata.json"
            metadata.parent.mkdir()
            metadata.write_text("{}", encoding="utf-8")
            run_dir = Path(directory) / "run"
            run_dir.mkdir()

            run_seed.archive_existing_public_result(public_result, run_dir)

            self.assertTrue((run_dir / "preexisting_detected_danger.json").exists())
            self.assertFalse(public_result.exists())
            self.assertTrue(metadata.exists())

    def test_archive_refuses_non_result_file(self):
        with tempfile.TemporaryDirectory() as directory:
            bad_file = Path(directory) / "danger_truth.json"
            bad_file.write_text("{}", encoding="utf-8")
            run_dir = Path(directory) / "run"
            run_dir.mkdir()
            with self.assertRaises(ValueError):
                run_seed.archive_existing_public_result(bad_file, run_dir)

    def _run_main_with_mocks(
        self,
        workspace: Path,
        run_danger_search_return: run_seed.RunTermination,
        write_result: bool = False,
        write_summary: dict | None = None,
    ) -> dict:
        """在最小临时工作区上运行 run_seed.main() 并返回 mission_summary。"""
        run_dir = workspace / "results" / "runs" / "20260716_120000_seed_42"
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "logs").mkdir(parents=True, exist_ok=True)
        public_result = workspace / "results" / "detected_danger.json"
        public_result.parent.mkdir(parents=True, exist_ok=True)
        summary_path = run_dir / "mission_runtime_summary.json"
        config_path = run_dir / "params.yaml"

        def fake_run_danger_search(*args, **kwargs):
            if write_result:
                public_result.write_text(json.dumps({"detected": True}), encoding="utf-8")
            if write_summary is not None:
                summary_path.write_text(json.dumps(write_summary), encoding="utf-8")
            return run_danger_search_return

        with patch.object(sys, "argv", ["run_seed.py", "--seed", "42", "--headless"]), patch.object(
            run_seed, "find_workspace_dir", return_value=workspace
        ), patch.object(run_seed, "create_run_dir", return_value=run_dir), patch.object(
            run_seed, "find_conflicting_processes", return_value=[]
        ), patch.object(run_seed, "run_environment", return_value=_make_fake_env_process()), patch.object(
            run_seed, "run_danger_search", side_effect=fake_run_danger_search
        ), patch.object(run_seed, "public_result_file", return_value=public_result), patch.object(
            run_seed, "write_effective_config", return_value={}
        ), patch.object(run_seed, "write_yaml", return_value=None), patch.object(
            run_seed, "copy_runtime_logs", return_value=None
        ), patch.object(run_seed, "evaluate_run", return_value=False):
            returncode = run_seed.main()

        self.assertTrue((run_dir / "mission_summary.json").exists())
        summary = json.loads((run_dir / "mission_summary.json").read_text(encoding="utf-8"))
        summary["_returncode"] = returncode
        return summary

    def test_main_archives_old_result_and_deletes_only_result_file(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            public_result = workspace / "results" / "detected_danger.json"
            public_result.parent.mkdir(parents=True, exist_ok=True)
            public_result.write_text("old", encoding="utf-8")
            termination = run_seed.RunTermination(0, "algorithm_exit", "algorithm_exit", "danger_search")
            summary = self._run_main_with_mocks(workspace, termination, write_result=False)
            run_dir = workspace / "results" / "runs" / "20260716_120000_seed_42"
            self.assertTrue((run_dir / "preexisting_detected_danger.json").exists())
            self.assertFalse(public_result.exists())
            self.assertFalse(summary["completed"])

    def test_missing_summary_with_zero_exit_and_result_is_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            termination = run_seed.RunTermination(0, "algorithm_exit", "algorithm_exit", "danger_search")
            summary = self._run_main_with_mocks(workspace, termination, write_result=True)
            self.assertFalse(summary["completed"])
            self.assertIn("缺少或损坏 mission_runtime_summary.json", summary["failure_reason"])

    def test_corrupted_summary_is_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            run_dir = workspace / "results" / "runs" / "20260716_120000_seed_42"
            run_dir.mkdir(parents=True, exist_ok=True)
            summary_path = run_dir / "mission_runtime_summary.json"
            summary_path.write_text("bad json", encoding="utf-8")
            termination = run_seed.RunTermination(0, "algorithm_exit", "algorithm_exit", "danger_search")
            summary = self._run_main_with_mocks(workspace, termination, write_result=True)
            self.assertFalse(summary["completed"])
            self.assertIn("缺少或损坏 mission_runtime_summary.json", summary["failure_reason"])

    def test_failure_summary_is_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            termination = run_seed.RunTermination(0, "algorithm_exit", "algorithm_exit", "danger_search")
            summary = self._run_main_with_mocks(
                workspace,
                termination,
                write_result=True,
                write_summary=_make_run_summary(completed=False),
            )
            self.assertFalse(summary["completed"])

    def test_runner_timeout_is_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            termination = run_seed.RunTermination(-1, "runner_timeout", "wall_timeout", "run_seed.py")
            summary = self._run_main_with_mocks(
                workspace,
                termination,
                write_result=True,
                write_summary=_make_run_summary(),
            )
            self.assertFalse(summary["completed"])
            self.assertTrue(summary["external_timeout"])

    def test_return_home_completed_is_mission_completed(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            termination = run_seed.RunTermination(0, "algorithm_exit", "algorithm_exit", "danger_search")
            summary = self._run_main_with_mocks(
                workspace,
                termination,
                write_result=True,
                write_summary=_make_run_summary(),
            )
            self.assertTrue(summary["completed"])
            self.assertTrue(summary["runtime_completed"])
            self.assertTrue(summary["runtime_exploration_completed"])
            self.assertFalse(summary["normal_budget_return"])

    def test_old_result_not_reused_when_not_rewritten(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            public_result = workspace / "results" / "detected_danger.json"
            public_result.parent.mkdir(parents=True, exist_ok=True)
            public_result.write_text("old", encoding="utf-8")
            termination = run_seed.RunTermination(0, "algorithm_exit", "algorithm_exit", "danger_search")
            summary = self._run_main_with_mocks(workspace, termination, write_result=False)
            self.assertFalse(summary["completed"])
            self.assertFalse(summary["result_written"])

    def test_budget_return_is_closed_loop_but_not_exploration_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            termination = run_seed.RunTermination(0, "algorithm_exit", "algorithm_exit", "danger_search")
            summary = self._run_main_with_mocks(
                workspace,
                termination,
                write_result=True,
                write_summary=_make_run_summary(
                    exploration_completed=False,
                    terminal_reason="return_home_completed",
                ),
            )
            self.assertTrue(summary["completed"])
            self.assertTrue(summary["runtime_completed"])
            self.assertFalse(summary["runtime_exploration_completed"])
            self.assertTrue(summary["normal_budget_return"])


class StartupPostureGateTest(unittest.TestCase):
    def test_algorithm_health_does_not_repeat_rospy_posture_probe(self):
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            run_seed, "wait_for_ros_topic"
        ) as wait_topic, patch.object(run_seed, "_probe_robot_posture") as posture_probe:
            log_path = Path(temp_dir) / "algorithm_health_check.log"
            run_seed.check_startup_health(
                Path(temp_dir),
                log_path,
                require_localization=True,
            )
            log_content = log_path.read_text(encoding="utf-8")

        self.assertEqual(wait_topic.call_count, 2)
        posture_probe.assert_not_called()
        self.assertIn("不重复采样", log_content)

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

    def test_crouched_tilted_startup_fails_before_algorithm_launch(self):
        detail = run_seed._posture_breach_detail(
            {
                "samples": 459,
                "max_abs_roll": 0.3476,
                "max_abs_pitch": 0.4354,
                "last_abs_roll": 0.31,
                "last_abs_pitch": 0.42,
                "min_z": None,
            }
        )

        self.assertIsNotNone(detail)
        self.assertIn("startup_posture_failure", detail)

    def test_height_truth_is_not_used_by_formal_posture_gate(self):
        stats = {
            "samples": 100,
            "max_abs_roll": 0.05,
            "max_abs_pitch": 0.04,
            "min_z": 0.08,
        }
        self.assertIsNone(run_seed._posture_breach_detail(stats))

    def test_no_samples_fails_gate(self):
        detail = run_seed._posture_breach_detail(
            {"samples": 0, "max_abs_roll": 0.0, "max_abs_pitch": 0.0, "min_z": None}
        )
        self.assertIsNotNone(detail)
        self.assertIn("samples=0", detail)

    def test_boundary_roll_not_breached(self):
        # 阈值使用严格大于，恰好等于门限不触发。
        stats = {
            "samples": 10,
            "max_abs_roll": run_seed.POSTURE_MAX_ABS_ROLL,
            "max_abs_pitch": 0.0,
            "min_z": 0.20,
        }
        self.assertIsNone(run_seed._posture_breach_detail(stats))

    def test_fixed_stand_requires_complete_recent_stable_window(self):
        complete_samples = [
            (8.99 + index * 0.01, 0.05, 0.04)
            for index in range(102)
        ]
        stable, count = run_seed._is_stable_posture_window(
            complete_samples,
            now=10.0,
            window_seconds=1.0,
        )
        self.assertTrue(stable)
        self.assertGreaterEqual(count, 100)

        stable, count = run_seed._is_stable_posture_window(
            [(9.2, 0.05, 0.04), (10.0, 0.06, 0.05)],
            now=10.0,
            window_seconds=1.0,
        )
        self.assertFalse(stable)
        self.assertEqual(count, 2)

    def test_fixed_stand_rejects_tilted_recent_sample(self):
        stable, _ = run_seed._is_stable_posture_window(
            [(9.0, 0.05, 0.04), (9.5, 0.08, 0.31), (10.0, 0.06, 0.05)],
            now=10.0,
            window_seconds=1.0,
        )
        self.assertFalse(stable)

    def test_health_check_reuses_fixed_stand_posture_without_ros_probe(self):
        posture = {
            "samples": 50,
            "max_abs_roll": 0.08,
            "max_abs_pitch": 0.07,
            "last_abs_roll": 0.05,
            "last_abs_pitch": 0.04,
            "stable": True,
        }
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            run_seed, "wait_for_ros_topic"
        ), patch.object(run_seed, "_probe_robot_posture") as posture_probe:
            run_seed.check_startup_health(
                Path(temp_dir), Path(temp_dir) / "health.log", posture=posture
            )
        posture_probe.assert_not_called()


if __name__ == "__main__":
    unittest.main()

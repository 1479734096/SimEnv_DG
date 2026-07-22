import json
import math
import numpy as np
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.building_control_client import BuildingControlClient
from danger_search.building_feature_provider import (
    PublicDoorFeature,
    PublicElevatorFeature,
)
from danger_search.local_semantic_detector import DoorCandidate, OpeningObservation
from danger_search.rgbd_door_detector import DoorOpeningChange, DoorPanelObservation
from danger_search.ground_support_gate import (
    CLEAR as GROUND_SUPPORT_CLEAR,
    DROP_STOPPING,
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SUPPORT_PATH_UNAVAILABLE,
    GroundSupportAssessment,
    GroundSupportGateConfig,
    GroundSupportSafetyGate,
)
from danger_search.mapper import BLACK, NEGATIVE, OCC, VISITED, MultiFloorGridMap
import danger_search.mission_manager as mission_manager_module
from danger_search.mission_manager import (
    CHANGE_FLOOR,
    EXPLORE_FLOOR,
    FINISH,
    MISSION_FAILED,
    RETURN_HOME,
    WRITE_RESULT,
    _ProgressMonitor,
    _wait_for_localization_ready,
    nearest_forward_obstacle_distance,
    nearest_forward_obstacle_observation,
)
from danger_search.mission_manager import MissionManager
from danger_search.mission_manager import probe_target_for_attempt
from danger_search.navigation_tasks import DONE, RUNNING, OpenDoorAndPass, TakeElevator, FAILED
from danger_search.navigator import IDLE, STUCK
from danger_search.planner import PlanResult, BUDGET_EXHAUSTED, astar_plan
from danger_search.sensor_buffer import SensorBuffer


class MissionManagerHelperTest(unittest.TestCase):
    @staticmethod
    def _finish_return_home_plan(manager, pose):
        planner = manager._return_home_planner
        planner._thread.join(timeout=1.0)
        if planner._thread.is_alive():
            raise AssertionError("返航后台规划未在测试时限内完成")
        manager._tick_return_home(pose)

    def test_default_ground_support_contract_matches_localization_deadbands(self):
        manager = MissionManager.__new__(MissionManager)

        def parameter(name, default=None):
            return "diagnostic" if name == "runtime/profile" else default

        with patch.object(mission_manager_module, "_param", side_effect=parameter):
            manager._load_params()

        params = manager.ground_support_params
        self.assertEqual(params["odometry_linear_deadband"], 0.05)
        self.assertEqual(params["odometry_angular_deadband"], 0.03)
        self.assertGreaterEqual(
            params["stop_linear_threshold"],
            params["odometry_linear_deadband"],
        )
        self.assertGreaterEqual(
            params["stop_angular_threshold"],
            params["odometry_angular_deadband"],
        )

        installed = []
        manager.ground_support_params["production_gate_enabled"] = True
        manager.controller = SimpleNamespace(
            goal_tolerance=0.35,
            set_command_gate=lambda gate: installed.append(gate),
        )
        manager._configure_ground_support_gate()

        self.assertEqual(len(installed), 1)
        self.assertIs(installed[0], manager._ground_support_gate)
        self.assertEqual(
            manager._ground_support_gate.config.odometry_linear_deadband,
            0.05,
        )

    def test_external_sensor_topic_cannot_use_output_channel(self):
        manager = MissionManager.__new__(MissionManager)

        def parameter(name, default=None):
            if name == "runtime/profile":
                return "diagnostic"
            if name == "topics/rgb":
                return "/cmd_vel"
            return default

        with patch.object(mission_manager_module, "_param", side_effect=parameter):
            with self.assertRaisesRegex(ValueError, "不是允许的生产输入话题"):
                manager._load_params()

    def test_formal_runner_waits_until_latched_localization_ready(self):
        callbacks = []
        unregistered = []

        class Subscription:
            def unregister(self):
                unregistered.append(True)

        def subscribe(topic, message_type, callback, queue_size):
            callbacks.append((topic, message_type, queue_size))
            callback(SimpleNamespace(data=False))
            callback(SimpleNamespace(data=True))
            return Subscription()

        ready = _wait_for_localization_ready(
            subscriber_factory=subscribe,
            shutdown_check=lambda: False,
            poll_interval=0.01,
        )

        self.assertTrue(ready)
        self.assertEqual(callbacks[0][0], "/danger_search/localization/ready")
        self.assertEqual(callbacks[0][2], 1)
        self.assertEqual(unregistered, [True])

    def test_failed_elevator_task_records_false_summary_state(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = CHANGE_FLOOR
        manager.max_runtime = 100.0
        manager._elevator_success = True
        manager._mission_success_eligible = True
        manager._floor_transition_guard = None
        manager.return_after_floor_change = False
        manager.active_door = None
        manager.current_task = TakeElevator(0, 1)
        manager.current_task.tick = lambda _manager, _pose: FAILED
        manager.controller = SimpleNamespace(stop=lambda **_kwargs: None)
        manager.sensor_buffer = SimpleNamespace(snapshot=lambda: SimpleNamespace())
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=1.0,
                xyz=(0.0, 0.0, 0.3),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._publish_status = lambda: None
        manager._elapsed = lambda: 1.0
        manager._localization_allows_progress = lambda _sample: True
        manager._localization_health = True
        manager.localization_stance_health_required = True
        manager._update_pose_progress = lambda _pose: None
        manager._record_initial_entry_trace_pose = lambda _pose: None
        manager._record_floor_navigation_trace_pose = lambda _pose: None
        manager._sync_ground_support_task_context = lambda _pose: None
        manager._update_ground_support_odometry = lambda _sample: None
        manager._update_mapping_and_detection = lambda *_args: None
        manager._handle_ground_support_control = lambda _pose: False
        failures = []
        transitions = []
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs.get("reason_code"))
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(logwarn=lambda *_args, **_kwargs: None),
        ):
            manager.tick()

        self.assertFalse(manager._elevator_success)
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(failures, ["换层任务失败"])
        self.assertEqual(transitions[0][0], RETURN_HOME)

    def test_starting_floor_change_marks_attempt_as_not_yet_successful(self):
        manager = MissionManager.__new__(MissionManager)
        manager._elevator_success = None
        manager.current_floor = 0
        manager.floor_params = {"elevator_id": "elevator_main"}
        manager.building_client = SimpleNamespace(services_ready=lambda: True)
        manager.controller = SimpleNamespace(clear_path=lambda: None)
        manager._transition_state = lambda *_args, **_kwargs: None

        manager._begin_floor_change(
            1,
            return_after=False,
        )

        self.assertFalse(manager._elevator_success)

    def test_runtime_budget_uses_public_floor_count_table(self):
        table = [[1, 300.0], [2, 620.0], [3, 840.0]]
        self.assertEqual(
            mission_manager_module._effective_max_runtime(
                840.0,
                max_floor=1,
                runtime_by_floor=table,
            ),
            300.0,
        )
        self.assertEqual(
            mission_manager_module._effective_max_runtime(
                840.0,
                max_floor=2,
                runtime_by_floor=table,
            ),
            620.0,
        )
        self.assertEqual(
            mission_manager_module._effective_max_runtime(
                840.0,
                max_floor=3,
                runtime_by_floor=table,
            ),
            840.0,
        )
        self.assertEqual(
            mission_manager_module._effective_max_runtime(
                120.0,
                max_floor=2,
                runtime_by_floor=table,
            ),
            120.0,
        )
        self.assertEqual(
            mission_manager_module._effective_max_runtime(
                900.0,
                max_floor=4,
                runtime_by_floor=table,
            ),
            900.0,
        )

    def test_summary_output_accepts_not_yet_created_run_directory(self):
        """先验证路径再创建目录，不能要求 run-id 目录已存在。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            public_result = root / "results" / "detected_danger.json"
            public_result.parent.mkdir()
            summary = (
                public_result.parent
                / "runs"
                / "new_run"
                / "mission_runtime_summary.json"
            )

            validated = mission_manager_module._validate_summary_output_path(
                summary,
                public_result,
            )

            self.assertEqual(validated, summary)
            self.assertFalse(summary.parent.exists())

    def test_summary_output_rejects_symlinked_parent_outside_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = root / "results" / "runs"
            runs.mkdir(parents=True)
            outside = root / "outside"
            outside.mkdir()
            linked_run = runs / "linked_run"
            try:
                linked_run.symlink_to(outside, target_is_directory=True)
            except OSError:
                self.skipTest("当前环境无法创建目录符号链接")
            summary = linked_run / "mission_runtime_summary.json"

            with self.assertRaises(ValueError):
                mission_manager_module._validate_summary_output_path(
                    summary,
                    root / "results" / "detected_danger.json",
                )

    def test_only_sensor_confirmed_threshold_crossing_disables_unknown_support_hold(self):
        task = TakeElevator(0, 1)
        for phase in ("ENTER_CABIN", "EXIT_CABIN"):
            task.phase = phase
            self.assertFalse(
                mission_manager_module._ground_support_unknown_hold_enabled(task)
            )

        for phase in ("WAIT_TARGET_DOOR_OPEN", "CLOSE_TARGET_FLOOR_DOOR"):
            task.phase = phase
            self.assertTrue(
                mission_manager_module._ground_support_unknown_hold_enabled(task)
            )

        door_task = OpenDoorAndPass("main_entrance")
        door_task.phase = "GO_PASS_POINT"
        self.assertTrue(
            mission_manager_module._ground_support_unknown_hold_enabled(door_task)
        )
        door_task.mark_opening_sensor_confirmed()
        self.assertFalse(
            mission_manager_module._ground_support_unknown_hold_enabled(door_task)
        )
        door_task.phase = "GO_WAIT_POINT"
        self.assertTrue(
            mission_manager_module._ground_support_unknown_hold_enabled(door_task)
        )

    def test_unknown_support_hold_can_be_disabled_without_task_special_cases(self):
        """显式 DROP-only 回归模式应允许全局关闭 UNKNOWN 临时停车。"""
        self.assertFalse(
            mission_manager_module._ground_support_unknown_hold_enabled(
                None,
                configured=False,
            )
        )
        self.assertFalse(
            mission_manager_module._ground_support_unknown_hold_enabled(
                OpenDoorAndPass("main_entrance"),
                configured=False,
            )
        )

    def test_sync_ground_support_context_combines_config_and_task_phase(self):
        values = []
        manager = MissionManager.__new__(MissionManager)
        manager.ground_support_params = {"unknown_support_hold_enabled": False}
        manager._ground_support_gate = SimpleNamespace(
            set_unknown_support_hold_enabled=values.append
        )
        manager.current_task = None

        manager._sync_ground_support_task_context()

        self.assertEqual(values, [False])

    def test_sync_ground_support_context_keeps_confirmed_post_door_unknown_window(self):
        values = []
        manager = MissionManager.__new__(MissionManager)
        manager.ground_support_params = {"unknown_support_hold_enabled": True}
        manager._ground_support_gate = SimpleNamespace(
            set_unknown_support_hold_enabled=values.append
        )
        manager.current_task = None
        manager.last_door_opening_sensor_confirmed = True
        manager.last_door_context_until = 20.0
        manager._post_door_context_satisfied = lambda pose: False

        with patch("danger_search.mission_manager.get_time", return_value=10.0):
            manager._sync_ground_support_task_context((0.0, 0.0, 0.0))

        self.assertEqual(values, [False])

    def test_sync_ground_support_context_restores_unknown_after_post_door_window(self):
        values = []
        manager = MissionManager.__new__(MissionManager)
        manager.ground_support_params = {"unknown_support_hold_enabled": True}
        manager._ground_support_gate = SimpleNamespace(
            set_unknown_support_hold_enabled=values.append
        )
        manager.current_task = None
        manager.last_door_opening_sensor_confirmed = True
        manager.last_door_context_until = 9.0
        manager._post_door_context_satisfied = lambda pose: False

        with patch("danger_search.mission_manager.get_time", return_value=10.0):
            manager._sync_ground_support_task_context((0.0, 0.0, 0.0))

        self.assertEqual(values, [True])

    def test_ground_support_data_age_does_not_double_count_nominal_decision_cycle(self):
        self.assertAlmostEqual(
            mission_manager_module._ground_support_data_age(
                cloud_stamp=10.0,
                odom_stamp=10.002,
                control_time=10.10,
                decision_period=0.10,
            ),
            0.002,
        )
        self.assertAlmostEqual(
            mission_manager_module._ground_support_data_age(
                cloud_stamp=10.0,
                odom_stamp=10.002,
                control_time=10.14,
                decision_period=0.10,
            ),
            0.04,
        )

    def test_pose_provider_uses_nearest_internal_localization_sample(self):
        from danger_search.pose_provider import POSE_EXPIRED, POSE_MATCHED, POSE_PENDING, PoseProvider

        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        for stamp in (1.0, 1.02, 1.04):
            provider._on_odometry(self._odom_message(stamp))

        matched = provider.at(1.021)
        latest = provider.latest()

        self.assertAlmostEqual(
            matched.stamp,
            1.02,
        )
        self.assertAlmostEqual(
            latest.stamp,
            1.04,
        )

        pending = provider.lookup(1.20)
        self.assertEqual(pending.status, POSE_PENDING)
        provider._on_odometry(self._odom_message(1.30))
        expired = provider.lookup(1.20)
        self.assertEqual(expired.status, POSE_EXPIRED)
        self.assertIsNone(expired.sample)
        self.assertEqual(provider.lookup(1.021).status, POSE_MATCHED)

    def test_pose_provider_estimates_nominal_period_from_recent_samples(self):
        from danger_search.pose_provider import PoseProvider

        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        for stamp in (1.0, 1.1, 1.2, 1.3):
            provider._on_odometry(self._odom_message(stamp))

        self.assertAlmostEqual(provider.observed_period(default=0.1), 0.1)

    def test_localization_stale_stops_and_then_fails_without_advancing_task(self):
        manager = MissionManager.__new__(MissionManager)
        manager.localization_stale_minimum = 0.4
        manager.localization_unavailable_timeout = 3.0
        manager._localization_unavailable_since = None
        manager._localization_stale_reported = False
        manager._localization_failure_reported = False
        manager._mission_success_eligible = True
        manager.state = EXPLORE_FLOOR
        manager.pose_provider = SimpleNamespace(observed_period=lambda **_kwargs: 0.1)
        stopped = []
        reset = []
        transitions = []
        failures = []
        diagnostics = []
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
            reset_progress_timer=lambda pose=None: reset.append(pose),
        )
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))
        manager._record_failure = failures.append
        manager._log_structured = diagnostics.append
        sample = mission_manager_module.PoseSample(
            stamp=10.0,
            xyz=(1.0, 2.0, 0.39),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )

        self.assertFalse(manager._localization_allows_progress(sample, now=10.5))
        self.assertEqual(len(stopped), 1)
        self.assertEqual(reset, [sample.xy_yaw])
        self.assertEqual(transitions, [])

        self.assertFalse(manager._localization_allows_progress(sample, now=13.6))
        self.assertEqual(failures, ["定位数据持续不可用"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_unavailable",
        )
        self.assertFalse(manager._mission_success_eligible)

    def test_localization_recovers_before_failure_and_clears_hold(self):
        manager = MissionManager.__new__(MissionManager)
        manager.localization_stale_minimum = 0.4
        manager.localization_unavailable_timeout = 3.0
        manager._localization_unavailable_since = 10.5
        manager._localization_stale_reported = True
        manager._localization_failure_reported = False
        manager.pose_provider = SimpleNamespace(observed_period=lambda **_kwargs: 0.1)
        diagnostics = []
        manager.controller = SimpleNamespace(
            stop=lambda **_kwargs: self.fail("新鲜定位不应停车"),
            reset_progress_timer=lambda _pose=None: None,
        )
        manager._log_structured = diagnostics.append
        sample = mission_manager_module.PoseSample(
            stamp=10.8,
            xyz=(1.0, 2.0, 0.39),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )

        self.assertTrue(manager._localization_allows_progress(sample, now=10.9))
        self.assertIsNone(manager._localization_unavailable_since)
        self.assertFalse(manager._localization_stale_reported)
        self.assertEqual(diagnostics[-1]["event"], "localization_recovered")

    def test_stance_health_false_fails_immediately(self):
        manager = MissionManager.__new__(MissionManager)
        manager.localization_stance_health_required = True
        manager._localization_health = False
        manager._mission_success_eligible = True
        manager.state = EXPLORE_FLOOR
        stopped = []
        transitions = []
        failures = []
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
        )
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))
        manager._record_failure = failures.append

        self.assertTrue(manager._handle_localization_stance_health())
        self.assertEqual(stopped, [{"reason": "localization_stance_unhealthy"}])
        self.assertEqual(failures, ["定位姿态健康异常"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_stance_unhealthy",
        )

    def test_tick_prioritizes_explicit_stance_failure_over_stale_pose(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.max_runtime = 100.0
        manager._localization_health = False
        manager.localization_stance_health_required = True
        manager._mission_success_eligible = True
        manager._publish_status = lambda: None
        manager._elapsed = lambda: 1.0
        manager.sensor_buffer = SimpleNamespace(snapshot=lambda: SimpleNamespace())
        manager.pose_provider = SimpleNamespace(latest=lambda: None)
        manager._localization_allows_progress = lambda _sample: self.fail(
            "显式姿态异常不应被里程计陈旧状态遮蔽"
        )
        stopped = []
        failures = []
        transitions = []
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
        )
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )

        manager.tick()

        self.assertEqual(stopped, [{"reason": "localization_stance_unhealthy"}])
        self.assertEqual(failures, ["定位姿态健康异常"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_stance_unhealthy",
        )

    def test_unknown_stance_health_stops_then_fails_after_timeout(self):
        manager = MissionManager.__new__(MissionManager)
        manager.localization_stance_health_required = True
        manager.localization_unavailable_timeout = 3.0
        manager._localization_health = None
        manager._mission_success_eligible = True
        manager.state = EXPLORE_FLOOR
        clock = [1.0]
        stopped = []
        transitions = []
        failures = []
        manager._elapsed = lambda: clock[0]
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
        )
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )
        manager._record_failure = failures.append

        self.assertTrue(manager._handle_localization_stance_health())
        self.assertEqual(
            stopped,
            [{"reason": "localization_stance_health_pending"}],
        )
        self.assertEqual(transitions, [])

        clock[0] = 3.1
        self.assertTrue(manager._handle_localization_stance_health())
        self.assertEqual(failures, ["定位姿态健康状态持续不可用"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_stance_health_unavailable",
        )

    def test_stance_health_required_false_allows_unknown(self):
        manager = MissionManager.__new__(MissionManager)
        manager.localization_stance_health_required = False
        manager._localization_health = None

        self.assertFalse(manager._handle_localization_stance_health())

    def test_stance_health_true_allows_progress(self):
        manager = MissionManager.__new__(MissionManager)
        manager.localization_stance_health_required = True
        manager._localization_health = True

        self.assertFalse(manager._handle_localization_stance_health())

    def test_stop_distance_table_parser_preserves_replaceable_breakpoints(self):
        self.assertEqual(
            mission_manager_module._parse_stop_distance_table(
                [[0.30, 0.06], [0.60, 0.09337727739487302]]
            ),
            ((0.30, 0.06), (0.60, 0.09337727739487302)),
        )

    def test_optional_exit_cap_uses_zero_as_ros_safe_disabled_value(self):
        self.assertIsNone(
            mission_manager_module._optional_positive_float(
                0.0,
                "controller/exit_first_max_linear",
            )
        )
        self.assertEqual(
            mission_manager_module._optional_positive_float(
                0.25,
                "controller/exit_first_max_linear",
            ),
            0.25,
        )
        with self.assertRaises(ValueError):
            mission_manager_module._optional_positive_float(
                -0.1,
                "controller/exit_first_max_linear",
            )

    def test_disabled_ground_support_path_does_not_query_tf_or_change_control(self):
        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_gate = None
        manager._ground_support_grid = None
        manager.sensor_buffer = SimpleNamespace(
            frame_transform_matrix=lambda *args, **kwargs: self.fail("关闭时不得查询 TF")
        )

        manager._update_ground_support_assessment(
            SimpleNamespace(odom_msg=None),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            mission_manager_module.PoseSample(
                stamp=1.0,
                xyz=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            ),
            [],
            1.0,
        )

    def test_ground_support_timer_consumes_each_new_cloud_once(self):
        updates = []
        reads = []
        cloud = self._stamped_message(1.0)
        odom = self._odom_message(1.01)
        snapshot = SimpleNamespace(odom_msg=odom, cloud_msg=cloud, image_bundle=None)
        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_gate = object()
        manager._ground_support_grid = object()
        manager._last_ground_support_cloud_stamp = None
        manager._floor_transition_guard = None
        manager.current_floor = 0
        manager.current_task = None
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager.mapper = SimpleNamespace(estimate_floor_from_z=lambda _z: 0)
        manager.sensor_buffer = SimpleNamespace(
            snapshot=lambda: snapshot,
            read_cloud_xyz=lambda *args, **kwargs: reads.append(True) or [(0.0, 0.0, 0.0)],
        )
        manager.pose_provider = SimpleNamespace(
            at=lambda _stamp: mission_manager_module.PoseSample(
                stamp=1.01,
                xyz=(0.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._update_ground_support_assessment = (
            lambda _snapshot, _pose, _xyz, _sample, _points, stamp: updates.append(stamp)
        )

        manager._on_ground_support_timer(None)
        manager._on_ground_support_timer(None)

        self.assertEqual(reads, [True])
        self.assertEqual(updates, [1.0])

    def test_negative_obstacle_ends_without_ordinary_stuck_or_recover(self):
        transitions = []
        failures = []
        controller_calls = []
        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_gate = SimpleNamespace(
            state=SAFETY_BLOCKED,
            reason_code="negative_obstacle",
            check_wall_timeout=lambda _now: None,
            snapshot=lambda: {"state": SAFETY_BLOCKED},
        )
        manager._ground_support_last_state = SAFETY_BLOCKED
        manager.ground_support_params = {"negative_obstacle_replan_enabled": False}
        manager.state = EXPLORE_FLOOR
        manager._mission_success_eligible = True
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: controller_calls.append(("reset", pose)),
            stop=lambda **kwargs: controller_calls.append(("stop", kwargs)),
        )
        manager._record_failure = lambda reason: failures.append(reason)
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))

        handled = manager._handle_ground_support_control((1.0, 2.0, 0.0))

        self.assertTrue(handled)
        self.assertEqual(failures, ["negative_obstacle"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(transitions[0][1]["reason_code"], "negative_obstacle")
        self.assertFalse(manager._mission_success_eligible)
        self.assertNotIn("recover", [item[0] for item in controller_calls])

    def test_negative_obstacle_can_mark_local_black_and_replan_without_recover(self):
        failures = []
        transitions = []
        logs = []
        controller_calls = []
        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.10),),
                stop_confirmation_samples=1,
            )
        )
        gate.update_assessment(
            GroundSupportAssessment(
                stamp=10.0,
                received_wall_time=100.0,
                data_age=0.01,
                supported_distance=0.5,
                current_footprint_supported=True,
                observation_adequate=True,
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.7,
                drop_direction_world=(1.0, 0.0),
                received_control_time=100.0,
                drop_point_world=(1.0, 0.0),
            )
        )
        gate.filter_command(
            (0.3, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
            control_time=100.0,
        )
        gate.observe_odometry(
            1.0,
            linear_speed=0.0,
            angular_speed=0.0,
            wall_time=100.1,
        )
        self.assertEqual(gate.state, SAFETY_BLOCKED)

        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_gate = gate
        manager._ground_support_last_state = SAFETY_BLOCKED
        manager._ground_support_negative_replan_count = 0
        manager.ground_support_params = {
            "negative_obstacle_replan_enabled": True,
            "maximum_negative_obstacle_replans": 2,
            "sweep_half_width": 0.4,
        }
        manager.map_params = {"resolution": 0.1}
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.current_floor = 0
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.current_frontier = SimpleNamespace(centroid=(2.0, 0.0))
        manager._last_frontier_target = (2.0, 0.0)
        manager._last_frontier_score = 1.0
        manager._last_frontier_used_fallback = False
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: controller_calls.append(("reset", pose)),
            stop=lambda **kwargs: controller_calls.append(("stop", kwargs)),
            clear_path=lambda **kwargs: controller_calls.append(("clear", kwargs)),
        )
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))
        manager._elapsed = lambda: 12.0
        manager._log_structured = logs.append

        handled = manager._handle_ground_support_control((0.0, 0.0, 0.0))

        drop_cell = manager.mapper.floor(0).world_to_grid(1.0, 0.0)
        self.assertTrue(handled)
        self.assertEqual(manager.mapper.floor(0).data[drop_cell[1], drop_cell[0]], BLACK)
        manager.mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(1.0, 0.0, 0.6),
            points=[],
        )
        # 第 3 层 MAP-001 修复后，普通建图更新不得擦除安全 BLACK。
        self.assertEqual(manager.mapper.floor(0).data[drop_cell[1], drop_cell[0]], BLACK)
        manager._reapply_ground_support_black_marks(0)
        self.assertEqual(manager.mapper.floor(0).data[drop_cell[1], drop_cell[0]], BLACK)
        self.assertEqual(gate.state, GROUND_SUPPORT_CLEAR)
        self.assertIsNone(manager.current_frontier)
        self.assertIsNone(manager._last_frontier_target)
        self.assertEqual(failures, [])
        self.assertEqual(transitions, [])
        self.assertFalse(any(call[0] == "recover" for call in controller_calls))
        self.assertEqual(logs[-1]["event"], "negative_obstacle_local_replan")

    def test_wall_watch_ends_runner_when_sim_clock_cannot_advance(self):
        failures = []
        stop_reasons = []
        shutdown_reasons = []

        class TimeoutGate:
            state = DROP_STOPPING
            reason_code = "negative_obstacle"

            def check_wall_timeout(self, _wall_time):
                self.state = SAFETY_STOP_TIMEOUT
                self.reason_code = "ground_support_safety_stop_timeout"

        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_gate = TimeoutGate()
        manager._ground_support_wall_stop = SimpleNamespace(wait=lambda _timeout: False)
        manager._ground_support_forced_shutdown_reason = None
        manager._shutdown_requested = False
        manager._mission_success_eligible = True
        manager._record_failure = lambda reason: failures.append(reason)
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stop_reasons.append(kwargs.get("reason"))
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(signal_shutdown=lambda reason: shutdown_reasons.append(reason)),
        ):
            manager._ground_support_wall_watch()

        self.assertEqual(failures, ["ground_support_safety_stop_timeout"])
        self.assertEqual(stop_reasons, ["ground_support_safety_stop_timeout"])
        self.assertEqual(shutdown_reasons, ["ground_support_safety_stop_timeout"])
        self.assertEqual(
            manager._ground_support_forced_shutdown_reason,
            "ground_support_safety_stop_timeout",
        )
        self.assertTrue(manager._shutdown_requested)
        self.assertFalse(manager._mission_success_eligible)

    def test_support_thread_updates_are_applied_on_main_thread(self):
        logs = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 1
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager._ground_support_map_update_lock = threading.Lock()
        manager._ground_support_pending_negative = {}
        manager._ground_support_pending_supported = {}
        manager._elapsed = lambda: 1.0
        manager._log_structured = logs.append
        cell = manager.mapper.floor(1).world_to_grid(0.5, 0.0)

        manager._queue_ground_support_map_updates(
            {"floor": 1, "confirmed_negative_cells": [list(cell)]}
        )
        self.assertNotEqual(
            manager.mapper.floor(1).data[cell[1], cell[0]], NEGATIVE
        )
        manager._apply_pending_ground_support_map_updates()
        self.assertEqual(manager.mapper.floor(1).data[cell[1], cell[0]], NEGATIVE)

        manager._queue_ground_support_map_updates(
            {"floor": 1, "confirmed_supported_cells": [list(cell)]}
        )
        manager._apply_pending_ground_support_map_updates()
        self.assertNotEqual(manager.mapper.floor(1).data[cell[1], cell[0]], NEGATIVE)
        self.assertEqual(logs[-1]["negative_cleared_by_support"], 1)

    def test_stable_unknown_boundary_is_persisted_before_path_replan(self):
        logs = []
        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.10),),
            )
        )
        gate.state = SUPPORT_PATH_UNAVAILABLE
        gate.reason_code = "ground_support_path_unavailable"
        gate._latched_boundary_cells_world = (
            (0.50, -0.10),
            (0.50, 0.00),
            (0.50, 0.10),
        )
        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_gate = gate
        manager._ground_support_last_state = SUPPORT_PATH_UNAVAILABLE
        manager._ground_support_path_retry_key = None
        manager._ground_support_path_retry_count = 0
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.current_floor = 1
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.active_door = None
        manager.current_frontier = SimpleNamespace(centroid=(1.0, 0.0))
        manager._last_frontier_target = (1.0, 0.0)
        manager._last_frontier_score = 1.0
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: None,
        )
        manager._elapsed = lambda: 2.0
        manager._record_failure = lambda reason: None
        manager._log_structured = logs.append

        manager._handle_ground_support_control((0.0, 0.0, 0.0))

        grid = manager.mapper.floor(1)
        for point in ((0.50, -0.10), (0.50, 0.00), (0.50, 0.10)):
            cell = grid.world_to_grid(*point)
            self.assertEqual(grid.data[cell[1], cell[0]], NEGATIVE)
        self.assertIsNone(manager.current_frontier)
        self.assertTrue(
            any(item["event"] == "ground_support_boundary_persisted" for item in logs)
        )

    def test_unknown_timeout_fails_only_current_path_without_discarding_frontier(self):
        cleared = []
        failures = []
        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.09337727739487302),),
            )
        )
        gate.state = SUPPORT_PATH_UNAVAILABLE
        gate.reason_code = "ground_support_path_unavailable"
        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_gate = gate
        manager._ground_support_last_state = SUPPORT_PATH_UNAVAILABLE
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.active_door = None
        manager.current_frontier = SimpleNamespace(centroid=(3.0, 4.0))
        manager._last_frontier_target = (3.0, 4.0)
        manager._last_frontier_score = 1.0
        manager._last_frontier_used_fallback = False
        manager._ground_support_path_retry_key = None
        manager._ground_support_path_retry_count = 0
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: cleared.append(kwargs),
        )
        manager._record_failure = lambda reason: failures.append(reason)
        manager._log_structured = lambda payload: None

        handled = manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertTrue(handled)
        self.assertEqual(cleared, [{"reason": "ground_support_path_unavailable"}])
        self.assertEqual(failures, [])
        self.assertIsNotNone(manager.current_frontier)
        self.assertEqual(manager._last_frontier_target, (3.0, 4.0))
        self.assertEqual(gate.state, GROUND_SUPPORT_CLEAR)

    def test_unknown_timeout_preserves_open_door_parent_and_phase(self):
        cleared = []
        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.09337727739487302),),
            )
        )
        gate.state = SUPPORT_PATH_UNAVAILABLE
        gate.reason_code = "ground_support_path_unavailable"
        task = OpenDoorAndPass("main_entrance")
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, 0.0)
        task._pass_point = (1.0, 0.0)
        task._commit_point = (2.0, 0.0)
        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_gate = gate
        manager._ground_support_last_state = SUPPORT_PATH_UNAVAILABLE
        manager._ground_support_path_retry_key = None
        manager._ground_support_path_retry_count = 0
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager.state = EXPLORE_FLOOR
        manager.current_task = task
        manager.active_door = "main_entrance"
        manager.current_frontier = None
        manager._last_frontier_target = None
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: cleared.append(kwargs),
        )
        manager._log_structured = lambda payload: None

        handled = manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertTrue(handled)
        self.assertIs(manager.current_task, task)
        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(manager.active_door, "main_entrance")
        self.assertEqual(task.pass_point, (1.0, 0.0))
        self.assertEqual(cleared, [{"reason": "ground_support_path_unavailable"}])

    def test_same_open_door_phase_replan_is_bounded_without_recover(self):
        transitions = []
        failures = []
        recover_calls = []
        task = OpenDoorAndPass("main_entrance")
        task.phase = "GO_PASS_POINT"
        manager = MissionManager.__new__(MissionManager)
        manager._ground_support_last_state = SUPPORT_PATH_UNAVAILABLE
        manager._ground_support_path_retry_key = None
        manager._ground_support_path_retry_count = 0
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager.state = EXPLORE_FLOOR
        manager.current_task = task
        manager.active_door = "main_entrance"
        manager.current_frontier = None
        manager._last_frontier_target = None
        manager._mission_success_eligible = True
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: None,
            recover=lambda: recover_calls.append(True),
        )
        manager._record_failure = lambda reason: failures.append(reason)
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))
        manager._log_structured = lambda payload: None

        for _attempt in range(3):
            gate = GroundSupportSafetyGate(
                GroundSupportGateConfig(
                    enabled=True,
                    validated_max_speed=0.60,
                    stop_distance_table=((0.60, 0.09337727739487302),),
                )
            )
            gate.state = SUPPORT_PATH_UNAVAILABLE
            gate.reason_code = "ground_support_path_unavailable"
            manager._ground_support_gate = gate
            manager._ground_support_last_state = SUPPORT_PATH_UNAVAILABLE
            manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertIs(manager.current_task, task)
        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(recover_calls, [])
        self.assertEqual(failures, ["ground_support_path_replan_exhausted"])
        self.assertEqual(transitions[-1][0], WRITE_RESULT)

    def test_floor_transition_invalidates_old_floor_support_assessment(self):
        manager, _, _, reads, map_updates, _ = self._make_floor_transition_manager()
        invalidations = []
        manager._ground_support_gate = SimpleNamespace(
            invalidate_assessment=lambda: invalidations.append(True)
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.6),
        )

        self.assertEqual(invalidations, [True])
        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])

    @staticmethod
    def _stamped_message(stamp: float):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda value=stamp: value),
            )
        )

    @staticmethod
    def _odom_message(stamp: float):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda value=stamp: value),
            ),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.39),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                )
            ),
        )

    def _make_floor_transition_manager(self):
        clock = [0.0]
        diagnostics = []
        reads = []
        map_updates = []
        obstacle_updates = []
        manager = MissionManager.__new__(MissionManager)
        manager.start_time = 0.0
        manager.current_floor = 0
        manager.target_floor = 1
        manager.current_task = TakeElevator(0, 1)
        manager.current_task.phase = "WAIT_FOR_ODOM_STABLE"
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager._last_cloud_stamp = None
        manager._last_image_stamp = None
        manager._last_floor_alignment_mismatch_key = None
        manager._floor_transition_guard = None
        manager._latest_cloud_points = []
        manager._failure_reasons = []
        manager._mission_success_eligible = True
        manager.state = CHANGE_FLOOR
        manager.floor_params = {"service_timeout": 5.0}
        manager.depth_frame_is_optical = False
        manager.mapper = SimpleNamespace(
            floor_height=2.6,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
            estimate_floor_from_z=lambda z: max(0, int(round(float(z) / 2.6))),
            update_from_cloud=lambda **kwargs: map_updates.append(kwargs),
        )
        manager.controller = SimpleNamespace(
            set_nearest_obstacle_distance=lambda distance: obstacle_updates.append(distance),
            stop=lambda **kwargs: diagnostics.append(
                {"event": "controller_stop", "reason": kwargs.get("reason")}
            ),
        )
        manager.sensor_buffer = SimpleNamespace(
            read_cloud_xyz=lambda *args, **kwargs: reads.append(True) or [(0.8, 0.0, 0.5)],
            camera_to_target_matrix=lambda *args, **kwargs: None,
        )
        manager._elapsed = lambda: clock[0]
        manager._log_structured = lambda payload: diagnostics.append(payload)
        manager._transition_state = lambda state, **kwargs: (
            setattr(manager, "state", state),
            diagnostics.append(
                {
                    "event": "state_transition",
                    "state": state,
                    "reason_code": kwargs.get("reason_code"),
                }
            ),
        )
        return manager, clock, diagnostics, reads, map_updates, obstacle_updates

    def _mapping_snapshot(
        self,
        *,
        cloud_stamp: float,
        odom_stamp: float,
        image_bundle=None,
    ):
        return SimpleNamespace(
            odom_msg=self._stamped_message(odom_stamp),
            cloud_msg=self._stamped_message(cloud_stamp),
            image_bundle=image_bundle,
        )

    def test_forward_obstacle_distance_uses_raw_height_filtered_points(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )
        points = [
            (0.8, 0.1, 0.5),
            (0.3, 1.0, 0.5),
            (0.2, 0.0, 2.0),
            (-0.2, 0.0, 0.5),
        ]

        distance = nearest_forward_obstacle_distance(
            points,
            (0.0, 0.0, 0.0),
            floor_index=0,
            mapper=mapper,
        )

        self.assertAlmostEqual(distance, math.hypot(0.8, 0.1))

    def test_forward_obstacle_observation_reports_exact_nearest_echo_geometry(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )

        observation = nearest_forward_obstacle_observation(
            [(0.8, 0.1, 0.5), (0.35, -0.05, 0.22)],
            (0.0, 0.0, 0.0),
            floor_index=0,
            mapper=mapper,
        )

        self.assertIsNotNone(observation)
        self.assertEqual(observation.world_xyz, (0.35, -0.05, 0.22))
        self.assertAlmostEqual(observation.distance, math.hypot(0.35, 0.05))
        self.assertAlmostEqual(observation.forward, 0.35)
        self.assertAlmostEqual(observation.lateral, 0.05)
        self.assertAlmostEqual(observation.relative_height, 0.22)

    def test_forward_obstacle_distance_excludes_only_active_door_aperture(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )
        points = [
            # 已开门洞内的门槛回波，不应阻止已经对准的直线过门。
            (0.20, 0.45, 0.20),
            # 门框外缘仍位于机器人前向走廊内，必须继续作为障碍。
            (0.25, 0.65, 0.50),
        ]

        distance = nearest_forward_obstacle_distance(
            points,
            (0.0, 0.45, 0.0),
            floor_index=0,
            mapper=mapper,
            opened_door_exclusion=(0.30, 0.0, 0.0, 0.55, 0.80),
        )

        self.assertAlmostEqual(distance, math.hypot(0.25, 0.20))

    def test_opened_door_exclusion_only_activates_after_pass_path_commit(self):
        manager = MissionManager.__new__(MissionManager)
        manager.mapper = SimpleNamespace(
            resolution=0.1,
            obstacle_inflation_radius=0.35,
        )
        manager.door_clear_radius = 0.9
        manager.door_pass_offset = 0.8
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    yaw=math.pi / 2.0,
                    width=1.3,
                )
            }
        )
        task = OpenDoorAndPass("main_entrance")
        task.phase = "GO_PASS_POINT"
        manager.current_task = task

        self.assertIsNone(manager._active_opened_door_obstacle_exclusion())

        task._pass_path_started = True
        exclusion = manager._active_opened_door_obstacle_exclusion()
        self.assertIsNotNone(exclusion)
        self.assertAlmostEqual(exclusion[3], 0.55)
        self.assertAlmostEqual(exclusion[4], 1.25)

        manager.current_task = None
        self.assertIsNone(manager._active_opened_door_obstacle_exclusion())

    def test_cached_cloud_stamp_is_mapped_only_once(self):
        read_count = []
        map_count = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager._last_cloud_stamp = None
        manager._latest_cloud_points = []
        manager.sensor_buffer = SimpleNamespace(
            read_cloud_xyz=lambda *args, **kwargs: read_count.append(True) or []
        )
        manager.mapper = SimpleNamespace(
            floor_height=2.6,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
            estimate_floor_from_z=lambda z: max(0, int(round(float(z) / 2.6))),
            update_from_cloud=lambda **kwargs: map_count.append(kwargs),
        )
        manager.controller = SimpleNamespace(
            set_nearest_obstacle_distance=lambda distance: None,
        )
        stamp = SimpleNamespace(to_sec=lambda: 12.5)
        snapshot = SimpleNamespace(
            cloud_msg=SimpleNamespace(header=SimpleNamespace(stamp=stamp)),
            image_bundle=None,
        )

        manager._update_mapping_and_detection(snapshot, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        manager._update_mapping_and_detection(snapshot, (0.1, 0.0, 0.0), (0.1, 0.0, 0.6))

        self.assertEqual(len(read_count), 1)
        self.assertEqual(len(map_count), 1)

    def test_cloud_mapping_uses_cloud_stamp_matched_pose_not_latest_pose(self):
        from danger_search.pose_provider import PoseProvider

        manager, _clock, _diagnostics, _reads, map_updates, _obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None
        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        provider._samples.append(
            mission_manager_module.PoseSample(
                stamp=1.0,
                xyz=(1.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        provider._samples.append(
            mission_manager_module.PoseSample(
                stamp=2.0,
                xyz=(9.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager.pose_provider = provider

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=2.0),
            (9.0, 0.0, 0.0),
            (9.0, 0.0, 0.39),
        )

        self.assertEqual(map_updates[0]["robot_xyz"], (1.0, 0.0, 0.39))

    def test_rgbd_pending_pose_is_retried_before_frame_is_consumed(self):
        from danger_search.pose_provider import PoseProvider

        manager, _clock, _diagnostics, _reads, _map_updates, _obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None
        manager.camera_mount_xyz = (0.28, 0.0, 0.043)
        manager.camera_mount_rpy = (0.0, 0.0, 0.0)
        detections = []
        manager.detector = SimpleNamespace(
            detect_messages=lambda *args, **kwargs: detections.append(args[3]) or [],
        )
        manager.tracker = SimpleNamespace(update=lambda *args, **kwargs: None)
        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        provider._on_odometry(self._odom_message(1.0))
        manager.pose_provider = provider
        rgb_msg = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 2.0),
                frame_id="camera",
            )
        )
        bundle = SimpleNamespace(
            rgb_msg=rgb_msg,
            depth_msg=SimpleNamespace(),
            camera_info_msg=SimpleNamespace(),
        )
        snapshot = self._mapping_snapshot(
            cloud_stamp=1.0,
            odom_stamp=1.0,
            image_bundle=bundle,
        )

        manager._update_mapping_and_detection(snapshot, (0.0, 0.0, 0.0), (0.0, 0.0, 0.39))
        self.assertIsNone(manager._last_image_stamp)
        self.assertEqual(detections, [])

        provider._samples.append(
            mission_manager_module.PoseSample(
                stamp=2.0,
                xyz=(1.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._update_mapping_and_detection(snapshot, (0.0, 0.0, 0.0), (0.0, 0.0, 0.39))

        self.assertEqual(manager._last_image_stamp, 2.0)
        self.assertEqual(len(detections), 1)

    def test_floor_transition_latch_pauses_floor_scoped_cloud_processing(self):
        manager, _clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        snapshot = self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0)

        manager._update_mapping_and_detection(
            snapshot,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])
        self.assertEqual(manager._last_cloud_stamp, 1.0)
        self.assertEqual(manager._floor_transition_guard.skipped_cloud_count, 1)
        self.assertEqual(
            [payload["event"] for payload in diagnostics],
            ["floor_transition_ambiguous_enter", "skipped_floor_scoped_update"],
        )

    def test_explicit_floor_transition_uses_service_and_sensor_confirmation(self):
        from danger_search.pose_provider import PoseProvider

        manager, clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.localization_vertical_reference = "floor_local"
        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)

        def append_pose(stamp):
            provider._samples.append(
                mission_manager_module.PoseSample(
                    stamp=stamp,
                    xyz=(2.4, 2.6, 0.39),
                    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                    linear_xyz=(0.0, 0.0, 0.0),
                    angular_xyz=(0.0, 0.0, 0.0),
                )
            )

        append_pose(1.0)
        manager.pose_provider = provider
        manager.begin_floor_transition(
            source_floor=0,
            target_floor=1,
            phase="CALL_ELEVATOR_TO_TARGET_FLOOR",
        )
        manager.confirm_floor_transition(1)

        append_pose(2.0)
        clock[0] = 1.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=2.0, odom_stamp=2.0),
            (2.4, 2.6, 0.0),
            (2.4, 2.6, 0.39),
        )
        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(reads, [])

        append_pose(3.0)
        clock[0] = 1.1
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=3.0, odom_stamp=3.0),
            (2.4, 2.6, 0.0),
            (2.4, 2.6, 0.39),
        )

        self.assertIsNone(manager._floor_transition_guard)
        self.assertEqual(manager.current_floor, 1)
        self.assertEqual(reads, [True])
        self.assertEqual(len(map_updates), 1)
        self.assertEqual(map_updates[0]["floor_index"], 1)
        self.assertEqual(map_updates[0]["robot_xyz"], (2.4, 2.6, 2.99))
        self.assertEqual(map_updates[0]["points"], [(0.8, 0.0, 3.1)])
        self.assertEqual(len(obstacle_updates), 1)
        self.assertIn(
            "floor_transition_sensor_confirmed",
            [payload["event"] for payload in diagnostics],
        )
        self.assertIn(
            "floor_transition_ambiguous_exit",
            [payload["event"] for payload in diagnostics],
        )

    def test_target_elevator_opening_scan_always_faces_actual_door(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 1
        manager.initial_entry_geometry_tolerance = 0.25
        manager._last_cloud_stamp = 2.0
        manager._floor_transition_guard = None
        manager._latest_cloud_points = [(1.0, 0.0, 0.5)]
        manager._elevator_door_open_confirmation = {
            "door_id": "elevator_floor_1",
            "baseline": None,
            "last_image_stamp": None,
            "visual_confirmations": 0,
            "last_visual_change": None,
            "last_cloud_stamp": None,
            "livox_confirmations": 0,
            "logged": False,
        }
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "elevator_floor_1": SimpleNamespace(
                    x=1.0,
                    y=0.0,
                    yaw=math.pi,
                    width=1.4,
                )
            }
        )
        observed_poses = []
        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: observed_poses.append(
                kwargs["robot_pose"]
            )
            or None
        )

        self.assertFalse(
            manager.confirm_elevator_door_open(
                "elevator_floor_1",
                (2.4, 0.0, 0.0),
            )
        )
        self.assertEqual(len(observed_poses), 1)
        self.assertAlmostEqual(abs(observed_poses[0][2]), math.pi)

    def test_target_elevator_opening_does_not_reuse_livox_during_transition(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.initial_entry_geometry_tolerance = 0.25
        manager._last_cloud_stamp = 2.0
        manager._latest_cloud_points = [(1.0, 0.0, 0.5)]
        manager._floor_transition_guard = SimpleNamespace(explicit_transition=True)
        manager._elevator_door_open_confirmation = {
            "door_id": "elevator_floor_1",
            "baseline": None,
            "last_image_stamp": None,
            "visual_confirmations": 0,
            "last_visual_change": None,
            "last_cloud_stamp": None,
            "livox_confirmations": 0,
            "logged": False,
        }
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "elevator_floor_1": SimpleNamespace(
                    x=1.0,
                    y=0.0,
                    yaw=math.pi,
                    width=1.4,
                )
            }
        )
        observations = []
        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: observations.append(kwargs) or None
        )

        self.assertFalse(
            manager.confirm_elevator_door_open(
                "elevator_floor_1",
                (2.4, 0.0, 0.0),
            )
        )
        self.assertEqual(observations, [])

    def test_target_elevator_opening_uses_registered_rgbd_plane_without_baseline(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.initial_entry_geometry_tolerance = 0.25
        manager._last_cloud_stamp = 2.0
        manager._latest_cloud_points = [(1.0, 0.0, 0.5)]
        manager._floor_transition_guard = SimpleNamespace(explicit_transition=True)
        manager._elevator_door_open_confirmation = {
            "door_id": "elevator_floor_1",
            "baseline": None,
            "last_image_stamp": None,
            "visual_confirmations": 0,
            "last_visual_change": None,
            "last_cloud_stamp": None,
            "livox_confirmations": 0,
            "logged": False,
        }
        manager.building_client = SimpleNamespace(
            doors={
                "elevator_floor_1": SimpleNamespace(
                    x=1.0,
                    y=0.0,
                    yaw=math.pi,
                    width=1.4,
                )
            }
        )
        calls = []
        manager.rgbd_door_detector = SimpleNamespace(
            assess_registered_opening_messages=lambda *args, **kwargs: (
                calls.append(kwargs)
                or DoorOpeningChange(True, 0.0, 1.0, 1.0)
            )
        )
        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: self.fail(
                "楼层过渡期间不应读取旧 Livox 点云"
            )
        )
        manager._log_structured = lambda payload: None
        bundle = SimpleNamespace(
            rgb_msg=object(),
            depth_msg=object(),
            camera_info_msg=object(),
        )
        camera_to_world = np.eye(4)

        manager._update_elevator_door_visual_change(
            bundle, 3.0, camera_to_world
        )
        manager._update_elevator_door_visual_change(
            bundle, 4.0, camera_to_world
        )

        self.assertEqual(len(calls), 2)
        self.assertTrue(
            manager.confirm_elevator_door_open(
                "elevator_floor_1",
                (2.4, 0.0, 0.0),
            )
        )

    def test_floor_transition_latch_timeout_stops_and_writes_failure(self):
        manager, clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        manager.current_task = None

        clock[0] = 1.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=2.0, odom_stamp=2.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        clock[0] = 6.1
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=3.0, odom_stamp=3.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        clock[0] = 12.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=4.0, odom_stamp=4.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])
        self.assertEqual(manager.current_floor, 0)
        self.assertEqual(manager._failure_reasons, ["楼层过渡对齐超时"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(manager.state, WRITE_RESULT)
        self.assertIn(
            {
                "event": "controller_stop",
                "reason": "floor_transition_alignment_timeout",
            },
            diagnostics,
        )
        self.assertEqual(
            sum(payload["event"] == "floor_transition_alignment_timeout" for payload in diagnostics),
            1,
        )

    def test_floor_transition_latch_handles_silent_transport_failure_fail_closed(self):
        manager, _clock, _diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_floor = 1
        manager.current_task.current_floor = 0
        manager.current_task.target_floor = 1

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )
        manager.current_task = None
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=2.0, odom_stamp=2.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )

        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])
        self.assertEqual(manager.current_floor, 1)

    def test_floor_transition_latch_requires_two_fresh_aligned_odom_samples(self):
        manager, clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        stale_cloud = self._mapping_snapshot(cloud_stamp=10.0, odom_stamp=10.0)
        manager._update_mapping_and_detection(
            stale_cloud,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        manager.current_floor = 1

        clock[0] = 1.0
        manager._update_mapping_and_detection(
            stale_cloud,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        self.assertIsNotNone(manager._floor_transition_guard)

        clock[0] = 1.1
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=11.0, odom_stamp=11.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        self.assertIsNotNone(manager._floor_transition_guard)

        clock[0] = 1.2
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=12.0, odom_stamp=12.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertIsNone(manager._floor_transition_guard)
        self.assertEqual(len(reads), 1)
        self.assertEqual(len(map_updates), 1)
        self.assertEqual(map_updates[0]["floor_index"], 1)
        self.assertEqual(len(obstacle_updates), 1)
        self.assertEqual(manager._last_cloud_stamp, 12.0)
        self.assertEqual(
            sum(payload["event"] == "floor_transition_ambiguous_exit" for payload in diagnostics),
            1,
        )

    def test_floor_transition_latch_resets_alignment_after_new_mismatch(self):
        manager, clock, _diagnostics, _reads, map_updates, _obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        manager.current_floor = 1
        clock[0] = 1.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=2.0, odom_stamp=2.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        clock[0] = 2.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=3.0, odom_stamp=3.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )
        manager.current_floor = 0
        clock[0] = 3.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=4.0, odom_stamp=4.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )

        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(map_updates, [])

        clock[0] = 4.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=5.0, odom_stamp=5.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )
        self.assertIsNone(manager._floor_transition_guard)
        self.assertEqual(len(map_updates), 1)
        self.assertEqual(map_updates[0]["floor_index"], 0)

    def test_non_elevator_height_jitter_does_not_start_floor_transition_latch(self):
        manager, _clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertIsNone(manager._floor_transition_guard)
        self.assertEqual(len(reads), 1)
        self.assertEqual(len(map_updates), 1)
        self.assertEqual(map_updates[0]["floor_index"], 0)
        self.assertEqual(len(obstacle_updates), 1)
        self.assertEqual(diagnostics[0]["event"], "floor_alignment_mismatch")

    def test_floor_transition_latch_keeps_rgb_detection_running(self):
        manager, _clock, _diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        detections = []
        manager.detector = SimpleNamespace(
            detect_messages=lambda *args, **kwargs: detections.append(True) or [],
        )
        manager.tracker = SimpleNamespace(update=lambda *args, **kwargs: None)
        manager.camera_mount_xyz = (0.28, 0.0, 0.043)
        manager.camera_mount_rpy = (0.0, 0.0, 0.0)
        # 楼层过渡门控只暂停点云建图，不应阻止 RGB-D 检测；仍需要时间匹配位姿。
        manager.pose_provider = SimpleNamespace(
            at=lambda stamp: mission_manager_module.PoseSample(
                stamp=float(stamp or 0.0),
                xyz=(0.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
            if stamp == 1.0
            else None,
            sensor_to_world_matrix=lambda sample, **kwargs: np.eye(4),
        )
        rgb_msg = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 1.0),
                frame_id="camera",
            )
        )
        bundle = SimpleNamespace(
            rgb_msg=rgb_msg,
            depth_msg=SimpleNamespace(),
            camera_info_msg=SimpleNamespace(),
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(
                cloud_stamp=1.0,
                odom_stamp=1.0,
                image_bundle=bundle,
            ),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])
        self.assertEqual(detections, [True])

    def test_rgb_detection_uses_time_matched_pose_not_cloud_pose(self):
        """RGB-D 检测应使用与彩色图像时间戳匹配的内部定位位姿。"""
        from danger_search.pose_provider import PoseProvider

        manager, _clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None
        manager.camera_mount_xyz = (0.28, 0.0, 0.043)
        manager.camera_mount_rpy = (0.0, 0.0, 0.0)
        camera_to_world_matrices = []
        manager.detector = SimpleNamespace(
            detect_messages=lambda *args, **kwargs: camera_to_world_matrices.append(
                args[3]
            )
            or [],
        )
        manager.tracker = SimpleNamespace(update=lambda *args, **kwargs: None)

        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        # 云帧位姿在 x=0.5；图像时间戳位姿在 x=1.0
        provider._on_odometry(self._odom_message(1.0))
        matched_sample = mission_manager_module.PoseSample(
            stamp=2.0,
            xyz=(1.0, 0.0, 0.39),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )
        provider._samples.append(matched_sample)
        manager.pose_provider = provider

        rgb_msg = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 2.0),
                frame_id="camera",
            )
        )
        bundle = SimpleNamespace(
            rgb_msg=rgb_msg,
            depth_msg=SimpleNamespace(),
            camera_info_msg=SimpleNamespace(),
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(
                cloud_stamp=1.0,
                odom_stamp=1.0,
                image_bundle=bundle,
            ),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )

        self.assertEqual(len(camera_to_world_matrices), 1)
        self.assertAlmostEqual(
            camera_to_world_matrices[0][0, 3],
            1.0 + 0.28,  # 时间匹配位姿 x + camera_mount_xyz x
            places=3,
        )

    def test_rgb_detection_skips_frame_when_time_matched_pose_missing(self):
        """没有匹配时间戳的内部定位位姿时，应跳过该帧检测，不得回退。"""
        manager, _clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None
        detections = []
        manager.detector = SimpleNamespace(
            detect_messages=lambda *args, **kwargs: detections.append(True) or [],
        )
        manager.tracker = SimpleNamespace(update=lambda *args, **kwargs: None)
        manager.pose_provider = SimpleNamespace(
            at=lambda _stamp: None,
            sensor_to_world_matrix=lambda *args, **kwargs: self.fail(
                "不存在匹配位姿时不应构造相机外参矩阵"
            ),
        )

        rgb_msg = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 2.0),
                frame_id="camera",
            )
        )
        bundle = SimpleNamespace(
            rgb_msg=rgb_msg,
            depth_msg=SimpleNamespace(),
            camera_info_msg=SimpleNamespace(),
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(
                cloud_stamp=1.0,
                odom_stamp=1.0,
                image_bundle=bundle,
            ),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )

        self.assertEqual(detections, [])
        self.assertTrue(
            any(
                payload.get("event") == "rgbd_pose_lookup_skipped"
                for payload in diagnostics
            ),
            diagnostics,
        )

    def test_transition_state_records_structured_diagnostics_for_new_object(self):
        """轻量构造的 manager 也应记录完整状态转换上下文。"""
        logged_payloads = []
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_floor = 0
        manager.current_task = SimpleNamespace(name="ExploreTask", phase="tracking")
        manager.current_frontier = SimpleNamespace(centroid=(1.25, 2.5))
        manager._no_reachable_frontier_confirmations = 2
        manager.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.explorer = SimpleNamespace(
            last_selection_debug=SimpleNamespace(reachable_count=3),
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda _format, payload: logged_payloads.append(json.loads(payload))),
        ):
            manager._transition_state(
                RETURN_HOME,
                reason_code="floor_completion_ready",
                call_site="test.transition",
            )

        self.assertEqual(manager.state, RETURN_HOME)
        self.assertTrue(manager._return_home_started)
        self.assertEqual(len(manager._transition_history), 1)
        transition = manager._transition_history[0]
        self.assertEqual(transition["old_state"], EXPLORE_FLOOR)
        self.assertEqual(transition["new_state"], RETURN_HOME)
        self.assertEqual(transition["reason_code"], "floor_completion_ready")
        self.assertEqual(transition["call_site"], "test.transition")
        self.assertEqual(transition["current_task"], "ExploreTask")
        self.assertEqual(transition["last_reachable_frontier_count"], 3)
        self.assertTrue(transition["has_reachable_frontier"])
        self.assertEqual(transition["no_reachable_frontier_confirmations"], 2)
        self.assertEqual(transition["current_floor"], 0)
        self.assertTrue(transition["return_home_started"])
        self.assertFalse(transition["return_home_completed"])
        self.assertEqual(logged_payloads, [transition])

    def test_entering_return_home_clears_exploration_path_and_frontier(self):
        cleared = []
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_floor = 0
        manager.current_task = None
        manager.current_frontier = SimpleNamespace(centroid=(4.0, 8.0))
        manager._last_frontier_target = (4.0, 8.0)
        manager._last_frontier_score = 1.0
        manager._last_frontier_used_fallback = False
        manager._return_home_started = False
        manager._return_home_completed = False
        manager._no_reachable_frontier_confirmations = 0
        manager._transition_history = []
        manager.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.explorer = SimpleNamespace(last_selection_debug=None)
        manager.controller = SimpleNamespace(
            clear_path=lambda reason=None: cleared.append(reason),
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._transition_state(
                RETURN_HOME,
                reason_code="mission_insufficient_time_for_return",
                call_site="test.enter_return",
            )

        self.assertEqual(cleared, ["enter_return_home"])
        self.assertIsNone(manager.current_frontier)
        self.assertIsNone(manager._last_frontier_target)

    def test_external_shutdown_preserves_actual_state_in_result_diagnostics(self):
        """外部 ROS shutdown 写结果时必须保留实际状态和触发源。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary_path = root / "mission_runtime_summary.json"
            result_path = root / "detected_danger.json"
            manager = MissionManager.__new__(MissionManager)
            manager.state = EXPLORE_FLOOR
            manager.start_time = 0.0
            manager.current_floor = 0
            manager.max_floor = 1
            manager.current_task = None
            manager.current_frontier = None
            manager._no_reachable_frontier_confirmations = 0
            manager._finished = False
            manager._shutdown_requested = False
            manager._return_home_success = False
            manager._elevator_success = None
            manager._timeout_triggered = False
            manager._failure_reasons = []
            manager._last_pose = (1.0, 2.0, 0.0)
            manager._max_home_distance = 3.0
            manager.mission_summary_file = str(summary_path)
            manager.mapper = MultiFloorGridMap(
                resolution=0.5,
                width=4.0,
                length=4.0,
                origin_x=-2.0,
                origin_y=-2.0,
            )
            manager.explorer = SimpleNamespace(
                last_selection_debug=SimpleNamespace(reachable_count=2),
            )
            manager.tracker = SimpleNamespace(
                confirmed_positions=lambda now: [],
                tracks=[],
            )
            manager.result_writer = SimpleNamespace(
                write=lambda elapsed, positions: result_path,
            )
            manager.controller = SimpleNamespace(
                stop=lambda *args, **kwargs: None,
                command_debug_summary=lambda: {},
            )

            with patch.object(mission_manager_module, "get_time", return_value=50.2), patch.object(
                mission_manager_module,
                "rospy",
                SimpleNamespace(loginfo=lambda *args, **kwargs: None),
            ):
                manager.shutdown()

            summary = json.loads(summary_path.read_text(encoding="utf-8"))

        self.assertEqual(manager.state, EXPLORE_FLOOR)
        self.assertTrue(manager._finished)
        self.assertEqual(summary["state"], EXPLORE_FLOOR)
        self.assertEqual(summary["terminal_state"], MISSION_FAILED)
        self.assertEqual(summary["terminal_outcome"], "failed")
        self.assertFalse(summary["completed"])
        self.assertEqual(summary["state_at_result_write"], EXPLORE_FLOOR)
        self.assertEqual(summary["result_write_reason"], "external_ros_shutdown")
        self.assertEqual(summary["result_write_call_site"], "MissionManager.shutdown")
        self.assertFalse(summary["return_home_started"])
        self.assertFalse(summary["return_home_completed"])
        self.assertEqual(summary["transition_history"], [])
        self.assertEqual(summary["shutdown_context"]["state_at_shutdown"], EXPLORE_FLOOR)
        self.assertFalse(summary["shutdown_context"]["shutdown_requested_internally"])

    def test_floor_timeout_is_soft_budget_and_keeps_exploring(self):
        """仍有可达前沿或覆盖不足时，楼层预算只能告警，不能结束任务。"""
        logged_payloads = []
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.start_time = 0.0
        manager.floor_timeout = 180.0
        manager.max_floor = 1
        manager.min_floor_explore_time = 20.0
        manager.last_floor_uses_global_budget = True
        manager.current_floor = 0
        manager.current_task = None
        manager.current_frontier = SimpleNamespace(centroid=(2.0, 1.0))
        manager._no_reachable_frontier_confirmations = 0
        manager._return_home_started = False
        manager._return_home_completed = False
        manager._return_home_success = False
        manager._mission_success_eligible = True
        manager._floor_timeout_logged = False
        manager.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.explorer = SimpleNamespace(
            last_selection_debug=SimpleNamespace(reachable_count=6),
        )

        with patch.object(mission_manager_module, "get_time", return_value=183.0), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(
                loginfo=lambda _format, payload: logged_payloads.append(json.loads(payload))
            ),
        ):
            manager._log_floor_budget_exhausted(183.0)
            manager._log_floor_budget_exhausted(184.0)

        self.assertEqual(manager.state, EXPLORE_FLOOR)
        self.assertFalse(manager._return_home_started)
        self.assertTrue(manager._mission_success_eligible)
        self.assertEqual(len(logged_payloads), 1)
        self.assertEqual(
            logged_payloads[0]["action"],
            "continue_until_global_return_boundary",
        )
        self.assertEqual(
            logged_payloads[0]["reason_code"],
            "last_floor_continues_under_global_budget",
        )
        self.assertEqual(logged_payloads[0]["last_reachable_frontier_count"], 6)

    def test_successful_single_floor_completion_requires_return_home(self):
        """单层正常完成必须先进入 RETURN_HOME，返航成功后才可正常 FINISH。"""
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.start_time = 0.0
        manager.current_floor = 0
        manager.max_floor = 1
        manager.current_task = None
        manager.current_frontier = None
        manager._no_frontier_scan_until = 0.0
        manager._no_reachable_frontier_confirmations = 2
        manager._transition_history = []
        manager._return_home_started = False
        manager._return_home_completed = False
        manager._return_home_success = False
        manager._mission_success_eligible = True
        manager._normal_completion = False
        manager._failure_reasons = []
        manager._finished = False
        manager._shutdown_requested = False
        manager.home_pose = (0.0, 0.0, 0.0)
        manager.home_tolerance = 0.2
        manager.mission_summary_file = ""
        manager.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.explorer = SimpleNamespace(last_selection_debug=None)
        manager.controller = SimpleNamespace(
            clear_path=lambda *args, **kwargs: None,
            stop=lambda *args, **kwargs: None,
        )
        manager.tracker = SimpleNamespace(confirmed_positions=lambda now: [], tracks=[])
        manager.result_writer = SimpleNamespace(write=lambda elapsed, positions: Path("result.json"))

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._finish_current_floor(reason_code="floor_completion_ready")
            self.assertEqual(manager.state, RETURN_HOME)
            manager._tick_return_home((0.0, 0.0, 0.0))
            self.assertEqual(manager.state, WRITE_RESULT)
            manager._write_result(reason_code=manager._pending_result_write_reason)

        self.assertTrue(manager._normal_completion)
        self.assertTrue(manager._exploration_completed)
        self.assertEqual(manager._terminal_state, FINISH)
        self.assertEqual(manager._terminal_outcome, "success")

    def test_budget_and_low_progress_returns_are_not_exploration_complete(self):
        """返航闭环可以成功，但预算或低收益退出不能伪装成探索完成。"""
        for return_reason in (
            "mission_insufficient_time_for_return",
            "floor_timeout_return_home",
            "floor_low_progress_return_home",
            "floor_timeout_change_floor_return_home",
        ):
            with self.subTest(return_reason=return_reason):
                manager = MissionManager.__new__(MissionManager)
                manager.state = RETURN_HOME
                manager.start_time = 0.0
                manager.current_floor = 0
                manager.max_floor = 1
                manager.current_task = None
                manager.current_frontier = None
                manager._no_reachable_frontier_confirmations = 0
                manager._transition_history = []
                manager._return_home_started = True
                manager._return_home_completed = True
                manager._return_home_success = True
                manager._return_home_reason = return_reason
                manager._mission_success_eligible = True
                manager._timeout_triggered = False
                manager._failure_reasons = []
                manager._finished = False
                manager.mission_summary_file = ""
                manager.controller = SimpleNamespace(stop=lambda *args, **kwargs: None)
                manager.tracker = SimpleNamespace(
                    confirmed_positions=lambda now: [],
                    tracks=[],
                )
                manager.result_writer = SimpleNamespace(
                    write=lambda elapsed, positions: Path("result.json")
                )

                with patch.object(
                    mission_manager_module,
                    "get_time",
                    return_value=10.0,
                ), patch.object(
                    mission_manager_module,
                    "rospy",
                    SimpleNamespace(loginfo=lambda *args, **kwargs: None),
                ):
                    manager._write_result(reason_code="return_home_completed")

                self.assertTrue(manager._normal_completion)
                self.assertFalse(manager._exploration_completed)

    def test_floor_change_back_to_home_preserves_original_return_reason(self):
        """返航途中完成换层不能覆盖最初的预算或探索完成原因。"""
        manager = MissionManager.__new__(MissionManager)
        manager.state = RETURN_HOME
        manager.current_floor = 0
        manager.current_task = None
        manager.current_frontier = None
        manager._no_reachable_frontier_confirmations = 0
        manager._transition_history = []
        manager._return_home_started = True
        manager._return_home_completed = False
        manager._return_home_reason = "floor_timeout_return_home"
        manager.explorer = SimpleNamespace(last_selection_debug=None)

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._transition_state(
                RETURN_HOME,
                reason_code="floor_change_completed_return_home",
                call_site="test.floor_change",
            )

        self.assertEqual(manager._return_home_reason, "floor_timeout_return_home")

    def test_probe_targets_sweep_lanes_before_extending_forward(self):
        base_pose = (0.0, 0.0, 0.0)

        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 0), (1.2, 0.0))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 1), (1.2, -1.2))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 2), (1.2, 1.2))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 5), (2.4, 0.0))

    def test_probe_targets_follow_home_yaw(self):
        target = probe_target_for_attempt((2.0, 3.0, math.pi / 2.0), 1.0, 2)

        self.assertAlmostEqual(target[0], 1.0, places=6)
        self.assertAlmostEqual(target[1], 4.0, places=6)

    def test_probe_distance_has_lower_bound(self):
        target = probe_target_for_attempt((0.0, 0.0, 0.0), 0.0, 0)

        self.assertEqual(target, (0.05, 0.0))

    def test_initial_forward_target_requires_confirmed_free_foot_lanes(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=8.0,
            origin_x=-2.0,
            origin_y=-2.0,
        ).floor(0)
        for x in (-0.2, 0.0, 0.2):
            for index in range(1, 46):
                cell = grid.world_to_grid(x, 0.1 * index)
                grid.mark_free(*cell)

        target = mission_manager_module.initial_forward_free_target(
            grid,
            (0.0, 0.0, math.pi / 2.0),
            max_distance=5.0,
            corridor_half_width=0.2,
            minimum_distance=1.2,
        )

        self.assertIsNotNone(target)
        self.assertAlmostEqual(target[0], 0.0, places=6)
        self.assertAlmostEqual(target[1], 4.4, places=6)

    def test_initial_forward_target_does_not_cross_unknown_gap(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        ).floor(0)
        for x in (-0.2, 0.0, 0.2):
            for index in range(1, 8):
                cell = grid.world_to_grid(0.1 * index, x)
                grid.mark_free(*cell)

        target = mission_manager_module.initial_forward_free_target(
            grid,
            (0.0, 0.0, 0.0),
            max_distance=3.0,
            corridor_half_width=0.2,
            minimum_distance=1.2,
        )

        self.assertIsNone(target)

    def test_initial_entry_registers_sensor_opening_after_two_new_clouds(self):
        cleared_reasons = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.current_task = None
        manager.current_frontier = object()
        manager.active_door = None
        manager._last_frontier_target = (1.0, 1.0)
        manager._last_frontier_score = 1.0
        manager._last_frontier_used_fallback = True
        manager._latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager._last_cloud_stamp = 1.0
        manager._initial_entry_last_opening_stamp = None
        manager._initial_entry_opening_confirmations = 0
        manager._initial_entry_opening_observation = None
        manager._initial_entry_origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._initial_entry_heading = math.pi / 2.0
        manager.initial_entry_opening_confirmations = 2
        manager.initial_entry_geometry_tolerance = 0.25
        manager.initial_entry_opening_max_distance = 1.2
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_transition_hold_distance = 2.5
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.controller = SimpleNamespace(
            clear_path=lambda **kwargs: cleared_reasons.append(kwargs.get("reason"))
        )
        manager.building_client = SimpleNamespace(doors={})
        observation = OpeningObservation(
            distance=0.8,
            world_xy=(0.4, 0.0),
            yaw=math.pi / 2.0,
            width=0.80,
            evidence_count=30,
        )
        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: observation
        )
        entrance = SimpleNamespace(
            door_id="main_entrance",
            kind="main_entrance",
            floor_index=0,
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(
                manager._try_start_initial_entry_door_task(
                    (0.0, -0.8, math.pi / 2.0), entrance
                )
            )
            # 同一份 cloud 不得重复累计确认。
            self.assertFalse(
                manager._try_start_initial_entry_door_task(
                    (0.0, -0.8, math.pi / 2.0), entrance
                )
            )
            manager._last_cloud_stamp = 1.1
            self.assertTrue(
                manager._try_start_initial_entry_door_task(
                    (0.0, -0.8, math.pi / 2.0), entrance
                )
            )

        registered = manager.building_client.doors["main_entrance"]
        self.assertAlmostEqual(registered.x, 0.0)
        self.assertAlmostEqual(registered.y, 0.0)
        self.assertAlmostEqual(registered.width, 0.80)
        self.assertIsInstance(manager.current_task, OpenDoorAndPass)
        self.assertEqual(manager.current_task.phase, "CALL_OPEN")
        self.assertEqual(manager.active_door, "main_entrance")
        self.assertEqual(cleared_reasons, ["initial_entry_opening_confirmed"])

    def test_initial_entry_does_not_register_distant_rough_opening(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.current_task = None
        manager._latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager._last_cloud_stamp = 1.0
        manager._initial_entry_last_opening_stamp = None
        manager._initial_entry_opening_confirmations = 0
        manager._initial_entry_opening_observation = None
        manager.initial_entry_opening_confirmations = 1
        manager.initial_entry_geometry_tolerance = 0.25
        manager.initial_entry_opening_max_distance = 1.2
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: OpeningObservation(
                distance=2.2,
                world_xy=(0.3, 0.0),
                yaw=math.pi / 2.0,
                width=1.4,
                evidence_count=20,
            )
        )
        entrance = SimpleNamespace(
            door_id="main_entrance",
            kind="main_entrance",
            floor_index=0,
        )

        self.assertFalse(
            manager._try_start_initial_entry_door_task(
                (0.0, -2.2, math.pi / 2.0), entrance
            )
        )

    def test_open_main_entrance_refreshes_geometry_from_two_near_clouds(self):
        manager = MissionManager.__new__(MissionManager)
        manager._initial_entry_origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._initial_entry_heading = math.pi / 2.0
        manager._initial_entry_completed = False
        manager._door_refresh_last_stamp = None
        manager._door_refresh_confirmations = 0
        manager._door_refresh_observation = None
        manager._door_refresh_started_at = None
        manager._last_cloud_stamp = 1.0
        manager._latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.10
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": mission_manager_module.DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    -0.2,
                    0.0,
                    math.pi / 2.0,
                    1.8,
                )
            }
        )
        observations = iter(
            (
                OpeningObservation(0.8, (0.25, 0.02), math.pi / 2.0 + 0.1, 1.8, 30),
                OpeningObservation(0.8, (0.27, 0.01), math.pi / 2.0 + 0.1, 1.82, 32),
            )
        )
        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: next(observations)
        )
        failures = []
        manager._record_failure = lambda reason: failures.append(reason)

        with patch.object(mission_manager_module, "get_time", side_effect=(10.0, 10.1)), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.0, -0.8, math.pi / 2.0)
                ),
                RUNNING,
            )
            manager._last_cloud_stamp = 1.1
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.0, -0.8, math.pi / 2.0)
                ),
                DONE,
            )

        refreshed = manager.building_client.doors["main_entrance"]
        self.assertAlmostEqual(refreshed.x, 0.27)
        self.assertAlmostEqual(refreshed.y, 0.01)
        self.assertAlmostEqual(refreshed.yaw, math.pi / 2.0 + 0.1)
        self.assertAlmostEqual(refreshed.width, 1.82)
        self.assertEqual(failures, [])

    def test_return_main_entrance_refresh_rejects_wrong_axis_then_uses_current_geometry(self):
        manager = MissionManager.__new__(MissionManager)
        manager._initial_entry_origin_pose = (0.0, -2.2, math.pi / 2.0)
        manager._initial_entry_completed = True
        manager._door_refresh_last_stamp = None
        manager._door_refresh_confirmations = 0
        manager._door_refresh_observation = None
        wrong_axis = OpeningObservation(
            0.7, (-0.20, 0.52), -2.13, 2.20, 40
        )
        observation = OpeningObservation(
            0.7, (0.42, 0.08), math.pi / 2.0 + 0.04, 1.3, 40
        )
        manager._door_refresh_started_at = None
        manager._last_cloud_stamp = 2.0
        manager._latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.1
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": mission_manager_module.DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.3,
                )
            }
        )
        manager.current_task = None
        observations = iter((wrong_axis, observation, observation))
        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: next(observations)
        )
        manager._record_failure = lambda reason: self.fail(reason)

        with patch.object(mission_manager_module, "get_time", return_value=20.0), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logwarn_throttle=lambda *args, **kwargs: None,
            ),
        ):
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.35, -0.6, math.pi / 2.0)
                ),
                RUNNING,
            )
            unchanged = manager.building_client.doors["main_entrance"]
            self.assertAlmostEqual(unchanged.x, 0.0)
            self.assertAlmostEqual(unchanged.y, 0.0)
            self.assertAlmostEqual(unchanged.yaw, math.pi / 2.0)

            manager._last_cloud_stamp = 2.1
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.35, -0.6, math.pi / 2.0)
                ),
                RUNNING,
            )
            manager._last_cloud_stamp = 2.2
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.35, -0.6, math.pi / 2.0)
                ),
                DONE,
            )

        refreshed = manager.building_client.doors["main_entrance"]
        self.assertAlmostEqual(refreshed.x, 0.42)
        self.assertAlmostEqual(refreshed.y, 0.08)
        self.assertAlmostEqual(refreshed.yaw, math.pi / 2.0 + 0.04)

    def test_return_main_entrance_refresh_rejects_adjacent_opening_by_center_shift(self):
        manager = MissionManager.__new__(MissionManager)
        manager._initial_entry_origin_pose = (0.0, -2.2, math.pi / 2.0)
        manager._initial_entry_completed = True
        manager._door_refresh_last_stamp = None
        manager._door_refresh_confirmations = 0
        manager._door_refresh_observation = None
        manager._door_refresh_started_at = None
        manager._last_cloud_stamp = 3.0
        manager._latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.1
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": mission_manager_module.DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.12,
                    -0.09,
                    1.49,
                    1.15,
                )
            }
        )
        observed_queries = []

        def detect_opening_ahead(**kwargs):
            observed_queries.append(kwargs["robot_pose"])
            return OpeningObservation(
                0.7,
                (-0.34, 0.50),
                kwargs["robot_pose"][2],
                2.35,
                40,
            )

        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=detect_opening_ahead
        )
        manager._record_failure = lambda reason: self.fail(reason)

        with patch.object(mission_manager_module, "get_time", return_value=20.0), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logwarn_throttle=lambda *args, **kwargs: None,
            ),
        ):
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (-0.25, 0.33, -2.18)
                ),
                RUNNING,
            )

        unchanged = manager.building_client.doors["main_entrance"]
        self.assertAlmostEqual(unchanged.x, 0.12)
        self.assertAlmostEqual(unchanged.y, -0.09)
        self.assertEqual(manager._door_refresh_confirmations, 0)
        self.assertEqual(len(observed_queries), 1)
        query_yaw = observed_queries[0][2]
        self.assertAlmostEqual(
            0.5
            * abs(
                math.atan2(
                    math.sin(2.0 * (query_yaw - unchanged.yaw)),
                    math.cos(2.0 * (query_yaw - unchanged.yaw)),
                )
            ),
            0.0,
        )

    def test_open_main_entrance_refresh_timeout_fails_closed(self):
        manager = MissionManager.__new__(MissionManager)
        manager._initial_entry_origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._initial_entry_completed = False
        manager._door_refresh_last_stamp = None
        manager._door_refresh_confirmations = 0
        manager._door_refresh_observation = None
        manager._door_refresh_started_at = None
        manager._last_cloud_stamp = 1.0
        manager._latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.10
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": mission_manager_module.DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.8,
                )
            }
        )
        manager.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: OpeningObservation(
                0.8,
                (0.0, 0.0),
                math.pi / 2.0,
                0.9,
                20,
            )
        )
        failures = []
        manager._record_failure = lambda reason: failures.append(reason)

        with patch.object(mission_manager_module, "get_time", side_effect=(10.0, 13.1)):
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.0, -0.8, math.pi / 2.0)
                ),
                RUNNING,
            )
            manager._last_cloud_stamp = 1.1
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.0, -0.8, math.pi / 2.0)
                ),
                FAILED,
            )

        self.assertEqual(failures, ["主入口开放门洞近场重观测超时"])

    def test_livox_only_elevator_registration_requires_explicit_opt_in(self):
        manager = MissionManager.__new__(MissionManager)
        manager.max_floor = 2
        manager.current_floor = 0
        manager.current_task = None
        manager._last_cloud_stamp = 1.0
        manager._elevator_opening_last_stamp = None
        manager._elevator_opening_last_scan_stamp = None
        manager._elevator_opening_observation = None
        manager._elevator_opening_confirmations = 0
        manager._latest_cloud_points = [(2.0, 3.0, 0.5)]
        manager.initial_entry_geometry_tolerance = 0.25
        manager.main_entrance_xy = (0.0, 0.0)
        manager.door_pass_offset = 0.8
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.floor_params = {}
        elevator = PublicElevatorFeature(
            elevator_id="public_lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={elevator.elevator_id: elevator}
        )
        manager.building_client = BuildingControlClient()
        manager.building_client.set_public_features(
            door_ids={"ground_lift_door", "upper_lift_door"},
            elevators={"public_lift": {0, 1}},
            elevator_door_ids_by_floor={
                0: ["ground_lift_door"],
                1: ["upper_lift_door"],
            },
        )
        observation = OpeningObservation(
            distance=0.9,
            world_xy=(2.0, 3.0),
            yaw=0.0,
            width=1.0,
            evidence_count=42,
        )
        manager.local_detector = SimpleNamespace(
            detect_elevator_opening_nearby=lambda **kwargs: observation
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(manager._try_register_local_elevator((1.1, 3.0, 0.0)))
            self.assertEqual(manager.building_client.elevators, {})
            manager.livox_elevator_registration_enabled = True
            self.assertFalse(manager._try_register_local_elevator((1.1, 3.0, 0.0)))
            manager._last_cloud_stamp = 1.6
            self.assertTrue(manager._try_register_local_elevator((1.1, 3.0, 0.0)))

        registered = manager.building_client.elevators["public_lift"]
        self.assertEqual(set(registered.lobby_poses), {0, 1})
        self.assertAlmostEqual(registered.lobby_poses[0][0], 2.0)
        self.assertAlmostEqual(registered.lobby_poses[0][1], 3.0)
        self.assertAlmostEqual(abs(registered.lobby_poses[0][2]), math.pi)
        self.assertEqual(
            set(manager.building_client.doors),
            {"ground_lift_door", "upper_lift_door"},
        )
        self.assertEqual(manager.floor_params["elevator_id"], "public_lift")

    def test_elevator_registration_accepts_stable_rgbd_panel_geometry(self):
        manager = MissionManager.__new__(MissionManager)
        manager.max_floor = 2
        manager.current_floor = 0
        manager.current_task = None
        manager._last_cloud_stamp = None
        manager._elevator_opening_last_stamp = None
        manager._elevator_opening_last_scan_stamp = None
        manager._elevator_opening_observation = None
        manager._elevator_opening_confirmations = 0
        manager._elevator_visual_last_stamp = None
        manager._elevator_visual_candidate = None
        manager._elevator_visual_confirmations = 0
        manager._latest_rgbd_door_stamp = None
        manager._latest_rgbd_door_observations = []
        manager.initial_entry_geometry_tolerance = 0.25
        manager.elevator_visual_confirmations = 3
        manager.elevator_visual_min_width = 0.9
        manager.elevator_visual_max_width = 1.8
        manager.elevator_visual_max_distance = 4.0
        manager.main_entrance_xy = (0.0, 0.0)
        manager.door_pass_offset = 0.8
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.floor_params = {}
        elevator = PublicElevatorFeature(
            elevator_id="public_lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={elevator.elevator_id: elevator}
        )
        manager.building_client = BuildingControlClient()
        manager.building_client.set_public_features(
            door_ids={"ground_lift_door", "upper_lift_door"},
            elevators={"public_lift": {0, 1}},
            elevator_door_ids_by_floor={
                0: ["ground_lift_door"],
                1: ["upper_lift_door"],
            },
        )
        manager.local_detector = SimpleNamespace(
            detect_elevator_opening_nearby=lambda **kwargs: None
        )
        observation = DoorPanelObservation(
            stamp=1.0,
            distance=1.4,
            world_xy=(2.0, 3.0),
            outward_yaw=math.pi,
            width=1.35,
            height=1.8,
            plane_rmse=0.01,
            depth_inlier_count=1200,
            bbox=(100, 80, 180, 260),
            median_depth=1.4,
            center_dark_fraction=0.95,
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            for stamp in (1.0, 1.1):
                manager._latest_rgbd_door_stamp = stamp
                manager._latest_rgbd_door_observations = [
                    mission_manager_module.replace(observation, stamp=stamp)
                ]
                self.assertFalse(
                    manager._try_register_local_elevator((0.7, 3.0, 0.0))
                )
            manager._latest_rgbd_door_stamp = 1.2
            manager._latest_rgbd_door_observations = [
                mission_manager_module.replace(observation, stamp=1.2)
            ]
            self.assertTrue(
                manager._try_register_local_elevator((0.7, 3.0, 0.0))
            )

        registered = manager.building_client.elevators["public_lift"]
        self.assertEqual(set(registered.lobby_poses), {0, 1})
        self.assertAlmostEqual(registered.lobby_poses[0][0], 2.0)
        self.assertAlmostEqual(registered.lobby_poses[0][1], 3.0)
        self.assertAlmostEqual(abs(registered.lobby_poses[0][2]), math.pi)

    @staticmethod
    def _rgbd_elevator_registration_manager():
        """RGB-D 电梯注册的公共夹具：单电梯、双楼层、主入口位于原点。"""
        manager = MissionManager.__new__(MissionManager)
        manager.max_floor = 2
        manager.current_floor = 0
        manager.current_task = None
        manager._last_cloud_stamp = None
        manager._elevator_opening_last_stamp = None
        manager._elevator_opening_last_scan_stamp = None
        manager._elevator_opening_observation = None
        manager._elevator_opening_confirmations = 0
        manager._elevator_visual_last_stamp = None
        manager._elevator_visual_candidate = None
        manager._elevator_visual_confirmations = 0
        manager._latest_rgbd_door_stamp = None
        manager._latest_rgbd_door_observations = []
        manager.initial_entry_geometry_tolerance = 0.25
        manager.elevator_visual_confirmations = 3
        manager.elevator_visual_min_width = 0.65
        manager.elevator_visual_max_width = 1.8
        manager.elevator_visual_max_distance = 4.0
        manager.main_entrance_xy = (0.0, 0.0)
        manager.door_pass_offset = 0.8
        manager.mapper = SimpleNamespace(floor_height=2.6)
        manager.floor_params = {}
        elevator = PublicElevatorFeature(
            elevator_id="public_lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={elevator.elevator_id: elevator}
        )
        manager.building_client = BuildingControlClient()
        manager.building_client.set_public_features(
            door_ids={"ground_lift_door", "upper_lift_door"},
            elevators={"public_lift": {0, 1}},
            elevator_door_ids_by_floor={
                0: ["ground_lift_door"],
                1: ["upper_lift_door"],
            },
        )
        manager.local_detector = SimpleNamespace(
            detect_elevator_opening_nearby=lambda **kwargs: None
        )
        return manager

    def _feed_rgbd_panel(self, manager, stamp, world_xy, outward_yaw, width):
        manager._latest_rgbd_door_stamp = stamp
        manager._latest_rgbd_door_observations = [
            DoorPanelObservation(
                stamp=stamp,
                distance=2.0,
                world_xy=world_xy,
                outward_yaw=outward_yaw,
                width=width,
                height=1.0,
                plane_rmse=0.0,
                depth_inlier_count=1200,
                bbox=(100, 80, 90, 260),
                median_depth=2.0,
                center_dark_fraction=0.95,
            )
        ]
        return manager._try_register_local_elevator((0.7, 1.0, 0.0))

    def test_elevator_registration_accepts_single_sliding_panel_sequence(self):
        # 复现 /tmp/stage11_elevator_fullscan.bag 的三帧移动中单扇门板：
        # 位置与朝向连续、宽度 0.657~0.727 m，必须足以注册公开唯一电梯。
        manager = self._rgbd_elevator_registration_manager()
        frames = (
            (31.328, (1.608, 2.244), 3.121, 0.727),
            (31.428, (1.612, 2.295), 3.122, 0.703),
            (31.528, (1.615, 2.371), 3.123, 0.657),
        )
        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(self._feed_rgbd_panel(manager, *frames[0]))
            self.assertFalse(self._feed_rgbd_panel(manager, *frames[1]))
            self.assertTrue(self._feed_rgbd_panel(manager, *frames[2]))

        registered = manager.building_client.elevators["public_lift"]
        self.assertEqual(set(registered.lobby_poses), {0, 1})
        self.assertAlmostEqual(registered.lobby_poses[0][0], 1.615)
        self.assertAlmostEqual(registered.lobby_poses[0][1], 2.371)
        self.assertAlmostEqual(registered.lobby_poses[0][2], 3.123)
        self.assertEqual(
            set(manager.building_client.doors),
            {"ground_lift_door", "upper_lift_door"},
        )

    def test_elevator_registration_tolerates_short_intermittent_rgbd_gaps(self):
        manager = self._rgbd_elevator_registration_manager()
        manager.elevator_visual_confirmation_max_gap = 0.4
        frames = (
            (31.0, (1.61, 2.24), 3.121, 0.72),
            (31.2, (1.62, 2.29), 3.122, 0.70),
            (31.4, (1.63, 2.35), 3.123, 0.68),
        )
        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(self._feed_rgbd_panel(manager, *frames[0]))
            manager._latest_rgbd_door_stamp = 31.1
            manager._latest_rgbd_door_observations = []
            self.assertFalse(
                manager._try_register_local_elevator((0.7, 1.0, 0.0))
            )
            self.assertFalse(self._feed_rgbd_panel(manager, *frames[1]))
            manager._latest_rgbd_door_stamp = 31.3
            manager._latest_rgbd_door_observations = []
            self.assertFalse(
                manager._try_register_local_elevator((0.7, 1.0, 0.0))
            )
            self.assertTrue(self._feed_rgbd_panel(manager, *frames[2]))

        self.assertEqual(manager._elevator_visual_max_confirmations, 3)
        self.assertEqual(
            manager._elevator_visual_association_diagnostics[
                "intermittent_gap_held"
            ],
            2,
        )

    def test_elevator_registration_rejects_sub_panel_dark_objects(self):
        # 主入口门框立柱实测约 0.35~0.45 m 宽，仍须低于单扇门板下限。
        manager = self._rgbd_elevator_registration_manager()
        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            for index in range(4):
                self.assertFalse(
                    self._feed_rgbd_panel(
                        manager,
                        1.0 + 0.1 * index,
                        (2.0, 3.0),
                        math.pi,
                        0.45,
                    )
                )
        self.assertEqual(manager.building_client.elevators, {})

    def test_elevator_registration_rejects_panel_inside_entrance_exclusion(self):
        # 几何合法但位于主入口排除半径内的门板不得注册为电梯。
        manager = self._rgbd_elevator_registration_manager()
        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            for index in range(4):
                self.assertFalse(
                    self._feed_rgbd_panel(
                        manager,
                        1.0 + 0.1 * index,
                        (0.3, 0.2),
                        math.pi,
                        0.7,
                    )
                )
        self.assertEqual(manager.building_client.elevators, {})

    def test_elevator_registration_resets_on_inconsistent_candidate_geometry(self):
        # 候选位置跳变超出容差时确认计数必须重置，避免拼接不同物体。
        manager = self._rgbd_elevator_registration_manager()
        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(
                self._feed_rgbd_panel(manager, 1.0, (2.0, 3.0), math.pi, 0.7)
            )
            self.assertFalse(
                self._feed_rgbd_panel(manager, 1.1, (2.0, 3.0), math.pi, 0.7)
            )
            self.assertFalse(
                self._feed_rgbd_panel(manager, 1.2, (3.5, 3.0), math.pi, 0.7)
            )
            self.assertEqual(manager._elevator_visual_confirmations, 1)
            self.assertFalse(
                self._feed_rgbd_panel(manager, 1.3, (3.5, 3.0), math.pi, 0.7)
            )
            self.assertTrue(
                self._feed_rgbd_panel(manager, 1.4, (3.5, 3.0), math.pi, 0.7)
            )

        registered = manager.building_client.elevators["public_lift"]
        self.assertAlmostEqual(registered.lobby_poses[0][0], 3.5)

    def test_elevator_geometry_refresh_updates_only_runtime_semantic_geometry(self):
        manager = self._rgbd_elevator_registration_manager()
        manager.elevator_geometry_refresh_timeout = 6.0
        manager.elevator_geometry_refresh_max_shift = 0.75
        manager.elevator_geometry_refresh_max_yaw_change = 0.35
        manager._elevator_refresh_context = None
        manager._elevator_refresh_started_at = None
        manager._elevator_refresh_last_stamp = None
        manager._elevator_refresh_candidate = None
        manager._elevator_refresh_confirmations = 0
        manager._elevator_refresh_rejections = {}
        structured = []
        manager._log_structured = structured.append
        elevator_feature = manager.building_features.elevator_features["public_lift"]
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        ).floor(0)
        grid_snapshot = np.array(grid.data, copy=True)
        manager.mapper = SimpleNamespace(
            floor_height=2.6,
            floor=lambda _floor: grid,
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logerr=lambda *args, **kwargs: None,
            ),
        ):
            manager._register_elevator_geometry(
                elevator_feature,
                current_door_id="ground_lift_door",
                world_xy=(1.60, 2.30),
                outward_yaw=math.pi,
                width=0.70,
                source="test_initial",
                evidence_count=100,
            )
            statuses = []
            for index, center in enumerate(((1.72, 2.36), (1.73, 2.37), (1.74, 2.38))):
                stamp = 10.0 + 0.1 * index
                manager._latest_rgbd_door_stamp = stamp
                manager._latest_rgbd_door_observations = [
                    DoorPanelObservation(
                        stamp=stamp,
                        distance=1.0,
                        world_xy=center,
                        outward_yaw=math.pi - 0.02,
                        width=0.71,
                        height=1.0,
                        plane_rmse=0.0,
                        depth_inlier_count=1200,
                        bbox=(100, 80, 90, 260),
                        median_depth=1.0,
                        center_dark_fraction=0.95,
                    )
                ]
                with patch.object(mission_manager_module, "get_time", return_value=20.0):
                    statuses.append(
                        manager.refresh_elevator_geometry(
                            "public_lift",
                            0,
                            "ground_lift_door",
                            (0.8, 2.3, 0.0),
                        )
                    )

        self.assertEqual(statuses, [RUNNING, RUNNING, DONE])
        elevator = manager.building_client.elevators["public_lift"]
        self.assertEqual(set(elevator.lobby_poses), {0, 1})
        self.assertAlmostEqual(elevator.lobby_poses[0][0], 1.74)
        self.assertAlmostEqual(elevator.lobby_poses[1][1], 2.38)
        self.assertTrue(np.array_equal(grid.data, grid_snapshot))
        self.assertEqual(structured[-1]["event"], "elevator_geometry_refresh")
        self.assertAlmostEqual(structured[-1]["center_shift"], 0.161, places=3)

    def test_elevator_geometry_refresh_rejects_identity_jump_and_times_out(self):
        manager = self._rgbd_elevator_registration_manager()
        manager.elevator_geometry_refresh_timeout = 1.0
        manager.elevator_geometry_refresh_max_shift = 0.75
        manager.elevator_geometry_refresh_max_yaw_change = 0.35
        manager._elevator_refresh_context = None
        manager._elevator_refresh_started_at = None
        manager._elevator_refresh_last_stamp = None
        manager._elevator_refresh_candidate = None
        manager._elevator_refresh_confirmations = 0
        manager._elevator_refresh_rejections = {}
        structured = []
        manager._log_structured = structured.append
        manager.rgbd_door_detector = None
        elevator_feature = manager.building_features.elevator_features["public_lift"]

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logerr=lambda *args, **kwargs: None,
                logwarn=lambda *args, **kwargs: None,
            ),
        ):
            manager._register_elevator_geometry(
                elevator_feature,
                current_door_id="ground_lift_door",
                world_xy=(1.60, 2.30),
                outward_yaw=math.pi,
                width=0.70,
                source="test_initial",
                evidence_count=100,
            )
            manager._latest_rgbd_door_stamp = 10.0
            manager._latest_rgbd_door_observations = [
                DoorPanelObservation(
                    stamp=10.0,
                    distance=1.0,
                    world_xy=(3.0, 2.3),
                    outward_yaw=math.pi,
                    width=0.70,
                    height=1.0,
                    plane_rmse=0.0,
                    depth_inlier_count=1200,
                    bbox=(100, 80, 90, 260),
                    median_depth=1.0,
                    center_dark_fraction=0.95,
                )
            ]
            with patch.object(mission_manager_module, "get_time", return_value=20.0):
                self.assertEqual(
                    manager.refresh_elevator_geometry(
                        "public_lift", 0, "ground_lift_door", (0.8, 2.3, 0.0)
                    ),
                    RUNNING,
                )
            with patch.object(mission_manager_module, "get_time", return_value=21.1):
                self.assertEqual(
                    manager.refresh_elevator_geometry(
                        "public_lift", 0, "ground_lift_door", (0.8, 2.3, 0.0)
                    ),
                    IDLE,
                )

        elevator = manager.building_client.elevators["public_lift"]
        self.assertAlmostEqual(elevator.lobby_poses[0][0], 1.60)
        self.assertEqual(
            structured[-1]["event"], "elevator_geometry_refresh_unavailable"
        )
        self.assertEqual(structured[-1]["rejections"]["center_shift"], 1)

    def test_elevator_trace_candidates_stay_on_hall_side_and_traversable(self):
        manager = self._rgbd_elevator_registration_manager()
        elevator_feature = manager.building_features.elevator_features["public_lift"]
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        ).floor(0)
        for point in ((0.2, 2.3), (0.6, 2.3), (1.2, 2.3), (2.8, 2.3)):
            gx, gy = grid.world_to_grid(*point)
            grid.data[gy, gx] = VISITED
        manager.mapper = SimpleNamespace(floor=lambda _floor: grid)
        manager.elevator_visual_max_distance = 4.0
        manager._floor_navigation_traces = {
            0: [(0.2, 2.3), (0.6, 2.3), (1.2, 2.3), (2.8, 2.3)]
        }

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._register_elevator_geometry(
                elevator_feature,
                current_door_id="ground_lift_door",
                world_xy=(1.6, 2.3),
                outward_yaw=math.pi,
                width=0.70,
                source="test_initial",
                evidence_count=100,
            )

        candidates = manager.elevator_approach_trace_candidates(
            "public_lift", 0, max_candidates=4
        )

        self.assertEqual(candidates, [(0.6, 2.3)])

    def test_visual_preparation_closes_initially_open_elevator_only_once(self):
        manager = MissionManager.__new__(MissionManager)
        manager.rgbd_door_detector = object()
        manager.max_floor = 2
        manager.current_floor = 0
        manager._elevator_visual_prepare_task = None
        manager._elevator_visual_prepare_door_id = None
        manager._elevator_visual_prepare_result_logged = False
        manager.building_features = SimpleNamespace(
            doors={
                "lift_0": PublicDoorFeature(
                    door_id="lift_0",
                    kind="elevator",
                    floor_index=0,
                    initial_open=True,
                )
            }
        )
        close_calls = []
        task = SimpleNamespace(
            done=lambda: False,
            ok=lambda: True,
            message=lambda: "closing",
        )
        manager.building_client = SimpleNamespace(
            door_service_ready=lambda: True,
            close_door=lambda door_id: close_calls.append(door_id) or task,
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._start_elevator_visual_preparation()
            manager._start_elevator_visual_preparation()

        self.assertEqual(close_calls, ["lift_0"])
        self.assertTrue(manager.elevator_visual_preparation_pending("lift_0"))

    def test_visual_landmark_scan_keeps_observing_after_one_rotation_until_timeout(self):
        manager = MissionManager.__new__(MissionManager)
        manager.rgbd_door_detector = SimpleNamespace(
            diagnostics=lambda: {
                "frames": 5,
                "last": {"rejected_panel_width": 1},
                "cumulative": {"rejected_panel_width": 3},
                "last_observation_count": 0,
            }
        )
        manager.current_floor = 0
        manager.current_task = None
        manager._elevator_visual_scan_started = False
        manager._elevator_visual_scan_completed = False
        manager._elevator_visual_scan_until = 0.0
        manager._elevator_visual_scan_last_yaw = None
        manager._elevator_visual_scan_rotation = 0.0
        manager.elevator_visual_scan_duration = 18.0
        manager.elevator_visual_scan_angular_speed = 0.7
        manager.elevator_visual_scan_required_rotation = 5.8
        manager.last_door_id = "main"
        manager.last_door_xy = (0.0, 0.0)
        manager._post_door_context_satisfied = lambda pose: True
        manager._elevator_visual_prepare_task = SimpleNamespace(
            done=lambda: True,
            ok=lambda: True,
        )
        elevator = PublicElevatorFeature(
            elevator_id="lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={"lift": elevator},
            doors={
                "main": PublicDoorFeature("main", "main_entrance", 0, True)
            },
        )
        manager.building_client = SimpleNamespace(elevators={})
        manager.current_frontier = (1.0, 2.0)
        manager._last_frontier_target = (1.0, 2.0)
        manager._last_frontier_score = 3.0
        manager._last_frontier_used_fallback = True
        manager._latest_rgbd_door_observations = []
        manager._elevator_visual_candidate = None
        manager._elevator_visual_confirmations = 0
        diagnostics = []
        manager._log_structured = diagnostics.append
        calls = []
        manager.controller = SimpleNamespace(
            clear_path=lambda reason=None: calls.append(("clear", reason)),
            rotate_in_place=lambda speed: calls.append(("rotate", speed)),
            stop=lambda reason=None: calls.append(("stop", reason)),
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ), patch.object(mission_manager_module, "get_time", side_effect=(10.0, 11.0, 12.0, 13.0, 28.1)):
            self.assertTrue(manager._tick_elevator_visual_scan((0.0, 2.0, 2.0)))
            self.assertTrue(manager._tick_elevator_visual_scan((0.0, 2.0, -2.0)))
            self.assertTrue(manager._tick_elevator_visual_scan((0.0, 2.0, 0.0)))
            self.assertTrue(manager._tick_elevator_visual_scan((0.0, 2.0, 1.6)))
            self.assertFalse(manager._tick_elevator_visual_scan((0.0, 2.0, 1.6)))

        self.assertEqual(sum(name == "clear" for name, _ in calls), 1)
        self.assertEqual(sum(name == "rotate" for name, _ in calls), 4)
        self.assertEqual(sum(name == "stop" for name, _ in calls), 1)
        self.assertTrue(manager._elevator_visual_scan_completed)
        self.assertEqual(diagnostics[-1]["event"], "elevator_visual_scan_complete")
        self.assertEqual(diagnostics[-1]["result"], "landmark_not_found")
        self.assertEqual(
            diagnostics[-1]["detector"]["cumulative"]["rejected_panel_width"],
            3,
        )

    def test_public_door_ids_require_building_service_connection(self):
        self.assertTrue(
            mission_manager_module._building_services_required(
                max_floor=1,
                public_door_ids={"main_entrance"},
            )
        )

    def test_return_home_from_inside_reuses_registered_main_entrance_task(self):
        cleared = []
        manager = MissionManager.__new__(MissionManager)
        manager.home_pose = (0.0, -3.2, math.pi / 2.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.7
        manager.main_entrance_xy = (0.0, 0.0)
        manager.main_entrance_pass_vector = (0.0, 1.0)
        manager._return_home_main_entrance_crossed = False
        manager.failed_doors = set()
        manager.current_task = None
        manager.active_door = None
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": mission_manager_module.DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.0,
                )
            },
            get_door_path_points=lambda *args, **kwargs: (
                (0.0, 0.8),
                (0.0, -0.8),
                (0.0, -1.8),
            ),
        )
        manager.controller = SimpleNamespace(
            goal_tolerance=0.35,
            clear_path=lambda **kwargs: cleared.append(kwargs.get("reason")),
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._tick_return_home((0.0, 1.0, math.pi / 2.0))

        self.assertIsInstance(manager.current_task, OpenDoorAndPass)
        self.assertEqual(manager.current_task.door_id, "main_entrance")
        self.assertEqual(manager.active_door, "main_entrance")
        self.assertEqual(cleared, ["return_home_main_entrance"])

    def test_return_home_lateral_to_main_entrance_first_plans_wait_point(self):
        paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.home_pose = (0.0, -3.2, math.pi / 2.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.7
        manager.main_entrance_xy = (0.0, 0.0)
        manager.main_entrance_pass_vector = (0.0, 1.0)
        manager._return_home_main_entrance_crossed = False
        manager._return_home_entry_trace_attempted = False
        manager.failed_doors = set()
        manager.current_task = None
        manager.active_door = None
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_transition_hold_distance = 2.5
        manager.building_client = SimpleNamespace(
            doors={"main_entrance": SimpleNamespace()},
            get_door_path_points=lambda *args, **kwargs: (
                (0.0, 0.8),
                (0.0, -0.8),
                (0.0, -1.8),
            ),
        )
        manager.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=80.0,
            length=80.0,
            origin_x=-40.0,
            origin_y=-40.0,
        )
        stops = []
        manager.controller = SimpleNamespace(
            goal_tolerance=0.35,
            has_path=lambda: False,
            set_path=lambda path: paths.append(path),
            tick=lambda pose: "RUNNING",
            recover=lambda: None,
            stop=lambda reason=None: stops.append(reason),
        )
        planned = PlanResult(path=[(-1.7, 1.8), (0.0, 0.8)], status="reached")

        with patch.object(
            mission_manager_module,
            "astar_plan",
            return_value=planned,
        ) as astar, patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            pose = (-1.7, 1.8, -math.pi / 2.0)
            manager._tick_return_home(pose)
            self._finish_return_home_plan(manager, pose)

        self.assertIsNone(manager.current_task)
        self.assertEqual(paths, [planned.path])
        self.assertEqual(stops, ["return_home_plan_pending"])
        self.assertEqual(astar.call_args.args[2], (0.0, 0.8))

    def test_return_home_far_inside_plans_home_before_creating_door_task(self):
        paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.home_pose = (0.0, -3.2, math.pi / 2.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.7
        manager.main_entrance_xy = (0.0, 0.0)
        manager.main_entrance_pass_vector = (0.0, 1.0)
        manager._return_home_main_entrance_crossed = False
        manager._return_home_entry_trace_attempted = False
        manager.failed_doors = set()
        manager.current_task = None
        manager.active_door = None
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_transition_hold_distance = 2.5
        manager.building_client = SimpleNamespace(
            doors={"main_entrance": SimpleNamespace()},
            get_door_path_points=lambda *args, **kwargs: (
                (0.0, 0.8),
                (0.0, -0.8),
                (0.0, -1.8),
            ),
        )
        manager.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=80.0,
            length=80.0,
            origin_x=-40.0,
            origin_y=-40.0,
        )
        stops = []
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path: paths.append(path),
            tick=lambda pose: "RUNNING",
            recover=lambda: None,
            stop=lambda reason=None: stops.append(reason),
        )
        planned = PlanResult(path=[(0.0, 30.0), (0.0, -3.2)], status="reached")

        with patch.object(
            mission_manager_module,
            "astar_plan",
            return_value=planned,
        ) as astar:
            with patch.object(
                mission_manager_module,
                "rospy",
                SimpleNamespace(loginfo=lambda *args, **kwargs: None),
            ):
                pose = (0.0, 30.0, -math.pi / 2.0)
                manager._tick_return_home(pose)
                self._finish_return_home_plan(manager, pose)

        self.assertIsNone(manager.current_task)
        self.assertEqual(paths, [planned.path])
        self.assertEqual(stops, ["return_home_plan_pending"])
        self.assertEqual(astar.call_args.args[2], (0.0, 0.8))
        self.assertIsNone(astar.call_args.kwargs["max_iterations"])

    def test_return_home_uses_actual_floor_trace_when_live_grid_is_disconnected(self):
        paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.home_pose = (3.0, 0.0, 0.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.2
        manager._floor_navigation_traces = {
            0: [(3.0 - 0.5 * index, 0.0) for index in range(13)]
        }
        manager.mapper = MultiFloorGridMap(
            resolution=0.25,
            width=8.0,
            length=4.0,
            origin_x=-4.0,
            origin_y=-2.0,
            visited_radius=0.45,
        )
        grid = manager.mapper.floor(0)
        for point in manager._floor_navigation_traces[0]:
            grid.mark_visited_radius(point[0], point[1], 0.30)
        barrier_cell = grid.world_to_grid(0.0, 0.0)
        self.assertIsNotNone(barrier_cell)
        barrier_x = barrier_cell[0]
        grid.data[:, barrier_x] = OCC
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path: paths.append(path),
            tick=lambda pose: "running",
            recover=lambda: None,
            stop=lambda reason=None: None,
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(logwarn=lambda *args, **kwargs: None),
        ):
            pose = (-3.0, 0.0, 0.0)
            manager._tick_return_home(pose)
            self._finish_return_home_plan(manager, pose)

        self.assertEqual(len(paths), 1)
        self.assertLess(math.hypot(paths[0][0][0] + 3.0, paths[0][0][1]), 0.5)
        self.assertLess(math.hypot(paths[0][-1][0] - 3.0, paths[0][-1][1]), 0.5)
        self.assertTrue(np.all(grid.data[:, barrier_x] == OCC))

    def test_return_home_veto_changes_trace_fallback_first_segment(self):
        """起点在膨胀 OCC 内时，失败首段否决应迫使 trace 选择不同出口。"""
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
            visited_radius=0.45,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        grid.mark_occupied_radius(0.0, 0.0, 0.25)
        pose = (0.0, 0.18, -0.8)
        goal = (2.0, 1.0)
        trace = tuple(
            [
                (2.0, 1.0),
                (1.5, 1.0),
                (1.0, 1.0),
                (0.5, 1.0),
                (0.0, 1.0),
                (-0.3, 0.8),
                (-0.45, 0.6),
                (-0.35, 0.4),
                (0.0, 0.18),
            ]
        )
        start_cell = grid.world_to_grid(pose[0], pose[1])
        self.assertEqual(grid.data[start_cell[1], start_cell[0]], OCC)

        status, initial_path = manager._compute_return_home_plan(
            manager._copy_floor_grid(grid),
            pose,
            goal,
            trace_snapshot=trace,
            visited_radius=0.45,
        )
        self.assertEqual(status, "reached")
        initial_end = initial_path[1]

        status, vetoed_path = manager._compute_return_home_plan(
            manager._copy_floor_grid(grid),
            pose,
            goal,
            trace_snapshot=trace,
            visited_radius=0.45,
            veto_segments=((pose[0], pose[1], initial_end[0], initial_end[1]),),
            failed_signature=manager._return_segment_signature(initial_end),
            veto_lookahead=0.8,
        )

        self.assertEqual(status, "reached")
        self.assertNotEqual(
            manager._return_segment_signature(vetoed_path[1]),
            manager._return_segment_signature(initial_end),
        )
        self.assertFalse(
            manager._path_crosses_blocked_beyond(grid, vetoed_path, exemption=0.20)
        )

    def test_return_home_trace_candidate_is_checked_only_after_stuck_feedback(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.mapper = MultiFloorGridMap(
            resolution=0.25,
            width=8.0,
            length=4.0,
            origin_x=-4.0,
            origin_y=-2.0,
            visited_radius=0.45,
        )
        grid = manager.mapper.floor(0)
        trace = tuple((3.0 - 0.5 * index, 0.0) for index in range(13))
        for point in trace:
            grid.mark_visited_radius(point[0], point[1], 0.30)
        barrier_x = grid.world_to_grid(0.0, 0.0)[0]
        grid.data[:, barrier_x] = OCC
        pose = (-3.0, 0.0, 0.0)
        goal = (3.0, 0.0)

        initial_status, initial_path = manager._compute_return_home_plan(
            manager._copy_floor_grid(grid),
            pose,
            goal,
            trace_snapshot=trace,
            visited_radius=0.45,
        )
        self.assertEqual(initial_status, "reached")
        self.assertTrue(initial_path)

        rejected_status, rejected_path = manager._validate_return_home_candidate(
            initial_path,
            original_grid=grid,
            failed_signature=None,
        )
        self.assertEqual(rejected_status, "candidate_rejected_blocked")
        self.assertEqual(rejected_path, [])

        vetoed_status, vetoed_path = manager._compute_return_home_plan(
            manager._copy_floor_grid(grid),
            pose,
            goal,
            trace_snapshot=trace,
            visited_radius=0.45,
            veto_segments=((-3.0, 0.0, -2.5, 0.0),),
            failed_signature=manager._return_segment_signature((-2.5, 0.0)),
            veto_lookahead=0.8,
        )
        self.assertEqual(vetoed_status, "candidate_rejected_unreachable")
        self.assertEqual(vetoed_path, [])

    def test_return_home_stuck_feedback_is_bounded_and_fails_closed(self):
        recoveries = []
        cleared = []
        stops = []
        transitions = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.return_home_max_stuck_retries = 4
        manager._return_home_stuck_anchor = None
        manager._return_home_stuck_count = 0
        manager._return_home_failed_segments = []
        manager._return_home_last_failed_signature = None
        manager._return_home_candidate_reject_count = 0
        manager._return_home_last_stuck_obstacle = None
        manager._mission_success_eligible = True
        manager._failure_reasons = []
        manager._nearest_forward_obstacle_observation = SimpleNamespace(
            distance=0.236,
            world_xyz=(-0.27, 14.44, 1.2),
            forward=0.16,
            lateral=0.16,
            relative_height=1.2,
        )
        manager.controller = SimpleNamespace(
            recover=lambda reason=None: recoveries.append(reason),
            clear_path=lambda reason=None: cleared.append(reason),
            stop=lambda reason=None: stops.append(reason),
        )
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )
        segment = ((-0.25, 14.65), (0.15, 14.25))

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            for _ in range(4):
                manager._handle_return_home_stuck(
                    (-0.26, 14.67, -0.81),
                    segment,
                )

        self.assertEqual(recoveries, ["return_home_stuck"] * 3)
        self.assertEqual(manager._return_home_stuck_count, 4)
        self.assertEqual(len(manager._return_home_failed_segments), 1)
        self.assertEqual(
            manager._return_home_last_failed_signature,
            manager._return_segment_signature(segment[1]),
        )
        self.assertEqual(cleared, ["return_home_stuck_exhausted"])
        self.assertEqual(stops, ["return_home_stuck_exhausted"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "return_home_stuck_exhausted",
        )

    def test_return_home_veto_does_not_black_goal_neighborhood(self):
        manager = MissionManager.__new__(MissionManager)
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=2.0,
            length=2.0,
            origin_x=-1.0,
            origin_y=-1.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        goal = (0.4, 0.0)

        manager._apply_return_home_vetoes(
            grid,
            ((0.0, 0.0, 0.5, 0.0),),
            goal_xy=goal,
            lookahead=0.8,
        )

        goal_cell = grid.world_to_grid(*goal)
        self.assertNotEqual(grid.data[goal_cell[1], goal_cell[0]], BLACK)

    def test_return_home_feedback_parameters_reject_invalid_limits(self):
        manager = MissionManager.__new__(MissionManager)

        def parameter(name, default=None):
            if name == "runtime/profile":
                return "diagnostic"
            if name == "mission/return_home_max_stuck_retries":
                return 0
            return default

        with patch.object(mission_manager_module, "_param", side_effect=parameter):
            with self.assertRaisesRegex(ValueError, "返航卡滞"):
                manager._load_params()

    def test_floor_navigation_trace_is_sparse_and_pauses_only_during_floor_transition(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_floor = 0
        manager._floor_transition_guard = None
        manager._floor_navigation_traces = {}

        manager._record_floor_navigation_trace_pose((0.0, 0.0, 0.0))
        manager._record_floor_navigation_trace_pose((0.1, 0.0, 0.0))
        manager._record_floor_navigation_trace_pose((0.3, 0.0, 0.0))
        manager._floor_transition_guard = SimpleNamespace()
        manager._record_floor_navigation_trace_pose((0.6, 0.0, 0.0))
        manager._floor_transition_guard = None
        manager.state = RETURN_HOME
        manager._record_floor_navigation_trace_pose((0.9, 0.0, 0.0))

        self.assertEqual(
            manager._floor_navigation_traces[0],
            [(0.0, 0.0), (0.3, 0.0), (0.9, 0.0)],
        )

    def test_return_trace_keeps_fallback_attached_after_moving_from_return_start(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = RETURN_HOME
        manager.current_floor = 0
        manager._floor_transition_guard = None
        manager._floor_navigation_traces = {
            0: [(3.0 - 0.5 * index, 0.0) for index in range(13)]
        }
        manager._record_floor_navigation_trace_pose((-3.0, 0.3, 0.0))
        manager._record_floor_navigation_trace_pose((-3.0, 0.6, 0.0))
        manager._record_floor_navigation_trace_pose((-3.0, 0.9, 0.0))
        mapper = MultiFloorGridMap(
            resolution=0.25,
            width=8.0,
            length=5.0,
            origin_x=-4.0,
            origin_y=-2.0,
            visited_radius=0.45,
        )
        manager.mapper = mapper
        grid = mapper.floor(0)
        path = manager._return_path_from_navigation_trace(
            grid,
            (-3.0, 0.9, 0.0),
            (3.0, 0.0),
        )

        self.assertIsNotNone(path)
        self.assertLess(math.hypot(path[0][0] + 3.0, path[0][1] - 0.9), 0.5)
        self.assertLess(math.hypot(path[-1][0] - 3.0, path[-1][1]), 0.5)

    def test_opened_door_clear_covers_wait_point_and_inflation_on_both_sides(self):
        calls = []
        manager = MissionManager.__new__(MissionManager)
        manager.door_clear_radius = 0.9
        manager.door_pass_offset = 0.8
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": mission_manager_module.DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.4,
                )
            }
        )
        manager.mapper = SimpleNamespace(
            obstacle_inflation_radius=0.35,
            resolution=0.1,
            clear_dynamic_door_strip=lambda *args: calls.append(args),
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager.clear_opened_door_area("main_entrance")

        self.assertEqual(len(calls), 1)
        # depth 为完整长度，半深度需越过 0.8 m 等待点并覆盖 0.35 m 膨胀。
        self.assertAlmostEqual(calls[0][-1], 2.5)

    def test_initial_entry_trace_is_sparse_and_excludes_active_door_task(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager._initial_entry_origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._initial_entry_completed = False
        manager._initial_entry_trace = [(0.0, -3.2)]

        manager._record_initial_entry_trace_pose((0.0, -3.1, math.pi / 2.0))
        manager._record_initial_entry_trace_pose((0.0, -2.9, math.pi / 2.0))
        manager.current_task = OpenDoorAndPass("main_entrance")
        manager._record_initial_entry_trace_pose((0.0, -2.5, math.pi / 2.0))

        self.assertEqual(manager._initial_entry_trace, [(0.0, -3.2), (0.0, -2.9)])

    def test_initial_entry_realigns_locked_heading_between_forward_segments(self):
        rotations = []
        stops = []
        manager = MissionManager.__new__(MissionManager)
        manager.controller = SimpleNamespace(
            max_angular=1.2,
            rotate_in_place=lambda angular: rotations.append(angular),
            stop=lambda reason=None: stops.append(reason),
        )

        self.assertTrue(
            manager._tick_initial_entry_heading_alignment(
                (0.32, -0.55, 3.05),
                math.pi / 2.0,
            )
        )
        self.assertEqual(len(rotations), 1)
        self.assertAlmostEqual(rotations[0], -0.55)
        self.assertEqual(stops, [])

        rotations.clear()
        self.assertFalse(
            manager._tick_initial_entry_heading_alignment(
                (0.32, -0.55, math.pi / 2.0 + 0.1),
                math.pi / 2.0,
            )
        )
        self.assertEqual(rotations, [])

    def test_return_home_reverses_actual_initial_entry_trace_after_main_door(self):
        paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.home_pose = (0.0, -3.2, math.pi / 2.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.7
        manager.main_entrance_xy = (0.0, 0.0)
        manager.main_entrance_pass_vector = (0.0, 1.0)
        manager._return_home_main_entrance_crossed = True
        manager._return_home_entry_trace_attempted = False
        manager._initial_entry_trace = [
            (0.0, -3.2),
            (0.02, -2.8),
            (0.01, -2.4),
            (0.03, -1.1),
        ]
        manager.building_client = SimpleNamespace(doors={})
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path: paths.append(path),
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._tick_return_home((0.0, -0.8, -math.pi / 2.0))

        self.assertTrue(manager._return_home_entry_trace_attempted)
        self.assertEqual(len(paths), 1)
        self.assertEqual(paths[0][0], (0.0, -0.8))
        self.assertEqual(paths[0][-1], (0.0, -3.2))
        self.assertEqual(paths[0][1:-1], [(0.03, -1.1), (0.01, -2.4), (0.02, -2.8)])

    def test_should_open_door_accepts_visible_dynamic_candidate(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="main_entrance",
            distance=1.2,
            visible=True,
            kind="dynamic_door",
            floor_index=0,
            world_xy=(0.0, 0.0),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertTrue(manager._should_open_door(candidate))

    def test_should_open_door_rejects_elevator_candidate(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: True,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="elevator_floor_0",
            distance=1.2,
            visible=True,
            kind="elevator",
            floor_index=0,
            world_xy=(1.0, 2.6),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertFalse(manager._should_open_door(candidate))

    def test_should_open_door_does_not_interrupt_current_task(self):
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.current_task = object()
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="main_entrance",
            distance=1.2,
            visible=True,
            kind="dynamic_door",
            floor_index=0,
            world_xy=(0.0, 0.0),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertFalse(manager._should_open_door(candidate))

    def test_probe_unknown_space_starts_from_current_pose_after_passing_door(self):
        recorded_paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.home_pose = (0.0, -2.2, math.pi / 2.0)
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._probe_attempt = 0
        manager.current_frontier = object()
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.controller = SimpleNamespace(set_path=lambda path: recorded_paths.append(path))

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(loginfo=lambda *args, **kwargs: None)):
            self.assertTrue(manager._try_probe_unknown_space((0.0, 0.8, math.pi / 2.0)))

        self.assertEqual(len(recorded_paths), 1)
        self.assertAlmostEqual(recorded_paths[0][-1][0], 0.0, places=6)
        self.assertAlmostEqual(recorded_paths[0][-1][1], 2.0, places=6)

    def test_probe_unknown_space_uses_current_pose_heading(self):
        """短距离探测应沿当前机身朝向，不依赖固定世界方向参数。"""
        recorded_paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._probe_attempt = 0
        manager.current_frontier = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.controller = SimpleNamespace(set_path=lambda path: recorded_paths.append(path))

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertTrue(manager._try_probe_unknown_space((0.0, 0.0, 0.0)))

        self.assertEqual(len(recorded_paths), 1)
        self.assertAlmostEqual(recorded_paths[0][-1][0], 1.2, places=6)
        self.assertAlmostEqual(recorded_paths[0][-1][1], 0.0, places=6)

    def test_probe_unknown_space_skips_targets_behind_known_wall(self):
        recorded_paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.home_pose = (0.0, -2.2, math.pi / 2.0)
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._probe_attempt = 0
        manager.current_frontier = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        grid = manager.mapper.floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.0, 1.4))
        manager.controller = SimpleNamespace(set_path=lambda path: recorded_paths.append(path))

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(loginfo=lambda *args, **kwargs: None)):
            self.assertTrue(manager._try_probe_unknown_space((0.0, 0.8, math.pi / 2.0)))

        self.assertEqual(len(recorded_paths), 1)
        self.assertNotAlmostEqual(recorded_paths[0][-1][0], 0.0, places=6)

    def test_return_home_does_not_direct_fallback_through_blocked_wall(self):
        recorded_paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_pose = (2.0, 0.0, 0.0)
        manager.home_tolerance = 0.2
        manager.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        wall_x, _ = grid.world_to_grid(0.0, 0.0)
        for gy in range(grid.size_y):
            grid.mark_occupied(wall_x, gy)
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path: recorded_paths.append(path),
            tick=lambda pose: "idle",
            recover=lambda: None,
            stop=lambda reason=None: None,
        )

        pose = (-2.0, 0.0, 0.0)
        manager._tick_return_home(pose)
        self._finish_return_home_plan(manager, pose)

        self.assertEqual(recorded_paths, [])

    def test_remember_completed_door_records_transition_context(self):
        manager = MissionManager.__new__(MissionManager)
        manager.door_transition_duration = 3.0
        manager._post_door_frontier_debug = None
        manager.last_door_id = None
        manager.last_door_xy = None
        manager.last_door_pass_vector = None
        manager.last_door_transition_until = 0.0
        task = OpenDoorAndPass("main_entrance")
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)

        before = mission_manager_module.time.time()
        manager._remember_completed_door(task, (0.0, 1.75, math.pi / 2.0))

        self.assertEqual(manager.last_door_id, "main_entrance")
        self.assertEqual(manager.last_door_xy, (0.0, 0.0))
        self.assertEqual(manager.last_door_pass_vector, (0.0, 1.0))
        self.assertGreater(manager.last_door_transition_until, before)
        self.assertEqual(manager._post_door_frontier_debug["commit_point"], (0.0, 1.8))

    def test_frontier_selection_context_only_active_near_door_and_before_timeout(self):
        manager = MissionManager.__new__(MissionManager)
        manager.last_door_xy = (0.0, 0.0)
        manager.last_door_pass_vector = (0.0, 1.0)
        manager.last_door_transition_until = 105.0
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager._last_frontier_target = None
        manager.explorer = SimpleNamespace(
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )

        with patch.object(mission_manager_module, "get_time", return_value=103.0):
            context = manager._frontier_selection_context((0.0, 1.0, math.pi / 2.0))
            far_context = manager._frontier_selection_context((0.0, 3.0, math.pi / 2.0))

        self.assertIsNotNone(context)
        assert context is not None
        self.assertEqual(context.forbidden_half_plane_origin, (0.0, 0.0))
        self.assertEqual(context.forbidden_half_plane_normal, (0.0, 1.0))
        self.assertEqual(context.forbidden_radius, 1.5)
        self.assertTrue(context.skip_in_forbidden_half_plane)
        self.assertEqual(context.temporary_revisit_penalty, 6.0)
        self.assertIsNone(far_context)

        with patch.object(mission_manager_module, "get_time", return_value=106.0):
            expired_context = manager._frontier_selection_context((0.0, 1.0, math.pi / 2.0))

        self.assertIsNone(expired_context)

    def test_frontier_selection_context_persists_after_door_timer_until_post_door_progress(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.last_door_xy = (0.0, 0.0)
        manager.last_door_pass_vector = (0.0, 1.0)
        manager.last_door_transition_until = 100.0
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 2.0
        manager.post_door_min_known_area = 4.0
        manager._last_frontier_target = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.explorer = SimpleNamespace(
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )

        with patch.object(mission_manager_module, "get_time", return_value=106.0):
            context = manager._frontier_selection_context((0.0, 0.8, math.pi / 2.0))

        self.assertIsNotNone(context)
        assert context is not None
        self.assertTrue(context.skip_in_forbidden_half_plane)

    def test_post_door_context_has_absolute_expiry(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.last_door_id = "main_entrance"
        manager.last_door_xy = (0.0, 0.0)
        manager.last_door_pass_vector = (0.0, 1.0)
        manager.last_door_transition_until = 100.0
        manager.last_door_context_until = 105.0
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 5.0
        manager.post_door_min_known_area = 4.0
        manager._last_frontier_target = None
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.explorer = SimpleNamespace(
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )

        with patch.object(mission_manager_module, "get_time", return_value=106.0):
            context = manager._frontier_selection_context((0.0, 0.8, math.pi / 2.0))

        self.assertIsNone(context)

    def test_permanent_door_radius_scales_with_entrance_width(self):
        manager = MissionManager.__new__(MissionManager)
        manager.explorer = SimpleNamespace(door_front_half_plane_radius=1.5)
        manager.building_client = SimpleNamespace(
            doors={"main_entrance": SimpleNamespace(width=2.0)}
        )
        manager.permanent_forbidden_radius_scale = 1.5
        manager.permanent_forbidden_decay_scale = 3.0

        radius, decay, gate_half_width = manager._permanent_door_context_parameters()

        self.assertEqual(radius, 3.0)
        self.assertEqual(decay, 6.0)
        self.assertEqual(gate_half_width, 1.35)

    def test_frontier_selection_context_keeps_post_door_progress_beyond_hold_distance(self):
        manager = MissionManager.__new__(MissionManager)
        manager.last_door_xy = (0.0, 0.0)
        manager.last_door_pass_vector = (0.0, 1.0)
        manager.last_door_transition_until = 105.0
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 4.0
        manager.post_door_min_known_area = 0.0
        manager._last_frontier_target = None
        manager.explorer = SimpleNamespace(
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )

        with patch.object(mission_manager_module, "get_time", return_value=103.0):
            context = manager._frontier_selection_context((0.0, 3.0, math.pi / 2.0))

        self.assertIsNotNone(context)
        assert context is not None
        self.assertTrue(context.skip_in_forbidden_half_plane)

    def test_small_known_area_with_high_coverage_does_not_complete_floor(self):
        manager = MissionManager.__new__(MissionManager)
        manager.min_floor_explore_time = 20.0
        manager.min_floor_travel_distance = 6.0
        manager.min_known_area = 45.0
        manager.explore_coverage_ratio = 0.82
        manager.no_reachable_frontier_confirmations = 2
        manager._no_reachable_frontier_confirmations = 2
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager._max_home_distance = 8.0
        grid = MultiFloorGridMap(
            resolution=1.0,
            width=10.0,
            length=10.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        for gy in range(2):
            for gx in range(2):
                grid.mark_free(gx, gy)

        self.assertEqual(grid.coverage_ratio(), 1.0)
        self.assertLess(grid.known_area(), manager.min_known_area)
        self.assertFalse(manager._floor_completion_ready(grid, 30.0))

    def test_floor_completion_requires_repeated_no_reachable_confirmations(self):
        manager = MissionManager.__new__(MissionManager)
        manager.min_floor_explore_time = 20.0
        manager.min_floor_travel_distance = 6.0
        manager.min_known_area = 4.0
        manager.explore_coverage_ratio = 0.82
        manager.no_reachable_frontier_confirmations = 2
        manager._no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager._max_home_distance = 8.0
        grid = MultiFloorGridMap(
            resolution=1.0,
            width=10.0,
            length=10.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        for gy in range(3):
            for gx in range(3):
                grid.mark_free(gx, gy)

        self.assertFalse(manager._floor_completion_ready(grid, 30.0))
        manager._no_reachable_frontier_confirmations = 2
        self.assertTrue(manager._floor_completion_ready(grid, 30.0))


    def test_door_approach_selects_nearest_reachable_unopened_door(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager._door_approach_blacklist = {}
        manager.door_pass_offset = 0.8
        manager.active_door = None
        manager.current_task = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=12.0,
            length=12.0,
            origin_x=-6.0,
            origin_y=-6.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)

        def get_door_points(door_id, robot_pose, offset=0.8):
            # 两个门：near 在东侧，far 在西侧
            if door_id == "near_door":
                door_xy = (3.0, 0.0)
            else:
                door_xy = (-4.0, 0.0)
            # wait_point 在 door 北侧，简单示意
            return (door_xy[0], door_xy[1] + offset), (door_xy[0], door_xy[1] - offset)

        manager.building_client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                SimpleNamespace(door_id="far_door", floor_index=0),
                SimpleNamespace(door_id="near_door", floor_index=0),
            ],
            get_door_points=get_door_points,
        )

        selected = manager._try_select_door_approach((0.0, 0.0, 0.0))
        self.assertEqual(selected, "near_door")

    def test_door_approach_skips_unreachable_doors(self):
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager._door_approach_blacklist = {}
        manager.door_pass_offset = 0.8
        manager.active_door = None
        manager.current_task = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=12.0,
            length=12.0,
            origin_x=-6.0,
            origin_y=-6.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        # 在 x=0 处竖墙，把机器人（x<0）和门（x>0）隔开
        wall_x = grid.world_to_grid(0.0, 0.0)[0]
        for gy in range(grid.size_y):
            grid.mark_occupied(wall_x, gy)

        def get_door_points(door_id, robot_pose, offset=0.8):
            return ((3.0, 0.0 + offset), (3.0, 0.0 - offset))

        manager.building_client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                SimpleNamespace(door_id="blocked_door", floor_index=0),
            ],
            get_door_points=get_door_points,
        )

        selected = manager._try_select_door_approach((-3.0, 0.0, 0.0))
        self.assertIsNone(selected)
        self.assertIn("blocked_door", manager._door_approach_blacklist)

    def test_stuck_far_from_frontier_blacklists_region_without_blackmark(self):
        """远处卡住时只临时排除目标区域，不把远处目标写成地图障碍。"""
        recorded_failures: list[tuple[float, float]] = []
        black_marks: list[tuple] = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.manual_mode = False
        manager.start_time = 0.0
        manager.floor_start_time = 0.0
        manager.floor_timeout = 1000.0
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager.no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager._max_home_distance = 0.0
        manager._no_frontier_scan_until = 0.0
        manager._no_reachable_frontier_confirmations = 0
        manager._probe_attempt = 0
        manager._last_frontier_target = None
        manager._last_frontier_score = None
        manager._last_frontier_used_fallback = False
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_clear_radius = 0.9
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.current_task = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.5, width=20.0, length=20.0, origin_x=-10.0, origin_y=-10.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        # frontier 在远处，机器人在原点，距离 10m > 1.5m
        manager.current_frontier = SimpleNamespace(centroid=(10.0, 0.0))
        manager.explorer = SimpleNamespace(
            extract_frontiers=lambda g: [],
            record_stuck_failure=lambda g, xy: recorded_failures.append(xy),
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )
        manager.local_detector = SimpleNamespace(
            max_distance=3.0,
            detect_controllable_door=lambda **kw: None,
        )
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
            controllable_doors=lambda **kw: [],
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: True,
            tick=lambda pose: STUCK,
            recover=lambda: None,
            clear_path=lambda: None,
            stop=lambda *a, **k: None,
        )
        manager._door_approach_blacklist = {}
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager.last_door_xy = None
        manager.last_door_pass_vector = None
        manager.last_door_transition_until = 0.0
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager._post_door_frontier_debug = None
        manager._latest_cloud_points = []
        manager.mapper.mark_black = lambda floor, x, y, r: black_marks.append((x, y, r))

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(recorded_failures), 1)
        self.assertEqual(recorded_failures[0], (10.0, 0.0))
        self.assertEqual(black_marks, [])

    def test_stuck_near_frontier_only_blacklists_region_without_trapping_robot(self):
        """终点附近卡住也只排除 frontier，不能把当前位置写成永久禁区。"""
        recorded_failures: list[tuple[float, float]] = []
        black_marks: list[tuple] = []
        recover_calls: list[str] = []
        stop_reasons: list[str | None] = []
        events: list[dict] = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.manual_mode = False
        manager.start_time = 0.0
        manager.floor_start_time = 0.0
        manager.floor_timeout = 1000.0
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager.no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager._max_home_distance = 0.0
        manager._no_frontier_scan_until = 0.0
        manager._no_reachable_frontier_confirmations = 0
        manager._probe_attempt = 0
        manager._last_frontier_target = None
        manager._last_frontier_score = None
        manager._last_frontier_used_fallback = False
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_clear_radius = 0.9
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.current_task = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.5, width=20.0, length=20.0, origin_x=-10.0, origin_y=-10.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        # 复现实测：frontier 距机器人 0.4m，小于旧实现 0.6m 的 BLACK 半径。
        manager.current_frontier = SimpleNamespace(centroid=(0.4, 0.0))
        manager.explorer = SimpleNamespace(
            extract_frontiers=lambda g: [],
            record_stuck_failure=lambda g, xy: recorded_failures.append(xy),
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )
        manager.local_detector = SimpleNamespace(
            max_distance=3.0,
            detect_controllable_door=lambda **kw: None,
        )
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
            controllable_doors=lambda **kw: [],
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: True,
            tick=lambda pose: STUCK,
            recover=lambda: recover_calls.append("recover"),
            clear_path=lambda: None,
            stop=lambda *a, **k: stop_reasons.append(k.get("reason")),
        )
        manager._door_approach_blacklist = {}
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager.last_door_xy = None
        manager.last_door_pass_vector = None
        manager.last_door_transition_until = 0.0
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager._post_door_frontier_debug = None
        manager._latest_cloud_points = []
        manager.no_frontier_rotation_min_clearance = 0.40
        manager._nearest_radial_obstacle_distance = 0.30
        manager._no_frontier_rotation_blocked_logged = False
        manager._log_structured = events.append
        original_mark_black = manager.mapper.mark_black

        def record_mark_black(floor, x, y, radius):
            black_marks.append((x, y, radius))
            original_mark_black(floor, x, y, radius)

        manager.mapper.mark_black = record_mark_black

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(recorded_failures), 1)
        self.assertEqual(recorded_failures[0], (0.4, 0.0))
        self.assertEqual(black_marks, [])
        self.assertTrue(grid.is_traversable(*grid.world_to_grid(0.0, 0.0)))
        self.assertTrue(astar_plan(grid, (0.0, 0.0), (2.0, 0.0)).reached)
        self.assertEqual(recover_calls, [])
        self.assertEqual(stop_reasons, ["unsafe_recover_rotation_blocked"])
        self.assertEqual(events[0]["event"], "unsafe_rotation_blocked")
        self.assertEqual(events[0]["source"], "stuck_recovery")

    def test_main_entrance_failure_keeps_entrance_unset(self):
        """主入口过门失败后，main_entrance_xy/pass_vector 必须保持未设置。"""
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.start_time = 0.0
        manager.max_runtime = 1000.0
        manager._status_publisher = None
        manager._map_publisher = None
        manager._shutdown_requested = False
        manager._finished = False
        manager._timeout_triggered = False
        manager._failure_reasons = []
        manager._last_pose = None
        manager._last_status_publish = 0.0
        manager.status_publish_period = 1.0
        manager.home_pose = (0.0, 0.0, 0.0)
        manager.current_floor = 0
        manager._max_home_distance = 0.0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = "main_entrance"
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        # last_door_* 来自上一次其他门，不应被写入 main_entrance_*
        manager.last_door_xy = (5.0, 5.0)
        manager.last_door_pass_vector = (1.0, 0.0)
        manager.last_door_transition_until = 0.0
        manager.last_door_completion_pose = None
        manager.last_door_id = "other_door"
        manager.door_transition_duration = 3.0
        manager._post_door_frontier_debug = None
        manager.return_after_floor_change = False
        manager.target_floor = None
        manager.current_task = OpenDoorAndPass("main_entrance")
        manager._initial_entry_origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._initial_entry_completed = False
        manager._initial_entry_failed = False
        manager._mission_success_eligible = True
        transitions = []
        manager._transition_state = lambda new_state, **kwargs: (
            transitions.append((new_state, kwargs)),
            setattr(manager, "state", new_state),
        )
        # 让 task.tick 直接返回 FAILED
        manager.current_task.tick = lambda mgr, pose: FAILED
        manager.controller = SimpleNamespace(stop=lambda *a, **k: None)
        manager._latest_cloud_points = []
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager.mapper = MultiFloorGridMap(
            resolution=0.5, width=4.0, length=4.0, origin_x=-2.0, origin_y=-2.0,
        )
        manager.sensor_buffer = SimpleNamespace(
            snapshot=lambda: SimpleNamespace(
                odom_msg=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0), orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))),
                cloud_msg=None,
                image_bundle=None,
            ),
            read_cloud_xyz=lambda *a, **k: [],
        )
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=10.0, xyz=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0), angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._localization_health = True
        manager.localization_stance_health_required = True

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None, logerr=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager.tick()

        self.assertIsNone(manager.main_entrance_xy)
        self.assertIsNone(manager.main_entrance_pass_vector)
        self.assertIn("main_entrance", manager.failed_doors)
        self.assertIsNone(manager.current_task)
        self.assertEqual(manager.state, WRITE_RESULT)
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][1]["reason_code"], "main_entrance_crossing_failed")

    def test_main_entrance_success_sets_entrance_context(self):
        """主入口过门成功后，main_entrance_xy/pass_vector 应写入过门方向。"""
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager.start_time = 0.0
        manager.max_runtime = 1000.0
        manager._status_publisher = None
        manager._map_publisher = None
        manager._shutdown_requested = False
        manager._finished = False
        manager._timeout_triggered = False
        manager._failure_reasons = []
        manager._last_pose = None
        manager._last_status_publish = 0.0
        manager.status_publish_period = 1.0
        manager.home_pose = (0.0, 0.0, 0.0)
        manager.current_floor = 0
        manager._max_home_distance = 0.0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = "main_entrance"
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.post_door_min_distance = 0.9
        manager.door_pass_offset = 0.5
        manager.door_transition_duration = 3.0
        manager._post_door_frontier_debug = None
        manager.return_after_floor_change = False
        manager.target_floor = None
        manager.current_task = OpenDoorAndPass("main_entrance")
        # 模拟成功过门：tick 返回 DONE，并设置 wait/pass 点供 _remember_completed_door 使用
        manager.current_task._wait_point = (0.0, -0.8)
        manager.current_task._pass_point = (0.0, 0.8)
        manager.current_task._commit_point = (0.0, 1.8)
        from danger_search.navigator import DONE
        manager.current_task.tick = lambda mgr, pose: DONE
        manager.controller = SimpleNamespace(stop=lambda *a, **k: None)
        manager._latest_cloud_points = []
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager.mapper = MultiFloorGridMap(
            resolution=0.5, width=4.0, length=4.0, origin_x=-2.0, origin_y=-2.0,
        )
        manager.sensor_buffer = SimpleNamespace(
            snapshot=lambda: SimpleNamespace(
                odom_msg=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0), orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))),
                cloud_msg=None,
                image_bundle=None,
            ),
            read_cloud_xyz=lambda *a, **k: [],
        )
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=10.0, xyz=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0), angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._localization_health = True
        manager.localization_stance_health_required = True

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None, logerr=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager.tick()

        self.assertEqual(manager.main_entrance_xy, (0.0, 0.0))
        self.assertEqual(manager.main_entrance_pass_vector, (0.0, 1.0))
        self.assertIn("main_entrance", manager.opened_doors)
        self.assertIsNone(manager.current_task)
        self.assertEqual(manager._post_door_clearance_target, (0.0, 0.9))

    def test_main_entrance_clearance_keeps_axis_before_frontier_handoff(self):
        paths = []
        tick_kwargs = []
        manager = MissionManager.__new__(MissionManager)
        manager._post_door_clearance_target = (0.0, 0.9)
        manager._post_door_clearance_door_id = "main_entrance"
        manager._post_door_clearance_path_started = False
        manager.main_entrance_clearance_max_linear = 0.4
        manager.controller = SimpleNamespace(
            goal_tolerance=0.25,
            has_path=lambda: False,
            set_path=lambda path: paths.append(path),
            tick=lambda pose, **kwargs: tick_kwargs.append(kwargs) or RUNNING,
        )

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            active = manager._tick_main_entrance_clearance(
                (0.12, 0.24, math.pi / 2.0)
            )

        self.assertTrue(active)
        self.assertEqual(paths, [[(0.12, 0.24), (0.0, 0.9)]])
        self.assertEqual(tick_kwargs[0]["max_linear"], 0.4)
        self.assertEqual(tick_kwargs[0]["goal_tolerance"], 0.12)

    def test_main_entrance_clearance_stuck_fails_instead_of_scanning(self):
        transitions = []
        cleared = []
        manager = MissionManager.__new__(MissionManager)
        manager.state = EXPLORE_FLOOR
        manager._post_door_clearance_target = (0.0, 0.9)
        manager._post_door_clearance_door_id = "main_entrance"
        manager._post_door_clearance_path_started = True
        manager.main_entrance_clearance_max_linear = 0.4
        manager._mission_success_eligible = True
        manager._failure_reasons = []
        manager.controller = SimpleNamespace(
            goal_tolerance=0.25,
            has_path=lambda: True,
            tick=lambda pose, **kwargs: STUCK,
            clear_path=lambda reason=None: cleared.append(reason),
        )
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )

        active = manager._tick_main_entrance_clearance(
            (0.12, 0.24, math.pi / 2.0)
        )

        self.assertTrue(active)
        self.assertEqual(cleared, ["main_entrance_clearance_failed"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "main_entrance_clearance_failed",
        )

    def test_door_approach_budget_exhausted_does_not_blacklist(self):
        """门接近规划预算耗尽时不应黑名单该门，可能可达应留待重试。"""
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager._door_approach_blacklist = {}
        manager.door_pass_offset = 0.8
        manager.active_door = None
        manager.current_task = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.5, width=12.0, length=12.0, origin_x=-6.0, origin_y=-6.0,
        )

        def get_door_points(door_id, robot_pose, offset=0.8):
            return ((3.0, 0.0 + offset), (3.0, 0.0 - offset))

        manager.building_client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                SimpleNamespace(door_id="far_door", floor_index=0),
            ],
            get_door_points=get_door_points,
        )

        with patch("danger_search.mission_manager.astar_plan", return_value=PlanResult(status=BUDGET_EXHAUSTED)):
            selected = manager._try_select_door_approach((0.0, 0.0, 0.0))

        # 预算耗尽：不选择该门，也不黑名单。
        self.assertIsNone(selected)
        self.assertEqual(manager._door_approach_blacklist, {})

    def test_return_home_budget_exhausted_waits_for_map(self):
        """返航规划预算耗尽时应等待地图更新，不退化为直线返航。"""
        recorded_paths = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_pose = (5.0, 0.0, 0.0)
        manager.home_tolerance = 0.2
        manager.mapper = MultiFloorGridMap(
            resolution=1.0, width=8.0, length=8.0, origin_x=-4.0, origin_y=-4.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        cleared = []
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path: recorded_paths.append(path),
            tick=lambda pose: "idle",
            recover=lambda: None,
            clear_path=lambda reason=None: cleared.append(reason),
            stop=lambda reason=None: None,
        )

        with patch("danger_search.mission_manager.astar_plan", return_value=PlanResult(status=BUDGET_EXHAUSTED)), \
             patch.object(mission_manager_module, "rospy", SimpleNamespace(logwarn=lambda *a, **k: None)):
            pose = (0.0, 0.0, 0.0)
            manager._tick_return_home(pose)
            self._finish_return_home_plan(manager, pose)

        # 预算耗尽：不设路径，清空当前路径等待重试。
        self.assertEqual(recorded_paths, [])
        self.assertEqual(cleared, ["return_home_budget_exhausted"])

    def test_return_home_slow_planning_does_not_block_tick(self):
        """返航规划可以耗时，但 10 Hz 主循环必须立即进入安全停车。"""
        release = threading.Event()
        started = threading.Event()
        stops = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_pose = (2.0, 0.0, 0.0)
        manager.home_tolerance = 0.2
        manager.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            stop=lambda reason=None: stops.append(reason),
        )

        def slow_plan(*_args, **_kwargs):
            started.set()
            release.wait(timeout=1.0)
            return "unreachable", []

        manager._compute_return_home_plan = slow_plan
        started_at = time.monotonic()
        manager._tick_return_home((-2.0, 0.0, 0.0))
        elapsed = time.monotonic() - started_at

        self.assertTrue(started.wait(timeout=0.2))
        self.assertLess(elapsed, 0.1)
        self.assertEqual(stops, ["return_home_plan_pending"])
        release.set()
        manager._return_home_planner._thread.join(timeout=1.0)


class TimeBudgetDecisionTest(unittest.TestCase):
    """方案 C：全局时间预算与楼层/返航决策单元测试。"""

    def _make_manager(self, **overrides) -> MissionManager:
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.max_floor = overrides.get("max_floor", 1)
        manager.max_runtime = overrides.get("max_runtime", 120.0)
        manager.floor_timeout = overrides.get("floor_timeout", 40.0)
        manager.last_floor_uses_global_budget = overrides.get(
            "last_floor_uses_global_budget", True
        )
        manager.min_floor_explore_time = overrides.get("min_floor_explore_time", 5.0)
        manager.return_safety_buffer = overrides.get("return_safety_buffer", 10.0)
        manager.return_speed_estimate = overrides.get("return_speed_estimate", 0.5)
        manager.floor_change_time_estimate = overrides.get("floor_change_time_estimate", 20.0)
        manager.floor_finish_time_ratio = overrides.get("floor_finish_time_ratio", 0.75)
        manager.exploration_progress_window = overrides.get("exploration_progress_window", 10.0)
        manager.min_exploration_progress_cells = overrides.get("min_exploration_progress_cells", 20)
        manager.min_remaining_frontiers = overrides.get("min_remaining_frontiers", 2)
        manager.home_pose = overrides.get("home_pose", (0.0, 0.0, 0.0))
        manager._progress_history = []
        manager.tracker = SimpleNamespace(confirmed_positions=lambda **kw: [])
        manager.mapper = MultiFloorGridMap(
            resolution=0.5, width=10.0, length=10.0, origin_x=-5.0, origin_y=-5.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        return manager

    def test_mission_time_insufficient_triggers_return_home(self):
        """总剩余时间不足以支持返航时，必须立即返航。"""
        manager = self._make_manager(max_runtime=100.0, return_safety_buffer=10.0)
        # 机器人在 (15,0)，返航路径约 15m，保守时间约 15/0.5=30s；
        # 加上 return_safety_buffer=10s，mission_elapsed=65 时，65+30+10>=100，应返航。
        action, reason = manager._decide_mission_action(
            (15.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(20.0, 0.0))] * 5,
            floor_elapsed=20.0,
            mission_elapsed=65.0,
        )
        self.assertEqual(action, "RETURN_HOME")
        self.assertEqual(reason, "mission_insufficient_time_for_return")

    def test_plenty_of_time_allows_continue(self):
        """时间充足且有 frontier 时继续探索。"""
        manager = self._make_manager(max_runtime=120.0)
        action, reason = manager._decide_mission_action(
            (1.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(3.0, 0.0))] * 5,
            floor_elapsed=10.0,
            mission_elapsed=10.0,
        )
        self.assertEqual(action, "CONTINUE")
        self.assertEqual(reason, "budget_allows_continue")

    def test_floor_timeout_single_floor_uses_global_budget(self):
        """单层仍有收益且返航时间充足时，不应浪费剩余总预算。"""
        manager = self._make_manager(max_floor=1, max_runtime=120.0, floor_timeout=40.0)
        action, reason = manager._decide_mission_action(
            (3.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))] * 3,
            floor_elapsed=45.0,
            mission_elapsed=50.0,
        )
        self.assertEqual(action, "CONTINUE")
        self.assertEqual(reason, "last_floor_uses_global_budget")

    def test_floor_timeout_legacy_switch_returns_home(self):
        """关闭新语义时，单层到达原楼层上限后仍立即返航。"""
        manager = self._make_manager(
            max_floor=1,
            max_runtime=120.0,
            floor_timeout=40.0,
            last_floor_uses_global_budget=False,
        )
        action, reason = manager._decide_mission_action(
            (3.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))] * 3,
            floor_elapsed=45.0,
            mission_elapsed=50.0,
        )
        self.assertEqual(action, "RETURN_HOME")
        self.assertEqual(reason, "floor_timeout_return_home")

    def test_last_floor_of_multi_floor_uses_global_budget(self):
        """多层任务的最后一层同样由全局返航边界收口。"""
        manager = self._make_manager(max_floor=3, max_runtime=200.0, floor_timeout=40.0)
        manager.current_floor = 2
        action, reason = manager._decide_mission_action(
            (1.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))] * 3,
            floor_elapsed=45.0,
            mission_elapsed=80.0,
        )
        self.assertEqual(action, "CONTINUE")
        self.assertEqual(reason, "last_floor_uses_global_budget")

    def test_floor_timeout_multi_floor_with_elevator_changes_floor(self):
        """多楼层且存在可用电梯、剩余时间足够时，楼超时后换层。"""
        manager = self._make_manager(max_floor=2, max_runtime=200.0, floor_timeout=40.0)
        manager.building_client = SimpleNamespace(
            elevators={
                "main": SimpleNamespace(
                    lobby_poses={0: (0.0, 0.0, 0.0), 1: (0.0, 0.0, 0.0)},
                )
            }
        )
        action, reason = manager._decide_mission_action(
            (1.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))],
            floor_elapsed=45.0,
            mission_elapsed=50.0,
        )
        self.assertEqual(action, "FINISH_FLOOR")
        self.assertEqual(reason, "floor_timeout_change_floor")

    def test_floor_soft_timeout_low_progress_returns_home(self):
        """楼层软超时后探索收益低且 frontier 少，应结束当前层返航。"""
        manager = self._make_manager(
            max_floor=1,
            max_runtime=120.0,
            floor_timeout=40.0,
            floor_finish_time_ratio=0.75,
        )
        # 填充历史使 known_cells 无增长
        now = 100.0
        manager._progress_history = [
            {"time": now - 10.0, "known_cells": 100, "free_or_visited_cells": 80, "confirmed_dangers": 0},
            {"time": now, "known_cells": 100, "free_or_visited_cells": 80, "confirmed_dangers": 0},
        ]
        with patch.object(mission_manager_module, "get_time", return_value=now):
            action, reason = manager._decide_mission_action(
                (3.0, 0.0, 0.0),
                manager.mapper.floor(0),
                [SimpleNamespace(centroid=(5.0, 0.0))],  # 少于 min_remaining_frontiers=2
                floor_elapsed=35.0,  # >= 40*0.75
                mission_elapsed=40.0,
            )
        self.assertEqual(action, "RETURN_HOME")
        self.assertEqual(reason, "floor_low_progress_return_home")

    def test_no_home_pose_defers_decision(self):
        """home_pose 尚未记录时不应触发返航。"""
        manager = self._make_manager()
        manager.home_pose = None
        action, reason = manager._decide_mission_action(
            (1.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(3.0, 0.0))] * 5,
            floor_elapsed=10.0,
            mission_elapsed=10.0,
        )
        self.assertEqual(action, "CONTINUE")
        self.assertEqual(reason, "home_pose_not_recorded")


class ProgressMonitorDiagnosticOnlyTest(unittest.TestCase):
    """阶段二：ProgressMonitor 改为 diagnostic-only 的链路测试。"""

    def _make_explore_manager(self, *, frontiers_list=None, select_result=None):
        """构造可调用 _tick_explore 的轻量 MissionManager。"""
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.manual_mode = False
        manager.start_time = 0.0
        manager.floor_start_time = 0.0
        manager.floor_timeout = 1000.0
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager.no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._max_home_distance = 0.0
        manager._no_frontier_scan_until = 0.0
        manager._no_reachable_frontier_confirmations = 0
        manager._last_no_frontier_signature = None
        manager._no_frontier_escape_state = "idle"
        manager._no_frontier_escape_target = None
        manager._no_frontier_escape_deadline = 0.0
        manager.no_frontier_rotation_min_clearance = 0.40
        manager.no_frontier_retreat_max_distance = 2.0
        manager.no_frontier_retreat_clearance_margin = 0.10
        manager._nearest_radial_obstacle_distance = 1.0
        manager._floor_navigation_traces = {0: []}
        manager._probe_attempt = 0
        manager._last_frontier_target = None
        manager._last_frontier_score = None
        manager._last_frontier_used_fallback = False
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_clear_radius = 0.9
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.current_task = None
        manager.current_frontier = None
        manager.mapper = MultiFloorGridMap(
            resolution=0.5, width=20.0, length=20.0, origin_x=-10.0, origin_y=-10.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        manager.explorer = SimpleNamespace(
            extract_frontiers=lambda g: list(frontiers_list or []),
            record_stuck_failure=lambda g, xy: None,
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
            last_selection_debug=None,
        )
        if select_result is not None:
            manager.explorer.select_frontier = lambda *a, **kw: select_result
        manager.local_detector = SimpleNamespace(
            max_distance=3.0,
            detect_controllable_door=lambda **kw: None,
        )
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
            controllable_doors=lambda **kw: [],
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            tick=lambda pose: "running",
            recover=lambda: None,
            clear_path=lambda *a, **k: None,
            stop=lambda *a, **k: None,
            set_path=lambda p: None,
            reverse_speed=-0.15,
            reverse_straight=lambda **kwargs: True,
        )
        manager._door_approach_blacklist = {}
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager.last_door_xy = None
        manager.last_door_pass_vector = None
        manager.last_door_transition_until = 0.0
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager._post_door_frontier_debug = None
        manager._latest_cloud_points = []
        manager._progress_monitor = None
        manager.progress_monitor_cooldown = 10.0
        manager.state = EXPLORE_FLOOR
        manager.home_pose = None
        manager.max_runtime = 1000.0
        manager.return_safety_buffer = 30.0
        manager.return_speed_estimate = 0.5
        manager.floor_change_time_estimate = 40.0
        manager.floor_finish_time_ratio = 0.75
        manager.exploration_progress_window = 15.0
        manager.min_exploration_progress_cells = 30
        manager.min_remaining_frontiers = 2
        manager.max_floor = 1
        manager._progress_history = []
        manager._last_budget_decision = None
        manager._last_budget_decision_time = 0.0
        manager._floor_timeout_logged = False
        manager._status_publisher = None
        manager._last_status_publish = 0.0
        manager.status_publish_period = 1.0
        manager._map_publisher = None
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        return manager

    def test_explore_does_not_use_progress_monitor_to_filter_frontiers(self):
        """frontier 选择不再受 ProgressMonitor.filter_frontiers 影响。"""
        from danger_search.frontier import Frontier
        frontier = Frontier(cells=((10, 10),), centroid=(5.0, 0.0), information_gain=10.0)
        select_result = (frontier, [(0.0, 0.0), (5.0, 0.0)])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )
        # filter_frontiers 返回空列表，模拟 ProgressMonitor 过滤掉所有 frontier
        monitor = _ProgressMonitor(
            goal_tolerance=0.35, window=5.0, repeat=3,
            min_displacement=0.20, cooldown=10.0,
        )
        monitor.filter_frontiers = lambda now, frontiers: []
        manager._progress_monitor = monitor

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        # 尽管 filter_frontiers 返回空，select_frontier 仍被调用并选中 frontier
        self.assertIsNotNone(manager.current_frontier)
        self.assertEqual(manager.current_frontier.centroid, (5.0, 0.0))

    def test_active_path_skips_unused_frontier_extraction_before_soft_budget(self):
        manager = self._make_explore_manager()
        extraction_calls = []
        manager.explorer.extract_frontiers = lambda grid: extraction_calls.append(grid) or []
        manager.controller.has_path = lambda: True
        manager.controller.tick = lambda pose: "running"
        manager.floor_timeout = 100.0
        manager.floor_finish_time_ratio = 0.75

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(extraction_calls, [])

    def test_active_path_refreshes_frontiers_when_soft_budget_uses_count(self):
        manager = self._make_explore_manager()
        extraction_calls = []
        manager.explorer.extract_frontiers = lambda grid: extraction_calls.append(grid) or []
        manager.controller.has_path = lambda: True
        manager.controller.tick = lambda pose: "running"
        manager.floor_timeout = 100.0
        manager.floor_finish_time_ratio = 0.75
        manager.home_pose = None

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ), patch.object(mission_manager_module, "get_time", return_value=80.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(extraction_calls), 1)

    def test_progress_monitor_degenerate_does_not_clear_path_or_reselect(self):
        """退化检测不执行 cooldown、clear_path 或主动重选。"""
        from danger_search.frontier import Frontier
        frontier = Frontier(cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0)
        select_result = (frontier, [(0.0, 0.0), (0.30, 0.0)])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )
        clear_calls = []
        manager.controller.clear_path = lambda *a, **k: clear_calls.append(True)

        monitor = _ProgressMonitor(
            goal_tolerance=0.35, window=5.0, repeat=3,
            min_displacement=0.20, cooldown=10.0,
        )
        # 预填选择历史，使 check_degenerate 必然返回 is_degenerate=True
        for i in range(5):
            monitor.record_selection(float(i) * 0.1, (0.30, 0.0))
        manager._progress_monitor = monitor

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=0.5):
            manager._tick_explore((0.0, 0.0, 0.0))

        # 退化检测不应触发 clear_path
        self.assertEqual(clear_calls, [])
        # current_frontier 不应被重置（保留选中的 frontier）
        self.assertIsNotNone(manager.current_frontier)

    def test_progress_monitor_still_records_pose_and_selection(self):
        """ProgressMonitor 仍记录 pose 和 selection 诊断数据。"""
        from danger_search.frontier import Frontier
        frontier = Frontier(cells=((10, 10),), centroid=(5.0, 0.0), information_gain=10.0)
        select_result = (frontier, [(0.0, 0.0), (5.0, 0.0)])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )

        record_pose_calls = []
        record_selection_calls = []
        monitor = _ProgressMonitor(
            goal_tolerance=0.35, window=5.0, repeat=3,
            min_displacement=0.20, cooldown=10.0,
        )
        monitor.record_pose = lambda t, p: record_pose_calls.append((t, p))
        monitor.record_selection = lambda t, xy: record_selection_calls.append((t, xy))
        manager._progress_monitor = monitor

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((1.0, 2.0, 0.0))

        self.assertGreater(len(record_pose_calls), 0)
        self.assertEqual(record_pose_calls[0][1], (1.0, 2.0, 0.0))
        self.assertGreater(len(record_selection_calls), 0)
        self.assertEqual(record_selection_calls[0][1], (5.0, 0.0))

    def test_inside_tolerance_rejection_only_at_source(self):
        """inside_goal_tolerance 拒绝只发生在 FrontierExplorer 源头，不依赖 ProgressMonitor。"""
        from danger_search.frontier import FrontierExplorer
        from danger_search.mapper import FloorGrid

        grid = FloorGrid(
            floor_index=0, resolution=0.1,
            width=4.0, length=4.0, origin_x=-2.0, origin_y=-2.0,
        )
        for gy in range(5, 35):
            for gx in range(5, 35):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(min_cluster_size=1, goal_tolerance=0.35)
        recorded_failures = []
        explorer.record_failure = lambda g, xy: recorded_failures.append(xy)

        frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        # 选中的目标应在 tolerance 外
        if frontier is not None and path:
            self.assertGreater(
                math.hypot(frontier.centroid[0], frontier.centroid[1]), 0.35,
            )
        # 不记录 failure
        self.assertEqual(recorded_failures, [])

    def test_all_candidates_inside_tolerance_enters_no_frontier_flow(self):
        """所有候选过近时进入现有 no-frontier/probe 流程，不形成高频循环。"""
        manager = self._make_explore_manager(
            frontiers_list=[],
            select_result=(None, []),
        )
        manager._no_frontier_scan_until = 0.0
        manager.no_frontier_scan_duration = 8.0
        manager.home_pose = (0.0, 0.0, 0.0)
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager._max_home_distance = 10.0

        set_path_calls = []
        manager.controller.set_path = lambda p: set_path_calls.append(p)

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        # 没有 frontier 时进入 probe 流程（set_path 被调用来执行短距离探测）
        # 或进入 scan 流程（rotate_in_place）
        # 无论哪种，都不应记录 failure
        self.assertEqual(manager._no_reachable_frontier_confirmations, 0)

    def test_all_inside_tolerance_candidates_can_start_probe(self):
        """列表非空但全部过近时，也应进入现有短距离探测流程。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug

        frontier = Frontier(
            cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0,
        )
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=(None, []),
        )
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1,
            reachable_count=0,
            blacklisted_count=0,
            unreachable_count=0,
            context_filtered_count=0,
            used_fallback=False,
            final_target=None,
            final_score=0.0,
            path_length=0.0,
            inside_tolerance_count=1,
        )
        manager.no_frontier_scan_duration = 8.0
        recorded_paths = []
        manager.controller.set_path = lambda path: recorded_paths.append(path)
        manager.controller.rotate_in_place = lambda value: None

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(recorded_paths), 1)
        self.assertEqual(manager._no_reachable_frontier_confirmations, 0)

    def test_all_inside_tolerance_no_repeat_astar_same_pose_map(self):
        """所有候选过近且 pose/map 不变时，连续多个 tick 不重复执行完整 A*。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        frontier = Frontier(cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0)
        select_result = (None, [])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )
        # 设置 last_selection_debug 模拟所有候选过近
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1,
            reachable_count=0,
            blacklisted_count=0,
            unreachable_count=0,
            context_filtered_count=0,
            used_fallback=False,
            final_target=None,
            final_score=0.0,
            path_length=0.0,
            inside_tolerance_count=1,
        )
        manager.no_frontier_scan_duration = 8.0
        manager._try_probe_unknown_space = lambda pose: False
        select_calls = []
        manager.explorer.select_frontier = lambda *a, **kw: select_calls.append(kw) or (None, [])
        manager.explorer.extract_frontiers = lambda g: [frontier]

        rotate_calls = []
        manager.controller.rotate_in_place = lambda v: rotate_calls.append(v)

        times = [10.0, 10.1, 10.2, 10.3, 10.5, 11.0, 12.0]
        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            for t in times:
                with patch.object(mission_manager_module, "get_time", return_value=t):
                    manager._tick_explore((0.0, 0.0, 0.0))

        # select_frontier 只在第一个 tick 调用一次（之后扫描门控阻止重复 A*）
        self.assertEqual(len(select_calls), 1)
        # 后续 tick 执行 rotate_in_place
        self.assertGreater(len(rotate_calls), 0)
        # 不记录 failure
        self.assertEqual(manager._no_reachable_frontier_confirmations, 0)

    def test_scan_same_count_changed_frontier_restores_selection(self):
        """扫描中 frontier 数量相同但内容变化时，立即恢复选择。"""
        from danger_search.frontier import Frontier
        frontier_old = Frontier(cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0)
        frontier_new = Frontier(cells=((60, 60),), centroid=(3.0, 3.0), information_gain=15.0)
        manager = self._make_explore_manager(
            frontiers_list=[frontier_old],
            select_result=(None, []),
        )
        manager.no_frontier_scan_duration = 1.0
        manager._try_probe_unknown_space = lambda pose: False

        from danger_search.frontier import FrontierSelectionDebug
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1, reachable_count=0, blacklisted_count=0,
            unreachable_count=0, context_filtered_count=0, used_fallback=False,
            final_target=None, final_score=0.0, path_length=0.0, inside_tolerance_count=1,
        )

        current_frontiers = [frontier_old]
        select_results = [(None, [])]

        def extract(g):
            return list(current_frontiers)
        def select_fn(*a, **kw):
            return select_results[-1]

        manager.explorer.extract_frontiers = extract
        manager.explorer.select_frontier = select_fn

        rotate_calls = []
        manager.controller.rotate_in_place = lambda v: rotate_calls.append(v)

        set_path_calls = []
        manager.controller.set_path = lambda p: set_path_calls.append(p)

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            # tick 1: 启动扫描
            with patch.object(mission_manager_module, "get_time", return_value=10.0):
                manager._tick_explore((0.0, 0.0, 0.0))
            # tick 2: 扫描中，frontier 数量不变，跳过
            with patch.object(mission_manager_module, "get_time", return_value=10.5):
                manager._tick_explore((0.0, 0.0, 0.0))
            # tick 3: 旧 frontier 被新 frontier 替换（数量仍为 1），恢复选择
            current_frontiers = [frontier_new]
            select_results.append((frontier_new, [(0.0, 0.0), (3.0, 3.0)]))
            with patch.object(mission_manager_module, "get_time", return_value=10.6):
                manager._tick_explore((0.0, 0.0, 0.0))

        # 新 frontier 被选中
        self.assertIsNotNone(manager.current_frontier)
        self.assertEqual(manager.current_frontier.centroid, (3.0, 3.0))
        self.assertGreater(len(set_path_calls), 0)

    def test_scan_robot_cell_change_restores_selection(self):
        """扫描中机器人移动到新栅格时，即使 frontier 不变也应恢复选择。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        near_frontier = Frontier(
            cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0,
        )
        valid_frontier = Frontier(
            cells=((26, 26),), centroid=(3.0, 3.0), information_gain=15.0,
        )
        manager = self._make_explore_manager(frontiers_list=[near_frontier])
        manager.no_frontier_scan_duration = 8.0
        manager._try_probe_unknown_space = lambda pose: False
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1, reachable_count=0, blacklisted_count=0,
            unreachable_count=0, context_filtered_count=0, used_fallback=False,
            final_target=None, final_score=0.0, path_length=0.0,
            inside_tolerance_count=1,
        )
        select_results = [
            (None, []),
            (valid_frontier, [(1.0, 0.0), (3.0, 3.0)]),
        ]
        select_calls = []

        def select_fn(*args, **kwargs):
            result = select_results[len(select_calls)]
            select_calls.append(kwargs)
            return result

        manager.explorer.select_frontier = select_fn
        manager.explorer.extract_frontiers = lambda grid: [near_frontier]
        manager.controller.rotate_in_place = lambda value: None
        set_path_calls = []
        manager.controller.set_path = lambda path: set_path_calls.append(path)

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            with patch.object(mission_manager_module, "get_time", return_value=10.0):
                manager._tick_explore((0.0, 0.0, 0.0))
            with patch.object(mission_manager_module, "get_time", return_value=10.1):
                manager._tick_explore((1.0, 0.0, 0.0))

        self.assertEqual(len(select_calls), 2)
        self.assertIs(manager.current_frontier, valid_frontier)
        self.assertEqual(len(set_path_calls), 1)

    def test_no_reachable_confirmations_not_incremented_per_tick(self):
        """no_reachable_frontier_confirmations 不按 tick 快速累加，只在扫描完成后增长。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        frontier = Frontier(cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0)
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=(None, []),
        )
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1, reachable_count=0, blacklisted_count=0,
            unreachable_count=0, context_filtered_count=0, used_fallback=False,
            final_target=None, final_score=0.0, path_length=0.0, inside_tolerance_count=1,
        )
        manager.no_frontier_scan_duration = 8.0
        manager._try_probe_unknown_space = lambda pose: False

        manager.explorer.select_frontier = lambda *a, **kw: (None, [])
        manager.explorer.extract_frontiers = lambda g: [frontier]
        manager.controller.rotate_in_place = lambda v: None

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            for t in [10.0, 10.1, 10.2, 10.3, 10.4, 10.5]:
                with patch.object(mission_manager_module, "get_time", return_value=t):
                    manager._tick_explore((0.0, 0.0, 0.0))

        # 所有候选过近时 confirmations 不增长
        self.assertEqual(manager._no_reachable_frontier_confirmations, 0)

    def test_no_frontier_scan_stops_instead_of_rotating_when_wall_is_close(self):
        """贴墙无前沿扫描必须零速等待，不能让机身旋转包络扫墙。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager.no_frontier_scan_duration = 4.0
        manager.no_frontier_rotation_min_clearance = 0.40
        manager._nearest_radial_obstacle_distance = 0.20
        manager._no_frontier_scan_until = 0.0
        manager._no_frontier_rotation_blocked_logged = False
        rotate_calls = []
        stop_reasons = []
        events = []
        manager.controller.rotate_in_place = rotate_calls.append
        manager.controller.stop = lambda *args, **kwargs: stop_reasons.append(
            kwargs.get("reason")
        )
        manager._log_structured = events.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            completed = manager._tick_no_frontier_scan([])

        self.assertFalse(completed)
        self.assertEqual(rotate_calls, [])
        self.assertEqual(stop_reasons, ["unsafe_no_frontier_rotation_blocked"])
        self.assertEqual(events[0]["event"], "unsafe_rotation_blocked")
        self.assertEqual(events[0]["clearance"], 0.2)

    def test_no_frontier_scan_rotates_when_radial_clearance_is_safe(self):
        manager = self._make_explore_manager(frontiers_list=[])
        manager.no_frontier_scan_duration = 4.0
        manager.no_frontier_rotation_min_clearance = 0.40
        manager._nearest_radial_obstacle_distance = 0.65
        manager._no_frontier_scan_until = 0.0
        manager._no_frontier_rotation_blocked_logged = False
        rotate_calls = []
        manager.controller.rotate_in_place = rotate_calls.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            completed = manager._tick_no_frontier_scan([])

        self.assertFalse(completed)
        self.assertEqual(rotate_calls, [0.45])

    def test_no_frontier_close_wall_retreats_on_traversed_trace_without_mutating_map(self):
        """贴墙无前沿时只沿真实轨迹退到宽处，不能改写地图制造通路。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager._nearest_radial_obstacle_distance = 0.33
        manager._floor_navigation_traces = {
            0: [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
        }
        grid = manager.mapper.floor(0)
        obstacle = grid.world_to_grid(2.0, 0.5)
        self.assertIsNotNone(obstacle)
        grid.mark_occupied(*obstacle)
        before = np.array(grid.data, copy=True)
        events = []
        manager._log_structured = events.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            completed = manager._tick_no_frontier_scan(
                [], pose=(2.0, 0.0, 0.0)
            )

        self.assertFalse(completed)
        self.assertEqual(manager._no_frontier_escape_state, "moving")
        self.assertEqual(manager._no_frontier_escape_target, (1.0, 0.0))
        self.assertGreater(manager._no_frontier_escape_deadline, 10.0)
        self.assertTrue(astar_plan(grid, (2.0, 0.0), (1.0, 0.0)).reached)
        np.testing.assert_array_equal(grid.data, before)
        self.assertEqual(events[-1]["event"], "no_frontier_retreat_started")

    def test_no_frontier_close_wall_without_safe_trace_finishes_floor_once(self):
        """没有安全沿迹候选时立即结束本层，不重复零速扫描。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager._nearest_radial_obstacle_distance = 0.33
        manager._floor_navigation_traces = {0: []}
        manager._try_probe_unknown_space = lambda pose: False
        manager._try_select_door_approach = lambda pose: None
        finish_calls = []
        manager._finish_current_floor = lambda **kwargs: finish_calls.append(kwargs)
        manager._log_structured = lambda event: None

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            manager._tick_explore((2.0, 0.0, 0.0))

        self.assertEqual(manager._no_frontier_escape_state, "exhausted")
        self.assertEqual(len(finish_calls), 1)
        self.assertEqual(
            finish_calls[0]["reason_code"], "no_frontier_escape_exhausted"
        )

    def test_no_frontier_retreat_allows_only_one_followup_scan(self):
        """退到宽处后的复扫完成即耗尽，不重新进入无界扫描循环。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager._no_frontier_escape_state = "scan_once"
        manager._nearest_radial_obstacle_distance = 0.65
        manager.no_frontier_scan_duration = 4.0
        rotate_calls = []
        manager.controller.rotate_in_place = rotate_calls.append

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            with patch.object(mission_manager_module, "get_time", return_value=10.0):
                self.assertFalse(
                    manager._tick_no_frontier_scan([], pose=(1.0, 0.0, 0.0))
                )
            with patch.object(mission_manager_module, "get_time", return_value=14.1):
                self.assertTrue(
                    manager._tick_no_frontier_scan([], pose=(1.0, 0.0, 0.0))
                )

        self.assertEqual(rotate_calls, [0.45])
        self.assertEqual(manager._no_frontier_escape_state, "exhausted")

    def test_no_frontier_retreat_reverses_without_turning_then_requests_rescan(self):
        manager = self._make_explore_manager(frontiers_list=[])
        manager._no_frontier_escape_state = "moving"
        manager._no_frontier_escape_target = (1.0, 0.0)
        manager._no_frontier_escape_deadline = 20.0
        reverse_calls = []
        manager.controller.reverse_straight = lambda **kwargs: reverse_calls.append(
            kwargs
        ) or True
        events = []
        manager._log_structured = events.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_no_frontier_escape_motion((2.0, 0.0, 0.0))
        self.assertEqual(reverse_calls, [{"reason": "no_frontier_retreat"}])
        self.assertEqual(manager._no_frontier_escape_state, "moving")

        with patch.object(mission_manager_module, "get_time", return_value=12.0):
            manager._tick_no_frontier_escape_motion((1.2, 0.0, 0.0))

        self.assertEqual(manager._no_frontier_escape_state, "scan_once")
        self.assertIsNone(manager._no_frontier_escape_target)
        self.assertEqual(events[-1]["event"], "no_frontier_retreat_completed")

    def test_nearest_radial_obstacle_distance_filters_height_and_self_returns(self):
        from danger_search.mapper import MultiFloorGridMap

        mapper = MultiFloorGridMap(
            floor_height=2.6,
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        points = [
            (0.02, 0.01, 2.9),  # 过近的自体回波
            (0.20, 0.00, 2.9),  # 当前楼层实体
            (0.10, 0.00, 4.5),  # 高于障碍过滤范围
        ]

        distance = mission_manager_module.nearest_radial_obstacle_distance(
            points,
            (0.0, 0.0, 0.0),
            floor_index=1,
            mapper=mapper,
        )

        self.assertAlmostEqual(distance, 0.20)

    def test_budget_exhausted_frontier_is_confirmed_before_no_reachable_count(self):
        """预算耗尽不是不可达；完整扫描后确认出路径应立即继续探索。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug

        unresolved = Frontier(
            cells=((10, 10),), centroid=(5.0, 0.0), information_gain=20.0,
        )
        confirmed = Frontier(
            cells=((10, 10),), centroid=(4.5, 0.0), information_gain=20.0,
        )
        manager = self._make_explore_manager(
            frontiers_list=[unresolved],
            select_result=(None, []),
        )
        manager.no_frontier_scan_duration = 8.0
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1,
            reachable_count=0,
            blacklisted_count=0,
            unreachable_count=0,
            context_filtered_count=0,
            used_fallback=False,
            final_target=None,
            final_score=0.0,
            path_length=0.0,
            budget_exhausted_count=1,
        )
        confirmation_calls = []
        manager.explorer.confirm_budget_exhausted_frontier = (
            lambda *args, **kwargs: confirmation_calls.append((args, kwargs))
            or (confirmed, [(0.0, 0.0), (4.5, 0.0)])
        )
        paths = []
        manager.controller.set_path = paths.append
        manager.controller.rotate_in_place = lambda value: None

        with patch.object(
            mission_manager_module,
            "rospy",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logwarn=lambda *args, **kwargs: None,
            ),
        ):
            with patch.object(mission_manager_module, "get_time", return_value=10.0):
                manager._tick_explore((0.0, 0.0, 0.0))
            with patch.object(mission_manager_module, "get_time", return_value=18.0):
                manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(confirmation_calls), 1)
        self.assertIs(manager.current_frontier, confirmed)
        self.assertEqual(paths, [[(0.0, 0.0), (4.5, 0.0)]])
        self.assertEqual(manager._no_reachable_frontier_confirmations, 0)

    def test_all_inside_confirmation_increments_once_after_full_scan(self):
        """完整扫描结束后仍全部过近时，只增加一次 confirmation 且不记录 failure。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        frontier = Frontier(
            cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0,
        )
        manager = self._make_explore_manager(frontiers_list=[frontier])
        manager.no_frontier_scan_duration = 8.0
        manager._try_probe_unknown_space = lambda pose: False
        manager.no_reachable_frontier_confirmations = 2
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1, reachable_count=0, blacklisted_count=0,
            unreachable_count=0, context_filtered_count=0, used_fallback=False,
            final_target=None, final_score=0.0, path_length=0.0,
            inside_tolerance_count=1,
        )
        select_calls = []
        manager.explorer.select_frontier = (
            lambda *args, **kwargs: select_calls.append(kwargs) or (None, [])
        )
        manager.explorer.extract_frontiers = lambda grid: [frontier]
        manager.controller.rotate_in_place = lambda value: None
        failures = []
        manager._record_failure = lambda reason: failures.append(reason)

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            for timestamp in (10.0, 10.1, 12.0, 17.9):
                with patch.object(
                    mission_manager_module, "get_time", return_value=timestamp,
                ):
                    manager._tick_explore((0.0, 0.0, 0.0))
            self.assertEqual(manager._no_reachable_frontier_confirmations, 0)
            with patch.object(mission_manager_module, "get_time", return_value=18.0):
                manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(select_calls), 2)
        self.assertEqual(manager._no_reachable_frontier_confirmations, 1)
        self.assertEqual(failures, [])

    def test_progress_monitor_intervention_count_remains_zero(self):
        """ProgressMonitor 主动干预次数仍为 0（diagnostic-only）。"""
        from danger_search.frontier import Frontier
        frontier = Frontier(cells=((10, 10),), centroid=(5.0, 0.0), information_gain=10.0)
        select_result = (frontier, [(0.0, 0.0), (5.0, 0.0)])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )

        cooldown_calls = []
        filter_calls = []
        monitor = _ProgressMonitor(
            goal_tolerance=0.35, window=5.0, repeat=3,
            min_displacement=0.20, cooldown=10.0,
        )
        monitor.start_cooldown = lambda *a, **k: cooldown_calls.append(True)
        monitor.filter_frontiers = lambda *a, **k: filter_calls.append(True) or a[1]
        manager._progress_monitor = monitor

        clear_calls = []
        manager.controller.clear_path = lambda *a, **k: clear_calls.append(True)

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(cooldown_calls, [])
        self.assertEqual(filter_calls, [])
        self.assertEqual(clear_calls, [])


if __name__ == "__main__":
    unittest.main()

import json
import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.local_semantic_detector import DoorCandidate
from danger_search.mapper import MultiFloorGridMap
import danger_search.mission_manager as mission_manager_module
from danger_search.mission_manager import (
    EXPLORE_FLOOR,
    FINISH,
    MISSION_FAILED,
    RETURN_HOME,
    WRITE_RESULT,
    _ProgressMonitor,
    nearest_forward_obstacle_distance,
)
from danger_search.mission_manager import MissionManager
from danger_search.mission_manager import probe_target_for_attempt
from danger_search.navigation_tasks import OpenDoorAndPass, TakeElevator, FAILED
from danger_search.navigator import STUCK
from danger_search.planner import PlanResult, BUDGET_EXHAUSTED


class MissionManagerHelperTest(unittest.TestCase):
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

    @staticmethod
    def _stamped_message(stamp: float):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda value=stamp: value),
            )
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
        )
        manager.sensor_buffer = SimpleNamespace(
            read_cloud_xyz=lambda *args, **kwargs: reads.append(True) or [(0.8, 0.0, 0.5)],
            camera_to_target_matrix=lambda *args, **kwargs: None,
        )
        manager._elapsed = lambda: clock[0]
        manager._log_structured = lambda payload: diagnostics.append(payload)
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

    def test_floor_transition_latch_survives_task_clear_and_only_reports_timeout(self):
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
        self.assertEqual(logged_payloads[0]["action"], "continue_until_min_floor_time")
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
        self.assertEqual(manager._terminal_state, FINISH)
        self.assertEqual(manager._terminal_outcome, "success")

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
        )

        manager._tick_return_home((-2.0, 0.0, 0.0))

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

    def test_stuck_near_frontier_blacklists_region_and_blackmarks(self):
        """终点附近卡住时既排除目标区域，也涂黑局部不可达终点。"""
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
        # frontier 在终点附近，距离 1.0m < 1.5m
        manager.current_frontier = SimpleNamespace(centroid=(1.0, 0.0))
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
        self.assertEqual(recorded_failures[0], (1.0, 0.0))
        self.assertEqual(len(black_marks), 1)

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

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None, logerr=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0), \
             patch.object(mission_manager_module, "odom_pose_xy_yaw", return_value=(0.0, 0.0, 0.0)), \
             patch.object(mission_manager_module, "odom_xyz", return_value=(0.0, 0.0, 0.0)):
            manager.tick()

        self.assertIsNone(manager.main_entrance_xy)
        self.assertIsNone(manager.main_entrance_pass_vector)
        self.assertIn("main_entrance", manager.failed_doors)
        self.assertIsNone(manager.current_task)

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

        with patch.object(mission_manager_module, "rospy", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None, logerr=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0), \
             patch.object(mission_manager_module, "odom_pose_xy_yaw", return_value=(0.0, 0.0, 0.0)), \
             patch.object(mission_manager_module, "odom_xyz", return_value=(0.0, 0.0, 0.0)):
            manager.tick()

        self.assertEqual(manager.main_entrance_xy, (0.0, 0.0))
        self.assertEqual(manager.main_entrance_pass_vector, (0.0, 1.0))
        self.assertIn("main_entrance", manager.opened_doors)
        self.assertIsNone(manager.current_task)

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
            clear_path=lambda: cleared.append(True),
        )

        with patch("danger_search.mission_manager.astar_plan", return_value=PlanResult(status=BUDGET_EXHAUSTED)), \
             patch.object(mission_manager_module, "rospy", SimpleNamespace(logwarn=lambda *a, **k: None)):
            manager._tick_return_home((0.0, 0.0, 0.0))

        # 预算耗尽：不设路径，清空当前路径等待重试。
        self.assertEqual(recorded_paths, [])
        self.assertEqual(cleared, [True])


class TimeBudgetDecisionTest(unittest.TestCase):
    """方案 C：全局时间预算与楼层/返航决策单元测试。"""

    def _make_manager(self, **overrides) -> MissionManager:
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.max_floor = overrides.get("max_floor", 1)
        manager.max_runtime = overrides.get("max_runtime", 120.0)
        manager.floor_timeout = overrides.get("floor_timeout", 40.0)
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

    def test_floor_timeout_single_floor_returns_home(self):
        """单层场景下楼超时后应返航。"""
        manager = self._make_manager(max_floor=1, max_runtime=120.0, floor_timeout=40.0)
        action, reason = manager._decide_mission_action(
            (3.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))],
            floor_elapsed=45.0,
            mission_elapsed=50.0,
        )
        self.assertEqual(action, "RETURN_HOME")
        self.assertEqual(reason, "floor_timeout_return_home")

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

    def test_all_inside_confirmation_increments_once_after_full_scan(self):
        """完整扫描结束后仍全部过近时，只增加一次 confirmation 且不记录 failure。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        frontier = Frontier(
            cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0,
        )
        manager = self._make_explore_manager(frontiers_list=[frontier])
        manager.no_frontier_scan_duration = 8.0
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

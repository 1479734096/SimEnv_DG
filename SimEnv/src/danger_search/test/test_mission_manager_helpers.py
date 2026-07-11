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
    nearest_forward_obstacle_distance,
)
from danger_search.mission_manager import MissionManager
from danger_search.mission_manager import probe_target_for_attempt
from danger_search.navigation_tasks import OpenDoorAndPass, FAILED
from danger_search.navigator import STUCK
from danger_search.planner import PlanResult, BUDGET_EXHAUSTED


class MissionManagerHelperTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()

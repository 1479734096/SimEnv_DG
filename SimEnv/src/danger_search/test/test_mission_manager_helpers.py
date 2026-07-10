import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.local_semantic_detector import DoorCandidate
from danger_search.mapper import MultiFloorGridMap
import danger_search.mission_manager as mission_manager_module
from danger_search.mission_manager import EXPLORE_FLOOR
from danger_search.mission_manager import MissionManager
from danger_search.mission_manager import probe_target_for_attempt
from danger_search.navigation_tasks import OpenDoorAndPass, FAILED
from danger_search.navigator import STUCK
from danger_search.planner import PlanResult, BUDGET_EXHAUSTED


class MissionManagerHelperTest(unittest.TestCase):
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

    def test_stuck_far_from_frontier_records_failure_without_blackmark(self):
        """远处卡住时仍累计 frontier 失败，但不涂黑目标栅格。"""
        recorded_failures: list[tuple[float, float]] = []
        black_marks: list[tuple] = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.manual_mode = False
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
            record_failure=lambda g, xy: recorded_failures.append(xy),
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

    def test_stuck_near_frontier_records_failure_and_blackmarks(self):
        """终点附近卡住时既累计失败也涂黑目标栅格。"""
        recorded_failures: list[tuple[float, float]] = []
        black_marks: list[tuple] = []
        manager = MissionManager.__new__(MissionManager)
        manager.current_floor = 0
        manager.manual_mode = False
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
            record_failure=lambda g, xy: recorded_failures.append(xy),
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


if __name__ == "__main__":
    unittest.main()

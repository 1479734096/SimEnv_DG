import math
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import danger_search.navigation_tasks as navigation_tasks_module

if navigation_tasks_module.rospy is None:
    navigation_tasks_module.rospy = MagicMock()

from danger_search.navigation_tasks import (
    DONE,
    FAILED,
    IDLE,
    NavigateToPose,
    OpenDoorAndPass,
    RUNNING,
    TakeElevator,
)
from danger_search.mapper import BLACK, FREE, MultiFloorGridMap
from danger_search.elevator_crossing_runtime import (
    ElevatorCrossingGeometry,
    ElevatorCrossingResult,
)
from danger_search.navigation_context import NavigationTaskContext
from danger_search.navigator import STUCK, ControlResult
from danger_search.planner import BUDGET_EXHAUSTED, PlanResult, UNREACHABLE


def _task_context(source) -> NavigationTaskContext:
    """把测试场景的显式依赖冻结为生产同款导航上下文。"""
    controller = source.controller

    def _recover_with_clearance(reason):
        recover = getattr(source, "_recover_with_rotation_clearance", None)
        if callable(recover):
            return bool(recover(reason))
        fallback = getattr(controller, "recover", None)
        if callable(fallback):
            fallback()
            return True
        return False

    def _confirm_floor(target_floor):
        confirm = getattr(source, "confirm_floor_transition", None)
        if callable(confirm):
            confirm(target_floor)
        else:
            source.current_floor = int(target_floor)

    return NavigationTaskContext(
        controller=controller,
        mapper=getattr(source, "mapper", SimpleNamespace(floor=lambda _floor: object())),
        building_client=getattr(source, "building_client", object()),
            current_floor=int(getattr(source, "current_floor", 0)),
            planning_constraints=None,
        door_pass_max_linear=getattr(source, "door_pass_max_linear", None),
        open_door_geometry_refresh_enabled=hasattr(
            source, "refresh_open_door_geometry"
        ),
        elevator_geometry_refresh_enabled=hasattr(
            source, "refresh_elevator_geometry"
        ),
        elevator_geometry_refresh_standoff=float(
            getattr(source, "elevator_geometry_refresh_standoff", 1.2)
        ),
        elevator_opening_confirmation_timeout=float(
            getattr(source, "elevator_opening_confirmation_timeout", 8.0)
        ),
        elevator_local_crossing_geometry_enabled=bool(
            getattr(source, "elevator_local_crossing_geometry_enabled", False)
        ),
        elevator_local_crossing_timeout=float(
            getattr(source, "elevator_local_crossing_timeout", 4.0)
        ),
        elevator_local_crossing_max_center_correction=float(
            getattr(source, "elevator_local_crossing_max_center_correction", 0.8)
        ),
        elevator_stuck_retry_limit=int(
            getattr(source, "elevator_stuck_retry_limit", 4)
        ),
        exit_first_max_linear=getattr(source, "exit_first_max_linear", None),
        exit_cabin_backoff_enabled=bool(
            getattr(source, "exit_cabin_backoff_enabled", False)
        ),
        clear_opened_door_area_fn=getattr(
            source, "clear_opened_door_area", lambda _door_id: None
        ),
        clear_elevator_crossing_area_fn=getattr(
            source, "clear_elevator_crossing_area", lambda _geometry: None
        ),
        refresh_open_door_geometry_fn=getattr(
            source, "refresh_open_door_geometry", lambda _door_id, _pose: DONE
        ),
        refresh_elevator_geometry_fn=getattr(
            source,
            "refresh_elevator_geometry",
            lambda _elevator_id, _floor, _door_id, _pose: IDLE,
        ),
        refresh_elevator_crossing_geometry_fn=getattr(
            source, "refresh_elevator_crossing_geometry", lambda **_kwargs: None
        ),
        elevator_crossing_obstacle_fn=getattr(
            source, "elevator_crossing_obstacle", lambda *_args, **_kwargs: None
        ),
        record_failure_fn=getattr(source, "_record_failure", lambda _reason: None),
        log_structured_fn=getattr(source, "_log_structured", lambda _payload: None),
        begin_floor_transition_fn=getattr(
            source, "begin_floor_transition", lambda **_kwargs: None
        ),
        confirm_floor_transition_fn=_confirm_floor,
        begin_door_open_confirmation_fn=getattr(
            source,
            "begin_elevator_door_open_confirmation",
            lambda _door_id: None,
        ),
        confirm_door_open_fn=getattr(
            source, "confirm_elevator_door_open", lambda _door_id, _pose: True
        ),
        finish_door_open_confirmation_fn=getattr(
            source,
            "finish_elevator_door_open_confirmation",
            lambda _door_id: None,
        ),
        elevator_visual_preparation_pending_fn=getattr(
            source,
            "elevator_visual_preparation_pending",
            lambda _door_id: False,
        ),
        recover_with_rotation_clearance_fn=_recover_with_clearance,
        elevator_start_retreat_target_fn=getattr(
            source,
            "elevator_start_retreat_target",
            lambda _pose, _floor: None,
        ),
            elevator_approach_trace_candidates_fn=getattr(
            source,
            "elevator_approach_trace_candidates",
                lambda _elevator_id, _floor, **_kwargs: (),
            ),
            plan_path_fn=lambda *args, **kwargs: navigation_tasks_module.astar_plan(
                *args, **kwargs
            ),
            segment_is_traversable_fn=lambda grid, start, end: not navigation_tasks_module._segment_crosses_blocked_cell(
                grid, start, end
            ),
            elevator_async_target_door_close_enabled=bool(
                getattr(source, "elevator_async_target_door_close_enabled", True)
            ),
            relocalize_landmark_fn=getattr(source, "relocalize_landmark", None),
            latest_cloud_stamp_fn=getattr(source, "latest_cloud_stamp", None),
            elevator_opening_confirmation_retry_limit=int(
                getattr(source, "elevator_opening_confirmation_retry_limit", 1)
            ),
            elevator_opening_confirmation_retry_timeout=float(
                getattr(source, "elevator_opening_confirmation_retry_timeout", 10.0)
            ),
            elevator_opening_confirmation_sensor_refresh=float(
                getattr(source, "elevator_opening_confirmation_sensor_refresh", 0.5)
            ),
        )


class NavigateToPoseTest(unittest.TestCase):
    def test_invalid_task_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            NavigateToPose((math.nan, 0.0))
        with self.assertRaises(ValueError):
            OpenDoorAndPass("")
        with self.assertRaises(ValueError):
            TakeElevator(0, 0)

    @staticmethod
    def _manager():
        stopped = []
        paths = []
        controller = SimpleNamespace(
            goal_tolerance=0.1,
            has_path=lambda: False,
            set_path=lambda path, **kwargs: paths.append(path),
            stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            poll_status=lambda: RUNNING,
                set_tracking_limits=lambda **kwargs: None,
        )
        return (
            SimpleNamespace(
                controller=controller,
                mapper=SimpleNamespace(floor=lambda _floor: object()),
                current_floor=0,
            ),
            stopped,
            paths,
        )

    def test_unreachable_target_fails_without_direct_path(self):
        manager, stopped, paths = self._manager()
        task = NavigateToPose((1.0, 0.0))

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=UNREACHABLE),
        ):
            status = task.tick(_task_context(manager), (0.0, 0.0, 0.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(paths, [])
        self.assertEqual(stopped, ["navigation_target_unreachable"])

    def test_budget_exhaustion_waits_without_direct_path(self):
        manager, stopped, paths = self._manager()
        task = NavigateToPose((1.0, 0.0))

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=BUDGET_EXHAUSTED),
        ):
            status = task.tick(_task_context(manager), (0.0, 0.0, 0.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(paths, [])
        self.assertEqual(stopped, ["navigation_budget_exhausted"])


class _DoneService:
    def done(self):
        return True

    def ok(self):
        return True

    def message(self):
        return ""


class _PendingService:
    def done(self):
        return False


class _FailedService:
    def __init__(self, message="failed"):
        self._message = message

    def done(self):
        return True

    def ok(self):
        return False

    def message(self):
        return self._message


class OpenDoorAndPassTest(unittest.TestCase):
    def test_open_door_success_clears_dynamic_obstacle_area(self):
        cleared = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "CALL_OPEN"
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_door_path_points=lambda *args, **kwargs: ((0.0, -0.8), (0.0, 0.8), (0.0, 1.8)),
                open_door=lambda door_id: _DoneService(),
            ),
            clear_opened_door_area=lambda door_id: cleared.append(door_id),
            controller=SimpleNamespace(stop=lambda: None),
        )

        self.assertEqual(task.tick(_task_context(manager), (0.0, -0.8, math.pi / 2.0)), RUNNING)
        self.assertEqual(task.tick(_task_context(manager), (0.0, -0.8, math.pi / 2.0)), RUNNING)

        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(cleared, ["main_entrance"])

    def test_open_door_refreshes_near_field_geometry_before_crossing(self):
        stopped = []
        cleared = []
        path_queries = []
        refresh_results = iter((RUNNING, DONE))

        def path_points(*args, **kwargs):
            path_queries.append(True)
            if len(path_queries) == 1:
                return ((0.0, -0.8), (0.0, 0.8), (0.0, 1.8))
            return ((0.5, -0.8), (0.5, 0.8), (0.5, 1.8))

        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "CALL_OPEN"
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_door_path_points=path_points,
                open_door=lambda door_id: _DoneService(),
            ),
            refresh_open_door_geometry=lambda door_id, pose: next(refresh_results),
            clear_opened_door_area=lambda door_id: cleared.append(door_id),
            controller=SimpleNamespace(
                goal_tolerance=0.25,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
                clear_path=lambda **kwargs: cleared.append(kwargs.get("reason")),
            ),
        )
        pose = (0.0, -0.8, math.pi / 2.0)

        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.phase, "GO_OPEN_WAIT_POINT")
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.phase, "REFRESH_OPENING")
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)

        self.assertEqual(task.phase, "GO_REFRESHED_WAIT_POINT")
        self.assertEqual(task.wait_point, (0.5, -0.8))
        self.assertEqual(task.pass_point, (0.5, 0.8))
        self.assertEqual(stopped, ["door_opening_refresh"])
        self.assertIn("door_opening_geometry_refreshed", cleared)

    def test_open_door_refreshes_geometry_and_skips_wait_point_when_close(self):
        stopped = []
        cleared = []
        path_queries = []
        refresh_results = iter((RUNNING, DONE))

        def path_points(*args, **kwargs):
            path_queries.append(True)
            if len(path_queries) == 1:
                return ((0.0, -0.8), (0.0, 0.8), (0.0, 1.8))
            return ((0.1, -0.8), (0.1, 0.8), (0.1, 1.8))

        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "CALL_OPEN"
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_door_path_points=path_points,
                open_door=lambda door_id: _DoneService(),
            ),
            refresh_open_door_geometry=lambda door_id, pose: next(refresh_results),
            clear_opened_door_area=lambda door_id: cleared.append(door_id),
            controller=SimpleNamespace(
                goal_tolerance=0.25,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
                clear_path=lambda **kwargs: cleared.append(kwargs.get("reason")),
            ),
        )
        pose = (0.0, -0.8, math.pi / 2.0)

        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.phase, "GO_OPEN_WAIT_POINT")
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.phase, "REFRESH_OPENING")
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)

        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(task.wait_point, (0.1, -0.8))
        self.assertEqual(task.pass_point, (0.1, 0.8))

    def test_open_door_service_failure_returns_failed(self):
        failures = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "CALL_OPEN"
        task._service_task = _FailedService("denied")
        task._retry_count = 1
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_door_path_points=lambda *args, **kwargs: ((0.0, -0.8), (0.0, 0.8), (0.0, 1.8)),
                open_door=lambda door_id: _FailedService("denied"),
            ),
            controller=SimpleNamespace(stop=lambda: None),
            _record_failure=lambda reason: failures.append(reason),
        )

        status = task.tick(_task_context(manager), (0.0, -0.8, math.pi / 2.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(failures, ["开门服务失败: denied"])

    def test_regular_door_keeps_original_pass_flow_when_refresh_hook_exists(self):
        task = OpenDoorAndPass("room_door_1", pass_offset=0.8, commit_distance=1.8)
        task.phase = "CALL_OPEN"
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_door_path_points=lambda *args, **kwargs: (
                    (0.0, -0.8),
                    (0.0, 0.8),
                    (0.0, 1.8),
                ),
                open_door=lambda door_id: _DoneService(),
            ),
            refresh_open_door_geometry=lambda door_id, pose: self.fail(
                "普通门不应进入主入口近场重观测"
            ),
            clear_opened_door_area=lambda door_id: None,
            controller=SimpleNamespace(stop=lambda **kwargs: None),
        )

        pose = (0.0, -0.8, math.pi / 2.0)
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
        self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)

        self.assertEqual(task.phase, "GO_PASS_POINT")

    def test_wait_point_blocked_opens_from_same_side_safe_approach(self):
        """门区膨胀隔离精确等待点时，安全同侧接近不得永久零速。"""
        stopped = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        grid = mapper.floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.0, -1.4))
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_door_path_points=lambda *args, **kwargs: (
                    (0.0, -0.8),
                    (0.0, 0.8),
                    (0.0, 1.8),
                ),
            ),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                has_path=lambda: False,
                set_path=lambda path, **kwargs: None,
                poll_status=lambda: RUNNING,
                recover=lambda: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            ),
        )

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=UNREACHABLE),
        ) as astar:
            status = task.tick(_task_context(manager), (0.0, -2.3, math.pi / 2.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "CALL_OPEN")
        self.assertEqual(stopped, ["door_wait_point_blocked_safe_open"])
        self.assertIsNone(astar.call_args.kwargs["max_iterations"])

    def test_wait_point_blocked_outside_safe_approach_fails_bounded(self):
        stopped = []
        failures = []
        cleared = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=10.0,
            length=10.0,
            origin_x=-5.0,
            origin_y=-5.0,
        )
        grid = mapper.floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.0, -1.4))
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_door_path_points=lambda *args, **kwargs: (
                    (0.0, -0.8),
                    (0.0, 0.8),
                    (0.0, 1.8),
                ),
            ),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                has_path=lambda: False,
                set_path=lambda path, **kwargs: None,
                poll_status=lambda: RUNNING,
                recover=lambda: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
                clear_path=lambda **kwargs: cleared.append(kwargs.get("reason")),
            ),
            _record_failure=lambda reason: failures.append(reason),
        )

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=UNREACHABLE),
        ), patch("danger_search.navigation_tasks.get_time", side_effect=[10.0, 13.1]):
            first = task.tick(_task_context(manager), (0.0, -4.0, math.pi / 2.0))
            second = task.tick(_task_context(manager), (0.0, -4.0, math.pi / 2.0))

        self.assertEqual(first, RUNNING)
        self.assertEqual(second, FAILED)
        self.assertEqual(task.phase, "GO_WAIT_POINT")
        self.assertEqual(stopped, ["door_wait_point_blocked", "door_wait_point_blocked"])
        self.assertEqual(cleared, ["door_crossing_failed"])
        self.assertEqual(failures, ["开门等待点持续不可达"])

    def test_wait_point_blocked_lateral_to_door_axis_does_not_open(self):
        """同侧但横向偏离门洞的机器人不得从侧面直接斜穿门墙。"""
        stopped = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        ).floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.6, -1.55))
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_door_path_points=lambda *args, **kwargs: (
                    (0.0, -0.8),
                    (0.0, 0.8),
                    (0.0, 1.8),
                ),
            ),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                has_path=lambda: False,
                set_path=lambda path, **kwargs: None,
                poll_status=lambda: RUNNING,
                recover=lambda: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            ),
        )

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=UNREACHABLE),
        ), patch("danger_search.navigation_tasks.get_time", return_value=10.0):
            status = task.tick(_task_context(manager), (1.2, -2.3, math.pi / 2.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "GO_WAIT_POINT")
        self.assertEqual(stopped, ["door_wait_point_blocked"])

    def test_commit_point_blocked_cleans_and_keeps_running(self):
        recorded_paths = []
        cleared_paths = []
        cleared_doors = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_COMMIT_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)

        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        grid = mapper.floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.0, 1.2))
        manager = SimpleNamespace(
            building_client=SimpleNamespace(),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                has_path=lambda: False,
                set_path=lambda path, **kwargs: recorded_paths.append(path),
                clear_path=lambda: cleared_paths.append(True),
                poll_status=lambda: RUNNING,
                recover=lambda: None,
            ),
            clear_opened_door_area=lambda door_id: cleared_doors.append(door_id),
        )

        with patch("danger_search.navigation_tasks.astar_plan", return_value=PlanResult(status=UNREACHABLE)):
            status = task.tick(_task_context(manager), (0.0, 0.8, math.pi / 2.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(recorded_paths, [])
        self.assertEqual(cleared_paths, [True])
        self.assertEqual(cleared_doors, ["main_entrance"])

    def test_pass_point_aligns_then_uses_direct_path_without_astar(self):
        """门洞通过阶段应稳定对准后直连真实 pass_point，不接受 A* 替代终点。"""
        recorded_paths = []
        stopped = []
        cleared_doors = []
        path_active = [False]
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)

        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        grid = mapper.floor(0)

        def set_path(path, **kwargs):
            recorded_paths.append(path)
            path_active[0] = True

        manager = SimpleNamespace(
            building_client=SimpleNamespace(),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                max_angular=1.2,
                recover_angular_speed=0.55,
                has_path=lambda: path_active[0],
                set_path=set_path,
                clear_path=lambda **kwargs: path_active.__setitem__(0, False),
                poll_status=lambda: RUNNING,
                recover=lambda: None,
                rotate_in_place=lambda angular: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            ),
            clear_opened_door_area=lambda door_id: cleared_doors.append(door_id),
        )
        pose = (0.2, -1.1, math.atan2(1.9, -0.2))

        with patch("danger_search.navigation_tasks.astar_plan") as astar:
            first = task.tick(_task_context(manager), pose)
            second = task.tick(_task_context(manager), pose)

        self.assertEqual(first, RUNNING)
        self.assertEqual(second, RUNNING)
        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(len(recorded_paths), 1)
        self.assertEqual(recorded_paths[0][0], (0.2, -1.1))
        self.assertEqual(recorded_paths[0][-1], (0.0, 0.8))
        self.assertEqual(cleared_doors, ["main_entrance"])
        self.assertEqual(
            stopped,
            ["door_pass_alignment_hold", "door_pass_alignment_hold"],
        )
        astar.assert_not_called()

    def test_active_pass_applies_dedicated_linear_cap(self):
        observed_limits = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)
        task._pass_path_started = True
        manager = SimpleNamespace(
            door_pass_max_linear=0.25,
            building_client=SimpleNamespace(),
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                has_path=lambda: True,
                set_tracking_limits=lambda **kw: observed_limits.append(kw),
                poll_status=lambda: RUNNING,
            ),
        )

        self.assertEqual(
            task.tick(_task_context(manager), (0.0, -0.6, math.pi / 2.0)),
            RUNNING,
        )

        self.assertEqual(observed_limits[0]["max_linear"], 0.25)
        self.assertEqual(observed_limits[0]["goal_tolerance"], 0.15)

    def test_opening_sensor_confirmation_is_explicit_and_latched(self):
        task = OpenDoorAndPass("main_entrance")

        self.assertFalse(task.opening_sensor_confirmed)

        task.mark_opening_sensor_confirmed()

        self.assertTrue(task.opening_sensor_confirmed)

    def test_pass_point_blocked_and_line_blocked_fails_bounded_without_recover(self):
        """GO_PASS_POINT 持续阻塞应明确失败，不能由通用 recover 改变过门方向。"""
        recovered = []
        stopped = []
        failures = []
        cleared_paths = []
        cleared_doors = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)

        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        grid = mapper.floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.0, 0.0))
        manager = SimpleNamespace(
            building_client=SimpleNamespace(),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                max_angular=1.2,
                recover_angular_speed=0.55,
                has_path=lambda: False,
                set_path=lambda path, **kwargs: None,
                clear_path=lambda **kwargs: cleared_paths.append(kwargs.get("reason")),
                poll_status=lambda: RUNNING,
                recover=lambda: recovered.append(True),
                rotate_in_place=lambda angular: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            ),
            clear_opened_door_area=lambda door_id: cleared_doors.append(door_id),
            _record_failure=lambda reason: failures.append(reason),
        )

        pose = (0.2, -1.1, math.atan2(1.9, -0.2))
        with patch("danger_search.navigation_tasks.get_time", side_effect=[10.0, 13.1]):
            align = task.tick(_task_context(manager), pose)
            first = task.tick(_task_context(manager), pose)
            second = task.tick(_task_context(manager), pose)

        self.assertEqual(align, RUNNING)
        self.assertEqual(first, RUNNING)
        self.assertEqual(second, FAILED)
        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(recovered, [])
        self.assertEqual(
            stopped,
            [
                "door_pass_alignment_hold",
                "door_pass_alignment_hold",
                "door_pass_blocked",
                "door_pass_alignment_hold",
                "door_pass_blocked",
            ],
        )
        self.assertEqual(cleared_doors, ["main_entrance", "main_entrance"])
        self.assertEqual(cleared_paths, ["door_crossing_failed"])
        self.assertEqual(failures, ["过门通过点持续被地图阻塞"])

    def test_pass_point_stuck_fails_without_generic_recover(self):
        recovered = []
        failures = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)
        task._pass_path_started = True
        manager = SimpleNamespace(
            building_client=SimpleNamespace(),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                has_path=lambda: True,
                set_tracking_limits=lambda **kwargs: None,
                poll_status=lambda: STUCK,
                recover=lambda: recovered.append(True),
                clear_path=lambda **kwargs: None,
            ),
            _record_failure=lambda reason: failures.append(reason),
        )

        status = task.tick(_task_context(manager), (0.0, -0.8, math.pi / 2.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(recovered, [])
        self.assertEqual(failures, ["过门通过阶段有命令但无有效位移"])

    def test_pass_point_does_not_accept_controller_done_outside_crossing_tolerance(self):
        failures = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)
        task._pass_path_started = True
        manager = SimpleNamespace(
            building_client=SimpleNamespace(),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                has_path=lambda: True,
                set_tracking_limits=lambda **kwargs: None,
                poll_status=lambda: DONE,
                clear_path=lambda **kwargs: None,
            ),
            _record_failure=lambda reason: failures.append(reason),
        )

        status = task.tick(_task_context(manager), (0.0, 0.61, math.pi / 2.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(failures, ["控制器提前结束过门路径"])

    def test_pass_point_done_uses_latest_controller_pose_when_task_pose_is_stale(self):
        failures = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)
        task._pass_path_started = True
        manager = SimpleNamespace(
            building_client=SimpleNamespace(),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                has_path=lambda: True,
                set_tracking_limits=lambda **kwargs: None,
                poll_status=lambda: DONE,
                tracking_debug_snapshot=lambda: {
                    "pose": (0.0, 0.66, math.pi / 2.0),
                },
                clear_path=lambda **kwargs: None,
            ),
            _record_failure=lambda reason: failures.append(reason),
        )

        status = task.tick(_task_context(manager), (0.0, 0.61, math.pi / 2.0))

        self.assertEqual(status, DONE)
        self.assertEqual(failures, [])

    def test_pass_path_commits_crossing_tolerance_before_controller_can_finish(self):
        recorded = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)
        task._pass_alignment_done = True

        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        manager = SimpleNamespace(
            mapper=SimpleNamespace(floor=lambda _floor: mapper.floor(0)),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                has_path=lambda: False,
                set_path=lambda path, **kwargs: recorded.append((path, kwargs)),
                set_tracking_limits=lambda **kwargs: None,
                poll_status=lambda: RUNNING,
                clear_path=lambda **kwargs: None,
            ),
            _record_failure=lambda _reason: None,
        )

        status = task.tick(_task_context(manager), (0.0, -0.8, math.pi / 2.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(len(recorded), 1)
        self.assertEqual(recorded[0][1]["goal_tolerance"], 0.15)

    def test_pass_alignment_uses_controller_align_mode_not_task_layer_rotation(self):
        """门洞细对准应交给固定频率 ALIGN 模式，任务层不再自转。

        任务层以 10Hz 直接发 rotate_in_place 压不住 A1 转向惯性：最小可执行
        转速 0.55 rad/s 会反复冲出 ±0.10 rad 对准窗口，确认数永远攒不满。
        """
        align_calls = []
        recorded_paths = []
        rotations = []
        cleared_doors = []
        path_active = [False]
        align_status = [IDLE, RUNNING, DONE]

        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)

        grid = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        ).floor(0)

        def poll_result():
            status = align_status.pop(0) if align_status else RUNNING
            reason = "alignment_done" if status == DONE else None
            return ControlResult(status=status, reason=reason, sequence=1)

        def set_path(path, **kwargs):
            recorded_paths.append(list(path))
            path_active[0] = True

        manager = SimpleNamespace(
            building_client=SimpleNamespace(),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                max_angular=1.2,
                recover_angular_speed=0.55,
                has_path=lambda: path_active[0],
                set_path=set_path,
                clear_path=lambda **kwargs: path_active.__setitem__(0, False),
                poll_status=lambda: RUNNING,
                poll_result=poll_result,
                begin_heading_alignment=lambda target_xy, **kwargs: align_calls.append(
                    (tuple(target_xy), kwargs)
                ),
                rotate_in_place=lambda angular: rotations.append(angular),
                stop=lambda **kwargs: None,
            ),
            clear_opened_door_area=lambda door_id: cleared_doors.append(door_id),
        )

        # yaw=0 时朝向严重偏离通过点：旧实现会在这里持续自转。
        pose = (0.2, -1.1, 0.0)
        with patch("danger_search.navigation_tasks.astar_plan") as astar:
            first = task.tick(_task_context(manager), pose)
            second = task.tick(_task_context(manager), pose)
            third = task.tick(_task_context(manager), pose)

        self.assertEqual([first, second, third], [RUNNING, RUNNING, RUNNING])
        # IDLE 时进入 ALIGN 模式，并带位置漂移保护锚点。
        self.assertEqual(len(align_calls), 1)
        self.assertEqual(align_calls[0][0], (0.0, 0.8))
        self.assertEqual(align_calls[0][1]["anchor_xy"], (0.2, -1.1))
        self.assertEqual(
            align_calls[0][1]["max_anchor_error"],
            task._PASS_ALIGN_ANCHOR_RADIUS,
        )
        # 任务层不得再自转。
        self.assertEqual(rotations, [])
        # 控制器确认对准后才提交直线路径。
        self.assertEqual(len(recorded_paths), 1)
        self.assertEqual(recorded_paths[0][0], (0.2, -1.1))
        self.assertEqual(recorded_paths[0][-1], (0.0, 0.8))
        self.assertTrue(task._pass_path_started)
        self.assertEqual(cleared_doors, ["main_entrance"])
        astar.assert_not_called()

    def test_pass_alignment_not_converging_fails_on_timeout_instead_of_livelock(self):
        """细对准长期不收敛必须显式失败，不能无限返回 RUNNING。

        回归实测：主入口 GO_PASS_POINT 持续摆动 395s，STUCK/IDLE 判定与阻塞
        超时都在 `_pass_path_started` 为 False 时不可达，只能靠外部杀进程。
        """
        failures = []
        cleared_paths = []
        task = OpenDoorAndPass("main_entrance", pass_offset=0.8, commit_distance=1.8)
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)

        manager = SimpleNamespace(
            building_client=SimpleNamespace(),
            mapper=SimpleNamespace(floor=lambda floor_index: object()),
            current_floor=0,
            controller=SimpleNamespace(
                goal_tolerance=0.35,
                corner_tolerance=0.15,
                max_angular=1.2,
                recover_angular_speed=0.55,
                has_path=lambda: False,
                set_path=lambda path, **kwargs: None,
                clear_path=lambda **kwargs: cleared_paths.append(kwargs.get("reason")),
                poll_status=lambda: RUNNING,
                # 控制器始终未确认对准（settle/align/confirm 反复循环）。
                poll_result=lambda: ControlResult(
                    status=RUNNING, reason=None, sequence=1
                ),
                begin_heading_alignment=lambda target_xy, **kwargs: None,
                rotate_in_place=lambda angular: None,
                stop=lambda **kwargs: None,
            ),
            _record_failure=lambda reason: failures.append(reason),
        )

        pose = (0.2, -1.1, 0.0)
        # 首 tick 装填预算起点，第二 tick 超出 _PASS_ALIGN_TIMEOUT。
        with patch(
            "danger_search.navigation_tasks.get_time",
            side_effect=[100.0, 100.0 + task._PASS_ALIGN_TIMEOUT + 0.1],
        ):
            first = task.tick(_task_context(manager), pose)
            second = task.tick(_task_context(manager), pose)

        self.assertEqual(first, RUNNING)
        self.assertEqual(second, FAILED)
        self.assertEqual(failures, ["门洞细对准超时未收敛"])
        self.assertEqual(cleared_paths, ["door_crossing_failed"])
        self.assertFalse(task._pass_path_started)


class TakeElevatorTest(unittest.TestCase):
    @staticmethod
    def _exit_manager(grid):
        class Controller:
            goal_tolerance = 0.35
            corner_tolerance = 0.15
            recover_duration = 0.5
            recover_angular_speed = 0.55
            reverse_speed = -0.15
            max_angular = 1.2

            def __init__(self):
                self.stop_reasons = []
                self.clear_reasons = []
                self.turn_commands = []
                self.reverse_reasons = []
                self.recover_reasons = []
                self.paths = []
                self.path_active = False
                self.tick_calls = []
                self.limits_calls = []
                self.align_calls = []
                self._align_started = False
                self._align_started = False
                self.next_tick_status = RUNNING

            def stop(self, *, reason="stop"):
                self.stop_reasons.append(reason)

            def clear_path(self, *, reason="path_cleared"):
                self.clear_reasons.append(reason)
                self.path_active = False

            def set_path(self, path, **kwargs):
                self.paths.append(list(path))
                self.path_active = True

            def set_tracking_limits(self, **kwargs):
                self.limits_calls.append(kwargs)
                return None

            def poll_status(self):
                return getattr(self, "next_tick_status", RUNNING)

            def begin_heading_alignment(self, target_xy, **kwargs):
                self.align_calls.append((tuple(target_xy), kwargs))
                self._align_started = True

            def end_heading_alignment(self):
                return None

            def poll_result(self):
                status = getattr(self, "next_align_status", None)
                if status is not None:
                    self.next_align_status = None
                    return ControlResult(
                        status=status,
                        reason=getattr(self, "next_align_reason", None),
                        sequence=1,
                    )
                if not getattr(self, "_align_started", False):
                    return ControlResult(status=IDLE, reason=None, sequence=1)
                return ControlResult(status=RUNNING, reason=None, sequence=1)

            def has_path(self):
                return self.path_active

            def set_path(self, path, **kwargs):
                self.paths.append(list(path))
                self.path_active = True

            def tick(self, pose, **kwargs):
                self.tick_calls.append((pose, kwargs))
                return self.next_tick_status

            def rotate_in_place(self, angular_z):
                self.turn_commands.append(angular_z)

            def reverse_straight(self, *, reason):
                self.reverse_reasons.append(reason)
                return True

            def recover(self, *, reason="stuck"):
                self.recover_reasons.append(reason)

        controller = Controller()
        elevator_door_ids = {
            0: "lift_door_ground",
            1: "lift_door_upper",
        }
        closed_doors = []
        client = SimpleNamespace(
            get_elevator_points=lambda elevator_id, floor: (
                (0.85, 2.60),
                (2.60, 2.60),
            ),
            elevator_door_id=lambda floor: elevator_door_ids[int(floor)],
            close_door=lambda door_id: closed_doors.append(door_id),
            closed_doors=closed_doors,
        )
        return SimpleNamespace(
            building_client=client,
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=1,
            controller=controller,
        )

    def test_exit_stuck_backs_into_cabin_then_realigns_before_retry(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        manager.exit_cabin_backoff_enabled = True
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task.cabin_transit_stage = "CROSS"

        with patch.object(task, "_tick_cabin_transit", return_value=STUCK), patch(
            "danger_search.navigation_tasks.get_time", return_value=10.0
        ):
            self.assertEqual(task.tick(_task_context(manager), (1.37, 2.60, math.pi)), RUNNING)

        self.assertEqual(task.phase, "EXIT_BACKOFF_SETTLE")
        self.assertEqual(task._retry_count, 1)
        self.assertGreater(task._exit_backoff_target_xy[0], 1.37)

        with patch("danger_search.navigation_tasks.get_time", return_value=10.6):
            task.tick(_task_context(manager), (1.37, 2.60, math.pi))
        self.assertEqual(task.phase, "EXIT_BACKOFF_ALIGN")

        # 朝向与出口相反时只能原地对准，不能开始倒车。
        with patch("danger_search.navigation_tasks.get_time", return_value=10.7):
            task.tick(_task_context(manager), (1.37, 2.60, 0.0))
        self.assertEqual(task.phase, "EXIT_BACKOFF_ALIGN")
        self.assertTrue(manager.controller.turn_commands)
        self.assertEqual(manager.controller.reverse_reasons, [])

        with patch("danger_search.navigation_tasks.get_time", return_value=10.8):
            task.tick(_task_context(manager), (1.37, 2.60, math.pi))
        self.assertEqual(task.phase, "EXIT_BACKOFF_REVERSE")

        with patch("danger_search.navigation_tasks.get_time", return_value=10.9):
            task.tick(_task_context(manager), (1.37, 2.60, math.pi))
        self.assertEqual(manager.controller.reverse_reasons, ["exit_cabin_backoff"])

        with patch("danger_search.navigation_tasks.get_time", return_value=12.0):
            task.tick(_task_context(manager), (1.72, 2.60, math.pi))
        self.assertEqual(task.phase, "EXIT_BACKOFF_REALIGN")

        with patch("danger_search.navigation_tasks.get_time", return_value=12.1):
            task.tick(_task_context(manager), (1.72, 2.60, math.pi))
        self.assertEqual(task.phase, "EXIT_CABIN")
        self.assertIn("exit_backoff_complete", manager.controller.clear_reasons)

    def test_enter_wait_point_aligns_to_registered_cabin_before_opening(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)

        with patch.object(task, "_navigate_to", return_value=True):
            self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, math.pi / 2.0)), RUNNING)

        self.assertEqual(task.phase, "ALIGN_TO_CURRENT_DOOR")
        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, math.pi / 2.0)), RUNNING)
        self.assertTrue(manager.controller.align_calls)
        self.assertEqual(task.phase, "ALIGN_TO_CURRENT_DOOR")

        manager.controller.next_align_status = DONE
        manager.controller.next_align_reason = "alignment_done"
        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.phase, "CALL_ELEVATOR_TO_CURRENT_FLOOR")

    def test_confirmed_current_door_refreshes_live_crossing_before_enter(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.elevator_local_crossing_geometry_enabled = True
        geometry = ElevatorCrossingGeometry(
            door_id="lift_door_ground",
            mode="enter",
            door_center=(1.42, 2.60),
            outward_yaw=math.pi,
            width=1.20,
            wait_point=(0.62, 2.60),
            cabin_center=(2.37, 2.60),
            generation_count=3,
            old_center_shift=0.57,
        )
        manager.refresh_elevator_crossing_geometry = lambda **_kwargs: (
            ElevatorCrossingResult(DONE, geometry=geometry)
        )
        task = TakeElevator(0, 1)
        task.phase = "WAIT_CURRENT_DOOR_OPEN"

        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.phase, "REFRESH_CURRENT_OPEN_CROSSING")
        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.phase, "ENTER_CABIN")
        self.assertEqual(task._current_crossing_geometry, geometry)

    def test_current_crossing_translates_remembered_door_by_stand_pose_offset(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.elevator_local_crossing_geometry_enabled = True
        captured = {}

        def refresh(**kwargs):
            captured.update(kwargs)
            return ElevatorCrossingResult(RUNNING)

        manager.refresh_elevator_crossing_geometry = refresh
        task = TakeElevator(0, 1)
        task.phase = "REFRESH_CURRENT_OPEN_CROSSING"

        task.tick(_task_context(manager), (0.95, 2.70, 0.0))

        self.assertAlmostEqual(captured["old_door_center"][0], 1.65, places=2)
        self.assertAlmostEqual(captured["old_door_center"][1], 2.60, places=2)

    def test_target_live_cabin_center_offset_fails_closed(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        manager.current_floor = 1
        manager.elevator_local_crossing_geometry_enabled = True
        geometry = ElevatorCrossingGeometry(
            door_id="lift_door_upper",
            mode="exit",
            door_center=(1.42, 2.60),
            outward_yaw=math.pi,
            width=1.20,
            wait_point=(0.62, 2.60),
            cabin_center=(2.37, 2.60),
            generation_count=3,
            old_center_shift=0.57,
        )
        manager.refresh_elevator_crossing_geometry = lambda **_kwargs: (
            ElevatorCrossingResult(DONE, geometry=geometry)
        )
        task = TakeElevator(0, 1)
        task.phase = "REFRESH_TARGET_OPEN_CROSSING"

        status = task.tick(_task_context(manager), (3.30, 2.60, math.pi))

        self.assertEqual(status, FAILED)
        self.assertEqual(task.failure_reason, "elevator_cabin_center_offset_exceeded")

    def test_target_empty_geometry_early_fallback_keeps_exit_alignment(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        manager.current_floor = 1
        manager.elevator_local_crossing_geometry_enabled = True
        manager.refresh_elevator_crossing_geometry = lambda **_kwargs: (
            ElevatorCrossingResult(
                FAILED,
                failure="elevator_open_crossing_geometry_not_observed",
                diagnostic={
                    "event": "elevator_local_crossing_geometry_early_fallback"
                },
            )
        )
        task = TakeElevator(0, 1)
        task.phase = "REFRESH_TARGET_OPEN_CROSSING"
        cabin_pose = (2.60, 2.60, math.pi / 2.0)

        self.assertEqual(task.tick(_task_context(manager), cabin_pose), RUNNING)
        self.assertEqual(task.phase, "EXIT_CABIN")
        self.assertIsNone(task._target_crossing_geometry)

        self.assertEqual(task.tick(_task_context(manager), cabin_pose), RUNNING)
        self.assertEqual(task.cabin_transit_stage, "ALIGN")
        self.assertEqual(task.tick(_task_context(manager), cabin_pose), RUNNING)
        self.assertTrue(manager.controller.align_calls)
        self.assertEqual(manager.controller.paths, [])

    def test_live_crossing_corridor_block_is_bounded(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.elevator_local_crossing_timeout = 4.0
        manager.elevator_crossing_obstacle = lambda *_args, **_kwargs: object()
        task = TakeElevator(0, 1)
        task._cabin_transit_mode = "enter"
        task._current_crossing_geometry = object()

        with patch(
            "danger_search.navigation_tasks._segment_crosses_blocked_cell",
            return_value=False,
        ), patch("danger_search.navigation_tasks.get_time", side_effect=(10.0, 14.1)):
            first = task._commit_cabin_cross(
                _task_context(manager),
                (0.85, 2.60, 0.0),
                (0.85, 2.60),
                (2.60, 2.60),
                target_door_id="lift_door_ground",
                position_tolerance=0.15,
                max_linear=None,
            )
            second = task._commit_cabin_cross(
                _task_context(manager),
                (0.85, 2.60, 0.0),
                (0.85, 2.60),
                (2.60, 2.60),
                target_door_id="lift_door_ground",
                position_tolerance=0.15,
                max_linear=None,
            )

        self.assertEqual(first, RUNNING)
        self.assertEqual(second, STUCK)
        self.assertEqual(task.failure_reason, "elevator_crossing_corridor_blocked")

    def test_elevator_unreachable_planning_fails_after_bounded_retries(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.elevator_planning_retry_limit = 3
        task = TakeElevator(0, 1)

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=UNREACHABLE),
        ) as planner, patch(
            "danger_search.navigation_tasks.get_time",
            side_effect=(0.0, 0.6, 1.2),
        ):
            statuses = [task.tick(_task_context(manager), (4.0, 4.0, 0.0)) for _ in range(3)]

        self.assertEqual(statuses, [RUNNING, RUNNING, FAILED])
        self.assertEqual(planner.call_count, 3)
        self.assertEqual(task.failure_reason, "elevator_navigation_unreachable")
        self.assertIn("elevator_navigation_unreachable", manager.controller.stop_reasons)

    def test_elevator_budget_exhaustion_expands_budget_then_fails(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.elevator_planning_retry_limit = 3
        task = TakeElevator(0, 1)

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=BUDGET_EXHAUSTED),
        ) as planner, patch(
            "danger_search.navigation_tasks.get_time",
            side_effect=(0.0, 0.6, 1.2),
        ):
            statuses = [task.tick(_task_context(manager), (4.0, 4.0, 0.0)) for _ in range(3)]

        self.assertEqual(statuses, [RUNNING, RUNNING, FAILED])
        self.assertEqual(
            [call.kwargs["max_iterations"] for call in planner.call_args_list],
            [20000, 40000, 80000],
        )
        self.assertEqual(
            task.failure_reason, "elevator_navigation_budget_exhausted"
        )

    def test_elevator_uses_safe_trace_approach_after_precise_goal_unreachable(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.elevator_planning_retry_limit = 3
        manager.elevator_approach_trace_candidates = (
            lambda elevator_id, floor, max_candidates: [(1.2, 2.6)]
        )
        manager.controller.next_tick_status = DONE
        task = TakeElevator(0, 1)

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            side_effect=(
                PlanResult(status=UNREACHABLE),
                PlanResult(status=UNREACHABLE),
                PlanResult(status=UNREACHABLE),
                PlanResult(path=[(4.0, 4.0), (1.2, 2.6)], status="reached"),
            ),
        ), patch(
            "danger_search.navigation_tasks.get_time",
            side_effect=(0.0, 0.6, 1.2, 1.3),
        ):
            self.assertEqual(task.tick(_task_context(manager), (4.0, 4.0, 0.0)), RUNNING)
            self.assertEqual(task.tick(_task_context(manager), (4.0, 4.0, 0.0)), RUNNING)
            self.assertEqual(task.tick(_task_context(manager), (4.0, 4.0, 0.0)), RUNNING)
            self.assertEqual(task.phase, "GO_TO_ELEVATOR_TRACE_APPROACH")
            self.assertEqual(task.tick(_task_context(manager), (4.0, 4.0, 0.0)), RUNNING)

        self.assertEqual(task.phase, "ALIGN_TO_CURRENT_DOOR")
        self.assertIsNone(task.failure_reason)

    def test_elevator_reverses_once_from_occ_start_then_retries_standard_plan(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.elevator_planning_retry_limit = 3
        retreat_requests = []
        manager.elevator_start_retreat_target = (
            lambda pose, floor: retreat_requests.append((pose, floor)) or (4.0, 3.0)
        )
        events = []
        manager._log_structured = events.append
        manager.controller.next_tick_status = DONE
        task = TakeElevator(0, 1)

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            side_effect=(
                PlanResult(status=UNREACHABLE),
                PlanResult(status=UNREACHABLE),
                PlanResult(status=UNREACHABLE),
                PlanResult(path=[(4.0, 3.2), (0.85, 2.6)], status="reached"),
            ),
        ), patch(
            "danger_search.navigation_tasks.get_time",
            side_effect=(0.0, 0.6, 1.2, 1.2, 1.3, 2.0),
        ):
            for _ in range(3):
                self.assertEqual(
                    task.tick(_task_context(manager), (4.0, 4.0, math.pi / 2.0)), RUNNING
                )
            self.assertEqual(task.phase, "ELEVATOR_START_RETREAT")
            self.assertEqual(
                task.tick(_task_context(manager), (4.0, 4.0, math.pi / 2.0)), RUNNING
            )
            self.assertEqual(
                manager.controller.reverse_reasons,
                ["elevator_start_clearance_retreat"],
            )
            self.assertEqual(
                task.tick(_task_context(manager), (4.0, 3.2, math.pi / 2.0)), RUNNING
            )
            self.assertEqual(task.phase, "GO_TO_ELEVATOR_DOOR")
            self.assertEqual(
                task.tick(_task_context(manager), (4.0, 3.2, math.pi / 2.0)), RUNNING
            )

        self.assertEqual(len(retreat_requests), 1)
        self.assertEqual(task.phase, "ALIGN_TO_CURRENT_DOOR")
        self.assertIsNone(task.failure_reason)
        self.assertEqual(
            [event["event"] for event in events if "start_retreat" in event["event"]],
            ["elevator_start_retreat_started", "elevator_start_retreat_completed"],
        )

    def test_elevator_start_retreat_heading_divergence_fails_closed(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        events = []
        manager._log_structured = events.append
        task = TakeElevator(0, 1)
        task.phase = "ELEVATOR_START_RETREAT"
        task._start_retreat_target_xy = (3.0, 4.0)
        task._start_retreat_deadline = 20.0

        with patch("danger_search.navigation_tasks.get_time", return_value=10.0):
            status = task.tick(_task_context(manager), (4.0, 4.0, math.pi / 2.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(task.failure_reason, "elevator_start_retreat_failed")
        self.assertEqual(manager.controller.reverse_reasons, [])
        self.assertIn("elevator_start_retreat_failed", manager.controller.stop_reasons)
        self.assertEqual(events[0]["detail"], "reverse_heading_diverged")

    def test_elevator_stuck_recovery_is_bounded(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.elevator_stuck_retry_limit = 4
        manager.controller.next_tick_status = STUCK
        task = TakeElevator(0, 1)

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(
                path=[(4.0, 4.0), (0.85, 2.60)], status="reached"
            ),
        ), patch(
            "danger_search.navigation_tasks.get_time",
            side_effect=(0.0, 0.1, 0.2, 0.3, 0.4),
        ):
            statuses = [task.tick(_task_context(manager), (4.0, 4.0, 0.0)) for _ in range(5)]

        self.assertEqual(statuses[-1], FAILED)
        self.assertEqual(len(manager.controller.recover_reasons), 4)
        self.assertEqual(
            task.failure_reason, "elevator_navigation_stuck_exhausted"
        )

    def test_elevator_stuck_recovery_uses_rotation_clearance_gate(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.controller.next_tick_status = STUCK
        gate_sources = []
        manager._recover_with_rotation_clearance = gate_sources.append
        task = TakeElevator(0, 1)

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(
                path=[(4.0, 4.0), (0.85, 2.60)], status="reached"
            ),
        ), patch("danger_search.navigation_tasks.get_time", return_value=0.0):
            status = task.tick(_task_context(manager), (4.0, 4.0, 0.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(gate_sources, ["elevator_navigation_stuck"])
        self.assertEqual(manager.controller.recover_reasons, [])

    def test_elevator_refresh_phase_replans_to_updated_wait_point(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        refresh_calls = []
        manager.refresh_elevator_geometry = lambda *args: refresh_calls.append(args) or DONE
        task = TakeElevator(0, 1)
        task.phase = "REFRESH_CURRENT_ELEVATOR_GEOMETRY"

        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.6, 0.0)), RUNNING)
        self.assertEqual(task.phase, "GO_TO_REFRESHED_ELEVATOR_DOOR")
        self.assertEqual(len(refresh_calls), 1)
        self.assertIn("elevator_geometry_refreshed", manager.controller.clear_reasons)

        with patch.object(task, "_navigate_to", return_value=True):
            self.assertEqual(task.tick(_task_context(manager), (0.85, 2.6, 0.0)), RUNNING)
        self.assertEqual(task.phase, "ALIGN_TO_REFRESHED_ELEVATOR_DOOR")

    def test_elevator_refresh_timeout_preserves_registered_geometry(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.refresh_elevator_geometry = lambda *args: IDLE
        task = TakeElevator(0, 1)
        task.phase = "REFRESH_CURRENT_ELEVATOR_GEOMETRY"

        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.6, 0.0)), RUNNING)

        self.assertEqual(task.phase, "GO_TO_REFRESHED_ELEVATOR_DOOR")
        self.assertIsNone(task.failure_reason)
        self.assertIn(
            "elevator_geometry_refresh_unavailable",
            manager.controller.clear_reasons,
        )

        with patch.object(task, "_navigate_to", return_value=True):
            self.assertEqual(task.tick(_task_context(manager), (0.85, 2.6, 0.0)), RUNNING)
        self.assertEqual(task.phase, "ALIGN_TO_REFRESHED_ELEVATOR_DOOR")

    def test_elevator_refresh_viewpoint_keeps_camera_farther_from_door(self):
        viewpoint = TakeElevator._refresh_viewpoint(
            (0.85, 2.6), (2.6, 2.6), 1.2
        )
        self.assertAlmostEqual(viewpoint[0], 0.45)
        self.assertAlmostEqual(viewpoint[1], 2.6)

    def test_first_exit_attempt_passes_local_linear_cap_without_shared_override(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        manager.exit_first_max_linear = 0.25
        manager.exit_cabin_backoff_enabled = False
        observed_polls = []

        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task._cabin_transit_mode = "exit"
        task.cabin_transit_stage = "CROSS"
        task._cabin_transit_start_xy = (2.0, 2.60)
        task._cabin_transit_target_xy = (0.85, 2.60)
        manager.controller.path_active = True

        def poll_controller():
            observed_polls.append(True)
            return RUNNING

        manager.controller.poll_status = poll_controller

        self.assertEqual(task.tick(_task_context(manager), (2.0, 2.60, math.pi)), RUNNING)
        task._retry_count = 1
        self.assertEqual(task.tick(_task_context(manager), (2.0, 2.60, math.pi)), RUNNING)

        # 首次退出限速 0.25 与重试无 cap 均由 set_path 原子提交，CROSS 只轮询状态。
        self.assertEqual(observed_polls, [True, True])
        self.assertFalse(hasattr(manager.controller, "max_linear"))

    def test_unconfigured_exit_cap_keeps_legacy_controller_call(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        observed_polls = []
        manager.controller.poll_status = lambda: observed_polls.append(True) or RUNNING
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task._cabin_transit_mode = "exit"
        task.cabin_transit_stage = "CROSS"
        task._cabin_transit_start_xy = (2.0, 2.60)
        task._cabin_transit_target_xy = (0.85, 2.60)
        manager.controller.path_active = True

        self.assertEqual(task.tick(_task_context(manager), (2.0, 2.60, math.pi)), RUNNING)

        # 限制（goal_tolerance/max_linear）在 set_path 时原子提交，CROSS 只轮询状态。
        self.assertEqual(observed_polls, [True])

    def test_disabled_exit_backoff_fails_on_first_stuck_without_recovery(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        manager.exit_cabin_backoff_enabled = False
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"

        with patch.object(task, "_tick_cabin_transit", return_value=STUCK):
            status = task.tick(_task_context(manager), (1.37, 2.60, math.pi))

        self.assertEqual(status, FAILED)
        self.assertEqual(task.phase, "EXIT_CABIN")
        self.assertEqual(task._retry_count, 0)
        self.assertEqual(manager.controller.reverse_reasons, [])

    def test_active_opened_door_id_requires_confirmed_crossing(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task.cabin_transit_stage = "ALIGN"

        self.assertIsNone(task.active_opened_door_id(manager.building_client))

        task.cabin_transit_stage = "CROSS"
        self.assertIsNone(task.active_opened_door_id(manager.building_client))

        task._door_open_confirmed.add("lift_door_upper")
        self.assertEqual(
            task.active_opened_door_id(manager.building_client),
            "lift_door_upper",
        )

        task.phase = "DONE"
        self.assertIsNone(task.active_opened_door_id(manager.building_client))

    def test_exit_backoff_rejects_blocked_segment(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        grid.mark_occupied(*grid.world_to_grid(1.55, 2.60))
        manager = self._exit_manager(grid)
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"

        manager.exit_cabin_backoff_enabled = True
        task.cabin_transit_stage = "CROSS"
        with patch.object(task, "_tick_cabin_transit", return_value=STUCK), patch(
            "danger_search.navigation_tasks.get_time", return_value=10.0
        ):
            status = task.tick(_task_context(manager), (1.37, 2.60, math.pi))

        self.assertEqual(status, FAILED)
        self.assertEqual(task.phase, "EXIT_CABIN")
        self.assertEqual(manager.controller.reverse_reasons, [])

    def test_exit_backoff_no_motion_times_out_without_second_recovery(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = TakeElevator(0, 1)
        task.phase = "EXIT_BACKOFF_REVERSE"
        task._retry_count = 1
        task._exit_backoff_start_xy = (1.37, 2.60)
        task._exit_backoff_target_xy = (1.72, 2.60)
        task._exit_backoff_direction = (1.0, 0.0)
        task._exit_backoff_exit_yaw = math.pi
        task._exit_backoff_deadline = 13.0

        with patch("danger_search.navigation_tasks.get_time", return_value=13.1):
            status = task.tick(_task_context(manager), (1.37, 2.60, math.pi))

        self.assertEqual(status, FAILED)
        self.assertEqual(manager.controller.reverse_reasons, [])
        self.assertIn("exit_backoff_failed", manager.controller.stop_reasons)

    def test_exit_skips_cabin_center_position_and_starts_in_align(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        calls = []

        def navigate(manager_arg, pose, target_xy, **kwargs):
            calls.append((target_xy, kwargs))
            return DONE

        with patch.object(task, "_navigate_to_with_status", side_effect=navigate):
            status = task.tick(_task_context(manager), (2.32, 2.53, 0.0))

        self.assertEqual(status, RUNNING)
        # 离厢直接从 ALIGN 开始，跳过回轿厢中心的 POSITION 导航，避免额外 180°+180° 掉头
        self.assertEqual(task.cabin_transit_stage, "ALIGN")
        self.assertEqual(calls, [])

    def test_cabin_transit_position_align_does_not_oscillate_at_grid_snap_boundary(self):
        """栅格量化终点与精确 cabin center 完成语义不一致的回归测试。

        复现条件：resolution=0.1，机器人到精确 cabin center 距离 0.1583m（>0.15），
        到 A* 栅格量化终点距离 0.1255m（<=0.15）。修复前 POSITION<->ALIGN 每 tick
        翻转，永不进入 CROSS；修复后 POSITION 进入 ALIGN 后不得再次回到 POSITION。
        """
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        # 轿厢中心格及到目标等待点的整段路径全部标 FREE，确保 A* 可达且直线跨越无阻塞
        for gx in range(5, 35):
            for gy in range(20, 30):
                grid.set_base_state(gx, gy, FREE)

        cabin_center = (2.960, 2.304)
        target_wait = (0.85, 2.60)
        robot = (3.032, 2.445)
        # 锁定复现条件：精确中心误差 0.1583，栅格量化终点误差约 0.1257
        self.assertAlmostEqual(
            math.hypot(cabin_center[0] - robot[0], cabin_center[1] - robot[1]),
            0.1583,
            places=3,
        )
        self.assertAlmostEqual(
            math.hypot(2.95 - robot[0], 2.35 - robot[1]),
            0.1257,
            places=3,
        )

        class SnapController:
            """复现 PathFollower 的 goal-reached：以 path[-1]（栅格量化终点）为准。"""
            goal_tolerance = 0.35
            corner_tolerance = 0.15
            recover_angular_speed = 0.55
            max_angular = 1.2

            def __init__(self):
                self._path = []
                self.stop_reasons = []
                self.clear_reasons = []
                self.turn_commands = []
                self.paths = []
                self.limits_calls = []
                self.align_calls = []
                self._align_started = False

            def set_path(self, path, **kwargs):
                self.paths.append(list(path))
                self._path = list(path)

            def has_path(self):
                return bool(self._path)

            def set_tracking_limits(self, **kwargs):
                self.limits_calls.append(kwargs)
                return None

            def poll_status(self):
                return self.poll_result().status

            def poll_result(self):
                pose = getattr(self, "_test_pose", None)
                if pose is not None and self._path:
                    goal = self._path[-1]
                    d = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
                    eff = self.goal_tolerance
                    if d <= eff:
                        self.clear_path(reason="goal_reached")
                        return ControlResult(status=DONE, reason="goal_reached", sequence=1)
                status = getattr(self, "next_align_status", None)
                if status is not None:
                    self.next_align_status = None
                    return ControlResult(
                        status=status,
                        reason=getattr(self, "next_align_reason", None),
                        sequence=1,
                    )
                return ControlResult(status=RUNNING, reason=None, sequence=1)

            def begin_heading_alignment(self, target_xy, **kwargs):
                self.align_calls.append((tuple(target_xy), kwargs))
                self._align_started = True

            def end_heading_alignment(self):
                return None

            def clear_path(self, *, reason="path_cleared"):
                self.clear_reasons.append(reason)
                self._path = []

            def stop(self, *, reason="stop"):
                self.stop_reasons.append(reason)

            def rotate_in_place(self, angular_z):
                self.turn_commands.append(angular_z)

            def recover(self, *, reason="stuck"):
                pass

            def tick(self, pose, **kwargs):
                goal_tolerance = kwargs.get("goal_tolerance", self.goal_tolerance)
                if not self._path:
                    return RUNNING
                goal = self._path[-1]
                d = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
                eff = min(self.goal_tolerance, max(0.0, float(goal_tolerance)))
                if d <= eff:
                    self.clear_path(reason="goal_reached")
                    return DONE
                return RUNNING

        controller = SnapController()
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: (target_wait, cabin_center),
                elevator_door_id=lambda floor: "lift_door_1",
            ),
            mapper=SimpleNamespace(floor=lambda idx: grid),
            current_floor=1,
            controller=controller,
        )
        task = TakeElevator(0, 1)
        # 朝向存在偏角（>0.80rad），进入常规 ALIGN 流程验证 POSITION/ALIGN 一致性
        yaw = math.atan2(target_wait[1] - robot[1], target_wait[0] - robot[0]) + 0.81
        pose = (robot[0], robot[1], yaw)
        manager.controller._test_pose = pose

        manager.controller.next_align_status = DONE
        manager.controller.next_align_reason = "alignment_done"
        stages = []
        for _ in range(6):
            task._tick_cabin_transit(
                _task_context(manager),
                pose,
                mode="enter",
                position_xy=cabin_center,
                target_xy=target_wait,
                door_id="lift_door_1",
            )
            stages.append(task.cabin_transit_stage)

        # 首个 stage 应为 ALIGN：POSITION 用严格容差，A* 量化终点 0.1255<=0.15 到达
        self.assertEqual(stages[0], "ALIGN")
        # 修复后不得再次回到 POSITION
        self.assertNotIn("POSITION", stages[1:])
        # 翻转计数应保持 0
        self.assertEqual(task._cabin_transit_position_align_flips, 0)
        # 朝向已对准且直线无阻塞，应在 2 次确认后进入 CROSS
        self.assertIn("CROSS", stages)

    def test_cabin_transit_position_align_flip_limit_returns_stuck(self):
        """POSITION/ALIGN 反复翻转超过上限时必须明确失败，不形成永久零速循环。"""
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = TakeElevator(0, 1)
        task._cabin_transit_mode = "enter"

        cabin_center = (2.60, 2.60)
        target_wait = (0.85, 2.60)
        # 机器人远离 cabin_center，使 ALIGN 的精确漂移检查始终超 acceptance
        pose = (2.60, 3.40, 0.0)
        self.assertGreater(
            math.hypot(cabin_center[0] - pose[0], cabin_center[1] - pose[1]),
            0.15 + 0.5 * math.sqrt(2) * 0.1,
        )

        flip_limit = TakeElevator._CABIN_TRANSIT_POSITION_ALIGN_FLIP_LIMIT
        status = RUNNING
        # 强制 POSITION 始终立即 DONE，ALIGN 始终漂移退回（控制器锁存
        # STUCK+alignment_drift），模拟语义退化。
        manager.controller.poll_result = lambda: ControlResult(
            status=STUCK, reason="alignment_drift", sequence=1
        )
        with patch.object(task, "_navigate_to_with_status", return_value=DONE):
            for _ in range(2 * (flip_limit + 1)):
                status = task._tick_cabin_transit(
                    _task_context(manager),
                    pose,
                    mode="enter",
                    position_xy=cabin_center,
                    target_xy=target_wait,
                    door_id="lift_door_1",
                )
                if status == STUCK:
                    break

        self.assertEqual(status, STUCK)
        self.assertGreater(task._cabin_transit_position_align_flips, flip_limit)

    def test_exit_aligns_in_place_twice_then_commits_single_straight_path(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task._cabin_transit_mode = "exit"
        task.cabin_transit_stage = "ALIGN"

        self.assertEqual(task.tick(_task_context(manager), (2.60, 2.60, 0.0)), RUNNING)
        self.assertTrue(manager.controller.align_calls)
        self.assertEqual(manager.controller.paths, [])

        # CONFIRM 由固定频率控制器完成；任务层收到 alignment_done 后提交直线跨越。
        manager.controller.next_align_status = DONE
        manager.controller.next_align_reason = "alignment_done"
        self.assertEqual(task.tick(_task_context(manager), (2.60, 2.60, math.pi)), RUNNING)
        self.assertEqual(task.cabin_transit_stage, "CROSS")
        self.assertEqual(
            manager.controller.paths,
            [[(2.60, 2.60), (0.85, 2.60)]],
        )

    def test_alignment_reuses_recover_angular_speed_above_rl_dead_zone(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task._cabin_transit_mode = "enter"
        task.cabin_transit_stage = "ALIGN"

        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, -0.14)), RUNNING)

        # 执行死区补偿（0.55 最小转速）由 PathFollower ALIGN 模式负责，
        # 任务层通过 begin_heading_alignment 进入后只轮询结果。
        self.assertTrue(manager.controller.align_calls)

    def test_exit_does_not_finish_at_general_goal_tolerance_or_replan_mid_crossing(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task._cabin_transit_mode = "exit"
        task.cabin_transit_stage = "CROSS"
        task._cabin_transit_start_xy = (2.60, 2.60)
        task._cabin_transit_target_xy = (0.85, 2.60)
        manager.controller.path_active = True

        with patch("danger_search.navigation_tasks.astar_plan") as planner:
            status = task.tick(_task_context(manager), (1.12, 2.60, math.pi))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "EXIT_CABIN")
        self.assertEqual(task.cabin_transit_stage, "CROSS")
        self.assertEqual(manager.controller.paths, [])
        # 限制（goal_tolerance）由 set_path 原子提交；CROSS 阶段只轮询状态。
        self.assertEqual(manager.controller.tick_calls, [])
        planner.assert_not_called()

        status = task.tick(_task_context(manager), (0.98, 2.60, math.pi))
        self.assertEqual(status, DONE)
        self.assertEqual(task.phase, "DONE")
        self.assertEqual(manager.building_client.closed_doors, ["lift_door_upper"])

    def test_exit_cabin_with_sync_door_close_falls_back_to_close_target_floor_door(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        manager.elevator_async_target_door_close_enabled = False
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task._cabin_transit_mode = "exit"
        task.cabin_transit_stage = "CROSS"
        task._cabin_transit_start_xy = (2.60, 2.60)
        task._cabin_transit_target_xy = (0.85, 2.60)
        manager.controller.path_active = True

        status = task.tick(_task_context(manager), (0.98, 2.60, math.pi))
        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "CLOSE_TARGET_FLOOR_DOOR")

    def _cross_grace_task(self, grid):
        """构造停在 CROSS 阶段、start/target 已提交的乘梯任务。"""
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task._cabin_transit_mode = "exit"
        task.cabin_transit_stage = "CROSS"
        task._cabin_transit_start_xy = (2.60, 2.60)
        task._cabin_transit_target_xy = (0.85, 2.60)
        return task

    def _cross_grace_tick(self, task, manager, pose):
        """直接推进 CROSS 阶段，绕开上层阶段机以隔离宽限语义。"""
        return task._tick_cabin_transit(
            _task_context(manager),
            pose,
            mode="exit",
            position_xy=(2.60, 2.60),
            target_xy=(0.85, 2.60),
            door_id="elevator_door_1",
        )

    def test_cross_done_but_task_pose_lagging_waits_instead_of_failing(self):
        """seed 8127 回归：控制器按同一容差判到、任务层位姿滞后时不得立即失败。

        真实失败链是控制器以新鲜位姿判到终点（0.123m<=0.15）触发 goal_reached
        并消费终态清掉路径，任务层却用同一 tick 起点较旧的位姿（0.412m）重算
        判未到达，直接 STUCK。宽限必须先于 path_lost 检查生效，否则被清掉的
        路径会立刻误判成路径丢失。
        """
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = self._cross_grace_task(grid)
        manager.controller.path_active = True
        manager.controller.next_tick_status = DONE

        # 任务层位姿仍在 0.27m 处：控制器报 DONE，但完成判定不成立。
        self.assertEqual(
            self._cross_grace_tick(task, manager, (1.12, 2.60, math.pi)),
            RUNNING,
        )
        self.assertEqual(
            task._cabin_transit_goal_grace,
            task._CABIN_TRANSIT_GOAL_GRACE_TICKS,
        )
        self.assertIn("elevator_transit_goal_grace", manager.controller.stop_reasons)

        # goal_reached 已清路径：宽限期内不得判成 path_lost。
        manager.controller.path_active = False
        self.assertEqual(
            self._cross_grace_tick(task, manager, (1.12, 2.60, math.pi)),
            RUNNING,
        )
        self.assertNotIn("elevator_transit_path_lost", manager.controller.stop_reasons)

        # 位姿追上后正常收工，而不是像 seed 8127 那样判失败。
        self.assertEqual(
            self._cross_grace_tick(task, manager, (0.98, 2.60, math.pi)),
            DONE,
        )
        self.assertIn("elevator_transit_completed", manager.controller.clear_reasons)

    def test_cross_grace_exhaustion_still_fails_closed(self):
        """宽限是有界的：位姿始终追不上必须显式 STUCK，不能无限等待。"""
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = self._cross_grace_task(grid)
        manager.controller.path_active = True
        manager.controller.next_tick_status = DONE

        pose = (1.12, 2.60, math.pi)
        self.assertEqual(self._cross_grace_tick(task, manager, pose), RUNNING)
        manager.controller.path_active = False

        # 装填后剩余预算逐 tick 递减，用尽当 tick 判失败。
        for _ in range(task._CABIN_TRANSIT_GOAL_GRACE_TICKS - 1):
            self.assertEqual(self._cross_grace_tick(task, manager, pose), RUNNING)

        self.assertEqual(self._cross_grace_tick(task, manager, pose), STUCK)
        self.assertEqual(task._cabin_transit_goal_grace, 0)

    def test_enter_cabin_propagates_deterministic_failed_instead_of_running(self):
        """seed 8127 第二次回归：确定性 FAILED 不得被吞成 RUNNING。

        _navigate_to_with_status 在 unreachable/budget/timeout 达上限后返回
        FAILED，POSITION 分支原样上抛。旧实现的 ENTER_CABIN 只判 DONE 和
        STUCK，FAILED 会掉进末尾的 return RUNNING；而失败不重置计数器，于是
        elevator_planning_retry_limit 的判决每 tick 触发又每 tick 被丢弃，
        形成零速无界活锁（实测挂 45.6s，attempt 涨到 44 仍未退出）。
        """
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task.cabin_transit_stage = "POSITION"

        # POSITION 且 _retry_count=0：STUCK 会走 recover 重试，FAILED 不得走。
        with patch.object(task, "_tick_cabin_transit", return_value=FAILED):
            self.assertEqual(
                task.tick(_task_context(manager), (0.70, 2.60, 0.0)),
                FAILED,
            )
        self.assertEqual(manager.controller.recover_reasons, [])

    def test_exit_cabin_propagates_deterministic_failed_instead_of_running(self):
        """与 ENTER_CABIN 对称：离厢侧同样不得把 FAILED 吞成 RUNNING。"""
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task.cabin_transit_stage = "POSITION"

        with patch.object(task, "_tick_cabin_transit", return_value=FAILED), patch(
            "danger_search.navigation_tasks.get_time", return_value=10.0
        ):
            self.assertEqual(
                task.tick(_task_context(manager), (2.60, 2.60, math.pi)),
                FAILED,
            )
        self.assertEqual(manager.controller.reverse_reasons, [])

    def test_enter_cabin_unreachable_position_fails_within_retry_limit(self):
        """端到端活性：A* 恒不可达时必须在重试上限内退出，不能无界等待。

        固定几何 + 推进时钟，直接驱动 task.tick。旧实现在此输入下永远返回
        RUNNING（复现 400 tick 未退出）。
        """
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task.cabin_transit_stage = "POSITION"

        ticks = {"now": 0.0}

        def advancing_clock():
            # 必须推进：_PLAN_RETRY_INTERVAL 用时间闸门控制重规划，
            # 冻结时钟会让计数器停在 1，掩盖活锁。
            ticks["now"] += 0.1
            return ticks["now"]

        statuses = []
        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=UNREACHABLE),
        ), patch(
            "danger_search.navigation_tasks.get_time",
            side_effect=advancing_clock,
        ):
            for _ in range(60):
                status = task.tick(_task_context(manager), (0.30, 2.60, 0.0))
                statuses.append(status)
                if status == FAILED:
                    break

        self.assertEqual(statuses[-1], FAILED)
        self.assertEqual(task.failure_reason, "elevator_navigation_unreachable")
        self.assertNotIn(RUNNING, statuses[len(statuses) - 1 :])

    def test_enter_clears_confirmed_door_area_before_first_position_plan(self):
        """修复 B：POSITION 首次规划前必须先清理已确认开放的门区。

        旧实现只在 ALIGN->CROSS 提交时清理，POSITION 规划时起点仍困在门框
        膨胀域内，A* 返回 obstacle_inflation_escape_unreachable，形成“门区
        要等 POSITION 完成才清、POSITION 又被没清的门区挡住”的死锁。
        """
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        events = []
        manager.clear_opened_door_area = lambda door_id: events.append(
            ("clear_door", door_id)
        )

        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task.cabin_transit_stage = "POSITION"

        def recording_plan(*_args, **_kwargs):
            events.append(("plan_path", None))
            return PlanResult(status=UNREACHABLE)

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            side_effect=recording_plan,
        ), patch("danger_search.navigation_tasks.get_time", return_value=10.0):
            task.tick(_task_context(manager), (0.30, 2.60, 0.0))

        self.assertEqual(events[0], ("clear_door", "lift_door_ground"))
        self.assertIn(("plan_path", None), events)
        self.assertLess(
            events.index(("clear_door", "lift_door_ground")),
            events.index(("plan_path", None)),
        )
        # 每次 transit 只清一次，不得每 tick 重复改写地图。
        self.assertEqual(
            [event for event in events if event[0] == "clear_door"],
            [("clear_door", "lift_door_ground")],
        )

    def test_enter_uses_same_position_align_and_cross_sequence(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task._cabin_transit_mode = "enter"
        task.cabin_transit_stage = "ALIGN"

        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.cabin_transit_stage, "ALIGN")
        self.assertTrue(manager.controller.align_calls)

        manager.controller.next_align_status = DONE
        manager.controller.next_align_reason = "alignment_done"
        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.cabin_transit_stage, "CROSS")
        self.assertEqual(
            manager.controller.paths,
            [[(0.85, 2.60), (2.60, 2.60)]],
        )

    def test_enter_refreshes_confirmed_door_strip_before_straight_commit(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        blocked_cell = grid.world_to_grid(1.55, 2.60)
        grid.mark_occupied(*blocked_cell)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        cleared = []

        def clear_door(door_id):
            cleared.append(door_id)
            gx, gy = blocked_cell
            grid.set_snapshot_cell(gx, gy, FREE)

        manager.clear_opened_door_area = clear_door
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task._cabin_transit_mode = "enter"
        task.cabin_transit_stage = "ALIGN"
        task._cabin_transit_alignment_count = 1

        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)
        manager.controller.next_align_status = DONE
        manager.controller.next_align_reason = "alignment_done"
        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)

        self.assertEqual(cleared, ["lift_door_ground"])
        self.assertEqual(task.cabin_transit_stage, "CROSS")
        self.assertEqual(
            manager.controller.paths,
            [[(0.85, 2.60), (2.60, 2.60)]],
        )

    def test_enter_refresh_does_not_override_black_cell(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        blocked_cell = grid.world_to_grid(1.55, 2.60)
        gx, gy = blocked_cell
        grid.mark_black(gx, gy, owner="test:elevator_entry")
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.clear_opened_door_area = lambda door_id: None
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task._cabin_transit_mode = "enter"
        task.cabin_transit_stage = "ALIGN"
        task._cabin_transit_alignment_count = 1

        with patch("danger_search.navigation_tasks.rospy.logwarn_throttle"):
            self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)

        self.assertEqual(task.cabin_transit_stage, "ALIGN")
        self.assertEqual(manager.controller.paths, [])

    def test_enter_alignment_confirmation_tolerates_small_residual_rotation(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task._cabin_transit_mode = "enter"
        task.cabin_transit_stage = "ALIGN"

        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.12)), RUNNING)
        self.assertTrue(manager.controller.align_calls)
        # 余转释放由固定频率控制器 CONFIRM 判据负责，任务层只轮询结果。
        manager.controller.next_align_status = DONE
        manager.controller.next_align_reason = "alignment_done"
        self.assertEqual(task.tick(_task_context(manager), (0.85, 2.60, 0.12)), RUNNING)
        self.assertEqual(task.cabin_transit_stage, "CROSS")
        self.assertEqual(
            manager.controller.paths,
            [[(0.85, 2.60), (2.60, 2.60)]],
        )
        self.assertEqual(manager.controller.turn_commands, [])

    def test_enter_only_keeps_existing_recovery_before_crossing(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task.cabin_transit_stage = "POSITION"

        with patch.object(task, "_tick_cabin_transit", return_value=STUCK):
            self.assertEqual(task.tick(_task_context(manager), (0.70, 2.60, 0.0)), RUNNING)

        self.assertEqual(
            manager.controller.recover_reasons,
            ["elevator_enter_position_stuck"],
        )

        task.cabin_transit_stage = "CROSS"
        with patch.object(task, "_tick_cabin_transit", return_value=STUCK):
            self.assertEqual(task.tick(_task_context(manager), (1.50, 2.60, 0.0)), FAILED)

        self.assertEqual(
            manager.controller.recover_reasons,
            ["elevator_enter_position_stuck"],
        )

    def test_robot_aligns_to_exit_while_current_elevator_door_closes(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "CLOSE_CURRENT_FLOOR_DOOR"
        task._service_task = _DoneService()

        # 关门服务完成即切入 CALL_ELEVATOR_TO_TARGET_FLOOR 发起升梯，调头在升梯中并行
        self.assertEqual(task.tick(_task_context(manager), (2.60, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.phase, "CALL_ELEVATOR_TO_TARGET_FLOOR")
        self.assertTrue(manager.controller.align_calls)

    def test_pending_service_holds_position_by_default(self):
        """STAGE24：普通服务等待仍须停车，保持原有失效安全语义。"""
        grid = MultiFloorGridMap(resolution=0.1, width=5.0, length=5.0).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "OPEN_CURRENT_FLOOR_DOOR"
        task._service_task = _PendingService()

        self.assertEqual(
            task.tick(_task_context(manager), (0.85, 2.60, 0.0)),
            RUNNING,
        )

        self.assertEqual(manager.controller.stop_reasons, ["stop"])

    def test_pending_close_service_does_not_interrupt_cabin_alignment(self):
        """STAGE24：关门与调头并行时，服务轮询不得发布竞争零速。"""
        grid = MultiFloorGridMap(resolution=0.1, width=5.0, length=5.0).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "CLOSE_CURRENT_FLOOR_DOOR"
        task._service_task = _PendingService()

        self.assertEqual(
            task.tick(_task_context(manager), (2.60, 2.60, 0.0)),
            RUNNING,
        )

        self.assertEqual(manager.controller.stop_reasons, [])
        self.assertEqual(
            manager.controller.align_calls,
            [((0.85, 2.60), {"max_angular": 0.65})],
        )

    def test_pending_floor_service_does_not_interrupt_cabin_alignment(self):
        """STAGE24：升梯与调头并行时，服务轮询不得发布竞争零速。"""
        grid = MultiFloorGridMap(resolution=0.1, width=5.0, length=5.0).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        task = TakeElevator(0, 1)
        task.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
        task._service_task = _PendingService()

        self.assertEqual(
            task.tick(_task_context(manager), (2.60, 2.60, 0.0)),
            RUNNING,
        )

        self.assertEqual(manager.controller.stop_reasons, [])
        self.assertEqual(
            manager.controller.align_calls,
            [((0.85, 2.60), {"max_angular": 0.65})],
        )

    def test_cabin_transit_enter_fast_path_accepts_080_boundary(self):
        """STAGE24：门前航向误差等于 0.80rad 时同 tick 进入 CROSS。"""
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        for gx in range(5, 35):
            for gy in range(15, 35):
                grid.set_base_state(gx, gy, FREE)
        cabin_center = (2.85, 2.15)
        current_wait = (0.65, 2.15)
        pose = (0.65, 2.15, -0.80)
        controller = SimpleNamespace(
            goal_tolerance=0.15,
            set_path=MagicMock(),
            clear_path=MagicMock(),
            stop=MagicMock(),
            poll_result=lambda: ControlResult(status=IDLE, reason=None, sequence=1),
            poll_status=lambda: IDLE,
            begin_heading_alignment=MagicMock(),
            end_heading_alignment=MagicMock(),
        )
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: (current_wait, cabin_center),
                elevator_door_id=lambda floor: "lift_door_0",
            ),
            mapper=SimpleNamespace(floor=lambda idx: grid),
            current_floor=0,
            controller=controller,
            elevator_crossing_geometry_enabled=False,
            elevator_local_crossing_geometry_enabled=False,
            planning_constraints=None,
        )
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        status = task._tick_cabin_transit(
            _task_context(manager),
            pose,
            mode="enter",
            position_xy=current_wait,
            target_xy=cabin_center,
            door_id="lift_door_0",
        )
        self.assertEqual(status, RUNNING)
        self.assertEqual(task.cabin_transit_stage, "CROSS")
        controller.set_path.assert_called_once()
        controller.begin_heading_alignment.assert_not_called()

    def test_cabin_transit_enter_above_080_falls_back_to_align(self):
        """STAGE24：门前航向误差超过 0.80rad 时保留 ALIGN 安全流程。"""
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        cabin_center = (2.85, 2.15)
        current_wait = (0.65, 2.15)
        pose = (0.65, 2.15, -0.801)
        controller = SimpleNamespace(
            goal_tolerance=0.15,
            set_path=MagicMock(),
            clear_path=MagicMock(),
            stop=MagicMock(),
            poll_result=lambda: ControlResult(status=IDLE, reason=None, sequence=1),
            poll_status=lambda: IDLE,
            begin_heading_alignment=MagicMock(),
            end_heading_alignment=MagicMock(),
        )
        manager = SimpleNamespace(
            mapper=SimpleNamespace(floor=lambda idx: grid),
            current_floor=0,
            controller=controller,
            planning_constraints=None,
        )
        task = TakeElevator(0, 1)

        status = task._tick_cabin_transit(
            _task_context(manager),
            pose,
            mode="enter",
            position_xy=current_wait,
            target_xy=cabin_center,
            door_id="lift_door_0",
        )

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.cabin_transit_stage, "ALIGN")
        controller.set_path.assert_not_called()
        controller.begin_heading_alignment.assert_not_called()

    def test_take_elevator_gates_wait_target_door_open_until_cabin_turn_complete(self):
        """STAGE22: 换层后门控等待门开口必须等待轿厢调头完成，先转正后开门确认。"""
        grid = MultiFloorGridMap(resolution=0.1, width=6.0, length=6.0).floor(1)
        for gx in range(0, 60):
            for gy in range(0, 60):
                grid.set_base_state(gx, gy, FREE)
        cabin_center = (2.85, 2.15)
        target_wait = (0.65, 2.15)
        pose = (2.85, 2.15, 0.0)

        controller = SimpleNamespace(
            goal_tolerance=0.15,
            stop=MagicMock(),
            clear_path=MagicMock(),
            poll_result=lambda: ControlResult(status=RUNNING, reason=None, sequence=1),
            poll_status=lambda: RUNNING,
            begin_heading_alignment=MagicMock(),
            end_heading_alignment=MagicMock(),
        )
        client = SimpleNamespace(
            get_elevator_points=lambda elevator_id, floor: (target_wait, cabin_center),
            elevator_door_id=lambda floor: "lift_door_1",
            call_elevator=lambda *a: None,
        )
        manager = SimpleNamespace(
            building_client=client,
            mapper=SimpleNamespace(floor=lambda idx: grid),
            current_floor=0,
            controller=controller,
            begin_floor_transition=MagicMock(),
            begin_elevator_door_open_confirmation=MagicMock(),
        )
        task = TakeElevator(0, 1)
        task.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
        task._service_task = _DoneService()

        # 调头未完成时，切入 WAIT_CABIN_TURN_COMPLETE 而不是直接进入 WAIT_TARGET_DOOR_OPEN
        status = task.tick(_task_context(manager), pose)
        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "WAIT_CABIN_TURN_COMPLETE")
        manager.begin_elevator_door_open_confirmation.assert_not_called()

        # 调头完成后，必须先显式调用目标层 open_door，不能把呼梯成功当成开门。
        controller.poll_result = lambda: ControlResult(status=DONE, reason="alignment_done", sequence=2)
        status = task.tick(_task_context(manager), pose)
        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "OPEN_TARGET_FLOOR_DOOR")
        self.assertTrue(task._cabin_turn_completed)

    def test_take_elevator_cabin_turn_timeout_fails_deterministically(self):
        """STAGE24：轿厢调头在 12s 边界内继续等待，超过后确定性失败。"""
        grid = MultiFloorGridMap(resolution=0.1, width=6.0, length=6.0).floor(1)
        cabin_center = (2.85, 2.15)
        target_wait = (0.65, 2.15)
        pose = (2.85, 2.15, 0.0)

        controller = SimpleNamespace(
            goal_tolerance=0.15,
            stop=MagicMock(),
            clear_path=MagicMock(),
            poll_result=lambda: ControlResult(status=RUNNING, reason=None, sequence=1),
            poll_status=lambda: RUNNING,
            begin_heading_alignment=MagicMock(),
            end_heading_alignment=MagicMock(),
        )
        client = SimpleNamespace(
            get_elevator_points=lambda elevator_id, floor: (target_wait, cabin_center),
            elevator_door_id=lambda floor: "lift_door_1",
            call_elevator=lambda *a: None,
        )
        manager = SimpleNamespace(
            building_client=client,
            mapper=SimpleNamespace(floor=lambda idx: grid),
            current_floor=0,
            controller=controller,
            _record_failure=MagicMock(),
        )
        task = TakeElevator(0, 1)
        task.phase = "WAIT_CABIN_TURN_COMPLETE"
        task._cabin_turn_wait_start = 100.0

        with patch("danger_search.navigation_tasks.get_time", return_value=112.0):
            self.assertEqual(task.tick(_task_context(manager), pose), RUNNING)
            self.assertIsNone(task.failure_reason)

        with patch("danger_search.navigation_tasks.get_time", return_value=112.01):
            self.assertEqual(task.tick(_task_context(manager), pose), FAILED)
            self.assertEqual(task.failure_reason, "elevator_cabin_turn_timeout")
            manager._record_failure.assert_called_once_with("elevator_cabin_turn_timeout")

    def test_target_door_confirmation_updates_floor_before_exit_mapping(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.target_floor = 1
        confirmed_floors = []
        manager.confirm_elevator_door_open = lambda door_id, pose: True
        manager.finish_elevator_door_open_confirmation = lambda door_id: None
        manager.clear_opened_door_area = lambda door_id: None
        manager.confirm_floor_transition = lambda floor: (
            confirmed_floors.append(floor),
            setattr(manager, "current_floor", floor),
        )
        task = TakeElevator(0, 1)
        task.phase = "WAIT_TARGET_DOOR_OPEN"

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(
                path=[(2.60, 2.60), (0.85, 2.60)], status="reached"
            ),
        ), patch(
            "danger_search.navigation_tasks._segment_crosses_blocked_cell",
            return_value=False,
        ):
            status = task.tick(_task_context(manager), (2.60, 2.60, math.pi))

        self.assertEqual(status, RUNNING)
        self.assertEqual(confirmed_floors, [1])
        self.assertEqual(manager.current_floor, 1)
        self.assertEqual(task.phase, "EXIT_CABIN")

    def test_floor_scoped_updates_are_latched_before_elevator_service_call(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        transition_calls = []
        service_calls = []
        manager.begin_floor_transition = lambda **kwargs: transition_calls.append(kwargs)
        manager.building_client.call_elevator = (
            lambda elevator_id, floor, open_doors: service_calls.append(
                (elevator_id, floor, open_doors)
            )
            or _DoneService()
        )
        task = TakeElevator(0, 1)
        task.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"

        self.assertEqual(task.tick(_task_context(manager), (2.60, 2.60, math.pi)), RUNNING)

        self.assertEqual(
            transition_calls,
            [
                {
                    "source_floor": 0,
                    "target_floor": 1,
                    "phase": "CALL_ELEVATOR_TO_TARGET_FLOOR",
                }
            ],
        )
        self.assertEqual(service_calls, [("elevator_main", 1, True)])

    def test_wait_target_door_requires_unblocked_straight_cabin_segment(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        grid.mark_occupied(*grid.world_to_grid(1.55, 2.60))
        manager = self._exit_manager(grid)
        manager.target_floor = 1
        task = TakeElevator(0, 1)
        task.phase = "WAIT_TARGET_DOOR_OPEN"

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(path=[(2.60, 2.60), (0.85, 2.60)], status="reached"),
        ) as planner, patch(
            "danger_search.navigation_tasks.rospy.logwarn_throttle",
            create=True,
        ):
            status = task.tick(_task_context(manager), (2.60, 2.60, math.pi))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "WAIT_TARGET_DOOR_OPEN")
        planner.assert_not_called()

    def test_elevator_service_failure_returns_failed(self):
        failures = []
        task = TakeElevator(0, 1)
        task.phase = "CLOSE_TARGET_FLOOR_DOOR"
        task._service_task = _FailedService("jammed")
        task._retry_count = 1
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: ((0.0, 0.0), (0.0, 1.0)),
                elevator_door_id=lambda floor: f"lift_door_{int(floor)}",
                close_door=lambda door_id: _FailedService("jammed"),
            ),
            controller=SimpleNamespace(stop=lambda: None),
            _record_failure=lambda reason: failures.append(reason),
        )

        status = task.tick(_task_context(manager), (0.0, 0.0, math.pi / 2.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(failures, ["电梯服务失败: jammed"])

    def test_wait_current_door_open_enters_when_only_unknown_cells_block_astar(self):
        cleared = []
        task = TakeElevator(0, 1)
        task.phase = "WAIT_CURRENT_DOOR_OPEN"
        task._wait_start_time = 0.0
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        ).floor(0)
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: ((0.0, 0.0), (0.0, 1.0)),
                elevator_door_id=lambda floor: f"lift_door_{int(floor)}",
            ),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(stop=lambda: None),
            clear_opened_door_area=lambda door_id: cleared.append(door_id),
        )

        with patch("danger_search.navigation_tasks.astar_plan", return_value=PlanResult(status=UNREACHABLE)) as planner, patch(
            "danger_search.navigation_tasks.get_time",
            return_value=11.0,
        ):
            status = task.tick(_task_context(manager), (0.0, 0.0, math.pi / 2.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "ENTER_CABIN")
        self.assertEqual(cleared, ["lift_door_0"])
        planner.assert_not_called()

    def test_wait_current_door_open_keeps_waiting_for_positive_blocker(self):
        task = TakeElevator(0, 1)
        task.phase = "WAIT_CURRENT_DOOR_OPEN"
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        ).floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.0, 0.5))
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: ((0.0, 0.0), (0.0, 1.0)),
                elevator_door_id=lambda floor: f"lift_door_{int(floor)}",
            ),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(stop=lambda: None),
            clear_opened_door_area=lambda door_id: None,
        )

        status = task.tick(_task_context(manager), (0.0, 0.0, math.pi / 2.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "WAIT_CURRENT_DOOR_OPEN")

    def test_wait_current_door_does_not_clear_map_before_sensor_confirmation(self):
        cleared = []
        confirmations = [False, True]
        task = TakeElevator(0, 1)
        task.phase = "WAIT_CURRENT_DOOR_OPEN"
        task._wait_start_time = 5.0
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: (
                    (0.85, 2.60),
                    (2.60, 2.60),
                ),
                elevator_door_id=lambda floor: f"lift_door_{int(floor)}",
            ),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(stop=lambda: None),
            confirm_elevator_door_open=lambda door_id, pose: confirmations.pop(0),
            finish_elevator_door_open_confirmation=lambda door_id: None,
            elevator_opening_confirmation_timeout=8.0,
            clear_opened_door_area=lambda door_id: cleared.append(door_id),
        )

        with patch(
            "danger_search.navigation_tasks.get_time", return_value=6.0
        ), patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(
                path=[(0.85, 2.60), (2.60, 2.60)], status="reached"
            ),
        ), patch(
            "danger_search.navigation_tasks._segment_crosses_blocked_cell",
            return_value=False,
        ):
            first = task.tick(_task_context(manager), (0.85, 2.60, 0.0))
            second = task.tick(_task_context(manager), (0.85, 2.60, 0.0))

        self.assertEqual(first, RUNNING)
        self.assertEqual(second, RUNNING)
        self.assertEqual(cleared, ["lift_door_0"])
        self.assertEqual(task.phase, "ENTER_CABIN")

    def test_open_current_door_waits_for_visual_preparation_service(self):
        task = TakeElevator(0, 1)
        task.phase = "OPEN_CURRENT_FLOOR_DOOR"
        open_calls = []
        stop_calls = []
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: (
                    (0.85, 2.60),
                    (2.60, 2.60),
                ),
                elevator_door_id=lambda floor: f"lift_door_{int(floor)}",
                open_door=lambda door_id: open_calls.append(door_id),
            ),
            elevator_visual_preparation_pending=lambda door_id: True,
            controller=SimpleNamespace(stop=lambda: stop_calls.append(True)),
        )

        status = task.tick(_task_context(manager), (0.85, 2.60, 0.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "OPEN_CURRENT_FLOOR_DOOR")
        self.assertEqual(open_calls, [])
        self.assertEqual(stop_calls, [True])

    def test_current_door_confirmation_timeout_reanchors_refreshes_and_recalls_once(self):
        stamps = [20.0]
        finished = []
        events = []
        controller = SimpleNamespace(
            stop=lambda **_kwargs: None,
            clear_path=lambda **_kwargs: None,
        )
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda _elevator_id, _floor: ((0.0, 0.0), (0.0, 1.0)),
                elevator_door_id=lambda floor: f"lift_door_{int(floor)}",
            ),
            mapper=SimpleNamespace(floor=lambda _floor: object()),
            current_floor=0,
            controller=controller,
            elevator_opening_confirmation_timeout=15.0,
            elevator_opening_confirmation_retry_limit=1,
            elevator_opening_confirmation_retry_timeout=10.0,
            elevator_opening_confirmation_sensor_refresh=0.5,
            latest_cloud_stamp=lambda: stamps[-1],
            relocalize_landmark=lambda *_args: SimpleNamespace(status=DONE, applied=False),
            confirm_elevator_door_open=lambda *_args: False,
            finish_elevator_door_open_confirmation=lambda door_id: finished.append(door_id),
            _log_structured=lambda event: events.append(event),
        )
        task = TakeElevator(0, 1)
        task.phase = "WAIT_CURRENT_DOOR_OPEN"
        task._wait_start_time = 0.0
        task._door_confirmation_started.add("lift_door_0")

        with patch("danger_search.navigation_tasks.get_time", return_value=16.0):
            self.assertEqual(task.tick(_task_context(manager), (0.0, 0.0, 0.0)), RUNNING)
        self.assertEqual(task.phase, "RETRY_CURRENT_DOOR_SENSOR_REFRESH")
        self.assertEqual(finished, ["lift_door_0"])
        self.assertEqual(events[-1]["attempt"], 1)

        stamps.append(20.1)
        with patch("danger_search.navigation_tasks.get_time", return_value=16.6):
            self.assertEqual(task.tick(_task_context(manager), (0.0, 0.0, 0.0)), RUNNING)
        self.assertEqual(task.phase, "RETRY_ALIGN_CURRENT_DOOR")
        with patch.object(task, "_tick_target_door_alignment", return_value=True):
            self.assertEqual(task.tick(_task_context(manager), (0.0, 0.0, 0.0)), RUNNING)
        self.assertEqual(task.phase, "CALL_ELEVATOR_TO_CURRENT_FLOOR")

    def test_target_door_retry_stays_in_cabin_and_only_reopens_after_fresh_cloud(self):
        stamps = [30.0]
        task = TakeElevator(0, 1)
        task.phase = "WAIT_TARGET_DOOR_OPEN"
        task._wait_start_time = 0.0
        task._door_confirmation_started.add("lift_door_1")
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda _elevator_id, _floor: ((0.0, 0.0), (0.0, 1.0)),
                elevator_door_id=lambda floor: f"lift_door_{int(floor)}",
            ),
            mapper=SimpleNamespace(floor=lambda _floor: object()),
            current_floor=0,
            controller=SimpleNamespace(
                stop=lambda **_kwargs: None,
                clear_path=lambda **_kwargs: None,
            ),
            elevator_opening_confirmation_timeout=15.0,
            latest_cloud_stamp=lambda: stamps[-1],
            confirm_elevator_door_open=lambda *_args: False,
            finish_elevator_door_open_confirmation=lambda _door_id: None,
        )
        with patch("danger_search.navigation_tasks.get_time", return_value=16.0):
            self.assertEqual(task.tick(_task_context(manager), (0.0, 1.0, 0.0)), RUNNING)
        self.assertEqual(task.phase, "RETRY_TARGET_DOOR_SENSOR_REFRESH")

        stamps.append(30.1)
        with patch("danger_search.navigation_tasks.get_time", return_value=16.6):
            self.assertEqual(task.tick(_task_context(manager), (0.0, 1.0, 0.0)), RUNNING)
        self.assertEqual(task.phase, "OPEN_TARGET_FLOOR_DOOR")


class TakeElevatorPhaseExhaustivenessTest(unittest.TestCase):
    """未注册的乘梯阶段必须显式失败，不得沿用兜底 return DONE。

    `TakeElevator.tick` 由 20 个平铺 `if self.phase == ...` 分支加末尾一条
    `return DONE` 组成。兜底对 `phase == "DONE"` 是正确的，但任何未注册
    phase（改名、拼写错误、新增阶段忘记登记）也会落到同一条兜底上，
    使任务层收到 DONE 并认为换层已完成 —— 一次谎报的乘梯成功。

    这与 NAV-006 同类但更严重：NAV-006 把 FAILED 吞成 RUNNING（活锁，
    至少不会推进任务），此处把未知吞成 DONE（假成功，任务据此推进到
    下一层）。修复前实测：未注册 phase 与空字符串 phase 均返回 done、
    failure_reason 为 None、无任何 stop 命令。
    """

    @staticmethod
    def _source():
        stop_reasons = []
        clear_reasons = []
        controller = SimpleNamespace(
            goal_tolerance=0.1,
            has_path=lambda: False,
            poll_status=lambda: RUNNING,
            stop=lambda **kwargs: stop_reasons.append(kwargs.get("reason")),
            clear_path=lambda **kwargs: clear_reasons.append(kwargs.get("reason")),
            set_path=lambda path, **kwargs: None,
        )
        source = SimpleNamespace(
            controller=controller,
            current_floor=0,
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: (
                    (0.85, 2.60),
                    (2.60, 2.60),
                ),
                elevator_door_id=lambda floor: f"lift_door_{int(floor)}",
            ),
        )
        return source, stop_reasons, clear_reasons

    def test_unregistered_phase_fails_instead_of_claiming_done(self):
        for phase in ("TYPO_PHASE_NOT_REGISTERED", ""):
            with self.subTest(phase=phase):
                source, stop_reasons, clear_reasons = self._source()
                task = TakeElevator(0, 1)
                task.phase = phase

                status = task.tick(_task_context(source), (0.85, 2.60, 0.0))

                self.assertEqual(
                    status,
                    FAILED,
                    f"未注册 phase {phase!r} 仍被兜底判为成功",
                )
                self.assertEqual(task.failure_reason, "elevator_unknown_phase")
                self.assertIn("elevator_unknown_phase", stop_reasons)
                self.assertIn("elevator_unknown_phase", clear_reasons)

    def test_done_phase_still_returns_done(self):
        # 兜底原本的正当用途必须保留：phase 已是 DONE 时返回 DONE 且不失败。
        source, stop_reasons, clear_reasons = self._source()
        task = TakeElevator(0, 1)
        task.phase = "DONE"

        status = task.tick(_task_context(source), (0.85, 2.60, 0.0))

        self.assertEqual(status, DONE)
        self.assertIsNone(task.failure_reason)
        self.assertEqual(stop_reasons, [])
        self.assertEqual(clear_reasons, [])

    def test_every_assigned_phase_is_handled_by_tick(self):
        """类内所有 `self.phase = X` 的 X 都必须被 tick 显式处理。

        用 AST 比对赋值集与处理集，防止新增阶段时忘记登记分支 —— 那正是
        兜底 fail-open 的触发条件。
        """
        import ast
        from pathlib import Path

        source_path = (
            Path(__file__).resolve().parents[1]
            / "danger_search"
            / "navigation_tasks.py"
        )
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        take_elevator = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.ClassDef) and node.name == "TakeElevator"
        )

        assigned = set()
        for node in ast.walk(take_elevator):
            if not isinstance(node, ast.Assign):
                continue
            for target in node.targets:
                if (
                    isinstance(target, ast.Attribute)
                    and target.attr == "phase"
                    and isinstance(node.value, ast.Constant)
                    and isinstance(node.value.value, str)
                ):
                    assigned.add(node.value.value)

        tick = next(
            node
            for node in take_elevator.body
            if isinstance(node, ast.FunctionDef) and node.name == "tick"
        )
        compared = set()
        prefixes = set()
        for node in ast.walk(tick):
            if isinstance(node, ast.Compare):
                for comparator in node.comparators:
                    if isinstance(comparator, ast.Constant) and isinstance(
                        comparator.value, str
                    ):
                        compared.add(comparator.value)
            # phase.startswith("EXIT_BACKOFF_") 形式的前缀派发
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "startswith"
                and node.args
                and isinstance(node.args[0], ast.Constant)
            ):
                prefixes.add(node.args[0].value)

        unhandled = {
            phase
            for phase in assigned - compared - {"DONE"}
            if not any(phase.startswith(prefix) for prefix in prefixes)
        }
        self.assertEqual(
            unhandled,
            set(),
            f"以下 phase 被赋值但 tick 未显式处理，会落进兜底：{sorted(unhandled)}",
        )


if __name__ == "__main__":
    unittest.main()

import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.navigation_tasks import DONE, FAILED, OpenDoorAndPass, RUNNING, TakeElevator
from danger_search.mapper import BLACK, FREE, MultiFloorGridMap
from danger_search.navigator import STUCK
from danger_search.planner import BUDGET_EXHAUSTED, PlanResult, UNREACHABLE


class _DoneService:
    def done(self):
        return True

    def ok(self):
        return True

    def message(self):
        return ""


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

        self.assertEqual(task.tick(manager, (0.0, -0.8, math.pi / 2.0)), RUNNING)
        self.assertEqual(task.tick(manager, (0.0, -0.8, math.pi / 2.0)), RUNNING)

        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(cleared, ["main_entrance"])

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

        status = task.tick(manager, (0.0, -0.8, math.pi / 2.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(failures, ["开门服务失败: denied"])

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
                set_path=lambda path: None,
                tick=lambda pose: RUNNING,
                recover=lambda: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            ),
        )

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=UNREACHABLE),
        ) as astar:
            status = task.tick(manager, (0.0, -2.3, math.pi / 2.0))

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
                set_path=lambda path: None,
                tick=lambda pose: RUNNING,
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
            first = task.tick(manager, (0.0, -4.0, math.pi / 2.0))
            second = task.tick(manager, (0.0, -4.0, math.pi / 2.0))

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
                set_path=lambda path: None,
                tick=lambda pose: RUNNING,
                recover=lambda: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            ),
        )

        with patch(
            "danger_search.navigation_tasks.astar_plan",
            return_value=PlanResult(status=UNREACHABLE),
        ), patch("danger_search.navigation_tasks.get_time", return_value=10.0):
            status = task.tick(manager, (1.2, -2.3, math.pi / 2.0))

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
                set_path=lambda path: recorded_paths.append(path),
                clear_path=lambda: cleared_paths.append(True),
                tick=lambda pose: RUNNING,
                recover=lambda: None,
            ),
            clear_opened_door_area=lambda door_id: cleared_doors.append(door_id),
        )

        with patch("danger_search.navigation_tasks.astar_plan", return_value=PlanResult(status=UNREACHABLE)):
            status = task.tick(manager, (0.0, 0.8, math.pi / 2.0))

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

        def set_path(path):
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
                tick=lambda pose, **kwargs: RUNNING,
                recover=lambda: None,
                rotate_in_place=lambda angular: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            ),
            clear_opened_door_area=lambda door_id: cleared_doors.append(door_id),
        )
        pose = (0.2, -1.1, math.atan2(1.9, -0.2))

        with patch("danger_search.navigation_tasks.astar_plan") as astar:
            first = task.tick(manager, pose)
            second = task.tick(manager, pose)

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
                set_path=lambda path: None,
                clear_path=lambda **kwargs: cleared_paths.append(kwargs.get("reason")),
                tick=lambda pose, **kwargs: RUNNING,
                recover=lambda: recovered.append(True),
                rotate_in_place=lambda angular: None,
                stop=lambda **kwargs: stopped.append(kwargs.get("reason")),
            ),
            clear_opened_door_area=lambda door_id: cleared_doors.append(door_id),
            _record_failure=lambda reason: failures.append(reason),
        )

        pose = (0.2, -1.1, math.atan2(1.9, -0.2))
        with patch("danger_search.navigation_tasks.get_time", side_effect=[10.0, 13.1]):
            align = task.tick(manager, pose)
            first = task.tick(manager, pose)
            second = task.tick(manager, pose)

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
                tick=lambda pose, **kwargs: STUCK,
                recover=lambda: recovered.append(True),
                clear_path=lambda **kwargs: None,
            ),
            _record_failure=lambda reason: failures.append(reason),
        )

        status = task.tick(manager, (0.0, -0.8, math.pi / 2.0))

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
                tick=lambda pose, **kwargs: DONE,
                clear_path=lambda **kwargs: None,
            ),
            _record_failure=lambda reason: failures.append(reason),
        )

        status = task.tick(manager, (0.0, 0.61, math.pi / 2.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(failures, ["控制器提前结束过门路径"])


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
                self.next_tick_status = RUNNING

            def stop(self, *, reason="stop"):
                self.stop_reasons.append(reason)

            def clear_path(self, *, reason="path_cleared"):
                self.clear_reasons.append(reason)
                self.path_active = False

            def has_path(self):
                return self.path_active

            def set_path(self, path):
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
        client = SimpleNamespace(
            get_elevator_points=lambda elevator_id, floor: (
                (0.85, 2.60),
                (2.60, 2.60),
            ),
            elevator_door_id=lambda floor: elevator_door_ids[int(floor)],
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
            self.assertEqual(task.tick(manager, (1.37, 2.60, math.pi)), RUNNING)

        self.assertEqual(task.phase, "EXIT_BACKOFF_SETTLE")
        self.assertEqual(task._retry_count, 1)
        self.assertGreater(task._exit_backoff_target_xy[0], 1.37)

        with patch("danger_search.navigation_tasks.get_time", return_value=10.6):
            task.tick(manager, (1.37, 2.60, math.pi))
        self.assertEqual(task.phase, "EXIT_BACKOFF_ALIGN")

        # 朝向与出口相反时只能原地对准，不能开始倒车。
        with patch("danger_search.navigation_tasks.get_time", return_value=10.7):
            task.tick(manager, (1.37, 2.60, 0.0))
        self.assertEqual(task.phase, "EXIT_BACKOFF_ALIGN")
        self.assertTrue(manager.controller.turn_commands)
        self.assertEqual(manager.controller.reverse_reasons, [])

        with patch("danger_search.navigation_tasks.get_time", return_value=10.8):
            task.tick(manager, (1.37, 2.60, math.pi))
        self.assertEqual(task.phase, "EXIT_BACKOFF_REVERSE")

        with patch("danger_search.navigation_tasks.get_time", return_value=10.9):
            task.tick(manager, (1.37, 2.60, math.pi))
        self.assertEqual(manager.controller.reverse_reasons, ["exit_cabin_backoff"])

        with patch("danger_search.navigation_tasks.get_time", return_value=12.0):
            task.tick(manager, (1.72, 2.60, math.pi))
        self.assertEqual(task.phase, "EXIT_BACKOFF_REALIGN")

        with patch("danger_search.navigation_tasks.get_time", return_value=12.1):
            task.tick(manager, (1.72, 2.60, math.pi))
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
            self.assertEqual(task.tick(manager, (0.85, 2.60, math.pi / 2.0)), RUNNING)

        self.assertEqual(task.phase, "ALIGN_TO_CURRENT_DOOR")
        self.assertEqual(task.tick(manager, (0.85, 2.60, math.pi / 2.0)), RUNNING)
        self.assertTrue(manager.controller.turn_commands)
        self.assertEqual(task.phase, "ALIGN_TO_CURRENT_DOOR")

        self.assertEqual(task.tick(manager, (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.phase, "ALIGN_TO_CURRENT_DOOR")
        self.assertEqual(task.tick(manager, (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.phase, "CALL_ELEVATOR_TO_CURRENT_FLOOR")

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
        observed_caps = []

        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task._cabin_transit_mode = "exit"
        task.cabin_transit_stage = "CROSS"
        task._cabin_transit_start_xy = (2.0, 2.60)
        task._cabin_transit_target_xy = (0.85, 2.60)
        manager.controller.path_active = True

        def tick_controller(pose, **kwargs):
            observed_caps.append(kwargs.get("max_linear"))
            return RUNNING

        manager.controller.tick = tick_controller

        self.assertEqual(task.tick(manager, (2.0, 2.60, math.pi)), RUNNING)
        task._retry_count = 1
        self.assertEqual(task.tick(manager, (2.0, 2.60, math.pi)), RUNNING)

        self.assertEqual(observed_caps, [0.25, None])
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
        observed_poses = []
        manager.controller.tick = lambda pose, **kwargs: observed_poses.append((pose, kwargs)) or RUNNING
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task._cabin_transit_mode = "exit"
        task.cabin_transit_stage = "CROSS"
        task._cabin_transit_start_xy = (2.0, 2.60)
        task._cabin_transit_target_xy = (0.85, 2.60)
        manager.controller.path_active = True

        self.assertEqual(task.tick(manager, (2.0, 2.60, math.pi)), RUNNING)

        self.assertEqual(observed_poses, [((2.0, 2.60, math.pi), {"goal_tolerance": 0.15})])

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
            status = task.tick(manager, (1.37, 2.60, math.pi))

        self.assertEqual(status, FAILED)
        self.assertEqual(task.phase, "EXIT_CABIN")
        self.assertEqual(task._retry_count, 0)
        self.assertEqual(manager.controller.reverse_reasons, [])

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
            status = task.tick(manager, (1.37, 2.60, math.pi))

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
            status = task.tick(manager, (1.37, 2.60, math.pi))

        self.assertEqual(status, FAILED)
        self.assertEqual(manager.controller.reverse_reasons, [])
        self.assertIn("exit_backoff_failed", manager.controller.stop_reasons)

    def test_exit_positions_at_cabin_center_with_tight_tolerance_before_alignment(self):
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
            status = task.tick(manager, (2.32, 2.53, 0.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.cabin_transit_stage, "ALIGN")
        # POSITION 仍用严格 position_tolerance；ALIGN 的漂移检查才放宽到 acceptance。
        self.assertEqual(calls, [((2.60, 2.60), {"goal_tolerance": 0.15})])

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
                grid.data[gy, gx] = FREE

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

            def set_path(self, path):
                self.paths.append(list(path))
                self._path = list(path)

            def has_path(self):
                return bool(self._path)

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
        # 朝向已对准目标，避免对准逻辑干扰，聚焦 POSITION/ALIGN 一致性
        yaw = math.atan2(target_wait[1] - robot[1], target_wait[0] - robot[0])
        pose = (robot[0], robot[1], yaw)

        stages = []
        for _ in range(6):
            task._tick_cabin_transit(
                manager,
                pose,
                mode="exit",
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
        task._cabin_transit_mode = "exit"

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
        # 强制 POSITION 始终立即 DONE，ALIGN 始终漂移退回，模拟语义退化
        with patch.object(task, "_navigate_to_with_status", return_value=DONE):
            for _ in range(2 * (flip_limit + 1)):
                status = task._tick_cabin_transit(
                    manager,
                    pose,
                    mode="exit",
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

        self.assertEqual(task.tick(manager, (2.60, 2.60, 0.0)), RUNNING)
        self.assertTrue(manager.controller.turn_commands)
        self.assertEqual(manager.controller.paths, [])

        self.assertEqual(task.tick(manager, (2.60, 2.60, math.pi)), RUNNING)
        self.assertEqual(manager.controller.paths, [])

        self.assertEqual(task.tick(manager, (2.60, 2.60, math.pi)), RUNNING)
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

        self.assertEqual(task.tick(manager, (0.85, 2.60, -0.14)), RUNNING)

        self.assertGreaterEqual(
            abs(manager.controller.turn_commands[-1]),
            manager.controller.recover_angular_speed,
        )

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
            status = task.tick(manager, (1.12, 2.60, math.pi))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "EXIT_CABIN")
        self.assertEqual(task.cabin_transit_stage, "CROSS")
        self.assertEqual(manager.controller.paths, [])
        self.assertEqual(
            manager.controller.tick_calls,
            [((1.12, 2.60, math.pi), {"goal_tolerance": 0.15})],
        )
        planner.assert_not_called()

        status = task.tick(manager, (0.98, 2.60, math.pi))
        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "CLOSE_TARGET_FLOOR_DOOR")

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

        self.assertEqual(task.tick(manager, (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.tick(manager, (0.85, 2.60, 0.0)), RUNNING)

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
            grid.data[gy, gx] = FREE

        manager.clear_opened_door_area = clear_door
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task._cabin_transit_mode = "enter"
        task.cabin_transit_stage = "ALIGN"
        task._cabin_transit_alignment_count = 1

        self.assertEqual(task.tick(manager, (0.85, 2.60, 0.0)), RUNNING)

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
        grid.data[gy, gx] = BLACK
        manager = self._exit_manager(grid)
        manager.current_floor = 0
        manager.clear_opened_door_area = lambda door_id: None
        task = TakeElevator(0, 1)
        task.phase = "ENTER_CABIN"
        task._cabin_transit_mode = "enter"
        task.cabin_transit_stage = "ALIGN"
        task._cabin_transit_alignment_count = 1

        with patch("danger_search.navigation_tasks.rospy.logwarn_throttle"):
            self.assertEqual(task.tick(manager, (0.85, 2.60, 0.0)), RUNNING)

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

        self.assertEqual(task.tick(manager, (0.85, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.tick(manager, (0.85, 2.60, 0.12)), RUNNING)

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
            self.assertEqual(task.tick(manager, (0.70, 2.60, 0.0)), RUNNING)

        self.assertEqual(
            manager.controller.recover_reasons,
            ["elevator_enter_position_stuck"],
        )

        task.cabin_transit_stage = "CROSS"
        with patch.object(task, "_tick_cabin_transit", return_value=STUCK):
            self.assertEqual(task.tick(manager, (1.50, 2.60, 0.0)), FAILED)

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

        self.assertEqual(task.tick(manager, (2.60, 2.60, 0.0)), RUNNING)
        self.assertEqual(task.phase, "ALIGN_TO_TARGET_DOOR")
        self.assertTrue(manager.controller.turn_commands)

        self.assertEqual(task.tick(manager, (2.60, 2.60, math.pi)), RUNNING)
        self.assertEqual(task.phase, "ALIGN_TO_TARGET_DOOR")
        self.assertEqual(task.tick(manager, (2.60, 2.60, math.pi)), RUNNING)
        self.assertEqual(task.phase, "CALL_ELEVATOR_TO_TARGET_FLOOR")
        self.assertEqual(manager.controller.paths, [])

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
            status = task.tick(manager, (2.60, 2.60, math.pi))

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

        self.assertEqual(task.tick(manager, (2.60, 2.60, math.pi)), RUNNING)

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
        self.assertEqual(service_calls, [("elevator_main", 1, False)])

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
            "danger_search.navigation_tasks.rospy.logwarn_throttle"
        ):
            status = task.tick(manager, (2.60, 2.60, math.pi))

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

        status = task.tick(manager, (0.0, 0.0, math.pi / 2.0))

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
            status = task.tick(manager, (0.0, 0.0, math.pi / 2.0))

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

        status = task.tick(manager, (0.0, 0.0, math.pi / 2.0))

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
            first = task.tick(manager, (0.85, 2.60, 0.0))
            second = task.tick(manager, (0.85, 2.60, 0.0))

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

        status = task.tick(manager, (0.85, 2.60, 0.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "OPEN_CURRENT_FLOOR_DOOR")
        self.assertEqual(open_calls, [])
        self.assertEqual(stop_calls, [True])


if __name__ == "__main__":
    unittest.main()

import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.navigation_tasks import DONE, FAILED, OpenDoorAndPass, RUNNING, TakeElevator
from danger_search.mapper import MultiFloorGridMap


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

        with patch("danger_search.navigation_tasks.astar_path", return_value=[]):
            status = task.tick(manager, (0.0, 0.8, math.pi / 2.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(recorded_paths, [])
        self.assertEqual(cleared_paths, [True])
        self.assertEqual(cleared_doors, ["main_entrance"])


class TakeElevatorTest(unittest.TestCase):
    def test_elevator_service_failure_returns_failed(self):
        failures = []
        task = TakeElevator(0, 1)
        task.phase = "CLOSE_TARGET_FLOOR_DOOR"
        task._service_task = _FailedService("jammed")
        task._retry_count = 1
        manager = SimpleNamespace(
            building_client=SimpleNamespace(
                get_elevator_points=lambda elevator_id, floor: ((0.0, 0.0), (0.0, 1.0)),
                close_door=lambda door_id: _FailedService("jammed"),
            ),
            controller=SimpleNamespace(stop=lambda: None),
            _record_failure=lambda reason: failures.append(reason),
        )

        status = task.tick(manager, (0.0, 0.0, math.pi / 2.0))

        self.assertEqual(status, FAILED)
        self.assertEqual(failures, ["电梯服务失败: jammed"])

    def test_wait_current_door_open_does_not_enter_without_path_after_timeout(self):
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
            ),
            mapper=SimpleNamespace(floor=lambda floor_index: grid),
            current_floor=0,
            controller=SimpleNamespace(stop=lambda: None),
            clear_opened_door_area=lambda door_id: cleared.append(door_id),
        )

        with patch("danger_search.navigation_tasks.astar_path", return_value=[]), patch(
            "danger_search.navigation_tasks.get_time",
            return_value=11.0,
        ):
            status = task.tick(manager, (0.0, 0.0, math.pi / 2.0))

        self.assertEqual(status, RUNNING)
        self.assertEqual(task.phase, "WAIT_CURRENT_DOOR_OPEN")
        self.assertEqual(cleared, ["elevator_floor_0"])


if __name__ == "__main__":
    unittest.main()

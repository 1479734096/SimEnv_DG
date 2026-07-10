"""导航任务状态机封装。"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

try:
    import rospy
except ImportError:
    rospy = None

from danger_search.navigator import DONE, IDLE, RUNNING, STUCK
from danger_search.planner import UNREACHABLE, astar_plan
from .time_utils import get_time

if TYPE_CHECKING:
    from danger_search.mission_manager import MissionManager
    from danger_search.building_control_client import AsyncServiceCall


FAILED = "failed"


class NavigationTask:
    def __init__(self, name: str):
        self.name = name

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        raise NotImplementedError


class NavigateToPose(NavigationTask):
    def __init__(self, target_xy: tuple[float, float], tolerance: float | None = None):
        super().__init__(f"NavigateToPose({target_xy[0]:.2f}, {target_xy[1]:.2f})")
        self.target_xy = target_xy
        self.tolerance = tolerance

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        tol = self.tolerance if self.tolerance is not None else manager.controller.goal_tolerance
        if math.hypot(self.target_xy[0] - pose[0], self.target_xy[1] - pose[1]) <= tol:
            manager.controller.clear_path()
            return DONE

        if not manager.controller.has_path():
            grid = manager.mapper.floor(manager.current_floor)
            result = astar_plan(grid, (pose[0], pose[1]), self.target_xy)
            if result.reached:
                manager.controller.set_path(result.path)
            else:
                # 不可达或预算耗尽都退化为直连，由后续 STUCK/障碍检测兜底。
                path = [(pose[0], pose[1]), self.target_xy]
                manager.controller.set_path(path)

        status = manager.controller.tick(pose)
        if status == STUCK:
            manager.controller.recover()
            return RUNNING  # 仍在恢复中
        return status


class OpenDoorAndPass(NavigationTask):
    def __init__(self, door_id: str, *, pass_offset: float = 0.8, commit_distance: float = 1.8):
        super().__init__(f"OpenDoorAndPass({door_id})")
        self.door_id = door_id
        self.pass_offset = float(pass_offset)
        self.commit_distance = float(commit_distance)
        self.phase = "GO_WAIT_POINT"
        self._wait_point: tuple[float, float] | None = None
        self._pass_point: tuple[float, float] | None = None
        self._commit_point: tuple[float, float] | None = None
        self._service_task: AsyncServiceCall | None = None
        self._retry_count = 0
        self._last_navigation_blocked = False
        self._commit_retry_count = 0
        self._commit_retry_start_time: float | None = None
        self._commit_retry_limit = 30
        self._commit_retry_timeout = 12.0

    @property
    def wait_point(self) -> tuple[float, float] | None:
        return self._wait_point

    @property
    def pass_point(self) -> tuple[float, float] | None:
        return self._pass_point

    @property
    def commit_point(self) -> tuple[float, float] | None:
        return self._commit_point

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        client = manager.building_client
        
        if self._wait_point is None or self._pass_point is None or self._commit_point is None:
            try:
                self._wait_point, self._pass_point, self._commit_point = client.get_door_path_points(
                    self.door_id,
                    (pose[0], pose[1]),
                    pass_offset=self.pass_offset,
                    commit_distance=self.commit_distance,
                )
            except ValueError as e:
                rospy.logerr(f"OpenDoorAndPass 失败: {e}")
                return FAILED

        if self.phase == "GO_WAIT_POINT":
            if self._navigate_to(manager, pose, self._wait_point):
                self.phase = "CALL_OPEN"
            return RUNNING

        if self.phase == "CALL_OPEN":
            status = self._run_service(manager, lambda: client.open_door(self.door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                _clear_opened_door_area(manager, self.door_id)
                self.phase = "GO_PASS_POINT"
            return RUNNING

        if self.phase == "GO_PASS_POINT":
            if self._navigate_to(manager, pose, self._pass_point):
                self.phase = "GO_COMMIT_POINT"
            return RUNNING

        if self.phase == "GO_COMMIT_POINT":
            advanced = self._navigate_to(
                manager,
                pose,
                self._commit_point,
                allow_direct_fallback=True,
            )
            if advanced:
                if rospy is not None and self._wait_point is not None and self._pass_point is not None:
                    rospy.loginfo(
                        "[%s] 完成过门：door_id=%s wait_point=(%.2f, %.2f) pass_point=(%.2f, %.2f) commit_point=(%.2f, %.2f) pose=(%.2f, %.2f, %.2f)",
                        self.name,
                        self.door_id,
                        self._wait_point[0],
                        self._wait_point[1],
                        self._pass_point[0],
                        self._pass_point[1],
                        self._commit_point[0],
                        self._commit_point[1],
                        pose[0],
                        pose[1],
                        pose[2],
                    )
                self.phase = "DONE"
                return DONE
            if self._last_navigation_blocked:
                return self._handle_commit_blocked(manager)
            self._commit_retry_count = 0
            self._commit_retry_start_time = None
            return RUNNING

        return DONE

    def _navigate_to(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        target_xy: tuple[float, float],
        *,
        allow_direct_fallback: bool = False,
    ) -> bool:
        self._last_navigation_blocked = False
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= manager.controller.goal_tolerance:
            manager.controller.clear_path()
            return True

        if not manager.controller.has_path():
            grid = manager.mapper.floor(manager.current_floor)
            result = astar_plan(grid, (pose[0], pose[1]), target_xy)
            if result.reached:
                manager.controller.set_path(result.path)
            elif result.unreachable:
                if allow_direct_fallback and not _segment_crosses_blocked_cell(grid, (pose[0], pose[1]), target_xy):
                    manager.controller.set_path([(pose[0], pose[1]), target_xy])
                else:
                    self._last_navigation_blocked = True
                    return False
            else:
                # 预算耗尽：可能可达，不退化为直连也不标记阻塞，等下轮重新规划。
                return False

        status = manager.controller.tick(pose)
        if status == STUCK:
            manager.controller.recover()
        return status == DONE

    def _handle_commit_blocked(self, manager: MissionManager) -> str:
        _clear_opened_door_area(manager, self.door_id)
        manager.controller.clear_path()
        now = get_time()
        if self._commit_retry_start_time is None:
            self._commit_retry_start_time = now
        self._commit_retry_count += 1
        if rospy is not None:
            rospy.logwarn(
                "[%s] 过门 commit 暂无可达路径，清理门区后重试 (%d/%d)",
                self.name,
                self._commit_retry_count,
                self._commit_retry_limit,
            )
        if (
            self._commit_retry_count >= self._commit_retry_limit
            or now - self._commit_retry_start_time >= self._commit_retry_timeout
        ):
            record_failure = getattr(manager, "_record_failure", None)
            if callable(record_failure):
                record_failure("过门 commit 无可达路径")
            return FAILED
        return RUNNING

    def _run_service(self, manager: MissionManager, task_factory) -> str:
        if self._service_task is None:
            self._service_task = task_factory()
            return RUNNING

        if not self._service_task.done():
            manager.controller.stop()
            return RUNNING

        ok = self._service_task.ok()
        message = self._service_task.message()
        if not ok:
            if rospy is not None:
                rospy.logwarn(f"[{self.name}] 服务失败：{message}")
            if self._retry_count < 1:
                self._retry_count += 1
                if rospy is not None:
                    rospy.loginfo(f"[{self.name}] 正在重试 ({self._retry_count}/1)...")
                self._service_task = None
                return RUNNING
            manager._record_failure(f"开门服务失败: {message}")
            return FAILED

        self._service_task = None
        self._retry_count = 0
        return DONE


def _segment_crosses_blocked_cell(
    grid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
) -> bool:
    start_cell = grid.world_to_grid(start_xy[0], start_xy[1])
    end_cell = grid.world_to_grid(end_xy[0], end_xy[1])
    if start_cell is None or end_cell is None:
        return True
    distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    step_count = max(1, int(math.ceil(distance / max(grid.resolution * 0.5, 1e-6))))
    for index in range(1, step_count + 1):
        ratio = index / step_count
        x = start_xy[0] + (end_xy[0] - start_xy[0]) * ratio
        y = start_xy[1] + (end_xy[1] - start_xy[1]) * ratio
        cell = grid.world_to_grid(x, y)
        if cell is None:
            return True
        gx, gy = cell
        if int(grid.data[gy, gx]) in (80, 100):
            return True
    return False


def _clear_opened_door_area(manager: MissionManager, door_id: str) -> None:
    clear_fn = getattr(manager, "clear_opened_door_area", None)
    if callable(clear_fn):
        clear_fn(door_id)


class TakeElevator(NavigationTask):
    def __init__(self, current_floor: int, target_floor: int, elevator_id: str = "elevator_main"):
        super().__init__(f"TakeElevator({current_floor}->{target_floor})")
        self.current_floor = current_floor
        self.target_floor = target_floor
        self.elevator_id = elevator_id
        self.phase = "GO_TO_ELEVATOR_DOOR"
        self._service_task: AsyncServiceCall | None = None
        self._retry_count = 0
        self._wait_start_time = 0.0

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        client = manager.building_client
        try:
            current_wait, cabin_center = client.get_elevator_points(self.elevator_id, self.current_floor)
            target_wait, _ = client.get_elevator_points(self.elevator_id, self.target_floor)
        except ValueError as e:
            rospy.logerr(f"TakeElevator 失败: {e}")
            return FAILED
            
        current_door_id = f"elevator_floor_{self.current_floor}"
        target_door_id = f"elevator_floor_{self.target_floor}"

        if self.phase == "GO_TO_ELEVATOR_DOOR":
            if self._navigate_to(manager, pose, current_wait):
                self.phase = "CALL_ELEVATOR_TO_CURRENT_FLOOR"
            return RUNNING

        if self.phase == "CALL_ELEVATOR_TO_CURRENT_FLOOR":
            status = self._run_service(manager, lambda: client.call_elevator(self.elevator_id, self.current_floor, False))
            if status == FAILED:
                return FAILED
            if status == DONE:
                self.phase = "OPEN_CURRENT_FLOOR_DOOR"
            return RUNNING

        if self.phase == "OPEN_CURRENT_FLOOR_DOOR":
            status = self._run_service(manager, lambda: client.open_door(current_door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                _clear_opened_door_area(manager, current_door_id)
                self.phase = "WAIT_CURRENT_DOOR_OPEN"
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "WAIT_CURRENT_DOOR_OPEN":
            manager.controller.stop()
            grid = manager.mapper.floor(manager.current_floor)
            cabin_result = astar_plan(grid, (pose[0], pose[1]), cabin_center)
            if cabin_result.reached:
                self.phase = "ENTER_CABIN"
            elif get_time() - self._wait_start_time > 10.0:
                if rospy is not None:
                    rospy.logwarn("[%s] 等待当前楼层电梯门可通行超时，继续等待地图连通", self.name)
                _clear_opened_door_area(manager, current_door_id)
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "ENTER_CABIN":
            status = self._navigate_to_with_status(manager, pose, cabin_center)
            if status == DONE:
                self.phase = "CLOSE_CURRENT_FLOOR_DOOR"
                self._retry_count = 0
            elif status == STUCK:
                if self._retry_count < 1:
                    self._retry_count += 1
                    rospy.logwarn(f"[{self.name}] 进入轿厢卡住，后退并重试...")
                    manager.controller.recover()
                else:
                    rospy.logerr(f"[{self.name}] 进入轿厢失败。")
                    return FAILED
            return RUNNING

        if self.phase == "CLOSE_CURRENT_FLOOR_DOOR":
            status = self._run_service(manager, lambda: client.close_door(current_door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                self.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
            return RUNNING

        if self.phase == "CALL_ELEVATOR_TO_TARGET_FLOOR":
            status = self._run_service(manager, lambda: client.call_elevator(self.elevator_id, self.target_floor, False))
            if status == FAILED:
                return FAILED
            if status == DONE:
                self.phase = "WAIT_FOR_ODOM_STABLE"
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "WAIT_FOR_ODOM_STABLE":
            # 电梯到目标楼层后定位跳变：等待稳定后再继续
            manager.controller.stop()
            if get_time() - self._wait_start_time > 2.0:
                self.phase = "OPEN_TARGET_FLOOR_DOOR"
                # 更新当前楼层
                manager.current_floor = self.target_floor
                manager.floor_start_time = get_time()
            return RUNNING

        if self.phase == "OPEN_TARGET_FLOOR_DOOR":
            status = self._run_service(manager, lambda: client.open_door(target_door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                _clear_opened_door_area(manager, target_door_id)
                self.phase = "WAIT_TARGET_DOOR_OPEN"
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "WAIT_TARGET_DOOR_OPEN":
            manager.controller.stop()
            grid = manager.mapper.floor(manager.target_floor if hasattr(manager, 'target_floor') and manager.target_floor is not None else self.target_floor)
            wait_result = astar_plan(grid, (pose[0], pose[1]), target_wait)
            if wait_result.reached:
                self.phase = "EXIT_CABIN"
            elif get_time() - self._wait_start_time > 10.0:
                if rospy is not None:
                    rospy.logwarn("[%s] 等待目标楼层电梯门可通行超时，继续等待地图连通", self.name)
                _clear_opened_door_area(manager, target_door_id)
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "EXIT_CABIN":
            status = self._navigate_to_with_status(manager, pose, target_wait)
            if status == DONE:
                self.phase = "CLOSE_TARGET_FLOOR_DOOR"
                self._retry_count = 0
            elif status == STUCK:
                if self._retry_count < 1:
                    self._retry_count += 1
                    rospy.logwarn(f"[{self.name}] 离开轿厢卡住，后退并重试...")
                    manager.controller.recover()
                else:
                    rospy.logerr(f"[{self.name}] 离开轿厢失败。")
                    return FAILED
            return RUNNING

        if self.phase == "CLOSE_TARGET_FLOOR_DOOR":
            status = self._run_service(manager, lambda: client.close_door(target_door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                self.phase = "DONE"
                return DONE
            return RUNNING

        return DONE

    def _navigate_to_with_status(self, manager: MissionManager, pose: tuple[float, float, float], target_xy: tuple[float, float]) -> str:
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= manager.controller.goal_tolerance:
            manager.controller.clear_path()
            return DONE

        if not manager.controller.has_path():
            grid = manager.mapper.floor(manager.current_floor)
            result = astar_plan(grid, (pose[0], pose[1]), target_xy)
            if result.reached:
                manager.controller.set_path(result.path)
            else:
                # 不可达或预算耗尽：停止并等待下轮重新规划。
                manager.controller.stop()
                return RUNNING

        return manager.controller.tick(pose)

    def _navigate_to(self, manager: MissionManager, pose: tuple[float, float, float], target_xy: tuple[float, float]) -> bool:
        status = self._navigate_to_with_status(manager, pose, target_xy)
        if status == STUCK:
            manager.controller.recover()
        return status == DONE

    def _run_service(self, manager: MissionManager, task_factory) -> str:
        if self._service_task is None:
            self._service_task = task_factory()
            return RUNNING

        if not self._service_task.done():
            manager.controller.stop()
            return RUNNING

        ok = self._service_task.ok()
        message = self._service_task.message()
        if not ok:
            if rospy is not None:
                rospy.logwarn(f"[{self.name}] 服务失败：{message}")
            if self._retry_count < 1:
                self._retry_count += 1
                if rospy is not None:
                    rospy.loginfo(f"[{self.name}] 正在重试 ({self._retry_count}/1)...")
                self._service_task = None
                return RUNNING
            # 重试失败
            manager._record_failure(f"电梯服务失败: {message}")
            return FAILED

        self._service_task = None
        self._retry_count = 0
        return DONE

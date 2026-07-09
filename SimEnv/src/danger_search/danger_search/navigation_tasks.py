"""导航任务状态机封装。"""
from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

try:
    import rospy
except ImportError:
    rospy = None

from danger_search.navigator import DONE, IDLE, RUNNING, STUCK
from danger_search.planner import astar_path

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
            path = astar_path(grid, (pose[0], pose[1]), self.target_xy)
            if not path:
                path = [(pose[0], pose[1]), self.target_xy]
            manager.controller.set_path(path)

        status = manager.controller.tick(pose)
        if status == STUCK:
            manager.controller.recover()
            return RUNNING  # 仍在恢复中
        return status


class OpenDoorAndPass(NavigationTask):
    def __init__(self, door_id: str):
        super().__init__(f"OpenDoorAndPass({door_id})")
        self.door_id = door_id
        self.phase = "GO_WAIT_POINT"
        self._wait_point: tuple[float, float] | None = None
        self._pass_point: tuple[float, float] | None = None
        self._service_task: AsyncServiceCall | None = None
        self._retry_count = 0

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        client = manager.building_client
        
        if self._wait_point is None or self._pass_point is None:
            try:
                self._wait_point, self._pass_point = client.get_door_points(self.door_id, (pose[0], pose[1]))
            except ValueError as e:
                rospy.logerr(f"OpenDoorAndPass 失败: {e}")
                return FAILED

        if self.phase == "GO_WAIT_POINT":
            if self._navigate_to(manager, pose, self._wait_point):
                self.phase = "CALL_OPEN"
            return RUNNING

        if self.phase == "CALL_OPEN":
            if self._run_service(manager, lambda: client.open_door(self.door_id)):
                self.phase = "GO_PASS_POINT"
            return RUNNING

        if self.phase == "GO_PASS_POINT":
            if self._navigate_to(manager, pose, self._pass_point):
                self.phase = "DONE"
                return DONE
            return RUNNING

        return DONE

    def _navigate_to(self, manager: MissionManager, pose: tuple[float, float, float], target_xy: tuple[float, float]) -> bool:
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= manager.controller.goal_tolerance:
            manager.controller.clear_path()
            return True

        if not manager.controller.has_path():
            grid = manager.mapper.floor(manager.current_floor)
            path = astar_path(grid, (pose[0], pose[1]), target_xy)
            if not path:
                path = [(pose[0], pose[1]), target_xy]
            manager.controller.set_path(path)

        status = manager.controller.tick(pose)
        if status == STUCK:
            manager.controller.recover()
        return status == DONE

    def _run_service(self, manager: MissionManager, task_factory) -> bool:
        if self._service_task is None:
            self._service_task = task_factory()
            return False

        if not self._service_task.done():
            manager.controller.stop()
            return False

        ok = self._service_task.ok()
        message = self._service_task.message()
        if not ok:
            rospy.logwarn(f"[{self.name}] 服务失败：{message}")
            if self._retry_count < 1:
                self._retry_count += 1
                rospy.loginfo(f"[{self.name}] 正在重试 ({self._retry_count}/1)...")
                self._service_task = None
                return False
            manager._record_failure(f"开门服务失败: {message}")
            return False # 实际上应该是返回 FAILED，但这个帮助方法只返回 bool，我们可以在调用处处理

        self._service_task = None
        return True


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
            if self._run_service(manager, lambda: client.call_elevator(self.elevator_id, self.current_floor, False)):
                self.phase = "OPEN_CURRENT_FLOOR_DOOR"
            return RUNNING

        if self.phase == "OPEN_CURRENT_FLOOR_DOOR":
            if self._run_service(manager, lambda: client.open_door(current_door_id)):
                self.phase = "ENTER_CABIN"
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
            if self._run_service(manager, lambda: client.close_door(current_door_id)):
                self.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
            return RUNNING

        if self.phase == "CALL_ELEVATOR_TO_TARGET_FLOOR":
            if self._run_service(manager, lambda: client.call_elevator(self.elevator_id, self.target_floor, False)):
                self.phase = "WAIT_FOR_ODOM_STABLE"
                self._wait_start_time = time.time()
            return RUNNING

        if self.phase == "WAIT_FOR_ODOM_STABLE":
            # 电梯到目标楼层后定位跳变：等待稳定后再继续
            manager.controller.stop()
            if time.time() - self._wait_start_time > 2.0:
                self.phase = "OPEN_TARGET_FLOOR_DOOR"
                # 更新当前楼层
                manager.current_floor = self.target_floor
                manager.floor_start_time = time.time()
            return RUNNING

        if self.phase == "OPEN_TARGET_FLOOR_DOOR":
            if self._run_service(manager, lambda: client.open_door(target_door_id)):
                self.phase = "EXIT_CABIN"
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
            if self._run_service(manager, lambda: client.close_door(target_door_id)):
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
            path = astar_path(grid, (pose[0], pose[1]), target_xy)
            if not path:
                path = [(pose[0], pose[1]), target_xy]
            manager.controller.set_path(path)

        return manager.controller.tick(pose)

    def _navigate_to(self, manager: MissionManager, pose: tuple[float, float, float], target_xy: tuple[float, float]) -> bool:
        status = self._navigate_to_with_status(manager, pose, target_xy)
        if status == STUCK:
            manager.controller.recover()
        return status == DONE

    def _run_service(self, manager: MissionManager, task_factory) -> bool:
        if self._service_task is None:
            self._service_task = task_factory()
            return False

        if not self._service_task.done():
            manager.controller.stop()
            return False

        ok = self._service_task.ok()
        message = self._service_task.message()
        if not ok:
            rospy.logwarn(f"[{self.name}] 服务失败：{message}")
            if self._retry_count < 1:
                self._retry_count += 1
                rospy.loginfo(f"[{self.name}] 正在重试 ({self._retry_count}/1)...")
                self._service_task = None
                return False
            # 重试失败
            manager._record_failure(f"电梯服务失败: {message}")
            return False

        self._service_task = None
        self._retry_count = 0
        return True

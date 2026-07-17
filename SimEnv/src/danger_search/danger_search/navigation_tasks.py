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


def _angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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
    _PASS_ALIGN_TOLERANCE = 0.10
    _PASS_ALIGN_CONFIRMATIONS = 2

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
        self._last_navigation_stuck = False
        self._pass_retry_start_time: float | None = None
        self._pass_retry_timeout = 3.0
        self._pass_alignment_count = 0
        self._pass_path_started = False
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
            return self._tick_active_pass(manager, pose)

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
            if self._last_navigation_stuck:
                return self._fail_crossing(manager, "过门深入阶段有命令但无有效位移")
            if self._last_navigation_blocked:
                return self._handle_commit_blocked(manager)
            self._commit_retry_count = 0
            self._commit_retry_start_time = None
            return RUNNING

        return DONE

    def _tick_active_pass(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
    ) -> str:
        """门洞中心对准后直线通过，并以真实位姿确认完整通过。"""
        if self._pass_point is None:
            return self._fail_crossing(manager, "缺少过门通过点")

        goal_tolerance = max(1e-3, float(manager.controller.goal_tolerance))
        corner_tolerance = float(
            getattr(manager.controller, "corner_tolerance", goal_tolerance)
        )
        if not math.isfinite(corner_tolerance) or corner_tolerance <= 0.0:
            corner_tolerance = goal_tolerance
        crossing_tolerance = min(goal_tolerance, corner_tolerance)

        if not self._pass_path_started:
            target_yaw = math.atan2(
                self._pass_point[1] - pose[1],
                self._pass_point[0] - pose[0],
            )
            yaw_error = _angle_wrap(target_yaw - pose[2])
            if abs(yaw_error) > self._PASS_ALIGN_TOLERANCE:
                self._pass_alignment_count = 0
                angular_limit = abs(float(getattr(manager.controller, "max_angular", 0.0)))
                minimum_speed = abs(
                    float(getattr(manager.controller, "recover_angular_speed", 0.0))
                )
                angular_speed = min(
                    angular_limit,
                    max(abs(1.7 * yaw_error), minimum_speed),
                )
                manager.controller.rotate_in_place(math.copysign(angular_speed, yaw_error))
                return RUNNING

            manager.controller.stop(reason="door_pass_alignment_hold")
            self._pass_alignment_count += 1
            if self._pass_alignment_count < self._PASS_ALIGN_CONFIRMATIONS:
                return RUNNING

            _clear_opened_door_area(manager, self.door_id)
            grid = manager.mapper.floor(manager.current_floor)
            if _segment_crosses_blocked_cell(
                grid,
                (pose[0], pose[1]),
                self._pass_point,
            ):
                return self._handle_pass_blocked(manager)

            manager.controller.set_path([(pose[0], pose[1]), self._pass_point])
            self._pass_path_started = True
            self._pass_retry_start_time = None
            return RUNNING

        endpoint_distance = math.hypot(
            self._pass_point[0] - pose[0],
            self._pass_point[1] - pose[1],
        )
        if endpoint_distance <= crossing_tolerance:
            manager.controller.clear_path(reason="door_pass_completed")
            return self._finish_pass(pose)
        if not manager.controller.has_path():
            return self._fail_crossing(manager, "过门直线路径意外丢失")

        status = manager.controller.tick(pose, goal_tolerance=crossing_tolerance)
        if status == DONE:
            endpoint_distance = math.hypot(
                self._pass_point[0] - pose[0],
                self._pass_point[1] - pose[1],
            )
            if endpoint_distance <= crossing_tolerance:
                return self._finish_pass(pose)
            return self._fail_crossing(manager, "控制器提前结束过门路径")
        if status in (STUCK, IDLE):
            return self._fail_crossing(manager, "过门通过阶段有命令但无有效位移")
        return RUNNING

    def _finish_pass(self, pose: tuple[float, float, float]) -> str:
        if rospy is not None and self._wait_point is not None and self._pass_point is not None:
            rospy.loginfo(
                "[%s] 完成过门：door_id=%s wait_point=(%.2f, %.2f) pass_point=(%.2f, %.2f) commit_point=(%.2f, %.2f) pose=(%.2f, %.2f, %.2f)",
                self.name,
                self.door_id,
                self._wait_point[0],
                self._wait_point[1],
                self._pass_point[0],
                self._pass_point[1],
                self._commit_point[0] if self._commit_point is not None else self._pass_point[0],
                self._commit_point[1] if self._commit_point is not None else self._pass_point[1],
                pose[0],
                pose[1],
                pose[2],
            )
        self.phase = "DONE"
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
        self._last_navigation_stuck = False
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
            if self.phase in ("GO_PASS_POINT", "GO_COMMIT_POINT"):
                self._last_navigation_stuck = True
            else:
                manager.controller.recover()
        return status == DONE

    def _handle_pass_blocked(self, manager: MissionManager) -> str:
        manager.controller.stop(reason="door_pass_blocked")
        now = get_time()
        if self._pass_retry_start_time is None:
            self._pass_retry_start_time = now
            return RUNNING
        if now - self._pass_retry_start_time < self._pass_retry_timeout:
            return RUNNING
        return self._fail_crossing(manager, "过门通过点持续被地图阻塞")

    def _fail_crossing(self, manager: MissionManager, reason: str) -> str:
        manager.controller.clear_path(reason="door_crossing_failed")
        record_failure = getattr(manager, "_record_failure", None)
        if callable(record_failure):
            record_failure(reason)
        if rospy is not None:
            rospy.logwarn("[%s] %s，终止本次过门任务", self.name, reason)
        return FAILED

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
    _CABIN_TRANSIT_ALIGN_TOLERANCE = 0.10
    _CABIN_TRANSIT_ALIGN_CONFIRMATIONS = 2
    _EXIT_BACKOFF_DISTANCE = 0.35
    _EXIT_BACKOFF_ALIGN_TOLERANCE = 0.12
    _EXIT_BACKOFF_ALIGN_TIMEOUT = 4.0
    _EXIT_BACKOFF_TIMEOUT_MARGIN = 1.5

    def __init__(self, current_floor: int, target_floor: int, elevator_id: str = "elevator_main"):
        super().__init__(f"TakeElevator({current_floor}->{target_floor})")
        self.current_floor = current_floor
        self.target_floor = target_floor
        self.elevator_id = elevator_id
        self.phase = "GO_TO_ELEVATOR_DOOR"
        self._service_task: AsyncServiceCall | None = None
        self._retry_count = 0
        self._wait_start_time = 0.0
        self.cabin_transit_stage: str | None = None
        self._cabin_transit_mode: str | None = None
        self._cabin_transit_alignment_count = 0
        self._cabin_transit_start_xy: tuple[float, float] | None = None
        self._cabin_transit_target_xy: tuple[float, float] | None = None
        self._exit_backoff_target_xy: tuple[float, float] | None = None
        self._exit_backoff_start_xy: tuple[float, float] | None = None
        self._exit_backoff_direction: tuple[float, float] | None = None
        self._exit_backoff_exit_yaw: float | None = None
        self._exit_backoff_phase_start = 0.0
        self._exit_backoff_deadline = 0.0

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        client = manager.building_client
        try:
            current_wait, cabin_center = client.get_elevator_points(self.elevator_id, self.current_floor)
            target_wait, _ = client.get_elevator_points(self.elevator_id, self.target_floor)
        except ValueError as e:
            rospy.logerr(f"TakeElevator 失败: {e}")
            return FAILED
            
        try:
            current_door_id = client.elevator_door_id(self.current_floor)
            target_door_id = client.elevator_door_id(self.target_floor)
        except ValueError as e:
            rospy.logerr(f"TakeElevator 失败: {e}")
            return FAILED

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
            if cabin_result.reached and not _segment_crosses_blocked_cell(
                grid,
                current_wait,
                cabin_center,
            ):
                self._reset_cabin_transit()
                self.phase = "ENTER_CABIN"
            elif get_time() - self._wait_start_time > 10.0:
                if rospy is not None:
                    rospy.logwarn("[%s] 等待当前楼层电梯门可通行超时，继续等待地图连通", self.name)
                _clear_opened_door_area(manager, current_door_id)
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "ENTER_CABIN":
            status = self._tick_cabin_transit(
                manager,
                pose,
                mode="enter",
                position_xy=current_wait,
                target_xy=cabin_center,
            )
            if status == DONE:
                self._reset_cabin_transit()
                self.phase = "CLOSE_CURRENT_FLOOR_DOOR"
                self._retry_count = 0
            elif status == STUCK:
                if self.cabin_transit_stage == "POSITION" and self._retry_count < 1:
                    self._retry_count += 1
                    manager.controller.recover(reason="elevator_enter_position_stuck")
                    return RUNNING
                if rospy is not None:
                    rospy.logerr(
                        "[%s] 进入轿厢主动跨越失败：stage=%s",
                        self.name,
                        self.cabin_transit_stage,
                    )
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
            if wait_result.reached and not _segment_crosses_blocked_cell(
                grid,
                cabin_center,
                target_wait,
            ):
                self._reset_cabin_transit()
                self.phase = "EXIT_CABIN"
            elif get_time() - self._wait_start_time > 10.0:
                if rospy is not None:
                    rospy.logwarn("[%s] 等待目标楼层电梯门可通行超时，继续等待地图连通", self.name)
                _clear_opened_door_area(manager, target_door_id)
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "EXIT_CABIN":
            exit_linear_cap = (
                getattr(manager, "exit_first_max_linear", None)
                if self._retry_count == 0
                else None
            )
            status = self._tick_cabin_transit(
                manager,
                pose,
                mode="exit",
                position_xy=cabin_center,
                target_xy=target_wait,
                max_linear=exit_linear_cap,
            )
            if status == DONE:
                self._reset_cabin_transit()
                self.phase = "CLOSE_TARGET_FLOOR_DOOR"
                self._retry_count = 0
            elif status == STUCK:
                if self._retry_count < 1:
                    if self.cabin_transit_stage != "CROSS":
                        if rospy is not None:
                            rospy.logerr(
                                "[%s] 离开轿厢准备阶段失败：stage=%s",
                                self.name,
                                self.cabin_transit_stage,
                            )
                        return FAILED
                    if not bool(getattr(manager, "exit_cabin_backoff_enabled", False)):
                        if rospy is not None:
                            rospy.logerr(
                                f"[{self.name}] 离开轿厢直线跨越卡住，默认禁用已证伪的倒车恢复。"
                            )
                        return FAILED
                    if not self._begin_exit_backoff(
                        manager,
                        pose,
                        cabin_center,
                        target_wait,
                    ):
                        if rospy is not None:
                            rospy.logerr(f"[{self.name}] 轿厢内倒车路径不安全，终止离厢重试。")
                        return FAILED
                else:
                    if rospy is not None:
                        rospy.logerr(f"[{self.name}] 离开轿厢失败。")
                    return FAILED
            return RUNNING

        if self.phase.startswith("EXIT_BACKOFF_"):
            return self._tick_exit_backoff(manager, pose, cabin_center)

        if self.phase == "CLOSE_TARGET_FLOOR_DOOR":
            status = self._run_service(manager, lambda: client.close_door(target_door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                self.phase = "DONE"
                return DONE
            return RUNNING

        return DONE

    def _tick_cabin_transit(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        *,
        mode: str,
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
        max_linear: float | None = None,
    ) -> str:
        if self._cabin_transit_mode != mode or self.cabin_transit_stage is None:
            self._begin_cabin_transit(mode, position_xy, target_xy)

        position_tolerance = self._cabin_transit_position_tolerance(manager)

        if self.cabin_transit_stage == "POSITION":
            status = self._navigate_to_with_status(
                manager,
                pose,
                position_xy,
                goal_tolerance=position_tolerance,
            )
            if status == DONE:
                self.cabin_transit_stage = "ALIGN"
                self._cabin_transit_alignment_count = 0
                self._log_cabin_transit_stage(
                    "ALIGN",
                    pose,
                    position_xy,
                    target_xy,
                )
                return RUNNING
            return status

        if self.cabin_transit_stage == "ALIGN":
            position_error = math.hypot(position_xy[0] - pose[0], position_xy[1] - pose[1])
            if position_error > position_tolerance:
                manager.controller.clear_path(reason="elevator_transit_position_drift")
                self.cabin_transit_stage = "POSITION"
                self._cabin_transit_alignment_count = 0
                self._log_cabin_transit_stage(
                    "POSITION",
                    pose,
                    position_xy,
                    target_xy,
                )
                return RUNNING

            target_dx = target_xy[0] - pose[0]
            target_dy = target_xy[1] - pose[1]
            if math.hypot(target_dx, target_dy) <= position_tolerance:
                manager.controller.clear_path(reason="elevator_transit_already_complete")
                return DONE

            target_yaw = math.atan2(target_dy, target_dx)
            yaw_error = _angle_wrap(target_yaw - pose[2])
            if abs(yaw_error) > self._CABIN_TRANSIT_ALIGN_TOLERANCE:
                self._cabin_transit_alignment_count = 0
                proportional_speed = abs(1.7 * yaw_error)
                minimum_speed = abs(
                    float(getattr(manager.controller, "recover_angular_speed", 0.0))
                )
                angular_limit = abs(float(getattr(manager.controller, "max_angular", 0.0)))
                # A1 的纯转向在小角速度下存在执行死区；复用已验证的恢复转速，
                # 避免对准阶段在阈值外持续发布无效命令。
                angular_speed = min(
                    angular_limit,
                    max(proportional_speed, minimum_speed),
                )
                manager.controller.rotate_in_place(math.copysign(angular_speed, yaw_error))
                return RUNNING

            manager.controller.stop(reason="elevator_transit_alignment_hold")
            self._cabin_transit_alignment_count += 1
            if self._cabin_transit_alignment_count < self._CABIN_TRANSIT_ALIGN_CONFIRMATIONS:
                return RUNNING

            grid = manager.mapper.floor(manager.current_floor)
            commit_start = (pose[0], pose[1])
            if _segment_crosses_blocked_cell(grid, commit_start, target_xy):
                self._cabin_transit_alignment_count = 0
                if rospy is not None:
                    rospy.logwarn_throttle(
                        2.0,
                        "[%s] 电梯直线跨越路径仍被地图阻塞，保持原地等待",
                        self.name,
                    )
                return RUNNING

            self._cabin_transit_start_xy = commit_start
            self._cabin_transit_target_xy = target_xy
            manager.controller.set_path([commit_start, target_xy])
            self.cabin_transit_stage = "CROSS"
            self._log_cabin_transit_stage(
                "CROSS",
                pose,
                commit_start,
                target_xy,
                yaw_error=yaw_error,
            )
            return RUNNING

        if self.cabin_transit_stage == "CROSS":
            start_xy = self._cabin_transit_start_xy
            end_xy = self._cabin_transit_target_xy
            if start_xy is None or end_xy is None:
                manager.controller.stop(reason="elevator_transit_missing_commit")
                return STUCK

            complete, progress, cross_track, endpoint_distance = self._cabin_transit_complete(
                pose,
                start_xy,
                end_xy,
                position_tolerance,
            )
            if complete:
                manager.controller.clear_path(reason="elevator_transit_completed")
                if rospy is not None:
                    rospy.loginfo(
                        "[%s] 电梯主动跨越完成：mode=%s progress=%.3f cross_track=%.3f endpoint_distance=%.3f",
                        self.name,
                        self._cabin_transit_mode,
                        progress,
                        cross_track,
                        endpoint_distance,
                    )
                return DONE

            if not manager.controller.has_path():
                manager.controller.stop(reason="elevator_transit_path_lost")
                return STUCK

            tick_kwargs = {"goal_tolerance": position_tolerance}
            if max_linear is not None:
                tick_kwargs["max_linear"] = max_linear
            status = manager.controller.tick(pose, **tick_kwargs)
            if status == DONE:
                complete, progress, cross_track, endpoint_distance = self._cabin_transit_complete(
                    pose,
                    start_xy,
                    end_xy,
                    position_tolerance,
                )
                if complete:
                    return DONE
                manager.controller.stop(reason="elevator_transit_incomplete_goal")
                return STUCK
            if status == IDLE:
                manager.controller.stop(reason="elevator_transit_unexpected_idle")
                return STUCK
            return status

        manager.controller.stop(reason="elevator_transit_unknown_stage")
        return STUCK

    def _begin_cabin_transit(
        self,
        mode: str,
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
    ) -> None:
        self._cabin_transit_mode = mode
        self.cabin_transit_stage = "POSITION"
        self._cabin_transit_alignment_count = 0
        self._cabin_transit_start_xy = None
        self._cabin_transit_target_xy = target_xy
        self._log_cabin_transit_stage(
            "POSITION",
            None,
            position_xy,
            target_xy,
        )

    def _reset_cabin_transit(self) -> None:
        self.cabin_transit_stage = None
        self._cabin_transit_mode = None
        self._cabin_transit_alignment_count = 0
        self._cabin_transit_start_xy = None
        self._cabin_transit_target_xy = None

    @staticmethod
    def _cabin_transit_position_tolerance(manager: MissionManager) -> float:
        goal_tolerance = max(1e-3, float(manager.controller.goal_tolerance))
        corner_tolerance = float(
            getattr(manager.controller, "corner_tolerance", goal_tolerance)
        )
        if not math.isfinite(corner_tolerance) or corner_tolerance <= 0.0:
            corner_tolerance = goal_tolerance
        return min(goal_tolerance, corner_tolerance)

    @staticmethod
    def _cabin_transit_complete(
        pose: tuple[float, float, float],
        start_xy: tuple[float, float],
        target_xy: tuple[float, float],
        tolerance: float,
    ) -> tuple[bool, float, float, float]:
        segment_dx = target_xy[0] - start_xy[0]
        segment_dy = target_xy[1] - start_xy[1]
        segment_length = math.hypot(segment_dx, segment_dy)
        endpoint_distance = math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1])
        if segment_length <= 1e-6:
            return endpoint_distance <= tolerance, 0.0, endpoint_distance, endpoint_distance

        unit_x = segment_dx / segment_length
        unit_y = segment_dy / segment_length
        pose_dx = pose[0] - start_xy[0]
        pose_dy = pose[1] - start_xy[1]
        progress = pose_dx * unit_x + pose_dy * unit_y
        cross_track = abs(pose_dx * unit_y - pose_dy * unit_x)
        complete = (
            endpoint_distance <= tolerance
            and progress >= segment_length - tolerance
            and cross_track <= tolerance
        )
        return complete, progress, cross_track, endpoint_distance

    def _log_cabin_transit_stage(
        self,
        stage: str,
        pose: tuple[float, float, float] | None,
        start_xy: tuple[float, float],
        target_xy: tuple[float, float],
        *,
        yaw_error: float | None = None,
    ) -> None:
        if rospy is None:
            return
        pose_text = "none" if pose is None else "(%.3f,%.3f,%.3f)" % pose
        yaw_text = "none" if yaw_error is None else f"{yaw_error:.3f}"
        rospy.loginfo(
            "[%s] 电梯主动跨越阶段：mode=%s stage=%s pose=%s start=(%.3f,%.3f) target=(%.3f,%.3f) yaw_error=%s",
            self.name,
            self._cabin_transit_mode,
            stage,
            pose_text,
            start_xy[0],
            start_xy[1],
            target_xy[0],
            target_xy[1],
            yaw_text,
        )

    def _begin_exit_backoff(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        cabin_center: tuple[float, float],
        target_wait: tuple[float, float],
    ) -> bool:
        if not self._prepare_exit_backoff_segment(manager, pose, cabin_center):
            return False
        reverse_speed = float(getattr(manager.controller, "reverse_speed", 0.0))
        if reverse_speed >= -1e-4:
            return False
        exit_dx = target_wait[0] - cabin_center[0]
        exit_dy = target_wait[1] - cabin_center[1]
        if math.hypot(exit_dx, exit_dy) <= 1e-6:
            return False
        self._exit_backoff_exit_yaw = math.atan2(exit_dy, exit_dx)
        self._exit_backoff_phase_start = get_time()
        self._retry_count += 1
        self.phase = "EXIT_BACKOFF_SETTLE"
        manager.controller.clear_path(reason="exit_backoff_start")
        manager.controller.stop(reason="exit_backoff_settle")
        if rospy is not None:
            rospy.logwarn(
                "[%s] 离开轿厢卡住，开始安全倒车：start=(%.2f, %.2f) target=(%.2f, %.2f)",
                self.name,
                pose[0],
                pose[1],
                self._exit_backoff_target_xy[0],
                self._exit_backoff_target_xy[1],
            )
        return True

    def _prepare_exit_backoff_segment(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        cabin_center: tuple[float, float],
    ) -> bool:
        dx = cabin_center[0] - pose[0]
        dy = cabin_center[1] - pose[1]
        cabin_distance = math.hypot(dx, dy)
        if cabin_distance <= 1e-6:
            return False
        backoff_distance = min(self._EXIT_BACKOFF_DISTANCE, cabin_distance)
        direction = (dx / cabin_distance, dy / cabin_distance)
        target_xy = (
            pose[0] + direction[0] * backoff_distance,
            pose[1] + direction[1] * backoff_distance,
        )
        grid = manager.mapper.floor(manager.current_floor)
        if _segment_crosses_blocked_cell(grid, (pose[0], pose[1]), target_xy):
            return False
        self._exit_backoff_start_xy = (pose[0], pose[1])
        self._exit_backoff_target_xy = target_xy
        self._exit_backoff_direction = direction
        return True

    def _tick_exit_backoff(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        cabin_center: tuple[float, float],
    ) -> str:
        now = get_time()
        if self.phase == "EXIT_BACKOFF_SETTLE":
            manager.controller.stop(reason="exit_backoff_settle")
            settle_duration = max(0.0, float(getattr(manager.controller, "recover_duration", 0.5)))
            if now - self._exit_backoff_phase_start >= settle_duration:
                self.phase = "EXIT_BACKOFF_ALIGN"
                self._exit_backoff_phase_start = now
            return RUNNING

        if self.phase in {"EXIT_BACKOFF_ALIGN", "EXIT_BACKOFF_REALIGN"}:
            if self._exit_backoff_exit_yaw is None:
                return self._fail_exit_backoff(manager, "缺少出口方向")
            heading_error = _angle_wrap(self._exit_backoff_exit_yaw - pose[2])
            if abs(heading_error) <= self._EXIT_BACKOFF_ALIGN_TOLERANCE:
                manager.controller.stop(reason="exit_backoff_aligned")
                if self.phase == "EXIT_BACKOFF_REALIGN":
                    manager.controller.clear_path(reason="exit_backoff_complete")
                    self._reset_cabin_transit()
                    self.phase = "EXIT_CABIN"
                    if rospy is not None:
                        rospy.loginfo(f"[{self.name}] 轿厢内倒车与出口重对准完成，重试离厢。")
                    return RUNNING
                if not self._prepare_exit_backoff_segment(manager, pose, cabin_center):
                    return self._fail_exit_backoff(manager, "对准后倒车路径不再安全")
                reverse_speed = abs(float(getattr(manager.controller, "reverse_speed", 0.0)))
                if reverse_speed <= 1e-4:
                    return self._fail_exit_backoff(manager, "倒车速度配置无效")
                self._exit_backoff_deadline = (
                    now
                    + self._EXIT_BACKOFF_DISTANCE / reverse_speed
                    + self._EXIT_BACKOFF_TIMEOUT_MARGIN
                )
                self.phase = "EXIT_BACKOFF_REVERSE"
                return RUNNING
            if now - self._exit_backoff_phase_start > self._EXIT_BACKOFF_ALIGN_TIMEOUT:
                return self._fail_exit_backoff(manager, "出口方向对准超时")
            angular_limit = min(0.45, abs(float(getattr(manager.controller, "max_angular", 0.45))))
            manager.controller.rotate_in_place(math.copysign(angular_limit, heading_error))
            return RUNNING

        if self.phase == "EXIT_BACKOFF_REVERSE":
            if (
                self._exit_backoff_start_xy is None
                or self._exit_backoff_target_xy is None
                or self._exit_backoff_direction is None
                or self._exit_backoff_exit_yaw is None
            ):
                return self._fail_exit_backoff(manager, "缺少倒车状态")
            dx = pose[0] - self._exit_backoff_start_xy[0]
            dy = pose[1] - self._exit_backoff_start_xy[1]
            progress = dx * self._exit_backoff_direction[0] + dy * self._exit_backoff_direction[1]
            target_distance = math.hypot(
                self._exit_backoff_target_xy[0] - pose[0],
                self._exit_backoff_target_xy[1] - pose[1],
            )
            if progress >= self._EXIT_BACKOFF_DISTANCE - 0.03 or target_distance <= 0.05:
                manager.controller.stop(reason="exit_backoff_distance_reached")
                self.phase = "EXIT_BACKOFF_REALIGN"
                self._exit_backoff_phase_start = now
                return RUNNING
            if now > self._exit_backoff_deadline:
                return self._fail_exit_backoff(manager, "倒车无有效位移或执行超时")
            if abs(_angle_wrap(self._exit_backoff_exit_yaw - pose[2])) > 0.35:
                return self._fail_exit_backoff(manager, "倒车期间朝向偏差过大")
            if not manager.controller.reverse_straight(reason="exit_cabin_backoff"):
                return self._fail_exit_backoff(manager, "倒车命令未生效")
            return RUNNING

        return self._fail_exit_backoff(manager, f"未知倒车阶段: {self.phase}")

    def _fail_exit_backoff(self, manager: MissionManager, reason: str) -> str:
        manager.controller.stop(reason="exit_backoff_failed")
        if rospy is not None:
            rospy.logerr(f"[{self.name}] 离开轿厢倒车恢复失败：{reason}。")
        return FAILED

    def _navigate_to_with_status(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        target_xy: tuple[float, float],
        *,
        max_linear: float | None = None,
        goal_tolerance: float | None = None,
    ) -> str:
        effective_goal_tolerance = float(manager.controller.goal_tolerance)
        if goal_tolerance is not None:
            effective_goal_tolerance = min(
                effective_goal_tolerance,
                max(0.0, float(goal_tolerance)),
            )
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= effective_goal_tolerance:
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

        if max_linear is None and goal_tolerance is None:
            return manager.controller.tick(pose)
        tick_kwargs = {}
        if max_linear is not None:
            tick_kwargs["max_linear"] = max_linear
        if goal_tolerance is not None:
            tick_kwargs["goal_tolerance"] = effective_goal_tolerance
        return manager.controller.tick(pose, **tick_kwargs)

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

"""普通导航、过门和乘电梯的非阻塞子状态机。

``MissionManager.current_task`` 同一时刻只持有一个任务，每次主 tick 调用一次
任务 ``tick``。任务可以规划路径、轮询异步服务或驱动 ``PathFollower``，但不会
自行循环等待。统一返回 ``RUNNING/DONE/FAILED``（控制器还可返回 STUCK/IDLE），
让顶层状态机清楚地区分仍在执行、成功和失败。
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

try:
    import rospy
except ImportError:
    rospy = None

from danger_search.navigator import DONE, IDLE, RUNNING, STUCK
from danger_search.mapper import BLACK, NEGATIVE, OCC
from danger_search.planner import DEFAULT_MAX_ITERATIONS, UNREACHABLE, astar_plan
from .time_utils import get_time

if TYPE_CHECKING:
    from danger_search.mission_manager import MissionManager
    from danger_search.building_control_client import AsyncServiceCall


FAILED = "failed"


def _angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class NavigationTask:
    """导航子任务最小接口；实现必须让每次 ``tick`` 快速返回。"""
    def __init__(self, name: str):
        self.name = name

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        raise NotImplementedError


class NavigateToPose(NavigationTask):
    """导航到一个 world XY 目标，路径丢失时重新规划。"""
    def __init__(self, target_xy: tuple[float, float], tolerance: float | None = None):
        try:
            target = (float(target_xy[0]), float(target_xy[1]))
            resolved_tolerance = None if tolerance is None else float(tolerance)
        except (IndexError, TypeError, ValueError):
            raise ValueError("导航目标和容差必须是有限数") from None
        if (
            not all(math.isfinite(value) for value in target)
            or resolved_tolerance is not None
            and (not math.isfinite(resolved_tolerance) or resolved_tolerance <= 0.0)
        ):
            raise ValueError("导航目标和容差必须是有限正值")
        super().__init__(f"NavigateToPose({target[0]:.2f}, {target[1]:.2f})")
        self.target_xy = target
        self.tolerance = resolved_tolerance

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
            elif result.unreachable:
                manager.controller.stop(reason="navigation_target_unreachable")
                return FAILED
            else:
                # 预算耗尽只表示本轮未完成搜索，不能穿过 OCC/BLACK 直连。
                manager.controller.stop(reason="navigation_budget_exhausted")
                return RUNNING

        status = manager.controller.tick(pose)
        if status == STUCK:
            manager.controller.recover()
            return RUNNING  # 仍在恢复中
        return status


class OpenDoorAndPass(NavigationTask):
    """动态门交互状态机：到等待侧、开门、直线过门并深入门后。

    等待/通过/commit 三点全部来自运行时传感器注册门几何。服务成功后仍由
    ``MissionManager.clear_opened_door_area`` 采用受限门板条带更新地图；真正
    过门阶段先原地对准，再使用两点直线和更严格端点容差，避免 A* 折线切门框。
    """
    _PASS_ALIGN_TOLERANCE = 0.10
    _PASS_ALIGN_CONFIRMATIONS = 2

    def __init__(self, door_id: str, *, pass_offset: float = 0.8, commit_distance: float = 1.8):
        door_id = str(door_id).strip()
        if not door_id:
            raise ValueError("过门任务必须提供公开门 ID")
        if not all(
            math.isfinite(float(value)) and float(value) > 0.0
            for value in (pass_offset, commit_distance)
        ):
            raise ValueError("过门偏移和深入距离必须是正有限数")
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
        self._opening_sensor_confirmed = False
        self._commit_retry_count = 0
        self._commit_retry_start_time: float | None = None
        self._commit_retry_limit = 30
        self._commit_retry_timeout = 12.0
        self._wait_navigation_failure: str | None = None
        self._wait_navigation_blocked_since: float | None = None
        self._wait_navigation_timeout = 3.0

    @property
    def wait_point(self) -> tuple[float, float] | None:
        return self._wait_point

    @property
    def pass_point(self) -> tuple[float, float] | None:
        return self._pass_point

    @property
    def commit_point(self) -> tuple[float, float] | None:
        return self._commit_point

    @property
    def opening_sensor_confirmed(self) -> bool:
        return self._opening_sensor_confirmed

    def mark_opening_sensor_confirmed(self) -> None:
        """记录开放门洞已由近场传感器连续确认。"""
        self._opening_sensor_confirmed = True

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        """按当前 ``phase`` 推进一步，所有服务等待期间保持零速。"""
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
                if rospy is not None:
                    rospy.logerr(f"OpenDoorAndPass 失败: {e}")
                return FAILED

        if self.phase == "GO_WAIT_POINT":
            if self._navigate_to(
                manager,
                pose,
                self._wait_point,
                allow_direct_fallback=True,
                planning_max_iterations=None,
            ):
                self.phase = "CALL_OPEN"
            elif self._last_navigation_blocked and self._can_open_from_safe_approach(pose):
                manager.controller.stop(reason="door_wait_point_blocked_safe_open")
                if self._wait_navigation_failure != "safe_open":
                    self._wait_navigation_failure = "safe_open"
                    if rospy is not None:
                        rospy.logwarn(
                            "[%s] 等待点不可达，但机器人已在等待侧安全接近范围内；原地开门后继续既有过门流程",
                            self.name,
                        )
                self.phase = "CALL_OPEN"
            elif self._last_navigation_blocked:
                return self._handle_wait_navigation_blocked(manager)
            return RUNNING

        if self.phase == "CALL_OPEN":
            status = self._run_service(manager, lambda: client.open_door(self.door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                _clear_opened_door_area(manager, self.door_id)
                # 主入口等开放门洞可能在接近过程中积累横向定位误差。生产编排器
                # 若提供近场重观测，就必须先用新鲜门洞几何重新回到等待轴线；普通
                # 门或精简测试没有该接口时保持原有流程。
                self.phase = (
                    "GO_OPEN_WAIT_POINT"
                    if self.door_id == "main_entrance"
                    and callable(getattr(manager, "refresh_open_door_geometry", None))
                    else "GO_PASS_POINT"
                )
            return RUNNING

        if self.phase == "GO_OPEN_WAIT_POINT":
            if self._navigate_to(
                manager,
                pose,
                self._wait_point,
                allow_direct_fallback=True,
                planning_max_iterations=None,
                max_linear=getattr(manager, "door_pass_max_linear", None),
            ):
                manager.controller.clear_path(reason="door_open_wait_reached")
                self.phase = "REFRESH_OPENING"
            elif self._last_navigation_blocked:
                return self._fail_crossing(manager, "开门后的门洞等待点不可达")
            return RUNNING

        if self.phase == "REFRESH_OPENING":
            refresh = getattr(manager, "refresh_open_door_geometry", None)
            if not callable(refresh):
                self.phase = "GO_PASS_POINT"
                return RUNNING
            status = refresh(self.door_id, pose)
            if status == RUNNING:
                manager.controller.stop(reason="door_opening_refresh")
                return RUNNING
            if status != DONE:
                return self._fail_crossing(manager, "开放门洞近场重观测失败")
            try:
                self._wait_point, self._pass_point, self._commit_point = (
                    client.get_door_path_points(
                        self.door_id,
                        (pose[0], pose[1]),
                        pass_offset=self.pass_offset,
                        commit_distance=self.commit_distance,
                    )
                )
            except ValueError as exc:
                return self._fail_crossing(manager, f"开放门洞几何更新失败: {exc}")
            manager.controller.clear_path(reason="door_opening_geometry_refreshed")
            self._pass_alignment_count = 0
            self._pass_path_started = False
            self.phase = "GO_REFRESHED_WAIT_POINT"
            return RUNNING

        if self.phase == "GO_REFRESHED_WAIT_POINT":
            if self._navigate_to(
                manager,
                pose,
                self._wait_point,
                allow_direct_fallback=True,
                planning_max_iterations=None,
                max_linear=getattr(manager, "door_pass_max_linear", None),
            ):
                manager.controller.clear_path(reason="door_refreshed_wait_reached")
                self.phase = "GO_PASS_POINT"
            elif self._last_navigation_blocked:
                return self._fail_crossing(manager, "更新后的门洞等待点不可达")
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

        tick_kwargs = {"goal_tolerance": crossing_tolerance}
        pass_linear_cap = getattr(manager, "door_pass_max_linear", None)
        if pass_linear_cap is not None:
            tick_kwargs["max_linear"] = pass_linear_cap
        status = manager.controller.tick(pose, **tick_kwargs)
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
        planning_max_iterations: int | None = DEFAULT_MAX_ITERATIONS,
        max_linear: float | None = None,
    ) -> bool:
        self._last_navigation_blocked = False
        self._last_navigation_stuck = False
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= manager.controller.goal_tolerance:
            manager.controller.clear_path()
            return True

        if not manager.controller.has_path():
            grid = manager.mapper.floor(manager.current_floor)
            result = astar_plan(
                grid,
                (pose[0], pose[1]),
                target_xy,
                max_iterations=planning_max_iterations,
            )
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

        tick_kwargs = {} if max_linear is None else {"max_linear": max_linear}
        status = manager.controller.tick(pose, **tick_kwargs)
        if status == STUCK:
            if self.phase in ("GO_PASS_POINT", "GO_COMMIT_POINT"):
                self._last_navigation_stuck = True
            else:
                manager.controller.recover()
        return status == DONE

    def _can_open_from_safe_approach(
        self,
        pose: tuple[float, float, float],
    ) -> bool:
        """等待点被栅格隔离时，允许在同侧且足够近的位置原地开门。"""
        if self._wait_point is None or self._pass_point is None:
            return False
        center_x = 0.5 * (self._wait_point[0] + self._pass_point[0])
        center_y = 0.5 * (self._wait_point[1] + self._pass_point[1])
        wait_x = self._wait_point[0] - center_x
        wait_y = self._wait_point[1] - center_y
        wait_norm = math.hypot(wait_x, wait_y)
        if wait_norm <= 1e-6:
            return False
        signed_wait_side = (
            (pose[0] - center_x) * wait_x
            + (pose[1] - center_y) * wait_y
        ) / wait_norm
        # 仅“位于同侧且离门不远”仍可能从门侧面斜穿墙体。安全开门点还须
        # 靠近门法向轴；该轴完全来自运行期传感器注册的 wait/pass 点。
        lateral_offset = abs(
            (pose[0] - center_x) * (-wait_y / wait_norm)
            + (pose[1] - center_y) * (wait_x / wait_norm)
        )
        lateral_limit = max(0.25, self.pass_offset)
        distance_to_center = math.hypot(pose[0] - center_x, pose[1] - center_y)
        safe_approach_limit = max(
            self.commit_distance + self.pass_offset,
            2.0 * self.pass_offset,
        )
        return (
            signed_wait_side >= self.pass_offset
            and lateral_offset <= lateral_limit
            and distance_to_center <= safe_approach_limit
        )

    def _handle_wait_navigation_blocked(self, manager: MissionManager) -> str:
        """等待短暂地图更新，但不允许不可达等待点形成永久零速。"""
        manager.controller.stop(reason="door_wait_point_blocked")
        now = get_time()
        if self._wait_navigation_blocked_since is None:
            self._wait_navigation_blocked_since = now
            return RUNNING
        if now - self._wait_navigation_blocked_since < self._wait_navigation_timeout:
            return RUNNING
        return self._fail_crossing(manager, "开门等待点持续不可达")

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
        if int(grid.data[gy, gx]) in (BLACK, NEGATIVE, OCC):
            return True
    return False


def _clear_opened_door_area(manager: MissionManager, door_id: str) -> None:
    clear_fn = getattr(manager, "clear_opened_door_area", None)
    if callable(clear_fn):
        clear_fn(door_id)


class TakeElevator(NavigationTask):
    """完整乘梯交互状态机。

    主流程为：到当前层厅外并对门—呼梯/开门—传感器确认开口—精确进厢—关门
    并朝目标出口—运输—目标层开门确认—确认逻辑楼层—精确出厢—关门。
    服务成功、固定等待时间和位姿 Z 都不能单独确认换层；只有目标层注册门开口
    获得传感器确认后才调用 ``MissionManager.confirm_floor_transition``。
    """
    _CABIN_TRANSIT_ALIGN_TOLERANCE = 0.10
    _CABIN_TRANSIT_ALIGN_RELEASE_TOLERANCE = 0.15
    _CABIN_TRANSIT_ALIGN_CONFIRMATIONS = 2
    # POSITION/ALIGN 翻转次数上限：栅格量化终点与精确 cabin center 的完成语义
    # 不一致时会出现每 tick 翻转的零速死循环；acceptance 已对齐二者，这里仅作
    # 安全网，避免今后其他语义退化再次形成永久振荡。
    _CABIN_TRANSIT_POSITION_ALIGN_FLIP_LIMIT = 4
    _EXIT_BACKOFF_DISTANCE = 0.35
    _EXIT_BACKOFF_ALIGN_TOLERANCE = 0.12
    _EXIT_BACKOFF_ALIGN_TIMEOUT = 4.0
    _EXIT_BACKOFF_TIMEOUT_MARGIN = 1.5
    _PLAN_RETRY_INTERVAL = 0.5
    _DEFAULT_PLAN_FAILURE_LIMIT = 3
    _DEFAULT_NAVIGATION_TIMEOUT = 120.0
    _DEFAULT_STUCK_RETRY_LIMIT = 4
    _DEFAULT_WAIT_STANDOFF = 0.8
    _DEFAULT_REFRESH_STANDOFF = 1.2

    def __init__(self, current_floor: int, target_floor: int, elevator_id: str = "elevator_main"):
        current_floor = int(current_floor)
        target_floor = int(target_floor)
        elevator_id = str(elevator_id).strip()
        if current_floor == target_floor or not elevator_id:
            raise ValueError("乘梯任务必须提供不同楼层和公开电梯 ID")
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
        self._cabin_transit_position_align_flips = 0
        self._cabin_transit_start_xy: tuple[float, float] | None = None
        self._cabin_transit_target_xy: tuple[float, float] | None = None
        self._cabin_transit_door_clear_refreshed = False
        self._exit_backoff_target_xy: tuple[float, float] | None = None
        self._exit_backoff_start_xy: tuple[float, float] | None = None
        self._exit_backoff_direction: tuple[float, float] | None = None
        self._exit_backoff_exit_yaw: float | None = None
        self._exit_backoff_phase_start = 0.0
        self._exit_backoff_deadline = 0.0
        self._door_confirmation_started: set[str] = set()
        self._door_open_confirmed: set[str] = set()
        self._exit_alignment_count = 0
        self._floor_transition_started = False
        self._floor_transition_confirmed = False
        self.failure_reason: str | None = None
        self._navigation_phase_key: str | None = None
        self._navigation_started_at: float | None = None
        self._navigation_next_plan_time = 0.0
        self._navigation_unreachable_count = 0
        self._navigation_budget_count = 0
        self._navigation_stuck_count = 0
        self._approach_trace_candidates: list[tuple[float, float]] = []
        self._approach_trace_index = 0
        self._approach_trace_attempted = False
        self._start_retreat_attempted = False
        self._start_retreat_target_xy: tuple[float, float] | None = None
        self._start_retreat_deadline = 0.0

    def tick(self, manager: MissionManager, pose: tuple[float, float, float]) -> str:
        """推进一个乘梯阶段；任一失败都显式返回 FAILED 而不猜测所在楼层。"""
        client = manager.building_client
        try:
            current_wait, cabin_center = client.get_elevator_points(self.elevator_id, self.current_floor)
            target_wait, _ = client.get_elevator_points(self.elevator_id, self.target_floor)
        except ValueError as e:
            if rospy is not None:
                rospy.logerr(f"TakeElevator 失败: {e}")
            return FAILED
        refresh_wait = self._refresh_viewpoint(
            current_wait,
            cabin_center,
            float(
                getattr(
                    manager,
                    "elevator_geometry_refresh_standoff",
                    self._DEFAULT_REFRESH_STANDOFF,
                )
            ),
        )

        try:
            current_door_id = client.elevator_door_id(self.current_floor)
            target_door_id = client.elevator_door_id(self.target_floor)
        except ValueError as e:
            if rospy is not None:
                rospy.logerr(f"TakeElevator 失败: {e}")
            return FAILED

        if self.phase == "GO_TO_ELEVATOR_DOOR":
            if self._navigate_to(manager, pose, refresh_wait):
                self._exit_alignment_count = 0
                self.phase = "ALIGN_TO_CURRENT_DOOR"
            elif self.failure_reason is not None:
                if (
                    self.failure_reason == "elevator_navigation_unreachable"
                    and self._begin_start_retreat_or_trace_approach(manager, pose)
                ):
                    return RUNNING
                return FAILED
            return RUNNING

        if self.phase == "ELEVATOR_START_RETREAT":
            return self._tick_start_retreat(manager, pose)

        if self.phase == "GO_TO_ELEVATOR_TRACE_APPROACH":
            if not self._approach_trace_candidates:
                return self._fail_navigation(
                    manager,
                    "elevator_trace_approach_unavailable",
                    "没有可供电梯粗接近的安全历史点",
                )
            target = self._approach_trace_candidates[self._approach_trace_index]
            if self._navigate_to(manager, pose, target):
                self._exit_alignment_count = 0
                self.phase = "ALIGN_TO_CURRENT_DOOR"
            elif self.failure_reason is not None:
                self._approach_trace_index += 1
                if self._approach_trace_index >= len(self._approach_trace_candidates):
                    return self._fail_navigation(
                        manager,
                        "elevator_trace_approach_exhausted",
                        "所有电梯粗接近历史点均不可达",
                    )
                manager.controller.clear_path(reason="elevator_trace_candidate_retry")
                self._reset_navigation_watchdog(clear_failure=True)
            return RUNNING

        if self.phase == "ALIGN_TO_CURRENT_DOOR":
            # 等待点只约束位置，A* 到达时的机身朝向可能让前向 RGB-D
            # 完全看不到门板。先朝传感器注册得到的轿厢中心对准，既为
            # 开门前后视觉确认保留同一视角，也避免侧着开始 ENTER。
            if self._tick_target_door_alignment(manager, pose, cabin_center):
                self._exit_alignment_count = 0
                self.phase = (
                    "REFRESH_CURRENT_ELEVATOR_GEOMETRY"
                    if callable(getattr(manager, "refresh_elevator_geometry", None))
                    else "CALL_ELEVATOR_TO_CURRENT_FLOOR"
                )
            return RUNNING

        if self.phase == "REFRESH_CURRENT_ELEVATOR_GEOMETRY":
            manager.controller.stop(reason="elevator_geometry_refresh")
            refresh = getattr(manager, "refresh_elevator_geometry", None)
            if not callable(refresh):
                self.phase = "CALL_ELEVATOR_TO_CURRENT_FLOOR"
                return RUNNING
            status = refresh(
                self.elevator_id,
                self.current_floor,
                current_door_id,
                pose,
            )
            if status == RUNNING:
                return RUNNING
            if status == IDLE:
                if rospy is not None:
                    rospy.logwarn(
                        "[%s] 电梯近场几何未刷新，保留此前连续确认几何并继续；"
                        "开门和跨越仍需局部传感器确认",
                        self.name,
                    )
                manager.controller.clear_path(
                    reason="elevator_geometry_refresh_unavailable"
                )
                self._reset_navigation_watchdog(clear_failure=True)
                self.phase = "CALL_ELEVATOR_TO_CURRENT_FLOOR"
                return RUNNING
            if status != DONE:
                self.failure_reason = "elevator_geometry_refresh_failed"
                return FAILED
            manager.controller.clear_path(reason="elevator_geometry_refreshed")
            self._reset_navigation_watchdog(clear_failure=True)
            self.phase = "GO_TO_REFRESHED_ELEVATOR_DOOR"
            return RUNNING

        if self.phase == "GO_TO_REFRESHED_ELEVATOR_DOOR":
            if self._navigate_to(manager, pose, current_wait):
                self._exit_alignment_count = 0
                self.phase = "ALIGN_TO_REFRESHED_ELEVATOR_DOOR"
            elif self.failure_reason is not None:
                return FAILED
            return RUNNING

        if self.phase == "ALIGN_TO_REFRESHED_ELEVATOR_DOOR":
            if self._tick_target_door_alignment(manager, pose, cabin_center):
                self._exit_alignment_count = 0
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
            preparation_pending = getattr(
                manager, "elevator_visual_preparation_pending", None
            )
            if callable(preparation_pending) and preparation_pending(current_door_id):
                manager.controller.stop()
                return RUNNING
            self._begin_door_open_confirmation(manager, current_door_id)
            status = self._run_service(manager, lambda: client.open_door(current_door_id))
            if status == FAILED:
                self._finish_door_open_confirmation(manager, current_door_id)
                return FAILED
            if status == DONE:
                self.phase = "WAIT_CURRENT_DOOR_OPEN"
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "WAIT_CURRENT_DOOR_OPEN":
            manager.controller.stop()
            if not self._door_opening_is_confirmed(
                manager,
                current_door_id,
                pose,
            ):
                if get_time() - self._wait_start_time > float(
                    getattr(manager, "elevator_opening_confirmation_timeout", 8.0)
                ):
                    self._finish_door_open_confirmation(manager, current_door_id)
                    if rospy is not None:
                        rospy.logerr(
                            "[%s] 当前层电梯门服务完成，但局部传感器未确认注册位置开口",
                            self.name,
                        )
                    return FAILED
                return RUNNING
            grid = manager.mapper.floor(manager.current_floor)
            # 门已由局部传感器确认打开后，轿厢内部仍可能因尚未进入而保持
            # UNKNOWN。普通 A* 只接受 FREE/VISITED，会形成“先进入才能建图、
            # 先建图才能进入”的循环。主动跨越阶段仍会重新检查实际起点到
            # 目标间的 OCC/BLACK，并保留局部正障碍制动。
            if not _segment_crosses_blocked_cell(
                grid,
                current_wait,
                cabin_center,
            ):
                self._reset_cabin_transit()
                self.phase = "ENTER_CABIN"
            elif get_time() - self._wait_start_time > 10.0:
                if rospy is not None:
                    rospy.logwarn("[%s] 等待当前楼层电梯门可通行超时，继续等待地图连通", self.name)
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "ENTER_CABIN":
            status = self._tick_cabin_transit(
                manager,
                pose,
                mode="enter",
                position_xy=current_wait,
                target_xy=cabin_center,
                door_id=current_door_id,
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
            aligned = self._tick_target_door_alignment(manager, pose, target_wait)
            if status == DONE and aligned:
                self.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
            elif status == DONE:
                self.phase = "ALIGN_TO_TARGET_DOOR"
            return RUNNING

        if self.phase == "ALIGN_TO_TARGET_DOOR":
            if self._tick_target_door_alignment(manager, pose, target_wait):
                self.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
            return RUNNING

        if self.phase == "CALL_ELEVATOR_TO_TARGET_FLOOR":
            if not self._floor_transition_started:
                begin_transition = getattr(manager, "begin_floor_transition", None)
                if callable(begin_transition):
                    try:
                        begin_transition(
                            source_floor=self.current_floor,
                            target_floor=self.target_floor,
                            phase=self.phase,
                        )
                    except (RuntimeError, ValueError) as exc:
                        if rospy is not None:
                            rospy.logerr("[%s] 无法锁存楼层过渡：%s", self.name, exc)
                        return FAILED
                self._floor_transition_started = True
            status = self._run_service(manager, lambda: client.call_elevator(self.elevator_id, self.target_floor, False))
            if status == FAILED:
                return FAILED
            if status == DONE:
                self.phase = "WAIT_FOR_ODOM_STABLE"
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "WAIT_FOR_ODOM_STABLE":
            # 合规平面定位的 z 是楼层局部高度，不能用固定等待冒充换层确认。
            # 这里只给服务动作和新传感器帧留出稳定时间，逻辑楼层在目标门
            # 被传感器确认后才更新。
            manager.controller.stop()
            if get_time() - self._wait_start_time > 2.0:
                self.phase = "OPEN_TARGET_FLOOR_DOOR"
            return RUNNING

        if self.phase == "OPEN_TARGET_FLOOR_DOOR":
            self._begin_door_open_confirmation(manager, target_door_id)
            status = self._run_service(manager, lambda: client.open_door(target_door_id))
            if status == FAILED:
                self._finish_door_open_confirmation(manager, target_door_id)
                return FAILED
            if status == DONE:
                self.phase = "WAIT_TARGET_DOOR_OPEN"
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "WAIT_TARGET_DOOR_OPEN":
            manager.controller.stop()
            if not self._door_opening_is_confirmed(
                manager,
                target_door_id,
                pose,
            ):
                if get_time() - self._wait_start_time > float(
                    getattr(manager, "elevator_opening_confirmation_timeout", 8.0)
                ):
                    self._finish_door_open_confirmation(manager, target_door_id)
                    if rospy is not None:
                        rospy.logerr(
                            "[%s] 目标层电梯门服务完成，但局部传感器未确认注册位置开口",
                            self.name,
                        )
                    return FAILED
                return RUNNING
            if not self._floor_transition_confirmed:
                confirm_transition = getattr(manager, "confirm_floor_transition", None)
                if callable(confirm_transition):
                    try:
                        confirm_transition(self.target_floor)
                    except (RuntimeError, ValueError) as exc:
                        if rospy is not None:
                            rospy.logerr("[%s] 目标门已确认但楼层过渡无效：%s", self.name, exc)
                        return FAILED
                else:
                    manager.current_floor = self.target_floor
                    manager.floor_start_time = get_time()
                self._floor_transition_confirmed = True
            grid = manager.mapper.floor(manager.target_floor if hasattr(manager, 'target_floor') and manager.target_floor is not None else self.target_floor)
            straight_segment_blocked = _segment_crosses_blocked_cell(
                grid,
                cabin_center,
                target_wait,
            )
            if not straight_segment_blocked:
                self._reset_cabin_transit()
                self.phase = "EXIT_CABIN"
            else:
                start_cell = grid.world_to_grid(pose[0], pose[1])
                goal_cell = grid.world_to_grid(target_wait[0], target_wait[1])
                start_value = (
                    int(grid.data[start_cell[1], start_cell[0]])
                    if start_cell is not None
                    else None
                )
                goal_value = (
                    int(grid.data[goal_cell[1], goal_cell[0]])
                    if goal_cell is not None
                    else None
                )
                if rospy is not None:
                    rospy.logwarn_throttle(
                        2.0,
                        "[%s] 目标层出厢门控未放行：straight_blocked=%s "
                        "start_cell=%s start_value=%s goal_cell=%s goal_value=%s",
                        self.name,
                        straight_segment_blocked,
                        start_cell,
                        start_value,
                        goal_cell,
                        goal_value,
                    )
            if self.phase == "WAIT_TARGET_DOOR_OPEN" and get_time() - self._wait_start_time > 10.0:
                if rospy is not None:
                    rospy.logwarn("[%s] 等待目标楼层电梯门可通行超时，继续等待地图连通", self.name)
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
                door_id=target_door_id,
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

    def _begin_door_open_confirmation(self, manager: MissionManager, door_id: str) -> None:
        if door_id in self._door_confirmation_started:
            return
        self._door_confirmation_started.add(door_id)
        begin = getattr(manager, "begin_elevator_door_open_confirmation", None)
        if callable(begin):
            begin(door_id)

    def _door_opening_is_confirmed(
        self,
        manager: MissionManager,
        door_id: str,
        pose: tuple[float, float, float],
    ) -> bool:
        if door_id in self._door_open_confirmed:
            return True
        confirm = getattr(manager, "confirm_elevator_door_open", None)
        if callable(confirm) and not bool(confirm(door_id, pose)):
            return False
        self._door_open_confirmed.add(door_id)
        _clear_opened_door_area(manager, door_id)
        self._finish_door_open_confirmation(manager, door_id)
        return True

    @staticmethod
    def _finish_door_open_confirmation(manager: MissionManager, door_id: str) -> None:
        finish = getattr(manager, "finish_elevator_door_open_confirmation", None)
        if callable(finish):
            finish(door_id)

    def _tick_target_door_alignment(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        target_wait: tuple[float, float],
    ) -> bool:
        """机器人完整入厢后，利用关门等待时间原地朝向出口。"""
        target_yaw = math.atan2(target_wait[1] - pose[1], target_wait[0] - pose[0])
        yaw_error = _angle_wrap(target_yaw - pose[2])
        if abs(yaw_error) > self._CABIN_TRANSIT_ALIGN_TOLERANCE:
            self._exit_alignment_count = 0
            proportional_speed = abs(1.7 * yaw_error)
            minimum_speed = abs(
                float(getattr(manager.controller, "recover_angular_speed", 0.0))
            )
            angular_limit = abs(float(getattr(manager.controller, "max_angular", 0.0)))
            angular_speed = min(
                angular_limit,
                max(proportional_speed, minimum_speed),
            )
            manager.controller.rotate_in_place(math.copysign(angular_speed, yaw_error))
            return False

        manager.controller.stop(reason="elevator_target_door_alignment_hold")
        self._exit_alignment_count += 1
        return self._exit_alignment_count >= self._CABIN_TRANSIT_ALIGN_CONFIRMATIONS

    def _tick_cabin_transit(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        *,
        mode: str,
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
        door_id: str,
        max_linear: float | None = None,
    ) -> str:
        """以 POSITION—ALIGN—CROSS 三阶段精确进入或离开轿厢。

        POSITION 到门外/轿厢中心，ALIGN 连续确认机身朝向，CROSS 只跟踪一条
        两点直线并用实际端点距离验收。独立阶段可防止普通路径跟踪提前切段，
        同时让卡住原因能够定位到准备、对准还是跨门槛。
        """
        if self._cabin_transit_mode != mode or self.cabin_transit_stage is None:
            self._begin_cabin_transit(mode, position_xy, target_xy)

        position_tolerance = self._cabin_transit_position_tolerance(manager)
        # PathFollower 以 A* 栅格量化终点判断 POSITION 到达，该终点与精确 cabin center
        # 最多相差半格对角线 0.5*sqrt(2)*resolution。ALIGN 的漂移检查却以精确 cabin center
        # 为准，若二者容差相同，会在临界距离上形成“POSITION 认为到达->ALIGN 认为漂移
        # ->回 POSITION”的零速死循环。修复方式是非对称：POSITION 保持严格
        # position_tolerance（保证 snapped 到达时精确距离 <= position_tolerance + 半格对角线），
        # ALIGN 的漂移检查放宽到把栅格量化误差计入的 position_acceptance。CROSS 的完整
        # 通过判定仍保留严格 position_tolerance。
        position_acceptance = self._cabin_transit_position_acceptance(manager)

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
            if position_error > position_acceptance:
                self._cabin_transit_position_align_flips += 1
                if self._cabin_transit_position_align_flips > self._CABIN_TRANSIT_POSITION_ALIGN_FLIP_LIMIT:
                    manager.controller.stop(reason="elevator_transit_position_align_oscillation")
                    if rospy is not None:
                        rospy.logerr(
                            "[%s] 电梯轿厢 POSITION/ALIGN 反复翻转超过上限，判定为确定性失败："
                            "mode=%s position_error=%.3f acceptance=%.3f",
                            self.name,
                            self._cabin_transit_mode,
                            position_error,
                            position_acceptance,
                        )
                    return STUCK
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
            alignment_limit = self._CABIN_TRANSIT_ALIGN_TOLERANCE
            if self._cabin_transit_alignment_count > 0:
                # A1 停止纯转向后仍会有少量余转。首次进入必须满足严格阈值，
                # 确认期间用窄迟滞避免在最小可执行转速两侧反复振荡。
                alignment_limit = self._CABIN_TRANSIT_ALIGN_RELEASE_TOLERANCE
            if abs(yaw_error) > alignment_limit:
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

            # 开门确认与完成对准之间仍会处理新点云，动态门板或门槛的膨胀
            # OCC 可能被重新写回。提交直线路径前只刷新一次既有门区清理；
            # clear_dynamic_door_strip 不会清除 BLACK，后面的正障碍检查仍保留。
            if not self._cabin_transit_door_clear_refreshed:
                _clear_opened_door_area(manager, door_id)
                self._cabin_transit_door_clear_refreshed = True
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
            self._cabin_transit_position_align_flips = 0
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
        self._cabin_transit_position_align_flips = 0
        self._cabin_transit_start_xy = None
        self._cabin_transit_target_xy = target_xy
        self._cabin_transit_door_clear_refreshed = False
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
        self._cabin_transit_position_align_flips = 0
        self._cabin_transit_start_xy = None
        self._cabin_transit_target_xy = None
        self._cabin_transit_door_clear_refreshed = False

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
    def _cabin_transit_position_acceptance(manager: MissionManager) -> float:
        """ALIGN 漂移检查用的接受半径。

        PathFollower 以 A* 栅格量化终点判断 POSITION 到达，该终点与精确 cabin center
        最多相差半格对角线 0.5*sqrt(2)*resolution。POSITION 仍用严格 position_tolerance，
        因此“snapped 到达”时精确距离必 <= position_tolerance + 半格对角线；ALIGN 的漂移
        检查改用本接受半径，保证不会把已到达的机器人退回 POSITION。
        """
        position_tolerance = TakeElevator._cabin_transit_position_tolerance(manager)
        grid = manager.mapper.floor(manager.current_floor)
        resolution = max(1e-6, float(getattr(grid, "resolution", 0.1)))
        grid_snap_radius = 0.5 * math.sqrt(2.0) * resolution
        goal_tolerance = max(1e-3, float(manager.controller.goal_tolerance))
        return min(goal_tolerance, position_tolerance + grid_snap_radius)

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
        now = get_time()
        if self._navigation_phase_key != self.phase:
            self._reset_navigation_watchdog(clear_failure=True)
            self._navigation_phase_key = self.phase
            self._navigation_started_at = now
        elif self._navigation_started_at is None:
            self._navigation_started_at = now

        navigation_timeout = float(
            getattr(
                manager,
                "elevator_navigation_timeout",
                self._DEFAULT_NAVIGATION_TIMEOUT,
            )
        )
        if now - self._navigation_started_at > navigation_timeout:
            return self._fail_navigation(
                manager,
                "elevator_navigation_timeout",
                f"导航阶段 {self.phase} 超过 {navigation_timeout:.1f}s",
            )

        effective_goal_tolerance = float(manager.controller.goal_tolerance)
        if goal_tolerance is not None:
            effective_goal_tolerance = min(
                effective_goal_tolerance,
                max(0.0, float(goal_tolerance)),
            )
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= effective_goal_tolerance:
            manager.controller.clear_path()
            self._reset_navigation_watchdog(clear_failure=True)
            return DONE

        if not manager.controller.has_path():
            if now < self._navigation_next_plan_time:
                manager.controller.stop(reason="elevator_navigation_plan_wait")
                return RUNNING
            grid = manager.mapper.floor(manager.current_floor)
            multiplier = 1 << min(self._navigation_budget_count, 2)
            max_iterations = DEFAULT_MAX_ITERATIONS * multiplier
            result = astar_plan(
                grid,
                (pose[0], pose[1]),
                target_xy,
                max_iterations=max_iterations,
            )
            if result.reached:
                manager.controller.set_path(result.path)
                self._navigation_unreachable_count = 0
                self._navigation_budget_count = 0
                self._navigation_next_plan_time = 0.0
                self._log_navigation_plan(
                    manager,
                    pose,
                    target_xy,
                    result.status,
                    max_iterations,
                    0,
                )
            elif result.budget_exhausted:
                self._navigation_budget_count += 1
                self._log_navigation_plan(
                    manager,
                    pose,
                    target_xy,
                    result.status,
                    max_iterations,
                    self._navigation_budget_count,
                )
                manager.controller.stop(reason="elevator_navigation_budget_exhausted")
                if self._navigation_budget_count >= int(
                    getattr(
                        manager,
                        "elevator_planning_retry_limit",
                        self._DEFAULT_PLAN_FAILURE_LIMIT,
                    )
                ):
                    return self._fail_navigation(
                        manager,
                        "elevator_navigation_budget_exhausted",
                        "A* 逐级扩大预算后仍未完成",
                    )
                self._navigation_next_plan_time = now + self._PLAN_RETRY_INTERVAL
                return RUNNING
            else:
                self._navigation_unreachable_count += 1
                self._log_navigation_plan(
                    manager,
                    pose,
                    target_xy,
                    result.status,
                    max_iterations,
                    self._navigation_unreachable_count,
                )
                manager.controller.stop(reason="elevator_navigation_unreachable")
                if self._navigation_unreachable_count >= int(
                    getattr(
                        manager,
                        "elevator_planning_retry_limit",
                        self._DEFAULT_PLAN_FAILURE_LIMIT,
                    )
                ):
                    return self._fail_navigation(
                        manager,
                        "elevator_navigation_unreachable",
                        "A* 连续确认目标不可达",
                    )
                self._navigation_next_plan_time = now + self._PLAN_RETRY_INTERVAL
                return RUNNING

        if max_linear is None and goal_tolerance is None:
            status = manager.controller.tick(pose)
        else:
            tick_kwargs = {}
            if max_linear is not None:
                tick_kwargs["max_linear"] = max_linear
            if goal_tolerance is not None:
                tick_kwargs["goal_tolerance"] = effective_goal_tolerance
            status = manager.controller.tick(pose, **tick_kwargs)
        if status == STUCK:
            self._navigation_stuck_count += 1
            stuck_limit = int(
                getattr(
                    manager,
                    "elevator_stuck_retry_limit",
                    self._DEFAULT_STUCK_RETRY_LIMIT,
                )
            )
            if self._navigation_stuck_count > stuck_limit:
                return self._fail_navigation(
                    manager,
                    "elevator_navigation_stuck_exhausted",
                    f"局部控制卡住恢复超过 {stuck_limit} 次",
                )
            recover_with_clearance = getattr(
                manager, "_recover_with_rotation_clearance", None
            )
            if callable(recover_with_clearance):
                recover_with_clearance("elevator_navigation_stuck")
            else:
                # 独立任务夹具可能没有 MissionManager 的净空辅助方法；生产实例
                # 始终通过上面的统一门控，兼容分支只保留任务的既有独立可测性。
                manager.controller.recover()
            return RUNNING
        if status == DONE:
            self._reset_navigation_watchdog(clear_failure=True)
        return status

    def _navigate_to(self, manager: MissionManager, pose: tuple[float, float, float], target_xy: tuple[float, float]) -> bool:
        status = self._navigate_to_with_status(manager, pose, target_xy)
        return status == DONE

    @classmethod
    def _refresh_viewpoint(
        cls,
        wait_xy: tuple[float, float],
        cabin_xy: tuple[float, float],
        standoff: float,
    ) -> tuple[float, float]:
        """从既有候梯点沿大厅方向后退，给前置 RGB-D 留出完整门板视场。"""
        dx = float(wait_xy[0]) - float(cabin_xy[0])
        dy = float(wait_xy[1]) - float(cabin_xy[1])
        norm = math.hypot(dx, dy)
        if norm <= 1e-9 or not math.isfinite(float(standoff)):
            return wait_xy
        extra = max(0.0, float(standoff) - cls._DEFAULT_WAIT_STANDOFF)
        return (
            float(wait_xy[0]) + extra * dx / norm,
            float(wait_xy[1]) + extra * dy / norm,
        )

    def _begin_start_retreat_or_trace_approach(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
    ) -> bool:
        if not self._start_retreat_attempted:
            self._start_retreat_attempted = True
            target_provider = getattr(manager, "elevator_start_retreat_target", None)
            target = (
                target_provider(pose, self.current_floor)
                if callable(target_provider)
                else None
            )
            if target is not None:
                self._start_retreat_target_xy = (
                    float(target[0]),
                    float(target[1]),
                )
                distance = math.hypot(
                    self._start_retreat_target_xy[0] - pose[0],
                    self._start_retreat_target_xy[1] - pose[1],
                )
                reverse_speed = abs(
                    float(getattr(manager.controller, "reverse_speed", -0.15))
                )
                self._start_retreat_deadline = get_time() + max(
                    4.0,
                    distance / max(0.05, reverse_speed) + 2.0,
                )
                manager.controller.clear_path(reason="elevator_start_retreat_start")
                self._reset_navigation_watchdog(clear_failure=True)
                self.phase = "ELEVATOR_START_RETREAT"
                self._log_start_retreat(
                    manager,
                    "elevator_start_retreat_started",
                    pose,
                    self._start_retreat_target_xy,
                )
                return True
        return self._begin_trace_approach(manager)

    def _tick_start_retreat(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
    ) -> str:
        target = self._start_retreat_target_xy
        if target is None:
            return self._fail_start_retreat(manager, pose, "target_missing")
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        if distance <= 0.25:
            manager.controller.stop(reason="elevator_start_retreat_completed")
            self._log_start_retreat(
                manager,
                "elevator_start_retreat_completed",
                pose,
                target,
            )
            self._start_retreat_target_xy = None
            self._start_retreat_deadline = 0.0
            self._reset_navigation_watchdog(clear_failure=True)
            self.phase = "GO_TO_ELEVATOR_DOOR"
            return RUNNING
        if get_time() >= self._start_retreat_deadline:
            return self._fail_start_retreat(manager, pose, "retreat_timeout")
        target_direction = math.atan2(target[1] - pose[1], target[0] - pose[0])
        backward_error = _angle_wrap(target_direction - pose[2] - math.pi)
        if abs(backward_error) > 0.45:
            return self._fail_start_retreat(
                manager,
                pose,
                "reverse_heading_diverged",
            )
        if not manager.controller.reverse_straight(
            reason="elevator_start_clearance_retreat"
        ):
            return self._fail_start_retreat(
                manager,
                pose,
                "reverse_command_rejected",
            )
        return RUNNING

    def _fail_start_retreat(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float],
        detail: str,
    ) -> str:
        target = self._start_retreat_target_xy
        self._log_start_retreat(
            manager,
            "elevator_start_retreat_failed",
            pose,
            target,
            detail=detail,
        )
        self._start_retreat_target_xy = None
        self._start_retreat_deadline = 0.0
        return self._fail_navigation(
            manager,
            "elevator_start_retreat_failed",
            detail,
        )

    def _log_start_retreat(
        self,
        manager: MissionManager,
        event: str,
        pose: tuple[float, float, float],
        target: tuple[float, float] | None,
        *,
        detail: str | None = None,
    ) -> None:
        log_structured = getattr(manager, "_log_structured", None)
        if not callable(log_structured):
            return
        payload = {
            "event": str(event),
            "task": self.name,
            "current_floor": int(self.current_floor),
            "pose": [round(float(pose[0]), 3), round(float(pose[1]), 3)],
            "target": None
            if target is None
            else [round(float(target[0]), 3), round(float(target[1]), 3)],
        }
        if detail is not None:
            payload["detail"] = str(detail)
        log_structured(payload)

    def _begin_trace_approach(self, manager: MissionManager) -> bool:
        if self._approach_trace_attempted:
            return False
        self._approach_trace_attempted = True
        candidate_provider = getattr(manager, "elevator_approach_trace_candidates", None)
        if not callable(candidate_provider):
            return False
        candidates = list(
            candidate_provider(
                self.elevator_id,
                self.current_floor,
                max_candidates=4,
            )
        )
        if not candidates:
            return False
        self._approach_trace_candidates = [
            (float(candidate[0]), float(candidate[1])) for candidate in candidates
        ]
        self._approach_trace_index = 0
        manager.controller.clear_path(reason="elevator_trace_approach_start")
        self._reset_navigation_watchdog(clear_failure=True)
        self.phase = "GO_TO_ELEVATOR_TRACE_APPROACH"
        if rospy is not None:
            rospy.logwarn(
                "[%s] 精确电梯等待点不可达，改用 %d 个已走轨迹上的安全粗接近点",
                self.name,
                len(self._approach_trace_candidates),
            )
        return True

    def _reset_navigation_watchdog(self, *, clear_failure: bool) -> None:
        self._navigation_phase_key = None
        self._navigation_started_at = None
        self._navigation_next_plan_time = 0.0
        self._navigation_unreachable_count = 0
        self._navigation_budget_count = 0
        self._navigation_stuck_count = 0
        if clear_failure:
            self.failure_reason = None

    def _fail_navigation(
        self,
        manager: MissionManager,
        reason_code: str,
        detail: str,
    ) -> str:
        self.failure_reason = str(reason_code)
        manager.controller.clear_path(reason=reason_code)
        manager.controller.stop(reason=reason_code)
        if rospy is not None:
            rospy.logerr("[%s] %s：%s", self.name, reason_code, detail)
        self._log_navigation_plan(
            manager,
            None,
            None,
            reason_code,
            0,
            max(
                self._navigation_unreachable_count,
                self._navigation_budget_count,
                self._navigation_stuck_count,
            ),
        )
        return FAILED

    def _log_navigation_plan(
        self,
        manager: MissionManager,
        pose: tuple[float, float, float] | None,
        target_xy: tuple[float, float] | None,
        result: str,
        max_iterations: int,
        attempt: int,
    ) -> None:
        log_structured = getattr(manager, "_log_structured", None)
        if not callable(log_structured):
            return
        log_structured(
            {
                "event": "elevator_navigation_plan",
                "task": self.name,
                "phase": self.phase,
                "result": str(result),
                "attempt": int(attempt),
                "max_iterations": int(max_iterations),
                "pose": None
                if pose is None
                else [round(float(pose[0]), 3), round(float(pose[1]), 3)],
                "target": None
                if target_xy is None
                else [round(float(target_xy[0]), 3), round(float(target_xy[1]), 3)],
            }
        )

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

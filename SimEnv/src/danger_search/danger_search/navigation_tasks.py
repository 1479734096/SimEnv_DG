"""普通导航、过门和乘电梯的非阻塞子状态机。

任务编排器同一时刻只持有一个任务，每次主 tick 调用一次任务 ``tick``。任务
可以规划路径、轮询异步服务或驱动 ``PathFollower``，但不会
自行循环等待。统一返回 ``RUNNING/DONE/FAILED``（控制器还可返回 STUCK/IDLE），
让顶层状态机清楚地区分仍在执行、成功和失败。
"""
from __future__ import annotations

import math
import time
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
    from danger_search.navigation_context import NavigationTaskContext
    from danger_search.building_control_client import AsyncServiceCall


FAILED = "failed"


def _angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class NavigationTask:
    """导航子任务最小接口；实现必须让每次 ``tick`` 快速返回。"""
    def __init__(self, name: str):
        self.name = name

    def tick(self, context: NavigationTaskContext, pose: tuple[float, float, float]) -> str:
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

    def tick(self, context: NavigationTaskContext, pose: tuple[float, float, float]) -> str:
        tol = self.tolerance if self.tolerance is not None else context.controller.goal_tolerance
        if math.hypot(self.target_xy[0] - pose[0], self.target_xy[1] - pose[1]) <= tol:
            context.controller.clear_path()
            return DONE

        if not context.controller.has_path():
            grid = context.mapper.floor(context.current_floor)
            result = context.plan_path(grid, (pose[0], pose[1]), self.target_xy)
            if result.reached:
                context.controller.set_path(result.path, reason="navigate_to_pose_path")
            elif result.unreachable:
                context.controller.stop(reason="navigation_target_unreachable")
                return FAILED
            else:
                # 预算耗尽只表示本轮未完成搜索，不能穿过 OCC/BLACK 直连。
                context.controller.stop(reason="navigation_budget_exhausted")
                return RUNNING

        status = context.controller.poll_status()
        if status == STUCK:
            context.controller.recover()
            return RUNNING  # 仍在恢复中
        return status


class OpenDoorAndPass(NavigationTask):
    """动态门交互状态机：到等待侧、开门、直线过门并深入门后。

    等待/通过/commit 三点全部来自运行时传感器注册门几何。服务成功后仍由
    编排器通过显式上下文提供受限门板条带更新；真正
    过门阶段先原地对准，再使用两点直线和更严格端点容差，避免 A* 折线切门框。
    """
    _PASS_ALIGN_TOLERANCE = 0.10
    _PASS_ALIGN_CONFIRMATIONS = 2
    # GO_PASS_POINT 细对准总预算：settle + 旋转 + 确认。超时必须显式失败，
    # 否则该分支只会一直返回 RUNNING，形成无超时活锁（实测可持续 395s）。
    _PASS_ALIGN_TIMEOUT = 12.0
    # 细对准期间允许的位置漂移半径：原地转向不应移动，超出即认为锚点失效。
    _PASS_ALIGN_ANCHOR_RADIUS = 0.30
    _MAIN_ENTRANCE_CENTERLINE_TOLERANCE = 0.25
    _MAIN_ENTRANCE_CENTERING_TIMEOUT = 30.0
    _SERVICE_TIMEOUT = 30.0
    _SERVICE_WALL_TIMEOUT = 180.0

    def __init__(
        self,
        door_id: str,
        *,
        pass_offset: float = 0.8,
        commit_distance: float = 1.8,
        wait_offset: float | None = None,
    ):
        door_id = str(door_id).strip()
        if not door_id:
            raise ValueError("过门任务必须提供公开门 ID")
        if not all(
            math.isfinite(float(value)) and float(value) > 0.0
            for value in (
                pass_offset,
                commit_distance,
                pass_offset if wait_offset is None else wait_offset,
            )
        ):
            raise ValueError("过门偏移和深入距离必须是正有限数")
        super().__init__(f"OpenDoorAndPass({door_id})")
        self.door_id = door_id
        self.pass_offset = float(pass_offset)
        self.commit_distance = float(commit_distance)
        self.wait_offset = self.pass_offset if wait_offset is None else float(wait_offset)
        self.phase = "GO_WAIT_POINT"
        self._wait_point: tuple[float, float] | None = None
        self._pass_point: tuple[float, float] | None = None
        self._commit_point: tuple[float, float] | None = None
        self._service_task: AsyncServiceCall | None = None
        self._service_started_at = 0.0
        self._service_started_wall = 0.0
        self.failure_reason: str | None = None
        self._retry_count = 0
        self._last_navigation_blocked = False
        self._last_navigation_stuck = False
        self._pass_retry_start_time: float | None = None
        self._pass_retry_timeout = 3.0
        self._pass_alignment_count = 0
        self._pass_path_started = False
        self._pass_align_start_time: float | None = None
        self._pass_alignment_done = False
        self._opening_sensor_confirmed = False
        self._commit_retry_count = 0
        self._commit_retry_start_time: float | None = None
        self._commit_retry_limit = 30
        self._commit_retry_timeout = 12.0
        self._wait_navigation_failure: str | None = None
        self._wait_navigation_blocked_since: float | None = None
        self._wait_navigation_timeout = 3.0
        self._landmark_relocalization_attempted = False
        self._main_entrance_centering_started_at: float | None = None
        self._main_entrance_centering_started_wall: float | None = None
        self._main_entrance_recenter_count = 0

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

    @property
    def door_center(self) -> tuple[float, float] | None:
        """由非对称等待/通过点还原传感器注册的门中心。"""
        if self._wait_point is None or self._pass_point is None:
            return None
        total = self.wait_offset + self.pass_offset
        return (
            (
                self.pass_offset * self._wait_point[0]
                + self.wait_offset * self._pass_point[0]
            )
            / total,
            (
                self.pass_offset * self._wait_point[1]
                + self.wait_offset * self._pass_point[1]
            )
            / total,
        )

    def mark_opening_sensor_confirmed(self) -> None:
        """记录开放门洞已由近场传感器连续确认。"""
        self._opening_sensor_confirmed = True

    def tick(self, context: NavigationTaskContext, pose: tuple[float, float, float]) -> str:
        """按当前 ``phase`` 推进一步，所有服务等待期间保持零速。"""
        client = context.building_client
        if (
            self.door_id == "main_entrance"
            and not self._pass_path_started
            and self.phase in {
                "GO_OPEN_WAIT_POINT", "REFRESH_OPENING", "GO_REFRESHED_WAIT_POINT", "GO_PASS_POINT",
            }
        ):
            self._start_main_entrance_centering()
            if self._main_entrance_centering_timed_out():
                return self._fail_crossing(context, "主入口站位与对准总超时")

        if self._wait_point is None or self._pass_point is None or self._commit_point is None:
            try:
                self._wait_point, self._pass_point, self._commit_point = client.get_door_path_points(
                    self.door_id,
                    (pose[0], pose[1]),
                    pass_offset=self.pass_offset,
                    commit_distance=self.commit_distance,
                    wait_offset=self.wait_offset,
                )
            except ValueError as e:
                if rospy is not None:
                    rospy.logerr(f"OpenDoorAndPass 失败: {e}")
                return FAILED

        if self.phase == "GO_WAIT_POINT":
            if self._navigate_to(
                context,
                pose,
                self._wait_point,
                allow_direct_fallback=True,
                planning_max_iterations=None,
            ):
                self.phase = (
                    "RELOCALIZE_MAIN_ENTRANCE"
                    if self.door_id == "main_entrance"
                    and not self._landmark_relocalization_attempted
                    and context.relocalize_landmark_fn is not None
                    else "CALL_OPEN"
                )
            elif self._last_navigation_blocked and self._can_open_from_safe_approach(pose):
                context.controller.stop(reason="door_wait_point_blocked_safe_open")
                if self._wait_navigation_failure != "safe_open":
                    self._wait_navigation_failure = "safe_open"
                    if rospy is not None:
                        rospy.logwarn(
                            "[%s] 等待点不可达，但机器人已在等待侧安全接近范围内；原地开门后继续既有过门流程",
                            self.name,
                        )
                self.phase = "CALL_OPEN"
            elif self._last_navigation_blocked:
                return self._handle_wait_navigation_blocked(context)
            return RUNNING

        if self.phase == "RELOCALIZE_MAIN_ENTRANCE":
            context.controller.stop(reason="main_entrance_relocalization")
            outcome = context.relocalize_landmark(
                "main_entrance",
                self.door_id,
                pose,
                f"{self.name}:main_entrance",
            )
            if outcome is not None and getattr(outcome, "status", IDLE) == RUNNING:
                return RUNNING
            self._landmark_relocalization_attempted = True
            if outcome is not None and bool(getattr(outcome, "applied", False)):
                self._wait_point = None
                self._pass_point = None
                self._commit_point = None
                context.controller.clear_path(reason="main_entrance_relocalized")
                self.phase = "GO_WAIT_POINT"
            else:
                self.phase = "CALL_OPEN"
            return RUNNING

        if self.phase == "CALL_OPEN":
            status = self._run_service(context, lambda: client.open_door(self.door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                _clear_opened_door_area(context, self.door_id)
                # 主入口等开放门洞可能在接近过程中积累横向定位误差。生产编排器
                # 若提供近场重观测，就必须先用新鲜门洞几何重新回到等待轴线；普通
                # 门或精简测试没有该接口时保持原有流程。
                self.phase = (
                    "GO_OPEN_WAIT_POINT"
                    if self.door_id == "main_entrance"
                    and context.open_door_geometry_refresh_enabled
                    else "GO_PASS_POINT"
                )
            return RUNNING

        if self.phase == "GO_OPEN_WAIT_POINT":
            if self._navigate_to(
                context,
                pose,
                self._wait_point,
                allow_direct_fallback=True,
                planning_max_iterations=None,
                max_linear=context.door_pass_max_linear,
            ):
                context.controller.clear_path(reason="door_open_wait_reached")
                self.phase = "REFRESH_OPENING"
            elif self._last_navigation_blocked:
                return self._fail_crossing(context, "开门后的门洞等待点不可达")
            return RUNNING

        if self.phase == "REFRESH_OPENING":
            correction = context.correct_entrance_geometry("before_pass", pose)
            if correction is not None:
                if getattr(correction, "reason", None) in {
                    "correction_ack_timeout", "corrected_pose_timeout",
                } or getattr(correction, "overlap_owner_ids", ()):
                    return FAILED
                if getattr(correction, "status", None) == RUNNING:
                    context.controller.stop(reason="entrance_geometry_before_pass")
                    return RUNNING
                if bool(getattr(correction, "applied", False)):
                    self._wait_point = None
                    self._pass_point = None
                    self._commit_point = None
                    # 纠偏后当前位置仍在已停车的门前，立即用新 epoch 的近场
                    # 点云刷新门几何，避免沿纠偏前的旧等待点再次横向撞门。
                    self.phase = "REFRESH_OPENING"
                    return RUNNING
            status = context.refresh_open_door_geometry(self.door_id, pose)
            if status == RUNNING:
                context.controller.stop(reason="door_opening_refresh")
                return RUNNING
            if status != DONE:
                return self._fail_crossing(context, "开放门洞近场重观测失败")
            try:
                self._wait_point, self._pass_point, self._commit_point = (
                    client.get_door_path_points(
                        self.door_id,
                        (pose[0], pose[1]),
                        pass_offset=self.pass_offset,
                        commit_distance=self.commit_distance,
                        wait_offset=self.wait_offset,
                    )
                )
            except ValueError as exc:
                return self._fail_crossing(context, f"开放门洞几何更新失败: {exc}")
            context.controller.clear_path(reason="door_opening_geometry_refreshed")
            self._pass_alignment_count = 0
            self._pass_path_started = False
            self._pass_align_start_time = None
            self._pass_alignment_done = False
            centerline_error = self._door_centerline_error(pose)
            wait_distance = math.hypot(
                self._wait_point[0] - pose[0], self._wait_point[1] - pose[1]
            )
            if (
                centerline_error <= self._MAIN_ENTRANCE_CENTERLINE_TOLERANCE
                and wait_distance <= 0.30
            ):
                self.phase = "GO_PASS_POINT"
            else:
                if rospy is not None:
                    rospy.loginfo(
                        "[%s] 主入口重新站位：横向偏差 %.3fm，等待点距离 %.3fm",
                        self.name,
                        centerline_error,
                        wait_distance,
                    )
                self.phase = "GO_REFRESHED_WAIT_POINT"
            return RUNNING

        if self.phase == "GO_REFRESHED_WAIT_POINT":
            if self._navigate_to(
                context,
                pose,
                self._wait_point,
                allow_direct_fallback=True,
                planning_max_iterations=None,
                max_linear=context.door_pass_max_linear,
                goal_tolerance=self._MAIN_ENTRANCE_CENTERLINE_TOLERANCE,
            ):
                context.controller.clear_path(reason="door_refreshed_wait_reached")
                self.phase = "GO_PASS_POINT"
            elif self._last_navigation_blocked:
                return self._fail_crossing(context, "更新后的门洞等待点不可达")
            return RUNNING

        if self.phase == "GO_PASS_POINT":
            return self._tick_active_pass(context, pose)

        if self.phase == "GO_COMMIT_POINT":
            advanced = self._navigate_to(
                context,
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
                return self._fail_crossing(context, "过门深入阶段有命令但无有效位移")
            if self._last_navigation_blocked:
                return self._handle_commit_blocked(context)
            self._commit_retry_count = 0
            self._commit_retry_start_time = None
            return RUNNING

        return DONE

    def _tick_active_pass(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
    ) -> str:
        """门洞中心对准后直线通过，并以真实位姿确认完整通过。"""
        if self._pass_point is None:
            return self._fail_crossing(context, "缺少过门通过点")

        goal_tolerance = max(1e-3, float(context.controller.goal_tolerance))
        corner_tolerance = float(
            getattr(context.controller, "corner_tolerance", goal_tolerance)
        )
        if not math.isfinite(corner_tolerance) or corner_tolerance <= 0.0:
            corner_tolerance = goal_tolerance
        crossing_tolerance = min(goal_tolerance, corner_tolerance)

        if not self._pass_path_started:
            if (
                self.door_id == "main_entrance"
                and self._door_centerline_error(pose)
                > self._MAIN_ENTRANCE_CENTERLINE_TOLERANCE
            ):
                self._main_entrance_recenter_count += 1
                if self._main_entrance_recenter_count > 3:
                    return self._fail_crossing(context, "主入口重新站位次数耗尽")
                context.controller.clear_path(reason="main_entrance_centerline_drift")
                self._pass_alignment_done = False
                self._pass_alignment_count = 0
                self._pass_align_start_time = None
                self.phase = "GO_REFRESHED_WAIT_POINT"
                return RUNNING
            align_status, watching = self._tick_pass_alignment(context, pose)
            if align_status == FAILED:
                return self._fail_crossing(context, "门洞细对准失败")
            if align_status != DONE:
                # 细对准必须有超时：转向未收敛时如果只返回 RUNNING，
                # 下面的 STUCK/IDLE 判定与阻塞超时都不可达，会形成
                # 无保护活锁（实测在主入口原地摆动数百秒直到外部杀进程）。
                if watching:
                    now = get_time()
                    if self._pass_align_start_time is None:
                        self._pass_align_start_time = now
                    elif now - self._pass_align_start_time >= self._PASS_ALIGN_TIMEOUT:
                        if rospy is not None:
                            rospy.logerr(
                                "[%s] 门洞细对准超时：door_id=%s 耗时=%.1fs "
                                "pose=(%.2f, %.2f, %.2f) pass_point=(%.2f, %.2f)",
                                self.name,
                                self.door_id,
                                now - self._pass_align_start_time,
                                pose[0],
                                pose[1],
                                pose[2],
                                self._pass_point[0],
                                self._pass_point[1],
                            )
                        return self._fail_crossing(context, "门洞细对准超时未收敛")
                return RUNNING

            self._pass_align_start_time = None
            _clear_opened_door_area(context, self.door_id)
            grid = context.mapper.floor(context.current_floor)
            if _segment_crosses_blocked_cell(
                grid,
                (pose[0], pose[1]),
                self._pass_point,
                constraints=context.planning_constraints,
            ):
                return self._handle_pass_blocked(context)

            pass_linear_cap = context.door_pass_max_linear
            context.controller.set_path(
                [(pose[0], pose[1]), self._pass_point],
                max_linear=pass_linear_cap,
                goal_tolerance=crossing_tolerance,
                reason="door_pass_point_path",
            )
            self._pass_path_started = True
            self._pass_retry_start_time = None
            return RUNNING

        endpoint_distance = math.hypot(
            self._pass_point[0] - pose[0],
            self._pass_point[1] - pose[1],
        )
        if endpoint_distance <= crossing_tolerance:
            context.controller.clear_path(reason="door_pass_completed")
            return self._finish_pass(pose)
        if not context.controller.has_path():
            return self._fail_crossing(context, "过门直线路径意外丢失")

        # 过门直线路径：限速与终点容差随路径原子提交（固定频率控制读取）。
        pass_linear_cap = context.door_pass_max_linear
        context.controller.set_tracking_limits(
            goal_tolerance=crossing_tolerance,
            max_linear=pass_linear_cap,
        )
        status = context.controller.poll_status()
        if status == DONE:
            completion_pose = pose
            tracking_snapshot = getattr(
                context.controller,
                "tracking_debug_snapshot",
                None,
            )
            if callable(tracking_snapshot):
                latest_pose = tracking_snapshot().get("pose")
                if (
                    isinstance(latest_pose, (list, tuple))
                    and len(latest_pose) >= 3
                    and all(math.isfinite(float(value)) for value in latest_pose[:3])
                ):
                    completion_pose = tuple(float(value) for value in latest_pose[:3])
            endpoint_distance = math.hypot(
                self._pass_point[0] - completion_pose[0],
                self._pass_point[1] - completion_pose[1],
            )
            if endpoint_distance <= crossing_tolerance:
                return self._finish_pass(completion_pose)
            return self._fail_crossing(context, "控制器提前结束过门路径")
        if status in (STUCK, IDLE):
            return self._fail_crossing(context, "过门通过阶段有命令但无有效位移")
        return RUNNING

    def _tick_pass_alignment(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
    ) -> tuple[str, bool]:
        """推进门洞细对准，返回 (状态, 是否计入超时预算)。

        DONE 表示朝向已确认、可以提交直线路径；FAILED 表示对准硬失败。
        第二个返回值仅在“正在转向或等待控制器收敛”时为 True：命中容差后的
        确认 tick 不计入预算，避免把正常收敛过程误判成超时。

        优先使用 PathFollower 的固定频率 ALIGN 模式（SETTLE->ALIGN->CONFIRM）：
        它带 yaw_rate 阻尼、静置判定、多帧确认与锚点漂移保护，跑在控制频率上。
        任务层以 10Hz 直接发 rotate_in_place 压不住 A1 的转向惯性——最小可执行
        转速 0.55 rad/s 会反复冲出 ±0.10 rad 的对准窗口，确认数永远攒不满。
        """
        if self._pass_alignment_done:
            return DONE, False

        begin_alignment = getattr(context.controller, "begin_heading_alignment", None)
        poll_result = getattr(context.controller, "poll_result", None)
        if not callable(begin_alignment) or not callable(poll_result):
            # 精简控制替身没有 ALIGN 接口：退回任务层自转，仍受上层超时保护。
            return self._tick_pass_alignment_legacy(context, pose)

        result = poll_result()
        if result.status == DONE and result.reason == "alignment_done":
            # 锁存：控制器完成后回到 idle，阻塞重试期间不得重新 SETTLE。
            self._pass_alignment_done = True
            return DONE, False
        if result.status == STUCK:
            if result.reason == "alignment_drift":
                # 位置漂移使锚点失效：重新进入对准，不算硬失败。
                begin_alignment(
                    self._pass_point,
                    anchor_xy=(pose[0], pose[1]),
                    max_anchor_error=self._PASS_ALIGN_ANCHOR_RADIUS,
                )
                return RUNNING, True
            if rospy is not None:
                rospy.logerr(
                    "[%s] 门洞细对准被控制器判定失败：door_id=%s reason=%s",
                    self.name,
                    self.door_id,
                    result.reason,
                )
            return FAILED, False
        if result.status == IDLE:
            begin_alignment(
                self._pass_point,
                anchor_xy=(pose[0], pose[1]),
                max_anchor_error=self._PASS_ALIGN_ANCHOR_RADIUS,
            )
        return RUNNING, True

    def _tick_pass_alignment_legacy(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
    ) -> tuple[str, bool]:
        """无 ALIGN 接口时的任务层自转对准（保留原有语义）。"""
        target_yaw = math.atan2(
            self._pass_point[1] - pose[1],
            self._pass_point[0] - pose[0],
        )
        yaw_error = _angle_wrap(target_yaw - pose[2])
        if abs(yaw_error) > self._PASS_ALIGN_TOLERANCE:
            self._pass_alignment_count = 0
            angular_limit = abs(float(getattr(context.controller, "max_angular", 0.0)))
            minimum_speed = abs(
                float(getattr(context.controller, "recover_angular_speed", 0.0))
            )
            angular_speed = min(
                angular_limit,
                max(abs(1.7 * yaw_error), minimum_speed),
            )
            context.controller.rotate_in_place(math.copysign(angular_speed, yaw_error))
            return RUNNING, True

        context.controller.stop(reason="door_pass_alignment_hold")
        self._pass_alignment_count += 1
        if self._pass_alignment_count < self._PASS_ALIGN_CONFIRMATIONS:
            return RUNNING, False
        return DONE, False

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
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
        target_xy: tuple[float, float],
        *,
        allow_direct_fallback: bool = False,
        planning_max_iterations: int | None = DEFAULT_MAX_ITERATIONS,
        max_linear: float | None = None,
        goal_tolerance: float | None = None,
    ) -> bool:
        self._last_navigation_blocked = False
        self._last_navigation_stuck = False
        effective_tolerance = (
            float(context.controller.goal_tolerance)
            if goal_tolerance is None
            else min(float(context.controller.goal_tolerance), float(goal_tolerance))
        )
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= effective_tolerance:
            context.controller.clear_path()
            return True

        if not context.controller.has_path():
            grid = context.mapper.floor(context.current_floor)
            result = context.plan_path(
                grid,
                (pose[0], pose[1]),
                target_xy,
                max_iterations=planning_max_iterations,
            )
            if result.reached:
                path_tolerance = None
                if goal_tolerance is not None:
                    # A* 可能量化或替换终点；扣除实际偏差，保证到点后仍在任务站位窗内。
                    endpoint_offset = math.hypot(
                        result.path[-1][0] - target_xy[0],
                        result.path[-1][1] - target_xy[1],
                    )
                    path_tolerance = effective_tolerance - endpoint_offset
                    if path_tolerance <= 0.0:
                        self._last_navigation_blocked = True
                        return False
                context.controller.set_path(
                    result.path,
                    max_linear=max_linear,
                    goal_tolerance=path_tolerance,
                    reason="door_navigate_planned_path",
                )
            elif result.unreachable:
                if allow_direct_fallback and context.segment_is_traversable(
                    grid, (pose[0], pose[1]), target_xy
                ):
                    context.controller.set_path(
                        [(pose[0], pose[1]), target_xy],
                        max_linear=max_linear,
                        goal_tolerance=(
                            effective_tolerance
                            if goal_tolerance is not None
                            else None
                        ),
                        reason="door_navigate_direct_path",
                    )
                else:
                    self._last_navigation_blocked = True
                    return False
            else:
                # 预算耗尽：可能可达，不退化为直连也不标记阻塞，等下轮重新规划。
                return False

        status = context.controller.poll_status()
        if status == STUCK:
            if self.phase in ("GO_PASS_POINT", "GO_COMMIT_POINT"):
                self._last_navigation_stuck = True
            else:
                context.controller.recover()
        # 宽松站位窗由下一 tick 的合法位姿验收，不能仅凭栅格终点的 DONE。
        return status == DONE and goal_tolerance is None

    def _door_centerline_error(
        self,
        pose: tuple[float, float, float],
    ) -> float:
        """返回当前位置到实测门中心轴的横向距离。"""
        if self._wait_point is None or self._pass_point is None:
            return math.inf
        axis_x = self._pass_point[0] - self._wait_point[0]
        axis_y = self._pass_point[1] - self._wait_point[1]
        axis_norm = math.hypot(axis_x, axis_y)
        if axis_norm <= 1e-6:
            return math.inf
        offset_x = pose[0] - self._wait_point[0]
        offset_y = pose[1] - self._wait_point[1]
        return abs(offset_x * axis_y - offset_y * axis_x) / axis_norm

    def _start_main_entrance_centering(self) -> None:
        if self._main_entrance_centering_started_at is None:
            self._main_entrance_centering_started_at = get_time()
            self._main_entrance_centering_started_wall = time.monotonic()

    def _main_entrance_centering_timed_out(self) -> bool:
        if self._main_entrance_centering_started_at is None:
            return False
        return (
            get_time() - self._main_entrance_centering_started_at
            >= self._MAIN_ENTRANCE_CENTERING_TIMEOUT
            or self._main_entrance_centering_started_wall is not None
            and time.monotonic() - self._main_entrance_centering_started_wall
            >= self._SERVICE_WALL_TIMEOUT
        )

    def _can_open_from_safe_approach(
        self,
        pose: tuple[float, float, float],
    ) -> bool:
        """等待点被栅格隔离时，允许在同侧且足够近的位置原地开门。"""
        if self._wait_point is None or self._pass_point is None:
            return False
        center = self.door_center
        if center is None:
            return False
        center_x, center_y = center
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
            self.commit_distance + self.wait_offset,
            2.0 * self.wait_offset,
        )
        return (
            signed_wait_side >= self.wait_offset
            and lateral_offset <= lateral_limit
            and distance_to_center <= safe_approach_limit
        )

    def _handle_wait_navigation_blocked(self, context: NavigationTaskContext) -> str:
        """等待短暂地图更新，但不允许不可达等待点形成永久零速。"""
        context.controller.stop(reason="door_wait_point_blocked")
        now = get_time()
        if self._wait_navigation_blocked_since is None:
            self._wait_navigation_blocked_since = now
            return RUNNING
        if now - self._wait_navigation_blocked_since < self._wait_navigation_timeout:
            return RUNNING
        return self._fail_crossing(context, "开门等待点持续不可达")

    def _handle_pass_blocked(self, context: NavigationTaskContext) -> str:
        context.controller.stop(reason="door_pass_blocked")
        now = get_time()
        if self._pass_retry_start_time is None:
            self._pass_retry_start_time = now
            return RUNNING
        if now - self._pass_retry_start_time < self._pass_retry_timeout:
            return RUNNING
        return self._fail_crossing(context, "过门通过点持续被地图阻塞")

    def _fail_crossing(self, context: NavigationTaskContext, reason: str) -> str:
        context.controller.clear_path(reason="door_crossing_failed")
        context.record_failure(reason)
        if rospy is not None:
            rospy.logwarn("[%s] %s，终止本次过门任务", self.name, reason)
        return FAILED

    def _handle_commit_blocked(self, context: NavigationTaskContext) -> str:
        _clear_opened_door_area(context, self.door_id)
        context.controller.clear_path()
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
            context.record_failure("过门 commit 无可达路径")
            return FAILED
        return RUNNING

    def _run_service(self, context: NavigationTaskContext, task_factory) -> str:
        if self.failure_reason == "door_service_timeout":
            return FAILED
        if self._service_task is None:
            context.controller.clear_path(reason="door_service_wait")
            self._service_task = task_factory()
            self._service_started_at = get_time()
            self._service_started_wall = time.monotonic()
            context.log_structured({
                "event": "door_service_started", "door_id": self.door_id,
                "phase": self.phase, "attempt": self._retry_count + 1,
            })
            return RUNNING

        if not self._service_task.done():
            if (
                get_time() - self._service_started_at > self._SERVICE_TIMEOUT
                or time.monotonic() - self._service_started_wall > self._SERVICE_WALL_TIMEOUT
            ):
                # 未收到响应不能推定环境动作已取消，不清空任务并重发请求。
                self.failure_reason = "door_service_timeout"
                context.log_structured({
                    "event": "door_service_timeout", "door_id": self.door_id,
                    "elapsed": get_time() - self._service_started_at,
                    "wall_elapsed": time.monotonic() - self._service_started_wall,
                    "request_cancelled": False,
                })
                return self._fail_crossing(context, self.failure_reason)
            context.controller.stop()
            return RUNNING

        ok = self._service_task.ok()
        message = self._service_task.message()
        context.log_structured({
            "event": "door_service_finished", "door_id": self.door_id,
            "ok": bool(ok), "elapsed": get_time() - self._service_started_at,
        })
        if not ok:
            if rospy is not None:
                rospy.logwarn(f"[{self.name}] 服务失败：{message}")
            if self._retry_count < 1:
                self._retry_count += 1
                if rospy is not None:
                    rospy.loginfo(f"[{self.name}] 正在重试 ({self._retry_count}/1)...")
                self._service_task = None
                return RUNNING
            self.failure_reason = "door_service_failed"
            return self._fail_crossing(context, f"开门服务失败: {message}")

        self._service_task = None
        self._retry_count = 0
        return DONE


def _segment_crosses_blocked_cell(
    grid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    constraints=None,
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
        if int(grid.data[gy, gx]) in (BLACK, NEGATIVE, OCC) or (
            constraints is not None and cell in constraints.blocked_cells
        ):
            return True
    return False


def _clear_opened_door_area(context: NavigationTaskContext, door_id: str) -> None:
    context.clear_opened_door_area(door_id)


class TakeElevator(NavigationTask):
    """完整乘梯交互状态机。

    主流程为：到当前层厅外并对门—呼梯/开门—传感器确认开口—精确进厢—关门
    并朝目标出口—运输—目标层开门确认—确认逻辑楼层—精确出厢—关门。
    服务成功、固定等待时间和位姿 Z 都不能单独确认换层；只有目标层注册门开口
    获得传感器确认后才通过显式上下文提交楼层转换。
    """
    _CABIN_TRANSIT_ALIGN_TOLERANCE = 0.10
    _CABIN_TRANSIT_ALIGN_RELEASE_TOLERANCE = 0.15
    _CABIN_TRANSIT_ALIGN_CONFIRMATIONS = 2
    _CABIN_TRANSIT_POSITION_ACCEPTANCE = 0.25
    _CABIN_TRANSIT_CENTERLINE_TOLERANCE = 0.25
    _CABIN_TRANSIT_PREPARE_TIMEOUT = 30.0
    # POSITION/ALIGN 翻转次数上限：栅格量化终点与精确 cabin center 的完成语义
    # 不一致时会出现每 tick 翻转的零速死循环；acceptance 已对齐二者，这里仅作
    # 安全网，避免今后其他语义退化再次形成永久振荡。
    _CABIN_TRANSIT_POSITION_ALIGN_FLIP_LIMIT = 4
    # 控制器报 goal_reached 后允许任务层位姿追上的宽限 tick 数。
    # 控制器用控制频率的新鲜位姿判到达，任务层用 mission tick 起点那份较旧的
    # 位姿复算；两者容差同为 position_tolerance，但读取时刻不同。容差 0.15m 在
    # 0.6m/s 下只需 0.25s 穿过，10Hz 下仅约 2.5 个 tick，任务层几乎必然落后，
    # 于是把已经到位的跨越误判成 elevator_transit_incomplete_goal 并失败。
    _CABIN_TRANSIT_GOAL_GRACE_TICKS = 5
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
    _CABIN_TURN_TIMEOUT = 12.0
    _SERVICE_TIMEOUT = 30.0
    _SERVICE_WALL_TIMEOUT = 180.0

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
        self._cabin_transit_position_door_cleared = False
        self._cabin_transit_goal_grace = 0
        self._cabin_transit_align_started_at: float | None = None
        self._cabin_transit_align_started_wall: float | None = None
        self._cabin_transit_prepare_started_at: float | None = None
        self._cabin_transit_prepare_started_wall: float | None = None
        self._cabin_turn_completed = False
        self._cabin_turn_wait_start: float | None = None
        self._current_open_wait_wall = 0.0
        self._door_alignment_phase: str | None = None
        self._door_alignment_started_at = 0.0
        self._door_alignment_started_wall = 0.0
        self._door_alignment_drift_retries = 0
        self._service_started_at = 0.0
        self._service_started_wall = 0.0
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
        self._current_crossing_geometry = None
        self._target_crossing_geometry = None
        self._crossing_blocked_since: float | None = None
        self._early_call_elevator_started = False
        self._early_call_elevator_task = None
        self._landmark_relocalization_attempted = False
        self._door_confirmation_retry_counts: dict[str, int] = {}
        self._retry_sensor_baseline: float | None = None
        self._retry_sensor_started_at = 0.0
        self._retry_relocalization_completed = False

    def tick(self, context: NavigationTaskContext, pose: tuple[float, float, float]) -> str:
        """推进一个乘梯阶段；任一失败都显式返回 FAILED 而不猜测所在楼层。"""
        client = context.building_client
        try:
            current_wait, cabin_center = client.get_elevator_points(self.elevator_id, self.current_floor)
            target_wait, _ = client.get_elevator_points(self.elevator_id, self.target_floor)
        except ValueError as e:
            if rospy is not None:
                rospy.logerr(f"TakeElevator 失败: {e}")
            return FAILED
        if self._current_crossing_geometry is not None:
            current_wait = self._current_crossing_geometry.wait_point
            cabin_center = self._current_crossing_geometry.cabin_center
        if self._target_crossing_geometry is not None:
            target_wait = self._target_crossing_geometry.wait_point
            cabin_center = self._target_crossing_geometry.cabin_center
        refresh_wait = self._refresh_viewpoint(
            current_wait,
            cabin_center,
            float(
                getattr(
                    context,
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
            if not self._early_call_elevator_started:
                self._early_call_elevator_started = True
                try:
                    # 保留远距离预呼梯的并行收益，但门板必须一直保留到近场
                    # 注册和二次对准完成；深色门面是该阶段唯一可靠的定位特征。
                    self._early_call_elevator_task = client.call_elevator(
                        self.elevator_id, self.current_floor, False
                    )
                except Exception as exc:
                    if rospy is not None:
                        rospy.logwarn("[%s] 远距离预呼梯发起异常，回退至近场呼梯: %s", self.name, exc)

            if self._navigate_to(context, pose, refresh_wait):
                self._exit_alignment_count = 0
                self.phase = "ALIGN_TO_CURRENT_DOOR"
            elif self.failure_reason is not None:
                if (
                    self.failure_reason == "elevator_navigation_unreachable"
                    and self._begin_start_retreat_or_trace_approach(context, pose)
                ):
                    return RUNNING
                return FAILED
            return RUNNING

        if self.phase == "ELEVATOR_START_RETREAT":
            return self._tick_start_retreat(context, pose)

        if self.phase == "GO_TO_ELEVATOR_TRACE_APPROACH":
            if not self._approach_trace_candidates:
                return self._fail_navigation(
                    context,
                    "elevator_trace_approach_unavailable",
                    "没有可供电梯粗接近的安全历史点",
                )
            target = self._approach_trace_candidates[self._approach_trace_index]
            if self._navigate_to(context, pose, target):
                self._exit_alignment_count = 0
                self.phase = "ALIGN_TO_CURRENT_DOOR"
            elif self.failure_reason is not None:
                self._approach_trace_index += 1
                if self._approach_trace_index >= len(self._approach_trace_candidates):
                    return self._fail_navigation(
                        context,
                        "elevator_trace_approach_exhausted",
                        "所有电梯粗接近历史点均不可达",
                    )
                context.controller.clear_path(reason="elevator_trace_candidate_retry")
                self._reset_navigation_watchdog(clear_failure=True)
            return RUNNING

        if self.phase == "ALIGN_TO_CURRENT_DOOR":
            # 等待点只约束位置，A* 到达时的机身朝向可能让前向 RGB-D
            # 完全看不到门板。先朝传感器注册得到的轿厢中心对准，既为
            # 开门前后视觉确认保留同一视角，也避免侧着开始 ENTER。
            if self._tick_target_door_alignment(context, pose, cabin_center):
                if self.failure_reason is not None:
                    return FAILED
                self._exit_alignment_count = 0
                self.phase = (
                    "RELOCALIZE_CURRENT_ELEVATOR"
                    if context.relocalize_landmark_fn is not None
                    else (
                        "REFRESH_CURRENT_ELEVATOR_GEOMETRY"
                        if context.elevator_geometry_refresh_enabled
                        else "CALL_ELEVATOR_TO_CURRENT_FLOOR"
                    )
                )
            return RUNNING

        if self.phase == "RELOCALIZE_CURRENT_ELEVATOR":
            context.controller.stop(reason="elevator_landmark_relocalization")
            outcome = context.relocalize_landmark(
                "elevator",
                current_door_id,
                pose,
                f"{self.name}:elevator",
            )
            if outcome is not None and getattr(outcome, "status", IDLE) == RUNNING:
                return RUNNING
            self._landmark_relocalization_attempted = True
            if outcome is not None and bool(getattr(outcome, "applied", False)):
                self._current_crossing_geometry = None
                self._target_crossing_geometry = None
                context.controller.clear_path(reason="elevator_landmark_relocalized")
                self._reset_navigation_watchdog(clear_failure=True)
                self.phase = "GO_TO_ELEVATOR_DOOR"
            else:
                self.phase = (
                    "REFRESH_CURRENT_ELEVATOR_GEOMETRY"
                    if context.elevator_geometry_refresh_enabled
                    else "CALL_ELEVATOR_TO_CURRENT_FLOOR"
                )
            return RUNNING

        if self.phase == "REFRESH_CURRENT_ELEVATOR_GEOMETRY":
            context.controller.stop(reason="elevator_geometry_refresh")
            status = context.refresh_elevator_geometry(
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
                        "先到既有候梯点，开门和跨越仍需局部传感器确认",
                        self.name,
                    )
                context.controller.clear_path(
                    reason="elevator_geometry_refresh_unavailable"
                )
                self._reset_navigation_watchdog(clear_failure=True)
                self.phase = "GO_TO_REFRESHED_ELEVATOR_DOOR"
                return RUNNING
            if status != DONE:
                self.failure_reason = "elevator_geometry_refresh_failed"
                return FAILED
            context.controller.clear_path(reason="elevator_geometry_refreshed")
            self._reset_navigation_watchdog(clear_failure=True)
            self.phase = "GO_TO_REFRESHED_ELEVATOR_DOOR"
            return RUNNING

        if self.phase == "GO_TO_REFRESHED_ELEVATOR_DOOR":
            if self._navigate_to(context, pose, current_wait):
                self._exit_alignment_count = 0
                self.phase = "ALIGN_TO_REFRESHED_ELEVATOR_DOOR"
            elif self.failure_reason is not None:
                return FAILED
            return RUNNING

        if self.phase == "ALIGN_TO_REFRESHED_ELEVATOR_DOOR":
            if self._tick_target_door_alignment(context, pose, cabin_center):
                if self.failure_reason is not None:
                    return FAILED
                self._exit_alignment_count = 0
                self.phase = "CALL_ELEVATOR_TO_CURRENT_FLOOR"
            return RUNNING

        if self.phase == "CALL_ELEVATOR_TO_CURRENT_FLOOR":
            status = self._run_service(
                context,
                lambda: (
                    self._early_call_elevator_task
                    if self._early_call_elevator_task is not None
                    else client.call_elevator(self.elevator_id, self.current_floor, False)
                ),
            )
            if status == FAILED:
                return FAILED
            if status == DONE:
                self.phase = "OPEN_CURRENT_FLOOR_DOOR"
            return RUNNING

        if self.phase == "OPEN_CURRENT_FLOOR_DOOR":
            if context.elevator_visual_preparation_pending(current_door_id):
                context.controller.stop()
                return RUNNING
            self._begin_door_open_confirmation(context, current_door_id)
            status = self._run_service(context, lambda: client.open_door(current_door_id))
            if status == FAILED:
                self._finish_door_open_confirmation(context, current_door_id)
                return FAILED
            if status == DONE:
                self.phase = "WAIT_CURRENT_DOOR_OPEN"
                self._wait_start_time = get_time()
                self._current_open_wait_wall = time.monotonic()
            return RUNNING

        if self.phase == "RETRY_CURRENT_DOOR_SENSOR_REFRESH":
            context.controller.stop(reason="elevator_current_door_retry_refresh")
            if not self._retry_relocalization_completed:
                outcome = context.relocalize_landmark(
                    "elevator",
                    current_door_id,
                    pose,
                    f"{self.name}:current_door_retry",
                )
                if outcome is not None and getattr(outcome, "status", IDLE) == RUNNING:
                    return RUNNING
                self._retry_relocalization_completed = True
            if not self._retry_sensor_is_fresh(context):
                if self._retry_refresh_expired(context):
                    return self._fail_door_confirmation_exhausted(
                        context, current_door_id, target=False,
                        reason="elevator_current_door_retry_sensor_timeout",
                    )
                return RUNNING
            self._exit_alignment_count = 0
            self.phase = "RETRY_ALIGN_CURRENT_DOOR"
            return RUNNING

        if self.phase == "RETRY_ALIGN_CURRENT_DOOR":
            if self._tick_target_door_alignment(context, pose, cabin_center):
                if self.failure_reason is not None:
                    return FAILED
                self._exit_alignment_count = 0
                self.phase = "CALL_ELEVATOR_TO_CURRENT_FLOOR"
            return RUNNING

        if self.phase == "WAIT_CURRENT_DOOR_OPEN":
            context.controller.stop()
            if time.monotonic() - self._current_open_wait_wall > self._SERVICE_WALL_TIMEOUT:
                return self._fail_navigation(
                    context, "elevator_current_entry_blocked_timeout",
                    "源层开门后的通行等待超过墙钟上限",
                )
            if not self._door_opening_is_confirmed(
                context,
                current_door_id,
                pose,
            ):
                if get_time() - self._wait_start_time > self._door_confirmation_timeout(
                    context, current_door_id
                ):
                    if self._start_door_confirmation_retry(
                        context, current_door_id, target=False
                    ):
                        return RUNNING
                    return self._fail_door_confirmation_exhausted(
                        context, current_door_id, target=False
                    )
                return RUNNING
            if context.elevator_local_crossing_geometry_enabled:
                # 与目标层一致：由新鲜局部几何负责穿越检查，不等待旧地图先连通。
                self._reset_cabin_transit()
                self.phase = "REFRESH_CURRENT_OPEN_CROSSING"
                return RUNNING
            grid = context.mapper.floor(context.current_floor)
            # 门已由局部传感器确认打开后，轿厢内部仍可能因尚未进入而保持
            # UNKNOWN。普通 A* 只接受 FREE/VISITED，会形成“先进入才能建图、
            # 先建图才能进入”的循环。主动跨越阶段仍会重新检查实际起点到
            # 目标间的 OCC/BLACK，并保留局部正障碍制动。
            if not _segment_crosses_blocked_cell(
                grid.without_negative_obstacles(),
                current_wait,
                cabin_center,
                constraints=None,
            ):
                self._reset_cabin_transit()
                self.phase = "ENTER_CABIN"
            elif get_time() - self._wait_start_time > self._SERVICE_TIMEOUT:
                return self._fail_navigation(
                    context, "elevator_current_entry_blocked_timeout",
                    "源层门已开放，但进梯正障碍检查持续不通过",
                )
            return RUNNING

        if self.phase == "REFRESH_CURRENT_OPEN_CROSSING":
            context.controller.stop(reason="elevator_local_crossing_refresh")
            door_center = self._door_center(current_wait, cabin_center)
            expected_yaw = math.atan2(
                door_center[1] - pose[1],
                door_center[0] - pose[0],
            )
            result = context.refresh_elevator_crossing_geometry(
                door_id=current_door_id,
                mode="enter",
                expected_crossing_yaw=expected_yaw,
                old_door_center=door_center,
                pose=pose,
            )
            if result is None or result.status == RUNNING:
                return RUNNING
            if result.status != DONE or result.geometry is None:
                if rospy is not None:
                    rospy.logwarn(
                        "[%s] 当前层开门后的 Livox 穿越几何未能连续确认（%s），平滑降级使用已注册电梯门几何",
                        self.name,
                        result.failure or "elevator_open_crossing_geometry_unavailable",
                    )
                self._reset_cabin_transit()
                self.phase = "ENTER_CABIN"
                return RUNNING
            self._current_crossing_geometry = result.geometry
            context.clear_elevator_crossing_area(result.geometry)
            self._reset_cabin_transit()
            self.phase = "ENTER_CABIN"
            return RUNNING

        if self.phase == "ENTER_CABIN":
            status = self._tick_cabin_transit(
                context,
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
                    context.controller.recover(reason="elevator_enter_position_stuck")
                    return RUNNING
                if rospy is not None:
                    rospy.logerr(
                        "[%s] 进入轿厢主动跨越失败：stage=%s",
                        self.name,
                        self.cabin_transit_stage,
                    )
                return FAILED
            elif status == FAILED:
                # 必须显式上抛：_navigate_to_with_status 的确定性失败
                # （unreachable/budget/timeout 达上限）返回 FAILED，若不在此
                # 判定就会掉进末尾的 return RUNNING 被吞掉。届时
                # elevator_planning_retry_limit 的判决每 tick 触发又每 tick
                # 丢弃，计数器不因失败重置，形成零速无界活锁
                # （seed 8127 实测 45.6s 内触发 42 次仍未退出）。
                if rospy is not None:
                    rospy.logerr(
                        "[%s] 进入轿厢准备阶段确定性失败：stage=%s reason=%s",
                        self.name,
                        self.cabin_transit_stage,
                        self.failure_reason,
                    )
                return FAILED
            return RUNNING

        if self.phase in {
            "CLOSE_CURRENT_FLOOR_DOOR", "CALL_ELEVATOR_TO_TARGET_FLOOR",
            "OPEN_TARGET_FLOOR_DOOR", "WAIT_TARGET_DOOR_OPEN",
            "RETRY_TARGET_DOOR_SENSOR_REFRESH",
        }:
            if self._tick_parallel_cabin_turn(context, pose, target_wait) == FAILED:
                return FAILED

        if self.phase == "CLOSE_CURRENT_FLOOR_DOOR":
            status = self._run_service(
                context,
                lambda: client.close_door(current_door_id),
                hold_position=False,
            )
            if status == FAILED:
                return FAILED
            if status == DONE:
                # 关门完成立即发起升梯，调头在升梯过程中继续并行完成，消除 0 层静止空等
                self.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
            return RUNNING

        if self.phase == "ALIGN_TO_TARGET_DOOR":
            if not self._cabin_turn_completed:
                aligned = self._tick_target_door_alignment(context, pose, target_wait)
                if aligned:
                    if self.failure_reason is not None:
                        return FAILED
                    self._cabin_turn_completed = True
                    self.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
            else:
                self.phase = "CALL_ELEVATOR_TO_TARGET_FLOOR"
            return RUNNING

        if self.phase == "CALL_ELEVATOR_TO_TARGET_FLOOR":
            if not self._floor_transition_started:
                try:
                    context.begin_floor_transition(
                        source_floor=self.current_floor,
                        target_floor=self.target_floor,
                        phase=self.phase,
                    )
                except (RuntimeError, ValueError) as exc:
                    if rospy is not None:
                        rospy.logerr("[%s] 无法锁存楼层过渡：%s", self.name, exc)
                    return FAILED
                self._floor_transition_started = True
            status = self._run_service(
                context,
                # 服务完成后立即显式开目标门；开门与调头并行，出梯再汇合。
                lambda: client.call_elevator(self.elevator_id, self.target_floor, False),
                hold_position=False,
            )
            if status == FAILED:
                return FAILED
            if status == DONE:
                self._cabin_turn_wait_start = get_time()
                self.phase = "OPEN_TARGET_FLOOR_DOOR"
            return RUNNING

        if self.phase == "WAIT_FOR_ODOM_STABLE":
            # 合规平面定位的 z 是楼层局部高度，不能用固定等待冒充换层确认。
            # 这里只给服务动作和新传感器帧留出稳定时间，逻辑楼层在目标门
            # 被传感器确认后才更新。
            context.controller.stop(reason="elevator_odom_stabilize")
            if get_time() - self._wait_start_time > 2.0:
                self.phase = "OPEN_TARGET_FLOOR_DOOR"
            return RUNNING

        if self.phase == "OPEN_TARGET_FLOOR_DOOR":
            self._begin_door_open_confirmation(context, target_door_id)
            status = self._run_service(
                context, lambda: client.open_door(target_door_id), hold_position=False
            )
            if status == FAILED:
                self._finish_door_open_confirmation(context, target_door_id)
                return FAILED
            if status == DONE:
                self.phase = "WAIT_TARGET_DOOR_OPEN"
                self._wait_start_time = get_time()
            return RUNNING

        if self.phase == "RETRY_TARGET_DOOR_SENSOR_REFRESH":
            # 目标层时机器人必须留在轿厢内；只等待新鲜局部观测后重新开门。
            if not self._retry_sensor_is_fresh(context):
                if self._retry_refresh_expired(context):
                    return self._fail_door_confirmation_exhausted(
                        context, target_door_id, target=True,
                        reason="elevator_target_door_retry_sensor_timeout",
                    )
                return RUNNING
            self.phase = "OPEN_TARGET_FLOOR_DOOR"
            return RUNNING

        if self.phase == "WAIT_TARGET_DOOR_OPEN":
            if not self._door_opening_is_confirmed(
                context,
                target_door_id,
                pose,
            ):
                if get_time() - self._wait_start_time > self._door_confirmation_timeout(
                    context, target_door_id
                ):
                    if self._start_door_confirmation_retry(
                        context, target_door_id, target=True
                    ):
                        return RUNNING
                    return self._fail_door_confirmation_exhausted(
                        context, target_door_id, target=True
                    )
                return RUNNING
            if not self._cabin_turn_completed:
                return RUNNING
            if not self._floor_transition_confirmed:
                try:
                    context.confirm_floor_transition(self.target_floor)
                except (RuntimeError, ValueError) as exc:
                    if rospy is not None:
                        rospy.logerr("[%s] 目标门已确认但楼层过渡无效：%s", self.name, exc)
                    return FAILED
                self._floor_transition_confirmed = True
            if context.elevator_local_crossing_geometry_enabled:
                self.phase = "REFRESH_TARGET_OPEN_CROSSING"
                return RUNNING
            grid = context.mapper.floor(self.target_floor)
            straight_segment_blocked = _segment_crosses_blocked_cell(
                # 这里只预检即将 CROSS 的直线段，不豁免当前对准或普通导航。
                grid.without_negative_obstacles(),
                cabin_center,
                target_wait,
                constraints=None,
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
            if self.phase == "WAIT_TARGET_DOOR_OPEN" and get_time() - self._wait_start_time > self._SERVICE_TIMEOUT:
                return self._fail_navigation(
                    context, "elevator_target_exit_blocked_timeout",
                    "目标门已开放，但出梯正障碍检查持续不通过",
                )
            return RUNNING

        if self.phase == "REFRESH_TARGET_OPEN_CROSSING":
            context.controller.stop(reason="elevator_local_crossing_refresh")
            door_center = self._door_center(
                cabin_center,
                target_wait,
                first_standoff=0.95,
            )
            expected_yaw = math.atan2(
                door_center[1] - pose[1],
                door_center[0] - pose[0],
            )
            result = context.refresh_elevator_crossing_geometry(
                door_id=target_door_id,
                mode="exit",
                expected_crossing_yaw=expected_yaw,
                old_door_center=door_center,
                pose=pose,
            )
            if result is None or result.status == RUNNING:
                return RUNNING
            if result.status != DONE or result.geometry is None:
                if rospy is not None:
                    rospy.logwarn(
                        "[%s] 目标层开门后的 Livox 穿越几何未能连续确认（%s），平滑降级使用已注册电梯门几何",
                        self.name,
                        result.failure or "elevator_open_crossing_geometry_unavailable",
                    )
                self._reset_cabin_transit()
                self.phase = "EXIT_CABIN"
                return RUNNING
            center_offset = math.hypot(
                result.geometry.cabin_center[0] - pose[0],
                result.geometry.cabin_center[1] - pose[1],
            )
            if center_offset > context.elevator_local_crossing_max_center_correction:
                return self._fail_navigation(
                    context,
                    "elevator_cabin_center_offset_exceeded",
                    "当前位姿与 Livox 轿厢中心偏差 %.3fm 超过 %.3fm"
                    % (
                        center_offset,
                        context.elevator_local_crossing_max_center_correction,
                    ),
                )
            self._target_crossing_geometry = result.geometry
            context.clear_elevator_crossing_area(result.geometry)
            self._reset_cabin_transit()
            self.phase = "EXIT_CABIN"
            return RUNNING

        if self.phase == "EXIT_CABIN":
            exit_linear_cap = context.exit_first_max_linear if self._retry_count == 0 else None
            status = self._tick_cabin_transit(
                context,
                pose,
                mode="exit",
                position_xy=cabin_center,
                target_xy=target_wait,
                door_id=target_door_id,
                max_linear=exit_linear_cap,
            )
            if status == DONE:
                self._reset_cabin_transit()
                if getattr(context, "elevator_async_target_door_close_enabled", True):
                    try:
                        client.close_door(target_door_id)
                        context.log_structured(
                            {
                                "event": "elevator_target_door_close_requested",
                                "door_id": str(target_door_id),
                                "mode": "async",
                            }
                        )
                    except Exception as exc:
                        if rospy is not None:
                            rospy.logwarn(
                                "[%s] 目标层电梯关门异步请求失败: %s",
                                self.name,
                                exc,
                            )
                    self._current_crossing_geometry = None
                    self._target_crossing_geometry = None
                    self.phase = "DONE"
                    return DONE
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
                    if not context.exit_cabin_backoff_enabled:
                        if rospy is not None:
                            rospy.logerr(
                                f"[{self.name}] 离开轿厢直线跨越卡住，默认禁用已证伪的倒车恢复。"
                            )
                        return FAILED
                    if not self._begin_exit_backoff(
                        context,
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
            elif status == FAILED:
                # 与 ENTER_CABIN 对称：确定性失败必须显式上抛，否则被
                # 末尾的 return RUNNING 吞掉，形成零速无界活锁。
                if rospy is not None:
                    rospy.logerr(
                        "[%s] 离开轿厢准备阶段确定性失败：stage=%s reason=%s",
                        self.name,
                        self.cabin_transit_stage,
                        self.failure_reason,
                    )
                return FAILED
            return RUNNING

        if self.phase.startswith("EXIT_BACKOFF_"):
            return self._tick_exit_backoff(context, pose, cabin_center)

        if self.phase == "CLOSE_TARGET_FLOOR_DOOR":
            status = self._run_service(context, lambda: client.close_door(target_door_id))
            if status == FAILED:
                return FAILED
            if status == DONE:
                self._current_crossing_geometry = None
                self._target_crossing_geometry = None
                self.phase = "DONE"
                return DONE
            return RUNNING

        if self.phase == "DONE":
            return DONE

        # 强制穷举：此前这里是裸 return DONE，任何未注册 phase（拼写错误、
        # 新增阶段忘记接分支、状态被外部改坏）都会被当成"乘梯完成"，任务层
        # 据此推进到下一层，实际根本没换层。这与 NAV-006 同类但更严重：
        # NAV-006 把 FAILED 吞成 RUNNING（活锁，至少不谎报成功），此处把
        # 未知吞成 DONE（假成功）。未知阶段无法安全推进，只能显式失败。
        return self._fail_navigation(
            context,
            "elevator_unknown_phase",
            f"未注册的乘梯阶段: {self.phase!r}",
        )

    def active_opened_door_id(self, client) -> str | None:
        """返回当前已确认开放且正在直线跨越的电梯门 ID。"""
        if self.cabin_transit_stage != "CROSS":
            return None
        if self.phase == "ENTER_CABIN":
            floor = self.current_floor
        elif self.phase == "EXIT_CABIN":
            floor = self.target_floor
        else:
            return None
        try:
            door_id = str(client.elevator_door_id(floor))
        except (AttributeError, TypeError, ValueError):
            return None
        return door_id if door_id in self._door_open_confirmed else None

    def _start_door_confirmation_retry(
        self,
        context: NavigationTaskContext,
        door_id: str,
        *,
        target: bool,
    ) -> bool:
        count = int(self._door_confirmation_retry_counts.get(door_id, 0))
        limit = max(0, int(context.elevator_opening_confirmation_retry_limit))
        if count >= limit:
            return False
        self._finish_door_open_confirmation(context, door_id)
        count += 1
        self._door_confirmation_retry_counts[door_id] = count
        self._retry_sensor_baseline = context.latest_cloud_stamp()
        self._retry_sensor_started_at = get_time()
        self._retry_relocalization_completed = False
        self._service_task = None
        if not target:
            # 远距离预呼梯任务已完成，重试必须真正重发呼梯。
            self._early_call_elevator_task = None
        self.phase = (
            "RETRY_TARGET_DOOR_SENSOR_REFRESH"
            if target
            else "RETRY_CURRENT_DOOR_SENSOR_REFRESH"
        )
        context.controller.clear_path(reason="elevator_door_confirmation_retry")
        context.log_structured(
            {
                "event": "elevator_door_confirmation_retry_started",
                "door_id": str(door_id),
                "side": "target" if target else "source",
                "attempt": count,
                "limit": limit,
                "sensor_refresh": float(
                    context.elevator_opening_confirmation_sensor_refresh
                ),
                "retry_timeout": float(
                    context.elevator_opening_confirmation_retry_timeout
                ),
            }
        )
        return True

    def _retry_sensor_is_fresh(self, context: NavigationTaskContext) -> bool:
        elapsed = get_time() - self._retry_sensor_started_at
        if elapsed < float(context.elevator_opening_confirmation_sensor_refresh):
            return False
        latest = context.latest_cloud_stamp()
        if latest is None:
            return False
        return self._retry_sensor_baseline is None or latest > self._retry_sensor_baseline

    def _retry_refresh_expired(self, context: NavigationTaskContext) -> bool:
        return get_time() - self._retry_sensor_started_at > float(
            context.elevator_opening_confirmation_retry_timeout
        )

    def _door_confirmation_timeout(
        self,
        context: NavigationTaskContext,
        door_id: str,
    ) -> float:
        if self._door_confirmation_retry_counts.get(door_id, 0) > 0:
            return float(context.elevator_opening_confirmation_retry_timeout)
        return float(context.elevator_opening_confirmation_timeout)

    def _fail_door_confirmation_exhausted(
        self,
        context: NavigationTaskContext,
        door_id: str,
        *,
        target: bool,
        reason: str | None = None,
    ) -> str:
        self._finish_door_open_confirmation(context, door_id)
        self.failure_reason = reason or (
            "elevator_target_door_confirmation_exhausted"
            if target
            else "elevator_current_door_confirmation_exhausted"
        )
        context.record_failure(self.failure_reason)
        context.log_structured(
            {
                "event": "elevator_door_confirmation_retry_exhausted",
                "door_id": str(door_id),
                "side": "target" if target else "source",
                "attempts": int(self._door_confirmation_retry_counts.get(door_id, 0)),
                "reason": self.failure_reason,
            }
        )
        if rospy is not None:
            rospy.logerr(
                "[%s] %s层电梯门局部开口确认重试耗尽: door_id=%s reason=%s",
                self.name,
                "目标" if target else "当前",
                door_id,
                self.failure_reason,
            )
        return FAILED

    def _begin_door_open_confirmation(self, context: NavigationTaskContext, door_id: str) -> None:
        if door_id in self._door_confirmation_started:
            return
        self._door_confirmation_started.add(door_id)
        context.begin_elevator_door_open_confirmation(door_id)

    def _door_opening_is_confirmed(
        self,
        context: NavigationTaskContext,
        door_id: str,
        pose: tuple[float, float, float],
    ) -> bool:
        if door_id in self._door_open_confirmed:
            return True
        if not context.confirm_elevator_door_open(door_id, pose):
            return False
        self._door_open_confirmed.add(door_id)
        context.log_structured({
            "event": "elevator_door_open_confirmed", "door_id": str(door_id),
            "phase": self.phase, "cabin_turn_completed": self._cabin_turn_completed,
        })
        _clear_opened_door_area(context, door_id)
        self._finish_door_open_confirmation(context, door_id)
        return True

    def _finish_door_open_confirmation(self, context: NavigationTaskContext, door_id: str) -> None:
        context.finish_elevator_door_open_confirmation(door_id)
        self._door_confirmation_started.discard(door_id)

    def _tick_parallel_cabin_turn(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
        target_wait: tuple[float, float],
    ) -> str:
        if self._cabin_turn_completed:
            context.controller.stop(reason="elevator_cabin_turn_completed")
            return DONE
        aligned = self._tick_target_door_alignment(context, pose, target_wait)
        if aligned and self.failure_reason is None:
            self._cabin_turn_completed = True
            context.log_structured({
                "event": "elevator_cabin_turn_completed", "phase": self.phase,
                "target_floor": self.target_floor, "pose": list(pose),
            })
            return DONE
        if (
            self._cabin_turn_wait_start is not None
            and get_time() - self._cabin_turn_wait_start > self._CABIN_TURN_TIMEOUT
        ):
            self.failure_reason = "elevator_cabin_turn_timeout"
        if self.failure_reason is not None:
            context.controller.clear_path(reason=self.failure_reason)
            context.record_failure(self.failure_reason)
            context.log_structured({
                "event": "elevator_cabin_turn_failed", "phase": self.phase,
                "reason": self.failure_reason, "pose": list(pose),
                "service_pending": self._service_task is not None,
            })
            return FAILED
        return RUNNING

    def _tick_target_door_alignment(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
        target_wait: tuple[float, float],
    ) -> bool:
        """利用关门/等待时间原地朝向目标（固定频率 ALIGN 模式）。

        对准由 PathFollower 控制定时器高频驱动（SETTLE->ALIGN->CONFIRM），
        本函数只幂等进入与轮询结果：
        - DONE+alignment_done：返回 True（进入下一阶段）；
        - STUCK+alignment_drift：重新进入对准（位置漂移后重试）；
        - 其他 STUCK（settle 超时等）：设置 failure_reason 并返回 True，
          调用方必须检查 failure_reason 走失败路径。
        """
        controller = context.controller
        bounded_door_alignment = self.phase in {
            "ALIGN_TO_CURRENT_DOOR", "ALIGN_TO_REFRESHED_ELEVATOR_DOOR",
            "RETRY_ALIGN_CURRENT_DOOR", "ALIGN_TO_TARGET_DOOR",
        }
        if bounded_door_alignment:
            if self._door_alignment_phase != self.phase:
                self._door_alignment_phase = self.phase
                self._door_alignment_started_at = get_time()
                self._door_alignment_started_wall = time.monotonic()
                self._door_alignment_drift_retries = 0
            if (
                get_time() - self._door_alignment_started_at > self._CABIN_TURN_TIMEOUT
                or time.monotonic() - self._door_alignment_started_wall > self._SERVICE_WALL_TIMEOUT
            ):
                self.failure_reason = "elevator_door_alignment_timeout"
                controller.clear_path(reason=self.failure_reason)
                context.record_failure(self.failure_reason)
                context.log_structured({
                    "event": "elevator_door_alignment_failed", "phase": self.phase,
                    "reason": self.failure_reason,
                    "elapsed": get_time() - self._door_alignment_started_at,
                    "wall_elapsed": time.monotonic() - self._door_alignment_started_wall,
                })
                return True
        result = controller.poll_result()
        if result.status == DONE and result.reason == "alignment_done":
            self._door_alignment_phase = None
            return True
        if result.status == STUCK:
            if result.reason == "alignment_drift":
                if bounded_door_alignment:
                    self._door_alignment_drift_retries += 1
                    if self._door_alignment_drift_retries > 3:
                        self.failure_reason = "elevator_door_alignment_retry_exhausted"
                        controller.clear_path(reason=self.failure_reason)
                        context.record_failure(self.failure_reason)
                        return True
                # 位置漂移：锚点已失效，重新进入对准（重新 SETTLE）。
                self._exit_alignment_count = 0
                controller.begin_heading_alignment(target_wait, max_angular=0.65)
                return False
            self.failure_reason = "elevator_alignment_failed:%s" % (
                result.reason or "unknown"
            )
            return True
        if result.status == IDLE:
            controller.begin_heading_alignment(target_wait, max_angular=0.65)
            return False
        return False

    def _commit_cabin_cross(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
        *,
        target_door_id: str,
        position_tolerance: float,
        max_linear: float | None,
    ) -> str:
        """对准完成后提交轿厢直线跨越路径（门区清理与阻塞检查保留）。

        返回 RUNNING（进入 CROSS 或等待路径解除阻塞）。
        """
        if not self._cabin_position_ready(
            pose,
            position_xy,
            target_xy,
            self._cabin_transit_position_acceptance(context),
            self._CABIN_TRANSIT_CENTERLINE_TOLERANCE,
        ):
            return self._retry_cabin_position(
                context, pose, position_xy, target_xy
            )
        if not self._cabin_transit_door_clear_refreshed:
            _clear_opened_door_area(context, target_door_id)
            self._cabin_transit_door_clear_refreshed = True
        grid = context.mapper.floor(context.current_floor)
        commit_start = (pose[0], pose[1])
        crossing_geometry = (
            self._current_crossing_geometry
            if self._cabin_transit_mode == "enter"
            else self._target_crossing_geometry
        )
        if crossing_geometry is None and _segment_crosses_blocked_cell(
            grid.without_negative_obstacles(),
            commit_start,
            target_xy,
            constraints=None,
        ):
            self._cabin_transit_alignment_count = 0
            if rospy is not None:
                rospy.logwarn_throttle(
                    2.0,
                    "[%s] 电梯直线跨越路径仍被地图阻塞，保持原地等待",
                    self.name,
                )
            return RUNNING
        if crossing_geometry is not None:
            obstacle = context.elevator_crossing_obstacle(
                commit_start,
                target_xy,
                crossing_geometry,
            )
            if obstacle is not None:
                now = get_time()
                if self._crossing_blocked_since is None:
                    self._crossing_blocked_since = now
                if now - self._crossing_blocked_since > context.elevator_local_crossing_timeout:
                    self.failure_reason = "elevator_crossing_corridor_blocked"
                    context.record_failure(self.failure_reason)
                    context.controller.stop(reason=self.failure_reason)
                    return STUCK
                self._cabin_transit_alignment_count = 0
                context.controller.stop(reason="elevator_crossing_corridor_wait")
                return RUNNING
        self._crossing_blocked_since = None
        self._cabin_transit_start_xy = commit_start
        self._cabin_transit_target_xy = target_xy
        # 先发布阶段，再装载路径，统一速度出口才能在首条命令就采用 CROSS 策略。
        self.cabin_transit_stage = "CROSS"
        context.controller.set_path(
            [commit_start, target_xy],
            max_linear=max_linear,
            goal_tolerance=position_tolerance,
            obstacle_stop_distance=0.15,
            obstacle_slow_distance=0.30,
            reason="elevator_cabin_cross_path",
        )
        self._cabin_transit_position_align_flips = 0
        self._log_cabin_transit_stage(
            "CROSS",
            pose,
            commit_start,
            target_xy,
        )
        return RUNNING

    def _tick_cabin_transit(
        self,
        context: NavigationTaskContext,
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

        if self.cabin_transit_stage in {"POSITION", "ALIGN"}:
            if (
                get_time() - self._cabin_transit_prepare_started_at
                >= self._CABIN_TRANSIT_PREPARE_TIMEOUT
                or time.monotonic() - self._cabin_transit_prepare_started_wall
                >= self._SERVICE_WALL_TIMEOUT
            ):
                return self._fail_navigation(
                    context, "elevator_cabin_prepare_timeout", "电梯站位与对准总超时"
                )

        position_tolerance = self._cabin_transit_position_tolerance(context)
        # 历史成功进梯样本距等待点约0.21m、横向最大约0.20m；
        # POSITION只需落入宽松站位窗，随后由ALIGN完成朝向收敛。
        position_acceptance = self._cabin_transit_position_acceptance(context)

        if self.cabin_transit_stage == "POSITION":
            if not self._cabin_transit_position_door_cleared:
                # 门已由局部传感器确认开启（WAIT_*_DOOR_OPEN 的前置条件），但门板
                # 残留的膨胀 OCC 仍覆盖候梯点与机器人当前格。此前只在
                # ALIGN->CROSS 提交时才清理，导致 POSITION 规划时起点仍困在膨胀
                # 域内，A* 返回 obstacle_inflation_escape_unreachable，形成
                # “门区要等 POSITION 完成才清、POSITION 又被没清的门区挡住”的
                # 死锁。clear_opened_door_area 的清理深度本就按门两侧等待点加
                # 膨胀半径设计，正是为此场景准备。
                _clear_opened_door_area(context, door_id)
                self._cabin_transit_position_door_cleared = True
            if self._cabin_position_ready(
                pose,
                position_xy,
                target_xy,
                position_acceptance,
                self._CABIN_TRANSIT_CENTERLINE_TOLERANCE,
            ):
                context.controller.clear_path(reason="elevator_centered_position_reached")
                self.cabin_transit_stage = "ALIGN"
                self._start_cabin_transit_alignment_timer()
                self._cabin_transit_alignment_count = 0
                self._log_cabin_transit_stage(
                    "ALIGN",
                    pose,
                    position_xy,
                    target_xy,
                )
                return RUNNING
            status = self._navigate_to_with_status(
                context,
                pose,
                position_xy,
                goal_tolerance=min(
                    position_acceptance, self._CABIN_TRANSIT_CENTERLINE_TOLERANCE
                ),
            )
            if status == DONE:
                # 控制器终态使用控制频率的新位姿，当前 tick 的 pose 可能更旧。
                # 下一 tick 用刷新后的位姿重新验收位置和中心线，再决定是否 ALIGN。
                return RUNNING
            return status

        if self.cabin_transit_stage == "ALIGN":
            self._start_cabin_transit_alignment_timer()
            if self._cabin_transit_alignment_timed_out():
                return self._fail_navigation(
                    context, "elevator_cabin_alignment_timeout", "电梯中心线朝向对准超时"
                )
            # 固定频率对准模式：SETTLE 停稳 -> ALIGN 旋转 -> CONFIRM 释放。
            # 进梯锚定门外中心线站位并朝轿厢中心；出梯锚定轿厢中心线站位
            # 并朝厅外等待点。漂移由控制器锁存 STUCK 后退回 POSITION。
            if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= position_tolerance:
                context.controller.clear_path(reason="elevator_transit_already_complete")
                return DONE
            result = context.controller.poll_result()
            if result.status == DONE and result.reason == "alignment_done":
                self._cabin_transit_alignment_count = 0
                # 对准完成：提交直线跨越路径（门区清理与阻塞检查保留）。
                return self._commit_cabin_cross(
                    context,
                    pose,
                    position_xy,
                    target_xy,
                    target_door_id=door_id,
                    position_tolerance=position_tolerance,
                    max_linear=max_linear,
                )
            if result.status == STUCK and result.reason == "alignment_drift":
                return self._retry_cabin_position(
                    context, pose, position_xy, target_xy
                )
            if result.status == STUCK:
                # 其他 STUCK（settle 超时等）不能当 drift 吞掉：显式失败。
                if rospy is not None:
                    rospy.logerr(
                        "[%s] 电梯轿厢对准失败：mode=%s reason=%s",
                        self.name,
                        self._cabin_transit_mode,
                        result.reason,
                    )
                return STUCK
            if result.status == IDLE:
                context.controller.begin_heading_alignment(
                    target_xy,
                    anchor_xy=position_xy,
                    max_anchor_error=position_acceptance,
                )
            return RUNNING

        if self.cabin_transit_stage == "CROSS":
            start_xy = self._cabin_transit_start_xy
            end_xy = self._cabin_transit_target_xy
            if start_xy is None or end_xy is None:
                context.controller.stop(reason="elevator_transit_missing_commit")
                return STUCK

            complete, progress, cross_track, endpoint_distance = self._cabin_transit_complete(
                pose,
                start_xy,
                end_xy,
                position_tolerance,
            )

            if complete:
                context.controller.clear_path(reason="elevator_transit_completed")
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

            if self._cabin_transit_goal_grace > 0:
                # 控制器已按同一容差判到终点并消费终态清掉路径，此时 has_path()
                # 必然为 False。宽限期内不能把它当成路径丢失：只等任务层位姿
                # 追上（零速等待），预算用尽才判失败。
                self._cabin_transit_goal_grace -= 1
                context.controller.stop(reason="elevator_transit_goal_grace")
                if self._cabin_transit_goal_grace > 0:
                    return RUNNING
                if rospy is not None:
                    rospy.logerr(
                        "[%s] CROSS阶段宽限用尽仍未确认到达：mode=%s "
                        "endpoint_distance=%.3f tolerance=%.3f progress=%.3f cross_track=%.3f",
                        self.name,
                        self._cabin_transit_mode,
                        endpoint_distance,
                        position_tolerance,
                        progress,
                        cross_track,
                    )
                return STUCK

            if not context.controller.has_path():
                context.controller.stop(reason="elevator_transit_path_lost")
                if rospy is not None:
                    rospy.logerr(
                        "[%s] CROSS阶段路径丢失：stage=%s mode=%s endpoint_distance=%.3f",
                        self.name,
                        self.cabin_transit_stage,
                        self._cabin_transit_mode,
                        math.hypot(end_xy[0] - pose[0], end_xy[1] - pose[1]) if end_xy else -1,
                    )
                return STUCK

            status = context.controller.poll_status()

            # 每 5 秒记录一次 CROSS 进度，用于定位失败前的收敛趋势。
            # 时间源必须用 time_utils.get_time()：rospy.get_time() 在 rospy 可导入
            # 但未 init_node 的进程（单元测试、离线回放）会抛 ROSInitException。
            # 复用上面已取得的 status，避免重复轮询控制器。
            if rospy is not None:
                now = get_time()
                if now - getattr(self, "_cross_last_log_time", 0.0) >= 5.0:
                    self._cross_last_log_time = now
                    rospy.loginfo(
                        "[%s] CROSS进行中: endpoint_distance=%.3f tolerance=%.3f "
                        "progress=%.3f cross_track=%.3f status=%s path_active=%s",
                        self.name,
                        endpoint_distance,
                        position_tolerance,
                        progress,
                        cross_track,
                        status,
                        context.controller.has_path(),
                    )

            if status == DONE:
                complete, progress, cross_track, endpoint_distance = self._cabin_transit_complete(
                    pose,
                    start_xy,
                    end_xy,
                    position_tolerance,
                )
                if complete:
                    self._cabin_transit_goal_grace = 0
                    return DONE
                # 控制器按同一容差判到了、任务层位姿还没追上：装填宽限，
                # 零速等位姿刷新，不在这一 tick 就判失败。
                self._cabin_transit_goal_grace = self._CABIN_TRANSIT_GOAL_GRACE_TICKS
                context.controller.stop(reason="elevator_transit_goal_grace")
                if rospy is not None:
                    rospy.logwarn(
                        "[%s] CROSS阶段控制器报告DONE但任务层未确认到达，进入宽限："
                        "endpoint_distance=%.3f tolerance=%.3f progress=%.3f cross_track=%.3f grace=%d",
                        self.name,
                        endpoint_distance,
                        position_tolerance,
                        progress,
                        cross_track,
                        self._cabin_transit_goal_grace,
                    )
                return RUNNING
            if status == IDLE:
                context.controller.stop(reason="elevator_transit_unexpected_idle")
                if rospy is not None:
                    rospy.logerr(
                        "[%s] CROSS阶段控制器意外进入IDLE：endpoint_distance=%.3f",
                        self.name,
                        math.hypot(end_xy[0] - pose[0], end_xy[1] - pose[1]) if end_xy else -1,
                    )
                return STUCK
            return status

        context.controller.stop(reason="elevator_transit_unknown_stage")
        return STUCK

    def _begin_cabin_transit(
        self,
        mode: str,
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
    ) -> None:
        self._cabin_transit_mode = mode
        # 进厢先到门外中心线站位，离厢先回到轿厢中心线站位；两者都在
        # POSITION 满足后才朝穿越目标对准，禁止从偏心位置斜穿门框。
        self.cabin_transit_stage = "POSITION"
        self._cabin_transit_alignment_count = 0
        self._cabin_transit_position_align_flips = 0
        self._cabin_transit_start_xy = None
        self._cabin_transit_target_xy = target_xy
        self._cabin_transit_door_clear_refreshed = False
        self._cabin_transit_position_door_cleared = False
        self._cabin_transit_goal_grace = 0
        self._cabin_transit_align_started_at = None
        self._cabin_transit_align_started_wall = None
        self._cabin_transit_prepare_started_at = get_time()
        self._cabin_transit_prepare_started_wall = time.monotonic()
        self._crossing_blocked_since = None
        self._log_cabin_transit_stage(
            self.cabin_transit_stage,
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
        self._cabin_transit_position_door_cleared = False
        self._cabin_transit_goal_grace = 0
        self._cabin_transit_align_started_at = None
        self._cabin_transit_align_started_wall = None
        self._cabin_transit_prepare_started_at = None
        self._cabin_transit_prepare_started_wall = None
        self._crossing_blocked_since = None

    def _retry_cabin_position(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
    ) -> str:
        self._cabin_transit_position_align_flips += 1
        if self._cabin_transit_position_align_flips > self._CABIN_TRANSIT_POSITION_ALIGN_FLIP_LIMIT:
            return self._fail_navigation(
                context, "elevator_transit_position_align_oscillation", "电梯重新站位次数耗尽"
            )
        context.controller.clear_path(reason="elevator_centerline_drift")
        self.cabin_transit_stage = "POSITION"
        self._cabin_transit_alignment_count = 0
        self._log_cabin_transit_stage("POSITION", pose, position_xy, target_xy)
        return RUNNING

    def _start_cabin_transit_alignment_timer(self) -> None:
        if self._cabin_transit_align_started_at is None:
            self._cabin_transit_align_started_at = get_time()
            self._cabin_transit_align_started_wall = time.monotonic()

    def _cabin_transit_alignment_timed_out(self) -> bool:
        if self._cabin_transit_align_started_at is None:
            return False
        return (
            get_time() - self._cabin_transit_align_started_at
            >= self._CABIN_TURN_TIMEOUT
            or self._cabin_transit_align_started_wall is not None
            and time.monotonic() - self._cabin_transit_align_started_wall
            >= self._SERVICE_WALL_TIMEOUT
        )

    @staticmethod
    def _cabin_position_ready(
        pose: tuple[float, float, float],
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
        distance_tolerance: float,
        centerline_tolerance: float,
    ) -> bool:
        """检查站位既接近目标点，又位于目标穿越中心线上。"""
        if math.hypot(position_xy[0] - pose[0], position_xy[1] - pose[1]) > distance_tolerance:
            return False
        axis_x = target_xy[0] - position_xy[0]
        axis_y = target_xy[1] - position_xy[1]
        axis_norm = math.hypot(axis_x, axis_y)
        if axis_norm <= 1e-6:
            return False
        offset_x = pose[0] - position_xy[0]
        offset_y = pose[1] - position_xy[1]
        cross_track = abs(offset_x * axis_y - offset_y * axis_x) / axis_norm
        return cross_track <= centerline_tolerance

    @staticmethod
    def _door_center(
        first_xy: tuple[float, float],
        second_xy: tuple[float, float],
        *,
        first_standoff: float = _DEFAULT_WAIT_STANDOFF,
    ) -> tuple[float, float]:
        """按既有 0.80/0.95m 候梯点语义恢复注册门面中心。"""
        dx = second_xy[0] - first_xy[0]
        dy = second_xy[1] - first_xy[1]
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return first_xy
        ratio = float(first_standoff) / distance
        return (first_xy[0] + ratio * dx, first_xy[1] + ratio * dy)

    @staticmethod
    def _cabin_transit_position_tolerance(context: NavigationTaskContext) -> float:
        goal_tolerance = max(1e-3, float(context.controller.goal_tolerance))
        corner_tolerance = float(
            getattr(context.controller, "corner_tolerance", goal_tolerance)
        )
        if not math.isfinite(corner_tolerance) or corner_tolerance <= 0.0:
            corner_tolerance = goal_tolerance
        return min(goal_tolerance, corner_tolerance)

    @staticmethod
    def _cabin_transit_position_acceptance(context: NavigationTaskContext) -> float:
        """历史成功站位窗；横向另限0.25m，不要求精确踩中目标坐标。"""
        goal_tolerance = max(1e-3, float(context.controller.goal_tolerance))
        return min(goal_tolerance, TakeElevator._CABIN_TRANSIT_POSITION_ACCEPTANCE)

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
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
        cabin_center: tuple[float, float],
        target_wait: tuple[float, float],
    ) -> bool:
        if not self._prepare_exit_backoff_segment(context, pose, cabin_center):
            return False
        reverse_speed = float(getattr(context.controller, "reverse_speed", 0.0))
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
        context.controller.clear_path(reason="exit_backoff_start")
        context.controller.stop(reason="exit_backoff_settle")
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
        context: NavigationTaskContext,
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
        grid = context.mapper.floor(context.current_floor)
        if _segment_crosses_blocked_cell(
            grid,
            (pose[0], pose[1]),
            target_xy,
            constraints=context.planning_constraints,
        ):
            return False
        self._exit_backoff_start_xy = (pose[0], pose[1])
        self._exit_backoff_target_xy = target_xy
        self._exit_backoff_direction = direction
        return True

    def _tick_exit_backoff(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
        cabin_center: tuple[float, float],
    ) -> str:
        now = get_time()
        if self.phase == "EXIT_BACKOFF_SETTLE":
            context.controller.stop(reason="exit_backoff_settle")
            settle_duration = max(0.0, float(getattr(context.controller, "recover_duration", 0.5)))
            if now - self._exit_backoff_phase_start >= settle_duration:
                self.phase = "EXIT_BACKOFF_ALIGN"
                self._exit_backoff_phase_start = now
            return RUNNING

        if self.phase in {"EXIT_BACKOFF_ALIGN", "EXIT_BACKOFF_REALIGN"}:
            if self._exit_backoff_exit_yaw is None:
                return self._fail_exit_backoff(context, "缺少出口方向")
            heading_error = _angle_wrap(self._exit_backoff_exit_yaw - pose[2])
            if abs(heading_error) <= self._EXIT_BACKOFF_ALIGN_TOLERANCE:
                context.controller.stop(reason="exit_backoff_aligned")
                if self.phase == "EXIT_BACKOFF_REALIGN":
                    context.controller.clear_path(reason="exit_backoff_complete")
                    self._reset_cabin_transit()
                    self.phase = "EXIT_CABIN"
                    if rospy is not None:
                        rospy.loginfo(f"[{self.name}] 轿厢内倒车与出口重对准完成，重试离厢。")
                    return RUNNING
                if not self._prepare_exit_backoff_segment(context, pose, cabin_center):
                    return self._fail_exit_backoff(context, "对准后倒车路径不再安全")
                reverse_speed = abs(float(getattr(context.controller, "reverse_speed", 0.0)))
                if reverse_speed <= 1e-4:
                    return self._fail_exit_backoff(context, "倒车速度配置无效")
                self._exit_backoff_deadline = (
                    now
                    + self._EXIT_BACKOFF_DISTANCE / reverse_speed
                    + self._EXIT_BACKOFF_TIMEOUT_MARGIN
                )
                self.phase = "EXIT_BACKOFF_REVERSE"
                return RUNNING
            if now - self._exit_backoff_phase_start > self._EXIT_BACKOFF_ALIGN_TIMEOUT:
                return self._fail_exit_backoff(context, "出口方向对准超时")
            angular_limit = min(0.45, abs(float(getattr(context.controller, "max_angular", 0.45))))
            context.controller.rotate_in_place(math.copysign(angular_limit, heading_error))
            return RUNNING

        if self.phase == "EXIT_BACKOFF_REVERSE":
            if (
                self._exit_backoff_start_xy is None
                or self._exit_backoff_target_xy is None
                or self._exit_backoff_direction is None
                or self._exit_backoff_exit_yaw is None
            ):
                return self._fail_exit_backoff(context, "缺少倒车状态")
            dx = pose[0] - self._exit_backoff_start_xy[0]
            dy = pose[1] - self._exit_backoff_start_xy[1]
            progress = dx * self._exit_backoff_direction[0] + dy * self._exit_backoff_direction[1]
            target_distance = math.hypot(
                self._exit_backoff_target_xy[0] - pose[0],
                self._exit_backoff_target_xy[1] - pose[1],
            )
            if progress >= self._EXIT_BACKOFF_DISTANCE - 0.03 or target_distance <= 0.05:
                context.controller.stop(reason="exit_backoff_distance_reached")
                self.phase = "EXIT_BACKOFF_REALIGN"
                self._exit_backoff_phase_start = now
                return RUNNING
            if now > self._exit_backoff_deadline:
                return self._fail_exit_backoff(context, "倒车无有效位移或执行超时")
            if abs(_angle_wrap(self._exit_backoff_exit_yaw - pose[2])) > 0.35:
                return self._fail_exit_backoff(context, "倒车期间朝向偏差过大")
            if not context.controller.reverse_straight(reason="exit_cabin_backoff"):
                return self._fail_exit_backoff(context, "倒车命令未生效")
            return RUNNING

        return self._fail_exit_backoff(context, f"未知倒车阶段: {self.phase}")

    def _fail_exit_backoff(self, context: NavigationTaskContext, reason: str) -> str:
        context.controller.stop(reason="exit_backoff_failed")
        if rospy is not None:
            rospy.logerr(f"[{self.name}] 离开轿厢倒车恢复失败：{reason}。")
        return FAILED

    def _navigate_to_with_status(
        self,
        context: NavigationTaskContext,
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
                context,
                "elevator_navigation_timeout",
                self._DEFAULT_NAVIGATION_TIMEOUT,
            )
        )
        if now - self._navigation_started_at > navigation_timeout:
            return self._fail_navigation(
                context,
                "elevator_navigation_timeout",
                f"导航阶段 {self.phase} 超过 {navigation_timeout:.1f}s",
            )

        effective_goal_tolerance = float(context.controller.goal_tolerance)
        if goal_tolerance is not None:
            effective_goal_tolerance = min(
                effective_goal_tolerance,
                max(0.0, float(goal_tolerance)),
            )
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= effective_goal_tolerance:
            context.controller.clear_path()
            self._reset_navigation_watchdog(clear_failure=True)
            return DONE

        if not context.controller.has_path():
            if now < self._navigation_next_plan_time:
                context.controller.stop(reason="elevator_navigation_plan_wait")
                return RUNNING
            grid = context.mapper.floor(context.current_floor)
            multiplier = 1 << min(self._navigation_budget_count, 2)
            max_iterations = DEFAULT_MAX_ITERATIONS * multiplier
            result = context.plan_path(
                grid,
                (pose[0], pose[1]),
                target_xy,
                max_iterations=max_iterations,
            )
            if result.reached:
                path_tolerance = (
                    effective_goal_tolerance if goal_tolerance is not None else None
                )
                if self.cabin_transit_stage == "POSITION" and path_tolerance is not None:
                    # 只调整站位路径的终点容差，不放宽任务站位窗或修改其他导航阶段。
                    endpoint_offset = math.hypot(
                        result.path[-1][0] - target_xy[0],
                        result.path[-1][1] - target_xy[1],
                    )
                    path_tolerance -= endpoint_offset
                    if path_tolerance <= 0.0:
                        return self._fail_navigation(
                            context,
                            "elevator_position_goal_outside_acceptance",
                            "规划替代终点超出电梯站位窗",
                        )
                context.controller.set_path(
                    result.path,
                    max_linear=max_linear,
                    goal_tolerance=path_tolerance,
                    reason="elevator_navigate_planned_path",
                )
                self._navigation_unreachable_count = 0
                self._navigation_budget_count = 0
                self._navigation_next_plan_time = 0.0
                self._log_navigation_plan(
                    context,
                    pose,
                    target_xy,
                    result.status,
                    max_iterations,
                    0,
                    reason_code=result.reason_code,
                    blocker=result.blocker,
                )
            elif result.budget_exhausted:
                self._navigation_budget_count += 1
                self._log_navigation_plan(
                    context,
                    pose,
                    target_xy,
                    result.status,
                    max_iterations,
                    self._navigation_budget_count,
                    reason_code=result.reason_code,
                    blocker=result.blocker,
                )
                context.controller.stop(reason="elevator_navigation_budget_exhausted")
                if self._navigation_budget_count >= int(
                    getattr(
                        context,
                        "elevator_planning_retry_limit",
                        self._DEFAULT_PLAN_FAILURE_LIMIT,
                    )
                ):
                    return self._fail_navigation(
                        context,
                        "elevator_navigation_budget_exhausted",
                        "A* 逐级扩大预算后仍未完成",
                    )
                self._navigation_next_plan_time = now + self._PLAN_RETRY_INTERVAL
                return RUNNING
            else:
                self._navigation_unreachable_count += 1
                self._log_navigation_plan(
                    context,
                    pose,
                    target_xy,
                    result.status,
                    max_iterations,
                    self._navigation_unreachable_count,
                    reason_code=result.reason_code,
                    blocker=result.blocker,
                )
                context.controller.stop(reason="elevator_navigation_unreachable")
                if self._navigation_unreachable_count >= int(
                    getattr(
                        context,
                        "elevator_planning_retry_limit",
                        self._DEFAULT_PLAN_FAILURE_LIMIT,
                    )
                ):
                    return self._fail_navigation(
                        context,
                        "elevator_navigation_unreachable",
                        "A* 连续确认目标不可达",
                    )
                self._navigation_next_plan_time = now + self._PLAN_RETRY_INTERVAL
                return RUNNING

        # 限速/终点容差随路径原子提交（固定频率控制读取），这里只轮询状态。
        status = context.controller.poll_status()
        if status == STUCK:
            self._navigation_stuck_count += 1
            stuck_limit = context.elevator_stuck_retry_limit
            if self._navigation_stuck_count > stuck_limit:
                return self._fail_navigation(
                    context,
                    "elevator_navigation_stuck_exhausted",
                    f"局部控制卡住恢复超过 {stuck_limit} 次",
                )
            context.recover_with_rotation_clearance("elevator_navigation_stuck")
            return RUNNING
        if status == DONE:
            self._reset_navigation_watchdog(clear_failure=True)
        return status

    def _navigate_to(self, context: NavigationTaskContext, pose: tuple[float, float, float], target_xy: tuple[float, float]) -> bool:
        status = self._navigate_to_with_status(context, pose, target_xy)
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
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
    ) -> bool:
        if not self._start_retreat_attempted:
            self._start_retreat_attempted = True
            target = context.elevator_start_retreat_target(pose, self.current_floor)
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
                    float(getattr(context.controller, "reverse_speed", -0.15))
                )
                self._start_retreat_deadline = get_time() + max(
                    4.0,
                    distance / max(0.05, reverse_speed) + 2.0,
                )
                context.controller.clear_path(reason="elevator_start_retreat_start")
                self._reset_navigation_watchdog(clear_failure=True)
                self.phase = "ELEVATOR_START_RETREAT"
                self._log_start_retreat(
                    context,
                    "elevator_start_retreat_started",
                    pose,
                    self._start_retreat_target_xy,
                )
                return True
        return self._begin_trace_approach(context)

    def _tick_start_retreat(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
    ) -> str:
        target = self._start_retreat_target_xy
        if target is None:
            return self._fail_start_retreat(context, pose, "target_missing")
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        if distance <= 0.25:
            context.controller.stop(reason="elevator_start_retreat_completed")
            self._log_start_retreat(
                context,
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
            return self._fail_start_retreat(context, pose, "retreat_timeout")
        target_direction = math.atan2(target[1] - pose[1], target[0] - pose[0])
        backward_error = _angle_wrap(target_direction - pose[2] - math.pi)
        if abs(backward_error) > 0.45:
            return self._fail_start_retreat(
                context,
                pose,
                "reverse_heading_diverged",
            )
        if not context.controller.reverse_straight(
            reason="elevator_start_clearance_retreat"
        ):
            return self._fail_start_retreat(
                context,
                pose,
                "reverse_command_rejected",
            )
        return RUNNING

    def _fail_start_retreat(
        self,
        context: NavigationTaskContext,
        pose: tuple[float, float, float],
        detail: str,
    ) -> str:
        target = self._start_retreat_target_xy
        self._log_start_retreat(
            context,
            "elevator_start_retreat_failed",
            pose,
            target,
            detail=detail,
        )
        self._start_retreat_target_xy = None
        self._start_retreat_deadline = 0.0
        return self._fail_navigation(
            context,
            "elevator_start_retreat_failed",
            detail,
        )

    def _log_start_retreat(
        self,
        context: NavigationTaskContext,
        event: str,
        pose: tuple[float, float, float],
        target: tuple[float, float] | None,
        *,
        detail: str | None = None,
    ) -> None:
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
        context.log_structured(payload)

    def _begin_trace_approach(self, context: NavigationTaskContext) -> bool:
        if self._approach_trace_attempted:
            return False
        self._approach_trace_attempted = True
        candidates = list(
            context.elevator_approach_trace_candidates(
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
        context.controller.clear_path(reason="elevator_trace_approach_start")
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
        context: NavigationTaskContext,
        reason_code: str,
        detail: str,
    ) -> str:
        self.failure_reason = str(reason_code)
        self._current_crossing_geometry = None
        self._target_crossing_geometry = None
        context.controller.clear_path(reason=reason_code)
        context.controller.stop(reason=reason_code)
        if rospy is not None:
            rospy.logerr("[%s] %s：%s", self.name, reason_code, detail)
        self._log_navigation_plan(
            context,
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
        context: NavigationTaskContext,
        pose: tuple[float, float, float] | None,
        target_xy: tuple[float, float] | None,
        result: str,
        max_iterations: int,
        attempt: int,
        *,
        reason_code: str | None = None,
        blocker: dict | None = None,
    ) -> None:
        context.log_structured(
            {
                "event": "elevator_navigation_plan",
                "task": self.name,
                "phase": self.phase,
                "result": str(result),
                "reason_code": reason_code,
                "blocker": blocker,
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

    def _run_service(
        self,
        context: NavigationTaskContext,
        task_factory,
        *,
        hold_position: bool = True,
    ) -> str:
        if self._service_task is None:
            self._service_task = task_factory()
            self._service_started_at = get_time()
            self._service_started_wall = time.monotonic()
            context.log_structured({
                "event": "elevator_service_started", "phase": self.phase,
                "attempt": self._retry_count + 1, "hold_position": hold_position,
            })
            return RUNNING

        if not self._service_task.done():
            if (
                get_time() - self._service_started_at > self._SERVICE_TIMEOUT
                or time.monotonic() - self._service_started_wall > self._SERVICE_WALL_TIMEOUT
            ):
                # 超时不等于服务端取消，不能重发可能仍在移动轿厢的请求。
                self.failure_reason = "elevator_service_timeout"
                context.controller.clear_path(reason=self.failure_reason)
                context.record_failure(self.failure_reason)
                context.log_structured({
                    "event": "elevator_service_timeout", "phase": self.phase,
                    "elapsed": get_time() - self._service_started_at,
                    "wall_elapsed": time.monotonic() - self._service_started_wall,
                    "request_cancelled": False,
                })
                return FAILED
            if hold_position:
                context.controller.stop()
            return RUNNING

        ok = self._service_task.ok()
        message = self._service_task.message()
        context.log_structured({
            "event": "elevator_service_finished", "phase": self.phase,
            "ok": bool(ok), "elapsed": get_time() - self._service_started_at,
            "wall_elapsed": time.monotonic() - self._service_started_wall,
        })
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
            self.failure_reason = "elevator_service_failed"
            context.controller.clear_path(reason=self.failure_reason)
            context.record_failure(f"电梯服务失败: {message}")
            return FAILED

        self._service_task = None
        self._retry_count = 0
        return DONE

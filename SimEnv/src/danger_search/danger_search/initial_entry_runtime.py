"""主入口探测与过门净空的有界运行状态。"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

from danger_search.building_control_client import DoorInfo
from danger_search.mission_reporting import MissionReportingRuntime
from danger_search.navigation_tasks import OpenDoorAndPass
from danger_search.navigator import DONE, IDLE, STUCK
from danger_search.time_utils import get_time


PROBE_LATERAL_OFFSETS = (0.0, -1.0, 1.0, -2.0, 2.0)
# 主入口检测不硬编码任何绝对坐标先验；以机器狗出生中轴线作为横向对准基准，纵向深度完全由雷达在线探测决定。


@dataclass(frozen=True)
class InitialEntryConfig:
    max_distance: float
    corridor_half_width: float
    probe_distance: float
    opening_confirmations: int
    opening_max_distance: float
    geometry_tolerance: float
    door_pass_offset: float
    door_commit_distance: float
    door_wait_offset: float = 1.6
    wait_timeout: float = 5.0
    center_tolerance: float = 0.60
    approach_max_linear: float = 0.44

    @classmethod
    def from_section(cls, section) -> "InitialEntryConfig":
        return cls(
            max_distance=float(section["initial_entry_max_distance"]),
            corridor_half_width=float(
                section["initial_entry_corridor_half_width"]
            ),
            probe_distance=float(section["probe_distance"]),
            opening_confirmations=max(
                1,
                int(section["initial_entry_opening_confirmations"]),
            ),
            opening_max_distance=float(
                section["initial_entry_opening_max_distance"]
            ),
            wait_timeout=float(section["initial_entry_wait_timeout"]),
            geometry_tolerance=float(
                section["initial_entry_geometry_tolerance"]
            ),
            door_pass_offset=float(section["door_pass_offset"]),
            door_wait_offset=float(section.get("door_wait_offset", 1.6)),
            door_commit_distance=float(section["door_commit_distance"]),
            center_tolerance=float(
                section.get("initial_entry_center_tolerance", 0.60)
            ),
            approach_max_linear=float(section["door_pass_max_linear"]),
        )


@dataclass(frozen=True)
class InitialEntryTickResult:
    handled: bool
    task: OpenDoorAndPass | None = None
    door_id: str | None = None
    clear_exploration_target: bool = False
    failure: str | None = None


@dataclass(frozen=True)
class InitialEntryClearanceResult:
    handled: bool
    failed: bool = False
    target: tuple[float, float] | None = None


@dataclass
class InitialEntryRuntime:
    target: tuple[float, float] | None = None
    origin_pose: tuple[float, float, float] | None = None
    heading: float | None = None
    completed: bool = False
    failed: bool = False
    retry_count: int = 0
    service_task: object | None = None
    trace: list[tuple[float, float]] = field(default_factory=list)
    trace_floor: int | None = None
    trace_map_epoch: int | None = None
    trace_localization_epoch: int | None = None
    trace_scope_valid: bool = True
    clearance_target: tuple[float, float] | None = None
    clearance_door_id: str | None = None
    clearance_path_started: bool = False
    free_corridor_wait_started_at: float | None = None
    free_corridor_wait_started_wall: float | None = None
    observation_retry_count: int = 0
    service_started_at: float | None = None
    service_started_wall: float | None = None
    alignment_started_at: float | None = None
    alignment_started_wall: float | None = None
    approach_started_at: float | None = None
    approach_started_wall: float | None = None
    observation_reached_at: float | None = None
    observation_not_before: float | None = None
    observation_wait_started_at: float | None = None
    observation_wait_started_wall: float | None = None
    opening_seen: bool = False

    def schedule_clearance(
        self,
        *,
        door_xy: tuple[float, float] | None,
        pass_vector: tuple[float, float] | None,
        pass_offset: float,
        minimum_distance: float,
    ) -> None:
        if door_xy is None or pass_vector is None:
            return
        distance = max(float(pass_offset), float(minimum_distance))
        self.clearance_target = (
            float(door_xy[0]) + float(pass_vector[0]) * distance,
            float(door_xy[1]) + float(pass_vector[1]) * distance,
        )
        self.clearance_door_id = "main_entrance"
        self.clearance_path_started = False

    def tick_clearance(
        self,
        pose: tuple[float, float, float],
        *,
        controller,
        max_linear: float,
        grid=None,
        segment_is_traversable=None,
    ) -> InitialEntryClearanceResult:
        target = self.clearance_target
        if target is None:
            return InitialEntryClearanceResult(False)
        tolerance = min(
            0.12,
            float(getattr(controller, "goal_tolerance", 0.12)),
        )
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        if distance <= tolerance:
            controller.clear_path(reason="main_entrance_clearance_completed")
            MissionReportingRuntime.loginfo(
                "主入口轴向离门完成：target=(%.2f, %.2f) "
                "pose=(%.2f, %.2f, %.2f)",
                target[0],
                target[1],
                pose[0],
                pose[1],
                pose[2],
            )
            self._clear_clearance()
            return InitialEntryClearanceResult(True, target=target)
        if not controller.has_path():
            if (
                grid is not None
                and callable(segment_is_traversable)
                and not segment_is_traversable(
                    grid, (pose[0], pose[1]), target
                )
            ):
                controller.clear_path(reason="main_entrance_clearance_blocked")
                self._clear_clearance()
                return InitialEntryClearanceResult(True, failed=True, target=target)
            controller.set_path(
                [(pose[0], pose[1]), target],
                max_linear=max_linear,
                goal_tolerance=tolerance,
                reason="initial_entry_clearance_path",
            )
            if not self.clearance_path_started:
                MissionReportingRuntime.loginfo(
                    "主入口过门后先沿门轴离开门框："
                    "target=(%.2f, %.2f) distance=%.2f",
                    target[0],
                    target[1],
                    distance,
                )
            self.clearance_path_started = True
        status = controller.poll_status()
        if status in (STUCK, IDLE):
            controller.clear_path(reason="main_entrance_clearance_failed")
            self._clear_clearance()
            return InitialEntryClearanceResult(True, failed=True, target=target)
        return InitialEntryClearanceResult(True, target=target)

    def _clear_clearance(self) -> None:
        self.clearance_target = None
        self.clearance_door_id = None
        self.clearance_path_started = False

    def tick(
        self,
        *,
        pose: tuple[float, float, float],
        grid,
        entrance,
        current_floor: int,
        controller,
        building_client,
        local_detector,
        door_tracker,
        cloud_stamp: float | None,
        cloud_points,
        floor_height: float,
        config: InitialEntryConfig,
        segment_is_traversable=None,
    ) -> InitialEntryTickResult:
        """推进完整主入口进入策略，不读写顶层任务状态。"""
        if self.completed or self.failed:
            return InitialEntryTickResult(False)
        if entrance is None or int(getattr(entrance, "floor_index", -1)) != 0:
            self.completed = True
            return InitialEntryTickResult(False)
        if int(current_floor) != 0:
            self.completed = True
            return InitialEntryTickResult(False)
        if self.origin_pose is None:
            self.origin_pose = tuple(float(value) for value in pose)
            self.heading = float(pose[2]) if len(pose) > 2 else (math.pi / 2.0)
            self.trace = [(float(pose[0]), float(pose[1]))]
            approach_dist = max(0.5, 3.2 - float(config.door_wait_offset))
            self.target = (
                float(pose[0]) + approach_dist * math.cos(self.heading),
                float(pose[1]) + approach_dist * math.sin(self.heading),
            )
            if math.hypot(self.target[0] - pose[0], self.target[1] - pose[1]) > config.max_distance:
                return self._fail(controller, "主入口固定观察点超出首次接近距离上限")

        if self.service_task is None:
            self.service_started_at = get_time()
            self.service_started_wall = time.monotonic()
            controller.poll_status()
            controller.clear_path(reason="initial_entry_opening")
            self.service_task = building_client.open_door("main_entrance")
            controller.stop(reason="initial_entry_opening")
            return InitialEntryTickResult(True)
        if not self.service_task.done():
            if (
                get_time() - self.service_started_at >= 30.0
                or time.monotonic() - self.service_started_wall >= 180.0
            ):
                return self._fail(controller, "主入口开门服务超时（未取消请求，不重发）")
            controller.stop(reason="initial_entry_opening")
            return InitialEntryTickResult(True)
        if not self.service_task.ok():
            return self._fail(controller, "主入口服务失败: " + self.service_task.message())

        target = self.target
        tolerance = min(0.12, float(controller.goal_tolerance))
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        if self.observation_reached_at is None:
            if self.approach_started_at is None:
                self.approach_started_at = get_time()
                self.approach_started_wall = time.monotonic()
            if (
                get_time() - self.approach_started_at >= 30.0
                or time.monotonic() - self.approach_started_wall >= 180.0
            ):
                return self._fail(controller, "主入口固定观察点接近超时")
            if distance <= tolerance:
                controller.poll_status()
                controller.clear_path(reason="initial_entry_observation_reached")
                self.observation_reached_at = get_time()
                self.observation_not_before = self.observation_reached_at + 0.5
                self.free_corridor_wait_started_at = None
                self.free_corridor_wait_started_wall = None
                self.observation_retry_count = 0
                door_tracker.confirmations = 0
                door_tracker.observation = None
                MissionReportingRuntime.loginfo(
                    "主入口固定观察点已停车：target=(%.2f, %.2f) pose=(%.2f, %.2f, %.2f)，重新累计停车后点云",
                    target[0], target[1], pose[0], pose[1], pose[2],
                )
                return InitialEntryTickResult(True)
        elif distance > 0.30:
            return self._fail(controller, "主入口停车观察期间位置漂移超限")
        if pose[1] > target[1] + 0.30:
            return self._fail(controller, "未确认门洞已越过主入口观察边界")

        if (
            self.observation_reached_at is None
            and self.free_corridor_wait_started_at is not None
            and (
                get_time() - self.free_corridor_wait_started_at >= config.wait_timeout
                or time.monotonic() - self.free_corridor_wait_started_wall >= 60.0
            )
        ):
            if self.observation_retry_count >= 1:
                return self._fail(controller, "主入口门洞/观察点路线确认重试耗尽")
            self.observation_retry_count += 1
            self.free_corridor_wait_started_at = get_time()
            self.free_corridor_wait_started_wall = time.monotonic()
            door_tracker.confirmations = 0
            door_tracker.observation = None
            # 保留 last_stamp，重试必须等待新帧，不能重复使用旧候选。
            MissionReportingRuntime.logwarn(
                "主入口观察点路线确认超时，重试1/1，不重发开门服务"
            )

        # 必须先检查已启动的观察预算，再处理转向，避免姿态扰动跳过截止时间。
        if self.observation_wait_started_at is not None and (
            get_time() - self.observation_wait_started_at >= config.wait_timeout
            or time.monotonic() - self.observation_wait_started_wall >= 60.0
        ):
            if self.observation_retry_count >= 1:
                return self._fail(controller, "主入口门洞确认重试耗尽")
            self.observation_retry_count += 1
            self.observation_wait_started_at = get_time()
            self.observation_wait_started_wall = time.monotonic()
            self.observation_not_before = get_time() + 0.5
            door_tracker.confirmations = 0
            door_tracker.observation = None
            MissionReportingRuntime.logwarn(
                "主入口门洞观察超时，重新累计新帧证据（重试1/1），不重发开门服务"
            )

        status = controller.poll_status()
        if status in (STUCK, DONE):
            controller.clear_path(reason="initial_entry_approach_interrupted")
        heading = (
            math.atan2(-pose[1], -pose[0])
            if self.observation_reached_at is not None
            else math.atan2(target[1] - pose[1], target[0] - pose[0])
        )
        was_aligning = self.alignment_started_at is not None
        if not controller.has_path() and self.align_heading(
            controller,
            pose,
            heading,
        ):
            if self.alignment_started_at is None:
                self.alignment_started_at = get_time()
                self.alignment_started_wall = time.monotonic()
            elif (
                get_time() - self.alignment_started_at >= 12.0
                or time.monotonic() - self.alignment_started_wall >= 180.0
            ):
                return self._fail(controller, "主入口接近朝向对准超时")
            return InitialEntryTickResult(True)
        if was_aligning and self.observation_reached_at is not None:
            controller.stop(reason="initial_entry_alignment_completed")
            # 姿态扰动只重置证据起点，不续期观察预算或增加重试机会。
            self.observation_not_before = get_time() + 0.5
            if self.observation_wait_started_at is None:
                self.observation_wait_started_at = get_time()
                self.observation_wait_started_wall = time.monotonic()
            door_tracker.confirmations = 0
            door_tracker.observation = None
            self.alignment_started_at = None
            self.alignment_started_wall = None
            MissionReportingRuntime.loginfo(
                "主入口观察朝向已稳定，重新等待0.5秒后累计点云"
            )
            return InitialEntryTickResult(True)
        self.alignment_started_at = None
        self.alignment_started_wall = None

        if self.observation_reached_at is not None and self.observation_wait_started_at is None:
            self.observation_wait_started_at = get_time()
            self.observation_wait_started_wall = time.monotonic()

        door_result = self.try_start_door_task(
            pose=pose,
            entrance=entrance,
            current_floor=current_floor,
            controller=controller,
            building_client=building_client,
            local_detector=local_detector,
            door_tracker=door_tracker,
            cloud_stamp=cloud_stamp,
            cloud_points=cloud_points,
            floor_height=floor_height,
            config=config,
        )
        if door_result.handled:
            return door_result

        if self.observation_reached_at is not None:
            return self._wait_for_observation(
                controller, "initial_entry_waiting_for_opening",
            )

        if not controller.has_path():
            if self.retry_count >= 2:
                return self._fail(controller, "主入口固定观察点接近重试耗尽")
            if not callable(segment_is_traversable):
                return self._fail(controller, "主入口观察点缺少路径安全检查")
            lateral = (-math.sin(heading), math.cos(heading))
            # 继续检查左右足迹带；固定坐标不代表路线已知安全，不绕行未知门区。
            if not all(
                segment_is_traversable(
                    grid,
                    (pose[0] + offset * lateral[0], pose[1] + offset * lateral[1]),
                    (target[0] + offset * lateral[0], target[1] + offset * lateral[1]),
                )
                for offset in (-config.corridor_half_width, 0.0, config.corridor_half_width)
            ):
                return self._wait_for_observation(controller, "initial_entry_planning_constraint")
            self.free_corridor_wait_started_at = None
            self.free_corridor_wait_started_wall = None
            self.retry_count += 1
            controller.set_path(
                [(pose[0], pose[1]), target],
                max_linear=config.approach_max_linear,
                goal_tolerance=tolerance,
                reason="initial_entry_observation_path",
            )
            MissionReportingRuntime.loginfo(
                "接近主入口固定观察点：target=(%.2f, %.2f) distance=%.2f max_linear=%.2f attempt=%d/2",
                target[0], target[1], distance, config.approach_max_linear, self.retry_count,
            )
        return InitialEntryTickResult(True)

    def _wait_for_observation(self, controller, reason: str) -> InitialEntryTickResult:
        if self.observation_reached_at is None and self.free_corridor_wait_started_at is None:
            self.free_corridor_wait_started_at = get_time()
            self.free_corridor_wait_started_wall = time.monotonic()
        controller.poll_status()
        controller.stop(reason=reason)
        return InitialEntryTickResult(True)

    def _fail(self, controller, reason: str) -> InitialEntryTickResult:
        self.failed = True
        controller.poll_status()
        controller.clear_path(reason="initial_entry_failed")
        return InitialEntryTickResult(True, failure=reason)

    @staticmethod
    def align_heading(
        controller,
        pose: tuple[float, float, float],
        heading: float,
    ) -> bool:
        error = math.atan2(
            math.sin(float(heading) - float(pose[2])),
            math.cos(float(heading) - float(pose[2])),
        )
        if abs(error) <= 0.18:
            return False
        rotate = getattr(controller, "rotate_in_place", None)
        if not callable(rotate):
            controller.stop(reason="initial_entry_heading_unavailable")
            return True
        maximum = min(
            0.55,
            max(0.0, float(getattr(controller, "max_angular", 0.55))),
        )
        rotate(max(-maximum, min(maximum, 0.8 * error)))
        return True

    def try_start_door_task(
        self,
        *,
        pose: tuple[float, float, float],
        entrance,
        current_floor: int,
        controller,
        building_client,
        local_detector,
        door_tracker,
        cloud_stamp: float | None,
        cloud_points,
        floor_height: float,
        config: InitialEntryConfig,
    ) -> InitialEntryTickResult:
        """连续点云确认入口门洞后返回待安装的门任务。"""
        if (
            cloud_stamp is None
            or not math.isfinite(cloud_stamp)
            or (door_tracker.last_stamp is not None and cloud_stamp <= door_tracker.last_stamp)
            or (
                self.observation_not_before is not None
                and cloud_stamp <= self.observation_not_before
            )
        ):
            return InitialEntryTickResult(False)
        expected_lateral_x = (
            float(self.origin_pose[0])
            if self.origin_pose is not None
            else float(pose[0])
        )
        observation = local_detector.detect_opening_ahead(
            robot_pose=pose,
            floor_index=int(current_floor),
            cloud_points=cloud_points,
            floor_height=float(floor_height),
            maximum_distance=config.opening_max_distance,
            expected_lateral_x=expected_lateral_x,
            maximum_lateral_error=config.center_tolerance,
        )
        if not door_tracker.observe(
            stamp=cloud_stamp,
            observation=observation,
            maximum_distance=config.opening_max_distance,
            tolerance=config.geometry_tolerance,
            required_confirmations=config.opening_confirmations,
        ):
            return InitialEntryTickResult(False)
        observation = door_tracker.observation
        if observation is None:  # pragma: no cover - tracker 合同保证
            raise RuntimeError("入口门洞确认成功但缺少观测")
        if self.observation_reached_at is None:
            if not self.opening_seen:
                self.opening_seen = True
                MissionReportingRuntime.loginfo(
                    "主入口远处候选已确认：center=(%.2f, %.2f) distance=%.2f，仍须先到固定观察点停车",
                    observation.world_xy[0], observation.world_xy[1], observation.distance,
                )
            return InitialEntryTickResult(False)

        door_id = str(entrance.door_id)
        registered_x = float(observation.world_xy[0])
        registered_y = float(observation.world_xy[1])
        building_client.doors[door_id] = DoorInfo(
            door_id=door_id,
            kind=str(entrance.kind),
            floor_index=int(entrance.floor_index),
            x=registered_x,
            y=registered_y,
            yaw=float(observation.yaw),
            width=float(observation.width),
        )
        controller.poll_status()
        controller.clear_path(reason="initial_entry_opening_confirmed")
        task = OpenDoorAndPass(
            door_id,
            pass_offset=config.door_pass_offset,
            commit_distance=config.door_commit_distance,
            wait_offset=config.door_wait_offset,
        )
        task.phase = "CALL_OPEN"
        MissionReportingRuntime.loginfo(
            "点云连续确认主入口门洞：door_id=%s observed_center=(%.2f, %.2f) "
            "approach_center=(%.2f, %.2f) yaw=%.2f width=%.2f distance=%.2f "
            "evidence=%d confirmations=%d",
            door_id,
            observation.world_xy[0],
            observation.world_xy[1],
            registered_x,
            registered_y,
            observation.yaw,
            observation.width,
            observation.distance,
            observation.evidence_count,
            door_tracker.confirmations,
        )
        return InitialEntryTickResult(
            True,
            task=task,
            door_id=door_id,
            clear_exploration_target=True,
        )


def probe_target_for_attempt(
    base_pose: tuple[float, float, float],
    probe_distance: float,
    attempt: int,
) -> tuple[float, float]:
    """按固定走廊序列生成短距离探测目标。"""
    distance = max(float(probe_distance), 0.05)
    attempt_index = max(int(attempt), 0)
    lane_index = attempt_index % len(PROBE_LATERAL_OFFSETS)
    forward_step = attempt_index // len(PROBE_LATERAL_OFFSETS) + 1
    forward = (math.cos(base_pose[2]), math.sin(base_pose[2]))
    lateral = (
        math.cos(base_pose[2] + math.pi / 2.0),
        math.sin(base_pose[2] + math.pi / 2.0),
    )
    lateral_offset = PROBE_LATERAL_OFFSETS[lane_index] * distance
    forward_distance = forward_step * distance
    return (
        base_pose[0]
        + forward_distance * forward[0]
        + lateral_offset * lateral[0],
        base_pose[1]
        + forward_distance * forward[1]
        + lateral_offset * lateral[1],
    )


def initial_forward_free_target(
    grid,
    pose: tuple[float, float, float],
    *,
    max_distance: float,
    corridor_half_width: float,
    minimum_distance: float,
) -> tuple[float, float] | None:
    """返回当前朝向上、左右足迹带均已确认可通行的最远目标。"""
    limit = max(0.0, float(max_distance))
    required = max(0.0, float(minimum_distance))
    half_width = max(0.0, float(corridor_half_width))
    step = max(float(grid.resolution), 0.05)
    forward = (math.cos(pose[2]), math.sin(pose[2]))
    lateral = (-forward[1], forward[0])
    offsets = (
        (0.0,) if half_width <= 1e-6 else (-half_width, 0.0, half_width)
    )
    farthest = 0.0
    for index in range(1, int(math.floor(limit / step)) + 1):
        distance = index * step
        traversable = True
        for offset in offsets:
            cell = grid.world_to_grid(
                pose[0] + distance * forward[0] + offset * lateral[0],
                pose[1] + distance * forward[1] + offset * lateral[1],
            )
            if cell is None or not grid.is_traversable(*cell):
                traversable = False
                break
        if not traversable:
            break
        farthest = distance
    target_distance = max(0.0, farthest - step)
    if target_distance + 1e-9 < required:
        return None
    return (
        pose[0] + target_distance * forward[0],
        pose[1] + target_distance * forward[1],
    )

"""主入口探测与过门净空的有界运行状态。"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

from danger_search.building_control_client import DoorInfo
from danger_search.mission_reporting import MissionReportingRuntime
from danger_search.navigation_tasks import OpenDoorAndPass
from danger_search.navigator import DONE, IDLE, STUCK


PROBE_LATERAL_OFFSETS = (0.0, -1.0, 1.0, -2.0, 2.0)


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
            geometry_tolerance=float(
                section["initial_entry_geometry_tolerance"]
            ),
            door_pass_offset=float(section["door_pass_offset"]),
            door_commit_distance=float(section["door_commit_distance"]),
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
    clearance_target: tuple[float, float] | None = None
    clearance_door_id: str | None = None
    clearance_path_started: bool = False

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
            controller.set_path([(pose[0], pose[1]), target])
            if not self.clearance_path_started:
                MissionReportingRuntime.loginfo(
                    "主入口过门后先沿门轴离开门框："
                    "target=(%.2f, %.2f) distance=%.2f",
                    target[0],
                    target[1],
                    distance,
                )
            self.clearance_path_started = True
        status = controller.tick(
            pose,
            max_linear=float(max_linear),
            goal_tolerance=tolerance,
        )
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
            self.heading = float(pose[2])
            self.trace = [(float(pose[0]), float(pose[1]))]

        heading = float(self.heading)
        origin = self.origin_pose
        forward = (math.cos(heading), math.sin(heading))
        forward_progress = max(
            0.0,
            (pose[0] - origin[0]) * forward[0]
            + (pose[1] - origin[1]) * forward[1],
        )
        remaining_distance = max(0.0, config.max_distance - forward_progress)
        if remaining_distance <= controller.goal_tolerance:
            controller.clear_path(reason="initial_entry_completed")
            self.completed = True
            MissionReportingRuntime.loginfo(
                "已沿传感器确认走廊深入主入口 %.2f m，开始常规 frontier 探索",
                forward_progress,
            )
            return InitialEntryTickResult(True)

        if self.service_task is None:
            self.service_task = building_client.open_door("main_entrance")
            controller.stop(reason="initial_entry_opening")
            return InitialEntryTickResult(True)
        if not self.service_task.done():
            controller.stop(reason="initial_entry_opening")
            return InitialEntryTickResult(True)
        if not self.service_task.ok():
            self.failed = True
            return InitialEntryTickResult(
                False,
                failure="主入口服务失败: " + self.service_task.message(),
            )

        if self.target is None and self.align_heading(
            controller,
            pose,
            heading,
        ):
            return InitialEntryTickResult(True)

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

        if self.target is None:
            target = initial_forward_free_target(
                grid,
                (pose[0], pose[1], heading),
                max_distance=remaining_distance,
                corridor_half_width=config.corridor_half_width,
                minimum_distance=max(
                    2.0 * float(controller.goal_tolerance),
                    config.probe_distance,
                ),
            )
            if target is None:
                controller.stop(reason="initial_entry_waiting_for_free_corridor")
                return InitialEntryTickResult(True)
            self.target = target
            controller.set_path([(pose[0], pose[1]), target])
            MissionReportingRuntime.loginfo(
                "按传感器确认的自由走廊进入主入口："
                "target=(%.2f, %.2f) distance=%.2f",
                target[0],
                target[1],
                math.hypot(target[0] - pose[0], target[1] - pose[1]),
            )

        target = self.target
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        if distance <= controller.goal_tolerance:
            controller.clear_path(reason="initial_entry_completed")
            self.target = None
            self.retry_count = 0
            return InitialEntryTickResult(True)
        if not controller.has_path():
            if self.retry_count >= 1:
                self.failed = True
                return InitialEntryTickResult(
                    False,
                    failure="主入口前向进入失败",
                )
            self.retry_count += 1
            controller.set_path([(pose[0], pose[1]), target])
        status = controller.tick(pose)
        if status == STUCK:
            controller.clear_path(reason="initial_entry_stuck")
        elif status == DONE:
            self.target = None
            self.retry_count = 0
        return InitialEntryTickResult(True)

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
        if cloud_stamp is None or cloud_stamp == door_tracker.last_stamp:
            return InitialEntryTickResult(False)
        observation = local_detector.detect_opening_ahead(
            robot_pose=pose,
            floor_index=int(current_floor),
            cloud_points=cloud_points,
            floor_height=float(floor_height),
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

        door_id = str(entrance.door_id)
        registered_x = float(observation.world_xy[0])
        registered_y = float(observation.world_xy[1])
        if self.origin_pose is not None and self.heading is not None:
            forward_x = math.cos(float(self.heading))
            forward_y = math.sin(float(self.heading))
            progress = (
                (registered_x - float(self.origin_pose[0])) * forward_x
                + (registered_y - float(self.origin_pose[1])) * forward_y
            )
            registered_x = float(self.origin_pose[0]) + progress * forward_x
            registered_y = float(self.origin_pose[1]) + progress * forward_y
        building_client.doors[door_id] = DoorInfo(
            door_id=door_id,
            kind=str(entrance.kind),
            floor_index=int(entrance.floor_index),
            x=registered_x,
            y=registered_y,
            yaw=float(observation.yaw),
            width=float(observation.width),
        )
        controller.clear_path(reason="initial_entry_opening_confirmed")
        task = OpenDoorAndPass(
            door_id,
            pass_offset=config.door_pass_offset,
            commit_distance=config.door_commit_distance,
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

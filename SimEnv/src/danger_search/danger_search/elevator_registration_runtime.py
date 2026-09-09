"""电梯候选连续确认与运行期几何注册。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from danger_search.building_control_client import DoorInfo, ElevatorInfo
from danger_search.navigation_tasks import FAILED
from danger_search.navigator import DONE, IDLE, RUNNING


@dataclass(frozen=True)
class ElevatorRegistrationConfig:
    visual_min_width: float
    visual_max_width: float
    visual_max_distance: float
    visual_confirmations: int
    visual_confirmation_max_gap: float
    geometry_tolerance: float
    door_pass_offset: float
    livox_enabled: bool
    refresh_timeout: float
    refresh_max_shift: float
    refresh_max_yaw_change: float


@dataclass(frozen=True)
class ElevatorRegistrationResult:
    registered: bool = False
    active_elevator_id: str | None = None
    info_message: tuple[object, ...] | None = None
    structured_events: tuple[dict[str, object], ...] = ()


@dataclass(frozen=True)
class ElevatorRefreshResult:
    status: str
    info_message: tuple[object, ...] | None = None
    warning_message: tuple[object, ...] | None = None
    error_message: tuple[object, ...] | None = None
    structured_event: dict[str, object] | None = None


@dataclass
class ElevatorRegistrationRuntime:
    opening_last_stamp: float | None = None
    opening_last_scan_stamp: float | None = None
    opening_observation: object | None = None
    opening_confirmations: int = 0
    latest_rgbd_door_stamp: float | None = None
    latest_rgbd_door_observations: list[object] = field(default_factory=list)
    visual_last_stamp: float | None = None
    visual_candidate: object | None = None
    visual_confirmations: int = 0
    visual_candidate_stamp: float | None = None
    visual_association_diagnostics: dict[str, int] = field(default_factory=dict)
    visual_max_confirmations: int = 0
    refresh_context: tuple[str, int, str] | None = None
    refresh_started_at: float | None = None
    refresh_last_stamp: float | None = None
    refresh_candidate: object | None = None
    refresh_confirmations: int = 0
    refresh_rejections: dict[str, int] = field(default_factory=dict)

    def record_visual_association(self, reason: str) -> None:
        key = str(reason)
        self.visual_association_diagnostics[key] = (
            int(self.visual_association_diagnostics.get(key, 0)) + 1
        )

    def reset_refresh(self) -> None:
        self.refresh_context = None
        self.refresh_started_at = None
        self.refresh_last_stamp = None
        self.refresh_candidate = None
        self.refresh_confirmations = 0
        self.refresh_rejections.clear()

    def reset_visual_candidate(self) -> None:
        self.visual_candidate = None
        self.visual_candidate_stamp = None
        self.visual_confirmations = 0
        self.visual_last_stamp = None

    def record_refresh_rejection(self, reason: str) -> None:
        key = str(reason)
        self.refresh_rejections[key] = int(self.refresh_rejections.get(key, 0)) + 1

    def try_register(
        self,
        *,
        pose: tuple[float, float, float],
        current_floor: int,
        max_floor: int,
        current_task_present: bool,
        building_features,
        building_client,
        local_detector,
        cloud_stamp: float | None,
        cloud_points,
        floor_height: float,
        entrance_xy: tuple[float, float] | None,
        config: ElevatorRegistrationConfig,
        landmark_monitor=None,
        elapsed: float = 0.0,
    ) -> ElevatorRegistrationResult:
        if max_floor <= 1 or current_task_present or building_features is None:
            return ElevatorRegistrationResult()
        if len(building_features.elevator_features) != 1:
            return ElevatorRegistrationResult()
        elevator_feature = next(iter(building_features.elevator_features.values()))
        try:
            current_door_id = building_client.elevator_door_id(current_floor)
        except ValueError:
            return ElevatorRegistrationResult()
        existing = building_client.elevators.get(elevator_feature.elevator_id)
        if existing is not None and current_floor in existing.lobby_poses:
            return ElevatorRegistrationResult()

        result = self._try_register_rgbd(
            pose=pose,
            current_floor=current_floor,
            elevator_feature=elevator_feature,
            current_door_id=current_door_id,
            entrance_xy=entrance_xy,
            building_client=building_client,
            config=config,
            landmark_monitor=landmark_monitor,
            elapsed=elapsed,
        )
        if result.registered or not config.livox_enabled:
            return result
        if cloud_stamp is None or cloud_stamp == self.opening_last_stamp:
            return ElevatorRegistrationResult()
        self.opening_last_stamp = cloud_stamp
        if (
            self.opening_last_scan_stamp is not None
            and float(cloud_stamp) - float(self.opening_last_scan_stamp) < 0.5
        ):
            return ElevatorRegistrationResult()
        self.opening_last_scan_stamp = float(cloud_stamp)
        observation = local_detector.detect_elevator_opening_nearby(
            robot_pose=pose,
            floor_index=current_floor,
            cloud_points=cloud_points[::2],
            floor_height=floor_height,
        )
        if observation is None:
            self.opening_observation = None
            self.opening_confirmations = 0
            return ElevatorRegistrationResult()
        if entrance_xy is not None and math.hypot(
            observation.world_xy[0] - entrance_xy[0],
            observation.world_xy[1] - entrance_xy[1],
        ) <= max(1.0, config.door_pass_offset):
            return ElevatorRegistrationResult()
        previous = self.opening_observation
        consistent = previous is not None and (
            math.hypot(
                observation.world_xy[0] - previous.world_xy[0],
                observation.world_xy[1] - previous.world_xy[1],
            )
            <= config.geometry_tolerance
            and abs(observation.width - previous.width)
            <= config.geometry_tolerance
        )
        self.opening_confirmations = self.opening_confirmations + 1 if consistent else 1
        self.opening_observation = observation
        if self.opening_confirmations < 2:
            return ElevatorRegistrationResult()
        outward_yaw = math.atan2(
            math.sin(observation.yaw + math.pi),
            math.cos(observation.yaw + math.pi),
        )
        return self.register_confirmed_geometry(
            elevator_feature=elevator_feature,
            current_door_id=current_door_id,
            current_floor=current_floor,
            world_xy=observation.world_xy,
            outward_yaw=outward_yaw,
            width=observation.width,
            source="livox_open_cabin",
            evidence_count=observation.evidence_count,
            building_client=building_client,
            landmark_monitor=landmark_monitor,
            elapsed=elapsed,
        )

    def _try_register_rgbd(
        self,
        *,
        pose: tuple[float, float, float],
        current_floor: int,
        elevator_feature,
        current_door_id: str,
        entrance_xy: tuple[float, float] | None,
        building_client,
        config: ElevatorRegistrationConfig,
        landmark_monitor,
        elapsed: float,
    ) -> ElevatorRegistrationResult:
        stamp = self.latest_rgbd_door_stamp
        if stamp is None or stamp == self.visual_last_stamp:
            return ElevatorRegistrationResult()
        self.visual_last_stamp = stamp
        candidates = []
        for observation in self.latest_rgbd_door_observations:
            if not config.visual_min_width <= observation.width <= config.visual_max_width:
                self.record_visual_association("rejected_width")
                continue
            if observation.distance > config.visual_max_distance:
                self.record_visual_association("rejected_distance")
                continue
            if entrance_xy is not None and math.hypot(
                observation.world_xy[0] - entrance_xy[0],
                observation.world_xy[1] - entrance_xy[1],
            ) <= max(1.0, config.door_pass_offset):
                self.record_visual_association("rejected_entrance_radius")
                continue
            candidates.append(observation)
        if not candidates:
            self.record_visual_association("no_eligible_candidate")
            if (
                self.visual_candidate is not None
                and self.visual_candidate_stamp is not None
                and 0.0 <= float(stamp) - float(self.visual_candidate_stamp)
                <= config.visual_confirmation_max_gap
            ):
                self.record_visual_association("intermittent_gap_held")
                return ElevatorRegistrationResult()
            self.visual_candidate = None
            self.visual_candidate_stamp = None
            self.visual_confirmations = 0
            return ElevatorRegistrationResult()
        self.record_visual_association("eligible_candidate")
        candidate = min(candidates, key=lambda item: item.distance)
        previous = self.visual_candidate
        previous_stamp = self.visual_candidate_stamp
        consistent = previous is not None and (
            previous_stamp is not None
            and 0.0 <= float(stamp) - float(previous_stamp)
            <= config.visual_confirmation_max_gap
            and math.hypot(
                candidate.world_xy[0] - previous.world_xy[0],
                candidate.world_xy[1] - previous.world_xy[1],
            )
            <= config.geometry_tolerance
            and abs(candidate.width - previous.width) <= config.geometry_tolerance
            and abs(
                math.atan2(
                    math.sin(candidate.outward_yaw - previous.outward_yaw),
                    math.cos(candidate.outward_yaw - previous.outward_yaw),
                )
            )
            <= config.geometry_tolerance
        )
        self.visual_confirmations = self.visual_confirmations + 1 if consistent else 1
        self.visual_candidate = candidate
        self.visual_candidate_stamp = float(stamp)
        if previous is not None and not consistent:
            self.record_visual_association("candidate_sequence_reset")
        self.visual_max_confirmations = max(
            self.visual_max_confirmations, self.visual_confirmations
        )
        if self.visual_confirmations < max(2, config.visual_confirmations):
            return ElevatorRegistrationResult()
        return self.register_confirmed_geometry(
            elevator_feature=elevator_feature,
            current_door_id=current_door_id,
            current_floor=current_floor,
            world_xy=candidate.world_xy,
            outward_yaw=candidate.outward_yaw,
            width=candidate.width,
            source="rgbd_dark_panel_pending_service_confirmation",
            evidence_count=candidate.depth_inlier_count,
            building_client=building_client,
            landmark_monitor=landmark_monitor,
            elapsed=elapsed,
        )

    def refresh_geometry(
        self,
        *,
        now: float,
        elevator_id: str,
        floor_index: int,
        door_id: str,
        pose: tuple[float, float, float],
        building_features,
        building_client,
        config: ElevatorRegistrationConfig,
        landmark_monitor=None,
        elapsed: float = 0.0,
    ) -> ElevatorRefreshResult:
        context = (str(elevator_id), int(floor_index), str(door_id))
        if context != self.refresh_context:
            self.reset_refresh()
            self.refresh_context = context
            self.refresh_started_at = now
        if self.refresh_started_at is None:
            self.refresh_started_at = now
        if now - float(self.refresh_started_at) > config.refresh_timeout:
            event = {
                "event": "elevator_geometry_refresh_unavailable",
                "elevator_id": str(elevator_id),
                "floor": int(floor_index),
                "door_id": str(door_id),
                "robot_pose": [round(float(value), 3) for value in pose],
                "rejections": dict(self.refresh_rejections),
            }
            self.reset_refresh()
            return ElevatorRefreshResult(
                IDLE,
                warning_message=(
                    "电梯关闭门板近场重观测超时，保留既有连续确认几何："
                    "elevator_id=%s floor=%d door_id=%s",
                    elevator_id,
                    floor_index,
                    door_id,
                ),
                structured_event=event,
            )
        try:
            expected_door_id = building_client.elevator_door_id(floor_index)
        except ValueError as exc:
            self.reset_refresh()
            return ElevatorRefreshResult(
                FAILED,
                error_message=("电梯近场重观测缺少公开门 ID：%s", exc),
            )
        existing = building_client.doors.get(str(door_id))
        elevator_feature = (
            None
            if building_features is None
            else building_features.elevator_features.get(str(elevator_id))
        )
        if (
            str(expected_door_id) != str(door_id)
            or existing is None
            or elevator_feature is None
            or int(floor_index) not in elevator_feature.served_floors
        ):
            self.reset_refresh()
            return ElevatorRefreshResult(
                FAILED,
                error_message=(
                    "电梯近场重观测的公开 ID 或既有运行期几何无效："
                    "elevator_id=%s floor=%d door_id=%s",
                    elevator_id,
                    floor_index,
                    door_id,
                ),
            )
        stamp = self.latest_rgbd_door_stamp
        if stamp is None or stamp == self.refresh_last_stamp:
            return ElevatorRefreshResult(RUNNING)
        self.refresh_last_stamp = stamp
        candidates = []
        for observation in self.latest_rgbd_door_observations:
            center_shift = math.hypot(
                observation.world_xy[0] - existing.x,
                observation.world_xy[1] - existing.y,
            )
            yaw_change = abs(
                math.atan2(
                    math.sin(observation.outward_yaw - existing.yaw),
                    math.cos(observation.outward_yaw - existing.yaw),
                )
            )
            if not config.visual_min_width <= observation.width <= config.visual_max_width:
                self.record_refresh_rejection("width")
                continue
            if observation.distance > config.visual_max_distance:
                self.record_refresh_rejection("distance")
                continue
            if center_shift > config.refresh_max_shift:
                self.record_refresh_rejection("center_shift")
                continue
            if yaw_change > config.refresh_max_yaw_change:
                self.record_refresh_rejection("yaw_change")
                continue
            candidates.append((center_shift, observation))
        if not self.latest_rgbd_door_observations:
            self.record_refresh_rejection("no_observation")
        if not candidates:
            self.refresh_candidate = None
            self.refresh_confirmations = 0
            return ElevatorRefreshResult(RUNNING)
        candidate = min(candidates, key=lambda item: item[0])[1]
        previous = self.refresh_candidate
        consistent = previous is not None and (
            math.hypot(
                candidate.world_xy[0] - previous.world_xy[0],
                candidate.world_xy[1] - previous.world_xy[1],
            )
            <= config.geometry_tolerance
            and abs(candidate.width - previous.width) <= config.geometry_tolerance
            and abs(
                math.atan2(
                    math.sin(candidate.outward_yaw - previous.outward_yaw),
                    math.cos(candidate.outward_yaw - previous.outward_yaw),
                )
            )
            <= config.geometry_tolerance
        )
        self.refresh_confirmations = self.refresh_confirmations + 1 if consistent else 1
        self.refresh_candidate = candidate
        if not consistent and previous is not None:
            self.record_refresh_rejection("inconsistent_sequence")
        if self.refresh_confirmations < max(2, config.visual_confirmations):
            return ElevatorRefreshResult(RUNNING)
        old_xy = (float(existing.x), float(existing.y))
        old_yaw = float(existing.yaw)
        registration = self.register_confirmed_geometry(
            elevator_feature=elevator_feature,
            current_door_id=str(door_id),
            current_floor=floor_index,
            world_xy=candidate.world_xy,
            outward_yaw=candidate.outward_yaw,
            width=candidate.width,
            source="rgbd_dark_panel_task_refresh",
            evidence_count=candidate.depth_inlier_count,
            building_client=building_client,
            landmark_monitor=landmark_monitor,
            elapsed=elapsed,
        )
        event = {
            "event": "elevator_geometry_refresh",
            "elevator_id": str(elevator_id),
            "floor": int(floor_index),
            "old_center": [round(old_xy[0], 3), round(old_xy[1], 3)],
            "new_center": [
                round(float(candidate.world_xy[0]), 3),
                round(float(candidate.world_xy[1]), 3),
            ],
            "center_shift": round(
                math.hypot(
                    candidate.world_xy[0] - old_xy[0],
                    candidate.world_xy[1] - old_xy[1],
                ),
                3,
            ),
            "yaw_shift": round(
                abs(
                    math.atan2(
                        math.sin(candidate.outward_yaw - old_yaw),
                        math.cos(candidate.outward_yaw - old_yaw),
                    )
                ),
                3,
            ),
            "robot_pose": [round(float(value), 3) for value in pose],
            "confirmations": int(self.refresh_confirmations),
        }
        self.reset_refresh()
        return ElevatorRefreshResult(
            DONE,
            info_message=registration.info_message,
            structured_event=event,
        )

    def register_confirmed_geometry(
        self,
        *,
        elevator_feature,
        current_door_id: str,
        current_floor: int,
        world_xy: tuple[float, float],
        outward_yaw: float,
        width: float,
        source: str,
        evidence_count: int,
        building_client,
        landmark_monitor,
        elapsed: float,
    ) -> ElevatorRegistrationResult:
        events = []
        if landmark_monitor is not None and landmark_monitor.freeze_anchor(
            stamp=self.latest_rgbd_door_stamp,
            floor=int(current_floor),
            world_xy=world_xy,
            outward_yaw=outward_yaw,
            width=width,
            source=source,
        ):
            anchor = landmark_monitor.anchor
            events.append(
                {
                    "event": "landmark_drift_anchor_frozen",
                    "elapsed": round(float(elapsed), 3),
                    "observation_stamp": anchor.stamp,
                    "floor": anchor.floor,
                    "world_xy": list(anchor.world_xy),
                    "outward_yaw": anchor.outward_yaw,
                    "width": anchor.width,
                    "source": anchor.source,
                }
            )
        existing_elevator = building_client.elevators.get(elevator_feature.elevator_id)
        lobby_poses = (
            dict(existing_elevator.lobby_poses) if existing_elevator is not None else {}
        )
        # 为尚未注册的楼层播种初始竖井先验（垂直竖井），已有楼层的独立坐标予以保留
        for floor in elevator_feature.served_floors:
            fid = int(floor)
            if fid not in lobby_poses:
                lobby_poses[fid] = (
                    float(world_xy[0]),
                    float(world_xy[1]),
                    float(outward_yaw),
                )
        # 当前层始终更新为本次观测/确认的最新几何
        lobby_poses[int(current_floor)] = (
            float(world_xy[0]),
            float(world_xy[1]),
            float(outward_yaw),
        )
        building_client.elevators[elevator_feature.elevator_id] = ElevatorInfo(
            elevator_id=elevator_feature.elevator_id,
            lobby_poses=lobby_poses,
        )
        # 门几何同理：为未初始化的服务楼层门建立先验，确保到达目标层时开门确认具备门几何上下文
        for floor in elevator_feature.served_floors:
            fid = int(floor)
            door_id = building_client.elevator_door_id(fid)
            if door_id not in building_client.doors:
                building_client.doors[door_id] = DoorInfo(
                    door_id=door_id,
                    kind="elevator",
                    floor_index=fid,
                    x=float(world_xy[0]),
                    y=float(world_xy[1]),
                    yaw=float(outward_yaw),
                    width=float(width),
                )
        # 当前层门信息更新为最新确认几何
        current_door = str(current_door_id)
        building_client.doors[current_door] = DoorInfo(
            door_id=current_door,
            kind="elevator",
            floor_index=int(current_floor),
            x=float(world_xy[0]),
            y=float(world_xy[1]),
            yaw=float(outward_yaw),
            width=float(width),
        )
        return ElevatorRegistrationResult(
            registered=True,
            active_elevator_id=elevator_feature.elevator_id,
            info_message=(
                "局部传感器连续确认电梯候选并注册运行期竖井几何："
                "elevator_id=%s door_id=%s floor=%d center=(%.2f, %.2f) "
                "outward_yaw=%.2f width=%.2f evidence=%d source=%s",
                elevator_feature.elevator_id,
                current_door_id,
                current_floor,
                world_xy[0],
                world_xy[1],
                outward_yaw,
                width,
                evidence_count,
                source,
            ),
            structured_events=tuple(events),
        )

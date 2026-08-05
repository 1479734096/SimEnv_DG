"""电梯视觉观测的跨帧状态账本。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from danger_search.elevator_registration_runtime import ElevatorRegistrationRuntime
from danger_search.mission_reporting import MissionReportingRuntime


@dataclass(frozen=True)
class ElevatorVisualScanConfig:
    duration: float
    angular_speed: float
    required_rotation: float


@dataclass(frozen=True)
class ElevatorVisualScanResult:
    handled: bool = False
    clear_exploration_target: bool = False
    info_message: tuple[object, ...] | None = None
    structured_event: dict[str, object] | None = None


@dataclass(frozen=True)
class ElevatorDoorConfirmationResult:
    confirmed: bool
    structured_event: dict[str, object] | None = None


@dataclass
class ElevatorObservationRuntime:
    registration: ElevatorRegistrationRuntime = field(
        default_factory=ElevatorRegistrationRuntime
    )
    door_open_confirmation: dict[str, object] | None = None
    visual_prepare_task: object | None = None
    visual_prepare_door_id: str | None = None
    visual_prepare_result_logged: bool = False
    visual_scan_started: bool = False
    visual_scan_completed: bool = False
    visual_scan_until: float = 0.0
    visual_scan_last_yaw: float | None = None
    visual_scan_rotation: float = 0.0

    def start_visual_preparation(
        self,
        *,
        rgbd_enabled: bool,
        max_floor: int,
        current_floor: int,
        building_features,
        building_client,
    ) -> None:
        """异步关闭公开声明为初始打开的电梯门，探索线程不等待。"""
        if (
            not rgbd_enabled
            or int(max_floor) <= 1
            or self.visual_prepare_door_id is not None
            or building_features is None
        ):
            return
        candidates = [
            door
            for door in building_features.doors.values()
            if door.kind == "elevator"
            and int(door.floor_index) == int(current_floor)
            and bool(door.initial_open)
        ]
        if len(candidates) != 1 or not building_client.door_service_ready():
            return
        door_id = str(candidates[0].door_id)
        self.visual_prepare_door_id = door_id
        try:
            self.visual_prepare_task = building_client.close_door(door_id)
        except Exception as exc:
            MissionReportingRuntime.logwarn(
                "电梯视觉准备未启动：door_id=%s error=%s",
                door_id,
                exc,
            )
            return
        MissionReportingRuntime.loginfo(
            "已异步请求关闭初始打开的电梯门以建立视觉地标："
            "door_id=%s；探索不等待",
            door_id,
        )

    def visual_preparation_pending(self, door_id: str) -> bool:
        task = self.visual_prepare_task
        return bool(
            task is not None
            and str(door_id) == str(self.visual_prepare_door_id)
            and not task.done()
        )

    def log_visual_preparation_result(self) -> None:
        task = self.visual_prepare_task
        if task is None or not task.done() or self.visual_prepare_result_logged:
            return
        self.visual_prepare_result_logged = True
        log = (
            MissionReportingRuntime.loginfo
            if task.ok()
            else MissionReportingRuntime.logwarn
        )
        log(
            "电梯视觉准备完成：door_id=%s ok=%s message=%s",
            self.visual_prepare_door_id,
            task.ok(),
            task.message(),
        )

    def tick_visual_scan(
        self,
        *,
        now: float,
        pose: tuple[float, float, float],
        current_floor: int,
        current_task_present: bool,
        building_features,
        building_client,
        rgbd_detector,
        controller,
        last_door_id: str | None,
        last_door_xy: tuple[float, float] | None,
        post_door_context_satisfied: bool,
        config: ElevatorVisualScanConfig,
    ) -> ElevatorVisualScanResult:
        """在主入口内执行一次有界全向视觉扫描。"""
        if (
            self.visual_scan_completed
            or rgbd_detector is None
            or int(current_floor) != 0
            or current_task_present
            or building_features is None
            or len(building_features.elevator_features) != 1
        ):
            return ElevatorVisualScanResult()

        elevator_feature = next(iter(building_features.elevator_features.values()))
        existing = building_client.elevators.get(elevator_feature.elevator_id)
        if existing is not None and 0 in existing.lobby_poses:
            event = None
            if self.visual_scan_started:
                controller.stop(reason="elevator_visual_landmark_found")
                event = self.visual_scan_diagnostics(
                    detector=rgbd_detector,
                    required_rotation=config.required_rotation,
                )
                event.update(
                    {
                        "event": "elevator_visual_scan_complete",
                        "result": "landmark_found",
                        "timed_out": False,
                    }
                )
            self.visual_scan_completed = True
            return ElevatorVisualScanResult(structured_event=event)

        preparation = self.visual_prepare_task
        if preparation is None or not preparation.done() or not preparation.ok():
            return ElevatorVisualScanResult()
        entrance_ids = {
            door.door_id
            for door in building_features.doors.values()
            if door.kind == "main_entrance"
        }
        if (
            last_door_id not in entrance_ids
            or last_door_xy is None
            or not post_door_context_satisfied
        ):
            return ElevatorVisualScanResult()

        clear_target = False
        info_message = None
        if not self.visual_scan_started:
            self.visual_scan_started = True
            self.visual_scan_until = now + float(config.duration)
            controller.clear_path(reason="elevator_visual_landmark_scan")
            clear_target = True
            self.visual_scan_last_yaw = float(pose[2])
            self.visual_scan_rotation = 0.0
            self.registration.visual_association_diagnostics = {}
            self.registration.visual_max_confirmations = int(
                self.registration.visual_confirmations
            )
            reset_detector_diagnostics = getattr(
                rgbd_detector, "reset_diagnostics", None
            )
            if callable(reset_detector_diagnostics):
                reset_detector_diagnostics()
            info_message = (
                "开始一次全向 RGB-D 电梯门地标扫描：timeout=%.1f angular=%.2f "
                "required_rotation=%.2f",
                config.duration,
                config.angular_speed,
                config.required_rotation,
            )

        last_yaw = self.visual_scan_last_yaw
        if last_yaw is not None:
            yaw_delta = math.atan2(
                math.sin(float(pose[2]) - float(last_yaw)),
                math.cos(float(pose[2]) - float(last_yaw)),
            )
            # 扫描方向固定为正，落脚扰动造成的反向回摆不能抵消已覆盖方位。
            if yaw_delta > 0.0:
                self.visual_scan_rotation += yaw_delta
        self.visual_scan_last_yaw = float(pose[2])
        rotation_complete = (
            float(self.visual_scan_rotation) >= float(config.required_rotation)
        )
        if now < float(self.visual_scan_until):
            if int(self.registration.visual_confirmations) > 0:
                controller.stop(reason="elevator_visual_candidate_confirmation")
            else:
                controller.rotate_in_place(config.angular_speed)
            return ElevatorVisualScanResult(
                handled=True,
                clear_exploration_target=clear_target,
                info_message=info_message,
            )

        controller.stop(reason="elevator_visual_landmark_scan_complete")
        self.visual_scan_completed = True
        diagnostic = self.visual_scan_diagnostics(
            detector=rgbd_detector,
            required_rotation=config.required_rotation,
        )
        diagnostic.update(
            {
                "event": "elevator_visual_scan_complete",
                "result": "landmark_not_found",
                "timed_out": True,
                "coverage_complete": bool(rotation_complete),
            }
        )
        return ElevatorVisualScanResult(
            clear_exploration_target=clear_target,
            info_message=(
                "全向 RGB-D 电梯门地标扫描结束：rotation=%.2f required=%.2f "
                "coverage_complete=%d timed_out=1，恢复正常探索",
                self.visual_scan_rotation,
                config.required_rotation,
                1 if rotation_complete else 0,
            ),
            structured_event=diagnostic,
        )

    def visual_scan_diagnostics(
        self,
        *,
        detector,
        required_rotation: float,
    ) -> dict[str, object]:
        """汇总扫描证据，不改变候选确认状态。"""
        detector_snapshot = getattr(detector, "diagnostics", None)
        detector_diagnostics = (
            detector_snapshot() if callable(detector_snapshot) else None
        )
        observations = [
            {
                "world_xy": [
                    round(float(observation.world_xy[0]), 3),
                    round(float(observation.world_xy[1]), 3),
                ],
                "outward_yaw": round(float(observation.outward_yaw), 3),
                "width": round(float(observation.width), 3),
                "height": round(float(observation.height), 3),
                "plane_rmse": round(float(observation.plane_rmse), 4),
                "distance": round(float(observation.distance), 3),
                "panel_state": str(observation.panel_state),
            }
            for observation in self.registration.latest_rgbd_door_observations
        ]
        candidate = self.registration.visual_candidate
        return {
            "rotation": round(self.visual_scan_rotation, 3),
            "required_rotation": round(float(required_rotation), 3),
            "confirmations": int(self.registration.visual_confirmations),
            "max_confirmations": int(self.registration.visual_max_confirmations),
            "association": dict(self.registration.visual_association_diagnostics),
            "candidate": (
                {
                    "world_xy": [
                        round(float(candidate.world_xy[0]), 3),
                        round(float(candidate.world_xy[1]), 3),
                    ],
                    "outward_yaw": round(float(candidate.outward_yaw), 3),
                    "width": round(float(candidate.width), 3),
                }
                if candidate is not None
                else None
            ),
            "latest_observations": observations,
            "detector": detector_diagnostics,
        }

    def begin_door_open_confirmation(
        self,
        *,
        door_id: str,
        building_client,
        geometry_tolerance: float,
    ) -> dict[str, object] | None:
        """保存同一视角门板基线，不发送服务或控制命令。"""
        context = self.door_open_confirmation
        if context is not None and context.get("door_id") == door_id:
            return None
        door = getattr(building_client, "doors", {}).get(door_id)
        baseline = None
        if door is not None:
            nearby = [
                observation
                for observation in self.registration.latest_rgbd_door_observations
                if math.hypot(
                    observation.world_xy[0] - door.x,
                    observation.world_xy[1] - door.y,
                )
                <= max(0.45, 2.0 * geometry_tolerance)
            ]
            if nearby:
                baseline = min(
                    nearby,
                    key=lambda observation: math.hypot(
                        observation.world_xy[0] - door.x,
                        observation.world_xy[1] - door.y,
                    ),
                )
        self.door_open_confirmation = {
            "door_id": str(door_id),
            "baseline": baseline,
            "last_image_stamp": getattr(baseline, "stamp", None),
            "visual_confirmations": 0,
            "last_visual_change": None,
            "last_cloud_stamp": None,
            "livox_confirmations": 0,
            "logged": False,
        }
        return {
            "event": "elevator_door_open_confirmation_started",
            "door_id": str(door_id),
            "door_registered": door is not None,
            "rgbd_observation_count": len(
                self.registration.latest_rgbd_door_observations
            ),
            "baseline_present": baseline is not None,
            "baseline_stamp": getattr(baseline, "stamp", None),
            "baseline_distance": getattr(baseline, "distance", None),
            "baseline_world_xy": getattr(baseline, "world_xy", None),
            "baseline_median_depth": getattr(baseline, "median_depth", None),
            "baseline_dark_fraction": getattr(
                baseline, "center_dark_fraction", None
            ),
        }

    def update_door_visual_change(
        self,
        *,
        bundle,
        image_stamp,
        camera_to_world,
        detector,
        building_client,
    ) -> None:
        context = self.door_open_confirmation
        if context is None or detector is None or image_stamp is None:
            return
        if image_stamp == context.get("last_image_stamp"):
            return
        context["last_image_stamp"] = image_stamp
        baseline = context.get("baseline")
        change = None
        if baseline is not None:
            change = detector.assess_opening_change_messages(
                baseline,
                bundle.rgb_msg,
                bundle.depth_msg,
            )
            if change.confirmed:
                context["visual_confirmation_source"] = "closed_frame_baseline"
        assess_registered = getattr(
            detector, "assess_registered_opening_messages", None
        )
        if (
            (change is None or not change.confirmed)
            and callable(assess_registered)
            and camera_to_world is not None
        ):
            door = getattr(building_client, "doors", {}).get(
                context.get("door_id")
            )
            if door is not None:
                registered_change = assess_registered(
                    bundle.rgb_msg,
                    bundle.depth_msg,
                    bundle.camera_info_msg,
                    camera_to_world,
                    door_xy=(door.x, door.y),
                    door_yaw=door.yaw,
                    door_width=door.width,
                )
                if registered_change.confirmed or change is None:
                    change = registered_change
                if registered_change.confirmed:
                    context["visual_confirmation_source"] = (
                        "registered_door_plane"
                    )
        if change is None:
            return
        context["last_visual_change"] = change
        context["visual_confirmations"] = (
            int(context.get("visual_confirmations", 0)) + 1
            if change.confirmed
            else 0
        )

    def confirm_door_open(
        self,
        *,
        door_id: str,
        pose: tuple[float, float, float],
        cloud_stamp: float | None,
        cloud_points,
        floor_index: int,
        floor_height: float | None,
        floor_transition_explicit: bool,
        geometry_tolerance: float,
        building_client,
        local_detector,
    ) -> ElevatorDoorConfirmationResult:
        """确认服务后的 RGB-D 变化或连续 Livox 开口。"""
        context = self.door_open_confirmation
        if context is None or context.get("door_id") != door_id:
            self.begin_door_open_confirmation(
                door_id=door_id,
                building_client=building_client,
                geometry_tolerance=geometry_tolerance,
            )
            context = self.door_open_confirmation
        assert context is not None
        if int(context.get("visual_confirmations", 0)) >= 2:
            confirmed = True
        else:
            confirmed = False
            if (
                not floor_transition_explicit
                and cloud_stamp is not None
                and cloud_stamp != context.get("last_cloud_stamp")
            ):
                context["last_cloud_stamp"] = cloud_stamp
                door = getattr(building_client, "doors", {}).get(door_id)
                if door is not None and floor_height is not None:
                    observation_yaw = math.atan2(
                        door.y - pose[1], door.x - pose[0]
                    )
                    observation = local_detector.detect_opening_ahead(
                        robot_pose=(pose[0], pose[1], observation_yaw),
                        floor_index=floor_index,
                        cloud_points=cloud_points,
                        floor_height=floor_height,
                    )
                    matches = observation is not None and (
                        math.hypot(
                            observation.world_xy[0] - door.x,
                            observation.world_xy[1] - door.y,
                        )
                        <= max(0.45, 2.0 * geometry_tolerance)
                        and abs(observation.width - door.width) <= 0.50
                    )
                    context["livox_confirmations"] = (
                        int(context.get("livox_confirmations", 0)) + 1
                        if matches
                        else 0
                    )
            confirmed = int(context.get("livox_confirmations", 0)) >= 2

        event = None
        if confirmed and not bool(context.get("logged", False)):
            context["logged"] = True
            change = context.get("last_visual_change")
            event = {
                "event": "elevator_door_open_sensor_confirmed",
                "door_id": str(door_id),
                "visual_confirmations": int(
                    context.get("visual_confirmations", 0)
                ),
                "livox_confirmations": int(
                    context.get("livox_confirmations", 0)
                ),
                "dark_fraction": getattr(change, "dark_fraction", None),
                "deeper_fraction": getattr(change, "deeper_fraction", None),
                "confirmation_source": context.get("visual_confirmation_source"),
            }
        return ElevatorDoorConfirmationResult(confirmed, event)

    def finish_door_open_confirmation(
        self,
        door_id: str,
    ) -> dict[str, object] | None:
        context = self.door_open_confirmation
        if context is None or context.get("door_id") != door_id:
            return None
        change = context.get("last_visual_change")
        event = {
            "event": "elevator_door_open_confirmation_finished",
            "door_id": str(door_id),
            "baseline_present": context.get("baseline") is not None,
            "visual_confirmations": int(context.get("visual_confirmations", 0)),
            "livox_confirmations": int(context.get("livox_confirmations", 0)),
            "last_dark_fraction": getattr(change, "dark_fraction", None),
            "last_valid_depth_fraction": getattr(
                change, "valid_depth_fraction", None
            ),
            "last_deeper_fraction": getattr(change, "deeper_fraction", None),
            "confirmation_source": context.get("visual_confirmation_source"),
        }
        self.door_open_confirmation = None
        return event

"""主入口门洞跨帧确认与近场刷新状态。

本模块只解释已经由合法点云检测器生成的门洞观测，不读取 ROS、不调用服务、不修改
地图或控制器。MissionManager 负责采集输入和编排任务，本模块负责有限状态与几何一致性。
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from danger_search.building_control_client import DoorInfo
from danger_search.local_semantic_detector import OpeningObservation
from danger_search.mission_reporting import MissionReportingRuntime
from danger_search.navigation_tasks import FAILED, OpenDoorAndPass
from danger_search.navigator import DONE, RUNNING


WAITING = "waiting"
PROCESS = "process"
ACCEPTED = "accepted"
TIMEOUT = "timeout"
REJECTED = "rejected"


def minimum_refresh_width(
    *,
    sweep_half_width: float,
    clearance_margin: float,
    measurement_resolution: float,
) -> float:
    """按门洞宽度量化误差修正净空阈值，同时不突破机器人扫掠宽度。"""
    sweep_width = 2.0 * max(0.0, float(sweep_half_width))
    requested_width = 2.0 * (
        max(0.0, float(sweep_half_width))
        + max(0.0, float(clearance_margin))
    )
    resolution = float(measurement_resolution)
    if not math.isfinite(resolution) or resolution < 0.0:
        resolution = 0.0
    return max(sweep_width, requested_width - resolution)


def _observations_are_consistent(
    previous: OpeningObservation,
    current: OpeningObservation,
    tolerance: float,
) -> bool:
    return (
        math.hypot(
            current.world_xy[0] - previous.world_xy[0],
            current.world_xy[1] - previous.world_xy[1],
        )
        <= tolerance
        and abs(current.width - previous.width) <= tolerance
        and abs(
            math.atan2(
                math.sin(current.yaw - previous.yaw),
                math.cos(current.yaw - previous.yaw),
            )
        )
        <= tolerance
    )


class InitialOpeningConfirmation:
    """确认初次入口门洞；重复 stamp 不推进确认代次。"""

    def __init__(self) -> None:
        self.last_stamp: float | None = None
        self.confirmations = 0
        self.observation: OpeningObservation | None = None

    def observe(
        self,
        *,
        stamp: float | None,
        observation: OpeningObservation | None,
        maximum_distance: float,
        tolerance: float,
        required_confirmations: int,
    ) -> bool:
        if stamp is None or stamp == self.last_stamp:
            return False
        self.last_stamp = float(stamp)
        if observation is None or observation.distance > maximum_distance:
            self.confirmations = 0
            self.observation = None
            return False

        consistent = self.observation is not None and _observations_are_consistent(
            self.observation, observation, tolerance
        )
        self.confirmations = self.confirmations + 1 if consistent else 1
        self.observation = observation
        return self.confirmations >= max(1, int(required_confirmations))


@dataclass(frozen=True)
class DoorRefreshAssessment:
    status: str
    reason: str | None = None
    observation: OpeningObservation | None = None
    confirmations: int = 0
    axis_error: float | None = None
    center_shift: float | None = None


@dataclass(frozen=True)
class DoorRefreshConfig:
    timeout: float
    maximum_center_shift: float
    clearance_margin: float
    opening_max_distance: float
    geometry_tolerance: float
    required_confirmations: int
    sweep_half_width: float


@dataclass(frozen=True)
class DoorRefreshResult:
    status: str
    failure: str | None = None
    diagnostic: dict[str, object] | None = None


class DoorRefreshConfirmation:
    """跟踪一次近场刷新，输出有界拒绝分类但不决定任务终局。"""

    def __init__(self) -> None:
        self.last_stamp: float | None = None
        self.started_at: float | None = None
        self.confirmations = 0
        self.observation: OpeningObservation | None = None
        self.rejection_counts: dict[str, int] = {}
        self.last_candidate: dict[str, float | int] | None = None

    def precheck(
        self,
        *,
        now: float,
        stamp: float | None,
        timeout: float,
    ) -> str:
        if self.started_at is None:
            self.started_at = float(now)
            self.rejection_counts = {}
            self.last_candidate = None
        if float(now) - self.started_at > float(timeout):
            return TIMEOUT
        if stamp is None or stamp == self.last_stamp:
            return WAITING
        self.last_stamp = float(stamp)
        return PROCESS

    @staticmethod
    def query_yaw(
        existing: DoorInfo,
        pose: tuple[float, float, float],
    ) -> float:
        yaw = float(existing.yaw)
        center_dx = float(existing.x) - float(pose[0])
        center_dy = float(existing.y) - float(pose[1])
        if center_dx * math.cos(yaw) + center_dy * math.sin(yaw) < 0.0:
            yaw = math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))
        return yaw

    def assess(
        self,
        *,
        observation: OpeningObservation | None,
        existing: DoorInfo,
        maximum_distance: float,
        minimum_width: float,
        tolerance: float,
        maximum_center_shift: float,
        required_confirmations: int,
    ) -> DoorRefreshAssessment:
        if observation is None:
            self.last_candidate = None
            return self._reject("no_candidate")

        self.last_candidate = {
            "distance": round(float(observation.distance), 3),
            "width": round(float(observation.width), 3),
            "x": round(float(observation.world_xy[0]), 3),
            "y": round(float(observation.world_xy[1]), 3),
            "yaw": round(float(observation.yaw), 3),
            "evidence_count": int(observation.evidence_count),
        }
        if observation.distance > maximum_distance:
            return self._reject("distance_exceeded", observation)
        if observation.width < minimum_width:
            return self._reject("width_insufficient", observation)

        yaw_delta = float(observation.yaw) - float(existing.yaw)
        axis_error = 0.5 * abs(
            math.atan2(math.sin(2.0 * yaw_delta), math.cos(2.0 * yaw_delta))
        )
        if axis_error > tolerance:
            return self._reject(
                "axis_mismatch", observation, axis_error=axis_error
            )

        center_shift = math.hypot(
            float(observation.world_xy[0]) - float(existing.x),
            float(observation.world_xy[1]) - float(existing.y),
        )
        if center_shift > maximum_center_shift:
            return self._reject(
                "center_shift_exceeded",
                observation,
                axis_error=axis_error,
                center_shift=center_shift,
            )

        previous = self.observation
        consistent = previous is not None and _observations_are_consistent(
            previous, observation, tolerance
        )
        self.confirmations = self.confirmations + 1 if consistent else 1
        if previous is not None and not consistent:
            self.rejection_counts["inconsistent_candidate"] = int(
                self.rejection_counts.get("inconsistent_candidate", 0)
            ) + 1
        self.observation = observation
        if self.confirmations < max(1, int(required_confirmations)):
            return DoorRefreshAssessment(
                status=WAITING,
                reason="inconsistent_candidate" if previous is not None and not consistent else None,
                observation=observation,
                confirmations=self.confirmations,
                axis_error=axis_error,
                center_shift=center_shift,
            )

        confirmations = self.confirmations
        self.started_at = None
        self.confirmations = 0
        self.observation = None
        return DoorRefreshAssessment(
            status=ACCEPTED,
            observation=observation,
            confirmations=confirmations,
            axis_error=axis_error,
            center_shift=center_shift,
        )

    def diagnostic_snapshot(
        self,
        *,
        door_id: str,
        pose: tuple[float, float, float] | None = None,
    ) -> dict[str, object]:
        snapshot: dict[str, object] = {
            "door_id": str(door_id),
            "rejection_counts": dict(sorted(self.rejection_counts.items())),
            "last_candidate": self.last_candidate,
        }
        if pose is not None:
            snapshot["pose"] = [round(float(value), 3) for value in pose]
        return snapshot

    def _reject(
        self,
        reason: str,
        observation: OpeningObservation | None = None,
        *,
        axis_error: float | None = None,
        center_shift: float | None = None,
    ) -> DoorRefreshAssessment:
        self.rejection_counts[reason] = int(self.rejection_counts.get(reason, 0)) + 1
        self.confirmations = 0
        self.observation = None
        return DoorRefreshAssessment(
            status=REJECTED,
            reason=reason,
            observation=observation,
            axis_error=axis_error,
            center_shift=center_shift,
        )


class DoorObservationRuntime:
    """聚合初次入口与返航刷新状态，生命周期与 MissionManager 实例一致。"""

    def __init__(self) -> None:
        self.initial = InitialOpeningConfirmation()
        self.refresh = DoorRefreshConfirmation()

    def refresh_geometry(
        self,
        *,
        door_id: str,
        entry_registered: bool,
        pose: tuple[float, float, float],
        now: float,
        cloud_stamp: float | None,
        current_floor: int,
        cloud_points,
        floor_height: float,
        local_detector,
        building_client,
        current_task,
        config: DoorRefreshConfig,
    ) -> DoorRefreshResult:
        """刷新开放主入口几何；任务编排器只消费冻结结果。"""
        if str(door_id) != "main_entrance" or not entry_registered:
            return DoorRefreshResult(DONE)
        precheck = self.refresh.precheck(
            now=now,
            stamp=cloud_stamp,
            timeout=config.timeout,
        )
        existing = building_client.doors.get(str(door_id))
        if precheck == TIMEOUT:
            diagnostic = self.refresh.diagnostic_snapshot(
                door_id=door_id,
                pose=pose,
            )
            if existing is not None:
                diagnostic["event"] = "main_entrance_refresh_timeout_degraded"
                self.refresh.started_at = None
                self.refresh.confirmations = 0
                self.refresh.observation = None
                if (
                    isinstance(current_task, OpenDoorAndPass)
                    and current_task.door_id == str(door_id)
                ):
                    current_task.mark_opening_sensor_confirmed()
                MissionReportingRuntime.logwarn(
                    "开放主入口近场重观测超时，保留既有连续确认门几何并降级放行："
                    "door_id=%s center=(%.2f, %.2f) yaw=%.2f width=%.2f",
                    door_id,
                    existing.x,
                    existing.y,
                    existing.yaw,
                    existing.width,
                )
                return DoorRefreshResult(DONE, diagnostic=diagnostic)
            self.refresh.started_at = None
            self.refresh.confirmations = 0
            self.refresh.observation = None
            diagnostic["event"] = "main_entrance_refresh_timeout"
            return DoorRefreshResult(
                FAILED,
                failure="主入口开放门洞近场重观测超时",
                diagnostic=diagnostic,
            )
        if precheck != PROCESS:
            return DoorRefreshResult(RUNNING)
        if existing is None:
            return DoorRefreshResult(
                FAILED,
                failure="主入口近场重观测时缺少已注册门几何",
            )
        query_yaw = self.refresh.query_yaw(existing, pose)
        observation = local_detector.detect_opening_ahead(
            robot_pose=(float(pose[0]), float(pose[1]), query_yaw),
            floor_index=int(current_floor),
            cloud_points=cloud_points,
            floor_height=float(floor_height),
            maximum_distance=config.opening_max_distance,
            expected_center_xy=(float(existing.x), float(existing.y)),
            maximum_center_error=config.maximum_center_shift,
        )
        minimum_width = minimum_refresh_width(
            sweep_half_width=config.sweep_half_width,
            clearance_margin=config.clearance_margin,
            measurement_resolution=getattr(
                local_detector,
                "opening_width_resolution",
                0.0,
            ),
        )
        assessment = self.refresh.assess(
            observation=observation,
            existing=existing,
            maximum_distance=config.opening_max_distance,
            minimum_width=minimum_width,
            tolerance=config.geometry_tolerance,
            maximum_center_shift=config.maximum_center_shift,
            required_confirmations=config.required_confirmations,
        )
        if assessment.reason == "axis_mismatch" and observation is not None:
            MissionReportingRuntime.logwarn_throttle(
                1.0,
                "主入口近场候选与已注册门轴不一致，拒绝覆盖："
                "candidate_yaw=%.2f registered_yaw=%.2f "
                "axis_error=%.2f tolerance=%.2f",
                observation.yaw,
                existing.yaw,
                assessment.axis_error,
                config.geometry_tolerance,
            )
            return DoorRefreshResult(RUNNING)
        if (
            assessment.reason == "center_shift_exceeded"
            and observation is not None
        ):
            MissionReportingRuntime.logwarn_throttle(
                1.0,
                "主入口近场候选距已注册中心过远，拒绝覆盖："
                "candidate=(%.2f, %.2f) registered=(%.2f, %.2f) "
                "shift=%.2f maximum=%.2f",
                observation.world_xy[0],
                observation.world_xy[1],
                existing.x,
                existing.y,
                assessment.center_shift,
                config.maximum_center_shift,
            )
            return DoorRefreshResult(RUNNING)
        if assessment.status != ACCEPTED:
            return DoorRefreshResult(RUNNING)
        observation = assessment.observation
        if observation is None:  # pragma: no cover - accepted 合同保证
            raise RuntimeError("入口近场刷新已接受但缺少观测")
        building_client.doors[str(door_id)] = DoorInfo(
            door_id=str(door_id),
            kind=existing.kind,
            floor_index=existing.floor_index,
            x=float(observation.world_xy[0]),
            y=float(observation.world_xy[1]),
            yaw=float(observation.yaw),
            width=float(observation.width),
        )
        if (
            isinstance(current_task, OpenDoorAndPass)
            and current_task.door_id == str(door_id)
        ):
            current_task.mark_opening_sensor_confirmed()
        MissionReportingRuntime.loginfo(
            "开放主入口近场重观测完成：observed_center=(%.2f, %.2f) "
            "aligned_center=(%.2f, %.2f) observed_yaw=%.2f aligned_yaw=%.2f "
            "width=%.2f minimum_width=%.2f confirmations=%d",
            observation.world_xy[0],
            observation.world_xy[1],
            observation.world_xy[0],
            observation.world_xy[1],
            observation.yaw,
            observation.yaw,
            observation.width,
            minimum_width,
            assessment.confirmations,
        )
        diagnostic = self.refresh.diagnostic_snapshot(door_id=door_id)
        diagnostic["event"] = "main_entrance_refresh_confirmed"
        return DoorRefreshResult(DONE, diagnostic=diagnostic)

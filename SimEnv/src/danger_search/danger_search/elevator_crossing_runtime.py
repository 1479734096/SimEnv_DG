"""电梯开门后的任务级局部穿越几何。"""
from __future__ import annotations

from dataclasses import dataclass
import math

from danger_search.map_update_runtime import ForwardObstacleObservation


RUNNING = "running"
DONE = "done"
FAILED = "failed"


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass(frozen=True)
class ElevatorCrossingConfig:
    timeout: float = 4.0
    exit_empty_fallback_timeout: float = 1.0
    exit_empty_fallback_frames: int = 10
    confirmations: int = 3
    confirmation_max_gap: float = 0.5
    center_tolerance: float = 0.15
    width_tolerance: float = 0.20
    yaw_tolerance: float = 0.15
    yaw_search_half_range: float = 0.20
    yaw_search_step: float = 0.05
    maximum_center_correction: float = 0.80
    sweep_half_width: float = 0.435393
    min_door_width: float = 1.00
    max_door_width: float = 1.50

    def __post_init__(self) -> None:
        values = (
            self.timeout,
            self.exit_empty_fallback_timeout,
            self.confirmation_max_gap,
            self.center_tolerance,
            self.width_tolerance,
            self.yaw_tolerance,
            self.yaw_search_half_range,
            self.yaw_search_step,
            self.maximum_center_correction,
            self.sweep_half_width,
            self.min_door_width,
            self.max_door_width,
        )
        if (
            not all(math.isfinite(value) and value > 0.0 for value in values)
            or self.min_door_width >= self.max_door_width
            or self.confirmations <= 0
            or self.exit_empty_fallback_frames <= 0
            or self.exit_empty_fallback_timeout > self.timeout
            or self.confirmation_max_gap > self.timeout
            or self.yaw_search_step > 2.0 * self.yaw_search_half_range
        ):
            raise ValueError("电梯局部穿越几何参数必须为有限正值且时序范围有效")


@dataclass(frozen=True)
class ElevatorCrossingGeometry:
    door_id: str
    mode: str
    door_center: tuple[float, float]
    outward_yaw: float
    width: float
    wait_point: tuple[float, float]
    cabin_center: tuple[float, float]
    generation_count: int
    old_center_shift: float


@dataclass(frozen=True)
class ElevatorCrossingResult:
    status: str
    geometry: ElevatorCrossingGeometry | None = None
    failure: str | None = None
    diagnostic: dict[str, object] | None = None


class ElevatorCrossingRuntime:
    """只在一次已确认开门的任务阶段内累计 Livox 开口证据。"""

    _WAIT_STANDOFF = 0.80
    _CABIN_STANDOFF = 0.95
    _STANDOFF_TOLERANCE = 0.20

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._key: tuple[str, str] | None = None
        self._started_at: float | None = None
        self._last_cloud_stamp: float | None = None
        self._last_confirmation_time: float | None = None
        self._candidate = None
        self._confirmations = 0
        self._empty_observation_count = 0

    def observe(
        self,
        *,
        now: float,
        door_id: str,
        mode: str,
        expected_crossing_yaw: float,
        old_door_center: tuple[float, float],
        pose: tuple[float, float, float],
        cloud_stamp: float | None,
        cloud_points,
        floor_index: int,
        floor_height: float,
        local_detector,
        config: ElevatorCrossingConfig,
    ) -> ElevatorCrossingResult:
        if mode not in ("enter", "exit"):
            raise ValueError("电梯穿越模式只能是 enter 或 exit")
        key = (str(door_id), str(mode))
        if self._key != key:
            self.reset()
            self._key = key
            self._started_at = float(now)
        assert self._started_at is not None
        if now - self._started_at > config.timeout:
            event = {
                "event": "elevator_local_crossing_geometry_unavailable",
                "door_id": str(door_id),
                "mode": mode,
                "confirmations": int(self._confirmations),
                "elapsed": round(now - self._started_at, 3),
            }
            self.reset()
            return ElevatorCrossingResult(
                FAILED,
                failure="elevator_open_crossing_geometry_unavailable",
                diagnostic=event,
            )
        if cloud_stamp is None or cloud_stamp == self._last_cloud_stamp:
            return ElevatorCrossingResult(RUNNING)
        self._last_cloud_stamp = float(cloud_stamp)

        observations = []
        if old_door_center is not None and pose is not None:
            actual_dist = math.hypot(
                old_door_center[0] - pose[0],
                old_door_center[1] - pose[1],
            )
            expected_standoff = (
                actual_dist
                if math.isfinite(actual_dist) and actual_dist > 0.1
                else (self._WAIT_STANDOFF if mode == "enter" else self._CABIN_STANDOFF)
            )
        else:
            expected_standoff = (
                self._WAIT_STANDOFF if mode == "enter" else self._CABIN_STANDOFF
            )
        tolerance = self._STANDOFF_TOLERANCE
        minimum_opening_distance = max(
            0.15,
            expected_standoff - tolerance,
        )
        maximum_opening_distance = (
            expected_standoff + tolerance
        )
        step_count = int(
            math.floor(config.yaw_search_half_range / config.yaw_search_step + 1e-9)
        )
        for index in range(-step_count, step_count + 1):
            yaw = expected_crossing_yaw + index * config.yaw_search_step
            observation = local_detector.detect_opening_ahead(
                robot_pose=(pose[0], pose[1], yaw),
                floor_index=floor_index,
                cloud_points=cloud_points,
                floor_height=floor_height,
                minimum_distance=minimum_opening_distance,
                maximum_distance=maximum_opening_distance,
                min_opening_width=config.min_door_width,
                max_opening_width=config.max_door_width,
            )
            if observation is not None:
                if config.min_door_width <= observation.width <= config.max_door_width:
                    observations.append(observation)
        if not observations:
            self._candidate = None
            self._confirmations = 0
            self._last_confirmation_time = None
            if mode == "exit":
                self._empty_observation_count += 1
            elapsed = now - self._started_at
            if (
                mode == "exit"
                and self._empty_observation_count
                >= config.exit_empty_fallback_frames
                and elapsed >= config.exit_empty_fallback_timeout
            ):
                event = {
                    "event": "elevator_local_crossing_geometry_early_fallback",
                    "door_id": str(door_id),
                    "mode": mode,
                    "reason": "opening_not_observed",
                    "empty_frame_count": int(self._empty_observation_count),
                    "elapsed": round(elapsed, 3),
                    "cloud_stamp": float(cloud_stamp),
                }
                self.reset()
                return ElevatorCrossingResult(
                    FAILED,
                    failure="elevator_open_crossing_geometry_not_observed",
                    diagnostic=event,
                )
            return ElevatorCrossingResult(
                RUNNING,
                diagnostic={
                    "event": "elevator_local_crossing_geometry_rejected",
                    "door_id": str(door_id),
                    "mode": mode,
                    "reason": "opening_not_observed",
                    "empty_frame_count": int(self._empty_observation_count),
                    "cloud_stamp": float(cloud_stamp),
                },
            )

        self._empty_observation_count = 0
        observation = min(
            observations,
            key=lambda item: (
                abs(_angle_diff(item.yaw, expected_crossing_yaw)),
                abs(float(item.distance) - float(expected_standoff)),
                -int(item.evidence_count),
            ),
        )
        consistent = self._candidate is not None and (
            math.hypot(
                observation.world_xy[0] - self._candidate.world_xy[0],
                observation.world_xy[1] - self._candidate.world_xy[1],
            )
            <= config.center_tolerance
            and abs(observation.width - self._candidate.width)
            <= config.width_tolerance
            and abs(_angle_diff(observation.yaw, self._candidate.yaw))
            <= config.yaw_tolerance
            and self._last_confirmation_time is not None
            and now - self._last_confirmation_time <= config.confirmation_max_gap
        )
        self._confirmations = self._confirmations + 1 if consistent else 1
        self._candidate = observation
        self._last_confirmation_time = float(now)
        diagnostic = {
            "event": "elevator_local_crossing_geometry_candidate",
            "door_id": str(door_id),
            "mode": mode,
            "confirmations": int(self._confirmations),
            "required_confirmations": int(config.confirmations),
            "cloud_stamp": float(cloud_stamp),
            "door_center": [
                round(float(observation.world_xy[0]), 3),
                round(float(observation.world_xy[1]), 3),
            ],
            "crossing_yaw": round(float(observation.yaw), 3),
            "distance": round(float(observation.distance), 3),
            "width": round(float(observation.width), 3),
        }
        if self._confirmations < config.confirmations:
            return ElevatorCrossingResult(RUNNING, diagnostic=diagnostic)

        crossing_yaw = float(observation.yaw)
        outward_yaw = (
            crossing_yaw if mode == "exit" else crossing_yaw + math.pi
        )
        outward_yaw = math.atan2(math.sin(outward_yaw), math.cos(outward_yaw))
        closest_axis = round(outward_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        closest_axis = math.atan2(math.sin(closest_axis), math.cos(closest_axis))
        if abs(_angle_diff(outward_yaw, closest_axis)) <= 0.35:
            outward_yaw = closest_axis
        outward = (math.cos(outward_yaw), math.sin(outward_yaw))
        door_center = (
            float(observation.world_xy[0]),
            float(observation.world_xy[1]),
        )
        wait_point = (
            door_center[0] + self._WAIT_STANDOFF * outward[0],
            door_center[1] + self._WAIT_STANDOFF * outward[1],
        )
        cabin_center = (
            door_center[0] - self._CABIN_STANDOFF * outward[0],
            door_center[1] - self._CABIN_STANDOFF * outward[1],
        )
        geometry = ElevatorCrossingGeometry(
            door_id=str(door_id),
            mode=mode,
            door_center=door_center,
            outward_yaw=outward_yaw,
            width=float(observation.width),
            wait_point=wait_point,
            cabin_center=cabin_center,
            generation_count=int(self._confirmations),
            old_center_shift=math.hypot(
                door_center[0] - old_door_center[0],
                door_center[1] - old_door_center[1],
            ),
        )
        diagnostic.update(
            {
                "event": "elevator_local_crossing_geometry_confirmed",
                "outward_yaw": round(outward_yaw, 3),
                "wait_point": [round(value, 3) for value in wait_point],
                "cabin_center": [round(value, 3) for value in cabin_center],
                "old_center_shift": round(geometry.old_center_shift, 3),
            }
        )
        self.reset()
        return ElevatorCrossingResult(DONE, geometry=geometry, diagnostic=diagnostic)


def crossing_corridor_obstacle(
    points,
    *,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    floor_index: int,
    floor_height: float,
    obstacle_min_height: float,
    obstacle_max_height: float,
    sweep_half_width: float,
    door_center: tuple[float, float],
    door_yaw: float,
    door_width: float,
    door_half_depth: float = 0.35,
    door_clearance_margin: float = 0.10,
) -> ForwardObstacleObservation | None:
    """返回有限穿越走廊内最近的实时点云障碍，门板平面残留除外。"""
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        return None
    heading = (dx / length, dy / length)
    lateral_axis = (-heading[1], heading[0])
    floor_z = floor_index * floor_height
    door_half_width = max(0.0, 0.5 * door_width - door_clearance_margin)
    result = None
    best = math.inf
    for px, py, pz in points:
        height = pz - floor_z
        if not obstacle_min_height <= height <= obstacle_max_height:
            continue
        door_dx = px - door_center[0]
        door_dy = py - door_center[1]
        door_normal = door_dx * math.cos(door_yaw) + door_dy * math.sin(door_yaw)
        door_tangent = -door_dx * math.sin(door_yaw) + door_dy * math.cos(door_yaw)
        if abs(door_normal) <= door_half_depth and abs(door_tangent) <= door_half_width:
            continue
        rel_x = px - start_xy[0]
        rel_y = py - start_xy[1]
        forward = rel_x * heading[0] + rel_y * heading[1]
        lateral = rel_x * lateral_axis[0] + rel_y * lateral_axis[1]
        if not 0.15 <= forward <= length or abs(lateral) > sweep_half_width:
            continue
        distance = math.hypot(rel_x, rel_y)
        if distance < best:
            best = distance
            result = ForwardObstacleObservation(
                distance=float(distance),
                world_xyz=(float(px), float(py), float(pz)),
                forward=float(forward),
                lateral=float(lateral),
                relative_height=float(height),
            )
    return result

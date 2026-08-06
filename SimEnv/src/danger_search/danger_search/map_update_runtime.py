"""正式地图与检测组件的单线程更新边界。"""
from __future__ import annotations

from dataclasses import dataclass
import math

from danger_search.detection_tracker import DetectionTracker
from danger_search.local_semantic_detector import LocalSemanticDetector
from danger_search.mapper import MultiFloorGridMap
from danger_search.red_sphere_detector import RedSphereDetector
from danger_search.rgbd_door_detector import DoorPanelObservation, RgbdDoorDetector


@dataclass(frozen=True)
class ForwardObstacleObservation:
    distance: float
    world_xyz: tuple[float, float, float]
    forward: float
    lateral: float
    relative_height: float


@dataclass(frozen=True)
class CloudMapResult:
    points: tuple[tuple[float, float, float], ...]
    forward_obstacle: ForwardObstacleObservation | None
    radial_obstacle_distance: float | None


@dataclass(frozen=True)
class ImageDetectionResult:
    door_observations: tuple[DoorPanelObservation, ...]


class MapUpdateRuntime:
    """拥有 Mapper、检测器和跟踪器；所有写操作由主线程调用。"""

    def __init__(
        self,
        *,
        map_params: dict,
        minimum_drop: float,
        detection_params: dict,
        tracker_params: dict,
        local_detection_params: dict,
        rgbd_door_detection_params: dict,
        rgbd_door_detection_enabled: bool,
        max_floor: int,
    ) -> None:
        # 帧消费后的派生状态与地图写入属于同一所有权边界。任务编排器只能
        # 读取这些摘要，不能再维护第二份时间戳或障碍缓存。
        self.last_cloud_stamp: float | None = None
        self.last_image_stamp: float | None = None
        self.latest_cloud_points: list[tuple[float, float, float]] = []
        self.nearest_forward_obstacle: ForwardObstacleObservation | None = None
        self.nearest_radial_obstacle_distance: float | None = None
        self.last_floor_alignment_mismatch_key = None
        self.last_rgbd_pose_lookup_log_time = 0.0
        mapper_params = dict(map_params)
        mapper_params["negative_minimum_drop"] = minimum_drop
        self.mapper = MultiFloorGridMap(**mapper_params)
        self.detector = RedSphereDetector(**detection_params)
        self.local_detector = LocalSemanticDetector(**local_detection_params)
        self.rgbd_door_detector = (
            RgbdDoorDetector(**rgbd_door_detection_params)
            if max_floor > 1 and rgbd_door_detection_enabled
            else None
        )
        self.tracker = DetectionTracker(**tracker_params)

    def update_cloud(
        self,
        *,
        floor_index: int,
        cloud_generation: int,
        map_epoch: int,
        localization_epoch: int,
        robot_xyz: tuple[float, float, float],
        robot_pose: tuple[float, float, float],
        points,
        opened_door_exclusion: tuple[float, float, float, float, float] | None,
    ) -> CloudMapResult:
        frozen_points = tuple(
            (float(point[0]), float(point[1]), float(point[2])) for point in points
        )
        self.mapper.update_from_cloud(
            floor_index=int(floor_index),
            robot_xyz=robot_xyz,
            points=frozen_points,
            cloud_generation=int(cloud_generation),
            map_epoch=int(map_epoch),
            localization_epoch=int(localization_epoch),
        )
        forward = nearest_forward_obstacle_observation(
            frozen_points,
            robot_pose,
            floor_index=floor_index,
            mapper=self.mapper,
            opened_door_exclusion=opened_door_exclusion,
        )
        radial = nearest_radial_obstacle_distance(
            frozen_points,
            robot_pose,
            floor_index=floor_index,
            mapper=self.mapper,
        )
        return CloudMapResult(frozen_points, forward, radial)

    def update_images(self, bundle, camera_to_world, *, now: float) -> ImageDetectionResult:
        candidates = self.detector.detect_messages(
            bundle.rgb_msg,
            bundle.depth_msg,
            bundle.camera_info_msg,
            camera_to_world,
            require_world_transform=True,
        )
        if candidates:
            self.tracker.update(candidates, now=now)
        observations = ()
        if self.rgbd_door_detector is not None:
            observations = tuple(
                self.rgbd_door_detector.detect_messages(
                    bundle.rgb_msg,
                    bundle.depth_msg,
                    bundle.camera_info_msg,
                    camera_to_world,
                )
            )
        return ImageDetectionResult(observations)


def nearest_radial_obstacle_distance(
    points,
    pose: tuple[float, float, float],
    *,
    floor_index: int,
    mapper: MultiFloorGridMap,
    max_distance: float = 1.5,
) -> float | None:
    floor_base_z = floor_index * mapper.floor_height
    best = math.inf
    for px, py, pz in points:
        z_rel = pz - floor_base_z
        if not mapper.obstacle_min_z <= z_rel <= mapper.obstacle_max_z:
            continue
        distance = math.hypot(px - pose[0], py - pose[1])
        if 0.05 <= distance <= max_distance:
            best = min(best, distance)
    return float(best) if math.isfinite(best) else None


def nearest_forward_obstacle_observation(
    points,
    pose: tuple[float, float, float],
    *,
    floor_index: int,
    mapper: MultiFloorGridMap,
    max_distance: float = 1.5,
    opened_door_exclusion: tuple[float, float, float, float, float] | None = None,
) -> ForwardObstacleObservation | None:
    cos_yaw = math.cos(pose[2])
    sin_yaw = math.sin(pose[2])
    floor_base_z = floor_index * mapper.floor_height
    best = math.inf
    result = None
    for px, py, pz in points:
        z_rel = pz - floor_base_z
        if not mapper.obstacle_min_z <= z_rel <= mapper.obstacle_max_z:
            continue
        if opened_door_exclusion is not None:
            door_x, door_y, door_yaw, door_half_width, door_half_depth = opened_door_exclusion
            door_dx = px - door_x
            door_dy = py - door_y
            along_normal = door_dx * math.cos(door_yaw) + door_dy * math.sin(door_yaw)
            along_tangent = -door_dx * math.sin(door_yaw) + door_dy * math.cos(door_yaw)
            if abs(along_normal) <= door_half_depth and abs(along_tangent) <= door_half_width:
                continue
        dx = px - pose[0]
        dy = py - pose[1]
        forward = dx * cos_yaw + dy * sin_yaw
        lateral = abs(-dx * sin_yaw + dy * cos_yaw)
        distance = math.hypot(dx, dy)
        if forward < 0.0 or lateral > 0.25 or distance > max_distance:
            continue
        if distance < best:
            best = distance
            result = ForwardObstacleObservation(
                float(distance),
                (float(px), float(py), float(pz)),
                float(forward),
                float(lateral),
                float(z_rel),
            )
    return result


def nearest_forward_obstacle_distance(*args, **kwargs) -> float | None:
    observation = nearest_forward_obstacle_observation(*args, **kwargs)
    return observation.distance if observation is not None else None

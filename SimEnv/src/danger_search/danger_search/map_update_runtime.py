"""正式地图与检测组件的单线程更新边界。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from danger_search.detection_tracker import CONFIRMED, DetectionTracker
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
    danger_obstacle_sync: "DangerObstacleSyncResult" | None = None
    matched_cloud_stamp: float | None = None
    candidate_diagnostics: tuple[dict, ...] = ()
    candidate_count: int = 0


@dataclass(frozen=True)
class DangerObstacleSyncResult:
    """确认或高质量预保护危险源写入语义 OCC 后的最小审计摘要。"""

    selected_track_ids: tuple[int, ...] = ()
    protective_track_ids: tuple[int, ...] = ()
    changed_owner_ids: tuple[str, ...] = ()
    changed_cell_count: int = 0
    overlap_owner_ids: tuple[str, ...] = ()
    translated_track_count: int = 0
    instantiated_floor_indices: tuple[int, ...] = ()


@dataclass(frozen=True)
class RegisteredCloudFrame:
    """供地标匹配和 RGB-D 候选距离修正使用的同坐标系有限历史。"""

    stamp: float
    floor: int
    localization_epoch: int
    points: tuple[tuple[float, float, float], ...]


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
        self._last_candidate_diagnostic_time = -math.inf
        self.recent_cloud_frames: deque[RegisteredCloudFrame] = deque(maxlen=6)
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

    def record_registered_cloud(
        self,
        *,
        stamp: float,
        floor: int,
        localization_epoch: int,
        points,
    ) -> None:
        """保存最近少量合法配准帧，定位 epoch 改变后旧帧不会参与匹配。"""
        value = float(stamp)
        if not math.isfinite(value) or value <= 0.0:
            return
        frozen = tuple(
            (float(point[0]), float(point[1]), float(point[2])) for point in points
        )
        if not frozen:
            return
        recent = getattr(self, "recent_cloud_frames", None)
        if recent is None:
            # 保留 __new__ 构造的纯边界测试兼容性；生产构造仍在 __init__ 显式创建。
            recent = deque(maxlen=6)
            self.recent_cloud_frames = recent
        recent.append(
            RegisteredCloudFrame(
                stamp=value,
                floor=int(floor),
                localization_epoch=int(localization_epoch),
                points=frozen,
            )
        )

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

    def update_images(
        self,
        bundle,
        camera_to_world,
        *,
        now: float,
        floor_index: int | None = None,
        robot_xy: tuple[float, float] | None = None,
        image_stamp: float | None = None,
        localization_epoch: int | None = None,
    ) -> ImageDetectionResult:
        recent_cloud = None
        matched_frame = None
        if image_stamp is not None and math.isfinite(image_stamp) and localization_epoch is not None:
            eligible = [
                frame for frame in self.recent_cloud_frames
                if frame.floor == floor_index
                and frame.localization_epoch == localization_epoch
                and math.isfinite(frame.stamp)
                and abs(frame.stamp - image_stamp) <= self.detector.lidar_max_time_delta
            ]
            if eligible:
                matched_frame = min(eligible, key=lambda frame: abs(frame.stamp - image_stamp))
                recent_cloud = matched_frame.points

        diagnostics = [] if now - self._last_candidate_diagnostic_time >= 2.0 else None
        candidates = self.detector.detect_messages(
            bundle.rgb_msg,
            bundle.depth_msg,
            bundle.camera_info_msg,
            camera_to_world,
            require_world_transform=True,
            cloud_points=recent_cloud,
            diagnostics=diagnostics,
        )
        if diagnostics:
            self._last_candidate_diagnostic_time = now
        if candidates:
            self.tracker.update(candidates, now=now)
        danger_obstacle_sync = None
        if floor_index is not None:
            # RGB-D 可能先于首帧点云确认目标；先创建当前层承载语义 OCC，
            # 但不为尚未进入的其他层凭空建图。
            self.mapper.floor(int(floor_index))
            danger_obstacle_sync = self.synchronize_confirmed_danger_obstacles(
                now=now,
                robot_floor=int(floor_index),
                robot_xy=robot_xy,
            )
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
        return ImageDetectionResult(
            observations, danger_obstacle_sync,
            matched_cloud_stamp=matched_frame.stamp if matched_frame is not None else None,
            candidate_diagnostics=tuple(diagnostics or ()),
            candidate_count=len(candidates),
        )

    def synchronize_confirmed_danger_obstacles(
        self,
        *,
        now: float | None = None,
        robot_floor: int | None = None,
        robot_xy: tuple[float, float] | None = None,
    ) -> DangerObstacleSyncResult:
        """将 NMS 后确认红球及高质量预保护轨迹同步为语义 OCC。"""
        obstacle_tracks = getattr(self.tracker, "obstacle_tracks", None)
        if callable(obstacle_tracks):
            selected_tracks = obstacle_tracks(now=now)
        else:
            # 保留纯边界夹具和旧实现的兼容性；生产 Tracker 一定提供预保护查询。
            selected_tracks = self.tracker.confirmed_tracks(now=now)
        selected_ids = tuple(int(track.track_id) for track in selected_tracks)
        protective_ids = tuple(
            int(track.track_id)
            for track in selected_tracks
            if getattr(track, "status", None) != CONFIRMED
        )
        floor_indices = self.mapper.instantiated_floor_indices
        if not floor_indices:
            return DangerObstacleSyncResult(
                selected_track_ids=selected_ids,
                protective_track_ids=protective_ids,
            )

        target_radius = float(self.detector.target_radius)
        obstacle_radius = target_radius + float(
            self.mapper.obstacle_inflation_radius
        )
        if (
            not math.isfinite(target_radius)
            or not math.isfinite(obstacle_radius)
            or target_radius <= 0.0
            or obstacle_radius <= 0.0
        ):
            raise ValueError("危险源语义 OCC 半径无效")

        desired_by_floor: dict[int, dict[str, tuple[float, float]]] = {
            int(floor): {} for floor in floor_indices
        }
        for track in selected_tracks:
            position = track.mean_position
            floor = self.mapper.estimate_floor_from_z(float(position[2]))
            if floor not in desired_by_floor:
                continue
            desired_by_floor[floor][f"danger:{int(track.track_id)}"] = (
                float(position[0]),
                float(position[1]),
            )

        changed_owner_ids: set[str] = set()
        changed_cells: set[tuple[int, int, int]] = set()
        overlap_owner_ids: set[str] = set()
        valid_robot_xy = _valid_robot_xy(robot_xy)
        for floor in floor_indices:
            grid = self.mapper.floor(int(floor))
            desired = desired_by_floor[int(floor)]
            robot_cell = (
                grid.world_to_grid(*valid_robot_xy)
                if valid_robot_xy is not None and int(floor) == robot_floor
                else None
            )
            owners = grid.semantic_occupied_owner_ids(prefix="danger:")
            for owner in sorted(owners | set(desired)):
                if owner in desired:
                    x, y = desired[owner]
                    affected = grid.replace_semantic_occupied_radius(
                        x,
                        y,
                        obstacle_radius,
                        owner=owner,
                    )
                else:
                    affected = grid.clear_semantic_occupied_owner(owner)
                if not affected:
                    continue
                changed_owner_ids.add(owner)
                changed_cells.update(
                    (int(floor), int(cell[0]), int(cell[1])) for cell in affected
                )
            if robot_cell is not None:
                # 即使 OCC 覆盖未变化，机器人也可能沿旧路径刚进入该格；每帧
                # 都必须检查当前站位，不能只在新增覆盖时才触发 fail-closed。
                overlap_owner_ids.update(
                    owner
                    for owner in grid.semantic_occupied_owners.get(
                        robot_cell,
                        (),
                    )
                    if owner in desired
                )

        return DangerObstacleSyncResult(
            selected_track_ids=selected_ids,
            protective_track_ids=protective_ids,
            changed_owner_ids=tuple(sorted(changed_owner_ids)),
            changed_cell_count=len(changed_cells),
            overlap_owner_ids=tuple(sorted(overlap_owner_ids)),
            instantiated_floor_indices=tuple(int(floor) for floor in floor_indices),
        )

    def apply_planar_correction(
        self,
        dx: float,
        dy: float,
        *,
        correction_yaw: float = 0.0,
        now: float | None = None,
        robot_floor: int | None = None,
        robot_xy: tuple[float, float] | None = None,
    ) -> DangerObstacleSyncResult:
        """在定位节点回执后同步平移轨迹，并重建全部已实例化楼层的语义 OCC。"""
        map_runtime = getattr(self.mapper, "transform_planar", None)
        if callable(map_runtime):
            map_runtime(float(dx), float(dy), float(correction_yaw))
        if abs(float(correction_yaw)) > 1e-12:
            translated = self.tracker.transform_planar(dx, dy, correction_yaw)
        else:
            translated = self.tracker.translate_planar(dx, dy)
        synchronized = self.synchronize_confirmed_danger_obstacles(
            now=now,
            robot_floor=robot_floor,
            robot_xy=robot_xy,
        )
        return DangerObstacleSyncResult(
            selected_track_ids=synchronized.selected_track_ids,
            protective_track_ids=synchronized.protective_track_ids,
            changed_owner_ids=synchronized.changed_owner_ids,
            changed_cell_count=synchronized.changed_cell_count,
            overlap_owner_ids=synchronized.overlap_owner_ids,
            translated_track_count=translated,
            instantiated_floor_indices=synchronized.instantiated_floor_indices,
        )


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


def _valid_robot_xy(
    robot_xy: tuple[float, float] | None,
) -> tuple[float, float] | None:
    if robot_xy is None:
        return None
    try:
        if len(robot_xy) != 2:
            return None
        x = float(robot_xy[0])
        y = float(robot_xy[1])
    except (IndexError, TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return (x, y)


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
        if forward < 0.20 or lateral > 0.22 or distance > max_distance:
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

"""通过楼层守卫授权后的感知提交边界。"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from danger_search.perception_runtime import (
    CloudPreparation,
    ImagePreparation,
    INPUT_MATCHED,
)
from danger_search.pose_provider import PoseProvider


@dataclass(frozen=True)
class CloudCommitResult:
    stamp: float
    generation: int
    map_result: object


@dataclass(frozen=True)
class ImageCommitResult:
    stamp: float
    generation: int
    bundle: object
    camera_to_world: np.ndarray | None
    detection_result: object


class PerceptionMappingRuntime:
    """组合感知游标与地图所有者，不拥有任务或楼层过渡策略。"""

    def __init__(
        self,
        *,
        perception,
        map_runtime,
        floor_coordinates,
        pose_provider,
    ) -> None:
        self.perception = perception
        self.map_runtime = map_runtime
        self.floor_coordinates = floor_coordinates
        self.pose_provider = pose_provider

    def discard_cloud(self, preparation: CloudPreparation) -> None:
        cloud_input = preparation.cloud_input
        if cloud_input is None:
            return
        self.perception.consume_cloud(cloud_input.generation)
        self.map_runtime.last_cloud_stamp = cloud_input.stamp

    def cache_cloud(
        self,
        preparation: CloudPreparation,
        *,
        floor_index: int,
        cloud_max_points: int,
        cloud_stride: int,
    ) -> None:
        """更新现有点云缓存但不写地图，供显式换层期间的目标门观测使用。"""
        if preparation.status != INPUT_MATCHED or preparation.cloud_input is None:
            raise ValueError("只有已匹配点云可更新感知缓存")
        cloud_input = preparation.cloud_input
        points = self.perception.read_cloud_xyz(
            cloud_input,
            max_points=int(cloud_max_points),
            stride=int(cloud_stride),
        )
        points = self.floor_coordinates.logical_points(points, int(floor_index))
        self.perception.consume_cloud(cloud_input.generation)
        self.map_runtime.last_cloud_stamp = cloud_input.stamp
        self.map_runtime.latest_cloud_points = list(points)
        record = getattr(self.map_runtime, "record_registered_cloud", None)
        if callable(record):
            record(
                stamp=float(cloud_input.stamp),
                floor=int(floor_index),
                localization_epoch=0,
                points=points,
            )

    def commit_cloud(
        self,
        preparation: CloudPreparation,
        *,
        floor_index: int,
        map_epoch: int,
        localization_epoch: int,
        cloud_max_points: int,
        cloud_stride: int,
        opened_door_exclusion,
    ) -> CloudCommitResult:
        if preparation.status != INPUT_MATCHED or preparation.cloud_input is None:
            raise ValueError("只有已匹配点云可提交地图")
        cloud_input = preparation.cloud_input
        sample = preparation.pose_sample
        points = self.perception.read_cloud_xyz(
            cloud_input,
            max_points=int(cloud_max_points),
            stride=int(cloud_stride),
        )
        points = self.floor_coordinates.logical_points(points, int(floor_index))
        self.perception.consume_cloud(cloud_input.generation)
        self.map_runtime.last_cloud_stamp = cloud_input.stamp
        result = self.map_runtime.update_cloud(
            floor_index=int(floor_index),
            cloud_generation=int(cloud_input.generation),
            map_epoch=int(map_epoch),
            localization_epoch=int(localization_epoch),
            robot_xyz=self.floor_coordinates.logical_xyz(
                sample.xyz,
                int(floor_index),
            ),
            robot_pose=sample.xy_yaw,
            points=points,
            opened_door_exclusion=opened_door_exclusion,
        )
        self.map_runtime.latest_cloud_points = list(result.points)
        record = getattr(self.map_runtime, "record_registered_cloud", None)
        if callable(record):
            record(
                stamp=float(cloud_input.stamp),
                floor=int(floor_index),
                localization_epoch=int(localization_epoch),
                points=result.points,
            )
        self.map_runtime.nearest_forward_obstacle = result.forward_obstacle
        self.map_runtime.nearest_radial_obstacle_distance = (
            result.radial_obstacle_distance
        )
        return CloudCommitResult(
            stamp=float(cloud_input.stamp),
            generation=int(cloud_input.generation),
            map_result=result,
        )

    def discard_image(self, preparation: ImagePreparation) -> None:
        image_input = preparation.image_input
        if image_input is None:
            return
        self.perception.consume_image(image_input.generation)
        valid_stamps = [stamp for stamp in image_input.stamps if stamp is not None]
        self.map_runtime.last_image_stamp = (
            float(valid_stamps[0]) if valid_stamps else None
        )

    def commit_image(
        self,
        preparation: ImagePreparation,
        *,
        floor_index: int,
        camera_mount_xyz,
        camera_mount_rpy,
        depth_frame_is_optical: bool,
        now: float,
    ) -> ImageCommitResult:
        if preparation.status != INPUT_MATCHED or preparation.image_input is None:
            raise ValueError("只有已匹配 RGB-D 帧可提交检测")
        image_input = preparation.image_input
        image_stamp = float(image_input.stamps[0])
        self.perception.consume_image(image_input.generation)
        self.map_runtime.last_image_stamp = image_stamp
        sample = preparation.pose_lookup.sample
        if sample is None:
            raise ValueError("已匹配 RGB-D 帧缺少位姿样本")
        if self.pose_provider is not None:
            camera_to_world = self.pose_provider.sensor_to_world_matrix(
                sample,
                base_to_sensor_xyz=camera_mount_xyz,
                base_to_sensor_rpy=camera_mount_rpy,
            )
        else:
            camera_to_world = camera_to_world_matrix(
                sample,
                camera_mount_xyz,
                camera_mount_rpy,
            )
        if camera_to_world is not None and depth_frame_is_optical:
            camera_to_world = camera_to_world @ ros_link_from_optical_matrix()
        if camera_to_world is not None:
            camera_to_world = np.array(camera_to_world, dtype=float, copy=True)
            camera_to_world[2, 3] += self.floor_coordinates.z_offset(
                int(floor_index)
            )
        result = self.map_runtime.update_images(
            image_input.bundle,
            camera_to_world,
            now=float(now),
            floor_index=int(floor_index),
            robot_xy=(float(sample.xy_yaw[0]), float(sample.xy_yaw[1])),
            image_stamp=image_stamp,
            localization_epoch=getattr(preparation.pose_lookup, "localization_epoch", None),
        )
        return ImageCommitResult(
            stamp=image_stamp,
            generation=int(image_input.generation),
            bundle=image_input.bundle,
            camera_to_world=camera_to_world,
            detection_result=result,
        )


def camera_to_world_matrix(sample, xyz, rpy) -> np.ndarray:
    """轻量测试对象未安装 PoseProvider 时复用同一固定外参计算。"""
    provider = PoseProvider.__new__(PoseProvider)
    return provider.sensor_to_world_matrix(
        sample,
        base_to_sensor_xyz=xyz,
        base_to_sensor_rpy=rpy,
    )


def ros_link_from_optical_matrix() -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )
    return matrix

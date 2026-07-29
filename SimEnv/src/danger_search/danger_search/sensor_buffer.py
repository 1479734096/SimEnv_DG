"""ROS 传感器的非阻塞接入与最近帧缓存。

ROS 回调只保存最新消息，不在回调线程做点云遍历或图像推理；10 Hz 主状态机
通过 ``snapshot`` 一次取得自洽引用，再根据消息时间戳到 ``PoseProvider`` 查询
位姿。RGB、深度和相机内参先由 ``ApproximateTimeSynchronizer`` 对齐，点云则
独立缓存，因为其频率和后续消费节奏不同。
"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Iterable

import numpy as np

from danger_search.interface_policy import (
    require_allowed_input_topic,
    require_internal_input_topic,
)

try:
    import message_filters
    import rospy
    import sensor_msgs.point_cloud2 as pc2
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
except ImportError:  # pragma: no cover - 离线语法检查环境可能没有 ROS
    message_filters = None
    rospy = None
    pc2 = None
    CameraInfo = Image = PointCloud2 = None


@dataclass
class ImageBundle:
    """时间近似同步的一组 RGB、深度和相机内参 ROS 消息。"""
    rgb_msg: object
    depth_msg: object
    camera_info_msg: object
    generation: int


@dataclass
class SensorSnapshot:
    """主状态机读取时刻的最近点云与最近 RGB-D 组合。"""
    cloud_msg: object | None
    image_bundle: ImageBundle | None
    cloud_generation: int
    image_generation: int


class SensorBuffer:
    """把 ROS 回调数据缓存起来，使业务计算与回调线程解耦。

    锁只保护消息引用的替换与复制，不保护耗时解析过程。调用方拿到的 ROS 消息
    在 Python 中仍持有引用，因此释放锁后读取是安全的，也不会阻塞新帧到达。
    """

    def __init__(
        self,
        *,
        cloud_topic: str,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        sync_slop: float = 0.08,
        sync_queue_size: int = 10,
        cloud_frame: str | None = None,
        rgbd_frame: str = "real_sense_optical_frame",
    ):
        normalized_cloud_topic = str(cloud_topic).strip()
        if normalized_cloud_topic.startswith(("/danger_search/", "/nav/")):
            self.cloud_topic = require_internal_input_topic(
                normalized_cloud_topic,
                purpose="参赛方配准建图点云输入",
            )
        else:
            self.cloud_topic = require_allowed_input_topic(
                normalized_cloud_topic,
                purpose="公开建图点云输入",
            )
        self.rgb_topic = require_allowed_input_topic(
            rgb_topic, purpose="RGB-D 彩色图输入"
        )
        self.depth_topic = require_allowed_input_topic(
            depth_topic, purpose="RGB-D 深度图输入"
        )
        self.camera_info_topic = require_allowed_input_topic(
            camera_info_topic, purpose="RGB-D 标定输入"
        )
        self.sync_slop = float(sync_slop)
        self.sync_queue_size = int(sync_queue_size)
        default_cloud_frame = (
            "world"
            if normalized_cloud_topic.startswith("/danger_search/")
            else "laser_livox"
        )
        self.cloud_frame = str(cloud_frame or default_cloud_frame).strip()
        self.rgbd_frame = str(rgbd_frame).strip()
        if not np.isfinite(self.sync_slop) or self.sync_slop < 0.0:
            raise ValueError("RGB-D 同步容差必须为非负有限数")
        if self.sync_queue_size <= 0:
            raise ValueError("RGB-D 同步队列长度必须为正数")
        if not self.cloud_frame or not self.rgbd_frame:
            raise ValueError("传感器 frame_id 契约不能为空")
        self._lock = Lock()
        self._cloud_msg = None
        self._image_bundle: ImageBundle | None = None
        self._cloud_generation = 0
        self._image_generation = 0
        self.rejected_cloud_count = 0
        self.rejected_image_bundle_count = 0
        self._subscribers = []

    def start(self) -> None:
        """建立点云订阅和 RGB-D 三路近似同步器。"""
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法启动 SensorBuffer")

        self._subscribers.append(rospy.Subscriber(self.cloud_topic, PointCloud2, self._on_cloud, queue_size=2))

        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
        )
        synchronizer.registerCallback(self._on_image_bundle)
        self._subscribers.extend([rgb_sub, depth_sub, info_sub, synchronizer])

    def snapshot(self) -> SensorSnapshot:
        """原子复制两类最新消息引用，不消费或清空缓存。"""
        with self._lock:
            return SensorSnapshot(
                cloud_msg=self._cloud_msg,
                image_bundle=self._image_bundle,
                cloud_generation=self._cloud_generation,
                image_generation=self._image_generation,
            )

    def read_cloud_xyz(self, cloud_msg, *, max_points: int = 25000, stride: int = 1) -> list[tuple[float, float, float]]:
        """按固定步长解码 ``PointCloud2``，并以点数上限约束单帧开销。

        返回值只保留有限的 XYZ 数值；字段选择和 ``skip_nans`` 由 ROS 官方读取器
        处理。调用方随后负责时间匹配和坐标系语义，本函数不擅自做变换。
        """
        point_limit = int(max_points)
        if point_limit <= 0:
            raise ValueError("点云读取上限必须为正数")
        if pc2 is None or cloud_msg is None:
            return []
        points: list[tuple[float, float, float]] = []
        step = max(1, int(stride))
        for index, point in enumerate(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % step != 0:
                continue
            xyz = (float(point[0]), float(point[1]), float(point[2]))
            if not np.all(np.isfinite(np.asarray(xyz, dtype=float))):
                continue
            points.append(xyz)
            if len(points) >= point_limit:
                break
        return points

    def _on_cloud(self, msg) -> None:
        """点云回调：以最新帧覆盖旧帧，防止处理落后时形成无界积压。"""
        with self._lock:
            if not message_contract_valid(msg, expected_frame=self.cloud_frame):
                self.rejected_cloud_count += 1
                return
            self._cloud_generation += 1
            self._cloud_msg = msg

    def _on_image_bundle(self, rgb_msg, depth_msg, camera_info_msg) -> None:
        """同步回调：把三路同一观测窗口内的消息作为不可拆组合保存。"""
        with self._lock:
            messages = (rgb_msg, depth_msg, camera_info_msg)
            stamps = [message_stamp_seconds(message) for message in messages]
            if (
                not all(
                    message_contract_valid(
                        message,
                        expected_frame=self.rgbd_frame,
                    )
                    for message in messages
                )
                or max(stamps) - min(stamps) > self.sync_slop
            ):
                self.rejected_image_bundle_count += 1
                return
            self._image_generation += 1
            self._image_bundle = ImageBundle(
                rgb_msg=rgb_msg,
                depth_msg=depth_msg,
                camera_info_msg=camera_info_msg,
                generation=self._image_generation,
            )


def odom_pose_xy_yaw(odom_msg) -> tuple[float, float, float] | None:
    if odom_msg is None:
        return None
    pose = odom_msg.pose.pose
    yaw = quaternion_to_yaw(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    return (float(pose.position.x), float(pose.position.y), yaw)


def odom_xyz(odom_msg) -> tuple[float, float, float] | None:
    if odom_msg is None:
        return None
    position = odom_msg.pose.pose.position
    return (float(position.x), float(position.y), float(position.z))


def odom_planar_speed(odom_msg) -> float | None:
    if odom_msg is None:
        return None
    linear = odom_msg.twist.twist.linear
    return float(np.hypot(float(linear.x), float(linear.y)))


def odom_angular_speed(odom_msg) -> float | None:
    if odom_msg is None:
        return None
    angular = odom_msg.twist.twist.angular
    return float(
        np.linalg.norm([float(angular.x), float(angular.y), float(angular.z)])
    )


def odom_rotation_matrix(odom_msg) -> np.ndarray | None:
    if odom_msg is None:
        return None
    orientation = odom_msg.pose.pose.orientation
    quaternion = np.asarray(
        [orientation.x, orientation.y, orientation.z, orientation.w],
        dtype=float,
    )
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        return np.eye(3)
    x, y, z, w = quaternion / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def message_stamp_seconds(message) -> float | None:
    """返回严格有效的 ROS 时间；零、负数、缺失和非有限值均拒绝。"""
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    to_sec = getattr(stamp, "to_sec", None)
    if not callable(to_sec):
        return None
    try:
        value = float(to_sec())
    except (TypeError, ValueError, OverflowError):
        return None
    return value if np.isfinite(value) and value > 0.0 else None


def message_frame_id(message) -> str | None:
    """返回非空 frame_id，缺失或空白时返回 ``None``。"""
    value = str(getattr(getattr(message, "header", None), "frame_id", "")).strip()
    return value or None


def message_contract_valid(message, *, expected_frame: str) -> bool:
    """检查生产传感器消息的时间与坐标系契约。"""
    return (
        message_stamp_seconds(message) is not None
        and message_frame_id(message) == str(expected_frame).strip()
    )


# 兼容既有只读测试和外部工具；生产代码使用无下划线的严格接口。
_message_stamp_seconds = message_stamp_seconds

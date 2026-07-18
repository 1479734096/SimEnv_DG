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


@dataclass
class SensorSnapshot:
    """主状态机读取时刻的最近点云与最近 RGB-D 组合。"""
    cloud_msg: object | None
    image_bundle: ImageBundle | None


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
    ):
        self.cloud_topic = cloud_topic
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.sync_slop = float(sync_slop)
        self.sync_queue_size = int(sync_queue_size)
        self._lock = Lock()
        self._cloud_msg = None
        self._image_bundle: ImageBundle | None = None
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
            )

    def read_cloud_xyz(self, cloud_msg, *, max_points: int = 25000, stride: int = 1) -> list[tuple[float, float, float]]:
        """按固定步长解码 ``PointCloud2``，并以点数上限约束单帧开销。

        返回值只保留有限的 XYZ 数值；字段选择和 ``skip_nans`` 由 ROS 官方读取器
        处理。调用方随后负责时间匹配和坐标系语义，本函数不擅自做变换。
        """
        if pc2 is None or cloud_msg is None:
            return []
        points: list[tuple[float, float, float]] = []
        step = max(1, int(stride))
        for index, point in enumerate(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % step != 0:
                continue
            points.append((float(point[0]), float(point[1]), float(point[2])))
            if len(points) >= max_points:
                break
        return points

    def _on_cloud(self, msg) -> None:
        """点云回调：以最新帧覆盖旧帧，防止处理落后时形成无界积压。"""
        with self._lock:
            self._cloud_msg = msg

    def _on_image_bundle(self, rgb_msg, depth_msg, camera_info_msg) -> None:
        """同步回调：把三路同一观测窗口内的消息作为不可拆组合保存。"""
        with self._lock:
            self._image_bundle = ImageBundle(rgb_msg=rgb_msg, depth_msg=depth_msg, camera_info_msg=camera_info_msg)


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


def _message_stamp_seconds(message) -> float | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    to_sec = getattr(stamp, "to_sec", None)
    if not callable(to_sec):
        return None
    value = float(to_sec())
    return value if np.isfinite(value) else None

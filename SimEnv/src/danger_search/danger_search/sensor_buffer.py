"""ROS 传感器非阻塞缓存。"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Iterable

import numpy as np

try:
    import message_filters
    import rospy
    import sensor_msgs.point_cloud2 as pc2
    import tf
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
except ImportError:  # pragma: no cover - 离线语法检查环境可能没有 ROS
    message_filters = None
    rospy = None
    pc2 = None
    tf = None
    Odometry = CameraInfo = Image = PointCloud2 = None


@dataclass
class ImageBundle:
    rgb_msg: object
    depth_msg: object
    camera_info_msg: object


@dataclass
class SensorSnapshot:
    odom_msg: object | None
    cloud_msg: object | None
    image_bundle: ImageBundle | None


class SensorBuffer:
    """把 ROS 回调数据缓存起来，主状态机只读取快照。"""

    def __init__(
        self,
        *,
        odom_topic: str,
        cloud_topic: str,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        sync_slop: float = 0.08,
        sync_queue_size: int = 10,
        target_frame: str = "odom",
    ):
        self.odom_topic = odom_topic
        self.cloud_topic = cloud_topic
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.sync_slop = float(sync_slop)
        self.sync_queue_size = int(sync_queue_size)
        self.target_frame = target_frame
        self._lock = Lock()
        self._odom_msg = None
        self._cloud_msg = None
        self._image_bundle: ImageBundle | None = None
        self._tf_listener = None
        self._subscribers = []

    def start(self) -> None:
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法启动 SensorBuffer")

        self._tf_listener = tf.TransformListener()
        self._subscribers.append(rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=5))
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
        with self._lock:
            return SensorSnapshot(
                odom_msg=self._odom_msg,
                cloud_msg=self._cloud_msg,
                image_bundle=self._image_bundle,
            )

    def camera_to_target_matrix(self, camera_frame: str, stamp) -> np.ndarray | None:
        if self._tf_listener is None:
            return None
        try:
            trans, quat = self._tf_listener.lookupTransform(self.target_frame, camera_frame, stamp)
        except Exception:
            try:
                trans, quat = self._tf_listener.lookupTransform(self.target_frame, camera_frame, rospy.Time(0))
            except Exception:
                return None
        matrix = tf.transformations.quaternion_matrix(quat)
        matrix[:3, 3] = np.array(trans, dtype=float)
        return matrix

    def read_cloud_xyz(self, cloud_msg, *, max_points: int = 25000, stride: int = 1) -> list[tuple[float, float, float]]:
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

    def _on_odom(self, msg) -> None:
        with self._lock:
            self._odom_msg = msg

    def _on_cloud(self, msg) -> None:
        with self._lock:
            self._cloud_msg = msg

    def _on_image_bundle(self, rgb_msg, depth_msg, camera_info_msg) -> None:
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


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))

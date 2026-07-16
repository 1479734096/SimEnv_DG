"""ROS 传感器非阻塞缓存。"""
from __future__ import annotations

from collections import deque
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
        self._odom_history = None
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

    def enable_cloud_odom_matching(self, *, max_samples: int = 256) -> None:
        """仅在生产支撑门开启时保留短 odom 历史，供点云时间匹配。"""
        with self._lock:
            history = deque(maxlen=max(2, int(max_samples)))
            if self._odom_msg is not None:
                history.append(self._odom_msg)
            self._odom_history = history

    def ground_support_snapshot(self) -> SensorSnapshot:
        """返回最新点云与时间戳最接近的 odom；普通快照语义保持不变。"""
        with self._lock:
            cloud_msg = self._cloud_msg
            odom_msg = self._odom_msg
            cloud_stamp = _message_stamp_seconds(cloud_msg)
            if cloud_stamp is not None and self._odom_history:
                stamped = [
                    (abs(stamp - cloud_stamp), message)
                    for message in self._odom_history
                    for stamp in [_message_stamp_seconds(message)]
                    if stamp is not None
                ]
                if stamped:
                    odom_msg = min(stamped, key=lambda item: item[0])[1]
            return SensorSnapshot(
                odom_msg=odom_msg,
                cloud_msg=cloud_msg,
                image_bundle=self._image_bundle,
            )

    def camera_to_target_matrix(self, camera_frame: str, stamp) -> np.ndarray | None:
        return self.frame_transform_matrix(
            self.target_frame,
            camera_frame,
            stamp,
            allow_latest=True,
        )

    def frame_transform_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp,
        *,
        allow_latest: bool = False,
    ) -> np.ndarray | None:
        """读取指定坐标变换；生产安全门只在显式启用后调用。"""
        if self._tf_listener is None:
            return None
        try:
            trans, quat = self._tf_listener.lookupTransform(target_frame, source_frame, stamp)
        except Exception:
            if not allow_latest or rospy is None:
                return None
            try:
                trans, quat = self._tf_listener.lookupTransform(
                    target_frame,
                    source_frame,
                    rospy.Time(0),
                )
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
            if self._odom_history is not None:
                self._odom_history.append(msg)

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

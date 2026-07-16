"""参赛方统一位姿入口。

生产模式仅订阅参赛方 ``scan_localization_node`` 发布的内部里程计；该节点
唯一外部输入为初始接口文档授权的 ``/scan``。这里不订阅 TF 或 Gazebo 真值。
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Lock
from typing import Iterable

import numpy as np

try:
    import rospy
    from nav_msgs.msg import Odometry
except ImportError:  # pragma: no cover - 离线测试环境
    rospy = None
    Odometry = None


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    linear_xyz: tuple[float, float, float]
    angular_xyz: tuple[float, float, float]

    @property
    def yaw(self) -> float:
        x, y, z, w = self.quaternion_xyzw
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    @property
    def xy_yaw(self) -> tuple[float, float, float]:
        return (self.xyz[0], self.xyz[1], self.yaw)

    @property
    def planar_speed(self) -> float:
        return float(np.hypot(self.linear_xyz[0], self.linear_xyz[1]))

    @property
    def angular_speed(self) -> float:
        return float(np.linalg.norm(self.angular_xyz))

    @property
    def rotation_matrix(self) -> np.ndarray:
        x, y, z, w = np.asarray(self.quaternion_xyzw, dtype=float)
        norm = float(np.linalg.norm([x, y, z, w]))
        if norm <= 1e-12:
            return np.eye(3)
        x, y, z, w = (x / norm, y / norm, z / norm, w / norm)
        return np.asarray(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )


class PoseProvider:
    """保存内部定位历史，并提供时间匹配的位姿快照。"""

    def __init__(self, odom_topic: str, *, max_samples: int = 256):
        self.odom_topic = str(odom_topic)
        self._lock = Lock()
        self._samples: deque[PoseSample] = deque(maxlen=max(2, int(max_samples)))
        self._subscriber = None

    def start(self) -> None:
        if rospy is None or Odometry is None:
            raise RuntimeError("当前环境没有 rospy，无法启动 PoseProvider")
        self._subscriber = rospy.Subscriber(self.odom_topic, Odometry, self._on_odometry, queue_size=16)

    def latest(self) -> PoseSample | None:
        with self._lock:
            return self._samples[-1] if self._samples else None

    def at(self, stamp: float | None, *, max_delta: float = 0.08) -> PoseSample | None:
        if stamp is None:
            return self.latest()
        with self._lock:
            if not self._samples:
                return None
            nearest = min(self._samples, key=lambda sample: abs(sample.stamp - float(stamp)))
        if abs(nearest.stamp - float(stamp)) > float(max_delta):
            return None
        return nearest

    def sensor_to_world_matrix(
        self,
        sample: PoseSample,
        *,
        base_to_sensor_xyz: Iterable[float],
        base_to_sensor_rpy: Iterable[float],
    ) -> np.ndarray:
        translation = np.asarray(tuple(base_to_sensor_xyz), dtype=float)
        rpy = np.asarray(tuple(base_to_sensor_rpy), dtype=float)
        if translation.shape != (3,) or rpy.shape != (3,):
            raise ValueError("固定传感器外参必须各含 3 个值")
        roll, pitch, yaw = rpy
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        base_to_sensor_rotation = np.asarray(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=float,
        )
        matrix = np.eye(4)
        matrix[:3, :3] = sample.rotation_matrix @ base_to_sensor_rotation
        matrix[:3, 3] = np.asarray(sample.xyz, dtype=float) + sample.rotation_matrix @ translation
        return matrix

    def _on_odometry(self, message) -> None:
        stamp = _stamp_seconds(message)
        if stamp is None:
            return
        pose = message.pose.pose
        twist = message.twist.twist
        sample = PoseSample(
            stamp=stamp,
            xyz=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
            quaternion_xyzw=(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
            linear_xyz=(float(twist.linear.x), float(twist.linear.y), float(twist.linear.z)),
            angular_xyz=(float(twist.angular.x), float(twist.angular.y), float(twist.angular.z)),
        )
        with self._lock:
            self._samples.append(sample)


def _stamp_seconds(message) -> float | None:
    if message is None:
        return None
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    try:
        return float(stamp)
    except (TypeError, ValueError):
        return None

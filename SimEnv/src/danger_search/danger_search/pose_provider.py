"""参赛方统一位姿入口与传感器时间对齐层。

生产模式仅订阅参赛方 ``scan_localization_node`` 发布的内部里程计；正式
定位后端只能使用初始接口文档授权的 Livox、RealSense、IMU 和控制先验。
这里不订阅 TF 或 Gazebo 真值。

``scan_localization_node`` 发布内部 ``Odometry`` 后，本模块保存一个有界历史。
``MissionManager`` 读取最新样本推进状态机，点云与图像处理则按各自消息时间戳
调用 ``lookup``，从而避免用“最新位姿”无条件解释较旧的传感器帧。
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Lock
from typing import Iterable

import numpy as np

from danger_search.interface_policy import (
    require_internal_input_topic,
    require_internal_output_topic,
)

try:
    import rospy
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
except ImportError:  # pragma: no cover - 离线测试环境
    rospy = None
    Vector3Stamped = None
    Odometry = None


@dataclass(frozen=True)
class PoseSample:
    """一次不可变的内部里程计快照。

    线速度沿用 Odometry 消息语义，姿态以 ROS ``xyzw`` 四元数保存；常用的
    平面 yaw、速度模长和旋转矩阵由只读属性按需计算。
    """
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


POSE_MATCHED = "MATCHED"
POSE_PENDING = "PENDING"
POSE_EXPIRED = "EXPIRED"


@dataclass(frozen=True)
class PoseLookup:
    """某个传感器时间戳的内部定位查询结果。"""

    status: str
    sample: PoseSample | None
    delta: float | None


class PoseProvider:
    """订阅内部里程计，保存线程安全历史并提供时间匹配查询。

    ROS 回调线程只负责追加不可变 ``PoseSample``；任务定时器在锁内复制引用后
    执行计算，避免长时间占用回调锁。历史长度有上限，防止长任务持续增长内存。
    """

    def __init__(
        self,
        odom_topic: str,
        *,
        correction_request_topic: str = "/danger_search/localization/correction_request",
        correction_state_topic: str = "/danger_search/localization/correction_state",
        max_samples: int = 256,
    ):
        self.odom_topic = require_internal_input_topic(
            odom_topic,
            purpose="统一位姿入口",
        )
        self._lock = Lock()
        self._samples: deque[PoseSample] = deque(maxlen=max(2, int(max_samples)))
        self._subscriber = None
        self._correction_publisher = None
        self._correction_subscriber = None
        self._correction_request_topic = require_internal_output_topic(
            correction_request_topic,
            purpose="定位平移纠正请求",
        )
        self._correction_state_topic = require_internal_input_topic(
            correction_state_topic,
            purpose="定位平移纠正回执",
        )
        self._next_correction_sequence = 1
        self._correction_acks: dict[int, tuple[float, float, float]] = {}
        self._frame_identity: tuple[str, str] | None = None
        self._continuity_epoch = 1

    @property
    def continuity_epoch(self) -> int:
        with self._lock:
            return self._continuity_epoch

    def start(self) -> None:
        """建立内部里程计订阅；构造对象本身不会产生 ROS 副作用。"""
        if rospy is None or Odometry is None or Vector3Stamped is None:
            raise RuntimeError("当前环境没有 rospy，无法启动 PoseProvider")
        self._subscriber = rospy.Subscriber(self.odom_topic, Odometry, self._on_odometry, queue_size=16)
        self._correction_publisher = rospy.Publisher(
            self._correction_request_topic,
            Vector3Stamped,
            queue_size=1,
        )
        self._correction_subscriber = rospy.Subscriber(
            self._correction_state_topic,
            Vector3Stamped,
            self._on_correction_state,
            queue_size=4,
        )

    def request_planar_correction(
        self,
        correction_xy: tuple[float, float],
        *,
        maximum_correction: float,
        maximum_linear_speed: float,
        maximum_angular_speed: float,
    ) -> int | None:
        """机器人静止时发布一次有序纠正请求；不满足门控则拒绝。"""
        try:
            dx, dy = (float(correction_xy[0]), float(correction_xy[1]))
            maximum = float(maximum_correction)
            linear_limit = float(maximum_linear_speed)
            angular_limit = float(maximum_angular_speed)
        except (IndexError, TypeError, ValueError):
            return None
        if (
            not all(
                np.isfinite(value)
                for value in (dx, dy, maximum, linear_limit, angular_limit)
            )
            or maximum <= 0.0
            or linear_limit <= 0.0
            or angular_limit <= 0.0
            or float(np.hypot(dx, dy)) <= 0.0
            or float(np.hypot(dx, dy)) > maximum
        ):
            return None
        sample = self.latest()
        if (
            sample is None
            or sample.planar_speed > linear_limit
            or sample.angular_speed > angular_limit
            or self._correction_publisher is None
            or Vector3Stamped is None
            or rospy is None
        ):
            return None
        with self._lock:
            sequence = self._next_correction_sequence
            self._next_correction_sequence += 1
        message = Vector3Stamped()
        message.header.seq = int(sequence)
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.vector.x = dx
        message.vector.y = dy
        message.vector.z = 0.0
        self._correction_publisher.publish(message)
        return sequence

    def correction_ack(self, sequence: int) -> tuple[float, float, float] | None:
        with self._lock:
            return self._correction_acks.get(int(sequence))

    def latest(self) -> PoseSample | None:
        """返回最新定位快照；尚未收到里程计时返回 ``None``。"""
        with self._lock:
            return self._samples[-1] if self._samples else None

    def observed_period(self, *, default: float = 0.1, max_intervals: int = 32) -> float:
        """返回近期定位样本的稳健周期，用于判断定位新鲜度。"""
        fallback = float(default)
        if not np.isfinite(fallback) or fallback <= 0.0:
            raise ValueError("默认定位周期必须为正有限数")
        with self._lock:
            stamps = [sample.stamp for sample in self._samples]
        intervals = [
            later - earlier
            for earlier, later in zip(stamps, stamps[1:])
            if np.isfinite(later - earlier) and later > earlier
        ]
        if not intervals:
            return fallback
        window = intervals[-max(1, int(max_intervals)):]
        return float(np.median(np.asarray(window, dtype=float)))

    def at(self, stamp: float | None, *, max_delta: float = 0.08) -> PoseSample | None:
        """兼容旧调用：只返回已时间匹配的样本。"""
        return self.lookup(stamp, max_delta=max_delta).sample

    def lookup(self, stamp: float | None, *, max_delta: float = 0.08) -> PoseLookup:
        """按传感器时间戳查询，区分可等待与已过期的定位缺口。

        ``PENDING`` 只表示最新定位尚可能覆盖该时间戳，调用方应保留最新
        传感器帧等待；一旦定位历史已越过容差窗口仍无法匹配，才返回
        ``EXPIRED`` 并允许消费该帧。
        """
        if stamp is None:
            sample = self.latest()
            return PoseLookup(
                POSE_MATCHED if sample is not None else POSE_PENDING,
                sample,
                None,
            )
        target = float(stamp)
        tolerance = float(max_delta)
        if not np.isfinite(target):
            raise ValueError("定位查询时间戳必须为有限数")
        if not np.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError("max_delta 必须为非负有限数")
        with self._lock:
            if not self._samples:
                return PoseLookup(POSE_PENDING, None, None)
            nearest = min(self._samples, key=lambda sample: abs(sample.stamp - target))
            latest_stamp = self._samples[-1].stamp
        delta = abs(nearest.stamp - target)
        if delta <= tolerance:
            return PoseLookup(POSE_MATCHED, nearest, delta)
        if latest_stamp < target + tolerance:
            return PoseLookup(POSE_PENDING, None, delta)
        return PoseLookup(POSE_EXPIRED, None, delta)

    def sensor_to_world_matrix(
        self,
        sample: PoseSample,
        *,
        base_to_sensor_xyz: Iterable[float],
        base_to_sensor_rpy: Iterable[float],
    ) -> np.ndarray:
        """组合机体位姿与固定外参，生成 sensor 到 world 的齐次变换。

        ``base_to_sensor_*`` 来自公开传感器安装参数。先在 base 坐标系应用固定
        旋转/平移，再用里程计姿态变换到 world；返回矩阵供点云和 RGB-D
        反投影复用，保证不同感知模块采用同一变换顺序。
        """
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
        """把 ROS Odometry 复制成与 ROS 消息生命周期无关的轻量快照。"""
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
        frame_identity = (
            str(getattr(getattr(message, "header", None), "frame_id", "")),
            str(getattr(message, "child_frame_id", "")),
        )
        with self._lock:
            if not _is_valid_sample(sample):
                return
            if self._frame_identity is None:
                self._frame_identity = frame_identity
            elif frame_identity != self._frame_identity:
                self._continuity_epoch += 1
                self._frame_identity = frame_identity
                self._samples.clear()
            # 查询逻辑依赖历史按时间递增；异常乱序帧不能破坏这一不变量。
            if self._samples and sample.stamp <= self._samples[-1].stamp:
                latest_stamp = self._samples[-1].stamp
                timebase_reset = (
                    latest_stamp - sample.stamp > 1.0
                    and sample.stamp < 0.5 * latest_stamp
                )
                if not timebase_reset:
                    return
                self._continuity_epoch += 1
                self._samples.clear()
            self._samples.append(sample)

    def _on_correction_state(self, message) -> None:
        """收到定位节点事务回执后切换连续性 epoch 并丢弃旧坐标样本。"""
        if message is None or str(getattr(message.header, "frame_id", "")) != "world":
            return
        sequence = int(getattr(message.header, "seq", 0))
        stamp = _stamp_seconds(message)
        values = (
            float(getattr(message.vector, "x", float("nan"))),
            float(getattr(message.vector, "y", float("nan"))),
        )
        if sequence <= 0 or stamp is None or not np.all(np.isfinite(values)):
            return
        with self._lock:
            if sequence in self._correction_acks:
                return
            self._correction_acks[sequence] = (values[0], values[1], stamp)
            self._continuity_epoch += 1
            self._samples = deque(
                (sample for sample in self._samples if sample.stamp > stamp),
                maxlen=self._samples.maxlen,
            )
            if len(self._correction_acks) > 16:
                oldest = sorted(self._correction_acks)[:-16]
                for old_sequence in oldest:
                    self._correction_acks.pop(old_sequence, None)


def _stamp_seconds(message) -> float | None:
    """以秒读取 ROS header 时间戳；缺失或非法消息返回 ``None``。"""
    if message is None:
        return None
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    if hasattr(stamp, "to_sec"):
        value = float(stamp.to_sec())
        return value if np.isfinite(value) and value > 0.0 else None
    try:
        value = float(stamp)
        return value if np.isfinite(value) and value > 0.0 else None
    except (TypeError, ValueError):
        return None


def _is_valid_sample(sample: PoseSample) -> bool:
    """拒绝会把非有限数或无效姿态带入规划链的内部里程计。"""
    values = (
        sample.stamp,
        *sample.xyz,
        *sample.quaternion_xyzw,
        *sample.linear_xyz,
        *sample.angular_xyz,
    )
    if not np.all(np.isfinite(np.asarray(values, dtype=float))):
        return False
    return float(np.linalg.norm(sample.quaternion_xyzw)) > 1e-12

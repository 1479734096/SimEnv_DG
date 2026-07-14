#!/usr/bin/env python3
"""低频采集 EXIT_CABIN 物理诊断；输出只供离线分析，不得接入运行决策。"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
from threading import Lock
from typing import Iterable

import numpy as np

try:
    import rospy
    import sensor_msgs.point_cloud2 as pc2
    from gazebo_msgs.msg import LinkStates, ModelStates
    from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
    from nav_msgs.msg import Odometry, Path as PathMessage
    from rosgraph_msgs.msg import Log
    from sensor_msgs.msg import Image, Imu, PointCloud2
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - 离线单元测试环境不要求 ROS
    rospy = None
    pc2 = None
    LinkStates = ModelStates = PoseStamped = Twist = WrenchStamped = None
    Odometry = PathMessage = Log = Image = Imu = PointCloud2 = String = None


ELEVATOR_TASK_PATTERN = re.compile(r"^TakeElevator\((-?\d+)->(-?\d+)\)$")
RELEVANT_POSE_TOKENS = (
    "dynamic_elevator",
    "elevator_threshold",
    "elevator_sill",
    "elevator_jamb",
)
FOOT_TOPICS = {
    "FR": "/visual/FR_foot_contact/the_force",
    "FL": "/visual/FL_foot_contact/the_force",
    "RR": "/visual/RR_foot_contact/the_force",
    "RL": "/visual/RL_foot_contact/the_force",
}


def parse_elevator_floors(task_name: object) -> tuple[int, int] | None:
    if not isinstance(task_name, str):
        return None
    match = ELEVATOR_TASK_PATTERN.match(task_name)
    if match is None:
        return None
    return (int(match.group(1)), int(match.group(2)))


def quaternion_to_rpy(quaternion) -> tuple[float, float, float]:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return (roll, pitch, yaw)


def pose_payload(pose) -> dict[str, float]:
    roll, pitch, yaw = quaternion_to_rpy(pose.orientation)
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def select_relevant_poses(names: Iterable[str], poses: Iterable[object]) -> dict[str, dict[str, float]]:
    selected: dict[str, dict[str, float]] = {}
    for name, pose in zip(names, poses):
        if name == "a1_gazebo" or name.startswith("a1_gazebo_") or any(
            token in name for token in RELEVANT_POSE_TOKENS
        ):
            selected[str(name)] = pose_payload(pose)
    return selected


def select_relevant_link_states(
    names: Iterable[str],
    poses: Iterable[object],
    twists: Iterable[object],
) -> dict[str, dict[str, float]]:
    """保留 A1 全部 model::link 及电梯门区 link 的世界状态。"""
    selected: dict[str, dict[str, float]] = {}
    duplicate_counts: Counter[str] = Counter()
    for name, pose, twist in zip(names, poses, twists):
        normalized_name = str(name)
        if not (
            normalized_name.startswith("a1_gazebo::")
            or any(token in normalized_name for token in RELEVANT_POSE_TOKENS)
        ):
            continue
        payload = pose_payload(pose)
        payload.update(
            {
                "linear_x": float(twist.linear.x),
                "linear_y": float(twist.linear.y),
                "linear_z": float(twist.linear.z),
                "angular_x": float(twist.angular.x),
                "angular_y": float(twist.angular.y),
                "angular_z": float(twist.angular.z),
            }
        )
        # Gazebo 可能发布重名 link；不能让后出现的异常样本覆盖真实足端。
        duplicate_counts[normalized_name] += 1
        occurrence = duplicate_counts[normalized_name]
        output_name = normalized_name if occurrence == 1 else f"{normalized_name}#{occurrence}"
        selected[output_name] = payload
    return selected


def _finite_stats(values: np.ndarray, *, prefix: str) -> dict[str, float | int | None]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {
            f"{prefix}_min": None,
            f"{prefix}_p10": None,
            f"{prefix}_median": None,
            f"{prefix}_p90": None,
            f"{prefix}_max": None,
        }
    return {
        f"{prefix}_min": float(np.min(finite)),
        f"{prefix}_p10": float(np.percentile(finite, 10)),
        f"{prefix}_median": float(np.median(finite)),
        f"{prefix}_p90": float(np.percentile(finite, 90)),
        f"{prefix}_max": float(np.max(finite)),
    }


def summarize_forward_cloud(
    points: Iterable[tuple[float, float, float]],
    *,
    robot_xyz_yaw: tuple[float, float, float, float],
    min_forward: float = 0.15,
    max_forward: float = 1.20,
    half_width: float = 0.45,
    bin_size: float = 0.20,
) -> dict[str, object]:
    """把 odom 点云投到机体前方足迹走廊，保留地面/门槛高度分布。"""
    robot_x, robot_y, robot_z, robot_yaw = robot_xyz_yaw
    cosine = math.cos(robot_yaw)
    sine = math.sin(robot_yaw)
    selected: list[tuple[float, float, float, float]] = []
    for point in points:
        x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        dx = x - robot_x
        dy = y - robot_y
        forward = cosine * dx + sine * dy
        lateral = -sine * dx + cosine * dy
        if min_forward <= forward <= max_forward and abs(lateral) <= half_width:
            selected.append((forward, lateral, z, z - robot_z))

    if not selected:
        return {
            "point_count": 0,
            "world_z_min": None,
            "world_z_max": None,
            "world_z_span": None,
            "relative_z_min": None,
            "relative_z_max": None,
            "relative_z_span": None,
            "forward_bins": [],
        }

    data = np.asarray(selected, dtype=float)
    world_z = data[:, 2]
    relative_z = data[:, 3]
    summary: dict[str, object] = {"point_count": int(data.shape[0])}
    summary.update(_finite_stats(world_z, prefix="world_z"))
    summary.update(_finite_stats(relative_z, prefix="relative_z"))
    summary["world_z_span"] = float(np.max(world_z) - np.min(world_z))
    summary["relative_z_span"] = float(np.max(relative_z) - np.min(relative_z))

    bins: list[dict[str, object]] = []
    bin_indexes = np.floor((data[:, 0] - min_forward) / bin_size).astype(int)
    for bin_index in sorted(set(int(value) for value in bin_indexes)):
        mask = bin_indexes == bin_index
        bin_z = world_z[mask]
        bins.append(
            {
                "forward_start": min_forward + bin_index * bin_size,
                "forward_end": min(max_forward, min_forward + (bin_index + 1) * bin_size),
                "point_count": int(np.count_nonzero(mask)),
                "world_z_min": float(np.min(bin_z)),
                "world_z_median": float(np.median(bin_z)),
                "world_z_max": float(np.max(bin_z)),
            }
        )
    summary["forward_bins"] = bins
    return summary


def summarize_depth_array(depth: np.ndarray, encoding: str) -> dict[str, float | int | None]:
    values = np.asarray(depth)
    if encoding.upper() in {"16UC1", "MONO16"}:
        metres = values.astype(np.float64) * 0.001
    else:
        metres = values.astype(np.float64)
    valid = metres[np.isfinite(metres) & (metres > 0.0) & (metres <= 20.0)]
    if valid.size == 0:
        return {"valid_count": 0, "min_m": None, "median_m": None, "p90_m": None, "max_m": None}
    return {
        "valid_count": int(valid.size),
        "min_m": float(np.min(valid)),
        "median_m": float(np.median(valid)),
        "p90_m": float(np.percentile(valid, 90)),
        "max_m": float(np.max(valid)),
    }


def summarize_depth_message(message) -> dict[str, object]:
    encoding = str(message.encoding)
    if encoding.upper() in {"16UC1", "MONO16"}:
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
    elif encoding.upper() == "32FC1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
    else:
        return {"encoding": encoding, "error": "unsupported_encoding"}
    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        return {"encoding": encoding, "error": "empty_image"}
    image = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=message.data,
        strides=(int(message.step), dtype.itemsize),
    )
    # 门槛通常出现在画面中下部；只统计中央 50% 宽、下方 55% 高，降低墙面干扰。
    roi = image[int(height * 0.45) :, int(width * 0.25) : max(int(width * 0.75), 1)]
    result: dict[str, object] = {
        "encoding": encoding,
        "roi": [int(height * 0.45), int(height), int(width * 0.25), max(int(width * 0.75), 1)],
    }
    result.update(summarize_depth_array(roi, encoding))
    return result


class ExitOutcomeTracker:
    """只在实际观察到 EXIT_CABIN 后判定本次出厢结果。"""

    def __init__(self):
        self.seen_exit = False
        self.task_name: str | None = None
        self.first_stuck_observed = False
        self.terminal_outcome: str | None = None
        self._initial_stuck_count: int | None = None

    @property
    def outcome(self) -> str | None:
        if self.terminal_outcome is not None:
            return self.terminal_outcome
        if self.first_stuck_observed:
            return "exit_cabin_first_stuck"
        return None

    def observe_status(self, status: dict[str, object]) -> None:
        task_name = status.get("current_task")
        phase = status.get("task_phase")
        controller = status.get("controller")
        stuck_count = None
        if isinstance(controller, dict):
            try:
                stuck_count = int(controller.get("stuck_count"))
            except (TypeError, ValueError):
                stuck_count = None
        if parse_elevator_floors(task_name) is not None and phase == "EXIT_CABIN":
            entering_exit = not self.seen_exit
            self.seen_exit = True
            self.task_name = str(task_name)
            if entering_exit:
                self._initial_stuck_count = stuck_count
            elif self._initial_stuck_count is None:
                self._initial_stuck_count = stuck_count
            elif stuck_count is not None and stuck_count > self._initial_stuck_count:
                self.first_stuck_observed = True
        if (
            self.seen_exit
            and self.terminal_outcome is None
            and task_name == self.task_name
            and phase in {"CLOSE_TARGET_FLOOR_DOOR", "DONE"}
        ):
            self.terminal_outcome = "exit_cabin_succeeded"

    def observe_log(self, message: str) -> None:
        if not self.seen_exit:
            return
        if "离开轿厢倒车恢复失败" in message:
            self.terminal_outcome = "exit_cabin_backoff_failed"
        elif "离开轿厢首次尝试卡住" in message or "离开轿厢卡住" in message:
            self.first_stuck_observed = True
        elif "离开轿厢失败" in message or "轿厢内倒车路径不安全" in message:
            self.terminal_outcome = "exit_cabin_failed"


def _message_stamp(message) -> float | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None or not hasattr(stamp, "to_sec"):
        return None
    return float(stamp.to_sec())


def _path_payload(message) -> dict[str, object] | None:
    if message is None:
        return None
    points = [
        [float(pose.pose.position.x), float(pose.pose.position.y)]
        for pose in message.poses
    ]
    return {
        "stamp": _message_stamp(message),
        "frame_id": str(message.header.frame_id),
        "point_count": len(points),
        "goal_xy": points[-1] if points else None,
    }


def _pose_stamped_payload(message) -> dict[str, object] | None:
    if message is None:
        return None
    return {
        "stamp": _message_stamp(message),
        "frame_id": str(message.header.frame_id),
        "x": float(message.pose.position.x),
        "y": float(message.pose.position.y),
        "z": float(message.pose.position.z),
    }


def _odom_payload(message) -> dict[str, object] | None:
    if message is None:
        return None
    payload: dict[str, object] = pose_payload(message.pose.pose)
    payload.update(
        {
            "stamp": _message_stamp(message),
            "linear_x": float(message.twist.twist.linear.x),
            "linear_y": float(message.twist.twist.linear.y),
            "linear_z": float(message.twist.twist.linear.z),
            "angular_z": float(message.twist.twist.angular.z),
        }
    )
    return payload


def _twist_payload(message) -> dict[str, float] | None:
    if message is None:
        return None
    return {
        "linear_x": float(message.linear.x),
        "linear_y": float(message.linear.y),
        "linear_z": float(message.linear.z),
        "angular_x": float(message.angular.x),
        "angular_y": float(message.angular.y),
        "angular_z": float(message.angular.z),
    }


def _imu_payload(message) -> dict[str, object] | None:
    if message is None:
        return None
    roll, pitch, yaw = quaternion_to_rpy(message.orientation)
    return {
        "stamp": _message_stamp(message),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "angular_velocity": [
            float(message.angular_velocity.x),
            float(message.angular_velocity.y),
            float(message.angular_velocity.z),
        ],
        "linear_acceleration": [
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
        ],
    }


def _wrench_payload(message) -> dict[str, object]:
    force = message.wrench.force
    torque = message.wrench.torque
    return {
        "stamp": _message_stamp(message),
        "force": [float(force.x), float(force.y), float(force.z)],
        "force_norm": math.sqrt(float(force.x) ** 2 + float(force.y) ** 2 + float(force.z) ** 2),
        "torque": [float(torque.x), float(torque.y), float(torque.z)],
    }


class ExitDiagnosticRecorder:
    def __init__(self, output_path: Path, summary_path: Path, stop_marker: Path | None, rate: float):
        self.output_path = output_path
        self.summary_path = summary_path
        self.stop_marker = stop_marker
        self.period = 1.0 / max(0.2, float(rate))
        self._lock = Lock()
        self._stream = output_path.open("w", encoding="utf-8", buffering=1)
        self._status: dict[str, object] = {}
        self._odom = None
        self._cmd = None
        self._imu = None
        self._cloud = None
        self._depth = None
        self._followed_path = None
        self._local_target = None
        self._models: dict[str, dict[str, float]] = {}
        self._links: dict[str, dict[str, float]] = {}
        self._feet: dict[str, object] = {}
        self._elevator_active = False
        self._tracker = ExitOutcomeTracker()
        self._outcome_written = False
        self._sample_errors: list[str] = []
        self._topic_counts: Counter[str] = Counter()
        self._sample_count = 0
        self._saved_link_names: set[str] = set()
        self._start_wall = datetime.now(timezone.utc).isoformat()
        self._start_sim = float(rospy.get_time())
        self._closed = False
        self._subscribers = []
        self._model_subscriber = None
        self._link_subscriber = None

        self._subscribers.extend(
            [
                rospy.Subscriber("/danger_search/status", String, self._on_status, queue_size=1),
                rospy.Subscriber("/cmd_vel", Twist, self._on_cmd, queue_size=1),
                rospy.Subscriber("/Odometry_gazebo", Odometry, self._on_odom, queue_size=1),
                rospy.Subscriber("/trunk_imu", Imu, self._on_imu, queue_size=1),
                rospy.Subscriber("/livox/Pointcloud2", PointCloud2, self._on_cloud, queue_size=1),
                rospy.Subscriber("/real_sense/depth/image_raw", Image, self._on_depth, queue_size=1),
                rospy.Subscriber(
                    "/danger_search/path_followed",
                    PathMessage,
                    self._on_followed_path,
                    queue_size=1,
                ),
                rospy.Subscriber(
                    "/danger_search/local_target",
                    PoseStamped,
                    self._on_local_target,
                    queue_size=1,
                ),
                rospy.Subscriber("/rosout_agg", Log, self._on_log, queue_size=20),
            ]
        )
        for foot, topic in FOOT_TOPICS.items():
            self._subscribers.append(
                rospy.Subscriber(topic, WrenchStamped, self._foot_callback(foot), queue_size=1)
            )
        self._timer = rospy.Timer(rospy.Duration(self.period), self._on_timer)
        rospy.on_shutdown(self.close)

    def _record_topic(self, name: str) -> None:
        self._topic_counts[name] += 1

    def _on_status(self, message) -> None:
        self._record_topic("status")
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            status = {"parse_error": str(message.data)}
        start_physics_sampling = False
        with self._lock:
            self._status = status
            self._elevator_active = parse_elevator_floors(status.get("current_task")) is not None
            self._tracker.observe_status(status)
            start_physics_sampling = self._elevator_active and self._link_subscriber is None
            if start_physics_sampling:
                self._link_subscriber = False
        if start_physics_sampling:
            # link_states 数据量较大，只在电梯任务出现后订阅，避免拖慢正常探索。
            self._model_subscriber = rospy.Subscriber(
                "/gazebo/model_states", ModelStates, self._on_models, queue_size=1
            )
            self._link_subscriber = rospy.Subscriber(
                "/gazebo/link_states", LinkStates, self._on_links, queue_size=1
            )
            self._subscribers.extend([self._model_subscriber, self._link_subscriber])

    def _on_cmd(self, message) -> None:
        self._record_topic("cmd_vel")
        with self._lock:
            self._cmd = message

    def _on_odom(self, message) -> None:
        self._record_topic("odom")
        with self._lock:
            self._odom = message

    def _on_imu(self, message) -> None:
        self._record_topic("trunk_imu")
        with self._lock:
            self._imu = message

    def _on_cloud(self, message) -> None:
        self._record_topic("cloud")
        with self._lock:
            if self._elevator_active or self._tracker.seen_exit:
                self._cloud = message

    def _on_depth(self, message) -> None:
        self._record_topic("depth")
        with self._lock:
            if self._elevator_active or self._tracker.seen_exit:
                self._depth = message

    def _on_followed_path(self, message) -> None:
        self._record_topic("path_followed")
        with self._lock:
            self._followed_path = message

    def _on_local_target(self, message) -> None:
        self._record_topic("local_target")
        with self._lock:
            self._local_target = message

    def _on_models(self, message) -> None:
        self._record_topic("model_states")
        with self._lock:
            if self._elevator_active or self._tracker.seen_exit:
                self._models = select_relevant_poses(message.name, message.pose)

    def _on_links(self, message) -> None:
        self._record_topic("link_states")
        with self._lock:
            if self._elevator_active or self._tracker.seen_exit:
                self._links = select_relevant_link_states(
                    message.name,
                    message.pose,
                    message.twist,
                )
                self._saved_link_names.update(self._links)

    def _on_log(self, message) -> None:
        self._record_topic("rosout")
        with self._lock:
            self._tracker.observe_log(str(message.msg))

    def _foot_callback(self, foot: str):
        def callback(message) -> None:
            self._record_topic(f"foot_{foot}")
            with self._lock:
                if self._elevator_active or self._tracker.seen_exit:
                    self._feet[foot] = message

        return callback

    def _cloud_summary(self, cloud, odom_payload: dict[str, object] | None) -> dict[str, object] | None:
        if cloud is None or odom_payload is None or pc2 is None:
            return None
        points = []
        for index, point in enumerate(
            pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if index % 3 != 0:
                continue
            points.append((float(point[0]), float(point[1]), float(point[2])))
            if len(points) >= 6000:
                break
        summary = summarize_forward_cloud(
            points,
            robot_xyz_yaw=(
                float(odom_payload["x"]),
                float(odom_payload["y"]),
                float(odom_payload["z"]),
                float(odom_payload["yaw"]),
            ),
        )
        summary.update(
            {
                "stamp": _message_stamp(cloud),
                "frame_id": str(cloud.header.frame_id),
                "sampled_input_count": len(points),
            }
        )
        return summary

    def _on_timer(self, _event) -> None:
        with self._lock:
            status = dict(self._status)
            odom = self._odom
            cmd = self._cmd
            imu = self._imu
            cloud = self._cloud
            depth = self._depth
            followed_path = self._followed_path
            local_target = self._local_target
            models = dict(self._models)
            links = dict(self._links)
            feet = dict(self._feet)
            active = self._elevator_active or self._tracker.seen_exit
            outcome = self._tracker.outcome
            seen_exit = self._tracker.seen_exit
            task_name = self._tracker.task_name
        if not active:
            return

        odom_data = _odom_payload(odom)
        row: dict[str, object] = {
            "sim_time": float(rospy.get_time()),
            "wall_time": datetime.now(timezone.utc).isoformat(),
            "status": status,
            "elevator_floors": parse_elevator_floors(status.get("current_task"))
            or parse_elevator_floors(task_name),
            "odom": odom_data,
            "cmd_vel": _twist_payload(cmd),
            "trunk_imu": _imu_payload(imu),
            "followed_path": _path_payload(followed_path),
            "local_target": _pose_stamped_payload(local_target),
            "foot_contacts": {name: _wrench_payload(message) for name, message in feet.items()},
            "model_poses": models,
            "link_states": links,
            "seen_exit_cabin": seen_exit,
            "exit_outcome": outcome,
        }
        try:
            row["forward_cloud"] = self._cloud_summary(cloud, odom_data)
        except Exception as exc:  # 诊断失败不得影响被观察进程
            row["forward_cloud"] = {"error": f"{type(exc).__name__}: {exc}"}
            self._sample_errors.append(str(row["forward_cloud"]["error"]))
        try:
            depth_data = summarize_depth_message(depth) if depth is not None else None
            if isinstance(depth_data, dict):
                depth_data["stamp"] = _message_stamp(depth)
            row["depth_roi"] = depth_data
        except Exception as exc:  # 诊断失败不得影响被观察进程
            row["depth_roi"] = {"error": f"{type(exc).__name__}: {exc}"}
            self._sample_errors.append(str(row["depth_roi"]["error"]))

        self._stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
        self._sample_count += 1
        if outcome is not None and not self._outcome_written:
            self._outcome_written = True
            if self.stop_marker is not None:
                self.stop_marker.write_text(
                    json.dumps(
                        {"outcome": outcome, "sim_time": row["sim_time"], "task": task_name},
                        ensure_ascii=False,
                        indent=2,
                    )
                    + "\n",
                    encoding="utf-8",
                )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        summary = {
            "diagnostic_scope": "offline_only",
            "must_not_be_used_for_runtime_decisions": True,
            "start_wall": self._start_wall,
            "end_wall": datetime.now(timezone.utc).isoformat(),
            "start_sim": self._start_sim,
            "end_sim": float(rospy.get_time()) if rospy is not None else None,
            "sample_rate_hz": 1.0 / self.period,
            "sample_count": self._sample_count,
            "topic_message_counts": dict(sorted(self._topic_counts.items())),
            "saved_link_names": sorted(self._saved_link_names),
            "foot_force_evidence_role": "auxiliary_only_zero_values_are_not_contact_evidence",
            "seen_exit_cabin": self._tracker.seen_exit,
            "exit_task": self._tracker.task_name,
            "exit_outcome": self._tracker.outcome,
            "exit_first_stuck_observed": self._tracker.first_stuck_observed,
            "exit_terminal_outcome": self._tracker.terminal_outcome,
            "sample_errors": self._sample_errors[-20:],
        }
        self.summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self._stream.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="JSONL 采样输出")
    parser.add_argument("--summary", type=Path, required=True, help="采集汇总 JSON")
    parser.add_argument("--stop-marker", type=Path, help="观察到 EXIT 结果后写入的标记文件")
    parser.add_argument("--rate", type=float, default=2.0, help="采样频率，默认 2 Hz")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if rospy is None:
        raise RuntimeError("当前环境没有 ROS Python 依赖，无法采集 EXIT_CABIN 诊断")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    if args.stop_marker is not None:
        args.stop_marker.parent.mkdir(parents=True, exist_ok=True)
    rospy.init_node("exit_cabin_diagnostic_recorder", anonymous=True, disable_signals=False)
    recorder = ExitDiagnosticRecorder(args.output, args.summary, args.stop_marker, args.rate)
    rospy.loginfo("EXIT_CABIN 外部诊断采集已启动：rate=%.2fHz output=%s", args.rate, args.output)
    rospy.spin()
    recorder.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

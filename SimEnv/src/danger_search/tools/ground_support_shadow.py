#!/usr/bin/env python3
"""独立运行的负障碍支撑网格 shadow 诊断器。

该进程只订阅正式运行数据并写低频 JSONL。它不发布 ROS 消息，不修改地图，
也不向导航或控制器返回结果。
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
from threading import Lock
import time

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from danger_search.support_shadow import ShadowSupportConfig, ShadowSupportGrid

try:
    import rospy
    import sensor_msgs.point_cloud2 as point_cloud2
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, Path as PathMessage
    from rosgraph_msgs.msg import Log
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except ImportError:  # pragma: no cover - 单元测试不需要 ROS
    rospy = point_cloud2 = None
    Twist = Odometry = PathMessage = Log = PointCloud2 = String = TFMessage = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--snapshot-rate", type=float, default=2.0)
    parser.add_argument(
        "--minimum-lookahead",
        type=float,
        default=None,
        help="仅覆盖 shadow 假想扫掠区的最小前视距离；不影响生产控制",
    )
    parser.add_argument(
        "--maximum-lookahead",
        type=float,
        default=None,
        help="仅覆盖 shadow 假想扫掠区的最大前视距离；不影响生产控制",
    )
    parser.add_argument(
        "--sweep-half-width",
        type=float,
        default=None,
        help="物理与误差包络推导出的诊断扫掠半宽；不改变生产控制",
    )
    parser.add_argument(
        "--threshold-width",
        type=float,
        action="append",
        default=None,
        help="相邻半宽敏感性值，可重复；不提供时沿用诊断默认值",
    )
    parser.add_argument(
        "--alert-state",
        type=Path,
        default=None,
        help="可选的低延迟原子状态文件，供独立诊断停车锁存器读取",
    )
    return parser.parse_args()


def _stamp_seconds(message, fallback: float) -> float:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    to_sec = getattr(stamp, "to_sec", None)
    return float(to_sec()) if callable(to_sec) else float(fallback)


def _quaternion_rotation(quaternion) -> np.ndarray:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _yaw_from_rotation(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _parse_structured_log(message: str) -> dict[str, object] | None:
    start = str(message).find("{")
    if start < 0:
        return None
    try:
        payload = json.loads(str(message)[start:])
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def _path_points(message) -> list[np.ndarray]:
    return [
        np.asarray([pose.pose.position.x, pose.pose.position.y], dtype=float)
        for pose in getattr(message, "poses", [])
    ]


def _distribution(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "minimum": None, "median": None, "p95": None, "maximum": None}
    array = np.asarray(values, dtype=float)
    return {
        "count": int(array.size),
        "minimum": float(np.min(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "maximum": float(np.max(array)),
    }


class GroundSupportShadowRecorder:
    def __init__(
        self,
        output: Path,
        summary: Path,
        snapshot_rate: float,
        minimum_lookahead: float | None = None,
        maximum_lookahead: float | None = None,
        sweep_half_width: float | None = None,
        threshold_widths: list[float] | None = None,
        alert_state: Path | None = None,
    ):
        self.output = output
        self.summary_path = summary
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.output.open("w", encoding="utf-8", buffering=1)
        self._lock = Lock()
        self._odom = None
        self._floor: int | None = None
        self._path: list[np.ndarray] = []
        self._command_linear = 0.0
        self._ambiguous = False
        self._sensor_translation: np.ndarray | None = None
        self._sensor_mount_rotation: np.ndarray | None = None
        self._topic_counts = Counter()
        self._event_counts = Counter()
        self._processing_seconds: list[float] = []
        self._cloud_odom_stamp_deltas: list[float] = []
        self._written_rows = 0
        self._start_wall = datetime.now(timezone.utc).isoformat()
        self._start_sim = float(rospy.get_time())
        self._closed = False
        self._alert_state_path = alert_state
        self._alert_first_active_stamp: float | None = None
        self._alert_consecutive_active_frames = 0

        snapshot_period = 1.0 / max(0.1, float(snapshot_rate))
        defaults = ShadowSupportConfig()
        resolved_minimum_lookahead = (
            defaults.minimum_lookahead
            if minimum_lookahead is None
            else float(minimum_lookahead)
        )
        resolved_maximum_lookahead = (
            defaults.maximum_lookahead
            if maximum_lookahead is None
            else float(maximum_lookahead)
        )
        if not (
            0.0 < resolved_minimum_lookahead <= resolved_maximum_lookahead
        ):
            raise ValueError("shadow 前视距离必须满足 0 < minimum <= maximum")
        resolved_sweep_half_width = (
            defaults.sweep_half_width
            if sweep_half_width is None
            else float(sweep_half_width)
        )
        resolved_threshold_widths = tuple(
            float(value)
            for value in (
                defaults.threshold_widths
                if threshold_widths is None
                else threshold_widths
            )
        )
        if resolved_sweep_half_width <= 0.0:
            raise ValueError("shadow 扫掠半宽必须大于 0")
        if not resolved_threshold_widths or any(value <= 0.0 for value in resolved_threshold_widths):
            raise ValueError("shadow 相邻半宽必须全部大于 0")
        config = ShadowSupportConfig(
            resolution=float(rospy.get_param("/danger_search/map/resolution", 0.1)),
            origin_x=float(rospy.get_param("/danger_search/map/origin_x", -14.0)),
            origin_y=float(rospy.get_param("/danger_search/map/origin_y", -6.0)),
            floor_height=float(rospy.get_param("/danger_search/map/floor_height", 2.6)),
            cloud_stride=int(rospy.get_param("/danger_search/map/cloud_stride", 3)),
            cloud_max_points=int(rospy.get_param("/danger_search/map/cloud_max_points", 18000)),
            snapshot_period=snapshot_period,
            minimum_lookahead=resolved_minimum_lookahead,
            maximum_lookahead=resolved_maximum_lookahead,
            sweep_half_width=resolved_sweep_half_width,
            threshold_widths=resolved_threshold_widths,
        )
        self.grid = ShadowSupportGrid(config)
        self._write(
            {
                "schema": "ground_support_shadow_start_v1",
                "sim_time": self._start_sim,
                "wall_time": self._start_wall,
                "shadow_only": True,
                "behavior_effect": False,
                "inputs": [
                    "/livox/Pointcloud2",
                    "/Odometry_gazebo",
                    "/tf_static",
                    "/danger_search/status",
                    "/danger_search/path_followed",
                    "/cmd_vel",
                    "/rosout_agg",
                ],
                "config": config.diagnostic_payload(),
            }
        )

        self._subscribers = [
            rospy.Subscriber("/Odometry_gazebo", Odometry, self._on_odom, queue_size=1),
            rospy.Subscriber("/danger_search/status", String, self._on_status, queue_size=1),
            rospy.Subscriber("/danger_search/path_followed", PathMessage, self._on_path, queue_size=1),
            rospy.Subscriber("/cmd_vel", Twist, self._on_command, queue_size=1),
            rospy.Subscriber("/tf_static", TFMessage, self._on_tf_static, queue_size=1),
            rospy.Subscriber("/rosout_agg", Log, self._on_rosout, queue_size=20),
            rospy.Subscriber("/livox/Pointcloud2", PointCloud2, self._on_cloud, queue_size=1),
        ]
        rospy.on_shutdown(self.close)

    def _write(self, payload: dict[str, object]) -> None:
        self._stream.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")
        self._written_rows += 1

    def _write_alert_state(self, result: dict[str, object]) -> None:
        if self._alert_state_path is None:
            return
        alert = result.get("hypothetical_alert")
        if not isinstance(alert, dict):
            return
        active = bool(alert.get("active", False))
        stamp = float(result.get("stamp", rospy.get_time()))
        if active:
            if self._alert_consecutive_active_frames == 0:
                self._alert_first_active_stamp = stamp
            self._alert_consecutive_active_frames += 1
        else:
            self._alert_first_active_stamp = None
            self._alert_consecutive_active_frames = 0
        payload = {
            "schema": "ground_support_shadow_alert_state_v1",
            "active": active,
            "cloud_stamp": stamp,
            "receive_sim_time": float(result.get("receive_sim_time", rospy.get_time())),
            "first_active_cloud_stamp": self._alert_first_active_stamp,
            "consecutive_active_frames": self._alert_consecutive_active_frames,
            "floor": result.get("floor"),
            "robot_xyz": result.get("robot_xyz"),
            "robot_yaw": result.get("robot_yaw"),
            "command_linear": result.get("command_linear"),
            "lookahead_distance": result.get("lookahead_distance"),
            "path_footprint_sweep": result.get("path_footprint_sweep"),
            "shadow_only": True,
            "production_gate_enabled": False,
        }
        self._alert_state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._alert_state_path.with_suffix(self._alert_state_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self._alert_state_path)

    def _on_odom(self, message) -> None:
        self._topic_counts["odom"] += 1
        with self._lock:
            self._odom = message

    def _on_status(self, message) -> None:
        self._topic_counts["status"] += 1
        try:
            payload = json.loads(message.data)
            floor = int(payload["floor"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            self._floor = floor

    def _on_path(self, message) -> None:
        self._topic_counts["path"] += 1
        path = _path_points(message)
        with self._lock:
            self._path = path

    def _on_command(self, message) -> None:
        self._topic_counts["cmd_vel"] += 1
        with self._lock:
            self._command_linear = float(message.linear.x)

    def _on_tf_static(self, message) -> None:
        self._topic_counts["tf_static"] += 1
        for transform in getattr(message, "transforms", []):
            if transform.header.frame_id.lstrip("/") != "base":
                continue
            if transform.child_frame_id.lstrip("/") != "laser_livox":
                continue
            translation = transform.transform.translation
            sensor_translation = np.asarray(
                [translation.x, translation.y, translation.z], dtype=float
            )
            sensor_rotation = _quaternion_rotation(transform.transform.rotation)
            with self._lock:
                self._sensor_translation = sensor_translation
                self._sensor_mount_rotation = sensor_rotation
            return

    def _on_rosout(self, message) -> None:
        self._topic_counts["rosout"] += 1
        payload = _parse_structured_log(getattr(message, "msg", ""))
        if payload is None:
            return
        event = payload.get("event")
        if event == "floor_transition_ambiguous_enter":
            with self._lock:
                self._ambiguous = True
            self._event_counts[event] += 1
            self._write(
                {
                    "schema": "ground_support_shadow_guard_event_v1",
                    "sim_time": float(rospy.get_time()),
                    "event": event,
                    "shadow_action": "latch_skip_support_updates",
                    "production_event": payload,
                }
            )
        elif event == "floor_transition_ambiguous_exit":
            try:
                confirmed_floor = int(payload["current_floor"])
            except (KeyError, TypeError, ValueError):
                self._event_counts["floor_transition_ambiguous_exit_missing_floor"] += 1
                self._write(
                    {
                        "schema": "ground_support_shadow_guard_event_v1",
                        "sim_time": float(rospy.get_time()),
                        "event": event,
                        "shadow_action": "keep_latch_missing_confirmed_floor",
                        "production_event": payload,
                    }
                )
                return
            with self._lock:
                # 生产状态已用新 odom 确认楼层；同步更新诊断快照，避免低频
                # /danger_search/status 的旧楼层把解锁后的首帧写入源楼层。
                self._floor = confirmed_floor
                self._ambiguous = False
            self._event_counts[event] += 1
            self._write(
                {
                    "schema": "ground_support_shadow_guard_event_v1",
                    "sim_time": float(rospy.get_time()),
                    "event": event,
                    "shadow_action": "resume_on_next_fresh_cloud",
                    "production_event": payload,
                }
            )

    def _on_cloud(self, message) -> None:
        self._topic_counts["cloud"] += 1
        started = time.perf_counter()
        now = float(rospy.get_time())
        stamp = _stamp_seconds(message, now)
        with self._lock:
            odom = self._odom
            floor = self._floor
            path = list(self._path)
            command_linear = self._command_linear
            ambiguous = self._ambiguous
            sensor_translation = self._sensor_translation
            sensor_mount_rotation = self._sensor_mount_rotation
        if odom is None or floor is None or sensor_translation is None or sensor_mount_rotation is None:
            self._event_counts["cloud_missing_snapshot_input"] += 1
            return

        try:
            points = np.asarray(
                list(
                    point_cloud2.read_points(
                        message,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ),
                dtype=float,
            ).reshape((-1, 3))
            position = odom.pose.pose.position
            body_rotation = _quaternion_rotation(odom.pose.pose.orientation)
            robot_position = np.asarray([position.x, position.y, position.z], dtype=float)
            sensor_origin = robot_position + body_rotation @ sensor_translation
            sensor_rotation = body_rotation @ sensor_mount_rotation
            odom_stamp = _stamp_seconds(odom, now)
            self._cloud_odom_stamp_deltas.append(odom_stamp - stamp)
            result = self.grid.process_frame(
                stamp=stamp,
                floor=floor,
                robot_xy=robot_position[:2],
                robot_yaw=_yaw_from_rotation(body_rotation),
                sensor_origin=sensor_origin,
                sensor_rotation=sensor_rotation,
                points=points,
                path=path,
                command_linear=command_linear,
                ambiguous=ambiguous,
            )
            result.update(
                {
                    "receive_sim_time": now,
                    "odom_stamp": odom_stamp,
                    "cloud_odom_stamp_delta": odom_stamp - stamp,
                    "robot_xyz": robot_position.tolist(),
                    "robot_yaw": _yaw_from_rotation(body_rotation),
                    "sensor_origin": sensor_origin.tolist(),
                    "command_linear": command_linear,
                    "path_point_count": len(path),
                    "floor_transition_ambiguous": ambiguous,
                }
            )
            if result.get("action") == "update":
                self._write_alert_state(result)
            if result.get("action") == "skip_floor_transition_ambiguous":
                self._event_counts["skipped_ambiguous_cloud"] += 1
                self._write(result)
            elif result.get("should_emit"):
                result["changed_grid_cells"] = self.grid.consume_changed_cells(floor)
                self._write(result)
            self._event_counts[str(result.get("action"))] += 1
        except Exception as exc:  # 诊断进程失败不得传播到被观察算法
            self._event_counts["cloud_processing_error"] += 1
            self._write(
                {
                    "schema": "ground_support_shadow_error_v1",
                    "sim_time": now,
                    "cloud_stamp": stamp,
                    "error": f"{type(exc).__name__}: {exc}",
                    "shadow_only": True,
                    "behavior_effect": False,
                }
            )
        finally:
            self._processing_seconds.append(time.perf_counter() - started)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._write(
            {
                "schema": "ground_support_shadow_final_grid_v1",
                "sim_time": float(rospy.get_time()) if rospy is not None else None,
                "shadow_only": True,
                "behavior_effect": False,
                "sparse_grid_by_floor": self.grid.sparse_grid_snapshot(),
            }
        )
        payload = self.grid.summary()
        payload.update(
            {
                "start_wall": self._start_wall,
                "end_wall": datetime.now(timezone.utc).isoformat(),
                "start_sim": self._start_sim,
                "end_sim": float(rospy.get_time()) if rospy is not None else None,
                "written_row_count": self._written_rows,
                "topic_message_counts": dict(self._topic_counts),
                "event_counts": dict(self._event_counts),
                "cloud_processing_seconds": _distribution(self._processing_seconds),
                "cloud_odom_stamp_delta": _distribution(self._cloud_odom_stamp_deltas),
                "has_control_publisher": False,
                "production_gate_enabled": False,
                "alert_state_path": (
                    str(self._alert_state_path)
                    if self._alert_state_path is not None
                    else None
                ),
            }
        )
        self.summary_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self._stream.close()


def main() -> int:
    args = parse_args()
    if rospy is None or point_cloud2 is None:
        raise SystemExit("当前环境没有 ROS Python 依赖")
    rospy.init_node("ground_support_shadow", anonymous=True, disable_signals=False)
    recorder = GroundSupportShadowRecorder(
        args.output,
        args.summary,
        args.snapshot_rate,
        args.minimum_lookahead,
        args.maximum_lookahead,
        args.sweep_half_width,
        args.threshold_width,
        args.alert_state,
    )
    rospy.loginfo(
        "负障碍 shadow 诊断已启动：不发布、不控制，output=%s",
        args.output,
    )
    rospy.spin()
    recorder.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

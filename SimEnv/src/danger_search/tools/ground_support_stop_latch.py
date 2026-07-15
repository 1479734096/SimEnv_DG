#!/usr/bin/env python3
"""负障碍动态实验专用的命令门控与零速度锁存器。

生产控制器在本实验中把命令发往独立 raw 话题，本进程是实际 ``/cmd_vel``
的唯一发布者。锁存前原样转发，收到新鲜且连续确认的 shadow 告警后忽略所有
后续 raw 命令，只保持零速度，直至机器人完全停止。该工具不进入参赛算法。
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
from threading import Lock

try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - 单元测试不需要 ROS
    rospy = None
    Twist = Odometry = String = None


RAW_COMMAND_TOPIC = "/danger_search/diagnostic_cmd_vel_raw"
OUTPUT_COMMAND_TOPIC = "/cmd_vel"


@dataclass
class StopLatchState:
    minimum_trigger_speed: float
    maximum_alert_age: float
    stop_speed_threshold: float
    stop_hold_seconds: float
    stop_position_tolerance: float
    latched: bool = False
    latched_at: float | None = None
    latch_position: tuple[float, float] | None = None
    latch_speed: float | None = None
    low_speed_since: float | None = None
    low_speed_anchor: tuple[float, float] | None = None

    def consider_alert(
        self,
        *,
        now: float,
        alert: dict[str, object],
        speed: float,
        position: tuple[float, float],
    ) -> bool:
        if self.latched or not bool(alert.get("active", False)):
            return False
        received_at = alert.get("receive_sim_time")
        if not isinstance(received_at, (int, float)):
            return False
        age = float(now) - float(received_at)
        if age < -1e-6 or age > max(0.0, float(self.maximum_alert_age)):
            return False
        if float(speed) < max(0.0, float(self.minimum_trigger_speed)):
            return False
        self.latched = True
        self.latched_at = float(now)
        self.latch_position = (float(position[0]), float(position[1]))
        self.latch_speed = float(speed)
        return True

    def output_command(
        self,
        command: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        if self.latched:
            return (0.0, 0.0, 0.0)
        return tuple(float(value) for value in command)

    def observe_motion(
        self,
        *,
        now: float,
        speed: float,
        position: tuple[float, float],
    ) -> bool:
        if not self.latched:
            return False
        position = (float(position[0]), float(position[1]))
        if float(speed) > max(0.0, float(self.stop_speed_threshold)):
            self.low_speed_since = None
            self.low_speed_anchor = None
            return False
        if self.low_speed_since is None or self.low_speed_anchor is None:
            self.low_speed_since = float(now)
            self.low_speed_anchor = position
            return False
        displacement = math.hypot(
            position[0] - self.low_speed_anchor[0],
            position[1] - self.low_speed_anchor[1],
        )
        if displacement > max(0.0, float(self.stop_position_tolerance)):
            self.low_speed_since = float(now)
            self.low_speed_anchor = position
            return False
        return float(now) - self.low_speed_since >= max(0.0, float(self.stop_hold_seconds))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--alert-state", required=True, type=Path)
    parser.add_argument("--event-log", required=True, type=Path)
    parser.add_argument("--stop-marker", required=True, type=Path)
    parser.add_argument("--raw-command-topic", default=RAW_COMMAND_TOPIC)
    parser.add_argument("--output-command-topic", default=OUTPUT_COMMAND_TOPIC)
    parser.add_argument("--publish-rate", type=float, default=50.0)
    parser.add_argument("--minimum-trigger-speed", type=float, default=0.10)
    parser.add_argument("--maximum-alert-age", type=float, default=0.25)
    parser.add_argument("--stop-speed-threshold", type=float, default=0.02)
    parser.add_argument("--stop-hold-seconds", type=float, default=0.50)
    parser.add_argument("--stop-position-tolerance", type=float, default=0.01)
    return parser.parse_args()


def _atomic_write(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _quaternion_roll_pitch(quaternion) -> tuple[float, float]:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def _rl_command(value: str) -> tuple[float, float, float] | None:
    match = re.search(
        r"cmd=\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)",
        str(value),
    )
    if match is None:
        return None
    return tuple(float(match.group(index)) for index in (1, 2, 3))


class DiagnosticStopLatch:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.state = StopLatchState(
            minimum_trigger_speed=args.minimum_trigger_speed,
            maximum_alert_age=args.maximum_alert_age,
            stop_speed_threshold=args.stop_speed_threshold,
            stop_hold_seconds=args.stop_hold_seconds,
            stop_position_tolerance=args.stop_position_tolerance,
        )
        self._lock = Lock()
        self._odom: dict[str, object] | None = None
        self._last_raw_command = (0.0, 0.0, 0.0)
        self._alert_payload: dict[str, object] | None = None
        self._alert_mtime_ns: int | None = None
        self._first_zero_publish_time: float | None = None
        self._first_rl_zero_time: float | None = None
        self._deceleration_start_time: float | None = None
        self._maximum_abs_roll = 0.0
        self._maximum_abs_pitch = 0.0
        self._raw_forward_count = 0
        self._zero_publish_count = 0
        self._stop_written = False
        args.event_log.parent.mkdir(parents=True, exist_ok=True)
        self._stream = args.event_log.open("w", encoding="utf-8", buffering=1)
        self._publisher = rospy.Publisher(args.output_command_topic, Twist, queue_size=1)
        self._raw_subscriber = rospy.Subscriber(
            args.raw_command_topic,
            Twist,
            self.on_raw_command,
            queue_size=1,
        )
        self._odom_subscriber = rospy.Subscriber(
            "/Odometry_gazebo",
            Odometry,
            self.on_odom,
            queue_size=1,
        )
        self._rl_subscriber = rospy.Subscriber(
            "/junior_ctrl/rl_status",
            String,
            self.on_rl_status,
            queue_size=10,
        )
        self._write_event(
            "start",
            raw_command_topic=args.raw_command_topic,
            output_command_topic=args.output_command_topic,
            non_production_diagnostic=True,
            relays_before_latch=True,
            holds_zero_after_latch=True,
        )
        rospy.Timer(rospy.Duration(1.0 / max(1.0, float(args.publish_rate))), self.tick)
        rospy.on_shutdown(self.close)

    def _write_event(self, event: str, **payload) -> None:
        row = {
            "schema": "ground_support_diagnostic_stop_event_v1",
            "event": event,
            "sim_time": float(rospy.get_time()),
            "wall_time": datetime.now(timezone.utc).isoformat(),
            **payload,
        }
        self._stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")

    def _publish_tuple(self, command: tuple[float, float, float]) -> None:
        message = Twist()
        message.linear.x = float(command[0])
        message.linear.y = float(command[1])
        message.angular.z = float(command[2])
        self._publisher.publish(message)

    def on_raw_command(self, message) -> None:
        command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        with self._lock:
            self._last_raw_command = command
            output = self.state.output_command(command)
            latched = self.state.latched
        self._publish_tuple(output)
        if latched:
            self._zero_publish_count += 1
        else:
            self._raw_forward_count += 1

    def on_odom(self, message) -> None:
        position = message.pose.pose.position
        roll, pitch = _quaternion_roll_pitch(message.pose.pose.orientation)
        speed = math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )
        with self._lock:
            self._odom = {
                "position": (float(position.x), float(position.y), float(position.z)),
                "speed": speed,
                "roll": roll,
                "pitch": pitch,
            }

    def on_rl_status(self, message) -> None:
        command = _rl_command(getattr(message, "data", ""))
        if command is None:
            return
        now = float(rospy.get_time())
        with self._lock:
            if (
                self.state.latched
                and self._first_rl_zero_time is None
                and max(abs(value) for value in command) <= 0.01
            ):
                self._first_rl_zero_time = now
                self._write_event("rl_received_zero", command=list(command))

    def _read_alert_state(self) -> dict[str, object] | None:
        try:
            stat = self.args.alert_state.stat()
        except FileNotFoundError:
            return self._alert_payload
        if stat.st_mtime_ns == self._alert_mtime_ns:
            return self._alert_payload
        try:
            payload = json.loads(self.args.alert_state.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return self._alert_payload
        if not isinstance(payload, dict):
            return self._alert_payload
        self._alert_mtime_ns = stat.st_mtime_ns
        self._alert_payload = payload
        return payload

    def tick(self, _event=None) -> None:
        now = float(rospy.get_time())
        alert = self._read_alert_state()
        with self._lock:
            odom = dict(self._odom) if self._odom is not None else None
            just_latched = False
            if alert is not None and odom is not None:
                position = odom["position"]
                just_latched = self.state.consider_alert(
                    now=now,
                    alert=alert,
                    speed=float(odom["speed"]),
                    position=(float(position[0]), float(position[1])),
                )
            latched = self.state.latched
            if latched and odom is not None:
                self._maximum_abs_roll = max(self._maximum_abs_roll, abs(float(odom["roll"])))
                self._maximum_abs_pitch = max(self._maximum_abs_pitch, abs(float(odom["pitch"])))
                if (
                    self._deceleration_start_time is None
                    and self.state.latch_speed is not None
                    and float(odom["speed"]) <= max(0.0, self.state.latch_speed - 0.02)
                ):
                    self._deceleration_start_time = now
                    self._write_event("odom_deceleration_started", odom_speed=odom["speed"])
                stopped = self.state.observe_motion(
                    now=now,
                    speed=float(odom["speed"]),
                    position=(float(position[0]), float(position[1])),
                )
            else:
                stopped = False

        if just_latched:
            self._write_event(
                "stop_latched",
                alert=alert,
                odom=odom,
                raw_command=list(self._last_raw_command),
            )
        if latched:
            self._publish_tuple((0.0, 0.0, 0.0))
            self._zero_publish_count += 1
            if self._first_zero_publish_time is None:
                self._first_zero_publish_time = now
                self._write_event("first_zero_published")
        if stopped and not self._stop_written and odom is not None:
            self._stop_written = True
            position = odom["position"]
            latch_position = self.state.latch_position or (float(position[0]), float(position[1]))
            payload = {
                "schema": "ground_support_diagnostic_stop_complete_v1",
                "non_production_diagnostic": True,
                "latched_at": self.state.latched_at,
                "alert": alert,
                "first_zero_publish_time": self._first_zero_publish_time,
                "first_rl_zero_time": self._first_rl_zero_time,
                "deceleration_start_time": self._deceleration_start_time,
                "stop_complete_time": now,
                "latch_speed": self.state.latch_speed,
                "latch_xy": list(latch_position),
                "stop_xyz": list(position),
                "stop_displacement": math.hypot(
                    float(position[0]) - latch_position[0],
                    float(position[1]) - latch_position[1],
                ),
                "maximum_abs_roll_after_latch": self._maximum_abs_roll,
                "maximum_abs_pitch_after_latch": self._maximum_abs_pitch,
                "raw_forward_count": self._raw_forward_count,
                "zero_publish_count": self._zero_publish_count,
            }
            _atomic_write(self.args.stop_marker, payload)
            self._write_event("stop_complete", result=payload)

    def close(self) -> None:
        if self._stream.closed:
            return
        self._write_event(
            "shutdown",
            latched=self.state.latched,
            stop_complete=self._stop_written,
            raw_forward_count=self._raw_forward_count,
            zero_publish_count=self._zero_publish_count,
        )
        self._stream.close()


def main() -> int:
    args = parse_args()
    if rospy is None or Twist is None:
        raise SystemExit("当前环境没有 ROS Python 依赖")
    if args.raw_command_topic == args.output_command_topic:
        raise SystemExit("诊断 raw 命令话题不得与实际输出话题相同")
    rospy.init_node("ground_support_diagnostic_stop_latch", anonymous=True)
    DiagnosticStopLatch(args)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

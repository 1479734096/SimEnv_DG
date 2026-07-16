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
    brake_trigger_speed: float | None = None
    latched: bool = False
    latched_at: float | None = None
    latch_position: tuple[float, float] | None = None
    latch_speed: float | None = None
    trigger_source: str | None = None
    low_speed_since: float | None = None
    low_speed_anchor: tuple[float, float] | None = None

    def _latch(
        self,
        *,
        now: float,
        speed: float,
        position: tuple[float, float],
        source: str,
    ) -> bool:
        if self.latched:
            return False
        self.latched = True
        self.latched_at = float(now)
        self.latch_position = (float(position[0]), float(position[1]))
        self.latch_speed = float(speed)
        self.trigger_source = str(source)
        return True

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
        return self._latch(
            now=now,
            speed=speed,
            position=position,
            source="shadow_alert",
        )

    def consider_brake_trigger(
        self,
        *,
        now: float,
        speed: float,
        position: tuple[float, float],
        command: tuple[float, float, float],
    ) -> bool:
        """按实测平移速度触发专用制动采集，不依赖支撑告警。"""
        threshold = self.brake_trigger_speed
        if self.latched or threshold is None:
            return False
        if not math.isfinite(float(speed)) or float(speed) < float(threshold):
            return False
        requested_translation = math.hypot(float(command[0]), float(command[1]))
        if requested_translation < max(0.0, float(self.minimum_trigger_speed)):
            return False
        return self._latch(
            now=now,
            speed=speed,
            position=position,
            source="measured_speed",
        )

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
    parser.add_argument("--alert-state", type=Path)
    parser.add_argument("--brake-trigger-speed", type=float)
    parser.add_argument("--brake-relay-before-trigger", action="store_true")
    parser.add_argument("--brake-command-linear-x", type=float, default=0.60)
    parser.add_argument("--brake-command-linear-y", type=float, default=0.35)
    parser.add_argument("--brake-ready-hold", type=float, default=0.50)
    parser.add_argument("--brake-acceleration-timeout", type=float, default=2.0)
    parser.add_argument("--brake-preturn-radians", type=float, default=0.0)
    parser.add_argument("--brake-preturn-angular", type=float, default=1.0)
    parser.add_argument("--brake-preturn-timeout", type=float, default=5.0)
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


def _quaternion_yaw(quaternion) -> float:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_distance(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(float(a) - float(b)), math.cos(float(a) - float(b))))


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
            brake_trigger_speed=args.brake_trigger_speed,
        )
        self._lock = Lock()
        self._odom: dict[str, object] | None = None
        self._last_raw_command = (0.0, 0.0, 0.0)
        self._alert_payload: dict[str, object] | None = None
        self._alert_mtime_ns: int | None = None
        self._rl_ready_since: float | None = None
        self._brake_preturn_started_at: float | None = None
        self._brake_preturn_start_yaw: float | None = None
        self._brake_command_started_at: float | None = None
        self._pending_latch_event = False
        self._first_zero_publish_time: float | None = None
        self._first_rl_zero_time: float | None = None
        self._deceleration_start_time: float | None = None
        self._maximum_abs_roll = 0.0
        self._maximum_abs_pitch = 0.0
        self._maximum_displacement_after_latch = 0.0
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
            trigger_mode=(
                "measured_speed_relay"
                if args.brake_relay_before_trigger
                else "measured_speed"
                if args.brake_trigger_speed is not None
                else "shadow_alert"
            ),
            brake_trigger_speed=args.brake_trigger_speed,
            brake_relay_before_trigger=args.brake_relay_before_trigger,
            brake_command=(
                [args.brake_command_linear_x, args.brake_command_linear_y, 0.0]
                if args.brake_trigger_speed is not None
                and not args.brake_relay_before_trigger
                else None
            ),
            brake_preturn_radians=args.brake_preturn_radians,
            brake_preturn_angular=args.brake_preturn_angular,
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
        yaw = _quaternion_yaw(message.pose.pose.orientation)
        now = float(rospy.get_time())
        speed = math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )
        just_latched = False
        with self._lock:
            self._odom = {
                "position": (float(position.x), float(position.y), float(position.z)),
                "speed": speed,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
            if (
                self.args.brake_trigger_speed is not None
                and (
                    getattr(self.args, "brake_relay_before_trigger", False)
                    or self._brake_command_started_at is not None
                )
            ):
                just_latched = self.state.consider_brake_trigger(
                    now=now,
                    speed=speed,
                    position=(float(position.x), float(position.y)),
                    command=self._last_raw_command,
                )
                if just_latched:
                    self._pending_latch_event = True
                    self._first_zero_publish_time = now
        if just_latched:
            self._publish_tuple((0.0, 0.0, 0.0))
            self._zero_publish_count += 1
            self._write_event("first_zero_published", source="high_rate_odom")

    def on_rl_status(self, message) -> None:
        value = str(getattr(message, "data", ""))
        command = _rl_command(value)
        ready = all(
            token in value
            for token in ("in_rl=1", "policy_loaded=1", "infer_thread=1")
        )
        now = float(rospy.get_time())
        with self._lock:
            if ready:
                if self._rl_ready_since is None:
                    self._rl_ready_since = now
            else:
                self._rl_ready_since = None
        if command is None:
            return
        with self._lock:
            if (
                self.state.latched
                and self._first_rl_zero_time is None
                and max(abs(value) for value in command) <= 0.01
            ):
                self._first_rl_zero_time = now
                self._write_event("rl_received_zero", command=list(command))

    def _read_alert_state(self) -> dict[str, object] | None:
        if self.args.alert_state is None:
            return None
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
            just_latched = self._pending_latch_event
            self._pending_latch_event = False
            brake_command_active = False
            if alert is not None and odom is not None:
                position = odom["position"]
                just_latched = self.state.consider_alert(
                    now=now,
                    alert=alert,
                    speed=float(odom["speed"]),
                    position=(float(position[0]), float(position[1])),
                )
            brake_ready = (
                self.args.brake_trigger_speed is not None
                and not getattr(self.args, "brake_relay_before_trigger", False)
                and self._rl_ready_since is not None
                and now - self._rl_ready_since
                >= max(0.0, float(self.args.brake_ready_hold))
            )
            if (
                brake_ready
                and odom is not None
                and self._brake_command_started_at is None
                and self._brake_preturn_started_at is None
            ):
                if abs(float(self.args.brake_preturn_radians)) > 1e-6:
                    self._brake_preturn_started_at = now
                    self._brake_preturn_start_yaw = float(odom["yaw"])
                    self._last_raw_command = (
                        0.0,
                        0.0,
                        math.copysign(
                            abs(float(self.args.brake_preturn_angular)),
                            float(self.args.brake_preturn_radians),
                        ),
                    )
                    self._write_event(
                        "diagnostic_brake_preturn_started",
                        command=list(self._last_raw_command),
                        start_yaw=self._brake_preturn_start_yaw,
                    )
                else:
                    self._brake_command_started_at = now
                    self._last_raw_command = (
                        float(self.args.brake_command_linear_x),
                        float(self.args.brake_command_linear_y),
                        0.0,
                    )
                    self._write_event(
                        "diagnostic_brake_command_started",
                        command=list(self._last_raw_command),
                    )
            if (
                odom is not None
                and self._brake_preturn_started_at is not None
                and self._brake_command_started_at is None
                and not self.state.latched
            ):
                progress = _angle_distance(
                    float(odom["yaw"]),
                    float(self._brake_preturn_start_yaw),
                )
                if progress >= abs(float(self.args.brake_preturn_radians)):
                    self._brake_command_started_at = now
                    self._last_raw_command = (
                        float(self.args.brake_command_linear_x),
                        float(self.args.brake_command_linear_y),
                        0.0,
                    )
                    self._write_event(
                        "diagnostic_brake_command_started",
                        command=list(self._last_raw_command),
                        preturn_progress=progress,
                        current_yaw=float(odom["yaw"]),
                    )
                elif (
                    now - self._brake_preturn_started_at
                    >= max(0.0, float(self.args.brake_preturn_timeout))
                ):
                    position = odom["position"]
                    just_latched = self.state._latch(
                        now=now,
                        speed=float(odom["speed"]),
                        position=(float(position[0]), float(position[1])),
                        source="brake_preturn_timeout",
                    )
            if (
                odom is not None
                and self.args.brake_trigger_speed is not None
                and self._brake_command_started_at is not None
            ):
                position = odom["position"]
                just_latched = self.state.consider_brake_trigger(
                    now=now,
                    speed=float(odom["speed"]),
                    position=(float(position[0]), float(position[1])),
                    command=self._last_raw_command,
                ) or just_latched
                if (
                    not self.state.latched
                    and now - self._brake_command_started_at
                    >= max(0.0, float(self.args.brake_acceleration_timeout))
                ):
                    just_latched = self.state._latch(
                        now=now,
                        speed=float(odom["speed"]),
                        position=(float(position[0]), float(position[1])),
                        source="brake_trigger_timeout",
                    )
            latched = self.state.latched
            brake_command_active = (
                self.args.brake_trigger_speed is not None
                and not getattr(self.args, "brake_relay_before_trigger", False)
                and (
                    self._brake_command_started_at is not None
                    or self._brake_preturn_started_at is not None
                )
                and not latched
            )
            if latched and odom is not None:
                position = odom["position"]
                latch_position = self.state.latch_position
                if latch_position is not None:
                    self._maximum_displacement_after_latch = max(
                        self._maximum_displacement_after_latch,
                        math.hypot(
                            float(position[0]) - latch_position[0],
                            float(position[1]) - latch_position[1],
                        ),
                    )
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
                trigger_source=self.state.trigger_source,
                alert=alert,
                odom=odom,
                raw_command=list(self._last_raw_command),
            )
        if brake_command_active:
            self._publish_tuple(self._last_raw_command)
            self._raw_forward_count += 1
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
                "trigger_source": self.state.trigger_source,
                "valid_speed_trigger": self.state.trigger_source == "measured_speed",
                "alert": alert,
                "first_zero_publish_time": self._first_zero_publish_time,
                "first_rl_zero_time": self._first_rl_zero_time,
                "deceleration_start_time": self._deceleration_start_time,
                "stop_complete_time": now,
                "first_stopped_time": self.state.low_speed_since,
                "latch_speed": self.state.latch_speed,
                "brake_command": (
                    list(self._last_raw_command)
                    if self.args.brake_trigger_speed is not None
                    else None
                ),
                "latch_xy": list(latch_position),
                "first_stopped_xy": list(self.state.low_speed_anchor or position[:2]),
                "stop_xyz": list(position),
                "stop_displacement": math.hypot(
                    float(position[0]) - latch_position[0],
                    float(position[1]) - latch_position[1],
                ),
                "maximum_displacement_after_latch": self._maximum_displacement_after_latch,
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
    if (args.alert_state is None) == (args.brake_trigger_speed is None):
        raise SystemExit("必须且只能选择 shadow 告警或实测速度制动触发")
    if args.brake_trigger_speed is not None and args.brake_trigger_speed <= 0.0:
        raise SystemExit("实测速度制动阈值必须为正数")
    if args.brake_relay_before_trigger and args.brake_trigger_speed is None:
        raise SystemExit("生产命令中继制动必须配置实测速度阈值")
    if args.brake_trigger_speed is not None:
        if (
            not args.brake_relay_before_trigger
            and math.hypot(args.brake_command_linear_x, args.brake_command_linear_y) <= 0.0
        ):
            raise SystemExit("专用制动采集命令必须包含平移分量")
        if not args.brake_relay_before_trigger and args.brake_acceleration_timeout <= 0.0:
            raise SystemExit("专用制动加速超时必须为正数")
        if args.brake_relay_before_trigger and abs(args.brake_preturn_radians) > 1e-6:
            raise SystemExit("生产命令中继制动不能配置诊断预转向")
        if abs(args.brake_preturn_radians) > math.pi:
            raise SystemExit("专用制动预转角必须位于 [-pi, pi]")
        if args.brake_preturn_radians and args.brake_preturn_angular <= 0.0:
            raise SystemExit("专用制动预转角速度必须为正数")
        if args.brake_preturn_timeout <= 0.0:
            raise SystemExit("专用制动预转超时必须为正数")
    rospy.init_node("ground_support_diagnostic_stop_latch", anonymous=True)
    DiagnosticStopLatch(args)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

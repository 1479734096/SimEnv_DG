#!/usr/bin/env python3
"""负障碍诊断专用的 kill-only 安全边界。

该工具只订阅状态与里程计，在满足显式测试边界时向所属 runner 发送
SIGINT。它不发布 ROS 消息，也不进入参赛算法进程。
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import signal
from threading import Lock

try:
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - 离线单元测试不需要 ROS
    rospy = None
    Odometry = String = None


@dataclass(frozen=True)
class KillOnlyBoundary:
    """全部已配置条件同时满足时才触发。"""

    floor: int | None = None
    min_x: float | None = None
    max_x: float | None = None
    min_y: float | None = None
    max_y: float | None = None
    min_speed: float | None = None
    min_world_x_velocity: float | None = None

    def configured(self) -> bool:
        return any(value is not None for value in asdict(self).values())

    def matches(
        self,
        *,
        floor: int | None,
        x: float,
        y: float,
        speed: float = 0.0,
        world_x_velocity: float = 0.0,
    ) -> bool:
        if self.floor is not None and floor != self.floor:
            return False
        if self.min_x is not None and x < self.min_x:
            return False
        if self.max_x is not None and x > self.max_x:
            return False
        if self.min_y is not None and y < self.min_y:
            return False
        if self.max_y is not None and y > self.max_y:
            return False
        if self.min_speed is not None and speed < self.min_speed:
            return False
        if (
            self.min_world_x_velocity is not None
            and world_x_velocity < self.min_world_x_velocity
        ):
            return False
        return self.configured()


@dataclass(frozen=True)
class PostureLimits:
    """任一已配置姿态危险条件成立时触发。"""

    minimum_z: float | None = None
    maximum_abs_roll: float | None = None
    maximum_abs_pitch: float | None = None
    active_after_sim: float = 0.0

    def configured(self) -> bool:
        return any(
            value is not None
            for key, value in asdict(self).items()
            if key != "active_after_sim"
        )

    def violation(
        self,
        *,
        z: float,
        roll: float,
        pitch: float,
        sim_time: float = math.inf,
    ) -> str | None:
        if sim_time < self.active_after_sim:
            return None
        if self.minimum_z is not None and z <= self.minimum_z:
            return "minimum_z"
        if self.maximum_abs_roll is not None and abs(roll) >= self.maximum_abs_roll:
            return "maximum_abs_roll"
        if self.maximum_abs_pitch is not None and abs(pitch) >= self.maximum_abs_pitch:
            return "maximum_abs_pitch"
        return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner-pid", required=True, type=int)
    parser.add_argument("--marker", required=True, type=Path)
    parser.add_argument("--kill-sim-time", type=float)
    parser.add_argument("--floor", type=int)
    parser.add_argument("--min-x", type=float)
    parser.add_argument("--max-x", type=float)
    parser.add_argument("--min-y", type=float)
    parser.add_argument("--max-y", type=float)
    parser.add_argument("--min-speed", type=float)
    parser.add_argument("--min-world-x-velocity", type=float)
    parser.add_argument("--minimum-z", type=float)
    parser.add_argument("--maximum-abs-roll", type=float)
    parser.add_argument("--maximum-abs-pitch", type=float)
    parser.add_argument("--posture-active-after-sim", type=float, default=0.0)
    return parser.parse_args()


class KillOnlyGuard:
    def __init__(
        self,
        *,
        runner_pid: int,
        marker: Path,
        kill_sim_time: float | None,
        boundary: KillOnlyBoundary,
        posture_limits: PostureLimits | None = None,
    ):
        self.runner_pid = int(runner_pid)
        self.marker = marker
        self.kill_sim_time = kill_sim_time
        self.boundary = boundary
        self.posture_limits = posture_limits or PostureLimits()
        self._lock = Lock()
        self._floor: int | None = None
        self._floor_observed_at: float | None = None
        self._odom: tuple[float, float, float, float, float, float, float, float, float] | None = None
        self._triggered = False

    def on_status(self, message) -> None:
        try:
            payload = json.loads(message.data)
            floor = int(payload["floor"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            self._floor = floor
            self._floor_observed_at = float(rospy.get_time())

    def on_odom(self, message) -> None:
        position = message.pose.pose.position
        roll, pitch = _quaternion_roll_pitch(message.pose.pose.orientation)
        stamp = _message_stamp(message)
        with self._lock:
            self._odom = (
                float(position.x),
                float(position.y),
                float(position.z),
                stamp,
                math.hypot(
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.linear.y),
                ),
                float(message.twist.twist.linear.x),
                float(message.twist.twist.linear.y),
                roll,
                pitch,
            )

    def check(self, _event=None) -> None:
        now = float(rospy.get_time())
        with self._lock:
            if self._triggered:
                return
            floor = self._floor
            floor_observed_at = self._floor_observed_at
            odom = self._odom

        reason = None
        details: dict[str, object] = {}
        if self.kill_sim_time is not None and now >= self.kill_sim_time:
            reason = "kill_only_sim_time"
            details["configured_sim_time"] = self.kill_sim_time
        elif odom is not None:
            x, y, z, odom_stamp, speed, world_vx, world_vy, roll, pitch = odom
            floor_is_fresh = (
                floor_observed_at is not None and now - floor_observed_at <= 2.0
            )
            odom_is_fresh = now - odom_stamp <= 0.25
            posture_violation = self.posture_limits.violation(
                z=z,
                roll=roll,
                pitch=pitch,
                sim_time=now,
            )
            if odom_is_fresh and posture_violation is not None:
                reason = "kill_only_posture_limit"
                details.update(
                    {
                        "posture_limits": asdict(self.posture_limits),
                        "posture_violation": posture_violation,
                        "floor": floor,
                        "odom_xyz": [x, y, z],
                        "odom_rpy": [roll, pitch, None],
                        "odom_stamp": odom_stamp,
                        "odom_linear_speed": speed,
                        "odom_world_velocity_xy": [world_vx, world_vy],
                    }
                )
            elif floor_is_fresh and odom_is_fresh and self.boundary.matches(
                floor=floor,
                x=x,
                y=y,
                speed=speed,
                world_x_velocity=world_vx,
            ):
                reason = "kill_only_spatial_boundary"
                details.update(
                    {
                        "boundary": asdict(self.boundary),
                        "floor": floor,
                        "odom_xyz": [x, y, z],
                        "odom_stamp": odom_stamp,
                        "odom_linear_speed": speed,
                        "odom_world_velocity_xy": [world_vx, world_vy],
                        "odom_rpy": [roll, pitch, None],
                    }
                )
        if reason is None:
            return

        with self._lock:
            if self._triggered:
                return
            self._triggered = True
        payload = {
            "schema": "ground_support_kill_only_guard_v1",
            "reason": reason,
            "sim_time": now,
            "runner_pid": self.runner_pid,
            "publishes_control": False,
            **details,
        }
        _write_marker(self.marker, payload)
        try:
            os.kill(self.runner_pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        rospy.signal_shutdown(reason)


def _message_stamp(message) -> float:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    to_sec = getattr(stamp, "to_sec", None)
    return float(to_sec()) if callable(to_sec) else float(rospy.get_time())


def _quaternion_roll_pitch(quaternion) -> tuple[float, float]:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def _write_marker(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main() -> int:
    args = parse_args()
    boundary = KillOnlyBoundary(
        floor=args.floor,
        min_x=args.min_x,
        max_x=args.max_x,
        min_y=args.min_y,
        max_y=args.max_y,
        min_speed=args.min_speed,
        min_world_x_velocity=args.min_world_x_velocity,
    )
    posture_limits = PostureLimits(
        minimum_z=args.minimum_z,
        maximum_abs_roll=args.maximum_abs_roll,
        maximum_abs_pitch=args.maximum_abs_pitch,
        active_after_sim=args.posture_active_after_sim,
    )
    if args.kill_sim_time is None and not boundary.configured() and not posture_limits.configured():
        raise SystemExit("必须配置 --kill-sim-time、空间边界或姿态上限")
    if args.runner_pid <= 1:
        raise SystemExit("拒绝向无效 runner PID 发送信号")
    if rospy is None:
        raise SystemExit("当前环境没有 rospy")

    rospy.init_node("ground_support_kill_only_guard", anonymous=True)
    guard = KillOnlyGuard(
        runner_pid=args.runner_pid,
        marker=args.marker,
        kill_sim_time=args.kill_sim_time,
        boundary=boundary,
        posture_limits=posture_limits,
    )
    rospy.Subscriber("/danger_search/status", String, guard.on_status, queue_size=1)
    rospy.Subscriber("/Odometry_gazebo", Odometry, guard.on_odom, queue_size=1)
    rospy.Timer(rospy.Duration(0.02), guard.check)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

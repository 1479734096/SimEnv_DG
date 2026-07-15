#!/usr/bin/env python3
"""负障碍动态停车实验专用的两段手动目标发布器。

该工具只向正式支持的 ``/nav/goal`` 发布两个诊断目标：先到安全预备点，再沿
正常 Planner/PathFollower/RL 链路接近观测区域。坐标必须由本次离线安全标注
显式传入，不会被参赛算法读取，也不会发布速度或修改 Gazebo 状态。
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from threading import Lock
import time

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - 单元测试不需要 ROS
    rospy = None
    PoseStamped = Odometry = String = None


class DynamicApproachGoal:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self._lock = Lock()
        self._status: dict[str, object] | None = None
        self._odom: dict[str, float] | None = None
        self._events: list[dict[str, object]] = []
        self._publisher = rospy.Publisher("/nav/goal", PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber("/danger_search/status", String, self._on_status, queue_size=1)
        rospy.Subscriber("/Odometry_gazebo", Odometry, self._on_odom, queue_size=1)

    def _on_status(self, message) -> None:
        try:
            payload = json.loads(message.data)
        except (AttributeError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            with self._lock:
                self._status = payload

    def _on_odom(self, message) -> None:
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        with self._lock:
            self._odom = {
                "x": float(position.x),
                "y": float(position.y),
                "z": float(position.z),
                "speed": math.hypot(float(velocity.x), float(velocity.y)),
                "sim_time": float(rospy.get_time()),
            }

    def _snapshot(self) -> tuple[dict[str, object] | None, dict[str, float] | None]:
        with self._lock:
            return (
                dict(self._status) if self._status is not None else None,
                dict(self._odom) if self._odom is not None else None,
            )

    def _record(self, event: str, **payload) -> None:
        self._events.append(
            {
                "event": event,
                "sim_time": float(rospy.get_time()),
                "wall_time": datetime.now(timezone.utc).isoformat(),
                **payload,
            }
        )

    def _wait_until(self, predicate, *, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            status, odom = self._snapshot()
            if predicate(status, odom):
                return True
            time.sleep(0.05)
        return False

    def _publish_goal(self, xy: tuple[float, float], label: str) -> None:
        message = PoseStamped()
        message.header.frame_id = "world"
        message.header.stamp = rospy.Time.now()
        message.pose.position.x = float(xy[0])
        message.pose.position.y = float(xy[1])
        message.pose.orientation.w = 1.0
        self._publisher.publish(message)
        self._record("goal_published", label=label, target_xy=list(xy))

    def _wait_arrival(self, target: tuple[float, float]) -> bool:
        return self._wait_until(
            lambda _status, odom: (
                odom is not None
                and math.hypot(odom["x"] - target[0], odom["y"] - target[1])
                <= self.args.arrival_tolerance
                and odom["speed"] <= self.args.arrival_speed
            ),
            timeout=self.args.goal_timeout,
        )

    def run(self) -> int:
        ready = self._wait_until(
            lambda status, odom: (
                status is not None
                and odom is not None
                and int(status.get("floor", -1)) == self.args.floor
                and not str(status.get("current_task") or "").startswith("TakeElevator")
            ),
            timeout=self.args.floor_timeout,
        )
        if not ready:
            self._record("failed", reason="floor_or_elevator_wait_timeout")
            self._write()
            return 1

        staging = tuple(self.args.staging)
        approach = tuple(self.args.approach)
        self._publish_goal(staging, "staging")
        if not self._wait_arrival(staging):
            self._record("failed", reason="staging_goal_timeout", snapshot=self._snapshot())
            self._write()
            return 1
        self._record("staging_arrived", snapshot=self._snapshot())
        time.sleep(max(0.0, self.args.staging_hold))

        self._publish_goal(approach, "dynamic_approach")
        completed = self._wait_until(
            lambda _status, _odom: self.args.stop_marker.is_file()
            or self.args.hard_kill_marker.is_file(),
            timeout=self.args.approach_timeout,
        )
        if completed:
            reason = (
                "diagnostic_stop_complete"
                if self.args.stop_marker.is_file()
                else "hard_kill_guard"
            )
            self._record("finished", reason=reason, snapshot=self._snapshot())
            self._write()
            return 0 if reason == "diagnostic_stop_complete" else 2
        self._record("failed", reason="approach_timeout", snapshot=self._snapshot())
        self._write()
        return 1

    def _write(self) -> None:
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        self.args.output.write_text(
            json.dumps(
                {
                    "schema": "ground_support_dynamic_goal_v1",
                    "non_production_diagnostic": True,
                    "publishes_velocity": False,
                    "events": self._events,
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--stop-marker", required=True, type=Path)
    parser.add_argument("--hard-kill-marker", required=True, type=Path)
    parser.add_argument("--floor", type=int, default=1)
    parser.add_argument("--staging", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--approach", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--floor-timeout", type=float, default=900.0)
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--approach-timeout", type=float, default=120.0)
    parser.add_argument("--arrival-tolerance", type=float, default=0.42)
    parser.add_argument("--arrival-speed", type=float, default=0.08)
    parser.add_argument("--staging-hold", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if rospy is None:
        raise SystemExit("当前环境没有 ROS Python 依赖")
    rospy.init_node("ground_support_dynamic_goal", anonymous=True)
    return DynamicApproachGoal(args).run()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""电梯轿厢内多朝向扫描目标发布器（任务一专用诊断）。

只发布 /nav/goal，不修改生产算法。所有目标点严格在轿厢安全区域内。
等待机器人到达楼层 1 并在轿厢上后，依次发送不同位置目标以改变朝向。
每个朝向稳定后保持若干秒，使 Livox 采集有效帧。

边界约束：
- 轿厢 x∈[1.65, 4.05], y∈[1.25, 3.95]
- 目标点限制在 x∈[2.1, 3.3], y∈[1.7, 3.3]（远离出口 x=1.65）
- 每次发送前校验机器人当前位置仍在轿厢内
- 不发布速度，不修改路径，不扶正机器人
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from threading import Lock

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:
    rospy = None
    PoseStamped = Odometry = String = None


CABIN_X_MIN = 1.65
CABIN_X_MAX = 4.05
CABIN_Y_MIN = 1.25
CABIN_Y_MAX = 3.95
GOAL_X_MIN = 2.1
GOAL_X_MAX = 3.3
GOAL_Y_MIN = 1.7
GOAL_Y_MAX = 3.3

GOAL_SEQUENCE = [
    (3.3, 2.6, "+X", "背离空带，朝电梯内部"),
    (2.6, 3.3, "+Y", "朝楼层1楼板/走廊方向"),
    (2.1, 2.6, "-X", "朝空带方向（关键朝向）"),
    (2.6, 1.7, "-Y", "朝电梯入口方向"),
    (2.1, 2.6, "-X", "重复朝空带方向（稳定性）"),
    (3.3, 2.6, "+X", "重复背离空带（基线）"),
]


def _quaternion_yaw(q) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class CabinScanGoalPublisher:
    def __init__(self, output: Path, dwell_seconds: float, goal_timeout: float):
        self.output = output
        self.dwell_seconds = dwell_seconds
        self.goal_timeout = goal_timeout
        self._lock = Lock()
        self._floor = None
        self._odom = None
        self._log_entries = []
        self._goal_pub = rospy.Publisher("/nav/goal", PoseStamped, queue_size=1, latch=True)
        self._status_sub = rospy.Subscriber(
            "/danger_search/status", String, self._on_status, queue_size=1)
        self._odom_sub = rospy.Subscriber(
            "/Odometry_gazebo", Odometry, self._on_odom, queue_size=1)

    def _on_status(self, msg):
        try:
            payload = json.loads(msg.data)
            floor = int(payload["floor"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            self._floor = floor

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = _quaternion_yaw(msg.pose.pose.orientation)
        speed = math.hypot(float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y))
        with self._lock:
            self._odom = {
                "x": float(p.x), "y": float(p.y), "z": float(p.z),
                "yaw": yaw, "speed": speed,
                "stamp": float(rospy.get_time()),
            }

    def _current_state(self):
        with self._lock:
            return self._floor, self._odom

    def _in_cabin(self, x, y):
        return CABIN_X_MIN <= x <= CABIN_X_MAX and CABIN_Y_MIN <= y <= CABIN_Y_MAX

    def _wait_for_floor1_on_cabin(self, timeout: float) -> bool:
        rate = rospy.Rate(2.0)
        start = rospy.get_time()
        rospy.loginfo("等待机器人到达楼层1并在轿厢上...")
        while not rospy.is_shutdown():
            floor, odom = self._current_state()
            if odom is not None and floor == 1 and self._in_cabin(odom["x"], odom["y"]):
                rospy.loginfo("已在楼层1轿厢上: (%.2f, %.2f, %.2f) yaw=%.2f", odom["x"], odom["y"], odom["z"], odom["yaw"])
                return True
            if rospy.get_time() - start > timeout:
                if floor is not None:
                    rospy.logwarn("超时: 当前楼层=%s, 位置=%s", floor, odom)
                else:
                    rospy.logwarn("超时: 未收到楼层状态")
                return False
            rate.sleep()
        return False

    def _send_goal(self, x, y, label, description):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        sim_time = float(rospy.get_time())
        rospy.loginfo("发送目标 (%.2f, %.2f) [%s]: %s (sim=%.1f)", x, y, label, description, sim_time)

    def _wait_for_arrival(self, target_x, target_y) -> dict:
        rate = rospy.Rate(5.0)
        start = rospy.get_time()
        while not rospy.is_shutdown():
            _, odom = self._current_state()
            if odom is not None:
                dist = math.hypot(odom["x"] - target_x, odom["y"] - target_y)
                if dist < 0.25 and odom["speed"] < 0.08:
                    return {"arrived": True, "dist": dist, "odom": odom, "elapsed": rospy.get_time() - start}
            if rospy.get_time() - start > self.goal_timeout:
                _, odom = self._current_state()
                return {"arrived": False, "dist": dist if odom else None, "odom": odom, "elapsed": self.goal_timeout}
            rate.sleep()
        return {"arrived": False, "dist": None, "odom": None, "elapsed": 0}

    def _dwell(self, label):
        rate = rospy.Rate(2.0)
        start = rospy.get_time()
        frames_seen = 0
        yaw_samples = []
        while not rospy.is_shutdown() and rospy.get_time() - start < self.dwell_seconds:
            _, odom = self._current_state()
            if odom is not None:
                yaw_samples.append(odom["yaw"])
            rate.sleep()
        _, odom = self._current_state()
        yaw_median = None
        if yaw_samples:
            yaw_sorted = sorted(yaw_samples)
            yaw_median = yaw_sorted[len(yaw_sorted) // 2]
        entry = {
            "label": label,
            "sim_time": float(rospy.get_time()),
            "dwell_seconds": self.dwell_seconds,
            "yaw_median": yaw_median,
            "yaw_samples_count": len(yaw_samples),
            "odom_at_dwell_end": odom,
        }
        self._log_entries.append(entry)
        rospy.loginfo("朝向 %s 采集完成: yaw=%.2f (%.0f°)", label, yaw_median or 0, math.degrees(yaw_median or 0))

    def run(self) -> int:
        if not self._wait_for_floor1_on_cabin(timeout=300.0):
            self._write_log({"error": "timeout_waiting_for_floor1_cabin", "entries": self._log_entries})
            return 1

        for i, (gx, gy, label, desc) in enumerate(GOAL_SEQUENCE):
            if rospy.is_shutdown():
                break
            _, odom = self._current_state()
            if odom is not None and not self._in_cabin(odom["x"], odom["y"]):
                rospy.logwarn("机器人已离开轿厢 (%.2f, %.2f)，停止发送目标", odom["x"], odom["y"])
                break
            self._send_goal(gx, gy, label, desc)
            arrival = self._wait_for_arrival(gx, gy)
            rospy.loginfo("目标 %s: arrived=%s dist=%.2f elapsed=%.1f", label, arrival["arrived"], arrival.get("dist", 0) or 0, arrival["elapsed"])
            if arrival["arrived"]:
                self._dwell(label)
            else:
                rospy.logwarn("目标 %s 未到达，仍采集当前朝向", label)
                self._dwell(label + "_partial")
            if i < len(GOAL_SEQUENCE) - 1:
                rospy.sleep(1.0)

        self._write_log({"schema": "cabin_scan_goals_v1", "entries": self._log_entries})
        rospy.loginfo("轿厢多朝向扫描完成，共 %d 个朝向", len(self._log_entries))
        return 0

    def _write_log(self, payload):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        rospy.loginfo("目标日志写出: %s", self.output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dwell-seconds", type=float, default=4.0)
    parser.add_argument("--goal-timeout", type=float, default=25.0)
    args = parser.parse_args()
    if rospy is None:
        raise SystemExit("当前环境没有 rospy")
    rospy.init_node("cabin_scan_goals", anonymous=True, disable_signals=True)
    publisher = CabinScanGoalPublisher(args.output, args.dwell_seconds, args.goal_timeout)
    raise SystemExit(publisher.run())


if __name__ == "__main__":
    main()

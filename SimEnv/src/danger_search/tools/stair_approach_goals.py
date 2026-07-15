#!/usr/bin/env python3
"""楼梯接近与通过诊断目标发布器（任务三专用）。

只发布 /nav/goal，不修改生产算法。分两阶段：
1. 引导到楼梯前安全预备点，确认姿态/点云/shadow 正常；
2. 指向楼梯上方合法目标，让生产控制链自行接近和尝试通过。

若机器人在第一段台阶持续无有效位移，立即停止并报告阻断。
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


STAIR_ENTRANCE_FLOOR0 = (-2.55, 1.55)
STAIR_BOUNDS_X = (-4.85, -1.65)
STAIR_BOUNDS_Y = (0.85, 6.85)

STAGING_GOAL = (-1.8, 1.0)
STAIR_TARGET_GOAL = (-2.5, 4.0)

NO_DISPLACEMENT_TIMEOUT = 15.0
NO_DISPLACEMENT_THRESHOLD = 0.03
STAGING_DWELL = 4.0


def _quaternion_rpy(q):
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class StairApproachPublisher:
    def __init__(self, output: Path):
        self.output = output
        self._lock = Lock()
        self._floor = None
        self._odom = None
        self._log = []
        self._goal_pub = rospy.Publisher("/nav/goal", PoseStamped, queue_size=1, latch=True)
        self._status_sub = rospy.Subscriber("/danger_search/status", String, self._on_status, queue_size=1)
        self._odom_sub = rospy.Subscriber("/Odometry_gazebo", Odometry, self._on_odom, queue_size=1)

    def _on_status(self, msg):
        try:
            payload = json.loads(msg.data)
            self._floor = int(payload["floor"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        roll, pitch, yaw = _quaternion_rpy(msg.pose.pose.orientation)
        speed = math.hypot(float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y))
        with self._lock:
            self._odom = {
                "x": float(p.x), "y": float(p.y), "z": float(p.z),
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "speed": speed, "stamp": float(rospy.get_time()),
            }

    def _state(self):
        with self._lock:
            return self._floor, self._odom

    def _send_goal(self, x, y, label):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        sim_t = float(rospy.get_time())
        rospy.loginfo("发送目标 (%.2f, %.2f) [%s] sim=%.1f", x, y, label, sim_t)
        self._log.append({"event": "send_goal", "label": label, "x": x, "y": y, "sim_time": sim_t})

    def _wait_for_robot_ready(self, timeout=60.0):
        rate = rospy.Rate(2.0)
        start = rospy.get_time()
        while not rospy.is_shutdown():
            floor, odom = self._state()
            if odom is not None and odom["speed"] < 0.1 and floor is not None:
                return True
            if rospy.get_time() - start > timeout:
                return False
            rate.sleep()
        return False

    def _wait_for_arrival(self, tx, ty, timeout=30.0):
        rate = rospy.Rate(5.0)
        start = rospy.get_time()
        while not rospy.is_shutdown():
            _, odom = self._state()
            if odom is not None:
                dist = math.hypot(odom["x"] - tx, odom["y"] - ty)
                if dist < 0.3 and odom["speed"] < 0.08:
                    return {"arrived": True, "dist": dist, "odom": dict(odom), "elapsed": rospy.get_time() - start}
            if rospy.get_time() - start > timeout:
                _, odom = self._state()
                return {"arrived": False, "dist": dist if odom else None, "odom": dict(odom) if odom else None, "elapsed": timeout}
            rate.sleep()
        return {"arrived": False, "dist": None, "odom": None, "elapsed": 0}

    def _monitor_stair_attempt(self, tx, ty, max_duration=60.0):
        rate = rospy.Rate(2.0)
        start = rospy.get_time()
        _, odom0 = self._state()
        if odom0 is None:
            return {"error": "no_odom"}
        start_xy = (odom0["x"], odom0["y"])
        last_progress_time = start
        last_xy = start_xy
        samples = []
        no_displacement_duration = 0.0
        while not rospy.is_shutdown():
            _, odom = self._state()
            if odom is None:
                rate.sleep()
                continue
            now = rospy.get_time()
            dist_from_start = math.hypot(odom["x"] - start_xy[0], odom["y"] - start_xy[1])
            dist_to_target = math.hypot(odom["x"] - tx, odom["y"] - ty)
            moved = math.hypot(odom["x"] - last_xy[0], odom["y"] - last_xy[1])
            if moved > NO_DISPLACEMENT_THRESHOLD:
                last_progress_time = now
                last_xy = (odom["x"], odom["y"])
                no_displacement_duration = 0.0
            else:
                no_displacement_duration = now - last_progress_time

            samples.append({
                "sim_time": now,
                "x": odom["x"], "y": odom["y"], "z": odom["z"],
                "roll": odom["roll"], "pitch": odom["pitch"], "yaw": odom["yaw"],
                "speed": odom["speed"],
                "dist_from_start": dist_from_start,
                "dist_to_target": dist_to_target,
                "no_displacement_duration": no_displacement_duration,
            })

            if dist_to_target < 0.3 and odom["speed"] < 0.08:
                return {"outcome": "reached_target", "samples": samples,
                        "final_dist_from_start": dist_from_start, "elapsed": now - start}

            if no_displacement_duration >= NO_DISPLACEMENT_TIMEOUT:
                return {"outcome": "no_displacement_timeout", "samples": samples,
                        "final_dist_from_start": dist_from_start, "elapsed": now - start,
                        "no_displacement_duration": no_displacement_duration}

            if now - start > max_duration:
                return {"outcome": "max_duration", "samples": samples,
                        "final_dist_from_start": dist_from_start, "elapsed": now - start}

            rate.sleep()
        return {"outcome": "interrupted", "samples": samples,
                "final_dist_from_start": 0, "elapsed": 0}

    def run(self) -> int:
        if not self._wait_for_robot_ready(timeout=60.0):
            self._write({"error": "timeout_waiting_for_robot_ready", "log": self._log})
            return 1

        floor, odom = self._state()
        rospy.loginfo("机器人就绪: floor=%s pos=(%.2f,%.2f,%.2f) speed=%.3f", floor, odom["x"], odom["y"], odom["z"], odom["speed"])

        # 阶段 1：引导到安全预备点
        self._send_goal(STAGING_GOAL[0], STAGING_GOAL[1], "staging")
        arrival = self._wait_for_arrival(STAGING_GOAL[0], STAGING_GOAL[1], timeout=40.0)
        rospy.loginfo("预备点: arrived=%s dist=%.2f elapsed=%.1f", arrival["arrived"], arrival.get("dist", 0) or 0, arrival["elapsed"])
        self._log.append({"event": "staging_arrival", **arrival})

        if not arrival["arrived"]:
            rospy.logwarn("未到达预备点，仍尝试楼梯目标")
        else:
            rospy.loginfo("预备点确认: roll=%.3f pitch=%.3f z=%.3f speed=%.3f", arrival["odom"]["roll"], arrival["odom"]["pitch"], arrival["odom"]["z"], arrival["odom"]["speed"])
            self._log.append({"event": "staging_confirmed", "posture": {"roll": arrival["odom"]["roll"], "pitch": arrival["odom"]["pitch"], "z": arrival["odom"]["z"]}})
            rospy.sleep(STAGING_DWELL)

        # 阶段 2：指向楼梯上方合法目标，让生产控制链自行接近
        self._send_goal(STAIR_TARGET_GOAL[0], STAIR_TARGET_GOAL[1], "stair_target")
        result = self._monitor_stair_attempt(STAIR_TARGET_GOAL[0], STAIR_TARGET_GOAL[1], max_duration=60.0)
        rospy.loginfo("楼梯尝试结果: %s dist_from_start=%.2f elapsed=%.1f", result["outcome"], result.get("final_dist_from_start", 0), result.get("elapsed", 0))
        self._log.append({"event": "stair_attempt_result", **result})

        self._write({"schema": "stair_approach_v1", "log": self._log, "outcome": result["outcome"]})
        rospy.loginfo("楼梯诊断完成: %s", result["outcome"])
        return 0

    def _write(self, payload):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        rospy.loginfo("楼梯诊断日志写出: %s", self.output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if rospy is None:
        raise SystemExit("当前环境没有 rospy")
    rospy.init_node("stair_approach_goals", anonymous=True, disable_signals=True)
    publisher = StairApproachPublisher(args.output)
    raise SystemExit(publisher.run())


if __name__ == "__main__":
    main()

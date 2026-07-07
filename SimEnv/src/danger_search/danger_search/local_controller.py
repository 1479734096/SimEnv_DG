"""局部路径跟踪控制器。"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:  # pragma: no cover - 离线环境可能没有 ROS
    rospy = None
    Twist = None


RUNNING = "running"
DONE = "done"
STUCK = "stuck"
IDLE = "idle"


@dataclass
class PathFollower:
    cmd_vel_topic: str
    max_linear: float = 0.35
    max_angular: float = 0.9
    goal_tolerance: float = 0.35
    waypoint_tolerance: float = 0.25
    pure_pursuit_lookahead: float = 0.65
    reverse_speed: float = -0.18
    recover_duration: float = 1.2
    no_progress_timeout: float = 5.0
    _publisher: object | None = field(default=None, init=False)
    _path: list[tuple[float, float]] = field(default_factory=list, init=False)
    _waypoint_index: int = field(default=0, init=False)
    _last_progress_time: float = field(default_factory=time.time, init=False)
    _last_goal_distance: float = field(default=math.inf, init=False)
    _recover_until: float = field(default=0.0, init=False)

    def start(self) -> None:
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法发布 /cmd_vel")
        self._publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

    def set_path(self, path: list[tuple[float, float]]) -> None:
        self._path = list(path)
        self._waypoint_index = 0
        self._last_progress_time = time.time()
        self._last_goal_distance = math.inf

    def has_path(self) -> bool:
        return bool(self._path)

    def clear_path(self) -> None:
        self._path = []
        self._waypoint_index = 0
        self.stop()

    def tick(self, pose_xy_yaw: tuple[float, float, float]) -> str:
        now = time.time()
        if self._recover_until > now:
            self._publish(self.reverse_speed, 0.0)
            return RUNNING
        if not self._path:
            self.stop()
            return IDLE

        x, y, yaw = pose_xy_yaw
        goal = self._path[-1]
        goal_distance = math.hypot(goal[0] - x, goal[1] - y)
        if goal_distance <= self.goal_tolerance:
            self.clear_path()
            return DONE

        if goal_distance + 0.05 < self._last_goal_distance:
            self._last_goal_distance = goal_distance
            self._last_progress_time = now
        elif now - self._last_progress_time > self.no_progress_timeout:
            self.clear_path()
            return STUCK

        target = self._lookahead_target(x, y)
        target_yaw = math.atan2(target[1] - y, target[0] - x)
        yaw_error = _angle_wrap(target_yaw - yaw)
        angular = _clamp(1.7 * yaw_error, -self.max_angular, self.max_angular)
        if abs(yaw_error) > 0.9:
            linear = 0.0
        else:
            linear = self.max_linear * max(0.25, 1.0 - abs(yaw_error) / 1.2)
        self._publish(linear, angular)
        return RUNNING

    def recover(self) -> None:
        self._recover_until = time.time() + self.recover_duration
        self._publish(self.reverse_speed, 0.0)

    def rotate_in_place(self, angular_z: float) -> None:
        self._publish(0.0, _clamp(float(angular_z), -self.max_angular, self.max_angular))

    def stop(self) -> None:
        self._publish(0.0, 0.0)

    def _lookahead_target(self, x: float, y: float) -> tuple[float, float]:
        while self._waypoint_index < len(self._path) - 1:
            current = self._path[self._waypoint_index]
            if math.hypot(current[0] - x, current[1] - y) > self.waypoint_tolerance:
                break
            self._waypoint_index += 1

        target = self._path[self._waypoint_index]
        for index in range(self._waypoint_index, len(self._path)):
            candidate = self._path[index]
            if math.hypot(candidate[0] - x, candidate[1] - y) >= self.pure_pursuit_lookahead:
                target = candidate
                self._waypoint_index = index
                break
        return target

    def _publish(self, linear_x: float, angular_z: float) -> None:
        if self._publisher is None or Twist is None:
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._publisher.publish(msg)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

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
    max_linear: float = 0.60
    max_lateral: float = 0.0
    max_angular: float = 0.0
    min_linear_fraction: float = 1.0
    goal_tolerance: float = 0.35
    waypoint_tolerance: float = 0.25
    pure_pursuit_lookahead: float = 0.65
    reverse_speed: float = -0.18
    recover_duration: float = 0.6
    no_progress_timeout: float = 12.0
    _publisher: object | None = field(default=None, init=False)
    _path: list[tuple[float, float]] = field(default_factory=list, init=False)
    _waypoint_index: int = field(default=0, init=False)
    _last_progress_time: float = field(default_factory=time.time, init=False)
    _last_goal_distance: float = field(default=math.inf, init=False)
    _recover_until: float = field(default=0.0, init=False)
    command_count: int = field(default=0, init=False)
    nonzero_linear_count: int = field(default=0, init=False)
    max_abs_linear_x: float = field(default=0.0, init=False)
    max_abs_linear_y: float = field(default=0.0, init=False)
    max_abs_angular_z: float = field(default=0.0, init=False)
    last_command: tuple[float, float, float] = field(default=(0.0, 0.0, 0.0), init=False)

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
        linear_x, linear_y, angular = compute_tracking_command(
            yaw_error,
            max_linear=self.max_linear,
            max_lateral=self.max_lateral,
            max_angular=self.max_angular,
            min_linear_fraction=self.min_linear_fraction,
        )
        self._publish(linear_x, angular, linear_y=linear_y)
        return RUNNING

    def recover(self) -> None:
        self._recover_until = time.time() + self.recover_duration
        self._publish(self.reverse_speed, 0.0)

    def rotate_in_place(self, angular_z: float) -> None:
        self._publish(0.0, _clamp(float(angular_z), -self.max_angular, self.max_angular))

    def stop(self, repeat: int = 1, interval: float = 0.02) -> None:
        count = max(1, int(repeat))
        for index in range(count):
            self._publish(0.0, 0.0)
            if index < count - 1:
                time.sleep(max(0.0, float(interval)))

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

    def _publish(self, linear_x: float, angular_z: float, *, linear_y: float = 0.0) -> None:
        if self._publisher is None or Twist is None:
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.command_count += 1
        if abs(msg.linear.x) > 1e-4 or abs(msg.linear.y) > 1e-4:
            self.nonzero_linear_count += 1
        self.max_abs_linear_x = max(self.max_abs_linear_x, abs(msg.linear.x))
        self.max_abs_linear_y = max(self.max_abs_linear_y, abs(msg.linear.y))
        self.max_abs_angular_z = max(self.max_abs_angular_z, abs(msg.angular.z))
        self.last_command = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._publisher.publish(msg)

    def command_debug_summary(self) -> dict[str, object]:
        return {
            "command_count": self.command_count,
            "nonzero_linear_count": self.nonzero_linear_count,
            "max_abs_linear_x": round(self.max_abs_linear_x, 3),
            "max_abs_linear_y": round(self.max_abs_linear_y, 3),
            "max_abs_angular_z": round(self.max_abs_angular_z, 3),
            "last_command": [round(value, 3) for value in self.last_command],
        }


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def compute_tracking_command(
    yaw_error: float,
    *,
    max_linear: float,
    max_lateral: float = 0.30,
    max_angular: float,
    min_linear_fraction: float = 0.25,
) -> tuple[float, float, float]:
    """生成前进、侧向加转向命令，避免 RL 控制器对纯角速度命令不响应。"""
    angular = _clamp(1.7 * yaw_error, -float(max_angular), float(max_angular))
    min_fraction = _clamp(float(min_linear_fraction), 0.0, 1.0)
    yaw_scale = max(min_fraction, 1.0 - abs(yaw_error) / math.pi)
    linear = max(0.0, float(max_linear)) * yaw_scale
    lateral = _clamp(math.sin(yaw_error) * float(max_lateral), -float(max_lateral), float(max_lateral))
    return linear, lateral, angular


def _angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

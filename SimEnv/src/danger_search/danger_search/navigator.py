"""局部路径跟踪控制器。"""
from __future__ import annotations
from typing import Iterator

from .time_utils import get_time

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
    max_lateral: float = 0.25
    max_angular: float = 1.2
    min_linear_fraction: float = 0.35
    goal_tolerance: float = 0.35
    waypoint_tolerance: float = 0.25
    corner_tolerance: float = 0.10
    pure_pursuit_lookahead: float = 0.45
    reverse_speed: float = -0.25
    recover_duration: float = 1.5
    no_progress_timeout: float = 8.0
    progress_distance: float = 0.25
    _publisher: object | None = field(default=None, init=False)
    _path: list[tuple[float, float]] = field(default_factory=list, init=False)
    _waypoint_index: int = field(default=0, init=False)
    _last_progress_time: float = field(default_factory=time.time, init=False)
    _last_goal_distance: float = field(default=math.inf, init=False)
    _last_progress_pose: tuple[float, float, float] | None = field(default=None, init=False)
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
        self._last_progress_time = get_time()
        self._last_goal_distance = math.inf
        self._last_progress_pose = None

    def has_path(self) -> bool:
        return bool(self._path)

    def clear_path(self) -> None:
        self._path = []
        self._waypoint_index = 0
        self.stop()

    def tick(self, pose_xy_yaw: tuple[float, float, float]) -> str:
        now = get_time()
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

        # 进度判定：以机器人自身位移为主，防止绕弯时到终点欧氏距离不下降而误判 STUCK。
        if self._last_progress_pose is None:
            self._last_progress_pose = pose_xy_yaw
        moved = math.hypot(
            x - self._last_progress_pose[0],
            y - self._last_progress_pose[1],
        )
        goal_approach = goal_distance + self.progress_distance < self._last_goal_distance
        if moved > self.progress_distance or goal_approach:
            self._last_progress_pose = pose_xy_yaw
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
        self._recover_until = get_time() + self.recover_duration
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
        # 推进 waypoint：直线段使用 waypoint_tolerance；遇到转角前缩小容忍，
        # 确保机器人先贴近拐角再切换到下一段，防止提前切角或跳到未来折返段。
        while self._waypoint_index < len(self._path) - 2:
            next_point = self._path[self._waypoint_index + 1]
            advance_tolerance = self.waypoint_tolerance
            if self._is_turn_vertex(self._waypoint_index + 1):
                advance_tolerance = min(self.waypoint_tolerance, self.corner_tolerance)
            if math.hypot(next_point[0] - x, next_point[1] - y) <= advance_tolerance:
                self._waypoint_index += 1
            else:
                break

        # 只在当前路径段上做投影和 lookahead，禁止跳到未来折返段。
        ax, ay = self._path[self._waypoint_index]
        bx, by = self._path[self._waypoint_index + 1]
        vx, vy = bx - ax, by - ay
        seg_len = math.hypot(vx, vy)
        if seg_len <= 1e-6:
            return bx, by

        t = _clamp(((x - ax) * vx + (y - ay) * vy) / (seg_len * seg_len), 0.0, 1.0)
        px, py = ax + t * vx, ay + t * vy
        distance = min(self.pure_pursuit_lookahead, seg_len * (1.0 - t))
        return px + distance * vx / seg_len, py + distance * vy / seg_len

    def _is_turn_vertex(self, vertex_index: int) -> bool:
        """判断 path[vertex_index] 是否为转角顶点（入段与出段方向变化明显）。"""
        if vertex_index <= 0 or vertex_index >= len(self._path) - 1:
            return False
        ax, ay = self._path[vertex_index - 1]
        bx, by = self._path[vertex_index]
        cx, cy = self._path[vertex_index + 1]
        in_x, in_y = bx - ax, by - ay
        out_x, out_y = cx - bx, cy - by
        in_len = math.hypot(in_x, in_y)
        out_len = math.hypot(out_x, out_y)
        if in_len <= 1e-6 or out_len <= 1e-6:
            return False
        cos_angle = (in_x * out_x + in_y * out_y) / (in_len * out_len)
        # 约 5 度以上方向变化视为转角。
        return cos_angle < 0.996

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

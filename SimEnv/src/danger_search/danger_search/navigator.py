"""局部路径跟踪控制器。"""
from __future__ import annotations
from typing import Iterator

from .time_utils import get_time

from dataclasses import dataclass, field
import math
import time

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Path as PathMessage
except ImportError:  # pragma: no cover - 离线环境可能没有 ROS
    rospy = None
    PoseStamped = None
    Twist = None
    PathMessage = None


RUNNING = "running"
DONE = "done"
STUCK = "stuck"
IDLE = "idle"


@dataclass
class PathFollower:
    cmd_vel_topic: str
    path_raw_topic: str = "/danger_search/path_raw"
    path_followed_topic: str = "/danger_search/path_followed"
    local_target_topic: str = "/danger_search/local_target"
    path_frame_id: str = "odom"
    max_linear: float = 0.60
    max_lateral: float = 0.25
    max_angular: float = 1.2
    min_linear_fraction: float = 0.35
    goal_tolerance: float = 0.35
    waypoint_tolerance: float = 0.25
    corner_tolerance: float = 0.15
    pure_pursuit_lookahead: float = 0.45
    reverse_speed: float = -0.15
    recover_duration: float = 0.5
    recover_turn_duration: float = 1.0
    recover_angular_speed: float = 0.55
    command_keepalive_period: float = 0.1
    obstacle_stop_distance: float = 0.50
    obstacle_slow_distance: float = 1.00
    no_progress_timeout: float = 8.0
    progress_distance: float = 0.25
    _publisher: object | None = field(default=None, init=False)
    _raw_path_publisher: object | None = field(default=None, init=False)
    _followed_path_publisher: object | None = field(default=None, init=False)
    _local_target_publisher: object | None = field(default=None, init=False)
    _raw_path: list[tuple[float, float]] = field(default_factory=list, init=False)
    _path: list[tuple[float, float]] = field(default_factory=list, init=False)
    _waypoint_index: int = field(default=0, init=False)
    _last_progress_time: float = field(default_factory=time.time, init=False)
    _last_goal_distance: float = field(default=math.inf, init=False)
    _last_progress_pose: tuple[float, float, float] | None = field(default=None, init=False)
    _recover_until: float = field(default=0.0, init=False)
    _recover_phase: str = field(default="idle", init=False)
    _recover_turn_direction: float = field(default=1.0, init=False)
    _keepalive_timer: object | None = field(default=None, init=False)
    _nearest_obstacle_distance: float | None = field(default=None, init=False)
    command_count: int = field(default=0, init=False)
    nonzero_linear_count: int = field(default=0, init=False)
    max_abs_linear_x: float = field(default=0.0, init=False)
    max_abs_linear_y: float = field(default=0.0, init=False)
    max_abs_angular_z: float = field(default=0.0, init=False)
    last_command: tuple[float, float, float] = field(default=(0.0, 0.0, 0.0), init=False)
    stuck_count: int = field(default=0, init=False)
    recover_count: int = field(default=0, init=False)
    _debug_pose: tuple[float, float, float] | None = field(default=None, init=False)
    _debug_current_waypoint: tuple[float, float] | None = field(default=None, init=False)
    _debug_local_target: tuple[float, float] | None = field(default=None, init=False)
    _debug_distance_to_target: float | None = field(default=None, init=False)
    _debug_heading_error: float | None = field(default=None, init=False)
    _debug_path_turn_angle: float | None = field(default=None, init=False)
    _debug_path_curvature: float | None = field(default=None, init=False)
    _debug_stuck_elapsed: float = field(default=0.0, init=False)
    _debug_replan_reason: str | None = field(default=None, init=False)
    _debug_recover_reason: str | None = field(default=None, init=False)
    _debug_zero_command_reason: str | None = field(default=None, init=False)
    _last_requested_command: tuple[float, float, float] = field(default=(0.0, 0.0, 0.0), init=False)
    _last_command_wall_time: float | None = field(default=None, init=False)
    keepalive_publish_count: int = field(default=0, init=False)

    def start(self) -> None:
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法发布 /cmd_vel")
        self._publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        if PathMessage is not None:
            self._raw_path_publisher = rospy.Publisher(
                self.path_raw_topic,
                PathMessage,
                queue_size=1,
                latch=True,
            )
            self._followed_path_publisher = rospy.Publisher(
                self.path_followed_topic,
                PathMessage,
                queue_size=1,
                latch=True,
            )
        if PoseStamped is not None:
            self._local_target_publisher = rospy.Publisher(
                self.local_target_topic,
                PoseStamped,
                queue_size=1,
            )
        if self.command_keepalive_period > 0.0:
            self._keepalive_timer = rospy.Timer(
                rospy.Duration(self.command_keepalive_period),
                self._on_keepalive_timer,
            )

    def set_path(self, path: list[tuple[float, float]]) -> None:
        self._raw_path = [(float(point[0]), float(point[1])) for point in path]
        self._path = _compress_collinear_path(self._raw_path)
        self._recover_phase = "idle"
        self._recover_until = 0.0
        self._waypoint_index = 0
        self._last_progress_time = get_time()
        self._last_goal_distance = math.inf
        self._last_progress_pose = None
        self._debug_stuck_elapsed = 0.0
        if self._path:
            self._debug_current_waypoint = self._path[min(1, len(self._path) - 1)]
        self._publish_path_diagnostics()

    def has_path(self) -> bool:
        # 恢复阶段由控制器自行完成，避免上层在转向尚未结束时同步重规划。
        return bool(self._path) or self._recover_phase != "idle"

    def set_nearest_obstacle_distance(self, distance: float | None) -> None:
        if distance is None or not math.isfinite(distance):
            self._nearest_obstacle_distance = None
        else:
            self._nearest_obstacle_distance = max(0.0, float(distance))

    def clear_path(self, reason: str = "path_cleared") -> None:
        self._raw_path = []
        self._path = []
        self._waypoint_index = 0
        self._recover_phase = "idle"
        self._recover_until = 0.0
        self._debug_replan_reason = str(reason)
        self._publish_path_diagnostics()
        self.stop(reason=reason)

    def tick(self, pose_xy_yaw: tuple[float, float, float]) -> str:
        now = get_time()
        self._debug_pose = tuple(float(value) for value in pose_xy_yaw)
        self._debug_stuck_elapsed = max(0.0, now - self._last_progress_time)
        if self._recover_phase == "settle":
            if now < self._recover_until:
                self._publish(0.0, 0.0, zero_reason=self._debug_recover_reason or "recover_settle")
                return RUNNING
            self._recover_phase = "turn"
            self._recover_until = now + max(0.0, self.recover_turn_duration)
        if self._recover_phase == "turn":
            if now < self._recover_until:
                self._publish(
                    0.0,
                    self._recover_turn_direction * abs(self.recover_angular_speed),
                    zero_reason="recover_turn",
                )
                return RUNNING
            self._recover_phase = "idle"
            self._debug_replan_reason = "recover_turn_completed"
            self.stop(reason="recover_replan")
            return IDLE
        if not self._path:
            self.stop(reason="no_path")
            return IDLE

        x, y, yaw = pose_xy_yaw
        goal = self._path[-1]
        goal_distance = math.hypot(goal[0] - x, goal[1] - y)
        if goal_distance <= self.goal_tolerance:
            self._debug_current_waypoint = goal
            self._record_target_debug(pose_xy_yaw, goal)
            self.clear_path(reason="goal_reached")
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
            self._debug_stuck_elapsed = 0.0
        elif self._debug_stuck_elapsed > self.no_progress_timeout:
            self.stuck_count += 1
            self.clear_path(reason="no_progress_timeout")
            return STUCK

        target = self._lookahead_target(x, y)
        self._debug_current_waypoint = self._current_waypoint()
        self._debug_path_turn_angle, self._debug_path_curvature = self._local_path_geometry()
        yaw_error = self._record_target_debug(pose_xy_yaw, target)
        self._publish_local_target(target)
        linear_x, linear_y, angular = compute_tracking_command(
            yaw_error,
            max_linear=self.max_linear,
            max_lateral=self.max_lateral,
            max_angular=self.max_angular,
            min_linear_fraction=self.min_linear_fraction,
            path_curvature=self._debug_path_curvature,
            nearest_obstacle_distance=self._nearest_obstacle_distance,
            obstacle_stop_distance=self.obstacle_stop_distance,
            obstacle_slow_distance=self.obstacle_slow_distance,
        )
        self._publish(linear_x, angular, linear_y=linear_y)
        return RUNNING

    def recover(self, reason: str = "stuck") -> None:
        # 不盲目后退：先稳定，再按当前目标方向原地转向，随后明确退出并重规划。
        self.recover_count += 1
        self._debug_recover_reason = str(reason)
        heading_error = self._debug_heading_error or 0.0
        if abs(heading_error) > 0.05:
            self._recover_turn_direction = 1.0 if heading_error > 0.0 else -1.0
        else:
            self._recover_turn_direction = 1.0 if self.recover_count % 2 else -1.0
        self._recover_phase = "settle"
        self._recover_until = get_time() + self.recover_duration
        self._publish(0.0, 0.0, zero_reason=reason)

    def rotate_in_place(self, angular_z: float) -> None:
        self._publish(
            0.0,
            _clamp(float(angular_z), -self.max_angular, self.max_angular),
            zero_reason="rotate_in_place_zero",
        )

    def stop(self, repeat: int = 1, interval: float = 0.02, *, reason: str = "stop") -> None:
        count = max(1, int(repeat))
        for index in range(count):
            self._publish(0.0, 0.0, zero_reason=reason)
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
            distance_to_vertex = math.hypot(next_point[0] - x, next_point[1] - y)
            if distance_to_vertex <= advance_tolerance or self._has_passed_current_segment(x, y):
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

    def _has_passed_current_segment(self, x: float, y: float) -> bool:
        """机器人已越过当前段终点时推进，避免在拐点容差外无限追逐旧点。"""
        ax, ay = self._path[self._waypoint_index]
        bx, by = self._path[self._waypoint_index + 1]
        vx, vy = bx - ax, by - ay
        segment_length = math.hypot(vx, vy)
        if segment_length <= 1e-6:
            return True
        projection = ((x - ax) * vx + (y - ay) * vy) / (segment_length * segment_length)
        cross_track = abs((x - ax) * vy - (y - ay) * vx) / segment_length
        allowed_cross_track = max(self.waypoint_tolerance * 1.5, self.corner_tolerance)
        return projection >= 0.98 and cross_track <= allowed_cross_track

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

    def _current_waypoint(self) -> tuple[float, float] | None:
        if not self._path:
            return None
        return self._path[min(self._waypoint_index + 1, len(self._path) - 1)]

    def _local_path_geometry(self) -> tuple[float | None, float | None]:
        vertex_index = self._waypoint_index + 1
        if vertex_index <= 0 or vertex_index >= len(self._path) - 1:
            return None, None
        bx, by = self._path[vertex_index]
        span_target = max(self.pure_pursuit_lookahead, self.waypoint_tolerance)

        back_index = vertex_index - 1
        in_span = math.hypot(
            bx - self._path[back_index][0],
            by - self._path[back_index][1],
        )
        while back_index > 0 and in_span < span_target:
            in_span += math.hypot(
                self._path[back_index][0] - self._path[back_index - 1][0],
                self._path[back_index][1] - self._path[back_index - 1][1],
            )
            back_index -= 1

        forward_index = vertex_index + 1
        out_span = math.hypot(
            self._path[forward_index][0] - bx,
            self._path[forward_index][1] - by,
        )
        while forward_index < len(self._path) - 1 and out_span < span_target:
            out_span += math.hypot(
                self._path[forward_index + 1][0] - self._path[forward_index][0],
                self._path[forward_index + 1][1] - self._path[forward_index][1],
            )
            forward_index += 1

        ax, ay = self._path[back_index]
        cx, cy = self._path[forward_index]
        if in_span <= 1e-6 or out_span <= 1e-6:
            return None, None
        turn_angle = _angle_wrap(math.atan2(cy - by, cx - bx) - math.atan2(by - ay, bx - ax))
        curvature = turn_angle / max(0.5 * (in_span + out_span), span_target, 1e-6)
        return turn_angle, curvature

    def _record_target_debug(
        self,
        pose_xy_yaw: tuple[float, float, float],
        target: tuple[float, float],
    ) -> float:
        x, y, yaw = pose_xy_yaw
        self._debug_local_target = (float(target[0]), float(target[1]))
        self._debug_distance_to_target = math.hypot(target[0] - x, target[1] - y)
        target_yaw = math.atan2(target[1] - y, target[0] - x)
        self._debug_heading_error = _angle_wrap(target_yaw - yaw)
        return self._debug_heading_error

    def _publish(
        self,
        linear_x: float,
        angular_z: float,
        *,
        linear_y: float = 0.0,
        zero_reason: str | None = None,
    ) -> None:
        requested = (float(linear_x), float(linear_y), float(angular_z))
        self._last_requested_command = requested
        if all(abs(value) <= 1e-4 for value in requested):
            self._debug_zero_command_reason = str(zero_reason or "zero_command")
        else:
            self._debug_zero_command_reason = None
        if self._publisher is None or Twist is None:
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = requested
        self.command_count += 1
        if abs(msg.linear.x) > 1e-4 or abs(msg.linear.y) > 1e-4:
            self.nonzero_linear_count += 1
        self.max_abs_linear_x = max(self.max_abs_linear_x, abs(msg.linear.x))
        self.max_abs_linear_y = max(self.max_abs_linear_y, abs(msg.linear.y))
        self.max_abs_angular_z = max(self.max_abs_angular_z, abs(msg.angular.z))
        self.last_command = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_command_wall_time = time.monotonic()
        self._publisher.publish(msg)

    def _on_keepalive_timer(self, _event) -> None:
        """控制回调被规划阻塞时持续发布最近命令，避免底层 0.5 秒超时。"""
        if self._publisher is None or Twist is None:
            return
        now = time.monotonic()
        if self._last_command_wall_time is None:
            return
        minimum_age = max(0.02, self.command_keepalive_period * 0.8)
        if now - self._last_command_wall_time < minimum_age:
            return
        linear_x, linear_y, angular_z = self._last_requested_command
        self.keepalive_publish_count += 1
        self._publish(
            linear_x,
            angular_z,
            linear_y=linear_y,
            zero_reason=self._debug_zero_command_reason,
        )

    def raw_path_snapshot(self) -> list[tuple[float, float]]:
        return list(self._raw_path)

    def followed_path_snapshot(self) -> list[tuple[float, float]]:
        return list(self._path)

    def _publish_path_diagnostics(self) -> None:
        if rospy is None or PathMessage is None or PoseStamped is None:
            return
        stamp = rospy.Time.now()
        for publisher, points in (
            (self._raw_path_publisher, self._raw_path),
            (self._followed_path_publisher, self._path),
        ):
            if publisher is None:
                continue
            message = PathMessage()
            message.header.stamp = stamp
            message.header.frame_id = self.path_frame_id
            for x, y in points:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                message.poses.append(pose)
            publisher.publish(message)

    def _publish_local_target(self, target: tuple[float, float]) -> None:
        if rospy is None or PoseStamped is None or self._local_target_publisher is None:
            return
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.path_frame_id
        message.pose.position.x = float(target[0])
        message.pose.position.y = float(target[1])
        message.pose.orientation.w = 1.0
        self._local_target_publisher.publish(message)

    def tracking_debug_snapshot(self) -> dict[str, object]:
        now = get_time()
        wall_now = time.monotonic()
        recover_remaining = max(0.0, self._recover_until - now)
        wall_command_age = (
            max(0.0, wall_now - self._last_command_wall_time)
            if self._last_command_wall_time is not None
            else None
        )
        linear_x, linear_y, angular_z = self._last_requested_command
        return {
            "pose": _point_list(self._debug_pose),
            "path_active": bool(self._path),
            "raw_path_size": len(self._raw_path),
            "path_size": len(self._path),
            "waypoint_index": self._waypoint_index,
            "current_waypoint": _point_list(self._debug_current_waypoint),
            "local_target": _point_list(self._debug_local_target),
            "distance_to_target": _finite_or_none(self._debug_distance_to_target),
            "heading_error": _finite_or_none(self._debug_heading_error),
            "path_turn_angle": _finite_or_none(self._debug_path_turn_angle),
            "path_curvature": _finite_or_none(self._debug_path_curvature),
            "nearest_obstacle_distance": _finite_or_none(self._nearest_obstacle_distance),
            "linear_command": linear_x,
            "lateral_command": linear_y,
            "angular_command": angular_z,
            "zero_command_reason": self._debug_zero_command_reason,
            "stuck_elapsed": _finite_or_none(self._debug_stuck_elapsed),
            "no_progress_timeout": self.no_progress_timeout,
            "replan_reason": self._debug_replan_reason,
            "recover_state": self._recover_phase,
            "recover_reason": self._debug_recover_reason,
            "recover_remaining": recover_remaining,
            "command_count": self.command_count,
            "stuck_count": self.stuck_count,
            "recover_count": self.recover_count,
            "keepalive_publish_count": self.keepalive_publish_count,
            "last_command_wall_time": self._last_command_wall_time,
            "wall_command_age": wall_command_age,
        }

    def command_debug_summary(self) -> dict[str, object]:
        return {
            "command_count": self.command_count,
            "nonzero_linear_count": self.nonzero_linear_count,
            "max_abs_linear_x": round(self.max_abs_linear_x, 3),
            "max_abs_linear_y": round(self.max_abs_linear_y, 3),
            "max_abs_angular_z": round(self.max_abs_angular_z, 3),
            "last_command": [round(value, 3) for value in self.last_command],
            "stuck_count": self.stuck_count,
            "recover_count": self.recover_count,
            "keepalive_publish_count": self.keepalive_publish_count,
            "tracking": self.tracking_debug_snapshot(),
        }


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _finite_or_none(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return float(value)


def _point_list(point: tuple[float, ...] | None) -> list[float] | None:
    if point is None:
        return None
    return [float(value) for value in point]


def compute_tracking_command(
    yaw_error: float,
    *,
    max_linear: float,
    max_lateral: float = 0.30,
    max_angular: float,
    min_linear_fraction: float = 0.25,
    path_curvature: float | None = None,
    nearest_obstacle_distance: float | None = None,
    obstacle_stop_distance: float = 0.50,
    obstacle_slow_distance: float = 1.00,
) -> tuple[float, float, float]:
    """按航向、曲率和前向障碍共同限制速度，不对全局速度一刀切。"""
    angular = _clamp(1.7 * yaw_error, -float(max_angular), float(max_angular))
    min_fraction = _clamp(float(min_linear_fraction), 0.0, 1.0)
    abs_error = abs(yaw_error)
    normalized_error = min(1.0, abs_error / (math.pi / 2.0))
    yaw_scale = (1.0 - normalized_error) ** 2
    if abs_error <= 0.15:
        yaw_scale = max(min_fraction, yaw_scale)
    elif abs_error >= math.pi / 2.0:
        # 保留极低爬行量以兼容不能响应纯角速度的 RL 控制器。
        yaw_scale = min(0.05, min_fraction)

    curvature = abs(float(path_curvature)) if path_curvature is not None else 0.0
    curvature_scale = 1.0 / (1.0 + 0.7 * curvature)

    obstacle_scale = 1.0
    if nearest_obstacle_distance is not None and math.isfinite(nearest_obstacle_distance):
        clearance = float(nearest_obstacle_distance)
        stop_distance = max(0.0, float(obstacle_stop_distance))
        slow_distance = max(stop_distance + 1e-6, float(obstacle_slow_distance))
        if clearance <= stop_distance:
            obstacle_scale = 0.0
        elif clearance < slow_distance:
            obstacle_scale = (clearance - stop_distance) / (slow_distance - stop_distance)

    speed_scale = min(yaw_scale, curvature_scale, obstacle_scale)
    linear = max(0.0, float(max_linear)) * speed_scale
    lateral_scale = min(obstacle_scale, max(0.2, yaw_scale))
    lateral = _clamp(
        math.sin(yaw_error) * float(max_lateral) * lateral_scale,
        -float(max_lateral),
        float(max_lateral),
    )
    return linear, lateral, angular


def _compress_collinear_path(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """删除重复和共线离散点，保留真实转角。"""
    compact: list[tuple[float, float]] = []
    for point in path:
        normalized = (float(point[0]), float(point[1]))
        if compact and math.hypot(normalized[0] - compact[-1][0], normalized[1] - compact[-1][1]) <= 1e-6:
            continue
        compact.append(normalized)
        while len(compact) >= 3:
            ax, ay = compact[-3]
            bx, by = compact[-2]
            cx, cy = compact[-1]
            cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
            dot = (bx - ax) * (cx - bx) + (by - ay) * (cy - by)
            if abs(cross) > 1e-8 or dot < 0.0:
                break
            compact.pop(-2)
    return compact


def _angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

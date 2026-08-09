"""局部路径跟踪器与全项目唯一的生产速度发送出口。

上层传入 world/odom 平面折线路径，本模块压缩共线点、在当前线段内计算
pure-pursuit 前视目标，并根据航向误差、局部曲率和前向实体净距生成机体系
``(linear_x, linear_y, angular_z)``。命令发布前还会经过可选
``GroundSupportSafetyGate``，所有停车、恢复、门/电梯直线跨越最终都复用
``_publish``，避免出现绕过安全锁存的竞争 publisher。
"""
from __future__ import annotations
from contextlib import contextmanager
from threading import RLock
from typing import Iterator

from .time_utils import get_time
from .interface_policy import (
    require_allowed_output_topic,
    require_internal_output_topic,
)

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
    """维护一条活动路径并以非阻塞 tick 产生速度指令。

    ``_raw_path`` 保存规划器原始输出用于诊断，``_path`` 只删除重复/共线点，
    不做改变几何可达性的平滑。恢复流程也由内部阶段表示，调用方每次 tick 即可，
    不需要 sleep 等待机器人动作。
    """
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
    # 直接控制命令（rotate_in_place/reverse_straight）的最大重放寿命。
    # 所有直接命令调用方都按 tick 重发，超过该寿命说明任务层已经停止驱动，
    # keepalive 必须归零，不能让最后一条非零命令无限重放。
    direct_command_max_age: float = 0.6
    obstacle_stop_distance: float = 0.50
    obstacle_slow_distance: float = 1.00
    no_progress_timeout: float = 8.0
    progress_distance: float = 0.25
    # 路线进度监督：只认"沿折线推进"，机体位移（绕圈/后退）不再算进度。
    route_progress_timeout: float = 8.0
    route_progress_improvement: float = 0.10
    # 控制周期参数（固定频率控制器）。
    pose_max_age: float = 0.30
    # 角速度阻尼与先对准再前进（含转向滞回释放条件）。
    yaw_rate_damping: float = 0.45
    yaw_rate_max_age: float = 0.30
    turn_in_place_heading: float = 0.70
    turn_in_place_rate: float = 0.45
    turn_release_heading: float = 0.20
    turn_release_rate: float = 0.20
    turn_release_samples: int = 3
    _publisher: object | None = field(default=None, init=False)
    _raw_path_publisher: object | None = field(default=None, init=False)
    _followed_path_publisher: object | None = field(default=None, init=False)
    _local_target_publisher: object | None = field(default=None, init=False)
    _raw_path: list[tuple[float, float]] = field(default_factory=list, init=False)
    _path: list[tuple[float, float]] = field(default_factory=list, init=False)
    _waypoint_index: int = field(default=0, init=False)
    _last_progress_time: float = field(default_factory=get_time, init=False)
    _last_goal_distance: float = field(default=math.inf, init=False)
    _last_progress_pose: tuple[float, float, float] | None = field(default=None, init=False)
    _last_progress_route: float = field(default=math.inf, init=False)
    _last_progress_waypoint_index: int = field(default=-1, init=False)
    _last_yaw: float | None = field(default=None, init=False)
    _last_yaw_time: float | None = field(default=None, init=False)
    _debug_yaw_rate: float | None = field(default=None, init=False)
    _debug_route_progress: float | None = field(default=None, init=False)
    _debug_turn_in_place: bool = field(default=False, init=False)
    _pose_provider: object | None = field(default=None, init=False)
    _tracking_status: str = field(default="IDLE", init=False)
    _terminal_pending: bool = field(default=False, init=False)
    _terminal_reason: str | None = field(default=None, init=False)
    _status_sequence: int = field(default=0, init=False)
    _turn_in_place_active: bool = field(default=False, init=False)
    _turn_release_count: int = field(default=0, init=False)
    _control_decision_count: int = field(default=0, init=False)
    _last_decision_period: float | None = field(default=None, init=False)
    _last_decision_wall: float | None = field(default=None, init=False)
    _debug_pose_age: float | None = field(default=None, init=False)
    _debug_yaw_rate_source: str | None = field(default=None, init=False)
    _debug_yaw_rate_valid: bool = field(default=False, init=False)
    _path_max_linear: float | None = field(default=None, init=False)
    _path_max_lateral: float | None = field(default=None, init=False)
    _path_goal_tolerance: float | None = field(default=None, init=False)
    _recover_until: float = field(default=0.0, init=False)
    _recover_phase: str = field(default="idle", init=False)
    _recover_turn_direction: float = field(default=1.0, init=False)
    # ALIGN 模式（固定频率原地对准）：SETTLE/ALIGN/CONFIRM 子状态机。
    _align_state: str = field(default="idle", init=False)
    _align_target_xy: tuple[float, float] | None = field(default=None, init=False)
    _align_anchor_xy: tuple[float, float] | None = field(default=None, init=False)
    _align_max_anchor_error: float | None = field(default=None, init=False)
    _align_confirm_count: int = field(default=0, init=False)
    _align_settle_samples: int = field(default=0, init=False)
    _align_settle_start: float = field(default=0.0, init=False)
    _align_settle_timeout: float = field(default=3.0, init=False)
    _align_release_heading: float = field(default=0.15, init=False)
    _align_release_rate: float = field(default=0.20, init=False)
    _align_release_samples: int = field(default=3, init=False)
    _align_deadband_omega: float = field(default=0.55, init=False)
    _align_deadband_error: float = field(default=0.10, init=False)
    _debug_align_yaw_error: float | None = field(default=None, init=False)
    _debug_align_anchor_error: float | None = field(default=None, init=False)
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
    _last_emitted_command: tuple[float, float, float] = field(default=(0.0, 0.0, 0.0), init=False)
    _active_motion_command: tuple[float, float, float] = field(
        default=(0.0, 0.0, 0.0),
        init=False,
    )
    _command_gate: object | None = field(default=None, init=False)
    _last_command_wall_time: float | None = field(default=None, init=False)
    # 只记录任务层显式请求的直接命令（rotate_in_place/reverse_straight/stop）；
    # keepalive 重放与固定频率决策不刷新它，用于判断直接命令是否已失去刷新方。
    _direct_command_wall_time: float | None = field(default=None, init=False)
    _active_motion_last_odom_stamp: float | None = field(default=None, init=False)
    _active_motion_stop_samples: int = field(default=0, init=False)
    _publish_failure_latched: bool = field(default=False, init=False)
    _publish_failure_reason: str | None = field(default=None, init=False)
    _command_transaction_lock: RLock = field(
        default_factory=RLock,
        init=False,
        repr=False,
    )
    keepalive_publish_count: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        self.cmd_vel_topic = require_allowed_output_topic(
            self.cmd_vel_topic,
            purpose="PathFollower 控制输出",
        )
        self.path_raw_topic = require_internal_output_topic(
            self.path_raw_topic,
            purpose="原始路径诊断输出",
        )
        self.path_followed_topic = require_internal_output_topic(
            self.path_followed_topic,
            purpose="跟踪路径诊断输出",
        )
        self.local_target_topic = require_internal_output_topic(
            self.local_target_topic,
            purpose="局部目标诊断输出",
        )
        nonnegative = (
            self.max_linear,
            self.max_lateral,
            self.max_angular,
            self.goal_tolerance,
            self.waypoint_tolerance,
            self.corner_tolerance,
            self.pure_pursuit_lookahead,
            self.recover_duration,
            self.recover_turn_duration,
            self.recover_angular_speed,
            self.command_keepalive_period,
            self.obstacle_stop_distance,
            self.obstacle_slow_distance,
            self.no_progress_timeout,
            self.progress_distance,
        )
        if (
            not all(math.isfinite(float(value)) and float(value) >= 0.0 for value in nonnegative)
            or not math.isfinite(float(self.min_linear_fraction))
            or not 0.0 <= float(self.min_linear_fraction) <= 1.0
            or not math.isfinite(float(self.reverse_speed))
            or float(self.reverse_speed) > 0.0
            or self.goal_tolerance <= 0.0
            or self.pure_pursuit_lookahead <= 0.0
            or self.obstacle_slow_distance <= self.obstacle_stop_distance
            or self.no_progress_timeout <= 0.0
            or self.progress_distance <= 0.0
        ):
            raise ValueError("路径跟踪速度、容差、恢复或进度参数无效")

    def start(self, pose_provider: object | None = None) -> None:
        """建立唯一 ``/cmd_vel`` publisher、诊断 publisher 和固定频率控制定时器。

        ``pose_provider`` 提供带时间戳与有符号角速度的位姿样本；路径/恢复
        模式由控制定时器每周期取新位姿重新计算命令（新鲜决策），不再把慢任务
        tick 的计算结果交给 keepalive 重复重放。
        """
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法发布 /cmd_vel")
        self._pose_provider = pose_provider
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
                self._on_control_timer,
            )

    def set_path(
        self,
        path: list[tuple[float, float]],
        *,
        max_linear: float | None = None,
        goal_tolerance: float | None = None,
        max_lateral: float | None = None,
        reason: str = "path_set",
    ) -> None:
        """替换活动路径并原子提交跟踪限制，重置进度监督、恢复与终态锁存。

        ``reason`` 归因“谁装的这条路径”。此前只有 ``clear_path`` 会写
        ``_debug_replan_reason``，装载新路径不写，导致快照里的 ``replan_reason``
        停留在上一次清理的旧值：路径已经换了一条，归因还指向前一个原因。
        """
        with self._command_transaction_lock:
            self._set_path_locked(
                path,
                max_linear=max_linear,
                goal_tolerance=goal_tolerance,
                max_lateral=max_lateral,
                reason=reason,
            )

    def _set_path_locked(
        self,
        path: list[tuple[float, float]],
        *,
        max_linear: float | None = None,
        goal_tolerance: float | None = None,
        max_lateral: float | None = None,
        reason: str = "path_set",
    ) -> None:
        try:
            parsed = [(float(point[0]), float(point[1])) for point in path]
        except (TypeError, ValueError, IndexError, KeyError):
            self.stop(reason="invalid_path")
            raise ValueError("路径必须由二维数值点组成") from None
        self._raw_path = parsed
        if not all(
            math.isfinite(value)
            for point in self._raw_path
            for value in point
        ):
            self.stop(reason="invalid_path")
            raise ValueError("路径点必须为有限 world XY")
        self._path = _compress_collinear_path(self._raw_path)
        # 归因写在两道校验之后：被拒绝的路径不得声称自己已装载，否则快照会
        # 显示一条从未生效的 reason。校验失败路径的归因由 stop("invalid_path")
        # 经 _debug_zero_command_reason 记录。
        self._debug_replan_reason = str(reason)
        self._recover_phase = "idle"
        self._recover_until = 0.0
        self._waypoint_index = 0
        self._last_progress_time = get_time()
        self._last_goal_distance = math.inf
        self._last_progress_pose = None
        self._last_progress_route = math.inf
        self._last_progress_waypoint_index = -1
        self._debug_stuck_elapsed = 0.0
        self._turn_in_place_active = False
        self._turn_release_count = 0
        self._tracking_status = "RUNNING"
        self._terminal_pending = False
        self._terminal_reason = None
        self._status_sequence += 1
        self._path_max_linear = (
            max(0.0, float(max_linear)) if max_linear is not None else None
        )
        self._path_max_lateral = (
            max(0.0, float(max_lateral)) if max_lateral is not None else None
        )
        self._path_goal_tolerance = (
            max(0.0, float(goal_tolerance)) if goal_tolerance is not None else None
        )
        if self._path:
            self._debug_current_waypoint = self._path[min(1, len(self._path) - 1)]
        self._publish_path_diagnostics()

    def has_path(self) -> bool:
        # 恢复/对准阶段由控制器自行完成，避免上层在转向尚未结束时同步重规划。
        return bool(self._path) or self._recover_phase != "idle" or self._align_state != "idle"

    def current_segment(
        self,
    ) -> tuple[tuple[float, float], tuple[float, float]] | None:
        """返回当前跟踪段的只读副本，供任务层记录失败反馈。"""
        if not self._path or len(self._path) < 2:
            return None
        index = min(self._waypoint_index, len(self._path) - 2)
        return tuple(self._path[index]), tuple(self._path[index + 1])

    def set_nearest_obstacle_distance(self, distance: float | None) -> None:
        if distance is None or not math.isfinite(distance):
            self._nearest_obstacle_distance = None
        else:
            self._nearest_obstacle_distance = max(0.0, float(distance))

    def set_command_gate(self, gate: object | None) -> None:
        """设置最终速度出口的可选安全门；None 保持原有命令链完全透明。"""
        with self._command_transaction_lock:
            self._command_gate = gate

    @contextmanager
    def command_transaction(self) -> Iterator[None]:
        """串行化 Gate 状态变更、最终过滤、发布与成功状态提交。"""
        with self._command_transaction_lock:
            yield

    def reset_progress_timer(
        self,
        pose_xy_yaw: tuple[float, float, float] | None = None,
    ) -> None:
        """安全观察暂停期间冻结普通 STUCK 计时，不改路径和目标。"""
        self._last_progress_time = get_time()
        self._debug_stuck_elapsed = 0.0
        if pose_xy_yaw is not None:
            self._last_progress_pose = tuple(float(value) for value in pose_xy_yaw)

    def clear_path(self, reason: str = "path_cleared") -> None:
        """清除路径和恢复状态，并立即通过统一出口发送零速。"""
        with self._command_transaction_lock:
            self._clear_path_locked(reason)

    def _clear_path_locked(self, reason: str) -> None:
        """锁内清除路径、恢复状态并发送零速；不重复获取事务锁。"""
        self._raw_path = []
        self._path = []
        self._waypoint_index = 0
        self._recover_phase = "idle"
        self._recover_until = 0.0
        self._align_state = "idle"
        self._align_target_xy = None
        self._align_anchor_xy = None
        self._align_confirm_count = 0
        self._align_settle_samples = 0
        self._turn_in_place_active = False
        self._turn_release_count = 0
        self._tracking_status = "IDLE"
        self._debug_replan_reason = str(reason)
        self._publish_path_diagnostics()
        self._stop_locked(reason=reason)

    def tick(
        self,
        pose_xy_yaw: tuple[float, float, float],
        *,
        max_linear: float | None = None,
        goal_tolerance: float | None = None,
    ) -> str:
        """兼容旧接口：同步执行一次控制决策（仅测试/回退路径使用）。

        生产路径由固定频率控制定时器驱动，任务层请改用 poll_status()。
        """
        sample = _SyncPoseSample(pose_xy_yaw)
        return self.step_once_for_test(
            sample,
            max_linear=max_linear,
            goal_tolerance=goal_tolerance,
        )

    def step_once_for_test(
        self,
        sample,
        *,
        max_linear: float | None = None,
        goal_tolerance: float | None = None,
    ) -> str:
        """离线单测用：同步执行一次控制决策；不进入生产调用链。

        max_linear/goal_tolerance 仅影响本次决策（与旧 tick 语义一致），
        不持久化到路径限制。
        """
        with self._command_transaction_lock:
            return self._control_step_locked(
                sample,
                temp_max_linear=max_linear,
                temp_goal_tolerance=goal_tolerance,
            )

    def poll_result(self) -> ControlResult:
        """任务层状态轮询：原子捕获并消费终态（status+reason+sequence）。

        终态未被消费前 has_path() 保持 True，防止慢任务层提前重规划；
        消费一次后回到 IDLE，reason 清空。
        """
        with self._command_transaction_lock:
            if self._terminal_pending:
                status = self._tracking_status
                reason = self._terminal_reason
                sequence = self._status_sequence
                self._terminal_pending = False
                self._tracking_status = "IDLE"
                self._terminal_reason = None
                self._status_sequence += 1
                self._clear_path_locked(reason=reason or "terminal_consumed")
                return ControlResult(status=status, reason=reason, sequence=sequence)
            if self._recover_phase != "idle":
                return ControlResult(status=RUNNING, reason=None, sequence=self._status_sequence)
            if self._align_state != "idle":
                return ControlResult(status=RUNNING, reason=None, sequence=self._status_sequence)
            if not self._path:
                return ControlResult(status=IDLE, reason=None, sequence=self._status_sequence)
            return ControlResult(
                status=self._tracking_status,
                reason=None,
                sequence=self._status_sequence,
            )

    def poll_status(self) -> str:
        """兼容接口：只返回状态（无原因）；新代码请使用 poll_result()。"""
        return self.poll_result().status

    def begin_heading_alignment(
        self,
        target_xy: tuple[float, float],
        *,
        anchor_xy: tuple[float, float] | None = None,
        max_anchor_error: float | None = None,
        settle_timeout: float | None = None,
    ) -> None:
        """进入固定频率原地对准模式（SETTLE -> ALIGN -> CONFIRM）。

        - 对准目标为 ``target_xy`` 相对当前位姿的方位角；
        - ``anchor_xy`` 与 ``max_anchor_error`` 提供位置漂移保护：对准期间
          位姿离开锚点超过半径立即零速并锁存 STUCK(alignment_drift)；
        - 相同目标重复调用幂等：不重置确认计数，任务层可每 tick 调用。
        """
        with self._command_transaction_lock:
            try:
                target = (float(target_xy[0]), float(target_xy[1]))
            except (TypeError, ValueError, IndexError, KeyError):
                raise ValueError("对准目标必须为二维数值点") from None
            if not all(math.isfinite(value) for value in target):
                raise ValueError("对准目标必须为有限 world XY")
            anchor = None
            if anchor_xy is not None:
                try:
                    anchor = (float(anchor_xy[0]), float(anchor_xy[1]))
                except (TypeError, ValueError, IndexError, KeyError):
                    raise ValueError("对准锚点必须为二维数值点") from None
                if not all(math.isfinite(value) for value in anchor):
                    raise ValueError("对准锚点必须为有限 world XY")
            max_error = (
                max(0.0, float(max_anchor_error))
                if max_anchor_error is not None
                else None
            )
            if self._align_state != "idle" and self._align_target_xy == target:
                # 幂等：目标不变时不重置确认/停稳计数。
                return
            self._align_target_xy = target
            self._align_anchor_xy = anchor
            self._align_max_anchor_error = max_error
            self._align_confirm_count = 0
            self._align_settle_samples = 0
            self._align_settle_start = get_time()
            if settle_timeout is not None:
                self._align_settle_timeout = max(0.5, float(settle_timeout))
            self._align_state = "settle"
            self._tracking_status = RUNNING
            self._terminal_pending = False
            self._terminal_reason = None
            self._status_sequence += 1
            self._path = []
            self._raw_path = []

    def end_heading_alignment(self) -> None:
        """退出对准模式并发送零速（任务层显式取消时调用）。"""
        with self._command_transaction_lock:
            if self._align_state != "idle":
                self._align_state = "idle"
                self._align_target_xy = None
                self._align_anchor_xy = None
                self._align_confirm_count = 0
                self._align_settle_samples = 0
                self._publish_locked(0.0, 0.0, zero_reason="alignment_cancelled")

    def set_tracking_limits(
        self,
        *,
        max_linear: float | None = None,
        goal_tolerance: float | None = None,
        max_lateral: float | None = None,
    ) -> None:
        """线程安全更新当前路径的跟踪限制（供动态调整场景使用）。"""
        with self._command_transaction_lock:
            self._set_tracking_limits_locked(
                max_linear=max_linear,
                goal_tolerance=goal_tolerance,
                max_lateral=max_lateral,
            )

    def _set_tracking_limits_locked(
        self,
        *,
        max_linear: float | None = None,
        goal_tolerance: float | None = None,
        max_lateral: float | None = None,
    ) -> None:
        if max_linear is not None:
            self._path_max_linear = max(0.0, float(max_linear))
        if goal_tolerance is not None:
            self._path_goal_tolerance = max(0.0, float(goal_tolerance))
        if max_lateral is not None:
            self._path_max_lateral = max(0.0, float(max_lateral))

    def _recover_step_locked(self, now: float) -> str:
        """锁内推进恢复阶段（settle/turn），时间驱动，不需要位姿。"""
        if self._recover_phase == "settle":
            if now < self._recover_until:
                self._publish_locked(
                    0.0,
                    0.0,
                    zero_reason=self._debug_recover_reason or "recover_settle",
                )
                return RUNNING
            self._recover_phase = "turn"
            self._recover_until = now + max(0.0, self.recover_turn_duration)
        if self._recover_phase == "turn":
            if now < self._recover_until:
                self._publish_locked(
                    0.0,
                    self._recover_turn_direction * abs(self.recover_angular_speed),
                    zero_reason="recover_turn",
                )
                return RUNNING
            self._recover_phase = "idle"
            # 恢复转向完成后必须清除旧路径，任务层才会重新 A*。
            self._clear_path_locked(reason="recover_turn_completed")
            return IDLE
        return RUNNING

    def _control_step_locked(
        self,
        sample,
        *,
        temp_max_linear: float | None = None,
        temp_goal_tolerance: float | None = None,
    ) -> str:
        """锁内执行一次新鲜控制决策（仅控制定时器或测试调用）。

        - 恢复阶段：时间驱动推进，不需要位姿；
        - 路径阶段：用样本 xy_yaw 与有符号角速度计算一次命令；
        - 位姿缺失/过期：发布零速并冻结进度计时（不清路径、不报 STUCK）。
        """
        self._control_decision_count += 1
        now = get_time()
        if self._recover_phase != "idle":
            return self._recover_step_locked(now)
        if not self._path:
            self._publish_locked(0.0, 0.0, zero_reason="no_path")
            self._tracking_status = "IDLE"
            return IDLE

        try:
            x, y, yaw = (float(value) for value in sample.xy_yaw)
        except (TypeError, ValueError):
            self._clear_path_locked(reason="invalid_pose")
            return STUCK
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self._clear_path_locked(reason="invalid_pose")
            return STUCK
        self._debug_pose = (x, y, yaw)
        self._debug_stuck_elapsed = max(0.0, now - self._last_progress_time)
        yaw_rate = None
        angular = getattr(sample, "angular_xyz", None)
        if angular is not None:
            try:
                yaw_rate = float(angular[2])
            except (TypeError, ValueError, IndexError):
                yaw_rate = None
        yaw_rate_valid = yaw_rate is not None and math.isfinite(yaw_rate)
        self._debug_yaw_rate = yaw_rate
        self._debug_yaw_rate_source = "odom" if yaw_rate_valid else "missing"
        self._debug_yaw_rate_valid = yaw_rate_valid

        goal = self._path[-1]
        goal_distance = math.hypot(goal[0] - x, goal[1] - y)
        effective_goal_tolerance = self.goal_tolerance
        if self._path_goal_tolerance is not None:
            effective_goal_tolerance = min(
                effective_goal_tolerance,
                max(0.0, float(self._path_goal_tolerance)),
            )
        if temp_goal_tolerance is not None:
            effective_goal_tolerance = min(
                effective_goal_tolerance,
                max(0.0, float(temp_goal_tolerance)),
            )
        if goal_distance <= effective_goal_tolerance:
            self._debug_current_waypoint = goal
            self._record_target_debug((x, y, yaw), goal)
            self._publish_locked(0.0, 0.0, zero_reason="goal_reached")
            # 先发布零速再锁存终态；路径保留到 poll_status 消费。
            self._tracking_status = DONE
            self._terminal_pending = True
            self._terminal_reason = "goal_reached"
            self._debug_replan_reason = "goal_reached"
            self._status_sequence += 1
            return DONE

        route_remaining = self._remaining_path_distance(x, y)
        self._debug_route_progress = round(route_remaining, 3)
        waypoint_advanced = (
            self._waypoint_index != self._last_progress_waypoint_index
        )
        route_improved = (
            route_remaining + self.route_progress_improvement
            < self._last_progress_route
        )
        if waypoint_advanced or route_improved:
            self._last_progress_waypoint_index = self._waypoint_index
            self._last_progress_route = route_remaining
            self._last_progress_pose = (x, y, yaw)
            self._last_progress_time = now
            self._debug_stuck_elapsed = 0.0
        elif self._debug_stuck_elapsed > self.route_progress_timeout:
            self.stuck_count += 1
            self._publish_locked(0.0, 0.0, zero_reason="route_progress_timeout")
            self._tracking_status = STUCK
            self._terminal_pending = True
            self._terminal_reason = "route_progress_timeout"
            self._debug_replan_reason = "route_progress_timeout"
            self._status_sequence += 1
            return STUCK

        target = self._lookahead_target(x, y)
        self._debug_current_waypoint = self._current_waypoint()
        self._debug_path_turn_angle, self._debug_path_curvature = (
            self._local_path_geometry()
        )
        yaw_error = self._record_target_debug((x, y, yaw), target)
        self._publish_local_target(target)
        linear_limit = self.max_linear
        if self._path_max_linear is not None:
            linear_limit = min(linear_limit, max(0.0, float(self._path_max_linear)))
        if temp_max_linear is not None:
            linear_limit = min(linear_limit, max(0.0, float(temp_max_linear)))
        lateral_limit = self.max_lateral
        if self._path_max_lateral is not None:
            lateral_limit = min(lateral_limit, max(0.0, float(self._path_max_lateral)))

        # 转向滞回：进入立即、退出需连续 3 个新鲜样本满足释放条件；
        # yaw_rate 缺失/过期时禁止退出转向状态。
        abs_error = abs(yaw_error)
        if not self._turn_in_place_active:
            if abs_error >= self.turn_in_place_heading or (
                yaw_rate_valid and abs(yaw_rate) >= self.turn_in_place_rate
            ):
                self._turn_in_place_active = True
                self._turn_release_count = 0
        else:
            if (
                yaw_rate_valid
                and abs_error <= self.turn_release_heading
                and abs(yaw_rate) <= self.turn_release_rate
            ):
                self._turn_release_count += 1
                if self._turn_release_count >= self.turn_release_samples:
                    self._turn_in_place_active = False
                    self._turn_release_count = 0
            else:
                self._turn_release_count = 0

        linear_x, linear_y, angular = compute_tracking_command(
            yaw_error,
            max_linear=linear_limit,
            max_lateral=lateral_limit,
            max_angular=self.max_angular,
            min_linear_fraction=self.min_linear_fraction,
            path_curvature=self._debug_path_curvature,
            nearest_obstacle_distance=self._nearest_obstacle_distance,
            obstacle_stop_distance=self.obstacle_stop_distance,
            obstacle_slow_distance=self.obstacle_slow_distance,
            measured_yaw_rate=yaw_rate if yaw_rate_valid else None,
            yaw_rate_damping=self.yaw_rate_damping,
            turn_in_place_active=self._turn_in_place_active,
        )
        self._debug_turn_in_place = self._turn_in_place_active
        self._publish_locked(linear_x, angular, linear_y=linear_y)
        return RUNNING

    def _on_control_timer(self, _event=None) -> None:
        """固定频率控制定时器：路径/恢复模式每周期取新位姿重新决策。

        直接控制模式（无路径且无恢复）保持 keepalive 语义，重放最近命令，
        保证 RL 控制器与地面支撑门持续收到命令。
        """
        with self._command_transaction_lock:
            if self._publisher is None or Twist is None:
                return
            now_wall = time.monotonic()
            if self._last_decision_wall is not None:
                self._last_decision_period = now_wall - self._last_decision_wall
            self._last_decision_wall = now_wall
            if self._recover_phase != "idle":
                self._control_step_locked(None)
                return
            if self._align_state != "idle":
                provider = self._pose_provider
                if provider is None:
                    return
                sample = provider.latest()
                if sample is None:
                    self._debug_pose_age = None
                    self._publish_locked(
                        0.0, 0.0, zero_reason="controller_pose_missing"
                    )
                    return
                stamp = getattr(sample, "stamp", None)
                age = (
                    float(get_time()) - float(stamp)
                    if stamp is not None
                    else None
                )
                if age is None or not math.isfinite(age) or age < 0.0 or age > self.pose_max_age:
                    self._debug_pose_age = age
                    self._publish_locked(
                        0.0, 0.0, zero_reason="controller_pose_stale"
                    )
                    return
                self._debug_pose_age = age
                self._align_step_locked(sample)
                return
            if self._path:
                provider = self._pose_provider
                if provider is None:
                    return
                sample = provider.latest()
                if sample is None:
                    self._debug_pose_age = None
                    self._publish_locked(
                        0.0, 0.0, zero_reason="controller_pose_missing"
                    )
                    return
                stamp = getattr(sample, "stamp", None)
                age = (
                    float(get_time()) - float(stamp)
                    if stamp is not None
                    else None
                )
                if age is None or not math.isfinite(age) or age < 0.0 or age > self.pose_max_age:
                    self._debug_pose_age = age
                    self._publish_locked(
                        0.0, 0.0, zero_reason="controller_pose_stale"
                    )
                    return
                self._debug_pose_age = age
                self._control_step_locked(sample)
            else:
                self._keepalive_replay_locked()

    def step_align_for_test(self, sample) -> str:
        """离线单测用：同步执行一次对准决策；不进入生产调用链。"""
        with self._command_transaction_lock:
            return self._align_step_locked(sample)

    def _align_step_locked(self, sample) -> str:
        """锁内执行一次固定频率对准决策（SETTLE/ALIGN/CONFIRM 子状态机）。

        - SETTLE：先发零等待机器人停稳（连续 3 帧角速度与线速度小），
          超时显式锁存 STUCK(alignment_settle_timeout)；
        - ALIGN：omega = clamp(1.7*err - 0.45*yaw_rate, ±max_angular)，
          仅在误差超死区且实测转速接近零时补偿执行死区转速；
        - CONFIRM：连续 3 个新鲜样本满足释放条件才锁存 DONE(alignment_done)；
        - 锚点漂移：位姿离开锚点超半径立即零速并锁存 STUCK(alignment_drift)。
        """
        self._control_decision_count += 1
        now = get_time()
        try:
            x, y, yaw = (float(value) for value in sample.xy_yaw)
        except (TypeError, ValueError):
            self._align_fail_locked("alignment_invalid_pose")
            return STUCK
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self._align_fail_locked("alignment_invalid_pose")
            return STUCK
        yaw_rate = None
        angular = getattr(sample, "angular_xyz", None)
        if angular is not None:
            try:
                yaw_rate = float(angular[2])
            except (TypeError, ValueError, IndexError):
                yaw_rate = None
        yaw_rate_valid = yaw_rate is not None and math.isfinite(yaw_rate)
        self._debug_yaw_rate = yaw_rate
        self._debug_yaw_rate_source = "odom" if yaw_rate_valid else "missing"
        self._debug_yaw_rate_valid = yaw_rate_valid

        anchor = self._align_anchor_xy
        if anchor is not None and self._align_max_anchor_error is not None:
            anchor_error = math.hypot(anchor[0] - x, anchor[1] - y)
            self._debug_align_anchor_error = round(anchor_error, 3)
            if anchor_error > self._align_max_anchor_error:
                self._align_fail_locked("alignment_drift")
                return STUCK
        else:
            self._debug_align_anchor_error = None

        if self._align_state == "settle":
            linear_x = 0.0
            linear_xyz = getattr(sample, "linear_xyz", None)
            if linear_xyz is not None:
                try:
                    linear_x = float(linear_xyz[0])
                except (TypeError, ValueError, IndexError):
                    linear_x = 0.0
            settled = (
                yaw_rate_valid
                and abs(yaw_rate) <= self._align_release_rate
                and abs(linear_x) <= 0.05
            )
            if settled:
                self._align_settle_samples += 1
            else:
                self._align_settle_samples = 0
            self._publish_locked(0.0, 0.0, zero_reason="alignment_settle")
            if self._align_settle_samples >= 3:
                self._align_state = "align"
                self._align_settle_samples = 0
            elif now - self._align_settle_start > self._align_settle_timeout:
                self._align_fail_locked("alignment_settle_timeout")
                return STUCK
            return RUNNING

        if self._align_state == "confirm":
            align_error = self._align_error_locked(x, y, yaw)
            if (
                yaw_rate_valid
                and abs(align_error) <= self._align_release_heading
                and abs(yaw_rate) <= self._align_release_rate
            ):
                self._align_confirm_count += 1
                self._publish_locked(0.0, 0.0, zero_reason="alignment_hold")
                if self._align_confirm_count >= self._align_release_samples:
                    self._publish_locked(0.0, 0.0, zero_reason="alignment_done")
                    self._align_state = "idle"
                    self._align_target_xy = None
                    self._align_anchor_xy = None
                    self._tracking_status = DONE
                    self._terminal_pending = True
                    self._terminal_reason = "alignment_done"
                    self._debug_replan_reason = "alignment_done"
                    self._status_sequence += 1
                    return DONE
                return RUNNING
            self._align_confirm_count = 0
            self._align_state = "align"

        # ALIGN 旋转：阻尼比例控制 + 执行死区补偿
        align_error = self._align_error_locked(x, y, yaw)
        omega = 1.7 * align_error
        if yaw_rate_valid:
            omega -= float(self.yaw_rate_damping) * yaw_rate
        omega = _clamp(omega, -float(self.max_angular), float(self.max_angular))
        if abs(align_error) >= self._align_deadband_error and (
            not yaw_rate_valid or abs(yaw_rate) < 0.05
        ):
            # A1 纯转向在小角速度下存在执行死区：误差超限且实测转速接近
            # 零时，用最小可执行转速兜底，避免持续发布无效命令。
            sign = 1.0 if omega >= 0.0 else -1.0
            if abs(omega) < self._align_deadband_omega:
                omega = sign * self._align_deadband_omega
        self._publish_locked(0.0, omega, zero_reason="alignment_rotate")
        if abs(align_error) <= self._align_deadband_error:
            self._align_state = "confirm"
            self._align_confirm_count = 0
        return RUNNING

    def _align_error_locked(self, x: float, y: float, yaw: float) -> float:
        target = self._align_target_xy
        if target is None:
            return 0.0
        target_yaw = math.atan2(target[1] - y, target[0] - x)
        error = _angle_wrap(target_yaw - yaw)
        self._debug_align_yaw_error = round(float(error), 4)
        return float(error)

    def _align_fail_locked(self, reason: str) -> None:
        """锁存 ALIGN 失败终态（不立即清路径，由 poll_result 消费）。"""
        self._publish_locked(0.0, 0.0, zero_reason=reason)
        self._align_state = "idle"
        self._align_target_xy = None
        self._align_anchor_xy = None
        self._tracking_status = STUCK
        self._terminal_pending = True
        self._terminal_reason = reason
        self._debug_replan_reason = reason
        self._status_sequence += 1

    def _keepalive_replay_locked(self) -> None:
        """锁内重放最近命令（直接控制模式）；调用方必须已持有事务锁。"""
        if self._publisher is None or Twist is None:
            return
        now = time.monotonic()
        if self._last_command_wall_time is None:
            return
        minimum_age = max(0.02, self.command_keepalive_period * 0.8)
        if now - self._last_command_wall_time < minimum_age:
            return
        linear_x, linear_y, angular_z = self._last_emitted_command
        # 直接控制命令全部由任务层每 tick 重发（rotate_in_place/reverse_straight）。
        # 一旦任务结束、状态机切换或上层异常导致不再刷新，无限重放最后一条非零
        # 命令会让机器人持续运动且无人负责停止。超龄即归零，零速可以继续重放
        # （维持 RL 控制器与地面支撑门的命令流）。
        if any(abs(value) > 1e-4 for value in (linear_x, linear_y, angular_z)):
            issued = self._direct_command_wall_time
            max_age = max(0.0, float(self.direct_command_max_age))
            if max_age > 0.0 and (issued is None or now - issued > max_age):
                self._publish_locked(
                    0.0,
                    0.0,
                    zero_reason="direct_command_expired",
                )
                self.keepalive_publish_count += 1
                return
        self._publish_locked(
            linear_x,
            angular_z,
            linear_y=linear_y,
            zero_reason=self._debug_zero_command_reason,
        )
        self.keepalive_publish_count += 1

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

    def _apply_deadzone_compensation(
        self,
        angular: float,
        *,
        deadband_omega: float = 0.55,
        deadband_threshold: float = 1e-6,
    ) -> float:
        """统一的死区补偿：小角速度提升到最小可执行转速。

        A1 电机在小角速度下存在执行死区，控制器发出的小转速命令
        实际无法执行。该方法将绝对值小于阈值的非零角速度提升到
        最小可执行转速，保持原符号方向。

        Args:
            angular: 原始角速度命令 (rad/s)
            deadband_omega: 最小可执行角速度 (rad/s)，默认 0.55
            deadband_threshold: 判断为零的阈值，默认 1e-6

        Returns:
            补偿后的角速度命令 (rad/s)
        """
        if abs(angular) <= deadband_threshold:
            # 零速命令不补偿
            return angular

        if abs(angular) < deadband_omega:
            # 小角速度补偿到最小可执行转速
            sign = 1.0 if angular >= 0.0 else -1.0
            return sign * deadband_omega

        # 大角速度不需要补偿
        return angular

    def rotate_in_place(self, angular_z: float) -> None:
        angular = _clamp(float(angular_z), -self.max_angular, self.max_angular)

        # A1 纯转向在小角速度下存在执行死区：用最小可执行转速兜底，
        # 避免任务层直接控制时发布无效命令。
        deadband_omega = 0.55
        if abs(angular) > 1e-6 and abs(angular) < deadband_omega:
            sign = 1.0 if angular >= 0.0 else -1.0
            angular = sign * deadband_omega

        self._publish(
            0.0,
            angular,
            zero_reason="rotate_in_place_zero",
        )

    def reverse_straight(self, *, reason: str = "reverse_straight") -> bool:
        """按既有倒车配置发布直线速度；由上层负责空间与距离约束。"""
        self._debug_recover_reason = str(reason)
        speed = _clamp(float(self.reverse_speed), -abs(self.max_linear), 0.0)
        if speed >= -1e-4:
            self.stop(reason=f"{reason}_invalid_speed")
            return False
        self._publish(speed, 0.0)
        return True

    def stop(self, repeat: int = 1, interval: float = 0.02, *, reason: str = "stop") -> None:
        count = max(1, int(repeat))
        for index in range(count):
            self._publish(0.0, 0.0, zero_reason=reason)
            if index < count - 1:
                time.sleep(max(0.0, float(interval)))

    def _stop_locked(self, *, reason: str) -> None:
        """锁内发送一次零速；调用方必须已持有事务锁。"""
        self._publish_locked(0.0, 0.0, zero_reason=reason)

    def _remaining_path_distance(self, x: float, y: float) -> float:
        """沿活动折线从当前位置投影点到终点的剩余距离（进度监督用）。

        只统计当前段投影后的剩余长度与后续折线长度，绕圈/后退不改变该值。
        """
        if not self._path:
            return 0.0
        if len(self._path) == 1:
            return math.hypot(self._path[0][0] - x, self._path[0][1] - y)
        index = min(self._waypoint_index, len(self._path) - 2)
        ax, ay = self._path[index]
        bx, by = self._path[index + 1]
        vx, vy = bx - ax, by - ay
        seg_len = math.hypot(vx, vy)
        if seg_len <= 1e-6:
            t = 0.0
        else:
            t = _clamp(
                ((x - ax) * vx + (y - ay) * vy) / (seg_len * seg_len),
                0.0,
                1.0,
            )
        remaining = seg_len * (1.0 - t)
        for i in range(index + 1, len(self._path) - 1):
            remaining += math.hypot(
                self._path[i + 1][0] - self._path[i][0],
                self._path[i + 1][1] - self._path[i][1],
            )
        return float(remaining)

    def _lookahead_target(self, x: float, y: float) -> tuple[float, float]:
        if len(self._path) == 1:
            return self._path[0]
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
        """过滤并发布一条速度命令，是生产控制的最终出口。

        生产状态只在 ``publish`` 正常返回后提交。publisher 尚未启动时只更新
        离线诊断值，不把它视为已经发送的活动运动命令。
        """
        with self._command_transaction_lock:
            # _publish 是直接控制命令的唯一外部入口（固定频率决策走
            # _publish_locked）。在这里打时间戳，keepalive 才能区分
            # “任务层仍在每 tick 刷新” 与 “命令已失去刷新方”。
            self._direct_command_wall_time = time.monotonic()
            self._publish_locked(
                linear_x,
                angular_z,
                linear_y=linear_y,
                zero_reason=zero_reason,
            )

    def _publish_locked(
        self,
        linear_x: float,
        angular_z: float,
        *,
        linear_y: float = 0.0,
        zero_reason: str | None = None,
    ) -> None:
        """在 ``_command_transaction_lock`` 内完成一次最终发送事务。

        仲裁（决定发什么、归因给谁）已收口到模块级纯函数 ``arbitrate_command``；
        本方法只负责事务性副作用：提交请求快照、发布、以及只描述“已成功交给
        publisher 的命令”的那批统计字段。
        """
        requested, zero_reason = sanitize_command_request(
            linear_x, angular_z, linear_y, zero_reason
        )
        self._last_requested_command = requested
        gate = self._command_gate
        wall_now = time.monotonic()
        emitted, resolved_zero_reason = arbitrate_command(
            requested,
            zero_reason=zero_reason,
            gate=gate,
            publish_failure_latched=self._publish_failure_latched,
            publish_failure_reason=self._publish_failure_reason,
            pose_yaw=self._debug_pose[2] if self._debug_pose is not None else 0.0,
            wall_time=wall_now,
            control_time_source=get_time,
        )
        if self._publisher is None or Twist is None:
            self._last_emitted_command = emitted
            self._debug_zero_command_reason = resolved_zero_reason
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = emitted
        try:
            self._publisher.publish(msg)
        except Exception as exc:
            self._publish_failure_latched = True
            self._publish_failure_reason = "command_publish_failure_latched"
            self._debug_zero_command_reason = self._publish_failure_reason
            raise RuntimeError("最终 /cmd_vel 发布失败，控制输出已锁存为零速") from exc

        # 以下字段只描述已经成功交给 ROS publisher 的命令。
        self._last_emitted_command = emitted
        self._debug_zero_command_reason = resolved_zero_reason
        self.command_count += 1
        if abs(msg.linear.x) > 1e-4 or abs(msg.linear.y) > 1e-4:
            self.nonzero_linear_count += 1
        self.max_abs_linear_x = max(self.max_abs_linear_x, abs(msg.linear.x))
        self.max_abs_linear_y = max(self.max_abs_linear_y, abs(msg.linear.y))
        self.max_abs_angular_z = max(self.max_abs_angular_z, abs(msg.angular.z))
        self.last_command = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_command_wall_time = wall_now
        if any(abs(value) > 1e-4 for value in emitted):
            self._active_motion_command = emitted
            self._active_motion_stop_samples = 0

    def raw_path_snapshot(self) -> list[tuple[float, float]]:
        return list(self._raw_path)

    def followed_path_snapshot(self) -> list[tuple[float, float]]:
        return list(self._path)

    def requested_command_snapshot(self) -> tuple[float, float, float]:
        with self._command_transaction_lock:
            return tuple(self._last_requested_command)

    def committed_command_snapshot(self) -> tuple[float, float, float]:
        """返回最近一条 publisher 正常返回的命令。"""
        with self._command_transaction_lock:
            return tuple(self.last_command)

    def active_motion_command_snapshot(self) -> tuple[float, float, float]:
        """返回最后成功提交的非零命令，直到新鲜里程计确认停稳。"""
        with self._command_transaction_lock:
            return tuple(self._active_motion_command)

    def observe_motion_odometry(
        self,
        stamp: float | None,
        *,
        linear_speed: float,
        angular_speed: float,
        wall_time: float,
    ) -> None:
        """在同一事务内更新 Gate，并维护制动期间的活动运动命令。"""
        with self._command_transaction_lock:
            gate = self._command_gate
            observe = getattr(gate, "observe_odometry", None)
            if callable(observe):
                observe(
                    stamp,
                    linear_speed=linear_speed,
                    angular_speed=angular_speed,
                    wall_time=wall_time,
                )
            try:
                resolved_stamp = float(stamp) if stamp is not None else math.nan
                resolved_linear = abs(float(linear_speed))
                resolved_angular = abs(float(angular_speed))
            except (TypeError, ValueError):
                self._active_motion_stop_samples = 0
                return
            if not all(
                math.isfinite(value)
                for value in (resolved_stamp, resolved_linear, resolved_angular)
            ):
                self._active_motion_stop_samples = 0
                return
            if (
                self._active_motion_last_odom_stamp is not None
                and resolved_stamp <= self._active_motion_last_odom_stamp
            ):
                return
            self._active_motion_last_odom_stamp = resolved_stamp
            config = getattr(gate, "config", None)
            linear_limit = float(getattr(config, "stop_linear_threshold", 0.05))
            angular_limit = float(getattr(config, "stop_angular_threshold", 0.05))
            required = max(1, int(getattr(config, "stop_confirmation_samples", 3)))
            last_success_is_zero = all(abs(value) <= 1e-4 for value in self.last_command)
            stopped = (
                last_success_is_zero
                and resolved_linear <= linear_limit
                and resolved_angular <= angular_limit
            )
            self._active_motion_stop_samples = (
                self._active_motion_stop_samples + 1 if stopped else 0
            )
            if self._active_motion_stop_samples >= required:
                self._active_motion_command = (0.0, 0.0, 0.0)

    def _publish_path_diagnostics(self) -> None:
        publishers = (self._raw_path_publisher, self._followed_path_publisher)
        if (
            rospy is None
            or PathMessage is None
            or PoseStamped is None
            or all(pub is None for pub in publishers)
        ):
            return
        try:
            stamp = rospy.Time.now()
        except rospy.exceptions.ROSInitException:
            return
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
        with self._command_transaction_lock:
            return self._tracking_debug_snapshot_locked()

    def _tracking_debug_snapshot_locked(self) -> dict[str, object]:
        now = get_time()
        wall_now = time.monotonic()
        recover_remaining = max(0.0, self._recover_until - now)
        wall_command_age = (
            max(0.0, wall_now - self._last_command_wall_time)
            if self._last_command_wall_time is not None
            else None
        )
        requested_linear_x, requested_linear_y, requested_angular_z = self._last_requested_command
        linear_x, linear_y, angular_z = self._last_emitted_command
        gate_snapshot = getattr(self._command_gate, "snapshot", None)
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
            "requested_linear_command": requested_linear_x,
            "requested_lateral_command": requested_linear_y,
            "requested_angular_command": requested_angular_z,
            "safety_gate": gate_snapshot() if callable(gate_snapshot) else None,
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
            "active_motion_command": [
                round(value, 3) for value in self._active_motion_command
            ],
            "active_motion_stop_samples": self._active_motion_stop_samples,
            "publish_failure_latched": self._publish_failure_latched,
            "publish_failure_reason": self._publish_failure_reason,
            "control_decision_count": self._control_decision_count,
            "control_decision_period": _finite_or_none(self._last_decision_period),
            "pose_age": _finite_or_none(self._debug_pose_age),
            "pose_max_age": self.pose_max_age,
            "yaw_rate": _finite_or_none(self._debug_yaw_rate),
            "yaw_rate_source": self._debug_yaw_rate_source,
            "yaw_rate_valid": self._debug_yaw_rate_valid,
            "turn_in_place_active": self._turn_in_place_active,
            "turn_release_count": self._turn_release_count,
            "tracking_status": self._tracking_status,
            "status_sequence": self._status_sequence,
            "terminal_pending": self._terminal_pending,
            "terminal_reason": self._terminal_reason,
            "route_progress": _finite_or_none(self._debug_route_progress),
            "controller_mode": (
                "RECOVER"
                if self._recover_phase != "idle"
                else (
                    "ALIGN"
                    if self._align_state != "idle"
                    else ("PATH" if self._path else "IDLE")
                )
            ),
            "align_state": self._align_state,
            "align_yaw_error": _finite_or_none(self._debug_align_yaw_error),
            "align_anchor_error": _finite_or_none(self._debug_align_anchor_error),
            "align_confirm_count": self._align_confirm_count,
            "align_settle_samples": self._align_settle_samples,
            "align_settle_elapsed": (
                round(
                    max(0.0, get_time() - self._align_settle_start), 2
                )
                if self._align_state in ("settle", "align", "confirm")
                else None
            ),
        }

    def command_debug_summary(self) -> dict[str, object]:
        with self._command_transaction_lock:
            return {
                "command_count": self.command_count,
                "nonzero_linear_count": self.nonzero_linear_count,
                "max_abs_linear_x": round(self.max_abs_linear_x, 3),
                "max_abs_linear_y": round(self.max_abs_linear_y, 3),
                "max_abs_angular_z": round(self.max_abs_angular_z, 3),
                "last_command": [round(value, 3) for value in self.last_command],
                "active_motion_command": [
                    round(value, 3) for value in self._active_motion_command
                ],
                "stuck_count": self.stuck_count,
                "recover_count": self.recover_count,
                "keepalive_publish_count": self.keepalive_publish_count,
                "publish_failure_latched": self._publish_failure_latched,
                "tracking": self._tracking_debug_snapshot_locked(),
            }


@dataclass(frozen=True)
class ControlResult:
    """一次控制轮询的原子结果：状态、终态原因与序号同时返回。

    ``reason`` 与 ``status`` 在同一把锁内捕获并消费，读取后不会被控制
    定时器的后续更新覆盖；``sequence`` 单调递增，用于任务层区分新旧终态。
    """

    status: str
    reason: str | None
    sequence: int


class _SyncPoseSample:
    """测试/兼容 tick 用的同步位姿样本（无有符号角速度来源）。"""

    def __init__(self, pose_xy_yaw):
        self._xy = tuple(float(value) for value in pose_xy_yaw)
        self.angular_xyz = (0.0, 0.0, 0.0)
        self.stamp = None

    @property
    def xy_yaw(self) -> tuple[float, float, float]:
        return self._xy


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


def sanitize_command_request(
    linear_x: float,
    angular_z: float,
    linear_y: float,
    zero_reason: str | None,
) -> tuple[tuple[float, float, float], str | None]:
    """把原始命令消毒为有限三元组；非法请求归零并改写归因。

    与 ``arbitrate_command`` 分开的原因：调用方必须在消毒之后、仲裁之前提交
    ``_last_requested_command``。该诊断字段的语义是“任务层请求的命令”，不是
    “门过滤后的命令”，两者合并会让请求快照被门结果污染。
    """
    try:
        requested = (float(linear_x), float(linear_y), float(angular_z))
    except (TypeError, ValueError):
        return (0.0, 0.0, 0.0), "invalid_requested_command"
    if not all(math.isfinite(value) for value in requested):
        return (0.0, 0.0, 0.0), "invalid_requested_command"
    return requested, zero_reason


def arbitrate_command(
    requested: tuple[float, float, float],
    *,
    zero_reason: str | None,
    gate,
    publish_failure_latched: bool,
    publish_failure_reason: str | None,
    pose_yaw: float,
    wall_time: float,
    control_time_source,
) -> tuple[tuple[float, ...], str | None]:
    """命令写入权的唯一仲裁点：决定最终发出的命令与零速归因。

    归因优先级（后者覆盖前者）：

    1. 调用方传入的 ``zero_reason``；
    2. 发布失败锁存 —— 一旦锁存即恒零，且**跳过门**（门不再有发言权）；
    3. 门过滤结果；
    4. 门畸形输出（抛异常/非三元组/非有限）→ ``invalid_command_gate_output``；
    5. 门致零归因 —— 仅当 ``emitted != requested`` 时才归责于门。

    第 5 条是本函数最容易被误改的语义：任务层自己请求零速、门也输出零速时
    ``emitted == requested``，归因必须保留调用方的 reason，否则“谁让 dog 停的”
    会被错误记成安全门致停，把任务层的主动停车伪装成安全事件。

    ``control_time_source`` 传 callable 而不是求值结果：原实现只在有门、filter
    可调用且未锁存时才读任务时间，而 ROS 时间不可用时 ``get_time`` 会抛错。
    提前求值会给无门场景凭空引入一个新的失败面。
    """
    emitted: tuple[float, ...] = (
        (0.0, 0.0, 0.0) if publish_failure_latched else requested
    )
    if publish_failure_latched:
        zero_reason = publish_failure_reason or "command_publish_failure_latched"
    if gate is not None and not publish_failure_latched:
        filter_command = getattr(gate, "filter_command", None)
        if callable(filter_command):
            try:
                filtered = filter_command(
                    requested,
                    pose_yaw=pose_yaw,
                    wall_time=wall_time,
                    control_time=control_time_source(),
                )
                emitted = tuple(float(value) for value in filtered)
            except Exception:
                # 最终安全门异常时仍必须覆盖上一条保活命令并发送零速。
                emitted = ()
            if len(emitted) != 3 or not all(
                math.isfinite(value) for value in emitted
            ):
                emitted = (0.0, 0.0, 0.0)
                zero_reason = "invalid_command_gate_output"
    resolved_zero_reason = None
    if all(abs(value) <= 1e-4 for value in emitted):
        if (
            emitted != requested
            and gate is not None
            and not publish_failure_latched
        ):
            reason = getattr(gate, "reason_code", None) or "ground_support_gate"
            zero_reason = f"ground_support_gate:{reason}"
        resolved_zero_reason = str(zero_reason or "zero_command")
    return emitted, resolved_zero_reason


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
    measured_yaw_rate: float | None = None,
    yaw_rate_damping: float = 0.45,
    turn_in_place_heading: float = 0.70,
    turn_in_place_rate: float = 0.45,
    turn_in_place_active: bool = False,
    deadband_omega: float = 0.55,
    deadband_error: float = 0.10,
    deadband_rate_threshold: float = 0.05,
) -> tuple[float, float, float]:
    """按航向、曲率和前向障碍共同限制速度，不对全局速度一刀切。

    前进速度取三种缩放中的最小值；侧移帮助 A1 在有限转向时对准目标；角速度
    按航向误差比例控制，并在提供新鲜实测角速度时叠加阻尼项，抑制转向过冲。
    原地转向状态由调用方（带滞回）维护：``turn_in_place_active`` 为真时
    禁止前进与侧移，只输出转向。
    """
    angular = 1.7 * yaw_error
    if measured_yaw_rate is not None and math.isfinite(measured_yaw_rate):
        angular -= float(yaw_rate_damping) * measured_yaw_rate
    angular = _clamp(angular, -float(max_angular), float(max_angular))

    # A1 纯转向在小角速度下存在执行死区：误差超限且实测转速接近零时，
    # 用最小可执行转速兜底，避免持续发布无效命令。
    if abs(yaw_error) >= deadband_error and (
        measured_yaw_rate is None or abs(measured_yaw_rate) < deadband_rate_threshold
    ):
        sign = 1.0 if angular >= 0.0 else -1.0
        if abs(angular) < deadband_omega:
            angular = sign * deadband_omega
    min_fraction = _clamp(float(min_linear_fraction), 0.0, 1.0)
    abs_error = abs(yaw_error)
    normalized_error = min(1.0, abs_error / (math.pi / 2.0))
    yaw_scale = (1.0 - normalized_error) ** 2
    if abs_error <= 0.15:
        yaw_scale = max(min_fraction, yaw_scale)
    elif abs_error >= math.pi / 2.0:
        # 保留极低爬行量以兼容不能响应纯角速度的 RL 控制器。
        yaw_scale = min(0.05, min_fraction)

    # 先对准再前进：原地转向状态由调用方滞回维护，此处只执行。
    if turn_in_place_active:
        yaw_scale = 0.0

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
    if turn_in_place_active:
        lateral_scale = 0.0
    else:
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

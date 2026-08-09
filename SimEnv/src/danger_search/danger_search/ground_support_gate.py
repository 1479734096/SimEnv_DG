"""最终速度出口前的局部地面支撑安全状态机与停车包络。

该模块不修改占据地图，也不负责重规划。它只把已验证的局部支撑评估转换为
最终速度钳制状态；关闭时必须完全透明。

状态分三类：``CLEAR`` 透明放行；``OBSERVATION_HOLD/STALE_DATA_HOLD`` 是可由
新鲜正支撑自动解除的临时零速；DROP、超包络速度和停车超时进入锁存状态，
必须由任务层写入局部禁区、重规划或结束任务。墙钟与控制/仿真时间同时保存，
从而在 ``/clock`` 冻结时仍能检测“已要求停车但无法确认停稳”的异常。
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable


CLEAR = "CLEAR"
OBSERVATION_HOLD = "OBSERVATION_HOLD"
STALE_DATA_HOLD = "STALE_DATA_HOLD"
DROP_STOPPING = "DROP_STOPPING"
SAFETY_BLOCKED = "SAFETY_BLOCKED"
SPEED_OUT_OF_ENVELOPE_STOPPING = "SPEED_OUT_OF_ENVELOPE_STOPPING"
SAFETY_STOP_TIMEOUT = "SAFETY_STOP_TIMEOUT"
SUPPORT_PATH_UNAVAILABLE = "SUPPORT_PATH_UNAVAILABLE"

_TEMPORARY_HOLDS = {OBSERVATION_HOLD, STALE_DATA_HOLD}
_STOPPING_STATES = {DROP_STOPPING, SPEED_OUT_OF_ENVELOPE_STOPPING}
_LATCHED_STATES = _STOPPING_STATES | {
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SUPPORT_PATH_UNAVAILABLE,
}


class ConservativeStopEnvelope:
    """按请求速度查工程保守停车距离，不插值且绝不向有效域外外推。

    当前表项尚待两轮独立测量封板。向上取档保证同一档内使用更保守的距离；
    超过配置最大速度时返回 ``None``，调用方立即零速并进入超包络处理，不能
    凭数学外推宣称安全。``validated_max_speed`` 名称为兼容既有配置而保留。
    """

    def __init__(
        self,
        table: Iterable[tuple[float, float]],
        *,
        validated_max_speed: float | None = None,
    ) -> None:
        resolved = sorted((float(speed), float(distance)) for speed, distance in table)
        if not resolved:
            raise ValueError("停车包络表不能为空")
        self.validated_max_speed = (
            float(resolved[-1][0])
            if validated_max_speed is None
            else float(validated_max_speed)
        )
        if not math.isfinite(self.validated_max_speed) or self.validated_max_speed <= 0.0:
            raise ValueError("validated_max_speed 必须是正有限数")
        previous_speed = 0.0
        previous_distance = 0.0
        for speed, distance in resolved:
            if not math.isfinite(speed) or speed <= previous_speed:
                raise ValueError("停车包络速度必须严格递增")
            if not math.isfinite(distance) or distance < previous_distance:
                raise ValueError("停车距离必须有限且单调不减")
            previous_speed = speed
            previous_distance = distance
        if resolved[-1][0] + 1e-9 < self.validated_max_speed:
            raise ValueError("停车包络表未覆盖 validated_max_speed")
        self.table = tuple(resolved)

    def distance_for(self, speed: float) -> float | None:
        value = abs(float(speed))
        if not math.isfinite(value):
            return None
        if value <= 1e-9:
            return 0.0
        if value > self.validated_max_speed + 1e-9:
            return None
        for upper_speed, distance in self.table:
            if value <= upper_speed + 1e-9:
                return distance
        # 构造函数已保证表覆盖有效域；保守地拒绝异常配置，而不是外推。
        return None


@dataclass(frozen=True)
class GroundSupportGateConfig:
    """停车包络、数据新鲜度、释放滞回和停稳确认参数。"""
    enabled: bool = False
    validated_max_speed: float | None = None
    stop_distance_table: tuple[tuple[float, float], ...] = (
        (0.60, 0.09337727739487302),
        (0.651, 0.10195826367975175),
    )
    localization_speed_anomaly_threshold: float = 1.0
    localization_speed_anomaly_timeout: float = 2.0
    drop_evidence_max_age: float = 0.05
    decision_period: float = 0.10
    rl_receive_delay: float = 0.009
    required_margin: float = 0.02
    boundary_hold_margin: float = 0.0
    boundary_persistence_confirmations: int = 3
    boundary_consistency_radius: float = 0.20
    release_hysteresis: float = 0.10
    release_supported_frames: int = 2
    observation_hold_timeout: float = 2.0
    safety_stop_timeout: float = 3.0
    wall_stall_timeout: float = 30.0
    odometry_linear_deadband: float = 0.05
    odometry_angular_deadband: float = 0.03
    stop_linear_threshold: float = 0.05
    stop_angular_threshold: float = 0.05
    stop_confirmation_samples: int = 3


@dataclass(frozen=True)
class GroundSupportAssessment:
    """由支撑网格生成的一次不可变局部走廊评估。

    ``drop_confirmed`` 表示有明确负支撑证据；``observation_adequate=False`` 只表示
    当前无法判断，二者不可混用。方向和交点用于确认当前命令是否确实驶向缺口。
    """
    stamp: float
    received_wall_time: float
    data_age: float
    supported_distance: float | None
    current_footprint_supported: bool
    observation_adequate: bool
    drop_confirmed: bool
    drop_intersects_sweep: bool
    drop_distance: float | None
    drop_direction_world: tuple[float, float] | None
    received_control_time: float | None = None
    support_boundary_detected: bool = False
    drop_point_world: tuple[float, float] | None = None
    support_boundary_cells_world: tuple[tuple[float, float], ...] = ()
    hold_context_id: int = 0
    support_command: tuple[float, float, float] | None = None
    support_lookahead_distance: float | None = None
    boundary_observable: bool | None = None
    boundary_reason_counts: tuple[tuple[str, int], ...] = ()
    floor: int = 0
    map_epoch: int = 0
    localization_epoch: int = 0
    cloud_generation: int = 0
    support_boundary_cells: tuple[tuple[int, int], ...] = ()
    grid_origin: tuple[float, float] | None = None
    grid_resolution: float | None = None
    drop_clusters: tuple["DropClusterEvidence", ...] = ()


@dataclass(frozen=True)
class DropClusterEvidence:
    cluster_id: str
    floor: int
    frame_id: str
    map_epoch: int
    localization_epoch: int
    stamp: float
    generation: int
    grid_origin: tuple[float, float]
    grid_resolution: float
    source_cells: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class NegativeClusterAck:
    floor: int
    map_epoch: int
    localization_epoch: int
    cluster_ids: tuple[str, ...]
    map_revision: int


@dataclass(frozen=True)
class SupportFailureWitness:
    """锁存触发 HOLD 的完整失败断面，供规划约束事务消费。"""

    sequence: int
    floor: int
    map_epoch: int
    localization_epoch: int
    stamp: float
    generation: int
    source_cells: tuple[tuple[int, int], ...]
    hold_context_id: int
    trigger_command: tuple[float, float, float]
    grid_origin: tuple[float, float]
    grid_resolution: float
    frame_id: str = "odom"


@dataclass(frozen=True)
class SupportConstraintAck:
    """规划约束安装完成后的强确认，禁止跨 scope 或旧 revision 复位 Gate。"""

    witness_sequence: int
    floor: int
    map_epoch: int
    localization_epoch: int
    constraint_revision: int


class GroundSupportSafetyGate:
    """只在最终命令出口过滤速度的锁存状态机。

    它同时观察算法请求速度、定位差分速度和最新支撑评估。停车表只按请求速度
    选档，因为定位差分包含配准校正，不能作为物理速度上界；定位速度仅用于多帧
    停稳确认和有界异常观察。DROP 必须新鲜、观测充分、与未来扫掠区相交且
    当前命令朝向缺口才锁存；UNKNOWN 只触发有限观察停车。关闭配置时
    ``filter_command`` 原样返回输入，保持控制链完全透明。
    """

    def __init__(self, config: GroundSupportGateConfig | None = None) -> None:
        self.config = config or GroundSupportGateConfig()
        finite_nonnegative = (
            self.config.drop_evidence_max_age,
            self.config.decision_period,
            self.config.rl_receive_delay,
            self.config.required_margin,
            self.config.boundary_hold_margin,
            self.config.boundary_consistency_radius,
            self.config.release_hysteresis,
            self.config.observation_hold_timeout,
            self.config.safety_stop_timeout,
            self.config.wall_stall_timeout,
            self.config.odometry_linear_deadband,
            self.config.odometry_angular_deadband,
            self.config.stop_linear_threshold,
            self.config.stop_angular_threshold,
        )
        if (
            not all(
                math.isfinite(float(value)) and float(value) >= 0.0
                for value in finite_nonnegative
            )
            or self.config.decision_period <= 0.0
            or self.config.observation_hold_timeout <= 0.0
            or self.config.safety_stop_timeout <= 0.0
            or self.config.wall_stall_timeout <= 0.0
            or int(self.config.release_supported_frames) <= 0
            or int(self.config.stop_confirmation_samples) <= 0
            or not math.isfinite(self.config.localization_speed_anomaly_threshold)
            or self.config.localization_speed_anomaly_threshold <= 0.0
            or not math.isfinite(self.config.localization_speed_anomaly_timeout)
            or self.config.localization_speed_anomaly_timeout <= 0.0
            or int(self.config.boundary_persistence_confirmations) <= 0
            or self.config.stop_linear_threshold
            < self.config.odometry_linear_deadband
            or self.config.stop_angular_threshold
            < self.config.odometry_angular_deadband
        ):
            raise ValueError(
                "地面支撑安全门时序、阈值、确认次数或里程计死区契约无效"
            )
        self.envelope = ConservativeStopEnvelope(
            self.config.stop_distance_table,
            validated_max_speed=self.config.validated_max_speed,
        )
        self.state = CLEAR
        self.reason_code: str | None = None
        self._assessment: GroundSupportAssessment | None = None
        self._assessment_sequence = 0
        self._last_release_sequence = 0
        self._fresh_supported_sequence_count = 0
        self._state_enter_wall_time: float | None = None
        self._state_enter_control_time: float | None = None
        self._last_control_time: float | None = None
        self._last_progress_wall_time: float | None = None
        self._last_odom_stamp: float | None = None
        self._actual_linear_speed = 0.0
        self._actual_angular_speed = 0.0
        self._speed_out_of_envelope_sample_count = 0
        self._speed_out_of_envelope_confirmed = False
        self._speed_out_of_envelope_trigger_source: str | None = None
        self._speed_out_of_envelope_trigger_requested_speed: float | None = None
        self._speed_out_of_envelope_trigger_actual_speed: float | None = None
        self._stopped_sample_count = 0
        self._last_requested_speed = 0.0
        self._temporary_hold_command: tuple[float, float, float] | None = None
        self._temporary_hold_pose_yaw: float | None = None
        self._hold_context_sequence = 0
        self._temporary_hold_context_id: int | None = None
        self._unknown_support_hold_enabled = True
        self._latched_drop_point_world: tuple[float, float] | None = None
        self._latched_drop_clusters: tuple[DropClusterEvidence, ...] = ()
        self._boundary_candidate_centroid: tuple[float, float] | None = None
        self._boundary_confirmation_count = 0
        self._latched_boundary_cells_world: tuple[tuple[float, float], ...] = ()
        self._pending_failure_witness: SupportFailureWitness | None = None
        self._failure_witness_sequence = 0
        self._last_constraint_ack_revision = 0
        self.last_stop_distance: float | None = 0.0
        self.true_drop_stop_count = 0
        self.observation_hold_count = 0
        self.observation_hold_release_without_witness_count = 0
        self.stale_data_hold_count = 0
        self.speed_out_of_envelope_count = 0
        self.automatic_resume_count = 0
        self.total_hold_time = 0.0

    @property
    def enabled(self) -> bool:
        return bool(self.config.enabled)

    @property
    def terminal(self) -> bool:
        return self.state in {
            SAFETY_BLOCKED,
            SAFETY_STOP_TIMEOUT,
            SUPPORT_PATH_UNAVAILABLE,
        }

    @property
    def command_blocked(self) -> bool:
        return self.state != CLEAR

    def update_assessment(self, assessment: GroundSupportAssessment) -> None:
        """接收时间单调的新评估并累计连续正支撑帧，用于滞回释放。"""
        if not self.enabled:
            return
        if not self._assessment_has_valid_numbers(assessment):
            # 畸形证据不能继续保留上一帧“安全”结论；清空后由唯一出口进入
            # UNKNOWN 临时停车，等待新的有限数据恢复。
            self._assessment = None
            self._fresh_supported_sequence_count = 0
            self._reset_boundary_candidate()
            return
        if self._assessment is not None and assessment.stamp <= self._assessment.stamp:
            return
        self._assessment = assessment
        if self.state in _TEMPORARY_HOLDS:
            self._capture_failure_witness(assessment)
        self._assessment_sequence += 1
        support_context_matches = (
            self.state not in _TEMPORARY_HOLDS
            or self._assessment_matches_temporary_hold(assessment)
        )
        if support_context_matches:
            self._update_boundary_candidate(assessment)
        else:
            self._reset_boundary_candidate()
        control_time = (
            assessment.received_control_time
            if assessment.received_control_time is not None
            else assessment.stamp
        )
        self._note_control_progress(control_time, assessment.received_wall_time)
        if (
            support_context_matches
            and assessment.observation_adequate
            and assessment.current_footprint_supported
            and assessment.data_age <= self.config.drop_evidence_max_age
        ):
            self._fresh_supported_sequence_count += 1
        else:
            self._fresh_supported_sequence_count = 0

    def set_unknown_support_hold_enabled(self, enabled: bool) -> None:
        """切换 UNKNOWN 临时观察停车；显式 DROP 和速度包络始终生效。"""
        if not self.enabled:
            return
        self._unknown_support_hold_enabled = bool(enabled)

    def support_sweep_context_snapshot(
        self,
        fallback_command: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], int]:
        """返回本帧 RequiredSupportSweep 必须绑定的命令和 HOLD 上下文。"""
        if (
            self.state in _TEMPORARY_HOLDS
            and self._temporary_hold_command is not None
            and self._temporary_hold_context_id is not None
        ):
            return (
                tuple(self._temporary_hold_command),
                int(self._temporary_hold_context_id),
            )
        try:
            command = tuple(float(value) for value in fallback_command)
        except (TypeError, ValueError):
            command = ()
        if len(command) != 3 or not all(math.isfinite(value) for value in command):
            command = (0.0, 0.0, 0.0)
        return command, 0

    def observe_odometry(
        self,
        stamp: float | None,
        *,
        linear_speed: float,
        angular_speed: float,
        wall_time: float,
    ) -> None:
        """用定位差分速度确认停稳，并对异常大值执行有界观察停车。"""
        if not self.enabled:
            return
        try:
            resolved_linear = abs(float(linear_speed))
            resolved_angular = abs(float(angular_speed))
        except (TypeError, ValueError):
            resolved_linear = math.inf
            resolved_angular = math.inf
        self._actual_linear_speed = (
            resolved_linear if math.isfinite(resolved_linear) else math.inf
        )
        self._actual_angular_speed = (
            resolved_angular if math.isfinite(resolved_angular) else math.inf
        )
        stamp_value = float(stamp) if stamp is not None else math.nan
        is_fresh = math.isfinite(stamp_value) and (
            self._last_odom_stamp is None or stamp_value > self._last_odom_stamp
        )
        if is_fresh:
            self._last_odom_stamp = stamp_value
            self._note_control_progress(stamp_value, wall_time)
            localization_speed_anomaly = (
                self._actual_linear_speed
                > self.config.localization_speed_anomaly_threshold
            )
            if localization_speed_anomaly:
                self._speed_out_of_envelope_sample_count += 1
                if (
                    self.state == SPEED_OUT_OF_ENVELOPE_STOPPING
                    and self._speed_out_of_envelope_trigger_source
                    == "localization_odometry"
                    and self._state_enter_control_time is not None
                    and stamp_value - self._state_enter_control_time
                    >= self.config.localization_speed_anomaly_timeout
                ):
                    self._speed_out_of_envelope_confirmed = True
                    self._enter_state(
                        SUPPORT_PATH_UNAVAILABLE,
                        "localization_speed_abnormal",
                        wall_time,
                        control_time=stamp_value,
                    )
                    return
            else:
                self._speed_out_of_envelope_sample_count = 0
                if (
                    self.state == SPEED_OUT_OF_ENVELOPE_STOPPING
                    and self._speed_out_of_envelope_trigger_source
                    == "localization_odometry"
                ):
                    # 位姿差分速度不能证明物理超速；异常期间立即零速，但新鲜
                    # 正常样本到达后自动恢复。持续异常仍由双时钟 watchdog
                    # 有界失败，不能无限停车。
                    self._leave_speed_out_of_envelope_hold(
                        wall_time,
                        control_time=stamp_value,
                    )
                    return
        if self.state not in _STOPPING_STATES:
            self._stopped_sample_count = 0
            return
        if not is_fresh:
            return
        if (
            self.state == SPEED_OUT_OF_ENVELOPE_STOPPING
            and self._speed_out_of_envelope_trigger_source
            == "localization_odometry"
        ):
            return
        stopped = (
            self._actual_linear_speed <= self.config.stop_linear_threshold
            and self._actual_angular_speed <= self.config.stop_angular_threshold
        )
        self._stopped_sample_count = self._stopped_sample_count + 1 if stopped else 0
        if self._stopped_sample_count < max(1, int(self.config.stop_confirmation_samples)):
            return
        if self.state == DROP_STOPPING:
            self._enter_state(
                SAFETY_BLOCKED,
                "negative_obstacle",
                wall_time,
                control_time=self._last_control_time,
            )
        else:
            self._enter_state(
                SUPPORT_PATH_UNAVAILABLE,
                "support_speed_out_of_envelope",
                wall_time,
                control_time=self._last_control_time,
            )

    def check_control_timeout(
        self,
        control_time: float,
        *,
        wall_time: float,
    ) -> None:
        """按仿真时间推进正常观察和停车上限。"""
        if not self.enabled:
            return
        self._note_control_progress(control_time, wall_time)
        if self._state_enter_control_time is None:
            return
        elapsed = max(0.0, float(control_time) - self._state_enter_control_time)
        if self.state in _STOPPING_STATES and elapsed >= self.config.safety_stop_timeout:
            self._enter_state(
                SAFETY_STOP_TIMEOUT,
                "ground_support_safety_stop_timeout",
                wall_time,
                control_time=control_time,
            )
        elif self.state in _TEMPORARY_HOLDS and elapsed >= self.config.observation_hold_timeout:
            if self._pending_failure_witness is None:
                # 观察期结束仍未捕获任何支撑失败边界（boundary/drop 证据均缺失）：
                # 数据陈旧/观测不充分不是"支撑失败证据"，直接解除临时 hold 继续，
                # 不能 SAFETY_BLOCKED 使任务失败。明确 DROP 由 _fresh_drop_blocks
                # 独立锁存停车（true_drop_stop），不经过本分支。
                self._leave_temporary_hold(wall_time, control_time)
                self.observation_hold_release_without_witness_count += 1
            else:
                self._enter_state(
                    SUPPORT_PATH_UNAVAILABLE,
                    "ground_support_path_unavailable",
                    wall_time,
                    control_time=control_time,
                )

    def check_wall_timeout(self, wall_time: float) -> None:
        """仅在仿真时钟或 odom 停止推进时作为生命周期 watchdog。"""
        if (
            not self.enabled
            or self.state == CLEAR
            or self._last_progress_wall_time is None
        ):
            return
        elapsed = max(0.0, float(wall_time) - self._last_progress_wall_time)
        if elapsed < self.config.wall_stall_timeout:
            return
        self._enter_state(
            SAFETY_STOP_TIMEOUT,
            "ground_support_clock_or_odom_stalled",
            wall_time,
            control_time=self._last_control_time,
        )

    def filter_command(
        self,
        command: tuple[float, float, float],
        *,
        pose_yaw: float,
        wall_time: float,
        control_time: float | None = None,
    ) -> tuple[float, float, float]:
        """在唯一速度出口决定原样放行或输出零速。

        顺序为：检查时钟/锁存状态、验证请求与实测速度包络、处理已有临时停车、
        检查明确 DROP，最后才判断是否接近未知支撑边界。任何锁存状态都优先于
        新命令，防止上层换目标后意外解除安全停车。
        """
        try:
            requested = tuple(float(value) for value in command)
        except (TypeError, ValueError):
            return (0.0, 0.0, 0.0)
        if len(requested) != 3 or not all(math.isfinite(value) for value in requested):
            return (0.0, 0.0, 0.0)
        if not self.enabled:
            return requested
        try:
            resolved_wall_time = float(wall_time)
            control_now = float(
                resolved_wall_time if control_time is None else control_time
            )
            resolved_pose_yaw = float(pose_yaw)
        except (TypeError, ValueError):
            return (0.0, 0.0, 0.0)
        if not all(
            math.isfinite(value)
            for value in (resolved_wall_time, control_now, resolved_pose_yaw)
        ):
            return (0.0, 0.0, 0.0)
        wall_time = resolved_wall_time
        pose_yaw = resolved_pose_yaw
        self.check_control_timeout(control_now, wall_time=wall_time)
        if self.state in _LATCHED_STATES:
            return (0.0, 0.0, 0.0)

        requested_speed = math.hypot(requested[0], requested[1])
        self._last_requested_speed = requested_speed
        requested_stop_distance = self.envelope.distance_for(requested_speed)
        localization_speed_anomaly = (
            self._actual_linear_speed
            > self.config.localization_speed_anomaly_threshold
        )
        if requested_stop_distance is None:
            self._begin_speed_out_of_envelope_stop(
                wall_time,
                control_now,
                source="requested_command",
                confirmed=True,
            )
            return (0.0, 0.0, 0.0)
        if localization_speed_anomaly:
            self._begin_speed_out_of_envelope_stop(
                wall_time,
                control_now,
                source="localization_odometry",
                confirmed=False,
            )
            return (0.0, 0.0, 0.0)

        # 表项表达“该请求档位下包含底盘响应的制动距离”，不能用含配准校正的
        # 定位差分速度重新选档。定位异常已在上方独立 fail-closed。
        stop_distance = requested_stop_distance
        self.last_stop_distance = stop_distance

        if self.state in _TEMPORARY_HOLDS:
            hold_command = self._temporary_hold_command or requested
            hold_pose_yaw = (
                self._temporary_hold_pose_yaw
                if self._temporary_hold_pose_yaw is not None
                else pose_yaw
            )
            assessment = self._assessment
            if self._fresh_drop_blocks(
                assessment,
                command=hold_command,
                pose_yaw=hold_pose_yaw,
                wall_time=wall_time,
                control_time=control_now,
            ):
                self.true_drop_stop_count += 1
                self._begin_drop_stop(assessment, wall_time, control_now)
                return (0.0, 0.0, 0.0)
            if not self._unknown_support_hold_enabled:
                # 电梯门槛跨越时视角无法补足正支撑；继续等待只会耗尽父任务超时。
                # 先检查上面的显式 DROP，再解除仅由 UNKNOWN 引起的临时停车。
                self._leave_temporary_hold(wall_time, control_now)
                return requested
            if self._can_release(wall_time, control_now):
                self._leave_temporary_hold(wall_time, control_now)
            else:
                if (
                    self.state == OBSERVATION_HOLD
                    and assessment is not None
                    and not self._assessment_is_usable(assessment, control_now)
                ):
                    self._enter_state(
                        STALE_DATA_HOLD,
                        "ground_support_stale",
                        wall_time,
                        control_time=control_now,
                    )
                return (0.0, 0.0, 0.0)

        if requested_speed <= 1e-4:
            return requested

        assessment = self._assessment
        if assessment is None:
            if not self._unknown_support_hold_enabled:
                return requested
            self._begin_temporary_hold_context(requested, pose_yaw)
            self.stale_data_hold_count += 1
            self._enter_state(
                STALE_DATA_HOLD,
                "ground_support_missing",
                wall_time,
                control_time=control_now,
            )
            return (0.0, 0.0, 0.0)
        fresh = self._assessment_is_usable(assessment, control_now)
        if self._fresh_drop_blocks(
            assessment,
            command=requested,
            pose_yaw=pose_yaw,
            wall_time=wall_time,
            control_time=control_now,
        ):
            self.true_drop_stop_count += 1
            self._begin_drop_stop(assessment, wall_time, control_now)
            return (0.0, 0.0, 0.0)

        if not self._unknown_support_hold_enabled:
            return requested

        guard_distance = self._guard_distance(
            requested_speed,
            stop_distance,
            data_age=self._latency_data_age(assessment, control_now),
        ) + self.config.boundary_hold_margin
        supported_distance = assessment.supported_distance
        near_supported_boundary = (
            assessment.current_footprint_supported
            and assessment.support_boundary_detected
            and supported_distance is not None
            and supported_distance <= guard_distance
        )
        if not near_supported_boundary:
            return requested
        self._begin_temporary_hold_context(requested, pose_yaw)
        self._capture_failure_witness(assessment)
        if not fresh:
            self.stale_data_hold_count += 1
            self._enter_state(
                STALE_DATA_HOLD,
                "ground_support_stale",
                wall_time,
                control_time=control_now,
            )
            return (0.0, 0.0, 0.0)
        # 新鲜且可见仍不等于有支撑；只要前方确认支撑前缀不足，就临时补观测。
        self.observation_hold_count += 1
        self._enter_state(
            OBSERVATION_HOLD,
            "ground_support_observation_hold",
            wall_time,
            control_time=control_now,
        )
        return (0.0, 0.0, 0.0)

    def evaluate_latest_drop(
        self,
        command: tuple[float, float, float],
        *,
        pose_yaw: float,
        wall_time: float,
        control_time: float | None = None,
    ) -> bool:
        """新 cloud 到达后立即锁存 DROP，实际零速仍由统一命令出口发布。"""
        if not self.enabled or self.state in _LATCHED_STATES:
            return False
        control_now = float(wall_time if control_time is None else control_time)
        self._note_control_progress(control_now, wall_time)
        intended_command = (
            self._temporary_hold_command
            if self.state in _TEMPORARY_HOLDS and self._temporary_hold_command is not None
            else tuple(float(value) for value in command)
        )
        intended_yaw = (
            self._temporary_hold_pose_yaw
            if self.state in _TEMPORARY_HOLDS
            and self._temporary_hold_pose_yaw is not None
            else float(pose_yaw)
        )
        if not self._fresh_drop_blocks(
            self._assessment,
            command=intended_command,
            pose_yaw=intended_yaw,
            wall_time=wall_time,
            control_time=control_now,
        ):
            return False
        self.true_drop_stop_count += 1
        self._begin_drop_stop(self._assessment, wall_time, control_now)
        return True

    def snapshot(self) -> dict[str, object]:
        assessment = asdict(self._assessment) if self._assessment is not None else None
        return {
            "enabled": self.enabled,
            "state": self.state,
            "reason_code": self.reason_code,
            "terminal": self.terminal,
            "unknown_support_hold_enabled": self._unknown_support_hold_enabled,
            "validated_max_speed": self.envelope.validated_max_speed,
            "stop_distance_table": [list(item) for item in self.envelope.table],
            "stop_envelope_index_source": "requested_command",
            "localization_speed_role": "stop_confirmation_and_anomaly_observation",
            "localization_speed_anomaly_threshold": (
                self.config.localization_speed_anomaly_threshold
            ),
            "localization_speed_anomaly_timeout": (
                self.config.localization_speed_anomaly_timeout
            ),
            "last_stop_distance": self.last_stop_distance,
            "actual_linear_speed": self._actual_linear_speed,
            "actual_angular_speed": self._actual_angular_speed,
            "true_drop_stop_count": self.true_drop_stop_count,
            "observation_hold_count": self.observation_hold_count,
            "stale_data_hold_count": self.stale_data_hold_count,
            "speed_out_of_envelope_count": self.speed_out_of_envelope_count,
            "speed_out_of_envelope_sample_count": (
                self._speed_out_of_envelope_sample_count
            ),
            "speed_out_of_envelope_confirmed": (
                self._speed_out_of_envelope_confirmed
            ),
            "speed_out_of_envelope_trigger_source": (
                self._speed_out_of_envelope_trigger_source
            ),
            "speed_out_of_envelope_trigger_requested_speed": (
                self._speed_out_of_envelope_trigger_requested_speed
            ),
            "speed_out_of_envelope_trigger_actual_speed": (
                self._speed_out_of_envelope_trigger_actual_speed
            ),
            "automatic_resume_count": self.automatic_resume_count,
            "total_hold_time": self.total_hold_time,
            "state_enter_control_time": self._state_enter_control_time,
            "latched_drop_point_world": (
                list(self._latched_drop_point_world)
                if self._latched_drop_point_world is not None
                else None
            ),
            "boundary_candidate_centroid": (
                list(self._boundary_candidate_centroid)
                if self._boundary_candidate_centroid is not None
                else None
            ),
            "boundary_confirmation_count": self._boundary_confirmation_count,
            "hold_context_id": self._temporary_hold_context_id,
            "hold_context_sequence": self._hold_context_sequence,
            "hold_command": (
                list(self._temporary_hold_command)
                if self._temporary_hold_command is not None
                else None
            ),
            "latched_boundary_cells_world": [
                list(point) for point in self._latched_boundary_cells_world
            ],
            "assessment": assessment,
        }

    @property
    def latched_drop_point_world(self) -> tuple[float, float] | None:
        return self._latched_drop_point_world

    @property
    def latched_boundary_cells_world(self) -> tuple[tuple[float, float], ...]:
        return self._latched_boundary_cells_world

    @property
    def failure_witness(self) -> SupportFailureWitness | None:
        return self._pending_failure_witness

    @property
    def latched_drop_clusters(self) -> tuple[DropClusterEvidence, ...]:
        return self._latched_drop_clusters

    def reset_negative_obstacle_after_replan(
        self,
        ack: NegativeClusterAck | None = None,
    ) -> bool:
        """停稳且外部已写入局部禁行区后，解除 DROP 锁存。"""
        expected = self._latched_drop_clusters
        if (
            self.state != SAFETY_BLOCKED
            or self.reason_code != "negative_obstacle"
            or not expected
            or ack is None
            or ack.floor != expected[0].floor
            or ack.map_epoch != expected[0].map_epoch
            or ack.localization_epoch != expected[0].localization_epoch
            or ack.cluster_ids != tuple(item.cluster_id for item in expected)
            or ack.map_revision <= 0
        ):
            return False
        self.state = CLEAR
        self.reason_code = None
        self._assessment = None
        self._fresh_supported_sequence_count = 0
        self._state_enter_wall_time = None
        self._state_enter_control_time = None
        self._stopped_sample_count = 0
        self._temporary_hold_command = None
        self._temporary_hold_pose_yaw = None
        self._temporary_hold_context_id = None
        self._latched_drop_point_world = None
        self._latched_drop_clusters = ()
        self._reset_boundary_candidate()
        return True

    def reset_path_unavailable(self, ack: SupportConstraintAck | None = None) -> bool:
        """只解除 UNKNOWN 超时导致的当前路径失败，永久安全状态不可复位。"""
        if (
            self.state != SUPPORT_PATH_UNAVAILABLE
            or self.reason_code != "ground_support_path_unavailable"
            or self._pending_failure_witness is None
            or ack is None
            or ack.witness_sequence != self._pending_failure_witness.sequence
            or ack.floor != self._pending_failure_witness.floor
            or ack.map_epoch != self._pending_failure_witness.map_epoch
            or ack.localization_epoch
            != self._pending_failure_witness.localization_epoch
            or ack.constraint_revision <= self._last_constraint_ack_revision
        ):
            return False
        self.state = CLEAR
        self.reason_code = None
        self._state_enter_wall_time = None
        self._state_enter_control_time = None
        self._stopped_sample_count = 0
        self._temporary_hold_command = None
        self._temporary_hold_pose_yaw = None
        self._temporary_hold_context_id = None
        self._last_constraint_ack_revision = int(ack.constraint_revision)
        self._pending_failure_witness = None
        self._reset_boundary_candidate()
        return True

    def invalidate_assessment(self) -> None:
        """换层歧义开始时丢弃旧楼层评估，不能跨楼层复用支撑证据。"""
        if not self.enabled:
            return
        self._assessment = None
        self._fresh_supported_sequence_count = 0
        if self.state in _TEMPORARY_HOLDS and self._temporary_hold_command is not None:
            self._hold_context_sequence += 1
            self._temporary_hold_context_id = self._hold_context_sequence
        self._reset_boundary_candidate()

    def invalidate_evidence_scope(self) -> None:
        """原子 scope 转换时撤销所有依赖旧证据的锁存上下文。"""
        if not self.enabled:
            return
        self.invalidate_assessment()
        self._pending_failure_witness = None
        self._temporary_hold_command = None
        self._temporary_hold_pose_yaw = None
        self._temporary_hold_context_id = None
        self._stopped_sample_count = 0
        if self.state != SAFETY_BLOCKED or self.reason_code != "negative_obstacle":
            # scope 变化后没有可复用的 HOLD 命令，也没有合法的进入时间。
            # 回到 CLEAR，让下一条非零命令经缺评估分支重新建立完整的
            # fail-closed HOLD 上下文；直接伪造 STALE_DATA_HOLD 会永久无法释放。
            self.state = CLEAR
            self.reason_code = None
            self._state_enter_wall_time = None
            self._state_enter_control_time = None

    def _guard_distance(
        self,
        speed: float,
        stop_distance: float,
        *,
        data_age: float = 0.0,
    ) -> float:
        return (
            stop_distance
            + abs(float(speed))
            * (
                self.config.decision_period
                + self.config.rl_receive_delay
                + max(0.0, float(data_age))
            )
            + self.config.required_margin
        )

    def _can_release(
        self,
        wall_time: float,
        control_time: float,
    ) -> bool:
        assessment = self._assessment
        command = self._temporary_hold_command
        if (
            assessment is None
            or command is None
            or self._assessment_sequence == self._last_release_sequence
            or not self._assessment_matches_temporary_hold(assessment)
        ):
            return False
        speed = math.hypot(command[0], command[1])
        stop_distance = self.envelope.distance_for(speed)
        if stop_distance is None or assessment.supported_distance is None:
            return False
        required = self._guard_distance(
            speed,
            stop_distance,
            data_age=self._latency_data_age(assessment, control_time),
        ) + self.config.boundary_hold_margin + self.config.release_hysteresis
        boundary_is_clear = not assessment.support_boundary_detected
        distance_is_clear = (
            assessment.supported_distance is not None
            and assessment.supported_distance >= required
        )
        return (
            self._assessment_is_usable(assessment, control_time)
            and assessment.observation_adequate
            and assessment.current_footprint_supported
            and not assessment.drop_confirmed
            and (boundary_is_clear or distance_is_clear)
            and self._fresh_supported_sequence_count
            >= max(1, int(self.config.release_supported_frames))
        )

    def _leave_temporary_hold(self, wall_time: float, control_time: float) -> None:
        if self._state_enter_control_time is not None:
            self.total_hold_time += max(
                0.0,
                control_time - self._state_enter_control_time,
            )
        elif self._state_enter_wall_time is not None:
            self.total_hold_time += max(0.0, wall_time - self._state_enter_wall_time)
        self.automatic_resume_count += 1
        self.state = CLEAR
        self.reason_code = None
        self._state_enter_wall_time = None
        self._state_enter_control_time = None
        self._last_release_sequence = self._assessment_sequence
        self._temporary_hold_command = None
        self._temporary_hold_pose_yaw = None
        self._temporary_hold_context_id = None
        self._reset_boundary_candidate()

    def _begin_temporary_hold_context(
        self,
        command: tuple[float, float, float],
        pose_yaw: float,
    ) -> None:
        """冻结触发 HOLD 的非零意图；安全零速不得改写该上下文。"""
        if self.state not in _TEMPORARY_HOLDS or self._temporary_hold_context_id is None:
            self._hold_context_sequence += 1
            self._temporary_hold_context_id = self._hold_context_sequence
            self._fresh_supported_sequence_count = 0
            self._pending_failure_witness = None
        self._temporary_hold_command = tuple(float(value) for value in command)
        self._temporary_hold_pose_yaw = float(pose_yaw)

    def _capture_failure_witness(
        self,
        assessment: GroundSupportAssessment,
    ) -> None:
        """同一 HOLD 只冻结首个完整断面；后续空评估不得清除它。"""
        if self._pending_failure_witness is not None:
            return
        cells = tuple(sorted(set(assessment.support_boundary_cells or ())))
        if (
            not cells
            or not self._assessment_matches_temporary_hold(assessment)
            or int(assessment.cloud_generation) <= 0
            or int(assessment.map_epoch) <= 0
            or int(assessment.localization_epoch) <= 0
            or assessment.grid_origin is None
            or assessment.grid_resolution is None
            or not math.isfinite(float(assessment.grid_resolution))
            or float(assessment.grid_resolution) <= 0.0
        ):
            return
        self._failure_witness_sequence += 1
        self._pending_failure_witness = SupportFailureWitness(
            sequence=self._failure_witness_sequence,
            floor=int(assessment.floor),
            map_epoch=int(assessment.map_epoch),
            localization_epoch=int(assessment.localization_epoch),
            stamp=float(assessment.stamp),
            generation=int(assessment.cloud_generation),
            source_cells=cells,
            hold_context_id=int(assessment.hold_context_id),
            trigger_command=tuple(float(value) for value in assessment.support_command),
            grid_origin=(
                float(assessment.grid_origin[0]),
                float(assessment.grid_origin[1]),
            ),
            grid_resolution=float(assessment.grid_resolution),
        )

    def _assessment_matches_temporary_hold(
        self,
        assessment: GroundSupportAssessment,
    ) -> bool:
        context_id = self._temporary_hold_context_id
        command = self._temporary_hold_command
        assessed_command = assessment.support_command
        if (
            context_id is None
            or command is None
            or int(assessment.hold_context_id) != int(context_id)
            or assessed_command is None
            or assessment.support_lookahead_distance is None
        ):
            return False
        return len(assessed_command) == 3 and all(
            abs(float(actual) - float(expected)) <= 1e-9
            for actual, expected in zip(assessed_command, command)
        )

    def _update_boundary_candidate(self, assessment: GroundSupportAssessment) -> None:
        cells = tuple(assessment.support_boundary_cells_world or ())
        qualifies = (
            assessment.support_boundary_detected
            and assessment.observation_adequate
            and assessment.current_footprint_supported
            and assessment.data_age <= self.config.drop_evidence_max_age
            and bool(cells)
        )
        if not qualifies:
            self._reset_boundary_candidate()
            return
        centroid = (
            sum(float(point[0]) for point in cells) / len(cells),
            sum(float(point[1]) for point in cells) / len(cells),
        )
        previous = self._boundary_candidate_centroid
        if previous is None or math.hypot(
            centroid[0] - previous[0], centroid[1] - previous[1]
        ) > self.config.boundary_consistency_radius:
            self._boundary_candidate_centroid = centroid
            self._boundary_confirmation_count = 1
            self._latched_boundary_cells_world = ()
            return
        self._boundary_candidate_centroid = centroid
        self._boundary_confirmation_count += 1
        if self._boundary_confirmation_count >= max(
            1, int(self.config.boundary_persistence_confirmations)
        ):
            self._latched_boundary_cells_world = tuple(
                (float(point[0]), float(point[1])) for point in cells
            )

    def _reset_boundary_candidate(self) -> None:
        self._boundary_candidate_centroid = None
        self._boundary_confirmation_count = 0
        self._latched_boundary_cells_world = ()

    def _fresh_drop_blocks(
        self,
        assessment: GroundSupportAssessment | None,
        *,
        command: tuple[float, float, float],
        pose_yaw: float,
        wall_time: float,
        control_time: float,
    ) -> bool:
        if assessment is None:
            return False
        return (
            self._assessment_is_usable(assessment, control_time)
            and assessment.observation_adequate
            and assessment.drop_confirmed
            and assessment.drop_intersects_sweep
            and self._moves_toward_drop(
                command,
                pose_yaw,
                assessment.drop_direction_world,
            )
        )

    def _begin_drop_stop(
        self,
        assessment: GroundSupportAssessment | None,
        wall_time: float,
        control_time: float,
    ) -> None:
        point = assessment.drop_point_world if assessment is not None else None
        if point is not None and all(math.isfinite(float(value)) for value in point):
            self._latched_drop_point_world = (float(point[0]), float(point[1]))
        clusters = tuple(assessment.drop_clusters) if assessment is not None else ()
        if not clusters or any(
            cluster.floor != int(assessment.floor)
            or cluster.map_epoch != int(assessment.map_epoch)
            or cluster.localization_epoch != int(assessment.localization_epoch)
            or abs(cluster.stamp - float(assessment.stamp)) > 1e-9
            or cluster.generation != int(assessment.cloud_generation)
            or not cluster.source_cells
            for cluster in clusters
        ):
            self._enter_state(
                SAFETY_BLOCKED,
                "drop_source_cluster_missing",
                wall_time,
                control_time=control_time,
            )
            return
        self._latched_drop_clusters = clusters
        self._enter_state(
            DROP_STOPPING,
            "negative_obstacle",
            wall_time,
            control_time=control_time,
        )

    def _begin_speed_out_of_envelope_stop(
        self,
        wall_time: float,
        control_time: float,
        *,
        source: str,
        confirmed: bool,
    ) -> None:
        self.speed_out_of_envelope_count += 1
        self._speed_out_of_envelope_confirmed = bool(confirmed)
        self._speed_out_of_envelope_trigger_source = str(source)
        self._speed_out_of_envelope_trigger_requested_speed = self._last_requested_speed
        self._speed_out_of_envelope_trigger_actual_speed = self._actual_linear_speed
        self.last_stop_distance = None
        self._enter_state(
            SPEED_OUT_OF_ENVELOPE_STOPPING,
            "support_speed_out_of_envelope",
            wall_time,
            control_time=control_time,
        )

    def _leave_speed_out_of_envelope_hold(
        self,
        wall_time: float,
        *,
        control_time: float,
    ) -> None:
        if self._state_enter_control_time is not None:
            self.total_hold_time += max(
                0.0,
                float(control_time) - self._state_enter_control_time,
            )
        elif self._state_enter_wall_time is not None:
            self.total_hold_time += max(0.0, wall_time - self._state_enter_wall_time)
        self.automatic_resume_count += 1
        self.state = CLEAR
        self.reason_code = None
        self._state_enter_wall_time = None
        self._state_enter_control_time = None
        self._stopped_sample_count = 0
        self._speed_out_of_envelope_confirmed = False

    def _enter_state(
        self,
        state: str,
        reason_code: str,
        wall_time: float,
        *,
        control_time: float | None = None,
    ) -> None:
        if self.state == state:
            return
        switching_temporary_reason = (
            self.state in _TEMPORARY_HOLDS and state in _TEMPORARY_HOLDS
        )
        if (
            self.state in _TEMPORARY_HOLDS
            and not switching_temporary_reason
            and (
                self._state_enter_control_time is not None
                or self._state_enter_wall_time is not None
            )
        ):
            if self._state_enter_control_time is not None and control_time is not None:
                self.total_hold_time += max(
                    0.0,
                    float(control_time) - self._state_enter_control_time,
                )
            elif self._state_enter_wall_time is not None:
                self.total_hold_time += max(0.0, wall_time - self._state_enter_wall_time)
        self.state = state
        self.reason_code = reason_code
        if not switching_temporary_reason:
            self._state_enter_wall_time = float(wall_time)
            self._state_enter_control_time = (
                float(control_time)
                if control_time is not None and math.isfinite(float(control_time))
                else self._last_control_time
            )
            self._last_progress_wall_time = float(wall_time)
        self._stopped_sample_count = 0

    def _note_control_progress(self, control_time: float, wall_time: float) -> None:
        value = float(control_time)
        if not math.isfinite(value):
            return
        if self._last_control_time is None or value > self._last_control_time + 1e-9:
            self._last_control_time = value
            self._last_progress_wall_time = float(wall_time)

    @staticmethod
    def _assessment_has_valid_numbers(assessment: GroundSupportAssessment) -> bool:
        try:
            required = (
                assessment.stamp,
                assessment.received_wall_time,
                assessment.data_age,
            )
            if not all(math.isfinite(float(value)) for value in required):
                return False
            if float(assessment.data_age) < 0.0:
                return False
            if assessment.received_control_time is not None and not math.isfinite(
                float(assessment.received_control_time)
            ):
                return False
            for value in (assessment.supported_distance, assessment.drop_distance):
                if value is not None and (
                    not math.isfinite(float(value)) or float(value) < 0.0
                ):
                    return False
            if (
                isinstance(assessment.hold_context_id, bool)
                or int(assessment.hold_context_id) < 0
            ):
                return False
            if assessment.support_command is not None and (
                len(assessment.support_command) != 3
                or not all(
                    math.isfinite(float(value))
                    for value in assessment.support_command
                )
            ):
                return False
            if assessment.support_lookahead_distance is not None and (
                not math.isfinite(float(assessment.support_lookahead_distance))
                or float(assessment.support_lookahead_distance) < 0.0
            ):
                return False
            if assessment.boundary_observable is not None and not isinstance(
                assessment.boundary_observable,
                bool,
            ):
                return False
            for reason, count in assessment.boundary_reason_counts:
                if (
                    not isinstance(reason, str)
                    or not reason
                    or isinstance(count, bool)
                    or int(count) < 0
                ):
                    return False
            for point in (assessment.drop_direction_world, assessment.drop_point_world):
                if point is not None and (
                    len(point) != 2
                    or not all(math.isfinite(float(value)) for value in point)
                ):
                    return False
            for point in assessment.support_boundary_cells_world:
                if (
                    len(point) != 2
                    or not all(math.isfinite(float(value)) for value in point)
                ):
                    return False
            for cluster in assessment.drop_clusters:
                if (
                    not isinstance(cluster, DropClusterEvidence)
                    or not cluster.cluster_id
                    or cluster.frame_id != "odom"
                    or cluster.floor != int(assessment.floor)
                    or cluster.map_epoch != int(assessment.map_epoch)
                    or cluster.localization_epoch
                    != int(assessment.localization_epoch)
                    or cluster.generation != int(assessment.cloud_generation)
                    or not math.isfinite(float(cluster.stamp))
                    or abs(cluster.stamp - float(assessment.stamp)) > 1e-9
                    or not cluster.source_cells
                    or assessment.grid_origin is None
                    or cluster.grid_origin
                    != tuple(float(value) for value in assessment.grid_origin)
                    or assessment.grid_resolution is None
                    or abs(
                        float(cluster.grid_resolution)
                        - float(assessment.grid_resolution)
                    ) > 1e-9
                    or not math.isfinite(float(cluster.grid_resolution))
                    or float(cluster.grid_resolution) <= 0.0
                    or any(
                        not isinstance(cell, (tuple, list))
                        or len(cell) != 2
                        or any(
                            isinstance(value, bool)
                            or not isinstance(value, int)
                            for value in cell
                        )
                        for cell in cluster.source_cells
                    )
                ):
                    return False
            if assessment.grid_origin is not None and (
                len(assessment.grid_origin) != 2
                or not all(
                    math.isfinite(float(value))
                    for value in assessment.grid_origin
                )
            ):
                return False
            if assessment.grid_resolution is not None and (
                not math.isfinite(float(assessment.grid_resolution))
                or float(assessment.grid_resolution) <= 0.0
            ):
                return False
            return True
        except (AttributeError, TypeError, ValueError):
            return False

    @staticmethod
    def _assessment_elapsed(
        assessment: GroundSupportAssessment,
        control_time: float,
    ) -> float:
        received = (
            assessment.received_control_time
            if assessment.received_control_time is not None
            else assessment.stamp
        )
        return max(0.0, float(control_time) - float(received))

    def _assessment_is_usable(
        self,
        assessment: GroundSupportAssessment,
        control_time: float,
    ) -> bool:
        return (
            max(0.0, float(assessment.data_age))
            <= self.config.drop_evidence_max_age
            and self._assessment_elapsed(assessment, control_time)
            <= self.config.decision_period
        )

    def _latency_data_age(
        self,
        assessment: GroundSupportAssessment,
        control_time: float,
    ) -> float:
        elapsed = self._assessment_elapsed(assessment, control_time)
        # 决策窗内的等待已由 decision_period 覆盖；超窗后才追加实际陈旧时间。
        return max(0.0, float(assessment.data_age)) + max(
            0.0,
            elapsed - self.config.decision_period,
        )

    @staticmethod
    def _moves_toward_drop(
        command: tuple[float, float, float],
        pose_yaw: float,
        direction_world: tuple[float, float] | None,
    ) -> bool:
        if direction_world is None:
            return False
        cos_yaw = math.cos(float(pose_yaw))
        sin_yaw = math.sin(float(pose_yaw))
        world_x = command[0] * cos_yaw - command[1] * sin_yaw
        world_y = command[0] * sin_yaw + command[1] * cos_yaw
        norm = math.hypot(direction_world[0], direction_world[1])
        if norm <= 1e-9:
            return False
        return (world_x * direction_world[0] + world_y * direction_world[1]) / norm > 1e-4

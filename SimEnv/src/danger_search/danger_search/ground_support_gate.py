"""局部地面支撑安全门的状态机与停车包络。

该模块不修改占据地图，也不负责重规划。它只把已验证的局部支撑评估转换为
最终速度钳制状态；关闭时必须完全透明。
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
    """按实测速度上界查表，不插值且绝不向有效域外外推。"""

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
    enabled: bool = False
    validated_max_speed: float | None = None
    stop_distance_table: tuple[tuple[float, float], ...] = (
        (0.60, 0.09337727739487302),
        (0.651, 0.10195826367975175),
    )
    speed_out_of_envelope_confirmations: int = 2
    drop_evidence_max_age: float = 0.05
    decision_period: float = 0.10
    rl_receive_delay: float = 0.009
    required_margin: float = 0.02
    release_hysteresis: float = 0.10
    release_supported_frames: int = 2
    observation_hold_timeout: float = 2.0
    safety_stop_timeout: float = 3.0
    stop_linear_threshold: float = 0.02
    stop_angular_threshold: float = 0.05
    stop_confirmation_samples: int = 3


@dataclass(frozen=True)
class GroundSupportAssessment:
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


class GroundSupportSafetyGate:
    """只对最终命令做局部安全判定的锁存状态机。"""

    def __init__(self, config: GroundSupportGateConfig | None = None) -> None:
        self.config = config or GroundSupportGateConfig()
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
        self._unknown_support_hold_enabled = True
        self._latched_drop_point_world: tuple[float, float] | None = None
        self.last_stop_distance: float | None = 0.0
        self.true_drop_stop_count = 0
        self.observation_hold_count = 0
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
        if not self.enabled:
            return
        if self._assessment is not None and assessment.stamp <= self._assessment.stamp:
            return
        self._assessment = assessment
        self._assessment_sequence += 1
        control_time = (
            assessment.received_control_time
            if assessment.received_control_time is not None
            else assessment.stamp
        )
        self._note_control_progress(control_time, assessment.received_wall_time)
        if (
            assessment.observation_adequate
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

    def observe_odometry(
        self,
        stamp: float | None,
        *,
        linear_speed: float,
        angular_speed: float,
        wall_time: float,
    ) -> None:
        if not self.enabled:
            return
        self._actual_linear_speed = abs(float(linear_speed))
        self._actual_angular_speed = abs(float(angular_speed))
        is_fresh = stamp is not None and (
            self._last_odom_stamp is None or float(stamp) > self._last_odom_stamp
        )
        if is_fresh:
            self._last_odom_stamp = float(stamp)
            self._note_control_progress(float(stamp), wall_time)
            actual_out_of_envelope = (
                self.envelope.distance_for(self._actual_linear_speed) is None
            )
            if actual_out_of_envelope:
                self._speed_out_of_envelope_sample_count += 1
                if (
                    self.state == SPEED_OUT_OF_ENVELOPE_STOPPING
                    and self._speed_out_of_envelope_sample_count
                    >= max(
                        1,
                        int(self.config.speed_out_of_envelope_confirmations),
                    )
                ):
                    self._speed_out_of_envelope_confirmed = True
            else:
                self._speed_out_of_envelope_sample_count = 0
                if (
                    self.state == SPEED_OUT_OF_ENVELOPE_STOPPING
                    and not self._speed_out_of_envelope_confirmed
                ):
                    # 位姿差分速度可能因单帧定位校正产生尖峰。首帧仍立即零速，
                    # 但只有连续新 odom 超界才升级为不可恢复的安全失败。
                    self._leave_speed_out_of_envelope_hold(
                        wall_time,
                        control_time=float(stamp),
                    )
                    return
        if self.state not in _STOPPING_STATES:
            self._stopped_sample_count = 0
            return
        if not is_fresh:
            return
        if (
            self.state == SPEED_OUT_OF_ENVELOPE_STOPPING
            and not self._speed_out_of_envelope_confirmed
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
        if elapsed < self.config.safety_stop_timeout:
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
        requested = tuple(float(value) for value in command)
        if not self.enabled:
            return requested
        control_now = float(wall_time if control_time is None else control_time)
        self.check_control_timeout(control_now, wall_time=wall_time)
        if self.state in _LATCHED_STATES:
            return (0.0, 0.0, 0.0)

        requested_speed = math.hypot(requested[0], requested[1])
        self._last_requested_speed = requested_speed
        requested_stop_distance = self.envelope.distance_for(requested_speed)
        actual_stop_distance = self.envelope.distance_for(self._actual_linear_speed)
        if requested_stop_distance is None:
            self._begin_speed_out_of_envelope_stop(
                wall_time,
                control_now,
                source="requested_command",
                confirmed=True,
            )
            return (0.0, 0.0, 0.0)
        if actual_stop_distance is None:
            self._begin_speed_out_of_envelope_stop(
                wall_time,
                control_now,
                source="localization_odometry",
                confirmed=(
                    self._speed_out_of_envelope_sample_count
                    >= max(
                        1,
                        int(self.config.speed_out_of_envelope_confirmations),
                    )
                ),
            )
            return (0.0, 0.0, 0.0)

        effective_speed = max(requested_speed, self._actual_linear_speed)
        stop_distance = self.envelope.distance_for(effective_speed)
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
            return requested
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
            effective_speed,
            stop_distance,
            data_age=self._latency_data_age(assessment, control_now),
        )
        supported_distance = assessment.supported_distance
        near_supported_boundary = (
            assessment.current_footprint_supported
            and assessment.support_boundary_detected
            and supported_distance is not None
            and supported_distance <= guard_distance
        )
        if not near_supported_boundary:
            return requested
        self._temporary_hold_command = requested
        self._temporary_hold_pose_yaw = float(pose_yaw)
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
            "assessment": assessment,
        }

    @property
    def latched_drop_point_world(self) -> tuple[float, float] | None:
        return self._latched_drop_point_world

    def reset_negative_obstacle_after_replan(self) -> bool:
        """停稳且外部已写入局部禁行区后，解除 DROP 锁存。"""
        if self.state != SAFETY_BLOCKED or self.reason_code != "negative_obstacle":
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
        self._latched_drop_point_world = None
        return True

    def reset_path_unavailable(self) -> bool:
        """只解除 UNKNOWN 超时导致的当前路径失败，永久安全状态不可复位。"""
        if (
            self.state != SUPPORT_PATH_UNAVAILABLE
            or self.reason_code != "ground_support_path_unavailable"
        ):
            return False
        self.state = CLEAR
        self.reason_code = None
        self._state_enter_wall_time = None
        self._state_enter_control_time = None
        self._stopped_sample_count = 0
        self._temporary_hold_command = None
        self._temporary_hold_pose_yaw = None
        return True

    def invalidate_assessment(self) -> None:
        """换层歧义开始时丢弃旧楼层评估，不能跨楼层复用支撑证据。"""
        if not self.enabled:
            return
        self._assessment = None
        self._fresh_supported_sequence_count = 0

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
        ):
            return False
        speed = max(math.hypot(command[0], command[1]), self._actual_linear_speed)
        stop_distance = self.envelope.distance_for(speed)
        if stop_distance is None or assessment.supported_distance is None:
            return False
        required = self._guard_distance(
            speed,
            stop_distance,
            data_age=self._latency_data_age(assessment, control_time),
        ) + self.config.release_hysteresis
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

"""GroundSupport 组件所有权与反馈队列边界。

安全判定仍完全属于 ``GroundSupportSafetyGate``，支撑证据仍完全属于
``ShadowSupportGrid``。本协调器只负责构造、生命周期和主线程反馈排空，避免
MissionManager 同时拥有三套内部状态。
"""
from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
import math
import time

import numpy as np

from danger_search.ground_support_gate import (
    GroundSupportAssessment,
    GroundSupportGateConfig,
    GroundSupportSafetyGate,
)
from danger_search.ground_support_runtime import GroundSupportFeedbackRuntime
from danger_search.support_shadow import ShadowSupportConfig, ShadowSupportGrid
from danger_search.time_utils import get_time


@dataclass(frozen=True)
class GroundSupportComponents:
    gate: GroundSupportSafetyGate | None
    grid: ShadowSupportGrid | None


@dataclass(frozen=True)
class NegativeObstacleReplan:
    floor: int
    point_world: tuple[float, float]
    black_radius: float
    attempt: int
    maximum_attempts: int


class GroundSupportCoordinator:
    """拥有 Gate、ShadowSupport 与反馈运行状态，不拥有任务策略。"""

    def __init__(
        self,
        *,
        params: dict,
        map_params: dict,
        cloud_max_points: int,
        path_goal_tolerance: float,
    ) -> None:
        self.params = params
        self.feedback = GroundSupportFeedbackRuntime()
        self.map_epoch = 0
        self.localization_epoch = 0
        self.last_cloud_generation = 0
        self.last_cloud_stamp: float | None = None
        if not params["production_gate_enabled"]:
            self.components = GroundSupportComponents(None, None)
            return
        gate_config = GroundSupportGateConfig(
            enabled=True,
            validated_max_speed=params["validated_max_speed"],
            stop_distance_table=params["stop_distance_table"],
            drop_evidence_max_age=params["drop_evidence_max_age"],
            decision_period=params["decision_period"],
            rl_receive_delay=params["rl_receive_delay"],
            required_margin=params["required_margin"],
            boundary_hold_margin=params["boundary_hold_margin"],
            boundary_persistence_confirmations=params["boundary_persistence_confirmations"],
            boundary_consistency_radius=params["boundary_consistency_radius"],
            release_hysteresis=params["release_hysteresis"],
            release_supported_frames=params["release_supported_frames"],
            observation_hold_timeout=params["observation_hold_timeout"],
            safety_stop_timeout=params["safety_stop_timeout"],
            wall_stall_timeout=params["wall_stall_timeout"],
            odometry_linear_deadband=params["odometry_linear_deadband"],
            odometry_angular_deadband=params["odometry_angular_deadband"],
            stop_linear_threshold=params["stop_linear_threshold"],
            stop_angular_threshold=params["stop_angular_threshold"],
            stop_confirmation_samples=params["stop_confirmation_samples"],
            localization_speed_anomaly_threshold=params["localization_speed_anomaly_threshold"],
            localization_speed_anomaly_timeout=params["localization_speed_anomaly_timeout"],
        )
        sweep_half_width = params["sweep_half_width"]
        support_config = ShadowSupportConfig(
            resolution=map_params["resolution"],
            origin_x=map_params["origin_x"],
            origin_y=map_params["origin_y"],
            floor_height=map_params["floor_height"],
            cloud_stride=1,
            cloud_max_points=cloud_max_points,
            support_band=params["support_band"],
            minimum_drop=params["minimum_drop"],
            minimum_cell_hits=params["minimum_cell_hits"],
            required_consecutive_frames=params["required_consecutive_frames"],
            minimum_alert_cells=params["minimum_alert_cells"],
            sweep_half_width=sweep_half_width,
            foot_band_center_offset=params["foot_band_center_offset"],
            foot_band_half_width=params["foot_band_half_width"],
            minimum_foot_band_coverage=params["minimum_foot_band_coverage"],
            maximum_longitudinal_unknown_gap=params["maximum_longitudinal_unknown_gap"],
            minimum_longitudinal_support_ratio=params["minimum_longitudinal_support_ratio"],
            minimum_lookahead=params["minimum_lookahead"],
            maximum_lookahead=params["maximum_lookahead"],
            footprint_front_reach=params["footprint_front_reach"],
            path_goal_tolerance=path_goal_tolerance,
            obstacle_min_height=map_params["obstacle_min_z"],
            obstacle_max_height=map_params["obstacle_max_z"],
            minimum_frame_points=params["minimum_frame_points"],
            local_support_evidence_radius=params["local_support_evidence_radius"],
            threshold_widths=(sweep_half_width,),
        )
        self.components = GroundSupportComponents(
            GroundSupportSafetyGate(gate_config),
            ShadowSupportGrid(support_config),
        )

    @property
    def gate(self) -> GroundSupportSafetyGate | None:
        return self.components.gate

    @property
    def grid(self) -> ShadowSupportGrid | None:
        return self.components.grid

    def advance_map_epoch(self) -> int:
        self.map_epoch += 1
        return self.map_epoch

    def advance_localization_epoch(self) -> int:
        self.localization_epoch += 1
        return self.localization_epoch

    def queue_map_updates(self, result: dict, *, default_floor: int) -> None:
        self.feedback.queue_map_updates(result, default_floor=default_floor)

    def apply_map_updates(self, mapper):
        return self.feedback.apply_map_updates(mapper)

    def observe_odometry(self, controller, pose_sample) -> None:
        """把时间匹配里程计提交给唯一控制/Gate 运动观测入口。"""
        if self.gate is None:
            return
        observe = getattr(controller, "observe_motion_odometry", None)
        if not callable(observe):
            observe = self.gate.observe_odometry
        observe(
            pose_sample.stamp,
            linear_speed=pose_sample.planar_speed,
            angular_speed=pose_sample.angular_speed,
            wall_time=time.monotonic(),
        )

    def assess_frame(
        self,
        *,
        controller,
        pose_provider,
        floor_index: int,
        pose: tuple[float, float, float],
        pose_sample,
        points,
        cloud_stamp: float | None,
    ) -> None:
        """用一帧合法点云更新 ShadowSupport 与最终 Gate 评估。"""
        gate = self.gate
        grid = self.grid
        if gate is None or grid is None or cloud_stamp is None:
            return
        if pose_provider is None:
            return
        params = self.params
        sensor_to_world = pose_provider.sensor_to_world_matrix(
            pose_sample,
            base_to_sensor_xyz=params["base_to_livox_xyz"],
            base_to_sensor_rpy=params["base_to_livox_rpy"],
        )
        sensor_origin = sensor_to_world[:3, 3]
        sensor_rotation = sensor_to_world[:3, :3]
        followed_path = [
            np.asarray(point, dtype=float)
            for point in controller.followed_path_snapshot()
        ]
        active_snapshot = getattr(
            controller,
            "active_motion_command_snapshot",
            None,
        )
        committed_snapshot = getattr(
            controller,
            "committed_command_snapshot",
            None,
        )
        with _controller_command_transaction(controller):
            active_motion = (
                active_snapshot()
                if callable(active_snapshot)
                else getattr(controller, "last_command", (0.0, 0.0, 0.0))
            )
            committed_command = (
                committed_snapshot()
                if callable(committed_snapshot)
                else getattr(controller, "last_command", (0.0, 0.0, 0.0))
            )
            context_snapshot = getattr(
                gate,
                "support_sweep_context_snapshot",
                None,
            )
            if callable(context_snapshot):
                support_command, hold_context_id = context_snapshot(
                    committed_command
                )
            else:
                support_command, hold_context_id = committed_command, 0
        result = grid.process_frame(
            stamp=cloud_stamp,
            floor=int(floor_index),
            robot_xy=np.asarray(pose[:2], dtype=float),
            robot_yaw=pose[2],
            sensor_origin=sensor_origin,
            sensor_rotation=sensor_rotation,
            points=np.asarray(points, dtype=float),
            path=followed_path,
            command_linear=math.hypot(
                float(support_command[0]),
                float(support_command[1]),
            ),
            drop_command_linear=math.hypot(
                float(active_motion[0]),
                float(active_motion[1]),
            ),
            ambiguous=False,
            local_only=True,
        )
        if result.get("action") != "update":
            return
        self.queue_map_updates(result, default_floor=int(floor_index))
        payload = result.get("local_corridor_assessment")
        if not isinstance(payload, dict):
            return
        ros_now = float(get_time())
        direction = payload.get("drop_direction_world")
        drop_point_world = None
        if isinstance(direction, (list, tuple)) and len(direction) >= 2:
            drop_point_world = (
                float(pose[0]) + float(direction[0]),
                float(pose[1]) + float(direction[1]),
            )
        boundary_cells_world = tuple(
            (float(point[0]), float(point[1]))
            for point in payload.get("support_boundary_cells_world", ())
            if isinstance(point, (list, tuple))
            and len(point) >= 2
            and math.isfinite(float(point[0]))
            and math.isfinite(float(point[1]))
        )
        raw_reason_counts = payload.get("boundary_reason_counts", {})
        if not isinstance(raw_reason_counts, dict):
            raw_reason_counts = {}
        reason_counts = []
        for reason, count in raw_reason_counts.items():
            if not isinstance(reason, str) or not reason or isinstance(count, bool):
                continue
            try:
                count_value = int(count)
            except (TypeError, ValueError):
                continue
            if count_value >= 0:
                reason_counts.append((reason, count_value))
        assessment = GroundSupportAssessment(
            stamp=float(cloud_stamp),
            received_wall_time=time.monotonic(),
            data_age=_ground_support_data_age(
                cloud_stamp=float(cloud_stamp),
                odom_stamp=pose_sample.stamp,
                control_time=ros_now,
                decision_period=params["decision_period"],
            ),
            supported_distance=_optional_finite(payload.get("supported_distance")),
            support_boundary_detected=bool(
                payload.get("support_boundary_detected", False)
            ),
            current_footprint_supported=bool(
                payload.get("current_footprint_supported", False)
            ),
            observation_adequate=bool(payload.get("observation_adequate", False)),
            drop_confirmed=bool(payload.get("drop_confirmed", False)),
            drop_intersects_sweep=bool(
                payload.get("drop_intersects_sweep", False)
            ),
            drop_distance=_optional_finite(payload.get("drop_distance")),
            drop_direction_world=(
                (float(direction[0]), float(direction[1]))
                if isinstance(direction, (list, tuple)) and len(direction) >= 2
                else None
            ),
            received_control_time=ros_now,
            drop_point_world=drop_point_world,
            support_boundary_cells_world=boundary_cells_world,
            hold_context_id=int(hold_context_id),
            support_command=tuple(float(value) for value in support_command),
            support_lookahead_distance=_optional_finite(
                result.get("support_lookahead_distance")
            ),
            boundary_observable=(
                bool(payload.get("boundary_observable"))
                if payload.get("boundary_observable") is not None
                else None
            ),
            boundary_reason_counts=tuple(sorted(reason_counts)),
        )
        with _controller_command_transaction(controller):
            active_motion = (
                active_snapshot()
                if callable(active_snapshot)
                else getattr(controller, "last_command", (0.0, 0.0, 0.0))
            )
            gate.update_assessment(assessment)
            if gate.evaluate_latest_drop(
                active_motion,
                pose_yaw=pose[2],
                wall_time=time.monotonic(),
                control_time=ros_now,
            ):
                controller.stop(reason="negative_obstacle")

    def resume_negative_obstacle(
        self,
        *,
        controller,
        mapper,
        floor_index: int,
        mission_state: str,
        task_active: bool,
        ordinary_states: frozenset[str],
        map_resolution: float,
    ) -> NegativeObstacleReplan | None:
        """持久化 Gate 已确认负障碍并原子复位，任务目标由调用方处理。"""
        params = self.params
        gate = self.gate
        if gate is None or not bool(
            params.get("negative_obstacle_replan_enabled", True)
        ):
            return None
        if task_active or mission_state not in ordinary_states:
            return None
        point = getattr(gate, "latched_drop_point_world", None)
        if point is None or not all(math.isfinite(float(value)) for value in point):
            return None
        maximum = max(
            0,
            int(params.get("maximum_negative_obstacle_replans", 3)),
        )
        if not self.feedback.can_replan_negative_obstacle(maximum):
            return None
        radius = max(
            float(map_resolution),
            float(params["sweep_half_width"]),
        )
        grid = mapper.floor(int(floor_index))
        if grid.world_to_grid(float(point[0]), float(point[1])) is None:
            return None
        mapper.mark_black(
            int(floor_index),
            float(point[0]),
            float(point[1]),
            radius,
        )
        controller.clear_path(reason="negative_obstacle_replan")
        with _controller_command_transaction(controller):
            if not gate.reset_negative_obstacle_after_replan():
                return None
            self.feedback.synchronize_gate_state(gate.state)
        attempt = self.feedback.record_negative_obstacle(
            floor=int(floor_index),
            x=float(point[0]),
            y=float(point[1]),
            radius=radius,
        )
        return NegativeObstacleReplan(
            floor=int(floor_index),
            point_world=(float(point[0]), float(point[1])),
            black_radius=radius,
            attempt=attempt,
            maximum_attempts=maximum,
        )

    def reapply_black_marks(self, mapper, floor_index: int) -> None:
        """重施本协调器登记的安全 BLACK，不从正式地图反推状态。"""
        for floor, x, y, radius in self.feedback.black_marks_for_floor(
            floor_index
        ):
            mapper.mark_black(floor, x, y, radius)


def _controller_command_transaction(controller):
    transaction = getattr(controller, "command_transaction", None)
    return transaction() if callable(transaction) else nullcontext()


def _ground_support_data_age(
    *,
    cloud_stamp: float,
    odom_stamp: float | None,
    control_time: float,
    decision_period: float,
) -> float:
    alignment_age = (
        abs(float(odom_stamp) - float(cloud_stamp))
        if odom_stamp is not None
        else math.inf
    )
    excess_source_age = max(
        0.0,
        float(control_time)
        - float(cloud_stamp)
        - max(0.0, float(decision_period)),
    )
    return max(alignment_age, excess_source_age)


def _optional_finite(value) -> float | None:
    if value is None:
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None

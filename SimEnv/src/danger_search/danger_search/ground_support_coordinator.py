"""GroundSupport 组件所有权与反馈队列边界。

安全判定仍完全属于 ``GroundSupportSafetyGate``，支撑证据仍完全属于
``ShadowSupportGrid``。本协调器只负责构造、生命周期和主线程反馈排空，避免
MissionManager 同时拥有三套内部状态。
"""
from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
import math
from threading import RLock
import time

import numpy as np

from danger_search.ground_support_gate import (
    DropClusterEvidence,
    GroundSupportAssessment,
    GroundSupportGateConfig,
    GroundSupportSafetyGate,
    SupportConstraintAck,
    NegativeClusterAck,
)
from danger_search.mapper import NegativeClusterRecord
from danger_search.ground_support_runtime import GroundSupportFeedbackRuntime
from danger_search.ground_support_runtime import ConstraintInstallResult
from danger_search.support_shadow import ShadowSupportConfig, ShadowSupportGrid
from danger_search.time_utils import get_time


@dataclass(frozen=True)
class GroundSupportComponents:
    gate: GroundSupportSafetyGate | None
    grid: ShadowSupportGrid | None


@dataclass(frozen=True)
class NegativeClusterPersistResult:
    floor: int
    cluster_ids: tuple[str, ...]
    source_cell_count: int
    map_revision: int


@dataclass(frozen=True)
class GroundSupportDirective:
    """协调器交给任务编排器的冻结策略结果。"""

    action: str
    reason_code: str
    region_id: str | None = None
    failure_count: int = 0
    witness_sequence: int | None = None
    constraint_ack: SupportConstraintAck | None = None


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
        self._scope_lock = RLock()
        self.feedback = GroundSupportFeedbackRuntime()
        self._active_floor = 0
        self._map_epochs: dict[int, int] = {0: 1}
        self.localization_epoch = 1
        self.last_cloud_generation = 0
        self.last_cloud_stamp: float | None = None
        self._last_hole_scope_revision = -1
        self._negative_obstacle_commit_count = 0
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

    @property
    def map_epoch(self) -> int:
        return self._map_epochs.get(self._active_floor, 1)

    def map_epoch_for_floor(self, floor: int) -> int:
        with self._scope_lock:
            return self._map_epochs.get(int(floor), 1)

    def activate_floor(self, floor: int) -> None:
        """切换活动楼层但不伪造地图重建或定位连续性变化。"""
        with self._scope_lock:
            self._active_floor = int(floor)
            self._map_epochs.setdefault(self._active_floor, 1)
            if self.gate is not None:
                self.gate.invalidate_evidence_scope()

    def invalidate_active_assessment(self) -> None:
        """楼层歧义期仅撤销可发布评估，不改变任何 epoch。"""
        with self._scope_lock:
            if self.gate is not None:
                self.gate.invalidate_evidence_scope()

    def advance_map_epoch(self, floor: int | None = None) -> int:
        """仅供显式地图重建调用；普通换层不得调用。"""
        with self._scope_lock:
            target_floor = self._active_floor if floor is None else int(floor)
            next_epoch = self._map_epochs.get(target_floor, 1) + 1
            self._map_epochs[target_floor] = next_epoch
            self.feedback.invalidate_scope(
                floor=target_floor,
                map_epoch=next_epoch,
                localization_epoch=None,
            )
            self.feedback.discard_pending_batches()
            if self.grid is not None:
                self.grid.invalidate_evidence(target_floor)
            if target_floor == self._active_floor and self.gate is not None:
                self.gate.invalidate_evidence_scope()
            return next_epoch

    def advance_localization_epoch(self) -> int:
        """仅供 PoseProvider 连续性中断调用。"""
        return self.synchronize_localization_epoch(self.localization_epoch + 1)

    def synchronize_localization_epoch(self, epoch: int) -> int:
        """把统一 PoseProvider 的连续性 epoch 原子提交到证据链。"""
        with self._scope_lock:
            requested = int(epoch)
            if requested < self.localization_epoch:
                raise ValueError("定位连续性 epoch 不得回退")
            if requested == self.localization_epoch:
                return self.localization_epoch
            self.localization_epoch = requested
            self.feedback.invalidate_scope(
                floor=None,
                map_epoch=None,
                localization_epoch=self.localization_epoch,
            )
            self.feedback.discard_pending_batches()
            if self.grid is not None:
                self.grid.invalidate_evidence()
            if self.gate is not None:
                self.gate.invalidate_evidence_scope()
            return self.localization_epoch

    def planning_constraints(self, floor: int):
        with self._scope_lock:
            return self.feedback.planning_constraints(
                floor=int(floor),
                map_epoch=int(self._map_epochs.get(int(floor), 1)),
                localization_epoch=int(self.localization_epoch),
            )

    def constraint_regions(self, floor: int):
        """返回冻结断面快照，禁止调用方持有或修改 feedback。"""
        with self._scope_lock:
            return self.feedback.constraint_regions(
                floor=int(floor),
                map_epoch=int(self._map_epochs.get(int(floor), 1)),
                localization_epoch=int(self.localization_epoch),
            )

    def observe_gate_transition(self, state: str):
        """在协调器锁内串行 Gate 状态镜像，禁止编排器直达 feedback。"""
        with self._scope_lock:
            return self.feedback.observe_gate_state(state)

    def set_gate_state_for_test(self, state: str) -> None:
        """测试辅助入口：只设置诊断镜像，不改变 Gate 安全状态。"""
        with self._scope_lock:
            self.feedback.synchronize_gate_state(state)

    def install_path_constraint(
        self,
        *,
        context_key: tuple[object, ...] | None,
        controller=None,
    ) -> GroundSupportDirective:
        """在单一事务中安装约束、校验 ACK 并复位 Gate。"""
        transaction = (
            _controller_command_transaction(controller)
            if controller is not None else nullcontext()
        )
        with transaction, self._scope_lock:
            directive = self._install_path_constraint_locked(
                context_key=context_key
            )
            if directive.action != "constraint_installed":
                return directive
            gate = self.gate
            if gate is None or not gate.reset_path_unavailable(
                directive.constraint_ack
            ):
                return GroundSupportDirective(
                    "terminal",
                    "support_constraint_ack_mismatch",
                    region_id=directive.region_id,
                    failure_count=directive.failure_count,
                    witness_sequence=directive.witness_sequence,
                )
            self.feedback.synchronize_gate_state(gate.state)
            return directive

    def _install_path_constraint_locked(
        self,
        *,
        context_key: tuple[object, ...] | None,
    ) -> GroundSupportDirective:
        gate = self.gate
        witness = getattr(gate, "failure_witness", None) if gate is not None else None
        if witness is None:
            return GroundSupportDirective(
                "terminal", "support_veto_source_missing"
            )
        if (
            witness.floor < 0
            or witness.map_epoch != int(self._map_epochs.get(witness.floor, 1))
            or witness.localization_epoch != int(self.localization_epoch)
        ):
            return GroundSupportDirective(
                "terminal", "support_veto_scope_mismatch",
                witness_sequence=witness.sequence,
            )
        if self.grid is None:
            return GroundSupportDirective(
                "terminal", "support_constraint_geometry_missing",
                witness_sequence=witness.sequence,
            )
        config = self.grid.config
        geometry_matches = (
            abs(float(witness.grid_resolution) - float(config.resolution)) <= 1e-9
            and abs(float(witness.grid_origin[0]) - float(config.origin_x)) <= 1e-9
            and abs(float(witness.grid_origin[1]) - float(config.origin_y)) <= 1e-9
        )
        if not geometry_matches:
            return GroundSupportDirective(
                "terminal", "support_constraint_geometry_mismatch",
                witness_sequence=witness.sequence,
            )
        blocked = _expand_source_cell_aabbs(
            witness.source_cells,
            resolution=float(witness.grid_resolution),
            sweep_half_width=float(self.params["sweep_half_width"]),
        )
        result: ConstraintInstallResult = self.feedback.install_constraint(
            witness,
            blocked_cells=blocked,
            context_key=context_key,
        )
        if not result.installed:
            return GroundSupportDirective(
                "terminal", result.reason_code,
                region_id=result.region_id,
                failure_count=result.failure_count,
                witness_sequence=witness.sequence,
            )
        ack = SupportConstraintAck(
            witness_sequence=witness.sequence,
            floor=witness.floor,
            map_epoch=witness.map_epoch,
            localization_epoch=witness.localization_epoch,
            constraint_revision=result.revision,
        )
        return GroundSupportDirective(
            "constraint_installed", result.reason_code,
            region_id=result.region_id,
            failure_count=result.failure_count,
            witness_sequence=witness.sequence,
            constraint_ack=ack,
        )

    def queue_confirmed_support(
        self,
        result: dict,
        *,
        default_floor: int,
        cloud_stamp: float,
        cloud_generation: int,
    ) -> None:
        with self._scope_lock:
            if not result.get("confirmed_supported_cells") or self.grid is None:
                return
            config = self.grid.config
            floor = int(default_floor)
            self.feedback.queue_confirmed_support(
                result,
                default_floor=floor,
                map_epoch=self._map_epochs.get(floor, 1),
                localization_epoch=self.localization_epoch,
                grid_origin=(config.origin_x, config.origin_y),
                grid_resolution=config.resolution,
                cloud_stamp=cloud_stamp,
                cloud_generation=cloud_generation,
            )

    def apply_confirmed_support(self, mapper=None):
        with self._scope_lock:
            if self.grid is None:
                update = self.feedback.apply_confirmed_support(
                    floor=self._active_floor,
                    map_epoch=self.map_epoch,
                    localization_epoch=self.localization_epoch,
                    grid_origin=(0.0, 0.0),
                    grid_resolution=1.0,
                )
            else:
                config = self.grid.config
                update = self.feedback.apply_confirmed_support(
                    floor=self._active_floor,
                    map_epoch=self.map_epoch,
                    localization_epoch=self.localization_epoch,
                    grid_origin=(config.origin_x, config.origin_y),
                    grid_resolution=config.resolution,
                )
            if mapper is not None:
                for batch in update.confirmed_batches:
                    mapper.clear_negative_clusters_with_support(
                        floor=batch.floor,
                        cells=batch.cells,
                        map_epoch=batch.map_epoch,
                        localization_epoch=batch.localization_epoch,
                        generation=batch.cloud_generation,
                        grid_origin=batch.grid_origin,
                        grid_resolution=batch.grid_resolution,
                    )
            return update

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
        cloud_generation: int = 0,
    ) -> None:
        """用一帧合法点云更新 ShadowSupport 与最终 Gate 评估。"""
        gate = self.gate
        grid = self.grid
        if gate is None or grid is None or cloud_stamp is None:
            return
        if pose_provider is None:
            return
        with _controller_command_transaction(controller):
            if gate.hole_detection_exempt:
                return
            hole_scope_revision = gate.hole_scope_revision
        with self._scope_lock:
            if int(floor_index) != self._active_floor:
                return
            if self._last_hole_scope_revision != hole_scope_revision:
                grid.invalidate_evidence()
                self.feedback.discard_pending_batches()
                self._last_hole_scope_revision = hole_scope_revision
            frame_scope = (
                int(floor_index),
                int(self._map_epochs.get(int(floor_index), 1)),
                int(self.localization_epoch),
            )
        params = self.params
        sensor_to_world = pose_provider.sensor_to_world_matrix(
            pose_sample,
            base_to_sensor_xyz=params["base_to_livox_xyz"],
            base_to_sensor_rpy=params["base_to_livox_rpy"],
        )
        sensor_to_world = np.array(sensor_to_world, dtype=float, copy=True)
        expected_floor_z = float(floor_index) * float(grid.config.floor_height)
        if sensor_to_world[2, 3] < expected_floor_z:
            sensor_to_world[2, 3] += expected_floor_z
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
        # ShadowSupport 是有状态证据场。处理旧帧期间必须阻止 scope 事务
        # 清空证据，否则旧帧可能在清空后重新写回。这里不持 controller 锁，
        # 避免与 controller -> scope 的全局锁序形成反向嵌套。
        with self._scope_lock:
            current_scope = (
                self._active_floor,
                self._map_epochs.get(self._active_floor, 1),
                self.localization_epoch,
            )
            if current_scope != frame_scope:
                return
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
            floor=int(floor_index),
            map_epoch=frame_scope[1],
            localization_epoch=frame_scope[2],
            cloud_generation=int(cloud_generation),
            support_boundary_cells=tuple(
                sorted(
                    {
                        (int(cell[0]), int(cell[1]))
                        for cell in payload.get("support_boundary_cells", ())
                        if isinstance(cell, (list, tuple)) and len(cell) >= 2
                    }
                )
            ),
            grid_origin=_optional_point(payload.get("grid_origin")),
            grid_resolution=_optional_finite(payload.get("grid_resolution")),
            drop_clusters=_drop_cluster_evidence(
                payload.get("drop_clusters"),
                floor=int(floor_index),
                map_epoch=frame_scope[1],
                localization_epoch=frame_scope[2],
                stamp=float(cloud_stamp),
                generation=int(cloud_generation),
                grid_origin=_optional_point(payload.get("grid_origin")),
                grid_resolution=_optional_finite(payload.get("grid_resolution")),
            ),
        )
        with _controller_command_transaction(controller), self._scope_lock:
            if gate.hole_detection_exempt or gate.hole_scope_revision != hole_scope_revision:
                return
            current_scope = (
                self._active_floor,
                self._map_epochs.get(self._active_floor, 1),
                self.localization_epoch,
            )
            if current_scope != frame_scope:
                return
            self.queue_confirmed_support(
                result,
                default_floor=int(floor_index),
                cloud_stamp=float(cloud_stamp),
                cloud_generation=int(cloud_generation),
            )
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

    def persist_latched_drop_clusters(
        self,
        *,
        controller,
        mapper,
        floor_index: int,
        mission_state: str,
        task_active: bool,
        ordinary_states: frozenset[str],
    ) -> NegativeClusterPersistResult | None:
        """原子持久化 Gate 锁存的完整 DROP 簇并以精确 ACK 复位。"""
        params = self.params
        gate = self.gate
        if gate is None or not bool(
            params.get("negative_obstacle_replan_enabled", True)
        ):
            return None
        if task_active or str(mission_state) not in ordinary_states:
            return None
        with _controller_command_transaction(controller), self._scope_lock:
            active_scope = (
                int(self._active_floor),
                int(self._map_epochs.get(self._active_floor, 1)),
                int(self.localization_epoch),
            )
            if int(floor_index) != active_scope[0]:
                return None
            clusters = tuple(getattr(gate, "latched_drop_clusters", ()))
            if not clusters or any(
                cluster.floor != active_scope[0]
                or cluster.frame_id != "odom"
                or cluster.map_epoch != active_scope[1]
                or cluster.localization_epoch != active_scope[2]
                for cluster in clusters
            ):
                return None
            records = tuple(
                NegativeClusterRecord(
                    cluster_id=cluster.cluster_id,
                    floor=cluster.floor,
                    frame_id=cluster.frame_id,
                    map_epoch=cluster.map_epoch,
                    localization_epoch=cluster.localization_epoch,
                    stamp=cluster.stamp,
                    generation=cluster.generation,
                    grid_origin=cluster.grid_origin,
                    grid_resolution=cluster.grid_resolution,
                    source_cells=frozenset(cluster.source_cells),
                )
                for cluster in clusters
            )
            maximum = max(
                0, int(params.get("maximum_negative_obstacle_replans", 3))
            )
            commit = mapper.persist_negative_clusters_atomic(
                records,
                allow_new=self._negative_obstacle_commit_count < maximum,
            )
            if not commit.committed:
                return None
            controller.clear_path(reason="negative_obstacle_replan")
            ack = NegativeClusterAck(
                floor=active_scope[0],
                map_epoch=active_scope[1],
                localization_epoch=active_scope[2],
                cluster_ids=commit.cluster_ids,
                map_revision=commit.revision,
            )
            if not gate.reset_negative_obstacle_after_replan(ack):
                return None
            self.feedback.synchronize_gate_state(gate.state)
            if commit.reason_code == "committed":
                self._negative_obstacle_commit_count += 1
            return NegativeClusterPersistResult(
                floor=active_scope[0],
                cluster_ids=commit.cluster_ids,
                source_cell_count=sum(
                    len(cluster.source_cells) for cluster in clusters
                ),
                map_revision=commit.revision,
            )


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


def _optional_point(value) -> tuple[float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    point = (float(value[0]), float(value[1]))
    return point if all(math.isfinite(component) for component in point) else None


def _drop_cluster_evidence(
    payload,
    *,
    floor: int,
    map_epoch: int,
    localization_epoch: int,
    stamp: float,
    generation: int,
    grid_origin: tuple[float, float] | None,
    grid_resolution: float | None,
) -> tuple[DropClusterEvidence, ...]:
    if (
        not isinstance(payload, (list, tuple))
        or grid_origin is None
        or grid_resolution is None
        or generation <= 0
    ):
        return ()
    evidence = []
    for index, raw_cluster in enumerate(payload):
        cells = tuple(sorted(_valid_grid_cells(raw_cluster)))
        if not cells:
            return ()
        evidence.append(
            DropClusterEvidence(
                cluster_id=(
                    f"drop:{floor}:{map_epoch}:{localization_epoch}:"
                    f"{generation}:{index}"
                ),
                floor=floor,
                frame_id="odom",
                map_epoch=map_epoch,
                localization_epoch=localization_epoch,
                stamp=stamp,
                generation=generation,
                grid_origin=grid_origin,
                grid_resolution=grid_resolution,
                source_cells=cells,
            )
        )
    return tuple(evidence)


def _valid_grid_cells(payload) -> set[tuple[int, int]]:
    if not isinstance(payload, (list, tuple)):
        return set()
    cells = set()
    for item in payload:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            return set()
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float, np.integer, np.floating))
            or not math.isfinite(float(value))
            or not float(value).is_integer()
            for value in item
        ):
            return set()
        try:
            cells.add((int(item[0]), int(item[1])))
        except (TypeError, ValueError, OverflowError):
            return set()
    return cells


def _expand_source_cell_aabbs(
    source_cells,
    *,
    resolution: float,
    sweep_half_width: float,
) -> set[tuple[int, int]]:
    """按格方块间的真实欧氏距离扩张失败断面。"""
    reach = int(math.ceil((float(sweep_half_width) + resolution) / resolution))
    blocked: set[tuple[int, int]] = set()
    for gx, gy in source_cells:
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                gap_x = max(0.0, (abs(dx) - 1) * resolution)
                gap_y = max(0.0, (abs(dy) - 1) * resolution)
                if math.hypot(gap_x, gap_y) <= sweep_half_width + 1e-9:
                    blocked.add((int(gx) + dx, int(gy) + dy))
    return blocked

import json
import math
import numpy as np
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.building_control_client import BuildingControlClient, DoorInfo
from danger_search.door_observation_runtime import DoorObservationRuntime
from danger_search.building_feature_provider import (
    PublicDoorFeature,
    PublicElevatorFeature,
)
from danger_search.local_semantic_detector import DoorCandidate, OpeningObservation
from danger_search.rgbd_door_detector import DoorOpeningChange, DoorPanelObservation
from danger_search.ground_support_gate import (
    CLEAR as GROUND_SUPPORT_CLEAR,
    DROP_STOPPING,
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SUPPORT_PATH_UNAVAILABLE,
    GroundSupportAssessment,
    GroundSupportGateConfig,
    GroundSupportSafetyGate,
    DropClusterEvidence,
    SupportFailureWitness,
)
from danger_search.ground_support_runtime import GroundSupportFeedbackRuntime
from danger_search.ground_support_coordinator import GroundSupportCoordinator
import danger_search.ground_support_coordinator as ground_support_coordinator_module
from danger_search.floor_transition_runtime import (
    FloorCoordinateModel,
    FloorTransitionRuntime,
)
from danger_search.frontier import FloorTerminationEvidence, Frontier
from danger_search.frontier_planning_runtime import (
    MODE_NORMAL,
    MODE_OPPORTUNISTIC,
    MODE_PREFETCH,
    FrontierPlanScope,
    FrontierPlanningRuntime,
)
from danger_search.elevator_observation_runtime import ElevatorObservationRuntime
from danger_search.exploration_runtime import (
    ExplorationBudgetInput,
    ExplorationRuntime,
    decide_exploration_budget,
)
from danger_search.return_home_runtime import ReturnHomeRuntime
import danger_search.return_home_runtime as return_home_runtime_module
from danger_search.initial_entry_runtime import (
    InitialEntryConfig,
    InitialEntryRuntime,
    initial_forward_free_target,
    probe_target_for_attempt,
)
from danger_search.perception_runtime import PerceptionRuntime
from danger_search.perception_mapping_runtime import PerceptionMappingRuntime
from danger_search.map_update_runtime import (
    CloudMapResult,
    ImageDetectionResult,
    nearest_forward_obstacle_observation as map_forward_observation,
    nearest_radial_obstacle_distance as map_radial_distance,
)
from danger_search.mission_config import (
    effective_max_runtime,
    load_mission_config,
    optional_positive_float,
    parse_stop_distance_table,
    validate_summary_output_path,
)
from danger_search.mission_node_runtime import wait_for_localization_ready
from danger_search.mission_reporting import MissionReportingRuntime
from danger_search.mapper import (
    BLACK,
    FREE,
    NEGATIVE,
    OCC,
    UNKNOWN,
    VISITED,
    MultiFloorGridMap,
    NegativeClusterRecord,
)
import danger_search.mission_manager as mission_manager_module
import danger_search.initial_entry_runtime as initial_entry_runtime_module
import danger_search.return_home_planner as return_home_planner_module
from danger_search.mission_manager import (
    CHANGE_FLOOR,
    EXPLORE_FLOOR,
    FINISH,
    FloorExitCertificate,
    MISSION_FAILED,
    RETURN_HOME,
    WRITE_RESULT,
    forward_obstacle_lateral_limit,
    nearest_forward_obstacle_distance,
    nearest_forward_obstacle_observation,
)
from danger_search.mission_manager import MissionManager
from danger_search.navigation_tasks import DONE, RUNNING, OpenDoorAndPass, TakeElevator, FAILED
from danger_search.navigator import IDLE, STUCK, PathProgressSnapshot
from danger_search.planner import PlanResult, BUDGET_EXHAUSTED, astar_plan
from danger_search.sensor_buffer import SensorBuffer
from danger_search.return_home_planner import (
    apply_return_home_vetoes,
    compute_return_home_plan,
    copy_floor_grid,
    path_crosses_blocked_beyond,
    return_path_from_navigation_trace,
    return_segment_signature,
    validate_return_home_candidate,
)
from danger_search.progress_monitor import ProgressMonitor

def _install_runtime_owners(manager):
    """为绕过构造器的单元测试安装新的职责所有者。"""
    # 保留旧私有方法通过 getattr 使用的探索预算默认值。策略迁出后测试必须
    # 显式提供完整事实，不能再依赖生产方法悄悄补默认值。
    manager.home_pose = None
    manager.max_runtime = 840.0
    manager.max_floor = 1
    manager.return_safety_buffer = 30.0
    manager.return_speed_estimate = 0.35
    manager.floor_change_time_estimate = 35.0
    manager.floor_timeout = 180.0
    manager.floor_finish_time_ratio = 0.75
    manager.exploration_progress_window = 15.0
    manager.min_exploration_progress_cells = 30
    manager.min_remaining_frontiers = 2
    manager.last_floor_uses_global_budget = True
    manager._map_runtime = _TestMapRuntime()
    manager._perception_runtime = PerceptionRuntime(SimpleNamespace())
    manager._ground_support = _TestGroundSupportCoordinator()
    manager._floor_transition_runtime = FloorTransitionRuntime()
    manager._elevator_observation = ElevatorObservationRuntime()
    manager._exploration = ExplorationRuntime()
    # 大量状态机单测验证既有同步分支；异步行为由专门用例显式开启。
    manager.async_frontier_planning_enabled = False
    manager._return_home = ReturnHomeRuntime()
    manager._entry = InitialEntryRuntime()
    manager._perception_mapping = PerceptionMappingRuntime(
        perception=manager._perception_runtime,
        map_runtime=manager._map_runtime,
        floor_coordinates=manager._floor_coordinates,
        pose_provider=manager.pose_provider,
    )
    return manager


def _make_test_manager() -> MissionManager:
    """通过正式构造边界创建不启动 ROS 的 diagnostic 任务对象。"""
    config = load_mission_config(
        lambda name, default=None: (
            "diagnostic" if name == "runtime/profile" else default
        )
    )
    manager = MissionManager(config=config)
    return _install_runtime_owners(manager)


def _initial_entry_config_for(manager) -> InitialEntryConfig:
    return InitialEntryConfig(
        max_distance=float(manager.initial_entry_max_distance),
        corridor_half_width=float(manager.initial_entry_corridor_half_width),
        probe_distance=float(manager.probe_distance),
        opening_confirmations=int(manager.initial_entry_opening_confirmations),
        opening_max_distance=float(manager.initial_entry_opening_max_distance),
        geometry_tolerance=float(manager.initial_entry_geometry_tolerance),
        door_pass_offset=float(manager.door_pass_offset),
        door_commit_distance=float(manager.door_commit_distance),
    )


def _try_start_entry_door(manager, pose, entrance) -> bool:
    result = manager._entry.try_start_door_task(
        pose=pose,
        entrance=entrance,
        current_floor=manager.current_floor,
        controller=manager.controller,
        building_client=manager.building_client,
        local_detector=manager.local_detector,
        door_tracker=manager._door_observation_runtime.initial,
        cloud_stamp=manager._map_runtime.last_cloud_stamp,
        cloud_points=manager._map_runtime.latest_cloud_points,
        floor_height=manager.mapper.floor_height,
        config=_initial_entry_config_for(manager),
    )
    if result.task is not None:
        manager.current_task = result.task
        manager.active_door = result.door_id
    if result.clear_exploration_target:
        manager.current_frontier = None
        manager._exploration.last_frontier_target = None
        manager._exploration.last_frontier_score = None
        manager._exploration.last_frontier_used_fallback = False
    return result.handled


def _register_test_elevator_geometry(manager, elevator_feature, **kwargs):
    """通过电梯注册运行时安装测试几何，不恢复已删除的管理器私有入口。"""
    result = manager._elevator_observation.registration.register_confirmed_geometry(
        elevator_feature=elevator_feature,
        current_floor=manager.current_floor,
        building_client=manager.building_client,
        landmark_monitor=getattr(manager, "_landmark_drift_monitor", None),
        elapsed=manager._elapsed(),
        **kwargs,
    )
    manager._apply_elevator_registration_result(result)
    return result


def _try_register_test_elevator(manager, pose) -> bool:
    """直接驱动电梯注册运行时，不恢复管理器旧私有方法。"""
    manager._elevator_observation.log_visual_preparation_result()
    result = manager._elevator_observation.registration.try_register(
        pose=pose,
        current_floor=manager.current_floor,
        max_floor=manager.max_floor,
        current_task_present=manager.current_task is not None,
        building_features=manager.building_features,
        building_client=manager.building_client,
        local_detector=manager.local_detector,
        cloud_stamp=manager._map_runtime.last_cloud_stamp,
        cloud_points=manager._map_runtime.latest_cloud_points,
        floor_height=manager.mapper.floor_height,
        entrance_xy=manager.main_entrance_xy,
        config=manager._elevator_registration_config(),
        landmark_monitor=getattr(manager, "_landmark_drift_monitor", None),
        elapsed=manager._elapsed(),
    )
    manager._apply_elevator_registration_result(result)
    return result.registered


def _door_candidate_decision(manager, candidate):
    return manager._door_navigation.candidate_decision(
        candidate,
        exploring=manager.state == EXPLORE_FLOOR,
        current_task_present=manager.current_task is not None,
        opened_doors=manager.opened_doors,
        failed_doors=manager.failed_doors,
        active_door=manager.active_door,
        building_client=manager.building_client,
        maximum_distance=manager.local_detector.max_distance,
    )


def _select_door_approach(manager, pose):
    return manager._door_navigation.select_approach(
        pose,
        floor_index=manager.current_floor,
        grid=manager.mapper.floor(manager.current_floor),
        opened_doors=manager.opened_doors,
        failed_doors=manager.failed_doors,
        building_client=manager.building_client,
        pass_offset=manager.door_pass_offset,
        now=mission_manager_module.get_time(),
    )


class _TestMapRuntime:
    def __init__(self):
        self.last_cloud_stamp = None
        self.last_image_stamp = None
        self.latest_cloud_points = []
        self.nearest_forward_obstacle = None
        self.nearest_radial_obstacle_distance = None
        self.last_floor_alignment_mismatch_key = None
        self.last_rgbd_pose_lookup_log_time = 0.0
        self.mapper = None
        self.detector = None
        self.local_detector = None
        self.rgbd_door_detector = None
        self.tracker = None

    def update_cloud(
        self,
        *,
        floor_index,
        cloud_generation,
        map_epoch,
        localization_epoch,
        robot_xyz,
        robot_pose,
        points,
        opened_door_exclusion,
    ):
        self.mapper.update_from_cloud(
            floor_index=floor_index,
            robot_xyz=robot_xyz,
            points=points,
            cloud_generation=cloud_generation,
            map_epoch=map_epoch,
            localization_epoch=localization_epoch,
        )
        return CloudMapResult(
            tuple(points),
            map_forward_observation(
                points,
                robot_pose,
                floor_index=floor_index,
                mapper=self.mapper,
                opened_door_exclusion=opened_door_exclusion,
            ),
            map_radial_distance(
                points,
                robot_pose,
                floor_index=floor_index,
                mapper=self.mapper,
            ),
        )

    def update_images(
        self,
        bundle,
        camera_to_world,
        *,
        now,
        floor_index=None,
        robot_xy=None,
    ):
        candidates = self.detector.detect_messages(
            bundle.rgb_msg,
            bundle.depth_msg,
            bundle.camera_info_msg,
            camera_to_world,
            require_world_transform=True,
        )
        if candidates:
            self.tracker.update(candidates, now=now)
        observations = ()
        if self.rgbd_door_detector is not None:
            observations = tuple(
                self.rgbd_door_detector.detect_messages(
                    bundle.rgb_msg,
                    bundle.depth_msg,
                    bundle.camera_info_msg,
                    camera_to_world,
                )
            )
        return ImageDetectionResult(observations)


class _TestGroundSupportCoordinator:
    def __init__(self):
        from threading import RLock

        self.params = {}
        self._scope_lock = RLock()
        self.gate = None
        self.grid = None
        self.feedback = GroundSupportFeedbackRuntime()
        self._active_floor = 0
        self._map_epochs = {0: 1, 1: 1}
        self.map_epoch = 1
        self.localization_epoch = 1
        self.last_cloud_generation = 0
        self.last_cloud_stamp = None
        self._negative_obstacle_commit_count = 0

    def apply_confirmed_support(self, mapper=None):
        return GroundSupportCoordinator.apply_confirmed_support(self, mapper)

    def queue_confirmed_support(self, result, *, default_floor, cloud_stamp, cloud_generation):
        return GroundSupportCoordinator.queue_confirmed_support(
            self,
            result,
            default_floor=default_floor,
            cloud_stamp=cloud_stamp,
            cloud_generation=cloud_generation,
        )

    def observe_odometry(self, controller, pose_sample):
        return GroundSupportCoordinator.observe_odometry(
            self,
            controller,
            pose_sample,
        )

    def planning_constraints(self, floor):
        return self.feedback.planning_constraints(
            floor=int(floor),
            map_epoch=int(self.map_epoch),
            localization_epoch=int(self.localization_epoch),
        )

    def constraint_regions(self, floor):
        return self.feedback.constraint_regions(
            floor=int(floor),
            map_epoch=int(self.map_epoch),
            localization_epoch=int(self.localization_epoch),
        )

    def map_epoch_for_floor(self, floor):
        return int(self.map_epoch)

    def observe_gate_transition(self, state):
        return self.feedback.observe_gate_state(state)

    def set_gate_state_for_test(self, state):
        self.feedback.synchronize_gate_state(state)

    def invalidate_active_assessment(self):
        gate = self.gate
        invalidate = getattr(gate, "invalidate_evidence_scope", None)
        if not callable(invalidate):
            invalidate = getattr(gate, "invalidate_assessment", None)
        if callable(invalidate):
            invalidate()

    def activate_floor(self, floor):
        self._active_floor = int(floor)
        self._map_epochs.setdefault(self._active_floor, 1)
        self.map_epoch = self._map_epochs[self._active_floor]
        self.invalidate_active_assessment()

    def advance_map_epoch(self):
        return GroundSupportCoordinator.advance_map_epoch(self)

    def advance_localization_epoch(self):
        return GroundSupportCoordinator.advance_localization_epoch(self)

    def install_path_constraint(self, *, context_key, controller=None):
        return GroundSupportCoordinator.install_path_constraint(
            self, context_key=context_key, controller=controller
        )

    def _install_path_constraint_locked(self, *, context_key):
        return GroundSupportCoordinator._install_path_constraint_locked(
            self, context_key=context_key
        )

    def assess_frame(self, **kwargs):
        return GroundSupportCoordinator.assess_frame(self, **kwargs)

    def persist_latched_drop_clusters(self, **kwargs):
        return GroundSupportCoordinator.persist_latched_drop_clusters(
            self, **kwargs
        )


def _set_support_failure_witness(
    manager,
    gate,
    *,
    sequence: int,
    cells,
) -> None:
    manager._ground_support.params = {"sweep_half_width": 0.2}
    manager._ground_support.grid = SimpleNamespace(
        config=SimpleNamespace(resolution=0.1, origin_x=-14.0, origin_y=-6.0)
    )
    gate._pending_failure_witness = SupportFailureWitness(
        sequence=sequence,
        floor=int(manager.current_floor),
        map_epoch=int(manager._ground_support.map_epoch),
        localization_epoch=int(manager._ground_support.localization_epoch),
        stamp=float(sequence),
        generation=sequence,
        source_cells=cells,
        hold_context_id=sequence,
        trigger_command=(0.2, 0.0, 0.0),
        grid_origin=(-14.0, -6.0),
        grid_resolution=0.1,
    )

class MissionManagerHelperTest(unittest.TestCase):
    def test_tick_wall_timeout_triggers_even_when_ground_support_is_clear(self):
        manager = _make_test_manager()
        manager.tick_wall_stall_timeout = 5.0
        manager.state = EXPLORE_FLOOR
        manager._last_tick_wall_time = 100.0
        manager._ground_support.gate = SimpleNamespace(state=GROUND_SUPPORT_CLEAR)

        timed_out, reason = manager.check_tick_wall_timeout(105.0)

        self.assertTrue(timed_out)
        self.assertEqual(reason, "mission_tick_wall_stalled")

    def test_tick_wall_timeout_is_not_armed_before_first_tick(self):
        manager = _make_test_manager()
        manager.tick_wall_stall_timeout = 5.0
        manager.state = EXPLORE_FLOOR
        manager._last_tick_wall_time = None

        timed_out, reason = manager.check_tick_wall_timeout(105.0)

        self.assertFalse(timed_out)
        self.assertEqual(reason, "mission_tick_wall_stall_not_armed")

    def test_tick_wall_timeout_can_be_disabled(self):
        manager = _make_test_manager()
        manager.tick_wall_stall_timeout = 0.0
        manager.state = EXPLORE_FLOOR
        manager._last_tick_wall_time = 100.0

        timed_out, reason = manager.check_tick_wall_timeout(105.0)

        self.assertFalse(timed_out)
        self.assertEqual(reason, "mission_tick_wall_stall_disabled")

    def test_danger_source_overlap_stops_and_enters_existing_result_failure_flow(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager._mission_success_eligible = True
        stopped = []
        cleared = []
        failures = []
        transitions = []
        events = []
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
            clear_path=lambda **kwargs: cleared.append(kwargs),
        )
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )
        manager._log_structured = events.append

        manager._handle_danger_source_overlap(
            owner_ids=("danger:7",),
            floor=0,
            source="rgbd_confirmation",
        )

        self.assertEqual(stopped, [{"reason": "danger_source_overlap"}])
        self.assertEqual(cleared, [{"reason": "danger_source_overlap"}])
        self.assertEqual(failures, ["danger_source_overlap"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(transitions[0][1]["reason_code"], "danger_source_overlap")
        self.assertEqual(events[0]["event"], "danger_source_overlap")

    def test_tick_stops_before_current_task_when_mapping_enters_write_result(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.max_runtime = 100.0
        manager._elapsed = lambda: 1.0
        manager._publish_status = lambda: None
        manager._perception_runtime = SimpleNamespace(snapshot=lambda: object())
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=1.0,
                xyz=(0.0, 0.0, 0.3),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._apply_localization_progress_decision = lambda _sample: True
        manager._update_pose_progress = lambda _pose: None
        manager._record_initial_entry_trace_pose = lambda _pose: None
        manager._record_floor_navigation_trace_pose = lambda _pose: None
        manager._sync_ground_support_task_context = lambda _pose: None
        manager._ground_support.observe_odometry = lambda *_args: None
        manager._update_mapping_and_detection = lambda *_args: setattr(
            manager,
            "state",
            WRITE_RESULT,
        )
        task_ticks = []
        manager.current_task = SimpleNamespace(
            tick=lambda *_args: task_ticks.append(True) or RUNNING
        )

        manager.tick()

        self.assertEqual(manager.state, WRITE_RESULT)
        self.assertEqual(task_ticks, [])

    def test_landmark_ack_reprojects_danger_tracks_before_resuming_navigation(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._landmark_relocalizer = SimpleNamespace(
            anchors={"elevator": SimpleNamespace(floor=0)},
            config=SimpleNamespace(correction_timeout=2.0),
        )
        manager._landmark_relocalization_operations = {
            "test": {"sequence": 11, "started_at": 0.0}
        }
        manager.pose_provider = SimpleNamespace(
            correction_ack=lambda sequence: (0.3, -0.2, 1.0)
            if sequence == 11
            else None,
            latest=lambda: SimpleNamespace(xy_yaw=(1.3, 1.8, 0.0)),
        )
        applied = []
        manager._map_runtime = SimpleNamespace(
            apply_planar_correction=lambda dx, dy, **kwargs: (
                applied.append((dx, dy, kwargs))
                or SimpleNamespace(
                    translated_track_count=1,
                    selected_track_ids=(7,),
                    changed_owner_ids=("danger:7",),
                    changed_cell_count=49,
                    overlap_owner_ids=(),
                )
            )
        )
        manager.controller = SimpleNamespace(
            stop=lambda **_kwargs: None,
            clear_path=lambda **_kwargs: None,
        )
        events = []
        manager._log_structured = events.append

        result = manager.relocalize_landmark(
            "elevator",
            "lift_door_0",
            (1.0, 2.0, 0.0),
            "test",
        )

        self.assertEqual(result.status, DONE)
        self.assertTrue(result.applied)
        self.assertEqual(len(applied), 1)
        self.assertEqual(applied[0][0:2], (0.3, -0.2))
        self.assertEqual(applied[0][2]["robot_floor"], 0)
        self.assertEqual(applied[0][2]["robot_xy"], (1.3, 1.8))
        self.assertTrue(math.isfinite(applied[0][2]["now"]))
        self.assertEqual(events[0]["event"], "danger_obstacle_reprojected")
        self.assertEqual(events[1]["event"], "sparse_landmark_correction_applied")

    def test_landmark_ack_overlap_keeps_navigation_task_from_advancing(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._landmark_relocalizer = SimpleNamespace(
            anchors={"elevator": SimpleNamespace(floor=0)},
            config=SimpleNamespace(correction_timeout=2.0),
        )
        manager._landmark_relocalization_operations = {
            "test": {"sequence": 12, "started_at": 0.0}
        }
        manager.pose_provider = SimpleNamespace(
            correction_ack=lambda sequence: (0.1, 0.0, 1.0)
            if sequence == 12
            else None,
            latest=lambda: SimpleNamespace(xy_yaw=(1.1, 2.0, 0.0)),
        )
        manager._map_runtime = SimpleNamespace(
            apply_planar_correction=lambda *_args, **_kwargs: SimpleNamespace(
                translated_track_count=1,
                selected_track_ids=(7,),
                changed_owner_ids=("danger:7",),
                changed_cell_count=49,
                overlap_owner_ids=("danger:7",),
            )
        )
        calls = []
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: calls.append(("stop", kwargs)),
            clear_path=lambda **kwargs: calls.append(("clear", kwargs)),
        )
        manager._handle_danger_source_overlap = lambda **kwargs: calls.append(
            ("overlap", kwargs)
        )
        manager._log_structured = lambda _event: None

        result = manager.relocalize_landmark(
            "elevator",
            "lift_door_0",
            (1.0, 2.0, 0.0),
            "test",
        )

        self.assertEqual(result.status, RUNNING)
        self.assertEqual(result.reason, "danger_source_overlap")
        self.assertIn(("overlap", {
            "owner_ids": ("danger:7",),
            "floor": 0,
            "source": "landmark_correction",
        }), calls)
        self.assertNotIn(
            ("clear", {"reason": "landmark_relocalization_applied"}),
            calls,
        )

    def test_ground_support_scope_transition_invalidates_all_old_evidence_atomically(self):
        manager = _make_test_manager()
        manager.ground_support_params["production_gate_enabled"] = True
        coordinator = GroundSupportCoordinator(
            params=manager.ground_support_params,
            map_params=manager.map_params,
            cloud_max_points=manager.cloud_max_points,
            path_goal_tolerance=0.35,
        )
        gate = coordinator.gate
        grid = coordinator.grid
        self.assertIsNotNone(gate)
        self.assertIsNotNone(grid)
        assert gate is not None and grid is not None
        witness = SupportFailureWitness(
            sequence=1, floor=0, map_epoch=1, localization_epoch=1,
            stamp=1.0, generation=1, source_cells=((1, 1),),
            hold_context_id=1, trigger_command=(0.1, 0.0, 0.0),
            grid_origin=(grid.config.origin_x, grid.config.origin_y),
            grid_resolution=grid.config.resolution,
        )
        coordinator.feedback.install_constraint(
            witness, blocked_cells={(1, 1)}
        )
        coordinator.feedback.queue_confirmed_support(
            {"floor": 0, "confirmed_supported_cells": [[1, 1]]},
            default_floor=0, map_epoch=1, localization_epoch=1,
            grid_origin=(grid.config.origin_x, grid.config.origin_y),
            grid_resolution=grid.config.resolution,
            cloud_stamp=2.0, cloud_generation=2,
        )
        gate._assessment = object()
        gate._pending_failure_witness = witness
        grid._cells[0][(1, 1)] = object()

        coordinator.advance_map_epoch(0)

        self.assertIsNone(gate._assessment)
        self.assertIsNone(gate.failure_witness)
        self.assertEqual(grid._cells.get(0), None)
        self.assertIsNone(coordinator.planning_constraints(0))
        self.assertEqual(coordinator.feedback._pending_supported, [])

    def test_floor_switch_does_not_advance_map_or_localization_epoch(self):
        manager = _make_test_manager()
        coordinator = manager._ground_support
        original_localization = coordinator.localization_epoch
        coordinator.activate_floor(1)
        self.assertEqual(coordinator.map_epoch, 1)
        coordinator.activate_floor(0)
        self.assertEqual(coordinator.map_epoch, 1)
        self.assertEqual(coordinator.localization_epoch, original_localization)

    def test_support_constraint_blocked_locks_success_only_at_floor_termination(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._last_pose = (0.0, 0.0, 0.0)
        manager._map_runtime.mapper = SimpleNamespace(
            floor=lambda _floor: object()
        )
        manager._mission_success_eligible = True
        constraints = SimpleNamespace(revision=9)
        manager._planning_constraints = lambda: constraints
        manager.explorer = SimpleNamespace(
            floor_termination_evidence=lambda _grid, _pose, current, **_kwargs: (
                FloorTerminationEvidence(
                    constraint_revision=9,
                    region_ids=("support:9",),
                    formal_reachable_count=2,
                    effective_reachable_count=0,
                    support_blocked_count=2,
                    fully_blocked=True,
                )
                if current is constraints else None
            )
        )
        failures = []
        logs = []
        manager._record_failure = failures.append
        manager._log_structured = logs.append

        self.assertTrue(manager._mission_success_eligible)
        self.assertTrue(manager._lock_support_constraint_blocked_completion())
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(failures, ["support_constraint_blocked"])
        self.assertTrue(manager._lock_support_constraint_blocked_completion())
        self.assertEqual(failures, ["support_constraint_blocked"])

    @staticmethod
    def _finish_return_home_plan(manager, pose):
        planner = manager._return_home.planner
        planner._thread.join(timeout=1.0)
        if planner._thread.is_alive():
            raise AssertionError("返航后台规划未在测试时限内完成")
        manager._tick_return_home(pose)

    def test_default_ground_support_contract_matches_localization_deadbands(self):
        def parameter(name, default=None):
            return "diagnostic" if name == "runtime/profile" else default

        config = load_mission_config(parameter)
        params = config.values["ground_support_params"]
        self.assertEqual(params["odometry_linear_deadband"], 0.05)
        self.assertEqual(params["odometry_angular_deadband"], 0.03)
        self.assertGreaterEqual(
            params["stop_linear_threshold"],
            params["odometry_linear_deadband"],
        )
        self.assertGreaterEqual(
            params["stop_angular_threshold"],
            params["odometry_angular_deadband"],
        )

        installed = []
        manager = _make_test_manager()
        config.install(manager)
        manager.ground_support_params["production_gate_enabled"] = True
        manager.controller = SimpleNamespace(
            goal_tolerance=0.35,
            set_command_gate=lambda gate: installed.append(gate),
            poll_status=lambda: 'RUNNING',
        )
        manager._ground_support = GroundSupportCoordinator(
            params=manager.ground_support_params,
            map_params=manager.map_params,
            cloud_max_points=manager.cloud_max_points,
            path_goal_tolerance=manager.controller.goal_tolerance,
        )
        manager.controller.set_command_gate(manager.ground_support_gate)

        self.assertEqual(len(installed), 1)
        self.assertIs(installed[0], manager._ground_support_gate)
        self.assertEqual(
            manager._ground_support_gate.config.odometry_linear_deadband,
            0.05,
        )

    def test_async_frontier_planning_defaults_and_time_validation(self):
        config = load_mission_config(
            lambda name, default=None: (
                "diagnostic" if name == "runtime/profile" else default
            )
        )
        self.assertTrue(config.values["async_frontier_planning_enabled"])
        self.assertEqual(config.values["opportunistic_replan_period"], 3.0)
        self.assertEqual(config.values["frontier_preplan_min_lead"], 0.5)
        self.assertEqual(config.values["frontier_preplan_max_lead"], 3.0)

        def invalid_parameter(name, default=None):
            if name == "runtime/profile":
                return "diagnostic"
            if name == "frontier/preplan_min_lead":
                return 4.0
            if name == "frontier/preplan_max_lead":
                return 3.0
            return default

        with self.assertRaisesRegex(ValueError, "异步前沿规划时间参数无效"):
            load_mission_config(invalid_parameter)

    def test_async_frontier_scope_rejects_terminal_opportunity_but_keeps_prefetch(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_floor = 0
        manager.current_task = None
        manager._exploration.last_frontier_target = (2.0, 0.0)
        constraints = manager._planning_constraints()
        common = dict(
            request_id=1,
            floor=0,
            map_epoch=manager._frontier_gain_map_epoch(),
            localization_epoch=manager._ground_support.localization_epoch,
            constraint_revision=getattr(constraints, "revision", 0),
            path_sequence=7,
            active_target=(2.0, 0.0),
        )
        running = PathProgressSnapshot(
            True, 1.0, (2.0, 0.0), 0.0, "frontier_selected_path",
            RUNNING, 9, 7, False,
        )
        terminal = PathProgressSnapshot(
            True, 0.0, (2.0, 0.0), 0.0, "frontier_selected_path",
            DONE, 10, 7, True,
        )

        self.assertTrue(
            manager._frontier_plan_scope_current(
                FrontierPlanScope(mode=MODE_OPPORTUNISTIC, **common), running
            )
        )
        self.assertFalse(
            manager._frontier_plan_scope_current(
                FrontierPlanScope(mode=MODE_OPPORTUNISTIC, **common), terminal
            )
        )
        self.assertTrue(
            manager._frontier_plan_scope_current(
                FrontierPlanScope(mode=MODE_PREFETCH, **common), terminal
            )
        )

        idle_stopped = PathProgressSnapshot(
            False, 0.0, None, 0.0, None,
            "IDLE", 11, 7, False,
        )
        idle_terminal_pending = PathProgressSnapshot(
            False, 0.0, None, 0.0, "main_entrance_clearance_completed",
            "IDLE", 12, 7, True,
        )
        self.assertTrue(
            manager._frontier_plan_scope_current(
                FrontierPlanScope(mode=MODE_NORMAL, **common), idle_stopped
            )
        )
        self.assertTrue(
            manager._frontier_plan_scope_current(
                FrontierPlanScope(mode=MODE_NORMAL, **common), idle_terminal_pending
            )
        )
        self.assertFalse(
            manager._frontier_plan_scope_current(
                FrontierPlanScope(mode=MODE_NORMAL, **common), running
            )
        )

    def test_external_sensor_topic_cannot_use_output_channel(self):
        def parameter(name, default=None):
            if name == "runtime/profile":
                return "diagnostic"
            if name == "topics/rgb":
                return "/cmd_vel"
            return default

        with self.assertRaisesRegex(ValueError, "不是允许的生产输入话题"):
            load_mission_config(parameter)

    def test_formal_runner_waits_until_latched_localization_ready(self):
        callbacks = []
        unregistered = []

        class Subscription:
            def unregister(self):
                unregistered.append(True)

        def subscribe(topic, message_type, callback, queue_size):
            callbacks.append((topic, message_type, queue_size))
            callback(SimpleNamespace(data=False))
            callback(SimpleNamespace(data=True))
            return Subscription()

        ready = wait_for_localization_ready(
            subscriber_factory=subscribe,
            shutdown_check=lambda: False,
            poll_interval=0.01,
        )

        self.assertTrue(ready)
        self.assertEqual(callbacks[0][0], "/danger_search/localization/ready")
        self.assertEqual(callbacks[0][2], 1)
        self.assertEqual(unregistered, [True])

    def test_failed_elevator_task_records_false_summary_state(self):
        manager = _make_test_manager()
        manager._map_runtime.mapper = SimpleNamespace()
        manager.state = CHANGE_FLOOR
        manager.max_runtime = 100.0
        manager._elevator_success = True
        manager._mission_success_eligible = True
        manager._floor_transition_runtime._guard = None
        manager.return_after_floor_change = False
        manager.active_door = None
        manager.current_task = TakeElevator(0, 1)
        manager.current_task.tick = lambda _manager, _pose: FAILED
        manager.navigation_task_context = lambda: object()
        manager.controller = SimpleNamespace(stop=lambda **_kwargs: None)
        manager._perception_runtime.sensor_buffer = SimpleNamespace(snapshot=lambda: SimpleNamespace())
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=1.0,
                xyz=(0.0, 0.0, 0.3),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._publish_status = lambda: None
        manager._elapsed = lambda: 1.0
        manager._apply_localization_progress_decision = lambda _sample: True
        manager._localization_health = True
        manager.localization_stance_health_required = True
        manager._update_pose_progress = lambda _pose: None
        manager._record_initial_entry_trace_pose = lambda _pose: None
        manager._record_floor_navigation_trace_pose = lambda _pose: None
        manager._sync_ground_support_task_context = lambda _pose: None
        manager._ground_support.observe_odometry = lambda *_args: None
        manager._update_mapping_and_detection = lambda *_args: None
        manager._handle_ground_support_control = lambda _pose: False
        failures = []
        transitions = []
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs.get("reason_code"))
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(logwarn=lambda *_args, **_kwargs: None),
        ):
            manager.tick()

        self.assertFalse(manager._elevator_success)
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(failures, ["换层任务失败"])
        self.assertEqual(transitions[0][0], RETURN_HOME)

    def test_successful_elevator_exit_records_context_and_clears_old_frontier_state(self):
        manager = _make_test_manager()
        manager.state = CHANGE_FLOOR
        manager.max_runtime = 100.0
        manager.current_floor = 0
        manager.return_after_floor_change = False
        manager.door_transition_duration = 1.5
        manager.post_door_context_max_duration = 8.0
        manager.post_door_min_known_area = 0.0
        manager.building_client.set_public_features(
            door_ids={"lift_door_0", "lift_door_1"},
            elevators={"elevator_main": {0, 1}},
            elevator_door_ids_by_floor={
                0: ["lift_door_0"],
                1: ["lift_door_1"],
            },
        )
        manager.building_client.doors["lift_door_1"] = DoorInfo(
            door_id="lift_door_1",
            kind="elevator",
            floor_index=1,
            x=1.0,
            y=2.6,
            yaw=math.pi,
            width=1.4,
        )
        manager.current_frontier = Frontier(
            cells=((1, 1),),
            centroid=(3.05, 2.05),
            information_gain=10.0,
            score=13.48,
        )
        manager._exploration.last_frontier_target = (3.05, 2.05)
        manager._exploration.last_frontier_score = 13.48
        manager._exploration.last_frontier_used_fallback = True
        manager._frontier_arrival_gain.begin(
            floor=0,
            map_epoch=1,
            target=(3.05, 2.05),
            known_cells=100,
            expected_gain=1708.0,
            is_real_frontier=True,
        )
        task = TakeElevator(0, 1)
        task.tick = lambda _context, _pose: DONE
        manager.current_task = task
        manager.navigation_task_context = lambda: object()
        clear_reasons = []
        manager.controller = SimpleNamespace(
            stop=lambda **_kwargs: None,
            clear_path=lambda reason=None: clear_reasons.append(reason),
        )
        manager._perception_runtime.sensor_buffer = SimpleNamespace(
            snapshot=lambda: SimpleNamespace()
        )
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=100.0,
                xyz=(0.87, 2.70, 0.3),
                quaternion_xyzw=(0.0, 0.0, 1.0, 0.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._publish_status = lambda: None
        manager._elapsed = lambda: 1.0
        manager._apply_localization_progress_decision = lambda _sample: True
        manager._update_pose_progress = lambda _pose: None
        manager._record_initial_entry_trace_pose = lambda _pose: None
        manager._record_floor_navigation_trace_pose = lambda _pose: None
        manager._sync_ground_support_task_context = lambda _pose: None
        manager._ground_support.observe_odometry = lambda *_args: None
        manager._update_mapping_and_detection = lambda *_args: None
        manager._handle_ground_support_control = lambda _pose: False
        manager._transition_state = lambda *_args, **_kwargs: None

        with patch.object(mission_manager_module, "get_time", return_value=100.0):
            manager.tick()

        self.assertEqual(manager.current_floor, 1)
        self.assertEqual(manager._door_navigation.last_door_id, "lift_door_1")
        self.assertEqual(manager._door_navigation.last_door_xy, (1.0, 2.6))
        self.assertAlmostEqual(manager._door_navigation.last_pass_vector[0], -1.0)
        self.assertAlmostEqual(manager._door_navigation.last_pass_vector[1], 0.0)
        self.assertEqual(manager._door_navigation.completion_pose, (0.87, 2.7, math.pi))
        self.assertEqual(manager._door_navigation.transition_until, 101.5)
        self.assertEqual(manager._door_navigation.context_until, 108.0)
        self.assertFalse(manager._door_navigation.opening_sensor_confirmed)
        self.assertIsNone(manager.current_frontier)
        self.assertIsNone(manager._exploration.last_frontier_target)
        self.assertIsNone(manager._exploration.last_frontier_score)
        self.assertFalse(manager._exploration.last_frontier_used_fallback)
        self.assertIsNone(manager._frontier_arrival_gain._context)
        self.assertIsNone(manager._frontier_arrival_gain._active)
        self.assertEqual(manager._frontier_arrival_gain._regions, [])
        self.assertEqual(clear_reasons, [None])

        with patch.object(mission_manager_module, "get_time", return_value=100.5):
            context = manager._frontier_selection_context((0.87, 2.70, math.pi))
        self.assertIsNotNone(context)
        assert context is not None
        self.assertEqual(context.door_id, "lift_door_1")
        cabin_adjustment, cabin_filtered, cabin_penalty = (
            manager.explorer._selection_context_effect((1.5, 2.6), context)
        )
        lobby_adjustment, lobby_filtered, lobby_penalty = (
            manager.explorer._selection_context_effect((0.0, 2.6), context)
        )
        self.assertTrue(cabin_filtered)
        self.assertEqual(cabin_adjustment, 0.0)
        self.assertIsNone(cabin_penalty)
        self.assertFalse(lobby_filtered)
        self.assertGreater(lobby_adjustment, 0.0)
        self.assertEqual(lobby_penalty, 0.0)

        with patch.object(mission_manager_module, "get_time", return_value=108.1):
            expired_temp_context = manager._frontier_selection_context((0.87, 2.70, math.pi))
        self.assertIsNotNone(expired_temp_context)
        assert expired_temp_context is not None
        # 临时抑制与前向加分虽已过期，但高层电梯门作为该楼层探索期的永久禁区，轿厢深处依然被过滤
        _, cabin_still_filtered, _ = (
            manager.explorer._selection_context_effect((1.5, 2.6), expired_temp_context)
        )
        self.assertTrue(cabin_still_filtered)

    def test_elevator_exit_context_does_not_disable_unknown_support_hold(self):
        values = []
        manager = _make_test_manager()
        manager.ground_support_params = {"unknown_support_hold_enabled": True}
        manager._ground_support.gate = SimpleNamespace(
            set_unknown_support_hold_enabled=values.append
        )
        manager.current_task = None
        manager._door_navigation.record_passage(
            door_id="lift_door_1",
            door_xy=(1.0, 2.6),
            pass_vector=(-1.0, 0.0),
            pose=(0.87, 2.70, math.pi),
            now=100.0,
            transition_duration=1.5,
            context_duration=8.0,
        )
        manager._post_door_context_satisfied = lambda _pose: False

        with patch.object(mission_manager_module, "get_time", return_value=100.5):
            manager._sync_ground_support_task_context((0.87, 2.70, math.pi))

        self.assertEqual(values, [True])

    def test_max_runtime_does_not_abort_active_return_home_closure(self):
        """720 秒只结束探索预算，已启动的返航收尾必须允许继续。"""
        manager = _make_test_manager()
        manager.state = RETURN_HOME
        manager.max_runtime = 720.0
        manager._return_home_started = True
        manager._timeout_triggered = False
        manager._mission_success_eligible = True
        manager.current_task = SimpleNamespace(
            name="OpenDoorAndPass(main_entrance)",
            phase="GO_PASS_POINT",
        )
        manager._publish_status = lambda: None
        manager._elapsed = lambda: 621.2
        manager._perception_runtime.sensor_buffer = SimpleNamespace(
            snapshot=lambda: SimpleNamespace()
        )
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=621.2,
                xyz=(0.0, -0.7, 0.3),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._apply_localization_progress_decision = lambda _sample: True
        manager._update_pose_progress = lambda _pose: None
        manager._record_initial_entry_trace_pose = lambda _pose: None
        manager._record_floor_navigation_trace_pose = lambda _pose: None
        manager._sync_ground_support_task_context = lambda _pose: None
        manager._ground_support.observe_odometry = lambda *_args: None
        manager._update_mapping_and_detection = lambda *_args: None
        manager._handle_ground_support_control = lambda _pose: False
        task_ticks = []
        manager._tick_current_task_status = lambda pose: (
            task_ticks.append(pose) or RUNNING
        )
        transitions = []
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs.get("reason_code"))
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(logwarn=lambda *_args, **_kwargs: None),
        ):
            manager.tick()

        self.assertEqual(task_ticks, [(0.0, -0.7, 0.0)])
        self.assertEqual(transitions, [])
        self.assertFalse(manager._timeout_triggered)
        self.assertTrue(manager._mission_success_eligible)

    def test_starting_floor_change_marks_attempt_as_not_yet_successful(self):
        manager = _make_test_manager()
        manager._elevator_success = None
        manager.current_floor = 0
        manager.floor_params = {"elevator_id": "elevator_main"}
        manager._active_elevator_id = "elevator_main"
        manager.building_client = SimpleNamespace(services_ready=lambda: True)
        manager.controller = SimpleNamespace(clear_path=lambda: None)
        manager._transition_state = lambda *_args, **_kwargs: None

        manager._begin_floor_change(
            1,
            return_after=False,
        )

        self.assertFalse(manager._elevator_success)

    def test_runtime_budget_uses_public_floor_count_table(self):
        table = [[1, 300.0], [2, 620.0], [3, 840.0]]
        self.assertEqual(
            effective_max_runtime(
                840.0,
                max_floor=1,
                runtime_by_floor=table,
            ),
            300.0,
        )
        self.assertEqual(
            effective_max_runtime(
                840.0,
                max_floor=2,
                runtime_by_floor=table,
            ),
            620.0,
        )
        self.assertEqual(
            effective_max_runtime(
                840.0,
                max_floor=3,
                runtime_by_floor=table,
            ),
            840.0,
        )
        self.assertEqual(
            effective_max_runtime(
                120.0,
                max_floor=2,
                runtime_by_floor=table,
            ),
            120.0,
        )
        self.assertEqual(
            effective_max_runtime(
                900.0,
                max_floor=4,
                runtime_by_floor=table,
            ),
            900.0,
        )

    def test_summary_output_accepts_not_yet_created_run_directory(self):
        """先验证路径再创建目录，不能要求 run-id 目录已存在。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            public_result = root / "results" / "detected_danger.json"
            public_result.parent.mkdir()
            summary = (
                public_result.parent
                / "runs"
                / "new_run"
                / "mission_runtime_summary.json"
            )

            validated = validate_summary_output_path(
                summary,
                public_result,
            )

            self.assertEqual(validated, summary)
            self.assertFalse(summary.parent.exists())

    def test_summary_output_rejects_symlinked_parent_outside_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = root / "results" / "runs"
            runs.mkdir(parents=True)
            outside = root / "outside"
            outside.mkdir()
            linked_run = runs / "linked_run"
            try:
                linked_run.symlink_to(outside, target_is_directory=True)
            except OSError:
                self.skipTest("当前环境无法创建目录符号链接")
            summary = linked_run / "mission_runtime_summary.json"

            with self.assertRaises(ValueError):
                validate_summary_output_path(
                    summary,
                    root / "results" / "detected_danger.json",
                )

    def test_only_sensor_confirmed_threshold_crossing_disables_unknown_support_hold(self):
        task = TakeElevator(0, 1)
        for phase in ("ENTER_CABIN", "EXIT_CABIN"):
            task.phase = phase
            self.assertFalse(
                mission_manager_module._ground_support_unknown_hold_enabled(task)
            )

        for phase in ("WAIT_TARGET_DOOR_OPEN", "CLOSE_TARGET_FLOOR_DOOR"):
            task.phase = phase
            self.assertTrue(
                mission_manager_module._ground_support_unknown_hold_enabled(task)
            )

        door_task = OpenDoorAndPass("main_entrance")
        door_task.phase = "GO_PASS_POINT"
        self.assertTrue(
            mission_manager_module._ground_support_unknown_hold_enabled(door_task)
        )

        self.assertFalse(
            mission_manager_module._ground_support_unknown_hold_enabled(
                None,
                trusted_return_trace=True,
            )
        )
        door_task.mark_opening_sensor_confirmed()
        self.assertFalse(
            mission_manager_module._ground_support_unknown_hold_enabled(door_task)
        )
        door_task.phase = "GO_WAIT_POINT"
        self.assertTrue(
            mission_manager_module._ground_support_unknown_hold_enabled(door_task)
        )

    def test_unknown_support_hold_can_be_disabled_without_task_special_cases(self):
        """显式 DROP-only 回归模式应允许全局关闭 UNKNOWN 临时停车。"""
        self.assertFalse(
            mission_manager_module._ground_support_unknown_hold_enabled(
                None,
                configured=False,
            )
        )
        self.assertFalse(
            mission_manager_module._ground_support_unknown_hold_enabled(
                OpenDoorAndPass("main_entrance"),
                configured=False,
            )
        )

    def test_sync_ground_support_context_combines_config_and_task_phase(self):
        values = []
        manager = _make_test_manager()
        manager.ground_support_params = {"unknown_support_hold_enabled": False}
        manager._ground_support.gate = SimpleNamespace(
            set_unknown_support_hold_enabled=values.append
        )
        manager.current_task = None

        manager._sync_ground_support_task_context()

        self.assertEqual(values, [False])

    def test_sync_ground_support_context_keeps_confirmed_post_door_unknown_window(self):
        values = []
        manager = _make_test_manager()
        manager.ground_support_params = {"unknown_support_hold_enabled": True}
        manager._ground_support.gate = SimpleNamespace(
            set_unknown_support_hold_enabled=values.append
        )
        manager.current_task = None
        manager._door_navigation.opening_sensor_confirmed = True
        manager._door_navigation.context_until = 20.0
        manager._post_door_context_satisfied = lambda pose: False

        with patch("danger_search.mission_manager.get_time", return_value=10.0):
            manager._sync_ground_support_task_context((0.0, 0.0, 0.0))

        self.assertEqual(values, [False])

    def test_sync_ground_support_context_restores_unknown_after_post_door_window(self):
        values = []
        manager = _make_test_manager()
        manager.ground_support_params = {"unknown_support_hold_enabled": True}
        manager._ground_support.gate = SimpleNamespace(
            set_unknown_support_hold_enabled=values.append
        )
        manager.current_task = None
        manager._door_navigation.opening_sensor_confirmed = True
        manager._door_navigation.context_until = 9.0
        manager._post_door_context_satisfied = lambda pose: False

        with patch("danger_search.mission_manager.get_time", return_value=10.0):
            manager._sync_ground_support_task_context((0.0, 0.0, 0.0))

        self.assertEqual(values, [True])

    def test_sync_ground_support_context_limits_return_trace_trust_to_scope(self):
        values = []
        diagnostics = []
        manager = _make_test_manager()
        manager.state = RETURN_HOME
        manager.current_floor = 0
        manager.current_task = None
        manager._return_home_main_entrance_crossed = True
        manager.ground_support_params = {"unknown_support_hold_enabled": True}
        manager._ground_support.gate = SimpleNamespace(
            set_unknown_support_hold_enabled=values.append
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: True,
            snapshot=lambda: {"replan_reason": "return_home_entry_trace_path"},
        )
        manager._log_structured = diagnostics.append
        manager._return_home.activate_entry_trace(
            ((0.0, -1.0), (0.0, -3.2)),
            floor=0,
            map_epoch=1,
            localization_epoch=1,
            maximum_deviation=0.35,
        )

        manager._sync_ground_support_task_context((0.2, -2.0, 0.0))
        manager._sync_ground_support_task_context((0.4, -2.0, 0.0))

        self.assertEqual(values, [False, True])
        self.assertIsNone(manager._return_home.entry_trace_scope)
        self.assertEqual(
            diagnostics[-1]["reason_code"],
            "entry_trace_deviation_exceeded",
        )

    def test_ground_support_data_age_does_not_double_count_nominal_decision_cycle(self):
        self.assertAlmostEqual(
            ground_support_coordinator_module._ground_support_data_age(
                cloud_stamp=10.0,
                odom_stamp=10.002,
                control_time=10.10,
                decision_period=0.10,
            ),
            0.002,
        )
        self.assertAlmostEqual(
            ground_support_coordinator_module._ground_support_data_age(
                cloud_stamp=10.0,
                odom_stamp=10.002,
                control_time=10.14,
                decision_period=0.10,
            ),
            0.04,
        )

    def test_pose_provider_uses_nearest_internal_localization_sample(self):
        from danger_search.pose_provider import POSE_EXPIRED, POSE_MATCHED, POSE_PENDING, PoseProvider

        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        for stamp in (1.0, 1.02, 1.04):
            provider._on_odometry(self._odom_message(stamp))

        matched = provider.at(1.021)
        latest = provider.latest()

        self.assertAlmostEqual(
            matched.stamp,
            1.02,
        )
        self.assertAlmostEqual(
            latest.stamp,
            1.04,
        )

        pending = provider.lookup(1.20)
        self.assertEqual(pending.status, POSE_PENDING)
        provider._on_odometry(self._odom_message(1.30))
        expired = provider.lookup(1.20)
        self.assertEqual(expired.status, POSE_EXPIRED)
        self.assertIsNone(expired.sample)
        self.assertEqual(provider.lookup(1.021).status, POSE_MATCHED)

    def test_pose_provider_estimates_nominal_period_from_recent_samples(self):
        from danger_search.pose_provider import PoseProvider

        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        for stamp in (1.0, 1.1, 1.2, 1.3):
            provider._on_odometry(self._odom_message(stamp))

        self.assertAlmostEqual(provider.observed_period(default=0.1), 0.1)

    def test_localization_stale_stops_and_then_fails_without_advancing_task(self):
        manager = _make_test_manager()
        manager.localization_stale_minimum = 0.4
        manager.localization_unavailable_timeout = 3.0
        manager._localization_unavailable_since = None
        manager._localization_stale_reported = False
        manager._localization_failure_reported = False
        manager._mission_success_eligible = True
        manager.state = EXPLORE_FLOOR
        manager.pose_provider = SimpleNamespace(observed_period=lambda **_kwargs: 0.1)
        stopped = []
        reset = []
        transitions = []
        failures = []
        diagnostics = []
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
            reset_progress_timer=lambda pose=None: reset.append(pose),
            poll_status=lambda: 'RUNNING',
        )
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))
        manager._record_failure = failures.append
        manager._log_structured = diagnostics.append
        sample = mission_manager_module.PoseSample(
            stamp=10.0,
            xyz=(1.0, 2.0, 0.39),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )

        self.assertFalse(manager._apply_localization_progress_decision(sample, now=10.5))
        self.assertEqual(len(stopped), 1)
        self.assertEqual(reset, [sample.xy_yaw])
        self.assertEqual(transitions, [])

        self.assertFalse(manager._apply_localization_progress_decision(sample, now=13.6))
        self.assertEqual(failures, ["定位数据持续不可用"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_unavailable",
        )
        self.assertFalse(manager._mission_success_eligible)

    def test_localization_recovers_before_failure_and_clears_hold(self):
        manager = _make_test_manager()
        manager.localization_stale_minimum = 0.4
        manager.localization_unavailable_timeout = 3.0
        manager._localization_unavailable_since = 10.5
        manager._localization_stale_reported = True
        manager._localization_failure_reported = False
        manager.pose_provider = SimpleNamespace(observed_period=lambda **_kwargs: 0.1)
        manager._localization_health = True
        diagnostics = []
        manager.controller = SimpleNamespace(
            stop=lambda **_kwargs: self.fail("新鲜定位不应停车"),
            reset_progress_timer=lambda _pose=None: None,
            poll_status=lambda: 'RUNNING',
        )
        manager._log_structured = diagnostics.append
        sample = mission_manager_module.PoseSample(
            stamp=10.8,
            xyz=(1.0, 2.0, 0.39),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )

        self.assertTrue(manager._apply_localization_progress_decision(sample, now=10.9))
        self.assertIsNone(manager._localization_unavailable_since)
        self.assertFalse(manager._localization_stale_reported)
        self.assertEqual(diagnostics[-1]["event"], "localization_recovered")

    def test_stance_health_false_fails_immediately(self):
        manager = _make_test_manager()
        manager.localization_stance_health_required = True
        manager._localization_health = False
        manager._mission_success_eligible = True
        manager.state = EXPLORE_FLOOR
        stopped = []
        transitions = []
        failures = []
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
            poll_status=lambda: 'RUNNING',
        )
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))
        manager._record_failure = failures.append
        self.assertFalse(
            manager._apply_localization_progress_decision(None, now=1.0)
        )
        self.assertEqual(stopped, [{"reason": "localization_stance_unhealthy"}])
        self.assertEqual(failures, ["定位姿态健康异常"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_stance_unhealthy",
        )

    def test_tick_prioritizes_explicit_stance_failure_over_stale_pose(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.max_runtime = 100.0
        manager._localization_health = False
        manager.localization_stance_health_required = True
        manager._mission_success_eligible = True
        manager._publish_status = lambda: None
        manager._elapsed = lambda: 1.0
        manager._perception_runtime.sensor_buffer = SimpleNamespace(snapshot=lambda: SimpleNamespace())
        manager.pose_provider = SimpleNamespace(latest=lambda: None)
        stopped = []
        failures = []
        transitions = []
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
            poll_status=lambda: 'RUNNING',
        )
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )

        manager.tick()

        self.assertEqual(stopped, [{"reason": "localization_stance_unhealthy"}])
        self.assertEqual(failures, ["定位姿态健康异常"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_stance_unhealthy",
        )

    def test_unknown_stance_health_stops_then_fails_after_timeout(self):
        manager = _make_test_manager()
        manager.localization_stance_health_required = True
        manager.localization_unavailable_timeout = 3.0
        manager.start_time = 0.0
        manager._localization_runtime = None
        manager._localization_health = None
        manager._mission_success_eligible = True
        manager.state = EXPLORE_FLOOR
        clock = [1.0]
        stopped = []
        transitions = []
        failures = []
        manager._elapsed = lambda: clock[0]
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stopped.append(kwargs),
            poll_status=lambda: 'RUNNING',
        )
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )
        manager._record_failure = failures.append
        sample_at = lambda stamp: mission_manager_module.PoseSample(
            stamp=stamp,
            xyz=(0.0, 0.0, 0.39),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )

        self.assertFalse(
            manager._apply_localization_progress_decision(sample_at(1.0), now=1.0)
        )
        self.assertEqual(
            stopped,
            [{"reason": "localization_stance_health_pending"}],
        )
        self.assertEqual(transitions, [])

        clock[0] = 3.1
        self.assertFalse(
            manager._apply_localization_progress_decision(sample_at(3.1), now=3.1)
        )
        self.assertEqual(failures, ["定位姿态健康状态持续不可用"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_stance_health_unavailable",
        )

    def test_stance_health_required_false_allows_unknown(self):
        manager = _make_test_manager()
        manager.localization_stance_health_required = False
        manager._localization_health = None

        manager.pose_provider = SimpleNamespace(
            observed_period=lambda **_kwargs: 0.1,
        )
        self.assertTrue(
            manager._apply_localization_progress_decision(
                mission_manager_module.PoseSample(
                    stamp=1.0,
                    xyz=(0.0, 0.0, 0.39),
                    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                    linear_xyz=(0.0, 0.0, 0.0),
                    angular_xyz=(0.0, 0.0, 0.0),
                ),
                now=1.0,
            )
        )

    def test_stance_health_true_allows_progress(self):
        manager = _make_test_manager()
        manager.localization_stance_health_required = True
        manager._localization_health = True

        manager.pose_provider = SimpleNamespace(
            observed_period=lambda **_kwargs: 0.1,
        )
        self.assertTrue(
            manager._apply_localization_progress_decision(
                mission_manager_module.PoseSample(
                    stamp=1.0,
                    xyz=(0.0, 0.0, 0.39),
                    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                    linear_xyz=(0.0, 0.0, 0.0),
                    angular_xyz=(0.0, 0.0, 0.0),
                ),
                now=1.0,
            )
        )

    def test_stop_distance_table_parser_preserves_replaceable_breakpoints(self):
        self.assertEqual(
            parse_stop_distance_table(
                [[0.30, 0.06], [0.60, 0.09337727739487302]]
            ),
            ((0.30, 0.06), (0.60, 0.09337727739487302)),
        )

    def test_optional_exit_cap_uses_zero_as_ros_safe_disabled_value(self):
        self.assertIsNone(
            optional_positive_float(
                0.0,
                "controller/exit_first_max_linear",
            )
        )
        self.assertEqual(
            optional_positive_float(
                0.25,
                "controller/exit_first_max_linear",
            ),
            0.25,
        )
        with self.assertRaises(ValueError):
            optional_positive_float(
                -0.1,
                "controller/exit_first_max_linear",
            )

    def test_disabled_ground_support_path_does_not_query_tf_or_change_control(self):
        manager = _make_test_manager()
        manager._ground_support.gate = None
        manager._ground_support.grid = None
        manager._perception_runtime.sensor_buffer = SimpleNamespace(
            frame_transform_matrix=lambda *args, **kwargs: self.fail("关闭时不得查询 TF")
        )

        manager._ground_support.assess_frame(
            controller=manager.controller,
            pose_provider=manager.pose_provider,
            floor_index=manager.current_floor,
            pose=(0.0, 0.0, 0.0),
            pose_sample=mission_manager_module.PoseSample(
                stamp=1.0,
                xyz=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            ),
            points=[],
            cloud_stamp=1.0,
        )

    def test_ground_support_assessment_survives_command_change_during_cloud_processing(self):
        assessments = []
        evaluated_commands = []
        processed_commands = []
        commands = iter(((0.10, 0.0, 0.0), (0.20, 0.0, 0.0)))
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._ground_support.map_epoch = 4
        manager._ground_support.localization_epoch = 7
        manager.ground_support_params = {
            "base_to_livox_xyz": (0.2, 0.0, 0.08),
            "base_to_livox_rpy": (0.0, 0.785, 0.0),
            "decision_period": 0.10,
        }
        manager._ground_support.params = manager.ground_support_params
        manager.pose_provider = SimpleNamespace(
            sensor_to_world_matrix=lambda *_args, **_kwargs: np.eye(4)
        )
        manager.controller = SimpleNamespace(
            followed_path_snapshot=lambda: [(0.0, 0.0), (1.0, 0.0)],
            active_motion_command_snapshot=lambda: next(commands),
            committed_command_snapshot=lambda: (0.0, 0.0, 0.0),
            command_transaction=lambda: mission_manager_module.nullcontext(),
            poll_status=lambda: 'RUNNING',
        )
        manager._ground_support.grid = SimpleNamespace(
            process_frame=lambda **kwargs: (
                processed_commands.append(
                    (kwargs["command_linear"], kwargs["drop_command_linear"])
                )
                or {
                "action": "update",
                "support_lookahead_distance": 0.60,
                "confirmed_negative_cells": [],
                "confirmed_supported_cells": [],
                "local_corridor_assessment": {
                    "supported_distance": 0.8,
                    "support_boundary_detected": False,
                    "current_footprint_supported": True,
                    "observation_adequate": True,
                    "drop_confirmed": False,
                    "drop_intersects_sweep": False,
                    "drop_distance": None,
                    "drop_direction_world": None,
                    "support_boundary_cells_world": [],
                    "boundary_observable": True,
                    "boundary_reason_counts": {},
                },
            }
            )
        )
        manager._ground_support.gate = SimpleNamespace(
            support_sweep_context_snapshot=lambda _fallback: (
                (0.30, 0.0, 0.0),
                9,
            ),
            update_assessment=assessments.append,
            evaluate_latest_drop=lambda command, **_kwargs: (
                evaluated_commands.append(command) or False
            ),
        )
        pose_sample = mission_manager_module.PoseSample(
            stamp=10.002,
            xyz=(0.0, 0.0, 0.39),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )

        with patch(
            "danger_search.ground_support_coordinator.get_time",
            return_value=10.05,
        ):
            manager._ground_support.assess_frame(
                controller=manager.controller,
                pose_provider=manager.pose_provider,
                floor_index=manager.current_floor,
                pose=(0.0, 0.0, 0.0),
                pose_sample=pose_sample,
                points=[(0.5, 0.0, 0.0)],
                cloud_stamp=10.0,
            )

        self.assertEqual(len(assessments), 1)
        self.assertEqual(processed_commands, [(0.30, 0.10)])
        self.assertEqual(evaluated_commands, [(0.20, 0.0, 0.0)])
        self.assertEqual(assessments[0].hold_context_id, 9)
        self.assertEqual(assessments[0].support_command, (0.30, 0.0, 0.0))
        self.assertEqual(assessments[0].support_lookahead_distance, 0.60)

    def test_ground_support_timer_consumes_each_new_cloud_once(self):
        updates = []
        reads = []
        cloud = self._stamped_message(1.0)
        odom = self._odom_message(1.01)
        snapshot = SimpleNamespace(odom_msg=odom, cloud_msg=cloud, image_bundle=None)
        manager = _make_test_manager()
        manager._ground_support.gate = object()
        manager._ground_support.grid = object()
        manager._last_ground_support_cloud_stamp = None
        manager._floor_transition_runtime._guard = None
        manager.current_floor = 0
        manager.current_task = None
        manager._floor_coordinates = FloorCoordinateModel("absolute", 2.6)
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager._map_runtime.mapper = SimpleNamespace(estimate_floor_from_z=lambda _z: 0)
        manager._perception_runtime.sensor_buffer = SimpleNamespace(
            snapshot=lambda: snapshot,
            read_cloud_xyz=lambda *args, **kwargs: reads.append(True) or [(0.0, 0.0, 0.0)],
        )
        manager.pose_provider = SimpleNamespace(
            at=lambda _stamp: mission_manager_module.PoseSample(
                stamp=1.01,
                xyz=(0.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._ground_support.assess_frame = (
            lambda **kwargs: updates.append(kwargs["cloud_stamp"])
        )

        manager.tick_ground_support()
        manager.tick_ground_support()

        self.assertEqual(reads, [True])
        self.assertEqual(updates, [1.0])

    def test_negative_obstacle_ends_without_ordinary_stuck_or_recover(self):
        transitions = []
        failures = []
        controller_calls = []
        manager = _make_test_manager()
        manager._ground_support.gate = SimpleNamespace(
            state=SAFETY_BLOCKED,
            reason_code="negative_obstacle",
            check_wall_timeout=lambda _now: None,
            snapshot=lambda: {"state": SAFETY_BLOCKED},
        )
        manager._ground_support.set_gate_state_for_test(SAFETY_BLOCKED)
        manager.ground_support_params = {"negative_obstacle_replan_enabled": False}
        manager._ground_support.params = manager.ground_support_params
        manager.state = EXPLORE_FLOOR
        manager._mission_success_eligible = True
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: controller_calls.append(("reset", pose)),
            stop=lambda **kwargs: controller_calls.append(("stop", kwargs)),
            poll_status=lambda: 'RUNNING',
        )
        manager._record_failure = lambda reason: failures.append(reason)
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))

        handled = manager._handle_ground_support_control((1.0, 2.0, 0.0))

        self.assertTrue(handled)
        self.assertEqual(failures, ["negative_obstacle"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(transitions[0][1]["reason_code"], "negative_obstacle")
        self.assertFalse(manager._mission_success_eligible)
        self.assertNotIn("recover", [item[0] for item in controller_calls])

    def test_negative_obstacle_persists_source_cluster_without_recover(self):
        failures = []
        transitions = []
        logs = []
        controller_calls = []
        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.10),),
                stop_confirmation_samples=1,
            )
        )
        gate.update_assessment(
            GroundSupportAssessment(
                stamp=10.0,
                received_wall_time=100.0,
                data_age=0.01,
                supported_distance=0.5,
                current_footprint_supported=True,
                observation_adequate=True,
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.7,
                drop_direction_world=(1.0, 0.0),
                received_control_time=100.0,
                drop_point_world=(1.0, 0.0),
                floor=0,
                map_epoch=1,
                localization_epoch=1,
                cloud_generation=1,
                grid_origin=(-2.0, -2.0),
                grid_resolution=0.1,
                drop_clusters=(
                    DropClusterEvidence(
                        cluster_id="drop:test:1",
                        floor=0,
                        frame_id="odom",
                        map_epoch=1,
                        localization_epoch=1,
                        stamp=10.0,
                        generation=1,
                        grid_origin=(-2.0, -2.0),
                        grid_resolution=0.1,
                        source_cells=((30, 20),),
                    ),
                ),
            )
        )
        filtered = gate.filter_command(
            (0.3, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
            control_time=100.0,
        )
        self.assertEqual(filtered, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, DROP_STOPPING)
        self.assertEqual(gate.reason_code, "negative_obstacle")
        self.assertEqual(gate.snapshot()["true_drop_stop_count"], 1)
        self.assertEqual(
            gate.filter_command(
                (0.3, 0.0, 0.4),
                pose_yaw=0.0,
                wall_time=100.05,
                control_time=100.05,
            ),
            (0.0, 0.0, 0.0),
        )
        gate.observe_odometry(
            1.0,
            linear_speed=0.0,
            angular_speed=0.0,
            wall_time=100.1,
        )
        self.assertEqual(gate.state, SAFETY_BLOCKED)

        manager = _make_test_manager()
        manager._ground_support.gate = gate
        manager._ground_support.set_gate_state_for_test(SAFETY_BLOCKED)
        manager.ground_support_params = {
            "negative_obstacle_replan_enabled": True,
            "maximum_negative_obstacle_replans": 2,
            "sweep_half_width": 0.4,
        }
        manager._ground_support.params = manager.ground_support_params
        manager.map_params = {"resolution": 0.1}
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager._ground_support.grid = SimpleNamespace(
            config=SimpleNamespace(
                resolution=0.1,
                origin_x=-2.0,
                origin_y=-2.0,
            )
        )
        manager.current_floor = 0
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.current_frontier = SimpleNamespace(centroid=(2.0, 0.0))
        manager._exploration.last_frontier_target = (2.0, 0.0)
        manager._exploration.last_frontier_score = 1.0
        manager._exploration.last_frontier_used_fallback = False
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: controller_calls.append(("reset", pose)),
            stop=lambda **kwargs: controller_calls.append(("stop", kwargs)),
            clear_path=lambda **kwargs: controller_calls.append(("clear", kwargs)),
            poll_status=lambda: 'RUNNING',
        )
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))
        manager._elapsed = lambda: 12.0
        manager._log_structured = logs.append

        handled = manager._handle_ground_support_control((0.0, 0.0, 0.0))

        drop_cell = manager.mapper.floor(0).world_to_grid(1.0, 0.0)
        self.assertTrue(handled)
        self.assertEqual(manager.mapper.floor(0).data[drop_cell[1], drop_cell[0]], NEGATIVE)
        manager.mapper.update_from_cloud(
            floor_index=0,
            robot_xyz=(1.0, 0.0, 0.6),
            points=[],
        )
        self.assertEqual(manager.mapper.floor(0).data[drop_cell[1], drop_cell[0]], NEGATIVE)
        self.assertEqual(gate.state, GROUND_SUPPORT_CLEAR)
        self.assertIsNone(manager.current_frontier)
        self.assertIsNone(manager._exploration.last_frontier_target)
        self.assertEqual(failures, [])
        self.assertEqual(transitions, [])
        self.assertFalse(any(call[0] == "recover" for call in controller_calls))
        self.assertEqual(logs[-1]["event"], "negative_clusters_persisted")

    def test_negative_obstacle_without_source_cluster_fails_explicitly(self):
        failures = []
        transitions = []
        gate = SimpleNamespace(
            state=SAFETY_BLOCKED,
            reason_code="negative_obstacle",
            latched_drop_point_world=(1.0, 0.0),
            check_control_timeout=lambda *_args, **_kwargs: None,
            snapshot=lambda: {"state": SAFETY_BLOCKED},
        )
        manager = _make_test_manager()
        manager._ground_support.gate = gate
        manager._ground_support.set_gate_state_for_test(SAFETY_BLOCKED)
        manager.ground_support_params = {
            "negative_obstacle_replan_enabled": True,
            "maximum_negative_obstacle_replans": 3,
            "sweep_half_width": 0.4,
        }
        manager._ground_support.params = manager.ground_support_params
        manager.map_params = {"resolution": 0.1}
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.current_floor = 0
        manager.current_task = None
        manager.state = EXPLORE_FLOOR
        manager._mission_success_eligible = True
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda _pose: None,
            stop=lambda **_kwargs: None,
            poll_status=lambda: 'RUNNING',
        )
        manager._record_failure = failures.append
        manager._transition_state = (
            lambda state, **kwargs: transitions.append((state, kwargs))
        )

        handled = manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertTrue(handled)
        self.assertEqual(failures, ["negative_obstacle"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(transitions[0][1]["reason_code"], "negative_obstacle")

    def test_drop_during_critical_task_never_modifies_map_or_resets_gate(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.state = EXPLORE_FLOOR
        manager.current_task = OpenDoorAndPass("main_entrance")
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        cluster = DropClusterEvidence(
            cluster_id="drop:critical", floor=0, frame_id="odom",
            map_epoch=1, localization_epoch=1, stamp=1.0, generation=1,
            grid_origin=(-2.0, -2.0), grid_resolution=0.1,
            source_cells=((20, 20),),
        )
        resets = []
        manager._ground_support.gate = SimpleNamespace(
            state=SAFETY_BLOCKED, reason_code="negative_obstacle",
            latched_drop_clusters=(cluster,),
            check_control_timeout=lambda *_a, **_k: None,
            snapshot=lambda: {"state": SAFETY_BLOCKED},
            reset_negative_obstacle_after_replan=lambda ack: resets.append(ack) or True,
        )
        manager._ground_support.set_gate_state_for_test(SAFETY_BLOCKED)
        failures = []
        transitions = []
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda _pose: None,
            stop=lambda **_kwargs: None,
            clear_path=lambda **_kwargs: None,
            poll_status=lambda: 'RUNNING',
        )
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))

        manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertEqual(manager.mapper.floor(0).negative_cluster_records, {})
        self.assertEqual(resets, [])
        self.assertEqual(failures, ["negative_obstacle"])
        self.assertEqual(transitions[0][0], WRITE_RESULT)

    def test_drop_budget_counts_only_new_commits_and_allows_idempotent_ack(self):
        manager = _make_test_manager()
        coordinator = manager._ground_support
        coordinator.params = {
            "negative_obstacle_replan_enabled": True,
            "maximum_negative_obstacle_replans": 1,
        }
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        clears = []
        resets = []
        controller = SimpleNamespace(
            clear_path=lambda **kwargs: clears.append(kwargs),
            poll_status=lambda: 'RUNNING',
        )
        gate = SimpleNamespace(state=SAFETY_BLOCKED)

        def reset(ack):
            resets.append(ack)
            gate.state = GROUND_SUPPORT_CLEAR
            return True

        gate.reset_negative_obstacle_after_replan = reset
        coordinator.gate = gate

        def cluster(cluster_id, cell):
            return DropClusterEvidence(
                cluster_id=cluster_id, floor=0, frame_id="odom",
                map_epoch=1, localization_epoch=1, stamp=1.0,
                generation=1, grid_origin=(-2.0, -2.0),
                grid_resolution=0.1, source_cells=(cell,),
            )

        first = cluster("drop:first", (20, 20))
        gate.latched_drop_clusters = (first,)
        committed = coordinator.persist_latched_drop_clusters(
            controller=controller, mapper=mapper, floor_index=0,
            mission_state=EXPLORE_FLOOR, task_active=False,
            ordinary_states=frozenset({EXPLORE_FLOOR, RETURN_HOME}),
        )
        gate.state = SAFETY_BLOCKED
        gate.latched_drop_clusters = (cluster("drop:second", (21, 20)),)
        exhausted = coordinator.persist_latched_drop_clusters(
            controller=controller, mapper=mapper, floor_index=0,
            mission_state=EXPLORE_FLOOR, task_active=False,
            ordinary_states=frozenset({EXPLORE_FLOOR, RETURN_HOME}),
        )
        gate.state = SAFETY_BLOCKED
        gate.latched_drop_clusters = (first,)
        replay = coordinator.persist_latched_drop_clusters(
            controller=controller, mapper=mapper, floor_index=0,
            mission_state=EXPLORE_FLOOR, task_active=False,
            ordinary_states=frozenset({EXPLORE_FLOOR, RETURN_HOME}),
        )

        self.assertIsNotNone(committed)
        self.assertIsNone(exhausted)
        self.assertIsNotNone(replay)
        self.assertEqual(set(mapper.floor(0).negative_cluster_records), {"drop:first"})
        self.assertEqual(len(resets), 2)
        self.assertEqual(len(clears), 2)

    def test_drop_from_stale_floor_scope_cannot_commit_or_ack(self):
        manager = _make_test_manager()
        coordinator = manager._ground_support
        coordinator.params = {
            "negative_obstacle_replan_enabled": True,
            "maximum_negative_obstacle_replans": 2,
        }
        coordinator.activate_floor(1)
        mapper = MultiFloorGridMap(
            resolution=0.1, width=4.0, length=4.0,
            origin_x=-2.0, origin_y=-2.0,
        )
        resets = []
        coordinator.gate = SimpleNamespace(
            latched_drop_clusters=(DropClusterEvidence(
                cluster_id="drop:old", floor=0, frame_id="odom",
                map_epoch=1, localization_epoch=1, stamp=1.0,
                generation=1, grid_origin=(-2.0, -2.0),
                grid_resolution=0.1, source_cells=((20, 20),),
            ),),
            reset_negative_obstacle_after_replan=lambda ack: resets.append(ack) or True,
        )

        result = coordinator.persist_latched_drop_clusters(
            controller=SimpleNamespace(clear_path=lambda **_kwargs: None),
            mapper=mapper, floor_index=0, mission_state=EXPLORE_FLOOR,
            task_active=False,
            ordinary_states=frozenset({EXPLORE_FLOOR, RETURN_HOME}),
        )

        self.assertIsNone(result)
        self.assertEqual(mapper.floor(0).negative_cluster_records, {})
        self.assertEqual(resets, [])

    def test_persistent_localization_speed_anomaly_is_not_replanned(self):
        failures = []
        transitions = []
        manager = _make_test_manager()
        manager._ground_support.gate = SimpleNamespace(
            state=SUPPORT_PATH_UNAVAILABLE,
            reason_code="localization_speed_abnormal",
            check_control_timeout=lambda *_args, **_kwargs: None,
            snapshot=lambda: {
                "state": SUPPORT_PATH_UNAVAILABLE,
                "reason_code": "localization_speed_abnormal",
            },
        )
        manager._ground_support.set_gate_state_for_test(
            SUPPORT_PATH_UNAVAILABLE
        )
        manager.state = EXPLORE_FLOOR
        manager._mission_success_eligible = True
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda _pose: None,
            stop=lambda **_kwargs: None,
            poll_status=lambda: 'RUNNING',
        )
        manager._record_failure = failures.append
        manager._transition_state = (
            lambda state, **kwargs: transitions.append((state, kwargs))
        )

        handled = manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertTrue(handled)
        self.assertEqual(failures, ["localization_speed_abnormal"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "localization_speed_abnormal",
        )

    def test_wall_timeout_is_converted_to_explicit_mission_failure(self):
        failures = []
        stop_reasons = []

        class TimeoutGate:
            state = DROP_STOPPING
            reason_code = "negative_obstacle"

            def check_wall_timeout(self, _wall_time):
                self.state = SAFETY_STOP_TIMEOUT
                self.reason_code = "ground_support_safety_stop_timeout"

        manager = _make_test_manager()
        manager._ground_support.gate = TimeoutGate()
        manager._ground_support_forced_shutdown_reason = None
        manager._shutdown_requested = False
        manager._mission_success_eligible = True
        manager._record_failure = lambda reason: failures.append(reason)
        manager.controller = SimpleNamespace(
            stop=lambda **kwargs: stop_reasons.append(kwargs.get("reason")),
            poll_status=lambda: 'RUNNING',
        )

        timed_out, reason = manager.check_ground_support_wall_timeout(123.0)
        self.assertTrue(timed_out)
        manager.handle_ground_support_wall_timeout(reason)

        self.assertEqual(failures, ["ground_support_safety_stop_timeout"])
        self.assertEqual(stop_reasons, ["ground_support_safety_stop_timeout"])
        self.assertEqual(
            manager._ground_support_forced_shutdown_reason,
            "ground_support_safety_stop_timeout",
        )
        self.assertTrue(manager._shutdown_requested)
        self.assertFalse(manager._mission_success_eligible)

    def test_support_thread_updates_are_applied_on_main_thread(self):
        logs = []
        manager = _make_test_manager()
        manager.current_floor = 1
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager._ground_support.grid = SimpleNamespace(
            config=SimpleNamespace(
                resolution=0.1,
                origin_x=-2.0,
                origin_y=-2.0,
            )
        )
        manager._ground_support.activate_floor(1)
        manager._elapsed = lambda: 1.0
        manager._log_structured = logs.append
        cell = manager.mapper.floor(1).world_to_grid(0.5, 0.0)

        manager._ground_support.queue_confirmed_support(
            {"floor": 1, "confirmed_negative_cells": [list(cell)]},
            default_floor=manager.current_floor,
            cloud_stamp=1.0,
            cloud_generation=1,
        )
        self.assertNotEqual(
            manager.mapper.floor(1).data[cell[1], cell[0]], NEGATIVE
        )
        update = manager._ground_support.apply_confirmed_support(manager.mapper)
        self.assertNotEqual(manager.mapper.floor(1).data[cell[1], cell[0]], NEGATIVE)
        self.assertFalse(update.changed)

        committed = manager.mapper.persist_negative_clusters_atomic(
            (
                NegativeClusterRecord(
                    cluster_id="drop:1:1:1:1:0",
                    floor=1,
                    frame_id="odom",
                    map_epoch=1,
                    localization_epoch=1,
                    stamp=1.0,
                    generation=1,
                    grid_origin=(-2.0, -2.0),
                    grid_resolution=0.1,
                    source_cells=frozenset({cell}),
                ),
            )
        )
        self.assertTrue(committed.committed)
        self.assertEqual(manager.mapper.floor(1).data[cell[1], cell[0]], NEGATIVE)
        manager._ground_support.queue_confirmed_support(
            {"floor": 1, "confirmed_supported_cells": [list(cell)]},
            default_floor=manager.current_floor,
            cloud_stamp=2.0,
            cloud_generation=2,
        )
        update = manager._ground_support.apply_confirmed_support(manager.mapper)
        self.assertNotEqual(manager.mapper.floor(1).data[cell[1], cell[0]], NEGATIVE)
        self.assertTrue(update.changed)

    def test_stable_unknown_boundary_is_persisted_before_path_replan(self):
        logs = []
        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.10),),
            )
        )
        gate.state = SUPPORT_PATH_UNAVAILABLE
        gate.reason_code = "ground_support_path_unavailable"
        manager = _make_test_manager()
        manager._ground_support.gate = gate
        manager._ground_support.set_gate_state_for_test(
            SUPPORT_PATH_UNAVAILABLE
        )
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.current_floor = 1
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.active_door = None
        manager.current_frontier = SimpleNamespace(centroid=(1.0, 0.0))
        manager._exploration.last_frontier_target = (1.0, 0.0)
        manager._exploration.last_frontier_score = 1.0
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: None,
            poll_status=lambda: 'RUNNING',
        )
        manager._elapsed = lambda: 2.0
        manager._record_failure = lambda reason: None
        manager._log_structured = logs.append
        source_cells = tuple(
            manager.mapper.floor(1).world_to_grid(*point)
            for point in ((0.50, -0.10), (0.50, 0.00), (0.50, 0.10))
        )
        _set_support_failure_witness(
            manager, gate, sequence=1, cells=source_cells
        )

        manager._handle_ground_support_control((0.0, 0.0, 0.0))

        grid = manager.mapper.floor(1)
        for point in ((0.50, -0.10), (0.50, 0.00), (0.50, 0.10)):
            cell = grid.world_to_grid(*point)
            self.assertNotEqual(grid.data[cell[1], cell[0]], NEGATIVE)
        self.assertIsNone(manager.current_frontier)
        constraints = manager._ground_support.planning_constraints(1)
        self.assertIsNotNone(constraints)
        self.assertTrue(set(source_cells).issubset(constraints.blocked_cells))
        self.assertTrue(
            any(item["event"] == "ground_support_constraint_installed" for item in logs)
        )

    def test_distinct_confirmed_boundaries_do_not_share_frontier_retry_budget(self):
        manager = _make_test_manager()
        manager._ground_support.set_gate_state_for_test(
            SUPPORT_PATH_UNAVAILABLE
        )
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        manager.current_floor = 1
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.active_door = None
        manager._mission_success_eligible = True
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: None,
            poll_status=lambda: 'RUNNING',
        )
        failures = []
        transitions = []
        manager._record_failure = failures.append
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )
        manager._elapsed = lambda: 2.0
        manager._log_structured = lambda payload: None

        for sequence, (target, source_cell) in enumerate(
            (((1.0, 0.0), (10, 10)), ((-1.0, 0.0), (30, 30))),
            start=1,
        ):
            gate = GroundSupportSafetyGate(
                GroundSupportGateConfig(
                    enabled=True,
                    validated_max_speed=0.60,
                    stop_distance_table=((0.60, 0.10),),
                )
            )
            gate.state = SUPPORT_PATH_UNAVAILABLE
            gate.reason_code = "ground_support_path_unavailable"
            manager._ground_support.gate = gate
            manager._ground_support.set_gate_state_for_test(
                SUPPORT_PATH_UNAVAILABLE
            )
            manager.current_frontier = SimpleNamespace(centroid=target)
            manager._exploration.last_frontier_target = target
            manager._exploration.last_frontier_score = 1.0
            _set_support_failure_witness(
                manager, gate, sequence=sequence, cells=(source_cell,)
            )

            manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertEqual(failures, [])
        self.assertEqual(transitions, [])
        constraints = manager._ground_support.planning_constraints(1)
        self.assertEqual(len(constraints.support_regions), 2)
        self.assertEqual(
            sorted(
                region.failure_count
                for region in manager._ground_support.constraint_regions(1)
            ),
            [1, 1],
        )

    def test_unknown_timeout_fails_only_current_path_without_discarding_frontier(self):
        cleared = []
        failures = []
        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.09337727739487302),),
            )
        )
        gate.state = SUPPORT_PATH_UNAVAILABLE
        gate.reason_code = "ground_support_path_unavailable"
        manager = _make_test_manager()
        manager._ground_support.gate = gate
        manager._ground_support.set_gate_state_for_test(
            SUPPORT_PATH_UNAVAILABLE
        )
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.active_door = None
        manager.current_frontier = SimpleNamespace(centroid=(3.0, 4.0))
        manager._exploration.last_frontier_target = (3.0, 4.0)
        manager._exploration.last_frontier_score = 1.0
        manager._exploration.last_frontier_used_fallback = False
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: cleared.append(kwargs),
            poll_status=lambda: 'RUNNING',
        )
        manager._record_failure = lambda reason: failures.append(reason)
        manager._log_structured = lambda payload: None
        _set_support_failure_witness(
            manager, gate, sequence=1, cells=((10, 10),)
        )

        handled = manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertTrue(handled)
        self.assertEqual(cleared, [{"reason": "ground_support_path_unavailable"}])
        self.assertEqual(failures, [])
        self.assertIsNone(manager.current_frontier)
        self.assertIsNone(manager._exploration.last_frontier_target)
        self.assertEqual(gate.state, GROUND_SUPPORT_CLEAR)

    def test_unknown_timeout_preserves_open_door_parent_and_phase(self):
        cleared = []
        gate = GroundSupportSafetyGate(
            GroundSupportGateConfig(
                enabled=True,
                validated_max_speed=0.60,
                stop_distance_table=((0.60, 0.09337727739487302),),
            )
        )
        gate.state = SUPPORT_PATH_UNAVAILABLE
        gate.reason_code = "ground_support_path_unavailable"
        task = OpenDoorAndPass("main_entrance")
        task.phase = "GO_PASS_POINT"
        task._wait_point = (0.0, 0.0)
        task._pass_point = (1.0, 0.0)
        task._commit_point = (2.0, 0.0)
        manager = _make_test_manager()
        manager._ground_support.gate = gate
        manager._ground_support.set_gate_state_for_test(
            SUPPORT_PATH_UNAVAILABLE
        )
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager.state = EXPLORE_FLOOR
        manager.current_task = task
        manager.active_door = "main_entrance"
        manager.current_frontier = None
        manager._exploration.last_frontier_target = None
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: cleared.append(kwargs),
            poll_status=lambda: 'RUNNING',
        )
        manager._log_structured = lambda payload: None
        _set_support_failure_witness(
            manager, gate, sequence=1, cells=((10, 10),)
        )

        handled = manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertTrue(handled)
        self.assertIs(manager.current_task, task)
        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(manager.active_door, "main_entrance")
        self.assertEqual(task.pass_point, (1.0, 0.0))
        self.assertEqual(cleared, [{"reason": "ground_support_path_unavailable"}])

    def test_same_open_door_phase_replan_is_bounded_without_recover(self):
        transitions = []
        failures = []
        recover_calls = []
        task = OpenDoorAndPass("main_entrance")
        task.phase = "GO_PASS_POINT"
        manager = _make_test_manager()
        manager._ground_support.set_gate_state_for_test(
            SUPPORT_PATH_UNAVAILABLE
        )
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager.state = EXPLORE_FLOOR
        manager.current_task = task
        manager.active_door = "main_entrance"
        manager.current_frontier = None
        manager._exploration.last_frontier_target = None
        manager._mission_success_eligible = True
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda pose: None,
            stop=lambda **kwargs: None,
            clear_path=lambda **kwargs: None,
            recover=lambda: recover_calls.append(True),
            poll_status=lambda: 'RUNNING',
        )
        manager._record_failure = lambda reason: failures.append(reason)
        manager._transition_state = lambda state, **kwargs: transitions.append((state, kwargs))
        manager._log_structured = lambda payload: None

        for sequence in range(1, 4):
            gate = GroundSupportSafetyGate(
                GroundSupportGateConfig(
                    enabled=True,
                    validated_max_speed=0.60,
                    stop_distance_table=((0.60, 0.09337727739487302),),
                )
            )
            gate.state = SUPPORT_PATH_UNAVAILABLE
            gate.reason_code = "ground_support_path_unavailable"
            manager._ground_support.gate = gate
            manager._ground_support.set_gate_state_for_test(
                SUPPORT_PATH_UNAVAILABLE
            )
            _set_support_failure_witness(
                manager, gate, sequence=sequence, cells=((10, 10),)
            )
            manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertIs(manager.current_task, task)
        self.assertEqual(task.phase, "GO_PASS_POINT")
        self.assertEqual(recover_calls, [])
        self.assertEqual(failures, [])
        self.assertEqual(transitions, [])
        region = manager._ground_support.constraint_regions(0)[0]
        self.assertEqual(region.failure_count, 3)
        self.assertEqual(
            manager._tick_current_task_status((0.0, 0.0, 0.0)), FAILED
        )
        self.assertEqual(task.failure_reason, "ground_support_constraint_exhausted")
        self.assertEqual(recover_calls, [])

    def test_return_home_support_region_exhaustion_uses_bounded_reject_budget(self):
        manager = _make_test_manager()
        manager.state = RETURN_HOME
        manager.current_floor = 0
        manager.current_task = None
        manager.return_home_max_candidate_rejects = 1
        manager.ground_support_params = {"maximum_same_phase_replans": 2}
        manager.controller = SimpleNamespace(
            reset_progress_timer=lambda _pose: None,
            stop=lambda **_kwargs: None,
            clear_path=lambda **_kwargs: None,
            poll_status=lambda: 'RUNNING',
        )
        manager._log_structured = lambda _payload: None
        failures = []
        manager._fail_return_home_stuck = lambda **kwargs: failures.append(kwargs["reason_code"])
        for sequence in range(1, 4):
            gate = GroundSupportSafetyGate(GroundSupportGateConfig(
                enabled=True, validated_max_speed=0.6,
                stop_distance_table=((0.6, 0.1),),
            ))
            gate.state = SUPPORT_PATH_UNAVAILABLE
            gate.reason_code = "ground_support_path_unavailable"
            manager._ground_support.gate = gate
            manager._ground_support.set_gate_state_for_test(SUPPORT_PATH_UNAVAILABLE)
            _set_support_failure_witness(
                manager, gate, sequence=sequence, cells=((10, 10),)
            )
            manager._handle_ground_support_control((0.0, 0.0, 0.0))

        self.assertEqual(manager._return_home.candidate_reject_count, 1)
        self.assertEqual(failures, ["return_home_stuck_exhausted"])

    def test_invalid_floor_exit_certificate_is_sticky_and_ignores_stale_debug(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._last_pose = (0.0, 0.0, 0.0)
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=1.0, width=3.0, length=3.0,
            origin_x=0.0, origin_y=0.0,
        )
        evidence = SimpleNamespace(
            constraint_revision=7,
            region_ids=("support:7",),
            formal_reachable_count=2,
            effective_reachable_count=0,
            support_blocked_count=2,
            fully_blocked=True,
        )
        manager.explorer = SimpleNamespace(
            last_selection_debug=SimpleNamespace(
                formal_reachable_count=99,
                effective_reachable_count=98,
            ),
            floor_termination_evidence=lambda *_a, **_k: evidence,
        )
        manager._planning_constraints = lambda: SimpleNamespace(
            revision=7, support_regions=()
        )
        manager._record_floor_exit_certificate("first")
        evidence.fully_blocked = False
        manager._record_floor_exit_certificate("second")

        certificate = manager._floor_exit_certificates[0]
        self.assertFalse(certificate.valid)
        self.assertEqual(certificate.reason_code, "first")
        self.assertEqual(certificate.formal_reachable_count, 2)
        self.assertEqual(certificate.effective_reachable_count, 0)

    def test_drop_cluster_cell_conversion_rejects_non_integer_without_exception(self):
        evidence = ground_support_coordinator_module._drop_cluster_evidence(
            [[[1.2, 2]]], floor=0, map_epoch=1, localization_epoch=1,
            stamp=1.0, generation=1, grid_origin=(0.0, 0.0),
            grid_resolution=0.1,
        )
        self.assertEqual(evidence, ())

    def test_floor_transition_invalidates_old_floor_support_assessment(self):
        manager, _, _, reads, map_updates, _ = self._make_floor_transition_manager()
        invalidations = []
        manager._ground_support.gate = SimpleNamespace(
            invalidate_assessment=lambda: invalidations.append(True)
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.6),
        )

        self.assertEqual(invalidations, [True])
        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])

    @staticmethod
    def _stamped_message(stamp: float):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda value=stamp: value),
            )
        )

    @staticmethod
    def _odom_message(stamp: float):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda value=stamp: value),
            ),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.39),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                )
            ),
        )

    def _make_floor_transition_manager(self):
        clock = [0.0]
        diagnostics = []
        reads = []
        map_updates = []
        obstacle_updates = []
        manager = _make_test_manager()
        manager.start_time = 0.0
        manager.current_floor = 0
        manager.target_floor = 1
        manager.current_task = TakeElevator(0, 1)
        manager.current_task.phase = "WAIT_FOR_ODOM_STABLE"
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager._map_runtime.last_cloud_stamp = None
        manager._map_runtime.last_image_stamp = None
        manager._map_runtime.last_floor_alignment_mismatch_key = None
        manager._floor_transition_runtime._guard = None
        manager._map_runtime.latest_cloud_points = []
        manager._failure_reasons = []
        manager._mission_success_eligible = True
        manager.state = CHANGE_FLOOR
        manager.floor_params = {"service_timeout": 5.0}
        manager.depth_frame_is_optical = False
        manager._floor_coordinates = FloorCoordinateModel("absolute", 2.6)
        # 这些测试直接把与 cloud_stamp 对齐的 pose_sample 传给建图入口，
        # 不测试 PoseProvider 的异步等待语义。
        manager.pose_provider = None
        manager._localization_runtime = None
        manager._map_runtime.mapper = SimpleNamespace(
            floor_height=2.6,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
            estimate_floor_from_z=lambda z: max(0, int(round(float(z) / 2.6))),
            update_from_cloud=lambda **kwargs: map_updates.append(kwargs),
        )
        manager.controller = SimpleNamespace(
            set_nearest_obstacle_distance=lambda distance: obstacle_updates.append(distance),
            stop=lambda **kwargs: diagnostics.append(
                {"event": "controller_stop", "reason": kwargs.get("reason")}
            ),
            poll_status=lambda: 'RUNNING',
        )
        manager._perception_runtime.sensor_buffer = SimpleNamespace(
            read_cloud_xyz=lambda *args, **kwargs: reads.append(True) or [(0.8, 0.0, 0.5)],
            camera_to_target_matrix=lambda *args, **kwargs: None,
        )
        manager._elapsed = lambda: clock[0]
        manager._log_structured = lambda payload: diagnostics.append(payload)
        manager._transition_state = lambda state, **kwargs: (
            setattr(manager, "state", state),
            diagnostics.append(
                {
                    "event": "state_transition",
                    "state": state,
                    "reason_code": kwargs.get("reason_code"),
                }
            ),
        )
        return manager, clock, diagnostics, reads, map_updates, obstacle_updates

    def _mapping_snapshot(
        self,
        *,
        cloud_stamp: float,
        odom_stamp: float,
        image_bundle=None,
    ):
        return SimpleNamespace(
            odom_msg=self._stamped_message(odom_stamp),
            cloud_msg=self._stamped_message(cloud_stamp),
            cloud_generation=max(1, int(round(cloud_stamp * 1000.0))),
            image_bundle=image_bundle,
            image_generation=(
                max(1, int(round(cloud_stamp * 1000.0)))
                if image_bundle is not None
                else 0
            ),
        )

    def test_forward_obstacle_distance_uses_raw_height_filtered_points(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )
        points = [
            (0.8, 0.1, 0.5),
            (0.3, 1.0, 0.5),
            (0.2, 0.0, 2.0),
            (-0.2, 0.0, 0.5),
        ]

        distance = nearest_forward_obstacle_distance(
            points,
            (0.0, 0.0, 0.0),
            floor_index=0,
            mapper=mapper,
        )

        self.assertAlmostEqual(distance, math.hypot(0.8, 0.1))

    def test_forward_obstacle_observation_reports_exact_nearest_echo_geometry(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )

        observation = nearest_forward_obstacle_observation(
            [(0.8, 0.1, 0.5), (0.35, -0.05, 0.22)],
            (0.0, 0.0, 0.0),
            floor_index=0,
            mapper=mapper,
        )

        self.assertIsNotNone(observation)
        self.assertEqual(observation.world_xyz, (0.35, -0.05, 0.22))
        self.assertAlmostEqual(observation.distance, math.hypot(0.35, 0.05))
        self.assertAlmostEqual(observation.forward, 0.35)
        self.assertAlmostEqual(observation.lateral, 0.05)
        self.assertAlmostEqual(observation.relative_height, 0.22)

    def test_forward_obstacle_ignores_side_doorposts_near_waist(self):
        """STAGE23 回归：机身侧面腰部/肋部（forward < 0.20m）的门框立柱绝不误判为前向阻挡。"""
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )
        # 模拟机器狗在窄门中贴边，门框立柱位于右侧腰部 (forward=0.01m, lateral=0.23m)
        points = [
            (0.01, 0.23, 0.50),   # 侧腰立柱
            (0.10, -0.23, 0.50),  # 左侧腰立柱
        ]
        observation = nearest_forward_obstacle_observation(
            points,
            (0.0, 0.0, 0.0),
            floor_index=0,
            mapper=mapper,
        )
        self.assertIsNone(observation)

    def test_forward_obstacle_distance_excludes_only_active_door_aperture(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )
        points = [
            # 已开门洞内的门槛回波，不应阻止已经对准的直线过门。
            (0.20, 0.45, 0.20),
            # 门框外缘仍位于机器人前向走廊内，必须继续作为障碍。
            (0.25, 0.58, 0.50),
        ]

        distance = nearest_forward_obstacle_distance(
            points,
            (0.0, 0.45, 0.0),
            floor_index=0,
            mapper=mapper,
            opened_door_exclusion=(0.30, 0.0, 0.0, 0.55, 0.80),
        )

        self.assertAlmostEqual(distance, math.hypot(0.25, 0.13))

    def test_forward_obstacle_lateral_limit_profile(self):
        self.assertAlmostEqual(forward_obstacle_lateral_limit(0.10), 0.16)
        self.assertAlmostEqual(forward_obstacle_lateral_limit(0.20), 0.16)
        self.assertAlmostEqual(forward_obstacle_lateral_limit(0.35), 0.16)
        self.assertAlmostEqual(forward_obstacle_lateral_limit(0.475), 0.19)
        self.assertAlmostEqual(forward_obstacle_lateral_limit(0.60), 0.22)
        self.assertAlmostEqual(forward_obstacle_lateral_limit(1.00), 0.22)

    def test_forward_obstacle_tapered_corridor_prevents_corner_freeze(self):
        """方案一测试：梯形前向走廊有效避免转角墙皮擦边误判急停，同时保持正向碰撞防护。"""
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )
        # 现场实测导致卡死的转角点 (forward~0.2001m, lateral~0.2078m) 必须被安全忽略
        corner_point = (0.2001, 0.2078, 0.50)
        obs_corner = nearest_forward_obstacle_observation(
            [corner_point],
            (0.0, 0.0, 0.0),
            floor_index=0,
            mapper=mapper,
        )
        self.assertIsNone(obs_corner)

        # 真正正前方的障碍物 (forward=0.20m, lateral=0.10m) 必须灵敏检出
        direct_obstacle = (0.20, 0.10, 0.50)
        obs_direct = nearest_forward_obstacle_observation(
            [direct_obstacle],
            (0.0, 0.0, 0.0),
            floor_index=0,
            mapper=mapper,
        )
        self.assertIsNotNone(obs_direct)
        self.assertAlmostEqual(obs_direct.distance, math.hypot(0.20, 0.10))

    def test_opened_door_exclusion_only_activates_after_pass_path_commit(self):
        manager = _make_test_manager()
        manager._map_runtime.mapper = SimpleNamespace(
            resolution=0.1,
            obstacle_inflation_radius=0.35,
        )
        manager.door_clear_radius = 0.9
        manager.door_pass_offset = 0.8
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    yaw=math.pi / 2.0,
                    width=1.3,
                )
            }
        )
        task = OpenDoorAndPass("main_entrance")
        task.phase = "GO_PASS_POINT"
        manager.current_task = task

        self.assertIsNone(manager._active_opened_door_obstacle_exclusion())

        task._pass_path_started = True
        exclusion = manager._active_opened_door_obstacle_exclusion()
        self.assertIsNotNone(exclusion)
        self.assertAlmostEqual(exclusion[3], 0.55)
        self.assertAlmostEqual(exclusion[4], 1.25)

        manager.current_task = None
        self.assertIsNone(manager._active_opened_door_obstacle_exclusion())

    def test_elevator_door_exclusion_requires_confirmed_crossing_and_stays_thin(self):
        manager = _make_test_manager()
        manager._map_runtime.mapper = SimpleNamespace(
            resolution=0.1,
            obstacle_inflation_radius=0.25,
        )
        manager.building_client = SimpleNamespace(
            doors={
                "elevator_floor_1": DoorInfo(
                    "elevator_floor_1",
                    "elevator",
                    1,
                    1.61,
                    2.31,
                    math.pi,
                    0.70,
                )
            },
            elevator_door_id=lambda floor: f"elevator_floor_{int(floor)}",
        )
        task = TakeElevator(0, 1)
        task.phase = "EXIT_CABIN"
        task.cabin_transit_stage = "CROSS"
        manager.current_task = task

        self.assertIsNone(manager._active_opened_door_obstacle_exclusion())

        task._door_open_confirmed.add("elevator_floor_1")
        exclusion = manager._active_opened_door_obstacle_exclusion()

        self.assertIsNotNone(exclusion)
        self.assertAlmostEqual(exclusion[3], 0.60)
        self.assertAlmostEqual(exclusion[4], 0.35)

    def test_elevator_door_exclusion_filters_plane_but_not_obstacle_beyond_it(self):
        mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=0.0,
            origin_y=0.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.40,
        )
        pose = (1.97, 2.28, math.pi)
        points = [
            (1.80, 2.04, 0.90),
            (1.20, 2.28, 0.90),
        ]

        observation = nearest_forward_obstacle_observation(
            points,
            pose,
            floor_index=0,
            mapper=mapper,
            opened_door_exclusion=(1.61, 2.31, math.pi, 0.60, 0.35),
        )

        self.assertIsNotNone(observation)
        self.assertAlmostEqual(observation.world_xyz[0], 1.20)
        self.assertAlmostEqual(observation.distance, 0.77)

    def test_cached_cloud_stamp_is_mapped_only_once(self):
        read_count = []
        map_count = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager._map_runtime.last_cloud_stamp = None
        manager._map_runtime.latest_cloud_points = []
        manager.pose_provider = None
        manager._localization_runtime = None
        manager._perception_runtime.sensor_buffer = SimpleNamespace(
            read_cloud_xyz=lambda *args, **kwargs: read_count.append(True) or []
        )
        manager._map_runtime.mapper = SimpleNamespace(
            floor_height=2.6,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
            estimate_floor_from_z=lambda z: max(0, int(round(float(z) / 2.6))),
            update_from_cloud=lambda **kwargs: map_count.append(kwargs),
        )
        manager.controller = SimpleNamespace(
            set_nearest_obstacle_distance=lambda distance: None,
            poll_status=lambda: 'RUNNING',
        )
        stamp = SimpleNamespace(to_sec=lambda: 12.5)
        snapshot = SimpleNamespace(
            cloud_msg=SimpleNamespace(header=SimpleNamespace(stamp=stamp)),
            image_bundle=None,
        )

        manager._update_mapping_and_detection(snapshot, (0.0, 0.0, 0.0), (0.0, 0.0, 0.6))
        manager._update_mapping_and_detection(snapshot, (0.1, 0.0, 0.0), (0.1, 0.0, 0.6))

        self.assertEqual(len(read_count), 1)
        self.assertEqual(len(map_count), 1)

    def test_cloud_mapping_uses_cloud_stamp_matched_pose_not_latest_pose(self):
        from danger_search.pose_provider import PoseProvider

        manager, _clock, _diagnostics, _reads, map_updates, _obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None
        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        provider._samples.append(
            mission_manager_module.PoseSample(
                stamp=1.0,
                xyz=(1.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        provider._samples.append(
            mission_manager_module.PoseSample(
                stamp=2.0,
                xyz=(9.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager.pose_provider = provider

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=2.0),
            (9.0, 0.0, 0.0),
            (9.0, 0.0, 0.39),
        )

        self.assertEqual(map_updates[0]["robot_xyz"], (1.0, 0.0, 0.39))

    def test_rgbd_pending_pose_is_retried_before_frame_is_consumed(self):
        from danger_search.pose_provider import PoseProvider

        manager, _clock, _diagnostics, _reads, _map_updates, _obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None
        manager.camera_mount_xyz = (0.28, 0.0, 0.043)
        manager.camera_mount_rpy = (0.0, 0.0, 0.0)
        detections = []
        manager._map_runtime.detector = SimpleNamespace(
            detect_messages=lambda *args, **kwargs: detections.append(args[3]) or [],
        )
        manager._map_runtime.tracker = SimpleNamespace(update=lambda *args, **kwargs: None)
        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        provider._on_odometry(self._odom_message(1.0))
        manager.pose_provider = provider
        rgb_msg = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 2.0),
                frame_id="camera",
            )
        )
        bundle = SimpleNamespace(
            rgb_msg=rgb_msg,
            depth_msg=rgb_msg,
            camera_info_msg=rgb_msg,
        )
        snapshot = self._mapping_snapshot(
            cloud_stamp=1.0,
            odom_stamp=1.0,
            image_bundle=bundle,
        )

        manager._update_mapping_and_detection(snapshot, (0.0, 0.0, 0.0), (0.0, 0.0, 0.39))
        self.assertIsNone(manager._map_runtime.last_image_stamp)
        self.assertEqual(detections, [])

        provider._samples.append(
            mission_manager_module.PoseSample(
                stamp=2.0,
                xyz=(1.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._update_mapping_and_detection(snapshot, (0.0, 0.0, 0.0), (0.0, 0.0, 0.39))

        self.assertEqual(manager._map_runtime.last_image_stamp, 2.0)
        self.assertEqual(len(detections), 1)

    def test_floor_transition_latch_pauses_floor_scoped_cloud_processing(self):
        manager, _clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        snapshot = self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0)

        manager._update_mapping_and_detection(
            snapshot,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])
        self.assertEqual(manager._map_runtime.last_cloud_stamp, 1.0)
        self.assertEqual(manager._floor_transition_guard.skipped_cloud_count, 1)
        self.assertEqual(
            [payload["event"] for payload in diagnostics],
            ["floor_transition_ambiguous_enter", "skipped_floor_scoped_update"],
        )

    def test_explicit_floor_transition_uses_service_and_sensor_confirmation(self):
        from danger_search.pose_provider import PoseProvider

        manager, clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager._floor_coordinates = FloorCoordinateModel("floor_local", 2.6)
        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)

        def append_pose(stamp):
            provider._samples.append(
                mission_manager_module.PoseSample(
                    stamp=stamp,
                    xyz=(2.4, 2.6, 0.39),
                    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                    linear_xyz=(0.0, 0.0, 0.0),
                    angular_xyz=(0.0, 0.0, 0.0),
                )
            )

        append_pose(1.0)
        manager.pose_provider = provider
        manager.begin_floor_transition(
            source_floor=0,
            target_floor=1,
            phase="CALL_ELEVATOR_TO_TARGET_FLOOR",
        )
        manager.confirm_floor_transition(1)

        append_pose(2.0)
        clock[0] = 1.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=2.0, odom_stamp=2.0),
            (2.4, 2.6, 0.0),
            (2.4, 2.6, 0.39),
        )
        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(reads, [True])

        append_pose(3.0)
        clock[0] = 1.1
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=3.0, odom_stamp=3.0),
            (2.4, 2.6, 0.0),
            (2.4, 2.6, 0.39),
        )

        self.assertIsNone(manager._floor_transition_guard)
        self.assertEqual(manager.current_floor, 1)
        self.assertEqual(reads, [True, True])
        self.assertEqual(len(map_updates), 1)
        self.assertEqual(map_updates[0]["floor_index"], 1)
        self.assertEqual(map_updates[0]["robot_xyz"], (2.4, 2.6, 2.99))
        self.assertEqual(map_updates[0]["points"], [(0.8, 0.0, 3.1)])
        self.assertEqual(len(obstacle_updates), 1)
        self.assertIn(
            "floor_transition_sensor_confirmed",
            [payload["event"] for payload in diagnostics],
        )
        self.assertIn(
            "floor_transition_ambiguous_exit",
            [payload["event"] for payload in diagnostics],
        )

    def test_target_elevator_opening_scan_always_faces_actual_door(self):
        manager = _make_test_manager()
        manager.current_floor = 1
        manager.initial_entry_geometry_tolerance = 0.25
        manager._map_runtime.last_cloud_stamp = 2.0
        manager._floor_transition_runtime._guard = None
        manager._map_runtime.latest_cloud_points = [(1.0, 0.0, 0.5)]
        manager._elevator_observation.door_open_confirmation = {
            "door_id": "elevator_floor_1",
            "baseline": None,
            "last_image_stamp": None,
            "visual_confirmations": 0,
            "last_visual_change": None,
            "last_cloud_stamp": None,
            "livox_confirmations": 0,
            "logged": False,
        }
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "elevator_floor_1": SimpleNamespace(
                    x=1.0,
                    y=0.0,
                    yaw=math.pi,
                    width=1.4,
                )
            }
        )
        observed_poses = []
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: observed_poses.append(
                kwargs["robot_pose"]
            )
            or None
        )

        self.assertFalse(
            manager.confirm_elevator_door_open(
                "elevator_floor_1",
                (2.4, 0.0, 0.0),
            )
        )
        self.assertEqual(len(observed_poses), 1)
        self.assertAlmostEqual(abs(observed_poses[0][2]), math.pi)

    def test_target_elevator_opening_uses_target_floor_livox_during_transition(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.initial_entry_geometry_tolerance = 0.25
        manager._map_runtime.last_cloud_stamp = 2.0
        manager._map_runtime.latest_cloud_points = [(1.0, 0.0, 0.5)]
        manager._floor_transition_runtime._guard = SimpleNamespace(
            explicit_transition=True,
            target_floor=1,
        )
        manager._elevator_observation.door_open_confirmation = {
            "door_id": "elevator_floor_1",
            "baseline": None,
            "last_image_stamp": None,
            "visual_confirmations": 0,
            "last_visual_change": None,
            "last_cloud_stamp": None,
            "livox_confirmations": 0,
            "logged": False,
        }
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "elevator_floor_1": SimpleNamespace(
                    x=1.0,
                    y=0.0,
                    yaw=math.pi,
                    width=1.4,
                )
            }
        )
        observations = []
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: observations.append(kwargs)
            or SimpleNamespace(world_xy=(1.0, 0.0), width=1.4)
        )

        self.assertFalse(
            manager.confirm_elevator_door_open(
                "elevator_floor_1",
                (2.4, 0.0, 0.0),
            )
        )
        manager._map_runtime.last_cloud_stamp = 3.0
        self.assertTrue(
            manager.confirm_elevator_door_open(
                "elevator_floor_1",
                (2.4, 0.0, 0.0),
            )
        )
        self.assertEqual([item["floor_index"] for item in observations], [1, 1])

    def test_target_elevator_opening_uses_rgbd_and_livox_dual_channel_during_transition(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.initial_entry_geometry_tolerance = 0.25
        manager._map_runtime.last_cloud_stamp = 2.0
        manager._map_runtime.latest_cloud_points = [(1.0, 0.0, 0.5)]
        manager._floor_transition_runtime._guard = SimpleNamespace(
            explicit_transition=True,
            target_floor=1,
        )
        manager._elevator_observation.door_open_confirmation = {
            "door_id": "elevator_floor_1",
            "baseline": None,
            "last_image_stamp": None,
            "visual_confirmations": 0,
            "last_visual_change": None,
            "last_cloud_stamp": None,
            "livox_confirmations": 0,
            "logged": False,
        }
        manager.building_client = SimpleNamespace(
            doors={
                "elevator_floor_1": SimpleNamespace(
                    x=1.0,
                    y=0.0,
                    yaw=math.pi,
                    width=1.4,
                )
            }
        )
        calls = []
        manager._map_runtime.rgbd_door_detector = SimpleNamespace(
            assess_registered_opening_messages=lambda *args, **kwargs: (
                calls.append(kwargs)
                or DoorOpeningChange(True, 0.0, 1.0, 1.0)
            )
        )
        livox_calls = []
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: livox_calls.append(kwargs) or None
        )
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager._log_structured = lambda payload: None
        bundle = SimpleNamespace(
            rgb_msg=object(),
            depth_msg=object(),
            camera_info_msg=object(),
        )
        camera_to_world = np.eye(4)

        for image_stamp in (3.0, 4.0):
            manager._elevator_observation.update_door_visual_change(
                bundle=bundle,
                image_stamp=image_stamp,
                camera_to_world=camera_to_world,
                detector=manager.rgbd_door_detector,
                building_client=manager.building_client,
            )

        self.assertEqual(len(calls), 2)
        self.assertTrue(
            manager.confirm_elevator_door_open(
                "elevator_floor_1",
                (2.4, 0.0, 0.0),
            )
        )

    def test_explicit_floor_transition_caches_target_floor_cloud_without_mapping(self):
        manager, _clock, _diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager._floor_coordinates = FloorCoordinateModel("floor_local", 2.6)
        manager.begin_floor_transition(
            source_floor=0,
            target_floor=1,
            phase="CALL_ELEVATOR_TO_TARGET_FLOOR",
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.39),
        )

        self.assertEqual(reads, [True])
        self.assertEqual(manager._map_runtime.latest_cloud_points, [(0.8, 0.0, 3.1)])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])

    def test_floor_transition_latch_timeout_stops_and_writes_failure(self):
        manager, clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        manager.current_task = None

        clock[0] = 1.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=2.0, odom_stamp=2.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        clock[0] = 6.1
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=3.0, odom_stamp=3.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        clock[0] = 12.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=4.0, odom_stamp=4.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])
        self.assertEqual(manager.current_floor, 0)
        self.assertEqual(manager._failure_reasons, ["楼层过渡对齐超时"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(manager.state, WRITE_RESULT)
        self.assertIn(
            {
                "event": "controller_stop",
                "reason": "floor_transition_alignment_timeout",
            },
            diagnostics,
        )
        self.assertEqual(
            sum(payload["event"] == "floor_transition_alignment_timeout" for payload in diagnostics),
            1,
        )

    def test_floor_transition_latch_handles_silent_transport_failure_fail_closed(self):
        manager, _clock, _diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_floor = 1
        manager.current_task.current_floor = 0
        manager.current_task.target_floor = 1

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )
        manager.current_task = None
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=2.0, odom_stamp=2.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )

        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])
        self.assertEqual(manager.current_floor, 1)

    def test_floor_transition_latch_requires_two_fresh_aligned_odom_samples(self):
        manager, clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        stale_cloud = self._mapping_snapshot(cloud_stamp=10.0, odom_stamp=10.0)
        manager._update_mapping_and_detection(
            stale_cloud,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        manager.current_floor = 1

        clock[0] = 1.0
        manager._update_mapping_and_detection(
            stale_cloud,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        self.assertIsNotNone(manager._floor_transition_guard)

        clock[0] = 1.1
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=11.0, odom_stamp=11.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        self.assertIsNotNone(manager._floor_transition_guard)

        clock[0] = 1.2
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=12.0, odom_stamp=12.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertIsNone(manager._floor_transition_guard)
        self.assertEqual(len(reads), 1)
        self.assertEqual(len(map_updates), 1)
        self.assertEqual(map_updates[0]["floor_index"], 1)
        self.assertEqual(len(obstacle_updates), 1)
        self.assertEqual(manager._map_runtime.last_cloud_stamp, 12.0)
        self.assertEqual(
            sum(payload["event"] == "floor_transition_ambiguous_exit" for payload in diagnostics),
            1,
        )

    def test_floor_transition_latch_resets_alignment_after_new_mismatch(self):
        manager, clock, _diagnostics, _reads, map_updates, _obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        manager.current_floor = 1
        clock[0] = 1.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=2.0, odom_stamp=2.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )
        clock[0] = 2.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=3.0, odom_stamp=3.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )
        manager.current_floor = 0
        clock[0] = 3.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=4.0, odom_stamp=4.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )

        self.assertIsNotNone(manager._floor_transition_guard)
        self.assertEqual(map_updates, [])

        clock[0] = 4.0
        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=5.0, odom_stamp=5.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )
        self.assertIsNone(manager._floor_transition_guard)
        self.assertEqual(len(map_updates), 1)
        self.assertEqual(map_updates[0]["floor_index"], 0)

    def test_non_elevator_height_jitter_does_not_start_floor_transition_latch(self):
        manager, _clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None

        manager._update_mapping_and_detection(
            self._mapping_snapshot(cloud_stamp=1.0, odom_stamp=1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertIsNone(manager._floor_transition_guard)
        self.assertEqual(len(reads), 1)
        self.assertEqual(len(map_updates), 1)
        self.assertEqual(map_updates[0]["floor_index"], 0)
        self.assertEqual(len(obstacle_updates), 1)
        self.assertEqual(diagnostics[0]["event"], "floor_alignment_mismatch")

    def test_floor_transition_latch_keeps_rgb_detection_running(self):
        manager, _clock, _diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        detections = []
        manager._map_runtime.detector = SimpleNamespace(
            detect_messages=lambda *args, **kwargs: detections.append(True) or [],
        )
        manager._map_runtime.tracker = SimpleNamespace(update=lambda *args, **kwargs: None)
        manager.camera_mount_xyz = (0.28, 0.0, 0.043)
        manager.camera_mount_rpy = (0.0, 0.0, 0.0)
        # 楼层过渡门控只暂停点云建图，不应阻止 RGB-D 检测；仍需要时间匹配位姿。
        manager.pose_provider = SimpleNamespace(
            at=lambda stamp: mission_manager_module.PoseSample(
                stamp=float(stamp or 0.0),
                xyz=(0.0, 0.0, 0.39),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0),
                angular_xyz=(0.0, 0.0, 0.0),
            )
            if stamp == 1.0
            else None,
            sensor_to_world_matrix=lambda sample, **kwargs: np.eye(4),
        )
        rgb_msg = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 1.0),
                frame_id="camera",
            )
        )
        bundle = SimpleNamespace(
            rgb_msg=rgb_msg,
            depth_msg=rgb_msg,
            camera_info_msg=rgb_msg,
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(
                cloud_stamp=1.0,
                odom_stamp=1.0,
                image_bundle=bundle,
            ),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.916),
        )

        self.assertEqual(reads, [])
        self.assertEqual(map_updates, [])
        self.assertEqual(obstacle_updates, [])
        self.assertEqual(detections, [True])

    def test_rgb_detection_uses_time_matched_pose_not_cloud_pose(self):
        """RGB-D 检测应使用与彩色图像时间戳匹配的内部定位位姿。"""
        from danger_search.pose_provider import PoseProvider

        manager, _clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None
        manager.camera_mount_xyz = (0.28, 0.0, 0.043)
        manager.camera_mount_rpy = (0.0, 0.0, 0.0)
        camera_to_world_matrices = []
        manager._map_runtime.detector = SimpleNamespace(
            detect_messages=lambda *args, **kwargs: camera_to_world_matrices.append(
                args[3]
            )
            or [],
        )
        manager._map_runtime.tracker = SimpleNamespace(update=lambda *args, **kwargs: None)

        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        # 云帧位姿在 x=0.5；图像时间戳位姿在 x=1.0
        provider._on_odometry(self._odom_message(1.0))
        matched_sample = mission_manager_module.PoseSample(
            stamp=2.0,
            xyz=(1.0, 0.0, 0.39),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_xyz=(0.0, 0.0, 0.0),
            angular_xyz=(0.0, 0.0, 0.0),
        )
        provider._samples.append(matched_sample)
        manager.pose_provider = provider

        rgb_msg = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 2.0),
                frame_id="camera",
            )
        )
        bundle = SimpleNamespace(
            rgb_msg=rgb_msg,
            depth_msg=rgb_msg,
            camera_info_msg=rgb_msg,
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(
                cloud_stamp=1.0,
                odom_stamp=1.0,
                image_bundle=bundle,
            ),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )

        self.assertEqual(len(camera_to_world_matrices), 1)
        self.assertAlmostEqual(
            camera_to_world_matrices[0][0, 3],
            1.0 + 0.28,  # 时间匹配位姿 x + camera_mount_xyz x
            places=3,
        )

    def test_rgb_detection_skips_frame_when_time_matched_pose_missing(self):
        """没有匹配时间戳的内部定位位姿时，应跳过该帧检测，不得回退。"""
        manager, _clock, diagnostics, reads, map_updates, obstacle_updates = (
            self._make_floor_transition_manager()
        )
        manager.current_task = None
        detections = []
        manager._map_runtime.detector = SimpleNamespace(
            detect_messages=lambda *args, **kwargs: detections.append(True) or [],
        )
        manager._map_runtime.tracker = SimpleNamespace(update=lambda *args, **kwargs: None)
        manager.pose_provider = SimpleNamespace(
            at=lambda _stamp: None,
            sensor_to_world_matrix=lambda *args, **kwargs: self.fail(
                "不存在匹配位姿时不应构造相机外参矩阵"
            ),
        )

        rgb_msg = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 2.0),
                frame_id="camera",
            )
        )
        bundle = SimpleNamespace(
            rgb_msg=rgb_msg,
            depth_msg=rgb_msg,
            camera_info_msg=rgb_msg,
        )

        manager._update_mapping_and_detection(
            self._mapping_snapshot(
                cloud_stamp=1.0,
                odom_stamp=1.0,
                image_bundle=bundle,
            ),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.316),
        )

        self.assertEqual(detections, [])
        self.assertTrue(
            any(
                payload.get("event") == "rgbd_pose_lookup_skipped"
                for payload in diagnostics
            ),
            diagnostics,
        )

    def test_transition_state_records_structured_diagnostics_for_new_object(self):
        """轻量构造的 manager 也应记录完整状态转换上下文。"""
        logged_payloads = []
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_floor = 0
        manager.current_task = SimpleNamespace(name="ExploreTask", phase="tracking")
        manager.current_frontier = SimpleNamespace(centroid=(1.25, 2.5))
        manager._exploration.no_reachable_frontier_confirmations = 2
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.explorer = SimpleNamespace(
            last_selection_debug=SimpleNamespace(reachable_count=3),
        )

        with patch.object(
            MissionReportingRuntime,
            "log_structured",
            side_effect=lambda payload: logged_payloads.append(dict(payload)),
        ):
            manager._transition_state(
                RETURN_HOME,
                reason_code="floor_completion_ready",
                call_site="test.transition",
            )

        self.assertEqual(manager.state, RETURN_HOME)
        self.assertTrue(manager._return_home_started)
        self.assertEqual(len(manager._transition_history), 1)
        transition = manager._transition_history[0]
        self.assertEqual(transition["old_state"], EXPLORE_FLOOR)
        self.assertEqual(transition["new_state"], RETURN_HOME)
        self.assertEqual(transition["reason_code"], "floor_completion_ready")
        self.assertEqual(transition["call_site"], "test.transition")
        self.assertEqual(transition["current_task"], "ExploreTask")
        self.assertEqual(transition["last_reachable_frontier_count"], 3)
        self.assertTrue(transition["has_reachable_frontier"])
        self.assertEqual(transition["no_reachable_frontier_confirmations"], 2)
        self.assertEqual(transition["current_floor"], 0)
        self.assertTrue(transition["return_home_started"])
        self.assertFalse(transition["return_home_completed"])
        self.assertEqual(logged_payloads, [transition])

    def test_entering_return_home_clears_exploration_path_and_frontier(self):
        cleared = []
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_floor = 0
        manager.current_task = None
        manager.current_frontier = SimpleNamespace(centroid=(4.0, 8.0))
        manager._exploration.last_frontier_target = (4.0, 8.0)
        manager._exploration.last_frontier_score = 1.0
        manager._exploration.last_frontier_used_fallback = False
        manager._return_home_started = False
        manager._return_home_completed = False
        manager._exploration.no_reachable_frontier_confirmations = 0
        manager._transition_history = []
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.explorer = SimpleNamespace(
            last_selection_debug=None,
            floor_termination_evidence=lambda *_args, **_kwargs: (
                FloorTerminationEvidence(
                    constraint_revision=0,
                    region_ids=(),
                    formal_reachable_count=0,
                    effective_reachable_count=0,
                    support_blocked_count=0,
                    fully_blocked=False,
                )
            ),
        )
        manager.controller = SimpleNamespace(
            clear_path=lambda reason=None: cleared.append(reason),
            poll_status=lambda: 'RUNNING',
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._transition_state(
                RETURN_HOME,
                reason_code="mission_insufficient_time_for_return",
                call_site="test.enter_return",
            )

        self.assertEqual(cleared, ["enter_return_home"])
        self.assertIsNone(manager.current_frontier)
        self.assertIsNone(manager._exploration.last_frontier_target)

    def test_external_shutdown_preserves_actual_state_in_result_diagnostics(self):
        """外部 ROS shutdown 写结果时必须保留实际状态和触发源。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary_path = root / "mission_runtime_summary.json"
            result_path = root / "detected_danger.json"
            manager = _make_test_manager()
            manager.state = EXPLORE_FLOOR
            manager.start_time = 0.0
            manager.current_floor = 0
            manager.max_floor = 1
            manager.current_task = None
            manager.current_frontier = None
            manager._exploration.no_reachable_frontier_confirmations = 0
            manager._finished = False
            manager._shutdown_requested = False
            manager._return_home_success = False
            manager._elevator_success = None
            manager._timeout_triggered = False
            manager._failure_reasons = []
            manager._last_pose = (1.0, 2.0, 0.0)
            manager._max_home_distance = 3.0
            manager.mission_summary_file = str(summary_path)
            manager._map_runtime.mapper = MultiFloorGridMap(
                resolution=0.5,
                width=4.0,
                length=4.0,
                origin_x=-2.0,
                origin_y=-2.0,
            )
            manager.explorer = SimpleNamespace(
                last_selection_debug=SimpleNamespace(reachable_count=2),
            )
            manager._map_runtime.tracker = SimpleNamespace(
                confirmed_positions=lambda now: [],
                tracks=[],
            )
            manager._reporting = MissionReportingRuntime(
                result_file=result_path,
                mission_summary_file=summary_path,
                status_topic="/test/status",
                map_topic="/test/map",
            )
            manager.controller = SimpleNamespace(
                stop=lambda *args, **kwargs: None,
                command_debug_summary=lambda: {},
            poll_status=lambda: 'RUNNING',
            )

            with patch.object(mission_manager_module, "get_time", return_value=50.2), patch.object(
                mission_manager_module,
                "MissionReportingRuntime",
                SimpleNamespace(loginfo=lambda *args, **kwargs: None),
            ):
                manager.handle_shutdown()

            summary = json.loads(summary_path.read_text(encoding="utf-8"))

        self.assertEqual(manager.state, EXPLORE_FLOOR)
        self.assertTrue(manager._finished)
        self.assertEqual(summary["state"], EXPLORE_FLOOR)
        self.assertEqual(summary["terminal_state"], MISSION_FAILED)
        self.assertEqual(summary["terminal_outcome"], "failed")
        self.assertFalse(summary["completed"])
        self.assertEqual(summary["state_at_result_write"], EXPLORE_FLOOR)
        self.assertEqual(summary["result_write_reason"], "external_ros_shutdown")
        self.assertEqual(
            summary["result_write_call_site"],
            "MissionManager.handle_shutdown",
        )
        self.assertFalse(summary["return_home_started"])
        self.assertFalse(summary["return_home_completed"])
        self.assertEqual(summary["transition_history"], [])
        self.assertEqual(summary["shutdown_context"]["state_at_shutdown"], EXPLORE_FLOOR)
        self.assertFalse(summary["shutdown_context"]["shutdown_requested_internally"])

    def test_floor_timeout_is_soft_budget_and_keeps_exploring(self):
        """仍有可达前沿或覆盖不足时，楼层预算只能告警，不能结束任务。"""
        logged_payloads = []
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.start_time = 0.0
        manager.floor_timeout = 180.0
        manager.max_floor = 1
        manager.min_floor_explore_time = 20.0
        manager.last_floor_uses_global_budget = True
        manager.current_floor = 0
        manager.current_task = None
        manager.current_frontier = SimpleNamespace(centroid=(2.0, 1.0))
        manager._exploration.no_reachable_frontier_confirmations = 0
        manager._return_home_started = False
        manager._return_home_completed = False
        manager._return_home_success = False
        manager._mission_success_eligible = True
        manager._exploration.floor_timeout_logged = False
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.explorer = SimpleNamespace(
            last_selection_debug=SimpleNamespace(reachable_count=6),
        )

        with patch.object(
            mission_manager_module,
            "get_time",
            return_value=183.0,
        ), patch.object(
            MissionReportingRuntime,
            "log_structured",
            side_effect=lambda payload: logged_payloads.append(dict(payload)),
        ):
            manager._log_floor_budget_exhausted(183.0)
            manager._log_floor_budget_exhausted(184.0)

        self.assertEqual(manager.state, EXPLORE_FLOOR)
        self.assertFalse(manager._return_home_started)
        self.assertTrue(manager._mission_success_eligible)
        self.assertEqual(len(logged_payloads), 1)
        self.assertEqual(
            logged_payloads[0]["action"],
            "continue_until_global_return_boundary",
        )
        self.assertEqual(
            logged_payloads[0]["reason_code"],
            "last_floor_continues_under_global_budget",
        )
        self.assertEqual(logged_payloads[0]["last_reachable_frontier_count"], 6)

    def test_successful_single_floor_completion_requires_return_home(self):
        """单层正常完成必须先进入 RETURN_HOME，返航成功后才可正常 FINISH。"""
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.start_time = 0.0
        manager.current_floor = 0
        manager.max_floor = 1
        manager.current_task = None
        manager.current_frontier = None
        manager._exploration.no_frontier_scan_until = 0.0
        manager._exploration.no_reachable_frontier_confirmations = 2
        manager._transition_history = []
        manager._return_home_started = False
        manager._return_home_completed = False
        manager._return_home_success = False
        manager._mission_success_eligible = True
        manager._normal_completion = False
        manager._failure_reasons = []
        manager._finished = False
        manager._shutdown_requested = False
        manager.home_pose = (0.0, 0.0, 0.0)
        manager.home_tolerance = 0.2
        manager.mission_summary_file = ""
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        manager.explorer = SimpleNamespace(
            last_selection_debug=None,
            floor_termination_evidence=lambda *_args, **_kwargs: (
                FloorTerminationEvidence(
                    constraint_revision=0,
                    region_ids=(),
                    formal_reachable_count=0,
                    effective_reachable_count=0,
                    support_blocked_count=0,
                    fully_blocked=False,
                )
            ),
        )
        manager.controller = SimpleNamespace(
            clear_path=lambda *args, **kwargs: None,
            stop=lambda *args, **kwargs: None,
            poll_status=lambda: 'RUNNING',
        )
        manager._map_runtime.tracker = SimpleNamespace(confirmed_positions=lambda now: [], tracks=[])
        manager._reporting = SimpleNamespace(
            write_result=lambda **_kwargs: Path("result.json")
        )

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._finish_current_floor(reason_code="floor_completion_ready")
            self.assertEqual(manager.state, RETURN_HOME)
            manager._tick_return_home((0.0, 0.0, 0.0))
            self.assertEqual(manager.state, WRITE_RESULT)
            manager._write_result(reason_code=manager._pending_result_write_reason)

        self.assertTrue(manager._normal_completion)
        self.assertTrue(manager._exploration_completed)
        self.assertEqual(manager._terminal_state, FINISH)
        self.assertEqual(manager._terminal_outcome, "success")

    def test_budget_and_low_progress_returns_are_not_exploration_complete(self):
        """返航闭环可以成功，但预算或低收益退出不能伪装成探索完成。"""
        for return_reason in (
            "mission_insufficient_time_for_return",
            "floor_timeout_return_home",
            "floor_low_progress_return_home",
            "floor_timeout_change_floor_return_home",
        ):
            with self.subTest(return_reason=return_reason):
                manager = _make_test_manager()
                manager.state = RETURN_HOME
                manager.start_time = 0.0
                manager.current_floor = 0
                manager.max_floor = 1
                manager.current_task = None
                manager.current_frontier = None
                manager._exploration.no_reachable_frontier_confirmations = 0
                manager._transition_history = []
                manager._return_home_started = True
                manager._return_home_completed = True
                manager._return_home_success = True
                manager._return_home.reason = return_reason
                manager._mission_success_eligible = True
                manager._timeout_triggered = False
                manager._failure_reasons = []
                manager._finished = False
                manager.mission_summary_file = ""
                manager.controller = SimpleNamespace(stop=lambda *args, **kwargs: None)
                manager._map_runtime.tracker = SimpleNamespace(
                    confirmed_positions=lambda now: [],
                    tracks=[],
                )
                manager._reporting = SimpleNamespace(
                    write_result=lambda **_kwargs: Path("result.json")
                )
                manager._explored_floor_ids = {0}
                manager._floor_exit_certificates = {
                    0: FloorExitCertificate(
                        floor=0,
                        map_epoch=1,
                        localization_epoch=1,
                        constraint_revision=0,
                        reason_code=return_reason,
                        formal_reachable_count=0,
                        effective_reachable_count=0,
                        region_ids=(),
                        valid=True,
                    )
                }

                with patch.object(
                    mission_manager_module,
                    "get_time",
                    return_value=10.0,
                ), patch.object(
                    mission_manager_module,
                    "MissionReportingRuntime",
                    SimpleNamespace(loginfo=lambda *args, **kwargs: None),
                ):
                    manager._write_result(reason_code="return_home_completed")

                self.assertTrue(manager._normal_completion)
                self.assertFalse(manager._exploration_completed)

    def test_return_home_result_reports_missing_or_invalid_floor_certificate(self):
        for mode, failure_reasons, expected_reason in (
            ("missing", (), "floor_exit_certificate_missing"),
            ("invalid", (), "floor_exit_certificate_invalid"),
            ("invalid", ("localization_unavailable",), "localization_unavailable"),
        ):
            with self.subTest(mode=mode, failure_reasons=failure_reasons):
                manager = _make_test_manager()
                manager.state = WRITE_RESULT
                manager._return_home_started = True
                manager._return_home_completed = True
                manager._return_home_success = True
                manager._mission_success_eligible = True
                manager._explored_floor_ids = {0}
                manager._floor_exit_certificates = {}
                manager._failure_reasons = list(failure_reasons)
                if mode == "invalid":
                    manager._floor_exit_certificates[0] = FloorExitCertificate(
                        floor=0, map_epoch=1, localization_epoch=1,
                        constraint_revision=3, reason_code="support_blocked",
                        formal_reachable_count=2,
                        effective_reachable_count=0,
                        region_ids=("support:3",), valid=False,
                    )
                manager.controller = SimpleNamespace(
                    stop=lambda *args, **kwargs: None,
            poll_status=lambda: 'RUNNING',
                )
                manager._map_runtime.tracker = SimpleNamespace(
                    confirmed_positions=lambda now: [], tracks=[]
                )
                manager._reporting = SimpleNamespace(
                    write_result=lambda **_kwargs: Path("result.json")
                )

                manager._write_result(reason_code="return_home_completed")

                self.assertEqual(manager._terminal_outcome, "failed")
                self.assertEqual(manager._terminal_reason, expected_reason)
                self.assertEqual(manager._result_write_reason, expected_reason)
                self.assertTrue(manager._return_home_completed)

    def test_floor_change_back_to_home_preserves_original_return_reason(self):
        """返航途中完成换层不能覆盖最初的预算或探索完成原因。"""
        manager = _make_test_manager()
        manager.state = RETURN_HOME
        manager.current_floor = 0
        manager.current_task = None
        manager.current_frontier = None
        manager._exploration.no_reachable_frontier_confirmations = 0
        manager._transition_history = []
        manager._return_home_started = True
        manager._return_home_completed = False
        manager._return_home.reason = "floor_timeout_return_home"
        manager.explorer = SimpleNamespace(last_selection_debug=None)

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._transition_state(
                RETURN_HOME,
                reason_code="floor_change_completed_return_home",
                call_site="test.floor_change",
            )

        self.assertEqual(manager._return_home.reason, "floor_timeout_return_home")

    def test_probe_targets_sweep_lanes_before_extending_forward(self):
        base_pose = (0.0, 0.0, 0.0)

        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 0), (1.2, 0.0))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 1), (1.2, -1.2))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 2), (1.2, 1.2))
        self.assertEqual(probe_target_for_attempt(base_pose, 1.2, 5), (2.4, 0.0))

    def test_probe_targets_follow_home_yaw(self):
        target = probe_target_for_attempt((2.0, 3.0, math.pi / 2.0), 1.0, 2)

        self.assertAlmostEqual(target[0], 1.0, places=6)
        self.assertAlmostEqual(target[1], 4.0, places=6)

    def test_probe_distance_has_lower_bound(self):
        target = probe_target_for_attempt((0.0, 0.0, 0.0), 0.0, 0)

        self.assertEqual(target, (0.05, 0.0))

    def test_initial_forward_target_requires_confirmed_free_foot_lanes(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=8.0,
            origin_x=-2.0,
            origin_y=-2.0,
        ).floor(0)
        for x in (-0.2, 0.0, 0.2):
            for index in range(1, 46):
                cell = grid.world_to_grid(x, 0.1 * index)
                grid.mark_free(*cell)

        target = initial_forward_free_target(
            grid,
            (0.0, 0.0, math.pi / 2.0),
            max_distance=5.0,
            corridor_half_width=0.2,
            minimum_distance=1.2,
        )

        self.assertIsNotNone(target)
        self.assertAlmostEqual(target[0], 0.0, places=6)
        self.assertAlmostEqual(target[1], 4.4, places=6)

    def test_initial_forward_target_does_not_cross_unknown_gap(self):
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        ).floor(0)
        for x in (-0.2, 0.0, 0.2):
            for index in range(1, 8):
                cell = grid.world_to_grid(0.1 * index, x)
                grid.mark_free(*cell)

        target = initial_forward_free_target(
            grid,
            (0.0, 0.0, 0.0),
            max_distance=3.0,
            corridor_half_width=0.2,
            minimum_distance=1.2,
        )

        self.assertIsNone(target)

    def test_initial_entry_requests_public_open_service_even_when_declared_open(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._entry.completed = False
        manager._entry.failed = False
        manager._entry.origin_pose = None
        manager._entry.heading = None
        manager._entry.trace = []
        manager._entry.service_task = None
        manager.initial_entry_max_distance = 5.0
        manager.controller = SimpleNamespace(
            goal_tolerance=0.35,
            stop=lambda **kwargs: stops.append(kwargs.get("reason")),
            poll_status=lambda: 'RUNNING',
        )
        manager.building_features = SimpleNamespace(
            doors={
                "main_entrance": PublicDoorFeature(
                    "main_entrance", "main_entrance", 0, True
                )
            }
        )
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        task = SimpleNamespace(done=lambda: False)
        calls = []
        stops = []
        manager.building_client = SimpleNamespace(
            open_door=lambda door_id: calls.append(door_id) or task
        )

        self.assertTrue(manager._run_initial_entry((0.0, -2.2, 0.0), None))
        self.assertTrue(manager._run_initial_entry((0.0, -2.2, 0.0), None))

        self.assertEqual(calls, ["main_entrance"])
        self.assertEqual(stops, ["initial_entry_opening", "initial_entry_opening"])

    def test_initial_entry_registers_sensor_opening_after_two_new_clouds(self):
        cleared_reasons = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.current_task = None
        manager.current_frontier = object()
        manager.active_door = None
        manager._exploration.last_frontier_target = (1.0, 1.0)
        manager._exploration.last_frontier_score = 1.0
        manager._exploration.last_frontier_used_fallback = True
        manager._map_runtime.latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager._map_runtime.last_cloud_stamp = 1.0
        manager._door_observation_runtime = DoorObservationRuntime()
        manager._entry.origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._entry.heading = math.pi / 2.0
        manager.initial_entry_opening_confirmations = 2
        manager.initial_entry_geometry_tolerance = 0.25
        manager.initial_entry_opening_max_distance = 1.2
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_transition_hold_distance = 2.5
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.controller = SimpleNamespace(
            clear_path=lambda **kwargs: cleared_reasons.append(kwargs.get("reason")),
            poll_status=lambda: 'RUNNING',
        )
        manager.building_client = SimpleNamespace(doors={})
        observation = OpeningObservation(
            distance=0.8,
            world_xy=(0.4, 0.0),
            yaw=math.pi / 2.0,
            width=0.80,
            evidence_count=30,
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: observation
        )
        entrance = SimpleNamespace(
            door_id="main_entrance",
            kind="main_entrance",
            floor_index=0,
        )

        with patch.object(
            initial_entry_runtime_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(
                _try_start_entry_door(
                    manager,
                    (0.0, -0.8, math.pi / 2.0), entrance
                )
            )
            # 同一份 cloud 不得重复累计确认。
            self.assertFalse(
                _try_start_entry_door(
                    manager,
                    (0.0, -0.8, math.pi / 2.0), entrance
                )
            )
            manager._map_runtime.last_cloud_stamp = 1.1
            self.assertTrue(
                _try_start_entry_door(
                    manager,
                    (0.0, -0.8, math.pi / 2.0), entrance
                )
            )

        registered = manager.building_client.doors["main_entrance"]
        self.assertAlmostEqual(registered.x, 0.0)
        self.assertAlmostEqual(registered.y, 0.0)
        self.assertAlmostEqual(registered.width, 0.80)
        self.assertIsInstance(manager.current_task, OpenDoorAndPass)
        self.assertEqual(manager.current_task.phase, "CALL_OPEN")
        self.assertEqual(manager.active_door, "main_entrance")
        self.assertEqual(cleared_reasons, ["initial_entry_opening_confirmed"])

    def test_initial_entry_does_not_register_distant_rough_opening(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.current_task = None
        manager._map_runtime.latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager._map_runtime.last_cloud_stamp = 1.0
        manager._door_observation_runtime = DoorObservationRuntime()
        manager.initial_entry_opening_confirmations = 1
        manager.initial_entry_geometry_tolerance = 0.25
        manager.initial_entry_opening_max_distance = 1.2
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: OpeningObservation(
                distance=2.2,
                world_xy=(0.3, 0.0),
                yaw=math.pi / 2.0,
                width=1.4,
                evidence_count=20,
            )
        )
        entrance = SimpleNamespace(
            door_id="main_entrance",
            kind="main_entrance",
            floor_index=0,
        )

        self.assertFalse(
            _try_start_entry_door(
                manager,
                (0.0, -2.2, math.pi / 2.0), entrance
            )
        )

    def test_open_main_entrance_refreshes_geometry_from_two_near_clouds(self):
        manager = _make_test_manager()
        manager._entry.origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._entry.heading = math.pi / 2.0
        manager._entry.completed = False
        manager._door_observation_runtime = DoorObservationRuntime()
        manager._map_runtime.last_cloud_stamp = 1.0
        manager._map_runtime.latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.10
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    -0.2,
                    0.0,
                    math.pi / 2.0,
                    1.8,
                )
            }
        )
        observations = iter(
            (
                OpeningObservation(0.8, (0.25, 0.02), math.pi / 2.0 + 0.1, 1.8, 30),
                OpeningObservation(0.8, (0.27, 0.01), math.pi / 2.0 + 0.1, 1.82, 32),
            )
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: next(observations)
        )
        failures = []
        manager._record_failure = lambda reason: failures.append(reason)

        with patch.object(mission_manager_module, "get_time", side_effect=(10.0, 10.1)), patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.0, -0.8, math.pi / 2.0)
                ),
                RUNNING,
            )
            manager._map_runtime.last_cloud_stamp = 1.1
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.0, -0.8, math.pi / 2.0)
                ),
                DONE,
            )

        refreshed = manager.building_client.doors["main_entrance"]
        self.assertAlmostEqual(refreshed.x, 0.27)
        self.assertAlmostEqual(refreshed.y, 0.01)
        self.assertAlmostEqual(refreshed.yaw, math.pi / 2.0 + 0.1)
        self.assertAlmostEqual(refreshed.width, 1.82)
        self.assertEqual(failures, [])

    def test_return_main_entrance_refresh_rejects_wrong_axis_then_uses_current_geometry(self):
        manager = _make_test_manager()
        manager._entry.origin_pose = (0.0, -2.2, math.pi / 2.0)
        manager._entry.completed = True
        manager._door_observation_runtime = DoorObservationRuntime()
        wrong_axis = OpeningObservation(
            0.7, (-0.20, 0.52), -2.13, 2.20, 40
        )
        observation = OpeningObservation(
            0.7, (0.42, 0.08), math.pi / 2.0 + 0.04, 1.3, 40
        )
        manager._map_runtime.last_cloud_stamp = 2.0
        manager._map_runtime.latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.1
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.3,
                )
            }
        )
        manager.current_task = None
        observations = iter((wrong_axis, observation, observation))
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: next(observations)
        )
        manager._record_failure = lambda reason: self.fail(reason)

        with patch.object(mission_manager_module, "get_time", return_value=20.0), patch.object(
            MissionReportingRuntime,
            "logwarn_throttle",
        ), patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logwarn_throttle=lambda *args, **kwargs: None,
            ),
        ):
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.35, -0.6, math.pi / 2.0)
                ),
                RUNNING,
            )
            unchanged = manager.building_client.doors["main_entrance"]
            self.assertAlmostEqual(unchanged.x, 0.0)
            self.assertAlmostEqual(unchanged.y, 0.0)
            self.assertAlmostEqual(unchanged.yaw, math.pi / 2.0)

            manager._map_runtime.last_cloud_stamp = 2.1
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.35, -0.6, math.pi / 2.0)
                ),
                RUNNING,
            )
            manager._map_runtime.last_cloud_stamp = 2.2
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.35, -0.6, math.pi / 2.0)
                ),
                DONE,
            )

        refreshed = manager.building_client.doors["main_entrance"]
        self.assertAlmostEqual(refreshed.x, 0.42)
        self.assertAlmostEqual(refreshed.y, 0.08)
        self.assertAlmostEqual(refreshed.yaw, math.pi / 2.0 + 0.04)

    def test_return_main_entrance_refresh_rejects_adjacent_opening_by_center_shift(self):
        manager = _make_test_manager()
        manager._entry.origin_pose = (0.0, -2.2, math.pi / 2.0)
        manager._entry.completed = True
        manager._door_observation_runtime = DoorObservationRuntime()
        manager._map_runtime.last_cloud_stamp = 3.0
        manager._map_runtime.latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.1
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.12,
                    -0.09,
                    1.49,
                    1.15,
                )
            }
        )
        observed_queries = []

        def detect_opening_ahead(**kwargs):
            observed_queries.append(kwargs["robot_pose"])
            return OpeningObservation(
                0.7,
                (-0.34, 0.50),
                kwargs["robot_pose"][2],
                2.35,
                40,
            )

        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=detect_opening_ahead
        )
        manager._record_failure = lambda reason: self.fail(reason)

        with patch.object(mission_manager_module, "get_time", return_value=20.0), patch.object(
            MissionReportingRuntime,
            "logwarn_throttle",
        ), patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logwarn_throttle=lambda *args, **kwargs: None,
            ),
        ):
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (-0.25, 0.33, -2.18)
                ),
                RUNNING,
            )

        unchanged = manager.building_client.doors["main_entrance"]
        self.assertAlmostEqual(unchanged.x, 0.12)
        self.assertAlmostEqual(unchanged.y, -0.09)
        self.assertEqual(
            manager._door_observation_runtime.refresh.confirmations,
            0,
        )
        self.assertEqual(len(observed_queries), 1)
        query_yaw = observed_queries[0][2]
        self.assertAlmostEqual(
            0.5
            * abs(
                math.atan2(
                    math.sin(2.0 * (query_yaw - unchanged.yaw)),
                    math.cos(2.0 * (query_yaw - unchanged.yaw)),
                )
            ),
            0.0,
        )

    def test_open_main_entrance_refresh_timeout_fails_closed(self):
        manager = _make_test_manager()
        manager._entry.origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._entry.completed = False
        manager._door_observation_runtime = DoorObservationRuntime()
        manager._map_runtime.last_cloud_stamp = 1.0
        manager._map_runtime.latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.10
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.8,
                )
            }
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: OpeningObservation(
                0.8,
                (0.0, 0.0),
                math.pi / 2.0,
                0.9,
                20,
            )
        )
        failures = []
        events = []
        manager._record_failure = lambda reason: failures.append(reason)
        manager._log_structured = lambda payload: events.append(payload)

        with patch.object(mission_manager_module, "get_time", side_effect=(10.0, 13.1)):
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.0, -0.8, math.pi / 2.0)
                ),
                RUNNING,
            )
            manager._map_runtime.last_cloud_stamp = 1.1
            self.assertEqual(
                manager.refresh_open_door_geometry(
                    "main_entrance", (0.0, -0.8, math.pi / 2.0)
                ),
                FAILED,
            )

        self.assertEqual(failures, ["主入口开放门洞近场重观测超时"])
        self.assertEqual(
            manager._door_observation_runtime.refresh.rejection_counts,
            {"width_insufficient": 1},
        )
        self.assertEqual(events[-1]["event"], "main_entrance_refresh_timeout")
        self.assertEqual(
            events[-1]["rejection_counts"], {"width_insufficient": 1}
        )
        self.assertAlmostEqual(events[-1]["last_candidate"]["width"], 0.9)

    def test_open_main_entrance_refresh_classifies_candidate_rejections(self):
        manager = _make_test_manager()
        manager._entry.origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._door_observation_runtime = DoorObservationRuntime()
        manager._map_runtime.last_cloud_stamp = 1.0
        manager._map_runtime.latest_cloud_points = [(0.0, 0.0, 0.5)]
        manager.current_floor = 0
        manager.door_refresh_timeout = 3.0
        manager.door_refresh_max_center_shift = 0.60
        manager.door_clearance_margin = 0.10
        manager.initial_entry_opening_max_distance = 1.5
        manager.initial_entry_geometry_tolerance = 0.3
        manager.initial_entry_opening_confirmations = 2
        manager.ground_support_params = {"sweep_half_width": 0.435393}
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.8,
                )
            }
        )
        observations = iter(
            (
                None,
                OpeningObservation(1.6, (0.0, 0.0), math.pi / 2.0, 1.8, 20),
                OpeningObservation(0.8, (0.0, 0.0), math.pi / 2.0, 0.9, 20),
            )
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **kwargs: next(observations)
        )
        manager._record_failure = lambda reason: self.fail(reason)

        with patch.object(mission_manager_module, "get_time", return_value=10.0):
            for stamp in (1.0, 1.1, 1.2):
                manager._map_runtime.last_cloud_stamp = stamp
                self.assertEqual(
                    manager.refresh_open_door_geometry(
                        "main_entrance", (0.0, -0.8, math.pi / 2.0)
                    ),
                    RUNNING,
                )

        self.assertEqual(
            manager._door_observation_runtime.refresh.rejection_counts,
            {
                "distance_exceeded": 1,
                "no_candidate": 1,
                "width_insufficient": 1,
            },
        )

    def test_livox_only_elevator_registration_requires_explicit_opt_in(self):
        manager = _make_test_manager()
        manager.max_floor = 2
        manager.current_floor = 0
        manager.current_task = None
        manager._map_runtime.last_cloud_stamp = 1.0
        manager._elevator_observation.registration.opening_last_stamp = None
        manager._elevator_observation.registration.opening_last_scan_stamp = None
        manager._elevator_observation.registration.opening_observation = None
        manager._elevator_observation.registration.opening_confirmations = 0
        manager._map_runtime.latest_cloud_points = [(2.0, 3.0, 0.5)]
        manager.initial_entry_geometry_tolerance = 0.25
        manager.main_entrance_xy = (0.0, 0.0)
        manager.door_pass_offset = 0.8
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.floor_params = {}
        elevator = PublicElevatorFeature(
            elevator_id="public_lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={elevator.elevator_id: elevator}
        )
        manager.building_client = BuildingControlClient()
        manager.building_client.set_public_features(
            door_ids={"ground_lift_door", "upper_lift_door"},
            elevators={"public_lift": {0, 1}},
            elevator_door_ids_by_floor={
                0: ["ground_lift_door"],
                1: ["upper_lift_door"],
            },
        )
        observation = OpeningObservation(
            distance=0.9,
            world_xy=(2.0, 3.0),
            yaw=0.0,
            width=1.0,
            evidence_count=42,
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_elevator_opening_nearby=lambda **kwargs: observation
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(_try_register_test_elevator(manager, (1.1, 3.0, 0.0)))
            self.assertEqual(manager.building_client.elevators, {})
            manager.livox_elevator_registration_enabled = True
            self.assertFalse(_try_register_test_elevator(manager, (1.1, 3.0, 0.0)))
            manager._map_runtime.last_cloud_stamp = 1.6
            self.assertTrue(_try_register_test_elevator(manager, (1.1, 3.0, 0.0)))

        registered = manager.building_client.elevators["public_lift"]
        self.assertEqual(set(registered.lobby_poses), {0, 1})
        self.assertAlmostEqual(registered.lobby_poses[0][0], 2.0)
        self.assertAlmostEqual(registered.lobby_poses[0][1], 3.0)
        self.assertAlmostEqual(abs(registered.lobby_poses[0][2]), math.pi)
        self.assertEqual(
            set(manager.building_client.doors),
            {"ground_lift_door", "upper_lift_door"},
        )
        self.assertEqual(manager._active_elevator_id, "public_lift")
        self.assertNotIn("elevator_id", manager.floor_params)

    def test_elevator_registration_accepts_stable_rgbd_panel_geometry(self):
        manager = _make_test_manager()
        manager.max_floor = 2
        manager.current_floor = 0
        manager.current_task = None
        manager._map_runtime.last_cloud_stamp = None
        manager._elevator_observation.registration.opening_last_stamp = None
        manager._elevator_observation.registration.opening_last_scan_stamp = None
        manager._elevator_observation.registration.opening_observation = None
        manager._elevator_observation.registration.opening_confirmations = 0
        manager._elevator_observation.registration.visual_last_stamp = None
        manager._elevator_observation.registration.visual_candidate = None
        manager._elevator_observation.registration.visual_confirmations = 0
        manager._elevator_observation.registration.latest_rgbd_door_stamp = None
        manager._elevator_observation.registration.latest_rgbd_door_observations = []
        manager.initial_entry_geometry_tolerance = 0.25
        manager.elevator_visual_confirmations = 3
        manager.elevator_visual_min_width = 0.9
        manager.elevator_visual_max_width = 1.8
        manager.elevator_visual_max_distance = 4.0
        manager.main_entrance_xy = (0.0, 0.0)
        manager.door_pass_offset = 0.8
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.floor_params = {}
        elevator = PublicElevatorFeature(
            elevator_id="public_lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={elevator.elevator_id: elevator}
        )
        manager.building_client = BuildingControlClient()
        manager.building_client.set_public_features(
            door_ids={"ground_lift_door", "upper_lift_door"},
            elevators={"public_lift": {0, 1}},
            elevator_door_ids_by_floor={
                0: ["ground_lift_door"],
                1: ["upper_lift_door"],
            },
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_elevator_opening_nearby=lambda **kwargs: None
        )
        observation = DoorPanelObservation(
            stamp=1.0,
            distance=1.4,
            world_xy=(2.0, 3.0),
            outward_yaw=math.pi,
            width=1.35,
            height=1.8,
            plane_rmse=0.01,
            depth_inlier_count=1200,
            bbox=(100, 80, 180, 260),
            median_depth=1.4,
            center_dark_fraction=0.95,
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            for stamp in (1.0, 1.1):
                manager._elevator_observation.registration.latest_rgbd_door_stamp = stamp
                manager._elevator_observation.registration.latest_rgbd_door_observations = [
                    mission_manager_module.replace(observation, stamp=stamp)
                ]
                self.assertFalse(
                    _try_register_test_elevator(manager, (0.7, 3.0, 0.0))
                )
            manager._elevator_observation.registration.latest_rgbd_door_stamp = 1.2
            manager._elevator_observation.registration.latest_rgbd_door_observations = [
                mission_manager_module.replace(observation, stamp=1.2)
            ]
            self.assertTrue(
                _try_register_test_elevator(manager, (0.7, 3.0, 0.0))
            )

        registered = manager.building_client.elevators["public_lift"]
        self.assertEqual(set(registered.lobby_poses), {0, 1})
        self.assertAlmostEqual(registered.lobby_poses[0][0], 2.0)
        self.assertAlmostEqual(registered.lobby_poses[0][1], 3.0)
        self.assertAlmostEqual(abs(registered.lobby_poses[0][2]), math.pi)

    @staticmethod
    def _rgbd_elevator_registration_manager():
        """RGB-D 电梯注册的公共夹具：单电梯、双楼层、主入口位于原点。"""
        manager = _make_test_manager()
        manager.max_floor = 2
        manager.current_floor = 0
        manager.current_task = None
        manager._map_runtime.last_cloud_stamp = None
        manager._elevator_observation.registration.opening_last_stamp = None
        manager._elevator_observation.registration.opening_last_scan_stamp = None
        manager._elevator_observation.registration.opening_observation = None
        manager._elevator_observation.registration.opening_confirmations = 0
        manager._elevator_observation.registration.visual_last_stamp = None
        manager._elevator_observation.registration.visual_candidate = None
        manager._elevator_observation.registration.visual_confirmations = 0
        manager._elevator_observation.registration.latest_rgbd_door_stamp = None
        manager._elevator_observation.registration.latest_rgbd_door_observations = []
        manager.initial_entry_geometry_tolerance = 0.25
        manager.elevator_visual_confirmations = 3
        manager.elevator_visual_min_width = 0.65
        manager.elevator_visual_max_width = 1.8
        manager.elevator_visual_max_distance = 4.0
        manager.main_entrance_xy = (0.0, 0.0)
        manager.door_pass_offset = 0.8
        manager._map_runtime.mapper = SimpleNamespace(floor_height=2.6)
        manager.floor_params = {}
        elevator = PublicElevatorFeature(
            elevator_id="public_lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={elevator.elevator_id: elevator}
        )
        manager.building_client = BuildingControlClient()
        manager.building_client.set_public_features(
            door_ids={"ground_lift_door", "upper_lift_door"},
            elevators={"public_lift": {0, 1}},
            elevator_door_ids_by_floor={
                0: ["ground_lift_door"],
                1: ["upper_lift_door"],
            },
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            detect_elevator_opening_nearby=lambda **kwargs: None
        )
        return manager

    def _feed_rgbd_panel(self, manager, stamp, world_xy, outward_yaw, width):
        manager._elevator_observation.registration.latest_rgbd_door_stamp = stamp
        manager._elevator_observation.registration.latest_rgbd_door_observations = [
            DoorPanelObservation(
                stamp=stamp,
                distance=2.0,
                world_xy=world_xy,
                outward_yaw=outward_yaw,
                width=width,
                height=1.0,
                plane_rmse=0.0,
                depth_inlier_count=1200,
                bbox=(100, 80, 90, 260),
                median_depth=2.0,
                center_dark_fraction=0.95,
            )
        ]
        return _try_register_test_elevator(manager, (0.7, 1.0, 0.0))

    def test_elevator_registration_accepts_single_sliding_panel_sequence(self):
        # 复现 /tmp/stage11_elevator_fullscan.bag 的三帧移动中单扇门板：
        # 位置与朝向连续、宽度 0.657~0.727 m，必须足以注册公开唯一电梯。
        manager = self._rgbd_elevator_registration_manager()
        frames = (
            (31.328, (1.608, 2.244), 3.121, 0.727),
            (31.428, (1.612, 2.295), 3.122, 0.703),
            (31.528, (1.615, 2.371), 3.123, 0.657),
        )
        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(self._feed_rgbd_panel(manager, *frames[0]))
            self.assertFalse(self._feed_rgbd_panel(manager, *frames[1]))
            self.assertTrue(self._feed_rgbd_panel(manager, *frames[2]))

        registered = manager.building_client.elevators["public_lift"]
        self.assertEqual(set(registered.lobby_poses), {0, 1})
        self.assertAlmostEqual(registered.lobby_poses[0][0], 1.615)
        self.assertAlmostEqual(registered.lobby_poses[0][1], 2.371)
        self.assertAlmostEqual(registered.lobby_poses[0][2], 3.123)
        self.assertEqual(
            set(manager.building_client.doors),
            {"ground_lift_door", "upper_lift_door"},
        )

    def test_elevator_registration_tolerates_short_intermittent_rgbd_gaps(self):
        manager = self._rgbd_elevator_registration_manager()
        manager.elevator_visual_confirmation_max_gap = 0.4
        frames = (
            (31.0, (1.61, 2.24), 3.121, 0.72),
            (31.2, (1.62, 2.29), 3.122, 0.70),
            (31.4, (1.63, 2.35), 3.123, 0.68),
        )
        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(self._feed_rgbd_panel(manager, *frames[0]))
            manager._elevator_observation.registration.latest_rgbd_door_stamp = 31.1
            manager._elevator_observation.registration.latest_rgbd_door_observations = []
            self.assertFalse(
                _try_register_test_elevator(manager, (0.7, 1.0, 0.0))
            )
            self.assertFalse(self._feed_rgbd_panel(manager, *frames[1]))
            manager._elevator_observation.registration.latest_rgbd_door_stamp = 31.3
            manager._elevator_observation.registration.latest_rgbd_door_observations = []
            self.assertFalse(
                _try_register_test_elevator(manager, (0.7, 1.0, 0.0))
            )
            self.assertTrue(self._feed_rgbd_panel(manager, *frames[2]))

        self.assertEqual(manager._elevator_observation.registration.visual_max_confirmations, 3)
        self.assertEqual(
            manager._elevator_observation.registration.visual_association_diagnostics[
                "intermittent_gap_held"
            ],
            2,
        )

    def test_elevator_registration_rejects_sub_panel_dark_objects(self):
        # 主入口门框立柱实测约 0.35~0.45 m 宽，仍须低于单扇门板下限。
        manager = self._rgbd_elevator_registration_manager()
        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            for index in range(4):
                self.assertFalse(
                    self._feed_rgbd_panel(
                        manager,
                        1.0 + 0.1 * index,
                        (2.0, 3.0),
                        math.pi,
                        0.45,
                    )
                )
        self.assertEqual(manager.building_client.elevators, {})

    def test_elevator_registration_rejects_panel_inside_entrance_exclusion(self):
        # 几何合法但位于主入口排除半径内的门板不得注册为电梯。
        manager = self._rgbd_elevator_registration_manager()
        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            for index in range(4):
                self.assertFalse(
                    self._feed_rgbd_panel(
                        manager,
                        1.0 + 0.1 * index,
                        (0.3, 0.2),
                        math.pi,
                        0.7,
                    )
                )
        self.assertEqual(manager.building_client.elevators, {})

    def test_elevator_registration_resets_on_inconsistent_candidate_geometry(self):
        # 候选位置跳变超出容差时确认计数必须重置，避免拼接不同物体。
        manager = self._rgbd_elevator_registration_manager()
        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertFalse(
                self._feed_rgbd_panel(manager, 1.0, (2.0, 3.0), math.pi, 0.7)
            )
            self.assertFalse(
                self._feed_rgbd_panel(manager, 1.1, (2.0, 3.0), math.pi, 0.7)
            )
            self.assertFalse(
                self._feed_rgbd_panel(manager, 1.2, (3.5, 3.0), math.pi, 0.7)
            )
            self.assertEqual(manager._elevator_observation.registration.visual_confirmations, 1)
            self.assertFalse(
                self._feed_rgbd_panel(manager, 1.3, (3.5, 3.0), math.pi, 0.7)
            )
            self.assertTrue(
                self._feed_rgbd_panel(manager, 1.4, (3.5, 3.0), math.pi, 0.7)
            )

        registered = manager.building_client.elevators["public_lift"]
        self.assertAlmostEqual(registered.lobby_poses[0][0], 3.5)

    def test_elevator_geometry_refresh_updates_only_runtime_semantic_geometry(self):
        manager = self._rgbd_elevator_registration_manager()
        manager.elevator_geometry_refresh_timeout = 6.0
        manager.elevator_geometry_refresh_max_shift = 0.75
        manager.elevator_geometry_refresh_max_yaw_change = 0.35
        manager._elevator_observation.registration.refresh_context = None
        manager._elevator_observation.registration.refresh_started_at = None
        manager._elevator_observation.registration.refresh_last_stamp = None
        manager._elevator_observation.registration.refresh_candidate = None
        manager._elevator_observation.registration.refresh_confirmations = 0
        manager._elevator_observation.registration.refresh_rejections = {}
        structured = []
        manager._log_structured = structured.append
        elevator_feature = manager.building_features.elevator_features["public_lift"]
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        ).floor(0)
        grid_snapshot = np.array(grid.data, copy=True)
        manager._map_runtime.mapper = SimpleNamespace(
            floor_height=2.6,
            floor=lambda _floor: grid,
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logerr=lambda *args, **kwargs: None,
            ),
        ):
            _register_test_elevator_geometry(manager,
                elevator_feature,
                current_door_id="ground_lift_door",
                world_xy=(1.60, 2.30),
                outward_yaw=math.pi,
                width=0.70,
                source="test_initial",
                evidence_count=100,
            )
            statuses = []
            for index, center in enumerate(((1.72, 2.36), (1.73, 2.37), (1.74, 2.38))):
                stamp = 10.0 + 0.1 * index
                manager._elevator_observation.registration.latest_rgbd_door_stamp = stamp
                manager._elevator_observation.registration.latest_rgbd_door_observations = [
                    DoorPanelObservation(
                        stamp=stamp,
                        distance=1.0,
                        world_xy=center,
                        outward_yaw=math.pi - 0.02,
                        width=0.71,
                        height=1.0,
                        plane_rmse=0.0,
                        depth_inlier_count=1200,
                        bbox=(100, 80, 90, 260),
                        median_depth=1.0,
                        center_dark_fraction=0.95,
                    )
                ]
                with patch.object(mission_manager_module, "get_time", return_value=20.0):
                    statuses.append(
                        manager.refresh_elevator_geometry(
                            "public_lift",
                            0,
                            "ground_lift_door",
                            (0.8, 2.3, 0.0),
                        )
                    )

        self.assertEqual(statuses, [RUNNING, RUNNING, DONE])
        elevator = manager.building_client.elevators["public_lift"]
        self.assertEqual(set(elevator.lobby_poses), {0, 1})
        self.assertAlmostEqual(elevator.lobby_poses[0][0], 1.74)
        self.assertAlmostEqual(elevator.lobby_poses[1][1], 2.38)
        self.assertTrue(np.array_equal(grid.data, grid_snapshot))
        self.assertEqual(structured[-1]["event"], "elevator_geometry_refresh")
        self.assertAlmostEqual(structured[-1]["center_shift"], 0.161, places=3)

    def test_elevator_geometry_refresh_rejects_identity_jump_and_times_out(self):
        manager = self._rgbd_elevator_registration_manager()
        manager.elevator_geometry_refresh_timeout = 1.0
        manager.elevator_geometry_refresh_max_shift = 0.75
        manager.elevator_geometry_refresh_max_yaw_change = 0.35
        manager._elevator_observation.registration.refresh_context = None
        manager._elevator_observation.registration.refresh_started_at = None
        manager._elevator_observation.registration.refresh_last_stamp = None
        manager._elevator_observation.registration.refresh_candidate = None
        manager._elevator_observation.registration.refresh_confirmations = 0
        manager._elevator_observation.registration.refresh_rejections = {}
        structured = []
        manager._log_structured = structured.append
        manager._map_runtime.rgbd_door_detector = None
        elevator_feature = manager.building_features.elevator_features["public_lift"]

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logerr=lambda *args, **kwargs: None,
                logwarn=lambda *args, **kwargs: None,
            ),
        ):
            _register_test_elevator_geometry(manager,
                elevator_feature,
                current_door_id="ground_lift_door",
                world_xy=(1.60, 2.30),
                outward_yaw=math.pi,
                width=0.70,
                source="test_initial",
                evidence_count=100,
            )
            manager._elevator_observation.registration.latest_rgbd_door_stamp = 10.0
            manager._elevator_observation.registration.latest_rgbd_door_observations = [
                DoorPanelObservation(
                    stamp=10.0,
                    distance=1.0,
                    world_xy=(3.0, 2.3),
                    outward_yaw=math.pi,
                    width=0.70,
                    height=1.0,
                    plane_rmse=0.0,
                    depth_inlier_count=1200,
                    bbox=(100, 80, 90, 260),
                    median_depth=1.0,
                    center_dark_fraction=0.95,
                )
            ]
            with patch.object(mission_manager_module, "get_time", return_value=20.0):
                self.assertEqual(
                    manager.refresh_elevator_geometry(
                        "public_lift", 0, "ground_lift_door", (0.8, 2.3, 0.0)
                    ),
                    RUNNING,
                )
            with patch.object(mission_manager_module, "get_time", return_value=21.1):
                self.assertEqual(
                    manager.refresh_elevator_geometry(
                        "public_lift", 0, "ground_lift_door", (0.8, 2.3, 0.0)
                    ),
                    IDLE,
                )

        elevator = manager.building_client.elevators["public_lift"]
        self.assertAlmostEqual(elevator.lobby_poses[0][0], 1.60)
        self.assertEqual(
            structured[-1]["event"], "elevator_geometry_refresh_unavailable"
        )
        self.assertEqual(structured[-1]["rejections"]["center_shift"], 1)

    def test_elevator_trace_candidates_stay_on_hall_side_and_traversable(self):
        manager = self._rgbd_elevator_registration_manager()
        elevator_feature = manager.building_features.elevator_features["public_lift"]
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        ).floor(0)
        for point in ((0.2, 2.3), (0.6, 2.3), (1.2, 2.3), (2.8, 2.3)):
            gx, gy = grid.world_to_grid(*point)
            grid.set_base_state(gx, gy, VISITED)
        manager._map_runtime.mapper = SimpleNamespace(floor=lambda _floor: grid)
        manager.elevator_visual_max_distance = 4.0
        manager._floor_navigation_traces = {
            0: [(0.2, 2.3), (0.6, 2.3), (1.2, 2.3), (2.8, 2.3)]
        }

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            _register_test_elevator_geometry(manager,
                elevator_feature,
                current_door_id="ground_lift_door",
                world_xy=(1.6, 2.3),
                outward_yaw=math.pi,
                width=0.70,
                source="test_initial",
                evidence_count=100,
            )

        candidates = manager.elevator_approach_trace_candidates(
            "public_lift", 0, max_candidates=4
        )

        self.assertEqual(candidates, [(0.6, 2.3)])

    def test_elevator_start_retreat_uses_only_initial_occ_inflation_and_trace(self):
        manager = _make_test_manager()
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        grid.fill_base(FREE)
        start = grid.world_to_grid(2.0, 2.0)
        for offset in range(3):
            grid.mark_occupied(start[0] - offset, start[1])
        manager._map_runtime.mapper = SimpleNamespace(floor=lambda _floor: grid)
        manager._floor_navigation_traces = {
            1: [(1.0, 2.0), (1.8, 2.0), (2.0, 2.0)]
        }
        manager.no_frontier_rotation_min_clearance = 0.40
        manager.no_frontier_retreat_clearance_margin = 0.10

        target = manager.elevator_start_retreat_target((2.0, 2.0, 0.0), 1)

        self.assertEqual(target, (1.0, 2.0))
        self.assertEqual(int(grid.data[start[1], start[0]]), OCC)

    def test_elevator_start_retreat_rejects_safety_cells_and_occ_reentry(self):
        manager = _make_test_manager()
        grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(1)
        grid.fill_base(FREE)
        manager._map_runtime.mapper = SimpleNamespace(floor=lambda _floor: grid)
        manager._floor_navigation_traces = {1: [(1.0, 2.0), (2.0, 2.0)]}
        manager.no_frontier_rotation_min_clearance = 0.40
        manager.no_frontier_retreat_clearance_margin = 0.10
        start = grid.world_to_grid(2.0, 2.0)
        reentry = grid.world_to_grid(1.5, 2.0)

        grid.mark_black(*start, owner="test:elevator_retreat")
        self.assertIsNone(
            manager.elevator_start_retreat_target((2.0, 2.0, 0.0), 1)
        )

        grid.clear_black(*start, owner="test:elevator_retreat")
        grid.mark_occupied(*start)
        grid.mark_occupied(*reentry)
        self.assertIsNone(
            manager.elevator_start_retreat_target((2.0, 2.0, 0.0), 1)
        )

    def test_visual_preparation_closes_initially_open_elevator_only_once(self):
        manager = _make_test_manager()
        manager._map_runtime.rgbd_door_detector = object()
        manager.max_floor = 2
        manager.current_floor = 0
        manager._elevator_observation.visual_prepare_task = None
        manager._elevator_observation.visual_prepare_door_id = None
        manager._elevator_observation.visual_prepare_result_logged = False
        manager.building_features = SimpleNamespace(
            doors={
                "lift_0": PublicDoorFeature(
                    door_id="lift_0",
                    kind="elevator",
                    floor_index=0,
                    initial_open=True,
                )
            }
        )
        close_calls = []
        task = SimpleNamespace(
            done=lambda: False,
            ok=lambda: True,
            message=lambda: "closing",
        )
        manager.building_client = SimpleNamespace(
            door_service_ready=lambda: True,
            close_door=lambda door_id: close_calls.append(door_id) or task,
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager.start_elevator_visual_preparation()
            manager.start_elevator_visual_preparation()

        self.assertEqual(close_calls, ["lift_0"])
        self.assertTrue(manager.elevator_visual_preparation_pending("lift_0"))

    def test_visual_landmark_scan_keeps_observing_after_one_rotation_until_timeout(self):
        manager = _make_test_manager()
        manager._map_runtime.rgbd_door_detector = SimpleNamespace(
            diagnostics=lambda: {
                "frames": 5,
                "last": {"rejected_panel_width": 1},
                "cumulative": {"rejected_panel_width": 3},
                "last_observation_count": 0,
            }
        )
        manager.current_floor = 0
        manager.current_task = None
        manager._elevator_observation.visual_scan_started = False
        manager._elevator_observation.visual_scan_completed = False
        manager._elevator_observation.visual_scan_until = 0.0
        manager._elevator_observation.visual_scan_last_yaw = None
        manager._elevator_observation.visual_scan_rotation = 0.0
        manager.elevator_visual_scan_duration = 18.0
        manager.elevator_visual_scan_angular_speed = 0.7
        manager.elevator_visual_scan_required_rotation = 5.8
        manager._door_navigation.last_door_id = "main"
        manager._door_navigation.last_door_xy = (0.0, 0.0)
        manager._post_door_context_satisfied = lambda pose: True
        manager._elevator_observation.visual_prepare_task = SimpleNamespace(
            done=lambda: True,
            ok=lambda: True,
        )
        elevator = PublicElevatorFeature(
            elevator_id="lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={"lift": elevator},
            doors={
                "main": PublicDoorFeature("main", "main_entrance", 0, True)
            },
        )
        manager.building_client = SimpleNamespace(elevators={})
        manager.current_frontier = (1.0, 2.0)
        manager._exploration.last_frontier_target = (1.0, 2.0)
        manager._exploration.last_frontier_score = 3.0
        manager._exploration.last_frontier_used_fallback = True
        manager._elevator_observation.registration.latest_rgbd_door_observations = []
        manager._elevator_observation.registration.visual_candidate = None
        manager._elevator_observation.registration.visual_confirmations = 0
        diagnostics = []
        manager._log_structured = diagnostics.append
        calls = []
        manager.controller = SimpleNamespace(
            clear_path=lambda reason=None: calls.append(("clear", reason)),
            rotate_in_place=lambda speed: calls.append(("rotate", speed)),
            stop=lambda reason=None: calls.append(("stop", reason)),
            poll_status=lambda: 'RUNNING',
        )

        def tick_visual_scan(now, pose):
            result = manager._elevator_observation.tick_visual_scan(
                now=now,
                pose=pose,
                current_floor=manager.current_floor,
                current_task_present=manager.current_task is not None,
                building_features=manager.building_features,
                building_client=manager.building_client,
                rgbd_detector=manager.rgbd_door_detector,
                controller=manager.controller,
                last_door_id=manager._door_navigation.last_door_id,
                last_door_xy=manager._door_navigation.last_door_xy,
                post_door_context_satisfied=manager._post_door_context_satisfied(
                    pose
                ),
                config=manager._elevator_visual_scan_config(),
            )
            manager._apply_elevator_visual_scan_result(result)
            return result.handled

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertTrue(tick_visual_scan(10.0, (0.0, 2.0, 2.0)))
            self.assertTrue(tick_visual_scan(11.0, (0.0, 2.0, -2.0)))
            self.assertTrue(tick_visual_scan(12.0, (0.0, 2.0, 0.0)))
            self.assertTrue(tick_visual_scan(13.0, (0.0, 2.0, 1.6)))
            self.assertFalse(tick_visual_scan(28.1, (0.0, 2.0, 1.6)))

        self.assertEqual(sum(name == "clear" for name, _ in calls), 1)
        self.assertEqual(sum(name == "rotate" for name, _ in calls), 4)
        self.assertEqual(sum(name == "stop" for name, _ in calls), 1)
        self.assertTrue(manager._elevator_observation.visual_scan_completed)
        self.assertEqual(diagnostics[-1]["event"], "elevator_visual_scan_complete")
        self.assertEqual(diagnostics[-1]["result"], "landmark_not_found")
        self.assertEqual(
            diagnostics[-1]["detector"]["cumulative"]["rejected_panel_width"],
            3,
        )

    def test_stage25_clockwise_visual_scan_and_accumulation(self):
        """STAGE25：顺时针快速扫描下发负角速度，并正确累计顺时针（负向）转角。"""
        manager = _make_test_manager()
        manager._map_runtime.rgbd_door_detector = SimpleNamespace(
            diagnostics=lambda: {
                "frames": 5,
                "last": {"rejected_panel_width": 1},
                "cumulative": {"rejected_panel_width": 3},
                "last_observation_count": 0,
            }
        )
        manager.current_floor = 0
        manager.current_task = None
        manager._elevator_observation.visual_scan_started = False
        manager._elevator_observation.visual_scan_completed = False
        manager._elevator_observation.visual_scan_until = 0.0
        manager._elevator_observation.visual_scan_last_yaw = None
        manager._elevator_observation.visual_scan_rotation = 0.0
        manager.elevator_visual_scan_duration = 18.0
        manager.elevator_visual_scan_angular_speed = 0.90
        manager.elevator_visual_scan_required_rotation = 5.80
        manager.elevator_visual_scan_clockwise = True
        manager._door_navigation.last_door_id = "main"
        manager._door_navigation.last_door_xy = (0.0, 0.0)
        manager._post_door_context_satisfied = lambda pose: True
        manager._elevator_observation.visual_prepare_task = SimpleNamespace(
            done=lambda: True,
            ok=lambda: True,
        )
        elevator = PublicElevatorFeature(
            elevator_id="lift",
            current_floor=0,
            served_floors=frozenset({0, 1}),
        )
        manager.building_features = SimpleNamespace(
            elevator_features={"lift": elevator},
            doors={
                "main": PublicDoorFeature("main", "main_entrance", 0, True)
            },
        )
        manager.building_client = SimpleNamespace(elevators={})
        manager._elevator_observation.registration.latest_rgbd_door_observations = []
        manager._elevator_observation.registration.visual_candidate = None
        manager._elevator_observation.registration.visual_confirmations = 0
        diagnostics = []
        manager._log_structured = diagnostics.append
        calls = []
        manager.controller = SimpleNamespace(
            clear_path=lambda reason=None: calls.append(("clear", reason)),
            rotate_in_place=lambda speed: calls.append(("rotate", speed)),
            stop=lambda reason=None: calls.append(("stop", reason)),
            poll_status=lambda: "RUNNING",
        )

        def tick(now, yaw):
            return manager._elevator_observation.tick_visual_scan(
                now=now,
                pose=(0.0, 1.4, yaw),
                current_floor=manager.current_floor,
                current_task_present=manager.current_task is not None,
                building_features=manager.building_features,
                building_client=manager.building_client,
                rgbd_detector=manager.rgbd_door_detector,
                controller=manager.controller,
                last_door_id=manager._door_navigation.last_door_id,
                last_door_xy=manager._door_navigation.last_door_xy,
                post_door_context_satisfied=True,
                config=manager._elevator_visual_scan_config(),
            )

        # 第 1 帧：启动扫描，下发负角速度 -0.90 rad/s
        res1 = tick(10.0, 2.0)
        self.assertTrue(res1.handled)
        self.assertEqual(calls[-1], ("rotate", -0.90))
        self.assertAlmostEqual(manager._elevator_observation.visual_scan_rotation, 0.0)

        # 第 2 帧：顺时针旋转 2.0 rad（yaw 从 2.0 降到 0.0，delta = -2.0）
        res2 = tick(11.0, 0.0)
        self.assertTrue(res2.handled)
        self.assertEqual(calls[-1], ("rotate", -0.90))
        self.assertAlmostEqual(manager._elevator_observation.visual_scan_rotation, 2.0)

        # 第 3 帧：反向扰动（yaw 从 0.0 上升到 0.2），不计入累计
        res3 = tick(11.5, 0.2)
        self.assertTrue(res3.handled)
        self.assertAlmostEqual(manager._elevator_observation.visual_scan_rotation, 2.0)

        # 第 4 帧：顺时针旋转至 -2.0（yaw 从 0.2 降到 -2.0，delta = -2.2）
        res4 = tick(12.5, -2.0)
        self.assertTrue(res4.handled)
        self.assertAlmostEqual(manager._elevator_observation.visual_scan_rotation, 4.2)

        # 第 5 帧：顺时针跨越 -pi 转到 2.5（delta = 2.5 - (-2.0) - 2pi = -1.7831853）
        res5 = tick(14.0, 2.5)
        self.assertTrue(res5.handled)
        self.assertGreater(manager._elevator_observation.visual_scan_rotation, 5.80)

        # 第 6 帧：超时或转角完成退出
        res6 = tick(28.1, 2.5)
        self.assertFalse(res6.handled)
        self.assertTrue(manager._elevator_observation.visual_scan_completed)

    def test_public_door_ids_require_building_service_connection(self):
        self.assertTrue(
            mission_manager_module._building_services_required(
                max_floor=1,
                public_door_ids={"main_entrance"},
            )
        )

    def test_return_home_from_inside_reuses_registered_main_entrance_task(self):
        cleared = []
        manager = _make_test_manager()
        manager.home_pose = (0.0, -3.2, math.pi / 2.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.7
        manager.main_entrance_xy = (0.0, 0.0)
        manager.main_entrance_pass_vector = (0.0, 1.0)
        manager._return_home_main_entrance_crossed = False
        manager.failed_doors = set()
        manager.current_task = None
        manager.active_door = None
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.0,
                )
            },
            get_door_path_points=lambda *args, **kwargs: (
                (0.0, 0.8),
                (0.0, -0.8),
                (0.0, -1.8),
            ),
        )
        manager.controller = SimpleNamespace(
            goal_tolerance=0.35,
            clear_path=lambda **kwargs: cleared.append(kwargs.get("reason")),
            poll_status=lambda: 'RUNNING',
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._tick_return_home((0.0, 1.0, math.pi / 2.0))

        self.assertIsInstance(manager.current_task, OpenDoorAndPass)
        self.assertEqual(manager.current_task.door_id, "main_entrance")
        self.assertEqual(manager.active_door, "main_entrance")
        self.assertEqual(cleared, ["return_home_main_entrance"])

    def test_return_home_lateral_to_main_entrance_first_plans_wait_point(self):
        paths = []
        manager = _make_test_manager()
        manager.home_pose = (0.0, -3.2, math.pi / 2.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.7
        manager.main_entrance_xy = (0.0, 0.0)
        manager.main_entrance_pass_vector = (0.0, 1.0)
        manager._return_home_main_entrance_crossed = False
        manager._return_home.entry_trace_attempted = False
        manager.failed_doors = set()
        manager.current_task = None
        manager.active_door = None
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_transition_hold_distance = 2.5
        manager.building_client = SimpleNamespace(
            doors={"main_entrance": SimpleNamespace()},
            get_door_path_points=lambda *args, **kwargs: (
                (0.0, 0.8),
                (0.0, -0.8),
                (0.0, -1.8),
            ),
        )
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=80.0,
            length=80.0,
            origin_x=-40.0,
            origin_y=-40.0,
        )
        stops = []
        manager.controller = SimpleNamespace(
            goal_tolerance=0.35,
            has_path=lambda: False,
            set_path=lambda path, **kwargs: paths.append(path),
            poll_status=lambda: "RUNNING",
            recover=lambda: None,
            stop=lambda reason=None: stops.append(reason),
        )
        planned = PlanResult(path=[(-1.7, 1.8), (0.0, 0.8)], status="reached")

        with patch.object(
            return_home_planner_module,
            "astar_plan",
            return_value=planned,
        ) as astar, patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            pose = (-1.7, 1.8, -math.pi / 2.0)
            manager._tick_return_home(pose)
            self._finish_return_home_plan(manager, pose)

        self.assertIsNone(manager.current_task)
        self.assertEqual(paths, [planned.path])
        self.assertEqual(stops, ["return_home_plan_pending"])
        self.assertEqual(astar.call_args.args[2], (0.0, 0.8))

    def test_return_home_far_inside_plans_home_before_creating_door_task(self):
        paths = []
        manager = _make_test_manager()
        manager.home_pose = (0.0, -3.2, math.pi / 2.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.7
        manager.main_entrance_xy = (0.0, 0.0)
        manager.main_entrance_pass_vector = (0.0, 1.0)
        manager._return_home_main_entrance_crossed = False
        manager._return_home.entry_trace_attempted = False
        manager.failed_doors = set()
        manager.current_task = None
        manager.active_door = None
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_transition_hold_distance = 2.5
        manager.building_client = SimpleNamespace(
            doors={"main_entrance": SimpleNamespace()},
            get_door_path_points=lambda *args, **kwargs: (
                (0.0, 0.8),
                (0.0, -0.8),
                (0.0, -1.8),
            ),
        )
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=80.0,
            length=80.0,
            origin_x=-40.0,
            origin_y=-40.0,
        )
        stops = []
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path, **kwargs: paths.append(path),
            poll_status=lambda: "RUNNING",
            recover=lambda: None,
            stop=lambda reason=None: stops.append(reason),
        )
        planned = PlanResult(path=[(0.0, 30.0), (0.0, -3.2)], status="reached")

        with patch.object(
            return_home_planner_module,
            "astar_plan",
            return_value=planned,
        ) as astar:
            with patch.object(
                mission_manager_module,
                "MissionReportingRuntime",
                SimpleNamespace(loginfo=lambda *args, **kwargs: None),
            ):
                pose = (0.0, 30.0, -math.pi / 2.0)
                manager._tick_return_home(pose)
                self._finish_return_home_plan(manager, pose)

        self.assertIsNone(manager.current_task)
        self.assertEqual(paths, [planned.path])
        self.assertEqual(stops, ["return_home_plan_pending"])
        self.assertEqual(astar.call_args.args[2], (0.0, 0.8))
        self.assertIsNone(astar.call_args.kwargs["max_iterations"])

    def test_return_home_uses_actual_floor_trace_when_live_grid_is_disconnected(self):
        paths = []
        manager = _make_test_manager()
        manager.home_pose = (3.0, 0.0, 0.0)
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.2
        manager._floor_navigation_traces = {
            0: [(3.0 - 0.5 * index, 0.0) for index in range(13)]
        }
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.25,
            width=8.0,
            length=4.0,
            origin_x=-4.0,
            origin_y=-2.0,
            visited_radius=0.45,
        )
        grid = manager.mapper.floor(0)
        for point in manager._floor_navigation_traces[0]:
            grid.mark_visited_radius(point[0], point[1], 0.30)
        barrier_cell = grid.world_to_grid(0.0, 0.0)
        self.assertIsNotNone(barrier_cell)
        barrier_x = barrier_cell[0]
        for gy in range(grid.size_y):
            grid.mark_occupied(barrier_x, gy)
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path, **kwargs: paths.append(path),
            poll_status=lambda: "running",
            recover=lambda: None,
            stop=lambda reason=None: None,
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(logwarn=lambda *args, **kwargs: None),
        ):
            pose = (-3.0, 0.0, 0.0)
            manager._tick_return_home(pose)
            self._finish_return_home_plan(manager, pose)

        self.assertEqual(len(paths), 1)
        self.assertLess(math.hypot(paths[0][0][0] + 3.0, paths[0][0][1]), 0.5)
        self.assertLess(math.hypot(paths[0][-1][0] - 3.0, paths[0][-1][1]), 0.5)
        self.assertTrue(np.all(grid.data[:, barrier_x] == OCC))

    def test_return_home_veto_changes_trace_fallback_first_segment(self):
        """起点在膨胀 OCC 内时，失败首段否决应迫使 trace 选择不同出口。"""
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
            visited_radius=0.45,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        grid.mark_occupied_radius(0.0, 0.0, 0.25)
        pose = (0.0, 0.18, -0.8)
        goal = (2.0, 1.0)
        trace = tuple(
            [
                (2.0, 1.0),
                (1.5, 1.0),
                (1.0, 1.0),
                (0.5, 1.0),
                (0.0, 1.0),
                (-0.3, 0.8),
                (-0.45, 0.6),
                (-0.35, 0.4),
                (0.0, 0.18),
            ]
        )
        start_cell = grid.world_to_grid(pose[0], pose[1])
        self.assertEqual(grid.data[start_cell[1], start_cell[0]], OCC)

        status, initial_path = compute_return_home_plan(
            copy_floor_grid(grid),
            pose,
            goal,
            trace_snapshot=trace,
            visited_radius=0.45,
        )
        self.assertEqual(status, "reached")
        initial_end = initial_path[1]

        status, vetoed_path = compute_return_home_plan(
            copy_floor_grid(grid),
            pose,
            goal,
            trace_snapshot=trace,
            visited_radius=0.45,
            veto_segments=((pose[0], pose[1], initial_end[0], initial_end[1]),),
            failed_signature=return_segment_signature(initial_end),
            veto_lookahead=0.8,
        )

        self.assertEqual(status, "reached")
        self.assertNotEqual(
            return_segment_signature(vetoed_path[1]),
            return_segment_signature(initial_end),
        )
        self.assertFalse(
            path_crosses_blocked_beyond(grid, vetoed_path, exemption=0.20)
        )

    def test_return_home_trace_candidate_is_checked_only_after_stuck_feedback(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.25,
            width=8.0,
            length=4.0,
            origin_x=-4.0,
            origin_y=-2.0,
            visited_radius=0.45,
        )
        grid = manager.mapper.floor(0)
        trace = tuple((3.0 - 0.5 * index, 0.0) for index in range(13))
        for point in trace:
            grid.mark_visited_radius(point[0], point[1], 0.30)
        barrier_x = grid.world_to_grid(0.0, 0.0)[0]
        for gy in range(grid.size_y):
            grid.mark_occupied(barrier_x, gy)
        pose = (-3.0, 0.0, 0.0)
        goal = (3.0, 0.0)

        initial_status, initial_path = compute_return_home_plan(
            copy_floor_grid(grid),
            pose,
            goal,
            trace_snapshot=trace,
            visited_radius=0.45,
        )
        self.assertEqual(initial_status, "reached")
        self.assertTrue(initial_path)

        rejected_status, rejected_path = validate_return_home_candidate(
            initial_path,
            original_grid=grid,
            failed_signature=None,
        )
        self.assertEqual(rejected_status, "candidate_rejected_blocked")
        self.assertEqual(rejected_path, [])

        vetoed_status, vetoed_path = compute_return_home_plan(
            copy_floor_grid(grid),
            pose,
            goal,
            trace_snapshot=trace,
            visited_radius=0.45,
            veto_segments=((-3.0, 0.0, -2.5, 0.0),),
            failed_signature=return_segment_signature((-2.5, 0.0)),
            veto_lookahead=0.8,
        )
        # 统一起点逃逸搜索可在首段 veto 后生成替代候选；该候选仍必须由
        # 正式地图屏障复验拒绝，原因应保留为更具体的 blocked。
        self.assertEqual(vetoed_status, "candidate_rejected_blocked")
        self.assertEqual(vetoed_path, [])

    def test_return_home_stuck_feedback_is_bounded_and_fails_closed(self):
        recoveries = []
        cleared = []
        stops = []
        transitions = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.return_home_max_stuck_retries = 4
        manager._return_home.stuck_anchor = None
        manager._return_home.stuck_count = 0
        manager._return_home.failed_segments = []
        manager._return_home.last_failed_signature = None
        manager._return_home.candidate_reject_count = 0
        manager._return_home.last_stuck_obstacle = None
        manager._mission_success_eligible = True
        manager._failure_reasons = []
        manager._map_runtime.nearest_forward_obstacle = SimpleNamespace(
            distance=0.236,
            world_xyz=(-0.27, 14.44, 1.2),
            forward=0.16,
            lateral=0.16,
            relative_height=1.2,
        )
        manager._map_runtime.nearest_radial_obstacle_distance = 1.0
        manager.controller = SimpleNamespace(
            recover=lambda reason=None: recoveries.append(reason),
            clear_path=lambda reason=None: cleared.append(reason),
            stop=lambda reason=None: stops.append(reason),
            poll_status=lambda: 'RUNNING',
        )
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )
        segment = ((-0.25, 14.65), (0.15, 14.25))

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            for _ in range(4):
                manager._handle_return_home_stuck(
                    (-0.26, 14.67, -0.81),
                    segment,
                )

        self.assertEqual(recoveries, ["return_home_stuck"] * 3)
        self.assertEqual(manager._return_home.stuck_count, 4)
        self.assertEqual(len(manager._return_home.failed_segments), 1)
        self.assertEqual(
            manager._return_home.last_failed_signature,
            return_segment_signature(segment[1]),
        )
        self.assertEqual(cleared, ["return_home_stuck_exhausted"])
        self.assertEqual(stops, ["return_home_stuck_exhausted"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "return_home_stuck_exhausted",
        )

    def test_return_home_stuck_retreats_straight_when_rotation_is_unsafe(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.return_home_max_stuck_retries = 4
        manager._return_home.stuck_anchor = None
        manager._return_home.stuck_count = 0
        manager._return_home.failed_segments = []
        manager._return_home.last_failed_signature = None
        manager._return_home.last_stuck_obstacle = None
        manager._return_home.retreat_anchor = None
        manager._return_home.retreat_target = None
        manager._return_home.planner = None
        manager._map_runtime.nearest_forward_obstacle = None
        manager._map_runtime.nearest_radial_obstacle_distance = 0.30
        manager.no_frontier_rotation_min_clearance = 0.40
        manager.no_frontier_retreat_clearance_margin = 0.10
        manager.no_frontier_retreat_max_distance = 2.0
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        manager._floor_navigation_traces = {
            0: [(2.0, 3.0), (3.0, 3.0), (4.0, 3.0)]
        }
        recoveries = []
        cleared = []
        stops = []
        reverse_calls = []
        manager.controller = SimpleNamespace(
            reverse_speed=-0.15,
            recover=lambda reason=None: recoveries.append(reason),
            clear_path=lambda reason=None: cleared.append(reason),
            stop=lambda reason=None: stops.append(reason),
            reverse_straight=lambda **kwargs: reverse_calls.append(kwargs) or True,
            poll_status=lambda: 'RUNNING',
        )
        manager._mission_diagnostic_snapshot = lambda: {}
        events = []
        manager._log_structured = events.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._handle_return_home_stuck(
                (4.0, 3.0, 0.0), ((4.0, 3.0), (5.0, 3.0))
            )
            manager._tick_return_home_retreat((4.0, 3.0, 0.0))

        self.assertEqual(recoveries, [])
        self.assertEqual(manager._return_home.retreat_target, (3.0, 3.0))
        self.assertEqual(cleared, ["return_home_retreat_started"])
        self.assertEqual(reverse_calls, [{"reason": "return_home_retreat"}])
        self.assertEqual(events[-1]["event"], "return_home_retreat_started")

        with patch.object(mission_manager_module, "get_time", return_value=11.0):
            manager._tick_return_home_retreat((3.2, 3.0, 0.0))

        self.assertIsNone(manager._return_home.retreat_target)
        self.assertEqual(stops, ["return_home_retreat_completed"])
        self.assertEqual(events[-1]["event"], "return_home_retreat_completed")

    def test_return_home_stuck_without_safe_retreat_never_rotates_in_place(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.return_home_max_stuck_retries = 4
        manager._return_home.stuck_anchor = None
        manager._return_home.stuck_count = 0
        manager._return_home.failed_segments = []
        manager._return_home.last_failed_signature = None
        manager._return_home.last_stuck_obstacle = None
        manager._return_home.retreat_anchor = None
        manager._return_home.retreat_target = None
        manager._map_runtime.nearest_forward_obstacle = None
        manager._map_runtime.nearest_radial_obstacle_distance = 0.30
        manager.no_frontier_rotation_min_clearance = 0.40
        manager.no_frontier_retreat_clearance_margin = 0.10
        manager.no_frontier_retreat_max_distance = 2.0
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        manager._floor_navigation_traces = {0: []}
        recoveries = []
        cleared = []
        stops = []
        manager.controller = SimpleNamespace(
            recover=lambda reason=None: recoveries.append(reason),
            clear_path=lambda reason=None: cleared.append(reason),
            stop=lambda reason=None: stops.append(reason),
            poll_status=lambda: 'RUNNING',
        )
        manager._mission_diagnostic_snapshot = lambda: {}
        manager._log_structured = lambda payload: None

        manager._handle_return_home_stuck(
            (2.0, 2.0, 0.0), ((2.0, 2.0), (3.0, 2.0))
        )

        self.assertEqual(recoveries, [])
        self.assertEqual(cleared, ["return_home_retreat_unavailable"])
        self.assertEqual(stops, ["return_home_retreat_unavailable"])

    def test_return_home_veto_does_not_black_goal_neighborhood(self):
        manager = _make_test_manager()
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=2.0,
            length=2.0,
            origin_x=-1.0,
            origin_y=-1.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        goal = (0.4, 0.0)

        apply_return_home_vetoes(
            grid,
            ((0.0, 0.0, 0.5, 0.0),),
            goal_xy=goal,
            lookahead=0.8,
        )

        goal_cell = grid.world_to_grid(*goal)
        self.assertNotEqual(grid.data[goal_cell[1], goal_cell[0]], BLACK)

    def test_return_home_feedback_parameters_reject_invalid_limits(self):
        def parameter(name, default=None):
            if name == "runtime/profile":
                return "diagnostic"
            if name == "mission/return_home_max_stuck_retries":
                return 0
            return default

        with self.assertRaisesRegex(ValueError, "返航卡滞"):
            load_mission_config(parameter)

    def test_floor_navigation_trace_is_sparse_and_pauses_only_during_floor_transition(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_floor = 0
        manager._floor_transition_runtime._guard = None
        manager._floor_navigation_traces = {}

        manager._record_floor_navigation_trace_pose((0.0, 0.0, 0.0))
        manager._record_floor_navigation_trace_pose((0.1, 0.0, 0.0))
        manager._record_floor_navigation_trace_pose((0.3, 0.0, 0.0))
        manager._floor_transition_runtime._guard = SimpleNamespace()
        manager._record_floor_navigation_trace_pose((0.6, 0.0, 0.0))
        manager._floor_transition_runtime._guard = None
        manager.state = RETURN_HOME
        manager._record_floor_navigation_trace_pose((0.9, 0.0, 0.0))

        self.assertEqual(
            manager._floor_navigation_traces[0],
            [(0.0, 0.0), (0.3, 0.0), (0.9, 0.0)],
        )

    def test_return_trace_keeps_fallback_attached_after_moving_from_return_start(self):
        manager = _make_test_manager()
        manager.state = RETURN_HOME
        manager.current_floor = 0
        manager._floor_transition_runtime._guard = None
        manager._floor_navigation_traces = {
            0: [(3.0 - 0.5 * index, 0.0) for index in range(13)]
        }
        manager._record_floor_navigation_trace_pose((-3.0, 0.3, 0.0))
        manager._record_floor_navigation_trace_pose((-3.0, 0.6, 0.0))
        manager._record_floor_navigation_trace_pose((-3.0, 0.9, 0.0))
        mapper = MultiFloorGridMap(
            resolution=0.25,
            width=8.0,
            length=5.0,
            origin_x=-4.0,
            origin_y=-2.0,
            visited_radius=0.45,
        )
        manager._map_runtime.mapper = mapper
        grid = mapper.floor(0)
        path = return_path_from_navigation_trace(
            grid,
            (-3.0, 0.9, 0.0),
            (3.0, 0.0),
            trace_points=tuple(manager._floor_navigation_traces[0]),
            visited_radius=mapper.visited_radius,
        )

        self.assertIsNotNone(path)
        self.assertLess(math.hypot(path[0][0] + 3.0, path[0][1] - 0.9), 0.5)
        self.assertLess(math.hypot(path[-1][0] - 3.0, path[-1][1]), 0.5)

    def test_opened_door_clear_covers_wait_point_and_inflation_on_both_sides(self):
        calls = []
        manager = _make_test_manager()
        manager.door_clear_radius = 0.9
        manager.door_pass_offset = 0.8
        manager.building_client = SimpleNamespace(
            doors={
                "main_entrance": DoorInfo(
                    "main_entrance",
                    "main_entrance",
                    0,
                    0.0,
                    0.0,
                    math.pi / 2.0,
                    1.4,
                )
            }
        )
        manager._map_runtime.mapper = SimpleNamespace(
            obstacle_inflation_radius=0.35,
            resolution=0.1,
            clear_dynamic_door_strip=lambda *args: calls.append(args),
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager.clear_opened_door_area("main_entrance")

        self.assertEqual(len(calls), 1)
        # depth 为完整长度，半深度需越过 0.8 m 等待点并覆盖 0.35 m 膨胀。
        self.assertAlmostEqual(calls[0][-1], 2.5)

    def test_initial_entry_trace_is_sparse_and_excludes_active_door_task(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager._entry.origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._entry.completed = False
        manager._entry.trace = [(0.0, -3.2)]

        manager._record_initial_entry_trace_pose((0.0, -3.1, math.pi / 2.0))
        manager._record_initial_entry_trace_pose((0.0, -2.9, math.pi / 2.0))
        manager.current_task = OpenDoorAndPass("main_entrance")
        manager._record_initial_entry_trace_pose((0.0, -2.5, math.pi / 2.0))

        self.assertEqual(manager._entry.trace, [(0.0, -3.2), (0.0, -2.9)])
        self.assertTrue(manager._entry.trace_scope_valid)
        self.assertEqual(manager._entry.trace_floor, 0)
        self.assertEqual(manager._entry.trace_map_epoch, 1)
        self.assertEqual(manager._entry.trace_localization_epoch, 1)

    def test_initial_entry_trace_scope_is_invalidated_by_continuity_change(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager._entry.origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._entry.completed = False
        manager._entry.trace = [(0.0, -3.2)]

        manager._record_initial_entry_trace_pose((0.0, -2.9, math.pi / 2.0))
        manager._ground_support.localization_epoch = 2
        manager._record_initial_entry_trace_pose((0.0, -2.6, math.pi / 2.0))

        self.assertFalse(manager._entry.trace_scope_valid)

    def test_initial_entry_realigns_locked_heading_between_forward_segments(self):
        rotations = []
        stops = []
        manager = _make_test_manager()
        manager.controller = SimpleNamespace(
            max_angular=1.2,
            rotate_in_place=lambda angular: rotations.append(angular),
            stop=lambda reason=None: stops.append(reason),
            poll_status=lambda: 'RUNNING',
        )

        self.assertTrue(
            manager._entry.align_heading(
                manager.controller,
                (0.32, -0.55, 3.05),
                math.pi / 2.0,
            )
        )
        self.assertEqual(len(rotations), 1)
        self.assertAlmostEqual(rotations[0], -0.55)
        self.assertEqual(stops, [])

        rotations.clear()
        self.assertFalse(
            manager._entry.align_heading(
                manager.controller,
                (0.32, -0.55, math.pi / 2.0 + 0.1),
                math.pi / 2.0,
            )
        )
        self.assertEqual(rotations, [])

    def test_return_home_reverses_actual_initial_entry_trace_after_main_door(self):
        paths = []
        manager = _make_test_manager()
        manager.home_pose = (0.0, -3.2, math.pi / 2.0)
        manager.state = RETURN_HOME
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_tolerance = 0.7
        manager.main_entrance_xy = (0.0, 0.0)
        manager.main_entrance_pass_vector = (0.0, 1.0)
        manager._return_home_main_entrance_crossed = True
        manager._return_home.entry_trace_attempted = False
        manager._entry.trace = [
            (0.0, -3.2),
            (0.02, -2.8),
            (0.01, -2.4),
            (0.03, -1.1),
        ]
        manager._entry.trace_floor = 0
        manager._entry.trace_map_epoch = 1
        manager._entry.trace_localization_epoch = 1
        manager._entry.trace_scope_valid = True
        manager.building_client = SimpleNamespace(doors={})
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path, **kwargs: paths.append(path),
            poll_status=lambda: 'RUNNING',
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            manager._tick_return_home((0.0, -0.8, -math.pi / 2.0))

        self.assertTrue(manager._return_home.entry_trace_attempted)
        self.assertEqual(len(paths), 1)
        self.assertEqual(paths[0][0], (0.0, -0.8))
        self.assertEqual(paths[0][-1], (0.0, -3.2))
        self.assertEqual(paths[0][1:-1], [(0.03, -1.1), (0.01, -2.4), (0.02, -2.8)])
        self.assertIsNotNone(manager._return_home.entry_trace_scope)

    def test_door_navigation_accepts_visible_dynamic_candidate(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager._map_runtime.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="main_entrance",
            distance=1.2,
            visible=True,
            kind="dynamic_door",
            floor_index=0,
            world_xy=(0.0, 0.0),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertTrue(_door_candidate_decision(manager, candidate).allowed)

    def test_door_navigation_rejects_elevator_candidate(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_task = None
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager._map_runtime.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: True,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="elevator_floor_0",
            distance=1.2,
            visible=True,
            kind="elevator",
            floor_index=0,
            world_xy=(1.0, 2.6),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertFalse(_door_candidate_decision(manager, candidate).allowed)

    def test_door_navigation_does_not_interrupt_current_task(self):
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.current_task = object()
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager._map_runtime.local_detector = SimpleNamespace(max_distance=3.0)
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
        )
        candidate = DoorCandidate(
            door_id="main_entrance",
            distance=1.2,
            visible=True,
            kind="dynamic_door",
            floor_index=0,
            world_xy=(0.0, 0.0),
            source="local_pointcloud",
            evidence_count=8,
        )

        self.assertFalse(_door_candidate_decision(manager, candidate).allowed)

    def test_probe_unknown_space_starts_from_current_pose_after_passing_door(self):
        recorded_paths = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.home_pose = (0.0, -2.2, math.pi / 2.0)
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._exploration.probe_attempt = 0
        manager.current_frontier = object()
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.controller = SimpleNamespace(set_path=lambda path, **kwargs: recorded_paths.append(path))

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(loginfo=lambda *args, **kwargs: None)):
            self.assertTrue(manager._try_probe_unknown_space((0.0, 0.8, math.pi / 2.0)))

        self.assertEqual(len(recorded_paths), 1)
        self.assertAlmostEqual(recorded_paths[0][-1][0], 0.0, places=6)
        self.assertAlmostEqual(recorded_paths[0][-1][1], 2.0, places=6)

    def test_probe_unknown_space_uses_current_pose_heading(self):
        """短距离探测应沿当前机身朝向，不依赖固定世界方向参数。"""
        recorded_paths = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._exploration.probe_attempt = 0
        manager.current_frontier = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.controller = SimpleNamespace(set_path=lambda path, **kwargs: recorded_paths.append(path))

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            self.assertTrue(manager._try_probe_unknown_space((0.0, 0.0, 0.0)))

        self.assertEqual(len(recorded_paths), 1)
        self.assertAlmostEqual(recorded_paths[0][-1][0], 1.2, places=6)
        self.assertAlmostEqual(recorded_paths[0][-1][1], 0.0, places=6)

    def test_probe_unknown_space_skips_targets_behind_known_wall(self):
        recorded_paths = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.home_pose = (0.0, -2.2, math.pi / 2.0)
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._exploration.probe_attempt = 0
        manager.current_frontier = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        grid = manager.mapper.floor(0)
        grid.mark_occupied(*grid.world_to_grid(0.0, 1.4))
        manager.controller = SimpleNamespace(set_path=lambda path, **kwargs: recorded_paths.append(path))

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(loginfo=lambda *args, **kwargs: None)):
            self.assertTrue(manager._try_probe_unknown_space((0.0, 0.8, math.pi / 2.0)))

        self.assertEqual(len(recorded_paths), 1)
        self.assertNotAlmostEqual(recorded_paths[0][-1][0], 0.0, places=6)

    def test_return_home_does_not_direct_fallback_through_blocked_wall(self):
        recorded_paths = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_pose = (2.0, 0.0, 0.0)
        manager.home_tolerance = 0.2
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        wall_x, _ = grid.world_to_grid(0.0, 0.0)
        for gy in range(grid.size_y):
            grid.mark_occupied(wall_x, gy)
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path, **kwargs: recorded_paths.append(path),
            poll_status=lambda: "idle",
            recover=lambda: None,
            stop=lambda reason=None: None,
        )

        pose = (-2.0, 0.0, 0.0)
        manager._tick_return_home(pose)
        self._finish_return_home_plan(manager, pose)

        self.assertEqual(recorded_paths, [])

    def test_door_navigation_records_completed_transition_context(self):
        manager = _make_test_manager()
        manager.door_transition_duration = 3.0
        manager._door_navigation.frontier_debug = None
        manager._door_navigation.last_door_id = None
        manager._door_navigation.last_door_xy = None
        manager._door_navigation.last_pass_vector = None
        manager._door_navigation.transition_until = 0.0
        task = OpenDoorAndPass("main_entrance")
        task._wait_point = (0.0, -0.8)
        task._pass_point = (0.0, 0.8)
        task._commit_point = (0.0, 1.8)

        before = mission_manager_module.get_time()
        manager._door_navigation.record_completed(
            task,
            (0.0, 1.75, math.pi / 2.0),
            now=before,
            transition_duration=manager.door_transition_duration,
            context_duration=manager.post_door_context_max_duration,
        )

        self.assertEqual(manager._door_navigation.last_door_id, "main_entrance")
        self.assertEqual(manager._door_navigation.last_door_xy, (0.0, 0.0))
        self.assertEqual(manager._door_navigation.last_pass_vector, (0.0, 1.0))
        self.assertGreater(manager._door_navigation.transition_until, before)
        self.assertEqual(
            manager._door_navigation.frontier_debug["commit_point"],
            (0.0, 1.8),
        )

    def test_frontier_selection_context_only_active_near_door_and_before_timeout(self):
        manager = _make_test_manager()
        manager._door_navigation.last_door_xy = (0.0, 0.0)
        manager._door_navigation.last_pass_vector = (0.0, 1.0)
        manager._door_navigation.transition_until = 105.0
        manager._door_navigation.context_until = 105.0
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager._exploration.last_frontier_target = None
        manager.explorer = SimpleNamespace(
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )

        with patch.object(mission_manager_module, "get_time", return_value=103.0):
            context = manager._frontier_selection_context((0.0, 1.0, math.pi / 2.0))
            far_context = manager._frontier_selection_context((0.0, 3.0, math.pi / 2.0))

        self.assertIsNotNone(context)
        assert context is not None
        self.assertEqual(context.forbidden_half_plane_origin, (0.0, 0.0))
        self.assertEqual(context.forbidden_half_plane_normal, (0.0, 1.0))
        self.assertEqual(context.forbidden_radius, 1.5)
        self.assertTrue(context.skip_in_forbidden_half_plane)
        self.assertEqual(context.temporary_revisit_penalty, 6.0)
        self.assertIsNone(far_context)

        with patch.object(mission_manager_module, "get_time", return_value=106.0):
            expired_context = manager._frontier_selection_context((0.0, 1.0, math.pi / 2.0))

        self.assertIsNone(expired_context)

    def test_frontier_selection_context_persists_after_door_timer_until_post_door_progress(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._door_navigation.last_door_xy = (0.0, 0.0)
        manager._door_navigation.last_pass_vector = (0.0, 1.0)
        manager._door_navigation.transition_until = 100.0
        manager._door_navigation.context_until = 110.0
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 2.0
        manager.post_door_min_known_area = 4.0
        manager._exploration.last_frontier_target = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.explorer = SimpleNamespace(
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )

        with patch.object(mission_manager_module, "get_time", return_value=106.0):
            context = manager._frontier_selection_context((0.0, 0.8, math.pi / 2.0))

        self.assertIsNotNone(context)
        assert context is not None
        self.assertTrue(context.skip_in_forbidden_half_plane)

    def test_post_door_context_has_absolute_expiry(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager._door_navigation.last_door_id = "main_entrance"
        manager._door_navigation.last_door_xy = (0.0, 0.0)
        manager._door_navigation.last_pass_vector = (0.0, 1.0)
        manager._door_navigation.transition_until = 100.0
        manager._door_navigation.context_until = 105.0
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 5.0
        manager.post_door_min_known_area = 4.0
        manager._exploration.last_frontier_target = None
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        manager.explorer = SimpleNamespace(
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )

        with patch.object(mission_manager_module, "get_time", return_value=106.0):
            context = manager._frontier_selection_context((0.0, 0.8, math.pi / 2.0))

        self.assertIsNone(context)

    def test_permanent_door_radius_scales_with_entrance_width(self):
        manager = _make_test_manager()
        manager.explorer = SimpleNamespace(door_front_half_plane_radius=1.5)
        manager.building_client = SimpleNamespace(
            doors={"main_entrance": SimpleNamespace(width=2.0)}
        )
        manager.permanent_forbidden_radius_scale = 1.5
        manager.permanent_forbidden_decay_scale = 3.0

        radius, decay, gate_half_width = manager._permanent_door_context_parameters()

        self.assertEqual(radius, 3.0)
        self.assertEqual(decay, 6.0)
        self.assertEqual(gate_half_width, 1.35)

    def test_frontier_selection_context_keeps_post_door_progress_beyond_hold_distance(self):
        manager = _make_test_manager()
        manager._door_navigation.last_door_xy = (0.0, 0.0)
        manager._door_navigation.last_pass_vector = (0.0, 1.0)
        manager._door_navigation.transition_until = 105.0
        manager._door_navigation.context_until = 105.0
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 4.0
        manager.post_door_min_known_area = 0.0
        manager._exploration.last_frontier_target = None
        manager.explorer = SimpleNamespace(
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )

        with patch.object(mission_manager_module, "get_time", return_value=103.0):
            context = manager._frontier_selection_context((0.0, 3.0, math.pi / 2.0))

        self.assertIsNotNone(context)
        assert context is not None
        self.assertTrue(context.skip_in_forbidden_half_plane)

    def test_small_known_area_with_high_coverage_does_not_complete_floor(self):
        manager = _make_test_manager()
        manager.min_floor_explore_time = 20.0
        manager.min_floor_travel_distance = 6.0
        manager.min_known_area = 45.0
        manager.explore_coverage_ratio = 0.82
        manager.no_reachable_frontier_confirmations = 2
        manager._exploration.no_reachable_frontier_confirmations = 2
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager._max_home_distance = 8.0
        grid = MultiFloorGridMap(
            resolution=1.0,
            width=10.0,
            length=10.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        for gy in range(2):
            for gx in range(2):
                grid.mark_free(gx, gy)

        self.assertEqual(grid.coverage_ratio(), 1.0)
        self.assertLess(grid.known_area(), manager.min_known_area)
        self.assertFalse(manager._floor_completion_ready(grid, 30.0))

    def test_floor_completion_requires_repeated_no_reachable_confirmations(self):
        manager = _make_test_manager()
        manager.min_floor_explore_time = 20.0
        manager.min_floor_travel_distance = 6.0
        manager.min_known_area = 4.0
        manager.explore_coverage_ratio = 0.82
        manager.no_reachable_frontier_confirmations = 2
        manager._exploration.no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager._max_home_distance = 8.0
        grid = MultiFloorGridMap(
            resolution=1.0,
            width=10.0,
            length=10.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        for gy in range(3):
            for gx in range(3):
                grid.mark_free(gx, gy)

        self.assertFalse(manager._floor_completion_ready(grid, 30.0))
        manager._exploration.no_reachable_frontier_confirmations = 2
        self.assertTrue(manager._floor_completion_ready(grid, 30.0))


    def test_door_approach_selects_nearest_reachable_unopened_door(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager._door_navigation.approach_blacklist = {}
        manager.door_pass_offset = 0.8
        manager.active_door = None
        manager.current_task = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=12.0,
            length=12.0,
            origin_x=-6.0,
            origin_y=-6.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)

        def get_door_points(door_id, robot_pose, offset=0.8):
            # 两个门：near 在东侧，far 在西侧
            if door_id == "near_door":
                door_xy = (3.0, 0.0)
            else:
                door_xy = (-4.0, 0.0)
            # wait_point 在 door 北侧，简单示意
            return (door_xy[0], door_xy[1] + offset), (door_xy[0], door_xy[1] - offset)

        manager.building_client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                SimpleNamespace(door_id="far_door", floor_index=0),
                SimpleNamespace(door_id="near_door", floor_index=0),
            ],
            get_door_points=get_door_points,
        )

        selected = _select_door_approach(manager, (0.0, 0.0, 0.0))
        self.assertEqual(selected, "near_door")

    def test_door_approach_skips_unreachable_doors(self):
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager._door_navigation.approach_blacklist = {}
        manager.door_pass_offset = 0.8
        manager.active_door = None
        manager.current_task = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5,
            width=12.0,
            length=12.0,
            origin_x=-6.0,
            origin_y=-6.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        # 在 x=0 处竖墙，把机器人（x<0）和门（x>0）隔开
        wall_x = grid.world_to_grid(0.0, 0.0)[0]
        for gy in range(grid.size_y):
            grid.mark_occupied(wall_x, gy)

        def get_door_points(door_id, robot_pose, offset=0.8):
            return ((3.0, 0.0 + offset), (3.0, 0.0 - offset))

        manager.building_client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                SimpleNamespace(door_id="blocked_door", floor_index=0),
            ],
            get_door_points=get_door_points,
        )

        selected = _select_door_approach(manager, (-3.0, 0.0, 0.0))
        self.assertIsNone(selected)
        self.assertIn("blocked_door", manager._door_navigation.approach_blacklist)

    def test_stuck_far_from_frontier_blacklists_region_without_blackmark(self):
        """远处卡住时只临时排除目标区域，不把远处目标写成地图障碍。"""
        recorded_failures: list[tuple[float, float]] = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.manual_mode = False
        manager.start_time = 0.0
        manager.floor_start_time = 0.0
        manager.floor_timeout = 1000.0
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager.no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager._max_home_distance = 0.0
        manager._exploration.no_frontier_scan_until = 0.0
        manager._exploration.no_reachable_frontier_confirmations = 0
        manager._exploration.probe_attempt = 0
        manager._exploration.last_frontier_target = None
        manager._exploration.last_frontier_score = None
        manager._exploration.last_frontier_used_fallback = False
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_clear_radius = 0.9
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.current_task = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5, width=20.0, length=20.0, origin_x=-10.0, origin_y=-10.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        # frontier 在远处，机器人在原点，距离 10m > 1.5m
        manager.current_frontier = SimpleNamespace(centroid=(10.0, 0.0))
        manager.explorer = SimpleNamespace(
            extract_frontiers=lambda g: [],
            record_stuck_failure=lambda g, xy: recorded_failures.append(xy),
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            max_distance=3.0,
            detect_controllable_door=lambda **kw: None,
        )
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
            controllable_doors=lambda **kw: [],
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: True,
            poll_status=lambda: STUCK,
            recover=lambda: None,
            clear_path=lambda: None,
            stop=lambda *a, **k: None,
        )
        manager._door_navigation.approach_blacklist = {}
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager._door_navigation.last_door_xy = None
        manager._door_navigation.last_pass_vector = None
        manager._door_navigation.transition_until = 0.0
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager._door_navigation.frontier_debug = None
        manager._map_runtime.latest_cloud_points = []
        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(recorded_failures), 1)
        self.assertEqual(recorded_failures[0], (10.0, 0.0))
        self.assertFalse(np.any(grid.data == BLACK))

    def test_stuck_near_frontier_only_blacklists_region_without_trapping_robot(self):
        """终点附近卡住也只排除 frontier，不能把当前位置写成永久禁区。"""
        recorded_failures: list[tuple[float, float]] = []
        black_marks: list[tuple] = []
        recover_calls: list[str] = []
        stop_reasons: list[str | None] = []
        events: list[dict] = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.manual_mode = False
        manager.start_time = 0.0
        manager.floor_start_time = 0.0
        manager.floor_timeout = 1000.0
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager.no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager._max_home_distance = 0.0
        manager._exploration.no_frontier_scan_until = 0.0
        manager._exploration.no_reachable_frontier_confirmations = 0
        manager._exploration.probe_attempt = 0
        manager._exploration.last_frontier_target = None
        manager._exploration.last_frontier_score = None
        manager._exploration.last_frontier_used_fallback = False
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_clear_radius = 0.9
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.current_task = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5, width=20.0, length=20.0, origin_x=-10.0, origin_y=-10.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        # 复现实测：frontier 距机器人 0.4m，小于旧实现 0.6m 的 BLACK 半径。
        manager.current_frontier = SimpleNamespace(centroid=(0.4, 0.0))
        manager.explorer = SimpleNamespace(
            extract_frontiers=lambda g: [],
            record_stuck_failure=lambda g, xy: recorded_failures.append(xy),
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            max_distance=3.0,
            detect_controllable_door=lambda **kw: None,
        )
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
            controllable_doors=lambda **kw: [],
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: True,
            poll_status=lambda: STUCK,
            recover=lambda: recover_calls.append("recover"),
            clear_path=lambda: None,
            stop=lambda *a, **k: stop_reasons.append(k.get("reason")),
        )
        manager._door_navigation.approach_blacklist = {}
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager._door_navigation.last_door_xy = None
        manager._door_navigation.last_pass_vector = None
        manager._door_navigation.transition_until = 0.0
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager._door_navigation.frontier_debug = None
        manager._map_runtime.latest_cloud_points = []
        manager.no_frontier_rotation_min_clearance = 0.40
        manager._map_runtime.nearest_radial_obstacle_distance = 0.30
        manager._exploration.no_frontier_rotation_blocked_logged = False
        manager._log_structured = events.append
        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(recorded_failures), 1)
        self.assertEqual(recorded_failures[0], (0.4, 0.0))
        self.assertFalse(np.any(grid.data == BLACK))
        self.assertTrue(grid.is_traversable(*grid.world_to_grid(0.0, 0.0)))
        self.assertTrue(astar_plan(grid, (0.0, 0.0), (2.0, 0.0)).reached)
        self.assertEqual(recover_calls, [])
        self.assertEqual(stop_reasons, ["unsafe_recover_rotation_blocked", "no_frontier_retreat_unavailable"])
        self.assertEqual(events[0]["event"], "unsafe_rotation_blocked")
        self.assertEqual(events[0]["source"], "stuck_recovery")

    def test_stuck_with_trace_initiates_retreat(self):
        """贴墙旋转不安全卡住时，若存在安全历史轨迹应直接启动直线后退脱困。"""
        events: list[dict] = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.manual_mode = False
        manager.start_time = 0.0
        manager.floor_start_time = 0.0
        manager.floor_timeout = 1000.0
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager.no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = 0.0
        manager._max_home_distance = 0.0
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1, width=20.0, length=20.0, origin_x=-10.0, origin_y=-10.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        manager._floor_navigation_traces = {0: [(-0.6, 0.0), (0.0, 0.0)]}
        manager.current_frontier = SimpleNamespace(centroid=(0.4, 0.0))
        manager.explorer = SimpleNamespace(
            extract_frontiers=lambda g: [],
            record_stuck_failure=lambda g, xy: None,
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
        )
        manager._map_runtime.local_detector = SimpleNamespace(
            max_distance=3.0,
            detect_controllable_door=lambda **kw: None,
        )
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
            controllable_doors=lambda **kw: [],
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: True,
            poll_status=lambda: STUCK,
            recover=lambda: None,
            clear_path=lambda: None,
            stop=lambda *a, **k: None,
            reverse_speed=-0.15,
        )
        manager._door_navigation.approach_blacklist = {}
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager._door_navigation.last_door_xy = None
        manager._door_navigation.last_pass_vector = None
        manager._door_navigation.transition_until = 0.0
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager._door_navigation.frontier_debug = None
        manager._map_runtime.latest_cloud_points = []
        manager.no_frontier_rotation_min_clearance = 0.40
        manager._map_runtime.nearest_radial_obstacle_distance = 0.30
        manager._exploration.no_frontier_rotation_blocked_logged = False
        manager._log_structured = events.append
        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(manager._exploration.no_frontier_escape_state, "moving")
        self.assertEqual(manager._exploration.no_frontier_escape_target, (-0.6, 0.0))
        event_names = [e.get("event") for e in events]
        self.assertIn("no_frontier_retreat_started", event_names)

    def test_main_entrance_failure_keeps_entrance_unset(self):
        """主入口过门失败后，main_entrance_xy/pass_vector 必须保持未设置。"""
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.start_time = 0.0
        manager.max_runtime = 1000.0
        manager._reporting = SimpleNamespace(
            status_enabled=False,
            publish_status=lambda _report: None,
        )
        manager._shutdown_requested = False
        manager._finished = False
        manager._timeout_triggered = False
        manager._failure_reasons = []
        manager._last_pose = None
        manager._last_status_publish = 0.0
        manager.status_publish_period = 1.0
        manager.home_pose = (0.0, 0.0, 0.0)
        manager.current_floor = 0
        manager._max_home_distance = 0.0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = "main_entrance"
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        # last_door_* 来自上一次其他门，不应被写入 main_entrance_*
        manager._door_navigation.last_door_xy = (5.0, 5.0)
        manager._door_navigation.last_pass_vector = (1.0, 0.0)
        manager._door_navigation.transition_until = 0.0
        manager._door_navigation.completion_pose = None
        manager._door_navigation.last_door_id = "other_door"
        manager.door_transition_duration = 3.0
        manager._door_navigation.frontier_debug = None
        manager.return_after_floor_change = False
        manager.target_floor = None
        manager.current_task = OpenDoorAndPass("main_entrance")
        manager._entry.origin_pose = (0.0, -3.2, math.pi / 2.0)
        manager._entry.completed = False
        manager._entry.failed = False
        manager._mission_success_eligible = True
        transitions = []
        manager._transition_state = lambda new_state, **kwargs: (
            transitions.append((new_state, kwargs)),
            setattr(manager, "state", new_state),
        )
        # 让 task.tick 直接返回 FAILED
        manager.current_task.tick = lambda mgr, pose: FAILED
        manager.navigation_task_context = lambda: object()
        manager.controller = SimpleNamespace(stop=lambda *a, **k: None)
        manager._map_runtime.latest_cloud_points = []
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5, width=4.0, length=4.0, origin_x=-2.0, origin_y=-2.0,
        )
        manager._perception_runtime.sensor_buffer = SimpleNamespace(
            snapshot=lambda: SimpleNamespace(
                odom_msg=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0), orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))),
                cloud_msg=None,
                image_bundle=None,
            ),
            read_cloud_xyz=lambda *a, **k: [],
        )
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=10.0, xyz=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0), angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._localization_health = True
        manager.localization_stance_health_required = True

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None, logerr=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager.tick()

        self.assertIsNone(manager.main_entrance_xy)
        self.assertIsNone(manager.main_entrance_pass_vector)
        self.assertIn("main_entrance", manager.failed_doors)
        self.assertIsNone(manager.current_task)
        self.assertEqual(manager.state, WRITE_RESULT)
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][1]["reason_code"], "main_entrance_crossing_failed")

    def test_main_entrance_success_sets_entrance_context(self):
        """主入口过门成功后，main_entrance_xy/pass_vector 应写入过门方向。"""
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager.start_time = 0.0
        manager.max_runtime = 1000.0
        manager._reporting = SimpleNamespace(
            status_enabled=False,
            publish_status=lambda _report: None,
        )
        manager._shutdown_requested = False
        manager._finished = False
        manager._timeout_triggered = False
        manager._failure_reasons = []
        manager._last_pose = None
        manager._last_status_publish = 0.0
        manager.status_publish_period = 1.0
        manager.home_pose = (0.0, 0.0, 0.0)
        manager.current_floor = 0
        manager._max_home_distance = 0.0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = "main_entrance"
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.post_door_min_distance = 0.9
        manager.door_pass_offset = 0.5
        manager.door_transition_duration = 3.0
        manager._door_navigation.frontier_debug = None
        manager.return_after_floor_change = False
        manager.target_floor = None
        manager.current_task = OpenDoorAndPass("main_entrance")
        # 模拟成功过门：tick 返回 DONE，并设置 wait/pass 点供门导航运行时记录。
        manager.current_task._wait_point = (0.0, -0.8)
        manager.current_task._pass_point = (0.0, 0.8)
        manager.current_task._commit_point = (0.0, 1.8)
        from danger_search.navigator import DONE
        manager.current_task.tick = lambda mgr, pose: DONE
        manager.navigation_task_context = lambda: object()
        manager.controller = SimpleNamespace(stop=lambda *a, **k: None)
        manager._map_runtime.latest_cloud_points = []
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5, width=4.0, length=4.0, origin_x=-2.0, origin_y=-2.0,
        )
        manager._perception_runtime.sensor_buffer = SimpleNamespace(
            snapshot=lambda: SimpleNamespace(
                odom_msg=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0), orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))),
                cloud_msg=None,
                image_bundle=None,
            ),
            read_cloud_xyz=lambda *a, **k: [],
        )
        manager.pose_provider = SimpleNamespace(
            latest=lambda: mission_manager_module.PoseSample(
                stamp=10.0, xyz=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_xyz=(0.0, 0.0, 0.0), angular_xyz=(0.0, 0.0, 0.0),
            )
        )
        manager._localization_health = True
        manager.localization_stance_health_required = True

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None, logerr=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager.tick()

        self.assertEqual(manager.main_entrance_xy, (0.0, 0.0))
        self.assertEqual(manager.main_entrance_pass_vector, (0.0, 1.0))
        self.assertIn("main_entrance", manager.opened_doors)
        self.assertIsNone(manager.current_task)
        self.assertEqual(manager._entry.clearance_target, (0.0, 0.9))

    def test_main_entrance_clearance_keeps_axis_before_frontier_handoff(self):
        paths = []
        set_path_kwargs = []
        manager = _make_test_manager()
        manager._entry.clearance_target = (0.0, 0.9)
        manager._entry.clearance_door_id = "main_entrance"
        manager._entry.clearance_path_started = False
        manager.main_entrance_clearance_max_linear = 0.4
        manager.controller = SimpleNamespace(
            goal_tolerance=0.25,
            has_path=lambda: False,
            set_path=lambda path, **kwargs: (paths.append(path), set_path_kwargs.append(kwargs)),
            poll_status=lambda: RUNNING,
        )

        with patch.object(
            initial_entry_runtime_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None),
        ):
            result = manager._entry.tick_clearance(
                (0.12, 0.24, math.pi / 2.0),
                controller=manager.controller,
                max_linear=manager.main_entrance_clearance_max_linear,
            )

        self.assertTrue(result.handled)
        self.assertEqual(paths, [[(0.12, 0.24), (0.0, 0.9)]])
        # 限速与终点容差随路径原子提交（固定频率控制读取）
        self.assertEqual(set_path_kwargs[0]["max_linear"], 0.4)
        self.assertEqual(set_path_kwargs[0]["goal_tolerance"], 0.12)

    def test_main_entrance_clearance_stuck_fails_instead_of_scanning(self):
        transitions = []
        cleared = []
        manager = _make_test_manager()
        manager.state = EXPLORE_FLOOR
        manager._entry.clearance_target = (0.0, 0.9)
        manager._entry.clearance_door_id = "main_entrance"
        manager._entry.clearance_path_started = True
        manager.main_entrance_clearance_max_linear = 0.4
        manager._mission_success_eligible = True
        manager._failure_reasons = []
        manager.controller = SimpleNamespace(
            goal_tolerance=0.25,
            has_path=lambda: True,
            poll_status=lambda: STUCK,
            clear_path=lambda reason=None: cleared.append(reason),
        )
        manager._transition_state = lambda state, **kwargs: transitions.append(
            (state, kwargs)
        )

        result = manager._entry.tick_clearance(
            (0.12, 0.24, math.pi / 2.0),
            controller=manager.controller,
            max_linear=manager.main_entrance_clearance_max_linear,
        )
        if result.failed:
            manager._apply_initial_entry_clearance_failure(result.target)

        self.assertTrue(result.handled)
        self.assertEqual(cleared, ["main_entrance_clearance_failed"])
        self.assertFalse(manager._mission_success_eligible)
        self.assertEqual(transitions[0][0], WRITE_RESULT)
        self.assertEqual(
            transitions[0][1]["reason_code"],
            "main_entrance_clearance_failed",
        )

    def test_door_approach_budget_exhausted_does_not_blacklist(self):
        """门接近规划预算耗尽时不应黑名单该门，可能可达应留待重试。"""
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager._door_navigation.approach_blacklist = {}
        manager.door_pass_offset = 0.8
        manager.active_door = None
        manager.current_task = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5, width=12.0, length=12.0, origin_x=-6.0, origin_y=-6.0,
        )

        def get_door_points(door_id, robot_pose, offset=0.8):
            return ((3.0, 0.0 + offset), (3.0, 0.0 - offset))

        manager.building_client = SimpleNamespace(
            controllable_doors=lambda floor_index=None, include_elevators=True: [
                SimpleNamespace(door_id="far_door", floor_index=0),
            ],
            get_door_points=get_door_points,
        )

        with patch(
            "danger_search.door_navigation_runtime.astar_plan",
            return_value=PlanResult(status=BUDGET_EXHAUSTED),
        ):
            selected = _select_door_approach(manager, (0.0, 0.0, 0.0))

        # 预算耗尽：不选择该门，也不黑名单。
        self.assertIsNone(selected)
        self.assertEqual(manager._door_navigation.approach_blacklist, {})

    def test_return_home_budget_exhausted_waits_for_map(self):
        """返航规划预算耗尽时应等待地图更新，不退化为直线返航。"""
        recorded_paths = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_pose = (5.0, 0.0, 0.0)
        manager.home_tolerance = 0.2
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=1.0, width=8.0, length=8.0, origin_x=-4.0, origin_y=-4.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        cleared = []
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            set_path=lambda path, **kwargs: recorded_paths.append(path),
            poll_status=lambda: "idle",
            recover=lambda: None,
            clear_path=lambda reason=None: cleared.append(reason),
            stop=lambda reason=None: None,
        )

        with patch("danger_search.return_home_planner.astar_plan", return_value=PlanResult(status=BUDGET_EXHAUSTED)), \
             patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(logwarn=lambda *a, **k: None)):
            pose = (0.0, 0.0, 0.0)
            manager._tick_return_home(pose)
            self._finish_return_home_plan(manager, pose)

        # 预算耗尽：不设路径，清空当前路径等待重试。
        self.assertEqual(recorded_paths, [])
        self.assertEqual(cleared, ["return_home_budget_exhausted"])

    def test_return_home_slow_planning_does_not_block_tick(self):
        """返航规划可以耗时，但 10 Hz 主循环必须立即进入安全停车。"""
        release = threading.Event()
        started = threading.Event()
        stops = []
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.max_floor = 1
        manager.home_pose = (2.0, 0.0, 0.0)
        manager.home_tolerance = 0.2
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=1.0,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            stop=lambda reason=None: stops.append(reason),
            poll_status=lambda: 'RUNNING',
        )

        def slow_plan(*_args, **_kwargs):
            started.set()
            release.wait(timeout=1.0)
            return "unreachable", []

        with patch.object(
            return_home_runtime_module,
            "compute_return_home_plan",
            side_effect=slow_plan,
        ):
            started_at = time.monotonic()
            manager._tick_return_home((-2.0, 0.0, 0.0))
            elapsed = time.monotonic() - started_at

            self.assertTrue(started.wait(timeout=0.2))
            self.assertLess(elapsed, 0.1)
            self.assertEqual(stops, ["return_home_plan_pending"])
            release.set()
            manager._return_home.planner._thread.join(timeout=1.0)


class TimeBudgetDecisionTest(unittest.TestCase):
    """方案 C：全局时间预算与楼层/返航决策单元测试。"""

    def _make_manager(self, **overrides) -> MissionManager:
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.max_floor = overrides.get("max_floor", 1)
        manager.max_runtime = overrides.get("max_runtime", 120.0)
        manager.floor_timeout = overrides.get("floor_timeout", 40.0)
        manager.last_floor_uses_global_budget = overrides.get(
            "last_floor_uses_global_budget", True
        )
        manager.min_floor_explore_time = overrides.get("min_floor_explore_time", 5.0)
        manager.return_safety_buffer = overrides.get("return_safety_buffer", 10.0)
        manager.return_speed_estimate = overrides.get("return_speed_estimate", 0.5)
        manager.floor_change_time_estimate = overrides.get("floor_change_time_estimate", 20.0)
        manager.floor_finish_time_ratio = overrides.get("floor_finish_time_ratio", 0.75)
        manager.exploration_progress_window = overrides.get("exploration_progress_window", 10.0)
        manager.min_exploration_progress_cells = overrides.get("min_exploration_progress_cells", 20)
        manager.min_remaining_frontiers = overrides.get("min_remaining_frontiers", 2)
        manager.home_pose = overrides.get("home_pose", (0.0, 0.0, 0.0))
        manager._exploration.progress_history = []
        manager._map_runtime.tracker = SimpleNamespace(confirmed_positions=lambda **kw: [])
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5, width=10.0, length=10.0, origin_x=-5.0, origin_y=-5.0,
        )
        grid = manager.mapper.floor(0)
        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.mark_free(gx, gy)
        return manager

    def _decide(
        self,
        manager,
        pose,
        _grid,
        frontiers,
        *,
        floor_elapsed,
        mission_elapsed,
    ):
        progress = manager._exploration.recent_progress(
            now=mission_manager_module.get_time(),
            window_seconds=manager.exploration_progress_window,
        )
        next_floor_change_time = (
            manager._estimate_floor_change_time(manager.current_floor + 1, pose)
            if manager.current_floor + 1 < manager.max_floor
            else float("inf")
        )
        decision = decide_exploration_budget(
            ExplorationBudgetInput(
                home_recorded=manager.home_pose is not None,
                mission_elapsed=mission_elapsed,
                floor_elapsed=floor_elapsed,
                max_runtime=manager.max_runtime,
                return_time=manager._estimate_return_time(pose),
                return_safety_buffer=manager.return_safety_buffer,
                floor_timeout=manager.floor_timeout,
                floor_finish_time_ratio=manager.floor_finish_time_ratio,
                known_cells_growth=progress["known_cells_growth"],
                exploration_progress_window=manager.exploration_progress_window,
                minimum_progress_cells=manager.min_exploration_progress_cells,
                frontier_count=len(frontiers),
                minimum_remaining_frontiers=manager.min_remaining_frontiers,
                current_floor=manager.current_floor,
                maximum_floor=manager.max_floor,
                next_floor_change_time=next_floor_change_time,
                last_floor_uses_global_budget=manager.last_floor_uses_global_budget,
            )
        )
        return decision.action, decision.reason_code

    def test_mission_time_insufficient_triggers_return_home(self):
        """总剩余时间不足以支持返航时，必须立即返航。"""
        manager = self._make_manager(max_runtime=100.0, return_safety_buffer=10.0)
        # 机器人在 (15,0)，返航路径约 15m，保守时间约 15/0.5=30s；
        # 加上 return_safety_buffer=10s，mission_elapsed=65 时，65+30+10>=100，应返航。
        action, reason = self._decide(manager,
            (15.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(20.0, 0.0))] * 5,
            floor_elapsed=20.0,
            mission_elapsed=65.0,
        )
        self.assertEqual(action, "RETURN_HOME")
        self.assertEqual(reason, "mission_insufficient_time_for_return")

    def test_plenty_of_time_allows_continue(self):
        """时间充足且有 frontier 时继续探索。"""
        manager = self._make_manager(max_runtime=120.0)
        action, reason = self._decide(manager,
            (1.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(3.0, 0.0))] * 5,
            floor_elapsed=10.0,
            mission_elapsed=10.0,
        )
        self.assertEqual(action, "CONTINUE")
        self.assertEqual(reason, "budget_allows_continue")

    def test_floor_timeout_single_floor_uses_global_budget(self):
        """单层仍有收益且返航时间充足时，不应浪费剩余总预算。"""
        manager = self._make_manager(max_floor=1, max_runtime=120.0, floor_timeout=40.0)
        now = mission_manager_module.get_time()
        manager._exploration.progress_history = [
            {"time": now - 5.0, "known_cells": 100, "free_or_visited_cells": 80, "confirmed_dangers": 0},
            {"time": now, "known_cells": 130, "free_or_visited_cells": 110, "confirmed_dangers": 0},
        ]
        action, reason = self._decide(manager,
            (3.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))] * 3,
            floor_elapsed=45.0,
            mission_elapsed=50.0,
        )
        self.assertEqual(action, "CONTINUE")
        self.assertEqual(reason, "last_floor_uses_global_budget")

    def test_floor_timeout_legacy_switch_returns_home(self):
        """关闭新语义时，单层到达原楼层上限后仍立即返航。"""
        manager = self._make_manager(
            max_floor=1,
            max_runtime=120.0,
            floor_timeout=40.0,
            last_floor_uses_global_budget=False,
        )
        action, reason = self._decide(manager,
            (3.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))] * 3,
            floor_elapsed=45.0,
            mission_elapsed=50.0,
        )
        self.assertEqual(action, "RETURN_HOME")
        self.assertEqual(reason, "floor_timeout_return_home")

    def test_last_floor_of_multi_floor_uses_global_budget(self):
        """多层任务的最后一层同样由全局返航边界收口。"""
        manager = self._make_manager(max_floor=3, max_runtime=200.0, floor_timeout=40.0)
        manager.current_floor = 2
        now = mission_manager_module.get_time()
        manager._exploration.progress_history = [
            {"time": now - 5.0, "known_cells": 100, "free_or_visited_cells": 80, "confirmed_dangers": 0},
            {"time": now, "known_cells": 130, "free_or_visited_cells": 110, "confirmed_dangers": 0},
        ]
        action, reason = self._decide(manager,
            (1.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))] * 3,
            floor_elapsed=45.0,
            mission_elapsed=80.0,
        )
        self.assertEqual(action, "CONTINUE")
        self.assertEqual(reason, "last_floor_uses_global_budget")

    def test_floor_timeout_multi_floor_with_elevator_changes_floor(self):
        """多楼层且存在可用电梯、剩余时间足够时，楼超时后换层。"""
        manager = self._make_manager(max_floor=2, max_runtime=200.0, floor_timeout=40.0)
        manager.building_client = SimpleNamespace(
            elevators={
                "main": SimpleNamespace(
                    lobby_poses={0: (0.0, 0.0, 0.0), 1: (0.0, 0.0, 0.0)},
                )
            }
        )
        action, reason = self._decide(manager,
            (1.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(5.0, 0.0))],
            floor_elapsed=45.0,
            mission_elapsed=50.0,
        )
        self.assertEqual(action, "FINISH_FLOOR")
        self.assertEqual(reason, "floor_timeout_change_floor")

    def test_floor_soft_timeout_low_progress_returns_home(self):
        """楼层软超时后探索收益低且 frontier 少，应结束当前层返航。"""
        manager = self._make_manager(
            max_floor=1,
            max_runtime=120.0,
            floor_timeout=40.0,
            floor_finish_time_ratio=0.75,
        )
        # 填充历史使 known_cells 无增长
        now = 100.0
        manager._exploration.progress_history = [
            {"time": now - 10.0, "known_cells": 100, "free_or_visited_cells": 80, "confirmed_dangers": 0},
            {"time": now, "known_cells": 100, "free_or_visited_cells": 80, "confirmed_dangers": 0},
        ]
        with patch.object(mission_manager_module, "get_time", return_value=now):
            action, reason = self._decide(manager,
                (3.0, 0.0, 0.0),
                manager.mapper.floor(0),
                [SimpleNamespace(centroid=(5.0, 0.0))],  # 少于 min_remaining_frontiers=2
                floor_elapsed=35.0,  # >= 40*0.75
                mission_elapsed=40.0,
            )
        self.assertEqual(action, "RETURN_HOME")
        self.assertEqual(reason, "floor_low_progress_return_home")

    def test_no_home_pose_defers_decision(self):
        """home_pose 尚未记录时不应触发返航。"""
        manager = self._make_manager()
        manager.home_pose = None
        action, reason = self._decide(manager,
            (1.0, 0.0, 0.0),
            manager.mapper.floor(0),
            [SimpleNamespace(centroid=(3.0, 0.0))] * 5,
            floor_elapsed=10.0,
            mission_elapsed=10.0,
        )
        self.assertEqual(action, "CONTINUE")
        self.assertEqual(reason, "home_pose_not_recorded")


class ProgressMonitorDiagnosticOnlyTest(unittest.TestCase):
    """阶段二：ProgressMonitor 改为 diagnostic-only 的链路测试。"""

    def _make_explore_manager(self, *, frontiers_list=None, select_result=None):
        """构造可调用 _tick_explore 的轻量 MissionManager。"""
        manager = _make_test_manager()
        manager.current_floor = 0
        manager.manual_mode = False
        manager.start_time = 0.0
        manager.floor_start_time = 0.0
        manager.floor_timeout = 1000.0
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager.no_reachable_frontier_confirmations = 1
        manager.min_corridor_progress_distance = 0.0
        manager.probe_yaw = math.pi / 2.0
        manager.probe_distance = 1.2
        manager._max_home_distance = 0.0
        manager._exploration.no_frontier_scan_until = 0.0
        manager._exploration.no_reachable_frontier_confirmations = 0
        manager._exploration.last_no_frontier_signature = None
        manager._exploration.no_frontier_escape_state = "idle"
        manager._exploration.no_frontier_escape_target = None
        manager._exploration.no_frontier_escape_deadline = 0.0
        manager.no_frontier_rotation_min_clearance = 0.40
        manager.no_frontier_retreat_max_distance = 2.0
        manager.no_frontier_retreat_clearance_margin = 0.10
        manager._map_runtime.nearest_radial_obstacle_distance = 1.0
        manager._floor_navigation_traces = {0: []}
        manager._exploration.probe_attempt = 0
        manager._exploration.last_frontier_target = None
        manager._exploration.last_frontier_score = None
        manager._exploration.last_frontier_used_fallback = False
        manager.door_pass_offset = 0.8
        manager.door_commit_distance = 1.8
        manager.door_clear_radius = 0.9
        manager.opened_doors = set()
        manager.failed_doors = set()
        manager.active_door = None
        manager.current_task = None
        manager.current_frontier = None
        manager._map_runtime.mapper = MultiFloorGridMap(
            resolution=0.5, width=20.0, length=20.0, origin_x=-10.0, origin_y=-10.0,
        )
        grid = manager.mapper.floor(0)
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        manager.explorer = SimpleNamespace(
            extract_frontiers=lambda g: list(frontiers_list or []),
            record_stuck_failure=lambda g, xy: None,
            door_front_half_plane_radius=1.5,
            door_front_half_plane_penalty=6.0,
            last_selection_debug=None,
        )
        if select_result is not None:
            manager.explorer.select_frontier = lambda *a, **kw: select_result
        manager._map_runtime.local_detector = SimpleNamespace(
            max_distance=3.0,
            detect_controllable_door=lambda **kw: None,
        )
        manager.building_client = SimpleNamespace(
            is_elevator_door=lambda door_id: False,
            door_service_ready=lambda: True,
            controllable_doors=lambda **kw: [],
        )
        manager.controller = SimpleNamespace(
            has_path=lambda: False,
            poll_status=lambda: "running",
            recover=lambda: None,
            clear_path=lambda *a, **k: None,
            stop=lambda *a, **k: None,
            set_path=lambda p, **kwargs: None,
            reverse_speed=-0.15,
            reverse_straight=lambda **kwargs: True,
        )
        manager._door_navigation.approach_blacklist = {}
        manager.door_transition_hold_distance = 2.5
        manager.post_door_min_distance = 0.0
        manager.post_door_min_known_area = 0.0
        manager._door_navigation.last_door_xy = None
        manager._door_navigation.last_pass_vector = None
        manager._door_navigation.transition_until = 0.0
        manager.main_entrance_xy = None
        manager.main_entrance_pass_vector = None
        manager.frontier_current_region_radius = 1.2
        manager.frontier_switch_score_margin = 2.0
        manager._door_navigation.frontier_debug = None
        manager._map_runtime.latest_cloud_points = []
        manager._progress_monitor = None
        manager.progress_monitor_cooldown = 10.0
        manager.state = EXPLORE_FLOOR
        manager.home_pose = None
        manager.max_runtime = 1000.0
        manager.return_safety_buffer = 30.0
        manager.return_speed_estimate = 0.5
        manager.floor_change_time_estimate = 40.0
        manager.floor_finish_time_ratio = 0.75
        manager.exploration_progress_window = 15.0
        manager.min_exploration_progress_cells = 30
        manager.min_remaining_frontiers = 2
        manager.max_floor = 1
        manager._exploration.progress_history = []
        manager._exploration.last_budget_decision = None
        manager._exploration.last_budget_decision_time = 0.0
        manager._exploration.floor_timeout_logged = False
        manager._reporting = SimpleNamespace(
            status_enabled=False,
            publish_status=lambda _report: None,
        )
        manager._last_status_publish = 0.0
        manager.status_publish_period = 1.0
        manager.cloud_max_points = 100
        manager.cloud_stride = 1
        return manager

    @staticmethod
    def _tick_no_frontier_runtime(
        manager,
        *,
        now: float,
        frontiers_present: bool = False,
        pose=None,
    ):
        """直接验证探索子状态机，不保留已删除的 MissionManager 私有接口。"""
        result = manager._exploration.tick_no_frontier_scan(
            now=now,
            frontiers_present=frontiers_present,
            pose=pose,
            grid=manager.mapper.floor(manager.current_floor),
            navigation_trace=list(
                manager._floor_navigation_traces.get(manager.current_floor, ())
            ),
            controller=manager.controller,
            radial_clearance=manager._map_runtime.nearest_radial_obstacle_distance,
            current_floor=manager.current_floor,
            config=manager._no_frontier_recovery_config(),
        )
        manager._apply_exploration_runtime_result(result)
        return result

    def test_explore_does_not_use_progress_monitor_to_filter_frontiers(self):
        """frontier 选择不再受 ProgressMonitor.filter_frontiers 影响。"""
        from danger_search.frontier import Frontier
        frontier = Frontier(cells=((10, 10),), centroid=(5.0, 0.0), information_gain=10.0)
        select_result = (frontier, [(0.0, 0.0), (5.0, 0.0)])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )
        # filter_frontiers 返回空列表，模拟 ProgressMonitor 过滤掉所有 frontier
        monitor = ProgressMonitor(
            goal_tolerance=0.35, window=5.0, repeat=3,
            min_displacement=0.20, cooldown=10.0,
        )
        monitor.filter_frontiers = lambda now, frontiers: []
        manager._progress_monitor = monitor

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        # 尽管 filter_frontiers 返回空，select_frontier 仍被调用并选中 frontier
        self.assertIsNotNone(manager.current_frontier)
        self.assertEqual(manager.current_frontier.centroid, (5.0, 0.0))

    def test_normal_frontier_astar_runs_in_background_before_install(self):
        frontier = Frontier(cells=(), centroid=(3.0, 0.0), information_gain=0.0)
        manager = self._make_explore_manager(frontiers_list=[frontier])
        manager.async_frontier_planning_enabled = True
        manager._frontier_planning = FrontierPlanningRuntime(
            timeout=5.0,
            preplan_min_lead=0.5,
            preplan_max_lead=3.0,
            opportunistic_period=3.0,
        )
        manager._frontier_plan_request_id = 0
        manager._frontier_plan_timeout_logged = False
        manager._prefetched_frontier_plan = None
        worker = SimpleNamespace(
            _fail_counts={},
            last_selection_debug=None,
            select_frontier=lambda *args, **kwargs: (
                frontier,
                [(0.0, 0.0), (3.0, 0.0)],
            ),
        )
        manager.explorer.planning_copy = lambda: worker
        manager.explorer.apply_planning_failure_deltas = lambda deltas: None
        manager.explorer.is_blacklisted = lambda grid, target: False
        installed = []
        manager.controller.planning_progress_snapshot = lambda: PathProgressSnapshot(
            active=False,
            remaining_distance=None,
            final_goal=None,
            final_heading=None,
            path_reason=None,
            status=IDLE,
            status_sequence=0,
            path_generation=0,
            terminal_pending=False,
        )
        manager.controller.set_path = lambda path, **kwargs: installed.append(
            (list(path), kwargs.get("reason"))
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logwarn=lambda *args, **kwargs: None,
            ),
        ), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))
            self.assertEqual(installed, [])
            deadline = time.monotonic() + 1.0
            while not installed and time.monotonic() < deadline:
                time.sleep(0.001)
                manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(installed[0][1], "frontier_selected_path")

    def test_ready_prefetch_is_installed_when_old_frontier_reports_done(self):
        old = Frontier(cells=((24, 20),), centroid=(2.0, 0.0), information_gain=5.0)
        new = Frontier(cells=((30, 20),), centroid=(5.0, 0.0), information_gain=8.0)
        manager = self._make_explore_manager(frontiers_list=[old, new])
        manager.async_frontier_planning_enabled = True
        manager.current_frontier = old
        manager._exploration.last_frontier_target = old.centroid
        manager._exploration.last_frontier_score = 1.0
        manager._frontier_planning = FrontierPlanningRuntime(
            timeout=5.0,
            preplan_min_lead=0.5,
            preplan_max_lead=3.0,
            opportunistic_period=3.0,
        )
        manager._frontier_planning.route_eta = lambda remaining: 0.1
        manager._frontier_plan_request_id = 0
        manager._frontier_plan_timeout_logged = False
        manager._prefetched_frontier_plan = None
        worker = SimpleNamespace(
            _fail_counts={},
            last_selection_debug=None,
            select_frontier=lambda *args, **kwargs: (
                new,
                [(2.0, 0.0), (5.0, 0.0)],
            ),
        )
        manager.explorer.planning_copy = lambda: worker
        manager.explorer.is_blacklisted = lambda grid, target: False
        manager.explorer.apply_planning_failure_deltas = lambda deltas: None
        # 让新候选在实时地图上仍保有未知邻域。
        grid = manager.mapper.floor(0)
        grid.set_base_state(30, 21, UNKNOWN)
        installed = []
        statuses = [RUNNING, DONE]
        manager.controller.has_path = lambda: True
        manager.controller.planning_progress_snapshot = lambda: PathProgressSnapshot(
            active=True,
            remaining_distance=0.1,
            final_goal=(2.0, 0.0),
            final_heading=0.0,
            path_reason="frontier_selected_path",
            status=RUNNING,
            status_sequence=1,
            path_generation=1,
            terminal_pending=False,
        )
        manager.controller.poll_status = lambda: statuses.pop(0)
        manager.controller.set_path = lambda path, **kwargs: installed.append(
            (list(path), kwargs.get("reason"))
        )

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logwarn=lambda *args, **kwargs: None,
            ),
        ), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((2.0, 0.0, 0.0))
            deadline = time.monotonic() + 1.0
            while not installed and time.monotonic() < deadline:
                time.sleep(0.001)
                manager._tick_explore((2.0, 0.0, 0.0))

        self.assertEqual(installed[0][1], "frontier_prefetched_path")
        self.assertEqual(manager.current_frontier, new)

    def test_active_path_skips_unused_frontier_extraction_before_soft_budget(self):
        manager = self._make_explore_manager()
        extraction_calls = []
        manager.explorer.extract_frontiers = lambda grid: extraction_calls.append(grid) or []
        manager.controller.has_path = lambda: True
        manager.controller.tick = lambda pose: "running"
        manager.floor_timeout = 100.0
        manager.floor_finish_time_ratio = 0.75

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(extraction_calls, [])

    def test_active_path_refreshes_frontiers_when_soft_budget_uses_count(self):
        manager = self._make_explore_manager()
        extraction_calls = []
        manager.explorer.extract_frontiers = lambda grid: extraction_calls.append(grid) or []
        manager.controller.has_path = lambda: True
        manager.controller.tick = lambda pose: "running"
        manager.floor_timeout = 100.0
        manager.floor_finish_time_ratio = 0.75
        manager.home_pose = None

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ), patch.object(mission_manager_module, "get_time", return_value=80.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(extraction_calls), 1)

    def test_progress_monitor_degenerate_does_not_clear_path_or_reselect(self):
        """退化检测不执行 cooldown、clear_path 或主动重选。"""
        from danger_search.frontier import Frontier
        frontier = Frontier(cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0)
        select_result = (frontier, [(0.0, 0.0), (0.30, 0.0)])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )
        clear_calls = []
        manager.controller.clear_path = lambda *a, **k: clear_calls.append(True)

        monitor = ProgressMonitor(
            goal_tolerance=0.35, window=5.0, repeat=3,
            min_displacement=0.20, cooldown=10.0,
        )
        # 预填选择历史，使 check_degenerate 必然返回 is_degenerate=True
        for i in range(5):
            monitor.record_selection(float(i) * 0.1, (0.30, 0.0))
        manager._progress_monitor = monitor

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=0.5):
            manager._tick_explore((0.0, 0.0, 0.0))

        # 退化检测不应触发 clear_path
        self.assertEqual(clear_calls, [])
        # current_frontier 不应被重置（保留选中的 frontier）
        self.assertIsNotNone(manager.current_frontier)

    def test_progress_monitor_still_records_pose_and_selection(self):
        """ProgressMonitor 仍记录 pose 和 selection 诊断数据。"""
        from danger_search.frontier import Frontier
        frontier = Frontier(cells=((10, 10),), centroid=(5.0, 0.0), information_gain=10.0)
        select_result = (frontier, [(0.0, 0.0), (5.0, 0.0)])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )

        record_pose_calls = []
        record_selection_calls = []
        monitor = ProgressMonitor(
            goal_tolerance=0.35, window=5.0, repeat=3,
            min_displacement=0.20, cooldown=10.0,
        )
        monitor.record_pose = lambda t, p: record_pose_calls.append((t, p))
        monitor.record_selection = lambda t, xy: record_selection_calls.append((t, xy))
        manager._progress_monitor = monitor

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((1.0, 2.0, 0.0))

        self.assertGreater(len(record_pose_calls), 0)
        self.assertEqual(record_pose_calls[0][1], (1.0, 2.0, 0.0))
        self.assertGreater(len(record_selection_calls), 0)
        self.assertEqual(record_selection_calls[0][1], (5.0, 0.0))

    def test_inside_tolerance_rejection_only_at_source(self):
        """inside_goal_tolerance 拒绝只发生在 FrontierExplorer 源头，不依赖 ProgressMonitor。"""
        from danger_search.frontier import FrontierExplorer
        from danger_search.mapper import FloorGrid

        grid = FloorGrid(
            floor_index=0, resolution=0.1,
            width=4.0, length=4.0, origin_x=-2.0, origin_y=-2.0,
        )
        for gy in range(5, 35):
            for gx in range(5, 35):
                grid.mark_free(gx, gy)
        explorer = FrontierExplorer(min_cluster_size=1, goal_tolerance=0.35)
        recorded_failures = []
        explorer.record_failure = lambda g, xy: recorded_failures.append(xy)

        frontier, path = explorer.select_frontier(grid, (0.0, 0.0, 0.0))

        # 选中的目标应在 tolerance 外
        if frontier is not None and path:
            self.assertGreater(
                math.hypot(frontier.centroid[0], frontier.centroid[1]), 0.35,
            )
        # 不记录 failure
        self.assertEqual(recorded_failures, [])

    def test_all_candidates_inside_tolerance_enters_no_frontier_flow(self):
        """所有候选过近时进入现有 no-frontier/probe 流程，不形成高频循环。"""
        manager = self._make_explore_manager(
            frontiers_list=[],
            select_result=(None, []),
        )
        manager._exploration.no_frontier_scan_until = 0.0
        manager.no_frontier_scan_duration = 8.0
        manager.home_pose = (0.0, 0.0, 0.0)
        manager.min_floor_explore_time = 0.0
        manager.min_floor_travel_distance = 0.0
        manager.min_known_area = 0.0
        manager.explore_coverage_ratio = 0.0
        manager._max_home_distance = 10.0

        set_path_calls = []
        manager.controller.set_path = lambda p, **kwargs: set_path_calls.append(p)

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        # 没有 frontier 时进入 probe 流程（set_path 被调用来执行短距离探测）
        # 或进入 scan 流程（rotate_in_place）
        # 无论哪种，都不应记录 failure
        self.assertEqual(manager._exploration.no_reachable_frontier_confirmations, 0)

    def test_all_inside_tolerance_candidates_can_start_probe(self):
        """列表非空但全部过近时，也应进入现有短距离探测流程。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug

        frontier = Frontier(
            cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0,
        )
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=(None, []),
        )
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1,
            reachable_count=0,
            blacklisted_count=0,
            unreachable_count=0,
            context_filtered_count=0,
            used_fallback=False,
            final_target=None,
            final_score=0.0,
            path_length=0.0,
            inside_tolerance_count=1,
        )
        manager.no_frontier_scan_duration = 8.0
        recorded_paths = []
        manager.controller.set_path = lambda path, **kwargs: recorded_paths.append(path)
        manager.controller.rotate_in_place = lambda value: None

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(recorded_paths), 1)
        self.assertEqual(manager._exploration.no_reachable_frontier_confirmations, 0)

    def test_all_inside_tolerance_no_repeat_astar_same_pose_map(self):
        """所有候选过近且 pose/map 不变时，连续多个 tick 不重复执行完整 A*。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        frontier = Frontier(cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0)
        select_result = (None, [])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )
        # 设置 last_selection_debug 模拟所有候选过近
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1,
            reachable_count=0,
            blacklisted_count=0,
            unreachable_count=0,
            context_filtered_count=0,
            used_fallback=False,
            final_target=None,
            final_score=0.0,
            path_length=0.0,
            inside_tolerance_count=1,
        )
        manager.no_frontier_scan_duration = 8.0
        manager._try_probe_unknown_space = lambda pose: False
        select_calls = []
        manager.explorer.select_frontier = lambda *a, **kw: select_calls.append(kw) or (None, [])
        manager.explorer.extract_frontiers = lambda g: [frontier]

        rotate_calls = []
        manager.controller.rotate_in_place = lambda v: rotate_calls.append(v)

        times = [10.0, 10.1, 10.2, 10.3, 10.5, 11.0, 12.0]
        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            for t in times:
                with patch.object(mission_manager_module, "get_time", return_value=t):
                    manager._tick_explore((0.0, 0.0, 0.0))

        # select_frontier 只在第一个 tick 调用一次（之后扫描门控阻止重复 A*）
        self.assertEqual(len(select_calls), 1)
        # 后续 tick 执行 rotate_in_place
        self.assertGreater(len(rotate_calls), 0)
        # 不记录 failure
        self.assertEqual(manager._exploration.no_reachable_frontier_confirmations, 0)

    def test_scan_same_count_changed_frontier_restores_selection(self):
        """扫描中 frontier 数量相同但内容变化时，立即恢复选择。"""
        from danger_search.frontier import Frontier
        frontier_old = Frontier(cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0)
        frontier_new = Frontier(cells=((60, 60),), centroid=(3.0, 3.0), information_gain=15.0)
        manager = self._make_explore_manager(
            frontiers_list=[frontier_old],
            select_result=(None, []),
        )
        manager.no_frontier_scan_duration = 1.0
        manager._try_probe_unknown_space = lambda pose: False

        from danger_search.frontier import FrontierSelectionDebug
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1, reachable_count=0, blacklisted_count=0,
            unreachable_count=0, context_filtered_count=0, used_fallback=False,
            final_target=None, final_score=0.0, path_length=0.0, inside_tolerance_count=1,
        )

        current_frontiers = [frontier_old]
        select_results = [(None, [])]

        def extract(g):
            return list(current_frontiers)
        def select_fn(*a, **kw):
            return select_results[-1]

        manager.explorer.extract_frontiers = extract
        manager.explorer.select_frontier = select_fn

        rotate_calls = []
        manager.controller.rotate_in_place = lambda v: rotate_calls.append(v)

        set_path_calls = []
        manager.controller.set_path = lambda p, **kwargs: set_path_calls.append(p)

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            # tick 1: 启动扫描
            with patch.object(mission_manager_module, "get_time", return_value=10.0):
                manager._tick_explore((0.0, 0.0, 0.0))
            # tick 2: 扫描中，frontier 数量不变，跳过
            with patch.object(mission_manager_module, "get_time", return_value=10.5):
                manager._tick_explore((0.0, 0.0, 0.0))
            # tick 3: 旧 frontier 被新 frontier 替换（数量仍为 1），恢复选择
            current_frontiers = [frontier_new]
            select_results.append((frontier_new, [(0.0, 0.0), (3.0, 3.0)]))
            with patch.object(mission_manager_module, "get_time", return_value=10.6):
                manager._tick_explore((0.0, 0.0, 0.0))

        # 新 frontier 被选中
        self.assertIsNotNone(manager.current_frontier)
        self.assertEqual(manager.current_frontier.centroid, (3.0, 3.0))
        self.assertGreater(len(set_path_calls), 0)

    def test_scan_robot_cell_change_restores_selection(self):
        """扫描中机器人移动到新栅格时，即使 frontier 不变也应恢复选择。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        near_frontier = Frontier(
            cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0,
        )
        valid_frontier = Frontier(
            cells=((26, 26),), centroid=(3.0, 3.0), information_gain=15.0,
        )
        manager = self._make_explore_manager(frontiers_list=[near_frontier])
        manager.no_frontier_scan_duration = 8.0
        manager._try_probe_unknown_space = lambda pose: False
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1, reachable_count=0, blacklisted_count=0,
            unreachable_count=0, context_filtered_count=0, used_fallback=False,
            final_target=None, final_score=0.0, path_length=0.0,
            inside_tolerance_count=1,
        )
        select_results = [
            (None, []),
            (valid_frontier, [(1.0, 0.0), (3.0, 3.0)]),
        ]
        select_calls = []

        def select_fn(*args, **kwargs):
            result = select_results[len(select_calls)]
            select_calls.append(kwargs)
            return result

        manager.explorer.select_frontier = select_fn
        manager.explorer.extract_frontiers = lambda grid: [near_frontier]
        manager.controller.rotate_in_place = lambda value: None
        set_path_calls = []
        manager.controller.set_path = lambda path, **kwargs: set_path_calls.append(path)

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            with patch.object(mission_manager_module, "get_time", return_value=10.0):
                manager._tick_explore((0.0, 0.0, 0.0))
            with patch.object(mission_manager_module, "get_time", return_value=10.1):
                manager._tick_explore((1.0, 0.0, 0.0))

        self.assertEqual(len(select_calls), 2)
        self.assertIs(manager.current_frontier, valid_frontier)
        self.assertEqual(len(set_path_calls), 1)

    def test_no_reachable_confirmations_not_incremented_per_tick(self):
        """no_reachable_frontier_confirmations 不按 tick 快速累加，只在扫描完成后增长。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        frontier = Frontier(cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0)
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=(None, []),
        )
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1, reachable_count=0, blacklisted_count=0,
            unreachable_count=0, context_filtered_count=0, used_fallback=False,
            final_target=None, final_score=0.0, path_length=0.0, inside_tolerance_count=1,
        )
        manager.no_frontier_scan_duration = 8.0
        manager._try_probe_unknown_space = lambda pose: False

        manager.explorer.select_frontier = lambda *a, **kw: (None, [])
        manager.explorer.extract_frontiers = lambda g: [frontier]
        manager.controller.rotate_in_place = lambda v: None

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            for t in [10.0, 10.1, 10.2, 10.3, 10.4, 10.5]:
                with patch.object(mission_manager_module, "get_time", return_value=t):
                    manager._tick_explore((0.0, 0.0, 0.0))

        # 所有候选过近时 confirmations 不增长
        self.assertEqual(manager._exploration.no_reachable_frontier_confirmations, 0)

    def test_no_frontier_scan_stops_instead_of_rotating_when_wall_is_close(self):
        """贴墙无前沿扫描必须零速等待，不能让机身旋转包络扫墙。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager.no_frontier_scan_duration = 4.0
        manager.no_frontier_rotation_min_clearance = 0.40
        manager._map_runtime.nearest_radial_obstacle_distance = 0.20
        manager._exploration.no_frontier_scan_until = 0.0
        manager._exploration.no_frontier_rotation_blocked_logged = False
        rotate_calls = []
        stop_reasons = []
        events = []
        manager.controller.rotate_in_place = rotate_calls.append
        manager.controller.stop = lambda *args, **kwargs: stop_reasons.append(
            kwargs.get("reason")
        )
        manager._log_structured = events.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            completed = self._tick_no_frontier_runtime(
                manager, now=10.0
            ).completed

        self.assertFalse(completed)
        self.assertEqual(rotate_calls, [])
        self.assertEqual(stop_reasons, ["unsafe_no_frontier_rotation_blocked"])
        self.assertEqual(events[0]["event"], "unsafe_rotation_blocked")
        self.assertEqual(events[0]["clearance"], 0.2)

    def test_no_frontier_scan_rotates_when_radial_clearance_is_safe(self):
        manager = self._make_explore_manager(frontiers_list=[])
        manager.no_frontier_scan_duration = 4.0
        manager.no_frontier_rotation_min_clearance = 0.40
        manager._map_runtime.nearest_radial_obstacle_distance = 0.65
        manager._exploration.no_frontier_scan_until = 0.0
        manager._exploration.no_frontier_rotation_blocked_logged = False
        rotate_calls = []
        manager.controller.rotate_in_place = rotate_calls.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            completed = self._tick_no_frontier_runtime(
                manager, now=10.0
            ).completed

        self.assertFalse(completed)
        self.assertEqual(rotate_calls, [0.45])

    def test_no_frontier_close_wall_retreats_on_traversed_trace_without_mutating_map(self):
        """贴墙无前沿时只沿真实轨迹退到宽处，不能改写地图制造通路。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager._map_runtime.nearest_radial_obstacle_distance = 0.33
        manager._floor_navigation_traces = {
            0: [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
        }
        grid = manager.mapper.floor(0)
        obstacle = grid.world_to_grid(2.0, 0.5)
        self.assertIsNotNone(obstacle)
        grid.mark_occupied(*obstacle)
        before = np.array(grid.data, copy=True)
        events = []
        manager._log_structured = events.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            completed = self._tick_no_frontier_runtime(
                manager,
                now=10.0,
                pose=(2.0, 0.0, 0.0),
            ).completed

        self.assertFalse(completed)
        self.assertEqual(manager._exploration.no_frontier_escape_state, "moving")
        self.assertEqual(manager._exploration.no_frontier_escape_target, (1.0, 0.0))
        self.assertGreater(manager._exploration.no_frontier_escape_deadline, 10.0)
        self.assertTrue(astar_plan(grid, (2.0, 0.0), (1.0, 0.0)).reached)
        np.testing.assert_array_equal(grid.data, before)
        self.assertEqual(events[-1]["event"], "no_frontier_retreat_started")

    def test_no_frontier_close_wall_without_safe_trace_finishes_floor_once(self):
        """没有安全沿迹候选时立即结束本层，不重复零速扫描。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager._map_runtime.nearest_radial_obstacle_distance = 0.33
        manager._floor_navigation_traces = {0: []}
        manager._try_probe_unknown_space = lambda pose: False
        manager._door_navigation.select_approach = lambda *args, **kwargs: None
        finish_calls = []
        manager._finish_current_floor = lambda **kwargs: finish_calls.append(kwargs)
        manager._log_structured = lambda event: None

        with patch.object(mission_manager_module, "get_time", return_value=10.0), patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            manager._tick_explore((2.0, 0.0, 0.0))

        self.assertEqual(manager._exploration.no_frontier_escape_state, "exhausted")
        self.assertEqual(len(finish_calls), 1)
        self.assertEqual(
            finish_calls[0]["reason_code"], "no_frontier_escape_exhausted"
        )

    def test_no_frontier_retreat_allows_only_one_followup_scan(self):
        """退到宽处后的复扫完成即耗尽，不重新进入无界扫描循环。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager._exploration.no_frontier_escape_state = "scan_once"
        manager._map_runtime.nearest_radial_obstacle_distance = 0.65
        manager.no_frontier_scan_duration = 4.0
        rotate_calls = []
        manager.controller.rotate_in_place = rotate_calls.append

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(loginfo=lambda *args, **kwargs: None, logwarn=lambda *args, **kwargs: None),
        ):
            with patch.object(mission_manager_module, "get_time", return_value=10.0):
                self.assertFalse(
                    self._tick_no_frontier_runtime(
                        manager,
                        now=10.0,
                        pose=(1.0, 0.0, 0.0),
                    ).completed
                )
            with patch.object(mission_manager_module, "get_time", return_value=14.1):
                self.assertTrue(
                    self._tick_no_frontier_runtime(
                        manager,
                        now=14.1,
                        pose=(1.0, 0.0, 0.0),
                    ).completed
                )

        self.assertEqual(rotate_calls, [0.45])
        self.assertEqual(manager._exploration.no_frontier_escape_state, "exhausted")

    def test_no_frontier_retreat_reverses_without_turning_then_requests_rescan(self):
        manager = self._make_explore_manager(frontiers_list=[])
        manager._exploration.no_frontier_escape_state = "moving"
        manager._exploration.no_frontier_escape_target = (1.0, 0.0)
        manager._exploration.no_frontier_escape_deadline = 20.0
        reverse_calls = []
        manager.controller.reverse_straight = lambda **kwargs: reverse_calls.append(
            kwargs
        ) or True
        events = []
        manager._log_structured = events.append

        with patch.object(mission_manager_module, "get_time", return_value=10.0):
            result = manager._exploration.tick_no_frontier_escape_motion(
                now=10.0,
                pose=(2.0, 0.0, 0.0),
                controller=manager.controller,
                current_floor=manager.current_floor,
            )
            manager._apply_exploration_runtime_result(result)
        self.assertEqual(reverse_calls, [{"reason": "no_frontier_retreat"}])
        self.assertEqual(manager._exploration.no_frontier_escape_state, "moving")

        with patch.object(mission_manager_module, "get_time", return_value=12.0):
            result = manager._exploration.tick_no_frontier_escape_motion(
                now=12.0,
                pose=(1.2, 0.0, 0.0),
                controller=manager.controller,
                current_floor=manager.current_floor,
            )
            manager._apply_exploration_runtime_result(result)

        self.assertEqual(manager._exploration.no_frontier_escape_state, "scan_once")
        self.assertIsNone(manager._exploration.no_frontier_escape_target)
        self.assertEqual(events[-1]["event"], "no_frontier_retreat_completed")

    def test_no_frontier_retreat_stalled_fast_aborts(self):
        """倒车超过 2.5 秒但实际位移不足 0.05m 时，应立即触发 retreat_stalled 快速熔断。"""
        manager = self._make_explore_manager(frontiers_list=[])
        manager._exploration.no_frontier_escape_state = "moving"
        manager._exploration.no_frontier_escape_target = (0.0, 0.0)
        manager._exploration.no_frontier_escape_deadline = 30.0
        manager._exploration.no_frontier_escape_start_time = 10.0
        manager._exploration.no_frontier_escape_start_pose = (2.0, 0.0, 0.0)
        manager.controller.reverse_straight = lambda **kwargs: True

        # 2.6 秒过去，位置几乎没变 (2.001, 0.0, 0.0) -> 位移 0.001m < 0.05m
        result = manager._exploration.tick_no_frontier_escape_motion(
            now=12.6,
            pose=(2.001, 0.0, 0.0),
            controller=manager.controller,
            current_floor=manager.current_floor,
        )
        self.assertEqual(manager._exploration.no_frontier_escape_state, "exhausted")

    def test_nearest_radial_obstacle_distance_filters_height_and_self_returns(self):
        from danger_search.mapper import MultiFloorGridMap

        mapper = MultiFloorGridMap(
            floor_height=2.6,
            resolution=0.1,
            width=8.0,
            length=8.0,
            origin_x=-4.0,
            origin_y=-4.0,
        )
        points = [
            (0.02, 0.01, 2.9),  # 过近的自体回波
            (0.20, 0.00, 2.9),  # 当前楼层实体
            (0.10, 0.00, 4.5),  # 高于障碍过滤范围
        ]

        distance = mission_manager_module.nearest_radial_obstacle_distance(
            points,
            (0.0, 0.0, 0.0),
            floor_index=1,
            mapper=mapper,
        )

        self.assertAlmostEqual(distance, 0.20)

    def test_budget_exhausted_frontier_is_confirmed_before_no_reachable_count(self):
        """预算耗尽不是不可达；完整扫描后确认出路径应立即继续探索。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug

        unresolved = Frontier(
            cells=((10, 10),), centroid=(5.0, 0.0), information_gain=20.0,
        )
        confirmed = Frontier(
            cells=((10, 10),), centroid=(4.5, 0.0), information_gain=20.0,
        )
        manager = self._make_explore_manager(
            frontiers_list=[unresolved],
            select_result=(None, []),
        )
        manager.no_frontier_scan_duration = 8.0
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1,
            reachable_count=0,
            blacklisted_count=0,
            unreachable_count=0,
            context_filtered_count=0,
            used_fallback=False,
            final_target=None,
            final_score=0.0,
            path_length=0.0,
            budget_exhausted_count=1,
        )
        confirmation_calls = []
        manager.explorer.confirm_budget_exhausted_frontier = (
            lambda *args, **kwargs: confirmation_calls.append((args, kwargs))
            or (confirmed, [(0.0, 0.0), (4.5, 0.0)])
        )
        paths = []
        manager.controller.set_path = lambda path, **kwargs: paths.append(path)
        manager.controller.rotate_in_place = lambda value: None

        with patch.object(
            mission_manager_module,
            "MissionReportingRuntime",
            SimpleNamespace(
                loginfo=lambda *args, **kwargs: None,
                logwarn=lambda *args, **kwargs: None,
            ),
        ):
            with patch.object(mission_manager_module, "get_time", return_value=10.0):
                manager._tick_explore((0.0, 0.0, 0.0))
            with patch.object(mission_manager_module, "get_time", return_value=18.0):
                manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(confirmation_calls), 1)
        self.assertIs(manager.current_frontier, confirmed)
        self.assertEqual(paths, [[(0.0, 0.0), (4.5, 0.0)]])
        self.assertEqual(manager._exploration.no_reachable_frontier_confirmations, 0)

    def test_all_inside_confirmation_increments_once_after_full_scan(self):
        """完整扫描结束后仍全部过近时，只增加一次 confirmation 且不记录 failure。"""
        from danger_search.frontier import Frontier, FrontierSelectionDebug
        frontier = Frontier(
            cells=((10, 10),), centroid=(0.30, 0.0), information_gain=10.0,
        )
        manager = self._make_explore_manager(frontiers_list=[frontier])
        manager.no_frontier_scan_duration = 8.0
        manager._try_probe_unknown_space = lambda pose: False
        manager.no_reachable_frontier_confirmations = 2
        manager.explorer.last_selection_debug = FrontierSelectionDebug(
            candidate_count=1, reachable_count=0, blacklisted_count=0,
            unreachable_count=0, context_filtered_count=0, used_fallback=False,
            final_target=None, final_score=0.0, path_length=0.0,
            inside_tolerance_count=1,
        )
        select_calls = []
        manager.explorer.select_frontier = (
            lambda *args, **kwargs: select_calls.append(kwargs) or (None, [])
        )
        manager.explorer.extract_frontiers = lambda grid: [frontier]
        manager.controller.rotate_in_place = lambda value: None
        failures = []
        manager._record_failure = lambda reason: failures.append(reason)

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )):
            for timestamp in (10.0, 10.1, 12.0, 17.9):
                with patch.object(
                    mission_manager_module, "get_time", return_value=timestamp,
                ):
                    manager._tick_explore((0.0, 0.0, 0.0))
            self.assertEqual(manager._exploration.no_reachable_frontier_confirmations, 0)
            with patch.object(mission_manager_module, "get_time", return_value=18.0):
                manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(len(select_calls), 2)
        self.assertEqual(manager._exploration.no_reachable_frontier_confirmations, 1)
        self.assertEqual(failures, [])

    def test_progress_monitor_intervention_count_remains_zero(self):
        """ProgressMonitor 主动干预次数仍为 0（diagnostic-only）。"""
        from danger_search.frontier import Frontier
        frontier = Frontier(cells=((10, 10),), centroid=(5.0, 0.0), information_gain=10.0)
        select_result = (frontier, [(0.0, 0.0), (5.0, 0.0)])
        manager = self._make_explore_manager(
            frontiers_list=[frontier],
            select_result=select_result,
        )

        cooldown_calls = []
        filter_calls = []
        monitor = ProgressMonitor(
            goal_tolerance=0.35, window=5.0, repeat=3,
            min_displacement=0.20, cooldown=10.0,
        )
        monitor.start_cooldown = lambda *a, **k: cooldown_calls.append(True)
        monitor.filter_frontiers = lambda *a, **k: filter_calls.append(True) or a[1]
        manager._progress_monitor = monitor

        clear_calls = []
        manager.controller.clear_path = lambda *a, **k: clear_calls.append(True)

        with patch.object(mission_manager_module, "MissionReportingRuntime", SimpleNamespace(
            loginfo=lambda *a, **k: None, logwarn=lambda *a, **k: None,
        )), patch.object(mission_manager_module, "get_time", return_value=10.0):
            manager._tick_explore((0.0, 0.0, 0.0))

        self.assertEqual(cooldown_calls, [])
        self.assertEqual(filter_calls, [])
        self.assertEqual(clear_calls, [])


if __name__ == "__main__":
    unittest.main()

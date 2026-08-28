"""自主搜索节点的编排层与主任务状态机。

该模块不重新实现感知、规划或控制算法，而是把它们按 10 Hz 定时器串成一条
可审计的数据流：

``内部定位/RGB-D/点云 -> 时间匹配 -> 建图与检测 -> 任务决策 -> 路径 -> /cmd_vel``

主状态按以下方向推进：

``INIT -> WAIT_CONTROLLER_READY -> EXPLORE_FLOOR``
``EXPLORE_FLOOR <-> CHANGE_FLOOR -> RETURN_HOME -> WRITE_RESULT -> FINISH``

任一不可恢复异常会进入 ``WRITE_RESULT``，先保存当前已确认检测结果，再以
``FAILED`` 结束。门、电梯和普通导航由 ``NavigationTask`` 子状态机分多次 tick
推进，ROS 服务和电梯动作不会阻塞主定时器。所有状态转换都经过
``_transition_state`` 记录原因、调用点和诊断快照。
"""
from __future__ import annotations

import math
from contextlib import nullcontext
from dataclasses import dataclass, replace
import time

import numpy as np

from danger_search.door_observation_runtime import (
    DoorRefreshConfig,
    DoorObservationRuntime,
)
from danger_search.door_navigation_runtime import DoorNavigationRuntime
from danger_search.navigation_tasks import NavigationTask, TakeElevator, OpenDoorAndPass, NavigateToPose, FAILED
from danger_search.sparse_landmark_relocalizer import LandmarkRelocalizationOutcome
from danger_search.frontier import Frontier, FrontierSelectionContext
from danger_search.frontier_planning_runtime import (
    MODE_NORMAL,
    MODE_OPPORTUNISTIC,
    MODE_PREFETCH,
    FrontierPlanOutcome,
    FrontierPlanScope,
    FrontierPlanningRuntime,
    compute_frontier_plan,
    frontier_still_relevant,
    live_frontier_failure_deltas,
    rebase_and_validate_frontier_path,
)
from danger_search.ground_support_gate import (
    CLEAR as GROUND_SUPPORT_CLEAR,
    DROP_STOPPING,
    OBSERVATION_HOLD,
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SPEED_OUT_OF_ENVELOPE_STOPPING,
    STALE_DATA_HOLD,
    SUPPORT_PATH_UNAVAILABLE,
    GroundSupportSafetyGate,
)
from danger_search.ground_support_runtime import (
    ACTION_CLEAR as GROUND_SUPPORT_ACTION_CLEAR,
    ACTION_HOLD as GROUND_SUPPORT_ACTION_HOLD,
    ACTION_NEGATIVE_OBSTACLE as GROUND_SUPPORT_ACTION_NEGATIVE_OBSTACLE,
    ACTION_PATH_REPLAN as GROUND_SUPPORT_ACTION_PATH_REPLAN,
    ACTION_TERMINAL as GROUND_SUPPORT_ACTION_TERMINAL,
    classify_gate_action,
)
from .time_utils import get_time
from danger_search.navigator import DONE, IDLE, RUNNING, STUCK, PathFollower
from danger_search.planner import (
    BUDGET_EXHAUSTED,
    DEFAULT_MAX_ITERATIONS,
    astar_path,
    astar_plan,
    path_length,
    segment_is_traversable,
    simplify_path_line_of_sight,
)
from danger_search.mapper import (
    BLACK,
    FREE,
    NEGATIVE,
    OCC,
    UNKNOWN,
    VISITED,
    FloorGrid,
)
from danger_search.rgbd_door_detector import (
    DoorPanelObservation,
)
from danger_search.landmark_drift_monitor import LandmarkDriftConfig
from danger_search.mission_reporting import MissionReportingRuntime, StatusReport
from danger_search.return_home_planner import (
    initial_entry_reverse_path,
)
from danger_search.perception_runtime import (
    INPUT_INVALID,
    INPUT_MATCHED,
    INPUT_NONE,
    INPUT_POSE_EXPIRED,
    INPUT_POSE_PENDING,
)
from danger_search.map_update_runtime import (
    ForwardObstacleObservation,
    MapUpdateRuntime,
    nearest_forward_obstacle_distance,
    nearest_forward_obstacle_observation,
    nearest_radial_obstacle_distance,
)
from danger_search.elevator_crossing_runtime import (
    ElevatorCrossingConfig,
    ElevatorCrossingRuntime,
    crossing_corridor_obstacle,
)
from danger_search.floor_transition_runtime import FloorTransitionRuntime
from danger_search.navigation_context import NavigationTaskContext
from danger_search.elevator_observation_runtime import (
    ElevatorObservationRuntime,
    ElevatorVisualScanConfig,
    ElevatorVisualScanResult,
)
from danger_search.elevator_registration_runtime import (
    ElevatorRefreshResult,
    ElevatorRegistrationConfig,
    ElevatorRegistrationResult,
)
from danger_search.elevator_navigation_runtime import (
    elevator_approach_trace_candidates as select_elevator_approach_trace_candidates,
    elevator_start_retreat_target as select_elevator_start_retreat_target,
)
from danger_search.exploration_runtime import (
    ExplorationBudgetInput,
    FrontierArrivalGainRuntime,
    ExplorationRuntimeResult,
    ExplorationRuntime,
    NoFrontierRecoveryConfig,
    decide_exploration_budget,
)
from danger_search.return_home_runtime import ReturnHomeRuntime
from danger_search.initial_entry_runtime import (
    InitialEntryConfig,
    InitialEntryRuntime,
    probe_target_for_attempt,
)
from danger_search.pose_provider import (
    PoseLookup,
    PoseSample,
)
from danger_search.localization_runtime import (
    LOCALIZATION_ALLOW,
    LOCALIZATION_WAIT_ODOMETRY,
    LocalizationRuntime,
    time_matched_pose_lookup as _time_matched_pose_lookup,
)
from danger_search.mission_components import MissionRuntimeComponents
from danger_search.mission_config import (
    MissionConfig,
    effective_max_runtime as _effective_max_runtime,
    load_mission_config,
    validate_summary_output_path as _validate_summary_output_path,
)

# 条件导入rospy用于日志输出
try:
    import rospy
except ImportError:
    rospy = None


INIT = "INIT"
WAIT_CONTROLLER_READY = "WAIT_CONTROLLER_READY"
EXPLORE_FLOOR = "EXPLORE_FLOOR"
CHANGE_FLOOR = "CHANGE_FLOOR"
RETURN_HOME = "RETURN_HOME"
WRITE_RESULT = "WRITE_RESULT"
FINISH = "FINISH"
MISSION_FAILED = "FAILED"
LOCALIZATION_READY_TOPIC = "/danger_search/localization/ready"
LOCALIZATION_HEALTH_TOPIC = "/danger_search/localization/health"
EXPLORATION_COMPLETE_RETURN_REASONS = frozenset(
    {"floor_completion_ready", "floor_completion_ready_return_home"}
)


def _return_reason_indicates_exploration_complete(reason: str | None) -> bool:
    """只有完整扫描确认的返航才表示探索完成，预算返航只表示闭环完成。"""
    return reason in EXPLORATION_COMPLETE_RETURN_REASONS


@dataclass(frozen=True)
class FloorExitCertificate:
    """楼层离开时冻结的规划事实；正常成功必须覆盖全部已探索楼层。"""

    floor: int
    map_epoch: int
    localization_epoch: int
    constraint_revision: int
    reason_code: str
    formal_reachable_count: int
    effective_reachable_count: int
    region_ids: tuple[str, ...]
    valid: bool


class MissionManager:
    """Timer 驱动的顶层任务编排器。

    本类只拥有任务状态、预算和终局语义；感知、地图、安全证据与规划约束均由
    专用组件持有，并以冻结快照或 directive 交付。PathFollower 仍是唯一速度
    发送入口，异步服务与规划结果只在主 tick 中应用。
    """

    def __init__(
        self,
        *,
        config: MissionConfig | None = None,
        components: MissionRuntimeComponents | None = None,
    ):
        # 顶层状态、全局/单层计时和楼层位置。这里先建立纯运行状态，参数加载与
        # ROS 组件构造放在后面，确保配置校验失败时尚未产生订阅或发布副作用。
        self.state = INIT
        self.start_time = get_time()
        self.floor_start_time = self.start_time
        self.home_pose: tuple[float, float, float] | None = None
        self.current_floor = 0
        self.target_floor: int | None = None
        self.return_after_floor_change = False
        self.current_frontier: Frontier | None = None
        self._service_task = None
        self._change_phase: str | None = None
        self._exploration = ExplorationRuntime()
        self._frontier_arrival_gain = FrontierArrivalGainRuntime()
        self._last_status_publish = 0.0
        # rospy.Timer 使用仿真时间；墙钟线程用该值判断 timer 是否长期未进入。
        self._last_tick_wall_time: float | None = None
        self._tick_wall_forced_shutdown_reason: str | None = None
        # 主入口探测是一段独立的小状态机：先由合法传感器寻找安全直行目标，
        # 必要时注册入口门并异步开门，轨迹同时留给最终返航反向复用。
        self._entry = InitialEntryRuntime()
        self._door_observation_runtime = DoorObservationRuntime()
        # 主入口任务只负责确认越过门面；机身完全离开门框前禁止把控制权
        # 交给可能立即侧转的普通前沿。
        self._floor_navigation_traces: dict[int, list[tuple[float, float]]] = {}
        # 多帧电梯观测由独立账本持有；编排器只决定何时读取或提交服务动作。
        self._elevator_observation = ElevatorObservationRuntime()
        self._elevator_crossing = ElevatorCrossingRuntime()
        self._node_runtime = None
        self._finished = False
        self._shutdown_requested = False
        self._transition_history: list[dict[str, object]] = []
        # 任务终态信息与“是否写过结果”分开保存。这样无论正常结束、算法失败、
        # ROS 外部关闭还是安全线程强制退出，都能且只能写出一份语义明确的结果。
        self._return_home_started = False
        self._return_home_completed = False
        self._return_home_main_entrance_crossed = False
        self._return_home = ReturnHomeRuntime()
        self._mission_success_eligible = True
        self._invalid_floor_completion_evidence: list[dict[str, object]] = []
        self._floor_exit_certificates: dict[int, FloorExitCertificate] = {}
        self._explored_floor_ids: set[int] = set()
        self._normal_completion = False
        self._terminal_outcome = "running"
        self._terminal_reason: str | None = None
        self._terminal_state: str | None = None
        self._pending_result_write_reason: str | None = None
        self._state_at_result_write: str | None = None
        self._result_write_reason: str | None = None
        self._result_write_call_site: str | None = None
        self._last_shutdown_diagnostic: dict[str, object] | None = None
        self._return_home_success = False
        self._forced_task_failure_reason: str | None = None
        self._elevator_success: bool | None = None
        self._timeout_triggered = False
        self._failure_reasons: list[str] = []
        self._last_pose: tuple[float, float, float] | None = None
        self._last_xyz: tuple[float, float, float] | None = None
        self._floor_transition_runtime = FloorTransitionRuntime()
        self._max_home_distance = 0.0
        # 导航任务层。正式 profile 禁用 manual_goal；门和电梯任务仍通过同一个
        # current_task 槽位串行执行，防止多个控制行为竞争 /cmd_vel。
        self.manual_goal: tuple[float, float] | None = None
        self.manual_mode = False
        self.current_task: NavigationTask | None = None
        self.opened_doors: set[str] = set()
        self.failed_doors: set[str] = set()
        self.active_door: str | None = None
        self._door_navigation = DoorNavigationRuntime()

        self.main_entrance_xy: tuple[float, float] | None = None
        self.main_entrance_pass_vector: tuple[float, float] | None = None

        # 前沿惯性和进度监督状态用于抑制目标频繁切换及局部循环，不会篡改原始
        # 地图；冷却到期后目标区域可以重新参与选择。
        # 负障碍安全链位于最终控制出口之前。ShadowSupportGrid 产生支撑评估，
        # GroundSupportSafetyGate 过滤控制命令；墙钟线程仅处理仿真时钟冻结场景。
        self._ground_support_sensor_mount: np.ndarray | None = None
        self._ground_support_forced_shutdown_reason: str | None = None

        # 参数及公开接口先全部校验，再由唯一装配根创建组件。MissionManager 只
        # 绑定长期组件引用，不再知道各组件的构造参数。
        self.config = config or load_mission_config()
        self.config.install(self)
        self._frontier_planning = FrontierPlanningRuntime(
            timeout=self.frontier_planning_timeout,
            preplan_min_lead=self.frontier_preplan_min_lead,
            preplan_max_lead=self.frontier_preplan_max_lead,
            opportunistic_period=self.opportunistic_replan_period,
        )
        self._frontier_plan_request_id = 0
        self._prefetched_frontier_plan: tuple[
            FrontierPlanScope, FrontierPlanOutcome
        ] | None = None
        self._frontier_plan_timeout_logged = False
        self._frontier_plan_timed_out_request_id: int | None = None
        self._entry_config = InitialEntryConfig.from_section(self.config.entry)
        self._door_refresh_config = DoorRefreshConfig(
            timeout=float(self.config.entry["door_refresh_timeout"]),
            maximum_center_shift=float(
                self.config.entry["door_refresh_max_center_shift"]
            ),
            clearance_margin=float(
                self.config.entry["door_clearance_margin"]
            ),
            opening_max_distance=float(
                self.config.entry["initial_entry_opening_max_distance"]
            ),
            geometry_tolerance=float(
                self.config.entry["initial_entry_geometry_tolerance"]
            ),
            required_confirmations=int(
                self.config.entry["initial_entry_opening_confirmations"]
            ),
            sweep_half_width=float(
                self.config.mapping["ground_support_params"]["sweep_half_width"]
            ),
        )
        self._elevator_crossing_config = ElevatorCrossingConfig(
            timeout=float(self.elevator_local_crossing_timeout),
            exit_empty_fallback_timeout=float(
                self.elevator_exit_empty_fallback_timeout
            ),
            exit_empty_fallback_frames=int(
                self.elevator_exit_empty_fallback_frames
            ),
            confirmations=int(self.elevator_local_crossing_confirmations),
            confirmation_max_gap=float(
                self.elevator_local_crossing_confirmation_max_gap
            ),
            center_tolerance=float(self.elevator_local_crossing_center_tolerance),
            width_tolerance=float(self.elevator_local_crossing_width_tolerance),
            yaw_tolerance=float(self.elevator_local_crossing_yaw_tolerance),
            yaw_search_half_range=float(
                self.elevator_local_crossing_yaw_search_half_range
            ),
            yaw_search_step=float(self.elevator_local_crossing_yaw_search_step),
            maximum_center_correction=float(
                self.elevator_local_crossing_max_center_correction
            ),
            sweep_half_width=float(
                self.config.mapping["ground_support_params"]["sweep_half_width"]
            ),
        )
        self._active_elevator_id = str(
            self.floor_params.get("elevator_id", "elevator_main")
        )
        components = components or MissionRuntimeComponents.build(
            self.config,
            start_time=self.start_time,
        )
        self._components = components
        self._landmark_drift_monitor = components.landmark_drift_monitor
        self._landmark_relocalizer = components.landmark_relocalizer
        self._landmark_relocalization_operations: dict[str, dict[str, object]] = {}
        self._reporting = components.reporting
        self._perception_runtime = components.perception
        self.pose_provider = components.pose_provider
        self._localization_runtime = components.localization
        self._sync_localization_runtime_state()
        self._localization_health: bool | None = None
        self._localization_health_stamp: float | None = None
        self._map_runtime = components.map_runtime
        self.explorer = components.explorer
        self.controller = components.controller
        self._ground_support = components.ground_support
        self.building_client = components.building_client
        self._progress_monitor = components.progress_monitor
        self._floor_coordinates = components.floor_coordinates
        self._perception_mapping = components.perception_mapping

    @property
    def sensor_buffer(self):
        return self._perception_runtime.sensor_buffer

    @property
    def mapper(self):
        return self._map_runtime.mapper

    @property
    def detector(self):
        return self._map_runtime.detector

    @property
    def local_detector(self):
        return self._map_runtime.local_detector

    @property
    def rgbd_door_detector(self):
        return self._map_runtime.rgbd_door_detector

    @property
    def tracker(self):
        return self._map_runtime.tracker

    @property
    def _ground_support_gate(self):
        return self._ground_support.gate

    @property
    def _ground_support_grid(self):
        return self._ground_support.grid

    @property
    def _ground_support_map_epoch(self) -> int:
        return self._ground_support.map_epoch

    @property
    def _ground_support_localization_epoch(self) -> int:
        return self._ground_support.localization_epoch

    @property
    def _floor_transition_guard(self):
        return self._floor_transition_runtime.snapshot

    @property
    def reporting(self) -> MissionReportingRuntime:
        return self._reporting

    @property
    def ground_support_gate(self) -> GroundSupportSafetyGate | None:
        return self._ground_support.gate

    @property
    def ground_support_assessment_period(self) -> float:
        return float(self.ground_support_params["assessment_period"])

    def attach_node_runtime(self, runtime) -> None:
        """注入唯一 ROS 生命周期边缘；重复绑定不同实例属于架构错误。"""
        current = getattr(self, "_node_runtime", None)
        if current is not None and current is not runtime:
            raise RuntimeError("MissionManager 已绑定其他节点运行时")
        self._node_runtime = runtime

    def navigation_task_context(self) -> NavigationTaskContext:
        """冻结当前 tick 的子任务依赖，禁止任务反向读取编排器私有状态。"""
        return NavigationTaskContext(
            controller=self.controller,
            mapper=self.mapper,
            building_client=self.building_client,
            current_floor=int(self.current_floor),
            planning_constraints=self._planning_constraints(),
            door_pass_max_linear=getattr(self, "door_pass_max_linear", None),
            open_door_geometry_refresh_enabled=True,
            elevator_geometry_refresh_enabled=bool(
                getattr(self, "elevator_geometry_refresh_enabled", True)
            ),
            elevator_geometry_refresh_standoff=float(
                getattr(self, "elevator_geometry_refresh_standoff", 1.2)
            ),
            elevator_opening_confirmation_timeout=float(
                getattr(self, "elevator_opening_confirmation_timeout", 15.0)
            ),
            elevator_local_crossing_geometry_enabled=bool(
                getattr(self, "elevator_local_crossing_geometry_enabled", True)
            ),
            elevator_local_crossing_timeout=float(
                getattr(self, "elevator_local_crossing_timeout", 4.0)
            ),
            elevator_local_crossing_max_center_correction=float(
                getattr(self, "elevator_local_crossing_max_center_correction", 0.8)
            ),
            elevator_stuck_retry_limit=int(self.elevator_stuck_retry_limit),
            exit_first_max_linear=getattr(self, "exit_first_max_linear", None),
            exit_cabin_backoff_enabled=bool(
                getattr(self, "exit_cabin_backoff_enabled", False)
            ),
            clear_opened_door_area_fn=self.clear_opened_door_area,
            clear_elevator_crossing_area_fn=self.clear_elevator_crossing_area,
            refresh_open_door_geometry_fn=self.refresh_open_door_geometry,
            refresh_elevator_geometry_fn=self.refresh_elevator_geometry,
            refresh_elevator_crossing_geometry_fn=self.refresh_elevator_crossing_geometry,
            elevator_crossing_obstacle_fn=self.elevator_crossing_obstacle,
            record_failure_fn=self._record_failure,
            log_structured_fn=self._log_structured,
            begin_floor_transition_fn=self.begin_floor_transition,
            confirm_floor_transition_fn=self.confirm_floor_transition,
            begin_door_open_confirmation_fn=self.begin_elevator_door_open_confirmation,
            confirm_door_open_fn=self.confirm_elevator_door_open,
            finish_door_open_confirmation_fn=self.finish_elevator_door_open_confirmation,
            elevator_visual_preparation_pending_fn=self.elevator_visual_preparation_pending,
            recover_with_rotation_clearance_fn=self._recover_with_rotation_clearance,
            elevator_start_retreat_target_fn=self.elevator_start_retreat_target,
            elevator_approach_trace_candidates_fn=self.elevator_approach_trace_candidates,
            plan_path_fn=self._plan_path,
            segment_is_traversable_fn=self._segment_is_traversable,
            elevator_async_target_door_close_enabled=bool(
                getattr(self, "elevator_async_target_door_close_enabled", True)
            ),
            relocalize_landmark_fn=self.relocalize_landmark,
            latest_cloud_stamp_fn=lambda: self._map_runtime.last_cloud_stamp,
            elevator_opening_confirmation_retry_limit=int(
                getattr(self, "elevator_opening_confirmation_retry_limit", 1)
            ),
            elevator_opening_confirmation_retry_timeout=float(
                getattr(self, "elevator_opening_confirmation_retry_timeout", 10.0)
            ),
            elevator_opening_confirmation_sensor_refresh=float(
                getattr(self, "elevator_opening_confirmation_sensor_refresh", 0.5)
            ),
        )

    def _planning_constraints(self):
        return self._ground_support.planning_constraints(self.current_floor)

    def _plan_path(
        self,
        grid,
        start_xy,
        goal_xy,
        max_iterations=DEFAULT_MAX_ITERATIONS,
    ):
        return astar_plan(
            grid,
            start_xy,
            goal_xy,
            max_iterations=max_iterations,
            constraints=self._planning_constraints(),
        )

    def _segment_is_traversable(self, grid, start_xy, end_xy) -> bool:
        return segment_is_traversable(
            grid,
            start_xy,
            end_xy,
            constraints=self._planning_constraints(),
        )

    def building_services_required(self) -> bool:
        return _building_services_required(
            max_floor=self.max_floor,
            public_door_ids=self.building_client.public_door_ids,
        )

    def start_elevator_visual_preparation(self) -> None:
        self._elevator_observation.start_visual_preparation(
            rgbd_enabled=self.rgbd_door_detector is not None,
            max_floor=self.max_floor,
            current_floor=self.current_floor,
            building_features=self.building_features,
            building_client=self.building_client,
        )

    def elevator_visual_preparation_pending(self, door_id: str) -> bool:
        return self._elevator_observation.visual_preparation_pending(door_id)

    def _elevator_visual_scan_config(self) -> ElevatorVisualScanConfig:
        return ElevatorVisualScanConfig(
            duration=float(self.elevator_visual_scan_duration),
            angular_speed=float(self.elevator_visual_scan_angular_speed),
            required_rotation=float(self.elevator_visual_scan_required_rotation),
            clockwise=bool(getattr(self, "elevator_visual_scan_clockwise", True)),
        )

    def _elevator_registration_config(self) -> ElevatorRegistrationConfig:
        return ElevatorRegistrationConfig(
            visual_min_width=float(self.elevator_visual_min_width),
            visual_max_width=float(self.elevator_visual_max_width),
            visual_max_distance=float(self.elevator_visual_max_distance),
            visual_confirmations=int(self.elevator_visual_confirmations),
            visual_confirmation_max_gap=float(
                self.elevator_visual_confirmation_max_gap
            ),
            geometry_tolerance=float(self.initial_entry_geometry_tolerance),
            door_pass_offset=float(self.door_pass_offset),
            livox_enabled=bool(self.livox_elevator_registration_enabled),
            refresh_timeout=float(self.elevator_geometry_refresh_timeout),
            refresh_max_shift=float(self.elevator_geometry_refresh_max_shift),
            refresh_max_yaw_change=float(
                self.elevator_geometry_refresh_max_yaw_change
            ),
        )

    def _apply_elevator_registration_result(
        self,
        result: ElevatorRegistrationResult,
    ) -> None:
        if result.active_elevator_id is not None:
            self._active_elevator_id = result.active_elevator_id
        if result.info_message is not None:
            MissionReportingRuntime.loginfo(*result.info_message)
        for event in result.structured_events:
            self._log_structured(event)
        if result.registered:
            try:
                door_id = self.building_client.elevator_door_id(self.current_floor)
            except ValueError:
                return
            self._freeze_landmark_anchor("elevator", door_id)

    def _current_landmark_frames(self) -> list[tuple[tuple[float, float, float], ...]]:
        """只返回当前楼层、当前定位连续段内的已配准 Livox 帧。"""
        epoch = int(getattr(self.pose_provider, "continuity_epoch", 0))
        return [
            frame.points
            for frame in tuple(
                getattr(self._map_runtime, "recent_cloud_frames", ())
            )
            if int(frame.floor) == int(self.current_floor)
            and int(frame.localization_epoch) == epoch
        ]

    def _freeze_landmark_anchor(self, landmark_id: str, door_id: str) -> bool:
        """冻结首次合法静态结构；后续语义门几何不得覆盖。"""
        relocalizer = getattr(self, "_landmark_relocalizer", None)
        door = getattr(self.building_client, "doors", {}).get(str(door_id))
        if relocalizer is None or door is None:
            return False
        frames = self._current_landmark_frames()
        if len(frames) < int(relocalizer.config.required_frames):
            return False
        frozen = relocalizer.freeze_anchor(
            landmark_id,
            floor=int(door.floor_index),
            world_xy=(float(door.x), float(door.y)),
            outward_yaw=float(door.yaw),
            width=float(door.width),
            frames=frames,
            floor_height=float(self.mapper.floor_height),
        )
        if frozen:
            self._log_structured(
                {
                    "event": "sparse_landmark_anchor_frozen",
                    "landmark_id": str(landmark_id),
                    "door_id": str(door_id),
                    "floor": int(door.floor_index),
                    "frame_count": len(frames),
                }
            )
        return bool(frozen)

    def _freeze_available_landmark_anchors(self) -> None:
        """注册当刻帧数不足时，在后续新点云到达后补冻结一次。"""
        relocalizer = getattr(self, "_landmark_relocalizer", None)
        if relocalizer is None:
            return
        doors = getattr(self.building_client, "doors", {})
        if "main_entrance" not in relocalizer.anchors and "main_entrance" in doors:
            self._freeze_landmark_anchor("main_entrance", "main_entrance")
        if "elevator" in relocalizer.anchors:
            return
        try:
            door_id = self.building_client.elevator_door_id(self.current_floor)
        except (AttributeError, ValueError):
            return
        if door_id in doors:
            self._freeze_landmark_anchor("elevator", door_id)

    def relocalize_landmark(
        self,
        landmark_id: str,
        door_id: str,
        pose: tuple[float, float, float],
        operation_key: str,
    ) -> LandmarkRelocalizationOutcome:
        """在已停车的任务阶段评估锚点，并等待定位节点原子回执。"""
        del pose  # 匹配输入是与里程计同坐标的已配准点云。
        key = str(operation_key)
        landmark = str(landmark_id)
        if landmark == "main_entrance" and self.state != RETURN_HOME:
            return LandmarkRelocalizationOutcome(IDLE, reason="context_not_allowed")
        relocalizer = getattr(self, "_landmark_relocalizer", None)
        anchor = (
            getattr(relocalizer, "anchors", {}).get(landmark)
            if relocalizer is not None
            else None
        )
        if anchor is None or int(anchor.floor) != int(self.current_floor):
            return LandmarkRelocalizationOutcome(IDLE, reason="anchor_unavailable")

        self.controller.stop(reason=f"landmark_relocalization_{landmark}")
        operation = self._landmark_relocalization_operations.get(key)
        now = get_time()
        if operation is not None and operation.get("sequence") is not None:
            sequence = int(operation["sequence"])
            ack = self.pose_provider.correction_ack(sequence)
            if ack is not None:
                self._landmark_relocalization_operations.pop(key, None)
                self.controller.clear_path(reason="landmark_relocalization_applied")
                self._log_structured(
                    {
                        "event": "sparse_landmark_correction_applied",
                        "landmark_id": landmark,
                        "door_id": str(door_id),
                        "sequence": sequence,
                        "correction_xy": [float(ack[0]), float(ack[1])],
                    }
                )
                return LandmarkRelocalizationOutcome(DONE, applied=True)
            if now - float(operation["started_at"]) >= relocalizer.config.correction_timeout:
                self._landmark_relocalization_operations.pop(key, None)
                return LandmarkRelocalizationOutcome(IDLE, reason="correction_ack_timeout")
            return LandmarkRelocalizationOutcome(RUNNING)

        evaluation = relocalizer.evaluate(
            landmark,
            frames=self._current_landmark_frames(),
            floor_height=float(self.mapper.floor_height),
        )
        self._log_structured(
            {
                "event": "sparse_landmark_relocalization_evaluated",
                "landmark_id": landmark,
                "door_id": str(door_id),
                "status": evaluation.status,
                "correction_xy": evaluation.correction_xy,
                "overlap": evaluation.overlap,
                "residual_p70": evaluation.residual_p70,
                "uniqueness_margin": evaluation.uniqueness_margin,
                "frame_spread": evaluation.frame_spread,
                "point_count": evaluation.point_count,
            }
        )
        if evaluation.status == "no_correction_needed":
            return LandmarkRelocalizationOutcome(DONE)
        if evaluation.status != "accepted" or evaluation.correction_xy is None:
            return LandmarkRelocalizationOutcome(IDLE, reason=evaluation.status)
        sequence = self.pose_provider.request_planar_correction(
            evaluation.correction_xy,
            maximum_correction=relocalizer.config.maximum_correction,
            maximum_linear_speed=relocalizer.config.maximum_linear_speed,
            maximum_angular_speed=relocalizer.config.maximum_angular_speed,
        )
        if sequence is None:
            return LandmarkRelocalizationOutcome(IDLE, reason="correction_request_rejected")
        self._landmark_relocalization_operations[key] = {
            "sequence": int(sequence),
            "started_at": now,
        }
        return LandmarkRelocalizationOutcome(RUNNING)

    def _apply_elevator_refresh_result(
        self,
        result: ElevatorRefreshResult,
    ) -> str:
        if result.info_message is not None:
            MissionReportingRuntime.loginfo(*result.info_message)
        if result.warning_message is not None:
            MissionReportingRuntime.logwarn(*result.warning_message)
        if result.error_message is not None:
            MissionReportingRuntime.logerr(*result.error_message)
        if result.structured_event is not None:
            if result.structured_event.get("event") == "elevator_geometry_refresh_unavailable":
                diagnostic = self._elevator_observation.visual_scan_diagnostics(
                    detector=self.rgbd_door_detector,
                    required_rotation=self.elevator_visual_scan_required_rotation,
                )
                diagnostic.update(result.structured_event)
                self._log_structured(diagnostic)
            else:
                self._log_structured(result.structured_event)
        return result.status

    def _apply_elevator_visual_scan_result(
        self,
        result: ElevatorVisualScanResult,
    ) -> None:
        if result.clear_exploration_target:
            self._frontier_arrival_gain.cancel()
            self.current_frontier = None
            self._exploration.last_frontier_target = None
            self._exploration.last_frontier_score = None
            self._exploration.last_frontier_used_fallback = False
        if result.info_message is not None:
            MissionReportingRuntime.loginfo(*result.info_message)
        if result.structured_event is not None:
            self._log_structured(result.structured_event)

    def accept_manual_goal(self, x: float, y: float) -> None:
        self.manual_mode = True
        self.manual_goal = (float(x), float(y))
        self.current_task = NavigateToPose(self.manual_goal)
        self.controller.clear_path()
        self._frontier_arrival_gain.cancel()
        self.current_frontier = None
        MissionReportingRuntime.loginfo(
            "收到新的手动导航目标点: (%.2f, %.2f)，已切换到手动目标模式！",
            self.manual_goal[0],
            self.manual_goal[1],
        )

    def accept_localization_health(self, value: bool, stamp: float) -> None:
        """接收 ROS 边缘已校验的姿态健康业务事件。"""
        stamp_value = float(stamp)
        if not math.isfinite(stamp_value):
            return
        self._localization_health = bool(value)
        self._localization_health_stamp = stamp_value
        if not self._localization_health:
            MissionReportingRuntime.logwarn_throttle(
                2.0,
                "定位姿态健康异常：%s=false",
                LOCALIZATION_HEALTH_TOPIC,
            )

    def check_ground_support_wall_timeout(
        self,
        monotonic_now: float,
    ) -> tuple[bool, str]:
        gate = self._ground_support_gate
        if gate is None:
            return False, "ground_support_gate_disabled"
        with _controller_command_transaction(self.controller):
            gate.check_wall_timeout(float(monotonic_now))
            timed_out = gate.state == SAFETY_STOP_TIMEOUT
            reason = gate.reason_code or "ground_support_safety_stop_timeout"
            if timed_out:
                self.controller.stop(reason=reason)
        return timed_out, reason

    def check_tick_wall_timeout(
        self,
        monotonic_now: float,
    ) -> tuple[bool, str]:
        """检查主任务 tick 是否长期未被 ROS 定时器调用。

        该判据只观察 tick 入口的墙钟时间，不读取支撑门状态，避免把仿真器
        停止与地面不安全混为一谈。首次 tick 前不武装，配置非正值显式关闭。
        """
        timeout = float(getattr(self, "tick_wall_stall_timeout", 0.0))
        if timeout <= 0.0:
            return False, "mission_tick_wall_stall_disabled"
        last_tick = getattr(self, "_last_tick_wall_time", None)
        if last_tick is None:
            return False, "mission_tick_wall_stall_not_armed"
        now = float(monotonic_now)
        if not math.isfinite(now) or now < float(last_tick):
            return False, "mission_tick_wall_stall_clock_invalid"
        if getattr(self, "state", None) in (FINISH, MISSION_FAILED):
            return False, "mission_tick_wall_stall_terminal"
        return (
            now - float(last_tick) >= timeout,
            "mission_tick_wall_stalled",
        )

    def handle_tick_wall_timeout(self, reason: str) -> None:
        """将 tick 墙钟停滞转换为一次明确的基础设施失败。"""
        if self._tick_wall_forced_shutdown_reason is not None:
            return
        self._tick_wall_forced_shutdown_reason = str(reason)
        self._mission_success_eligible = False
        self._record_failure(str(reason))
        self.controller.stop(reason=str(reason))
        self._shutdown_requested = True

    def handle_ground_support_wall_timeout(self, reason: str) -> None:
        self._ground_support_forced_shutdown_reason = str(reason)
        self._mission_success_eligible = False
        self._record_failure(str(reason))
        self._shutdown_requested = True

    def handle_unexpected_exception(self, exc: Exception) -> None:
        self._record_failure(f"状态机异常: {exc}")
        self.controller.stop()
        self._mission_success_eligible = False
        if self.state not in (WRITE_RESULT, FINISH, MISSION_FAILED):
            self._transition_state(
                WRITE_RESULT,
                reason_code="state_machine_exception",
                call_site="MissionNodeRuntime._on_timer",
                diagnostic_context={"exception": repr(exc)},
            )

    def handle_shutdown(self) -> None:
        """处理节点关闭业务语义；ROS 资源由 MissionNodeRuntime 管理。"""
        write_reason = (
            getattr(self, "_tick_wall_forced_shutdown_reason", None)
            or getattr(self, "_ground_support_forced_shutdown_reason", None)
            or "mission_requested_shutdown"
            if bool(getattr(self, "_shutdown_requested", False))
            else "external_ros_shutdown"
        )
        diagnostic = self._mission_diagnostic_snapshot()
        diagnostic.update(
            {
                "event": "mission_shutdown",
                "state_at_shutdown": getattr(self, "state", None),
                "shutdown_requested_internally": bool(
                    getattr(self, "_shutdown_requested", False)
                ),
                "result_already_written": bool(getattr(self, "_finished", False)),
                "result_write_reason": write_reason,
            }
        )
        self._last_shutdown_diagnostic = diagnostic
        self._log_structured(diagnostic)
        self.controller.stop(repeat=5)
        if not bool(getattr(self, "_finished", False)):
            self._write_result(
                reason_code=write_reason,
                call_site="MissionManager.handle_shutdown",
            )

    def _get_localization_runtime(
        self,
        *,
        start_time_fallback: float = 0.0,
    ) -> LocalizationRuntime:
        """返回统一定位运行时，并兼容历史轻量测试对象。"""
        provider = getattr(self, "pose_provider", None)
        runtime = getattr(self, "_localization_runtime", None)
        if runtime is not None and runtime.pose_provider is provider:
            return runtime

        runtime = LocalizationRuntime(
            provider,
            stale_minimum=float(getattr(self, "localization_stale_minimum", 0.4)),
            unavailable_timeout=float(
                getattr(self, "localization_unavailable_timeout", 3.0)
            ),
            start_time=float(getattr(self, "start_time", start_time_fallback)),
        )
        runtime.restore_health_state(
            unavailable_since=getattr(
                self, "_localization_unavailable_since", None
            ),
            stale_reported=bool(
                getattr(self, "_localization_stale_reported", False)
            ),
            failure_reported=bool(
                getattr(self, "_localization_failure_reported", False)
            ),
        )
        self._localization_runtime = runtime
        self._sync_localization_runtime_state()
        return runtime

    def _sync_localization_runtime_state(self) -> None:
        """保留旧诊断字段，避免检查点和轻量测试失去可读性。"""
        runtime = getattr(self, "_localization_runtime", None)
        if runtime is None:
            return
        self._localization_unavailable_since = runtime.unavailable_since
        self._localization_stale_reported = runtime.stale_reported
        self._localization_failure_reported = runtime.failure_reported
        self._pose_lookup_stats = runtime.lookup_stats()

    def _apply_localization_progress_decision(
        self,
        pose_sample: PoseSample | None,
        *,
        now: float | None = None,
    ) -> bool:
        """执行定位运行时的冻结决策；返回是否允许本 tick 继续。"""
        current_time = float(get_time() if now is None else now)
        runtime = self._get_localization_runtime(
            start_time_fallback=current_time,
        )
        decision = runtime.decide_progress(
            pose_sample,
            now=current_time,
            stance_health=getattr(self, "_localization_health", None),
            stance_required=bool(
                getattr(self, "localization_stance_health_required", True)
            ),
        )
        self._sync_localization_runtime_state()
        assessment = decision.assessment

        if decision.action == LOCALIZATION_ALLOW:
            if assessment is not None and assessment.recovered:
                self._log_structured(
                    {
                        "event": "localization_recovered",
                        "elapsed": round(assessment.elapsed, 3),
                        "unavailable_duration": round(
                            assessment.unavailable_duration, 3
                        ),
                        "pose_stamp": assessment.pose_stamp,
                        "pose_age": round(assessment.pose_age, 4),
                        "stale_limit": round(assessment.stale_limit, 4),
                    }
                )
            return True

        self.controller.stop(reason=decision.stop_reason)
        if decision.action == LOCALIZATION_WAIT_ODOMETRY or decision.reason_code == "localization_unavailable":
            reset_progress = getattr(self.controller, "reset_progress_timer", None)
            if callable(reset_progress):
                reset_progress(
                    pose_sample.xy_yaw if pose_sample is not None else None
                )
            if assessment is not None and assessment.stale_started:
                self._log_structured(
                    {
                        "event": "localization_stale",
                        "elapsed": round(assessment.elapsed, 3),
                        "pose_stamp": assessment.pose_stamp,
                        "pose_age": (
                            round(assessment.pose_age, 4)
                            if math.isfinite(assessment.pose_age)
                            else None
                        ),
                        "observed_period": round(
                            assessment.observed_period, 4
                        ),
                        "stale_limit": round(assessment.stale_limit, 4),
                    }
                )
        if not decision.terminal:
            return False

        self._mission_success_eligible = False
        self._record_failure(decision.failure_reason)
        if self.state not in {WRITE_RESULT, FINISH, MISSION_FAILED}:
            context = None
            if assessment is not None:
                context = {
                    "unavailable_duration": round(
                        assessment.unavailable_duration, 3
                    ),
                    "pose_age": (
                        round(assessment.pose_age, 4)
                        if math.isfinite(assessment.pose_age)
                        else None
                    ),
                    "stale_limit": round(assessment.stale_limit, 4),
                }
            self._transition_state(
                WRITE_RESULT,
                reason_code=decision.reason_code,
                call_site="MissionManager.tick:localization_decision",
                diagnostic_context=context,
            )
        return False

    def tick(self) -> None:
        """推进一次主状态机。

        每轮严格按“终态/预算检查—读取快照—定位健康—感知更新—安全门控—
        当前子任务—顶层状态”的顺序执行。前面的环节一旦要求等待或停车就立即
        返回，后续规划不会基于陈旧数据继续推进。
        """
        self._last_tick_wall_time = time.monotonic()
        self._publish_status()
        if self.state in (FINISH, MISSION_FAILED):
            self.controller.stop()
            outcome = "finished" if self.state == FINISH else "failed"
            self._request_shutdown(f"danger_search mission {outcome}")
            return
        # max_runtime 是探索预算。预算耗尽后仍必须让已经启动的返航闭环完成，
        # 否则会在电梯、主门或写结果的最后一步把安全收尾误判成任务失败。
        return_closure_active = bool(
            self._return_home_started
            and self.state in {RETURN_HOME, CHANGE_FLOOR, WRITE_RESULT}
        )
        if self._elapsed() >= self.max_runtime and not return_closure_active:
            MissionReportingRuntime.logwarn("接近时间预算，写出当前检测结果")
            self._timeout_triggered = True
            self._record_failure("探索超时")
            self._mission_success_eligible = False
            self._transition_state(
                WRITE_RESULT,
                reason_code="mission_timeout",
                call_site="MissionManager.tick:max_runtime",
            )
        if self.state == WRITE_RESULT:
            self._write_result(
                reason_code=getattr(self, "_pending_result_write_reason", None),
                call_site="MissionManager.tick:write_result",
            )
            terminal_state = FINISH if self._normal_completion else MISSION_FAILED
            terminal_reason = "normal_result_written" if self._normal_completion else "failed_result_written"
            self._transition_state(
                terminal_state,
                reason_code=terminal_reason,
                call_site="MissionManager.tick:write_result",
            )
            outcome = "finished" if self._normal_completion else "failed"
            self._request_shutdown(f"danger_search mission {outcome}")
            return

        snapshot = self._perception_runtime.snapshot()
        pose_provider = getattr(self, "pose_provider", None)
        latest_pose = getattr(pose_provider, "latest", None)
        pose_sample = latest_pose() if callable(latest_pose) else None
        continuity_epoch = getattr(pose_provider, "continuity_epoch", None)
        if continuity_epoch is not None:
            with _controller_command_transaction(self.controller):
                self._ground_support.synchronize_localization_epoch(
                    int(continuity_epoch)
                )
        if not self._apply_localization_progress_decision(pose_sample):
            return
        pose = pose_sample.xy_yaw
        xyz = pose_sample.xyz

        self._last_xyz = xyz
        self._update_pose_progress(pose)
        self._record_initial_entry_trace_pose(pose)
        self._record_floor_navigation_trace_pose(pose)
        self._sync_ground_support_task_context(pose)
        self._ground_support.observe_odometry(self.controller, pose_sample)
        self._update_mapping_and_detection(snapshot, pose, xyz, pose_sample)
        self._freeze_available_landmark_anchors()
        constraint_update = self._ground_support.apply_confirmed_support(self.mapper)
        if constraint_update.revoked_region_ids:
            self._log_structured(
                {
                    "event": "ground_support_constraint_revoked",
                    "elapsed": round(float(self._elapsed()), 3),
                    "region_ids": list(constraint_update.revoked_region_ids),
                }
            )
        if self._handle_ground_support_control(pose):
            return

        if self.current_task is not None:
            status = self._tick_current_task_status(pose)
            if status in (DONE, FAILED):
                completed_task_name = getattr(self.current_task, "name", type(self.current_task).__name__)
                completed_task_phase = getattr(self.current_task, "phase", None)
                completed_task_failure_reason = getattr(
                    self.current_task, "failure_reason", None
                )
                is_initial_entry_task = False
                if isinstance(self.current_task, OpenDoorAndPass):
                    is_initial_entry_task = (
                        self.current_task.door_id == "main_entrance"
                        and self._entry.origin_pose is not None
                        and not self._entry.completed
                    )
                    if status == DONE:
                        self.opened_doors.add(self.current_task.door_id)
                        self._door_navigation.record_completed(
                            self.current_task,
                            pose,
                            now=get_time(),
                            transition_duration=self.door_transition_duration,
                            context_duration=self.post_door_context_max_duration,
                        )

                        # 仅当主入口真正成功通过后才记录禁回半平面，避免失败时
                        # 把上一次其他门的过门方向错误地写成永久禁回区域。
                        if self.current_task.door_id == "main_entrance":
                            self.main_entrance_xy = self._door_navigation.last_door_xy
                            self.main_entrance_pass_vector = self._door_navigation.last_pass_vector
                            self._entry.schedule_clearance(
                                door_xy=self._door_navigation.last_door_xy,
                                pass_vector=self._door_navigation.last_pass_vector,
                                pass_offset=self.door_pass_offset,
                                minimum_distance=self.post_door_min_distance,
                            )
                    else:
                        self.failed_doors.add(self.current_task.door_id)
                    if is_initial_entry_task:
                        self._entry.completed = status == DONE
                        self._entry.failed = status == FAILED
                    if (
                        self.state == RETURN_HOME
                        and self.current_task.door_id == "main_entrance"
                        and status == DONE
                    ):
                        self._return_home_main_entrance_crossed = True
                self.active_door = None
                if status == FAILED:
                    MissionReportingRuntime.logwarn("当前任务 %s 失败，放弃该门/任务", self.current_task.name)

                if isinstance(self.current_task, TakeElevator) and status == DONE:
                    self._elevator_success = True
                    self.current_floor = self.current_task.target_floor
                    self.controller.clear_path()
                    self._frontier_arrival_gain.reset()
                    self.current_frontier = None
                    self._exploration.last_frontier_target = None
                    self._exploration.last_frontier_score = None
                    self._exploration.last_frontier_used_fallback = False
                    now = get_time()
                    self._record_elevator_exit_context(
                        self.current_task,
                        pose,
                        now=now,
                    )
                    self.floor_start_time = now
                    self._exploration.floor_timeout_logged = False
                elif isinstance(self.current_task, TakeElevator) and status == FAILED:
                    # 已发起过电梯任务不等于成功；摘要必须记录真实终态。
                    self._elevator_success = False

                self.current_task = None

                if status == FAILED and is_initial_entry_task:
                    reason = "main_entrance_crossing_failed"
                    self._record_failure(reason)
                    self._mission_success_eligible = False
                    self._transition_state(
                        WRITE_RESULT,
                        reason_code=reason,
                        call_site="MissionManager.tick:initial_entry_failed",
                        diagnostic_context={
                            "task": completed_task_name,
                            "task_phase": completed_task_phase,
                        },
                    )
                    return

                if self.state == CHANGE_FLOOR:
                    outcome = "completed" if status == DONE else "failed"
                    destination = "return_home" if self.return_after_floor_change else "explore_floor"
                    if status == FAILED:
                        failure_text = "换层任务失败"
                        if completed_task_failure_reason:
                            failure_text += f": {completed_task_failure_reason}"
                        self._record_failure(failure_text)
                        self._mission_success_eligible = False
                        transition_guard = getattr(self, "_floor_transition_guard", None)
                        if transition_guard is not None:
                            # 运输已经开始但楼层尚未可靠闭环时，不能猜测所在楼层
                            # 并继续导航；保持零速并明确结束本轮。
                            next_state = WRITE_RESULT
                            destination = "floor_alignment_unconfirmed"
                        else:
                            next_state = WRITE_RESULT if self.return_after_floor_change else RETURN_HOME
                    else:
                        next_state = RETURN_HOME if self.return_after_floor_change else EXPLORE_FLOOR
                    transition_reason = f"floor_change_{outcome}_{destination}"
                    if status == FAILED and completed_task_failure_reason:
                        transition_reason = (
                            f"floor_change_failed_{completed_task_failure_reason}"
                        )
                    self._transition_state(
                        next_state,
                        reason_code=transition_reason,
                        call_site="MissionManager.tick:floor_change_task",
                        diagnostic_context={
                            "current_task": completed_task_name,
                            "current_task_phase": completed_task_phase,
                            "task_failure_reason": completed_task_failure_reason,
                        },
                    )
            return

        clearance_result = self._entry.tick_clearance(
            pose,
            controller=self.controller,
            max_linear=self.main_entrance_clearance_max_linear,
            grid=self.mapper.floor(self.current_floor),
            segment_is_traversable=self._segment_is_traversable,
        )
        if clearance_result.failed:
            self._apply_initial_entry_clearance_failure(clearance_result.target)
        if clearance_result.handled:
            return

        if self.state == INIT:
            if pose is None or xyz is None:
                return
            self.home_pose = pose
            if rospy is not None:
                rospy.loginfo(
                    "[MissionManager] ✅ home_pose已记录: [%.2f, %.2f, %.2f]",
                    pose[0], pose[1], pose[2]
                )
            self.current_floor = self._floor_coordinates.estimate_floor(
                xyz,
                current_floor=self.current_floor,
            )
            self.floor_start_time = get_time()
            self._transition_state(
                WAIT_CONTROLLER_READY,
                reason_code="home_pose_recorded",
                call_site="MissionManager.tick:init",
            )
            MissionReportingRuntime.loginfo("记录起点：x=%.2f y=%.2f floor=%d", pose[0], pose[1], self.current_floor)
            return

        if self.state == WAIT_CONTROLLER_READY:
            self.controller.stop()
            if self._elapsed() >= self.startup_delay:
                self._transition_state(
                    EXPLORE_FLOOR,
                    reason_code="startup_delay_elapsed",
                    call_site="MissionManager.tick:controller_ready",
                )
            return
        if self.state == EXPLORE_FLOOR:
            self._tick_explore(pose)

        elif self.state == RETURN_HOME:
            self._tick_return_home(pose)

    def begin_floor_transition(
        self,
        *,
        source_floor: int,
        target_floor: int,
        phase: str,
    ) -> None:
        """在允许的电梯服务开始运输前锁存楼层相关点云处理。"""
        source_floor = int(source_floor)
        target_floor = int(target_floor)
        if source_floor == target_floor:
            return
        existing = self._floor_transition_runtime.snapshot
        if existing is not None and (
            existing.explicit_transition
            and existing.source_floor == source_floor
            and existing.target_floor == target_floor
        ):
            return
        pose_provider = getattr(self, "pose_provider", None)
        latest_pose = getattr(pose_provider, "latest", None)
        pose_sample = latest_pose() if callable(latest_pose) else None
        odom_stamp = getattr(pose_sample, "stamp", None)
        if odom_stamp is not None and not math.isfinite(float(odom_stamp)):
            odom_stamp = None
        now = float(self._elapsed())
        guard = self._floor_transition_runtime.begin_explicit(
            source_floor=source_floor,
            target_floor=target_floor,
            phase=phase,
            now=now,
            odom_stamp=odom_stamp,
        )
        if guard is None:
            return
        with _controller_command_transaction(self.controller):
            self._ground_support.invalidate_active_assessment()
        self._log_structured(
            {
                "event": "floor_transition_ambiguous_enter",
                "enter_time": round(now, 3),
                "enter_reason": "elevator_service_transition_started",
                "source_floor": source_floor,
                "current_floor": int(self.current_floor),
                "mapper_estimated_floor": source_floor,
                "target_floor": target_floor,
                "current_task": getattr(getattr(self, "current_task", None), "name", None),
                "current_task_phase": str(phase),
                "odom_stamp": odom_stamp,
                "floor_transition_ambiguous": True,
            }
        )

    def confirm_floor_transition(self, target_floor: int) -> None:
        """目标门被传感器确认后更新逻辑楼层，等待新里程计再恢复写图。"""
        target_floor = int(target_floor)
        pose_provider = getattr(self, "pose_provider", None)
        latest_pose = getattr(pose_provider, "latest", None)
        pose_sample = latest_pose() if callable(latest_pose) else None
        odom_stamp = getattr(pose_sample, "stamp", None)
        if odom_stamp is not None and not math.isfinite(float(odom_stamp)):
            odom_stamp = None
        guard = self._floor_transition_runtime.confirm(
            target_floor,
            odom_stamp=odom_stamp,
        )
        self.current_floor = target_floor
        self.floor_start_time = get_time()
        with _controller_command_transaction(self.controller):
            self._ground_support.activate_floor(target_floor)
        self._log_structured(
            {
                "event": "floor_transition_sensor_confirmed",
                "elapsed": round(float(self._elapsed()), 3),
                "source_floor": guard.source_floor,
                "current_floor": target_floor,
                "target_floor": target_floor,
                "confirmation_reason": "elevator_service_and_target_door_sensor_confirmed",
                "odom_stamp": odom_stamp,
                "floor_transition_ambiguous": True,
            }
        )

    def _floor_scoped_updates_are_blocked(
        self,
        snapshot,
        estimated_floor: int,
        pose_sample: PoseSample | None = None,
    ) -> bool:
        """更新换层锁存，并返回当前 tick 是否应暂停楼层相关点云处理。"""
        current_floor = int(self.current_floor)
        current_task = getattr(self, "current_task", None)
        if not self._floor_transition_runtime.active and not (
            isinstance(current_task, TakeElevator)
            and int(estimated_floor) != current_floor
        ):
            return False
        odom_stamp = pose_sample.stamp if pose_sample is not None else None
        now = float(self._elapsed())
        decision = self._floor_transition_runtime.evaluate(
            current_floor=current_floor,
            estimated_floor=estimated_floor,
            is_elevator_task=isinstance(current_task, TakeElevator),
            task_target_floor=getattr(
                current_task,
                "target_floor",
                getattr(self, "target_floor", None),
            ),
            task_phase=getattr(current_task, "phase", None),
            odom_stamp=odom_stamp,
            now=now,
        )
        guard = self._floor_transition_runtime.snapshot
        if decision.entered and guard is not None:
            with _controller_command_transaction(self.controller):
                self._ground_support.invalidate_active_assessment()
            self._log_structured(
                {
                    "event": "floor_transition_ambiguous_enter",
                    "enter_time": round(now, 3),
                    "enter_reason": "take_elevator_floor_mismatch",
                    "source_floor": guard.source_floor,
                    "current_floor": current_floor,
                    "mapper_estimated_floor": int(estimated_floor),
                    "target_floor": guard.target_floor,
                    "current_task": getattr(current_task, "name", None),
                    "current_task_phase": guard.enter_phase,
                    "odom_stamp": odom_stamp,
                    "floor_transition_ambiguous": True,
                }
            )

        if decision.exited is not None:
            guard = decision.exited
            self._log_structured(
            {
                "event": "floor_transition_ambiguous_exit",
                "enter_time": round(guard.entered_at, 3),
                "exit_time": round(now, 3),
                "exit_reason": (
                    "service_sensor_transition_and_fresh_odom_confirmed"
                    if guard.explicit_transition
                    else "fresh_odom_floor_alignment_confirmed"
                ),
                "duration": round(max(0.0, now - guard.entered_at), 3),
                "source_floor": guard.source_floor,
                "current_floor": current_floor,
                "mapper_estimated_floor": int(estimated_floor),
                "target_floor": guard.target_floor,
                "enter_phase": guard.enter_phase,
                "current_task_phase": getattr(current_task, "phase", None),
                "alignment_odom_samples": guard.alignment_sample_count,
                "last_alignment_odom_stamp": guard.last_alignment_odom_stamp,
                "skipped_cloud_count": guard.skipped_cloud_count,
                "first_skipped_at": guard.first_skipped_at,
                "last_skipped_at": guard.last_skipped_at,
                "first_skipped_cloud_stamp": guard.first_skipped_cloud_stamp,
                "last_skipped_cloud_stamp": guard.last_skipped_cloud_stamp,
                "floor_transition_ambiguous": False,
            }
            )
        return decision.blocked

    def _record_skipped_floor_scoped_update(
        self,
        *,
        cloud_stamp: float | None,
        estimated_floor: int,
    ) -> None:
        now = float(self._elapsed())
        current_task = getattr(self, "current_task", None)
        phase = getattr(current_task, "phase", None)
        guard, should_log = self._floor_transition_runtime.record_skipped(
            cloud_stamp=cloud_stamp,
            current_floor=self.current_floor,
            estimated_floor=estimated_floor,
            task_phase=phase,
            now=now,
        )
        if guard is None or not should_log:
            return
        self._log_structured(
            {
                "event": "skipped_floor_scoped_update",
                "elapsed": round(now, 3),
                "source_floor": guard.source_floor,
                "current_floor": int(self.current_floor),
                "mapper_estimated_floor": int(estimated_floor),
                "target_floor": guard.target_floor,
                "current_task": (
                    getattr(current_task, "name", type(current_task).__name__)
                    if current_task is not None
                    else None
                ),
                "current_task_phase": phase,
                "cloud_stamp": cloud_stamp,
                "skipped_cloud_count": guard.skipped_cloud_count,
                "mapper_update_applied": False,
                "obstacle_query_applied": False,
                "floor_transition_ambiguous": True,
            }
        )

    def _report_floor_transition_alignment_timeout(self, estimated_floor: int) -> None:
        guard = getattr(self, "_floor_transition_guard", None)
        if guard is None or guard.task_exit_at is None or guard.timeout_reported:
            return
        now = float(self._elapsed())
        service_timeout = float(getattr(self, "floor_params", {}).get("service_timeout", 5.0))
        if now - guard.task_exit_at < max(0.0, service_timeout):
            return
        guard = self._floor_transition_runtime.mark_timeout_reported()
        if guard is None:
            return
        self._record_failure("楼层过渡对齐超时")
        self._mission_success_eligible = False
        controller = getattr(self, "controller", None)
        stop = getattr(controller, "stop", None)
        if callable(stop):
            stop(reason="floor_transition_alignment_timeout")
        current_task = getattr(self, "current_task", None)
        self._log_structured(
            {
                "event": "floor_transition_alignment_timeout",
                "enter_time": round(guard.entered_at, 3),
                "task_exit_time": round(guard.task_exit_at, 3),
                "timeout_time": round(now, 3),
                "timeout_reason": "task_exited_before_floor_alignment",
                "duration": round(max(0.0, now - guard.entered_at), 3),
                "source_floor": guard.source_floor,
                "current_floor": int(self.current_floor),
                "mapper_estimated_floor": int(estimated_floor),
                "target_floor": guard.target_floor,
                "enter_phase": guard.enter_phase,
                "current_task": (
                    getattr(current_task, "name", type(current_task).__name__)
                    if current_task is not None
                    else None
                ),
                "current_task_phase": getattr(current_task, "phase", None),
                "alignment_odom_samples": guard.alignment_sample_count,
                "skipped_cloud_count": guard.skipped_cloud_count,
                "first_skipped_at": guard.first_skipped_at,
                "last_skipped_at": guard.last_skipped_at,
                "first_skipped_cloud_stamp": guard.first_skipped_cloud_stamp,
                "last_skipped_cloud_stamp": guard.last_skipped_cloud_stamp,
                "floor_transition_ambiguous": True,
                "automatic_recovery_applied": False,
                "terminal_action": WRITE_RESULT,
            }
        )
        if getattr(self, "state", None) not in {
            WRITE_RESULT,
            FINISH,
            MISSION_FAILED,
        }:
            self._transition_state(
                WRITE_RESULT,
                reason_code="floor_transition_alignment_timeout",
                call_site=(
                    "MissionManager._report_floor_transition_alignment_timeout"
                ),
                diagnostic_context={
                    "explicit_transition": bool(guard.explicit_transition),
                    "source_floor": guard.source_floor,
                    "target_floor": guard.target_floor,
                    "mapper_estimated_floor": int(estimated_floor),
                },
            )

    def _update_mapping_and_detection(self, snapshot, pose, xyz, pose_sample=None) -> None:
        preview_cloud = self._perception_runtime.cloud_input(snapshot)
        preview_stamp = preview_cloud.stamp if preview_cloud is not None else None
        if pose_sample is None:
            pose_sample = _pose_sample_from_values(
                stamp=preview_stamp if preview_stamp is not None else 0.0,
                pose=pose,
                xyz=xyz,
            )
        localization_runtime = self._get_localization_runtime()
        cloud_preparation = self._perception_runtime.prepare_cloud(
            snapshot,
            localization_runtime=localization_runtime,
            fallback_pose_sample=pose_sample,
        )
        cloud_input = cloud_preparation.cloud_input
        cloud_lookup = cloud_preparation.pose_lookup
        mapping_sample = cloud_preparation.pose_sample
        cloud_stamp = cloud_input.stamp if cloud_input is not None else None
        cloud_generation = cloud_input.generation if cloud_input is not None else 0
        raw_mapping_xyz = mapping_sample.xyz
        estimated_floor = self._floor_coordinates.estimate_floor(
            raw_mapping_xyz,
            current_floor=self.current_floor,
        )
        mapping_xyz = self._floor_coordinates.logical_xyz(
            raw_mapping_xyz,
            self.current_floor,
        )
        floor_scoped_updates_blocked = self._floor_scoped_updates_are_blocked(
            snapshot, estimated_floor, mapping_sample
        )
        if estimated_floor == self.current_floor:
            self._map_runtime.last_floor_alignment_mismatch_key = None
        if cloud_preparation.status != INPUT_NONE:
            if cloud_preparation.status == INPUT_INVALID:
                self._perception_mapping.discard_cloud(cloud_preparation)
                self._log_structured(
                    {
                        "event": "cloud_rejected",
                        "elapsed": round(float(self._elapsed()), 3),
                        "reason": "invalid_or_missing_timestamp",
                        "cloud_generation": cloud_generation,
                    }
                )
            elif cloud_preparation.status == INPUT_POSE_PENDING:
                if self._record_pose_lookup(
                    "cloud", cloud_lookup, stamp=cloud_stamp
                ):
                    self._log_structured(
                        {
                            "event": "cloud_pose_lookup_pending",
                            "elapsed": round(float(self._elapsed()), 3),
                            "cloud_stamp": cloud_stamp,
                            "pose_delta": cloud_lookup.delta,
                        }
                    )
            elif cloud_preparation.status == INPUT_POSE_EXPIRED:
                self._record_pose_lookup("cloud", cloud_lookup)
                self._perception_mapping.discard_cloud(cloud_preparation)
                self._log_structured(
                    {
                        "event": "cloud_pose_lookup_skipped",
                        "elapsed": round(float(self._elapsed()), 3),
                        "cloud_stamp": cloud_stamp,
                        "pose_delta": cloud_lookup.delta,
                        "reason": "time_matched_localization_expired",
                    }
                )
            elif floor_scoped_updates_blocked:
                self._record_pose_lookup("cloud", cloud_lookup)
                guard = self._floor_transition_guard
                if (
                    guard is not None
                    and guard.explicit_transition
                    and guard.target_floor is not None
                    and cloud_preparation.status == INPUT_MATCHED
                ):
                    # 显式换层仍禁止写图，但保留按目标楼层切片的新 Livox，
                    # 让目标门确认不再读取运输前的旧楼层点云。
                    self._perception_mapping.cache_cloud(
                        cloud_preparation,
                        floor_index=int(guard.target_floor),
                        cloud_max_points=self.cloud_max_points,
                        cloud_stride=self.cloud_stride,
                    )
                else:
                    # 非显式歧义帧只消费不解码，避免对齐后重放旧帧。
                    self._perception_mapping.discard_cloud(cloud_preparation)
                self._record_skipped_floor_scoped_update(
                    cloud_stamp=cloud_stamp,
                    estimated_floor=estimated_floor,
                )
            elif cloud_preparation.status == INPUT_MATCHED:
                self._record_pose_lookup("cloud", cloud_lookup)
                cloud_commit = self._perception_mapping.commit_cloud(
                    cloud_preparation,
                    floor_index=self.current_floor,
                    map_epoch=self._ground_support.map_epoch_for_floor(
                        self.current_floor
                    ),
                    localization_epoch=self._ground_support.localization_epoch,
                    cloud_max_points=self.cloud_max_points,
                    cloud_stride=self.cloud_stride,
                    opened_door_exclusion=self._active_opened_door_obstacle_exclusion(),
                )
                cloud_result = cloud_commit.map_result
                if estimated_floor != self.current_floor:
                    current_task = getattr(self, "current_task", None)
                    mismatch_key = (
                        int(self.current_floor),
                        int(estimated_floor),
                        getattr(current_task, "phase", None),
                    )
                    if mismatch_key != self._map_runtime.last_floor_alignment_mismatch_key:
                        self._map_runtime.last_floor_alignment_mismatch_key = mismatch_key
                        self._log_structured(
                            {
                                "event": "floor_alignment_mismatch",
                                "elapsed": round(float(self._elapsed()), 3),
                                "odom_xyz": [round(float(value), 3) for value in mapping_xyz],
                                "mapper_estimated_floor": int(estimated_floor),
                                "current_floor": int(self.current_floor),
                                "target_floor": getattr(self, "target_floor", None),
                                "current_task": (
                                    getattr(current_task, "name", type(current_task).__name__)
                                    if current_task is not None
                                    else None
                                ),
                                "current_task_phase": (
                                    getattr(current_task, "phase", None)
                                    if current_task is not None
                                    else None
                                ),
                                "cloud_stamp": cloud_stamp,
                                "map_update_floor": int(self.current_floor),
                                "mapper_update_applied": True,
                            }
                        )
                observation = cloud_result.forward_obstacle
                self._map_runtime.nearest_forward_obstacle = observation
                self._map_runtime.nearest_radial_obstacle_distance = cloud_result.radial_obstacle_distance
                set_obstacle_distance = getattr(self.controller, "set_nearest_obstacle_distance", None)
                if callable(set_obstacle_distance):
                    set_obstacle_distance(
                        observation.distance if observation is not None else None
                    )

        self._report_floor_transition_alignment_timeout(estimated_floor)

        image_preparation = self._perception_runtime.prepare_image(
            snapshot,
            localization_runtime=localization_runtime,
        )
        image_input = image_preparation.image_input
        if image_preparation.status == INPUT_NONE:
            return
        bundle = image_input.bundle
        image_generation = image_input.generation
        image_stamps = image_input.stamps
        if image_preparation.status == INPUT_INVALID:
            self._perception_mapping.discard_image(image_preparation)
            self._log_structured(
                {
                    "event": "rgbd_rejected",
                    "elapsed": round(float(self._elapsed()), 3),
                    "reason": "invalid_or_unsynchronized_timestamps",
                    "image_generation": image_generation,
                }
            )
            return
        image_stamp = float(image_stamps[0])
        image_lookup = image_preparation.pose_lookup
        if image_preparation.status == INPUT_POSE_PENDING:
            if not self._record_pose_lookup(
                "rgbd", image_lookup, stamp=image_stamp
            ):
                return
            self._log_structured(
                {
                    "event": "rgbd_pose_lookup_pending",
                    "elapsed": round(float(self._elapsed()), 3),
                    "image_stamp": image_stamp,
                    "pose_delta": image_lookup.delta,
                }
            )
            return
        if image_preparation.status == INPUT_POSE_EXPIRED:
            self._record_pose_lookup("rgbd", image_lookup)
            self._perception_mapping.discard_image(image_preparation)
            # 不使用最近云帧位姿、TF 或 Gazebo 真值兜底，避免 RGB-D 检测
            # 与彩色图像不同步导致伪目标。
            now = get_time()
            if now - self._map_runtime.last_rgbd_pose_lookup_log_time > 5.0:
                self._map_runtime.last_rgbd_pose_lookup_log_time = now
                self._log_structured(
                    {
                        "event": "rgbd_pose_lookup_skipped",
                        "elapsed": round(float(self._elapsed()), 3),
                        "image_stamp": image_stamp,
                        "pose_delta": image_lookup.delta,
                        "reason": "time_matched_localization_expired",
                    }
                )
            return
        self._record_pose_lookup("rgbd", image_lookup)
        image_pose = image_lookup.sample
        if image_pose is None:
            self._perception_mapping.discard_image(image_preparation)
            return
        image_commit = self._perception_mapping.commit_image(
            image_preparation,
            floor_index=self.current_floor,
            camera_mount_xyz=self.camera_mount_xyz,
            camera_mount_rpy=self.camera_mount_rpy,
            depth_frame_is_optical=self.depth_frame_is_optical,
            now=get_time(),
        )
        camera_to_world = image_commit.camera_to_world
        detection_result = image_commit.detection_result
        if self.rgbd_door_detector is not None:
            door_observations = list(detection_result.door_observations)
            registration = self._elevator_observation.registration
            registration.latest_rgbd_door_stamp = image_stamp
            registration.latest_rgbd_door_observations = door_observations
            self._update_landmark_drift_diagnostic(
                door_observations,
                image_stamp=image_stamp,
            )
            self._elevator_observation.update_door_visual_change(
                bundle=bundle,
                image_stamp=image_stamp,
                camera_to_world=camera_to_world,
                detector=self.rgbd_door_detector,
                building_client=self.building_client,
            )

    def _update_landmark_drift_diagnostic(
        self,
        observations: list[DoorPanelObservation],
        *,
        image_stamp: float | None,
    ) -> None:
        """记录电梯地标残差；诊断结果不得影响本轮任务行为。"""
        monitor = getattr(self, "_landmark_drift_monitor", None)
        if monitor is None or not monitor.config.enabled:
            return
        features = getattr(self, "building_features", None)
        elevator_features = getattr(features, "elevator_features", {})
        evaluation = monitor.observe(
            observations,
            stamp=image_stamp,
            floor=int(getattr(self, "current_floor", 0)),
            ordinary_exploration=(
                getattr(self, "state", None) == EXPLORE_FLOOR
                and getattr(self, "current_task", None) is None
                and getattr(self, "_floor_transition_guard", None) is None
            ),
            elevator_count=len(elevator_features),
        )
        if evaluation.status not in {"candidate", "confirmed"}:
            return
        observation = evaluation.observation
        self._log_structured(
            {
                "event": "landmark_drift_observation",
                "status": evaluation.status,
                "elapsed": round(float(self._elapsed()), 3),
                "observation_stamp": evaluation.stamp,
                "sequence_start_stamp": evaluation.sequence_start_stamp,
                "floor": int(self.current_floor),
                "candidate_count": int(evaluation.candidate_count),
                "confirmations": int(evaluation.confirmations),
                "confirmed": bool(evaluation.confirmed),
                "residual_xy": list(evaluation.residual_xy or ()),
                "residual_norm": evaluation.residual_norm,
                "normal_residual": evaluation.normal_residual,
                "tangent_residual": evaluation.tangent_residual,
                "observation": (
                    {
                        "world_xy": list(observation.world_xy),
                        "outward_yaw": float(observation.outward_yaw),
                        "width": float(observation.width),
                        "distance": float(observation.distance),
                        "plane_rmse": float(observation.plane_rmse),
                        "depth_inlier_count": int(
                            observation.depth_inlier_count
                        ),
                        "panel_state": str(observation.panel_state),
                    }
                    if observation is not None
                    else None
                ),
            }
        )

    def _record_pose_lookup(
        self,
        stream: str,
        lookup: PoseLookup,
        *,
        stamp: float | None = None,
    ) -> bool:
        """累计时间匹配结果；只用于运行诊断，不影响任务决策。"""
        recorded = self._get_localization_runtime().record_lookup(
            stream,
            lookup,
            stamp=stamp,
        )
        self._sync_localization_runtime_state()
        return recorded


    def tick_ground_support(self, _event=None) -> None:
        """只在安全门开启时及时消费新点云，不改变 10 Hz 任务状态机节拍。"""
        gate = getattr(self, "_ground_support_gate", None)
        grid = getattr(self, "_ground_support_grid", None)
        if gate is None or grid is None:
            return
        snapshot = self._perception_runtime.snapshot()
        cloud_input = self._perception_runtime.current_cloud(snapshot)
        cloud_msg = cloud_input.message if cloud_input is not None else None
        cloud_generation = cloud_input.generation if cloud_input is not None else 0
        if (
            cloud_msg is None
            or cloud_generation
            == int(self._ground_support.last_cloud_generation)
        ):
            return
        self._ground_support.last_cloud_generation = cloud_generation
        cloud_stamp = cloud_input.stamp
        if cloud_stamp is None:
            return
        # 支撑层有独立消费游标。即使本帧因换层歧义或 TF 缺失跳过，也不得重放旧帧。
        self._ground_support.last_cloud_stamp = cloud_stamp
        pose_provider = getattr(self, "pose_provider", None)
        pose_sample = pose_provider.at(cloud_stamp) if pose_provider is not None else None
        if pose_sample is None:
            return
        pose = pose_sample.xy_yaw
        raw_xyz = pose_sample.xyz
        estimated_floor = self._floor_coordinates.estimate_floor(
            raw_xyz,
            current_floor=self.current_floor,
        )
        xyz = self._floor_coordinates.logical_xyz(raw_xyz, self.current_floor)
        current_task = getattr(self, "current_task", None)
        transition_ambiguous = (
            self._floor_transition_runtime.active
            or (
                isinstance(current_task, TakeElevator)
                and int(estimated_floor) != int(self.current_floor)
            )
        )
        if transition_ambiguous:
            with _controller_command_transaction(self.controller):
                self._ground_support.invalidate_active_assessment()
            return
        points = self._perception_runtime.read_cloud_xyz(
            cloud_input,
            max_points=self.cloud_max_points,
            stride=self.cloud_stride,
        )
        points = self._floor_coordinates.logical_points(points, self.current_floor)
        self._ground_support.assess_frame(
            controller=self.controller,
            pose_provider=self.pose_provider,
            floor_index=self.current_floor,
            pose=pose,
            pose_sample=pose_sample,
            points=points,
            cloud_stamp=cloud_stamp,
            cloud_generation=cloud_generation,
        )


    def _handle_ground_support_control(
        self,
        pose: tuple[float, float, float],
    ) -> bool:
        gate = getattr(self, "_ground_support_gate", None)
        if gate is None:
            return False
        with _controller_command_transaction(self.controller):
            check_control_timeout = getattr(gate, "check_control_timeout", None)
            if callable(check_control_timeout):
                check_control_timeout(
                    float(get_time()),
                    wall_time=time.monotonic(),
                )
            state = gate.state
            reason = gate.reason_code or "ground_support_safety_failure"
            gate_snapshot = gate.snapshot()
            if state != GROUND_SUPPORT_CLEAR:
                self.controller.stop(reason=reason)
        transition = self._ground_support.observe_gate_transition(state)
        if transition.changed:
            self._log_structured(
                {
                    "event": "ground_support_gate_transition",
                    "elapsed": round(float(self._elapsed()), 3),
                    "old_state": transition.old_state,
                    "new_state": state,
                    "reason_code": reason,
                    "gate": gate_snapshot,
                }
            )
        action = classify_gate_action(state, reason)
        if action == GROUND_SUPPORT_ACTION_CLEAR:
            return False

        reset_progress = getattr(self.controller, "reset_progress_timer", None)
        if callable(reset_progress):
            reset_progress(pose)
        # 临时 hold 和停车过程都在任务 tick 之前截断，普通 STUCK/recover 无法接管。
        if action == GROUND_SUPPORT_ACTION_HOLD:
            return True

        if action == GROUND_SUPPORT_ACTION_NEGATIVE_OBSTACLE:
            persisted = self._ground_support.persist_latched_drop_clusters(
                controller=self.controller,
                mapper=self.mapper,
                floor_index=self.current_floor,
                mission_state=self.state,
                task_active=self.current_task is not None,
                ordinary_states=frozenset({EXPLORE_FLOOR, RETURN_HOME}),
            )
            if persisted is not None:
                self.current_frontier = None
                self._exploration.last_frontier_target = None
                self._exploration.last_frontier_score = None
                self._exploration.last_frontier_used_fallback = False
                self._log_structured(
                    {
                        "event": "negative_clusters_persisted",
                        "elapsed": round(float(self._elapsed()), 3),
                        "floor": persisted.floor,
                        "mission_state": self.state,
                        "cluster_ids": list(persisted.cluster_ids),
                        "source_cell_count": persisted.source_cell_count,
                        "map_revision": persisted.map_revision,
                        "ordinary_recover_called": False,
                    }
                )
                return True
            # 局部重规划未启用、超预算或证据不完整时保持原 Gate 的锁存终局，
            # 不能把“无法安全恢复”误当成已处理。
            action = GROUND_SUPPORT_ACTION_TERMINAL
        if action == GROUND_SUPPORT_ACTION_TERMINAL:
            self._record_failure(reason)
            self._mission_success_eligible = False
            if self.state not in (WRITE_RESULT, FINISH, MISSION_FAILED):
                self._transition_state(
                    WRITE_RESULT,
                    reason_code=reason,
                    call_site="MissionManager._handle_ground_support_control",
                )
            return True

        if action == GROUND_SUPPORT_ACTION_PATH_REPLAN:
            interrupted_task = self.current_task
            task_phase = getattr(interrupted_task, "phase", None)
            interrupted_frontier_target = self._exploration.last_frontier_target
            if interrupted_task is not None:
                context_key = ("task", getattr(interrupted_task, "name", ""), task_phase)
            elif interrupted_frontier_target is not None:
                context_key = (
                    "frontier",
                    round(float(interrupted_frontier_target[0]), 3),
                    round(float(interrupted_frontier_target[1]), 3),
                )
            else:
                context_key = ("state", self.state)
            directive = self._ground_support.install_path_constraint(
                context_key=context_key,
                controller=self.controller,
            )
            if directive.action != "constraint_installed":
                self._record_failure(directive.reason_code)
                self._mission_success_eligible = False
                if self.state not in (WRITE_RESULT, FINISH, MISSION_FAILED):
                    self._transition_state(
                        WRITE_RESULT,
                        reason_code=directive.reason_code,
                        call_site=(
                            "MissionManager._handle_ground_support_control:"
                            "constraint_install_failed"
                        ),
                    )
                return True
            self.controller.clear_path(reason=reason)
            maximum_replans = max(
                0,
                int(self.ground_support_params.get("maximum_same_phase_replans", 2)),
            )
            region_exhausted = int(directive.failure_count) > maximum_replans
            if interrupted_task is None and self.state == EXPLORE_FLOOR:
                self.current_frontier = None
                self._exploration.last_frontier_target = None
                self._exploration.last_frontier_score = None
                self._exploration.last_frontier_used_fallback = False
            elif interrupted_task is not None and region_exhausted:
                self._forced_task_failure_reason = (
                    "ground_support_constraint_exhausted"
                )
            elif (
                interrupted_task is None
                and self.state == RETURN_HOME
                and region_exhausted
            ):
                self._return_home.candidate_reject_count += 1
                if self._return_home.candidate_reject_count >= int(
                    getattr(self, "return_home_max_candidate_rejects", 3)
                ):
                    self._fail_return_home_stuck(
                        reason_code="return_home_stuck_exhausted"
                    )
            self._log_structured(
                {
                    "event": "ground_support_constraint_installed",
                    "reason_code": reason,
                    "region_id": directive.region_id,
                    "failure_count": directive.failure_count,
                    "task": getattr(interrupted_task, "name", None),
                    "task_phase": task_phase,
                    "context_key": list(context_key),
                    "parent_task_preserved": interrupted_task is not None,
                }
            )
            return True
        return True


    def _sync_ground_support_task_context(
        self,
        pose: tuple[float, float, float] | None = None,
    ) -> None:
        """受控门槛及其短暂离门阶段只关闭 UNKNOWN hold，显式 DROP 保持生效。"""
        gate = getattr(self, "_ground_support_gate", None)
        setter = getattr(gate, "set_unknown_support_hold_enabled", None)
        if callable(setter):
            configured = bool(
                getattr(self, "ground_support_params", {}).get(
                    "unknown_support_hold_enabled", False
                )
            )
            enabled = _ground_support_unknown_hold_enabled(
                self.current_task,
                configured=configured,
                trusted_return_trace=self._return_entry_trace_is_trusted(pose),
            )
            if (
                enabled
                and pose is not None
                and self._door_navigation.opening_sensor_confirmed
                and get_time() <= self._door_navigation.context_until
                and not self._post_door_context_satisfied(pose)
            ):
                enabled = False
            with _controller_command_transaction(getattr(self, "controller", None)):
                setter(enabled)

    def _return_entry_trace_is_trusted(
        self,
        pose: tuple[float, float, float] | None,
    ) -> bool:
        """验证原路返航仍处于本轮已实际通过轨迹的受限作用域。"""
        runtime = getattr(self, "_return_home", None)
        scope = getattr(runtime, "entry_trace_scope", None)
        if scope is None or pose is None:
            return False
        if self.state != RETURN_HOME or not getattr(
            self, "_return_home_main_entrance_crossed", False
        ):
            runtime.entry_trace_scope = None
            self._log_structured(
                {
                    "event": "return_home_entry_trace_trust_revoked",
                    "reason_code": "entry_trace_context_ended",
                    "maximum_deviation": round(
                        float(runtime.entry_trace_maximum_observed_deviation), 3
                    ),
                }
            )
            return False
        has_path = getattr(self.controller, "has_path", None)
        path_active = bool(has_path()) if callable(has_path) else False
        snapshot = getattr(self.controller, "snapshot", None)
        path_reason = None
        if callable(snapshot):
            payload = snapshot()
            if isinstance(payload, dict):
                path_reason = payload.get("replan_reason")
        decision = runtime.evaluate_entry_trace_trust(
            pose=pose,
            floor=int(self.current_floor),
            map_epoch=int(
                self._ground_support.map_epoch_for_floor(self.current_floor)
            ),
            localization_epoch=int(self._ground_support.localization_epoch),
            path_active=path_active,
            path_reason=path_reason,
        )
        if not decision.active:
            self._log_structured(
                {
                    "event": "return_home_entry_trace_trust_revoked",
                    "reason_code": decision.reason_code,
                    "maximum_deviation": (
                        round(float(decision.maximum_deviation), 3)
                        if decision.maximum_deviation is not None
                        else None
                    ),
                }
            )
        return decision.active

    # ------------------------------------------------------------------
    # 方案 C：全局时间预算与楼层/返航决策层
    # ------------------------------------------------------------------

    def _record_progress_sample(self, grid) -> None:
        """记录当前楼层探索状态样本，用于评估近期收益。"""
        now = get_time()
        known = int(np.count_nonzero(grid.data != UNKNOWN))
        free_or_visited = int(np.count_nonzero((grid.data == FREE) | (grid.data == VISITED)))
        tracker = getattr(self, "tracker", None)
        confirmed_dangers = 0
        if tracker is not None:
            try:
                confirmed_dangers = len(tracker.confirmed_positions(now=now))
            except Exception:
                pass
        window = getattr(self, "exploration_progress_window", 15.0)
        floor_timeout = getattr(self, "floor_timeout", 180.0)
        self._exploration.record_progress(
            now=now,
            known_cells=known,
            free_or_visited_cells=free_or_visited,
            confirmed_dangers=confirmed_dangers,
            retention_seconds=max(window, floor_timeout) * 2,
        )

    def _recent_progress(self, window_seconds: float) -> dict[str, float]:
        """计算最近 window_seconds 内的探索增量（不用 coverage_ratio）。"""
        return self._exploration.recent_progress(
            now=get_time(),
            window_seconds=window_seconds,
        )

    def _estimate_return_time(self, pose: tuple[float, float, float]) -> float:
        """从当前位置返航到起点的保守时间估计（单位：秒）。

        优先使用 A* 可达路径长度除以保守速度；若不可达，使用欧氏距离。
        安全余量由配置参数 return_safety_buffer 和 return_speed_estimate 控制，
        本函数不再叠加隐藏的乘数或常数，便于用真实返航数据校准。
        """
        home_pose = getattr(self, "home_pose", None)
        if home_pose is None:
            return float(getattr(self, "max_runtime", 840.0))
        grid = self.mapper.floor(self.current_floor)
        result = self._plan_path(
            grid, (pose[0], pose[1]), (home_pose[0], home_pose[1])
        )
        if result.reached and result.path:
            distance = path_length(result.path)
        else:
            distance = math.hypot(home_pose[0] - pose[0], home_pose[1] - pose[1])
        # 保守速度由 return_speed_estimate 配置；安全余量由 return_safety_buffer 配置。
        base_time = distance / max(0.1, self.return_speed_estimate)
        # 如果需要跨楼层返航，增加换层时间
        floor_change_overhead = 0.0
        if self.current_floor != 0 and self.max_floor > 1:
            floor_change_overhead = self.floor_change_time_estimate
        return base_time + floor_change_overhead

    def _has_usable_floor_change(self, target_floor: int) -> bool:
        """检查是否存在已发现且可用的换层设施（当前只有电梯）。"""
        if self.max_floor <= 1:
            return False
        elevators = getattr(getattr(self, "building_client", None), "elevators", {})
        if not elevators:
            return False
        for elevator in elevators.values():
            lobby_poses = getattr(elevator, "lobby_poses", {})
            if target_floor in lobby_poses and self.current_floor in lobby_poses:
                return True
        return False

    def _elevator_route_ready_check(
        self,
        pose: tuple[float, float, float],
    ) -> tuple[bool, str | None, object | None]:
        """换层前路线就绪预检：电梯等待点在当前已知地图上 A* 是否可达。

        只使用当前合法 FloorGrid 与已注册电梯几何，不读取 layout/world；
        A* 沿用正式 constraints，UNKNOWN 不可通行（fail-closed）。
        返回 (ready, reason, blocker)。
        """
        if self.current_floor + 1 >= self.max_floor:
            return True, None, None
        building_client = getattr(self, "building_client", None)
        if building_client is None:
            return False, "no_usable_elevator", None
        elevators = getattr(building_client, "elevators", {})
        target_floor = self.current_floor + 1
        elevator_id = None
        for elevator_id_candidate, elevator in elevators.items():
            lobby_poses = getattr(elevator, "lobby_poses", {})
            if (
                target_floor in lobby_poses
                and self.current_floor in lobby_poses
            ):
                elevator_id = elevator_id_candidate
                break
        if elevator_id is None:
            return False, "no_usable_elevator", None
        try:
            wait_point, _ = building_client.get_elevator_points(
                elevator_id, self.current_floor
            )
        except (ValueError, TypeError):
            return False, "elevator_points_unavailable", None
        grid = self.mapper.floor(self.current_floor)
        result = astar_plan(
            grid,
            (float(pose[0]), float(pose[1])),
            (float(wait_point[0]), float(wait_point[1])),
            constraints=self._planning_constraints(),
        )
        start_xy = (float(pose[0]), float(pose[1]))
        goal_xy = (float(wait_point[0]), float(wait_point[1]))
        if result.reached:
            # 防止 A* 目标吸附假可达：目标不可通行时 A* 会把目标吸附到
            # 附近最近可通行格（起点也参与吸附），path 终点会远离真实目标。
            # 只有路径终点真正贴近电梯等待点才算就绪。
            if result.path:
                end = result.path[-1]
                if (
                    math.hypot(end[0] - goal_xy[0], end[1] - goal_xy[1])
                    <= getattr(self, "elevator_route_goal_tolerance", 0.5)
                ):
                    return True, None, None
            # 吸附假可达：目标区域实际不可达，进入 UNKNOWN/硬障碍分类。
            if self._optimistic_unknown_route_reachable(grid, start_xy, goal_xy):
                return False, "unknown_blocked", None
            return False, "goal_region_blocked", None
        if result.reason_code == "open_set_exhausted":
            # 区分 UNKNOWN 阻断与硬障碍围堵：把 UNKNOWN 当可穿后若乐观可达，
            # 则阻断源于未探索区域（延后探索可解）；否则是真实硬障碍（fail-closed）。
            if self._optimistic_unknown_route_reachable(grid, start_xy, goal_xy):
                return False, "unknown_blocked", result.blocker
        return False, result.reason_code, result.blocker

    def _optimistic_unknown_route_reachable(
        self,
        grid,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
    ) -> bool:
        """乐观可达性：副本中将 UNKNOWN 视为可通行（不改正式地图），
        仅用于把 open_set_exhausted 分类为 UNKNOWN 阻断或真实硬障碍。
        障碍核心/膨胀与 BLACK/NEGATIVE 语义在副本中保持，仍不可穿。
        """
        optimistic = grid.copy()
        for gy in range(optimistic.size_y):
            for gx in range(optimistic.size_x):
                if int(optimistic.observation_layer[gy, gx]) == UNKNOWN:
                    optimistic.set_base_state(gx, gy, FREE)
        result = astar_plan(
            optimistic,
            start_xy,
            goal_xy,
            constraints=None,
        )
        return result.reached

    def _floor_change_route_ready(
        self,
        floor_elapsed: float,
        pose: tuple[float, float, float],
        *,
        decision_reason: str,
    ) -> bool:
        """换层前路线就绪门：FINISH_FLOOR 的统一前置不变量。

        - 未到检查点（floor_elapsed < floor_timeout×route_check_ratio）直接放行；
        - 电梯等待点 A* 可达 → 放行换层；
        - UNKNOWN 阻断或当前位置暂时无法从障碍膨胀区逃逸 → 延后换层继续脱离；
          预算裁剪（effective_guidance_time < 15s）后显式失败
          elevator_route_not_ready，不盲目进入 TakeElevator；
        - HIGH_HARD/支撑约束/起点逃逸等其他不可达 → fail-closed 显式失败，
          记录 blocker，不把楼层标记为完成。
        """
        check_point = self.floor_timeout * self.route_check_ratio
        if floor_elapsed < check_point:
            return True
        ready, reason, blocker = self._elevator_route_ready_check(pose)
        if ready:
            return True
        deferred_route_reasons = {
            "unknown_blocked",
            "obstacle_inflation_escape_unreachable",
        }
        if reason not in deferred_route_reasons:
            self._record_failure(
                "elevator_route_not_ready_%s" % (reason or "blocked")
            )
            self._transition_state(
                WRITE_RESULT,
                reason_code="elevator_route_not_ready",
                call_site="MissionManager._floor_change_route_ready",
                diagnostic_context={
                    "decision_reason": decision_reason,
                    "route_reason": reason,
                    "blocker": (
                        dict(blocker) if isinstance(blocker, dict) else None
                    ),
                    "floor_elapsed": round(float(floor_elapsed), 2),
                },
            )
            return False
        remaining = self.max_runtime - self._elapsed()
        change_time = self._estimate_floor_change_time(
            self.current_floor + 1, pose
        )
        return_time = self._estimate_return_time(pose)
        effective_guidance = (
            remaining - change_time - return_time - self.return_safety_buffer
        )
        if math.isfinite(effective_guidance) and effective_guidance < 15.0:
            self._record_failure("elevator_route_not_ready_budget")
            self._transition_state(
                WRITE_RESULT,
                reason_code="elevator_route_not_ready",
                call_site="MissionManager._floor_change_route_ready",
                diagnostic_context={
                    "decision_reason": decision_reason,
                    "route_reason": reason,
                    "effective_guidance_time": round(
                        max(0.0, effective_guidance), 2
                    ),
                    "blocker": (
                        dict(blocker) if isinstance(blocker, dict) else None
                    ),
                },
            )
            return False
        MissionReportingRuntime.loginfo(
            "换层前路线未就绪，延后换层继续脱离："
            "floor=%d reason=%s effective_guidance=%.1f blocker=%s",
            self.current_floor,
            reason,
            max(0.0, effective_guidance) if math.isfinite(effective_guidance) else -1.0,
            dict(blocker) if isinstance(blocker, dict) else None,
        )
        return False

    def _estimate_floor_change_time(self, target_floor: int, pose: tuple[float, float, float]) -> float:
        """估算从当前位置换到目标楼层的总时间。"""
        if not self._has_usable_floor_change(target_floor):
            return float("inf")
        elevators = getattr(getattr(self, "building_client", None), "elevators", {})
        best_distance = float("inf")
        for elevator in elevators.values():
            lobby_poses = getattr(elevator, "lobby_poses", {})
            current_lobby = lobby_poses.get(self.current_floor)
            if current_lobby is None:
                continue
            distance = math.hypot(current_lobby[0] - pose[0], current_lobby[1] - pose[1])
            best_distance = min(best_distance, distance)
        if math.isfinite(best_distance):
            travel_time = best_distance / max(0.1, self.return_speed_estimate)
        else:
            travel_time = self.floor_change_time_estimate
        return travel_time + self.floor_change_time_estimate

    @staticmethod
    def _frontier_scan_signature(
        grid,
        pose: tuple[float, float, float],
        frontiers: list[Frontier],
    ) -> tuple[object, ...]:
        """生成扫描期可重复签名，忽略原地旋转造成的 yaw 变化。"""
        robot_cell = grid.world_to_grid(pose[0], pose[1])
        frontier_cells = tuple(
            sorted(tuple(sorted(frontier.cells)) for frontier in frontiers)
        )
        return (grid.floor_index, robot_cell, frontier_cells)

    def _start_async_frontier_plan(
        self,
        *,
        mode: str,
        grid: FloorGrid,
        pose: tuple[float, float, float],
        frontiers: list[Frontier],
        progress_snapshot,
    ) -> bool:
        """冻结一次探索规划；后台线程不得接触实时地图或控制器。"""
        if not self.async_frontier_planning_enabled or self._frontier_planning.busy:
            return False
        worker_explorer = self.explorer.planning_copy()
        grid_snapshot = grid.copy()
        constraints = self._planning_constraints()
        selection_context = self._frontier_selection_context(pose)
        start_pose = tuple(float(value) for value in pose)
        candidates = list(frontiers)
        active_target = self._exploration.last_frontier_target
        if mode == MODE_PREFETCH:
            if progress_snapshot.final_goal is None:
                return False
            start_pose = (
                float(progress_snapshot.final_goal[0]),
                float(progress_snapshot.final_goal[1]),
                float(
                    pose[2]
                    if progress_snapshot.final_heading is None
                    else progress_snapshot.final_heading
                ),
            )
            if active_target is not None:
                candidates = [
                    frontier
                    for frontier in candidates
                    if math.hypot(
                        frontier.centroid[0] - active_target[0],
                        frontier.centroid[1] - active_target[1],
                    ) > self.frontier_current_region_radius
                ]
            if selection_context is not None:
                selection_context = replace(
                    selection_context,
                    current_target=None,
                    current_score=None,
                    current_region_radius=0.0,
                    switch_score_margin=0.0,
                )
        self._frontier_plan_request_id += 1
        scope = FrontierPlanScope(
            request_id=self._frontier_plan_request_id,
            mode=mode,
            floor=int(self.current_floor),
            map_epoch=self._frontier_gain_map_epoch(),
            localization_epoch=int(self._ground_support.localization_epoch),
            constraint_revision=int(getattr(constraints, "revision", 0)),
            path_sequence=int(progress_snapshot.path_generation),
            active_target=active_target,
        )
        frozen_frontiers = tuple(candidates)
        self._frontier_planning.start(
            scope,
            compute=lambda: compute_frontier_plan(
                worker_explorer,
                grid_snapshot,
                start_pose,
                frozen_frontiers,
                selection_context=selection_context,
                planning_constraints=constraints,
            ),
        )
        self._frontier_plan_timeout_logged = False
        self._frontier_plan_timed_out_request_id = None
        if mode == MODE_OPPORTUNISTIC:
            self._frontier_planning.mark_opportunistic_started(get_time())
        return True

    def _frontier_plan_scope_current(
        self,
        scope: FrontierPlanScope,
        progress_snapshot,
    ) -> bool:
        constraints = self._planning_constraints()
        if (
            self.state != EXPLORE_FLOOR
            or self.current_task is not None
            or int(self.current_floor) != scope.floor
            or self._frontier_gain_map_epoch() != scope.map_epoch
            or int(self._ground_support.localization_epoch) != scope.localization_epoch
            or int(getattr(constraints, "revision", 0)) != scope.constraint_revision
            or int(progress_snapshot.path_generation) != scope.path_sequence
            or self._exploration.last_frontier_target != scope.active_target
        ):
            return False
        if scope.mode == MODE_NORMAL:
            return not progress_snapshot.active
        if scope.mode == MODE_OPPORTUNISTIC:
            return progress_snapshot.active and not progress_snapshot.terminal_pending
        return progress_snapshot.active or progress_snapshot.terminal_pending

    def _poll_async_frontier_plan(self, progress_snapshot):
        poll = self._frontier_planning.poll()
        if poll.status == "timed_out":
            if not self._frontier_plan_timeout_logged:
                self._frontier_plan_timeout_logged = True
                self._frontier_plan_timed_out_request_id = poll.scope.request_id
                MissionReportingRuntime.logwarn(
                    "前沿规划超过墙钟 %.1f 秒，隔离迟到结果",
                    self.frontier_planning_timeout,
                )
            return None
        if poll.status in {"idle", "pending"}:
            return None
        self._frontier_plan_timeout_logged = False
        if poll.status == "late":
            MissionReportingRuntime.logwarn("丢弃超过墙钟期限的前沿规划结果")
            return None
        if poll.status == "error":
            MissionReportingRuntime.logwarn(
                "前沿规划线程异常：%s",
                poll.error,
            )
            return None
        scope = poll.scope
        outcome = poll.outcome
        if (
            scope is None
            or not isinstance(outcome, FrontierPlanOutcome)
            or getattr(self, "_frontier_plan_timed_out_request_id", None)
            == scope.request_id
            or not self._frontier_plan_scope_current(scope, progress_snapshot)
        ):
            MissionReportingRuntime.loginfo("丢弃已过期的异步前沿规划结果")
            return None
        self.explorer.last_selection_debug = outcome.debug
        debug = outcome.debug
        self._log_structured({
            "event": "async_frontier_plan",
            "request_id": scope.request_id,
            "mode": scope.mode,
            "floor": scope.floor,
            "status": outcome.status,
            "duration": round(float(poll.duration or 0.0), 4),
            "target": (
                list(outcome.frontier.centroid)
                if outcome.frontier is not None else None
            ),
            "candidate_count": (
                int(debug.candidate_count) if debug is not None else 0
            ),
            "reachable_count": (
                int(debug.reachable_count) if debug is not None else 0
            ),
            "retained_by_inertia": bool(
                debug.retained_by_inertia if debug is not None else False
            ),
            "constraint_revision": scope.constraint_revision,
        })
        return scope, outcome

    def _validated_async_frontier_path(
        self,
        grid: FloorGrid,
        pose: tuple[float, float, float],
        scope: FrontierPlanScope,
        outcome: FrontierPlanOutcome,
    ) -> list[tuple[float, float]] | None:
        frontier = outcome.frontier
        if frontier is None or not outcome.path:
            return None
        if self.explorer.is_blacklisted(grid, frontier.centroid):
            return None
        if scope.mode != MODE_NORMAL and frontier.information_gain <= 0.0:
            return None
        if frontier.information_gain > 0.0 and not frontier_still_relevant(grid, frontier):
            return None
        return rebase_and_validate_frontier_path(
            grid,
            pose,
            outcome.path,
            planning_constraints=self._planning_constraints(),
        )

    def _install_async_frontier_path(
        self,
        *,
        grid: FloorGrid,
        pose: tuple[float, float, float],
        frontier: Frontier,
        path: list[tuple[float, float]],
        reason: str,
    ) -> None:
        """在主线程原子替换探索目标，保持唯一速度出口不变。"""
        self._frontier_arrival_gain.cancel()
        self._log_frontier_switch(frontier, path, pose)
        self.current_frontier = frontier
        self.controller.set_path(path, reason=reason)
        self._frontier_planning.reset_route_progress()
        self._frontier_planning.mark_opportunistic_started(get_time())
        self._begin_frontier_arrival_gain(grid, frontier)
        self._exploration.reset_no_frontier_escape()
        self._exploration.no_frontier_scan_until = 0.0
        self._exploration.no_reachable_frontier_confirmations = 0
        self._exploration.probe_attempt = 0

    def _tick_explore(self, pose: tuple[float, float, float]) -> None:
        self._explored_floor_ids.add(int(self.current_floor))
        grid = self.mapper.floor(self.current_floor)
        now = get_time()
        progress_monitor = getattr(self, "_progress_monitor", None)
        if progress_monitor is not None:
            progress_monitor.record_pose(now, pose)
        if self.manual_mode:
            if self.current_task is None:
                self.controller.stop()
            return

        # 公开起点位于主入口外时，先沿当前合法姿态和已确认 FREE 走廊深入，
        # 避免普通 frontier 评分优先选择入口外更近的侧向边界。目标完全由
        # 当前传感器地图推导，不读取门坐标、布局 metadata 或固定世界坐标。
        if self._run_initial_entry(pose, grid):
            return

        self._elevator_observation.log_visual_preparation_result()
        registration_result = self._elevator_observation.registration.try_register(
            pose=pose,
            current_floor=self.current_floor,
            max_floor=self.max_floor,
            current_task_present=self.current_task is not None,
            building_features=self.building_features,
            building_client=self.building_client,
            local_detector=self.local_detector,
            cloud_stamp=self._map_runtime.last_cloud_stamp,
            cloud_points=self._map_runtime.latest_cloud_points,
            floor_height=self.mapper.floor_height,
            entrance_xy=self.main_entrance_xy,
            config=self._elevator_registration_config(),
            landmark_monitor=getattr(self, "_landmark_drift_monitor", None),
            elapsed=self._elapsed(),
        )
        self._apply_elevator_registration_result(registration_result)
        elevator_scan = self._elevator_observation.tick_visual_scan(
            now=get_time(),
            pose=pose,
            current_floor=self.current_floor,
            current_task_present=self.current_task is not None,
            building_features=self.building_features,
            building_client=self.building_client,
            rgbd_detector=self.rgbd_door_detector,
            controller=self.controller,
            last_door_id=self._door_navigation.last_door_id,
            last_door_xy=self._door_navigation.last_door_xy,
            post_door_context_satisfied=self._post_door_context_satisfied(pose),
            config=self._elevator_visual_scan_config(),
        )
        self._apply_elevator_visual_scan_result(elevator_scan)
        if elevator_scan.handled:
            return
        if self._exploration.no_frontier_escape_state == "moving":
            self._apply_exploration_runtime_result(
                self._exploration.tick_no_frontier_escape_motion(
                    now=get_time(),
                    pose=pose,
                    controller=self.controller,
                    current_floor=self.current_floor,
                )
            )
            return

        floor_elapsed = now - self.floor_start_time
        mission_elapsed = self._elapsed()
        path_active = self.controller.has_path()
        # 有效路径执行期间，frontier 只在楼层软预算开始依赖其数量后才有用途。
        # 普通移动阶段跳过全图提取，避免 10 Hz 主循环被无用计算阻塞；路径为空时
        # 仍使用当前地图立即重提取，不缓存旧候选。
        budget_needs_frontiers = (
            floor_elapsed
            >= self.floor_timeout
            * float(getattr(self, "floor_finish_time_ratio", 0.75))
        )
        frontiers = (
            self.explorer.extract_frontiers(grid)
            if not path_active or budget_needs_frontiers
            else []
        )
        self._record_progress_sample(grid)

        # 方案 C：全局时间预算决策
        if floor_elapsed >= self.min_floor_explore_time:
            progress = self._recent_progress(self.exploration_progress_window)
            return_time = self._estimate_return_time(pose)
            next_floor_change_time = (
                self._estimate_floor_change_time(self.current_floor + 1, pose)
                if self.current_floor + 1 < self.max_floor
                else float("inf")
            )
            decision = decide_exploration_budget(
                ExplorationBudgetInput(
                    home_recorded=self.home_pose is not None,
                    mission_elapsed=mission_elapsed,
                    floor_elapsed=floor_elapsed,
                    max_runtime=self.max_runtime,
                    return_time=return_time,
                    return_safety_buffer=self.return_safety_buffer,
                    floor_timeout=self.floor_timeout,
                    floor_finish_time_ratio=self.floor_finish_time_ratio,
                    known_cells_growth=progress["known_cells_growth"],
                    exploration_progress_window=self.exploration_progress_window,
                    minimum_progress_cells=self.min_exploration_progress_cells,
                    frontier_count=len(frontiers),
                    minimum_remaining_frontiers=self.min_remaining_frontiers,
                    current_floor=self.current_floor,
                    maximum_floor=self.max_floor,
                    next_floor_change_time=next_floor_change_time,
                    last_floor_uses_global_budget=self.last_floor_uses_global_budget,
                )
            )
            action, reason_code = decision.action, decision.reason_code
            if action != "CONTINUE":
                # 避免相邻 tick 反复切换导致状态循环
                last_decision = self._exploration.last_budget_decision
                last_action, last_reason = last_decision or (None, None)
                last_decision_time = self._exploration.last_budget_decision_time
                now = get_time()
                if action != last_action or (now - last_decision_time) > 5.0:
                    self._exploration.last_budget_decision = (action, reason_code)
                    self._exploration.last_budget_decision_time = now
                    diagnostic = self._mission_diagnostic_snapshot()
                    diagnostic.update({
                        "event": "mission_budget_decision",
                        "action": action,
                        "reason_code": reason_code,
                        "floor_elapsed": round(floor_elapsed, 2),
                        "mission_elapsed": round(mission_elapsed, 2),
                        "max_runtime": getattr(self, "max_runtime", 840.0),
                        "floor_timeout": getattr(self, "floor_timeout", 180.0),
                        "return_time_estimate": round(self._estimate_return_time(pose), 2),
                        "frontier_count": len(frontiers),
                    })
                    self._log_structured(diagnostic)

                    if action == "RETURN_HOME":
                        self._record_floor_exit_certificate(reason_code)
                        self._transition_state(
                            RETURN_HOME,
                            reason_code=reason_code,
                            call_site="MissionManager._tick_explore:budget_decision",
                        )
                        return
                    elif action == "FINISH_FLOOR":
                        if not self._floor_change_route_ready(
                            floor_elapsed,
                            pose,
                            decision_reason=reason_code,
                        ):
                            return
                        self._finish_current_floor(
                            reason_code=reason_code,
                            call_site="MissionManager._tick_explore:budget_decision",
                        )
                        return

        if floor_elapsed >= self.floor_timeout:
            self._log_floor_budget_exhausted(floor_elapsed)

        candidate = self.local_detector.detect_controllable_door(
            robot_pose=pose,
            floor_index=self.current_floor,
            cloud_points=self._map_runtime.latest_cloud_points,
            building_client=self.building_client,
            floor_height=self.mapper.floor_height,
            allow_elevator=False,
        )
        door_decision = self._door_navigation.candidate_decision(
            candidate,
            exploring=self.state == EXPLORE_FLOOR,
            current_task_present=self.current_task is not None,
            opened_doors=self.opened_doors,
            failed_doors=self.failed_doors,
            active_door=self.active_door,
            building_client=self.building_client,
            maximum_distance=self.local_detector.max_distance,
        )
        if door_decision.failure_reason is not None:
            self._record_failure(door_decision.failure_reason)
        if door_decision.allowed:
            assert candidate is not None
            self.controller.clear_path()
            self._exploration.reset_no_frontier_escape()
            self._frontier_arrival_gain.cancel()
            self.current_frontier = None
            self._exploration.last_frontier_target = None
            self._exploration.last_frontier_score = None
            self._exploration.last_frontier_used_fallback = False
            self.active_door = candidate.door_id
            self.current_task = self._door_navigation.create_task(
                candidate.door_id,
                pass_offset=self.door_pass_offset,
                commit_distance=self.door_commit_distance,
            )
            MissionReportingRuntime.loginfo(
                "局部检测到可控动态门：door_id=%s distance=%.2f source=%s evidence=%d",
                candidate.door_id,
                candidate.distance,
                candidate.source,
                candidate.evidence_count,
            )
            return

        async_normal_result = None
        progress_snapshot = None
        if self.async_frontier_planning_enabled:
            progress_snapshot = self.controller.planning_progress_snapshot()
            self._frontier_planning.observe_route_progress(
                progress_snapshot.remaining_distance
                if progress_snapshot.active and not progress_snapshot.terminal_pending
                else None
            )
            completed_plan = self._poll_async_frontier_plan(progress_snapshot)
            if completed_plan is not None:
                scope, outcome = completed_plan
                if scope.mode == MODE_NORMAL:
                    async_normal_result = completed_plan
                elif scope.mode == MODE_PREFETCH:
                    self._prefetched_frontier_plan = completed_plan
                elif scope.mode == MODE_OPPORTUNISTIC:
                    candidate_path = self._validated_async_frontier_path(
                        grid,
                        pose,
                        scope,
                        outcome,
                    )
                    frontier = outcome.frontier
                    previous = self._exploration.last_frontier_target
                    switched_region = bool(
                        frontier is not None
                        and previous is not None
                        and math.hypot(
                            frontier.centroid[0] - previous[0],
                            frontier.centroid[1] - previous[1],
                        ) > self.frontier_current_region_radius
                    )
                    retained = bool(
                        outcome.debug is not None
                        and outcome.debug.retained_by_inertia
                    )
                    if (
                        frontier is not None
                        and candidate_path is not None
                        and switched_region
                        and not retained
                    ):
                        self._install_async_frontier_path(
                            grid=grid,
                            pose=pose,
                            frontier=frontier,
                            path=candidate_path,
                            reason="frontier_opportunistic_replan",
                        )
                        path_active = True
                        progress_snapshot = self.controller.planning_progress_snapshot()

            if (
                path_active
                and progress_snapshot is not None
                and progress_snapshot.active
                and not progress_snapshot.terminal_pending
                and self.current_frontier is not None
                and self._prefetched_frontier_plan is None
                and progress_snapshot.path_reason in {
                    "frontier_selected_path",
                    "frontier_confirmed_path",
                    "frontier_opportunistic_replan",
                    "frontier_prefetched_path",
                }
                and not self._frontier_planning.busy
            ):
                route_eta = self._frontier_planning.route_eta(
                    progress_snapshot.remaining_distance
                )
                if (
                    route_eta is not None
                    and route_eta <= self._frontier_planning.preplan_lead()
                    and self._prefetched_frontier_plan is None
                ):
                    active_frontiers = (
                        frontiers
                        if budget_needs_frontiers
                        else self.explorer.extract_frontiers(grid)
                    )
                    self._start_async_frontier_plan(
                        mode=MODE_PREFETCH,
                        grid=grid,
                        pose=pose,
                        frontiers=active_frontiers,
                        progress_snapshot=progress_snapshot,
                    )
                elif self._frontier_planning.opportunistic_due(
                    now,
                    route_eta=route_eta,
                ):
                    active_frontiers = (
                        frontiers
                        if budget_needs_frontiers
                        else self.explorer.extract_frontiers(grid)
                    )
                    self._start_async_frontier_plan(
                        mode=MODE_OPPORTUNISTIC,
                        grid=grid,
                        pose=pose,
                        frontiers=active_frontiers,
                        progress_snapshot=progress_snapshot,
                    )

        if not path_active:
            # 远距离高置信度危险源主动走近核验
            tracker = getattr(self, "tracker", None)
            if (
                tracker is not None
                and hasattr(tracker, "get_pending_inspection_target")
                and getattr(self, "danger_approach_verification_enabled", True)
                and floor_elapsed < self.floor_timeout * float(getattr(self, "floor_finish_time_ratio", 0.75))
            ):
                pending_target = tracker.get_pending_inspection_target(
                    robot_xy=(pose[0], pose[1]),
                    min_observe_dist=float(getattr(self, "danger_approach_trigger_distance", 2.2)),
                    min_confidence=0.70,
                )
                if pending_target is not None:
                    from danger_search.exploration_runtime import compute_danger_approach_pose

                    approach_pose = compute_danger_approach_pose(
                        grid,
                        pose,
                        pending_target,
                        standoff_distance=float(getattr(self, "danger_approach_standoff_distance", 1.3)),
                    )
                    if approach_pose is not None:
                        plan = self._plan_path(
                            grid,
                            (pose[0], pose[1]),
                            (approach_pose[0], approach_pose[1]),
                        )
                        if plan is not None and plan.path:
                            self.controller.set_path(
                                list(plan.path),
                                reason="danger_approach_verification",
                            )
                            tracker.mark_inspected(pending_target)
                            MissionReportingRuntime.loginfo(
                                "发现远距危险源候选，执行近场走近核验：target=(%.2f, %.2f) approach=(%.2f, %.2f, yaw=%.2f)",
                                pending_target[0],
                                pending_target[1],
                                approach_pose[0],
                                approach_pose[1],
                                approach_pose[2],
                            )
                            return
                    tracker.mark_inspected(pending_target)

            scan_signature = self._frontier_scan_signature(grid, pose, frontiers)
            # 扫描门控：pose 栅格和 frontier 内容均未变时，避免逐 tick A* 空转。
            if self._exploration.no_frontier_scan_until > 0:
                now_gate = get_time()
                if now_gate < self._exploration.no_frontier_scan_until:
                    if scan_signature == self._exploration.last_no_frontier_signature:
                        event = self._exploration.rotate_for_no_frontier_scan(
                            controller=self.controller,
                            radial_clearance=self._map_runtime.nearest_radial_obstacle_distance,
                            current_floor=self.current_floor,
                            required_clearance=self.no_frontier_rotation_min_clearance,
                        )
                        if event is not None:
                            self._log_structured(event)
                        return
                    # pose 或 frontier 内容变化，清除扫描状态并立即重试选择。
                    self._exploration.no_frontier_scan_until = 0.0
            self._exploration.last_no_frontier_signature = scan_signature

            self.controller.stop(reason="frontier_planning")
            selection_context = self._frontier_selection_context(pose)

            frontier, path = None, []
            if self.async_frontier_planning_enabled and frontiers:
                if async_normal_result is None:
                    if not self._frontier_planning.busy:
                        assert progress_snapshot is not None
                        self._start_async_frontier_plan(
                            mode=MODE_NORMAL,
                            grid=grid,
                            pose=pose,
                            frontiers=frontiers,
                            progress_snapshot=progress_snapshot,
                        )
                    return
                normal_scope, normal_outcome = async_normal_result
                self.explorer.apply_planning_failure_deltas(
                    live_frontier_failure_deltas(
                        grid,
                        normal_outcome.failure_deltas,
                    )
                )
                candidate_path = self._validated_async_frontier_path(
                    grid,
                    pose,
                    normal_scope,
                    normal_outcome,
                )
                if candidate_path is not None:
                    frontier = normal_outcome.frontier
                    path = candidate_path
            elif frontiers:
                frontier, path = self.explorer.select_frontier(
                    grid,
                    pose,
                    selection_context=selection_context,
                    frontiers=frontiers,
                    planning_constraints=self._planning_constraints(),
                )

            # ProgressMonitor 仅作诊断：记录选择历史并检测退化循环，
            # 但不执行 filter_frontiers、start_cooldown、clear_path 或主动重选。
            # 过近落点已在 FrontierExplorer 源头通过 inside_goal_tolerance 过滤。
            if frontier is not None and path and progress_monitor is not None:
                progress_monitor.record_selection(now, frontier.centroid)
                is_degenerate, repeat_count, displacement, reason_code = (
                    progress_monitor.check_degenerate(now, frontier.centroid)
                )
                if is_degenerate and progress_monitor.should_log(now, interval=5.0):
                    MissionReportingRuntime.logwarn(
                        "检测到退化 frontier 循环：target=(%.2f, %.2f) "
                        "repeat=%d displacement=%.3f reason=%s",
                        frontier.centroid[0],
                        frontier.centroid[1],
                        repeat_count,
                        displacement,
                        reason_code,
                    )
                    diagnostic = self._mission_diagnostic_snapshot()
                    diagnostic.update({
                        "event": "mission_progress_monitor",
                        "action": "diagnostic_only",
                        "reason_code": reason_code,
                        "target": list(frontier.centroid),
                        "repeat_count": int(repeat_count),
                        "displacement": round(displacement, 3),
                        "region_radius": round(progress_monitor.region_radius, 3),
                    })
                    self._log_structured(diagnostic)

            self._log_post_door_frontier_debug(frontier, path, pose)

            # 没有未知前沿、或只能拿到自由空间 fallback 时，主动寻找并开启未探索的动态门。
            no_real_frontier = frontier is None or not path or frontier.information_gain <= 0.0
            if no_real_frontier or not frontiers:
                door_id = self._door_navigation.select_approach(
                    pose,
                    floor_index=self.current_floor,
                    grid=grid,
                    opened_doors=self.opened_doors,
                    failed_doors=self.failed_doors,
                    building_client=self.building_client,
                    pass_offset=self.door_pass_offset,
                    now=get_time(),
                    planning_constraints=self._planning_constraints(),
                )
                if door_id is not None:
                    self.controller.clear_path()
                    self._frontier_arrival_gain.cancel()
                    self.current_frontier = None
                    self._exploration.last_frontier_target = None
                    self._exploration.last_frontier_score = None
                    self._exploration.last_frontier_used_fallback = False
                    self.active_door = door_id
                    self.current_task = self._door_navigation.create_task(
                        door_id,
                        pass_offset=self.door_pass_offset,
                        commit_distance=self.door_commit_distance,
                    )
                    MissionReportingRuntime.loginfo(
                        "选择主动靠近并开启未探索门：door_id=%s", door_id
                    )
                    self._exploration.reset_no_frontier_escape()
                    self._exploration.no_frontier_scan_until = 0.0
                    self._exploration.no_reachable_frontier_confirmations = 0
                    self._exploration.probe_attempt = 0
                    return

            if frontier is None or not path:
                # 检查本轮无目标是否仅因 inside_goal_tolerance 拒绝
                debug = self.explorer.last_selection_debug
                planning_unresolved = bool(
                    debug is not None and debug.budget_exhausted_count > 0
                )
                all_inside_tolerance = (
                    debug is not None
                    and debug.candidate_count > 0
                    and debug.inside_tolerance_count >= debug.candidate_count - debug.blacklisted_count
                    and debug.unreachable_count == 0
                    and debug.reachable_count == 0
                )
                escape_state = self._exploration.no_frontier_escape_state
                if (
                    escape_state == "idle"
                    and (not frontiers or all_inside_tolerance)
                    and self._try_probe_unknown_space(pose)
                ):
                    self._exploration.reset_no_frontier_escape()
                    self._exploration.no_reachable_frontier_confirmations = 0
                    return
                scan_result = self._exploration.tick_no_frontier_scan(
                    now=get_time(),
                    frontiers_present=bool(frontiers),
                    pose=pose,
                    grid=grid,
                    navigation_trace=list(
                        self._floor_navigation_traces.get(
                            int(self.current_floor), ()
                        )
                    ),
                    controller=self.controller,
                    radial_clearance=self._map_runtime.nearest_radial_obstacle_distance,
                    current_floor=self.current_floor,
                    config=self._no_frontier_recovery_config(),
                    record_failure=(
                        not all_inside_tolerance and not planning_unresolved
                    ),
                    planning_constraints=self._planning_constraints(),
                )
                self._apply_exploration_runtime_result(scan_result)
                if scan_result.completed:
                    if planning_unresolved:
                        confirm = getattr(
                            self.explorer,
                            "confirm_budget_exhausted_frontier",
                            None,
                        )
                        if callable(confirm):
                            confirmed_frontier, confirmed_path = confirm(
                                grid,
                                pose,
                                selection_context=selection_context,
                                planning_constraints=self._planning_constraints(),
                            )
                            if confirmed_frontier is not None and confirmed_path:
                                self._exploration.reset_no_frontier_escape()
                                self.current_frontier = confirmed_frontier
                                self.controller.set_path(confirmed_path, reason="frontier_confirmed_path")
                                self._begin_frontier_arrival_gain(
                                    grid,
                                    confirmed_frontier,
                                )
                                self._exploration.no_reachable_frontier_confirmations = 0
                                self._exploration.probe_attempt = 0
                                MissionReportingRuntime.loginfo(
                                    "完整确认预算未决前沿可达：floor=%d "
                                    "target=(%.2f, %.2f)",
                                    self.current_floor,
                                    confirmed_frontier.centroid[0],
                                    confirmed_frontier.centroid[1],
                                )
                                return
                    if self._exploration.no_frontier_escape_state == "exhausted":
                        self._finish_current_floor(
                            reason_code="no_frontier_escape_exhausted",
                            call_site="MissionManager._tick_explore:no_frontier_escape",
                        )
                        return
                    self._exploration.no_reachable_frontier_confirmations += 1
                    if self._floor_completion_ready(grid, floor_elapsed):
                        self._finish_current_floor(
                            reason_code="floor_completion_ready",
                            call_site="MissionManager._tick_explore:completion_ready",
                        )
                    else:
                        MissionReportingRuntime.loginfo(
                            "楼层完成条件未满足，继续扫描：elapsed=%.1f/%.1f max_home=%.2f/%.2f known_area=%.2f/%.2f coverage=%.2f/%.2f no_reachable=%d/%d corridor_progress=%.2f/%.2f",
                            floor_elapsed,
                            self.min_floor_explore_time,
                            self._max_home_distance,
                            self.min_floor_travel_distance,
                            grid.known_area(),
                            self.min_known_area,
                            grid.coverage_ratio(),
                            self.explore_coverage_ratio,
                            self._exploration.no_reachable_frontier_confirmations,
                            self.no_reachable_frontier_confirmations,
                            self._known_corridor_progress(grid),
                            self.min_corridor_progress_distance,
                        )
                return
            self._log_frontier_switch(frontier, path, pose)
            self._exploration.reset_no_frontier_escape()
            self.current_frontier = frontier
            self.controller.set_path(path, reason="frontier_selected_path")
            if self.async_frontier_planning_enabled:
                self._frontier_planning.reset_route_progress()
                self._frontier_planning.mark_opportunistic_started(get_time())
            self._begin_frontier_arrival_gain(grid, frontier)
            self._exploration.no_frontier_scan_until = 0.0
            self._exploration.no_reachable_frontier_confirmations = 0
            self._exploration.probe_attempt = 0
            MissionReportingRuntime.loginfo("选择前沿：floor=%d score=%.2f target=(%.2f, %.2f)",
                          self.current_floor, frontier.score, frontier.centroid[0], frontier.centroid[1])

        active_async_scope = self._frontier_planning.active_scope
        if (
            self.async_frontier_planning_enabled
            and progress_snapshot is not None
            and progress_snapshot.terminal_pending
            and active_async_scope is not None
            and active_async_scope.mode == MODE_PREFETCH
            and getattr(self, "_frontier_plan_timed_out_request_id", None)
            != active_async_scope.request_id
        ):
            # 控制器已经锁存 DONE 并持续零速，先等到达前规划返回再消费终态，
            # 否则 clear_path 会改变路径代次，使合法预规划结果无法归属旧路径。
            return

        status = self.controller.poll_status()
        if status == STUCK:
            self._frontier_arrival_gain.cancel()
            if self.current_frontier is not None:
                # STUCK 已经过进度超时确认，临时排除目标附近候选；只改规划黑名单，
                # 不把远处拐角写成永久地图障碍。
                self.explorer.record_stuck_failure(grid, self.current_frontier.centroid)
            if self._rotation_clearance_is_safe():
                self.controller.recover()
            else:
                self._log_rotation_clearance_blocked("stuck_recovery")
                self.controller.stop(reason="unsafe_recover_rotation_blocked")
                retreat_result = self._exploration._start_no_frontier_retreat(
                    now=get_time(),
                    pose=pose,
                    grid=grid,
                    navigation_trace=list(
                        self._floor_navigation_traces.get(
                            int(self.current_floor), ()
                        )
                    ),
                    controller=self.controller,
                    current_floor=self.current_floor,
                    config=self._no_frontier_recovery_config(),
                    planning_constraints=self._planning_constraints(),
                )
                self._apply_exploration_runtime_result(retreat_result)
            self.current_frontier = None
            self._exploration.last_frontier_target = None
            self._exploration.last_frontier_score = None
            self._exploration.last_frontier_used_fallback = False
        elif status == DONE:
            if self.current_frontier is not None:
                arrival = self._frontier_arrival_gain.arrive(
                    floor=self.current_floor,
                    map_epoch=self._frontier_gain_map_epoch(),
                    known_cells=int(np.count_nonzero(grid.data != UNKNOWN)),
                )
                if arrival is not None:
                    self._log_structured(arrival.diagnostic())
                    if arrival.cooldown_center is not None:
                        self.explorer.blacklist_region(
                            grid,
                            arrival.cooldown_center,
                            radius=1.0,
                        )
            self.current_frontier = None
            prefetched = self._prefetched_frontier_plan
            self._prefetched_frontier_plan = None
            if prefetched is not None:
                scope, outcome = prefetched
                candidate_path = self._validated_async_frontier_path(
                    grid,
                    pose,
                    scope,
                    outcome,
                )
                if outcome.frontier is not None and candidate_path is not None:
                    self._install_async_frontier_path(
                        grid=grid,
                        pose=pose,
                        frontier=outcome.frontier,
                        path=candidate_path,
                        reason="frontier_prefetched_path",
                    )
        elif status == IDLE:
            self._frontier_arrival_gain.cancel()
            self.current_frontier = None

    def _frontier_gain_map_epoch(self) -> int:
        epoch_for_floor = getattr(self._ground_support, "map_epoch_for_floor", None)
        if callable(epoch_for_floor):
            return int(epoch_for_floor(self.current_floor))
        return int(getattr(self._ground_support, "map_epoch", 1))

    def _begin_frontier_arrival_gain(self, grid, frontier: Frontier) -> None:
        self._frontier_arrival_gain.begin(
            floor=self.current_floor,
            map_epoch=self._frontier_gain_map_epoch(),
            target=frontier.centroid,
            known_cells=int(np.count_nonzero(grid.data != UNKNOWN)),
            expected_gain=frontier.expected_unknown_gain,
            is_real_frontier=frontier.information_gain > 0.0,
        )

    def _frontier_selection_context(
        self,
        pose: tuple[float, float, float],
    ) -> FrontierSelectionContext | None:
        context = self._frontier_inertia_context()

        now = get_time()
        # 永久过滤只覆盖按入口门宽推导的近门区域，远处采用连续衰减。
        entrance_xy = getattr(self, "main_entrance_xy", None)
        entrance_vec = getattr(self, "main_entrance_pass_vector", None)
        if entrance_xy is not None and entrance_vec is not None:
            permanent_radius, permanent_decay, gate_half_width = self._permanent_door_context_parameters()
            if context is None:
                context = FrontierSelectionContext(
                    permanent_forbidden_half_plane_origin=entrance_xy,
                    permanent_forbidden_half_plane_normal=entrance_vec,
                    permanent_forbidden_radius=permanent_radius,
                    permanent_forbidden_penalty=self.permanent_forbidden_penalty,
                    permanent_forbidden_decay_distance=permanent_decay,
                    permanent_gate_half_width=gate_half_width,
                    door_id="main_entrance",
                    context_time=now,
                )
            else:
                context = replace(
                    context,
                    permanent_forbidden_half_plane_origin=entrance_xy,
                    permanent_forbidden_half_plane_normal=entrance_vec,
                    permanent_forbidden_radius=permanent_radius,
                    permanent_forbidden_penalty=self.permanent_forbidden_penalty,
                    permanent_forbidden_decay_distance=permanent_decay,
                    permanent_gate_half_width=gate_half_width,
                    door_id="main_entrance",
                    context_time=now,
                )

        if self._door_navigation.last_door_xy is None or self._door_navigation.last_pass_vector is None:
            return context
        if now > self._door_navigation.context_until:
            return context
        timer_active = now <= self._door_navigation.transition_until
        progress_active = not self._post_door_context_satisfied(pose)
        if not timer_active and not progress_active:
            return context
        if timer_active and not progress_active and self.door_transition_hold_distance > 0.0:
            distance_to_door = math.hypot(pose[0] - self._door_navigation.last_door_xy[0], pose[1] - self._door_navigation.last_door_xy[1])
            if distance_to_door > self.door_transition_hold_distance:
                return context

        if context is None:
            context = FrontierSelectionContext(
                forbidden_half_plane_origin=self._door_navigation.last_door_xy,
                forbidden_half_plane_normal=self._door_navigation.last_pass_vector,
                forbidden_radius=self.explorer.door_front_half_plane_radius,
                skip_in_forbidden_half_plane=True,
                preferred_origin=self._door_navigation.last_door_xy,
                preferred_direction=self._door_navigation.last_pass_vector,
                temporary_revisit_penalty=self.explorer.door_front_half_plane_penalty,
                door_id=self._door_navigation.last_door_id,
                temporary_context_expires_at=self._door_navigation.context_until,
                context_time=now,
            )
        else:
            context = replace(
                context,
                forbidden_half_plane_origin=self._door_navigation.last_door_xy,
                forbidden_half_plane_normal=self._door_navigation.last_pass_vector,
                forbidden_radius=self.explorer.door_front_half_plane_radius,
                skip_in_forbidden_half_plane=True,
                preferred_origin=self._door_navigation.last_door_xy,
                preferred_direction=self._door_navigation.last_pass_vector,
                temporary_revisit_penalty=self.explorer.door_front_half_plane_penalty,
                door_id=self._door_navigation.last_door_id,
                temporary_context_expires_at=self._door_navigation.context_until,
                context_time=now,
            )
        return context

    def _record_elevator_exit_context(
        self,
        task: TakeElevator,
        pose: tuple[float, float, float],
        *,
        now: float,
    ) -> bool:
        """用目标楼层已注册电梯门记录离厢后的临时前沿上下文。"""
        try:
            door_id = str(self.building_client.elevator_door_id(task.target_floor))
        except (AttributeError, TypeError, ValueError) as exc:
            MissionReportingRuntime.logwarn(
                "电梯任务完成但目标楼层门 ID 无效，无法建立离厢前沿上下文：%s",
                exc,
            )
            return False
        doors = getattr(self.building_client, "doors", {})
        door = doors.get(door_id) if isinstance(doors, dict) else None
        if (
            door is None
            or getattr(door, "kind", None) != "elevator"
            or int(getattr(door, "floor_index", -1)) != int(task.target_floor)
        ):
            MissionReportingRuntime.logwarn(
                "电梯任务完成但目标门缺少已注册传感器几何：door_id=%s floor=%d",
                door_id,
                int(task.target_floor),
            )
            return False
        yaw = float(door.yaw)
        return self._door_navigation.record_passage(
            door_id=door_id,
            door_xy=(float(door.x), float(door.y)),
            pass_vector=(math.cos(yaw), math.sin(yaw)),
            pose=pose,
            now=now,
            transition_duration=self.door_transition_duration,
            context_duration=self.post_door_context_max_duration,
        )

    def _permanent_door_context_parameters(self) -> tuple[float, float, float]:
        door_width = 0.0
        doors = getattr(getattr(self, "building_client", None), "doors", {})
        entrance = doors.get("main_entrance") if isinstance(doors, dict) else None
        if entrance is not None:
            door_width = max(0.0, float(getattr(entrance, "width", 0.0)))
        base_radius = max(0.0, float(self.explorer.door_front_half_plane_radius))
        map_clearance = max(
            0.0,
            float(getattr(getattr(self, "mapper", None), "obstacle_inflation_radius", 0.35)),
        )
        if door_width > 0.0:
            radius = max(base_radius, door_width * self.permanent_forbidden_radius_scale)
            decay = max(radius, door_width * self.permanent_forbidden_decay_scale)
            gate_half_width = door_width / 2.0 + map_clearance
        else:
            radius = base_radius
            decay = max(radius, radius * self.permanent_forbidden_decay_scale)
            gate_half_width = base_radius / 2.0 + map_clearance
        return radius, decay, gate_half_width

    def _frontier_inertia_context(self) -> FrontierSelectionContext | None:
        if self._exploration.last_frontier_target is None:
            return None
        return FrontierSelectionContext(
            current_target=self._exploration.last_frontier_target,
            current_score=self._exploration.last_frontier_score,
            current_region_radius=self.frontier_current_region_radius,
            switch_score_margin=self.frontier_switch_score_margin,
        )

    def _post_door_context_satisfied(self, pose: tuple[float, float, float]) -> bool:
        grid = (
            self.mapper.floor(self.current_floor)
            if self.post_door_min_known_area > 0.0
            else None
        )
        return self._door_navigation.context_satisfied(
            pose,
            grid=grid,
            minimum_distance=self.post_door_min_distance,
            minimum_known_area=self.post_door_min_known_area,
        )

    def _log_frontier_switch(
        self,
        frontier: Frontier,
        path: list[tuple[float, float]],
        pose: tuple[float, float, float],
    ) -> None:
        debug = self.explorer.last_selection_debug
        previous_target = self._exploration.last_frontier_target
        previous_score = self._exploration.last_frontier_score
        previous_distance = (
            math.hypot(frontier.centroid[0] - previous_target[0], frontier.centroid[1] - previous_target[1])
            if previous_target is not None
            else 0.0
        )
        door_side = self._door_navigation.side_label(frontier.centroid)
        used_fallback = bool(debug.used_fallback) if debug is not None else False
        retained_by_inertia = bool(debug.retained_by_inertia) if debug is not None else False
        reason = "首次选择"
        if previous_target is not None:
            if retained_by_inertia:
                reason = f"保留当前区域：{debug.inertia_reason if debug is not None else ''}"
            elif previous_distance <= self.frontier_current_region_radius:
                reason = "同一区域延续"
            else:
                margin = debug.switch_score_margin if debug is not None else self.frontier_switch_score_margin
                current_region_score = debug.current_region_score if debug is not None else None
                if current_region_score is None:
                    reason = "当前区域无可达候选，切换"
                else:
                    reason = f"新目标明显更优：current_region_score={current_region_score:.2f} margin={margin:.2f}"
        MissionReportingRuntime.loginfo(
            "frontier切换：reason=%s target=(%.2f, %.2f) score=%.2f path_length=%.2f previous=%s previous_score=%s switch_distance=%.2f door_side=%s fallback=%s retained_by_inertia=%s",
            reason,
            frontier.centroid[0],
            frontier.centroid[1],
            frontier.score,
            path_length(path) if path else 0.0,
            f"({previous_target[0]:.2f}, {previous_target[1]:.2f})" if previous_target is not None else "None",
            f"{previous_score:.2f}" if previous_score is not None else "None",
            previous_distance,
            door_side,
            used_fallback,
            retained_by_inertia,
        )
        self._exploration.last_frontier_target = frontier.centroid
        self._exploration.last_frontier_score = frontier.score
        self._exploration.last_frontier_used_fallback = used_fallback

    def clear_opened_door_area(self, door_id: str) -> None:
        door = self.building_client.doors.get(door_id)
        if door is None:
            return
        # clear_dynamic_door_strip 的 depth 是完整长度。清理范围至少覆盖门两侧
        # 等待点及占据膨胀，否则机器人从门内返航时会站在膨胀 OCC 中，A* 和
        # 直连检查都无法离开。门宽和法向均来自传感器注册结果，不扩展到门外
        # commit 区域，也不使用场景固定坐标。
        half_depth = max(
            self.door_clear_radius,
            self.door_pass_offset
            + self.mapper.obstacle_inflation_radius
            + self.mapper.resolution,
        )
        depth = 2.0 * half_depth
        self.mapper.clear_dynamic_door_strip(door.floor_index, door.x, door.y, door.yaw, door.width, depth)
        MissionReportingRuntime.loginfo(
            "已清理开门后的动态障碍残留：door_id=%s floor=%d center=(%.2f, %.2f) yaw=%.2f width=%.2f depth=%.2f",
            door_id,
            door.floor_index,
            door.x,
            door.y,
            door.yaw,
            door.width,
            depth,
        )

    def clear_elevator_crossing_area(self, geometry) -> None:
        """按当前 Livox 门洞清理本次穿越条带，不改写长期注册几何。"""
        half_depth = max(
            0.95 + self.mapper.obstacle_inflation_radius + self.mapper.resolution,
            self.door_clear_radius,
        )
        self.mapper.clear_dynamic_door_strip(
            self.current_floor,
            geometry.door_center[0],
            geometry.door_center[1],
            geometry.outward_yaw,
            geometry.width,
            2.0 * half_depth,
        )
        self._log_structured(
            {
                "event": "elevator_local_crossing_strip_cleared",
                "door_id": geometry.door_id,
                "mode": geometry.mode,
                "floor": int(self.current_floor),
                "center": [round(value, 3) for value in geometry.door_center],
                "yaw": round(geometry.outward_yaw, 3),
                "width": round(geometry.width, 3),
                "depth": round(2.0 * half_depth, 3),
            }
        )

    def _active_opened_door_obstacle_exclusion(
        self,
    ) -> tuple[float, float, float, float, float] | None:
        """返回当前主动过门阶段可忽略的已开门洞棱柱。"""
        task = getattr(self, "current_task", None)
        if isinstance(task, OpenDoorAndPass):
            if getattr(task, "phase", None) not in {"GO_PASS_POINT", "GO_COMMIT_POINT"}:
                return None
            # 服务成功、完成门洞对准并提交直线路径后才允许过滤。这样等待点导航
            # 和普通探索仍保留完整正障碍制动，门服务失败也不会被掩盖。
            if not bool(getattr(task, "_pass_path_started", False)):
                return None
            door_id = task.door_id
            elevator_crossing = False
        elif isinstance(task, TakeElevator):
            active_door_id = getattr(task, "active_opened_door_id", None)
            if not callable(active_door_id):
                return None
            building_client = getattr(self, "building_client", None)
            door_id = active_door_id(building_client)
            if door_id is None:
                return None
            elevator_crossing = True
        else:
            return None

        door = getattr(getattr(self, "building_client", None), "doors", {}).get(door_id)
        if door is None:
            return None
        edge_margin = max(0.05, float(self.mapper.resolution))
        if elevator_crossing:
            # RGB-D 电梯注册宽度表示单扇滑动门板，而不是完整门洞。仅在门已由
            # 传感器确认打开且任务提交直线跨越后，按一扇门板宽度估计门洞半宽。
            half_width = max(0.0, float(door.width) - edge_margin)
        else:
            half_width = max(0.0, 0.5 * float(door.width) - edge_margin)
        if half_width <= 0.0:
            return None
        if elevator_crossing:
            # 只忽略门平面附近的门板/门框回波；门后实体仍进入前向停车链。
            half_depth = max(
                2.0 * float(self.mapper.resolution),
                float(self.mapper.obstacle_inflation_radius)
                + float(self.mapper.resolution),
            )
        else:
            half_depth = max(
                float(self.door_clear_radius),
                float(self.door_pass_offset)
                + float(self.mapper.obstacle_inflation_radius)
                + float(self.mapper.resolution),
            )
        return (
            float(door.x),
            float(door.y),
            float(door.yaw),
            half_width,
            half_depth,
        )

    def _floor_completion_ready(self, grid, floor_elapsed: float) -> bool:
        return (
            floor_elapsed >= self.min_floor_explore_time
            and self._max_home_distance >= self.min_floor_travel_distance
            and grid.known_area() >= self.min_known_area
            and grid.coverage_ratio() >= self.explore_coverage_ratio
            and self._exploration.no_reachable_frontier_confirmations >= self.no_reachable_frontier_confirmations
            and self._known_corridor_progress(grid) >= self.min_corridor_progress_distance
        )

    def _known_corridor_progress(self, grid) -> float:
        if self.min_corridor_progress_distance <= 0.0:
            return 0.0
        extent_x, extent_y = grid.known_bbox_extent()
        return extent_y if abs(math.sin(self.probe_yaw)) >= abs(math.cos(self.probe_yaw)) else extent_x

    def _apply_initial_entry_clearance_failure(
        self,
        target: tuple[float, float] | None,
    ) -> None:
        self._record_failure("主入口离门阶段有命令但无有效位移")
        self._mission_success_eligible = False
        self._transition_state(
            WRITE_RESULT,
            reason_code="main_entrance_clearance_failed",
            call_site="MissionManager.tick:initial_entry_clearance",
            diagnostic_context={
                "clearance_target": list(target) if target is not None else None
            },
        )

    def _log_post_door_frontier_debug(
        self,
        frontier: Frontier | None,
        path: list[tuple[float, float]],
        pose: tuple[float, float, float],
    ) -> None:
        debug = self._door_navigation.frontier_debug
        if debug is None:
            return
        door_id = str(debug["door_id"])
        wait_point = debug["wait_point"]
        pass_point = debug["pass_point"]
        commit_point = debug["commit_point"]
        completion_pose = debug["completion_pose"]
        side = "未选择目标"
        target_text = "None"
        score = 0.0
        path_len = 0.0
        if frontier is not None:
            target_text = f"({frontier.centroid[0]:.2f}, {frontier.centroid[1]:.2f})"
            score = frontier.score
            path_len = path_length(path) if path else 0.0
            side = self._door_navigation.side_label(frontier.centroid)
            if isinstance(wait_point, tuple) and isinstance(pass_point, tuple):
                side = self._door_navigation.classify_goal_side(
                    wait_point, pass_point, frontier.centroid
                )
        MissionReportingRuntime.loginfo(
            "过门后首次前沿选择：door_id=%s frontier_target=%s score=%.2f path_length=%.2f relative_side=%s commit_point=(%.2f, %.2f) current_pose=(%.2f, %.2f, %.2f) completion_pose=(%.2f, %.2f, %.2f)",
            door_id,
            target_text,
            score,
            path_len,
            side,
            commit_point[0] if isinstance(commit_point, tuple) else float("nan"),
            commit_point[1] if isinstance(commit_point, tuple) else float("nan"),
            pose[0],
            pose[1],
            pose[2],
            completion_pose[0],
            completion_pose[1],
            completion_pose[2],
        )
        self._door_navigation.frontier_debug = None

    def _try_probe_unknown_space(self, pose: tuple[float, float, float]) -> bool:
        grid = self.mapper.floor(self.current_floor)
        # 探测序列以当前合法定位姿态为基准，避免固定世界方向在不同起点或
        # 过门后制造不必要的大角度原地转向。
        base_pose = pose
        for _ in range(12):
            target = probe_target_for_attempt(base_pose, self.probe_distance, self._exploration.probe_attempt)
            self._exploration.probe_attempt += 1
            target_cell = grid.world_to_grid(target[0], target[1])
            if target_cell is None:
                continue
            cell_value = int(grid.data[target_cell[1], target_cell[0]])
            if cell_value in (BLACK, NEGATIVE, OCC):
                continue
            if segment_crosses_blocked_cell(
                grid,
                (pose[0], pose[1]),
                target,
                constraints=self._planning_constraints(),
            ):
                continue
            self._frontier_arrival_gain.cancel()
            self.current_frontier = None
            self._exploration.last_frontier_target = None
            self._exploration.last_frontier_score = None
            self._exploration.last_frontier_used_fallback = False
            self.controller.set_path([(pose[0], pose[1]), target], reason="probe_unknown_space_path")
            MissionReportingRuntime.loginfo("无前沿，执行短距离探测目标：target=(%.2f, %.2f)", target[0], target[1])
            return True
        return False

    def _run_initial_entry(self, pose, grid) -> bool:
        """应用主入口运行时的一次冻结决策。"""
        features = getattr(self, "building_features", None)
        entrance = (
            getattr(features, "doors", {}).get("main_entrance")
            if features is not None
            else None
        )
        result = self._entry.tick(
            pose=pose,
            grid=grid,
            entrance=entrance,
            current_floor=self.current_floor,
            controller=self.controller,
            building_client=self.building_client,
            local_detector=self.local_detector,
            door_tracker=self._door_observation_runtime.initial,
            cloud_stamp=self._map_runtime.last_cloud_stamp,
            cloud_points=self._map_runtime.latest_cloud_points,
            floor_height=self.mapper.floor_height,
            config=self._entry_config,
            segment_is_traversable=self._segment_is_traversable,
        )
        if result.failure is not None:
            self._record_failure(result.failure)
        if result.clear_exploration_target:
            self._frontier_arrival_gain.cancel()
            self.current_frontier = None
            self._exploration.last_frontier_target = None
            self._exploration.last_frontier_score = None
            self._exploration.last_frontier_used_fallback = False
        if result.task is not None:
            self.active_door = result.door_id
            self.current_task = result.task
            if result.door_id is not None:
                self._freeze_landmark_anchor("main_entrance", result.door_id)
        return bool(result.handled)

    def refresh_open_door_geometry(
        self,
        door_id: str,
        pose: tuple[float, float, float],
    ) -> str:
        """应用门观测运行时的近场刷新结果。"""
        result = self._door_observation_runtime.refresh_geometry(
            door_id=door_id,
            entry_registered=self._entry.origin_pose is not None,
            pose=pose,
            now=get_time(),
            cloud_stamp=self._map_runtime.last_cloud_stamp,
            current_floor=self.current_floor,
            cloud_points=self._map_runtime.latest_cloud_points,
            floor_height=self.mapper.floor_height,
            local_detector=self.local_detector,
            building_client=self.building_client,
            current_task=self.current_task,
            config=self._door_refresh_config,
        )
        if result.failure is not None:
            self._record_failure(result.failure)
        if result.diagnostic is not None:
            self._log_structured(result.diagnostic)
        return result.status

    def refresh_elevator_geometry(
        self,
        elevator_id: str,
        floor_index: int,
        door_id: str,
        pose: tuple[float, float, float],
    ) -> str:
        result = self._elevator_observation.registration.refresh_geometry(
            now=get_time(),
            elevator_id=elevator_id,
            floor_index=floor_index,
            door_id=door_id,
            pose=pose,
            building_features=self.building_features,
            building_client=self.building_client,
            config=self._elevator_registration_config(),
            landmark_monitor=getattr(self, "_landmark_drift_monitor", None),
            elapsed=self._elapsed(),
        )
        return self._apply_elevator_refresh_result(result)

    def refresh_elevator_crossing_geometry(
        self,
        *,
        door_id: str,
        mode: str,
        expected_crossing_yaw: float,
        old_door_center: tuple[float, float],
        pose: tuple[float, float, float],
    ):
        """用当前 Livox 刷新本次穿越几何，不写回长期电梯注册。"""
        result = self._elevator_crossing.observe(
            now=get_time(),
            door_id=door_id,
            mode=mode,
            expected_crossing_yaw=expected_crossing_yaw,
            old_door_center=old_door_center,
            pose=pose,
            cloud_stamp=self._map_runtime.last_cloud_stamp,
            cloud_points=self._map_runtime.latest_cloud_points,
            floor_index=self.current_floor,
            floor_height=float(self.mapper.floor_height),
            local_detector=self.local_detector,
            config=self._elevator_crossing_config,
        )
        if result.diagnostic is not None:
            self._log_structured(result.diagnostic)
        return result

    def elevator_crossing_obstacle(
        self,
        start_xy: tuple[float, float],
        end_xy: tuple[float, float],
        geometry,
    ):
        observation = crossing_corridor_obstacle(
            self._map_runtime.latest_cloud_points,
            start_xy=start_xy,
            end_xy=end_xy,
            floor_index=self.current_floor,
            floor_height=float(self.mapper.floor_height),
            obstacle_min_height=float(self.mapper.obstacle_min_z),
            obstacle_max_height=float(self.mapper.obstacle_max_z),
            sweep_half_width=float(self._elevator_crossing_config.sweep_half_width),
            door_center=geometry.door_center,
            door_yaw=geometry.outward_yaw,
            door_width=geometry.width,
        )
        if observation is not None:
            self._log_structured(
                {
                    "event": "elevator_crossing_corridor_blocked",
                    "door_id": geometry.door_id,
                    "mode": geometry.mode,
                    "distance": round(observation.distance, 3),
                    "forward": round(observation.forward, 3),
                    "lateral": round(observation.lateral, 3),
                    "world_xyz": [round(value, 3) for value in observation.world_xyz],
                }
            )
        return observation

    def elevator_approach_trace_candidates(
        self,
        elevator_id: str,
        floor_index: int,
        *,
        max_candidates: int = 4,
    ) -> list[tuple[float, float]]:
        elevator = self.building_client.elevators.get(str(elevator_id))
        return select_elevator_approach_trace_candidates(
            elevator=elevator,
            floor_index=floor_index,
            navigation_trace=self._floor_navigation_traces.get(
                int(floor_index), ()
            ),
            grid=self.mapper.floor(int(floor_index)),
            visual_max_distance=self.elevator_visual_max_distance,
            max_candidates=max_candidates,
        )

    def elevator_start_retreat_target(
        self,
        pose: tuple[float, float, float],
        floor_index: int,
    ) -> tuple[float, float] | None:
        grid = self.mapper.floor(int(floor_index))
        return select_elevator_start_retreat_target(
            pose=pose,
            grid=grid,
            navigation_trace=self._floor_navigation_traces.get(
                int(floor_index), ()
            ),
            rotation_min_clearance=self.no_frontier_rotation_min_clearance,
            retreat_clearance_margin=self.no_frontier_retreat_clearance_margin,
        )

    def begin_elevator_door_open_confirmation(self, door_id: str) -> None:
        event = self._elevator_observation.begin_door_open_confirmation(
            door_id=door_id,
            building_client=self.building_client,
            geometry_tolerance=self.initial_entry_geometry_tolerance,
        )
        if event is not None:
            self._log_structured(event)

    def confirm_elevator_door_open(
        self,
        door_id: str,
        pose: tuple[float, float, float],
    ) -> bool:
        guard = self._floor_transition_guard
        mapper = self.mapper
        explicit_transition = bool(
            guard is not None and guard.explicit_transition
        )
        observation_floor = int(self.current_floor)
        if explicit_transition and guard.target_floor is not None:
            observation_floor = int(guard.target_floor)
        result = self._elevator_observation.confirm_door_open(
            door_id=door_id,
            pose=pose,
            cloud_stamp=self._map_runtime.last_cloud_stamp,
            cloud_points=self._map_runtime.latest_cloud_points,
            floor_index=observation_floor,
            floor_height=(
                float(mapper.floor_height) if mapper is not None else None
            ),
            floor_transition_explicit=explicit_transition,
            geometry_tolerance=self.initial_entry_geometry_tolerance,
            building_client=self.building_client,
            local_detector=self.local_detector,
        )
        if result.structured_event is not None:
            self._log_structured(result.structured_event)
        return result.confirmed

    def finish_elevator_door_open_confirmation(self, door_id: str) -> None:
        event = self._elevator_observation.finish_door_open_confirmation(door_id)
        if event is not None:
            self._log_structured(event)

    def _no_frontier_recovery_config(self) -> NoFrontierRecoveryConfig:
        return NoFrontierRecoveryConfig(
            scan_duration=float(self.no_frontier_scan_duration),
            rotation_min_clearance=float(self.no_frontier_rotation_min_clearance),
            retreat_clearance_margin=float(self.no_frontier_retreat_clearance_margin),
            retreat_max_distance=float(self.no_frontier_retreat_max_distance),
        )

    def _apply_exploration_runtime_result(
        self,
        result: ExplorationRuntimeResult,
    ) -> None:
        """只应用探索子状态机的任务侧日志与失败记账。"""
        if result.failure_reason is not None:
            self._record_failure(result.failure_reason)
        if result.info_message is not None:
            MissionReportingRuntime.loginfo(*result.info_message)
        if result.warning_message is not None:
            MissionReportingRuntime.logwarn(*result.warning_message)
        for event in result.structured_events:
            self._log_structured(event)

    def _rotation_clearance_is_safe(self) -> bool:
        """仅允许在点云确认机身旋转包络有足够净距时原地转向。"""
        clearance = self._map_runtime.nearest_radial_obstacle_distance
        threshold = float(
            getattr(self, "no_frontier_rotation_min_clearance", 0.40)
        )
        return self._exploration.rotation_clearance_is_safe(clearance, threshold)

    def _recover_with_rotation_clearance(self, source: str) -> bool:
        """仅在机身旋转包络安全时启动局部恢复。"""
        if self._rotation_clearance_is_safe():
            self.controller.recover()
            return True
        self.controller.stop(reason="unsafe_recover_rotation_blocked")
        self._log_rotation_clearance_blocked(source)
        return False

    def _log_rotation_clearance_blocked(self, source: str) -> None:
        if self._exploration.no_frontier_rotation_blocked_logged:
            return
        self._exploration.no_frontier_rotation_blocked_logged = True
        clearance = self._map_runtime.nearest_radial_obstacle_distance
        self._log_structured(
            {
                "event": "unsafe_rotation_blocked",
                "source": str(source),
                "current_floor": int(self.current_floor),
                "clearance": (
                    round(float(clearance), 4)
                    if clearance is not None and math.isfinite(float(clearance))
                    else None
                ),
                "required_clearance": round(
                    float(getattr(self, "no_frontier_rotation_min_clearance", 0.40)),
                    4,
                ),
            }
        )

    def _finish_current_floor(
        self,
        *,
        reason_code: str = "floor_finished",
        call_site: str = "MissionManager._finish_current_floor",
    ) -> None:
        self._frontier_arrival_gain.cancel()
        self._record_floor_exit_certificate(reason_code)
        no_frontier_count = self._exploration.no_reachable_frontier_confirmations
        self.controller.clear_path()
        self._exploration.no_frontier_scan_until = 0.0
        self._exploration.no_reachable_frontier_confirmations = 0
        self._exploration.last_no_frontier_signature = None
        self._exploration.reset_no_frontier_escape()

        can_change_floor = (
            self.current_floor + 1 < self.max_floor
            and self._has_usable_floor_change(self.current_floor + 1)
        )
        if can_change_floor:
            # 需要剩余时间足够完成换层、新楼层最低探索和返航
            pose = self._last_pose or self.home_pose
            mission_elapsed = self._elapsed()
            remaining = self.max_runtime - mission_elapsed
            change_time = self._estimate_floor_change_time(self.current_floor + 1, pose)
            return_time = self._estimate_return_time(pose)
            # 换层后至少需要在新楼层探索一段时间，再返航
            min_new_floor_time = min(self.min_floor_explore_time, 20.0)
            if change_time + min_new_floor_time + return_time + self.return_safety_buffer < remaining:
                self._begin_floor_change(
                    self.current_floor + 1,
                    return_after=False,
                    reason_code=f"{reason_code}_change_floor",
                    call_site=call_site,
                    diagnostic_context={
                        "no_reachable_frontier_confirmations": no_frontier_count,
                        "estimated_change_time": round(change_time, 2),
                        "estimated_return_time": round(return_time, 2),
                        "remaining_time": round(remaining, 2),
                    },
                )
                return
            else:
                MissionReportingRuntime.loginfo(
                    "剩余时间不足以支持换层后继续探索和返航，改为返航："
                    "remaining=%.1f change=%.1f min_floor=%.1f return=%.1f buffer=%.1f",
                    remaining, change_time, min_new_floor_time, return_time, self.return_safety_buffer
                )
        self._transition_state(
            RETURN_HOME,
            reason_code=f"{reason_code}_return_home",
            call_site=call_site,
            diagnostic_context={
                "no_reachable_frontier_confirmations": no_frontier_count,
                "can_change_floor": can_change_floor,
            },
        )

    def _record_floor_exit_certificate(self, reason_code: str) -> FloorExitCertificate:
        """所有正常 EXPLORE_FLOOR 出口共用的冻结证书入口。"""
        self._explored_floor_ids.add(int(self.current_floor))
        constraints = self._planning_constraints()
        pose = self._last_pose or self.home_pose
        freeze = getattr(self.explorer, "floor_termination_evidence", None)
        evidence = (
            freeze(
                self.mapper.floor(self.current_floor),
                pose,
                constraints,
            )
            if callable(freeze) and pose is not None else None
        )
        blocked = self._lock_support_constraint_blocked_completion(evidence)
        certificate = FloorExitCertificate(
            floor=int(self.current_floor),
            map_epoch=int(self._ground_support.map_epoch_for_floor(self.current_floor)),
            localization_epoch=int(self._ground_support.localization_epoch),
            constraint_revision=int(
                getattr(evidence, "constraint_revision", 0)
            ),
            reason_code=str(reason_code),
            formal_reachable_count=int(
                getattr(evidence, "formal_reachable_count", 0)
            ),
            effective_reachable_count=int(
                getattr(evidence, "effective_reachable_count", 0)
            ),
            region_ids=tuple(getattr(evidence, "region_ids", ())),
            valid=evidence is not None and not blocked,
        )
        previous = self._floor_exit_certificates.get(certificate.floor)
        if previous is not None and not previous.valid:
            return previous
        self._floor_exit_certificates[certificate.floor] = certificate
        self._log_structured(
            {
                "event": "floor_exit_certificate",
                **certificate.__dict__,
                "region_ids": list(certificate.region_ids),
            }
        )
        return certificate

    def _lock_support_constraint_blocked_completion(self, evidence=None) -> bool:
        """仅在楼层终止边界锁定仍未解除的临时约束隔离。"""
        if evidence is None:
            constraints = self._planning_constraints()
            freeze = getattr(self.explorer, "floor_termination_evidence", None)
            pose = self._last_pose or self.home_pose
            if not callable(freeze) or pose is None:
                return False
            evidence = freeze(
                self.mapper.floor(self.current_floor), pose, constraints
            )
        if not bool(getattr(evidence, "fully_blocked", False)):
            return False
        key = (int(self.current_floor), int(evidence.constraint_revision))
        if any(
            (item.get("floor"), item.get("constraint_revision")) == key
            for item in self._invalid_floor_completion_evidence
        ):
            return True
        payload = {
            "floor": key[0],
            "reason_code": "support_constraint_blocked",
            "constraint_revision": key[1],
            "region_ids": list(evidence.region_ids),
            "formal_reachable_count": evidence.formal_reachable_count,
            "support_blocked_count": evidence.support_blocked_count,
        }
        self._invalid_floor_completion_evidence.append(payload)
        self._mission_success_eligible = False
        self._record_failure("support_constraint_blocked")
        self._log_structured(
            {"event": "floor_completion_invalid", **payload}
        )
        return True

    def _tick_current_task_status(
        self,
        pose: tuple[float, float, float],
    ) -> str:
        """消费一次强制任务失败，否则正常 tick 当前任务。"""
        forced_failure = self._forced_task_failure_reason
        if forced_failure is None:
            return self.current_task.tick(self.navigation_task_context(), pose)
        self._forced_task_failure_reason = None
        try:
            self.current_task.failure_reason = forced_failure
        except (AttributeError, TypeError):
            pass
        return FAILED

    def _log_floor_budget_exhausted(self, floor_elapsed: float) -> None:
        """记录楼层配额到期后仍继续探索的原因。"""
        if self._exploration.floor_timeout_logged:
            return
        self._exploration.floor_timeout_logged = True
        diagnostic = self._mission_diagnostic_snapshot()
        waiting_for_minimum = floor_elapsed < float(
            getattr(self, "min_floor_explore_time", 0.0)
        )
        if waiting_for_minimum:
            reason_code = "floor_timeout_before_min_explore_time"
            action = "continue_until_min_floor_time"
        else:
            reason_code = "last_floor_continues_under_global_budget"
            action = "continue_until_global_return_boundary"
        diagnostic.update(
            {
                "event": "mission_floor_budget_exhausted",
                "reason_code": reason_code,
                "floor_elapsed": round(float(floor_elapsed), 3),
                "floor_timeout": self.floor_timeout,
                "min_floor_explore_time": getattr(self, "min_floor_explore_time", 0.0),
                "last_floor_uses_global_budget": bool(
                    getattr(self, "last_floor_uses_global_budget", True)
                ),
                "action": action,
            }
        )
        self._log_structured(diagnostic)

    def _tick_return_home(self, pose: tuple[float, float, float]) -> None:
        if self.home_pose is None:
            if rospy is not None:
                rospy.logerr("[MissionManager] ⚠️ RETURN_HOME失败: home_pose未初始化")
            self._record_failure("缺少返航起点")
            self._transition_state(
                WRITE_RESULT,
                reason_code="missing_home_pose",
                call_site="MissionManager._tick_return_home",
            )
            return

        # 返航开始时记录home_pose（仅首次）
        if not getattr(self, "_return_home_started_logged", False):
            if rospy is not None:
                rospy.loginfo(
                    "[MissionManager] 开始返航: 当前位置=[%.2f, %.2f], 目标home_pose=[%.2f, %.2f, %.2f], 距离=%.2fm",
                    pose[0], pose[1],
                    self.home_pose[0], self.home_pose[1], self.home_pose[2],
                    math.hypot(self.home_pose[0] - pose[0], self.home_pose[1] - pose[1])
                )
            self._return_home_started_logged = True
        if self.current_floor != 0 and self.max_floor > 1:
            self._begin_floor_change(
                0,
                return_after=True,
                reason_code="return_home_change_floor",
                call_site="MissionManager._tick_return_home",
            )
            return
        if self._return_home.retreat_target is not None:
            self._tick_return_home_retreat(pose)
            return
        self._return_home.clear_stuck_if_departed(pose)
        distance = math.hypot(self.home_pose[0] - pose[0], self.home_pose[1] - pose[1])
        if distance <= self.home_tolerance:
            self._return_home_success = True
            self._return_home_completed = True
            success_eligible_for_reason = bool(
                getattr(self, "_mission_success_eligible", True)
            )
            self._transition_state(
                WRITE_RESULT,
                reason_code=(
                    "return_home_completed"
                    if success_eligible_for_reason
                    else "return_home_completed_after_failure"
                ),
                call_site="MissionManager._tick_return_home",
            )
            return
        entrance_xy = getattr(self, "main_entrance_xy", None)
        entrance_vector = getattr(self, "main_entrance_pass_vector", None)
        entrance = getattr(
            getattr(self, "building_client", None), "doors", {}
        ).get("main_entrance")
        navigation_goal = (self.home_pose[0], self.home_pose[1])
        if (
            not getattr(self, "_return_home_main_entrance_crossed", False)
            and entrance_xy is not None
            and entrance_vector is not None
            and entrance is not None
        ):
            signed_progress = (
                (pose[0] - entrance_xy[0]) * entrance_vector[0]
                + (pose[1] - entrance_xy[1]) * entrance_vector[1]
            )
            entrance_distance = math.hypot(
                pose[0] - entrance_xy[0],
                pose[1] - entrance_xy[1],
            )
            wait_point = None
            try:
                wait_point, _pass_point, _commit_point = (
                    self.building_client.get_door_path_points(
                        "main_entrance",
                        (pose[0], pose[1]),
                        pass_offset=self.door_pass_offset,
                        commit_distance=self.door_commit_distance,
                    )
                )
            except (KeyError, ValueError):
                pass
            activation_distance = max(
                1.0,
                float(getattr(self, "door_transition_hold_distance", 2.5)),
            )
            wait_distance = (
                math.hypot(pose[0] - wait_point[0], pose[1] - wait_point[1])
                if wait_point is not None
                else math.inf
            )
            task_start_distance = max(
                float(getattr(self.controller, "goal_tolerance", 0.35)),
                float(self.door_pass_offset),
            )
            if (
                signed_progress > 0.10
                and entrance_distance <= activation_distance
                and wait_distance <= task_start_distance
            ):
                if "main_entrance" in self.failed_doors:
                    self._mission_success_eligible = False
                    self._record_failure("主入口返航通过失败")
                    self._transition_state(
                        WRITE_RESULT,
                        reason_code="return_home_main_entrance_failed",
                        call_site="MissionManager._tick_return_home",
                    )
                    return
                self.controller.clear_path(reason="return_home_main_entrance")
                self.active_door = "main_entrance"
                self.current_task = OpenDoorAndPass(
                    "main_entrance",
                    pass_offset=self.door_pass_offset,
                    commit_distance=self.door_commit_distance,
                )
                MissionReportingRuntime.loginfo("返航先经已注册主入口离开建筑，再继续回到起点")
                return
            if signed_progress > 0.10 and wait_point is not None:
                navigation_goal = wait_point
        if (
            getattr(self, "_return_home_main_entrance_crossed", False)
            and not self._return_home.entry_trace_attempted
            and not self.controller.has_path()
        ):
            reverse_trace = initial_entry_reverse_path(
                pose,
                self.home_pose,
                tuple(self._entry.trace),
            )
            constraints = self._planning_constraints()
            if reverse_trace is not None and constraints is not None:
                grid = self.mapper.floor(self.current_floor)
                reverse_trace = simplify_path_line_of_sight(
                    grid,
                    reverse_trace,
                    constraints=constraints,
                )
                if any(
                    not segment_is_traversable(
                        grid,
                        start,
                        end,
                        constraints=constraints,
                    )
                    for start, end in zip(reverse_trace, reverse_trace[1:])
                ):
                    reverse_trace = None
            if reverse_trace is not None:
                self._return_home.entry_trace_attempted = True
                if (
                    not getattr(self._entry, "trace_scope_valid", True)
                    or getattr(self._entry, "trace_floor", None) is None
                    or getattr(self._entry, "trace_map_epoch", None) is None
                    or getattr(self._entry, "trace_localization_epoch", None) is None
                ):
                    reverse_trace = None
            if reverse_trace is not None:
                with _controller_command_transaction(self.controller):
                    self.controller.set_path(
                        reverse_trace,
                        reason="return_home_entry_trace_path",
                    )
                    scope = self._return_home.activate_entry_trace(
                        tuple(reverse_trace),
                        floor=int(self.current_floor),
                        map_epoch=int(self._entry.trace_map_epoch),
                        localization_epoch=int(self._entry.trace_localization_epoch),
                        maximum_deviation=0.35,
                    )
                    self._sync_ground_support_task_context(pose)
                self._log_structured(
                    {
                        "event": "return_home_entry_trace_trust_activated",
                        "floor": scope.floor,
                        "map_epoch": scope.map_epoch,
                        "localization_epoch": scope.localization_epoch,
                        "maximum_deviation": scope.maximum_deviation,
                        "waypoint_count": len(scope.path),
                    }
                )
                MissionReportingRuntime.loginfo(
                    "主入口外复用本轮已实际通过的入场轨迹返航：waypoints=%d",
                    len(reverse_trace),
                )
                return
        if not self.controller.has_path():
            now = get_time()
            if now < self._return_home.next_plan_time:
                self.controller.stop(reason="return_home_plan_wait")
                return
            grid = self.mapper.floor(self.current_floor)
            if self._return_home.planner is None:
                self._return_home.start_plan(
                    floor=self.current_floor,
                    goal_xy=navigation_goal,
                    grid=grid,
                    pose=pose,
                    navigation_trace=self._floor_navigation_traces.get(
                        int(self.current_floor), ()
                    ),
                    visited_radius=self.mapper.visited_radius,
                    veto_lookahead=self.return_home_veto_lookahead,
                    planning_constraints=self._planning_constraints(),
                )
                self.controller.stop(reason="return_home_plan_pending")
                return
            plan_poll = self._return_home.poll_plan(
                current_floor=self.current_floor,
                goal_xy=navigation_goal,
                current_constraint_revision=int(
                    getattr(self._planning_constraints(), "revision", 0)
                ),
            )
            if plan_poll.status == "pending":
                self.controller.stop(reason="return_home_plan_pending")
                return
            if plan_poll.status == "stale":
                self.controller.stop(reason="return_home_plan_stale")
                return
            plan_status = plan_poll.status
            planned_path = list(plan_poll.path)
            if plan_status == "reached":
                self._return_home.candidate_reject_count = 0
                self.controller.set_path(planned_path, reason="return_home_planned_path")
            elif str(plan_status).startswith("candidate_rejected"):
                self._return_home.candidate_reject_count += 1
                reject_count = self._return_home.candidate_reject_count
                self._log_structured(
                    {
                        **self._mission_diagnostic_snapshot(),
                        "event": "return_home_candidate_rejected",
                        "reason_code": str(plan_status),
                        "candidate_reject_count": reject_count,
                    }
                )
                if reject_count >= int(
                    getattr(self, "return_home_max_candidate_rejects", 3)
                ):
                    self._fail_return_home_stuck(
                        reason_code="return_home_stuck_exhausted"
                    )
                    return
                self.controller.clear_path(reason=str(plan_status))
                self._return_home.next_plan_time = now + 1.0
                return
            elif plan_status == "error":
                self.controller.stop(reason="return_home_plan_error")
                self._record_failure("返航规划线程异常")
                self._mission_success_eligible = False
                self._transition_state(
                    WRITE_RESULT,
                    reason_code="return_home_plan_error",
                    call_site="MissionManager._tick_return_home",
                    diagnostic_context={
                        "planner_exception": plan_poll.planner_error
                    },
                )
                return
            else:
                clear_path = getattr(self.controller, "clear_path", None)
                if callable(clear_path):
                    clear_path(reason=f"return_home_{plan_status}")
                self._return_home.next_plan_time = now + 1.0
                return
        current_segment = None
        segment_snapshot = getattr(self.controller, "current_segment", None)
        if callable(segment_snapshot):
            current_segment = segment_snapshot()
        status = self.controller.poll_status()
        if status == STUCK:
            self._handle_return_home_stuck(pose, current_segment)

    def _handle_return_home_stuck(
        self,
        pose: tuple[float, float, float],
        segment: tuple[tuple[float, float], tuple[float, float]] | None,
    ) -> None:
        feedback = self._return_home.record_stuck(
            pose,
            segment,
            self._map_runtime.nearest_forward_obstacle,
        )
        self._log_structured(
            {
                **self._mission_diagnostic_snapshot(),
                "event": "return_home_stuck_feedback",
                "reason_code": "local_segment_unexecutable",
                "stuck_pose": [round(float(value), 4) for value in pose],
                "failed_segment": (
                    list(self._return_home.failed_segments[-1])
                    if self._return_home.failed_segments
                    else None
                ),
                "nearest_forward_obstacle": feedback.obstacle_payload,
                "stuck_count": feedback.stuck_count,
            }
        )
        if feedback.stuck_count >= int(
            getattr(self, "return_home_max_stuck_retries", 4)
        ):
            self._fail_return_home_stuck(reason_code="return_home_stuck_exhausted")
            return
        if self._rotation_clearance_is_safe():
            self.controller.recover(reason="return_home_stuck")
            return
        if self._try_start_return_home_retreat(pose):
            return
        self.controller.clear_path(reason="return_home_retreat_unavailable")
        self.controller.stop(reason="return_home_retreat_unavailable")

    def _try_start_return_home_retreat(
        self,
        pose: tuple[float, float, float],
    ) -> bool:
        grid = self.mapper.floor(self.current_floor)
        selection = self._return_home.select_retreat(
            pose=pose,
            grid=grid,
            navigation_trace=self._floor_navigation_traces.get(
                int(self.current_floor), ()
            ),
            rotation_min_clearance=self.no_frontier_rotation_min_clearance,
            clearance_margin=self.no_frontier_retreat_clearance_margin,
            maximum_distance=self.no_frontier_retreat_max_distance,
            planning_constraints=self._planning_constraints(),
        )
        if selection is None:
            self._log_structured(
                {
                    "event": "return_home_retreat_unavailable",
                    "current_floor": int(self.current_floor),
                    "stuck_pose": [round(float(value), 3) for value in pose],
                }
            )
            return False

        self.controller.clear_path(reason="return_home_retreat_started")
        self._return_home.begin_retreat(
            selection,
            now=get_time(),
            reverse_speed=self.controller.reverse_speed,
        )
        self._log_structured(
            {
                "event": "return_home_retreat_started",
                "current_floor": int(self.current_floor),
                "target": [round(value, 3) for value in selection.target],
                "path_length": round(selection.path_length, 3),
                "required_clearance": round(selection.required_clearance, 3),
            }
        )
        return True

    def _tick_return_home_retreat(
        self,
        pose: tuple[float, float, float],
    ) -> None:
        step = self._return_home.tick_retreat(pose, now=get_time())
        if step.action == "inactive":
            return
        if step.action == "completed":
            self.controller.stop(reason="return_home_retreat_completed")
            self._log_structured(
                {
                    "event": "return_home_retreat_completed",
                    "current_floor": int(self.current_floor),
                    "target": list(step.target),
                }
            )
            return
        if step.action == "timeout":
            self.controller.stop(reason="return_home_retreat_timeout")
            self._fail_return_home_stuck(reason_code="return_home_retreat_failed")
            return
        if step.action == "heading_invalid" or not self.controller.reverse_straight(
            reason="return_home_retreat"
        ):
            self._return_home.abort_retreat()
            self.controller.stop(reason="return_home_retreat_failed")
            self._fail_return_home_stuck(reason_code="return_home_retreat_failed")

    def _fail_return_home_stuck(self, *, reason_code: str) -> None:
        """返航局部反馈耗尽时保持零速并写出明确失败结果。"""
        self.controller.clear_path(reason=reason_code)
        self.controller.stop(reason=reason_code)
        self._record_failure("返航路径在局部障碍处重复不可执行")
        self._mission_success_eligible = False
        self._transition_state(
            WRITE_RESULT,
            reason_code=reason_code,
            call_site="MissionManager._fail_return_home_stuck",
            diagnostic_context={
                "return_home_stuck_count": self._return_home.stuck_count,
                "return_home_candidate_reject_count": self._return_home.candidate_reject_count,
                "return_home_failed_segments": [
                    list(segment)
                    for segment in self._return_home.failed_segments
                ],
                "return_home_last_stuck_obstacle": self._return_home.last_stuck_obstacle,
            },
        )

    def _record_initial_entry_trace_pose(
        self,
        pose: tuple[float, float, float],
    ) -> None:
        """稀疏记录主入口外已实际通过的合法定位轨迹，供返航反向复用。"""
        if (
            self._entry.origin_pose is None
            or self._entry.completed
            or self.current_task is not None
            or self.state != EXPLORE_FLOOR
        ):
            return
        floor = int(self.current_floor)
        map_epoch = int(self._ground_support.map_epoch_for_floor(floor))
        localization_epoch = int(self._ground_support.localization_epoch)
        if self._entry.trace_floor is None:
            self._entry.trace_floor = floor
            self._entry.trace_map_epoch = map_epoch
            self._entry.trace_localization_epoch = localization_epoch
        elif (
            floor != self._entry.trace_floor
            or map_epoch != self._entry.trace_map_epoch
            or localization_epoch != self._entry.trace_localization_epoch
        ):
            self._entry.trace_scope_valid = False
            return
        point = (float(pose[0]), float(pose[1]))
        trace = self._entry.trace
        if not trace or math.hypot(point[0] - trace[-1][0], point[1] - trace[-1][1]) >= 0.25:
            trace.append(point)

    def _record_floor_navigation_trace_pose(
        self,
        pose: tuple[float, float, float],
    ) -> None:
        """稀疏记录每层真实行驶轨迹，供占据图被后续噪声割裂时安全返航。"""
        # 返航途中仍可能被新障碍截断。继续记录机器人已经实际走过的返航段，
        # 才能在实时地图不连通时先沿安全来路退回，再接入本层历史轨迹。
        if self.state in (WRITE_RESULT, FINISH, MISSION_FAILED):
            return
        guard = getattr(self, "_floor_transition_guard", None)
        if guard is not None:
            return
        floor_index = int(self.current_floor)
        traces = getattr(self, "_floor_navigation_traces", None)
        if traces is None:
            traces = {}
            self._floor_navigation_traces = traces
        point = (float(pose[0]), float(pose[1]))
        trace = traces.setdefault(floor_index, [])
        if not trace or math.hypot(point[0] - trace[-1][0], point[1] - trace[-1][1]) >= 0.25:
            trace.append(point)

    def _begin_floor_change(
        self,
        target_floor: int,
        *,
        return_after: bool,
        reason_code: str = "floor_change_requested",
        call_site: str = "MissionManager._begin_floor_change",
        diagnostic_context: dict[str, object] | None = None,
    ) -> None:
        if not self.building_client.services_ready():
            MissionReportingRuntime.logwarn("门/电梯服务不可用，停止探索并写出已有结果")
            self._elevator_success = False
            self._record_failure("门/电梯服务不可用")
            self._transition_state(
                WRITE_RESULT,
                reason_code="floor_services_unavailable",
                call_site=call_site,
                diagnostic_context=diagnostic_context,
            )
            return
        # 三态摘要中 None 只表示从未尝试；任务一旦发起，即使随后被全局
        # 预算或外部故障中断，也不能继续报告为“未尝试”。
        self._elevator_success = False
        self.target_floor = int(target_floor)
        self.return_after_floor_change = bool(return_after)
        self.controller.clear_path()
        self._transition_state(
            CHANGE_FLOOR,
            reason_code=reason_code,
            call_site=call_site,
            diagnostic_context=diagnostic_context,
        )
        self.current_task = TakeElevator(
            self.current_floor,
            target_floor,
            elevator_id=self._active_elevator_id,
        )
        MissionReportingRuntime.loginfo("准备换层：%d -> %d", self.current_floor, self.target_floor)

    def _write_result(
        self,
        *,
        reason_code: str | None = None,
        call_site: str = "MissionManager._write_result",
    ) -> None:
        requested_reason = reason_code or "result_write_requested"
        self._state_at_result_write = getattr(self, "state", None)
        self._result_write_call_site = call_site
        explored_floors = set(getattr(self, "_explored_floor_ids", ()))
        certificates = getattr(self, "_floor_exit_certificates", {})
        certificates_missing = bool(
            not explored_floors or not explored_floors <= set(certificates)
        )
        certificates_invalid = bool(
            not certificates_missing
            and any(not certificates[floor].valid for floor in explored_floors)
        )
        certificates_complete = bool(
            not certificates_missing and not certificates_invalid
        )
        resolved_reason = requested_reason
        if requested_reason == "return_home_completed" and not certificates_complete:
            higher_priority_failure = next(
                (
                    failure
                    for failure in getattr(self, "_failure_reasons", ())
                    if failure and failure != "support_constraint_blocked"
                ),
                None,
            )
            if higher_priority_failure is not None:
                resolved_reason = higher_priority_failure
            elif certificates_missing:
                resolved_reason = "floor_exit_certificate_missing"
            else:
                resolved_reason = "floor_exit_certificate_invalid"
        self._result_write_reason = resolved_reason
        self._normal_completion = bool(
            resolved_reason == "return_home_completed"
            and getattr(self, "_mission_success_eligible", True)
            and getattr(self, "_return_home_started", False)
            and getattr(self, "_return_home_completed", False)
            and getattr(self, "_return_home_success", False)
            and certificates_complete
        )
        # 时间预算不足导致的返航闭环成功，不能标记为“探索完成”。
        self._exploration_completed = bool(
            self._normal_completion
            and not getattr(self, "_timeout_triggered", False)
            and _return_reason_indicates_exploration_complete(
                self._return_home.reason
            )
        )
        self._terminal_outcome = "success" if self._normal_completion else "failed"
        self._terminal_reason = resolved_reason
        self._terminal_state = FINISH if self._normal_completion else MISSION_FAILED
        diagnostic = self._mission_diagnostic_snapshot()
        diagnostic.update(
            {
                "event": "mission_result_write",
                "state_at_result_write": self._state_at_result_write,
                "result_write_reason": resolved_reason,
                "call_site": call_site,
            }
        )
        self._log_structured(diagnostic)
        positions = self.tracker.confirmed_positions(now=get_time())
        summary = self._mission_summary_snapshot(len(positions))
        path = self._reporting.write_result(
            elapsed=self._elapsed(),
            positions=positions,
            summary=summary,
        )
        self.controller.stop(repeat=5)
        self._finished = True
        MissionReportingRuntime.loginfo("已写出危险源检测结果：%s，数量=%d", path, len(positions))

    def _publish_status(self) -> None:
        if not self._reporting.status_enabled:
            return
        now = get_time()
        if now - self._last_status_publish < self.status_publish_period:
            return
        self._last_status_publish = now

        grid = self.mapper.floor(self.current_floor)
        confirmed_count = len(self.tracker.confirmed_positions(now=now))
        active_goal = self.manual_goal
        if active_goal is None and self.current_frontier is not None:
            active_goal = self.current_frontier.centroid
        if active_goal is None:
            followed_path = self.controller.followed_path_snapshot()
            active_goal = followed_path[-1] if followed_path else None
        controller_debug = self.controller.tracking_debug_snapshot()
        payload = {
            "state": self.state,
            "floor": self.current_floor,
            "elapsed": round(self._elapsed(), 2),
            "confirmed_count": confirmed_count,
            "track_count": len(self.tracker.tracks),
            "current_task": self.current_task.name if self.current_task else None,
            "task_phase": getattr(self.current_task, "phase", None) if self.current_task else None,
            "coverage_ratio": round(grid.coverage_ratio(), 3),
            "known_bbox_fill_ratio": round(grid.known_bbox_fill_ratio(), 3),
            "traversable_known_ratio": round(grid.traversable_known_ratio(), 3),
            "visited_traversable_ratio": round(grid.visited_traversable_ratio(), 3),
            "current_goal": (
                [round(active_goal[0], 2), round(active_goal[1], 2)]
                if active_goal is not None
                else None
            ),
            # 改用公开快照而非伸手取控制器私有量：
            # _last_goal_distance 是死字段（全仓库只被赋 math.inf，从无第二个写入
            # 点），原式在 has_path() 为真时恒得 Infinity，经 json.dumps 默认
            # allow_nan 写出裸 Infinity —— /mission_status 话题上是非法 JSON。
            # route_progress 才是真实剩余航程（_remaining_path_distance 每周期
            # 更新），且已由 _finite_or_none 兜住非有限值。
            "path_length": (
                controller_debug["route_progress"]
                if controller_debug["route_progress"] is not None
                else 0.0
            ),
            # recover_remaining = max(0.0, _recover_until - now)，与原来的
            # _recover_until > now 精确等价，无需再读私有量。
            "is_stuck": controller_debug["recover_remaining"] > 0.0,
            "controller": controller_debug,
        }
        self._reporting.publish_status(
            StatusReport(
                payload=payload,
                grid=grid,
                floor_index=self.current_floor,
                floor_height=self.mapper.floor_height,
            )
        )
        self._log_runtime_diagnostics()

    def _log_runtime_diagnostics(self) -> None:
        """按状态发布周期汇总控制、规划和建图诊断，避免在 10 Hz 主循环刷屏。"""
        diagnostic = self._mission_diagnostic_snapshot()
        diagnostic["event"] = "mission_runtime"
        diagnostic["pose"] = (
            [round(float(value), 3) for value in self._last_pose]
            if self._last_pose is not None
            else None
        )
        last_xyz = getattr(self, "_last_xyz", None)
        diagnostic["odom_xyz"] = (
            [round(float(value), 3) for value in last_xyz]
            if last_xyz is not None
            else None
        )
        diagnostic["mapper_estimated_floor"] = (
            self._floor_coordinates.estimate_floor(
                last_xyz,
                current_floor=self.current_floor,
            )
            if last_xyz is not None
            else None
        )
        diagnostic["localization_vertical_reference"] = getattr(
            self, "localization_vertical_reference", "absolute"
        )
        diagnostic["logical_xyz"] = (
            [
                round(float(value), 3)
                for value in self._floor_coordinates.logical_xyz(
                    last_xyz,
                    self.current_floor,
                )
            ]
            if last_xyz is not None
            else None
        )

        controller_snapshot = getattr(self.controller, "tracking_debug_snapshot", None)
        diagnostic["controller"] = controller_snapshot() if callable(controller_snapshot) else None

        mapper_snapshot = getattr(self.mapper, "latest_debug_snapshot", None)
        latest_mapper_debug = mapper_snapshot() if callable(mapper_snapshot) else None
        diagnostic["mapper"] = dict(vars(latest_mapper_debug)) if latest_mapper_debug is not None else None

        obstacle = self._map_runtime.nearest_forward_obstacle
        diagnostic["nearest_forward_obstacle"] = (
            {
                "distance": round(float(obstacle.distance), 4),
                "world_xyz": [round(float(value), 4) for value in obstacle.world_xyz],
                "forward": round(float(obstacle.forward), 4),
                "lateral": round(float(obstacle.lateral), 4),
                "relative_height": round(float(obstacle.relative_height), 4),
            }
            if obstacle is not None
            else None
        )
        radial_clearance = self._map_runtime.nearest_radial_obstacle_distance
        diagnostic["nearest_radial_obstacle_distance"] = (
            round(float(radial_clearance), 4)
            if radial_clearance is not None and math.isfinite(float(radial_clearance))
            else None
        )
        current_task = getattr(self, "current_task", None)
        diagnostic["cabin_transit_stage"] = (
            getattr(current_task, "cabin_transit_stage", None)
            if isinstance(current_task, TakeElevator)
            else None
        )

        selection_debug = getattr(self.explorer, "last_selection_debug", None)
        diagnostic["frontier_selection_wall_duration"] = getattr(
            selection_debug,
            "selection_wall_duration",
            None,
        )
        self._log_structured(diagnostic)

    def _elapsed(self) -> float:
        return get_time() - self.start_time

    def _transition_state(
        self,
        new_state: str,
        *,
        reason_code: str,
        call_site: str,
        diagnostic_context: dict[str, object] | None = None,
    ) -> None:
        """执行唯一的顶层状态转换并保存结构化审计记录。

        ``reason_code`` 描述机器可聚合的原因，``call_site`` 标识决策位置，额外
        上下文保存当时的规划/安全信息。进入返航时清除探索目标，进入写结果时
        锁存写出原因，避免旧路径或后续 shutdown 覆盖真实终态。
        """
        old_state = getattr(self, "state", None)
        entering_return_home = new_state == RETURN_HOME and old_state != RETURN_HOME
        if entering_return_home:
            # 旧状态留下的后台结果基于旧地图/旧目标，不能在新返航周期复用。
            self._return_home.planner = None
            clear_path = getattr(getattr(self, "controller", None), "clear_path", None)
            if callable(clear_path):
                clear_path(reason="enter_return_home")
            self._return_home.reset_feedback()
        self.state = new_state
        if new_state != RETURN_HOME:
            self._return_home.planner = None
        if new_state == RETURN_HOME:
            if not getattr(self, "_return_home_started", False) or not self._return_home.reason:
                self._return_home.reason = reason_code
            self._return_home_started = True
        if new_state == WRITE_RESULT:
            self._pending_result_write_reason = reason_code

        diagnostic = self._mission_diagnostic_snapshot()
        if diagnostic_context:
            diagnostic.update(diagnostic_context)
        diagnostic.update(
            {
                "event": "mission_state_transition",
                "old_state": old_state,
                "new_state": new_state,
                "reason_code": reason_code,
                "call_site": call_site,
            }
        )
        history = getattr(self, "_transition_history", None)
        if history is None:
            history = []
            self._transition_history = history
        history.append(diagnostic)
        self._log_structured(diagnostic)
        if entering_return_home:
            self.current_frontier = None
            self._exploration.last_frontier_target = None
            self._exploration.last_frontier_score = None
            self._exploration.last_frontier_used_fallback = False

    def _mission_diagnostic_snapshot(self) -> dict[str, object]:
        coverage_ratio = None
        mapper = getattr(self, "mapper", None)
        floor = int(getattr(self, "current_floor", 0))
        if mapper is not None:
            try:
                coverage_ratio = round(float(mapper.floor(floor).coverage_ratio()), 3)
            except (AttributeError, IndexError, TypeError, ValueError):
                coverage_ratio = None

        selection_debug = getattr(getattr(self, "explorer", None), "last_selection_debug", None)
        reachable_count = getattr(selection_debug, "reachable_count", None)
        current_frontier = getattr(getattr(self, "current_frontier", None), "centroid", None)
        if current_frontier is not None:
            current_frontier = [round(float(value), 3) for value in current_frontier]
        current_task = getattr(self, "current_task", None)
        task_name = (
            getattr(current_task, "name", type(current_task).__name__)
            if current_task is not None
            else None
        )

        elapsed = None
        if hasattr(self, "start_time"):
            try:
                elapsed = round(float(self._elapsed()), 3)
            except (TypeError, ValueError):
                elapsed = None
        floor_transition_guard = getattr(self, "_floor_transition_guard", None)
        support_gate = getattr(self, "_ground_support_gate", None)
        support_snapshot = getattr(support_gate, "snapshot", None)
        return {
            "state": getattr(self, "state", None),
            "elapsed": elapsed,
            "coverage_ratio": coverage_ratio,
            "current_task": task_name,
            "current_task_phase": getattr(current_task, "phase", None) if current_task is not None else None,
            "current_frontier": current_frontier,
            "last_reachable_frontier_count": reachable_count,
            "has_reachable_frontier": reachable_count > 0 if reachable_count is not None else None,
            "no_reachable_frontier_confirmations": self._exploration.no_reachable_frontier_confirmations,
            "no_frontier_escape": {
                "state": self._exploration.no_frontier_escape_state,
                "target": (
                    list(self._exploration.no_frontier_escape_target)
                    if self._exploration.no_frontier_escape_target is not None
                    else None
                ),
            },
            "current_floor": floor,
            "target_floor": getattr(self, "target_floor", None),
            "floor_transition_ambiguous": floor_transition_guard is not None,
            "floor_transition_skipped_cloud_count": (
                floor_transition_guard.skipped_cloud_count
                if floor_transition_guard is not None
                else 0
            ),
            "ground_support_gate": (
                support_snapshot() if callable(support_snapshot) else None
            ),
            "pose_lookup": {
                stream: dict(values)
                for stream, values in getattr(self, "_pose_lookup_stats", {}).items()
            },
            "return_home_started": bool(getattr(self, "_return_home_started", False)),
            "return_home_completed": bool(
                getattr(self, "_return_home_completed", False)
                or getattr(self, "_return_home_success", False)
            ),
            "return_home_feedback": {
                "stuck_anchor": (
                    list(self._return_home.stuck_anchor)
                    if self._return_home.stuck_anchor is not None
                    else None
                ),
                "stuck_count": self._return_home.stuck_count,
                "candidate_reject_count": self._return_home.candidate_reject_count,
                "failed_segments": [
                    list(segment)
                    for segment in self._return_home.failed_segments
                ],
                "last_stuck_obstacle": self._return_home.last_stuck_obstacle,
            },
            "mission_success_eligible": bool(
                getattr(self, "_mission_success_eligible", True)
            ),
            "invalid_floor_completion_evidence": list(
                getattr(self, "_invalid_floor_completion_evidence", ())
            ),
            "terminal_outcome": getattr(self, "_terminal_outcome", "running"),
        }

    @staticmethod
    def _log_structured(payload: dict[str, object]) -> None:
        logger = getattr(MissionReportingRuntime, "log_structured", None)
        if callable(logger):
            logger(payload)

    def _record_failure(self, reason: str) -> None:
        if reason and reason not in self._failure_reasons:
            self._failure_reasons.append(reason)

    def _update_pose_progress(self, pose: tuple[float, float, float]) -> None:
        self._last_pose = pose
        if self.home_pose is None:
            return
        self._max_home_distance = max(
            self._max_home_distance,
            math.hypot(pose[0] - self.home_pose[0], pose[1] - self.home_pose[1]),
        )

    def _request_shutdown(self, reason: str) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        runtime = self._node_runtime
        if runtime is None:
            raise RuntimeError("MissionManager 尚未绑定节点运行时")
        runtime.request_shutdown(reason)

    def _mission_summary_snapshot(self, detected_count: int) -> dict[str, object] | None:
        if not self.mission_summary_file:
            return None
        grid = self.mapper.floor(self.current_floor)
        return {
            "exploration_time": round(self._elapsed(), 2),
            "state": getattr(self, "_state_at_result_write", None) or self.state,
            "terminal_state": getattr(self, "_terminal_state", None),
            "terminal_outcome": getattr(self, "_terminal_outcome", "failed"),
            "terminal_reason": getattr(self, "_terminal_reason", None),
            "completed": bool(getattr(self, "_normal_completion", False)),
            "exploration_completed": bool(
                getattr(self, "_exploration_completed", False)
            ),
            "return_home_reason": self._return_home.reason,
            "mission_success_eligible": bool(
                getattr(self, "_mission_success_eligible", True)
            ),
            "state_at_result_write": getattr(self, "_state_at_result_write", None),
            "result_write_reason": getattr(self, "_result_write_reason", None),
            "result_write_call_site": getattr(self, "_result_write_call_site", None),
            "transition_history": list(getattr(self, "_transition_history", [])),
            "shutdown_context": getattr(self, "_last_shutdown_diagnostic", None),
            "current_floor": self.current_floor,
            "max_floor": self.max_floor,
            "return_home_success": self._return_home_success,
            "return_home_started": bool(getattr(self, "_return_home_started", False)),
            "return_home_completed": bool(
                getattr(self, "_return_home_completed", False) or self._return_home_success
            ),
            "elevator_success": self._elevator_success,
            "timeout_triggered": self._timeout_triggered,
            "failure_reason": list(self._failure_reasons),
            "detected_count": int(detected_count),
            "track_count": len(self.tracker.tracks),
            "confirmed_count": int(detected_count),
            "final_pose": list(self._last_pose) if self._last_pose is not None else None,
            "max_home_distance": round(self._max_home_distance, 3),
            "coverage_ratio": round(grid.coverage_ratio(), 3),
            "known_area": round(grid.known_area(), 2),
            "visited_area": round(grid.visited_area(), 2),
            "controller_debug": self.controller.command_debug_summary(),
            "ground_support_gate": (
                self._ground_support_gate.snapshot()
                if getattr(self, "_ground_support_gate", None) is not None
                else None
            ),
            "pose_lookup": {
                stream: dict(values)
                for stream, values in getattr(self, "_pose_lookup_stats", {}).items()
            },
        }



def _pose_sample_from_values(*, stamp: float, pose, xyz) -> PoseSample:
    half_yaw = float(pose[2]) * 0.5
    return PoseSample(
        stamp=float(stamp),
        xyz=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
        quaternion_xyzw=(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)),
        linear_xyz=(0.0, 0.0, 0.0),
        angular_xyz=(0.0, 0.0, 0.0),
    )


def _ground_support_unknown_hold_enabled(
    task,
    *,
    configured: bool = True,
    trusted_return_trace: bool = False,
) -> bool:
    """返回当前任务是否启用 UNKNOWN 临时停车；显式 DROP 不受此值影响。"""
    if not configured:
        return False
    controlled_elevator_threshold = (
        isinstance(task, TakeElevator)
        and getattr(task, "phase", None) in {"ENTER_CABIN", "EXIT_CABIN"}
    )
    controlled_open_door_threshold = (
        isinstance(task, OpenDoorAndPass)
        and getattr(task, "phase", None) == "GO_PASS_POINT"
        and bool(getattr(task, "opening_sensor_confirmed", False))
    )
    return not (
        controlled_elevator_threshold
        or controlled_open_door_threshold
        or bool(trusted_return_trace)
    )


def _controller_command_transaction(controller):
    """兼容生产 PathFollower 与轻量测试替身的控制事务上下文。"""
    transaction = getattr(controller, "command_transaction", None)
    return transaction() if callable(transaction) else nullcontext()


def _building_services_required(*, max_floor: int, public_door_ids) -> bool:
    """公开门 ID 即需要连接门服务，空间几何是否已注册与连接无关。"""
    return int(max_floor) > 1 or bool(public_door_ids)


def segment_crosses_blocked_cell(
    grid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    constraints=None,
) -> bool:
    start_cell = grid.world_to_grid(start_xy[0], start_xy[1])
    end_cell = grid.world_to_grid(end_xy[0], end_xy[1])
    if start_cell is None or end_cell is None:
        return True
    distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    step_count = max(1, int(math.ceil(distance / max(grid.resolution * 0.5, 1e-6))))
    for index in range(1, step_count + 1):
        ratio = index / step_count
        x = start_xy[0] + (end_xy[0] - start_xy[0]) * ratio
        y = start_xy[1] + (end_xy[1] - start_xy[1]) * ratio
        cell = grid.world_to_grid(x, y)
        if cell is None:
            return True
        gx, gy = cell
        if int(grid.data[gy, gx]) in (BLACK, NEGATIVE, OCC) or (
            constraints is not None and cell in constraints.blocked_cells
        ):
            return True
    return False

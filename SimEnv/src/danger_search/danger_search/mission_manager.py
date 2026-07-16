"""danger_search 主任务状态机。"""
from __future__ import annotations

import json
import math
import os
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from threading import Event, Thread
import time

import numpy as np

try:
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
except ImportError:  # pragma: no cover - 离线环境可能没有 ROS
    rospy = None
    String = None
    PoseStamped = None
    OccupancyGrid = None

from danger_search.detection_tracker import DetectionTracker
from danger_search.building_control_client import BuildingControlClient, DoorInfo
from danger_search.navigation_tasks import NavigationTask, TakeElevator, OpenDoorAndPass, NavigateToPose, FAILED
from danger_search.frontier import Frontier, FrontierExplorer, FrontierSelectionContext
from danger_search.ground_support_gate import (
    CLEAR as GROUND_SUPPORT_CLEAR,
    DROP_STOPPING,
    OBSERVATION_HOLD,
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SPEED_OUT_OF_ENVELOPE_STOPPING,
    STALE_DATA_HOLD,
    SUPPORT_PATH_UNAVAILABLE,
    GroundSupportAssessment,
    GroundSupportGateConfig,
    GroundSupportSafetyGate,
)
from .time_utils import get_time
from danger_search.local_semantic_detector import DoorCandidate, LocalSemanticDetector
from danger_search.navigator import DONE, IDLE, STUCK, PathFollower
from danger_search.planner import BUDGET_EXHAUSTED, astar_path, astar_plan, path_length
from danger_search.mapper import FREE, UNKNOWN, VISITED, MultiFloorGridMap
from danger_search.red_sphere_detector import RedSphereDetector
from danger_search.result_writer import ResultWriter
from danger_search.sensor_buffer import (
    SensorBuffer,
    odom_angular_speed,
    odom_planar_speed,
    odom_pose_xy_yaw,
    odom_rotation_matrix,
    odom_xyz,
)
from danger_search.support_shadow import ShadowSupportConfig, ShadowSupportGrid


INIT = "INIT"
WAIT_CONTROLLER_READY = "WAIT_CONTROLLER_READY"
EXPLORE_FLOOR = "EXPLORE_FLOOR"
CHANGE_FLOOR = "CHANGE_FLOOR"
RETURN_HOME = "RETURN_HOME"
WRITE_RESULT = "WRITE_RESULT"
FINISH = "FINISH"
MISSION_FAILED = "FAILED"
PROBE_LATERAL_OFFSETS = (0.0, -1.0, 1.0, -2.0, 2.0)
FLOOR_ALIGNMENT_REQUIRED_ODOM_SAMPLES = 2


@dataclass
class _FloorTransitionGuard:
    """电梯换层期间 floor-scoped 点云处理的锁存诊断状态。"""

    entered_at: float
    enter_odom_stamp: float | None
    source_floor: int
    enter_estimated_floor: int
    target_floor: int | None
    enter_phase: str | None
    last_mismatch_odom_stamp: float | None
    alignment_floor: int | None = None
    alignment_sample_count: int = 0
    last_alignment_odom_stamp: float | None = None
    task_exit_at: float | None = None
    timeout_reported: bool = False
    skipped_cloud_count: int = 0
    first_skipped_at: float | None = None
    last_skipped_at: float | None = None
    first_skipped_cloud_stamp: float | None = None
    last_skipped_cloud_stamp: float | None = None
    last_skip_log_key: tuple[int, int, str | None] | None = None


class _ProgressMonitor:
    """任务层最小进度监督器。

    检测机器人在短时间内反复选中同一近邻目标、实际位移接近零的退化循环。
    触发后仅对局部区域设置短期 cooldown，不修改地图，也不直接触发返航。
    """

    def __init__(
        self,
        *,
        goal_tolerance: float,
        window: float,
        repeat: int,
        min_displacement: float,
        cooldown: float,
    ) -> None:
        self.region_radius = max(0.30, float(goal_tolerance) * 1.5)
        self.window = float(window)
        self.repeat = max(2, int(repeat))
        self.min_displacement = float(min_displacement)
        self.cooldown = float(cooldown)
        self._selections: deque[tuple[float, tuple[float, float]]] = deque()
        self._poses: deque[tuple[float, tuple[float, float, float]]] = deque()
        self._cooldowns: list[tuple[float, tuple[float, float], str]] = []
        self._loop_count = 0
        self._last_log_time = 0.0
        self._last_reason_code: str | None = None

    def record_pose(self, timestamp: float, pose: tuple[float, float, float]) -> None:
        self._poses.append((float(timestamp), pose))
        self._expire(timestamp)

    def record_selection(self, timestamp: float, target_xy: tuple[float, float]) -> None:
        self._selections.append((float(timestamp), target_xy))
        self._expire(timestamp)

    def check_degenerate(
        self,
        timestamp: float,
        target_xy: tuple[float, float],
    ) -> tuple[bool, int, float, str | None]:
        """检查当前目标是否构成退化循环。

        返回 (is_degenerate, repeat_count, displacement, reason_code)。
        """
        self._expire(timestamp)
        repeat_count = sum(
            1
            for t, xy in self._selections
            if self._same_region(xy, target_xy) and (timestamp - t) <= self.window
        )
        displacement = self._displacement(timestamp)
        if repeat_count >= self.repeat and displacement < self.min_displacement:
            self._loop_count += 1
            self._last_reason_code = "degenerate_goal_reached_loop"
            return True, repeat_count, displacement, self._last_reason_code
        self._loop_count = 0
        self._last_reason_code = None
        return False, repeat_count, displacement, None

    def start_cooldown(
        self,
        timestamp: float,
        center_xy: tuple[float, float],
        reason_code: str,
    ) -> None:
        self._cooldowns.append((float(timestamp) + self.cooldown, center_xy, reason_code))
        self._loop_count = 0

    def is_cooled(self, timestamp: float, target_xy: tuple[float, float]) -> bool:
        self._expire(timestamp)
        for _, center, _ in self._cooldowns:
            if self._same_region(center, target_xy):
                return True
        return False

    def filter_frontiers(
        self,
        timestamp: float,
        frontiers: list[Frontier],
    ) -> list[Frontier]:
        self._expire(timestamp)
        return [f for f in frontiers if not self.is_cooled(timestamp, f.centroid)]

    def should_log(self, timestamp: float, interval: float = 5.0) -> bool:
        if timestamp - self._last_log_time >= interval:
            self._last_log_time = timestamp
            return True
        return False

    def cooldown_reason(self, timestamp: float, target_xy: tuple[float, float]) -> str | None:
        self._expire(timestamp)
        for expires, center, reason in self._cooldowns:
            if self._same_region(center, target_xy):
                return reason
        return None

    def _same_region(
        self,
        a: tuple[float, float],
        b: tuple[float, float],
    ) -> bool:
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= self.region_radius

    def _displacement(self, timestamp: float) -> float:
        recent = [(t, p) for t, p in self._poses if (timestamp - t) <= self.window]
        if len(recent) < 2:
            return 0.0
        first = recent[0][1]
        last = recent[-1][1]
        return math.hypot(last[0] - first[0], last[1] - first[1])

    def _expire(self, timestamp: float) -> None:
        while self._selections and (timestamp - self._selections[0][0]) > self.window:
            self._selections.popleft()
        while self._poses and (timestamp - self._poses[0][0]) > self.window:
            self._poses.popleft()
        self._cooldowns = [
            (expires, center, reason)
            for expires, center, reason in self._cooldowns
            if expires > timestamp
        ]


class MissionManager:
    """Timer 驱动的主状态机，避免在主循环里阻塞等待传感器。"""

    def __init__(self):
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法运行 mission_manager")
        self.state = INIT
        self.start_time = get_time()
        self.floor_start_time = self.start_time
        self.home_pose: tuple[float, float, float] | None = None
        self.current_floor = 0
        self.target_floor: int | None = None
        self.return_after_floor_change = False
        self.current_frontier: Frontier | None = None
        self._last_image_stamp: float | None = None
        self._last_cloud_stamp: float | None = None
        self._service_task = None
        self._change_phase: str | None = None
        self._no_frontier_scan_until = 0.0
        self._no_reachable_frontier_confirmations = 0
        self._last_no_frontier_signature = None
        self._floor_timeout_logged = False
        self._last_status_publish = 0.0
        self._probe_attempt = 0
        self._timer = None
        self._finished = False
        self._shutdown_requested = False
        self._transition_history: list[dict[str, object]] = []
        self._return_home_started = False
        self._return_home_completed = False
        self._mission_success_eligible = True
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
        self._elevator_success: bool | None = None
        self._timeout_triggered = False
        self._failure_reasons: list[str] = []
        self._last_pose: tuple[float, float, float] | None = None
        self._last_xyz: tuple[float, float, float] | None = None
        self._last_floor_alignment_mismatch_key = None
        self._floor_transition_guard: _FloorTransitionGuard | None = None
        self._max_home_distance = 0.0
        self.manual_goal: tuple[float, float] | None = None
        self.manual_mode = False
        self.current_task: NavigationTask | None = None
        self.opened_doors: set[str] = set()
        self.failed_doors: set[str] = set()
        self.active_door: str | None = None
        self._door_approach_blacklist: dict[str, float] = {}
        self._latest_cloud_points: list[tuple[float, float, float]] = []
        self._post_door_frontier_debug: dict[str, object] | None = None
        self.last_door_id: str | None = None
        self.last_door_xy: tuple[float, float] | None = None
        self.last_door_pass_vector: tuple[float, float] | None = None
        self.last_door_transition_until: float = 0.0
        self.last_door_context_until: float = 0.0
        self.last_door_completion_pose: tuple[float, float, float] | None = None

        self.main_entrance_xy: tuple[float, float] | None = None
        self.main_entrance_pass_vector: tuple[float, float] | None = None

        self._last_frontier_target: tuple[float, float] | None = None
        self._last_frontier_score: float | None = None
        self._last_frontier_used_fallback = False

        # 方案 C：探索进展历史，用于判断当前层是否仍有收益
        self._progress_history: list[dict[str, object]] = []
        self._last_budget_decision: tuple[str, str] | None = None
        self._last_budget_decision_time: float = 0.0
        self._ground_support_gate: GroundSupportSafetyGate | None = None
        self._ground_support_grid: ShadowSupportGrid | None = None
        self._ground_support_sensor_mount: np.ndarray | None = None
        self._last_ground_support_cloud_stamp: float | None = None
        self._ground_support_timer = None
        self._ground_support_last_state = GROUND_SUPPORT_CLEAR
        self._ground_support_wall_stop = Event()
        self._ground_support_wall_thread: Thread | None = None
        self._ground_support_forced_shutdown_reason: str | None = None
        self._ground_support_path_retry_key = None
        self._ground_support_path_retry_count = 0
        self._ground_support_negative_replan_count = 0
        self._ground_support_negative_obstacle_marks: list[
            tuple[int, float, float, float]
        ] = []

        self._load_params()
        self._status_publisher = None
        self.sensor_buffer = SensorBuffer(
            odom_topic=self.odom_topic,
            cloud_topic=self.cloud_topic,
            rgb_topic=self.rgb_topic,
            depth_topic=self.depth_topic,
            camera_info_topic=self.camera_info_topic,
        )
        self.mapper = MultiFloorGridMap(**self.map_params)
        self.explorer = FrontierExplorer(**self.frontier_params)
        self.controller = PathFollower(
            cmd_vel_topic=self.cmd_vel_topic,
            path_raw_topic=self.path_raw_topic,
            path_followed_topic=self.path_followed_topic,
            local_target_topic=self.local_target_topic,
            **self.controller_params,
        )
        self._configure_ground_support_gate()
        self.detector = RedSphereDetector(**self.detection_params)
        self.local_detector = LocalSemanticDetector(**self.local_detection_params)
        self.tracker = DetectionTracker(**self.tracker_params)
        metadata_abs_path = str(Path(__file__).resolve().parents[3] / "generated_building" / "layout_metadata.json")
        self.building_client = BuildingControlClient(
            metadata_path=metadata_abs_path,
            service_timeout=self.floor_params.get("service_timeout", 5.0),
        )
        self._progress_monitor = _ProgressMonitor(
            goal_tolerance=float(self.controller_params.get("goal_tolerance", 0.35)),
            window=self.progress_monitor_window,
            repeat=self.progress_monitor_repeat,
            min_displacement=self.progress_monitor_min_displacement,
            cooldown=self.progress_monitor_cooldown,
        )
        self.result_writer = ResultWriter(self.result_file)

    def _configure_ground_support_gate(self) -> None:
        params = self.ground_support_params
        if not params["production_gate_enabled"]:
            return
        gate_config = GroundSupportGateConfig(
            enabled=True,
            validated_max_speed=params["validated_max_speed"],
            stop_distance_table=params["stop_distance_table"],
            drop_evidence_max_age=params["drop_evidence_max_age"],
            decision_period=params["decision_period"],
            rl_receive_delay=params["rl_receive_delay"],
            required_margin=params["required_margin"],
            release_hysteresis=params["release_hysteresis"],
            release_supported_frames=params["release_supported_frames"],
            observation_hold_timeout=params["observation_hold_timeout"],
            safety_stop_timeout=params["safety_stop_timeout"],
            stop_linear_threshold=params["stop_linear_threshold"],
            stop_angular_threshold=params["stop_angular_threshold"],
            stop_confirmation_samples=params["stop_confirmation_samples"],
        )
        sweep_half_width = params["sweep_half_width"]
        support_config = ShadowSupportConfig(
            resolution=self.map_params["resolution"],
            origin_x=self.map_params["origin_x"],
            origin_y=self.map_params["origin_y"],
            floor_height=self.map_params["floor_height"],
            # MissionManager 已按生产建图配置抽样，禁止在安全门内重复降采样。
            cloud_stride=1,
            cloud_max_points=self.cloud_max_points,
            support_band=params["support_band"],
            minimum_drop=params["minimum_drop"],
            minimum_cell_hits=params["minimum_cell_hits"],
            required_consecutive_frames=params["required_consecutive_frames"],
            minimum_alert_cells=params["minimum_alert_cells"],
            sweep_half_width=sweep_half_width,
            foot_band_center_offset=params["foot_band_center_offset"],
            foot_band_half_width=params["foot_band_half_width"],
            minimum_foot_band_coverage=params["minimum_foot_band_coverage"],
            maximum_longitudinal_unknown_gap=params[
                "maximum_longitudinal_unknown_gap"
            ],
            minimum_longitudinal_support_ratio=params[
                "minimum_longitudinal_support_ratio"
            ],
            minimum_lookahead=params["minimum_lookahead"],
            maximum_lookahead=params["maximum_lookahead"],
            footprint_front_reach=params["footprint_front_reach"],
            path_goal_tolerance=self.controller.goal_tolerance,
            obstacle_min_height=self.map_params["obstacle_min_z"],
            obstacle_max_height=self.map_params["obstacle_max_z"],
            minimum_frame_points=params["minimum_frame_points"],
            local_support_evidence_radius=params["local_support_evidence_radius"],
            threshold_widths=(sweep_half_width,),
        )
        self._ground_support_gate = GroundSupportSafetyGate(gate_config)
        self._ground_support_grid = ShadowSupportGrid(support_config)
        self.sensor_buffer.enable_cloud_odom_matching()
        self.controller.set_command_gate(self._ground_support_gate)

    def start(self) -> None:
        self.sensor_buffer.start()
        self.controller.start()
        if self.max_floor > 1 or self.building_client.has_controllable_doors():
            self.building_client.start(connect_elevator=self.max_floor > 1)
        self._status_publisher = rospy.Publisher(self.status_topic, String, queue_size=1)
        if OccupancyGrid is not None:
            self._map_publisher = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1, latch=True)
        else:
            self._map_publisher = None
        if PoseStamped is not None:
            self._goal_subscriber = rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)
        if self._ground_support_gate is not None:
            self._ground_support_timer = rospy.Timer(
                rospy.Duration(self.ground_support_params["assessment_period"]),
                self._on_ground_support_timer,
            )
            self._ground_support_wall_thread = Thread(
                target=self._ground_support_wall_watch,
                name="ground-support-wall-watch",
                daemon=True,
            )
            self._ground_support_wall_thread.start()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("danger_search 已启动，等待传感器数据")

    def _goal_callback(self, msg) -> None:
        self.manual_mode = True
        self.manual_goal = (msg.pose.position.x, msg.pose.position.y)
        self.current_task = NavigateToPose(self.manual_goal)

        self.controller.clear_path()
        self.current_frontier = None
        rospy.loginfo("收到新的手动导航目标点: (%.2f, %.2f)，已切换到手动目标模式！", self.manual_goal[0], self.manual_goal[1])

    def _ground_support_wall_watch(self) -> None:
        """仿真时钟冻结时仍能执行安全停车异常上限。"""
        gate = self._ground_support_gate
        while gate is not None and not self._ground_support_wall_stop.wait(0.05):
            gate.check_wall_timeout(time.monotonic())
            if gate.state != SAFETY_STOP_TIMEOUT:
                continue
            reason = gate.reason_code or "ground_support_safety_stop_timeout"
            self._ground_support_forced_shutdown_reason = reason
            self._mission_success_eligible = False
            self._record_failure(reason)
            self.controller.stop(reason=reason)
            self._shutdown_requested = True
            rospy.signal_shutdown(reason)
            return

    def shutdown(self) -> None:
        wall_stop = getattr(self, "_ground_support_wall_stop", None)
        if wall_stop is not None:
            wall_stop.set()
        write_reason = (
            getattr(self, "_ground_support_forced_shutdown_reason", None)
            or "mission_requested_shutdown"
            if bool(getattr(self, "_shutdown_requested", False))
            else "external_ros_shutdown"
        )
        diagnostic = self._mission_diagnostic_snapshot()
        diagnostic.update(
            {
                "event": "mission_shutdown",
                "state_at_shutdown": getattr(self, "state", None),
                "shutdown_requested_internally": bool(getattr(self, "_shutdown_requested", False)),
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
                call_site="MissionManager.shutdown",
            )

    def _load_params(self) -> None:
        self.cmd_vel_topic = _param("topics/cmd_vel", "/cmd_vel")
        self.odom_topic = _param("topics/odom", "/Odometry_gazebo")
        self.cloud_topic = _param("topics/cloud", "/livox/Pointcloud2")
        self.rgb_topic = _param("topics/rgb", "/real_sense/rgb/image_raw")
        self.depth_topic = _param("topics/depth", "/real_sense/depth/image_raw")
        self.camera_info_topic = _param("topics/camera_info", "/real_sense/rgb/camera_info")
        self.status_topic = _param("topics/status", "/nav/status")
        self.goal_topic = _param("topics/goal", "/nav/goal")
        self.map_topic = _param("topics/map", "/nav/map_debug")
        self.path_raw_topic = _param("topics/path_raw", "/danger_search/path_raw")
        self.path_followed_topic = _param(
            "topics/path_followed",
            "/danger_search/path_followed",
        )
        self.local_target_topic = _param(
            "topics/local_target",
            "/danger_search/local_target",
        )

        self.startup_delay = float(_param("mission/startup_delay", 2.0))
        self.max_runtime = float(_param("mission/max_runtime", 590.0))
        self.max_floor = int(_param("mission/max_floor", 1))
        self.floor_timeout = float(_param("mission/floor_timeout", 180.0))
        self.last_floor_uses_global_budget = bool(
            _param("mission/last_floor_uses_global_budget", True)
        )
        self.min_floor_explore_time = float(_param("mission/min_floor_explore_time", 20.0))

        # 方案 C：全局时间预算参数
        self.return_safety_buffer = float(_param("mission/return_safety_buffer", 30.0))
        self.return_speed_estimate = float(_param("mission/return_speed_estimate", 0.35))
        self.floor_change_time_estimate = float(_param("mission/floor_change_time_estimate", 40.0))
        self.floor_finish_time_ratio = float(_param("mission/floor_finish_time_ratio", 0.75))
        self.exploration_progress_window = float(_param("mission/exploration_progress_window", 15.0))
        self.min_exploration_progress_cells = int(_param("mission/min_exploration_progress_cells", 30))
        self.min_remaining_frontiers = int(_param("mission/min_remaining_frontiers", 2))
        self.min_floor_change_remaining_ratio = float(_param("mission/min_floor_change_remaining_ratio", 0.25))
        self.min_floor_travel_distance = float(_param("mission/min_floor_travel_distance", 6.0))
        self.min_known_area = float(_param("mission/min_known_area", 45.0))
        self.no_reachable_frontier_confirmations = max(1, int(_param("mission/no_reachable_frontier_confirmations", 2)))
        self.min_corridor_progress_distance = float(_param("mission/min_corridor_progress_distance", 0.0))
        self.post_door_min_known_area = float(_param("mission/post_door_min_known_area", 8.0))
        self.post_door_min_distance = float(_param("mission/post_door_min_distance", 2.0))
        self.home_tolerance = float(_param("mission/home_tolerance", 0.7))
        self.explore_coverage_ratio = float(_param("mission/explore_coverage_ratio", 0.82))
        self.no_frontier_scan_duration = float(_param("mission/no_frontier_scan_duration", 8.0))
        self.probe_distance = float(_param("mission/probe_distance", 1.2))
        self.probe_yaw = float(_param("mission/probe_yaw", math.pi / 2.0))
        self.door_pass_offset = float(_param("mission/door_pass_offset", 0.8))
        self.door_commit_distance = float(_param("mission/door_commit_distance", 1.8))
        self.door_clear_radius = float(_param("mission/door_clear_radius", 0.9))
        self.door_transition_duration = float(_param("mission/door_transition_duration", 3.0))
        self.door_transition_hold_distance = float(_param("mission/door_transition_hold_distance", 2.5))
        self.post_door_context_max_duration = float(
            _param("mission/post_door_context_max_duration", 12.0)
        )
        self.permanent_forbidden_radius_scale = float(
            _param("mission/permanent_forbidden_radius_scale", 1.5)
        )
        self.permanent_forbidden_penalty = float(
            _param("mission/permanent_forbidden_penalty", 3.0)
        )
        self.permanent_forbidden_decay_scale = float(
            _param("mission/permanent_forbidden_decay_scale", 3.0)
        )
        self.status_publish_period = float(_param("mission/status_publish_period", 1.0))
        self.exit_cabin_backoff_enabled = bool(
            _param("mission/exit_cabin_backoff_enabled", False)
        )
        self.result_file = str(_param("result_file", os.path.join(os.getcwd(), "results", "detected_danger.json")))
        self.mission_summary_file = str(_param("mission_summary_file", ""))

        self.map_params = {
            "resolution": float(_param("map/resolution", 0.10)),
            "width": float(_param("map/width", 28.0)),
            "length": float(_param("map/length", 48.0)),
            "origin_x": float(_param("map/origin_x", -14.0)),
            "origin_y": float(_param("map/origin_y", -6.0)),
            "floor_height": float(_param("map/floor_height", 2.6)),
            "obstacle_min_z": float(_param("map/obstacle_min_z", 0.15)),
            "obstacle_max_z": float(_param("map/obstacle_max_z", 1.40)),
            "obstacle_inflation_radius": float(_param("map/obstacle_inflation_radius", 0.25)),
            "raycast_max_range": float(_param("map/raycast_max_range", 12.0)),
            "point_free_clear_max_range": float(
                _param("map/point_free_clear_max_range", 1.5)
            ),
            "visited_radius": float(_param("map/visited_radius", 0.45)),
            "free_clear_enabled": bool(_param("map/free_clear_enabled", True)),
            "free_clear_max_range": float(_param("map/free_clear_max_range", 8.0)),
            "free_clear_angle_step": float(_param("map/free_clear_angle_step", 0.1745)),
            "free_clear_obstacle_margin": float(_param("map/free_clear_obstacle_margin", 0.25)),
            "free_clear_min_points_per_bin": int(_param("map/free_clear_min_points_per_bin", 2)),
        }
        self.cloud_stride = int(_param("map/cloud_stride", 3))
        self.cloud_max_points = int(_param("map/cloud_max_points", 18000))
        stop_distance_table = _parse_stop_distance_table(
            _param(
                "ground_support/stop_distance_table",
                [
                    [0.60, 0.09337727739487302],
                    [0.651, 0.10195826367975175],
                    [0.80, 0.13],
                ],
            )
        )
        self.ground_support_params = {
            "production_gate_enabled": bool(
                _param("ground_support/production_gate_enabled", False)
            ),
            "unknown_support_hold_enabled": bool(
                _param("ground_support/unknown_support_hold_enabled", True)
            ),
            "negative_obstacle_replan_enabled": bool(
                _param("ground_support/negative_obstacle_replan_enabled", True)
            ),
            # 有效速度域由可替换停车表最后一档决定，避免扩速时双参数漂移。
            "validated_max_speed": float(stop_distance_table[-1][0]),
            "stop_distance_table": stop_distance_table,
            "sweep_half_width": float(
                _param("ground_support/sweep_half_width", 0.435393)
            ),
            "foot_band_center_offset": float(
                _param("ground_support/foot_band_center_offset", 0.14)
            ),
            "foot_band_half_width": float(
                _param("ground_support/foot_band_half_width", 0.10)
            ),
            "minimum_foot_band_coverage": float(
                _param("ground_support/minimum_foot_band_coverage", 0.60)
            ),
            "maximum_longitudinal_unknown_gap": float(
                _param("ground_support/maximum_longitudinal_unknown_gap", 0.10)
            ),
            "minimum_longitudinal_support_ratio": float(
                _param("ground_support/minimum_longitudinal_support_ratio", 0.75)
            ),
            "minimum_lookahead": float(
                _param("ground_support/minimum_lookahead", 0.50)
            ),
            "maximum_lookahead": float(
                _param("ground_support/maximum_lookahead", 0.80)
            ),
            "footprint_front_reach": float(
                _param("ground_support/footprint_front_reach", 0.30)
            ),
            "support_band": float(_param("ground_support/support_band", 0.08)),
            "minimum_drop": float(_param("ground_support/minimum_drop", 0.30)),
            "minimum_cell_hits": int(
                _param("ground_support/minimum_cell_hits", 2)
            ),
            "required_consecutive_frames": int(
                _param("ground_support/required_consecutive_frames", 2)
            ),
            "minimum_alert_cells": int(
                _param("ground_support/minimum_alert_cells", 3)
            ),
            "minimum_frame_points": int(
                _param("ground_support/minimum_frame_points", 50)
            ),
            "local_support_evidence_radius": float(
                _param("ground_support/local_support_evidence_radius", 3.0)
            ),
            "assessment_period": max(
                0.005,
                float(_param("ground_support/assessment_period", 0.02)),
            ),
            "drop_evidence_max_age": float(
                _param("ground_support/drop_evidence_max_age", 0.05)
            ),
            "decision_period": float(
                _param("ground_support/decision_period", 0.10)
            ),
            "rl_receive_delay": float(
                _param("ground_support/rl_receive_delay", 0.009)
            ),
            "required_margin": float(
                _param("ground_support/required_margin", 0.02)
            ),
            "release_hysteresis": float(
                _param("ground_support/release_hysteresis", 0.10)
            ),
            "release_supported_frames": int(
                _param("ground_support/release_supported_frames", 2)
            ),
            "observation_hold_timeout": float(
                _param("ground_support/observation_hold_timeout", 2.0)
            ),
            "safety_stop_timeout": float(
                _param("ground_support/safety_stop_timeout", 3.0)
            ),
            "stop_linear_threshold": float(
                _param("ground_support/stop_linear_threshold", 0.02)
            ),
            "stop_angular_threshold": float(
                _param("ground_support/stop_angular_threshold", 0.05)
            ),
            "stop_confirmation_samples": int(
                _param("ground_support/stop_confirmation_samples", 3)
            ),
            "maximum_same_phase_replans": max(
                0,
                int(_param("ground_support/maximum_same_phase_replans", 2)),
            ),
            "maximum_negative_obstacle_replans": max(
                0,
                int(_param("ground_support/maximum_negative_obstacle_replans", 3)),
            ),
            "base_frame": str(_param("ground_support/base_frame", "base")),
            "sensor_frame": str(
                _param("ground_support/sensor_frame", "laser_livox")
            ),
        }
        self.frontier_params = {
            "min_cluster_size": int(_param("frontier/min_cluster_size", 8)),
            "blacklist_ttl": float(_param("frontier/blacklist_ttl", 60.0)),
            "fail_limit": int(_param("frontier/fail_limit", 3)),
            "stuck_blacklist_radius": float(_param("frontier/stuck_blacklist_radius", 1.20)),
            "information_gain_weight": float(_param("frontier/information_gain_weight", 2.0)),
            "path_cost_weight": float(_param("frontier/path_cost_weight", 1.0)),
            "yaw_cost_weight": float(_param("frontier/yaw_cost_weight", 0.3)),
            "revisit_penalty_weight": float(_param("frontier/revisit_penalty_weight", 2.0)),
            "frontier_obstacle_clearance": float(_param("frontier/frontier_obstacle_clearance", 0.25)),
            "frontier_unknown_standoff": float(_param("frontier/frontier_unknown_standoff", 0.20)),
            "frontier_min_information_gain": float(_param("frontier/frontier_min_information_gain", 1.0)),
            "door_front_half_plane_penalty": float(_param("frontier/door_front_half_plane_penalty", 6.0)),
            "door_front_half_plane_radius": float(_param("frontier/door_front_half_plane_radius", 1.5)),
            "door_preferred_direction_bonus": float(_param("frontier/door_preferred_direction_bonus", 1.5)),
            "max_planning_candidates": int(_param("frontier/max_planning_candidates", 8)),
            "max_goal_candidates_per_frontier": int(
                _param("frontier/max_goal_candidates_per_frontier", 2)
            ),
            "max_fallback_planning_candidates": int(
                _param("frontier/max_fallback_planning_candidates", 8)
            ),
            "goal_tolerance": float(_param("controller/goal_tolerance", 0.35)),
        }
        self.frontier_switch_score_margin = float(_param("frontier/switch_score_margin", 2.0))
        self.frontier_current_region_radius = float(_param("frontier/current_region_radius", 1.2))
        self.exit_first_max_linear = _optional_positive_float(
            _param("controller/exit_first_max_linear", None),
            "controller/exit_first_max_linear",
        )
        self.controller_params = {
            "max_linear": float(_param("controller/max_linear", 0.60)),
            "max_lateral": float(_param("controller/max_lateral", 0.25)),
            "max_angular": float(_param("controller/max_angular", 1.2)),
            "min_linear_fraction": float(_param("controller/min_linear_fraction", 0.35)),
            "goal_tolerance": float(_param("controller/goal_tolerance", 0.35)),
            "waypoint_tolerance": float(_param("controller/waypoint_tolerance", 0.25)),
            "corner_tolerance": float(_param("controller/corner_tolerance", 0.15)),
            "pure_pursuit_lookahead": float(_param("controller/pure_pursuit_lookahead", 0.45)),
            "reverse_speed": float(_param("controller/reverse_speed", -0.15)),
            "recover_duration": float(_param("controller/recover_duration", 0.5)),
            "recover_turn_duration": float(_param("controller/recover_turn_duration", 1.0)),
            "recover_angular_speed": float(_param("controller/recover_angular_speed", 0.55)),
            "command_keepalive_period": float(
                _param("controller/command_keepalive_period", 0.1)
            ),
            "obstacle_stop_distance": float(_param("controller/obstacle_stop_distance", 0.50)),
            "obstacle_slow_distance": float(_param("controller/obstacle_slow_distance", 1.00)),
            "no_progress_timeout": float(_param("frontier/no_progress_timeout", 8.0)),
            "progress_distance": float(_param("controller/progress_distance", 0.25)),
        }
        # 任务层最小进度监督参数
        self.progress_monitor_window = float(_param("mission/progress_monitor_window", 5.0))
        self.progress_monitor_repeat = max(2, int(_param("mission/progress_monitor_repeat", 3)))
        self.progress_monitor_min_displacement = float(_param("mission/progress_monitor_min_displacement", 0.20))
        self.progress_monitor_cooldown = float(_param("mission/progress_monitor_cooldown", 10.0))
        self.detection_params = {
            "min_area": int(_param("detection/min_area", 35)),
            "min_circularity": float(_param("detection/min_circularity", 0.55)),
            "min_aspect": float(_param("detection/min_aspect", 0.65)),
            "max_aspect": float(_param("detection/max_aspect", 1.55)),
            "min_radius": float(_param("detection/min_radius", 0.10)),
            "max_radius": float(_param("detection/max_radius", 0.22)),
            "max_rmse": float(_param("detection/max_rmse", 0.035)),
            "ransac_iterations": int(_param("detection/ransac_iterations", 80)),
            "ransac_sample_size": int(_param("detection/ransac_sample_size", 4)),
            "min_depth": float(_param("detection/min_depth", 0.35)),
            "max_depth": float(_param("detection/max_depth", 8.0)),
        }
        self.depth_frame_is_optical = bool(_param("detection/depth_frame_is_optical", True))
        self.local_detection_params = {
            "max_distance": float(_param("local_semantic/max_door_distance", 3.0)),
            "fov_half_angle": float(_param("local_semantic/door_fov_half_angle", 0.85)),
            "evidence_radius": float(_param("local_semantic/door_evidence_radius", 0.75)),
            "min_evidence_points": int(_param("local_semantic/min_door_evidence_points", 6)),
            "min_evidence_height": float(_param("local_semantic/min_door_evidence_height", 0.15)),
            "max_evidence_height": float(_param("local_semantic/max_door_evidence_height", 1.80)),
            "use_truth_proximity_detector": bool(_param("local_semantic/use_truth_proximity_detector", False)),
        }
        self.tracker_params = {
            "association_radius": float(_param("tracker/association_radius", 0.45)),
            "confirmation_window": float(_param("tracker/confirmation_window", 10.0)),
            "min_hits": int(_param("tracker/min_hits", 4)),
            "max_position_std": float(_param("tracker/max_position_std", 0.25)),
            "min_confidence": float(_param("tracker/min_confidence", 0.75)),
            "max_average_rmse": float(_param("tracker/max_average_rmse", 0.04)),
            "nms_radius": float(_param("tracker/nms_radius", 0.9)),
            "stale_after": float(_param("tracker/stale_after", 30.0)),
        }
        self.floor_params = {
            "elevator_id": str(_param("floor/elevator_id", "elevator_main")),
            "lobby_xy": tuple(float(v) for v in _param("floor/elevator_lobby_xy", [1.0, 2.6])),
            "inside_xy": tuple(float(v) for v in _param("floor/elevator_inside_xy", [2.6, 2.6])),
            "service_timeout": float(_param("floor/service_timeout", 5.0)),
        }

    def _on_timer(self, _event) -> None:
        try:
            self.tick()
        except Exception as exc:
            rospy.logerr("danger_search 状态机异常：%s", exc)
            self._record_failure(f"状态机异常: {exc}")
            self.controller.stop()
            self._mission_success_eligible = False
            if self.state not in (WRITE_RESULT, FINISH, MISSION_FAILED):
                self._transition_state(
                    WRITE_RESULT,
                    reason_code="state_machine_exception",
                    call_site="MissionManager._on_timer",
                    diagnostic_context={"exception": repr(exc)},
                )

    def tick(self) -> None:
        self._publish_status()
        if self.state in (FINISH, MISSION_FAILED):
            self.controller.stop()
            outcome = "finished" if self.state == FINISH else "failed"
            self._request_shutdown(f"danger_search mission {outcome}")
            return
        if self._elapsed() >= self.max_runtime:
            rospy.logwarn("接近时间预算，写出当前检测结果")
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

        snapshot_factory = getattr(
            self.sensor_buffer,
            "ground_support_snapshot",
            self.sensor_buffer.snapshot,
        )
        snapshot = snapshot_factory()
        pose = odom_pose_xy_yaw(snapshot.odom_msg)
        xyz = odom_xyz(snapshot.odom_msg)
        if pose is None or xyz is None:
            self.controller.stop()
            return

        self._last_xyz = xyz
        self._update_pose_progress(pose)
        self._sync_ground_support_task_context()
        self._update_ground_support_odometry(snapshot)
        self._update_mapping_and_detection(snapshot, pose, xyz)
        if self._handle_ground_support_control(pose):
            return

        if self.current_task is not None:
            status = self.current_task.tick(self, pose)
            if status in (DONE, FAILED):
                completed_task_name = getattr(self.current_task, "name", type(self.current_task).__name__)
                completed_task_phase = getattr(self.current_task, "phase", None)
                if isinstance(self.current_task, OpenDoorAndPass):
                    if status == DONE:
                        self.opened_doors.add(self.current_task.door_id)
                        self._remember_completed_door(self.current_task, pose)

                        # 仅当主入口真正成功通过后才记录禁回半平面，避免失败时
                        # 把上一次其他门的过门方向错误地写成永久禁回区域。
                        if self.current_task.door_id == "main_entrance":
                            self.main_entrance_xy = self.last_door_xy
                            self.main_entrance_pass_vector = self.last_door_pass_vector
                    else:
                        self.failed_doors.add(self.current_task.door_id)
                self.active_door = None
                if status == FAILED:
                    rospy.logwarn("当前任务 %s 失败，放弃该门/任务", self.current_task.name)

                if isinstance(self.current_task, TakeElevator) and status == DONE:
                    self.current_floor = self.current_task.target_floor
                    self.controller.clear_path()
                    self.floor_start_time = get_time()
                    self._floor_timeout_logged = False

                self.current_task = None

                if self.state == CHANGE_FLOOR:
                    outcome = "completed" if status == DONE else "failed"
                    destination = "return_home" if self.return_after_floor_change else "explore_floor"
                    if status == FAILED:
                        self._record_failure("换层任务失败")
                        self._mission_success_eligible = False
                        next_state = WRITE_RESULT if self.return_after_floor_change else RETURN_HOME
                    else:
                        next_state = RETURN_HOME if self.return_after_floor_change else EXPLORE_FLOOR
                    self._transition_state(
                        next_state,
                        reason_code=f"floor_change_{outcome}_{destination}",
                        call_site="MissionManager.tick:floor_change_task",
                        diagnostic_context={
                            "current_task": completed_task_name,
                            "current_task_phase": completed_task_phase,
                        },
                    )
            return

        if self.state == INIT:
            if pose is None or xyz is None:
                return
            self.home_pose = pose
            self.current_floor = self.mapper.estimate_floor_from_z(xyz[2])
            self.floor_start_time = get_time()
            self._transition_state(
                WAIT_CONTROLLER_READY,
                reason_code="home_pose_recorded",
                call_site="MissionManager.tick:init",
            )
            rospy.loginfo("记录起点：x=%.2f y=%.2f floor=%d", pose[0], pose[1], self.current_floor)
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

    def _floor_scoped_updates_are_blocked(self, snapshot, estimated_floor: int) -> bool:
        """更新换层锁存，并返回当前 tick 是否应暂停楼层相关点云处理。"""
        current_floor = int(self.current_floor)
        current_task = getattr(self, "current_task", None)
        guard = getattr(self, "_floor_transition_guard", None)
        if guard is None and not (
            isinstance(current_task, TakeElevator)
            and int(estimated_floor) != current_floor
        ):
            return False

        odom_stamp = _message_stamp_seconds(getattr(snapshot, "odom_msg", None))
        if odom_stamp is not None and not math.isfinite(odom_stamp):
            odom_stamp = None
        now = float(self._elapsed())

        if guard is None:
            target_floor = getattr(current_task, "target_floor", getattr(self, "target_floor", None))
            guard = _FloorTransitionGuard(
                entered_at=now,
                enter_odom_stamp=odom_stamp,
                source_floor=current_floor,
                enter_estimated_floor=int(estimated_floor),
                target_floor=int(target_floor) if target_floor is not None else None,
                enter_phase=getattr(current_task, "phase", None),
                last_mismatch_odom_stamp=odom_stamp,
            )
            self._floor_transition_guard = guard
            support_gate = getattr(self, "_ground_support_gate", None)
            invalidate_support = getattr(support_gate, "invalidate_assessment", None)
            if callable(invalidate_support):
                invalidate_support()
            self._log_structured(
                {
                    "event": "floor_transition_ambiguous_enter",
                    "enter_time": round(now, 3),
                    "enter_reason": "take_elevator_floor_mismatch",
                    "source_floor": guard.source_floor,
                    "current_floor": current_floor,
                    "mapper_estimated_floor": int(estimated_floor),
                    "target_floor": guard.target_floor,
                    "current_task": current_task.name,
                    "current_task_phase": guard.enter_phase,
                    "odom_stamp": odom_stamp,
                    "floor_transition_ambiguous": True,
                }
            )

        if not isinstance(current_task, TakeElevator) and guard.task_exit_at is None:
            guard.task_exit_at = now

        if int(estimated_floor) != current_floor:
            guard.alignment_floor = None
            guard.alignment_sample_count = 0
            guard.last_alignment_odom_stamp = None
            if odom_stamp is not None and (
                guard.last_mismatch_odom_stamp is None
                or odom_stamp > guard.last_mismatch_odom_stamp
            ):
                guard.last_mismatch_odom_stamp = odom_stamp
            return True

        freshness_floor = guard.last_mismatch_odom_stamp
        if freshness_floor is None:
            freshness_floor = guard.enter_odom_stamp
        odom_is_fresh = (
            odom_stamp is not None
            and (freshness_floor is None or odom_stamp > freshness_floor)
        )
        if guard.alignment_floor != current_floor:
            guard.alignment_floor = current_floor
            guard.alignment_sample_count = 0
            guard.last_alignment_odom_stamp = None
        if odom_is_fresh and (
            guard.last_alignment_odom_stamp is None
            or odom_stamp > guard.last_alignment_odom_stamp
        ):
            guard.alignment_sample_count += 1
            guard.last_alignment_odom_stamp = odom_stamp

        if guard.alignment_sample_count < FLOOR_ALIGNMENT_REQUIRED_ODOM_SAMPLES:
            return True

        self._log_structured(
            {
                "event": "floor_transition_ambiguous_exit",
                "enter_time": round(guard.entered_at, 3),
                "exit_time": round(now, 3),
                "exit_reason": "fresh_odom_floor_alignment_confirmed",
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
        self._floor_transition_guard = None
        return False

    def _record_skipped_floor_scoped_update(
        self,
        *,
        cloud_stamp: float | None,
        estimated_floor: int,
    ) -> None:
        guard = getattr(self, "_floor_transition_guard", None)
        if guard is None:
            return
        now = float(self._elapsed())
        guard.skipped_cloud_count += 1
        if guard.first_skipped_at is None:
            guard.first_skipped_at = now
            guard.first_skipped_cloud_stamp = cloud_stamp
        guard.last_skipped_at = now
        guard.last_skipped_cloud_stamp = cloud_stamp

        current_task = getattr(self, "current_task", None)
        phase = getattr(current_task, "phase", None)
        log_key = (int(self.current_floor), int(estimated_floor), phase)
        if log_key == guard.last_skip_log_key:
            return
        guard.last_skip_log_key = log_key
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
        guard.timeout_reported = True
        self._record_failure("楼层过渡对齐超时")
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
            }
        )

    def _update_mapping_and_detection(self, snapshot, pose, xyz) -> None:
        cloud_stamp = _message_stamp_seconds(snapshot.cloud_msg)
        cloud_is_new = cloud_stamp is None or cloud_stamp != getattr(self, "_last_cloud_stamp", None)
        estimated_floor = self.mapper.estimate_floor_from_z(xyz[2])
        floor_scoped_updates_blocked = self._floor_scoped_updates_are_blocked(
            snapshot,
            estimated_floor,
        )
        if estimated_floor == self.current_floor:
            self._last_floor_alignment_mismatch_key = None
        if cloud_is_new:
            if floor_scoped_updates_blocked:
                # 被门控的 cloud 必须标记为已消费，避免对齐后重放旧帧。
                self._last_cloud_stamp = cloud_stamp
                self._record_skipped_floor_scoped_update(
                    cloud_stamp=cloud_stamp,
                    estimated_floor=estimated_floor,
                )
            else:
                points = self.sensor_buffer.read_cloud_xyz(
                    snapshot.cloud_msg,
                    max_points=self.cloud_max_points,
                    stride=self.cloud_stride,
                )
                self._latest_cloud_points = points
                self._last_cloud_stamp = cloud_stamp
                self.mapper.update_from_cloud(
                    floor_index=self.current_floor,
                    robot_xyz=xyz,
                    points=points,
                )
                if getattr(self, "_ground_support_gate", None) is not None:
                    self._reapply_ground_support_black_marks(self.current_floor)
                if estimated_floor != self.current_floor:
                    current_task = getattr(self, "current_task", None)
                    mismatch_key = (
                        int(self.current_floor),
                        int(estimated_floor),
                        getattr(current_task, "phase", None),
                    )
                    if mismatch_key != getattr(self, "_last_floor_alignment_mismatch_key", None):
                        self._last_floor_alignment_mismatch_key = mismatch_key
                        self._log_structured(
                            {
                                "event": "floor_alignment_mismatch",
                                "elapsed": round(float(self._elapsed()), 3),
                                "odom_xyz": [round(float(value), 3) for value in xyz],
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
                set_obstacle_distance = getattr(self.controller, "set_nearest_obstacle_distance", None)
                if callable(set_obstacle_distance):
                    set_obstacle_distance(
                        nearest_forward_obstacle_distance(
                            points,
                            pose,
                            floor_index=self.current_floor,
                            mapper=self.mapper,
                        )
                    )

        self._report_floor_transition_alignment_timeout(estimated_floor)

        bundle = snapshot.image_bundle
        if bundle is None:
            return
        stamp = bundle.rgb_msg.header.stamp.to_sec() if hasattr(bundle.rgb_msg.header.stamp, "to_sec") else None
        if stamp is not None and self._last_image_stamp == stamp:
            return
        self._last_image_stamp = stamp
        camera_frame = bundle.rgb_msg.header.frame_id
        camera_to_world = self.sensor_buffer.camera_to_target_matrix(camera_frame, bundle.rgb_msg.header.stamp)
        if camera_to_world is not None and self.depth_frame_is_optical:
            camera_to_world = camera_to_world @ _ros_link_from_optical_matrix()
        candidates = self.detector.detect_messages(
            bundle.rgb_msg,
            bundle.depth_msg,
            bundle.camera_info_msg,
            camera_to_world,
            require_world_transform=True,
        )
        if candidates:
            self.tracker.update(candidates, now=get_time())

    def _update_ground_support_odometry(self, snapshot) -> None:
        gate = getattr(self, "_ground_support_gate", None)
        if gate is None:
            return
        linear_speed = odom_planar_speed(snapshot.odom_msg)
        angular_speed = odom_angular_speed(snapshot.odom_msg)
        gate.observe_odometry(
            _message_stamp_seconds(snapshot.odom_msg),
            linear_speed=linear_speed if linear_speed is not None else 0.0,
            angular_speed=angular_speed if angular_speed is not None else 0.0,
            wall_time=time.monotonic(),
        )

    def _on_ground_support_timer(self, _event) -> None:
        """只在安全门开启时及时消费新点云，不改变 10 Hz 任务状态机节拍。"""
        gate = getattr(self, "_ground_support_gate", None)
        grid = getattr(self, "_ground_support_grid", None)
        if gate is None or grid is None:
            return
        snapshot = self.sensor_buffer.snapshot()
        cloud_stamp = _message_stamp_seconds(snapshot.cloud_msg)
        if cloud_stamp is None or cloud_stamp <= (
            getattr(self, "_last_ground_support_cloud_stamp", None) or -math.inf
        ):
            return
        # 支撑层有独立消费游标。即使本帧因换层歧义或 TF 缺失跳过，也不得重放旧帧。
        self._last_ground_support_cloud_stamp = cloud_stamp
        pose = odom_pose_xy_yaw(snapshot.odom_msg)
        xyz = odom_xyz(snapshot.odom_msg)
        if pose is None or xyz is None:
            return
        estimated_floor = self.mapper.estimate_floor_from_z(xyz[2])
        current_task = getattr(self, "current_task", None)
        transition_ambiguous = (
            getattr(self, "_floor_transition_guard", None) is not None
            or (
                isinstance(current_task, TakeElevator)
                and int(estimated_floor) != int(self.current_floor)
            )
        )
        if transition_ambiguous:
            gate.invalidate_assessment()
            return
        points = self.sensor_buffer.read_cloud_xyz(
            snapshot.cloud_msg,
            max_points=self.cloud_max_points,
            stride=self.cloud_stride,
        )
        self._update_ground_support_assessment(
            snapshot,
            pose,
            xyz,
            points,
            cloud_stamp,
        )

    def _update_ground_support_assessment(
        self,
        snapshot,
        pose: tuple[float, float, float],
        xyz: tuple[float, float, float],
        points,
        cloud_stamp: float | None,
    ) -> None:
        gate = getattr(self, "_ground_support_gate", None)
        grid = getattr(self, "_ground_support_grid", None)
        if gate is None or grid is None or cloud_stamp is None:
            return
        if getattr(self, "_ground_support_sensor_mount", None) is None:
            transform = getattr(self.sensor_buffer, "frame_transform_matrix", None)
            if callable(transform):
                self._ground_support_sensor_mount = transform(
                    self.ground_support_params["base_frame"],
                    self.ground_support_params["sensor_frame"],
                    rospy.Time(0),
                    allow_latest=True,
                )
        mount = self._ground_support_sensor_mount
        body_rotation = odom_rotation_matrix(snapshot.odom_msg)
        if mount is None or body_rotation is None:
            return

        robot_position = np.asarray(xyz, dtype=float)
        sensor_origin = robot_position + body_rotation @ mount[:3, 3]
        sensor_rotation = body_rotation @ mount[:3, :3]
        followed_path = [
            np.asarray(point, dtype=float)
            for point in self.controller.followed_path_snapshot()
        ]
        emitted = getattr(self.controller, "last_command", (0.0, 0.0, 0.0))
        command_linear = math.hypot(float(emitted[0]), float(emitted[1]))
        result = grid.process_frame(
            stamp=cloud_stamp,
            floor=self.current_floor,
            robot_xy=np.asarray(pose[:2], dtype=float),
            robot_yaw=pose[2],
            sensor_origin=sensor_origin,
            sensor_rotation=sensor_rotation,
            points=np.asarray(points, dtype=float),
            path=followed_path,
            command_linear=command_linear,
            ambiguous=False,
            local_only=True,
        )
        if result.get("action") != "update":
            return
        payload = result.get("local_corridor_assessment")
        if not isinstance(payload, dict):
            return
        odom_stamp = _message_stamp_seconds(snapshot.odom_msg)
        ros_now = float(rospy.get_time())
        data_age = _ground_support_data_age(
            cloud_stamp=float(cloud_stamp),
            odom_stamp=odom_stamp,
            control_time=ros_now,
            decision_period=self.ground_support_params["decision_period"],
        )
        direction = payload.get("drop_direction_world")
        drop_point_world = None
        if isinstance(direction, (list, tuple)) and len(direction) >= 2:
            drop_point_world = (
                float(pose[0]) + float(direction[0]),
                float(pose[1]) + float(direction[1]),
            )
        gate.update_assessment(
            GroundSupportAssessment(
                stamp=float(cloud_stamp),
                received_wall_time=time.monotonic(),
                data_age=float(data_age),
                supported_distance=_optional_finite(payload.get("supported_distance")),
                support_boundary_detected=bool(
                    payload.get("support_boundary_detected", False)
                ),
                current_footprint_supported=bool(
                    payload.get("current_footprint_supported", False)
                ),
                observation_adequate=bool(
                    payload.get("observation_adequate", False)
                ),
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
            )
        )
        requested_snapshot = getattr(self.controller, "requested_command_snapshot", None)
        requested = (
            requested_snapshot()
            if callable(requested_snapshot)
            else (0.0, 0.0, 0.0)
        )
        if gate.evaluate_latest_drop(
            requested,
            pose_yaw=pose[2],
            wall_time=time.monotonic(),
            control_time=ros_now,
        ):
            self.controller.stop(reason="negative_obstacle")

    def _handle_ground_support_control(
        self,
        pose: tuple[float, float, float],
    ) -> bool:
        gate = getattr(self, "_ground_support_gate", None)
        if gate is None:
            return False
        check_control_timeout = getattr(gate, "check_control_timeout", None)
        if callable(check_control_timeout):
            check_control_timeout(
                float(get_time()),
                wall_time=time.monotonic(),
            )
        state = gate.state
        last_state = getattr(self, "_ground_support_last_state", GROUND_SUPPORT_CLEAR)
        if state != last_state:
            self._log_structured(
                {
                    "event": "ground_support_gate_transition",
                    "elapsed": round(float(self._elapsed()), 3),
                    "old_state": last_state,
                    "new_state": state,
                    "reason_code": gate.reason_code,
                    "gate": gate.snapshot(),
                }
            )
            self._ground_support_last_state = state
        if state == GROUND_SUPPORT_CLEAR:
            return False

        reset_progress = getattr(self.controller, "reset_progress_timer", None)
        if callable(reset_progress):
            reset_progress(pose)
        self.controller.stop(reason=gate.reason_code or "ground_support_hold")
        # 临时 hold 和停车过程都在任务 tick 之前截断，普通 STUCK/recover 无法接管。
        if state in {
            OBSERVATION_HOLD,
            STALE_DATA_HOLD,
            DROP_STOPPING,
            SPEED_OUT_OF_ENVELOPE_STOPPING,
        }:
            return True

        reason = gate.reason_code or "ground_support_safety_failure"
        if (
            state == SAFETY_BLOCKED
            and reason == "negative_obstacle"
            and self._resume_after_negative_obstacle(pose, gate)
        ):
            return True
        if state in {SAFETY_BLOCKED, SAFETY_STOP_TIMEOUT} or (
            state == SUPPORT_PATH_UNAVAILABLE
            and reason == "support_speed_out_of_envelope"
        ):
            self._record_failure(reason)
            self._mission_success_eligible = False
            if self.state not in (WRITE_RESULT, FINISH, MISSION_FAILED):
                self._transition_state(
                    WRITE_RESULT,
                    reason_code=reason,
                    call_site="MissionManager._handle_ground_support_control",
                )
            return True

        if state == SUPPORT_PATH_UNAVAILABLE:
            interrupted_task = self.current_task
            task_phase = getattr(interrupted_task, "phase", None)
            if interrupted_task is not None:
                retry_key = ("task", id(interrupted_task), task_phase)
            elif self._last_frontier_target is not None:
                retry_key = (
                    "frontier",
                    round(float(self._last_frontier_target[0]), 3),
                    round(float(self._last_frontier_target[1]), 3),
                )
            else:
                retry_key = ("state", self.state)
            if retry_key == getattr(self, "_ground_support_path_retry_key", None):
                self._ground_support_path_retry_count = (
                    getattr(self, "_ground_support_path_retry_count", 0) + 1
                )
            else:
                self._ground_support_path_retry_key = retry_key
                self._ground_support_path_retry_count = 1

            self.controller.clear_path(reason=reason)
            maximum_replans = max(
                0,
                int(
                    getattr(self, "ground_support_params", {}).get(
                        "maximum_same_phase_replans",
                        2,
                    )
                ),
            )
            log_structured = getattr(self, "_log_structured", None)
            if callable(log_structured):
                log_structured(
                    {
                        "event": "ground_support_path_replan",
                        "reason_code": reason,
                        "task": getattr(interrupted_task, "name", None),
                        "task_phase": task_phase,
                        "active_door": self.active_door,
                        "attempt": self._ground_support_path_retry_count,
                        "maximum_same_phase_replans": maximum_replans,
                        "parent_task_preserved": interrupted_task is not None,
                    }
                )
            if self._ground_support_path_retry_count > maximum_replans:
                exhausted_reason = "ground_support_path_replan_exhausted"
                self._record_failure(exhausted_reason)
                self._mission_success_eligible = False
                if self.state not in (WRITE_RESULT, FINISH, MISSION_FAILED):
                    self._transition_state(
                        WRITE_RESULT,
                        reason_code=exhausted_reason,
                        call_site=(
                            "MissionManager._handle_ground_support_control:"
                            "path_replan_exhausted"
                        ),
                        diagnostic_context={
                            "task": getattr(interrupted_task, "name", None),
                            "task_phase": task_phase,
                            "active_door": self.active_door,
                            "attempt": self._ground_support_path_retry_count,
                        },
                    )
                return True
            gate.reset_path_unavailable()
            self._ground_support_last_state = gate.state
            return True
        return True

    def _resume_after_negative_obstacle(
        self,
        pose: tuple[float, float, float],
        gate: GroundSupportSafetyGate,
    ) -> bool:
        """把已确认空带写入现有禁行栅格，再让当前楼层任务重新规划。"""
        params = getattr(self, "ground_support_params", {})
        if not bool(params.get("negative_obstacle_replan_enabled", True)):
            return False
        if self.current_task is not None or self.state not in {EXPLORE_FLOOR, RETURN_HOME}:
            return False
        point = getattr(gate, "latched_drop_point_world", None)
        if point is None or not all(math.isfinite(float(value)) for value in point):
            return False
        maximum = max(0, int(params.get("maximum_negative_obstacle_replans", 3)))
        count = int(getattr(self, "_ground_support_negative_replan_count", 0))
        if count >= maximum:
            return False

        radius = max(
            float(self.map_params["resolution"]),
            float(params["sweep_half_width"]),
        )
        grid = self.mapper.floor(self.current_floor)
        if grid.world_to_grid(float(point[0]), float(point[1])) is None:
            return False

        self.mapper.mark_black(
            self.current_floor,
            float(point[0]),
            float(point[1]),
            radius,
        )
        marks = getattr(self, "_ground_support_negative_obstacle_marks", None)
        if marks is None:
            marks = []
            self._ground_support_negative_obstacle_marks = marks
        marks.append((self.current_floor, float(point[0]), float(point[1]), radius))
        self.controller.clear_path(reason="negative_obstacle_replan")
        self.current_frontier = None
        self._last_frontier_target = None
        self._last_frontier_score = None
        self._last_frontier_used_fallback = False
        if not gate.reset_negative_obstacle_after_replan():
            return False

        self._ground_support_negative_replan_count = count + 1
        self._ground_support_last_state = gate.state
        self._log_structured(
            {
                "event": "negative_obstacle_local_replan",
                "elapsed": round(float(self._elapsed()), 3),
                "floor": self.current_floor,
                "mission_state": self.state,
                "drop_point_world": [float(point[0]), float(point[1])],
                "black_radius": radius,
                "attempt": self._ground_support_negative_replan_count,
                "maximum_attempts": maximum,
                "ordinary_recover_called": False,
            }
        )
        return True

    def _reapply_ground_support_black_marks(self, floor_index: int) -> None:
        """仅在生产安全门登记过空带后，防止建图把局部禁行区重新清空。"""
        for floor, x, y, radius in getattr(
            self,
            "_ground_support_negative_obstacle_marks",
            (),
        ):
            if floor == floor_index:
                self.mapper.mark_black(floor, x, y, radius)

    def _sync_ground_support_task_context(self) -> None:
        """仅为受控电梯门槛跨越关闭 UNKNOWN hold，显式 DROP 仍由安全门处理。"""
        gate = getattr(self, "_ground_support_gate", None)
        setter = getattr(gate, "set_unknown_support_hold_enabled", None)
        if callable(setter):
            configured = bool(
                getattr(self, "ground_support_params", {}).get(
                    "unknown_support_hold_enabled", True
                )
            )
            setter(
                _ground_support_unknown_hold_enabled(
                    self.current_task,
                    configured=configured,
                )
            )

    # ------------------------------------------------------------------
    # 方案 C：全局时间预算与楼层/返航决策层
    # ------------------------------------------------------------------

    def _record_progress_sample(self, grid) -> None:
        """记录当前楼层探索状态样本，用于评估近期收益。"""
        if not hasattr(self, "_progress_history"):
            self._progress_history = []
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
        self._progress_history.append({
            "time": now,
            "known_cells": known,
            "free_or_visited_cells": free_or_visited,
            "confirmed_dangers": confirmed_dangers,
        })
        # 只保留最近两个窗口的数据，避免无限增长
        window = getattr(self, "exploration_progress_window", 15.0)
        floor_timeout = getattr(self, "floor_timeout", 180.0)
        cutoff = now - max(window, floor_timeout) * 2
        while self._progress_history and self._progress_history[0]["time"] < cutoff:
            self._progress_history.pop(0)

    def _recent_progress(self, window_seconds: float) -> dict[str, float]:
        """计算最近 window_seconds 内的探索增量（不用 coverage_ratio）。"""
        now = get_time()
        cutoff = now - window_seconds
        samples = [s for s in self._progress_history if s["time"] >= cutoff]
        if len(samples) < 2:
            return {"known_cells_growth": 0.0, "free_or_visited_growth": 0.0, "danger_growth": 0.0}
        first = samples[0]
        last = samples[-1]
        dt = max(1e-6, float(last["time"]) - float(first["time"]))
        return {
            "known_cells_growth": (int(last["known_cells"]) - int(first["known_cells"])) / dt,
            "free_or_visited_growth": (int(last["free_or_visited_cells"]) - int(first["free_or_visited_cells"])) / dt,
            "danger_growth": (int(last["confirmed_dangers"]) - int(first["confirmed_dangers"])) / dt,
        }

    def _estimate_return_time(self, pose: tuple[float, float, float]) -> float:
        """从当前位置返航到起点的保守时间估计（单位：秒）。

        优先使用 A* 可达路径长度除以保守速度；若不可达，使用欧氏距离。
        安全余量由配置参数 return_safety_buffer 和 return_speed_estimate 控制，
        本函数不再叠加隐藏的乘数或常数，便于用真实返航数据校准。
        """
        home_pose = getattr(self, "home_pose", None)
        if home_pose is None:
            return float(getattr(self, "max_runtime", 590.0))
        grid = self.mapper.floor(self.current_floor)
        result = astar_plan(grid, (pose[0], pose[1]), (home_pose[0], home_pose[1]))
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

    def _decide_mission_action(
        self,
        pose: tuple[float, float, float],
        grid,
        frontiers: list[Frontier],
        floor_elapsed: float,
        mission_elapsed: float,
    ) -> tuple[str, str]:
        """根据全局时间预算决定下一步行动。

        返回 (action, reason_code)：
        - action: "CONTINUE" | "FINISH_FLOOR" | "RETURN_HOME"
        - reason_code: 可解释的状态切换原因
        """
        max_runtime = float(getattr(self, "max_runtime", 590.0))
        return_safety_buffer = float(getattr(self, "return_safety_buffer", 30.0))
        floor_timeout = float(getattr(self, "floor_timeout", 180.0))
        floor_finish_time_ratio = float(getattr(self, "floor_finish_time_ratio", 0.75))
        exploration_progress_window = float(getattr(self, "exploration_progress_window", 15.0))
        min_exploration_progress_cells = int(getattr(self, "min_exploration_progress_cells", 30))
        min_remaining_frontiers = int(getattr(self, "min_remaining_frontiers", 2))
        max_floor = int(getattr(self, "max_floor", 1))
        current_floor = int(getattr(self, "current_floor", 0))
        last_floor_uses_global_budget = bool(
            getattr(self, "last_floor_uses_global_budget", True)
        )
        has_next_floor = current_floor + 1 < max_floor

        # 起点尚未记录时不做返航决策（通常在 INIT 后才有 home_pose）
        if getattr(self, "home_pose", None) is None:
            return "CONTINUE", "home_pose_not_recorded"

        return_time = self._estimate_return_time(pose)
        remaining = max_runtime - mission_elapsed

        # 1. 总时间不足以完成返航：必须立即返航
        if mission_elapsed + return_time + return_safety_buffer >= max_runtime:
            return "RETURN_HOME", "mission_insufficient_time_for_return"

        # 2. 非最后一层的硬上限用于给后续楼层留出时间。最后一层没有
        # 换层配额需要保护，由全局返航边界决定何时结束，避免在仍有
        # 可达前沿时浪费比赛总时间。
        if floor_elapsed >= floor_timeout:
            if has_next_floor:
                change_time = self._estimate_floor_change_time(current_floor + 1, pose)
                if change_time + return_time + return_safety_buffer < remaining:
                    return "FINISH_FLOOR", "floor_timeout_change_floor"
                return "RETURN_HOME", "floor_timeout_return_home"
            if not last_floor_uses_global_budget:
                return "RETURN_HOME", "floor_timeout_return_home"

        # 3. 楼层软超时：检查是否仍有探索收益
        if floor_elapsed >= floor_timeout * floor_finish_time_ratio:
            progress = self._recent_progress(exploration_progress_window)
            known_growth = progress["known_cells_growth"] * exploration_progress_window
            has_progress = known_growth >= min_exploration_progress_cells
            has_frontiers = len(frontiers) >= min_remaining_frontiers

            if not has_progress and not has_frontiers:
                if has_next_floor:
                    change_time = self._estimate_floor_change_time(current_floor + 1, pose)
                    if change_time + return_time + return_safety_buffer < remaining:
                        return "FINISH_FLOOR", "floor_low_progress_change_floor"
                # 剩余时间还够继续探索其他楼层或返航，但当前层已无收益
                return "RETURN_HOME", "floor_low_progress_return_home"

        # 4. 继续当前层探索。最后一层越过楼层配额后仍由步骤 1 的
        # 全局返航预算约束，不会越过 max_runtime。
        if floor_elapsed >= floor_timeout and not has_next_floor:
            return "CONTINUE", "last_floor_uses_global_budget"
        return "CONTINUE", "budget_allows_continue"

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

    def _tick_explore(self, pose: tuple[float, float, float]) -> None:
        grid = self.mapper.floor(self.current_floor)
        now = get_time()
        progress_monitor = getattr(self, "_progress_monitor", None)
        if progress_monitor is not None:
            progress_monitor.record_pose(now, pose)
        if self.manual_mode:
            if self.current_task is None:
                self.controller.stop()
            return

        frontiers = self.explorer.extract_frontiers(grid)
        self._record_progress_sample(grid)

        floor_elapsed = now - self.floor_start_time
        mission_elapsed = self._elapsed()

        # 方案 C：全局时间预算决策
        if floor_elapsed >= self.min_floor_explore_time:
            action, reason_code = self._decide_mission_action(
                pose, grid, frontiers, floor_elapsed, mission_elapsed
            )
            if action != "CONTINUE":
                # 避免相邻 tick 反复切换导致状态循环
                last_decision = getattr(self, "_last_budget_decision", None)
                last_action, last_reason = last_decision or (None, None)
                last_decision_time = getattr(self, "_last_budget_decision_time", 0.0)
                now = get_time()
                if action != last_action or (now - last_decision_time) > 5.0:
                    self._last_budget_decision = (action, reason_code)
                    self._last_budget_decision_time = now
                    diagnostic = self._mission_diagnostic_snapshot()
                    diagnostic.update({
                        "event": "mission_budget_decision",
                        "action": action,
                        "reason_code": reason_code,
                        "floor_elapsed": round(floor_elapsed, 2),
                        "mission_elapsed": round(mission_elapsed, 2),
                        "max_runtime": getattr(self, "max_runtime", 590.0),
                        "floor_timeout": getattr(self, "floor_timeout", 180.0),
                        "return_time_estimate": round(self._estimate_return_time(pose), 2),
                        "frontier_count": len(frontiers),
                    })
                    self._log_structured(diagnostic)

                    if action == "RETURN_HOME":
                        self._transition_state(
                            RETURN_HOME,
                            reason_code=reason_code,
                            call_site="MissionManager._tick_explore:budget_decision",
                        )
                        return
                    elif action == "FINISH_FLOOR":
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
            cloud_points=self._latest_cloud_points,
            building_client=self.building_client,
            floor_height=self.mapper.floor_height,
            allow_elevator=False,
        )
        if self._should_open_door(candidate):
            assert candidate is not None
            self.controller.clear_path()
            self.current_frontier = None
            self._last_frontier_target = None
            self._last_frontier_score = None
            self._last_frontier_used_fallback = False
            self.active_door = candidate.door_id
            self.current_task = OpenDoorAndPass(
                candidate.door_id,
                pass_offset=self.door_pass_offset,
                commit_distance=self.door_commit_distance,
            )
            rospy.loginfo(
                "局部检测到可控动态门：door_id=%s distance=%.2f source=%s evidence=%d",
                candidate.door_id,
                candidate.distance,
                candidate.source,
                candidate.evidence_count,
            )
            return

        if not self.controller.has_path():
            scan_signature = self._frontier_scan_signature(grid, pose, frontiers)
            # 扫描门控：pose 栅格和 frontier 内容均未变时，避免逐 tick A* 空转。
            if self._no_frontier_scan_until > 0:
                now_gate = get_time()
                if now_gate < self._no_frontier_scan_until:
                    if scan_signature == self._last_no_frontier_signature:
                        self.controller.rotate_in_place(0.45)
                        return
                    # pose 或 frontier 内容变化，清除扫描状态并立即重试选择。
                    self._no_frontier_scan_until = 0.0
            self._last_no_frontier_signature = scan_signature

            self.controller.stop(reason="frontier_planning")
            selection_context = self._frontier_selection_context(pose)

            frontier, path = None, []
            if frontiers:
                frontier, path = self.explorer.select_frontier(
                    grid,
                    pose,
                    selection_context=selection_context,
                    frontiers=frontiers,
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
                    rospy.logwarn(
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
                door_id = self._try_select_door_approach(pose)
                if door_id is not None:
                    self._start_door_approach(door_id)
                    self._no_frontier_scan_until = 0.0
                    self._no_reachable_frontier_confirmations = 0
                    self._probe_attempt = 0
                    return

            if frontier is None or not path:
                # 检查本轮无目标是否仅因 inside_goal_tolerance 拒绝
                debug = self.explorer.last_selection_debug
                all_inside_tolerance = (
                    debug is not None
                    and debug.candidate_count > 0
                    and debug.inside_tolerance_count >= debug.candidate_count - debug.blacklisted_count
                    and debug.unreachable_count == 0
                    and debug.reachable_count == 0
                )
                if not frontiers and self._try_probe_unknown_space(pose):
                    self._no_reachable_frontier_confirmations = 0
                    return
                if self._tick_no_frontier_scan(
                    frontiers,
                    record_failure=not all_inside_tolerance,
                ):
                    self._no_reachable_frontier_confirmations += 1
                    if self._floor_completion_ready(grid, floor_elapsed):
                        self._finish_current_floor(
                            reason_code="floor_completion_ready",
                            call_site="MissionManager._tick_explore:completion_ready",
                        )
                    else:
                        rospy.loginfo(
                            "楼层完成条件未满足，继续扫描：elapsed=%.1f/%.1f max_home=%.2f/%.2f known_area=%.2f/%.2f coverage=%.2f/%.2f no_reachable=%d/%d corridor_progress=%.2f/%.2f",
                            floor_elapsed,
                            self.min_floor_explore_time,
                            self._max_home_distance,
                            self.min_floor_travel_distance,
                            grid.known_area(),
                            self.min_known_area,
                            grid.coverage_ratio(),
                            self.explore_coverage_ratio,
                            self._no_reachable_frontier_confirmations,
                            self.no_reachable_frontier_confirmations,
                            self._known_corridor_progress(grid),
                            self.min_corridor_progress_distance,
                        )
                return
            self._log_frontier_switch(frontier, path, pose)
            self.current_frontier = frontier
            self.controller.set_path(path)
            self._no_frontier_scan_until = 0.0
            self._no_reachable_frontier_confirmations = 0
            self._probe_attempt = 0
            rospy.loginfo("选择前沿：floor=%d score=%.2f target=(%.2f, %.2f)",
                          self.current_floor, frontier.score, frontier.centroid[0], frontier.centroid[1])

        status = self.controller.tick(pose)
        if status == STUCK:
            if self.current_frontier is not None:
                # STUCK 已经过进度超时确认，临时排除目标附近候选；只改规划黑名单，
                # 不把远处拐角写成永久地图障碍。
                self.explorer.record_stuck_failure(grid, self.current_frontier.centroid)
                distance = math.hypot(
                    pose[0] - self.current_frontier.centroid[0],
                    pose[1] - self.current_frontier.centroid[1],
                )
                if distance < 1.5:
                    self.mapper.mark_black(
                        self.current_floor,
                        self.current_frontier.centroid[0],
                        self.current_frontier.centroid[1],
                        0.6,
                    )
            self.controller.recover()
            self.current_frontier = None
            self._last_frontier_target = None
            self._last_frontier_score = None
            self._last_frontier_used_fallback = False
        elif status in (DONE, IDLE):
            self.current_frontier = None

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

        if self.last_door_xy is None or self.last_door_pass_vector is None:
            return context
        if now > getattr(self, "last_door_context_until", math.inf):
            return context
        timer_active = now <= self.last_door_transition_until
        progress_active = not self._post_door_context_satisfied(pose)
        if not timer_active and not progress_active:
            return context
        if timer_active and not progress_active and self.door_transition_hold_distance > 0.0:
            distance_to_door = math.hypot(pose[0] - self.last_door_xy[0], pose[1] - self.last_door_xy[1])
            if distance_to_door > self.door_transition_hold_distance:
                return context

        if context is None:
            context = FrontierSelectionContext(
                forbidden_half_plane_origin=self.last_door_xy,
                forbidden_half_plane_normal=self.last_door_pass_vector,
                forbidden_radius=self.explorer.door_front_half_plane_radius,
                skip_in_forbidden_half_plane=True,
                preferred_origin=self.last_door_xy,
                preferred_direction=self.last_door_pass_vector,
                temporary_revisit_penalty=self.explorer.door_front_half_plane_penalty,
                door_id=getattr(self, "last_door_id", None),
                temporary_context_expires_at=getattr(self, "last_door_context_until", None),
                context_time=now,
            )
        else:
            context = replace(
                context,
                forbidden_half_plane_origin=self.last_door_xy,
                forbidden_half_plane_normal=self.last_door_pass_vector,
                forbidden_radius=self.explorer.door_front_half_plane_radius,
                skip_in_forbidden_half_plane=True,
                preferred_origin=self.last_door_xy,
                preferred_direction=self.last_door_pass_vector,
                temporary_revisit_penalty=self.explorer.door_front_half_plane_penalty,
                door_id=getattr(self, "last_door_id", None),
                temporary_context_expires_at=getattr(self, "last_door_context_until", None),
                context_time=now,
            )
        return context

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
        if getattr(self, "_last_frontier_target", None) is None:
            return None
        return FrontierSelectionContext(
            current_target=self._last_frontier_target,
            current_score=self._last_frontier_score,
            current_region_radius=self.frontier_current_region_radius,
            switch_score_margin=self.frontier_switch_score_margin,
        )

    def _post_door_context_satisfied(self, pose: tuple[float, float, float]) -> bool:
        if self.last_door_xy is None or self.last_door_pass_vector is None:
            return True
        progress = self._post_door_progress_distance(pose)
        known_area = self._post_door_known_area() if self.post_door_min_known_area > 0.0 else 0.0
        return progress >= self.post_door_min_distance and known_area >= self.post_door_min_known_area

    def _post_door_progress_distance(self, pose: tuple[float, float, float]) -> float:
        if self.last_door_xy is None or self.last_door_pass_vector is None:
            return 0.0
        dx = pose[0] - self.last_door_xy[0]
        dy = pose[1] - self.last_door_xy[1]
        progress = dx * self.last_door_pass_vector[0] + dy * self.last_door_pass_vector[1]
        return max(0.0, progress)

    def _post_door_known_area(self) -> float:
        if self.last_door_xy is None or self.last_door_pass_vector is None:
            return 0.0
        grid = self.mapper.floor(self.current_floor)

        # 只在以门为中心、半径为所需推进距离 + 2.0 的矩形框内统计
        radius_world = self.post_door_min_distance + 2.0
        door_grid = grid.world_to_grid(self.last_door_xy[0], self.last_door_xy[1])
        if door_grid is None:
            return 0.0
        cgx, cgy = door_grid
        radius_grid = int(radius_world / grid.resolution)

        min_gx = max(0, cgx - radius_grid)
        max_gx = min(grid.size_x, cgx + radius_grid + 1)
        min_gy = max(0, cgy - radius_grid)
        max_gy = min(grid.size_y, cgy + radius_grid + 1)

        known_count = 0
        for gy in range(min_gy, max_gy):
            for gx in range(min_gx, max_gx):
                if int(grid.data[gy, gx]) == -1:
                    continue
                wx, wy = grid.grid_to_world(gx, gy)
                side = (wx - self.last_door_xy[0]) * self.last_door_pass_vector[0] + (
                    wy - self.last_door_xy[1]
                ) * self.last_door_pass_vector[1]
                if side >= -0.05:
                    known_count += 1
        return float(known_count * grid.resolution * grid.resolution)

    def _log_frontier_switch(
        self,
        frontier: Frontier,
        path: list[tuple[float, float]],
        pose: tuple[float, float, float],
    ) -> None:
        debug = self.explorer.last_selection_debug
        previous_target = self._last_frontier_target
        previous_score = self._last_frontier_score
        previous_distance = (
            math.hypot(frontier.centroid[0] - previous_target[0], frontier.centroid[1] - previous_target[1])
            if previous_target is not None
            else 0.0
        )
        door_side = self._door_side_label(frontier.centroid)
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
        if rospy is not None:
            rospy.loginfo(
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
        self._last_frontier_target = frontier.centroid
        self._last_frontier_score = frontier.score
        self._last_frontier_used_fallback = used_fallback

    def _door_side_label(self, target_xy: tuple[float, float]) -> str:
        if self.last_door_xy is None or self.last_door_pass_vector is None:
            return "无门上下文"
        signed_distance = (target_xy[0] - self.last_door_xy[0]) * self.last_door_pass_vector[0] + (
            target_xy[1] - self.last_door_xy[1]
        ) * self.last_door_pass_vector[1]
        if signed_distance > 0.05:
            return "门后侧"
        if signed_distance < -0.05:
            return "门前侧"
        return "门中线附近"

    def clear_opened_door_area(self, door_id: str) -> None:
        door = self.building_client.doors.get(door_id)
        if door is None:
            return
        depth = max(self.door_clear_radius, self.mapper.obstacle_inflation_radius + self.mapper.resolution)
        self.mapper.clear_dynamic_door_strip(door.floor_index, door.x, door.y, door.yaw, door.width, depth)
        rospy.loginfo(
            "已清理开门后的动态障碍残留：door_id=%s floor=%d center=(%.2f, %.2f) yaw=%.2f width=%.2f depth=%.2f",
            door_id,
            door.floor_index,
            door.x,
            door.y,
            door.yaw,
            door.width,
            depth,
        )

    def _floor_completion_ready(self, grid, floor_elapsed: float) -> bool:
        return (
            floor_elapsed >= self.min_floor_explore_time
            and self._max_home_distance >= self.min_floor_travel_distance
            and grid.known_area() >= self.min_known_area
            and grid.coverage_ratio() >= self.explore_coverage_ratio
            and self._no_reachable_frontier_confirmations >= self.no_reachable_frontier_confirmations
            and self._known_corridor_progress(grid) >= self.min_corridor_progress_distance
        )

    def _known_corridor_progress(self, grid) -> float:
        if self.min_corridor_progress_distance <= 0.0:
            return 0.0
        extent_x, extent_y = grid.known_bbox_extent()
        return extent_y if abs(math.sin(self.probe_yaw)) >= abs(math.cos(self.probe_yaw)) else extent_x

    def _remember_completed_door(self, task: OpenDoorAndPass, pose: tuple[float, float, float]) -> None:
        if task.wait_point is None or task.pass_point is None:
            self._post_door_frontier_debug = None
            return
        center_xy = (
            0.5 * (task.wait_point[0] + task.pass_point[0]),
            0.5 * (task.wait_point[1] + task.pass_point[1]),
        )
        pass_vec_x = task.pass_point[0] - center_xy[0]
        pass_vec_y = task.pass_point[1] - center_xy[1]
        pass_norm = math.hypot(pass_vec_x, pass_vec_y)
        if pass_norm > 1e-6:
            self.last_door_pass_vector = (pass_vec_x / pass_norm, pass_vec_y / pass_norm)
        else:
            self.last_door_pass_vector = None
        self.last_door_id = task.door_id
        self.last_door_xy = center_xy
        self.last_door_completion_pose = pose
        now = get_time()
        self.last_door_transition_until = now + self.door_transition_duration
        context_duration = max(
            self.door_transition_duration,
            getattr(self, "post_door_context_max_duration", self.door_transition_duration),
        )
        self.last_door_context_until = now + context_duration
        self._post_door_frontier_debug = {
            "door_id": task.door_id,
            "wait_point": task.wait_point,
            "pass_point": task.pass_point,
            "commit_point": task.commit_point,
            "completion_pose": pose,
        }

    def _log_post_door_frontier_debug(
        self,
        frontier: Frontier | None,
        path: list[tuple[float, float]],
        pose: tuple[float, float, float],
    ) -> None:
        debug = self._post_door_frontier_debug
        if debug is None or rospy is None:
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
            if isinstance(wait_point, tuple) and isinstance(pass_point, tuple):
                side = self._classify_goal_side(wait_point, pass_point, frontier.centroid)
        rospy.loginfo(
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
        self._post_door_frontier_debug = None

    @staticmethod
    def _classify_goal_side(
        wait_point: tuple[float, float],
        pass_point: tuple[float, float],
        target_xy: tuple[float, float],
    ) -> str:
        center_x = 0.5 * (wait_point[0] + pass_point[0])
        center_y = 0.5 * (wait_point[1] + pass_point[1])
        dir_x = pass_point[0] - wait_point[0]
        dir_y = pass_point[1] - wait_point[1]
        norm = math.hypot(dir_x, dir_y)
        if norm <= 1e-6:
            return "无法判断"
        signed_distance = ((target_xy[0] - center_x) * dir_x + (target_xy[1] - center_y) * dir_y) / norm
        if signed_distance > 0.05:
            return "门后侧(通过点一侧)"
        if signed_distance < -0.05:
            return "门前侧(等待点一侧)"
        return "门中线附近"

    def _try_probe_unknown_space(self, pose: tuple[float, float, float]) -> bool:
        grid = self.mapper.floor(self.current_floor)
        base_pose = (pose[0], pose[1], self.probe_yaw)
        for _ in range(12):
            target = probe_target_for_attempt(base_pose, self.probe_distance, self._probe_attempt)
            self._probe_attempt += 1
            target_cell = grid.world_to_grid(target[0], target[1])
            if target_cell is None:
                continue
            cell_value = int(grid.data[target_cell[1], target_cell[0]])
            if cell_value in (80, 100):
                continue
            if segment_crosses_blocked_cell(grid, (pose[0], pose[1]), target):
                continue
            self.current_frontier = None
            self._last_frontier_target = None
            self._last_frontier_score = None
            self._last_frontier_used_fallback = False
            self.controller.set_path([(pose[0], pose[1]), target])
            rospy.loginfo("无前沿，执行短距离探测目标：target=(%.2f, %.2f)", target[0], target[1])
            return True
        return False

    def _tick_no_frontier_scan(
        self,
        frontiers: list[Frontier],
        *,
        record_failure: bool = True,
    ) -> bool:
        now = get_time()
        if self._no_frontier_scan_until <= 0.0:
            self._no_frontier_scan_until = now + self.no_frontier_scan_duration
            if frontiers:
                rospy.loginfo("当前无可达前沿，原地扫描 %.1f 秒后再判断", self.no_frontier_scan_duration)
            else:
                rospy.loginfo("当前无前沿，原地扫描 %.1f 秒后再判断", self.no_frontier_scan_duration)
        if now < self._no_frontier_scan_until:
            self.controller.rotate_in_place(0.45)
            return False
        self.controller.stop()
        self._no_frontier_scan_until = 0.0
        if frontiers:
            if record_failure:
                self._record_failure("无可达前沿")
            rospy.logwarn("扫描后仍无可达前沿，结束当前楼层")
        return True

    def _should_open_door(self, candidate: DoorCandidate | None) -> bool:
        if candidate is None:
            return False
        if self.state != EXPLORE_FLOOR:
            return False
        if self.current_task is not None:
            return False
        if candidate.kind != "dynamic_door":
            return False
        if candidate.door_id in self.opened_doors:
            return False
        if candidate.door_id in self.failed_doors:
            return False
        if candidate.door_id == self.active_door:
            return False
        if self.building_client.is_elevator_door(candidate.door_id):
            return False
        if candidate.distance >= self.local_detector.max_distance:
            return False
        if not candidate.visible:
            return False
        if not self.building_client.door_service_ready():
            self._record_failure("门服务不可用")
            return False
        return True

    def _unopened_controllable_doors(self) -> list[DoorInfo]:
        """返回当前楼层尚未打开且未失败的动态门（排除短暂黑名单）。"""
        now = get_time()
        result: list[DoorInfo] = []
        for door in self.building_client.controllable_doors(
            floor_index=self.current_floor, include_elevators=False
        ):
            if door.door_id in self.opened_doors or door.door_id in self.failed_doors:
                continue
            if self._door_approach_blacklist.get(door.door_id, 0.0) > now:
                continue
            result.append(door)
        return result

    def _try_select_door_approach(self, pose: tuple[float, float, float]) -> str | None:
        """当没有优质 frontier 时，选择最近的可到达未开门作为主动接近目标。"""
        unopened = self._unopened_controllable_doors()
        if not unopened:
            return None
        grid = self.mapper.floor(self.current_floor)
        robot_xy = (pose[0], pose[1])
        best_id: str | None = None
        best_length = math.inf
        now = get_time()
        for door in unopened:
            try:
                wait_point, _ = self.building_client.get_door_points(
                    door.door_id, robot_xy, offset=self.door_pass_offset
                )
            except ValueError:
                continue
            result = astar_plan(grid, robot_xy, wait_point)
            if result.reached:
                length = path_length(result.path)
                if length < best_length:
                    best_length = length
                    best_id = door.door_id
            elif result.unreachable:
                # 确实不可达，黑名单 20 秒避免反复尝试
                self._door_approach_blacklist[door.door_id] = now + 20.0
            # 预算耗尽：可能可达，不黑名单，留待下轮重试
        return best_id

    def _start_door_approach(self, door_id: str) -> None:
        """主动发起过门任务，直接尝试打开并穿过指定动态门。"""
        self.controller.clear_path()
        self.current_frontier = None
        self._last_frontier_target = None
        self._last_frontier_score = None
        self._last_frontier_used_fallback = False
        self.active_door = door_id
        self.current_task = OpenDoorAndPass(
            door_id,
            pass_offset=self.door_pass_offset,
            commit_distance=self.door_commit_distance,
        )
        if rospy is not None:
            rospy.loginfo("选择主动靠近并开启未探索门：door_id=%s", door_id)

    def _finish_current_floor(
        self,
        *,
        reason_code: str = "floor_finished",
        call_site: str = "MissionManager._finish_current_floor",
    ) -> None:
        no_frontier_count = self._no_reachable_frontier_confirmations
        self.controller.clear_path()
        self._no_frontier_scan_until = 0.0
        self._no_reachable_frontier_confirmations = 0
        self._last_no_frontier_signature = None

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
                rospy.loginfo(
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

    def _log_floor_budget_exhausted(self, floor_elapsed: float) -> None:
        """记录楼层配额到期后仍继续探索的原因。"""
        if getattr(self, "_floor_timeout_logged", False):
            return
        self._floor_timeout_logged = True
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
            self._record_failure("缺少返航起点")
            self._transition_state(
                WRITE_RESULT,
                reason_code="missing_home_pose",
                call_site="MissionManager._tick_return_home",
            )
            return
        if self.current_floor != 0 and self.max_floor > 1:
            self._begin_floor_change(
                0,
                return_after=True,
                reason_code="return_home_change_floor",
                call_site="MissionManager._tick_return_home",
            )
            return
        distance = math.hypot(self.home_pose[0] - pose[0], self.home_pose[1] - pose[1])
        if distance <= self.home_tolerance:
            self._return_home_success = True
            self._return_home_completed = True
            normal_completion = bool(getattr(self, "_mission_success_eligible", True))
            self._transition_state(
                WRITE_RESULT,
                reason_code=(
                    "return_home_completed"
                    if normal_completion
                    else "return_home_completed_after_failure"
                ),
                call_site="MissionManager._tick_return_home",
            )
            return
        if not self.controller.has_path():
            grid = self.mapper.floor(self.current_floor)
            result = astar_plan(grid, (pose[0], pose[1]), (self.home_pose[0], self.home_pose[1]))
            if result.reached:
                self.controller.set_path(result.path)
            elif result.unreachable:
                fallback_path = [(pose[0], pose[1]), (self.home_pose[0], self.home_pose[1])]
                if segment_crosses_blocked_cell(grid, fallback_path[0], fallback_path[-1]):
                    if rospy is not None:
                        rospy.logwarn("返航暂无可达路径，且直线返航会穿过已知障碍；等待地图更新后重试")
                    clear_path = getattr(self.controller, "clear_path", None)
                    if callable(clear_path):
                        clear_path()
                    return
                self.controller.set_path(fallback_path)
            else:
                # 预算耗尽：可能可达，不退化为直线返航，等待地图更新后重新规划。
                if rospy is not None:
                    rospy.logwarn("返航规划预算耗尽，等待地图更新后重试")
                clear_path = getattr(self.controller, "clear_path", None)
                if callable(clear_path):
                    clear_path()
                return
        status = self.controller.tick(pose)
        if status == STUCK:
            self.controller.recover()

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
            rospy.logwarn("门/电梯服务不可用，停止探索并写出已有结果")
            self._elevator_success = False
            self._record_failure("门/电梯服务不可用")
            self._transition_state(
                WRITE_RESULT,
                reason_code="floor_services_unavailable",
                call_site=call_site,
                diagnostic_context=diagnostic_context,
            )
            return
        if self._elevator_success is None:
            self._elevator_success = True
        self.target_floor = int(target_floor)
        self.return_after_floor_change = bool(return_after)
        self.controller.clear_path()
        self._transition_state(
            CHANGE_FLOOR,
            reason_code=reason_code,
            call_site=call_site,
            diagnostic_context=diagnostic_context,
        )
        self.current_task = TakeElevator(self.current_floor, target_floor, elevator_id=self.floor_params.get("elevator_id", "elevator_main"))
        rospy.loginfo("准备换层：%d -> %d", self.current_floor, self.target_floor)

    def _write_result(
        self,
        *,
        reason_code: str | None = None,
        call_site: str = "MissionManager._write_result",
    ) -> None:
        resolved_reason = reason_code or "result_write_requested"
        self._state_at_result_write = getattr(self, "state", None)
        self._result_write_reason = resolved_reason
        self._result_write_call_site = call_site
        self._normal_completion = bool(
            resolved_reason == "return_home_completed"
            and getattr(self, "_mission_success_eligible", True)
            and getattr(self, "_return_home_started", False)
            and getattr(self, "_return_home_completed", False)
            and getattr(self, "_return_home_success", False)
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
        path = self.result_writer.write(self._elapsed(), positions)
        self.controller.stop(repeat=5)
        self._finished = True
        self._write_mission_summary(path, len(positions))
        rospy.loginfo("已写出危险源检测结果：%s，数量=%d", path, len(positions))

    def _publish_status(self) -> None:
        if self._status_publisher is None or String is None:
            return
        now = get_time()
        if now - self._last_status_publish < self.status_publish_period:
            return
        self._last_status_publish = now

        grid = self.mapper.floor(self.current_floor)
        if self._map_publisher is not None:
            msg = OccupancyGrid()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "odom"
            msg.info.resolution = grid.resolution
            msg.info.width = grid.size_x
            msg.info.height = grid.size_y
            msg.info.origin.position.x = grid.origin_x
            msg.info.origin.position.y = grid.origin_y
            msg.info.origin.position.z = self.current_floor * self.mapper.floor_height
            msg.info.origin.orientation.w = 1.0
            msg.data = grid.data.flatten().tolist()
            self._map_publisher.publish(msg)

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
            "path_length": round(self.controller._last_goal_distance, 2) if self.controller.has_path() else 0.0,
            "is_stuck": self.controller._recover_until > now,
            "controller": controller_debug,
        }
        self._status_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))
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
            int(self.mapper.estimate_floor_from_z(last_xyz[2]))
            if last_xyz is not None
            else None
        )

        controller_snapshot = getattr(self.controller, "tracking_debug_snapshot", None)
        diagnostic["controller"] = controller_snapshot() if callable(controller_snapshot) else None

        mapper_snapshot = getattr(self.mapper, "latest_debug_snapshot", None)
        latest_mapper_debug = mapper_snapshot() if callable(mapper_snapshot) else None
        diagnostic["mapper"] = dict(vars(latest_mapper_debug)) if latest_mapper_debug is not None else None

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
        old_state = getattr(self, "state", None)
        self.state = new_state
        if new_state == RETURN_HOME:
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
            "no_reachable_frontier_confirmations": int(
                getattr(self, "_no_reachable_frontier_confirmations", 0)
            ),
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
            "return_home_started": bool(getattr(self, "_return_home_started", False)),
            "return_home_completed": bool(
                getattr(self, "_return_home_completed", False)
                or getattr(self, "_return_home_success", False)
            ),
            "mission_success_eligible": bool(
                getattr(self, "_mission_success_eligible", True)
            ),
            "terminal_outcome": getattr(self, "_terminal_outcome", "running"),
        }

    @staticmethod
    def _log_structured(payload: dict[str, object]) -> None:
        logger = getattr(rospy, "loginfo", None) if rospy is not None else None
        if callable(logger):
            logger("%s", json.dumps(payload, ensure_ascii=False, sort_keys=True))

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
        rospy.signal_shutdown(reason)

    def _write_mission_summary(self, result_path: Path, detected_count: int) -> None:
        if not self.mission_summary_file:
            return
        summary_path = Path(self.mission_summary_file)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        grid = self.mapper.floor(self.current_floor)
        payload = {
            "exploration_time": round(self._elapsed(), 2),
            "state": getattr(self, "_state_at_result_write", None) or self.state,
            "terminal_state": getattr(self, "_terminal_state", None),
            "terminal_outcome": getattr(self, "_terminal_outcome", "failed"),
            "terminal_reason": getattr(self, "_terminal_reason", None),
            "completed": bool(getattr(self, "_normal_completion", False)),
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
            "result_file": str(result_path),
        }
        summary_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def nearest_forward_obstacle_distance(
    points,
    pose: tuple[float, float, float],
    *,
    floor_index: int,
    mapper: MultiFloorGridMap,
    max_distance: float = 1.5,
) -> float | None:
    """从实体点云回波计算前向净距，避免把膨胀层重复当作墙体。

    前向障碍检测使用固定半宽的矩形走廊，而非随距离扩张的楔形。
    A1 trunk 半宽约 0.10m，固定半宽 0.25m 已包含机身宽度与安全余量，
    同时避免门口两侧门框、走廊侧墙被误判为正前方障碍导致压速停滞。
    """
    cos_yaw = math.cos(pose[2])
    sin_yaw = math.sin(pose[2])
    floor_base_z = floor_index * mapper.floor_height
    best = math.inf
    # 固定半宽：机身半宽 ~0.10m + 安全余量，与旧版 forward=0 处保持一致。
    corridor_half_width = 0.25
    for px, py, pz in points:
        z_rel = pz - floor_base_z
        if not mapper.obstacle_min_z <= z_rel <= mapper.obstacle_max_z:
            continue
        dx = px - pose[0]
        dy = py - pose[1]
        forward = dx * cos_yaw + dy * sin_yaw
        lateral = abs(-dx * sin_yaw + dy * cos_yaw)
        distance = math.hypot(dx, dy)
        if forward < 0.0 or lateral > corridor_half_width or distance > max_distance:
            continue
        best = min(best, distance)
    return None if not math.isfinite(best) else best


def _message_stamp_seconds(message) -> float | None:
    if message is None:
        return None
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    to_sec = getattr(stamp, "to_sec", None)
    return float(to_sec()) if callable(to_sec) else None


def _ground_support_unknown_hold_enabled(task, *, configured: bool = True) -> bool:
    """返回当前任务是否启用 UNKNOWN 临时停车；显式 DROP 不受此值影响。"""
    if not configured:
        return False
    return not (
        isinstance(task, TakeElevator)
        and getattr(task, "phase", None) in {"ENTER_CABIN", "EXIT_CABIN"}
    )


def _ground_support_data_age(
    *,
    cloud_stamp: float,
    odom_stamp: float | None,
    control_time: float,
    decision_period: float,
) -> float:
    """返回决策周期之外的额外陈旧量，并保留 cloud/odom 配准误差。"""
    alignment_age = (
        abs(float(odom_stamp) - float(cloud_stamp))
        if odom_stamp is not None
        else math.inf
    )
    excess_source_age = max(
        0.0,
        float(control_time) - float(cloud_stamp) - max(0.0, float(decision_period)),
    )
    return max(alignment_age, excess_source_age)


def _optional_positive_float(value, name: str) -> float | None:
    if value is None or value == "":
        return None
    parsed = float(value)
    if parsed == 0.0:
        return None
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{name} 必须为非负有限数")
    return parsed


def _parse_stop_distance_table(value) -> tuple[tuple[float, float], ...]:
    try:
        table = tuple((float(item[0]), float(item[1])) for item in value)
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError("ground_support/stop_distance_table 必须是 [[速度, 距离], ...]") from exc
    if not table:
        raise ValueError("ground_support/stop_distance_table 不能为空")
    return table


def _optional_finite(value) -> float | None:
    if value is None:
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def _param(path: str, default):
    node_name = rospy.get_name().rstrip("/")
    candidate_names = [
        f"{node_name}/{path}",
        "~" + path,
        "/" + path,
    ]
    for name in candidate_names:
        if rospy.has_param(name):
            return rospy.get_param(name)
    return default


def probe_target_for_attempt(
    base_pose: tuple[float, float, float],
    probe_distance: float,
    attempt: int,
) -> tuple[float, float]:
    """按固定走廊扫描序列生成短距离探测目标，避免无前沿时只在起点附近抖动。"""
    distance = max(float(probe_distance), 0.05)
    attempt_index = max(int(attempt), 0)
    lane_index = attempt_index % len(PROBE_LATERAL_OFFSETS)
    forward_step = attempt_index // len(PROBE_LATERAL_OFFSETS) + 1
    forward = (math.cos(base_pose[2]), math.sin(base_pose[2]))
    lateral = (math.cos(base_pose[2] + math.pi / 2.0), math.sin(base_pose[2] + math.pi / 2.0))
    lateral_offset = PROBE_LATERAL_OFFSETS[lane_index] * distance
    forward_distance = forward_step * distance
    return (
        base_pose[0] + forward_distance * forward[0] + lateral_offset * lateral[0],
        base_pose[1] + forward_distance * forward[1] + lateral_offset * lateral[1],
    )


def segment_crosses_blocked_cell(
    grid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
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
        if int(grid.data[gy, gx]) in (80, 100):
            return True
    return False


def _ros_link_from_optical_matrix() -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )
    return matrix


def main() -> int:
    if rospy is None:
        raise RuntimeError("当前环境没有 rospy，无法运行 danger_search_node")
    rospy.init_node("danger_search")
    manager = MissionManager()
    manager.start()
    rospy.spin()
    return 0

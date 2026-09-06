"""任务配置加载与公开接口校验。

本模块是 ROS 参数到任务配置的唯一边界。MissionManager 只接收已经校验的
MissionConfig，不直接访问参数服务器。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from types import MappingProxyType
from typing import Callable, Mapping

try:
    import rospy
except ImportError:  # pragma: no cover - 离线测试不要求 ROS
    rospy = None

from danger_search.building_feature_provider import BuildingFeatureProvider
from danger_search.interface_policy import (
    require_allowed_input_topic,
    require_allowed_output_topic,
    require_internal_input_topic,
    require_internal_output_topic,
    require_manual_control_topic,
)
from danger_search.landmark_drift_monitor import LandmarkDriftConfig
from danger_search.entrance_geometry_corrector import EntranceGeometryConfig
from danger_search.sparse_landmark_relocalizer import SparseRelocalizationConfig


ParameterGetter = Callable[[str, object], object]


def effective_max_runtime(
    configured_max_runtime: float,
    *,
    max_floor: int,
    runtime_by_floor: object,
) -> float:
    """按公开楼层数采用配置预算，专项配置可进一步缩短。"""
    configured = float(configured_max_runtime)
    if configured <= 0.0:
        raise ValueError("任务时间预算必须为正")
    limits: dict[int, float] = {}
    for item in runtime_by_floor if isinstance(runtime_by_floor, (list, tuple)) else ():
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError("mission/runtime_by_floor 每项必须是 [楼层数, 秒数]")
        floor_count = int(item[0])
        limit = float(item[1])
        if floor_count <= 0 or limit <= 0.0 or floor_count in limits:
            raise ValueError("mission/runtime_by_floor 包含非法或重复楼层数")
        limits[floor_count] = limit
    if int(max_floor) in limits:
        return min(configured, limits[int(max_floor)])
    return configured


def validate_summary_output_path(path: str | Path, public_result: Path) -> Path:
    """限制 mission summary 只能写入公开结果目录下的运行归档目录。"""
    summary_path = Path(path)
    if not summary_path.is_absolute():
        raise ValueError("mission summary 必须使用绝对路径")
    if summary_path.name != "mission_runtime_summary.json":
        raise ValueError("mission summary 文件名必须是 mission_runtime_summary.json")
    if summary_path.is_symlink():
        raise ValueError("mission summary 路径不能是符号链接")
    allowed_root = public_result.parent / "runs"
    try:
        summary_path.parent.resolve(strict=False).relative_to(
            allowed_root.resolve(strict=False)
        )
    except (OSError, ValueError) as exc:
        raise ValueError(
            f"mission summary 输出越出参赛方运行归档目录: {summary_path}"
        ) from exc
    return summary_path


def optional_positive_float(value, name: str) -> float | None:
    if value is None or value == "":
        return None
    parsed = float(value)
    if parsed == 0.0:
        return None
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{name} 必须为非负有限数")
    return parsed


def parse_stop_distance_table(value) -> tuple[tuple[float, float], ...]:
    try:
        table = tuple((float(item[0]), float(item[1])) for item in value)
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError(
            "ground_support/stop_distance_table 必须是 [[速度, 距离], ...]"
        ) from exc
    if not table:
        raise ValueError("ground_support/stop_distance_table 不能为空")
    return table


def ros_parameter(path: str, default):
    if rospy is None:
        raise RuntimeError("当前环境没有 rospy，无法读取任务配置")
    node_name = rospy.get_name().rstrip("/")
    for name in (f"{node_name}/{path}", "~" + path, "/" + path):
        if rospy.has_param(name):
            return rospy.get_param(name)
    return default


# 保持迁移前参数 schema 的局部命名，避免配置迁移改变任何默认值或校验语义。
_effective_max_runtime = effective_max_runtime
_validate_summary_output_path = validate_summary_output_path
_optional_positive_float = optional_positive_float
_parse_stop_distance_table = parse_stop_distance_table


@dataclass(frozen=True)
class MissionConfigSection:
    """一组职责相关、只读的已校验配置。"""

    values: Mapping[str, object]

    def __getitem__(self, name: str):
        return self.values[name]

    def get(self, name: str, default=None):
        return self.values.get(name, default)


@dataclass(frozen=True)
class MissionConfig:
    """一次任务启动时冻结的分组配置快照。"""

    values: Mapping[str, object]
    runtime: MissionConfigSection
    topics: MissionConfigSection
    localization: MissionConfigSection
    mapping: MissionConfigSection
    entry: MissionConfigSection
    elevator: MissionConfigSection
    exploration: MissionConfigSection
    return_home: MissionConfigSection
    output: MissionConfigSection

    def install(self, target: object) -> None:
        """临时安装旧扁平消费者；职责迁移完成后删除。"""
        for name, value in self.values.items():
            setattr(target, name, value)


_CONFIG_GROUP_FIELDS = {
    "runtime": frozenset(
        {
            "runtime_profile",
            "building_features",
            "startup_delay",
            "max_runtime",
            "max_floor",
            "tick_wall_stall_timeout",
            "manual_goal_enabled",
            "last_floor_uses_global_budget",
            "danger_approach_verification_enabled",
            "danger_approach_trigger_distance",
            "danger_approach_standoff_distance",
            "danger_approach_max_anchor_error",
            "danger_verification_timeout",
            "danger_verification_min_hits",
            "danger_inspection_retry_cooldown",
            "danger_inspection_timeout",
            "danger_verification_min_observation_span",
        }
    ),
    "topics": frozenset(
        {
            "cmd_vel_topic",
            "localization_odom_topic",
            "localization_correction_request_topic",
            "localization_correction_state_topic",
            "cloud_topic",
            "rgb_topic",
            "depth_topic",
            "camera_info_topic",
            "status_topic",
            "goal_topic",
            "map_topic",
            "path_raw_topic",
            "path_followed_topic",
            "local_target_topic",
        }
    ),
    "localization": frozenset(
        {
            "camera_mount_xyz",
            "camera_mount_rpy",
            "localization_stale_minimum",
            "localization_unavailable_timeout",
            "localization_stance_health_required",
            "localization_vertical_reference",
            "global_frame_id",
            "landmark_drift_config",
            "sparse_relocalization_config",
            "entrance_geometry_config",
        }
    ),
    "mapping": frozenset(
        {
            "map_params",
            "ground_support_params",
            "detection_params",
            "tracker_params",
            "local_detection_params",
            "rgbd_door_detection_params",
            "rgbd_door_detection_enabled",
            "cloud_max_points",
            "cloud_stride",
            "depth_frame_is_optical",
        }
    ),
    "entry": frozenset(
        {
            "door_clear_radius",
            "door_clearance_margin",
            "door_commit_distance",
            "door_pass_max_linear",
            "door_pass_offset",
            "door_wait_offset",
            "door_refresh_max_center_shift",
            "door_refresh_timeout",
            "door_transition_duration",
            "door_transition_hold_distance",
            "initial_entry_corridor_half_width",
            "initial_entry_geometry_tolerance",
            "initial_entry_max_distance",
            "initial_entry_opening_confirmations",
            "initial_entry_opening_max_distance",
            "initial_entry_center_tolerance",
            "initial_entry_wait_timeout",
            "main_entrance_clearance_max_linear",
            "post_door_context_max_duration",
            "post_door_min_distance",
            "post_door_min_known_area",
            "probe_distance",
            "probe_yaw",
        }
    ),
    "elevator": frozenset(
        {
            "floor_params",
            "floor_change_time_estimate",
            "min_floor_change_remaining_ratio",
            "min_floor_travel_distance",
            "elevator_geometry_refresh_enabled",
            "elevator_geometry_refresh_max_shift",
            "elevator_geometry_refresh_max_yaw_change",
            "elevator_geometry_refresh_standoff",
            "elevator_geometry_refresh_timeout",
            "elevator_navigation_timeout",
            "elevator_opening_confirmation_timeout",
            "elevator_opening_confirmation_retry_limit",
            "elevator_opening_confirmation_retry_timeout",
            "elevator_opening_confirmation_sensor_refresh",
            "elevator_local_crossing_geometry_enabled",
            "elevator_local_crossing_timeout",
            "elevator_exit_empty_fallback_timeout",
            "elevator_exit_empty_fallback_frames",
            "elevator_local_crossing_confirmations",
            "elevator_local_crossing_confirmation_max_gap",
            "elevator_local_crossing_center_tolerance",
            "elevator_local_crossing_width_tolerance",
            "elevator_local_crossing_yaw_tolerance",
            "elevator_local_crossing_yaw_search_half_range",
            "elevator_local_crossing_yaw_search_step",
            "elevator_local_crossing_max_center_correction",
            "elevator_planning_retry_limit",
            "elevator_stuck_retry_limit",
            "elevator_visual_confirmation_max_gap",
            "elevator_visual_confirmations",
            "elevator_visual_max_distance",
            "elevator_visual_max_width",
            "elevator_visual_min_width",
            "elevator_visual_scan_angular_speed",
            "elevator_visual_scan_clockwise",
            "elevator_visual_scan_duration",
            "elevator_visual_scan_required_rotation",
            "exit_cabin_backoff_enabled",
            "exit_first_max_linear",
            "elevator_async_target_door_close_enabled",
            "livox_elevator_registration_enabled",
        }
    ),
    "exploration": frozenset(
        {
            "frontier_params",
            "floor_timeout",
            "floor_finish_time_ratio",
            "route_check_ratio",
            "explore_coverage_ratio",
            "frontier_current_region_radius",
            "frontier_switch_score_margin",
            "async_frontier_planning_enabled",
            "opportunistic_replan_period",
            "frontier_preplan_min_lead",
            "frontier_preplan_max_lead",
            "frontier_planning_timeout",
            "min_exploration_progress_cells",
            "min_floor_explore_time",
            "min_known_area",
            "min_remaining_frontiers",
            "no_frontier_retreat_clearance_margin",
            "no_frontier_retreat_max_distance",
            "no_frontier_rotation_min_clearance",
            "no_frontier_scan_duration",
            "no_reachable_frontier_confirmations",
            "permanent_forbidden_decay_scale",
            "permanent_forbidden_penalty",
            "permanent_forbidden_radius_scale",
            "progress_monitor_cooldown",
            "progress_monitor_min_displacement",
            "progress_monitor_repeat",
            "progress_monitor_window",
            "exploration_progress_window",
        }
    ),
    "return_home": frozenset(
        {
            "controller_params",
            "home_tolerance",
            "min_corridor_progress_distance",
            "return_home_max_candidate_rejects",
            "return_home_max_stuck_retries",
            "return_home_veto_lookahead",
            "return_safety_buffer",
            "return_cleanup_timeout",
            "return_cleanup_wall_timeout",
            "return_speed_estimate",
        }
    ),
    "output": frozenset(
        {
            "mission_summary_file",
            "result_file",
            "status_publish_period",
        }
    ),
}


def _group_config(values: Mapping[str, object]) -> dict[str, MissionConfigSection]:
    assigned: set[str] = set()
    sections: dict[str, MissionConfigSection] = {}
    for group_name, fields in _CONFIG_GROUP_FIELDS.items():
        duplicate = assigned.intersection(fields)
        if duplicate:
            raise RuntimeError(f"任务配置字段重复分组: {sorted(duplicate)}")
        assigned.update(fields)
        missing = fields.difference(values)
        if missing:
            raise RuntimeError(
                f"任务配置组 {group_name} 缺少字段: {sorted(missing)}"
            )
        sections[group_name] = MissionConfigSection(
            MappingProxyType({name: values[name] for name in sorted(fields)})
        )
    unassigned = set(values).difference(assigned)
    if unassigned:
        raise RuntimeError(f"任务配置字段尚未分组: {sorted(unassigned)}")
    return sections


class _MissionConfigBuilder:
    def __init__(self, parameter_getter: ParameterGetter) -> None:
        self._parameter_getter = parameter_getter

    def _populate(self) -> None:
        """加载、分组并验证本节点全部 ROS 参数。

        正式 profile 先读取公开场景能力，再对每个外部话题和服务做“静态白名单
        + 本场景声明”双重校验。随后构造 mission/map/controller/detection 等参数
        字典供对应组件消费，避免组件自行从参数服务器读取而形成隐藏配置入口。
        """
        _param = self._parameter_getter
        self.runtime_profile = str(_param("runtime/profile", "formal")).strip().lower()
        if self.runtime_profile not in {"formal", "diagnostic"}:
            raise ValueError("runtime/profile 只能是 formal 或 diagnostic")
        scene_info_path = _param(
            "localization/team_scene_info_path",
            os.path.join(os.getcwd(), "generated_building", "team_scene_info.json"),
        )
        self.building_features = (
            BuildingFeatureProvider.load(scene_info_path, purpose="正式任务场景信息")
            if self.runtime_profile == "formal"
            else None
        )
        if self.building_features is not None:
            for service in ("/set_door_state", "/call_elevator"):
                self.building_features.require_service(service, purpose="建筑控制服务")

        def external_topic(topic: str, purpose: str) -> str:
            normalized = require_allowed_input_topic(topic, purpose=purpose)
            if self.building_features is not None:
                self.building_features.require_topic(normalized, purpose=purpose)
            return normalized

        self.cmd_vel_topic = require_allowed_output_topic(
            _param("topics/cmd_vel", "/cmd_vel"),
            purpose="导航控制输出",
        )
        if self.building_features is not None:
            self.building_features.require_topic(self.cmd_vel_topic, purpose="导航控制输出")
        self.localization_odom_topic = require_internal_input_topic(_param(
            "topics/localization_odom",
            "/danger_search/localization/odom",
        ), purpose="参赛方定位输出")
        self.localization_correction_request_topic = require_internal_output_topic(
            _param(
                "topics/localization_correction_request",
                "/danger_search/localization/correction_request",
            ),
            purpose="定位平移纠正请求",
        )
        self.localization_correction_state_topic = require_internal_input_topic(
            _param(
                "topics/localization_correction_state",
                "/danger_search/localization/correction_state",
            ),
            purpose="定位平移纠正回执",
        )
        self.cloud_topic = require_internal_input_topic(_param(
            "topics/cloud",
            "/danger_search/localization/cloud_registered",
        ), purpose="参赛方配准点云")
        self.rgb_topic = external_topic(
            _param("topics/rgb", "/real_sense/rgb/image_raw"),
            purpose="危险源 RGB 输入",
        )
        self.depth_topic = external_topic(
            _param("topics/depth", "/real_sense/depth/image_raw"),
            purpose="危险源深度输入",
        )
        self.camera_info_topic = external_topic(
            _param("topics/camera_info", "/real_sense/rgb/camera_info"),
            purpose="危险源相机标定输入",
        )
        self.manual_goal_enabled = bool(_param("mission/manual_goal_enabled", False))
        if self.runtime_profile == "formal" and self.manual_goal_enabled:
            raise ValueError("正式运行不允许启用未在初始接口文档中的 /nav/goal")
        self.camera_mount_xyz = tuple(
            float(value)
            for value in _param("localization/base_to_camera_xyz", [0.28, 0.0, 0.043])
        )
        self.camera_mount_rpy = tuple(
            float(value)
            for value in _param("localization/base_to_camera_rpy", [0.0, 0.0, 0.0])
        )
        self.localization_stale_minimum = float(
            _param("localization/stale_minimum", 0.4)
        )
        self.localization_unavailable_timeout = float(
            _param("localization/unavailable_timeout", 3.0)
        )
        self.localization_stance_health_required = bool(
            _param("localization/stance_health_required", True)
        )
        self.global_frame_id = str(
            _param("localization/global_frame_id", "world")
        ).strip()
        if self.global_frame_id not in {"world", "danger_search_relative"}:
            raise ValueError(
                "localization/global_frame_id 仅支持 world 或 danger_search_relative"
            )
        if self.runtime_profile == "formal" and self.global_frame_id != "world":
            raise ValueError("正式运行的 localization/global_frame_id 必须为 world")
        self.localization_vertical_reference = str(
            _param("localization/vertical_reference", "floor_local")
        ).strip()
        if self.localization_vertical_reference not in {"floor_local", "absolute"}:
            raise ValueError("localization/vertical_reference 仅支持 floor_local 或 absolute")
        if (
            not math.isfinite(self.localization_stale_minimum)
            or self.localization_stale_minimum <= 0.0
            or not math.isfinite(self.localization_unavailable_timeout)
            or self.localization_unavailable_timeout <= self.localization_stale_minimum
        ):
            raise ValueError("定位新鲜度阈值必须为正，且不可用超时必须更长")
        self.status_topic = require_internal_output_topic(_param("topics/status", "/nav/status"), purpose="任务状态")
        self.goal_topic = require_manual_control_topic(_param("topics/goal", "/nav/goal"), purpose="手动控制型测试目标")
        self.map_topic = require_internal_output_topic(_param("topics/map", "/nav/map_debug"), purpose="地图调试")
        self.path_raw_topic = require_internal_output_topic(_param("topics/path_raw", "/danger_search/path_raw"), purpose="原始路径调试")
        self.path_followed_topic = require_internal_output_topic(_param(
            "topics/path_followed",
            "/danger_search/path_followed",
        ), purpose="跟踪路径调试")
        self.local_target_topic = require_internal_output_topic(_param(
            "topics/local_target",
            "/danger_search/local_target",
        ), purpose="局部目标调试")

        self.startup_delay = float(_param("mission/startup_delay", 2.0))
        configured_max_runtime = float(_param("mission/max_runtime", 600.0))
        runtime_by_floor = _param(
            "mission/runtime_by_floor",
            [[1, 540.0], [2, 840.0], [3, 1280.0]],
        )
        self.max_floor = int(_param("mission/max_floor", 1))
        if self.runtime_profile == "formal" and self.building_features is not None:
            public_elevators = self.building_features.elevator_features
            if len(public_elevators) != 1:
                raise ValueError("正式多楼层运行要求公开 schema 恰好声明一个电梯")
            elevator_feature = next(iter(public_elevators.values()))
            served_floors = sorted(elevator_feature.served_floors)
            if served_floors != list(range(len(served_floors))):
                raise ValueError("正式多楼层运行要求 served_floors 从 0 连续编号")
            elevator_doors_by_floor = {
                floor: [
                    door.door_id
                    for door in self.building_features.doors.values()
                    if door.kind == "elevator" and door.floor_index == floor
                ]
                for floor in served_floors
            }
            if any(len(door_ids) != 1 for door_ids in elevator_doors_by_floor.values()):
                raise ValueError("正式多楼层运行要求每个 served floor 恰好声明一个电梯门")
            self.max_floor = len(served_floors)
        self.max_runtime = _effective_max_runtime(
            configured_max_runtime,
            max_floor=self.max_floor,
            runtime_by_floor=runtime_by_floor,
        )
        self.floor_timeout = float(_param("mission/floor_timeout", 360.0))
        self.last_floor_uses_global_budget = bool(
            _param("mission/last_floor_uses_global_budget", True)
        )
        self.min_floor_explore_time = float(_param("mission/min_floor_explore_time", 150.0))

        # 方案 C：全局时间预算参数
        self.return_safety_buffer = float(_param("mission/return_safety_buffer", 30.0))
        self.return_cleanup_timeout = float(_param("mission/return_cleanup_timeout", 180.0))
        self.return_cleanup_wall_timeout = float(_param("mission/return_cleanup_wall_timeout", 600.0))
        if any(not math.isfinite(value) or value <= 0.0 for value in (
            self.return_cleanup_timeout, self.return_cleanup_wall_timeout,
        )):
            raise ValueError("超时返航收尾的任务时钟和墙钟上限必须为有限正数")
        self.return_speed_estimate = float(_param("mission/return_speed_estimate", 0.35))
        self.floor_change_time_estimate = float(_param("mission/floor_change_time_estimate", 70.0))
        self.floor_finish_time_ratio = float(_param("mission/floor_finish_time_ratio", 0.75))
        self.route_check_ratio = float(_param("mission/route_check_ratio", 0.75))
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
        self.danger_approach_verification_enabled = bool(
            _param("mission/danger_approach_verification_enabled", True)
        )
        self.danger_approach_trigger_distance = float(
            _param("mission/danger_approach_trigger_distance", 2.2)
        )
        self.danger_approach_standoff_distance = float(
            _param("mission/danger_approach_standoff_distance", 1.3)
        )
        self.danger_approach_max_anchor_error = float(
            _param("mission/danger_approach_max_anchor_error", 0.5)
        )
        self.danger_verification_timeout = float(
            _param("mission/danger_verification_timeout", 5.0)
        )
        raw_danger_verification_min_hits = _param(
            "mission/danger_verification_min_hits", 2
        )
        self.danger_inspection_retry_cooldown = float(
            _param("mission/danger_inspection_retry_cooldown", 5.0)
        )
        self.danger_inspection_timeout = float(
            _param("mission/danger_inspection_timeout", 30.0)
        )
        self.danger_verification_min_observation_span = float(
            _param("mission/danger_verification_min_observation_span", 0.08)
        )
        if (
            not math.isfinite(self.danger_approach_trigger_distance)
            or not math.isfinite(self.danger_approach_standoff_distance)
            or self.danger_approach_trigger_distance <= 0.0
            or self.danger_approach_standoff_distance <= 0.0
            or self.danger_approach_trigger_distance
            <= self.danger_approach_standoff_distance
            or not math.isfinite(self.danger_approach_max_anchor_error)
            or self.danger_approach_max_anchor_error <= 0.0
            or not math.isfinite(self.danger_verification_timeout)
            or self.danger_verification_timeout <= 0.0
            or isinstance(raw_danger_verification_min_hits, bool)
            or not isinstance(raw_danger_verification_min_hits, int)
            or raw_danger_verification_min_hits < 2
            or not math.isfinite(self.danger_inspection_retry_cooldown)
            or self.danger_inspection_retry_cooldown < 0.0
            or not math.isfinite(self.danger_inspection_timeout)
            or self.danger_inspection_timeout <= self.danger_verification_timeout
            or not math.isfinite(self.danger_verification_min_observation_span)
            or not 0.0 < self.danger_verification_min_observation_span
            < self.danger_verification_timeout
        ):
            raise ValueError("危险源接近与近场核验参数必须为有限有效值")
        self.danger_verification_min_hits = raw_danger_verification_min_hits
        raw_max_stuck_retries = _param("mission/return_home_max_stuck_retries", 4)
        raw_max_candidate_rejects = _param(
            "mission/return_home_max_candidate_rejects", 3
        )
        if (
            isinstance(raw_max_stuck_retries, bool)
            or not isinstance(raw_max_stuck_retries, int)
            or raw_max_stuck_retries <= 0
            or isinstance(raw_max_candidate_rejects, bool)
            or not isinstance(raw_max_candidate_rejects, int)
            or raw_max_candidate_rejects <= 0
        ):
            raise ValueError("返航卡滞和候选拒绝上限必须为正整数")
        self.return_home_max_stuck_retries = raw_max_stuck_retries
        self.return_home_max_candidate_rejects = raw_max_candidate_rejects
        self.return_home_veto_lookahead = float(
            _param("mission/return_home_veto_lookahead", 0.8)
        )
        if (
            not math.isfinite(self.return_home_veto_lookahead)
            or self.return_home_veto_lookahead < 0.15
        ):
            raise ValueError("返航失败首段否决距离必须为不小于 0.15m 的有限数")
        self.elevator_navigation_timeout = float(
            _param("mission/elevator_navigation_timeout", 120.0)
        )
        raw_elevator_planning_retry_limit = _param(
            "mission/elevator_planning_retry_limit", 3
        )
        raw_elevator_stuck_retry_limit = _param(
            "mission/elevator_stuck_retry_limit", 4
        )
        if (
            not math.isfinite(self.elevator_navigation_timeout)
            or self.elevator_navigation_timeout <= 0.0
            or isinstance(raw_elevator_planning_retry_limit, bool)
            or not isinstance(raw_elevator_planning_retry_limit, int)
            or raw_elevator_planning_retry_limit <= 0
            or isinstance(raw_elevator_stuck_retry_limit, bool)
            or not isinstance(raw_elevator_stuck_retry_limit, int)
            or raw_elevator_stuck_retry_limit <= 0
        ):
            raise ValueError("电梯导航超时和规划/卡住重试上限必须为有限正值")
        self.elevator_planning_retry_limit = raw_elevator_planning_retry_limit
        self.elevator_stuck_retry_limit = raw_elevator_stuck_retry_limit
        default_coverage = 0.80 if self.max_floor == 1 else 0.65
        self.explore_coverage_ratio = float(
            _param("mission/explore_coverage_ratio", default_coverage)
        )
        self.no_frontier_scan_duration = float(_param("mission/no_frontier_scan_duration", 8.0))
        self.no_frontier_rotation_min_clearance = float(
            _param("mission/no_frontier_rotation_min_clearance", 0.40)
        )
        self.no_frontier_retreat_max_distance = float(
            _param("mission/no_frontier_retreat_max_distance", 0.8)
        )
        self.no_frontier_retreat_clearance_margin = float(
            _param("mission/no_frontier_retreat_clearance_margin", 0.10)
        )
        if (
            not math.isfinite(self.no_frontier_rotation_min_clearance)
            or self.no_frontier_rotation_min_clearance <= 0.0
            or not math.isfinite(self.no_frontier_retreat_max_distance)
            or self.no_frontier_retreat_max_distance < 0.5
            or not math.isfinite(self.no_frontier_retreat_clearance_margin)
            or self.no_frontier_retreat_clearance_margin < 0.0
        ):
            raise ValueError("无前沿扫描与沿迹后退参数必须为有限有效值")
        self.probe_distance = float(_param("mission/probe_distance", 1.2))
        self.probe_yaw = float(_param("mission/probe_yaw", math.pi / 2.0))
        self.initial_entry_max_distance = float(
            _param("mission/initial_entry_max_distance", 5.0)
        )
        self.initial_entry_corridor_half_width = float(
            _param("mission/initial_entry_corridor_half_width", 0.20)
        )
        self.initial_entry_opening_confirmations = max(
            1, int(_param("mission/initial_entry_opening_confirmations", 2))
        )
        self.initial_entry_opening_max_distance = float(
            _param("mission/initial_entry_opening_max_distance", 3.0)
        )
        self.initial_entry_center_tolerance = float(
            _param("mission/initial_entry_center_tolerance", 0.60)
        )
        self.initial_entry_geometry_tolerance = float(
            _param("mission/initial_entry_geometry_tolerance", 0.25)
        )
        self.initial_entry_wait_timeout = float(
            _param("mission/initial_entry_wait_timeout", 5.0)
        )
        if (
            not math.isfinite(self.initial_entry_max_distance)
            or self.initial_entry_max_distance <= 0.0
            or not math.isfinite(self.initial_entry_corridor_half_width)
            or self.initial_entry_corridor_half_width < 0.0
            or not math.isfinite(self.initial_entry_opening_max_distance)
            or self.initial_entry_opening_max_distance <= 0.0
            or not math.isfinite(self.initial_entry_center_tolerance)
            or self.initial_entry_center_tolerance <= 0.0
            or not math.isfinite(self.initial_entry_geometry_tolerance)
            or self.initial_entry_geometry_tolerance <= 0.0
            or not math.isfinite(self.initial_entry_wait_timeout)
            or self.initial_entry_wait_timeout <= 0.0
        ):
            raise ValueError("初始入口探测参数必须为有限正值")
        self.door_pass_offset = float(_param("mission/door_pass_offset", 0.8))
        self.door_wait_offset = float(_param("mission/door_wait_offset", 1.6))
        self.door_commit_distance = float(_param("mission/door_commit_distance", 1.8))
        self.door_pass_max_linear = float(
            _param("mission/door_pass_max_linear", 0.25)
        )
        self.main_entrance_clearance_max_linear = float(
            _param("mission/main_entrance_clearance_max_linear", 0.4)
        )
        self.door_refresh_timeout = float(
            _param("mission/door_refresh_timeout", 3.0)
        )
        self.door_refresh_max_center_shift = float(
            _param("mission/door_refresh_max_center_shift", 0.60)
        )
        self.door_clearance_margin = float(
            _param("mission/door_clearance_margin", 0.10)
        )
        if (
            not math.isfinite(self.door_pass_offset)
            or self.door_pass_offset <= 0.0
            or not math.isfinite(self.door_wait_offset)
            or self.door_wait_offset <= 0.0
            or not math.isfinite(self.door_commit_distance)
            or self.door_commit_distance <= 0.0
            or self.door_commit_distance < self.door_pass_offset
            or self.initial_entry_opening_max_distance < self.door_wait_offset + 0.30
            or not math.isfinite(self.door_pass_max_linear)
            or self.door_pass_max_linear <= 0.0
            or not math.isfinite(self.main_entrance_clearance_max_linear)
            or self.main_entrance_clearance_max_linear <= 0.0
            or not math.isfinite(self.door_refresh_timeout)
            or self.door_refresh_timeout <= 0.0
            or not math.isfinite(self.door_refresh_max_center_shift)
            or self.door_refresh_max_center_shift <= 0.0
            or not math.isfinite(self.door_clearance_margin)
            or self.door_clearance_margin < 0.0
        ):
            raise ValueError(
                "过门参数必须有限有效：深入距离不小于通过距离，观测上限至少比等待距离多0.30m"
            )
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
        self.tick_wall_stall_timeout = float(
            _param("mission/tick_wall_stall_timeout", 60.0)
        )
        if not math.isfinite(self.tick_wall_stall_timeout):
            raise ValueError("mission/tick_wall_stall_timeout 必须为有限数")
        self.exit_cabin_backoff_enabled = bool(
            _param("mission/exit_cabin_backoff_enabled", False)
        )
        configured_result = str(_param("result_file", "")).strip()
        self.mission_summary_file = str(_param("mission_summary_file", "")).strip()
        if self.building_features is not None:
            public_result = self.building_features.result_file
            if configured_result and Path(configured_result).resolve() != public_result.resolve():
                raise ValueError("正式运行结果文件必须使用 team_scene_info.json 公布的路径")
            if self.mission_summary_file:
                self.mission_summary_file = str(
                    _validate_summary_output_path(self.mission_summary_file, public_result)
                )
            self.result_file = str(public_result)
        else:
            self.result_file = configured_result or os.path.join(os.getcwd(), "results", "detected_danger.json")

        self.map_params = {
            "resolution": float(_param("map/resolution", 0.10)),
            "width": float(_param("map/width", 28.0)),
            "length": float(_param("map/length", 48.0)),
            "origin_x": float(_param("map/origin_x", -14.0)),
            "origin_y": float(_param("map/origin_y", -6.0)),
            "floor_height": float(_param("map/floor_height", 2.6)),
            "obstacle_min_z": float(_param("map/obstacle_min_z", 0.20)),
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
            "evidence_lifecycle_enabled": bool(
                _param("map/evidence_lifecycle_enabled", True)
            ),
            "low_height_max_z": float(_param("map/low_height_max_z", 0.40)),
            "low_confirm_generations": int(
                _param("map/low_confirm_generations", 3)
            ),
            "low_confirm_window_generations": int(
                _param("map/low_confirm_window_generations", 20)
            ),
            "low_candidate_counter_generations": int(
                _param("map/low_candidate_counter_generations", 3)
            ),
            "low_candidate_counter_positions": int(
                _param("map/low_candidate_counter_positions", 2)
            ),
            "low_confirmed_counter_generations": int(
                _param("map/low_confirmed_counter_generations", 6)
            ),
            "low_confirmed_counter_positions": int(
                _param("map/low_confirmed_counter_positions", 3)
            ),
            "low_near_exempt_radius": float(
                _param("map/low_near_exempt_radius", 0.0)
            ),
            "low_candidate_max_range": float(
                _param("map/low_candidate_max_range", 0.0)
            ),
            "trail_protection_enabled": bool(
                _param("map/trail_protection_enabled", True)
            ),
            "trail_protection_radius": float(
                _param("map/trail_protection_radius", 0.25)
            ),
            "trail_protection_near_range": float(
                _param("map/trail_protection_near_range", 2.0)
            ),
            "trail_protection_confirm_generations": int(
                _param("map/trail_protection_confirm_generations", 3)
            ),
            "trail_protection_confirm_positions": int(
                _param("map/trail_protection_confirm_positions", 2)
            ),
            "counter_pose_separation": float(
                _param("map/counter_pose_separation", 0.15)
            ),
        }
        self.cloud_stride = int(_param("map/cloud_stride", 3))
        self.cloud_max_points = int(_param("map/cloud_max_points", 18000))
        stop_distance_table = _parse_stop_distance_table(
            _param(
                "ground_support/stop_distance_table",
                [
                    [0.60, 0.09337727739487302],
                    [0.651, 0.10195826367975175],
                    [0.80, 0.15200000000000000],
                    [1.00, 0.23000000000000000],
                ],
            )
        )
        self.ground_support_params = {
            "production_gate_enabled": bool(
                _param("ground_support/production_gate_enabled", False)
            ),
            "unknown_support_hold_enabled": bool(
                _param("ground_support/unknown_support_hold_enabled", False)
            ),
            "negative_obstacle_replan_enabled": bool(
                _param("ground_support/negative_obstacle_replan_enabled", True)
            ),
            # 有效速度域由可替换停车表最后一档决定，避免扩速时双参数漂移。
            "validated_max_speed": float(stop_distance_table[-1][0]),
            "stop_distance_table": stop_distance_table,
            "localization_speed_anomaly_threshold": float(
                _param("ground_support/localization_speed_anomaly_threshold", 1.20)
            ),
            "localization_speed_anomaly_timeout": float(
                _param("ground_support/localization_speed_anomaly_timeout", 2.0)
            ),
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
            "boundary_hold_margin": float(
                _param("ground_support/boundary_hold_margin", 0.30)
            ),
            "boundary_persistence_confirmations": int(
                _param("ground_support/boundary_persistence_confirmations", 3)
            ),
            "boundary_consistency_radius": float(
                _param("ground_support/boundary_consistency_radius", 0.20)
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
            "wall_stall_timeout": float(
                _param("ground_support/wall_stall_timeout", 30.0)
            ),
            # 必须与第 2 层定位发布速度死区一致；停车阈值不得小于测量死区。
            "odometry_linear_deadband": float(
                _param("ground_support/odometry_linear_deadband", 0.05)
            ),
            "odometry_angular_deadband": float(
                _param("ground_support/odometry_angular_deadband", 0.03)
            ),
            "stop_linear_threshold": float(
                _param("ground_support/stop_linear_threshold", 0.05)
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
            # 初始 docs/sensors-and-topics.md 公布的固定 Livox 外参；正式链不查询 TF。
            "base_to_livox_xyz": tuple(
                float(value)
                for value in _param("localization/base_to_livox_xyz", [0.2, 0.0, 0.08])
            ),
            "base_to_livox_rpy": tuple(
                float(value)
                for value in _param("localization/base_to_livox_rpy", [0.0, 0.785, 0.0])
            ),
        }
        if (
            self.runtime_profile == "formal"
            and not self.ground_support_params["production_gate_enabled"]
        ):
            raise ValueError("正式运行必须启用地面支撑安全门")
        if (
            self.runtime_profile == "formal"
            and not self.localization_stance_health_required
        ):
            raise ValueError("正式运行必须启用定位持续姿态健康检查")
        self.frontier_params = {
            "min_cluster_size": int(_param("frontier/min_cluster_size", 8)),
            "blacklist_ttl": float(_param("frontier/blacklist_ttl", 60.0)),
            "fail_limit": int(_param("frontier/fail_limit", 3)),
            "stuck_blacklist_radius": float(_param("frontier/stuck_blacklist_radius", 1.20)),
            "information_gain_weight": float(_param("frontier/information_gain_weight", 2.0)),
            "path_cost_weight": float(_param("frontier/path_cost_weight", 1.0)),
            "yaw_cost_weight": float(_param("frontier/yaw_cost_weight", 0.4)),
            "path_turn_cost_weight": float(
                _param("frontier/path_turn_cost_weight", 0.7)
            ),
            "forward_alignment_weight": float(
                _param("frontier/forward_alignment_weight", 0.8)
            ),
            "revisit_penalty_weight": float(_param("frontier/revisit_penalty_weight", 2.0)),
            "frontier_obstacle_clearance": float(_param("frontier/frontier_obstacle_clearance", 0.25)),
            "frontier_unknown_standoff": float(_param("frontier/frontier_unknown_standoff", 0.20)),
            "frontier_min_information_gain": float(_param("frontier/frontier_min_information_gain", 1.0)),
            "expected_gain_radius": float(
                _param("frontier/expected_gain_radius", 3.0)
            ),
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
            "min_far_expected_gain": float(
                _param("frontier/min_far_expected_gain", 4.0)
            ),
            "far_prune_distance": float(
                _param("frontier/far_prune_distance", 3.5)
            ),
        }
        self.frontier_switch_score_margin = float(_param("frontier/switch_score_margin", 2.0))
        self.frontier_current_region_radius = float(_param("frontier/current_region_radius", 1.2))
        self.async_frontier_planning_enabled = bool(
            _param("frontier/async_planning_enabled", True)
        )
        self.opportunistic_replan_period = float(
            _param("frontier/opportunistic_replan_period", 3.0)
        )
        self.frontier_preplan_min_lead = float(
            _param("frontier/preplan_min_lead", 0.5)
        )
        self.frontier_preplan_max_lead = float(
            _param("frontier/preplan_max_lead", 3.0)
        )
        self.frontier_planning_timeout = float(
            _param("frontier/planning_timeout", 5.0)
        )
        if (
            not math.isfinite(self.opportunistic_replan_period)
            or self.opportunistic_replan_period < 0.0
            or not all(
                math.isfinite(value) and value > 0.0
                for value in (
                    self.frontier_preplan_min_lead,
                    self.frontier_preplan_max_lead,
                    self.frontier_planning_timeout,
                )
            )
            or self.frontier_preplan_min_lead > self.frontier_preplan_max_lead
        ):
            raise ValueError("异步前沿规划时间参数无效")
        self.exit_first_max_linear = _optional_positive_float(
            _param("controller/exit_first_max_linear", None),
            "controller/exit_first_max_linear",
        )
        self.controller_params = {
            "max_linear": float(_param("controller/max_linear", 0.80)),
            "max_lateral": float(_param("controller/max_lateral", 0.25)),
            "max_angular": float(_param("controller/max_angular", 0.90)),
            "min_linear_fraction": float(_param("controller/min_linear_fraction", 0.35)),
            "arc_turn_min_linear_fraction": float(
                _param("controller/arc_turn_min_linear_fraction", 0.0)
            ),
            "goal_tolerance": float(_param("controller/goal_tolerance", 0.35)),
            "waypoint_tolerance": float(_param("controller/waypoint_tolerance", 0.25)),
            "corner_tolerance": float(_param("controller/corner_tolerance", 0.15)),
            "pure_pursuit_lookahead": float(_param("controller/pure_pursuit_lookahead", 0.45)),
            "reverse_speed": float(_param("controller/reverse_speed", -0.35)),
            "recover_duration": float(_param("controller/recover_duration", 0.5)),
            "recover_turn_duration": float(_param("controller/recover_turn_duration", 1.0)),
            "recover_angular_speed": float(_param("controller/recover_angular_speed", 0.70)),
            "command_keepalive_period": float(
                _param("controller/command_keepalive_period", 0.1)
            ),
            "pose_max_age": float(_param("controller/pose_max_age", 0.30)),
            "yaw_rate_damping": float(_param("controller/yaw_rate_damping", 0.45)),
            "turn_in_place_heading": float(
                _param("controller/turn_in_place_heading", 0.70)
            ),
            "turn_in_place_rate": float(
                _param("controller/turn_in_place_rate", 0.65)
            ),
            "turn_release_heading": float(
                _param("controller/turn_release_heading", 0.20)
            ),
            "turn_release_rate": float(
                _param("controller/turn_release_rate", 0.20)
            ),
            "turn_release_samples": int(
                _param("controller/turn_release_samples", 3)
            ),
            "route_progress_timeout": float(
                _param("controller/route_progress_timeout", 8.0)
            ),
            "route_progress_improvement": float(
                _param("controller/route_progress_improvement", 0.10)
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
            "target_radius": float(_param("detection/target_radius", 0.15)),
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
            "opening_min_width": float(_param("local_semantic/opening_min_width", 0.65)),
            "opening_max_width": float(_param("local_semantic/opening_max_width", 2.50)),
            "opening_plane_depth": float(_param("local_semantic/opening_plane_depth", 0.30)),
            "opening_min_side_height_span": float(
                _param("local_semantic/opening_min_side_height_span", 0.30)
            ),
            "use_truth_proximity_detector": bool(_param("local_semantic/use_truth_proximity_detector", False)),
        }
        if self.local_detection_params["max_distance"] < self.initial_entry_opening_max_distance:
            raise ValueError("门洞检测搜索范围不能小于主入口确认距离上限")
        self.rgbd_door_detection_enabled = bool(
            _param("local_semantic/rgbd_door_detection_enabled", True)
        )
        self.livox_elevator_registration_enabled = bool(
            _param("local_semantic/livox_elevator_registration_enabled", False)
        )
        self.rgbd_door_detection_params = {
            "maximum_value": int(_param("local_semantic/rgbd_dark_max_value", 50)),
            "maximum_saturation": int(
                _param("local_semantic/rgbd_dark_max_saturation", 85)
            ),
            "minimum_pixel_area": int(
                _param("local_semantic/rgbd_minimum_pixel_area", 500)
            ),
            "minimum_depth": float(
                _param("local_semantic/rgbd_minimum_depth", 0.40)
            ),
            "maximum_depth": float(
                _param("local_semantic/rgbd_maximum_depth", 5.0)
            ),
            "minimum_depth_pixels": int(
                _param("local_semantic/rgbd_minimum_depth_pixels", 250)
            ),
            "minimum_panel_width": float(
                _param("local_semantic/rgbd_minimum_panel_width", 0.65)
            ),
            "maximum_panel_width": float(
                _param("local_semantic/rgbd_maximum_panel_width", 2.20)
            ),
            "minimum_panel_height": float(
                _param("local_semantic/rgbd_minimum_panel_height", 0.80)
            ),
            "maximum_plane_rmse": float(
                _param("local_semantic/rgbd_maximum_plane_rmse", 0.04)
            ),
            "maximum_vertical_normal_z": float(
                _param("local_semantic/rgbd_maximum_vertical_normal_z", 0.25)
            ),
            "minimum_opening_depth_change": float(
                _param("local_semantic/rgbd_minimum_opening_depth_change", 0.30)
            ),
            "maximum_opening_dark_ratio": float(
                _param("local_semantic/rgbd_maximum_opening_dark_ratio", 0.45)
            ),
            "minimum_opening_deeper_fraction": float(
                _param("local_semantic/rgbd_minimum_opening_deeper_fraction", 0.20)
            ),
            "minimum_opening_valid_depth_fraction": float(
                _param(
                    "local_semantic/rgbd_minimum_opening_valid_depth_fraction",
                    0.20,
                )
            ),
        }
        self.elevator_visual_confirmations = max(
            2, int(_param("local_semantic/elevator_visual_confirmations", 3))
        )
        self.elevator_visual_confirmation_max_gap = float(
            _param("local_semantic/elevator_visual_confirmation_max_gap", 0.4)
        )
        self.elevator_visual_min_width = float(
            _param("local_semantic/elevator_visual_min_width", 0.65)
        )
        self.elevator_visual_max_width = float(
            _param("local_semantic/elevator_visual_max_width", 1.80)
        )
        self.elevator_visual_max_distance = float(
            _param("local_semantic/elevator_visual_max_distance", 4.0)
        )
        self.elevator_opening_confirmation_timeout = float(
            _param("local_semantic/elevator_opening_confirmation_timeout", 15.0)
        )
        self.elevator_opening_confirmation_retry_limit = int(
            _param("local_semantic/elevator_opening_confirmation_retry_limit", 1)
        )
        self.elevator_opening_confirmation_retry_timeout = float(
            _param("local_semantic/elevator_opening_confirmation_retry_timeout", 10.0)
        )
        self.elevator_opening_confirmation_sensor_refresh = float(
            _param("local_semantic/elevator_opening_confirmation_sensor_refresh", 0.5)
        )
        self.elevator_local_crossing_geometry_enabled = bool(
            _param("mission/elevator_local_crossing_geometry_enabled", True)
        )
        self.elevator_async_target_door_close_enabled = bool(
            _param("mission/elevator_async_target_door_close_enabled", True)
        )
        self.elevator_local_crossing_timeout = float(
            _param("mission/elevator_local_crossing_timeout", 4.0)
        )
        self.elevator_exit_empty_fallback_timeout = float(
            _param("mission/elevator_exit_empty_fallback_timeout", 1.0)
        )
        self.elevator_exit_empty_fallback_frames = int(
            _param("mission/elevator_exit_empty_fallback_frames", 10)
        )
        self.elevator_local_crossing_confirmations = int(
            _param("mission/elevator_local_crossing_confirmations", 3)
        )
        self.elevator_local_crossing_confirmation_max_gap = float(
            _param("mission/elevator_local_crossing_confirmation_max_gap", 0.5)
        )
        self.elevator_local_crossing_center_tolerance = float(
            _param("mission/elevator_local_crossing_center_tolerance", 0.15)
        )
        self.elevator_local_crossing_width_tolerance = float(
            _param("mission/elevator_local_crossing_width_tolerance", 0.20)
        )
        self.elevator_local_crossing_yaw_tolerance = float(
            _param("mission/elevator_local_crossing_yaw_tolerance", 0.15)
        )
        self.elevator_local_crossing_yaw_search_half_range = float(
            _param("mission/elevator_local_crossing_yaw_search_half_range", 0.20)
        )
        self.elevator_local_crossing_yaw_search_step = float(
            _param("mission/elevator_local_crossing_yaw_search_step", 0.05)
        )
        self.elevator_local_crossing_max_center_correction = float(
            _param("mission/elevator_local_crossing_max_center_correction", 0.80)
        )
        self.elevator_geometry_refresh_enabled = bool(
            _param("local_semantic/elevator_geometry_refresh_enabled", True)
        )
        self.elevator_geometry_refresh_timeout = float(
            _param("local_semantic/elevator_geometry_refresh_timeout", 6.0)
        )
        self.elevator_geometry_refresh_standoff = float(
            _param("local_semantic/elevator_geometry_refresh_standoff", 1.2)
        )
        self.elevator_geometry_refresh_max_shift = float(
            _param("local_semantic/elevator_geometry_refresh_max_shift", 0.75)
        )
        self.elevator_geometry_refresh_max_yaw_change = float(
            _param("local_semantic/elevator_geometry_refresh_max_yaw_change", 0.35)
        )
        self.elevator_visual_scan_duration = float(
            _param("local_semantic/elevator_visual_scan_duration", 18.0)
        )
        self.elevator_visual_scan_angular_speed = float(
            _param("local_semantic/elevator_visual_scan_angular_speed", 0.90)
        )
        self.elevator_visual_scan_required_rotation = float(
            _param("local_semantic/elevator_visual_scan_required_rotation", 5.80)
        )
        self.elevator_visual_scan_clockwise = bool(
            _param("local_semantic/elevator_visual_scan_clockwise", True)
        )
        self.landmark_drift_config = LandmarkDriftConfig(
            enabled=bool(
                _param("local_semantic/landmark_drift_diagnostic_enabled", False)
            ),
            minimum_width=float(
                _param("local_semantic/landmark_drift_minimum_width", 0.65)
            ),
            maximum_width=float(
                _param("local_semantic/landmark_drift_maximum_width", 1.80)
            ),
            maximum_distance=float(
                _param("local_semantic/landmark_drift_maximum_distance", 4.0)
            ),
            maximum_yaw_difference=float(
                _param("local_semantic/landmark_drift_maximum_yaw_difference", 0.35)
            ),
            maximum_width_difference=float(
                _param("local_semantic/landmark_drift_maximum_width_difference", 0.25)
            ),
            required_confirmations=int(
                _param("local_semantic/landmark_drift_required_confirmations", 3)
            ),
            maximum_confirmation_gap=float(
                _param("local_semantic/landmark_drift_maximum_confirmation_gap", 0.4)
            ),
            maximum_residual_spread=float(
                _param("local_semantic/landmark_drift_maximum_residual_spread", 0.15)
            ),
        )
        self.sparse_relocalization_config = SparseRelocalizationConfig(
            enabled=bool(_param("localization/sparse_relocalization_enabled", True)),
            anchor_radius=float(_param("localization/sparse_anchor_radius", 12.0)),
            voxel_size=float(_param("localization/sparse_voxel_size", 0.10)),
            minimum_points=int(_param("localization/sparse_minimum_points", 120)),
            maximum_points=int(_param("localization/sparse_maximum_points", 2400)),
            minimum_correction=float(
                _param("localization/sparse_minimum_correction", 0.08)
            ),
            maximum_correction=float(
                _param("localization/sparse_maximum_correction", 1.20)
            ),
            coarse_step=float(_param("localization/sparse_coarse_step", 0.10)),
            minimum_overlap=float(
                _param("localization/sparse_minimum_overlap", 0.55)
            ),
            maximum_residual_p70=float(
                _param("localization/sparse_maximum_residual_p70", 0.18)
            ),
            minimum_uniqueness_margin=float(
                _param("localization/sparse_minimum_uniqueness_margin", 0.08)
            ),
            required_frames=int(_param("localization/sparse_required_frames", 3)),
            maximum_frame_spread=float(
                _param("localization/sparse_maximum_frame_spread", 0.08)
            ),
            maximum_linear_speed=float(
                _param("localization/sparse_maximum_linear_speed", 0.08)
            ),
            maximum_angular_speed=float(
                _param("localization/sparse_maximum_angular_speed", 0.10)
            ),
            correction_timeout=float(
                _param("localization/sparse_correction_timeout", 2.0)
            ),
        )
        self.entrance_geometry_config = EntranceGeometryConfig(
            enabled=bool(_param("localization/entrance_geometry_enabled", True)),
            apply_correction=bool(_param("localization/entrance_geometry_apply_correction", False)),
            center_x=float(_param("localization/entrance_geometry_center_x", 0.0)),
            wall_y=float(_param("localization/entrance_geometry_wall_y", 0.09)),
            opening_width=float(_param("localization/entrance_geometry_opening_width", 2.0)),
            wall_half_length=float(_param("localization/entrance_geometry_wall_half_length", 5.51)),
            wall_thickness=float(_param("localization/entrance_geometry_wall_thickness", 0.18)),
            voxel_size=float(_param("localization/entrance_geometry_voxel_size", 0.08)),
            minimum_points=int(_param("localization/entrance_geometry_minimum_points", 80)),
            required_frames=int(_param("localization/entrance_geometry_required_frames", 3)),
            maximum_frame_spread=float(_param("localization/entrance_geometry_maximum_frame_spread", 0.08)),
            minimum_correction=float(_param("localization/entrance_geometry_minimum_correction", 0.08)),
            maximum_correction=float(_param("localization/entrance_geometry_maximum_correction", 0.50)),
            coarse_step=float(_param("localization/entrance_geometry_coarse_step", 0.05)),
            maximum_residual_p70=float(_param("localization/entrance_geometry_maximum_residual_p70", 0.12)),
            minimum_uniqueness_margin=float(_param("localization/entrance_geometry_minimum_uniqueness_margin", 0.04)),
            maximum_yaw_correction=float(_param("localization/entrance_geometry_maximum_yaw_correction", 0.12)),
            yaw_coarse_step=float(_param("localization/entrance_geometry_yaw_coarse_step", 0.02)),
            maximum_yaw_spread=float(_param("localization/entrance_geometry_maximum_yaw_spread", 0.03)),
            minimum_yaw_correction=float(_param("localization/entrance_geometry_minimum_yaw_correction", 0.01)),
            minimum_residual_improvement=float(_param("localization/entrance_geometry_minimum_residual_improvement", 0.03)),
            minimum_side_points=int(_param("localization/entrance_geometry_minimum_side_points", 16)),
            timeout=float(_param("localization/entrance_geometry_timeout", 3.0)),
            maximum_frame_age=float(_param("localization/entrance_geometry_maximum_frame_age", 1.0)),
            maximum_linear_speed=float(_param("localization/entrance_geometry_maximum_linear_speed", 0.08)),
            maximum_angular_speed=float(_param("localization/entrance_geometry_maximum_angular_speed", 0.10)),
        )
        if not (
            math.isfinite(self.elevator_visual_min_width)
            and math.isfinite(self.elevator_visual_max_width)
            and math.isfinite(self.elevator_visual_max_distance)
            and math.isfinite(self.elevator_visual_confirmation_max_gap)
            and 0.0 < self.elevator_visual_confirmation_max_gap <= 1.0
            and math.isfinite(self.elevator_opening_confirmation_timeout)
            and 0.0 < self.elevator_visual_min_width < self.elevator_visual_max_width
            and self.elevator_visual_max_distance > 0.0
            and self.elevator_opening_confirmation_timeout > 0.0
            and self.elevator_opening_confirmation_retry_limit in (0, 1, 2)
            and math.isfinite(self.elevator_opening_confirmation_retry_timeout)
            and self.elevator_opening_confirmation_retry_timeout > 0.0
            and math.isfinite(self.elevator_opening_confirmation_sensor_refresh)
            and 0.0 < self.elevator_opening_confirmation_sensor_refresh
            <= self.elevator_opening_confirmation_retry_timeout
            and math.isfinite(self.elevator_local_crossing_timeout)
            and self.elevator_local_crossing_timeout > 0.0
            and math.isfinite(self.elevator_exit_empty_fallback_timeout)
            and 0.0 < self.elevator_exit_empty_fallback_timeout
            <= self.elevator_local_crossing_timeout
            and self.elevator_exit_empty_fallback_frames > 0
            and self.elevator_local_crossing_confirmations > 0
            and math.isfinite(self.elevator_local_crossing_confirmation_max_gap)
            and 0.0 < self.elevator_local_crossing_confirmation_max_gap
            <= self.elevator_local_crossing_timeout
            and math.isfinite(self.elevator_local_crossing_center_tolerance)
            and self.elevator_local_crossing_center_tolerance > 0.0
            and math.isfinite(self.elevator_local_crossing_width_tolerance)
            and self.elevator_local_crossing_width_tolerance > 0.0
            and math.isfinite(self.elevator_local_crossing_yaw_tolerance)
            and 0.0 < self.elevator_local_crossing_yaw_tolerance <= math.pi
            and math.isfinite(self.elevator_local_crossing_yaw_search_half_range)
            and self.elevator_local_crossing_yaw_search_half_range > 0.0
            and math.isfinite(self.elevator_local_crossing_yaw_search_step)
            and 0.0 < self.elevator_local_crossing_yaw_search_step
            <= 2.0 * self.elevator_local_crossing_yaw_search_half_range
            and math.isfinite(self.elevator_local_crossing_max_center_correction)
            and self.elevator_local_crossing_max_center_correction > 0.0
            and math.isfinite(self.elevator_geometry_refresh_timeout)
            and self.elevator_geometry_refresh_timeout > 0.0
            and math.isfinite(self.elevator_geometry_refresh_standoff)
            and 0.8 <= self.elevator_geometry_refresh_standoff <= 2.0
            and math.isfinite(self.elevator_geometry_refresh_max_shift)
            and self.elevator_geometry_refresh_max_shift > 0.0
            and math.isfinite(self.elevator_geometry_refresh_max_yaw_change)
            and 0.0 < self.elevator_geometry_refresh_max_yaw_change <= math.pi
            and math.isfinite(self.elevator_visual_scan_duration)
            and self.elevator_visual_scan_duration > 0.0
            and math.isfinite(self.elevator_visual_scan_angular_speed)
            and self.elevator_visual_scan_angular_speed > 0.0
            and math.isfinite(self.elevator_visual_scan_required_rotation)
            and 0.0 < self.elevator_visual_scan_required_rotation <= 2.0 * math.pi
        ):
            raise ValueError("RGB-D 电梯门检测参数必须为有限正值且宽度范围有效")
        if self.runtime_profile == "formal" and self.local_detection_params[
            "use_truth_proximity_detector"
        ]:
            raise ValueError("正式运行禁止启用 truth_proximity_debug 门检测回退")
        self.tracker_params = {
            "inspection_max_attempts": _param("tracker/inspection_max_attempts", 3),
            "association_radius": float(_param("tracker/association_radius", 0.45)),
            "confirmation_window": float(_param("tracker/confirmation_window", 10.0)),
            "min_hits": int(_param("tracker/min_hits", 4)),
            "max_position_std": float(_param("tracker/max_position_std", 0.25)),
            "min_confidence": float(_param("tracker/min_confidence", 0.75)),
            "max_average_rmse": float(_param("tracker/max_average_rmse", 0.04)),
            "nms_radius": float(_param("tracker/nms_radius", 0.9)),
            "stale_after": float(_param("tracker/stale_after", 30.0)),
            "obstacle_min_hits": int(
                _param("tracker/obstacle_min_hits", 2)
            ),
            "obstacle_min_confidence": float(
                _param("tracker/obstacle_min_confidence", 0.90)
            ),
            "obstacle_max_average_rmse": float(
                _param("tracker/obstacle_max_average_rmse", 0.025)
            ),
            "obstacle_max_age": float(
                _param("tracker/obstacle_max_age", 0.75)
            ),
            "obstacle_min_observation_span": float(
                _param("tracker/obstacle_min_observation_span", 0.08)
            ),
        }
        if self.runtime_profile == "formal":
            elevator_id = next(iter(self.building_features.elevator_features))
        else:
            elevator_id = str(_param("floor/elevator_id", "elevator_main")).strip()
        self.floor_params = {
            "elevator_id": elevator_id,
            "service_timeout": float(_param("floor/service_timeout", 5.0)),
        }



def load_mission_config(
    parameter_getter: ParameterGetter | None = None,
) -> MissionConfig:
    """读取并完整校验任务配置，不产生 ROS 发布/订阅副作用。"""
    builder = _MissionConfigBuilder(parameter_getter or ros_parameter)
    builder._populate()
    values = {
        name: value
        for name, value in vars(builder).items()
        if name != "_parameter_getter"
    }
    frozen_values = MappingProxyType(dict(values))
    sections = _group_config(frozen_values)
    return MissionConfig(values=frozen_values, **sections)

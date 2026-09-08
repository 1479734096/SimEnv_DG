"""任务运行组件的唯一装配边界。

本模块只负责依据已经校验的 ``MissionConfig`` 构造长期运行组件。任务状态机
不再知道各组件的构造参数，测试也可以注入同一形状的替身而不绕过构造器。
"""
from __future__ import annotations

from dataclasses import dataclass

from danger_search.building_control_client import BuildingControlClient
from danger_search.frontier import FrontierExplorer
from danger_search.floor_transition_runtime import FloorCoordinateModel
from danger_search.ground_support_coordinator import GroundSupportCoordinator
from danger_search.landmark_drift_monitor import LandmarkDriftMonitor
from danger_search.localization_runtime import LocalizationRuntime
from danger_search.map_update_runtime import MapUpdateRuntime
from danger_search.mission_config import MissionConfig
from danger_search.mission_reporting import MissionReportingRuntime
from danger_search.navigator import PathFollower
from danger_search.perception_runtime import PerceptionRuntime
from danger_search.perception_mapping_runtime import PerceptionMappingRuntime
from danger_search.pose_provider import PoseProvider
from danger_search.progress_monitor import ProgressMonitor
from danger_search.sparse_landmark_relocalizer import SparseLandmarkRelocalizer


@dataclass(frozen=True)
class MissionRuntimeComponents:
    """一次任务拥有的长期运行组件。

    字段冻结只禁止替换组件引用；组件内部仍按各自职责维护运行状态。
    """

    landmark_drift_monitor: LandmarkDriftMonitor
    landmark_relocalizer: SparseLandmarkRelocalizer
    reporting: MissionReportingRuntime
    perception: PerceptionRuntime
    pose_provider: PoseProvider
    localization: LocalizationRuntime
    map_runtime: MapUpdateRuntime
    explorer: FrontierExplorer
    controller: PathFollower
    ground_support: GroundSupportCoordinator
    building_client: BuildingControlClient
    progress_monitor: ProgressMonitor
    floor_coordinates: FloorCoordinateModel
    perception_mapping: PerceptionMappingRuntime

    @classmethod
    def build(
        cls,
        config: MissionConfig,
        *,
        start_time: float,
    ) -> "MissionRuntimeComponents":
        """按固定依赖顺序构造生产组件，不启动 ROS 订阅或定时器。"""
        runtime = config.runtime
        topics = config.topics
        localization_config = config.localization
        mapping = config.mapping
        elevator = config.elevator
        exploration = config.exploration
        return_home = config.return_home
        output = config.output
        landmark_drift_monitor = LandmarkDriftMonitor(
            localization_config["landmark_drift_config"]
        )
        landmark_relocalizer = SparseLandmarkRelocalizer(
            localization_config["sparse_relocalization_config"]
        )
        reporting = MissionReportingRuntime(
            result_file=output["result_file"],
            mission_summary_file=output["mission_summary_file"],
            status_topic=topics["status_topic"],
            map_topic=topics["map_topic"],
            frame_id=localization_config["global_frame_id"],
        )
        perception = PerceptionRuntime.from_topics(
            cloud_topic=topics["cloud_topic"],
            rgb_topic=topics["rgb_topic"],
            depth_topic=topics["depth_topic"],
            camera_info_topic=topics["camera_info_topic"],
        )
        pose_provider = PoseProvider(
            topics["localization_odom_topic"],
            expected_frame_id=localization_config.get("global_frame_id", "world"),
            correction_request_topic=topics[
                "localization_correction_request_topic"
            ],
            correction_state_topic=topics["localization_correction_state_topic"],
        )
        localization = LocalizationRuntime(
            pose_provider,
            stale_minimum=localization_config["localization_stale_minimum"],
            unavailable_timeout=localization_config[
                "localization_unavailable_timeout"
            ],
            start_time=float(start_time),
        )
        map_runtime = MapUpdateRuntime(
            map_params=mapping["map_params"],
            minimum_drop=mapping["ground_support_params"]["minimum_drop"],
            detection_params=mapping["detection_params"],
            tracker_params=mapping["tracker_params"],
            local_detection_params=mapping["local_detection_params"],
            rgbd_door_detection_params=mapping[
                "rgbd_door_detection_params"
            ],
            rgbd_door_detection_enabled=mapping[
                "rgbd_door_detection_enabled"
            ],
            max_floor=runtime["max_floor"],
        )
        explorer = FrontierExplorer(**exploration["frontier_params"])
        controller = PathFollower(
            cmd_vel_topic=topics["cmd_vel_topic"],
            path_raw_topic=topics["path_raw_topic"],
            path_followed_topic=topics["path_followed_topic"],
            local_target_topic=topics["local_target_topic"],
            path_frame_id=localization_config["global_frame_id"],
            **return_home["controller_params"],
        )
        ground_support = GroundSupportCoordinator(
            params=mapping["ground_support_params"],
            map_params=mapping["map_params"],
            cloud_max_points=mapping["cloud_max_points"],
            path_goal_tolerance=controller.goal_tolerance,
        )
        controller.set_command_gate(ground_support.gate)
        building_client = BuildingControlClient(
            service_timeout=elevator["floor_params"].get("service_timeout", 5.0),
        )
        building_features = runtime["building_features"]
        if building_features is not None:
            elevator_doors_by_floor: dict[int, list[str]] = {}
            for door in building_features.doors.values():
                if door.kind == "elevator":
                    elevator_doors_by_floor.setdefault(door.floor_index, []).append(
                        door.door_id
                    )
            building_client.set_public_features(
                door_ids=building_features.door_ids,
                elevators=building_features.elevators,
                elevator_door_ids_by_floor=elevator_doors_by_floor,
            )
        progress_monitor = ProgressMonitor(
            goal_tolerance=float(
                return_home["controller_params"].get("goal_tolerance", 0.35)
            ),
            window=exploration["progress_monitor_window"],
            repeat=exploration["progress_monitor_repeat"],
            min_displacement=exploration["progress_monitor_min_displacement"],
            cooldown=exploration["progress_monitor_cooldown"],
        )
        floor_coordinates = FloorCoordinateModel(
            localization_config["localization_vertical_reference"],
            mapping["map_params"]["floor_height"],
        )
        perception_mapping = PerceptionMappingRuntime(
            perception=perception,
            map_runtime=map_runtime,
            floor_coordinates=floor_coordinates,
            pose_provider=pose_provider,
        )
        return cls(
            landmark_drift_monitor=landmark_drift_monitor,
            landmark_relocalizer=landmark_relocalizer,
            reporting=reporting,
            perception=perception,
            pose_provider=pose_provider,
            localization=localization,
            map_runtime=map_runtime,
            explorer=explorer,
            controller=controller,
            ground_support=ground_support,
            building_client=building_client,
            progress_monitor=progress_monitor,
            floor_coordinates=floor_coordinates,
            perception_mapping=perception_mapping,
        )

from types import SimpleNamespace
from unittest.mock import patch
from dataclasses import FrozenInstanceError

from danger_search.mission_components import MissionRuntimeComponents
from danger_search.mission_config import load_mission_config
from danger_search.mission_manager import MissionManager
from danger_search.floor_transition_runtime import FloorCoordinateModel
from danger_search.perception_mapping_runtime import PerceptionMappingRuntime


def _diagnostic_config():
    return load_mission_config(
        lambda name, default=None: "diagnostic" if name == "runtime/profile" else default
    )


def _injected_components():
    localization = SimpleNamespace(
        pose_provider=None,
        unavailable_since=None,
        stale_reported=False,
        failure_reported=False,
        lookup_stats=lambda: {},
    )
    ground_support = SimpleNamespace(
        gate=None,
        grid=None,
        feedback=SimpleNamespace(),
        map_epoch=0,
        localization_epoch=0,
    )
    floor_coordinates = FloorCoordinateModel("floor_local", 2.6)
    map_runtime = SimpleNamespace(
        mapper=None,
        detector=None,
        local_detector=None,
        rgbd_door_detector=None,
        tracker=None,
    )
    perception = SimpleNamespace(sensor_buffer=SimpleNamespace())
    return MissionRuntimeComponents(
        landmark_drift_monitor=SimpleNamespace(),
        landmark_relocalizer=SimpleNamespace(),
        reporting=SimpleNamespace(),
        perception=perception,
        pose_provider=None,
        localization=localization,
        map_runtime=map_runtime,
        explorer=SimpleNamespace(),
        controller=SimpleNamespace(),
        ground_support=ground_support,
        building_client=SimpleNamespace(),
        progress_monitor=SimpleNamespace(),
        floor_coordinates=floor_coordinates,
        perception_mapping=PerceptionMappingRuntime(
            perception=perception,
            map_runtime=map_runtime,
            floor_coordinates=floor_coordinates,
            pose_provider=None,
        ),
    )


def test_mission_manager_binds_injected_components_without_rebuilding():
    config = _diagnostic_config()
    components = _injected_components()
    components.localization.pose_provider = components.pose_provider

    with patch.object(MissionRuntimeComponents, "build") as build:
        manager = MissionManager(config=config, components=components)

    build.assert_not_called()
    assert manager._components is components
    assert manager.controller is components.controller
    assert manager._map_runtime is components.map_runtime
    assert manager._ground_support is components.ground_support
    assert manager._active_elevator_id == config.values["floor_params"]["elevator_id"]


def test_runtime_component_root_is_frozen():
    components = _injected_components()
    try:
        components.controller = SimpleNamespace()
    except (AttributeError, FrozenInstanceError):
        pass
    else:
        raise AssertionError("组件装配根必须冻结长期组件引用")


def test_config_fields_are_partitioned_once_into_read_only_sections():
    config = _diagnostic_config()
    sections = (
        config.runtime,
        config.topics,
        config.localization,
        config.mapping,
        config.entry,
        config.elevator,
        config.exploration,
        config.return_home,
        config.output,
    )
    grouped_names = [name for section in sections for name in section.values]

    assert len(grouped_names) == len(set(grouped_names))
    assert set(grouped_names) == set(config.values)
    try:
        config.runtime.values["max_floor"] = 99
    except TypeError:
        pass
    else:
        raise AssertionError("配置分组必须只读")


def test_frontier_scoring_defaults_include_real_path_turn_penalty():
    config = _diagnostic_config()
    params = config.values["frontier_params"]

    assert params["yaw_cost_weight"] == 0.4
    assert params["path_turn_cost_weight"] == 0.7
    assert params["min_far_expected_gain"] == 16.0


def test_global_frame_is_wired_to_map_and_path_diagnostics():
    config = load_mission_config(
        lambda name, default=None: (
            "diagnostic"
            if name == "runtime/profile"
            else "danger_search_relative"
            if name == "localization/global_frame_id"
            else default
        )
    )

    components = MissionRuntimeComponents.build(config, start_time=0.0)

    assert components.reporting._frame_id == "danger_search_relative"
    assert components.controller.path_frame_id == "danger_search_relative"

import math
from types import SimpleNamespace

import pytest

from danger_search.landmark_drift_monitor import (
    LandmarkDriftConfig,
    LandmarkDriftMonitor,
)
from danger_search.floor_transition_runtime import FloorTransitionRuntime
from danger_search.elevator_observation_runtime import ElevatorObservationRuntime
from danger_search.mission_manager import EXPLORE_FLOOR, MissionManager
from danger_search.mission_config import load_mission_config
from danger_search.rgbd_door_detector import DoorPanelObservation


def _observation(stamp: float, x: float) -> DoorPanelObservation:
    return DoorPanelObservation(
        stamp=stamp,
        distance=1.0,
        world_xy=(x, 2.0),
        outward_yaw=math.pi,
        width=0.8,
        height=1.8,
        plane_rmse=0.01,
        depth_inlier_count=400,
        bbox=(0, 0, 10, 10),
        median_depth=1.0,
        center_dark_fraction=0.9,
    )


def _make_manager() -> MissionManager:
    config = load_mission_config(
        lambda name, default=None: (
            "diagnostic" if name == "runtime/profile" else default
        )
    )
    return MissionManager(config=config)


def _register_geometry(manager, feature, **kwargs):
    result = manager._elevator_observation.registration.register_confirmed_geometry(
        elevator_feature=feature,
        current_floor=manager.current_floor,
        building_client=manager.building_client,
        landmark_monitor=manager._landmark_drift_monitor,
        elapsed=manager._elapsed(),
        **kwargs,
    )
    manager._apply_elevator_registration_result(result)


def test_geometry_refresh_does_not_overwrite_frozen_landmark_anchor():
    manager = _make_manager()
    manager._landmark_drift_monitor = LandmarkDriftMonitor(
        LandmarkDriftConfig(enabled=True)
    )
    manager._elevator_observation = ElevatorObservationRuntime()
    manager._elevator_observation.registration.latest_rgbd_door_stamp = 10.0
    manager.current_floor = 0
    manager.floor_params = {}
    manager.building_client = SimpleNamespace(
        elevators={},
        doors={},
        elevator_door_id=lambda floor: f"elevator_door_{floor}",
    )
    events = []
    manager._log_structured = events.append
    manager._elapsed = lambda: 2.0
    feature = SimpleNamespace(
        elevator_id="elevator_main",
        served_floors=(0, 1),
    )

    _register_geometry(
        manager,
        feature,
        current_door_id="elevator_door_0",
        world_xy=(2.0, 2.0),
        outward_yaw=math.pi,
        width=0.8,
        source="rgbd_initial",
        evidence_count=400,
    )
    manager._elevator_observation.registration.latest_rgbd_door_stamp = 20.0
    _register_geometry(
        manager,
        feature,
        current_door_id="elevator_door_1",
        world_xy=(1.4, 2.0),
        outward_yaw=math.pi,
        width=0.8,
        source="rgbd_refresh",
        evidence_count=400,
    )

    assert manager._landmark_drift_monitor.anchor.world_xy == (2.0, 2.0)
    assert [
        event["event"] for event in events
    ] == ["landmark_drift_anchor_frozen"]
    assert manager.building_client.doors["elevator_door_1"].x == 1.4


def test_diagnostic_observations_only_emit_logs_and_confirm_after_three_frames():
    manager = _make_manager()
    manager._landmark_drift_monitor = LandmarkDriftMonitor(
        LandmarkDriftConfig(enabled=True)
    )
    manager._landmark_drift_monitor.freeze_anchor(
        stamp=10.0,
        floor=0,
        world_xy=(2.0, 2.0),
        outward_yaw=math.pi,
        width=0.8,
        source="rgbd",
    )
    manager.building_features = SimpleNamespace(
        elevator_features={"elevator_main": object()}
    )
    manager.state = EXPLORE_FLOOR
    manager.current_floor = 1
    manager.current_task = None
    manager._floor_transition_runtime = FloorTransitionRuntime()
    manager._elapsed = lambda: 3.0
    events = []
    manager._log_structured = events.append

    for index in range(3):
        stamp = 20.0 + 0.2 * index
        manager._update_landmark_drift_diagnostic(
            [_observation(stamp, 1.4)],
            image_stamp=stamp,
        )

    assert len(events) == 3
    assert events[-1]["event"] == "landmark_drift_observation"
    assert events[-1]["confirmed"]
    assert events[-1]["residual_xy"] == pytest.approx([-0.6, 0.0])
    assert manager.current_floor == 1
    assert manager.current_task is None


def test_disabled_diagnostic_has_no_log_or_state_side_effect():
    manager = _make_manager()
    manager._landmark_drift_monitor = LandmarkDriftMonitor(
        LandmarkDriftConfig(enabled=False)
    )
    events = []
    manager._log_structured = events.append

    manager._update_landmark_drift_diagnostic(
        [_observation(20.0, 1.4)],
        image_stamp=20.0,
    )

    assert events == []

import math
from types import SimpleNamespace

from danger_search.door_navigation_runtime import DoorNavigationRuntime


def test_record_passage_normalizes_sensor_direction_and_keeps_support_hold_enabled():
    runtime = DoorNavigationRuntime()

    recorded = runtime.record_passage(
        door_id="lift_door_1",
        door_xy=(1.0, 2.0),
        pass_vector=(-2.0, 0.0),
        pose=(0.8, 2.1, math.pi),
        now=100.0,
        transition_duration=1.5,
        context_duration=8.0,
    )

    assert recorded is True
    assert runtime.last_door_id == "lift_door_1"
    assert runtime.last_door_xy == (1.0, 2.0)
    assert runtime.last_pass_vector == (-1.0, 0.0)
    assert runtime.completion_pose == (0.8, 2.1, math.pi)
    assert runtime.transition_until == 101.5
    assert runtime.context_until == 108.0
    assert runtime.opening_sensor_confirmed is False


def test_normal_door_completion_still_records_original_debug_geometry():
    runtime = DoorNavigationRuntime()
    task = SimpleNamespace(
        door_id="room_door_1",
        wait_point=(0.0, -0.8),
        pass_point=(0.0, 0.8),
        commit_point=(0.0, 1.8),
        opening_sensor_confirmed=True,
    )

    recorded = runtime.record_completed(
        task,
        (0.0, 1.75, math.pi / 2.0),
        now=20.0,
        transition_duration=1.5,
        context_duration=8.0,
    )

    assert recorded is True
    assert runtime.last_door_id == "room_door_1"
    assert runtime.last_door_xy == (0.0, 0.0)
    assert runtime.last_pass_vector == (0.0, 1.0)
    assert runtime.opening_sensor_confirmed is True
    assert runtime.frontier_debug == {
        "door_id": "room_door_1",
        "wait_point": (0.0, -0.8),
        "pass_point": (0.0, 0.8),
        "commit_point": (0.0, 1.8),
        "completion_pose": (0.0, 1.75, math.pi / 2.0),
    }

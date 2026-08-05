import math

from danger_search.building_control_client import DoorInfo
from danger_search.door_observation_runtime import (
    ACCEPTED,
    PROCESS,
    REJECTED,
    TIMEOUT,
    WAITING,
    DoorObservationRuntime,
)
from danger_search.local_semantic_detector import OpeningObservation


def _door() -> DoorInfo:
    return DoorInfo("main_entrance", "main_entrance", 0, 0.0, 0.0, math.pi / 2.0, 1.8)


def _observation(*, distance=0.8, x=0.0, y=0.0, yaw=math.pi / 2.0, width=1.8):
    return OpeningObservation(distance, (x, y), yaw, width, 20)


def test_initial_confirmation_requires_new_consistent_stamps():
    tracker = DoorObservationRuntime().initial
    first = _observation(x=0.1)
    second = _observation(x=0.12)

    assert not tracker.observe(
        stamp=1.0,
        observation=first,
        maximum_distance=1.5,
        tolerance=0.3,
        required_confirmations=2,
    )
    assert not tracker.observe(
        stamp=1.0,
        observation=second,
        maximum_distance=1.5,
        tolerance=0.3,
        required_confirmations=2,
    )
    assert tracker.observe(
        stamp=1.1,
        observation=second,
        maximum_distance=1.5,
        tolerance=0.3,
        required_confirmations=2,
    )
    assert tracker.confirmations == 2


def test_refresh_precheck_preserves_duplicate_stamp_and_times_out():
    tracker = DoorObservationRuntime().refresh
    assert tracker.precheck(now=10.0, stamp=1.0, timeout=3.0) == PROCESS
    assert tracker.precheck(now=10.1, stamp=1.0, timeout=3.0) == WAITING
    assert tracker.precheck(now=13.1, stamp=1.1, timeout=3.0) == TIMEOUT


def test_refresh_classifies_rejections_without_accepting_geometry():
    tracker = DoorObservationRuntime().refresh
    existing = _door()
    cases = (
        (None, "no_candidate"),
        (_observation(distance=1.6), "distance_exceeded"),
        (_observation(width=0.9), "width_insufficient"),
        (_observation(yaw=0.0), "axis_mismatch"),
        (_observation(x=0.8), "center_shift_exceeded"),
    )
    for observation, reason in cases:
        result = tracker.assess(
            observation=observation,
            existing=existing,
            maximum_distance=1.5,
            minimum_width=1.1,
            tolerance=0.3,
            maximum_center_shift=0.6,
            required_confirmations=2,
        )
        assert result.status == REJECTED
        assert result.reason == reason
    assert tracker.rejection_counts == {reason: 1 for _observation, reason in cases}


def test_refresh_accepts_two_consistent_observations_and_keeps_summary():
    tracker = DoorObservationRuntime().refresh
    existing = _door()
    first = _observation(x=0.1, width=1.3)
    second = _observation(x=0.12, width=1.32)

    result = tracker.assess(
        observation=first,
        existing=existing,
        maximum_distance=1.5,
        minimum_width=1.1,
        tolerance=0.3,
        maximum_center_shift=0.6,
        required_confirmations=2,
    )
    assert result.status == WAITING
    result = tracker.assess(
        observation=second,
        existing=existing,
        maximum_distance=1.5,
        minimum_width=1.1,
        tolerance=0.3,
        maximum_center_shift=0.6,
        required_confirmations=2,
    )
    assert result.status == ACCEPTED
    assert result.confirmations == 2
    assert tracker.started_at is None
    assert tracker.diagnostic_snapshot(door_id="main_entrance")["last_candidate"]["width"] == 1.32


def test_refresh_query_yaw_faces_registered_center_from_both_sides():
    existing = _door()
    from_south = DoorObservationRuntime().refresh.query_yaw(existing, (0.0, -1.0, 0.0))
    from_north = DoorObservationRuntime().refresh.query_yaw(existing, (0.0, 1.0, 0.0))
    assert abs(from_south - math.pi / 2.0) < 1e-9
    assert abs(abs(from_north - from_south) - math.pi) < 1e-9

import math

import pytest

from danger_search.landmark_drift_monitor import (
    LandmarkDriftConfig,
    LandmarkDriftMonitor,
)
from danger_search.rgbd_door_detector import DoorPanelObservation


def _observation(
    stamp: float,
    *,
    x: float = 1.4,
    y: float = 2.0,
    yaw: float = math.pi,
    width: float = 0.8,
    distance: float = 1.2,
    state: str = "closed",
) -> DoorPanelObservation:
    return DoorPanelObservation(
        stamp=stamp,
        distance=distance,
        world_xy=(x, y),
        outward_yaw=yaw,
        width=width,
        height=1.8,
        plane_rmse=0.01,
        depth_inlier_count=500,
        bbox=(1, 2, 3, 4),
        median_depth=distance,
        center_dark_fraction=0.9,
        panel_state=state,
    )


def _monitor(**overrides) -> LandmarkDriftMonitor:
    values = dict(enabled=True)
    values.update(overrides)
    monitor = LandmarkDriftMonitor(LandmarkDriftConfig(**values))
    assert monitor.freeze_anchor(
        stamp=1.0,
        floor=0,
        world_xy=(2.0, 2.0),
        outward_yaw=math.pi,
        width=0.8,
        source="rgbd",
    )
    return monitor


def test_disabled_monitor_never_freezes_or_evaluates():
    monitor = LandmarkDriftMonitor(LandmarkDriftConfig(enabled=False))

    assert not monitor.freeze_anchor(
        stamp=1.0,
        floor=0,
        world_xy=(2.0, 2.0),
        outward_yaw=0.0,
        width=0.8,
        source="rgbd",
    )
    result = monitor.observe(
        [_observation(2.0)],
        stamp=2.0,
        floor=1,
        ordinary_exploration=True,
        elevator_count=1,
    )

    assert result.status == "disabled"
    assert monitor.anchor is None


def test_anchor_is_frozen_once_and_refresh_cannot_overwrite_it():
    monitor = _monitor()

    assert not monitor.freeze_anchor(
        stamp=2.0,
        floor=0,
        world_xy=(8.0, 9.0),
        outward_yaw=0.0,
        width=1.0,
        source="refresh",
    )
    assert monitor.anchor.world_xy == (2.0, 2.0)
    assert monitor.anchor.source == "rgbd"


def test_three_consistent_frames_confirm_vector_and_door_normal_residual():
    monitor = _monitor()
    results = []
    for index, x in enumerate((1.41, 1.39, 1.40)):
        stamp = 2.0 + index * 0.2
        results.append(
            monitor.observe(
                [_observation(stamp, x=x)],
                stamp=stamp,
                floor=1,
                ordinary_exploration=True,
                elevator_count=1,
            )
        )

    assert [result.confirmations for result in results] == [1, 2, 3]
    assert results[-1].confirmed
    assert results[-1].status == "confirmed"
    assert results[-1].residual_xy == pytest.approx((-0.6, 0.0))
    assert results[-1].normal_residual == pytest.approx(0.6)
    assert results[-1].tangent_residual == pytest.approx(0.0)
    assert results[-1].sequence_start_stamp == pytest.approx(2.0)


def test_gap_or_residual_spread_restarts_confirmation_sequence():
    monitor = _monitor(maximum_residual_spread=0.15)
    first = monitor.observe(
        [_observation(2.0, x=1.4)],
        stamp=2.0,
        floor=1,
        ordinary_exploration=True,
        elevator_count=1,
    )
    spread = monitor.observe(
        [_observation(2.2, x=1.8)],
        stamp=2.2,
        floor=1,
        ordinary_exploration=True,
        elevator_count=1,
    )
    gap = monitor.observe(
        [_observation(3.0, x=1.8)],
        stamp=3.0,
        floor=1,
        ordinary_exploration=True,
        elevator_count=1,
    )

    assert first.confirmations == 1
    assert spread.confirmations == 1
    assert gap.confirmations == 1


@pytest.mark.parametrize(
    "observation",
    [
        _observation(2.0, width=0.4),
        _observation(2.0, distance=4.1),
        _observation(2.0, yaw=2.0),
        _observation(2.0, width=1.2),
        _observation(2.0, state="open"),
    ],
)
def test_quality_and_identity_mismatch_are_rejected(observation):
    monitor = _monitor()

    result = monitor.observe(
        [observation],
        stamp=2.0,
        floor=1,
        ordinary_exploration=True,
        elevator_count=1,
    )

    assert result.status == "no_eligible_observation"
    assert result.confirmations == 0


@pytest.mark.parametrize(
    ("floor", "ordinary_exploration", "elevator_count"),
    [(0, True, 1), (1, False, 1), (1, True, 0), (1, True, 2)],
)
def test_context_rejection_is_fail_closed(
    floor,
    ordinary_exploration,
    elevator_count,
):
    monitor = _monitor()

    result = monitor.observe(
        [_observation(2.0)],
        stamp=2.0,
        floor=floor,
        ordinary_exploration=ordinary_exploration,
        elevator_count=elevator_count,
    )

    assert result.status == "context_rejected"
    assert result.confirmations == 0


def test_multiple_panels_are_counted_and_nearest_frozen_panel_is_reported():
    monitor = _monitor()

    result = monitor.observe(
        [
            _observation(2.0, x=1.3, distance=1.0),
            _observation(2.0, x=1.9, distance=1.1),
        ],
        stamp=2.0,
        floor=1,
        ordinary_exploration=True,
        elevator_count=1,
    )

    assert result.candidate_count == 2
    assert result.residual_xy == pytest.approx((-0.1, 0.0))

import numpy as np
import pytest

from danger_search.entrance_geometry_corrector import (
    EntranceGeometryConfig,
    EntranceGeometryCorrector,
)


def _wall_points(shift=(0.0, 0.0)):
    rng = np.random.default_rng(8127)
    points = []
    for x in np.r_[np.linspace(-5.5, -1.1, 280), np.linspace(1.1, 5.5, 280)]:
        for y in (0.0, 0.09, 0.18):
            points.append(
                (
                    float(x) + shift[0],
                    float(y) + shift[1],
                    float(rng.uniform(0.3, 1.8)),
                )
            )
    return points


def test_fixed_entrance_geometry_recovers_xy_without_yaw_change():
    corrector = EntranceGeometryCorrector(
        EntranceGeometryConfig(minimum_points=40, minimum_uniqueness_margin=0.02)
    )
    result = corrector.evaluate(
        [_wall_points((0.40, -0.20)) for _ in range(3)],
        floor_height=2.6,
    )

    assert result.status == "accepted"
    assert result.correction_xy == pytest.approx((-0.40, 0.20), abs=0.08)
    assert result.correction_norm < 0.50
    assert result.residual_p70 < 0.12


def test_fixed_entrance_geometry_recovers_bounded_yaw():
    reference = np.asarray(_wall_points(), dtype=float)
    yaw = 0.06
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    rotation = np.asarray(((cosine, -sine), (sine, cosine)))
    translation = np.asarray((0.12, -0.08))
    current = reference.copy()
    current[:, :2] = (current[:, :2] - translation) @ rotation.T
    corrector = EntranceGeometryCorrector(
        EntranceGeometryConfig(minimum_points=40, minimum_uniqueness_margin=0.02)
    )
    result = corrector.evaluate(
        [current.tolist() for _ in range(3)],
        floor_height=2.6,
    )

    assert result.status == "accepted"
    assert result.correction_yaw == pytest.approx(-yaw, abs=0.021)
    assert result.residual_improvement >= 0.03
    assert result.residual_p70 < 0.12


def test_inconsistent_frames_are_rejected():
    corrector = EntranceGeometryCorrector(
        EntranceGeometryConfig(minimum_points=40, maximum_frame_spread=0.05)
    )
    result = corrector.evaluate(
        [
            _wall_points((0.10, 0.0)),
            _wall_points((0.35, -0.1)),
            _wall_points((-0.15, 0.1)),
        ],
        floor_height=2.6,
    )

    assert result.status == "frame_spread_exceeded"


def test_small_error_is_verified_noop():
    corrector = EntranceGeometryCorrector(
        EntranceGeometryConfig(minimum_points=40)
    )
    result = corrector.evaluate(
        [_wall_points((0.03, -0.02)) for _ in range(3)],
        floor_height=2.6,
    )

    assert result.status == "no_correction_needed"


class _Controller:
    def __init__(self):
        self.reasons = []

    def stop(self, reason=None):
        self.reasons.append(reason)

    def clear_path(self, reason=None):
        self.reasons.append(reason)


class _PoseProvider:
    def __init__(self, ack=None):
        self.ack = ack
        self.requests = 0

    def correction_ack(self, sequence):
        return self.ack

    def request_planar_correction(self, correction_xy, **kwargs):
        self.requests += 1
        return 4


def test_correction_timeout_has_single_bounded_fallback():
    corrector = EntranceGeometryCorrector(
        EntranceGeometryConfig(minimum_points=40, timeout=1.0)
    )
    controller = _Controller()
    pose_provider = _PoseProvider()
    first = corrector.tick(
        "startup",
        now=0.0,
        frames=[],
        latest_cloud_stamp=0.0,
        floor_height=2.6,
        pose_provider=pose_provider,
        controller=controller,
        map_runtime=object(),
        robot_pose=(0.0, -3.2, 1.57),
        robot_floor=0,
    )
    second = corrector.tick(
        "startup",
        now=1.1,
        frames=[],
        latest_cloud_stamp=0.0,
        floor_height=2.6,
        pose_provider=pose_provider,
        controller=controller,
        map_runtime=object(),
        robot_pose=(0.0, -3.2, 1.57),
        robot_floor=0,
    )
    third = corrector.tick(
        "startup",
        now=1.2,
        frames=[],
        latest_cloud_stamp=0.0,
        floor_height=2.6,
        pose_provider=pose_provider,
        controller=controller,
        map_runtime=object(),
        robot_pose=(0.0, -3.2, 1.57),
        robot_floor=0,
    )

    assert first.status == "running"
    assert second.status == "done"
    assert third.status == "done"
    assert pose_provider.requests == 0


def test_accepted_correction_waits_for_ack_then_clears_path():
    corrector = EntranceGeometryCorrector(
        EntranceGeometryConfig(minimum_points=40, minimum_uniqueness_margin=0.02)
    )
    controller = _Controller()
    pose_provider = _PoseProvider(ack=None)
    map_runtime = type("MapRuntime", (), {"apply_planar_correction": lambda *args, **kwargs: None})()
    frames = [_wall_points((0.30, -0.12)) for _ in range(3)]
    requested = corrector.tick(
        "startup",
        now=1.0,
        frames=frames,
        latest_cloud_stamp=1.0,
        floor_height=2.6,
        pose_provider=pose_provider,
        controller=controller,
        map_runtime=map_runtime,
        robot_pose=(0.0, -3.2, 1.57),
        robot_floor=0,
    )
    assert requested.status == "running"
    assert pose_provider.requests == 1

    pose_provider.ack = (-0.30, 0.12, 1.0)
    applied = corrector.tick(
        "startup",
        now=1.1,
        frames=frames,
        latest_cloud_stamp=1.0,
        floor_height=2.6,
        pose_provider=pose_provider,
        controller=controller,
        map_runtime=map_runtime,
        robot_pose=(0.0, -3.2, 1.57),
        robot_floor=0,
    )
    assert applied.status == "done"
    assert applied.applied
    assert applied.correction_xy == (-0.30, 0.12)
    assert any("applied" in str(reason) for reason in controller.reasons)


def test_yaw_ack_is_forwarded_to_map_runtime():
    corrector = EntranceGeometryCorrector(
        EntranceGeometryConfig(minimum_points=40, minimum_uniqueness_margin=0.02)
    )
    controller = _Controller()
    pose_provider = _PoseProvider(ack=None)
    calls = []

    class MapRuntime:
        def apply_planar_correction(self, *args, **kwargs):
            calls.append((args, kwargs))

    frames = [_wall_points((0.30, -0.12)) for _ in range(3)]
    requested = corrector.tick(
        "startup",
        now=1.0,
        frames=frames,
        latest_cloud_stamp=1.0,
        floor_height=2.6,
        pose_provider=pose_provider,
        controller=controller,
        map_runtime=MapRuntime(),
        robot_pose=(0.0, -3.2, 1.57),
        robot_floor=0,
    )
    assert requested.status == "running"
    pose_provider.ack = (-0.30, 0.12, 0.04, 1.0)
    applied = corrector.tick(
        "startup",
        now=1.1,
        frames=frames,
        latest_cloud_stamp=1.0,
        floor_height=2.6,
        pose_provider=pose_provider,
        controller=controller,
        map_runtime=MapRuntime(),
        robot_pose=(0.0, -3.2, 1.57),
        robot_floor=0,
    )

    assert applied.applied
    assert applied.correction_yaw == pytest.approx(0.04)
    assert calls[0][1]["correction_yaw"] == pytest.approx(0.04)

import math

import numpy as np
import pytest

from danger_search.sparse_landmark_relocalizer import (
    SparseLandmarkRelocalizer,
    SparseRelocalizationConfig,
)


def _structure(shift=(0.0, 0.0)):
    rng = np.random.default_rng(8127)
    points = []
    for _ in range(900):
        points.append(
            (
                4.0 + shift[0],
                float(rng.uniform(-4.0, 4.0)) + shift[1],
                float(rng.uniform(0.3, 1.8)),
            )
        )
    for _ in range(700):
        points.append(
            (
                float(rng.uniform(-3.3, 4.0)) + shift[0],
                3.0 + shift[1],
                float(rng.uniform(0.3, 1.8)),
            )
        )
    for center_x, center_y in ((-2.2, -1.4), (1.3, -2.6), (2.7, 1.1)):
        for _ in range(180):
            points.append(
                (
                    center_x + float(rng.normal(0.0, 0.04)) + shift[0],
                    center_y + float(rng.normal(0.0, 0.04)) + shift[1],
                    float(rng.uniform(0.3, 1.8)),
                )
            )
    return points


def _relocalizer(**overrides):
    values = dict(
        minimum_points=80,
        minimum_overlap=0.45,
        minimum_uniqueness_margin=0.02,
        maximum_frame_spread=0.12,
    )
    values.update(overrides)
    return SparseLandmarkRelocalizer(SparseRelocalizationConfig(**values))


def test_recovers_translation_from_three_livox_frames():
    relocalizer = _relocalizer()
    anchor_frames = [_structure() for _ in range(3)]
    assert relocalizer.freeze_anchor(
        "elevator",
        floor=0,
        world_xy=(0.0, 0.0),
        outward_yaw=0.0,
        width=0.9,
        frames=anchor_frames,
        floor_height=2.6,
    )

    result = relocalizer.evaluate(
        "elevator",
        frames=[_structure(shift=(0.48, -0.31)) for _ in range(3)],
        floor_height=2.6,
    )

    assert result.status == "accepted"
    assert result.correction_xy == pytest.approx((-0.48, 0.31), abs=0.04)
    assert result.residual_p70 < 0.10


def test_anchor_is_frozen_once_and_dynamic_door_strip_is_masked():
    relocalizer = _relocalizer()
    frames = [_structure() for _ in range(3)]
    assert relocalizer.freeze_anchor(
        "main_entrance",
        floor=0,
        world_xy=(0.0, 0.0),
        outward_yaw=math.pi / 2.0,
        width=1.2,
        frames=frames,
        floor_height=2.6,
    )
    first = relocalizer.anchors["main_entrance"]

    assert not relocalizer.freeze_anchor(
        "main_entrance",
        floor=0,
        world_xy=(9.0, 9.0),
        outward_yaw=0.0,
        width=0.8,
        frames=frames,
        floor_height=2.6,
    )
    assert relocalizer.anchors["main_entrance"] is first
    assert not np.any(
        (np.abs(first.points[:, 1]) <= 0.70)
        & (np.abs(first.points[:, 0]) <= 1.0)
    )


def test_quality_gates_reject_inconsistent_frames_and_excessive_shift():
    relocalizer = _relocalizer(maximum_correction=0.8)
    assert relocalizer.freeze_anchor(
        "elevator",
        floor=0,
        world_xy=(0.0, 0.0),
        outward_yaw=0.0,
        width=0.9,
        frames=[_structure() for _ in range(3)],
        floor_height=2.6,
    )

    inconsistent = relocalizer.evaluate(
        "elevator",
        frames=[
            _structure(shift=(0.2, 0.0)),
            _structure(shift=(0.5, 0.0)),
            _structure(shift=(-0.1, 0.0)),
        ],
        floor_height=2.6,
    )
    excessive = relocalizer.evaluate(
        "elevator",
        frames=[_structure(shift=(1.0, 0.0)) for _ in range(3)],
        floor_height=2.6,
    )

    assert inconsistent.status == "frame_spread_exceeded"
    assert excessive.status in {"correction_exceeded", "match_unavailable"}


def test_small_residual_is_a_verified_noop():
    relocalizer = _relocalizer()
    assert relocalizer.freeze_anchor(
        "main_entrance",
        floor=0,
        world_xy=(0.0, 0.0),
        outward_yaw=0.0,
        width=1.0,
        frames=[_structure() for _ in range(3)],
        floor_height=2.6,
    )

    result = relocalizer.evaluate(
        "main_entrance",
        frames=[_structure(shift=(0.02, 0.0)) for _ in range(3)],
        floor_height=2.6,
    )

    assert result.status == "no_correction_needed"
    assert result.correction_norm < 0.08

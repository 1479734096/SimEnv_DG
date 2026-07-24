from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "offline" / "analyze_landmark_drift.py"
SPEC = importlib.util.spec_from_file_location("analyze_landmark_drift", SCRIPT)
analyzer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyzer)


def _observation(
    *,
    stamp: float,
    elapsed: float,
    residual_x: float,
    confirmed: bool = False,
    candidate_count: int = 1,
) -> dict[str, object]:
    return {
        "event": "landmark_drift_observation",
        "observation_stamp": stamp,
        "sequence_start_stamp": stamp - (0.4 if confirmed else 0.0),
        "elapsed": elapsed,
        "floor": 1,
        "candidate_count": candidate_count,
        "confirmed": confirmed,
        "residual_xy": [residual_x, 0.0],
    }


def _passing_records() -> tuple[list[dict[str, object]], list[dict[str, float]]]:
    records = [
        {
            "event": "landmark_drift_anchor_frozen",
            "observation_stamp": 10.0,
        }
    ]
    drift = [{"elapsed": 0.0, "error_x": 0.0, "error_y": 0.0}]
    for index in range(10):
        stamp = 10.1 + index * 0.1
        records.append(
            _observation(
                stamp=stamp,
                elapsed=1.0 + index * 0.1,
                residual_x=0.01,
            )
        )
        drift.append(
            {
                "elapsed": stamp - 10.0,
                "error_x": 0.01,
                "error_y": 0.0,
            }
        )
    records.append(
        _observation(
            stamp=12.0,
            elapsed=3.0,
            residual_x=0.50,
            confirmed=True,
        )
    )
    drift.append({"elapsed": 2.0, "error_x": 0.50, "error_y": 0.0})
    records.append(
        {
            "event": "mission_state_transition",
            "current_floor": 1,
            "new_state": "CHANGE_FLOOR",
            "reason_code": "floor_completion_ready",
            "elapsed": 5.0,
        }
    )
    return records, drift


def test_frozen_diagnostic_gate_passes_consistent_landmark():
    records, drift = _passing_records()

    result = analyzer.analyze(
        records,
        drift,
        first_aligned_estimate_stamp=10.0,
    )

    assert result["passed"]
    assert result["lead_time"] == 2.0
    assert result["confirmation_latency"] == pytest.approx(0.4)
    assert result["low_drift_observation_count"] == 10
    assert result["first_qualified"]["vector_error"] == 0.0


def test_multiple_panel_identity_blocks_admission():
    records, drift = _passing_records()
    records[-2]["candidate_count"] = 2

    result = analyzer.analyze(
        records,
        drift,
        first_aligned_estimate_stamp=10.0,
    )

    assert not result["passed"]
    assert result["qualified_count"] == 0
    assert not result["gates"]["single_panel_identity"]


def test_low_drift_large_residual_is_counted_as_false_confirmation():
    records, drift = _passing_records()
    records[1]["residual_xy"] = [0.40, 0.0]

    result = analyzer.analyze(
        records,
        drift,
        first_aligned_estimate_stamp=10.0,
    )

    assert not result["passed"]
    assert result["low_drift_false_confirmation_count"] == 1


def test_missing_anchor_is_explicit_failure():
    result = analyzer.analyze(
        [],
        [],
        first_aligned_estimate_stamp=10.0,
    )

    assert not result["passed"]
    assert result["reason"] == "anchor_missing"


def test_loader_ignores_non_json_and_corrupt_rows():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "mission.log"
        path.write_text(
            "普通日志\n"
            + json.dumps({"event": "landmark_drift_anchor_frozen"})
            + "\n{损坏\n",
            encoding="utf-8",
        )

        records = analyzer.load_structured_records(path)

    assert records == [{"event": "landmark_drift_anchor_frozen"}]

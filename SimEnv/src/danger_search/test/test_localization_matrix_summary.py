"""定位 seed 矩阵离线裁决测试。"""

import csv
import importlib.util
import json
from pathlib import Path
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "offline" / "summarize_localization_matrix.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("localization_matrix_summary", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载离线工具：{SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = _load_module()


def _row(seed, *, rmse=0.08, completed=True, localization_failure=False):
    return {
        "seed": seed,
        "valid_evidence": True,
        "mission_completed": completed,
        "localization_failure": localization_failure,
        "rms_position_error": rmse,
        "p95_position_error": 0.12,
        "maximum_position_error": 0.18,
        "final_position_error": 0.10,
        "maximum_absolute_yaw_error": 0.003,
        "odom_frequency_hz": 9.9,
        "maximum_output_gap": 0.2,
    }


def _write_evidence(directory, seed, *, completed=True, rmse=0.08):
    directory.mkdir(parents=True)
    (directory / "TEST_SUMMARY.txt").write_text(
        "\n".join(
            [
                "result=" + ("通过" if completed else "失败"),
                "detail=测试证据",
                f"seed={seed}",
                "floor_count=2",
                "floor_timeout=33",
                "min_floor_explore_time=5",
                "max_runtime=180",
                "full_cycle=1",
                "record_drift=1",
                "record_rgbd=0",
                "offline_truth=1",
                "localization_diagnostics=1",
                "localization_backend=adaptive_shared_map",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    result = {
        "event": "mission_result_write",
        "terminal_outcome": "success" if completed else "failed",
        "result_write_reason": "normal" if completed else "planner_failed",
        "return_home_completed": completed,
    }
    (directory / "mission.log").write_text(
        f"[INFO]: {json.dumps(result)}\n"
        "[INFO]: 共享地图 source=livox selected=(rgbd:12 livox:88) both_invalid=1\n",
        encoding="utf-8",
    )
    (directory / "localization_drift.json").write_text(
        json.dumps(
            {
                "sample_count": 3,
                "duration": 0.2,
                "truth_distance": 0.1,
                "rms_position_error": rmse,
                "p95_position_error": 0.12,
                "maximum_position_error": 0.18,
                "final_position_error": 0.1,
                "maximum_absolute_yaw_error": 0.003,
            }
        ),
        encoding="utf-8",
    )
    with (directory / "localization_drift.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["elapsed", "position_error"])
        writer.writerows([(0.0, 0.0), (0.1, rmse), (0.2, rmse)])


def test_accepts_eighteen_of_twenty_below_point_one():
    rows = [_row(seed, rmse=0.08 if seed < 18 else 0.12) for seed in range(20)]
    result = MODULE.evaluate_matrix(rows)
    assert result["verdict"] == "acceptable"
    assert result["rmse_below_010_count"] == 18


def test_clear_improvement_requires_sixteen_below_point_zero_five_and_all_below_point_one():
    rows = [_row(seed, rmse=0.04 if seed < 16 else 0.08) for seed in range(20)]
    result = MODULE.evaluate_matrix(rows)
    assert result["verdict"] == "clear_improvement"
    rows[-1]["rms_position_error"] = 0.12
    assert MODULE.evaluate_matrix(rows)["verdict"] == "acceptable"


def test_catastrophic_outlier_and_localization_failure_veto_promotion():
    rows = [_row(seed, rmse=0.08) for seed in range(20)]
    rows[-1]["rms_position_error"] = 0.20
    assert MODULE.evaluate_matrix(rows)["verdict"] == "rejected"
    rows[-1]["rms_position_error"] = 0.08
    rows[-1]["localization_failure"] = True
    assert MODULE.evaluate_matrix(rows)["verdict"] == "rejected"


def test_task_early_exit_is_not_counted_as_closed_loop():
    rows = [_row(seed, completed=seed < 17) for seed in range(20)]
    result = MODULE.evaluate_matrix(rows)
    assert result["mission_completed_count"] == 17
    assert result["verdict"] == "rejected"


def test_missing_evidence_keeps_matrix_incomplete():
    rows = [_row(seed) for seed in range(19)]
    rows.append({"seed": 19, "valid_evidence": False})
    result = MODULE.evaluate_matrix(rows)
    assert result["valid_run_count"] == 19
    assert result["verdict"] == "insufficient_evidence"


def test_loader_uses_latest_valid_attempt_and_extracts_diagnostics():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        invalid = root / "seed_42" / "attempt_1"
        invalid.mkdir(parents=True)
        (invalid / "TEST_SUMMARY.txt").write_text("seed=42\n", encoding="utf-8")
        valid = root / "seed_42" / "attempt_2"
        _write_evidence(valid, 42)

        evidence = MODULE.load_seed_evidence(root, 42, "adaptive_shared_map")

    assert evidence["valid_evidence"] is True
    assert evidence["mission_completed"] is True
    assert evidence["rgbd_selected"] == 12
    assert evidence["livox_selected"] == 88
    assert evidence["both_invalid"] == 1
    assert evidence["odom_frequency_hz"] == pytest.approx(10.0)
    assert evidence["maximum_output_gap"] == pytest.approx(0.1)


def test_mission_parser_keeps_multiple_structured_records_on_one_line():
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "mission.log"
        path.write_text(
            '[INFO] {"event":"mission_runtime"} trailing '
            '{"event":"mission_result_write","terminal_outcome":"success"}\n',
            encoding="utf-8",
        )

        records = MODULE.load_mission_records(path)

    assert [record["event"] for record in records] == [
        "mission_runtime",
        "mission_result_write",
    ]

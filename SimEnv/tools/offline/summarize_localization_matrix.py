#!/usr/bin/env python3
"""汇总定位 seed 矩阵并按预登记门槛裁决；只读取离线证据。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Iterable


DEFAULT_SEEDS = (
    8127,
    1612,
    8072,
    6946,
    4962,
    42,
    137,
    911,
    2027,
    3141,
    4099,
    5273,
    6203,
    7331,
    8459,
    9511,
    10657,
    11777,
    12907,
    14029,
)
EXPECTED_BACKEND = "adaptive_shared_map"
SELECTION_PATTERN = re.compile(
    r"selected=\(rgbd:(\d+) livox:(\d+)\) both_invalid=(\d+)"
)


def _finite(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_test_summary(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not path.is_file():
        return result
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            result[key.strip()] = value.strip()
    return result


def load_json(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def load_mission_records(path: Path) -> list[dict[str, object]]:
    if not path.is_file():
        return []
    decoder = json.JSONDecoder()
    records: list[dict[str, object]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        offset = 0
        while True:
            start = line.find("{", offset)
            if start < 0:
                break
            try:
                payload, consumed = decoder.raw_decode(line[start:])
            except json.JSONDecodeError:
                offset = start + 1
                continue
            if isinstance(payload, dict) and payload.get("event"):
                records.append(payload)
            offset = start + max(consumed, 1)
    return records


def output_continuity(path: Path) -> tuple[float | None, float | None]:
    if not path.is_file():
        return None, None
    elapsed: list[float] = []
    try:
        with path.open(encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                value = _finite(row.get("elapsed"))
                if value is not None:
                    elapsed.append(value)
    except (OSError, csv.Error):
        return None, None
    if len(elapsed) < 2 or elapsed[-1] <= elapsed[0]:
        return None, None
    gaps = [current - previous for previous, current in zip(elapsed, elapsed[1:])]
    if not gaps or any(gap <= 0.0 for gap in gaps):
        return None, None
    duration = elapsed[-1] - elapsed[0]
    return (len(elapsed) - 1) / duration, max(gaps)


def selection_counts(path: Path) -> tuple[int | None, int | None, int | None]:
    if not path.is_file():
        return None, None, None
    latest = None
    for match in SELECTION_PATTERN.finditer(
        path.read_text(encoding="utf-8", errors="replace")
    ):
        latest = match
    if latest is None:
        return None, None, None
    return tuple(int(value) for value in latest.groups())


def _configuration_matches(
    summary: dict[str, str], seed: int, backend: str
) -> bool:
    expected_strings = {
        "seed": str(seed),
        "localization_backend": backend,
        "floor_count": "2",
        "full_cycle": "1",
        "record_drift": "1",
        "record_rgbd": "0",
        "offline_truth": "1",
        "localization_diagnostics": "1",
    }
    if any(summary.get(key) != value for key, value in expected_strings.items()):
        return False
    expected_numbers = {
        "floor_timeout": 33.0,
        "min_floor_explore_time": 5.0,
        "max_runtime": 180.0,
    }
    return all(
        _finite(summary.get(key)) == value for key, value in expected_numbers.items()
    )


def analyze_run(run_dir: Path, seed: int, backend: str) -> dict[str, object]:
    summary = load_test_summary(run_dir / "TEST_SUMMARY.txt")
    drift = load_json(run_dir / "localization_drift.json")
    records = load_mission_records(run_dir / "mission.log")
    result_records = [
        record for record in records if record.get("event") == "mission_result_write"
    ]
    latest_result = result_records[-1] if result_records else None
    runtime_count = sum(record.get("event") == "mission_runtime" for record in records)
    terminal_outcome = (
        str(latest_result.get("terminal_outcome") or "unknown")
        if latest_result is not None
        else "unknown"
    )
    reason = (
        str(latest_result.get("result_write_reason") or "unknown")
        if latest_result is not None
        else ("process_exited_without_result" if runtime_count else "no_mission_evidence")
    )
    mission_completed = bool(
        latest_result is not None
        and terminal_outcome == "success"
        and latest_result.get("return_home_completed") is True
    )
    localization_failure = bool(
        terminal_outcome != "success"
        and (
            reason.startswith("localization_")
            or "localization_stance_unhealthy" in reason
        )
    )

    required_drift = {
        key: _finite(drift.get(key))
        for key in (
            "duration",
            "truth_distance",
            "rms_position_error",
            "p95_position_error",
            "maximum_position_error",
            "final_position_error",
            "maximum_absolute_yaw_error",
        )
    }
    sample_count = drift.get("sample_count")
    sample_count_valid = isinstance(sample_count, int) and sample_count >= 2
    frequency, maximum_gap = output_continuity(run_dir / "localization_drift.csv")
    rgbd_selected, livox_selected, both_invalid = selection_counts(
        run_dir / "mission.log"
    )
    configuration_matches = _configuration_matches(summary, seed, backend)
    has_mission_evidence = latest_result is not None or runtime_count > 0
    valid_evidence = bool(
        configuration_matches
        and has_mission_evidence
        and sample_count_valid
        and all(value is not None for value in required_drift.values())
        and frequency is not None
        and maximum_gap is not None
    )

    return {
        "seed": seed,
        "run_dir": str(run_dir.resolve()),
        "valid_evidence": valid_evidence,
        "configuration_matches": configuration_matches,
        "runner_result": summary.get("result", "missing"),
        "runner_detail": summary.get("detail", "missing"),
        "terminal_outcome": terminal_outcome,
        "reason": reason,
        "mission_completed": mission_completed,
        "localization_failure": localization_failure,
        "runtime_sample_count": runtime_count,
        "sample_count": sample_count if sample_count_valid else None,
        **required_drift,
        "odom_frequency_hz": frequency,
        "maximum_output_gap": maximum_gap,
        "rgbd_selected": rgbd_selected,
        "livox_selected": livox_selected,
        "both_invalid": both_invalid,
    }


def load_seed_evidence(root: Path, seed: int, backend: str) -> dict[str, object]:
    attempts = sorted((root / f"seed_{seed}").glob("attempt_*"))
    analyzed = [analyze_run(path, seed, backend) for path in attempts if path.is_dir()]
    valid = [row for row in analyzed if row["valid_evidence"]]
    if valid:
        return valid[-1]
    if analyzed:
        return analyzed[-1]
    return {
        "seed": seed,
        "run_dir": None,
        "valid_evidence": False,
        "configuration_matches": False,
        "terminal_outcome": "missing",
        "reason": "missing_run",
        "mission_completed": False,
        "localization_failure": False,
    }


def evaluate_matrix(rows: Iterable[dict[str, object]]) -> dict[str, object]:
    runs = list(rows)
    valid = [row for row in runs if row.get("valid_evidence") is True]
    rmse_below_005 = sum(
        float(row["rms_position_error"]) < 0.05 for row in valid
    )
    rmse_below_010 = sum(
        float(row["rms_position_error"]) < 0.10 for row in valid
    )
    completed = sum(row.get("mission_completed") is True for row in valid)
    localization_failures = sum(
        row.get("localization_failure") is True for row in valid
    )
    guards = {
        "all_rmse_below_020": all(
            float(row["rms_position_error"]) < 0.20 for row in valid
        ),
        "all_p95_below_020": all(
            float(row["p95_position_error"]) < 0.20 for row in valid
        ),
        "all_final_below_020": all(
            float(row["final_position_error"]) < 0.20 for row in valid
        ),
        "all_maximum_below_035": all(
            float(row["maximum_position_error"]) < 0.35 for row in valid
        ),
        "all_yaw_below_0005": all(
            float(row["maximum_absolute_yaw_error"]) < 0.005 for row in valid
        ),
        "all_frequency_at_least_8": all(
            float(row["odom_frequency_hz"]) >= 8.0 for row in valid
        ),
        "all_gaps_at_most_05": all(
            float(row["maximum_output_gap"]) <= 0.5 for row in valid
        ),
    }
    complete_matrix = len(runs) == 20 and len(valid) == 20
    acceptable = bool(
        complete_matrix
        and rmse_below_010 >= 18
        and completed >= 18
        and localization_failures == 0
        and all(guards.values())
    )
    clear_improvement = bool(
        acceptable and rmse_below_005 >= 16 and rmse_below_010 == 20
    )
    if not complete_matrix:
        verdict = "insufficient_evidence"
    elif clear_improvement:
        verdict = "clear_improvement"
    elif acceptable:
        verdict = "acceptable"
    else:
        verdict = "rejected"
    return {
        "verdict": verdict,
        "expected_run_count": 20,
        "valid_run_count": len(valid),
        "rmse_below_005_count": rmse_below_005,
        "rmse_below_010_count": rmse_below_010,
        "mission_completed_count": completed,
        "localization_failure_count": localization_failures,
        "guards": guards,
        "runs": runs,
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "seed",
        "valid_evidence",
        "terminal_outcome",
        "reason",
        "mission_completed",
        "localization_failure",
        "truth_distance",
        "rms_position_error",
        "p95_position_error",
        "maximum_position_error",
        "final_position_error",
        "maximum_absolute_yaw_error",
        "odom_frequency_hz",
        "maximum_output_gap",
        "rgbd_selected",
        "livox_selected",
        "both_invalid",
        "run_dir",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(path: Path, result: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# 定位 Seed 矩阵摘要",
        "",
        f"- 裁决：`{result['verdict']}`",
        f"- 有效证据：`{result['valid_run_count']}/20`",
        f"- RMSE < 0.05 m：`{result['rmse_below_005_count']}/20`",
        f"- RMSE < 0.10 m：`{result['rmse_below_010_count']}/20`",
        f"- 完整闭环：`{result['mission_completed_count']}/20`",
        f"- 定位失败：`{result['localization_failure_count']}`",
        "",
        "| seed | 有效 | 终态 | 原因 | 闭环 | RMSE | P95 | 末端 | yaw 最大 | Hz | 最大间隔 |",
        "|---:|:---:|---|---|:---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in result["runs"]:
        def show(key: str, digits: int = 4) -> str:
            value = _finite(row.get(key))
            return f"{value:.{digits}f}" if value is not None else "-"

        lines.append(
            "| {seed} | {valid} | {outcome} | {reason} | {completed} | "
            "{rmse} | {p95} | {final} | {yaw} | {hz} | {gap} |".format(
                seed=row["seed"],
                valid="是" if row.get("valid_evidence") else "否",
                outcome=row.get("terminal_outcome", "-"),
                reason=row.get("reason", "-"),
                completed="是" if row.get("mission_completed") else "否",
                rmse=show("rms_position_error"),
                p95=show("p95_position_error"),
                final=show("final_position_error"),
                yaw=show("maximum_absolute_yaw_error", 5),
                hz=show("odom_frequency_hz", 2),
                gap=show("maximum_output_gap", 3),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", type=Path)
    parser.add_argument("--seeds", nargs="+", type=int, default=list(DEFAULT_SEEDS))
    parser.add_argument("--backend", default=EXPECTED_BACKEND)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--validate-run", type=Path)
    parser.add_argument("--seed", type=int)
    args = parser.parse_args()

    if args.validate_run is not None:
        if args.seed is None:
            parser.error("--validate-run 必须同时提供 --seed")
        row = analyze_run(args.validate_run, args.seed, args.backend)
        print(json.dumps(row, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if row["valid_evidence"] else 1

    if args.root is None:
        parser.error("缺少矩阵证据根目录")
    if len(args.seeds) != 20 or len(set(args.seeds)) != 20:
        parser.error("矩阵必须包含 20 个互不重复的 seed")
    rows = [load_seed_evidence(args.root, seed, args.backend) for seed in args.seeds]
    result = evaluate_matrix(rows)
    json_path = args.json or args.root / "matrix_summary.json"
    csv_path = args.csv or args.root / "matrix_summary.csv"
    markdown_path = args.markdown or args.root / "matrix_summary.md"
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_csv(csv_path, rows)
    write_markdown(markdown_path, result)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

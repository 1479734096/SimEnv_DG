#!/usr/bin/env python3
"""离线评估电梯地标残差能否解释定位平移漂移。

生产任务只记录合法 RGB-D 派生残差。本工具在任务结束后读取独立真值生成的
漂移 CSV；输出不得回流到同一轮生产决策。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable


def load_structured_records(path: Path) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            start = line.find("{")
            if start < 0:
                continue
            try:
                payload = json.loads(line[start:])
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict):
                records.append(payload)
    return records


def load_drift_samples(path: Path) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                sample = {
                    "elapsed": float(row["elapsed"]),
                    "error_x": float(row["error_x"]),
                    "error_y": float(row["error_y"]),
                }
            except (KeyError, TypeError, ValueError):
                continue
            if all(math.isfinite(value) for value in sample.values()):
                samples.append(sample)
    samples.sort(key=lambda item: item["elapsed"])
    return samples


def _nearest_sample(
    samples: list[dict[str, float]],
    elapsed: float,
    maximum_gap: float,
) -> dict[str, float] | None:
    if not samples:
        return None
    sample = min(samples, key=lambda item: abs(item["elapsed"] - elapsed))
    if abs(sample["elapsed"] - elapsed) > maximum_gap:
        return None
    return sample


def _pair(value: object) -> tuple[float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        pair = (float(value[0]), float(value[1]))
    except (TypeError, ValueError):
        return None
    return pair if all(math.isfinite(item) for item in pair) else None


def _finite(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _integer(value: object, default: int = -1) -> int:
    parsed = _finite(value)
    if parsed is None or not parsed.is_integer():
        return default
    return int(parsed)


def _direction_cosine(
    left: tuple[float, float],
    right: tuple[float, float],
) -> float | None:
    left_norm = math.hypot(*left)
    right_norm = math.hypot(*right)
    if left_norm <= 1e-9 or right_norm <= 1e-9:
        return None
    return (
        left[0] * right[0] + left[1] * right[1]
    ) / (left_norm * right_norm)


def _distribution(values: Iterable[float]) -> dict[str, float] | None:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None

    def percentile(fraction: float) -> float:
        return ordered[int(round((len(ordered) - 1) * fraction))]

    return {
        "minimum": ordered[0],
        "p50": percentile(0.50),
        "p95": percentile(0.95),
        "maximum": ordered[-1],
    }


def _first_consequence_elapsed(
    records: Iterable[dict[str, object]],
    *,
    floor: int,
) -> float | None:
    values = []
    for record in records:
        if record.get("event") != "mission_state_transition":
            continue
        if _integer(record.get("current_floor")) != int(floor):
            continue
        reason = str(record.get("reason_code") or "")
        if (
            "elevator_trace_approach_exhausted" in reason
            or record.get("new_state") in {"CHANGE_FLOOR", "RETURN_HOME"}
        ):
            elapsed = _finite(record.get("elapsed"))
            if elapsed is not None:
                values.append(elapsed)
    return min(values) if values else None


def analyze(
    records: Iterable[dict[str, object]],
    drift_samples: list[dict[str, float]],
    *,
    first_aligned_estimate_stamp: float,
    maximum_alignment_gap: float = 0.15,
    high_drift_threshold: float = 0.30,
    minimum_residual: float = 0.25,
    maximum_vector_error: float = 0.15,
    minimum_direction_cosine: float = 0.90,
    low_drift_threshold: float = 0.15,
    minimum_low_observations: int = 10,
    minimum_lead_time: float = 1.0,
    maximum_confirmation_latency: float = 1.0,
) -> dict[str, object]:
    ordered = list(records)
    anchors = [
        record
        for record in ordered
        if record.get("event") == "landmark_drift_anchor_frozen"
    ]
    observations = [
        record
        for record in ordered
        if record.get("event") == "landmark_drift_observation"
    ]
    if not anchors:
        return {
            "passed": False,
            "reason": "anchor_missing",
            "anchor_count": 0,
            "observation_count": len(observations),
        }
    anchor = anchors[0]
    anchor_stamp = _finite(anchor.get("observation_stamp"))
    if anchor_stamp is None:
        return {
            "passed": False,
            "reason": "anchor_stamp_invalid",
            "anchor_count": len(anchors),
            "observation_count": len(observations),
        }
    anchor_sample = _nearest_sample(
        drift_samples,
        anchor_stamp - first_aligned_estimate_stamp,
        maximum_alignment_gap,
    )
    if anchor_sample is None:
        return {
            "passed": False,
            "reason": "anchor_truth_alignment_missing",
            "anchor_count": len(anchors),
            "observation_count": len(observations),
        }

    evaluated = []
    for record in observations:
        stamp = _finite(record.get("observation_stamp"))
        residual = _pair(record.get("residual_xy"))
        if stamp is None or residual is None:
            continue
        sample = _nearest_sample(
            drift_samples,
            stamp - first_aligned_estimate_stamp,
            maximum_alignment_gap,
        )
        if sample is None:
            continue
        truth_change = (
            sample["error_x"] - anchor_sample["error_x"],
            sample["error_y"] - anchor_sample["error_y"],
        )
        vector_error = math.hypot(
            residual[0] - truth_change[0],
            residual[1] - truth_change[1],
        )
        evaluated.append(
            {
                "stamp": stamp,
                "elapsed": _finite(record.get("elapsed")),
                "sequence_start_stamp": _finite(
                    record.get("sequence_start_stamp")
                ),
                "floor": _integer(record.get("floor")),
                "confirmed": bool(record.get("confirmed", False)),
                "candidate_count": _integer(record.get("candidate_count"), 0),
                "residual_xy": list(residual),
                "residual_norm": math.hypot(*residual),
                "truth_drift_change_xy": list(truth_change),
                "truth_drift_change_norm": math.hypot(*truth_change),
                "vector_error": vector_error,
                "direction_cosine": _direction_cosine(residual, truth_change),
            }
        )

    confirmed = [item for item in evaluated if item["confirmed"]]
    low_observations = [
        item
        for item in evaluated
        if item["truth_drift_change_norm"] <= low_drift_threshold
    ]
    low_false = [
        item for item in low_observations if item["residual_norm"] >= high_drift_threshold
    ]
    high_confirmed = [
        item
        for item in confirmed
        if item["truth_drift_change_norm"] >= high_drift_threshold
    ]
    qualifying = [
        item
        for item in high_confirmed
        if item["residual_norm"] >= minimum_residual
        and item["vector_error"] <= maximum_vector_error
        and item["direction_cosine"] is not None
        and item["direction_cosine"] >= minimum_direction_cosine
        and item["candidate_count"] == 1
    ]
    consequence_elapsed = (
        _first_consequence_elapsed(
            ordered,
            floor=int(qualifying[0]["floor"]),
        )
        if qualifying
        else None
    )
    lead_time = None
    if consequence_elapsed is not None and qualifying[0]["elapsed"] is not None:
        lead_time = consequence_elapsed - float(qualifying[0]["elapsed"])
    confirmation_latency = None
    if qualifying and qualifying[0]["sequence_start_stamp"] is not None:
        confirmation_latency = (
            float(qualifying[0]["stamp"])
            - float(qualifying[0]["sequence_start_stamp"])
        )

    gates = {
        "high_drift_reproduced": bool(high_confirmed),
        "qualified_landmark_before_consequence": bool(
            qualifying
            and lead_time is not None
            and lead_time >= minimum_lead_time
        ),
        "low_drift_sample_count": len(low_observations) >= minimum_low_observations,
        "zero_low_drift_false_confirmations": len(low_false) == 0,
        "confirmation_latency": bool(
            confirmation_latency is not None
            and confirmation_latency <= maximum_confirmation_latency
        ),
        "single_panel_identity": bool(
            qualifying and int(qualifying[0]["candidate_count"]) == 1
        ),
    }
    return {
        "passed": all(gates.values()),
        "reason": "passed" if all(gates.values()) else "diagnostic_gate_failed",
        "anchor_count": len(anchors),
        "observation_count": len(observations),
        "evaluated_observation_count": len(evaluated),
        "confirmed_count": len(confirmed),
        "high_drift_confirmed_count": len(high_confirmed),
        "qualified_count": len(qualifying),
        "low_drift_observation_count": len(low_observations),
        "low_drift_false_confirmation_count": len(low_false),
        "confirmed_distributions": {
            "residual_norm": _distribution(
                item["residual_norm"] for item in confirmed
            ),
            "truth_drift_change_norm": _distribution(
                item["truth_drift_change_norm"] for item in confirmed
            ),
            "vector_error": _distribution(
                item["vector_error"] for item in confirmed
            ),
        },
        "consequence_elapsed": consequence_elapsed,
        "lead_time": lead_time,
        "confirmation_latency": confirmation_latency,
        "gates": gates,
        "first_qualified": qualifying[0] if qualifying else None,
        "best_confirmed": (
            min(confirmed, key=lambda item: item["vector_error"])
            if confirmed
            else None
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="离线评估电梯门板地标残差与定位真值漂移增量",
    )
    parser.add_argument("mission_log", type=Path)
    parser.add_argument("drift_csv", type=Path)
    parser.add_argument("drift_json", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    for path in (args.mission_log, args.drift_csv, args.drift_json):
        if not path.is_file():
            parser.error(f"输入不存在：{path}")
    drift_summary = json.loads(args.drift_json.read_text(encoding="utf-8"))
    origin = _finite(drift_summary.get("first_aligned_estimate_stamp"))
    if origin is None:
        parser.error("漂移摘要缺少 first_aligned_estimate_stamp")
    result = analyze(
        load_structured_records(args.mission_log),
        load_drift_samples(args.drift_csv),
        first_aligned_estimate_stamp=origin,
    )
    payload = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if args.json is not None:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())

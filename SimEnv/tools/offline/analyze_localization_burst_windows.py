#!/usr/bin/env python3
"""跨运行验证 RGB-D/Livox 短窗分歧的离线区分能力。"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
import random
from typing import Iterable, Sequence


MINIMUM_TRUTH_DISTANCE = 0.02
MINIMUM_SENSOR_COVERAGE = 0.60
LOW_ERROR_MAXIMUM = 0.02
HIGH_ERROR_MINIMUM = 0.05
MINIMUM_CLASS_COUNT = 8
MINIMUM_AUC = 0.70
MINIMUM_BOOTSTRAP_LOWER = 0.55
MINIMUM_LIVOX_RMSE_IMPROVEMENT = 0.15
BOOTSTRAP_BLOCK_SIZE = 5
BOOTSTRAP_REPETITIONS = 1000
BOOTSTRAP_SEED = 20260723

REQUIRED_FIELDS = (
    "truth_distance",
    "rgbd_x",
    "rgbd_y",
    "livox_x",
    "livox_y",
    "rgbd_coverage",
    "livox_coverage",
    "rgbd_error_norm",
    "livox_error_norm",
)


@dataclass(frozen=True)
class LabeledWindow:
    score: float
    high_error: bool
    rgbd_error: float
    livox_error: float


def _finite_float(row: dict[str, str], field: str, row_number: int) -> float:
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"第 {row_number} 行字段 {field} 无效") from error
    if not math.isfinite(value):
        raise ValueError(f"第 {row_number} 行字段 {field} 不是有限值")
    return value


def load_labeled_windows(path: Path) -> tuple[list[LabeledWindow], dict[str, int]]:
    """按冻结口径读取有效高/低误差窗口；中间标签只计数不参与判定。"""
    if not path.is_file():
        raise ValueError(f"窗口 CSV 不存在：{path}")
    output: list[LabeledWindow] = []
    counters = {
        "total": 0,
        "ineligible": 0,
        "middle_error": 0,
    }
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = [field for field in REQUIRED_FIELDS if field not in (reader.fieldnames or ())]
        if missing:
            raise ValueError(f"窗口 CSV 缺少字段：{', '.join(missing)}")
        for row_number, row in enumerate(reader, start=2):
            counters["total"] += 1
            values = {
                field: _finite_float(row, field, row_number)
                for field in REQUIRED_FIELDS
            }
            if (
                values["truth_distance"] < MINIMUM_TRUTH_DISTANCE
                or values["rgbd_coverage"] < MINIMUM_SENSOR_COVERAGE
                or values["livox_coverage"] < MINIMUM_SENSOR_COVERAGE
            ):
                counters["ineligible"] += 1
                continue
            rgbd_error = values["rgbd_error_norm"]
            if rgbd_error <= LOW_ERROR_MAXIMUM:
                high_error = False
            elif rgbd_error >= HIGH_ERROR_MINIMUM:
                high_error = True
            else:
                counters["middle_error"] += 1
                continue
            score = math.hypot(
                values["rgbd_x"] - values["livox_x"],
                values["rgbd_y"] - values["livox_y"],
            )
            output.append(
                LabeledWindow(
                    score=score,
                    high_error=high_error,
                    rgbd_error=rgbd_error,
                    livox_error=values["livox_error_norm"],
                )
            )
    return output, counters


def roc_auc(samples: Sequence[LabeledWindow]) -> float:
    """用高误差为正类计算含并列值的 Mann-Whitney AUC。"""
    positives = [sample.score for sample in samples if sample.high_error]
    negatives = [sample.score for sample in samples if not sample.high_error]
    if not positives or not negatives:
        return math.nan
    wins = 0.0
    for positive in positives:
        for negative in negatives:
            if positive > negative:
                wins += 1.0
            elif positive == negative:
                wins += 0.5
    return wins / (len(positives) * len(negatives))


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def block_bootstrap_auc(
    samples: Sequence[LabeledWindow],
    *,
    block_size: int = BOOTSTRAP_BLOCK_SIZE,
    repetitions: int = BOOTSTRAP_REPETITIONS,
    seed: int = BOOTSTRAP_SEED,
) -> dict[str, float | int]:
    """按时间连续块重采样，避免把相邻窗口当作独立样本。"""
    point = roc_auc(samples)
    if (
        len(samples) < 2
        or block_size <= 0
        or repetitions <= 0
        or not math.isfinite(point)
    ):
        return {
            "auc": point,
            "ci95_low": math.nan,
            "ci95_high": math.nan,
            "valid_repetitions": 0,
        }
    blocks = [
        list(samples[start : start + block_size])
        for start in range(0, len(samples), block_size)
    ]
    generator = random.Random(seed)
    values: list[float] = []
    for _ in range(repetitions):
        resampled: list[LabeledWindow] = []
        while len(resampled) < len(samples):
            resampled.extend(generator.choice(blocks))
        value = roc_auc(resampled[: len(samples)])
        if math.isfinite(value):
            values.append(value)
    return {
        "auc": point,
        "ci95_low": _percentile(values, 0.025),
        "ci95_high": _percentile(values, 0.975),
        "valid_repetitions": len(values),
    }


def _rmse(values: Iterable[float]) -> float:
    items = list(values)
    if not items:
        return math.nan
    return math.sqrt(sum(value * value for value in items) / len(items))


def analyze_run(name: str, path: Path, role: str) -> dict:
    samples, counters = load_labeled_windows(path)
    high = [sample for sample in samples if sample.high_error]
    low = [sample for sample in samples if not sample.high_error]
    bootstrap = block_bootstrap_auc(samples)
    rgbd_rmse = _rmse(sample.rgbd_error for sample in high)
    livox_rmse = _rmse(sample.livox_error for sample in high)
    improvement = (
        (rgbd_rmse - livox_rmse) / rgbd_rmse
        if math.isfinite(rgbd_rmse) and rgbd_rmse > 0.0
        else math.nan
    )
    auc = float(bootstrap["auc"])
    lower = float(bootstrap["ci95_low"])
    passed = bool(
        len(high) >= MINIMUM_CLASS_COUNT
        and len(low) >= MINIMUM_CLASS_COUNT
        and math.isfinite(auc)
        and auc >= MINIMUM_AUC
        and math.isfinite(lower)
        and lower >= MINIMUM_BOOTSTRAP_LOWER
        and math.isfinite(improvement)
        and improvement >= MINIMUM_LIVOX_RMSE_IMPROVEMENT
    )
    reasons: list[str] = []
    if len(high) < MINIMUM_CLASS_COUNT or len(low) < MINIMUM_CLASS_COUNT:
        reasons.append("class_count")
    if not math.isfinite(auc) or auc < MINIMUM_AUC:
        reasons.append("auc")
    if not math.isfinite(lower) or lower < MINIMUM_BOOTSTRAP_LOWER:
        reasons.append("bootstrap_lower")
    if (
        not math.isfinite(improvement)
        or improvement < MINIMUM_LIVOX_RMSE_IMPROVEMENT
    ):
        reasons.append("livox_rmse_improvement")
    return {
        "name": str(name),
        "role": str(role),
        "path": str(path.resolve()),
        "total_window_count": counters["total"],
        "ineligible_window_count": counters["ineligible"],
        "middle_error_window_count": counters["middle_error"],
        "eligible_labeled_window_count": len(samples),
        "high_error_count": len(high),
        "low_error_count": len(low),
        "auc": auc,
        "bootstrap_ci95_low": lower,
        "bootstrap_ci95_high": float(bootstrap["ci95_high"]),
        "bootstrap_valid_repetitions": int(bootstrap["valid_repetitions"]),
        "high_error_rgbd_rmse": rgbd_rmse,
        "high_error_livox_rmse": livox_rmse,
        "high_error_livox_relative_improvement": improvement,
        "passed": passed,
        "failure_reasons": reasons,
    }


def _parse_run(value: str) -> tuple[str, Path]:
    name, separator, raw_path = value.partition("=")
    if not separator or not name.strip() or not raw_path.strip():
        raise ValueError("运行参数必须为 NAME=CSV_PATH")
    return name.strip(), Path(raw_path).expanduser()


def evaluate(
    validation_runs: Sequence[tuple[str, Path]],
    confirmation_runs: Sequence[tuple[str, Path]],
) -> dict:
    runs = [
        analyze_run(name, path, "validation")
        for name, path in validation_runs
    ]
    runs.extend(
        analyze_run(name, path, "confirmation")
        for name, path in confirmation_runs
    )
    passed = bool(runs and all(run["passed"] for run in runs))
    return {
        "feature": "rgbd_livox_delta_disagreement_norm",
        "label": {
            "low_error_maximum": LOW_ERROR_MAXIMUM,
            "high_error_minimum": HIGH_ERROR_MINIMUM,
            "middle_error_excluded": True,
        },
        "eligibility": {
            "minimum_truth_distance": MINIMUM_TRUTH_DISTANCE,
            "minimum_rgbd_coverage": MINIMUM_SENSOR_COVERAGE,
            "minimum_livox_coverage": MINIMUM_SENSOR_COVERAGE,
        },
        "acceptance": {
            "minimum_class_count": MINIMUM_CLASS_COUNT,
            "minimum_auc": MINIMUM_AUC,
            "bootstrap_block_size": BOOTSTRAP_BLOCK_SIZE,
            "bootstrap_repetitions": BOOTSTRAP_REPETITIONS,
            "bootstrap_seed": BOOTSTRAP_SEED,
            "minimum_bootstrap_ci95_lower": MINIMUM_BOOTSTRAP_LOWER,
            "minimum_high_error_livox_rmse_improvement":
                MINIMUM_LIVOX_RMSE_IMPROVEMENT,
            "all_runs_must_pass": True,
        },
        "validation_run_count": len(validation_runs),
        "confirmation_run_count": len(confirmation_runs),
        "runs": runs,
        "passed": passed,
        "decision": (
            "允许进入默认关闭的生产候选设计"
            if passed
            else "冻结特征未跨运行通过，停止生产候选"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="跨运行离线验证 RGB-D/Livox 短窗分歧",
    )
    parser.add_argument(
        "--validation-run",
        action="append",
        default=[],
        metavar="NAME=CSV_PATH",
    )
    parser.add_argument(
        "--confirmation-run",
        action="append",
        default=[],
        metavar="NAME=CSV_PATH",
    )
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    try:
        validation_runs = [_parse_run(value) for value in args.validation_run]
        confirmation_runs = [_parse_run(value) for value in args.confirmation_run]
        if not validation_runs:
            raise ValueError("至少需要一轮验证数据")
        if not confirmation_runs:
            raise ValueError("至少需要一轮确认数据")
        result = evaluate(validation_runs, confirmation_runs)
    except ValueError as error:
        parser.error(str(error))
    payload = json.dumps(
        result,
        ensure_ascii=False,
        indent=2,
        sort_keys=True,
        allow_nan=False,
    )
    if args.json is not None:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

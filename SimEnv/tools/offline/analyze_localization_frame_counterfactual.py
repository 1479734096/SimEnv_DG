#!/usr/bin/env python3
"""逐 RGB-D 帧离线复算 Livox 分歧门及两种累计反事实。"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
import sys
from typing import Sequence


MINIMUM_DEPTH_INTERVAL = 0.10
MAXIMUM_DEPTH_INTERVAL = 0.35
MINIMUM_LIVOX_COVERAGE = 0.60
DISAGREEMENT_THRESHOLD = 0.05
LOW_ERROR_MAXIMUM = 0.02
HIGH_ERROR_MINIMUM = 0.05
MINIMUM_LOCAL_RMSE_IMPROVEMENT = 0.15
MAXIMUM_LOW_ERROR_REPLACEMENT_RATE = 0.05
MINIMUM_HIGH_ERROR_RECALL = 0.50
MAXIMUM_GLOBAL_RMSE_RATIO = 1.0
MAXIMUM_TRUTH_SYNC_GAP = 0.15
MAXIMUM_GLOBAL_SYNC_GAP = 0.10


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PlanarFrame:
    source: str
    stamp: float
    interval: float
    applied_x: float
    applied_y: float
    accepted: bool = True
    has_delta: bool = True


@dataclass(frozen=True)
class FrameComparison:
    stamp: float
    interval: float
    livox_coverage: float
    rgbd_x: float
    rgbd_y: float
    livox_x: float
    livox_y: float
    truth_x: float
    truth_y: float
    disagreement: float
    rgbd_error: float
    livox_error: float
    hold_error: float
    flagged: bool


def _finite_frame(frame: PlanarFrame) -> bool:
    return all(
        math.isfinite(value)
        for value in (
            frame.stamp,
            frame.interval,
            frame.applied_x,
            frame.applied_y,
        )
    )


def _interpolate(
    samples: Sequence[PoseSample],
    stamp: float,
    maximum_gap: float,
) -> PoseSample | None:
    if not samples:
        return None
    left = None
    for sample in samples:
        if abs(sample.stamp - stamp) <= 1e-9:
            return sample
        if sample.stamp > stamp:
            if left is None:
                return None
            if (
                stamp - left.stamp > maximum_gap
                or sample.stamp - stamp > maximum_gap
                or sample.stamp <= left.stamp
            ):
                return None
            ratio = (stamp - left.stamp) / (sample.stamp - left.stamp)
            yaw_delta = math.atan2(
                math.sin(sample.yaw - left.yaw),
                math.cos(sample.yaw - left.yaw),
            )
            return PoseSample(
                stamp=stamp,
                x=left.x + ratio * (sample.x - left.x),
                y=left.y + ratio * (sample.y - left.y),
                yaw=math.atan2(
                    math.sin(left.yaw + ratio * yaw_delta),
                    math.cos(left.yaw + ratio * yaw_delta),
                ),
            )
        left = sample
    return None


def _alignment(
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
) -> tuple[PoseSample, PoseSample, float, float]:
    if not estimates or not truths:
        raise ValueError("缺少定位或离线真值")
    estimate = estimates[0]
    truth = min(truths, key=lambda item: abs(item.stamp - estimate.stamp))
    if abs(truth.stamp - estimate.stamp) > MAXIMUM_GLOBAL_SYNC_GAP:
        raise ValueError("首份定位与离线真值无法对齐")
    offset = estimate.yaw - truth.yaw
    return estimate, truth, math.cos(offset), math.sin(offset)


def _aligned_truth_xy(
    truth: PoseSample,
    alignment: tuple[PoseSample, PoseSample, float, float],
) -> tuple[float, float]:
    estimate0, truth0, cosine, sine = alignment
    dx = truth.x - truth0.x
    dy = truth.y - truth0.y
    return (
        estimate0.x + cosine * dx - sine * dy,
        estimate0.y + sine * dx + cosine * dy,
    )


def _livox_overlap(
    start: float,
    end: float,
    frames: Sequence[PlanarFrame],
) -> tuple[float, float, float]:
    """只使用在 RGB-D 时刻已经结束的 Livox 帧，保证离线特征可在线实现。"""
    x = 0.0
    y = 0.0
    covered = 0.0
    for frame in frames:
        if frame.stamp > end + 1e-9:
            break
        frame_start = frame.stamp - frame.interval
        overlap = max(0.0, min(end, frame.stamp) - max(start, frame_start))
        if overlap <= 0.0:
            continue
        fraction = overlap / frame.interval
        x += fraction * frame.applied_x
        y += fraction * frame.applied_y
        covered += overlap
    duration = end - start
    return x, y, min(1.0, covered / duration) if duration > 0.0 else 0.0


def build_comparisons(
    frames: Sequence[PlanarFrame],
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
) -> tuple[list[FrameComparison], dict[str, int]]:
    alignment = _alignment(estimates, truths)
    depth_frames = sorted(
        (
            frame
            for frame in frames
            if frame.source == "rgbd_frame_gicp"
            and frame.accepted
            and frame.has_delta
            and _finite_frame(frame)
        ),
        key=lambda item: item.stamp,
    )
    livox_frames = sorted(
        (
            frame
            for frame in frames
            if frame.source == "livox_frame_gicp"
            and frame.accepted
            and frame.has_delta
            and _finite_frame(frame)
            and frame.interval > 0.0
        ),
        key=lambda item: item.stamp,
    )
    counters = {
        "accepted_rgbd": len(depth_frames),
        "invalid_interval": 0,
        "truth_unavailable": 0,
        "livox_coverage_unavailable": 0,
    }
    output: list[FrameComparison] = []
    for depth in depth_frames:
        if not (
            MINIMUM_DEPTH_INTERVAL
            <= depth.interval
            <= MAXIMUM_DEPTH_INTERVAL
        ):
            counters["invalid_interval"] += 1
            continue
        start = depth.stamp - depth.interval
        truth_start = _interpolate(
            truths, start, MAXIMUM_TRUTH_SYNC_GAP
        )
        truth_end = _interpolate(
            truths, depth.stamp, MAXIMUM_TRUTH_SYNC_GAP
        )
        if truth_start is None or truth_end is None:
            counters["truth_unavailable"] += 1
            continue
        livox_x, livox_y, coverage = _livox_overlap(
            start, depth.stamp, livox_frames
        )
        if coverage < MINIMUM_LIVOX_COVERAGE:
            counters["livox_coverage_unavailable"] += 1
            continue
        aligned_start = _aligned_truth_xy(truth_start, alignment)
        aligned_end = _aligned_truth_xy(truth_end, alignment)
        truth_x = aligned_end[0] - aligned_start[0]
        truth_y = aligned_end[1] - aligned_start[1]
        disagreement = math.hypot(
            depth.applied_x - livox_x,
            depth.applied_y - livox_y,
        )
        output.append(
            FrameComparison(
                stamp=depth.stamp,
                interval=depth.interval,
                livox_coverage=coverage,
                rgbd_x=depth.applied_x,
                rgbd_y=depth.applied_y,
                livox_x=livox_x,
                livox_y=livox_y,
                truth_x=truth_x,
                truth_y=truth_y,
                disagreement=disagreement,
                rgbd_error=math.hypot(
                    depth.applied_x - truth_x,
                    depth.applied_y - truth_y,
                ),
                livox_error=math.hypot(
                    livox_x - truth_x,
                    livox_y - truth_y,
                ),
                hold_error=math.hypot(truth_x, truth_y),
                flagged=disagreement >= DISAGREEMENT_THRESHOLD,
            )
        )
    return output, counters


def _rmse(values: Sequence[float]) -> float:
    if not values:
        return math.nan
    return math.sqrt(sum(value * value for value in values) / len(values))


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = int(math.floor(fraction * (len(ordered) - 1)))
    return ordered[index]


def _global_metrics(
    comparisons: Sequence[FrameComparison],
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
    action: str,
) -> dict[str, float | int]:
    alignment = _alignment(estimates, truths)
    corrections: list[tuple[float, float, float]] = []
    for item in comparisons:
        if not item.flagged:
            continue
        if action == "livox_substitution":
            dx = item.livox_x - item.rgbd_x
            dy = item.livox_y - item.rgbd_y
        elif action == "hold_position":
            dx = -item.rgbd_x
            dy = -item.rgbd_y
        else:
            raise ValueError(f"未知反事实动作：{action}")
        corrections.append((item.stamp, dx, dy))

    baseline_errors: list[float] = []
    candidate_errors: list[float] = []
    correction_x = 0.0
    correction_y = 0.0
    correction_index = 0
    for estimate in estimates:
        while (
            correction_index < len(corrections)
            and corrections[correction_index][0] <= estimate.stamp
        ):
            correction_x += corrections[correction_index][1]
            correction_y += corrections[correction_index][2]
            correction_index += 1
        truth = _interpolate(
            truths, estimate.stamp, MAXIMUM_GLOBAL_SYNC_GAP
        )
        if truth is None:
            continue
        aligned_x, aligned_y = _aligned_truth_xy(truth, alignment)
        baseline_errors.append(
            math.hypot(estimate.x - aligned_x, estimate.y - aligned_y)
        )
        candidate_errors.append(
            math.hypot(
                estimate.x + correction_x - aligned_x,
                estimate.y + correction_y - aligned_y,
            )
        )
    baseline_rmse = _rmse(baseline_errors)
    candidate_rmse = _rmse(candidate_errors)
    return {
        "aligned_pose_count": len(baseline_errors),
        "baseline_rmse": baseline_rmse,
        "candidate_rmse": candidate_rmse,
        "candidate_to_baseline_rmse_ratio": (
            candidate_rmse / baseline_rmse
            if math.isfinite(baseline_rmse) and baseline_rmse > 0.0
            else math.nan
        ),
        "baseline_p95": _percentile(baseline_errors, 0.95),
        "candidate_p95": _percentile(candidate_errors, 0.95),
        "baseline_final": (
            baseline_errors[-1] if baseline_errors else math.nan
        ),
        "candidate_final": (
            candidate_errors[-1] if candidate_errors else math.nan
        ),
        "final_correction_x": correction_x,
        "final_correction_y": correction_y,
    }


def _action_result(
    action: str,
    comparisons: Sequence[FrameComparison],
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
    high_recall: float,
    low_replacement_rate: float,
) -> dict:
    baseline = [item.rgbd_error for item in comparisons]
    if action == "livox_substitution":
        candidate = [
            item.livox_error if item.flagged else item.rgbd_error
            for item in comparisons
        ]
    elif action == "hold_position":
        candidate = [
            item.hold_error if item.flagged else item.rgbd_error
            for item in comparisons
        ]
    else:
        raise ValueError(f"未知反事实动作：{action}")
    baseline_rmse = _rmse(baseline)
    candidate_rmse = _rmse(candidate)
    improvement = (
        (baseline_rmse - candidate_rmse) / baseline_rmse
        if math.isfinite(baseline_rmse) and baseline_rmse > 0.0
        else math.nan
    )
    global_metrics = _global_metrics(
        comparisons, estimates, truths, action
    )
    global_ratio = float(
        global_metrics["candidate_to_baseline_rmse_ratio"]
    )
    reasons: list[str] = []
    if (
        not math.isfinite(improvement)
        or improvement < MINIMUM_LOCAL_RMSE_IMPROVEMENT
    ):
        reasons.append("local_rmse_improvement")
    if (
        not math.isfinite(high_recall)
        or high_recall < MINIMUM_HIGH_ERROR_RECALL
    ):
        reasons.append("high_error_recall")
    if (
        not math.isfinite(low_replacement_rate)
        or low_replacement_rate > MAXIMUM_LOW_ERROR_REPLACEMENT_RATE
    ):
        reasons.append("low_error_replacement_rate")
    if (
        not math.isfinite(global_ratio)
        or global_ratio > MAXIMUM_GLOBAL_RMSE_RATIO
    ):
        reasons.append("global_rmse_regression")
    return {
        "action": action,
        "local_baseline_rmse": baseline_rmse,
        "local_candidate_rmse": candidate_rmse,
        "local_relative_improvement": improvement,
        "global": global_metrics,
        "passed": not reasons,
        "failure_reasons": reasons,
    }


def analyze_run(
    name: str,
    role: str,
    bag_path: Path,
    frames: Sequence[PlanarFrame],
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
) -> tuple[dict, list[FrameComparison]]:
    comparisons, counters = build_comparisons(
        frames, estimates, truths
    )
    high = [
        item for item in comparisons
        if item.rgbd_error >= HIGH_ERROR_MINIMUM
    ]
    low = [
        item for item in comparisons
        if item.rgbd_error <= LOW_ERROR_MAXIMUM
    ]
    high_recall = (
        sum(item.flagged for item in high) / len(high)
        if high else math.nan
    )
    low_replacement_rate = (
        sum(item.flagged for item in low) / len(low)
        if low else math.nan
    )
    actions = [
        _action_result(
            action,
            comparisons,
            estimates,
            truths,
            high_recall,
            low_replacement_rate,
        )
        for action in ("livox_substitution", "hold_position")
    ]
    return {
        "name": name,
        "role": role,
        "bag": str(bag_path.resolve()),
        "accepted_rgbd_frame_count": counters["accepted_rgbd"],
        "eligible_frame_count": len(comparisons),
        "fallback_frame_count": (
            counters["invalid_interval"]
            + counters["truth_unavailable"]
            + counters["livox_coverage_unavailable"]
        ),
        "fallback_reasons": {
            "invalid_interval": counters["invalid_interval"],
            "truth_unavailable": counters["truth_unavailable"],
            "livox_coverage_unavailable":
                counters["livox_coverage_unavailable"],
        },
        "flagged_frame_count": sum(item.flagged for item in comparisons),
        "high_error_frame_count": len(high),
        "low_error_frame_count": len(low),
        "high_error_recall": high_recall,
        "low_error_replacement_rate": low_replacement_rate,
        "actions": actions,
        "passed": any(action["passed"] for action in actions),
    }, comparisons


def _load_quality_module():
    path = Path(__file__).with_name("analyze_localization_quality_bag.py")
    specification = importlib.util.spec_from_file_location(
        "_localization_quality_for_frame_counterfactual", path
    )
    if specification is None or specification.loader is None:
        raise ValueError(f"无法加载既有定位分析器：{path}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def load_bag(path: Path) -> tuple[
    list[PlanarFrame], list[PoseSample], list[PoseSample]
]:
    if not path.is_file():
        raise ValueError(f"bag 不存在：{path}")
    quality = _load_quality_module()
    import rosbag

    frames: list[PlanarFrame] = []
    with rosbag.Bag(str(path), "r") as bag:
        estimates_raw = quality._read_poses(
            bag, "/danger_search/localization/odom"
        )
        truths_raw = quality._read_poses(bag, "/ground_truth/base_w")
        for _topic, message, _bag_stamp in bag.read_messages(
            topics=["/rosout"]
        ):
            parsed = quality.parse_planar_frame(
                str(getattr(message, "msg", ""))
            )
            if parsed is None:
                continue
            frames.append(
                PlanarFrame(
                    source=parsed.source,
                    stamp=parsed.stamp,
                    interval=parsed.interval,
                    applied_x=parsed.applied_x,
                    applied_y=parsed.applied_y,
                    accepted=parsed.accepted,
                    has_delta=parsed.has_delta,
                )
            )
    estimates = [
        PoseSample(item.stamp, item.x, item.y, item.yaw)
        for item in estimates_raw
    ]
    truths = [
        PoseSample(item.stamp, item.x, item.y, item.yaw)
        for item in truths_raw
    ]
    if not frames or not estimates or not truths:
        raise ValueError(f"bag 缺少逐帧诊断、定位或离线真值：{path}")
    return frames, estimates, truths


def _parse_run(value: str) -> tuple[str, Path]:
    name, separator, raw_path = value.partition("=")
    if not separator or not name.strip() or not raw_path.strip():
        raise ValueError("运行参数必须为 NAME=BAG_PATH")
    return name.strip(), Path(raw_path).expanduser()


def _write_frames(
    directory: Path,
    name: str,
    comparisons: Sequence[FrameComparison],
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    safe_name = "".join(
        character if character.isalnum() or character in "-_" else "_"
        for character in name
    )
    path = directory / f"{safe_name}.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(FrameComparison.__annotations__)
        )
        writer.writeheader()
        for item in comparisons:
            writer.writerow(asdict(item))


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def evaluate(
    validation_runs: Sequence[tuple[str, Path]],
    confirmation_runs: Sequence[tuple[str, Path]],
    frames_csv_directory: Path | None = None,
) -> dict:
    runs: list[dict] = []
    for role, specifications in (
        ("validation", validation_runs),
        ("confirmation", confirmation_runs),
    ):
        for name, path in specifications:
            frames, estimates, truths = load_bag(path)
            result, comparisons = analyze_run(
                name, role, path, frames, estimates, truths
            )
            runs.append(result)
            if frames_csv_directory is not None:
                _write_frames(
                    frames_csv_directory, name, comparisons
                )
    action_names = ("livox_substitution", "hold_position")
    action_passes = {
        action_name: bool(runs) and all(
            next(
                action
                for action in run["actions"]
                if action["action"] == action_name
            )["passed"]
            for run in runs
        )
        for action_name in action_names
    }
    passed = bool(
        validation_runs
        and confirmation_runs
        and any(action_passes.values())
    )
    return {
        "feature": "causal_per_rgbd_frame_livox_disagreement_norm",
        "eligibility": {
            "minimum_depth_interval": MINIMUM_DEPTH_INTERVAL,
            "maximum_depth_interval": MAXIMUM_DEPTH_INTERVAL,
            "minimum_livox_coverage": MINIMUM_LIVOX_COVERAGE,
            "livox_frames_must_end_no_later_than_rgbd_stamp": True,
            "quality_missing_falls_back_to_existing_rgbd": True,
        },
        "trigger": {
            "disagreement_threshold": DISAGREEMENT_THRESHOLD,
            "high_error_minimum": HIGH_ERROR_MINIMUM,
            "low_error_maximum": LOW_ERROR_MAXIMUM,
        },
        "acceptance": {
            "minimum_local_rmse_improvement":
                MINIMUM_LOCAL_RMSE_IMPROVEMENT,
            "minimum_high_error_recall": MINIMUM_HIGH_ERROR_RECALL,
            "maximum_low_error_replacement_rate":
                MAXIMUM_LOW_ERROR_REPLACEMENT_RATE,
            "maximum_global_rmse_ratio": MAXIMUM_GLOBAL_RMSE_RATIO,
            "one_action_must_pass_all_runs": True,
        },
        "actions": {
            "livox_substitution":
                "异常帧以同窗 Livox 增量替换并累计",
            "hold_position": "异常帧不累计 RGB-D 平移",
        },
        "validation_run_count": len(validation_runs),
        "confirmation_run_count": len(confirmation_runs),
        "runs": runs,
        "action_passes_all_runs": action_passes,
        "passed": passed,
        "decision": (
            "允许进入默认关闭的生产候选实现"
            if passed
            else "两种累计动作均未跨运行通过，停止生产候选"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="逐 RGB-D 帧离线复算 Livox 分歧与累计反事实"
    )
    parser.add_argument(
        "--validation-run",
        action="append",
        default=[],
        metavar="NAME=BAG_PATH",
    )
    parser.add_argument(
        "--confirmation-run",
        action="append",
        default=[],
        metavar="NAME=BAG_PATH",
    )
    parser.add_argument("--frames-csv-directory", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    try:
        validation_runs = [
            _parse_run(value) for value in args.validation_run
        ]
        confirmation_runs = [
            _parse_run(value) for value in args.confirmation_run
        ]
        if not validation_runs:
            raise ValueError("至少需要一轮验证数据")
        if not confirmation_runs:
            raise ValueError("至少需要一轮确认数据")
        result = evaluate(
            validation_runs,
            confirmation_runs,
            args.frames_csv_directory,
        )
    except ValueError as error:
        parser.error(str(error))
    payload = json.dumps(
        _json_safe(result),
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

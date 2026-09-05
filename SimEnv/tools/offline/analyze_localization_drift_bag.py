#!/usr/bin/env python3
"""离线比较生产定位与 Gazebo 诊断真值，不向生产链提供任何数据。"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import rosbag


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DriftSample:
    elapsed: float
    position_error: float
    error_x: float
    error_y: float
    yaw_error: float
    truth_distance: float
    absolute_position_error: float = 0.0
    absolute_error_x: float = 0.0
    absolute_error_y: float = 0.0
    absolute_yaw_error: float = 0.0


def _angle_wrap(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quaternion(quaternion) -> float:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _message_stamp(message, bag_stamp) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        value = float(stamp.to_sec())
        if value > 0.0 and math.isfinite(value):
            return value
    return float(bag_stamp.to_sec())


def _read_pose_samples(
    bag: rosbag.Bag,
    topic: str,
) -> list[PoseSample]:
    samples: list[PoseSample] = []
    for _topic, message, bag_stamp in bag.read_messages(topics=[topic]):
        pose_with_covariance = getattr(message, "pose", None)
        pose = getattr(pose_with_covariance, "pose", pose_with_covariance)
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        if position is None or orientation is None:
            continue
        values = (
            float(position.x),
            float(position.y),
            _yaw_from_quaternion(orientation),
        )
        stamp = _message_stamp(message, bag_stamp)
        if math.isfinite(stamp) and all(math.isfinite(value) for value in values):
            samples.append(PoseSample(stamp, *values))
    samples.sort(key=lambda sample: sample.stamp)
    return samples


def _synchronize(
    estimates: list[PoseSample],
    truths: list[PoseSample],
    maximum_gap: float,
) -> list[tuple[PoseSample, PoseSample]]:
    pairs: list[tuple[PoseSample, PoseSample]] = []
    truth_index = 0
    for estimate in estimates:
        while (
            truth_index + 1 < len(truths)
            and abs(truths[truth_index + 1].stamp - estimate.stamp)
            <= abs(truths[truth_index].stamp - estimate.stamp)
        ):
            truth_index += 1
        if truth_index >= len(truths):
            break
        truth = truths[truth_index]
        if abs(truth.stamp - estimate.stamp) <= maximum_gap:
            pairs.append((estimate, truth))
    return pairs


def _compute_drift(
    pairs: list[tuple[PoseSample, PoseSample]],
) -> list[DriftSample]:
    if not pairs:
        return []
    initial_estimate, initial_truth = pairs[0]
    yaw_offset = _angle_wrap(initial_estimate.yaw - initial_truth.yaw)
    cos_offset = math.cos(yaw_offset)
    sin_offset = math.sin(yaw_offset)
    previous_truth = initial_truth
    truth_distance = 0.0
    samples: list[DriftSample] = []
    for estimate, truth in pairs:
        truth_distance += math.hypot(
            truth.x - previous_truth.x,
            truth.y - previous_truth.y,
        )
        previous_truth = truth
        truth_dx = truth.x - initial_truth.x
        truth_dy = truth.y - initial_truth.y
        aligned_truth_x = (
            initial_estimate.x + cos_offset * truth_dx - sin_offset * truth_dy
        )
        aligned_truth_y = (
            initial_estimate.y + sin_offset * truth_dx + cos_offset * truth_dy
        )
        error_x = estimate.x - aligned_truth_x
        error_y = estimate.y - aligned_truth_y
        yaw_error = _angle_wrap(
            (estimate.yaw - initial_estimate.yaw)
            - (truth.yaw - initial_truth.yaw)
        )
        samples.append(
            DriftSample(
                elapsed=estimate.stamp - initial_estimate.stamp,
                position_error=math.hypot(error_x, error_y),
                error_x=error_x,
                error_y=error_y,
                yaw_error=yaw_error,
                truth_distance=truth_distance,
                absolute_position_error=math.hypot(estimate.x - truth.x, estimate.y - truth.y),
                absolute_error_x=estimate.x - truth.x,
                absolute_error_y=estimate.y - truth.y,
                absolute_yaw_error=_angle_wrap(estimate.yaw - truth.yaw),
            )
        )
    return samples


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * fraction))
    return float(ordered[index])


def _summary(samples: list[DriftSample]) -> dict[str, float | int]:
    if not samples:
        return {"sample_count": 0}
    position_errors = [sample.position_error for sample in samples]
    yaw_errors = [abs(sample.yaw_error) for sample in samples]
    squared_sum = sum(value * value for value in position_errors)
    final = samples[-1]
    maximum_sample = max(samples, key=lambda sample: sample.position_error)
    tail_start = max(0.0, final.elapsed - 10.0)
    tail = [sample for sample in samples if sample.elapsed >= tail_start]
    tail_errors = [sample.position_error for sample in tail]
    return {
        "sample_count": len(samples),
        "duration": final.elapsed,
        "truth_distance": final.truth_distance,
        "final_position_error": final.position_error,
        "maximum_position_error": max(position_errors),
        "maximum_position_error_elapsed": maximum_sample.elapsed,
        "rms_position_error": math.sqrt(squared_sum / len(position_errors)),
        "p95_position_error": _percentile(position_errors, 0.95),
        "final_yaw_error": final.yaw_error,
        "maximum_absolute_yaw_error": max(yaw_errors),
        "position_error_per_truth_meter": (
            final.position_error / final.truth_distance
            if final.truth_distance > 1e-6
            else 0.0
        ),
        "last_10s_position_error_span": (
            max(tail_errors) - min(tail_errors) if tail_errors else 0.0
        ),
    }


def _absolute_summary(samples: list[DriftSample]) -> dict:
    """不拟合坐标变换，保留会影响危险源 world 坐标的初始化误差。"""
    if not samples:
        return {"sample_count": 0}
    errors = [sample.absolute_position_error for sample in samples]
    return {
        "sample_count": len(samples), "alignment": "none",
        "initial_position_error": samples[0].absolute_position_error,
        "initial_yaw_error": samples[0].absolute_yaw_error,
        "final_position_error": samples[-1].absolute_position_error,
        "final_yaw_error": samples[-1].absolute_yaw_error,
        "maximum_position_error": max(errors),
        "rms_position_error": math.sqrt(sum(value * value for value in errors) / len(errors)),
        "p95_position_error": _percentile(errors, 0.95),
        "maximum_absolute_yaw_error": max(abs(sample.absolute_yaw_error) for sample in samples),
    }


def _write_csv(path: Path, samples: Iterable[DriftSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(DriftSample.__annotations__))
        writer.writeheader()
        for sample in samples:
            writer.writerow(asdict(sample))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="离线计算首帧对齐漂移及未对齐 world 绝对 XY/yaw 误差（弧度）",
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--odom-topic",
        default="/danger_search/localization/odom",
    )
    parser.add_argument("--truth-topic", default="/ground_truth/base_w")
    parser.add_argument("--maximum-sync-gap", type=float, default=0.10)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()

    if not args.bag.is_file():
        parser.error(f"bag 不存在：{args.bag}")
    if not math.isfinite(args.maximum_sync_gap) or args.maximum_sync_gap <= 0.0:
        parser.error("--maximum-sync-gap 必须是有限正数")

    with rosbag.Bag(str(args.bag), "r") as bag:
        estimates = _read_pose_samples(bag, args.odom_topic)
        truths = _read_pose_samples(bag, args.truth_topic)
    if not estimates:
        parser.error(f"bag 中没有可用定位样本：{args.odom_topic}")
    if not truths:
        parser.error(f"bag 中没有可用诊断真值样本：{args.truth_topic}")

    pairs = _synchronize(estimates, truths, args.maximum_sync_gap)
    if len(pairs) < 2:
        parser.error("时间匹配样本不足，无法计算漂移")
    samples = _compute_drift(pairs)
    first_estimate_stamp = float(estimates[0].stamp)
    first_truth_stamp = float(truths[0].stamp)
    first_aligned_estimate_stamp = float(pairs[0][0].stamp)
    result = {
        "bag": str(args.bag.resolve()),
        "odom_topic": args.odom_topic,
        "truth_topic": args.truth_topic,
        "maximum_sync_gap": args.maximum_sync_gap,
        "first_estimate_stamp": first_estimate_stamp,
        "first_truth_stamp": first_truth_stamp,
        "first_aligned_estimate_stamp": first_aligned_estimate_stamp,
        "elapsed_time_origin": "first_aligned_estimate_header_stamp",
        "relative_alignment": "first_pose_se2",
        "absolute_world": _absolute_summary(samples),
        "first_matched_estimate": asdict(pairs[0][0]),
        "first_matched_truth": asdict(pairs[0][1]),
        **_summary(samples),
    }

    if args.csv is not None:
        _write_csv(args.csv, samples)
    payload = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if args.json is not None:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

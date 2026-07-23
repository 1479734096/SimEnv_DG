#!/usr/bin/env python3
"""离线分析定位校正质量；真值和诊断日志不会反馈到生产节点。"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import rosbag


NUMBER = r"(?:[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|nan|inf|-inf)"


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MapFrame:
    stamp: float
    raw_x: float
    raw_y: float
    applied_x: float
    applied_y: float
    source_points: int
    matched: int
    retained: int
    map_points: int
    iterations: int
    residual_p50: float
    residual_p70: float
    residual_p90: float
    residual_mean_x: float
    residual_mean_y: float
    residual_median_x: float
    residual_median_y: float
    residual_mad_x: float
    residual_mad_y: float
    residual_positive_x: float
    residual_positive_y: float
    residual_negative_x: float
    residual_negative_y: float
    covariance_min: float
    covariance_max: float
    anisotropy: float


@dataclass(frozen=True)
class PlanarFrame:
    source: str
    stamp: float
    interval: float
    applied_x: float
    applied_y: float


@dataclass(frozen=True)
class MotionWindow:
    stamp: float
    duration: float
    depth_x: float
    depth_y: float
    livox_x: float
    livox_y: float
    scan_to_map_x: float
    scan_to_map_y: float
    fused_x: float
    fused_y: float
    command_x: float
    command_y: float
    divergence_x: float
    divergence_y: float
    relation_x: int
    relation_y: int
    ratio_x: float
    ratio_y: float
    imu_abs_yaw: float
    depth_fresh_ratio: float
    frames: int


@dataclass(frozen=True)
class MotionWindowSample:
    stamp: float
    elapsed: float
    duration: float
    error_post: float
    error_without_map: float
    map_benefit: float
    map_benefit_x: float
    map_benefit_y: float
    depth_x: float
    depth_y: float
    livox_x: float
    livox_y: float
    scan_to_map_x: float
    scan_to_map_y: float
    fused_x: float
    fused_y: float
    command_x: float
    command_y: float
    divergence_x: float
    divergence_y: float
    relation_x: int
    relation_y: int
    ratio_x: float
    ratio_y: float
    imu_abs_yaw: float
    depth_fresh_ratio: float
    frames: int


@dataclass(frozen=True)
class QualitySample:
    stamp: float
    elapsed: float
    error_post: float
    error_pre: float
    benefit: float
    benefit_x: float
    benefit_y: float
    applied_x: float
    applied_y: float
    raw_x: float
    raw_y: float
    source_points: int
    matched: int
    retained: int
    matched_ratio: float
    retained_ratio: float
    iterations: int
    residual_p50: float
    residual_p70: float
    residual_p90: float
    residual_mean_x: float
    residual_mean_y: float
    residual_median_x: float
    residual_median_y: float
    residual_mad_x: float
    residual_mad_y: float
    residual_positive_x: float
    residual_positive_y: float
    residual_negative_x: float
    residual_negative_y: float
    anisotropy: float
    depth_velocity_x: float
    depth_velocity_y: float
    livox_velocity_x: float
    livox_velocity_y: float
    divergence_velocity_x: float
    divergence_velocity_y: float


MAP_PATTERN = re.compile(
    rf"localization_map_registration_frame scan_stamp=(?P<stamp>{NUMBER}).*?"
    rf"raw=\((?P<raw_x>{NUMBER}), (?P<raw_y>{NUMBER})\).*?"
    rf"applied=\((?P<applied_x>{NUMBER}), (?P<applied_y>{NUMBER})\).*?"
    rf"source_points=(?P<source_points>\d+) matched=(?P<matched>\d+) "
    rf"retained=(?P<retained>\d+) map_points=(?P<map_points>\d+) "
    rf"iterations=(?P<iterations>\d+).*?"
    rf"residual_p50=(?P<p50>{NUMBER}) residual_p70=(?P<p70>{NUMBER}) "
    rf"residual_p90=(?P<p90>{NUMBER}).*?"
    rf"residual_mean=\((?P<mean_x>{NUMBER}), (?P<mean_y>{NUMBER})\) "
    rf"residual_median=\((?P<median_x>{NUMBER}), (?P<median_y>{NUMBER})\) "
    rf"residual_mad=\((?P<mad_x>{NUMBER}), (?P<mad_y>{NUMBER})\) "
    rf"residual_positive=\((?P<positive_x>{NUMBER}), (?P<positive_y>{NUMBER})\) "
    rf"residual_negative=\((?P<negative_x>{NUMBER}), (?P<negative_y>{NUMBER})\) "
    rf"covariance_eigen=\((?P<cov_min>{NUMBER}), (?P<cov_max>{NUMBER})\) "
    rf"anisotropy=(?P<anisotropy>{NUMBER})"
)

PLANAR_PATTERN = re.compile(
    rf"localization_planar_registration_frame source=(?P<source>\S+) "
    rf"scan_stamp=(?P<stamp>{NUMBER}) frame_interval=(?P<interval>{NUMBER}).*?"
    rf"applied=\((?P<applied_x>{NUMBER}), (?P<applied_y>{NUMBER})\)"
)

MOTION_PATTERN = re.compile(
    rf"localization_motion_consistency_summary scan_stamp=(?P<stamp>{NUMBER}) "
    rf"duration=(?P<duration>{NUMBER}) "
    rf"depth=\((?P<depth_x>{NUMBER}), (?P<depth_y>{NUMBER})\) "
    rf"livox=\((?P<livox_x>{NUMBER}), (?P<livox_y>{NUMBER})\) "
    rf"scan_to_map=\((?P<map_x>{NUMBER}), (?P<map_y>{NUMBER})\) "
    rf"fused=\((?P<fused_x>{NUMBER}), (?P<fused_y>{NUMBER})\) "
    rf"command=\((?P<command_x>{NUMBER}), (?P<command_y>{NUMBER})\) "
    rf"divergence=\((?P<divergence_x>{NUMBER}), (?P<divergence_y>{NUMBER})\) "
    rf"axis_relation=\((?P<relation_x>-?\d+), (?P<relation_y>-?\d+)\) "
    rf"axis_ratio=\((?P<ratio_x>{NUMBER}), (?P<ratio_y>{NUMBER})\) "
    rf"imu_abs_yaw=(?P<imu_abs_yaw>{NUMBER}) "
    rf"depth_fresh_ratio=(?P<depth_fresh_ratio>{NUMBER}) frames=(?P<frames>\d+)"
)


def _finite_float(value: str) -> float:
    return float(value)


def parse_map_frame(message: str) -> MapFrame | None:
    match = MAP_PATTERN.search(message)
    if match is None:
        return None
    values = match.groupdict()
    return MapFrame(
        stamp=_finite_float(values["stamp"]),
        raw_x=_finite_float(values["raw_x"]),
        raw_y=_finite_float(values["raw_y"]),
        applied_x=_finite_float(values["applied_x"]),
        applied_y=_finite_float(values["applied_y"]),
        source_points=int(values["source_points"]),
        matched=int(values["matched"]),
        retained=int(values["retained"]),
        map_points=int(values["map_points"]),
        iterations=int(values["iterations"]),
        residual_p50=_finite_float(values["p50"]),
        residual_p70=_finite_float(values["p70"]),
        residual_p90=_finite_float(values["p90"]),
        residual_mean_x=_finite_float(values["mean_x"]),
        residual_mean_y=_finite_float(values["mean_y"]),
        residual_median_x=_finite_float(values["median_x"]),
        residual_median_y=_finite_float(values["median_y"]),
        residual_mad_x=_finite_float(values["mad_x"]),
        residual_mad_y=_finite_float(values["mad_y"]),
        residual_positive_x=_finite_float(values["positive_x"]),
        residual_positive_y=_finite_float(values["positive_y"]),
        residual_negative_x=_finite_float(values["negative_x"]),
        residual_negative_y=_finite_float(values["negative_y"]),
        covariance_min=_finite_float(values["cov_min"]),
        covariance_max=_finite_float(values["cov_max"]),
        anisotropy=_finite_float(values["anisotropy"]),
    )


def parse_planar_frame(message: str) -> PlanarFrame | None:
    match = PLANAR_PATTERN.search(message)
    if match is None:
        return None
    values = match.groupdict()
    return PlanarFrame(
        source=values["source"],
        stamp=_finite_float(values["stamp"]),
        interval=_finite_float(values["interval"]),
        applied_x=_finite_float(values["applied_x"]),
        applied_y=_finite_float(values["applied_y"]),
    )


def parse_motion_window(message: str) -> MotionWindow | None:
    match = MOTION_PATTERN.search(message)
    if match is None:
        return None
    values = match.groupdict()
    return MotionWindow(
        stamp=_finite_float(values["stamp"]),
        duration=_finite_float(values["duration"]),
        depth_x=_finite_float(values["depth_x"]),
        depth_y=_finite_float(values["depth_y"]),
        livox_x=_finite_float(values["livox_x"]),
        livox_y=_finite_float(values["livox_y"]),
        scan_to_map_x=_finite_float(values["map_x"]),
        scan_to_map_y=_finite_float(values["map_y"]),
        fused_x=_finite_float(values["fused_x"]),
        fused_y=_finite_float(values["fused_y"]),
        command_x=_finite_float(values["command_x"]),
        command_y=_finite_float(values["command_y"]),
        divergence_x=_finite_float(values["divergence_x"]),
        divergence_y=_finite_float(values["divergence_y"]),
        relation_x=int(values["relation_x"]),
        relation_y=int(values["relation_y"]),
        ratio_x=_finite_float(values["ratio_x"]),
        ratio_y=_finite_float(values["ratio_y"]),
        imu_abs_yaw=_finite_float(values["imu_abs_yaw"]),
        depth_fresh_ratio=_finite_float(values["depth_fresh_ratio"]),
        frames=int(values["frames"]),
    )


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (float(quaternion.w) * float(quaternion.z) +
               float(quaternion.x) * float(quaternion.y)),
        1.0 - 2.0 * (float(quaternion.y) ** 2 + float(quaternion.z) ** 2),
    )


def _message_stamp(message, bag_stamp) -> float:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    value = float(stamp.to_sec()) if stamp is not None else 0.0
    return value if value > 0.0 and math.isfinite(value) else float(bag_stamp.to_sec())


def _read_poses(bag: rosbag.Bag, topic: str) -> list[PoseSample]:
    samples: list[PoseSample] = []
    for _topic, message, bag_stamp in bag.read_messages(topics=[topic]):
        pose = getattr(getattr(message, "pose", None), "pose", None)
        if pose is None:
            continue
        samples.append(PoseSample(
            _message_stamp(message, bag_stamp),
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quaternion(pose.orientation),
        ))
    return sorted(samples, key=lambda item: item.stamp)


def _nearest(
    samples: Sequence, stamp: float, maximum_gap: float,
    stamps: Sequence[float] | None = None,
):
    if not samples:
        return None
    if stamps is None:
        stamps = [sample.stamp for sample in samples]
    index = bisect.bisect_left(stamps, stamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    nearest = min(candidates, key=lambda item: abs(item.stamp - stamp))
    return nearest if abs(nearest.stamp - stamp) <= maximum_gap else None


def _alignment(
    estimates: Sequence[PoseSample], truths: Sequence[PoseSample],
    truth_stamps: Sequence[float] | None = None,
):
    estimate = estimates[0]
    truth = _nearest(truths, estimate.stamp, 0.1, truth_stamps)
    if truth is None:
        raise ValueError("首份定位与真值无法对齐")
    yaw_offset = estimate.yaw - truth.yaw
    return estimate, truth, math.cos(yaw_offset), math.sin(yaw_offset)


def _aligned_truth_xy(truth: PoseSample, alignment) -> tuple[float, float]:
    estimate0, truth0, cosine, sine = alignment
    dx = truth.x - truth0.x
    dy = truth.y - truth0.y
    return (
        estimate0.x + cosine * dx - sine * dy,
        estimate0.y + sine * dx + cosine * dy,
    )


def correction_benefit(
    pose_after: tuple[float, float],
    truth: tuple[float, float],
    applied: tuple[float, float],
) -> tuple[float, float, float, float, float]:
    """同一时间戳反事实移除校正，隔离该校正本身的误差贡献。"""
    post_x = pose_after[0] - truth[0]
    post_y = pose_after[1] - truth[1]
    pre_x = post_x - applied[0]
    pre_y = post_y - applied[1]
    post_norm = math.hypot(post_x, post_y)
    pre_norm = math.hypot(pre_x, pre_y)
    return (
        post_norm,
        pre_norm,
        pre_norm - post_norm,
        abs(pre_x) - abs(post_x),
        abs(pre_y) - abs(post_y),
    )


def _velocity(frame: PlanarFrame | None) -> tuple[float, float]:
    if frame is None or not math.isfinite(frame.interval) or frame.interval <= 0.0:
        return math.nan, math.nan
    return frame.applied_x / frame.interval, frame.applied_y / frame.interval


def build_quality_samples(
    map_frames: Sequence[MapFrame],
    planar_frames: Sequence[PlanarFrame],
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
    maximum_gap: float,
) -> list[QualitySample]:
    depth_frames = [item for item in planar_frames if item.source == "rgbd_frame_gicp"]
    livox_frames = [item for item in planar_frames if item.source == "livox_frame_gicp"]
    truth_stamps = [item.stamp for item in truths]
    estimate_stamps = [item.stamp for item in estimates]
    depth_stamps = [item.stamp for item in depth_frames]
    livox_stamps = [item.stamp for item in livox_frames]
    alignment = _alignment(estimates, truths, truth_stamps)
    start_stamp = estimates[0].stamp
    output: list[QualitySample] = []
    for frame in map_frames:
        estimate = _nearest(estimates, frame.stamp, maximum_gap, estimate_stamps)
        truth = _nearest(truths, frame.stamp, maximum_gap, truth_stamps)
        if estimate is None or truth is None:
            continue
        aligned_truth = _aligned_truth_xy(truth, alignment)
        error_post, error_pre, benefit, benefit_x, benefit_y = correction_benefit(
            (estimate.x, estimate.y), aligned_truth,
            (frame.applied_x, frame.applied_y),
        )
        depth_velocity = _velocity(
            _nearest(depth_frames, frame.stamp, 0.25, depth_stamps)
        )
        livox_velocity = _velocity(
            _nearest(livox_frames, frame.stamp, 0.15, livox_stamps)
        )
        divergence_x = livox_velocity[0] - depth_velocity[0]
        divergence_y = livox_velocity[1] - depth_velocity[1]
        output.append(QualitySample(
            stamp=frame.stamp,
            elapsed=frame.stamp - start_stamp,
            error_post=error_post,
            error_pre=error_pre,
            benefit=benefit,
            benefit_x=benefit_x,
            benefit_y=benefit_y,
            applied_x=frame.applied_x,
            applied_y=frame.applied_y,
            raw_x=frame.raw_x,
            raw_y=frame.raw_y,
            source_points=frame.source_points,
            matched=frame.matched,
            retained=frame.retained,
            matched_ratio=frame.matched / frame.source_points if frame.source_points else 0.0,
            retained_ratio=frame.retained / frame.matched if frame.matched else 0.0,
            iterations=frame.iterations,
            residual_p50=frame.residual_p50,
            residual_p70=frame.residual_p70,
            residual_p90=frame.residual_p90,
            residual_mean_x=frame.residual_mean_x,
            residual_mean_y=frame.residual_mean_y,
            residual_median_x=frame.residual_median_x,
            residual_median_y=frame.residual_median_y,
            residual_mad_x=frame.residual_mad_x,
            residual_mad_y=frame.residual_mad_y,
            residual_positive_x=frame.residual_positive_x,
            residual_positive_y=frame.residual_positive_y,
            residual_negative_x=frame.residual_negative_x,
            residual_negative_y=frame.residual_negative_y,
            anisotropy=frame.anisotropy,
            depth_velocity_x=depth_velocity[0],
            depth_velocity_y=depth_velocity[1],
            livox_velocity_x=livox_velocity[0],
            livox_velocity_y=livox_velocity[1],
            divergence_velocity_x=divergence_x,
            divergence_velocity_y=divergence_y,
        ))
    return output


def build_motion_window_samples(
    windows: Sequence[MotionWindow],
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
    maximum_gap: float,
) -> list[MotionWindowSample]:
    truth_stamps = [item.stamp for item in truths]
    estimate_stamps = [item.stamp for item in estimates]
    alignment = _alignment(estimates, truths, truth_stamps)
    start_stamp = estimates[0].stamp
    output: list[MotionWindowSample] = []
    for window in windows:
        estimate = _nearest(estimates, window.stamp, maximum_gap, estimate_stamps)
        truth = _nearest(truths, window.stamp, maximum_gap, truth_stamps)
        if estimate is None or truth is None:
            continue
        aligned_truth = _aligned_truth_xy(truth, alignment)
        error_post, error_without_map, benefit, benefit_x, benefit_y = (
            correction_benefit(
                (estimate.x, estimate.y), aligned_truth,
                (window.scan_to_map_x, window.scan_to_map_y),
            )
        )
        output.append(MotionWindowSample(
            stamp=window.stamp,
            elapsed=window.stamp - start_stamp,
            duration=window.duration,
            error_post=error_post,
            error_without_map=error_without_map,
            map_benefit=benefit,
            map_benefit_x=benefit_x,
            map_benefit_y=benefit_y,
            depth_x=window.depth_x,
            depth_y=window.depth_y,
            livox_x=window.livox_x,
            livox_y=window.livox_y,
            scan_to_map_x=window.scan_to_map_x,
            scan_to_map_y=window.scan_to_map_y,
            fused_x=window.fused_x,
            fused_y=window.fused_y,
            command_x=window.command_x,
            command_y=window.command_y,
            divergence_x=window.divergence_x,
            divergence_y=window.divergence_y,
            relation_x=window.relation_x,
            relation_y=window.relation_y,
            ratio_x=window.ratio_x,
            ratio_y=window.ratio_y,
            imu_abs_yaw=window.imu_abs_yaw,
            depth_fresh_ratio=window.depth_fresh_ratio,
            frames=window.frames,
        ))
    return output


def _ranks(values: Sequence[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda index: values[index])
    ranks = [0.0] * len(values)
    cursor = 0
    while cursor < len(order):
        end = cursor + 1
        while end < len(order) and values[order[end]] == values[order[cursor]]:
            end += 1
        rank = 0.5 * (cursor + end - 1) + 1.0
        for position in range(cursor, end):
            ranks[order[position]] = rank
        cursor = end
    return ranks


def spearman(values: Sequence[float], targets: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(values, targets)
             if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return math.nan
    x_ranks = _ranks([item[0] for item in pairs])
    y_ranks = _ranks([item[1] for item in pairs])
    x_mean = sum(x_ranks) / len(x_ranks)
    y_mean = sum(y_ranks) / len(y_ranks)
    numerator = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_ranks, y_ranks))
    denominator = math.sqrt(
        sum((x - x_mean) ** 2 for x in x_ranks) *
        sum((y - y_mean) ** 2 for y in y_ranks)
    )
    return numerator / denominator if denominator > 0.0 else math.nan


def roc_auc(scores: Sequence[float], labels: Sequence[bool]) -> float:
    pairs = [(score, label) for score, label in zip(scores, labels)
             if math.isfinite(score)]
    positives = sum(1 for _score, label in pairs if label)
    negatives = len(pairs) - positives
    if positives == 0 or negatives == 0:
        return math.nan
    ranks = _ranks([item[0] for item in pairs])
    positive_rank_sum = sum(
        rank for rank, (_score, label) in zip(ranks, pairs) if label
    )
    return (positive_rank_sum - positives * (positives + 1) / 2.0) / (
        positives * negatives
    )


def top_error_growth_windows(
    estimates: Sequence[PoseSample], truths: Sequence[PoseSample],
    duration: float = 2.0, count: int = 10,
) -> list[dict[str, float]]:
    truth_stamps = [item.stamp for item in truths]
    alignment = _alignment(estimates, truths, truth_stamps)
    errors: list[tuple[float, float]] = []
    for estimate in estimates:
        truth = _nearest(truths, estimate.stamp, 0.1, truth_stamps)
        if truth is None:
            continue
        truth_xy = _aligned_truth_xy(truth, alignment)
        errors.append((estimate.stamp, math.hypot(
            estimate.x - truth_xy[0], estimate.y - truth_xy[1]
        )))
    candidates = []
    for index, (stamp, error) in enumerate(errors):
        target = stamp + duration
        end = min(range(index, len(errors)), key=lambda item: abs(errors[item][0] - target))
        if end <= index or abs(errors[end][0] - target) > 0.15:
            continue
        candidates.append((errors[end][1] - error, stamp, errors[end][0], error, errors[end][1]))
    selected = []
    for growth, start, end, start_error, end_error in sorted(candidates, reverse=True):
        if any(not (end <= item["start_stamp"] or start >= item["end_stamp"])
               for item in selected):
            continue
        selected.append({
            "growth": growth,
            "start_stamp": start,
            "end_stamp": end,
            "start_error": start_error,
            "end_error": end_error,
        })
        if len(selected) >= count:
            break
    return selected


def _feature_summary(samples: Sequence[QualitySample], axis: str) -> dict:
    correction_name = f"applied_{axis}"
    benefit_name = f"benefit_{axis}"
    eligible = [item for item in samples if abs(getattr(item, correction_name)) > 0.005]
    feature_names = [
        "matched_ratio", "retained_ratio", "iterations",
        "residual_p50", "residual_p70", "residual_p90",
        f"residual_mean_{axis}", f"residual_median_{axis}",
        f"residual_mad_{axis}", f"residual_positive_{axis}",
        f"residual_negative_{axis}", "anisotropy",
        f"divergence_velocity_{axis}",
    ]
    result = {
        "eligible_count": len(eligible),
        "beneficial_count": sum(getattr(item, benefit_name) > 0.0 for item in eligible),
        "harmful_count": sum(getattr(item, benefit_name) <= 0.0 for item in eligible),
        "features": {},
    }
    targets = [getattr(item, benefit_name) for item in eligible]
    labels = [target > 0.0 for target in targets]
    for name in feature_names:
        values = [float(getattr(item, name)) for item in eligible]
        auc = roc_auc(values, labels)
        result["features"][name] = {
            "spearman": spearman(values, targets),
            "roc_auc": auc,
            "auc_separation": max(auc, 1.0 - auc) if math.isfinite(auc) else math.nan,
        }
    return result


def _motion_summary(samples: Sequence[MotionWindowSample], axis: str) -> dict:
    correction_name = f"scan_to_map_{axis}"
    benefit_name = f"map_benefit_{axis}"
    eligible = [item for item in samples if abs(getattr(item, correction_name)) > 0.005]
    benefits = [getattr(item, benefit_name) for item in eligible]
    labels = [value > 0.0 for value in benefits]
    feature_names = [
        f"depth_{axis}", f"livox_{axis}", f"divergence_{axis}",
        f"relation_{axis}", f"ratio_{axis}", "imu_abs_yaw",
        "depth_fresh_ratio", "frames",
    ]
    return {
        "eligible_count": len(eligible),
        "beneficial_count": sum(value > 0.0 for value in benefits),
        "harmful_count": sum(value <= 0.0 for value in benefits),
        "features": {
            name: {
                "spearman": spearman(
                    [float(getattr(item, name)) for item in eligible], benefits
                ),
                "roc_auc": roc_auc(
                    [float(getattr(item, name)) for item in eligible], labels
                ),
            }
            for name in feature_names
        },
    }


def _write_csv(path: Path, samples: Iterable, sample_type) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(sample_type.__annotations__))
        writer.writeheader()
        for sample in samples:
            writer.writerow(asdict(sample))


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description="离线分析 scan-to-map 校正质量")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--odom-topic", default="/danger_search/localization/odom")
    parser.add_argument("--truth-topic", default="/ground_truth/base_w")
    parser.add_argument("--maximum-sync-gap", type=float, default=0.10)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--windows-csv", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error(f"bag 不存在：{args.bag}")

    map_frames: list[MapFrame] = []
    planar_frames: list[PlanarFrame] = []
    motion_windows: list[MotionWindow] = []
    with rosbag.Bag(str(args.bag), "r") as bag:
        estimates = _read_poses(bag, args.odom_topic)
        truths = _read_poses(bag, args.truth_topic)
        for _topic, message, _stamp in bag.read_messages(topics=["/rosout"]):
            text = str(getattr(message, "msg", ""))
            map_frame = parse_map_frame(text)
            if map_frame is not None:
                map_frames.append(map_frame)
            planar_frame = parse_planar_frame(text)
            if planar_frame is not None:
                planar_frames.append(planar_frame)
            motion_window = parse_motion_window(text)
            if motion_window is not None:
                motion_windows.append(motion_window)
    if not map_frames or not estimates or not truths:
        parser.error("bag 缺少逐帧定位诊断、odom 或离线真值")
    samples = build_quality_samples(
        sorted(map_frames, key=lambda item: item.stamp),
        sorted(planar_frames, key=lambda item: item.stamp),
        estimates, truths, args.maximum_sync_gap,
    )
    window_samples = build_motion_window_samples(
        sorted(motion_windows, key=lambda item: item.stamp),
        estimates, truths, args.maximum_sync_gap,
    )
    result = {
        "bag": str(args.bag.resolve()),
        "map_frame_count": len(map_frames),
        "aligned_sample_count": len(samples),
        "alignment_ratio": len(samples) / len(map_frames),
        "motion_window_count": len(motion_windows),
        "aligned_motion_window_count": len(window_samples),
        "motion_window_alignment_ratio": (
            len(window_samples) / len(motion_windows) if motion_windows else 0.0
        ),
        "axis_x": _feature_summary(samples, "x"),
        "axis_y": _feature_summary(samples, "y"),
        "motion_axis_x": _motion_summary(window_samples, "x"),
        "motion_axis_y": _motion_summary(window_samples, "y"),
        "benefit_spearman_with_error": spearman(
            [item.error_post for item in samples], [item.benefit for item in samples]
        ),
        "top_error_growth_windows": top_error_growth_windows(estimates, truths),
    }
    if args.csv is not None:
        _write_csv(args.csv, samples, QualitySample)
    if args.windows_csv is not None:
        _write_csv(args.windows_csv, window_samples, MotionWindowSample)
    payload = json.dumps(
        _json_safe(result), ensure_ascii=False, indent=2, sort_keys=True,
        allow_nan=False,
    )
    if args.json is not None:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

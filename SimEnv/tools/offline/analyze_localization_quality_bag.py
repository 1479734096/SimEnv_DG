#!/usr/bin/env python3
"""离线分析定位校正质量；真值和诊断日志不会反馈到生产节点。"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import random
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import rosbag
from scipy.spatial import cKDTree


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
    status: str = "accepted"
    accepted: bool = True
    has_delta: bool = True
    candidate_yaw: float = math.nan
    candidate_rotation_angle: float = math.nan


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
    yaw: float = math.nan
    benefit_forward: float = math.nan
    benefit_lateral: float = math.nan
    correction_forward: float = math.nan
    correction_lateral: float = math.nan
    divergence_forward: float = math.nan
    divergence_lateral: float = math.nan
    relation_forward: int = 0
    relation_lateral: int = 0
    direction_cosine: float = math.nan
    structure_valid: bool = False
    structure_ratio: float = math.nan
    structure_c_weak: float = math.nan
    structure_d_weak: float = math.nan
    structure_abs_c_weak: float = math.nan
    structure_relation_weak: int = 0


@dataclass(frozen=True)
class StructureFrame:
    stamp: float
    valid: bool
    point_count: int
    ratio: float
    weak_axis_x: float
    weak_axis_y: float


@dataclass(frozen=True)
class Pose3Sample:
    stamp: float
    x: float
    y: float
    z: float
    rotation: np.ndarray


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


@dataclass(frozen=True)
class SensorDeltaWindow:
    start_stamp: float
    end_stamp: float
    duration: float
    phase: str
    floor: int
    odom_start_x: float
    odom_start_y: float
    odom_end_x: float
    odom_end_y: float
    truth_start_x: float
    truth_start_y: float
    truth_end_x: float
    truth_end_y: float
    truth_distance: float
    rgbd_x: float
    rgbd_y: float
    livox_x: float
    livox_y: float
    scan_to_map_x: float
    scan_to_map_y: float
    rgbd_coverage: float
    livox_coverage: float
    rgbd_count: int
    livox_count: int
    yaw: float
    absolute_yaw_change: float
    depth_fresh_ratio: float
    truth_forward: float
    truth_lateral: float
    rgbd_error_forward: float
    rgbd_error_lateral: float
    livox_error_forward: float
    livox_error_lateral: float
    rgbd_error_norm: float
    livox_error_norm: float
    rgbd_rotation_mean: float = math.nan
    rgbd_rotation_max: float = math.nan
    rgbd_yaw_abs_mean: float = math.nan
    rgbd_yaw_abs_max: float = math.nan
    livox_rotation_mean: float = math.nan
    livox_rotation_max: float = math.nan
    livox_yaw_abs_mean: float = math.nan
    livox_yaw_abs_max: float = math.nan


@dataclass(frozen=True)
class MissionStateSample:
    stamp: float
    state: str
    floor: int


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
    rf"scan_stamp=(?P<stamp>{NUMBER}) frame_interval=(?P<interval>{NUMBER}) "
    rf"status=(?P<status>\S+) accepted=(?P<accepted>[01]) "
    rf"has_delta=(?P<has_delta>[01]).*?"
    rf"applied=\((?P<applied_x>{NUMBER}), (?P<applied_y>{NUMBER})\)"
    rf"(?: candidate_yaw=(?P<candidate_yaw>{NUMBER}) "
    rf"candidate_rotation_angle=(?P<candidate_rotation_angle>{NUMBER}))?"
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
        status=values["status"],
        accepted=values["accepted"] == "1",
        has_delta=values["has_delta"] == "1",
        candidate_yaw=(
            _finite_float(values["candidate_yaw"])
            if values["candidate_yaw"] is not None else math.nan
        ),
        candidate_rotation_angle=(
            _finite_float(values["candidate_rotation_angle"])
            if values["candidate_rotation_angle"] is not None else math.nan
        ),
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


def transition_bounds_from_rosout(
    entries: Sequence[tuple[float, str]],
) -> tuple[float, float] | None:
    start: float | None = None
    target_floor = None
    for stamp, text in entries:
        if not text.startswith("{"):
            continue
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            continue
        if payload.get("event") != "mission_state_transition":
            continue
        if (start is None and payload.get("old_state") == "EXPLORE_FLOOR"
                and payload.get("new_state") == "CHANGE_FLOOR"):
            current = payload.get("current_floor")
            target = payload.get("target_floor")
            if isinstance(current, int) and isinstance(target, int) and target != current:
                start = stamp
                target_floor = target
            continue
        if (start is not None and payload.get("current_task_phase") == "DONE"
                and payload.get("new_state") == "EXPLORE_FLOOR"
                and payload.get("current_floor") == target_floor):
            return start, stamp
    return None


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


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    roll_matrix = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    pitch_matrix = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    yaw_matrix = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return yaw_matrix @ pitch_matrix @ roll_matrix


def structure_frame_from_points(
    stamp: float,
    points_xy: np.ndarray,
    minimum_points: int = 30,
    maximum_isotropy_ratio: float = 0.5,
) -> StructureFrame:
    """用机器人局部 XY 点估计走廊强轴及其正交退化轴。"""
    points = np.asarray(points_xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("PCA 输入必须为 N×2 点集")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < minimum_points:
        return StructureFrame(stamp, False, len(points), math.nan,
                              math.nan, math.nan)
    centered = points - np.mean(points, axis=0)
    covariance = centered.T @ centered / float(len(centered))
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    maximum = float(eigenvalues[1])
    minimum = max(0.0, float(eigenvalues[0]))
    ratio = minimum / maximum if maximum > 1e-12 else math.nan
    if not math.isfinite(ratio) or ratio > maximum_isotropy_ratio:
        return StructureFrame(stamp, False, len(points), ratio,
                              math.nan, math.nan)
    strong = eigenvectors[:, 1].astype(float)
    if strong[0] < 0.0 or (abs(strong[0]) <= 1e-12 and strong[1] < 0.0):
        strong = -strong
    # 强轴固定朝机器人前方后，左手正交轴的符号也随之唯一确定。
    weak = np.array((-strong[1], strong[0]))
    return StructureFrame(stamp, True, len(points), ratio,
                          float(weak[0]), float(weak[1]))


def _read_scan_structures(
    bag: rosbag.Bag,
    topic: str = "/scan",
    maximum_radius: float = 4.0,
) -> list[StructureFrame]:
    base_to_livox_rotation = _rotation_from_rpy(0.0, 0.785, 0.0)
    base_to_livox_translation = np.array((0.2, 0.0, 0.08))
    output: list[StructureFrame] = []
    for _topic, message, bag_stamp in bag.read_messages(topics=[topic]):
        raw = np.asarray(
            [(float(point.x), float(point.y), float(point.z))
             for point in message.points],
            dtype=float,
        )
        stamp = _message_stamp(message, bag_stamp)
        if raw.size == 0:
            output.append(structure_frame_from_points(
                stamp, np.empty((0, 2), dtype=float)))
            continue
        finite = np.all(np.isfinite(raw), axis=1)
        ranges = np.linalg.norm(raw, axis=1)
        raw = raw[finite & (ranges >= 0.5) & (ranges <= 12.0)]
        base = raw @ base_to_livox_rotation.T + base_to_livox_translation
        local_xy = base[:, :2]
        local_xy = local_xy[np.linalg.norm(local_xy, axis=1) <= maximum_radius]
        output.append(structure_frame_from_points(stamp, local_xy))
    return output


def project_to_local(vector: tuple[float, float], yaw: float) -> tuple[float, float]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * vector[0] + sine * vector[1],
        -sine * vector[0] + cosine * vector[1],
    )


def _sign_relation(left: float, right: float, epsilon: float = 1e-12) -> int:
    product = left * right
    if product > epsilon:
        return 1
    if product < -epsilon:
        return -1
    return 0


def _direction_cosine(left: tuple[float, float], right: tuple[float, float]) -> float:
    denominator = math.hypot(*left) * math.hypot(*right)
    if denominator <= 1e-12:
        return math.nan
    return (left[0] * right[0] + left[1] * right[1]) / denominator


def _voxel_downsample(points: np.ndarray, size: float) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 3), dtype=float)
    keys = np.trunc(points / size).astype(np.int64)
    _unique, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def _lower_quantile(values: np.ndarray, fraction: float) -> float:
    ordered = np.sort(values)
    index = int(math.floor(fraction * float(len(ordered) - 1)))
    return float(ordered[index])


class DiagnosticMapReplay:
    """按生产匹配参数重建只读地图，并保留每个体素的插入阶段。"""

    def __init__(self):
        self._map: dict[tuple[int, int, int], tuple[np.ndarray, str, float]] = {}

    def reset(self) -> None:
        self._map.clear()

    @staticmethod
    def _map_key(point: np.ndarray) -> tuple[int, int, int]:
        key = np.floor(point / 0.12).astype(np.int64)
        return int(key[0]), int(key[1]), int(key[2])

    def _update(
        self,
        map_frame: np.ndarray,
        translation: np.ndarray,
        epoch: str,
        stamp: float,
    ) -> None:
        for point in map_frame:
            world = point + translation
            self._map.setdefault(self._map_key(world), (world, epoch, stamp))
        maximum_distance_squared = 12.0 ** 2
        self._map = {
            key: value for key, value in self._map.items()
            if float(np.dot(value[0] - translation, value[0] - translation))
            <= maximum_distance_squared
        }

    def process(
        self,
        oriented_vectors: np.ndarray,
        translation_prior: np.ndarray,
        epoch: str,
        stamp: float,
        update_translation: np.ndarray | None = None,
    ) -> dict:
        map_frame = _voxel_downsample(oriented_vectors, 0.12)
        source = _voxel_downsample(map_frame, 0.16)
        if len(source) < 80:
            return {"accepted": False, "status": "insufficient_source",
                    "correction": np.zeros(2), "retained": 0, "origins": {}}
        if not self._map:
            self._update(
                map_frame,
                update_translation if update_translation is not None
                else translation_prior,
                epoch,
                stamp,
            )
            return {"accepted": True, "status": "initial_frame",
                    "correction": np.zeros(2), "retained": len(source),
                    "origins": {}}

        values = list(self._map.values())
        map_points = np.asarray([item[0] for item in values], dtype=float)
        tree = cKDTree(map_points)
        estimate = np.asarray(translation_prior, dtype=float).copy()
        retained_indices = np.empty(0, dtype=np.int64)
        retained_count = 0
        valid_iteration = False
        for _iteration in range(12):
            world_source = source + estimate
            distances, indices = tree.query(
                world_source, k=1, distance_upper_bound=0.55
            )
            matched = np.isfinite(distances) & (indices < len(map_points))
            if int(np.count_nonzero(matched)) < 80:
                break
            matched_source = world_source[matched]
            matched_indices = indices[matched].astype(np.int64)
            residual = map_points[matched_indices] - matched_source
            planar_norm = np.linalg.norm(residual[:, :2], axis=1)
            cutoff = max(0.08, _lower_quantile(planar_norm, 0.70))
            retained_mask = planar_norm <= cutoff
            retained_count = int(np.count_nonzero(retained_mask))
            if retained_count < 80:
                break
            retained = residual[retained_mask, :2]
            retained_indices = matched_indices[retained_mask]
            delta = np.median(retained, axis=0)
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > 0.12:
                delta *= 0.12 / delta_norm
            estimate[:2] += delta
            valid_iteration = True
            if float(np.linalg.norm(delta)) < 0.0002:
                break
        if not valid_iteration or retained_count < 80:
            return {"accepted": False, "status": "insufficient_correspondences",
                    "correction": np.zeros(2), "retained": retained_count,
                    "origins": {}}

        correction = estimate[:2] - translation_prior[:2]
        correction_norm = float(np.linalg.norm(correction))
        if correction_norm > 0.10:
            correction *= 0.10 / correction_norm
            estimate[:2] = translation_prior[:2] + correction
        origins: dict[str, int] = {}
        for index in retained_indices:
            origin = values[int(index)][1]
            origins[origin] = origins.get(origin, 0) + 1
        self._update(
            map_frame,
            update_translation if update_translation is not None else estimate,
            epoch,
            stamp,
        )
        return {"accepted": True, "status": "accepted",
                "correction": correction, "retained": retained_count,
                "origins": origins}


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


def _quaternion_matrix(quaternion) -> np.ndarray:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("位姿四元数无效")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array((
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)),
    ))


def _read_pose3(bag: rosbag.Bag, topic: str) -> list[Pose3Sample]:
    samples: list[Pose3Sample] = []
    for _topic, message, bag_stamp in bag.read_messages(topics=[topic]):
        pose = getattr(getattr(message, "pose", None), "pose", None)
        if pose is None:
            continue
        samples.append(Pose3Sample(
            _message_stamp(message, bag_stamp),
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            _quaternion_matrix(pose.orientation),
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


def interpolate_pose(
    samples: Sequence[PoseSample], stamp: float, maximum_gap: float,
) -> PoseSample | None:
    """在线性位置与最短 yaw 弧上插值，避免同窗比较引入最近邻误差。"""
    if not samples:
        return None
    stamps = [item.stamp for item in samples]
    index = bisect.bisect_left(stamps, stamp)
    if index < len(samples) and abs(samples[index].stamp - stamp) <= 1e-9:
        return samples[index]
    if index == 0 or index >= len(samples):
        return None
    left = samples[index - 1]
    right = samples[index]
    if stamp - left.stamp > maximum_gap or right.stamp - stamp > maximum_gap:
        return None
    interval = right.stamp - left.stamp
    if interval <= 0.0:
        return None
    ratio = (stamp - left.stamp) / interval
    yaw_delta = math.atan2(
        math.sin(right.yaw - left.yaw), math.cos(right.yaw - left.yaw)
    )
    return PoseSample(
        stamp=stamp,
        x=left.x + ratio * (right.x - left.x),
        y=left.y + ratio * (right.y - left.y),
        yaw=math.atan2(
            math.sin(left.yaw + ratio * yaw_delta),
            math.cos(left.yaw + ratio * yaw_delta),
        ),
    )


def mission_state_samples_from_rosout(
    entries: Sequence[tuple[float, str]],
) -> list[MissionStateSample]:
    samples: list[MissionStateSample] = []
    for stamp, text in entries:
        if not text.startswith("{"):
            continue
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            continue
        if payload.get("event") != "mission_state_transition":
            continue
        state = payload.get("new_state")
        floor = payload.get("current_floor")
        if isinstance(state, str) and isinstance(floor, int):
            samples.append(MissionStateSample(stamp, state, floor))
    return sorted(samples, key=lambda item: item.stamp)


def floor_transition_intervals_from_rosout(
    entries: Sequence[tuple[float, str]],
) -> list[dict]:
    intervals: list[dict] = []
    pending: dict | None = None
    for stamp, text in entries:
        if not text.startswith("{"):
            continue
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            continue
        if payload.get("event") != "mission_state_transition":
            continue
        current = payload.get("current_floor")
        target = payload.get("target_floor")
        if (payload.get("old_state") == "EXPLORE_FLOOR"
                and payload.get("new_state") == "CHANGE_FLOOR"
                and isinstance(current, int) and isinstance(target, int)
                and current != target):
            pending = {"start": stamp, "source": current, "target": target}
            continue
        if (pending is not None
                and payload.get("new_state") == "EXPLORE_FLOOR"
                and payload.get("current_task_phase") == "DONE"
                and payload.get("current_floor") == pending["target"]):
            intervals.append({**pending, "end": stamp})
            pending = None
    return intervals


def mission_context_at_stamp(
    stamp: float,
    states: Sequence[MissionStateSample],
    transitions: Sequence[dict],
) -> tuple[str, int]:
    for transition in transitions:
        if transition["start"] <= stamp <= transition["end"]:
            return (
                f"elevator_transition_{transition['source']}_to_"
                f"{transition['target']}",
                int(transition["source"]),
            )
        if transition["end"] < stamp <= transition["end"] + 5.0:
            return f"post_elevator_floor_{transition['target']}_5s", int(
                transition["target"]
            )
    if not states:
        return "unknown", -1
    stamps = [item.stamp for item in states]
    index = bisect.bisect_right(stamps, stamp) - 1
    if index < 0:
        return "before_mission", -1
    sample = states[index]
    return sample.state.lower(), sample.floor


def _aligned_truth_delta(
    start: PoseSample, end: PoseSample, alignment,
) -> tuple[float, float]:
    _estimate0, _truth0, cosine, sine = alignment
    dx = end.x - start.x
    dy = end.y - start.y
    return cosine * dx - sine * dy, sine * dx + cosine * dy


def _absolute_yaw_change(
    samples: Sequence[PoseSample], start: float, end: float,
    start_pose: PoseSample, end_pose: PoseSample,
) -> float:
    sequence = [start_pose]
    sequence.extend(item for item in samples if start < item.stamp < end)
    sequence.append(end_pose)
    return sum(
        abs(math.atan2(
            math.sin(right.yaw - left.yaw), math.cos(right.yaw - left.yaw)
        ))
        for left, right in zip(sequence, sequence[1:])
    )


def build_sensor_delta_windows(
    planar_frames: Sequence[PlanarFrame],
    map_frames: Sequence[MapFrame],
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
    rosout_entries: Sequence[tuple[float, str]],
    motion_windows: Sequence[MotionWindow] = (),
    window_duration: float = 0.5,
    minimum_coverage: float = 0.60,
    maximum_sync_gap: float = 0.15,
) -> list[SensorDeltaWindow]:
    """在固定公共同窗中公平比较 RGB-D 与 Livox 帧间平移。"""
    if window_duration <= 0.0 or not 0.0 <= minimum_coverage <= 1.0:
        raise ValueError("同窗长度或覆盖率无效")
    if not estimates or not truths:
        return []
    valid_frames = [
        item for item in planar_frames
        if item.accepted and item.has_delta and item.interval > 0.0
        and math.isfinite(item.interval)
        and math.isfinite(item.applied_x) and math.isfinite(item.applied_y)
    ]
    depth_frames = [
        item for item in valid_frames if item.source == "rgbd_frame_gicp"
    ]
    livox_frames = [
        item for item in valid_frames if item.source == "livox_frame_gicp"
    ]
    if not depth_frames or not livox_frames:
        return []
    alignment = _alignment(estimates, truths)
    states = mission_state_samples_from_rosout(rosout_entries)
    transitions = floor_transition_intervals_from_rosout(rosout_entries)
    motion_stamps = [item.stamp for item in motion_windows]
    first_stamp = max(estimates[0].stamp, truths[0].stamp)
    last_stamp = min(estimates[-1].stamp, truths[-1].stamp)
    first_edge = math.ceil(first_stamp / window_duration) * window_duration
    last_edge = math.floor(last_stamp / window_duration) * window_duration
    output: list[SensorDeltaWindow] = []
    start = first_edge
    while start + window_duration <= last_edge + 1e-9:
        end = start + window_duration
        midpoint = 0.5 * (start + end)

        def frames_in_window(source: Sequence[PlanarFrame]) -> list[PlanarFrame]:
            return [
                item for item in source
                if start <= item.stamp - 0.5 * item.interval < end
            ]

        depth = frames_in_window(depth_frames)
        livox = frames_in_window(livox_frames)
        depth_coverage = min(
            1.0, sum(item.interval for item in depth) / window_duration
        )
        livox_coverage = min(
            1.0, sum(item.interval for item in livox) / window_duration
        )
        if (depth_coverage < minimum_coverage
                or livox_coverage < minimum_coverage):
            start = end
            continue
        estimate_start = interpolate_pose(estimates, start, maximum_sync_gap)
        estimate_end = interpolate_pose(estimates, end, maximum_sync_gap)
        estimate_mid = interpolate_pose(estimates, midpoint, maximum_sync_gap)
        truth_start = interpolate_pose(truths, start, maximum_sync_gap)
        truth_end = interpolate_pose(truths, end, maximum_sync_gap)
        if None in (estimate_start, estimate_end, estimate_mid,
                    truth_start, truth_end):
            start = end
            continue
        truth_delta = _aligned_truth_delta(truth_start, truth_end, alignment)
        depth_delta = (
            sum(item.applied_x for item in depth),
            sum(item.applied_y for item in depth),
        )
        livox_delta = (
            sum(item.applied_x for item in livox),
            sum(item.applied_y for item in livox),
        )
        def rotation_summary(
            frames: Sequence[PlanarFrame], attribute: str,
        ) -> tuple[float, float]:
            values = [
                abs(getattr(item, attribute)) for item in frames
                if math.isfinite(getattr(item, attribute))
            ]
            if not values:
                return math.nan, math.nan
            return sum(values) / len(values), max(values)

        depth_rotation_mean, depth_rotation_max = rotation_summary(
            depth, "candidate_rotation_angle"
        )
        depth_yaw_mean, depth_yaw_max = rotation_summary(
            depth, "candidate_yaw"
        )
        livox_rotation_mean, livox_rotation_max = rotation_summary(
            livox, "candidate_rotation_angle"
        )
        livox_yaw_mean, livox_yaw_max = rotation_summary(
            livox, "candidate_yaw"
        )
        map_delta = (
            sum(item.applied_x for item in map_frames
                if start <= item.stamp < end),
            sum(item.applied_y for item in map_frames
                if start <= item.stamp < end),
        )
        truth_local = project_to_local(truth_delta, estimate_mid.yaw)
        depth_error_local = project_to_local(
            (depth_delta[0] - truth_delta[0],
             depth_delta[1] - truth_delta[1]),
            estimate_mid.yaw,
        )
        livox_error_local = project_to_local(
            (livox_delta[0] - truth_delta[0],
             livox_delta[1] - truth_delta[1]),
            estimate_mid.yaw,
        )
        phase, floor = mission_context_at_stamp(
            midpoint, states, transitions
        )
        motion = _nearest(
            motion_windows, midpoint, 1.5, motion_stamps
        ) if motion_windows else None
        depth_fresh_ratio = (
            motion.depth_fresh_ratio if motion is not None else math.nan
        )
        output.append(SensorDeltaWindow(
            start_stamp=start,
            end_stamp=end,
            duration=window_duration,
            phase=phase,
            floor=floor,
            odom_start_x=estimate_start.x,
            odom_start_y=estimate_start.y,
            odom_end_x=estimate_end.x,
            odom_end_y=estimate_end.y,
            truth_start_x=_aligned_truth_xy(truth_start, alignment)[0],
            truth_start_y=_aligned_truth_xy(truth_start, alignment)[1],
            truth_end_x=_aligned_truth_xy(truth_end, alignment)[0],
            truth_end_y=_aligned_truth_xy(truth_end, alignment)[1],
            truth_distance=math.hypot(*truth_delta),
            rgbd_x=depth_delta[0],
            rgbd_y=depth_delta[1],
            livox_x=livox_delta[0],
            livox_y=livox_delta[1],
            scan_to_map_x=map_delta[0],
            scan_to_map_y=map_delta[1],
            rgbd_coverage=depth_coverage,
            livox_coverage=livox_coverage,
            rgbd_count=len(depth),
            livox_count=len(livox),
            yaw=estimate_mid.yaw,
            absolute_yaw_change=_absolute_yaw_change(
                estimates, start, end, estimate_start, estimate_end
            ),
            depth_fresh_ratio=depth_fresh_ratio,
            truth_forward=truth_local[0],
            truth_lateral=truth_local[1],
            rgbd_error_forward=depth_error_local[0],
            rgbd_error_lateral=depth_error_local[1],
            livox_error_forward=livox_error_local[0],
            livox_error_lateral=livox_error_local[1],
            rgbd_error_norm=math.hypot(*depth_error_local),
            livox_error_norm=math.hypot(*livox_error_local),
            rgbd_rotation_mean=depth_rotation_mean,
            rgbd_rotation_max=depth_rotation_max,
            rgbd_yaw_abs_mean=depth_yaw_mean,
            rgbd_yaw_abs_max=depth_yaw_max,
            livox_rotation_mean=livox_rotation_mean,
            livox_rotation_max=livox_rotation_max,
            livox_yaw_abs_mean=livox_yaw_mean,
            livox_yaw_abs_max=livox_yaw_max,
        ))
        start = end
    return output


def _percentile(values: Sequence[float], fraction: float) -> float:
    finite = sorted(float(value) for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = fraction * (len(finite) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    weight = position - lower
    return finite[lower] * (1.0 - weight) + finite[upper] * weight


def _sensor_source_metrics(
    windows: Sequence[SensorDeltaWindow], source: str,
) -> dict:
    if source not in ("rgbd", "livox"):
        raise ValueError("未知的里程计来源")
    norms = [getattr(item, f"{source}_error_norm") for item in windows]
    forward = [getattr(item, f"{source}_error_forward") for item in windows]
    lateral = [getattr(item, f"{source}_error_lateral") for item in windows]
    rotation_mean = [
        getattr(item, f"{source}_rotation_mean") for item in windows
    ]
    rotation_max = [
        getattr(item, f"{source}_rotation_max") for item in windows
    ]
    endpoint_x = 0.0
    endpoint_y = 0.0
    for item in windows:
        truth_x = item.truth_end_x - item.truth_start_x
        truth_y = item.truth_end_y - item.truth_start_y
        endpoint_x += getattr(item, f"{source}_x") - truth_x
        endpoint_y += getattr(item, f"{source}_y") - truth_y
    return {
        "count": len(windows),
        "rmse": (
            math.sqrt(sum(value * value for value in norms) / len(norms))
            if norms else math.nan
        ),
        "p95": _percentile(norms, 0.95),
        "mean_error": sum(norms) / len(norms) if norms else math.nan,
        "mean_forward_bias": (
            sum(forward) / len(forward) if forward else math.nan
        ),
        "mean_lateral_bias": (
            sum(lateral) / len(lateral) if lateral else math.nan
        ),
        "endpoint_error": math.hypot(endpoint_x, endpoint_y),
        "endpoint_error_x": endpoint_x,
        "endpoint_error_y": endpoint_y,
        "hidden_rotation_window_mean_p50": _percentile(rotation_mean, 0.50),
        "hidden_rotation_window_max_p95": _percentile(rotation_max, 0.95),
        "hidden_rotation_error_spearman": spearman(rotation_mean, norms),
    }


def _sensor_comparison_group(
    windows: Sequence[SensorDeltaWindow], minimum_truth_motion: float,
) -> dict:
    moving = [
        item for item in windows if item.truth_distance >= minimum_truth_motion
    ]
    rgbd = _sensor_source_metrics(moving, "rgbd")
    livox = _sensor_source_metrics(moving, "livox")

    def improvement(key: str) -> float:
        baseline = rgbd[key]
        candidate = livox[key]
        if not math.isfinite(baseline) or baseline <= 1e-12:
            return math.nan
        return (baseline - candidate) / baseline

    return {
        "window_count": len(windows),
        "moving_window_count": len(moving),
        "truth_distance": sum(item.truth_distance for item in moving),
        "rgbd": rgbd,
        "livox": livox,
        "livox_relative_improvement": {
            "rmse": improvement("rmse"),
            "p95": improvement("p95"),
            "endpoint_error": improvement("endpoint_error"),
        },
    }


def sensor_delta_summary(
    windows: Sequence[SensorDeltaWindow],
    minimum_truth_motion: float = 0.02,
) -> dict:
    phase_names = sorted({item.phase for item in windows})
    groups = {
        "all": _sensor_comparison_group(windows, minimum_truth_motion),
        "low_rotation": _sensor_comparison_group(
            [item for item in windows if item.absolute_yaw_change <= 0.10],
            minimum_truth_motion,
        ),
        "high_rotation": _sensor_comparison_group(
            [item for item in windows if item.absolute_yaw_change >= 0.25],
            minimum_truth_motion,
        ),
        "elevator_context": _sensor_comparison_group(
            [item for item in windows
             if item.phase.startswith("elevator_transition_")
             or item.phase.startswith("post_elevator_")],
            minimum_truth_motion,
        ),
    }
    groups["by_phase"] = {
        name: _sensor_comparison_group(
            [item for item in windows if item.phase == name],
            minimum_truth_motion,
        )
        for name in phase_names
    }
    overall = groups["all"]
    low_rotation = groups["low_rotation"]
    rmse_gain = overall["livox_relative_improvement"]["rmse"]
    p95_gain = overall["livox_relative_improvement"]["p95"]
    rgbd_low = low_rotation["rgbd"]["rmse"]
    livox_low = low_rotation["livox"]["rmse"]
    endpoint_gain = overall["livox_relative_improvement"]["endpoint_error"]
    enough_samples = overall["moving_window_count"] >= 20
    livox_primary_candidate = bool(
        enough_samples
        and math.isfinite(rmse_gain) and rmse_gain >= 0.15
        and math.isfinite(p95_gain) and p95_gain >= 0.15
        and math.isfinite(endpoint_gain) and endpoint_gain > 0.0
        and math.isfinite(rgbd_low) and math.isfinite(livox_low)
        and livox_low <= 1.05 * rgbd_low
    )
    return {
        "window_duration": windows[0].duration if windows else math.nan,
        "minimum_truth_motion": minimum_truth_motion,
        "groups": groups,
        "livox_primary_candidate": livox_primary_candidate,
        "decision_reason": (
            "通过预注册门槛" if livox_primary_candidate
            else "未同时满足 RMSE/P95 改善、低转角不退化和终点误差改善"
        ),
    }


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
    structures: Sequence[StructureFrame] = (),
) -> list[MotionWindowSample]:
    truth_stamps = [item.stamp for item in truths]
    estimate_stamps = [item.stamp for item in estimates]
    structure_stamps = [item.stamp for item in structures]
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
        post_world = (
            estimate.x - aligned_truth[0], estimate.y - aligned_truth[1]
        )
        pre_world = (
            post_world[0] - window.scan_to_map_x,
            post_world[1] - window.scan_to_map_y,
        )
        post_local = project_to_local(post_world, estimate.yaw)
        pre_local = project_to_local(pre_world, estimate.yaw)
        correction_local = project_to_local(
            (window.scan_to_map_x, window.scan_to_map_y), estimate.yaw
        )
        divergence_local = project_to_local(
            (window.divergence_x, window.divergence_y), estimate.yaw
        )
        structure = _nearest(
            structures, window.stamp, 0.15, structure_stamps
        )
        structure_valid = bool(structure is not None and structure.valid)
        if structure_valid:
            weak = (structure.weak_axis_x, structure.weak_axis_y)
            structure_c_weak = (
                correction_local[0] * weak[0] + correction_local[1] * weak[1]
            )
            structure_d_weak = (
                divergence_local[0] * weak[0] + divergence_local[1] * weak[1]
            )
            structure_ratio = structure.ratio
        else:
            structure_c_weak = math.nan
            structure_d_weak = math.nan
            structure_ratio = structure.ratio if structure is not None else math.nan
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
            yaw=estimate.yaw,
            benefit_forward=abs(pre_local[0]) - abs(post_local[0]),
            benefit_lateral=abs(pre_local[1]) - abs(post_local[1]),
            correction_forward=correction_local[0],
            correction_lateral=correction_local[1],
            divergence_forward=divergence_local[0],
            divergence_lateral=divergence_local[1],
            relation_forward=_sign_relation(
                correction_local[0], divergence_local[0]
            ),
            relation_lateral=_sign_relation(
                correction_local[1], divergence_local[1]
            ),
            direction_cosine=_direction_cosine(
                correction_local, divergence_local
            ),
            structure_valid=structure_valid,
            structure_ratio=structure_ratio,
            structure_c_weak=structure_c_weak,
            structure_d_weak=structure_d_weak,
            structure_abs_c_weak=abs(structure_c_weak),
            structure_relation_weak=_sign_relation(
                structure_c_weak, structure_d_weak
            ),
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


def describe_error_growth_windows(
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
    sensor_windows: Sequence[SensorDeltaWindow],
    rosout_entries: Sequence[tuple[float, str]],
    structures: Sequence[StructureFrame] = (),
    duration: float = 2.0,
    count: int = 10,
) -> list[dict]:
    """为最大漂移增长窗口补齐位置、任务阶段和候选误差来源。"""
    alignment = _alignment(estimates, truths)
    states = mission_state_samples_from_rosout(rosout_entries)
    transitions = floor_transition_intervals_from_rosout(rosout_entries)
    structure_stamps = [item.stamp for item in structures]
    details: list[dict] = []
    for candidate in top_error_growth_windows(
        estimates, truths, duration=duration, count=count
    ):
        start_stamp = candidate["start_stamp"]
        end_stamp = candidate["end_stamp"]
        midpoint = 0.5 * (start_stamp + end_stamp)
        estimate_start = interpolate_pose(estimates, start_stamp, 0.15)
        estimate_end = interpolate_pose(estimates, end_stamp, 0.15)
        estimate_mid = interpolate_pose(estimates, midpoint, 0.15)
        truth_start = interpolate_pose(truths, start_stamp, 0.15)
        truth_end = interpolate_pose(truths, end_stamp, 0.15)
        if None in (estimate_start, estimate_end, estimate_mid,
                    truth_start, truth_end):
            continue
        aligned_truth_start = _aligned_truth_xy(truth_start, alignment)
        aligned_truth_end = _aligned_truth_xy(truth_end, alignment)
        truth_delta = (
            aligned_truth_end[0] - aligned_truth_start[0],
            aligned_truth_end[1] - aligned_truth_start[1],
        )
        start_error = (
            estimate_start.x - aligned_truth_start[0],
            estimate_start.y - aligned_truth_start[1],
        )
        end_error = (
            estimate_end.x - aligned_truth_end[0],
            estimate_end.y - aligned_truth_end[1],
        )
        growth_vector = (
            end_error[0] - start_error[0], end_error[1] - start_error[1]
        )
        covered = [
            item for item in sensor_windows
            if item.start_stamp >= start_stamp - 1e-6
            and item.end_stamp <= end_stamp + 1e-6
        ]

        def summed(prefix: str) -> tuple[float, float]:
            return (
                sum(getattr(item, f"{prefix}_x") for item in covered),
                sum(getattr(item, f"{prefix}_y") for item in covered),
            )

        rgbd_delta = summed("rgbd")
        livox_delta = summed("livox")
        map_delta = summed("scan_to_map")
        sensor_truth_delta = (
            sum(item.truth_end_x - item.truth_start_x for item in covered),
            sum(item.truth_end_y - item.truth_start_y for item in covered),
        )
        rgbd_error = (
            rgbd_delta[0] - sensor_truth_delta[0],
            rgbd_delta[1] - sensor_truth_delta[1],
        )
        livox_error = (
            livox_delta[0] - sensor_truth_delta[0],
            livox_delta[1] - sensor_truth_delta[1],
        )
        rgbd_error_local = project_to_local(rgbd_error, estimate_mid.yaw)
        livox_error_local = project_to_local(livox_error, estimate_mid.yaw)
        map_local = project_to_local(map_delta, estimate_mid.yaw)
        growth_local = project_to_local(growth_vector, estimate_mid.yaw)
        phase, floor = mission_context_at_stamp(
            midpoint, states, transitions
        )
        structure = _nearest(
            structures, midpoint, 0.20, structure_stamps
        ) if structures else None
        rgbd_norm = math.hypot(*rgbd_error)
        livox_norm = math.hypot(*livox_error)
        map_growth_cosine = _direction_cosine(map_delta, growth_vector)
        hypotheses: list[str] = []
        if covered and livox_norm + 0.02 < rgbd_norm:
            hypotheses.append("rgbd_relative_underperformance")
        elif covered and rgbd_norm + 0.02 < livox_norm:
            hypotheses.append("livox_relative_underperformance")
        if (math.hypot(*map_delta) >= 0.02
                and math.isfinite(map_growth_cosine)
                and map_growth_cosine >= 0.5):
            hypotheses.append("scan_to_map_same_direction_as_error_growth")
        absolute_yaw_change = sum(
            item.absolute_yaw_change for item in covered
        )
        if absolute_yaw_change >= 0.25:
            hypotheses.append("high_rotation")
        if (phase.startswith("elevator_transition_")
                or phase.startswith("post_elevator_")):
            hypotheses.append("elevator_context")
        if not hypotheses:
            hypotheses.append("mixed_or_unresolved")
        details.append({
            **candidate,
            "phase": phase,
            "floor": floor,
            "odom_start": [estimate_start.x, estimate_start.y],
            "odom_end": [estimate_end.x, estimate_end.y],
            "truth_start": list(aligned_truth_start),
            "truth_end": list(aligned_truth_end),
            "error_start": list(start_error),
            "error_end": list(end_error),
            "error_growth_local": list(growth_local),
            "truth_delta": list(truth_delta),
            "sensor_truth_delta": list(sensor_truth_delta),
            "rgbd_delta": list(rgbd_delta),
            "livox_delta": list(livox_delta),
            "scan_to_map_delta": list(map_delta),
            "rgbd_error_local": list(rgbd_error_local),
            "livox_error_local": list(livox_error_local),
            "scan_to_map_local": list(map_local),
            "rgbd_error_norm": rgbd_norm,
            "livox_error_norm": livox_norm,
            "scan_to_map_growth_cosine": map_growth_cosine,
            "common_window_count": len(covered),
            "common_window_duration": sum(item.duration for item in covered),
            "rgbd_coverage_mean": (
                sum(item.rgbd_coverage for item in covered) / len(covered)
                if covered else math.nan
            ),
            "livox_coverage_mean": (
                sum(item.livox_coverage for item in covered) / len(covered)
                if covered else math.nan
            ),
            "absolute_yaw_change": absolute_yaw_change,
            "depth_fresh_ratio_mean": (
                sum(item.depth_fresh_ratio for item in covered
                    if math.isfinite(item.depth_fresh_ratio)) /
                sum(math.isfinite(item.depth_fresh_ratio) for item in covered)
                if any(math.isfinite(item.depth_fresh_ratio) for item in covered)
                else math.nan
            ),
            "structure_valid": bool(structure is not None and structure.valid),
            "structure_ratio": (
                structure.ratio if structure is not None else math.nan
            ),
            "diagnostic_hypotheses": hypotheses,
        })
    return details


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


def block_bootstrap_auc(
    scores: Sequence[float],
    labels: Sequence[bool],
    block_size: int = 5,
    repetitions: int = 1000,
    seed: int = 20260723,
) -> dict:
    pairs = [
        (float(score), bool(label))
        for score, label in zip(scores, labels)
        if math.isfinite(score)
    ]
    point = roc_auc([item[0] for item in pairs], [item[1] for item in pairs])
    if len(pairs) < 2 or block_size <= 0 or repetitions <= 0:
        return {"auc": point, "ci95_low": math.nan, "ci95_high": math.nan,
                "valid_repetitions": 0}
    generator = random.Random(seed)
    bootstrap: list[float] = []
    attempts = 0
    maximum_attempts = max(repetitions * 20, repetitions)
    while len(bootstrap) < repetitions and attempts < maximum_attempts:
        attempts += 1
        sampled: list[tuple[float, bool]] = []
        while len(sampled) < len(pairs):
            start = generator.randrange(len(pairs))
            sampled.extend(
                pairs[(start + offset) % len(pairs)]
                for offset in range(block_size)
            )
        sampled = sampled[:len(pairs)]
        value = roc_auc(
            [item[0] for item in sampled], [item[1] for item in sampled]
        )
        if math.isfinite(value):
            bootstrap.append(value)
        elif all(item[1] for item in pairs) or not any(item[1] for item in pairs):
            break
    if not bootstrap:
        return {"auc": point, "ci95_low": math.nan, "ci95_high": math.nan,
                "valid_repetitions": 0}
    bootstrap.sort()

    def percentile(fraction: float) -> float:
        position = fraction * (len(bootstrap) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        weight = position - lower
        return bootstrap[lower] * (1.0 - weight) + bootstrap[upper] * weight

    return {
        "auc": point,
        "ci95_low": percentile(0.025),
        "ci95_high": percentile(0.975),
        "valid_repetitions": len(bootstrap),
    }


LOCAL_FEATURES = (
    "correction_forward",
    "divergence_forward",
    "relation_forward",
    "correction_lateral",
    "divergence_lateral",
    "relation_lateral",
    "direction_cosine",
    "structure_abs_c_weak",
    "structure_c_weak",
    "structure_d_weak",
    "structure_relation_weak",
)


def _local_feature_summary(samples: Sequence[MotionWindowSample]) -> dict:
    eligible = [
        item for item in samples
        if math.hypot(item.scan_to_map_x, item.scan_to_map_y) > 0.005
        and abs(item.map_benefit) > 0.005
    ]
    labels = [item.map_benefit > 0.0 for item in eligible]
    output = {
        "eligible_count": len(eligible),
        "necessary_count": sum(labels),
        "harmful_count": len(labels) - sum(labels),
        "neutral_excluded_count": sum(
            math.hypot(item.scan_to_map_x, item.scan_to_map_y) > 0.005
            and abs(item.map_benefit) <= 0.005
            for item in samples
        ),
        "features": {},
    }
    for feature_index, name in enumerate(LOCAL_FEATURES):
        pairs = [
            (float(getattr(item, name)), label, item.map_benefit)
            for item, label in zip(eligible, labels)
            if math.isfinite(float(getattr(item, name)))
            and (not name.startswith("structure_") or item.structure_valid)
        ]
        values = [item[0] for item in pairs]
        feature_labels = [item[1] for item in pairs]
        benefits = [item[2] for item in pairs]
        statistics = block_bootstrap_auc(
            values, feature_labels, seed=20260723 + feature_index
        )
        auc = statistics["auc"]
        statistics.update({
            "sample_count": len(values),
            "necessary_count": sum(feature_labels),
            "harmful_count": len(feature_labels) - sum(feature_labels),
            "auc_separation": (
                max(auc, 1.0 - auc) if math.isfinite(auc) else math.nan
            ),
            "separation_ci95_low": (
                statistics["ci95_low"] if math.isfinite(auc) and auc >= 0.5
                else 1.0 - statistics["ci95_high"] if math.isfinite(auc)
                else math.nan
            ),
            "separation_ci95_high": (
                statistics["ci95_high"] if math.isfinite(auc) and auc >= 0.5
                else 1.0 - statistics["ci95_low"] if math.isfinite(auc)
                else math.nan
            ),
            "direction": (
                "necessary_high" if math.isfinite(auc) and auc >= 0.5
                else "necessary_low" if math.isfinite(auc)
                else "unavailable"
            ),
            "spearman_with_benefit": spearman(values, benefits),
        })
        output["features"][name] = statistics
    return output


def local_motion_summaries(samples: Sequence[MotionWindowSample]) -> dict:
    return {
        "all": _local_feature_summary(samples),
        "low_rotation": _local_feature_summary(
            [item for item in samples if item.imu_abs_yaw <= 0.3]
        ),
        "depth_fresh": _local_feature_summary(
            [item for item in samples if item.depth_fresh_ratio >= 0.9]
        ),
        "low_rotation_depth_fresh": _local_feature_summary([
            item for item in samples
            if item.imu_abs_yaw <= 0.3 and item.depth_fresh_ratio >= 0.9
        ]),
    }


def _epoch_for_stamp(stamp: float, transition: tuple[float, float]) -> str:
    if stamp < transition[0]:
        return "source_floor"
    if stamp <= transition[1]:
        return "elevator_transition"
    return "target_floor"


def _error_metrics(values: Sequence[float]) -> dict:
    finite = sorted(float(value) for value in values if math.isfinite(value))
    if not finite:
        return {"count": 0, "rmse": math.nan, "p95": math.nan,
                "mean": math.nan}
    p95_index = int(math.floor(0.95 * float(len(finite) - 1)))
    return {
        "count": len(finite),
        "rmse": math.sqrt(sum(value * value for value in finite) / len(finite)),
        "p95": finite[p95_index],
        "mean": sum(finite) / len(finite),
    }


def analyze_map_alias(
    bag: rosbag.Bag,
    map_frames: Sequence[MapFrame],
    estimates: Sequence[PoseSample],
    truths: Sequence[PoseSample],
    pose3_samples: Sequence[Pose3Sample],
    transition: tuple[float, float] | None,
    maximum_gap: float,
) -> dict:
    if transition is None:
        return {"available": False, "reason": "missing_floor_transition"}
    map_stamps = [item.stamp for item in map_frames]
    estimate_stamps = [item.stamp for item in estimates]
    truth_stamps = [item.stamp for item in truths]
    pose3_stamps = [item.stamp for item in pose3_samples]
    alignment = _alignment(estimates, truths, truth_stamps)
    extrinsic_rotation = _rotation_from_rpy(0.0, 0.785, 0.0)
    extrinsic_translation = np.array((0.2, 0.0, 0.08))
    baseline = DiagnosticMapReplay()
    reset = DiagnosticMapReplay()
    reset_applied = False
    correction_errors: list[float] = []
    post_frame_count = 0
    post_accepted_count = 0
    origin_totals = {
        "necessary": {"source_floor": 0, "elevator_transition": 0,
                      "target_floor": 0},
        "harmful": {"source_floor": 0, "elevator_transition": 0,
                    "target_floor": 0},
        "neutral": {"source_floor": 0, "elevator_transition": 0,
                    "target_floor": 0},
    }
    reset_original_errors: list[float] = []
    reset_candidate_errors: list[float] = []
    reset_improvement: list[float] = []
    processed = 0
    for _topic, message, bag_stamp in bag.read_messages(topics=["/scan"]):
        stamp = _message_stamp(message, bag_stamp)
        frame = _nearest(map_frames, stamp, 0.06, map_stamps)
        estimate = _nearest(estimates, stamp, maximum_gap, estimate_stamps)
        truth = _nearest(truths, stamp, maximum_gap, truth_stamps)
        pose3 = _nearest(pose3_samples, stamp, maximum_gap, pose3_stamps)
        if frame is None or estimate is None or truth is None or pose3 is None:
            continue
        raw = np.asarray(
            [(float(point.x), float(point.y), float(point.z))
             for point in message.points], dtype=float
        )
        if raw.size == 0:
            continue
        finite = np.all(np.isfinite(raw), axis=1)
        ranges = np.linalg.norm(raw, axis=1)
        raw = raw[finite & (ranges >= 0.5) & (ranges <= 12.0)]
        world_livox_rotation = pose3.rotation @ extrinsic_rotation
        oriented = raw @ world_livox_rotation.T
        final_sensor = np.array((pose3.x, pose3.y, pose3.z)) + (
            pose3.rotation @ extrinsic_translation
        )
        prior_sensor = final_sensor.copy()
        prior_sensor[:2] -= np.array((frame.applied_x, frame.applied_y))
        heights = prior_sensor[2] + oriented[:, 2]
        structural = oriented[(heights > 0.22) & (heights < 1.90)]
        epoch = _epoch_for_stamp(stamp, transition)
        baseline_result = baseline.process(
            structural, prior_sensor, epoch, stamp, final_sensor
        )
        if not reset_applied and stamp > transition[1]:
            reset.reset()
            reset_applied = True
        reset_result = reset.process(
            structural, prior_sensor, epoch, stamp, final_sensor
        )
        processed += 1
        if baseline_result["status"] == "accepted":
            logged = np.array((frame.applied_x, frame.applied_y))
            correction_errors.append(float(np.linalg.norm(
                baseline_result["correction"] - logged
            )))
        if stamp <= transition[1]:
            continue
        post_frame_count += 1
        if not baseline_result["accepted"]:
            continue
        post_accepted_count += 1
        aligned_truth = _aligned_truth_xy(truth, alignment)
        original_error, _without, benefit, _bx, _by = correction_benefit(
            (estimate.x, estimate.y), aligned_truth,
            (frame.applied_x, frame.applied_y),
        )
        label = "necessary" if benefit > 0.005 else (
            "harmful" if benefit < -0.005 else "neutral"
        )
        for origin, count in baseline_result["origins"].items():
            origin_totals[label][origin] += int(count)
        if reset_result["accepted"]:
            prior_base = np.array((
                estimate.x - frame.applied_x,
                estimate.y - frame.applied_y,
            ))
            reset_base = prior_base + reset_result["correction"]
            candidate_error = math.hypot(
                reset_base[0] - aligned_truth[0],
                reset_base[1] - aligned_truth[1],
            )
            reset_original_errors.append(original_error)
            reset_candidate_errors.append(candidate_error)
            reset_improvement.append(original_error - candidate_error)

    self_check = _error_metrics(correction_errors)
    reliable = self_check["rmse"] <= 0.02 if self_check["count"] else False

    def origin_summary(counts: dict[str, int]) -> dict:
        total = sum(counts.values())
        return {
            "match_count": total,
            "source_floor_fraction": (
                counts["source_floor"] / total if total else math.nan
            ),
            "transition_fraction": (
                counts["elevator_transition"] / total if total else math.nan
            ),
            "target_floor_fraction": (
                counts["target_floor"] / total if total else math.nan
            ),
        }

    return {
        "available": True,
        "transition_start": transition[0],
        "transition_end": transition[1],
        "processed_frame_count": processed,
        "post_transition_frame_count": post_frame_count,
        "post_transition_accepted_count": post_accepted_count,
        "self_check": self_check,
        "reliable": reliable,
        "match_origin": {
            label: origin_summary(counts)
            for label, counts in origin_totals.items()
        },
        "reset_ablation": {
            "original": _error_metrics(reset_original_errors),
            "reset_candidate": _error_metrics(reset_candidate_errors),
            "mean_error_improvement": (
                sum(reset_improvement) / len(reset_improvement)
                if reset_improvement else math.nan
            ),
            "improved_count": sum(value > 0.0 for value in reset_improvement),
            "worsened_count": sum(value < 0.0 for value in reset_improvement),
        },
        "interpretation_allowed": reliable,
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
    parser.add_argument("--sensor-windows-csv", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error(f"bag 不存在：{args.bag}")

    map_frames: list[MapFrame] = []
    planar_frames: list[PlanarFrame] = []
    motion_windows: list[MotionWindow] = []
    with rosbag.Bag(str(args.bag), "r") as bag:
        estimates = _read_poses(bag, args.odom_topic)
        pose3_samples = _read_pose3(bag, args.odom_topic)
        truths = _read_poses(bag, args.truth_topic)
        structures = _read_scan_structures(bag)
        rosout_entries: list[tuple[float, str]] = []
        for _topic, message, bag_stamp in bag.read_messages(topics=["/rosout"]):
            text = str(getattr(message, "msg", ""))
            rosout_entries.append((_message_stamp(message, bag_stamp), text))
            map_frame = parse_map_frame(text)
            if map_frame is not None:
                map_frames.append(map_frame)
            planar_frame = parse_planar_frame(text)
            if planar_frame is not None:
                planar_frames.append(planar_frame)
            motion_window = parse_motion_window(text)
            if motion_window is not None:
                motion_windows.append(motion_window)
        transition = transition_bounds_from_rosout(rosout_entries)
        map_alias = analyze_map_alias(
            bag,
            sorted(map_frames, key=lambda item: item.stamp),
            estimates,
            truths,
            pose3_samples,
            transition,
            args.maximum_sync_gap,
        )
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
        sorted(structures, key=lambda item: item.stamp),
    )
    sensor_windows = build_sensor_delta_windows(
        sorted(planar_frames, key=lambda item: item.stamp),
        sorted(map_frames, key=lambda item: item.stamp),
        estimates,
        truths,
        rosout_entries,
        sorted(motion_windows, key=lambda item: item.stamp),
        maximum_sync_gap=max(0.15, args.maximum_sync_gap),
    )
    result = {
        "bag": str(args.bag.resolve()),
        "map_frame_count": len(map_frames),
        "aligned_sample_count": len(samples),
        "alignment_ratio": len(samples) / len(map_frames),
        "motion_window_count": len(motion_windows),
        "aligned_motion_window_count": len(window_samples),
        "structure_frame_count": len(structures),
        "valid_structure_frame_count": sum(item.valid for item in structures),
        "motion_window_alignment_ratio": (
            len(window_samples) / len(motion_windows) if motion_windows else 0.0
        ),
        "axis_x": _feature_summary(samples, "x"),
        "axis_y": _feature_summary(samples, "y"),
        "motion_axis_x": _motion_summary(window_samples, "x"),
        "motion_axis_y": _motion_summary(window_samples, "y"),
        "local_motion": local_motion_summaries(window_samples),
        "sensor_delta_comparison": sensor_delta_summary(sensor_windows),
        "map_alias": map_alias,
        "benefit_spearman_with_error": spearman(
            [item.error_post for item in samples], [item.benefit for item in samples]
        ),
        "top_error_growth_windows": describe_error_growth_windows(
            estimates,
            truths,
            sensor_windows,
            rosout_entries,
            sorted(structures, key=lambda item: item.stamp),
        ),
    }
    if args.csv is not None:
        _write_csv(args.csv, samples, QualitySample)
    if args.windows_csv is not None:
        _write_csv(args.windows_csv, window_samples, MotionWindowSample)
    if args.sensor_windows_csv is not None:
        _write_csv(args.sensor_windows_csv, sensor_windows, SensorDeltaWindow)
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

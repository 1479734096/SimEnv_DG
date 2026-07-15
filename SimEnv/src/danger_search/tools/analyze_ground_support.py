#!/usr/bin/env python3
"""离线分析负障碍可观测性，并尽量复现生产 Mapper 快照语义。"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import re
import statistics
import sys
from typing import Callable, Iterable, Iterator, Sequence
import xml.etree.ElementTree as ElementTree

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

try:
    import rosbag
    import sensor_msgs.point_cloud2 as point_cloud2
except ImportError:  # pragma: no cover - 纯函数单元测试不需要 ROS
    rosbag = None
    point_cloud2 = None

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

from danger_search.mapper import (
    BLACK,
    FREE,
    OCC,
    UNKNOWN,
    VISITED,
    MultiFloorGridMap,
    _bresenham,
)


TICK_PERIOD = 0.1
FOOTPRINT_FRONT_REACH = 0.30
SUPPORT_LOOKAHEAD_SECONDS = 1.0
RAW_REQUIRED_TOPICS = {
    "/clock",
    "/cmd_vel",
    "/Odometry_gazebo",
    "/scan",
    "/livox/Pointcloud2",
    "/real_sense/depth/image_raw",
    "/real_sense/depth/camera_info",
    "/tf",
    "/tf_static",
    "/nav/map_debug",
    "/junior_ctrl/rl_status",
}
TOPIC_EXPECTED_HZ = {
    "/cmd_vel": 10.0,
    "/Odometry_gazebo": 100.0,
    "/scan": 10.0,
    "/livox/Pointcloud2": 10.0,
    "/real_sense/depth/image_raw": 10.0,
    "/nav/map_debug": 1.0,
}
MAP_VALUE_NAMES = {
    UNKNOWN: "UNKNOWN",
    FREE: "FREE",
    VISITED: "VISITED",
    BLACK: "BLACK",
    OCC: "OCC",
}


@dataclass(frozen=True)
class ReplayEvent:
    receive_time: float
    kind: str
    message: object


@dataclass(frozen=True)
class SnapshotDecision:
    tick_time: float
    odom: object
    cloud: object
    cloud_stamp: float | None
    floor: int
    action: str


@dataclass(frozen=True)
class MapComparison:
    exact: bool
    different_cells: int
    total_cells: int


@dataclass(frozen=True)
class SupportObservation:
    ever_observed: bool = True
    inside_fov: bool = True
    occluded: bool = False
    sufficient_points: bool = False
    support_point_count: int = 0
    lower_return_count: int = 0


@dataclass(frozen=True)
class SupportClassification:
    state: str
    reason: str


@dataclass(frozen=True)
class SceneCandidateConfirmation:
    minimum_frames: int | None
    scene_only: bool
    neighbor_thresholds_stable: bool
    independent_normal_scene_count: int
    production_ready: bool


@dataclass(frozen=True)
class TopicMetrics:
    topic: str
    message_count: int
    duration: float
    mean_hz: float | None
    median_hz: float | None
    max_gap: float | None
    gap_over_twice_period_count: int


@dataclass(frozen=True)
class TimelineValue:
    time: float
    value: object


@dataclass
class ReplaySummary:
    fidelity: str
    fidelity_reasons: list[str]
    tick_phase: float | None
    update_count: int
    skipped_ambiguous_count: int
    duplicate_cloud_count: int
    compared_map_count: int
    exact_map_count: int
    differing_cells: int
    cell_attribution_available: bool
    transition_counts: dict[str, int]
    cell_transitions: list[dict[str, object]]


def replay_snapshot_decisions(
    events: Sequence[ReplayEvent],
    *,
    tick_times: Sequence[float],
    floor_at: Callable[[float], int],
    ambiguous_at: Callable[[float], bool],
    cloud_stamp: Callable[[object], float | None],
) -> list[SnapshotDecision]:
    """按 Timer tick 获取 SensorBuffer 的最新 odom/cloud 快照。"""
    ordered = sorted(events, key=lambda item: item.receive_time)
    event_index = 0
    latest_odom = None
    latest_cloud = None
    last_consumed_stamp: float | None | object = _UNSET
    decisions: list[SnapshotDecision] = []
    for tick_time in tick_times:
        while event_index < len(ordered) and ordered[event_index].receive_time <= tick_time:
            event = ordered[event_index]
            if event.kind == "odom":
                latest_odom = event.message
            elif event.kind == "cloud":
                latest_cloud = event.message
            event_index += 1
        if latest_odom is None or latest_cloud is None:
            continue
        stamp = cloud_stamp(latest_cloud)
        if last_consumed_stamp is not _UNSET and stamp == last_consumed_stamp:
            continue
        last_consumed_stamp = stamp
        action = "skip_ambiguous" if ambiguous_at(tick_time) else "update"
        decisions.append(
            SnapshotDecision(
                tick_time=float(tick_time),
                odom=latest_odom,
                cloud=latest_cloud,
                cloud_stamp=stamp,
                floor=int(floor_at(tick_time)),
                action=action,
            )
        )
    return decisions


_UNSET = object()


def sample_cloud_points(
    points: Iterable[tuple[float, float, float]],
    *,
    stride: int,
    max_points: int,
) -> list[tuple[float, float, float]]:
    sampled: list[tuple[float, float, float]] = []
    step = max(1, int(stride))
    limit = max(0, int(max_points))
    if limit == 0:
        return sampled
    for index, point in enumerate(points):
        if index % step != 0:
            continue
        sampled.append((float(point[0]), float(point[1]), float(point[2])))
        if len(sampled) >= limit:
            break
    return sampled


def support_sweep_horizon(command_linear: float) -> float:
    """计算包含前足前伸的未来 1 秒足迹扫掠长度。"""
    return max(
        0.50,
        min(
            0.80,
            FOOTPRINT_FRONT_REACH
            + abs(float(command_linear)) * SUPPORT_LOOKAHEAD_SECONDS,
        ),
    )


def merge_replay_timeline(
    decisions: Sequence[SnapshotDecision],
    map_snapshots: Sequence[tuple[float, object]],
) -> list[tuple[float, str, object]]:
    """合并重放事件；生产 tick 在点云更新前先发布地图。"""
    items = [
        (float(time_value), "map", message)
        for time_value, message in map_snapshots
    ]
    items.extend(
        (float(decision.tick_time), "update", decision)
        for decision in decisions
    )
    return sorted(items, key=lambda item: (item[0], 0 if item[1] == "map" else 1))


def compare_map_arrays(actual: np.ndarray, replayed: np.ndarray) -> MapComparison:
    actual_array = np.asarray(actual)
    replayed_array = np.asarray(replayed)
    if actual_array.shape != replayed_array.shape:
        total = max(actual_array.size, replayed_array.size)
        return MapComparison(False, total, total)
    different = int(np.count_nonzero(actual_array != replayed_array))
    return MapComparison(different == 0, different, int(actual_array.size))


def classify_support_observation(observation: SupportObservation) -> SupportClassification:
    if not observation.ever_observed:
        return SupportClassification("UNKNOWN_SUPPORT", "never_observed")
    if not observation.inside_fov:
        return SupportClassification("UNKNOWN_SUPPORT", "outside_fov")
    if observation.occluded:
        return SupportClassification("UNKNOWN_SUPPORT", "occluded")
    if not observation.sufficient_points:
        return SupportClassification("UNKNOWN_SUPPORT", "insufficient_points")
    if observation.support_point_count > 0:
        return SupportClassification("SUPPORTED", "support_returns")
    if observation.lower_return_count > 0:
        return SupportClassification("NO_SUPPORT_EVIDENCE", "confirmed_lower_returns")
    return SupportClassification("UNKNOWN_SUPPORT", "visible_without_positive_drop_evidence")


def classify_support_cell(
    *,
    support_hits: int,
    lower_crossing_hits: int,
    inside_fov: bool,
    occluded: bool,
    minimum_hits: int,
) -> SupportClassification:
    """保守分类单个地面单元；无正回波永远不等于缺失地面。"""
    required = max(1, int(minimum_hits))
    if int(support_hits) >= required:
        return SupportClassification("SUPPORTED", "support_returns")
    if not inside_fov:
        return SupportClassification("UNKNOWN_SUPPORT", "outside_fov")
    if occluded:
        return SupportClassification("UNKNOWN_SUPPORT", "occluded")
    if int(lower_crossing_hits) >= required:
        return SupportClassification("NO_SUPPORT_EVIDENCE", "confirmed_lower_returns")
    return SupportClassification("UNKNOWN_SUPPORT", "insufficient_points")


def project_lower_returns_to_floor(
    *,
    sensor_origin: np.ndarray,
    points: np.ndarray,
    floor_z: float,
    minimum_drop: float,
) -> np.ndarray:
    """把明确命中下层的射线投影到其穿过当前楼层平面的位置。"""
    origin = np.asarray(sensor_origin, dtype=float).reshape(3)
    cloud = np.asarray(points, dtype=float).reshape((-1, 3))
    if cloud.size == 0:
        return np.empty((0, 3), dtype=float)
    lower = cloud[cloud[:, 2] < float(floor_z) - max(0.0, float(minimum_drop))]
    if lower.size == 0:
        return np.empty((0, 3), dtype=float)
    denominator = lower[:, 2] - origin[2]
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = (float(floor_z) - origin[2]) / denominator
    valid = np.isfinite(scale) & (scale > 0.0) & (scale < 1.0)
    return origin + scale[valid, None] * (lower[valid] - origin)


def select_scene_candidate_confirmation(
    *,
    positive_runs: Sequence[int],
    false_positive_runs: Sequence[int],
    neighbor_thresholds_stable: bool,
    independent_normal_scene_count: int,
) -> SceneCandidateConfirmation:
    """选择本场景候选连续帧数，但绝不自动晋升为生产默认值。"""
    maximum_false_positive_run = max((int(value) for value in false_positive_runs), default=0)
    minimum_positive_run = min((int(value) for value in positive_runs), default=0)
    candidate = maximum_false_positive_run + 1
    if minimum_positive_run <= 0 or candidate > minimum_positive_run:
        candidate = None
    scene_only = independent_normal_scene_count < 1
    production_ready = bool(
        candidate is not None
        and neighbor_thresholds_stable
        and not scene_only
    )
    return SceneCandidateConfirmation(
        minimum_frames=candidate,
        scene_only=scene_only,
        neighbor_thresholds_stable=bool(neighbor_thresholds_stable),
        independent_normal_scene_count=int(independent_normal_scene_count),
        production_ready=production_ready,
    )


def topic_metrics(topic: str, stamps: Sequence[float]) -> TopicMetrics:
    ordered = sorted(float(value) for value in stamps)
    if len(ordered) < 2:
        return TopicMetrics(topic, len(ordered), 0.0, None, None, None, 0)
    gaps = [end - start for start, end in zip(ordered, ordered[1:]) if end > start]
    duration = ordered[-1] - ordered[0]
    mean_hz = (len(ordered) - 1) / duration if duration > 0.0 else None
    median_gap = statistics.median(gaps) if gaps else None
    median_hz = 1.0 / median_gap if median_gap and median_gap > 0.0 else None
    expected = TOPIC_EXPECTED_HZ.get(topic)
    gap_limit = 2.0 / expected if expected else math.inf
    return TopicMetrics(
        topic=topic,
        message_count=len(ordered),
        duration=float(duration),
        mean_hz=float(mean_hz) if mean_hz is not None else None,
        median_hz=float(median_hz) if median_hz is not None else None,
        max_gap=float(max(gaps)) if gaps else None,
        gap_over_twice_period_count=sum(gap > gap_limit for gap in gaps),
    )


def _distribution(values: Sequence[float]) -> dict[str, float | int | None]:
    finite = np.asarray([float(value) for value in values if math.isfinite(float(value))])
    if finite.size == 0:
        return {"count": 0, "minimum": None, "p10": None, "median": None, "p90": None, "maximum": None}
    return {
        "count": int(finite.size),
        "minimum": float(np.min(finite)),
        "p10": float(np.percentile(finite, 10)),
        "median": float(np.median(finite)),
        "p90": float(np.percentile(finite, 90)),
        "maximum": float(np.max(finite)),
    }


def _performance_summary(index: dict[str, object]) -> dict[str, object]:
    sensor_summary: dict[str, object] = {}
    for name, samples in index.get("sensor_performance", {}).items():
        sensor_summary[name] = {
            "sim_update_rate": _distribution([item["sim_update_rate"] for item in samples]),
            "real_update_rate": _distribution([item["real_update_rate"] for item in samples]),
            "fps": _distribution([item["fps"] for item in samples]),
        }
    return {
        "real_time_factor": _distribution(index.get("real_time_factors", [])),
        "sensors": sensor_summary,
    }


def _capture_load_assessment(
    index: dict[str, object],
    *,
    reference_index: dict[str, object] | None,
) -> dict[str, object]:
    topic_stamps = index["topic_stamps"]
    required_rates = {
        topic: topic_metrics(topic, topic_stamps.get(topic, []))
        for topic in TOPIC_EXPECTED_HZ
    }
    rate_failures = []
    for topic, metrics in required_rates.items():
        expected = TOPIC_EXPECTED_HZ[topic]
        if metrics.mean_hz is None or metrics.mean_hz < expected * 0.85:
            rate_failures.append(
                f"{topic} 平均 {metrics.mean_hz!r}Hz，低于期望 {expected:.1f}Hz 的 85%"
            )
        allowed_large_gaps = max(2, int(metrics.message_count * 0.02))
        if metrics.gap_over_twice_period_count > allowed_large_gaps:
            rate_failures.append(
                f"{topic} 超过双周期的间隙 {metrics.gap_over_twice_period_count} 次"
            )

    current_rtf = _distribution(index.get("real_time_factors", []))
    reference_rtf = (
        _distribution(reference_index.get("real_time_factors", []))
        if reference_index is not None
        else None
    )
    current_median = current_rtf["median"]
    reference_median = reference_rtf["median"] if reference_rtf else None
    rtf_ratio = (
        float(current_median) / float(reference_median)
        if current_median is not None and reference_median not in (None, 0.0)
        else None
    )
    rtf_acceptable = rtf_ratio is not None and rtf_ratio >= 0.85
    return {
        "rate_failures": rate_failures,
        "current_real_time_factor": current_rtf,
        "reference_real_time_factor": reference_rtf,
        "median_rtf_ratio_to_reference": rtf_ratio,
        "rtf_acceptance_ratio": 0.85,
        "control_and_sensor_rate_acceptable": not rate_failures,
        "rtf_acceptable": rtf_acceptable,
        "diagnostic_load_acceptable": bool(not rate_failures and rtf_acceptable),
        "assessment_note": (
            "RTF 需与同代码、同场景的无原始采集参考运行比较；缺少参考时不自动通过"
        ),
    }


def value_at(timeline: Sequence[TimelineValue], time_value: float, default):
    if not timeline:
        return default
    times = [item.time for item in timeline]
    index = bisect_right(times, float(time_value)) - 1
    return timeline[index].value if index >= 0 else default


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--reference-run", type=Path)
    parser.add_argument(
        "--evaluation-path-run",
        type=Path,
        help="仅离线叠加该运行在坠落前的生产路径，不作为候选检测输入",
    )
    parser.add_argument(
        "--independent-normal-run",
        action="append",
        default=[],
        type=Path,
        help="独立正常场景原始采集，可重复指定",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--skip-mapper-replay", action="store_true")
    return parser.parse_args()


def _message_stamp(message, fallback: float | None = None) -> float | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    to_sec = getattr(stamp, "to_sec", None)
    return float(to_sec()) if callable(to_sec) else fallback


def _read_effective_config(run_dir: Path) -> dict[str, object]:
    if yaml is None:
        raise RuntimeError("缺少 PyYAML，无法读取有效配置")
    for name in ("danger_search.yaml", "params.yaml"):
        path = run_dir / name
        if path.is_file():
            payload = yaml.safe_load(path.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                return payload
    raise FileNotFoundError(f"{run_dir} 缺少 danger_search.yaml/params.yaml")


def _find_bag(run_dir: Path) -> Path:
    candidates = [
        run_dir / "observation" / "ground_support_telemetry.bag",
        run_dir / "observation" / "telemetry.bag",
    ]
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(f"{run_dir} 缺少可分析 rosbag")


def _collect_bag_index(bag_path: Path) -> dict[str, object]:
    topic_stamps: dict[str, list[float]] = {}
    floors: list[TimelineValue] = []
    maps: list[tuple[float, object]] = []
    real_time_factors: list[float] = []
    sensor_performance: dict[str, list[dict[str, float]]] = {}
    first_status_time = None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, recorded_at in bag.read_messages():
            receive_time = float(recorded_at.to_sec())
            topic_stamps.setdefault(topic, []).append(receive_time)
            if topic == "/danger_search/status":
                try:
                    payload = json.loads(message.data)
                    floor = int(payload["floor"])
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    continue
                floors.append(TimelineValue(receive_time, floor))
                if first_status_time is None:
                    first_status_time = receive_time
            elif topic == "/nav/map_debug":
                maps.append((_message_stamp(message, receive_time) or receive_time, message))
            elif topic == "/gazebo/performance_metrics":
                try:
                    real_time_factors.append(float(message.real_time_factor))
                except (AttributeError, TypeError, ValueError):
                    pass
                for sensor in getattr(message, "sensors", []):
                    sensor_performance.setdefault(str(sensor.name), []).append(
                        {
                            "sim_update_rate": float(sensor.sim_update_rate),
                            "real_update_rate": float(sensor.real_update_rate),
                            "fps": float(sensor.fps),
                        }
                    )
    return {
        "topic_stamps": topic_stamps,
        "floors": floors,
        "maps": maps,
        "first_status_time": first_status_time,
        "real_time_factors": real_time_factors,
        "sensor_performance": sensor_performance,
    }


_ROS_LOG_PATTERN = re.compile(r"\[[^\]]+,\s*([0-9.]+)\]:\s*(\{.*\})$")
_RL_COMMAND_PATTERN = re.compile(
    r"cmd=\(([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\)"
)


def _parse_rl_status_command(value: str) -> tuple[float, float, float] | None:
    match = _RL_COMMAND_PATTERN.search(str(value))
    if match is None:
        return None
    return tuple(float(match.group(index)) for index in (1, 2, 3))


def _read_mission_events(run_dir: Path) -> list[tuple[float, dict[str, object]]]:
    path = run_dir / "logs" / "danger_search.log"
    if not path.is_file():
        return []
    events: list[tuple[float, dict[str, object]]] = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = _ROS_LOG_PATTERN.search(line.strip())
            if match is None:
                continue
            try:
                payload = json.loads(match.group(2))
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict):
                events.append((float(match.group(1)), payload))
    return events


def _ambiguity_intervals(events: Sequence[tuple[float, dict[str, object]]]) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    entered = None
    for sim_time, payload in events:
        event = payload.get("event")
        if event == "floor_transition_ambiguous_enter" and entered is None:
            entered = sim_time
        elif event == "floor_transition_ambiguous_exit" and entered is not None:
            intervals.append((entered, sim_time))
            entered = None
    if entered is not None:
        intervals.append((entered, math.inf))
    return intervals


def _in_intervals(intervals: Sequence[tuple[float, float]], time_value: float) -> bool:
    return any(start <= time_value <= end for start, end in intervals)


def _map_array(message) -> np.ndarray:
    return np.asarray(message.data, dtype=np.int16).reshape(
        (int(message.info.height), int(message.info.width))
    )


def _read_static_sensor_transform(
    bag_path: Path,
    child_frame: str,
) -> tuple[np.ndarray, np.ndarray] | None:
    with rosbag.Bag(str(bag_path)) as bag:
        for _topic, message, _recorded_at in bag.read_messages(topics=["/tf_static"]):
            for transform in message.transforms:
                if transform.child_frame_id.lstrip("/") != child_frame.lstrip("/"):
                    continue
                if transform.header.frame_id.lstrip("/") != "base":
                    continue
                translation = transform.transform.translation
                return (
                    np.asarray([translation.x, translation.y, translation.z], dtype=float),
                    _quaternion_rotation(transform.transform.rotation),
                )
    return None


def _extract_pre_fall_path(run_dir: Path, floor_height: float) -> dict[str, object] | None:
    """从历史运行提取坠落开始前仍在执行的生产路径，仅供离线叠加。"""
    bag_path = _find_bag(run_dir)
    current_floor = 0
    path_history: list[tuple[float, list[np.ndarray]]] = []
    fall_time = None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, recorded_at in bag.read_messages(
            topics=["/danger_search/status", "/danger_search/path_followed", "/Odometry_gazebo"]
        ):
            receive_time = float(recorded_at.to_sec())
            if topic == "/danger_search/status":
                try:
                    current_floor = int(json.loads(message.data)["floor"])
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    pass
            elif topic == "/danger_search/path_followed":
                points = [
                    np.asarray([pose.pose.position.x, pose.pose.position.y], dtype=float)
                    for pose in message.poses
                ]
                if len(points) >= 2:
                    path_history.append((receive_time, points))
            elif current_floor > 0:
                vertical_velocity = float(message.twist.twist.linear.z)
                z = float(message.pose.pose.position.z)
                if vertical_velocity < -0.3 or z < current_floor * floor_height + 0.20:
                    fall_time = receive_time
                    break
    if fall_time is None:
        return None
    eligible = [item for item in path_history if item[0] < fall_time]
    if not eligible:
        return None
    published_at, path = eligible[-1]
    return {
        "source_run": str(run_dir.resolve()),
        "fall_time": fall_time,
        "floor": current_floor,
        "path_published_at": published_at,
        "path": path,
    }


def _parse_offline_surface_boxes(world_path: Path) -> dict[str, list[dict[str, float | int | str]]]:
    """读取场景几何只作离线真值标注，结果绝不进入候选检测。"""
    result: dict[str, list[dict[str, float | int | str]]] = {"slabs": [], "stairs": []}
    if not world_path.is_file():
        return result
    root = ElementTree.parse(str(world_path)).getroot()
    for link in root.iter("link"):
        name = str(link.attrib.get("name", ""))
        slab_match = re.fullmatch(r"slab_floor_(\d+)_\d+", name)
        stair_match = re.fullmatch(r"stair_.*_floor_(\d+)_step_\d+", name)
        if slab_match is None and stair_match is None:
            continue
        pose_node = link.find("pose")
        collision = link.find("collision")
        size_node = collision.find("geometry/box/size") if collision is not None else None
        if pose_node is None or pose_node.text is None or size_node is None or size_node.text is None:
            continue
        pose_values = [float(value) for value in pose_node.text.split()]
        size_values = [float(value) for value in size_node.text.split()]
        if len(pose_values) < 3 or len(size_values) < 3:
            continue
        floor = int((slab_match or stair_match).group(1))
        item = {
            "name": name,
            "floor": floor,
            "x_min": pose_values[0] - size_values[0] * 0.5,
            "x_max": pose_values[0] + size_values[0] * 0.5,
            "y_min": pose_values[1] - size_values[1] * 0.5,
            "y_max": pose_values[1] + size_values[1] * 0.5,
            "top_z": pose_values[2] + size_values[2] * 0.5,
        }
        result["slabs" if slab_match is not None else "stairs"].append(item)
    return result


def _odom_xyz(message) -> tuple[float, float, float]:
    position = message.pose.pose.position
    return (float(position.x), float(position.y), float(position.z))


def _quaternion_rotation(quaternion) -> np.ndarray:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _rotation_rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2, 0]))))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return roll, pitch, yaw


def _message_pose(message) -> tuple[np.ndarray, np.ndarray, float]:
    pose = message.pose.pose
    position = np.asarray(
        [pose.position.x, pose.position.y, pose.position.z],
        dtype=float,
    )
    rotation = _quaternion_rotation(pose.orientation)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return position, rotation, yaw


def _cell_for_xy(
    x: float,
    y: float,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[int, int]:
    return (
        int(math.floor((float(x) - origin_x) / resolution)),
        int(math.floor((float(y) - origin_y) / resolution)),
    )


def _point_cell_counts(
    points_xy: np.ndarray,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> Counter:
    points = np.asarray(points_xy, dtype=float).reshape((-1, 2))
    if len(points) == 0:
        return Counter()
    cells = np.floor(
        (points - np.asarray([origin_x, origin_y], dtype=float)) / resolution
    ).astype(np.int64)
    unique, counts = np.unique(cells, axis=0, return_counts=True)
    return Counter({
        (int(cell[0]), int(cell[1])): int(count)
        for cell, count in zip(unique, counts)
    })


def _future_path_polyline(
    position_xy: np.ndarray,
    path: Sequence[np.ndarray],
    *,
    yaw: float,
    horizon_distance: float,
) -> list[np.ndarray]:
    position = np.asarray(position_xy, dtype=float).reshape(2)
    horizon = max(0.05, float(horizon_distance))
    if len(path) < 2:
        return [
            position,
            position + horizon * np.asarray([math.cos(yaw), math.sin(yaw)]),
        ]
    best = None
    for index, (start, end) in enumerate(zip(path, path[1:])):
        start_array = np.asarray(start, dtype=float)
        end_array = np.asarray(end, dtype=float)
        delta = end_array - start_array
        squared = float(delta @ delta)
        fraction = (
            max(0.0, min(1.0, float((position - start_array) @ delta / squared)))
            if squared > 1e-12
            else 0.0
        )
        projected = start_array + fraction * delta
        distance = float(np.linalg.norm(projected - position))
        if best is None or distance < best[0]:
            best = (distance, index, projected)
    assert best is not None
    _, index, projected = best
    result = [position]
    if np.linalg.norm(projected - position) > 0.02:
        result.append(projected)
    remaining = horizon
    current = projected
    while remaining > 1e-9 and index < len(path) - 1:
        endpoint = np.asarray(path[index + 1], dtype=float)
        distance = float(np.linalg.norm(endpoint - current))
        if distance >= remaining and distance > 1e-12:
            result.append(current + (endpoint - current) * (remaining / distance))
            remaining = 0.0
            break
        if distance > 1e-12:
            result.append(endpoint)
            remaining -= distance
        index += 1
        current = endpoint
    if len(result) == 1:
        result.append(
            position + horizon * np.asarray([math.cos(yaw), math.sin(yaw)])
        )
    return result


def _rasterize_sweep_cells(
    polyline: Sequence[np.ndarray],
    *,
    half_width: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    radius_cells = int(math.ceil(half_width / resolution)) + 1
    for start, end in zip(polyline, polyline[1:]):
        start_array = np.asarray(start, dtype=float)
        end_array = np.asarray(end, dtype=float)
        distance = float(np.linalg.norm(end_array - start_array))
        sample_count = max(2, int(math.ceil(distance / max(0.02, resolution * 0.4))) + 1)
        for fraction in np.linspace(0.0, 1.0, sample_count):
            point = start_array + fraction * (end_array - start_array)
            center = _cell_for_xy(
                point[0],
                point[1],
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            for gx in range(center[0] - radius_cells, center[0] + radius_cells + 1):
                for gy in range(center[1] - radius_cells, center[1] + radius_cells + 1):
                    cell_center = np.asarray(
                        [
                            origin_x + (gx + 0.5) * resolution,
                            origin_y + (gy + 0.5) * resolution,
                        ]
                    )
                    if np.linalg.norm(cell_center - point) <= half_width:
                        cells.add((gx, gy))
    return cells


def _cell_center(
    cell: tuple[int, int],
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    z: float,
) -> np.ndarray:
    return np.asarray(
        [
            origin_x + (cell[0] + 0.5) * resolution,
            origin_y + (cell[1] + 0.5) * resolution,
            z,
        ],
        dtype=float,
    )


def _inside_livox_filtered_fov(
    target: np.ndarray,
    *,
    sensor_origin: np.ndarray,
    sensor_rotation: np.ndarray,
) -> bool:
    sensor_vector = sensor_rotation.T @ (np.asarray(target, dtype=float) - sensor_origin)
    distance = float(np.linalg.norm(sensor_vector))
    if distance < 0.2:
        return False
    elevation = math.degrees(
        math.atan2(sensor_vector[2], math.hypot(sensor_vector[0], sensor_vector[1]))
    )
    # 与 multi_floor_gazeboSim.launch 传给 pointcloud2livox 的 -7~55 度一致。
    return -7.0 <= elevation <= 55.0


def _path_points(message) -> list[np.ndarray]:
    return [
        np.asarray([pose.pose.position.x, pose.pose.position.y], dtype=float)
        for pose in getattr(message, "poses", [])
    ]


def _distance_and_progress_on_path(
    position_xy: np.ndarray,
    path: Sequence[np.ndarray],
) -> tuple[float, float]:
    """返回到折线的最近距离与归一化进度。"""
    if len(path) < 2:
        return math.inf, 0.0
    position = np.asarray(position_xy, dtype=float).reshape(2)
    segment_lengths = [
        float(np.linalg.norm(np.asarray(end) - np.asarray(start)))
        for start, end in zip(path, path[1:])
    ]
    total = sum(segment_lengths)
    traversed = 0.0
    best = (math.inf, 0.0)
    for start, end, segment_length in zip(path, path[1:], segment_lengths):
        start_array = np.asarray(start, dtype=float)
        delta = np.asarray(end, dtype=float) - start_array
        fraction = (
            max(0.0, min(1.0, float((position - start_array) @ delta / (segment_length ** 2))))
            if segment_length > 1e-9
            else 0.0
        )
        projected = start_array + fraction * delta
        candidate = (
            float(np.linalg.norm(position - projected)),
            (traversed + fraction * segment_length) / total if total > 1e-9 else 0.0,
        )
        if candidate[0] < best[0]:
            best = candidate
        traversed += segment_length
    return best


def _cell_is_occluded(
    cell: tuple[int, int],
    *,
    sensor_origin: np.ndarray,
    obstacle_counts: Counter,
    resolution: float,
    origin_x: float,
    origin_y: float,
    minimum_hits: int,
) -> bool:
    sensor_cell = _cell_for_xy(
        sensor_origin[0],
        sensor_origin[1],
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
    )
    ray = _bresenham(sensor_cell[0], sensor_cell[1], cell[0], cell[1])
    return any(obstacle_counts[item] >= minimum_hits for item in ray[1:-1])


def _classify_sweep(
    cells: Iterable[tuple[int, int]],
    *,
    support_counts: Counter,
    lower_counts: Counter,
    obstacle_counts: Counter,
    sensor_origin: np.ndarray,
    sensor_rotation: np.ndarray,
    floor_z: float,
    ever_visible: set[tuple[int, int]],
    minimum_hits: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> dict[str, object]:
    states: Counter = Counter()
    reasons: Counter = Counter()
    no_support_distances: list[float] = []
    for cell in cells:
        center = _cell_center(
            cell,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            z=floor_z,
        )
        has_direct_return = bool(support_counts[cell] > 0 or lower_counts[cell] > 0)
        inside_fov = bool(has_direct_return or _inside_livox_filtered_fov(
            center,
            sensor_origin=sensor_origin,
            sensor_rotation=sensor_rotation,
        ))
        if inside_fov:
            ever_visible.add(cell)
        occluded = bool(
            inside_fov
            and not has_direct_return
            and _cell_is_occluded(
                cell,
                sensor_origin=sensor_origin,
                obstacle_counts=obstacle_counts,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
                minimum_hits=minimum_hits,
            )
        )
        if support_counts[cell] >= minimum_hits:
            classification = SupportClassification("SUPPORTED", "support_returns")
        elif cell not in ever_visible:
            classification = SupportClassification("UNKNOWN_SUPPORT", "never_observed")
        elif not inside_fov:
            classification = SupportClassification("UNKNOWN_SUPPORT", "outside_fov")
        elif occluded:
            classification = SupportClassification("UNKNOWN_SUPPORT", "occluded")
        elif lower_counts[cell] >= minimum_hits:
            classification = SupportClassification(
                "NO_SUPPORT_EVIDENCE",
                "confirmed_lower_returns",
            )
        else:
            classification = SupportClassification("UNKNOWN_SUPPORT", "insufficient_points")
        states[classification.state] += 1
        reasons[classification.reason] += 1
        if classification.state == "NO_SUPPORT_EVIDENCE":
            no_support_distances.append(float(np.linalg.norm(center[:2] - sensor_origin[:2])))
    return {
        "cell_count": int(sum(states.values())),
        "state_counts": dict(states),
        "reason_counts": dict(reasons),
        "no_support_cell_count": int(states["NO_SUPPORT_EVIDENCE"]),
        "nearest_no_support_distance": (
            min(no_support_distances) if no_support_distances else None
        ),
    }


def _consecutive_alert_runs(
    samples: Sequence[tuple[float, int]],
    *,
    minimum_cells: int = 3,
    maximum_gap: float = 0.16,
) -> list[dict[str, object]]:
    runs: list[list[tuple[float, int]]] = []
    active: list[tuple[float, int]] = []
    for time_value, count in samples:
        if count < minimum_cells:
            if active:
                runs.append(active)
                active = []
            continue
        if active and time_value - active[-1][0] > maximum_gap:
            runs.append(active)
            active = []
        active.append((time_value, count))
    if active:
        runs.append(active)
    return [
        {
            "start": run[0][0],
            "end": run[-1][0],
            "frame_count": len(run),
            "maximum_cell_count": max(value for _time, value in run),
            "second_frame_time": run[1][0] if len(run) >= 2 else None,
            "third_frame_time": run[2][0] if len(run) >= 3 else None,
        }
        for run in runs
    ]


def _segment_name(status: dict[str, object] | None) -> str:
    if not status:
        return "unknown"
    task = status.get("current_task")
    phase = status.get("task_phase")
    task_text = str(task or "")
    if task_text.startswith("TakeElevator"):
        if phase and "ENTER" in str(phase):
            return "elevator_threshold_enter"
        if phase and "EXIT" in str(phase):
            return "elevator_threshold_exit"
        return "elevator_cabin"
    if task_text.startswith("OpenDoorAndPass"):
        return "door_or_door_unknown"
    state = status.get("state")
    return "flat_exploration" if state in ("EXPLORE", "RETURN_HOME") else str(state or "unknown")


def _status_payload(message) -> dict[str, object] | None:
    try:
        payload = json.loads(message.data)
    except (AttributeError, TypeError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _analyze_livox_support(
    bag_path: Path,
    *,
    config: dict[str, object],
    ambiguity_intervals: Sequence[tuple[float, float]],
    evaluation_path: Sequence[np.ndarray] | None,
) -> dict[str, object]:
    """逐帧检查生产可取得点云中的地面支撑和明确下层回波。"""
    map_config = dict(config.get("map", {}))
    resolution = float(map_config.get("resolution", 0.1))
    origin_x = float(map_config.get("origin_x", -14.0))
    origin_y = float(map_config.get("origin_y", -6.0))
    floor_height = float(map_config.get("floor_height", 2.6))
    stride = int(map_config.get("cloud_stride", 3))
    max_points = int(map_config.get("cloud_max_points", 18000))
    static_transform = _read_static_sensor_transform(bag_path, "laser_livox")
    if static_transform is None:
        return {"available": False, "reason": "缺少 base->laser_livox 静态变换"}
    sensor_translation, sensor_mount_rotation = static_transform

    latest_odom = None
    latest_status = None
    latest_cmd = 0.0
    latest_path: list[np.ndarray] = []
    ever_visible_by_floor: dict[int, set[tuple[int, int]]] = defaultdict(set)
    accumulated_support: dict[int, Counter] = defaultdict(Counter)
    accumulated_lower: dict[int, Counter] = defaultdict(Counter)
    frames: list[dict[str, object]] = []
    timing_cloud_receive_minus_stamp: list[float] = []
    timing_odom_stamp_minus_cloud_stamp: list[float] = []
    threshold_samples: dict[str, list[tuple[float, int]]] = defaultdict(list)
    segment_alerts: dict[str, list[int]] = defaultdict(list)
    segment_raw_alerts: dict[str, list[int]] = defaultdict(list)
    skipped_ambiguous = 0

    topics = [
        "/Odometry_gazebo",
        "/danger_search/status",
        "/danger_search/path_followed",
        "/cmd_vel",
        "/livox/Pointcloud2",
    ]
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, recorded_at in bag.read_messages(topics=topics):
            receive_time = float(recorded_at.to_sec())
            if topic == "/Odometry_gazebo":
                latest_odom = message
                continue
            if topic == "/danger_search/status":
                payload = _status_payload(message)
                if payload is not None:
                    latest_status = payload
                continue
            if topic == "/danger_search/path_followed":
                latest_path = _path_points(message)
                continue
            if topic == "/cmd_vel":
                latest_cmd = float(message.linear.x)
                continue
            if latest_odom is None or latest_status is None:
                continue

            cloud_stamp = _message_stamp(message, receive_time) or receive_time
            odom_stamp = _message_stamp(latest_odom, receive_time) or receive_time
            timing_cloud_receive_minus_stamp.append(receive_time - cloud_stamp)
            timing_odom_stamp_minus_cloud_stamp.append(odom_stamp - cloud_stamp)
            floor = int(latest_status.get("floor", 0))
            if _in_intervals(ambiguity_intervals, receive_time):
                skipped_ambiguous += 1
                frames.append(
                    {
                        "time": receive_time,
                        "cloud_stamp": cloud_stamp,
                        "floor": floor,
                        "segment": _segment_name(latest_status),
                        "action": "skip_floor_transition_ambiguous",
                    }
                )
                continue

            raw_points = list(point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ))
            raw_array = np.asarray(raw_points, dtype=float).reshape((-1, 3))
            points = np.asarray(
                sample_cloud_points(raw_points, stride=stride, max_points=max_points),
                dtype=float,
            ).reshape((-1, 3))
            position, body_rotation, yaw = _message_pose(latest_odom)
            roll, pitch, _ = _rotation_rpy(body_rotation)
            sensor_origin = position + body_rotation @ sensor_translation
            sensor_rotation = body_rotation @ sensor_mount_rotation
            floor_z = floor * floor_height
            support_points = points[np.abs(points[:, 2] - floor_z) <= 0.08]
            lower_crossings = project_lower_returns_to_floor(
                sensor_origin=sensor_origin,
                points=points,
                floor_z=floor_z,
                minimum_drop=0.30,
            )
            obstacle_points = points[
                (points[:, 2] >= floor_z + 0.15)
                & (points[:, 2] <= floor_z + 1.40)
            ]
            support_counts = _point_cell_counts(
                support_points[:, :2],
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            lower_counts = _point_cell_counts(
                lower_crossings[:, :2],
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            obstacle_counts = _point_cell_counts(
                obstacle_points[:, :2],
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            raw_support_points = raw_array[np.abs(raw_array[:, 2] - floor_z) <= 0.08]
            raw_lower_crossings = project_lower_returns_to_floor(
                sensor_origin=sensor_origin,
                points=raw_array,
                floor_z=floor_z,
                minimum_drop=0.30,
            )
            raw_obstacle_points = raw_array[
                (raw_array[:, 2] >= floor_z + 0.15)
                & (raw_array[:, 2] <= floor_z + 1.40)
            ]
            raw_support_counts = _point_cell_counts(
                raw_support_points[:, :2],
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            raw_lower_counts = _point_cell_counts(
                raw_lower_crossings[:, :2],
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            raw_obstacle_counts = _point_cell_counts(
                raw_obstacle_points[:, :2],
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            accumulated_support[floor].update(support_counts)
            accumulated_lower[floor].update(lower_counts)

            # 扫掠长度包含机身中心到前足的近似前伸，再加未来 1 秒平移距离。
            horizon = support_sweep_horizon(latest_cmd)
            actual_polyline = _future_path_polyline(
                position[:2],
                latest_path,
                yaw=yaw,
                horizon_distance=horizon,
            )
            actual_cells = _rasterize_sweep_cells(
                actual_polyline,
                half_width=0.22,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            actual = _classify_sweep(
                actual_cells,
                support_counts=support_counts,
                lower_counts=lower_counts,
                obstacle_counts=obstacle_counts,
                sensor_origin=sensor_origin,
                sensor_rotation=sensor_rotation,
                floor_z=floor_z,
                ever_visible=ever_visible_by_floor[floor],
                minimum_hits=2,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            actual_raw = _classify_sweep(
                actual_cells,
                support_counts=raw_support_counts,
                lower_counts=raw_lower_counts,
                obstacle_counts=raw_obstacle_counts,
                sensor_origin=sensor_origin,
                sensor_rotation=sensor_rotation,
                floor_z=floor_z,
                ever_visible=ever_visible_by_floor[floor],
                minimum_hits=2,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            segment = _segment_name(latest_status)
            segment_alerts[segment].append(int(actual["no_support_cell_count"]))
            segment_raw_alerts[segment].append(int(actual_raw["no_support_cell_count"]))
            threshold_samples["actual_sampled_w0.22_h2"].append(
                (receive_time, int(actual["no_support_cell_count"]))
            )
            threshold_samples["actual_raw_w0.22_h2"].append(
                (receive_time, int(actual_raw["no_support_cell_count"]))
            )

            evaluation = None
            path_distance = math.inf
            path_progress = 0.0
            if evaluation_path:
                path_distance, path_progress = _distance_and_progress_on_path(
                    position[:2],
                    evaluation_path,
                )
            if evaluation_path and path_distance <= 0.70 and path_progress >= 0.70:
                evaluation_variants = {}
                for width in (0.18, 0.22, 0.28):
                    polyline = _future_path_polyline(
                        position[:2],
                        evaluation_path,
                        yaw=yaw,
                        horizon_distance=horizon,
                    )
                    cells = _rasterize_sweep_cells(
                        polyline,
                        half_width=width,
                        resolution=resolution,
                        origin_x=origin_x,
                        origin_y=origin_y,
                    )
                    for minimum_hits in (1, 2, 3):
                        key = f"w{width:.2f}_h{minimum_hits}"
                        sampled_classification = _classify_sweep(
                            cells,
                            support_counts=support_counts,
                            lower_counts=lower_counts,
                            obstacle_counts=obstacle_counts,
                            sensor_origin=sensor_origin,
                            sensor_rotation=sensor_rotation,
                            floor_z=floor_z,
                            ever_visible=ever_visible_by_floor[floor],
                            minimum_hits=minimum_hits,
                            resolution=resolution,
                            origin_x=origin_x,
                            origin_y=origin_y,
                        )
                        raw_classification = _classify_sweep(
                            cells,
                            support_counts=raw_support_counts,
                            lower_counts=raw_lower_counts,
                            obstacle_counts=raw_obstacle_counts,
                            sensor_origin=sensor_origin,
                            sensor_rotation=sensor_rotation,
                            floor_z=floor_z,
                            ever_visible=ever_visible_by_floor[floor],
                            minimum_hits=minimum_hits,
                            resolution=resolution,
                            origin_x=origin_x,
                            origin_y=origin_y,
                        )
                        evaluation_variants[f"sampled_{key}"] = sampled_classification
                        evaluation_variants[f"raw_{key}"] = raw_classification
                        threshold_samples[f"evaluation_sampled_{key}"].append(
                            (receive_time, int(sampled_classification["no_support_cell_count"]))
                        )
                        threshold_samples[f"evaluation_raw_{key}"].append(
                            (receive_time, int(raw_classification["no_support_cell_count"]))
                        )
                evaluation = {
                    "sampled": evaluation_variants["sampled_w0.22_h2"],
                    "raw": evaluation_variants["raw_w0.22_h2"],
                }

            frames.append(
                {
                    "time": receive_time,
                    "cloud_stamp": cloud_stamp,
                    "latest_odom_stamp": odom_stamp,
                    "floor": floor,
                    "segment": segment,
                    "action": "analyze",
                    "robot_xyz": position.tolist(),
                    "robot_linear_velocity_xyz": [
                        float(latest_odom.twist.twist.linear.x),
                        float(latest_odom.twist.twist.linear.y),
                        float(latest_odom.twist.twist.linear.z),
                    ],
                    "robot_angular_velocity_xyz": [
                        float(latest_odom.twist.twist.angular.x),
                        float(latest_odom.twist.twist.angular.y),
                        float(latest_odom.twist.twist.angular.z),
                    ],
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "cmd_linear": latest_cmd,
                    "horizon_distance": horizon,
                    "raw_point_count": len(raw_points),
                    "sampled_point_count": int(len(points)),
                    "support_point_count": int(len(support_points)),
                    "lower_return_count": int(np.count_nonzero(points[:, 2] < floor_z - 0.30)),
                    "lower_crossing_count": int(len(lower_crossings)),
                    "actual_sweep": actual,
                    "actual_raw_sweep": actual_raw,
                    "evaluation_path_distance": path_distance if math.isfinite(path_distance) else None,
                    "evaluation_path_progress": path_progress,
                    "evaluation_sweep": evaluation,
                }
            )

    alert_runs = {
        key: _consecutive_alert_runs(samples)
        for key, samples in threshold_samples.items()
    }
    return {
        "available": True,
        "frame_count": len(frames),
        "analyzed_frame_count": sum(item["action"] == "analyze" for item in frames),
        "skipped_floor_transition_ambiguous_count": skipped_ambiguous,
        "cloud_receive_minus_cloud_stamp_seconds": _distribution(
            timing_cloud_receive_minus_stamp
        ),
        "latest_odom_stamp_minus_cloud_stamp_seconds": _distribution(
            timing_odom_stamp_minus_cloud_stamp
        ),
        "segment_actual_alert_summary": {
            segment: {
                "frame_count": len(values),
                "sampled_alert_frame_count": sum(value >= 3 for value in values),
                "sampled_maximum_no_support_cell_count": max(values, default=0),
                "raw_alert_frame_count": sum(
                    value >= 3 for value in segment_raw_alerts.get(segment, [])
                ),
                "raw_maximum_no_support_cell_count": max(
                    segment_raw_alerts.get(segment, []),
                    default=0,
                ),
            }
            for segment, values in sorted(segment_alerts.items())
        },
        "alert_runs": alert_runs,
        "frames": frames,
        "accumulated_support_counts": accumulated_support,
        "accumulated_lower_counts": accumulated_lower,
    }


def _inside_box(
    x: float,
    y: float,
    box: dict[str, float | int | str],
) -> bool:
    return bool(
        float(box["x_min"]) <= x <= float(box["x_max"])
        and float(box["y_min"]) <= y <= float(box["y_max"])
    )


def _final_map_support_summary(
    *,
    index: dict[str, object],
    support_counts_by_floor: dict[int, Counter],
    lower_counts_by_floor: dict[int, Counter],
    evaluation_path_info: dict[str, object] | None,
    surfaces: dict[str, list[dict[str, float | int | str]]],
    config: dict[str, object],
) -> dict[str, object]:
    """比较最终占据状态与独立支撑证据；场景几何仅作离线标签。"""
    map_config = dict(config.get("map", {}))
    resolution = float(map_config.get("resolution", 0.1))
    origin_x = float(map_config.get("origin_x", -14.0))
    origin_y = float(map_config.get("origin_y", -6.0))
    floor_height = float(map_config.get("floor_height", 2.6))
    final_maps: dict[int, object] = {}
    for _time_value, message in index.get("maps", []):
        floor = max(0, int(round(float(message.info.origin.position.z) / floor_height)))
        final_maps[floor] = message

    floor_summaries = {}
    for floor, message in sorted(final_maps.items()):
        array = _map_array(message)
        support_counts = support_counts_by_floor.get(floor, Counter())
        lower_counts = lower_counts_by_floor.get(floor, Counter())
        free_cells = np.argwhere((array == FREE) | (array == VISITED))
        evidence = Counter()
        for gy, gx in free_cells:
            cell = (int(gx), int(gy))
            if support_counts[cell] >= 2:
                evidence["SUPPORTED"] += 1
            elif lower_counts[cell] >= 2:
                evidence["NO_SUPPORT_EVIDENCE"] += 1
            else:
                evidence["UNKNOWN_SUPPORT"] += 1
        floor_summaries[str(floor)] = {
            "traversable_cell_count": int(len(free_cells)),
            "support_evidence_counts": dict(evidence),
            "note": "FREE/VISITED 不是地面支撑证据；无正证据仍为 UNKNOWN_SUPPORT",
        }

    path_summary = None
    if evaluation_path_info is not None:
        floor = int(evaluation_path_info.get("floor", 0))
        message = final_maps.get(floor)
        path = evaluation_path_info.get("path") or []
        if message is not None and len(path) >= 2:
            # 只审计历史生产路径最后 30%，对应进入坠落区域前的实际规划段。
            total_lengths = [
                float(np.linalg.norm(np.asarray(end) - np.asarray(start)))
                for start, end in zip(path, path[1:])
            ]
            total = sum(total_lengths)
            accumulated = 0.0
            tail: list[np.ndarray] = []
            for index_value, segment_length in enumerate(total_lengths):
                next_accumulated = accumulated + segment_length
                if next_accumulated >= total * 0.70:
                    start = np.asarray(path[index_value], dtype=float)
                    end = np.asarray(path[index_value + 1], dtype=float)
                    if not tail and segment_length > 1e-9 and accumulated < total * 0.70:
                        fraction = (total * 0.70 - accumulated) / segment_length
                        tail.append(start + fraction * (end - start))
                    if not tail:
                        tail.append(start)
                    tail.append(end)
                accumulated = next_accumulated
            cells = _rasterize_sweep_cells(
                tail,
                half_width=0.22,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            array = _map_array(message)
            state_counts: Counter = Counter()
            support_evidence: Counter = Counter()
            offline_surface: Counter = Counter()
            floor_boxes = [
                item
                for group in ("slabs", "stairs")
                for item in surfaces.get(group, [])
                if int(item["floor"]) == floor
                and abs(float(item["top_z"]) - floor * floor_height) <= 0.35
            ]
            for gx, gy in cells:
                if not (0 <= gy < array.shape[0] and 0 <= gx < array.shape[1]):
                    continue
                state_counts[MAP_VALUE_NAMES.get(int(array[gy, gx]), str(int(array[gy, gx])))] += 1
                if support_counts_by_floor.get(floor, Counter())[(gx, gy)] >= 2:
                    support_evidence["SUPPORTED"] += 1
                elif lower_counts_by_floor.get(floor, Counter())[(gx, gy)] >= 2:
                    support_evidence["NO_SUPPORT_EVIDENCE"] += 1
                else:
                    support_evidence["UNKNOWN_SUPPORT"] += 1
                center = _cell_center(
                    (gx, gy),
                    resolution=resolution,
                    origin_x=origin_x,
                    origin_y=origin_y,
                    z=floor * floor_height,
                )
                has_surface = any(_inside_box(center[0], center[1], box) for box in floor_boxes)
                offline_surface["static_surface" if has_surface else "no_static_surface"] += 1
            path_summary = {
                "source_run": evaluation_path_info["source_run"],
                "floor": floor,
                "cell_count": int(sum(state_counts.values())),
                "map_state_counts": dict(state_counts),
                "support_evidence_counts": dict(support_evidence),
                "offline_world_surface_labels": dict(offline_surface),
                "offline_only": True,
                "runtime_dependency": False,
            }
    return {
        "floors": floor_summaries,
        "historical_path_tail": path_summary,
        "causality_note": (
            "支撑证据能区分部分无支撑 FREE，但不能在近似 Mapper 回放下归因某一栅格由哪条射线写入"
        ),
    }


def _analyze_depth_support(
    bag_path: Path,
    *,
    config: dict[str, object],
    ambiguity_intervals: Sequence[tuple[float, float]],
) -> dict[str, object]:
    """稀疏反投影深度图，确认其是否在近场提供明确下层回波。"""
    static_transform = _read_static_sensor_transform(bag_path, "real_sense")
    if static_transform is None:
        return {"available": False, "reason": "缺少 base->real_sense 静态变换"}
    camera_translation, camera_rotation = static_transform
    map_config = dict(config.get("map", {}))
    floor_height = float(map_config.get("floor_height", 2.6))
    latest_odom = None
    latest_status = None
    camera_info = None
    last_stamp = None
    frame_count = 0
    skipped = 0
    near_range_values: list[float] = []
    lower_return_counts: list[int] = []
    support_return_counts: list[int] = []
    max_range_fractions: list[float] = []
    topics = [
        "/Odometry_gazebo",
        "/danger_search/status",
        "/real_sense/depth/camera_info",
        "/real_sense/depth/image_raw",
    ]
    optical_to_link = np.asarray(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=float,
    )
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, recorded_at in bag.read_messages(topics=topics):
            receive_time = float(recorded_at.to_sec())
            if topic == "/Odometry_gazebo":
                latest_odom = message
                continue
            if topic == "/danger_search/status":
                payload = _status_payload(message)
                if payload is not None:
                    latest_status = payload
                continue
            if topic == "/real_sense/depth/camera_info":
                camera_info = message
                continue
            stamp = _message_stamp(message, receive_time)
            if stamp == last_stamp or latest_odom is None or latest_status is None or camera_info is None:
                continue
            last_stamp = stamp
            if _in_intervals(ambiguity_intervals, receive_time):
                skipped += 1
                continue
            if message.encoding != "32FC1":
                continue
            depth = np.frombuffer(message.data, dtype=np.float32).reshape(
                (int(message.height), int(message.width))
            )
            sampled = depth[::8, ::8]
            yy, xx = np.mgrid[0:message.height:8, 0:message.width:8]
            valid = np.isfinite(sampled) & (sampled >= 0.4) & (sampled <= 8.0)
            if not np.any(valid):
                continue
            z = sampled[valid].astype(float)
            u = xx[valid].astype(float)
            v = yy[valid].astype(float)
            fx, fy = float(camera_info.K[0]), float(camera_info.K[4])
            cx, cy = float(camera_info.K[2]), float(camera_info.K[5])
            optical_points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
            link_points = (optical_to_link @ optical_points.T).T
            position, body_rotation, _yaw = _message_pose(latest_odom)
            camera_origin = position + body_rotation @ camera_translation
            world_rotation = body_rotation @ camera_rotation
            world_points = (world_rotation @ link_points.T).T + camera_origin
            floor = int(latest_status.get("floor", 0))
            floor_z = floor * floor_height
            horizontal = np.linalg.norm(world_points[:, :2] - position[:2], axis=1)
            near_mask = horizontal <= 1.0
            near_range_values.extend(horizontal[near_mask].tolist())
            lower_return_counts.append(int(np.count_nonzero(
                near_mask & (world_points[:, 2] < floor_z - 0.30)
            )))
            support_return_counts.append(int(np.count_nonzero(
                near_mask & (np.abs(world_points[:, 2] - floor_z) <= 0.08)
            )))
            max_range_fractions.append(float(np.mean(sampled[valid] >= 7.9)))
            frame_count += 1
    return {
        "available": frame_count > 0,
        "frame_count": frame_count,
        "skipped_floor_transition_ambiguous_count": skipped,
        "near_field_horizontal_range": _distribution(near_range_values),
        "near_field_lower_return_count_per_frame": _distribution(lower_return_counts),
        "near_field_support_return_count_per_frame": _distribution(support_return_counts),
        "max_range_fraction_per_frame": _distribution(max_range_fractions),
        "interpretation": (
            "近场无深度点或最大量程只能表示 UNKNOWN_SUPPORT；仅明确更低回波可作正证据"
        ),
    }


def _measure_kill_only_stop(
    run_dir: Path,
    bag_path: Path,
    *,
    alert_time: float | None,
) -> dict[str, object] | None:
    marker_path = run_dir / "observation" / "ground_support_kill_only_guard.json"
    if not marker_path.is_file():
        return None
    marker = json.loads(marker_path.read_text(encoding="utf-8"))
    if not isinstance(marker.get("odom_xyz"), list) or len(marker["odom_xyz"]) < 2:
        return {
            "available": False,
            "guard_time": float(marker.get("sim_time", 0.0)),
            "guard_reason": marker.get("reason"),
            "reason": "非空间 kill-only 终止没有边界触发位姿，不能测量终止后位移",
        }
    guard_time = float(marker["sim_time"])
    guard_xy = np.asarray(marker["odom_xyz"][:2], dtype=float)
    alert_position = None
    final_position = None
    maximum_after_guard = 0.0
    speed_at_alert = None
    speed_at_guard = None
    odom_samples: list[tuple[float, np.ndarray, float]] = []
    zero_command_time = None
    rl_zero_receive_time = None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, recorded_at in bag.read_messages(
            topics=["/Odometry_gazebo", "/cmd_vel", "/junior_ctrl/rl_status"]
        ):
            time_value = float(recorded_at.to_sec())
            if topic == "/cmd_vel":
                if (
                    time_value >= guard_time
                    and zero_command_time is None
                    and math.hypot(float(message.linear.x), float(message.linear.y)) <= 0.01
                    and abs(float(message.angular.z)) <= 0.01
                ):
                    zero_command_time = time_value
                continue
            if topic == "/junior_ctrl/rl_status":
                values = _parse_rl_status_command(message.data)
                if values is not None and time_value >= guard_time and rl_zero_receive_time is None:
                    if math.hypot(values[0], values[1]) <= 0.01 and abs(values[2]) <= 0.01:
                        rl_zero_receive_time = time_value
                continue
            position = np.asarray(_odom_xyz(message)[:2], dtype=float)
            speed = math.hypot(
                float(message.twist.twist.linear.x),
                float(message.twist.twist.linear.y),
            )
            odom_samples.append((time_value, position.copy(), speed))
            if alert_time is not None and time_value >= alert_time and alert_position is None:
                alert_position = position.copy()
                speed_at_alert = speed
            if time_value >= guard_time:
                if speed_at_guard is None:
                    speed_at_guard = speed
                maximum_after_guard = max(maximum_after_guard, float(np.linalg.norm(position - guard_xy)))
                final_position = position.copy()
    after_guard = [sample for sample in odom_samples if sample[0] >= guard_time]
    peak_sample = max(after_guard[:50], key=lambda item: item[2], default=None)
    deceleration_start_time = None
    if peak_sample is not None:
        peak_index = after_guard.index(peak_sample)
        threshold = max(0.02, peak_sample[2] - 0.02)
        for index_value in range(peak_index + 1, len(after_guard)):
            time_value, _position, speed = after_guard[index_value]
            window = [
                item[2]
                for item in after_guard[index_value:]
                if item[0] <= time_value + 0.03
            ]
            if window and max(window) <= threshold:
                deceleration_start_time = time_value
                break
    fully_stopped_time = None
    fully_stopped_position = None
    for index_value, (time_value, position, speed) in enumerate(after_guard):
        window = [
            item
            for item in after_guard[index_value:]
            if item[0] <= time_value + 0.10
        ]
        if (
            window
            and window[-1][0] - time_value >= 0.095
            and max(item[2] for item in window) <= 0.02
        ):
            fully_stopped_time = time_value
            fully_stopped_position = position
            break
    fully_stopped_observed = fully_stopped_time is not None
    final_speed = after_guard[-1][2] if after_guard else None
    return {
        "available": True,
        "guard_time": guard_time,
        "speed_at_guard": speed_at_guard,
        "marker_speed_at_guard": marker.get("odom_linear_speed"),
        "zero_command_time": zero_command_time,
        "zero_command_delay": (
            zero_command_time - guard_time if zero_command_time is not None else None
        ),
        "rl_zero_receive_time": rl_zero_receive_time,
        "rl_zero_receive_delay": (
            rl_zero_receive_time - guard_time if rl_zero_receive_time is not None else None
        ),
        "deceleration_start_time": deceleration_start_time,
        "deceleration_start_delay": (
            deceleration_start_time - guard_time
            if deceleration_start_time is not None
            else None
        ),
        "fully_stopped_observed": fully_stopped_observed,
        "fully_stopped_time": fully_stopped_time,
        "fully_stopped_delay": (
            fully_stopped_time - guard_time if fully_stopped_time is not None else None
        ),
        "full_stop_net_displacement": (
            float(np.linalg.norm(fully_stopped_position - guard_xy))
            if fully_stopped_position is not None
            else None
        ),
        "maximum_displacement_after_kill_signal": maximum_after_guard,
        "observation_end_time": after_guard[-1][0] if after_guard else None,
        "observation_end_speed": final_speed,
        "distance_is_lower_bound_only": not fully_stopped_observed,
        "alert_time": alert_time,
        "speed_at_alert": speed_at_alert,
        "net_displacement_alert_to_final": (
            float(np.linalg.norm(final_position - alert_position))
            if final_position is not None and alert_position is not None
            else None
        ),
        "measurement_scope": (
            "独立诊断侧 SIGINT runner 后的实测位移；不是生产控制器承诺的制动距离"
        ),
    }


def _measure_natural_stops(
    bag_path: Path,
    *,
    minimum_actual_speed: float = 0.25,
) -> dict[str, object]:
    """提取正常任务中 RL 收到零命令后仍完整可观测的停车事件。"""
    odom_samples: list[tuple[float, np.ndarray, float]] = []
    rl_changes: list[tuple[float, tuple[float, float, float]]] = []
    command_zero_times: list[float] = []
    previous_rl = None
    previous_cmd_linear_zero = None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, recorded_at in bag.read_messages(
            topics=["/Odometry_gazebo", "/cmd_vel", "/junior_ctrl/rl_status"]
        ):
            time_value = float(recorded_at.to_sec())
            if topic == "/Odometry_gazebo":
                position = np.asarray(_odom_xyz(message)[:2], dtype=float)
                speed = math.hypot(
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.linear.y),
                )
                odom_samples.append((time_value, position, speed))
            elif topic == "/cmd_vel":
                is_linear_zero = bool(
                    math.hypot(float(message.linear.x), float(message.linear.y)) <= 0.01
                )
                if is_linear_zero and previous_cmd_linear_zero is not True:
                    command_zero_times.append(time_value)
                previous_cmd_linear_zero = is_linear_zero
            else:
                command = _parse_rl_status_command(message.data)
                if command is None:
                    continue
                if previous_rl is None or any(
                    abs(command[index] - previous_rl[index]) > 1e-4
                    for index in range(3)
                ):
                    rl_changes.append((time_value, command))
                    previous_rl = command

    if not odom_samples:
        return {"available": False, "reason": "bag 中没有 odom"}
    odom_times = [item[0] for item in odom_samples]
    events = []
    for change_index, (zero_time, command) in enumerate(rl_changes):
        command_linear_is_zero = math.hypot(command[0], command[1]) <= 0.01
        if not command_linear_is_zero or change_index == 0:
            continue
        previous_linear_index = next(
            (
                index_value
                for index_value in range(change_index - 1, -1, -1)
                if math.hypot(
                    rl_changes[index_value][1][0],
                    rl_changes[index_value][1][1],
                )
                > 0.01
            ),
            None,
        )
        if previous_linear_index is None:
            continue
        previous_command = rl_changes[previous_linear_index][1]
        if math.hypot(previous_command[0], previous_command[1]) < 0.20:
            continue
        odom_index = max(0, bisect_right(odom_times, zero_time) - 1)
        start_time, start_position, start_speed = odom_samples[odom_index]
        if start_speed < minimum_actual_speed:
            continue
        next_linear_time = next(
            (
                time_value
                for time_value, later_command in rl_changes[change_index + 1 :]
                if math.hypot(later_command[0], later_command[1]) > 0.01
            ),
            math.inf,
        )
        if next_linear_time - zero_time < 0.25:
            continue

        stop_time = None
        stop_position = None
        stop_index = None
        search_end = min(next_linear_time, zero_time + 3.0)
        for candidate_index in range(odom_index, len(odom_samples)):
            time_value, position, speed = odom_samples[candidate_index]
            if time_value > search_end:
                break
            window = []
            for item in odom_samples[candidate_index:]:
                if item[0] > time_value + 0.10:
                    break
                window.append(item)
            if (
                window
                and window[-1][0] - time_value >= 0.095
                and max(item[2] for item in window) <= 0.02
            ):
                stop_time = time_value
                stop_position = position
                stop_index = candidate_index
                break
        if stop_index is None:
            continue
        travel = odom_samples[odom_index : stop_index + 1]
        cmd_zero_time = max(
            (time_value for time_value in command_zero_times if time_value <= zero_time),
            default=None,
        )
        events.append(
            {
                "rl_zero_time": zero_time,
                "cmd_zero_time": cmd_zero_time,
                "rl_receive_delay_from_cmd": (
                    zero_time - cmd_zero_time if cmd_zero_time is not None else None
                ),
                "start_actual_speed": start_speed,
                "start_xy": start_position.tolist(),
                "full_stop_time": stop_time,
                "full_stop_delay": stop_time - zero_time,
                "full_stop_xy": stop_position.tolist(),
                "full_stop_net_displacement": float(
                    np.linalg.norm(stop_position - start_position)
                ),
                "maximum_displacement_before_stop": max(
                    float(np.linalg.norm(position - start_position))
                    for _time, position, _speed in travel
                ),
                "minimum_linear_zero_hold_seconds": next_linear_time - zero_time,
            }
        )
    distances = [item["maximum_displacement_before_stop"] for item in events]
    return {
        "available": bool(events),
        "minimum_actual_speed": minimum_actual_speed,
        "event_count": len(events),
        "maximum_observed_stop_distance": max(distances, default=None),
        "stop_distance_distribution": _distribution(distances),
        "events": events,
        "scope": "仅使用正常运行中的 RL 零命令与 odom；场景类型需另作离线真值标注",
    }


class _TracingMapper(MultiFloorGridMap):
    """仅在离线进程跟踪生产私有射线方法，不改变其实现。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.frame_point_changes: set[tuple[int, int]] = set()
        self.frame_sector_changes: set[tuple[int, int]] = set()

    def begin_frame(self) -> None:
        self.frame_point_changes.clear()
        self.frame_sector_changes.clear()

    def _raycast_free(self, grid, start, end, *, stop_at_blocked=True):
        cells = _bresenham(start[0], start[1], end[0], end[1])[1:]
        unknown_before = {
            (gx, gy) for gx, gy in cells if int(grid.data[gy, gx]) == UNKNOWN
        }
        result = super()._raycast_free(grid, start, end, stop_at_blocked=stop_at_blocked)
        self.frame_point_changes.update(
            (gx, gy) for gx, gy in unknown_before if int(grid.data[gy, gx]) == FREE
        )
        return result

    def _clear_free_until_blocked(self, grid, start, end):
        cells = _bresenham(start[0], start[1], end[0], end[1])[1:]
        unknown_before = {
            (gx, gy) for gx, gy in cells if int(grid.data[gy, gx]) == UNKNOWN
        }
        result = super()._clear_free_until_blocked(grid, start, end)
        self.frame_sector_changes.update(
            (gx, gy) for gx, gy in unknown_before if int(grid.data[gy, gx]) == FREE
        )
        return result


def _tick_times(start: float, end: float, first_status_time: float | None) -> list[float]:
    if first_status_time is None:
        return []
    phase = math.fmod(first_status_time, TICK_PERIOD)
    first_index = math.ceil((start - phase) / TICK_PERIOD)
    last_index = math.floor((end - phase) / TICK_PERIOD)
    return [phase + index * TICK_PERIOD for index in range(first_index, last_index + 1)]


def replay_mapper(
    bag_path: Path,
    *,
    config: dict[str, object],
    index: dict[str, object],
    ambiguity_intervals: Sequence[tuple[float, float]],
) -> ReplaySummary:
    map_config = dict(config.get("map", {}))
    allowed_keys = {
        "resolution", "width", "length", "origin_x", "origin_y", "floor_height",
        "obstacle_min_z", "obstacle_max_z", "obstacle_inflation_radius",
        "raycast_max_range", "point_free_clear_max_range", "visited_radius",
        "free_clear_enabled", "free_clear_max_range", "free_clear_angle_step",
        "free_clear_obstacle_margin", "free_clear_min_points_per_bin",
    }
    mapper = _TracingMapper(**{key: value for key, value in map_config.items() if key in allowed_keys})
    stride = int(map_config.get("cloud_stride", 3))
    max_points = int(map_config.get("cloud_max_points", 18000))
    topic_stamps = index["topic_stamps"]
    all_times = sorted(
        topic_stamps.get("/Odometry_gazebo", []) + topic_stamps.get("/livox/Pointcloud2", [])
    )
    if not all_times:
        return ReplaySummary(
            "unavailable", ["缺少 odom 或 /livox/Pointcloud2"], None,
            0, 0, 0, 0, 0, 0, False, {}, [],
        )
    first_status_time = index["first_status_time"]
    tick_start = max(all_times[0], first_status_time) if first_status_time is not None else all_times[0]
    ticks = _tick_times(tick_start, all_times[-1], first_status_time)
    events: list[ReplayEvent] = []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, recorded_at in bag.read_messages(
            topics=["/Odometry_gazebo", "/livox/Pointcloud2"]
        ):
            events.append(
                ReplayEvent(
                    float(recorded_at.to_sec()),
                    "odom" if topic == "/Odometry_gazebo" else "cloud",
                    message,
                )
            )
    floors = index["floors"]
    decisions = replay_snapshot_decisions(
        events,
        tick_times=ticks,
        floor_at=lambda time_value: int(value_at(floors, time_value, 0)),
        ambiguous_at=lambda time_value: _in_intervals(ambiguity_intervals, time_value),
        cloud_stamp=lambda message: _message_stamp(message),
    )

    transition_counts = {"point_ray": 0, "sector_ray": 0, "visited": 0}
    cell_transitions: list[dict[str, object]] = []
    compared = exact = differing = 0
    for _event_time, event_kind, payload in merge_replay_timeline(decisions, index["maps"]):
        if event_kind == "map":
            message = payload
            floor_height = float(map_config.get("floor_height", 2.6))
            floor = max(0, int(round(float(message.info.origin.position.z) / floor_height)))
            comparison = compare_map_arrays(_map_array(message), mapper.floor(floor).data)
            compared += 1
            exact += int(comparison.exact)
            differing += comparison.different_cells
            continue

        decision = payload
        if decision.action != "update":
            continue
        points_iter = point_cloud2.read_points(
            decision.cloud,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        points = sample_cloud_points(points_iter, stride=stride, max_points=max_points)
        grid = mapper.floor(decision.floor)
        before = grid.data.copy()
        mapper.begin_frame()
        mapper.update_from_cloud(
            floor_index=decision.floor,
            robot_xyz=_odom_xyz(decision.odom),
            points=points,
        )
        point_changes = mapper.frame_point_changes
        sector_changes = mapper.frame_sector_changes
        visited_changes = np.argwhere((before != VISITED) & (grid.data == VISITED))
        transition_counts["point_ray"] += len(point_changes)
        transition_counts["sector_ray"] += len(sector_changes)
        transition_counts["visited"] += int(len(visited_changes))
        for source, cells in (("point_ray", point_changes), ("sector_ray", sector_changes)):
            for gx, gy in cells:
                wx, wy = grid.grid_to_world(gx, gy)
                cell_transitions.append(
                    {
                        "time": decision.tick_time,
                        "floor": decision.floor,
                        "source": source,
                        "gx": gx,
                        "gy": gy,
                        "x": wx,
                        "y": wy,
                    }
                )

    reasons: list[str] = []
    # rosbag 没有 MissionManager tick 事件；即使最终快照相同，也只能在逐快照全等时提高可信度。
    fidelity = "exact_snapshot_match" if compared > 0 and exact == compared else "approximate"
    if fidelity == "approximate":
        reasons.append("rosbag 未记录每次 MissionManager Timer tick，tick 相位由 status 推断")
        if compared:
            reasons.append(f"{compared - exact}/{compared} 个 map_debug 快照不一致")
        else:
            reasons.append("缺少可比较的 map_debug 快照")
        reasons.append("门洞清理、黑名单等非点云 Mapper 修改未纳入重放")
    if ambiguity_intervals:
        reasons.append("方案 G 区间来自结构化 enter/exit 日志")
    return ReplaySummary(
        fidelity=fidelity,
        fidelity_reasons=reasons,
        tick_phase=(math.fmod(index["first_status_time"], TICK_PERIOD) if index["first_status_time"] else None),
        update_count=sum(item.action == "update" for item in decisions),
        skipped_ambiguous_count=sum(item.action == "skip_ambiguous" for item in decisions),
        duplicate_cloud_count=max(0, len(ticks) - len(decisions)),
        compared_map_count=compared,
        exact_map_count=exact,
        differing_cells=differing,
        cell_attribution_available=fidelity == "exact_snapshot_match",
        transition_counts=transition_counts,
        cell_transitions=(cell_transitions if fidelity == "exact_snapshot_match" else []),
    )


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def analyze_run(
    run_dir: Path,
    *,
    reference_run: Path | None,
    evaluation_path_run: Path | None,
    independent_normal_runs: Sequence[Path],
    output_dir: Path,
    skip_mapper_replay: bool,
) -> dict[str, object]:
    if rosbag is None:
        raise RuntimeError("当前 Python 环境没有 rosbag")
    bag_path = _find_bag(run_dir)
    config = _read_effective_config(run_dir)
    index = _collect_bag_index(bag_path)
    topic_stamps = index["topic_stamps"]
    missing_topics = sorted(RAW_REQUIRED_TOPICS - set(topic_stamps))
    topic_report = {
        topic: asdict(topic_metrics(topic, stamps))
        for topic, stamps in sorted(topic_stamps.items())
        if topic in RAW_REQUIRED_TOPICS or topic in TOPIC_EXPECTED_HZ
    }
    mission_events = _read_mission_events(run_dir)
    intervals = _ambiguity_intervals(mission_events)
    floor_height = float(dict(config.get("map", {})).get("floor_height", 2.6))
    evaluation_path_info = (
        _extract_pre_fall_path(evaluation_path_run, floor_height)
        if evaluation_path_run is not None
        else None
    )
    evaluation_path = (
        evaluation_path_info.get("path")
        if evaluation_path_info is not None
        else None
    )
    support = _analyze_livox_support(
        bag_path,
        config=config,
        ambiguity_intervals=intervals,
        evaluation_path=evaluation_path,
    )
    frames = support.pop("frames", [])
    support_counts_by_floor = support.pop("accumulated_support_counts", {})
    lower_counts_by_floor = support.pop("accumulated_lower_counts", {})
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "ground_support_frames.jsonl").open("w", encoding="utf-8") as stream:
        for item in frames:
            stream.write(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")

    world_path = run_dir / "observation" / "offline_scene" / "competition_scene.world"
    surfaces = _parse_offline_surface_boxes(world_path)
    final_map_support = _final_map_support_summary(
        index=index,
        support_counts_by_floor=support_counts_by_floor,
        lower_counts_by_floor=lower_counts_by_floor,
        evaluation_path_info=evaluation_path_info,
        surfaces=surfaces,
        config=config,
    )
    depth_support = _analyze_depth_support(
        bag_path,
        config=config,
        ambiguity_intervals=intervals,
    )

    normal_contrasts = []
    for normal_run in independent_normal_runs:
        normal_bag = _find_bag(normal_run)
        normal_config = _read_effective_config(normal_run)
        normal_events = _read_mission_events(normal_run)
        normal_support = _analyze_livox_support(
            normal_bag,
            config=normal_config,
            ambiguity_intervals=_ambiguity_intervals(normal_events),
            evaluation_path=None,
        )
        normal_support.pop("frames", None)
        normal_support.pop("accumulated_support_counts", None)
        normal_support.pop("accumulated_lower_counts", None)
        normal_contrasts.append(
            {
                "run_dir": str(normal_run.resolve()),
                "scene_building_config_sha256": _sha256_file(
                    normal_run / "observation" / "offline_scene" / "building_config.json"
                ),
                "support": normal_support,
            }
        )

    evaluation_runs = support.get("alert_runs", {})
    neighbor_first_times = []
    neighbor_has_two_frames = True
    for width in (0.18, 0.22, 0.28):
        runs = evaluation_runs.get(f"evaluation_raw_w{width:.2f}_h2", [])
        eligible = [item for item in runs if int(item["frame_count"]) >= 2]
        if not eligible:
            neighbor_has_two_frames = False
            continue
        neighbor_first_times.append(float(eligible[0]["start"]))
    neighbor_stable = bool(
        neighbor_has_two_frames
        and len(neighbor_first_times) == 3
        and max(neighbor_first_times) - min(neighbor_first_times) <= 0.25
    )
    baseline_runs = evaluation_runs.get("evaluation_raw_w0.22_h2", [])
    persistent_baseline = [item for item in baseline_runs if int(item["frame_count"]) >= 2]
    earliest_alert_time = (
        float(persistent_baseline[0]["second_frame_time"])
        if persistent_baseline
        else None
    )
    stop_measurement = _measure_kill_only_stop(
        run_dir,
        bag_path,
        alert_time=earliest_alert_time,
    )
    natural_stop_measurements = _measure_natural_stops(bag_path)
    replay = None
    if not skip_mapper_replay and not missing_topics and point_cloud2 is not None:
        replay = replay_mapper(
            bag_path,
            config=config,
            index=index,
            ambiguity_intervals=intervals,
        )
        transitions_path = output_dir / "mapper_cell_transitions.jsonl"
        with transitions_path.open("w", encoding="utf-8") as stream:
            for item in replay.cell_transitions:
                stream.write(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")

    reference_metrics = None
    reference_index = None
    if reference_run is not None:
        reference_log = _read_mission_events(reference_run)
        reference_metrics = _runtime_performance(reference_log)
        try:
            reference_index = _collect_bag_index(_find_bag(reference_run))
        except FileNotFoundError:
            reference_index = None
    result = {
        "schema": "ground_support_offline_analysis_v2",
        "run_dir": str(run_dir.resolve()),
        "bag": str(bag_path.resolve()),
        "raw_evidence_complete": not missing_topics,
        "missing_topics": missing_topics,
        "topic_metrics": topic_report,
        "gazebo_performance": _performance_summary(index),
        "capture_load_assessment": _capture_load_assessment(
            index,
            reference_index=reference_index,
        ),
        "mission_runtime_performance": _runtime_performance(mission_events),
        "reference_runtime_performance": reference_metrics,
        "floor_transition_ambiguity_intervals": intervals,
        "sensor_support_analysis": support,
        "depth_support_analysis": depth_support,
        "final_map_support_comparison": final_map_support,
        "evaluation_path_overlay": (
            {
                "source_run": evaluation_path_info["source_run"],
                "fall_time": evaluation_path_info["fall_time"],
                "floor": evaluation_path_info["floor"],
                "path_published_at": evaluation_path_info["path_published_at"],
                "path": [point.tolist() for point in evaluation_path_info["path"]],
                "evidence_kind": "counterfactual_offline_overlay",
                "not_end_to_end_reproduction": True,
            }
            if evaluation_path_info is not None
            else None
        ),
        "independent_normal_contrasts": normal_contrasts,
        "candidate_threshold_assessment": {
            "evidence_stream": "raw_livox",
            "footprint_front_reach_m": FOOTPRINT_FRONT_REACH,
            "lookahead_seconds": SUPPORT_LOOKAHEAD_SECONDS,
            "minimum_cells_per_frame": 3,
            "candidate_consecutive_frames": 2 if earliest_alert_time is not None else None,
            "neighbor_widths_m": [0.18, 0.22, 0.28],
            "neighbor_thresholds_stable": neighbor_stable,
            "independent_normal_scene_count": len(normal_contrasts),
            "scene_only_candidate": True,
            "production_default_allowed": False,
            "reason": (
                "候选告警来自历史路径离线叠加，且没有合法楼梯实际通过片段；"
                "原始点云候选也不等价于当前 stride=3 的生产输入，不能由一个 seed 固化"
            ),
        },
        "kill_only_stop_measurement": stop_measurement,
        "natural_stop_measurements": natural_stop_measurements,
        "mapper_replay": (
            {key: value for key, value in asdict(replay).items() if key != "cell_transitions"}
            if replay is not None
            else None
        ),
        "implementation_admission": False,
        "implementation_admission_reason": (
            "原始证据缺失"
            if missing_topics
            else (
                "历史危险路径未在安全采集中被实际执行，且合法楼梯尚无通过对照；"
                "当前只证明传感器存在可用的明确下层回波候选信号"
            )
        ),
    }
    _write_json(output_dir / "ground_support_analysis.json", result)
    return result


def _runtime_performance(events: Sequence[tuple[float, dict[str, object]]]) -> dict[str, object]:
    samples: list[tuple[float, float, int | None]] = []
    for sim_time, payload in events:
        if payload.get("event") != "mission_runtime":
            continue
        controller = payload.get("controller")
        command_count = controller.get("command_count") if isinstance(controller, dict) else None
        elapsed = payload.get("elapsed", sim_time)
        try:
            samples.append((sim_time, float(elapsed), int(command_count) if command_count is not None else None))
        except (TypeError, ValueError):
            continue
    command_rates = []
    for start, end in zip(samples, samples[1:]):
        if start[2] is None or end[2] is None or end[1] <= start[1]:
            continue
        command_rates.append((end[2] - start[2]) / (end[1] - start[1]))
    return {
        "sample_count": len(samples),
        "median_command_rate_hz": statistics.median(command_rates) if command_rates else None,
        "minimum_command_rate_hz": min(command_rates) if command_rates else None,
    }


def _sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else run_dir / "observation" / "ground_support_analysis"
    )
    result = analyze_run(
        run_dir,
        reference_run=args.reference_run.resolve() if args.reference_run else None,
        evaluation_path_run=(
            args.evaluation_path_run.resolve() if args.evaluation_path_run else None
        ),
        independent_normal_runs=[item.resolve() for item in args.independent_normal_run],
        output_dir=output_dir,
        skip_mapper_replay=args.skip_mapper_replay,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

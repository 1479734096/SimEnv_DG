#!/usr/bin/env python3
"""从既有 bag 与 A1 模型离线估计负障碍诊断扫掠半宽。

足端真值和 URDF 只用于离线标定，不会进入 shadow 状态更新或参赛运行决策。
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from danger_search.support_shadow import SweepWidthComponents

try:
    import rosbag
except ImportError:  # pragma: no cover
    rosbag = None


FOOT_TOPICS = tuple(f"/ground_truth/{name}_foot" for name in ("FR", "FL", "RR", "RL"))


def _stamp(message, recorded_time) -> float:
    header_stamp = getattr(getattr(message, "header", None), "stamp", None)
    if header_stamp is not None and hasattr(header_stamp, "to_sec"):
        value = float(header_stamp.to_sec())
        if value > 0.0:
            return value
    return float(recorded_time.to_sec())


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _trunk_half_width(urdf: Path) -> float:
    root = ET.parse(urdf).getroot()
    for link in root.findall("link"):
        if link.get("name") != "trunk":
            continue
        box = link.find("collision/geometry/box")
        size = box.get("size") if box is not None else None
        if size:
            values = [float(value) for value in size.split()]
            if len(values) == 3:
                return values[1] / 2.0
    raise ValueError(f"{urdf} 中未找到 trunk box collision")


def _distance_and_heading(
    position: tuple[float, float],
    path: list[tuple[float, float]],
) -> tuple[float, float | None]:
    best_distance = math.inf
    best_heading = None
    x, y = position
    for start, end in zip(path, path[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            continue
        fraction = max(
            0.0,
            min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_squared),
        )
        projected = (start[0] + fraction * dx, start[1] + fraction * dy)
        distance = math.hypot(x - projected[0], y - projected[1])
        if distance < best_distance:
            best_distance = distance
            best_heading = math.atan2(dy, dx)
    return best_distance, best_heading


def estimate(
    *,
    bag_path: Path,
    config_path: Path,
    urdf_path: Path,
    safety_margin: float,
    neighbor_delta: float,
) -> dict[str, object]:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    resolution = float(config.get("map", {}).get("resolution", 0.1))
    max_linear = float(config.get("controller", {}).get("max_linear", 0.6))
    odometry = []
    commands = []
    paths = []
    foot_lateral = []
    foot_forward = []
    cloud_odom_deltas = []
    latest_odom_stamp = None

    topics = [
        "/Odometry_gazebo",
        "/cmd_vel",
        "/danger_search/path_followed",
        "/livox/Pointcloud2",
        *FOOT_TOPICS,
    ]
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, recorded_time in bag.read_messages(topics=topics):
            stamp = _stamp(message, recorded_time)
            if topic == "/Odometry_gazebo":
                position = message.pose.pose.position
                odometry.append(
                    (
                        stamp,
                        float(position.x),
                        float(position.y),
                        _yaw(message.pose.pose.orientation),
                    )
                )
                latest_odom_stamp = stamp
            elif topic == "/cmd_vel":
                commands.append(
                    (
                        stamp,
                        float(message.linear.x),
                        float(message.linear.y),
                        float(message.angular.z),
                    )
                )
            elif topic == "/danger_search/path_followed":
                paths.append(
                    (
                        stamp,
                        [
                            (float(item.pose.position.x), float(item.pose.position.y))
                            for item in message.poses
                        ],
                    )
                )
            elif topic in FOOT_TOPICS:
                # 赛方调试话题给出足端相对 base 的位置，仅用于离线包络标定。
                position = message.pose.pose.position
                foot_lateral.append(abs(float(position.y)))
                foot_forward.append(abs(float(position.x)))
            elif topic == "/livox/Pointcloud2" and latest_odom_stamp is not None:
                cloud_odom_deltas.append(abs(latest_odom_stamp - stamp))

    if not odometry or not commands or not paths or not foot_lateral:
        raise ValueError("bag 缺少 odom、cmd、path 或足端诊断数据")

    foot_lateral_p99 = float(np.percentile(foot_lateral, 99))
    foot_forward_p99 = float(np.percentile(foot_forward, 99))
    path_index = 0
    command_index = 0
    current_path: list[tuple[float, float]] = []
    current_command = (0.0, 0.0, 0.0)
    cross_track_errors = []
    heading_errors = []
    for stamp, x, y, robot_yaw in odometry[::5]:
        while path_index < len(paths) and paths[path_index][0] <= stamp:
            current_path = paths[path_index][1]
            path_index += 1
        while command_index < len(commands) and commands[command_index][0] <= stamp:
            current_command = commands[command_index][1:]
            command_index += 1
        if current_command[0] <= 0.1 or len(current_path) < 2:
            continue
        distance, path_heading = _distance_and_heading((x, y), current_path)
        if path_heading is None or distance >= 1.0:
            continue
        cross_track_errors.append(distance)
        heading_errors.append(abs(math.atan2(math.sin(robot_yaw - path_heading), math.cos(robot_yaw - path_heading))))
    if not cross_track_errors:
        raise ValueError("bag 中没有有效的行进中路径跟踪样本")

    lateral_tracking_p99 = float(np.percentile(cross_track_errors, 99))
    turning_expansions = [
        max(
            0.0,
            foot_forward_p99 * abs(math.sin(error))
            + foot_lateral_p99 * abs(math.cos(error))
            - foot_lateral_p99,
        )
        for error in heading_errors
    ]
    turning_sweep_p99 = float(np.percentile(turning_expansions, 99))
    cloud_odom_delay_max = max(cloud_odom_deltas) if cloud_odom_deltas else 0.0
    components = SweepWidthComponents(
        collision_half_width=_trunk_half_width(urdf_path),
        foot_lateral_extent=foot_lateral_p99,
        lateral_tracking_error=lateral_tracking_p99,
        turning_sweep=turning_sweep_p99,
        cloud_odom_registration_error=cloud_odom_delay_max * max_linear,
        grid_quantization_error=resolution / math.sqrt(2.0),
        safety_margin=float(safety_margin),
    )
    return {
        "schema": "ground_support_sweep_width_estimate_v1",
        "offline_calibration_only": True,
        "production_runtime_truth_input": False,
        "bag": str(bag_path),
        "config": str(config_path),
        "urdf": str(urdf_path),
        "sample_counts": {
            "foot": len(foot_lateral),
            "active_path_tracking": len(cross_track_errors),
            "cloud_odom_delta": len(cloud_odom_deltas),
        },
        "source_statistics": {
            "foot_lateral_p99": foot_lateral_p99,
            "foot_forward_p99": foot_forward_p99,
            "lateral_tracking_error_p99": lateral_tracking_p99,
            "heading_error_p99": float(np.percentile(heading_errors, 99)),
            "turning_sweep_p99": turning_sweep_p99,
            "cloud_odom_delay_max": cloud_odom_delay_max,
            "max_linear": max_linear,
            "resolution": resolution,
        },
        "components": {
            key: float(value)
            for key, value in components.__dict__.items()
        },
        "physical_half_width": components.physical_half_width(),
        "neighbor_widths": list(components.neighbor_widths(delta=neighbor_delta)),
        "neighbor_delta": float(neighbor_delta),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--safety-margin", type=float, default=0.02)
    parser.add_argument("--neighbor-delta", type=float, default=0.02)
    args = parser.parse_args()
    if rosbag is None:
        raise SystemExit("当前环境没有 rosbag Python 依赖")
    payload = estimate(
        bag_path=args.bag,
        config_path=args.config,
        urdf_path=args.urdf,
        safety_margin=args.safety_margin,
        neighbor_delta=args.neighbor_delta,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

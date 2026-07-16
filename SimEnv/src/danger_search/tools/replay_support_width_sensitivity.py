#!/usr/bin/env python3
"""用真实 bag 和 ShadowSupportGrid 复验物理半宽及网格原点偏移。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from danger_search.support_shadow import ShadowSupportConfig, ShadowSupportGrid

try:
    import rosbag
    import sensor_msgs.point_cloud2 as point_cloud2
except ImportError:  # pragma: no cover
    rosbag = point_cloud2 = None


def _stamp(message, recorded_time) -> float:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and hasattr(stamp, "to_sec"):
        value = float(stamp.to_sec())
        if value > 0.0:
            return value
    return float(recorded_time.to_sec())


def _rotation(quaternion) -> np.ndarray:
    x, y, z, w = (float(quaternion.x), float(quaternion.y), float(quaternion.z), float(quaternion.w))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _structured_log(message: str) -> dict[str, object] | None:
    start = str(message).find("{")
    if start < 0:
        return None
    try:
        payload = json.loads(str(message)[start:])
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def replay(args: argparse.Namespace) -> dict[str, object]:
    config_payload = yaml.safe_load(args.config.read_text(encoding="utf-8")) or {}
    map_config = config_payload.get("map", {})
    controller_config = config_payload.get("controller", {})
    support_config = config_payload.get("ground_support", {})
    resolution = float(map_config.get("resolution", 0.1))
    base_origin_x = float(map_config.get("origin_x", -14.0))
    base_origin_y = float(map_config.get("origin_y", -6.0))
    widths = tuple(float(value) for value in args.width)
    offsets = tuple(float(value) for value in args.offset)
    variant_count = len(widths) * len(offsets) * len(offsets)
    if variant_count > 9:
        raise ValueError(
            "单次回放最多 9 个网格组合，避免多个稀疏历史网格同时占用过多内存；"
            "请按扫掠宽度分次运行"
        )
    variants = {}
    for width in widths:
        for offset_x in offsets:
            for offset_y in offsets:
                key = f"w{width:.6f}_ox{offset_x:+.3f}_oy{offset_y:+.3f}"
                variants[key] = {
                    "width": width,
                    "offset_x": offset_x,
                    "offset_y": offset_y,
                    "grid": ShadowSupportGrid(
                        ShadowSupportConfig(
                            resolution=resolution,
                            origin_x=base_origin_x + offset_x,
                            origin_y=base_origin_y + offset_y,
                            floor_height=float(map_config.get("floor_height", 2.6)),
                            cloud_stride=int(map_config.get("cloud_stride", 3)),
                            cloud_max_points=int(map_config.get("cloud_max_points", 18000)),
                            sweep_half_width=width,
                            foot_band_center_offset=float(
                                support_config.get("foot_band_center_offset", 0.14)
                            ),
                            foot_band_half_width=float(
                                support_config.get("foot_band_half_width", 0.10)
                            ),
                            minimum_foot_band_coverage=float(
                                support_config.get("minimum_foot_band_coverage", 0.60)
                            ),
                            maximum_longitudinal_unknown_gap=float(
                                support_config.get(
                                    "maximum_longitudinal_unknown_gap",
                                    0.10,
                                )
                            ),
                            minimum_longitudinal_support_ratio=float(
                                support_config.get(
                                    "minimum_longitudinal_support_ratio",
                                    0.75,
                                )
                            ),
                            path_goal_tolerance=float(
                                controller_config.get("goal_tolerance", 0.35)
                            ),
                            threshold_widths=(width,),
                            snapshot_period=0.0,
                        )
                    ),
                    "evaluated": 0,
                    "active": 0,
                    "first_alert": None,
                    "longest_active_run": 0,
                    "current_active_run": 0,
                    "maximum_inactive_gap_frames": 0,
                    "current_inactive_run": 0,
                    "supported_distances": [],
                    "current_footprint_supported_frames": 0,
                    "left_band_coverages": [],
                    "right_band_coverages": [],
                    "corridor_samples": [],
                }

    odom = None
    floor = None
    path = []
    command_linear = 0.0
    ambiguous = False
    sensor_translation = None
    sensor_mount_rotation = None
    topics = [
        "/Odometry_gazebo",
        "/danger_search/status",
        "/danger_search/path_followed",
        "/cmd_vel",
        "/tf_static",
        "/rosout_agg",
        "/livox/Pointcloud2",
    ]
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, message, recorded_time in bag.read_messages(topics=topics):
            receive_time = float(recorded_time.to_sec())
            if topic == "/Odometry_gazebo":
                odom = message
            elif topic == "/danger_search/status":
                try:
                    floor = int(json.loads(message.data)["floor"])
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    pass
            elif topic == "/danger_search/path_followed":
                path = [
                    np.asarray([item.pose.position.x, item.pose.position.y], dtype=float)
                    for item in message.poses
                ]
            elif topic == "/cmd_vel":
                command_linear = float(message.linear.x)
            elif topic == "/tf_static":
                for transform in message.transforms:
                    if transform.header.frame_id.lstrip("/") == "base" and transform.child_frame_id.lstrip("/") == "laser_livox":
                        translation = transform.transform.translation
                        sensor_translation = np.asarray([translation.x, translation.y, translation.z], dtype=float)
                        sensor_mount_rotation = _rotation(transform.transform.rotation)
            elif topic == "/rosout_agg":
                payload = _structured_log(getattr(message, "msg", ""))
                if payload and payload.get("event") == "floor_transition_ambiguous_enter":
                    ambiguous = True
                elif payload and payload.get("event") == "floor_transition_ambiguous_exit":
                    ambiguous = False
            elif topic == "/livox/Pointcloud2":
                stamp = _stamp(message, recorded_time)
                if stamp < args.warmup_start or stamp > args.end:
                    continue
                if odom is None or floor is None or sensor_translation is None or sensor_mount_rotation is None:
                    continue
                points = np.asarray(
                    list(point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)),
                    dtype=float,
                ).reshape((-1, 3))
                position = odom.pose.pose.position
                body_rotation = _rotation(odom.pose.pose.orientation)
                robot_position = np.asarray([position.x, position.y, position.z], dtype=float)
                sensor_origin = robot_position + body_rotation @ sensor_translation
                sensor_rotation = body_rotation @ sensor_mount_rotation
                robot_yaw = math.atan2(float(body_rotation[1, 0]), float(body_rotation[0, 0]))
                for variant in variants.values():
                    result = variant["grid"].process_frame(
                        stamp=stamp,
                        floor=floor,
                        robot_xy=robot_position[:2],
                        robot_yaw=robot_yaw,
                        sensor_origin=sensor_origin,
                        sensor_rotation=sensor_rotation,
                        points=points,
                        path=path,
                        command_linear=command_linear,
                        ambiguous=ambiguous,
                    )
                    if stamp < args.start or result.get("action") != "update":
                        continue
                    active = bool(result["hypothetical_alert"]["active"])
                    assessment = result.get("local_corridor_assessment", {})
                    supported_distance = assessment.get("supported_distance")
                    if isinstance(supported_distance, (int, float)):
                        variant["supported_distances"].append(float(supported_distance))
                    if bool(assessment.get("current_footprint_supported", False)):
                        variant["current_footprint_supported_frames"] += 1
                    for key, output_key in (
                        ("minimum_left_foot_band_coverage", "left_band_coverages"),
                        ("minimum_right_foot_band_coverage", "right_band_coverages"),
                    ):
                        value = assessment.get(key)
                        if isinstance(value, (int, float)):
                            variant[output_key].append(float(value))
                    if args.include_corridor_samples:
                        variant["corridor_samples"].append(
                            {
                                "stamp": stamp,
                                "robot_xy": robot_position[:2].tolist(),
                                "command_linear": command_linear,
                                "path_points": len(path),
                                "supported_distance": supported_distance,
                                "support_boundary_detected": bool(
                                    assessment.get(
                                        "support_boundary_detected",
                                        False,
                                    )
                                ),
                                "positive_support_lookahead_distance": result.get(
                                    "positive_support_lookahead_distance"
                                ),
                                "current_footprint_supported": bool(
                                    assessment.get(
                                        "current_footprint_supported",
                                        False,
                                    )
                                ),
                                "left_band_coverage": assessment.get(
                                    "minimum_left_foot_band_coverage"
                                ),
                                "right_band_coverage": assessment.get(
                                    "minimum_right_foot_band_coverage"
                                ),
                                "maximum_bridged_unknown_gap": assessment.get(
                                    "maximum_bridged_unknown_gap"
                                ),
                            }
                        )
                    variant["evaluated"] += 1
                    if active:
                        variant["active"] += 1
                        variant["current_active_run"] += 1
                        variant["longest_active_run"] = max(
                            variant["longest_active_run"], variant["current_active_run"]
                        )
                        variant["current_inactive_run"] = 0
                        if variant["first_alert"] is None:
                            variant["first_alert"] = stamp
                    else:
                        variant["current_active_run"] = 0
                        variant["current_inactive_run"] += 1
                        variant["maximum_inactive_gap_frames"] = max(
                            variant["maximum_inactive_gap_frames"], variant["current_inactive_run"]
                        )

    results = []
    for key, variant in variants.items():
        evaluated = int(variant["evaluated"])
        supported_distances = variant["supported_distances"]
        left_band_coverages = variant["left_band_coverages"]
        right_band_coverages = variant["right_band_coverages"]
        results.append(
            {
                "key": key,
                "width": variant["width"],
                "origin_offset": [variant["offset_x"], variant["offset_y"]],
                "evaluated_frames": evaluated,
                "active_frames": int(variant["active"]),
                "active_ratio": float(variant["active"] / evaluated) if evaluated else 0.0,
                "first_alert_cloud_stamp": variant["first_alert"],
                "longest_active_run_frames": int(variant["longest_active_run"]),
                "maximum_inactive_gap_frames": int(variant["maximum_inactive_gap_frames"]),
                "supported_distance_min": (
                    min(supported_distances) if supported_distances else None
                ),
                "supported_distance_median": (
                    float(np.median(supported_distances))
                    if supported_distances
                    else None
                ),
                "supported_distance_max": (
                    max(supported_distances) if supported_distances else None
                ),
                "current_footprint_supported_ratio": (
                    float(variant["current_footprint_supported_frames"] / evaluated)
                    if evaluated
                    else 0.0
                ),
                "minimum_left_foot_band_coverage": (
                    min(left_band_coverages) if left_band_coverages else None
                ),
                "minimum_right_foot_band_coverage": (
                    min(right_band_coverages) if right_band_coverages else None
                ),
                **(
                    {"corridor_samples": variant["corridor_samples"]}
                    if args.include_corridor_samples
                    else {}
                ),
            }
        )
    return {
        "schema": "ground_support_width_grid_offset_replay_v1",
        "uses_shadow_support_grid": True,
        "bag": str(args.bag),
        "time_range": [args.start, args.end],
        "warmup_start": args.warmup_start,
        "widths": list(widths),
        "grid_origin_offsets": list(offsets),
        "variants": results,
        "all_variants_meet_95_percent": all(
            item["active_ratio"] >= 0.95 for item in results
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--width", action="append", required=True, type=float)
    parser.add_argument("--offset", action="append", required=True, type=float)
    parser.add_argument("--warmup-start", type=float, default=115.0)
    parser.add_argument("--start", type=float, default=120.165)
    parser.add_argument("--end", type=float, default=133.965)
    parser.add_argument(
        "--include-corridor-samples",
        action="store_true",
        help="输出时间窗内逐帧足迹带正支撑摘要，仅供短窗口诊断",
    )
    args = parser.parse_args()
    if rosbag is None or point_cloud2 is None:
        raise SystemExit("当前环境没有 ROS bag/PointCloud2 Python 依赖")
    payload = replay(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

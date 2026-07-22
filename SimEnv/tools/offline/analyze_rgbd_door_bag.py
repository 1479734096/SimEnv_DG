#!/usr/bin/env python3
"""离线统计 RGB-D 门板检测链的逐级通过情况。

输入只包含运行时已经允许的 RGB、深度、相机内参与内部定位录包；本工具不读取
场景布局或 Gazebo 真值。它用于区分“没有暗色候选”“候选缺少深度”和“几何
约束拒绝”，避免靠放宽正式阈值猜测问题。
"""
from __future__ import annotations

import argparse
from collections import Counter
import math
from pathlib import Path
import sys

import cv2
from cv_bridge import CvBridge
import numpy as np
import rosbag
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "src" / "danger_search"))

from danger_search.mission_manager import (  # noqa: E402
    _camera_to_world_matrix,
    _ros_link_from_optical_matrix,
)
from danger_search.pose_provider import PoseSample  # noqa: E402
from danger_search.rgbd_door_detector import RgbdDoorDetector  # noqa: E402


RGB_TOPIC = "/real_sense/rgb/image_raw/compressed"
DEPTH_TOPIC = "/real_sense/depth/image_raw"
CAMERA_INFO_TOPIC = "/real_sense/rgb/camera_info"
ODOM_TOPIC = "/danger_search/localization/odom"


def _stamp(message) -> float:
    return float(message.header.stamp.to_sec())


def _nearest(samples, stamp: float, maximum_delta: float):
    if not samples:
        return None
    candidate = min(samples, key=lambda item: abs(item[0] - stamp))
    return candidate[1] if abs(candidate[0] - stamp) <= maximum_delta else None


def _pose_sample(message) -> PoseSample:
    pose = message.pose.pose
    twist = message.twist.twist
    return PoseSample(
        stamp=_stamp(message),
        xyz=(pose.position.x, pose.position.y, pose.position.z),
        quaternion_xyzw=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        linear_xyz=(twist.linear.x, twist.linear.y, twist.linear.z),
        angular_xyz=(twist.angular.x, twist.angular.y, twist.angular.z),
    )


def _detector_from_config(config_path: Path) -> RgbdDoorDetector:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    semantic = config["local_semantic"]
    return RgbdDoorDetector(
        maximum_value=semantic["rgbd_dark_max_value"],
        maximum_saturation=semantic["rgbd_dark_max_saturation"],
        minimum_pixel_area=semantic["rgbd_minimum_pixel_area"],
        minimum_depth=semantic["rgbd_minimum_depth"],
        maximum_depth=semantic["rgbd_maximum_depth"],
        minimum_depth_pixels=semantic["rgbd_minimum_depth_pixels"],
        minimum_panel_width=semantic["rgbd_minimum_panel_width"],
        maximum_panel_width=semantic["rgbd_maximum_panel_width"],
        minimum_panel_height=semantic["rgbd_minimum_panel_height"],
        maximum_plane_rmse=semantic["rgbd_maximum_plane_rmse"],
        maximum_vertical_normal_z=semantic["rgbd_maximum_vertical_normal_z"],
        minimum_opening_depth_change=semantic[
            "rgbd_minimum_opening_depth_change"
        ],
        maximum_opening_dark_ratio=semantic[
            "rgbd_maximum_opening_dark_ratio"
        ],
        minimum_opening_deeper_fraction=semantic[
            "rgbd_minimum_opening_deeper_fraction"
        ],
        minimum_opening_valid_depth_fraction=semantic[
            "rgbd_minimum_opening_valid_depth_fraction"
        ],
    )


def analyze(args: argparse.Namespace) -> int:
    bridge = CvBridge()
    detector = _detector_from_config(args.config)
    if args.minimum_panel_width is not None:
        detector.minimum_panel_width = float(args.minimum_panel_width)
    if args.minimum_panel_height is not None:
        detector.minimum_panel_height = float(args.minimum_panel_height)
    topics = [RGB_TOPIC, DEPTH_TOPIC, CAMERA_INFO_TOPIC, ODOM_TOPIC]
    messages: dict[str, list[tuple[float, object]]] = {
        topic: [] for topic in topics
    }
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, _ in bag.read_messages(topics=topics):
            messages[topic].append((_stamp(message), message))

    camera_info = messages[CAMERA_INFO_TOPIC][-1][1]
    counters: Counter[str] = Counter()
    maxima = {
        "dark_area": 0.0,
        "valid_depth_pixels": 0,
        "estimated_width": 0.0,
        "estimated_height": 0.0,
    }
    detections = []
    frame_rows = []
    exported_frames = 0
    fx, fy = float(camera_info.K[0]), float(camera_info.K[4])
    for stamp, rgb_message in messages[RGB_TOPIC]:
        if stamp < args.start or stamp > args.end:
            continue
        counters["rgb_frames"] += 1
        depth_message = _nearest(messages[DEPTH_TOPIC], stamp, args.max_delta)
        odom_message = _nearest(messages[ODOM_TOPIC], stamp, args.max_delta)
        if depth_message is None:
            counters["depth_unsynchronised"] += 1
            continue
        if odom_message is None:
            counters["odom_unsynchronised"] += 1
            continue
        rgb = cv2.imdecode(
            np.frombuffer(rgb_message.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        depth = bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        if rgb is None or rgb.shape[:2] != depth.shape[:2]:
            counters["invalid_arrays"] += 1
            continue

        sample = _pose_sample(odom_message)
        if (
            args.export_dir is not None
            and counters["rgb_frames"] % args.export_stride == 0
        ):
            args.export_dir.mkdir(parents=True, exist_ok=True)
            output = args.export_dir / (
                f"frame_{stamp:.3f}_yaw_{sample.yaw:+.3f}.jpg"
            )
            cv2.imwrite(str(output), rgb)
            exported_frames += 1
        camera_to_world = _camera_to_world_matrix(
            sample,
            (0.28, 0.0, 0.043),
            (0.0, 0.0, 0.0),
        ) @ _ros_link_from_optical_matrix()
        observations = detector.detect_arrays(
            rgb,
            depth,
            camera_info.K,
            camera_to_world,
            stamp=stamp,
            diagnostics=counters,
        )
        detections.extend(observations)
        counters["final_observations"] += len(observations)

        mask = detector._dark_mask(rgb)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        qualifying = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            maxima["dark_area"] = max(maxima["dark_area"], area)
            if area < detector.minimum_pixel_area:
                continue
            x, y, width_px, height_px = cv2.boundingRect(contour)
            if width_px < 8 or height_px < 16:
                continue
            counters["large_dark_contours"] += 1
            contour_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
            ys, xs = np.where(contour_mask > 0)
            values = np.asarray(depth[ys, xs], dtype=float)
            if str(depth_message.encoding).startswith("16U"):
                values *= 0.001
            valid = (
                np.isfinite(values)
                & (values >= detector.minimum_depth)
                & (values <= detector.maximum_depth)
            )
            count = int(np.count_nonzero(valid))
            maxima["valid_depth_pixels"] = max(
                maxima["valid_depth_pixels"], count
            )
            if count < detector.minimum_depth_pixels:
                counters["contours_without_enough_depth"] += 1
                continue
            counters["contours_with_enough_depth"] += 1
            median_depth = float(np.median(values[valid]))
            estimated_width = width_px * median_depth / fx
            estimated_height = height_px * median_depth / fy
            maxima["estimated_width"] = max(
                maxima["estimated_width"], estimated_width
            )
            maxima["estimated_height"] = max(
                maxima["estimated_height"], estimated_height
            )
            qualifying.append(
                (area, count, estimated_width, estimated_height, median_depth)
            )
        if qualifying or observations:
            frame_rows.append((stamp, sample.yaw, qualifying, observations))

    print("RGB-D 门板离线诊断")
    print(f"  时间窗: {args.start:.3f} .. {args.end:.3f} s")
    for key in sorted(counters):
        print(f"  {key}: {counters[key]}")
    for key, value in maxima.items():
        print(f"  max_{key}: {value:.4f}")
    print("  代表帧（按最大估算门板面积排序，最多 12 帧）:")
    frame_rows.sort(
        key=lambda row: max(
            (item[2] * item[3] for item in row[2]),
            default=0.0,
        ),
        reverse=True,
    )
    for stamp, yaw, qualifying, observations in frame_rows[:12]:
        summary = [
            "area=%.0f depth=%d approx=%.2fx%.2f@%.2fm" % values
            for values in qualifying[:3]
        ]
        print(
            f"    t={stamp:.3f} yaw={yaw:.3f} contours={summary} "
            f"observations={len(observations)}"
        )
    if detections:
        widths = [item.width for item in detections]
        distances = [item.distance for item in detections]
        print(
            "  最终候选范围: width=%.3f..%.3f distance=%.3f..%.3f"
            % (min(widths), max(widths), min(distances), max(distances))
        )
        print("  最终候选样本（最多 10 个）:")
        for item in detections[:10]:
            print(
                "    t=%.3f xy=(%.3f, %.3f) yaw=%.3f size=%.3fx%.3f "
                "distance=%.3f rmse=%.4f"
                % (
                    item.stamp if item.stamp is not None else math.nan,
                    item.world_xy[0],
                    item.world_xy[1],
                    item.outward_yaw,
                    item.width,
                    item.height,
                    item.distance,
                    item.plane_rmse,
                )
            )
    if args.export_dir is not None:
        print(f"  已导出 RGB 帧: {exported_frames} -> {args.export_dir}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--max-delta", type=float, default=0.04)
    parser.add_argument(
        "--minimum-panel-width",
        type=float,
        help="仅用于离线对照的门板最小宽度，不修改正式配置",
    )
    parser.add_argument(
        "--minimum-panel-height",
        type=float,
        help="仅用于离线对照的门板最小高度，不修改正式配置",
    )
    parser.add_argument("--export-dir", type=Path)
    parser.add_argument("--export-stride", type=int, default=30)
    parser.add_argument(
        "--config",
        type=Path,
        default=WORKSPACE / "src" / "danger_search" / "config" / "danger_search.yaml",
    )
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(analyze(parse_args()))

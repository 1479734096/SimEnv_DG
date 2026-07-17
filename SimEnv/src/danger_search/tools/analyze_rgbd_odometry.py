#!/usr/bin/env python3
"""离线评估 RGB-D 视觉里程计；Gazebo 真值只用于算法输出后的误差标注。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag


def quaternion_rotation(quaternion) -> np.ndarray:
    x, y, z, w = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform(rotation: np.ndarray, translation) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation, dtype=np.float64)
    return result


def rpy_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def image_array(message) -> np.ndarray:
    if message.encoding == "rgb8":
        return np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3
        )
    if message.encoding == "32FC1":
        return np.frombuffer(message.data, dtype=np.float32).reshape(
            message.height, message.width
        )
    raise ValueError(f"不支持图像编码：{message.encoding}")


def estimate_relative_pose(
    previous_rgb: np.ndarray,
    previous_depth: np.ndarray,
    current_rgb: np.ndarray,
    camera_matrix: np.ndarray,
    orb,
    matcher,
) -> tuple[np.ndarray | None, int, int]:
    previous_gray = cv2.cvtColor(previous_rgb, cv2.COLOR_RGB2GRAY)
    current_gray = cv2.cvtColor(current_rgb, cv2.COLOR_RGB2GRAY)
    previous_keypoints, previous_descriptors = orb.detectAndCompute(previous_gray, None)
    current_keypoints, current_descriptors = orb.detectAndCompute(current_gray, None)
    if previous_descriptors is None or current_descriptors is None:
        return None, 0, 0

    candidate_matches = matcher.knnMatch(
        previous_descriptors, current_descriptors, k=2
    )
    matches = []
    for candidates in candidate_matches:
        if len(candidates) != 2:
            continue
        first, second = candidates
        if first.distance < 0.75 * second.distance:
            matches.append(first)
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    object_points = []
    image_points = []
    for match in matches:
        u, v = previous_keypoints[match.queryIdx].pt
        column = int(round(u))
        row = int(round(v))
        if not (0 <= row < previous_depth.shape[0] and 0 <= column < previous_depth.shape[1]):
            continue
        depth = float(previous_depth[row, column])
        if not math.isfinite(depth) or depth < 0.20 or depth > 8.0:
            continue
        object_points.append(
            ((u - cx) * depth / fx, (v - cy) * depth / fy, depth)
        )
        image_points.append(current_keypoints[match.trainIdx].pt)
    if len(object_points) < 25:
        return None, len(object_points), 0

    success, rotation_vector, translation, inliers = cv2.solvePnPRansac(
        np.asarray(object_points, dtype=np.float32),
        np.asarray(image_points, dtype=np.float32),
        camera_matrix,
        None,
        iterationsCount=150,
        reprojectionError=2.5,
        confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    inlier_count = 0 if inliers is None else int(len(inliers))
    if not success or inlier_count < 20:
        return None, len(object_points), inlier_count
    rotation, _ = cv2.Rodrigues(rotation_vector)
    relative = transform(rotation, translation.reshape(3))
    if np.linalg.norm(relative[:3, 3]) > 0.45:
        return None, len(object_points), inlier_count
    return relative, len(object_points), inlier_count


def analyze(path: Path, frame_stride: int) -> None:
    camera_matrix = None
    latest_truth = None
    pending_rgb = {}
    pending_depth = {}
    frame_index = 0
    previous = None
    world_camera = None
    base_camera = transform(
        rpy_rotation(-math.pi / 2.0, 0.0, -math.pi / 2.0),
        (0.28, 0.0, 0.043),
    )
    camera_base = np.linalg.inv(base_camera)
    orb = cv2.ORB_create(nfeatures=2500, fastThreshold=12)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    accepted = 0
    rejected = 0
    errors = []
    inlier_counts = []
    start_truth = None
    last_truth = None
    last_estimate = None

    topics = [
        "/real_sense/rgb/image_raw",
        "/real_sense/depth/image_raw",
        "/real_sense/rgb/camera_info",
        "/ground_truth/base_w",
    ]
    with rosbag.Bag(str(path)) as bag:
        for topic, message, _ in bag.read_messages(topics=topics):
            if topic == "/real_sense/rgb/camera_info":
                camera_matrix = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
                continue
            if topic == "/ground_truth/base_w":
                position = message.pose.pose.position
                latest_truth = (
                    message.header.stamp.to_sec(),
                    transform(
                        quaternion_rotation(message.pose.pose.orientation),
                        (position.x, position.y, position.z),
                    ),
                )
                continue
            stamp = message.header.stamp.to_nsec()
            if topic == "/real_sense/rgb/image_raw":
                pending_rgb[stamp] = message
            else:
                pending_depth[stamp] = message
            if stamp not in pending_rgb or stamp not in pending_depth:
                continue
            rgb_message = pending_rgb.pop(stamp)
            depth_message = pending_depth.pop(stamp)
            for old_stamp in [item for item in pending_rgb if item < stamp - 1_000_000_000]:
                pending_rgb.pop(old_stamp, None)
            for old_stamp in [item for item in pending_depth if item < stamp - 1_000_000_000]:
                pending_depth.pop(old_stamp, None)
            frame_index += 1
            if frame_index % frame_stride != 0 or camera_matrix is None or latest_truth is None:
                continue
            rgb = image_array(rgb_message)
            depth = image_array(depth_message)
            truth_transform = latest_truth[1]
            if previous is None:
                world_camera = truth_transform @ base_camera
                previous = (rgb.copy(), depth.copy())
                start_truth = truth_transform.copy()
                last_truth = truth_transform.copy()
                last_estimate = truth_transform.copy()
                continue
            relative, match_count, inlier_count = estimate_relative_pose(
                previous[0], previous[1], rgb, camera_matrix, orb, matcher
            )
            if relative is None:
                rejected += 1
                continue
            world_camera = world_camera @ np.linalg.inv(relative)
            estimated_base = world_camera @ camera_base
            error = float(
                np.linalg.norm(estimated_base[:3, 3] - truth_transform[:3, 3])
            )
            errors.append(error)
            inlier_counts.append(inlier_count)
            accepted += 1
            previous = (rgb.copy(), depth.copy())
            last_truth = truth_transform.copy()
            last_estimate = estimated_base.copy()

    if accepted == 0:
        raise RuntimeError("没有任何 RGB-D 相对位姿通过质量门槛")
    truth_displacement = float(
        np.linalg.norm(last_truth[:3, 3] - start_truth[:3, 3])
    )
    estimated_displacement = float(
        np.linalg.norm(last_estimate[:3, 3] - start_truth[:3, 3])
    )
    print(f"bag={path}")
    print(f"accepted={accepted} rejected={rejected} acceptance={accepted / (accepted + rejected):.3f}")
    print(f"inliers_median={float(np.median(inlier_counts)):.1f}")
    print(f"error_median={float(np.median(errors)):.3f} error_p95={float(np.quantile(errors, 0.95)):.3f} error_max={max(errors):.3f}")
    print(f"truth_displacement={truth_displacement:.3f} estimated_displacement={estimated_displacement:.3f}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frame-stride", type=int, default=2)
    args = parser.parse_args()
    if args.frame_stride <= 0:
        parser.error("--frame-stride 必须大于 0")
    analyze(args.bag, args.frame_stride)


if __name__ == "__main__":
    main()

"""基于合法 RGB-D 输入的动态门板候选与开门变化检测。

颜色只用于提出候选；最终候选必须同时满足深度平面、竖直方向和物理尺寸
约束。模块不读取场景文件，也不把外观候选直接解释为具体门 ID。

关闭状态检测先拟合深色竖直平面；打开状态既可以比较同视角基准门板，也可以
把运行时注册的门平面投影回当前图像，要求中央区域同时“变亮且深度变远”。
服务调用是否成功仍不足以清地图，必须由这些视觉变化或独立 Livox 开口确认。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - 仅在缺少 OpenCV 的离线环境触发
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - 仅在缺少 ROS 的离线环境触发
    CvBridge = None


@dataclass(frozen=True)
class DoorPanelObservation:
    """一帧 RGB-D 中通过深度几何验证的深色竖直门板。"""

    stamp: float | None
    distance: float
    world_xy: tuple[float, float]
    outward_yaw: float
    width: float
    height: float
    plane_rmse: float
    depth_inlier_count: int
    bbox: tuple[int, int, int, int]
    median_depth: float
    center_dark_fraction: float
    panel_state: str = "closed"


@dataclass(frozen=True)
class DoorOpeningChange:
    """同一视角下门板移开且后方深度增加的观测。"""

    confirmed: bool
    dark_fraction: float
    valid_depth_fraction: float
    deeper_fraction: float


class RgbdDoorDetector:
    """以深色外观提候选、以 RGB-D 几何拒绝阴影和小型黑色物体。

    所有公开方法在输入缺失、变换非法或 OpenCV 不可用时返回空/未确认结果，
    不抛出后再退回真值。这样感知失败只会让门保持关闭语义，不会放宽安全条件。
    """

    def __init__(
        self,
        *,
        maximum_value: int = 50,
        maximum_saturation: int = 85,
        minimum_pixel_area: int = 500,
        minimum_depth: float = 0.40,
        maximum_depth: float = 5.0,
        minimum_depth_pixels: int = 250,
        minimum_panel_width: float = 0.65,
        maximum_panel_width: float = 2.20,
        minimum_panel_height: float = 0.80,
        maximum_plane_rmse: float = 0.04,
        maximum_vertical_normal_z: float = 0.25,
        minimum_opening_depth_change: float = 0.30,
        maximum_opening_dark_ratio: float = 0.45,
        minimum_opening_deeper_fraction: float = 0.20,
        minimum_opening_valid_depth_fraction: float = 0.20,
    ):
        self.maximum_value = int(maximum_value)
        self.maximum_saturation = int(maximum_saturation)
        self.minimum_pixel_area = int(minimum_pixel_area)
        self.minimum_depth = float(minimum_depth)
        self.maximum_depth = float(maximum_depth)
        self.minimum_depth_pixels = int(minimum_depth_pixels)
        self.minimum_panel_width = float(minimum_panel_width)
        self.maximum_panel_width = float(maximum_panel_width)
        self.minimum_panel_height = float(minimum_panel_height)
        self.maximum_plane_rmse = float(maximum_plane_rmse)
        self.maximum_vertical_normal_z = float(maximum_vertical_normal_z)
        self.minimum_opening_depth_change = float(minimum_opening_depth_change)
        self.maximum_opening_dark_ratio = float(maximum_opening_dark_ratio)
        self.minimum_opening_deeper_fraction = float(minimum_opening_deeper_fraction)
        self.minimum_opening_valid_depth_fraction = float(
            minimum_opening_valid_depth_fraction
        )
        self._bridge = CvBridge() if CvBridge is not None else None

    def detect_messages(
        self,
        rgb_msg,
        depth_msg,
        camera_info_msg,
        camera_to_world: np.ndarray | None,
    ) -> list[DoorPanelObservation]:
        """转换 ROS 消息并调用数组检测核心；缺少 world 变换时拒绝检测。"""
        if self._bridge is None or camera_to_world is None:
            return []
        rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        return self.detect_arrays(
            rgb,
            depth,
            camera_info_msg.K,
            camera_to_world,
            stamp=_stamp_to_float(getattr(rgb_msg, "header", None)),
        )

    def detect_arrays(
        self,
        rgb_bgr: np.ndarray,
        depth_raw: np.ndarray,
        camera_k: Sequence[float],
        camera_to_world: np.ndarray | None,
        *,
        stamp: float | None = None,
    ) -> list[DoorPanelObservation]:
        """检测一帧深色竖直门板并返回几何验证后的候选。

        每个轮廓内部的有效深度像素先由相机内参反投影，随后拟合平面并检查法向、
        物理宽高和 RMSE，最后用 ``camera_to_world`` 得到门中心和朝外法向。
        """
        if cv2 is None or camera_to_world is None:
            return []
        if rgb_bgr is None or depth_raw is None or rgb_bgr.shape[:2] != depth_raw.shape[:2]:
            return []
        matrix = np.asarray(camera_to_world, dtype=float)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            return []
        try:
            fx, fy = float(camera_k[0]), float(camera_k[4])
            cx, cy = float(camera_k[2]), float(camera_k[5])
        except (IndexError, TypeError, ValueError):
            return []
        if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0.0 or fy <= 0.0:
            return []

        depth = _depth_to_meters(depth_raw)
        dark_mask = self._dark_mask(rgb_bgr)
        contours, _ = cv2.findContours(
            dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        observations: list[DoorPanelObservation] = []
        for contour in contours:
            if cv2.contourArea(contour) < self.minimum_pixel_area:
                continue
            x, y, width_px, height_px = cv2.boundingRect(contour)
            if width_px < 8 or height_px < 16:
                continue
            contour_mask = np.zeros(dark_mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
            ys, xs = np.where(contour_mask > 0)
            if len(xs) < self.minimum_depth_pixels:
                continue
            z = depth[ys, xs].astype(float)
            valid = (
                np.isfinite(z)
                & (z >= self.minimum_depth)
                & (z <= self.maximum_depth)
            )
            xs = xs[valid]
            ys = ys[valid]
            z = z[valid]
            if len(z) < self.minimum_depth_pixels:
                continue

            median_depth = float(np.median(z))
            plane_band = max(0.12, 0.08 * median_depth)
            near_plane = np.abs(z - median_depth) <= plane_band
            xs = xs[near_plane]
            ys = ys[near_plane]
            z = z[near_plane]
            if len(z) < self.minimum_depth_pixels:
                continue

            # 高分辨率门板像素无需全部进入 SVD；均匀抽样保持控制周期稳定。
            if len(z) > 5000:
                indices = np.linspace(0, len(z) - 1, 5000, dtype=int)
                xs, ys, z = xs[indices], ys[indices], z[indices]
            optical_points = np.column_stack(
                (
                    (xs.astype(float) - cx) * z / fx,
                    (ys.astype(float) - cy) * z / fy,
                    z,
                )
            )
            fit = _fit_plane(optical_points)
            if fit is None:
                continue
            normal_optical, center_optical, residuals = fit
            inliers = residuals <= max(0.02, 2.5 * self.maximum_plane_rmse)
            if int(np.count_nonzero(inliers)) < self.minimum_depth_pixels:
                continue
            refined = _fit_plane(optical_points[inliers])
            if refined is None:
                continue
            normal_optical, center_optical, residuals = refined
            plane_rmse = float(np.sqrt(np.mean(np.square(residuals))))
            if not math.isfinite(plane_rmse) or plane_rmse > self.maximum_plane_rmse:
                continue

            homogeneous = np.column_stack(
                (optical_points[inliers], np.ones(int(np.count_nonzero(inliers))))
            )
            world_points = (matrix @ homogeneous.T).T[:, :3]
            normal_world = matrix[:3, :3] @ normal_optical
            normal_norm = float(np.linalg.norm(normal_world))
            if normal_norm <= 1e-9:
                continue
            normal_world = normal_world / normal_norm
            if abs(float(normal_world[2])) > self.maximum_vertical_normal_z:
                continue

            horizontal_normal = np.asarray(
                [normal_world[0], normal_world[1], 0.0], dtype=float
            )
            horizontal_norm = float(np.linalg.norm(horizontal_normal))
            if horizontal_norm <= 1e-9:
                continue
            horizontal_normal /= horizontal_norm
            tangent = np.asarray(
                [-horizontal_normal[1], horizontal_normal[0], 0.0], dtype=float
            )
            tangent_projection = world_points @ tangent
            width = float(
                np.percentile(tangent_projection, 97.5)
                - np.percentile(tangent_projection, 2.5)
            )
            height = float(
                np.percentile(world_points[:, 2], 97.5)
                - np.percentile(world_points[:, 2], 2.5)
            )
            if not self.minimum_panel_width <= width <= self.maximum_panel_width:
                continue
            if height < self.minimum_panel_height:
                continue

            center_world = matrix @ np.asarray(
                [center_optical[0], center_optical[1], center_optical[2], 1.0],
                dtype=float,
            )
            camera_world = matrix[:3, 3]
            toward_camera = camera_world[:2] - center_world[:2]
            if float(np.dot(horizontal_normal[:2], toward_camera)) < 0.0:
                horizontal_normal *= -1.0
            outward_yaw = math.atan2(
                float(horizontal_normal[1]), float(horizontal_normal[0])
            )
            distance = float(np.linalg.norm(toward_camera))
            center_roi = _center_roi((x, y, width_px, height_px), dark_mask.shape)
            center_dark_fraction = _mask_fraction(dark_mask, center_roi)
            observations.append(
                DoorPanelObservation(
                    stamp=stamp,
                    distance=distance,
                    world_xy=(float(center_world[0]), float(center_world[1])),
                    outward_yaw=float(outward_yaw),
                    width=width,
                    height=height,
                    plane_rmse=plane_rmse,
                    depth_inlier_count=int(np.count_nonzero(inliers)),
                    bbox=(int(x), int(y), int(width_px), int(height_px)),
                    median_depth=median_depth,
                    center_dark_fraction=center_dark_fraction,
                )
            )
        observations.extend(
            self._pair_open_panels(observations, dark_mask, depth)
        )
        return sorted(observations, key=lambda item: item.distance)

    def _pair_open_panels(
        self,
        panels: list[DoorPanelObservation],
        dark_mask: np.ndarray,
        depth: np.ndarray,
    ) -> list[DoorPanelObservation]:
        """用左右深色滑门板和中间更深回波确认已打开的门洞。"""
        paired: list[DoorPanelObservation] = []
        for first_index, first in enumerate(panels):
            for second in panels[first_index + 1 :]:
                yaw_delta = abs(
                    math.atan2(
                        math.sin(first.outward_yaw - second.outward_yaw),
                        math.cos(first.outward_yaw - second.outward_yaw),
                    )
                )
                if yaw_delta > 0.25 or abs(first.median_depth - second.median_depth) > 0.35:
                    continue
                normal = np.asarray(
                    [math.cos(first.outward_yaw), math.sin(first.outward_yaw)],
                    dtype=float,
                )
                tangent = np.asarray([-normal[1], normal[0]], dtype=float)
                delta = np.asarray(second.world_xy) - np.asarray(first.world_xy)
                if abs(float(np.dot(delta, normal))) > 0.30:
                    continue
                center_separation = abs(float(np.dot(delta, tangent)))
                opening_width = center_separation - 0.5 * (
                    first.width + second.width
                )
                if not self.minimum_panel_width <= opening_width <= self.maximum_panel_width:
                    continue

                left, right = sorted((first, second), key=lambda item: item.bbox[0])
                left_end = left.bbox[0] + left.bbox[2]
                right_start = right.bbox[0]
                top = max(left.bbox[1], right.bbox[1])
                bottom = min(
                    left.bbox[1] + left.bbox[3],
                    right.bbox[1] + right.bbox[3],
                )
                gap_width = right_start - left_end
                gap_height = bottom - top
                if gap_width < 8 or gap_height < 16:
                    continue
                # 只取开口中央，避免滑到两侧的门板污染深度变化统计。
                inset_x = max(1, int(round(0.15 * gap_width)))
                inset_y = max(1, int(round(0.10 * gap_height)))
                gap_roi = (
                    left_end + inset_x,
                    top + inset_y,
                    gap_width - 2 * inset_x,
                    gap_height - 2 * inset_y,
                )
                x, y, width_px, height_px = gap_roi
                if width_px <= 0 or height_px <= 0:
                    continue
                region = depth[y : y + height_px, x : x + width_px]
                valid = (
                    np.isfinite(region)
                    & (region >= self.minimum_depth)
                    & (region <= self.maximum_depth)
                )
                if int(np.count_nonzero(valid)) < self.minimum_depth_pixels:
                    continue
                panel_depth = 0.5 * (first.median_depth + second.median_depth)
                deeper_fraction = float(
                    np.count_nonzero(
                        region[valid]
                        >= panel_depth + self.minimum_opening_depth_change
                    )
                    / max(1, int(np.count_nonzero(valid)))
                )
                if deeper_fraction < self.minimum_opening_deeper_fraction:
                    continue
                x0 = min(first.bbox[0], second.bbox[0])
                y0 = min(first.bbox[1], second.bbox[1])
                x1 = max(
                    first.bbox[0] + first.bbox[2],
                    second.bbox[0] + second.bbox[2],
                )
                y1 = max(
                    first.bbox[1] + first.bbox[3],
                    second.bbox[1] + second.bbox[3],
                )
                center = (
                    0.5 * (first.world_xy[0] + second.world_xy[0]),
                    0.5 * (first.world_xy[1] + second.world_xy[1]),
                )
                paired.append(
                    DoorPanelObservation(
                        stamp=first.stamp,
                        distance=0.5 * (first.distance + second.distance),
                        world_xy=center,
                        outward_yaw=math.atan2(
                            math.sin(first.outward_yaw)
                            + math.sin(second.outward_yaw),
                            math.cos(first.outward_yaw)
                            + math.cos(second.outward_yaw),
                        ),
                        width=opening_width,
                        height=min(first.height, second.height),
                        plane_rmse=max(first.plane_rmse, second.plane_rmse),
                        depth_inlier_count=(
                            first.depth_inlier_count + second.depth_inlier_count
                        ),
                        bbox=(x0, y0, x1 - x0, y1 - y0),
                        median_depth=panel_depth,
                        center_dark_fraction=_mask_fraction(dark_mask, gap_roi),
                        panel_state="open",
                    )
                )
        return paired

    def assess_registered_opening_messages(
        self,
        rgb_msg,
        depth_msg,
        camera_info_msg,
        camera_to_world: np.ndarray | None,
        *,
        door_xy: tuple[float, float],
        door_yaw: float,
        door_width: float,
    ) -> DoorOpeningChange:
        """以已注册门平面为基准，确认门洞后方出现更深回波。"""
        if self._bridge is None:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        return self.assess_registered_opening_arrays(
            rgb,
            depth,
            camera_info_msg.K,
            camera_to_world,
            door_xy=door_xy,
            door_yaw=door_yaw,
            door_width=door_width,
        )

    def assess_registered_opening_arrays(
        self,
        rgb_bgr: np.ndarray,
        depth_raw: np.ndarray,
        camera_k: Sequence[float],
        camera_to_world: np.ndarray | None,
        *,
        door_xy: tuple[float, float],
        door_yaw: float,
        door_width: float,
    ) -> DoorOpeningChange:
        """不依赖同视角闭门帧，用注册门平面提供深度基准。

        只检查门洞中央区域：闭合门板会在注册平面附近形成回波；门打开后，
        足量像素必须同时变亮并命中门平面之后的表面。门在视野外、斜视、
        深度不足或无回波都保持未确认。
        """
        if cv2 is None or camera_to_world is None:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        if (
            rgb_bgr is None
            or depth_raw is None
            or rgb_bgr.shape[:2] != depth_raw.shape[:2]
        ):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        matrix = np.asarray(camera_to_world, dtype=float)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        try:
            fx, fy = float(camera_k[0]), float(camera_k[4])
            cx, cy = float(camera_k[2]), float(camera_k[5])
            door_x, door_y = float(door_xy[0]), float(door_xy[1])
            width_m = float(door_width)
            yaw = float(door_yaw)
        except (IndexError, TypeError, ValueError):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        if (
            not all(
                math.isfinite(value)
                for value in (fx, fy, cx, cy, door_x, door_y, width_m, yaw)
            )
            or fx <= 0.0
            or fy <= 0.0
            or width_m <= 0.0
        ):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)

        camera_xy = matrix[:2, 3]
        view_xy = np.asarray([door_x, door_y], dtype=float) - camera_xy
        view_distance = float(np.linalg.norm(view_xy))
        if view_distance <= 1e-6:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        door_normal = np.asarray([math.cos(yaw), math.sin(yaw)], dtype=float)
        if abs(float(np.dot(view_xy / view_distance, door_normal))) < 0.75:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)

        try:
            world_to_camera = np.linalg.inv(matrix)
        except np.linalg.LinAlgError:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        door_camera = world_to_camera @ np.asarray(
            [door_x, door_y, float(matrix[2, 3]), 1.0], dtype=float
        )
        expected_depth = float(door_camera[2])
        if (
            not math.isfinite(expected_depth)
            or expected_depth < self.minimum_depth
            or expected_depth > self.maximum_depth
        ):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)

        height_px, width_px = depth_raw.shape[:2]
        center_u = fx * float(door_camera[0]) / expected_depth + cx
        center_v = fy * float(door_camera[1]) / expected_depth + cy
        projected_width = fx * width_m / expected_depth
        half_width = max(4, int(round(0.20 * projected_width)))
        half_height = max(8, int(round(0.22 * height_px)))
        x0 = max(0, int(math.floor(center_u)) - half_width)
        x1 = min(width_px, int(math.ceil(center_u)) + half_width)
        y0 = max(0, int(math.floor(center_v)) - half_height)
        y1 = min(height_px, int(math.ceil(center_v)) + half_height)
        if x1 - x0 < 8 or y1 - y0 < 16:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)

        rgb_roi = rgb_bgr[y0:y1, x0:x1]
        depth_roi = _depth_to_meters(depth_raw)[y0:y1, x0:x1]
        valid = (
            np.isfinite(depth_roi)
            & (depth_roi >= self.minimum_depth)
            & (depth_roi <= self.maximum_depth)
        )
        valid_count = int(np.count_nonzero(valid))
        area = max(1, int(depth_roi.size))
        valid_fraction = float(valid_count / area)
        deeper_fraction = (
            float(
                np.count_nonzero(
                    depth_roi[valid]
                    >= expected_depth + self.minimum_opening_depth_change
                )
                / valid_count
            )
            if valid_count
            else 0.0
        )
        dark_fraction = _mask_fraction(
            self._dark_mask(rgb_roi),
            (0, 0, rgb_roi.shape[1], rgb_roi.shape[0]),
        )
        confirmed = bool(
            valid_count >= self.minimum_depth_pixels
            and valid_fraction >= self.minimum_opening_valid_depth_fraction
            and deeper_fraction >= self.minimum_opening_deeper_fraction
            and dark_fraction <= self.maximum_opening_dark_ratio
        )
        return DoorOpeningChange(
            confirmed,
            dark_fraction,
            valid_fraction,
            deeper_fraction,
        )

    def assess_opening_change_messages(
        self,
        baseline: DoorPanelObservation,
        rgb_msg,
        depth_msg,
    ) -> DoorOpeningChange:
        if self._bridge is None:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        return self.assess_opening_change_arrays(baseline, rgb, depth)

    def assess_opening_change_arrays(
        self,
        baseline: DoorPanelObservation,
        rgb_bgr: np.ndarray,
        depth_raw: np.ndarray,
    ) -> DoorOpeningChange:
        if cv2 is None or rgb_bgr is None or depth_raw is None:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        if rgb_bgr.shape[:2] != depth_raw.shape[:2]:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        roi = _center_roi(baseline.bbox, rgb_bgr.shape[:2])
        x, y, width, height = roi
        if width <= 0 or height <= 0:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        dark = self._dark_mask(rgb_bgr)
        dark_fraction = _mask_fraction(dark, roi)
        depth = _depth_to_meters(depth_raw)[y : y + height, x : x + width]
        valid = (
            np.isfinite(depth)
            & (depth >= self.minimum_depth)
            & (depth <= self.maximum_depth)
        )
        valid_count = int(np.count_nonzero(valid))
        area = max(1, int(width * height))
        valid_fraction = float(valid_count / area)
        if valid_count:
            deeper = depth[valid] >= (
                baseline.median_depth + self.minimum_opening_depth_change
            )
            deeper_fraction = float(np.count_nonzero(deeper) / valid_count)
        else:
            deeper_fraction = 0.0
        baseline_dark = max(float(baseline.center_dark_fraction), 1e-6)
        confirmed = bool(
            dark_fraction <= baseline_dark * self.maximum_opening_dark_ratio
            and valid_fraction >= self.minimum_opening_valid_depth_fraction
            and deeper_fraction >= self.minimum_opening_deeper_fraction
        )
        return DoorOpeningChange(
            confirmed=confirmed,
            dark_fraction=dark_fraction,
            valid_depth_fraction=valid_fraction,
            deeper_fraction=deeper_fraction,
        )

    def _dark_mask(self, rgb_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.asarray([0, 0, 0], dtype=np.uint8),
            np.asarray(
                [179, self.maximum_saturation, self.maximum_value], dtype=np.uint8
            ),
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 11), dtype=np.uint8),
        )
        return cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), dtype=np.uint8),
        )


def _fit_plane(points: np.ndarray):
    """用 SVD 拟合点集平面，返回单位法向、中心和逐点绝对残差。"""
    if len(points) < 3:
        return None
    center = np.mean(points, axis=0)
    try:
        _u, _singular, vh = np.linalg.svd(points - center, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    normal = vh[-1]
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        return None
    normal = normal / norm
    residuals = np.abs((points - center) @ normal)
    return normal, center, residuals


def _depth_to_meters(depth_raw: np.ndarray) -> np.ndarray:
    if depth_raw.dtype == np.uint16:
        return depth_raw.astype(float) / 1000.0
    return depth_raw.astype(float)


def _center_roi(
    bbox: tuple[int, int, int, int],
    shape: tuple[int, ...],
) -> tuple[int, int, int, int]:
    x, y, width, height = (int(value) for value in bbox)
    inset_x = int(round(0.20 * width))
    inset_y = int(round(0.10 * height))
    x = max(0, x + inset_x)
    y = max(0, y + inset_y)
    width = min(int(shape[1]) - x, max(0, width - 2 * inset_x))
    height = min(int(shape[0]) - y, max(0, height - 2 * inset_y))
    return x, y, width, height


def _mask_fraction(mask: np.ndarray, roi: tuple[int, int, int, int]) -> float:
    x, y, width, height = roi
    if width <= 0 or height <= 0:
        return 0.0
    region = mask[y : y + height, x : x + width]
    return float(np.count_nonzero(region) / max(1, region.size))


def _stamp_to_float(header) -> float | None:
    stamp = getattr(header, "stamp", None) if header is not None else None
    to_sec = getattr(stamp, "to_sec", None)
    return float(to_sec()) if callable(to_sec) else None

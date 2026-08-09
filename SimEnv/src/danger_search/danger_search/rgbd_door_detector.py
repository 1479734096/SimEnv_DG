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
from typing import MutableMapping, Sequence

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
        if (
            not 0 <= self.maximum_value <= 255
            or not 0 <= self.maximum_saturation <= 255
            or self.minimum_pixel_area <= 0
            or self.minimum_depth_pixels <= 0
            or not all(
                math.isfinite(value) and value >= 0.0
                for value in (
                    self.minimum_depth,
                    self.maximum_depth,
                    self.minimum_panel_width,
                    self.maximum_panel_width,
                    self.minimum_panel_height,
                    self.maximum_plane_rmse,
                    self.maximum_vertical_normal_z,
                    self.minimum_opening_depth_change,
                    self.maximum_opening_dark_ratio,
                    self.minimum_opening_deeper_fraction,
                    self.minimum_opening_valid_depth_fraction,
                )
            )
            or self.minimum_depth <= 0.0
            or self.minimum_depth >= self.maximum_depth
            or self.minimum_panel_width <= 0.0
            or self.minimum_panel_width >= self.maximum_panel_width
            or self.minimum_panel_height <= 0.0
            or self.maximum_plane_rmse <= 0.0
            or self.maximum_vertical_normal_z > 1.0
            or self.maximum_opening_dark_ratio > 1.0
            or self.minimum_opening_deeper_fraction > 1.0
            or self.minimum_opening_valid_depth_fraction > 1.0
        ):
            raise ValueError("门板检测的颜色、尺度、深度、平面或开门参数无效")
        self._bridge = CvBridge() if CvBridge is not None else None
        self._diagnostic_frames = 0
        self._last_diagnostics: dict[str, int] = {}
        self._cumulative_diagnostics: dict[str, int] = {}
        self._last_observation_count = 0
        self._recent_panel_width_samples: list[dict[str, object]] = []

    def detect_messages(
        self,
        rgb_msg,
        depth_msg,
        camera_info_msg,
        camera_to_world: np.ndarray | None,
    ) -> list[DoorPanelObservation]:
        """转换 ROS 消息并调用数组检测核心；缺少 world 变换时拒绝检测。"""
        frame_diagnostics: dict[str, int] = {}
        if self._bridge is None:
            frame_diagnostics["missing_cv_bridge"] = 1
            self._store_diagnostics(frame_diagnostics, 0)
            return []
        if camera_to_world is None:
            frame_diagnostics["missing_world_transform"] = 1
            self._store_diagnostics(frame_diagnostics, 0)
            return []
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
            camera_k = camera_info_msg.K
            stamp = _stamp_to_float(getattr(rgb_msg, "header", None))
        except Exception:
            frame_diagnostics["message_conversion_failed"] = 1
            self._store_diagnostics(frame_diagnostics, 0)
            return []
        panel_width_samples: list[dict[str, object]] = []
        observations = self.detect_arrays(
            rgb,
            depth,
            camera_k,
            camera_to_world,
            stamp=stamp,
            diagnostics=frame_diagnostics,
            panel_width_samples=panel_width_samples,
        )
        self._store_diagnostics(
            frame_diagnostics,
            len(observations),
            panel_width_samples=panel_width_samples,
        )
        return observations

    def diagnostics(self) -> dict[str, object]:
        """返回只读检测统计；这些计数不参与任何检测或任务决策。"""
        return {
            "frames": int(self._diagnostic_frames),
            "last": dict(self._last_diagnostics),
            "cumulative": dict(self._cumulative_diagnostics),
            "last_observation_count": int(self._last_observation_count),
            "recent_panel_width_samples": [
                dict(sample) for sample in self._recent_panel_width_samples
            ],
        }

    def reset_diagnostics(self) -> None:
        """开始新的审计窗口；只清计数，不改变任何检测参数或缓存。"""
        self._diagnostic_frames = 0
        self._last_diagnostics = {}
        self._cumulative_diagnostics = {}
        self._last_observation_count = 0
        self._recent_panel_width_samples = []

    def _store_diagnostics(
        self,
        frame_diagnostics: MutableMapping[str, int],
        observation_count: int,
        *,
        panel_width_samples: Sequence[dict[str, object]] = (),
    ) -> None:
        self._diagnostic_frames += 1
        self._last_diagnostics = {
            str(name): int(count)
            for name, count in frame_diagnostics.items()
        }
        self._last_observation_count = int(observation_count)
        for name, count in self._last_diagnostics.items():
            self._cumulative_diagnostics[name] = (
                int(self._cumulative_diagnostics.get(name, 0)) + int(count)
            )
        # 只保留有限审计样本；宽度时序不参与检测或任务决策，扫描结束时一次性输出。
        self._recent_panel_width_samples.extend(
            dict(sample) for sample in panel_width_samples
        )
        if len(self._recent_panel_width_samples) > 256:
            self._recent_panel_width_samples = self._recent_panel_width_samples[-256:]

    def detect_arrays(
        self,
        rgb_bgr: np.ndarray,
        depth_raw: np.ndarray,
        camera_k: Sequence[float],
        camera_to_world: np.ndarray | None,
        *,
        stamp: float | None = None,
        diagnostics: MutableMapping[str, int] | None = None,
        panel_width_samples: list[dict[str, object]] | None = None,
    ) -> list[DoorPanelObservation]:
        """检测一帧深色竖直门板并返回几何验证后的候选。

        每个轮廓内部的有效深度像素先由相机内参反投影，随后拟合平面并检查法向、
        物理宽高和 RMSE，最后用 ``camera_to_world`` 得到门中心和朝外法向。
        """
        def record(name: str) -> None:
            if diagnostics is not None:
                diagnostics[name] = int(diagnostics.get(name, 0)) + 1

        if cv2 is None or camera_to_world is None:
            record("invalid_runtime_or_transform")
            return []
        if not _valid_rgbd_arrays(rgb_bgr, depth_raw):
            record("invalid_rgbd_arrays")
            return []
        try:
            matrix = np.asarray(camera_to_world, dtype=float)
            resolved_stamp = None if stamp is None else float(stamp)
        except (TypeError, ValueError):
            record("invalid_transform_value")
            return []
        if (
            matrix.shape != (4, 4)
            or not np.all(np.isfinite(matrix))
            or not _is_rigid_transform(matrix)
        ):
            record("invalid_rigid_transform")
            return []
        if resolved_stamp is not None and (
            not math.isfinite(resolved_stamp) or resolved_stamp < 0.0
        ):
            record("invalid_stamp")
            return []
        try:
            fx, fy = float(camera_k[0]), float(camera_k[4])
            cx, cy = float(camera_k[2]), float(camera_k[5])
        except (IndexError, TypeError, ValueError):
            record("invalid_camera_intrinsics")
            return []
        if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0.0 or fy <= 0.0:
            record("invalid_camera_intrinsics")
            return []

        depth = _depth_to_meters(depth_raw)
        dark_mask = self._dark_mask(rgb_bgr)
        contours, _ = cv2.findContours(
            dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        observations: list[DoorPanelObservation] = []
        for contour in contours:
            record("contours_total")
            if cv2.contourArea(contour) < self.minimum_pixel_area:
                record("rejected_pixel_area")
                continue
            record("passed_pixel_area")
            x, y, width_px, height_px = cv2.boundingRect(contour)
            if width_px < 8 or height_px < 16:
                record("rejected_pixel_extent")
                continue
            contour_mask = np.zeros(dark_mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
            ys, xs = np.where(contour_mask > 0)
            if len(xs) < self.minimum_depth_pixels:
                record("rejected_mask_pixels")
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
                record("rejected_valid_depth_pixels")
                continue
            record("passed_valid_depth_pixels")

            median_depth = float(np.median(z))
            valid_depth_count = int(len(z))

            # 斜视门板在相机光轴 z 方向有真实深度梯度，不能先围绕中位深度裁剪，
            # 否则会把同一平面的两侧误删并系统性压缩宽度。完整有效轮廓先拟合，
            # 再由平面残差内点剔除门框、背景等不同表面。
            if len(z) > 5000:
                indices = np.linspace(0, len(z) - 1, 5000, dtype=int)
                xs, ys, z = xs[indices], ys[indices], z[indices]
            record("passed_plane_candidate_pixels")
            optical_points = np.column_stack(
                (
                    (xs.astype(float) - cx) * z / fx,
                    (ys.astype(float) - cy) * z / fy,
                    z,
                )
            )
            fit = _fit_plane(optical_points)
            if fit is None:
                record("rejected_initial_plane_fit")
                continue
            normal_optical, center_optical, residuals = fit
            inliers = residuals <= max(0.02, 2.5 * self.maximum_plane_rmse)
            inlier_count = int(np.count_nonzero(inliers))
            if inlier_count < self.minimum_depth_pixels:
                record("rejected_plane_inliers")
                continue
            refined = _fit_plane(optical_points[inliers])
            if refined is None:
                record("rejected_refined_plane_fit")
                continue
            normal_optical, center_optical, residuals = refined
            plane_rmse = float(np.sqrt(np.mean(np.square(residuals))))
            if not math.isfinite(plane_rmse) or plane_rmse > self.maximum_plane_rmse:
                record("rejected_plane_rmse")
                continue
            record("passed_plane_rmse")

            homogeneous = np.column_stack(
                (optical_points[inliers], np.ones(int(np.count_nonzero(inliers))))
            )
            world_points = (matrix @ homogeneous.T).T[:, :3]
            normal_world = matrix[:3, :3] @ normal_optical
            normal_norm = float(np.linalg.norm(normal_world))
            if normal_norm <= 1e-9:
                record("rejected_world_normal")
                continue
            normal_world = normal_world / normal_norm
            if abs(float(normal_world[2])) > self.maximum_vertical_normal_z:
                record("rejected_vertical_normal")
                continue
            record("passed_vertical_normal")

            horizontal_normal = np.asarray(
                [normal_world[0], normal_world[1], 0.0], dtype=float
            )
            horizontal_norm = float(np.linalg.norm(horizontal_normal))
            if horizontal_norm <= 1e-9:
                record("rejected_horizontal_normal")
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
            camera_forward = matrix[:3, :3] @ np.asarray(
                [0.0, 0.0, 1.0], dtype=float
            )
            camera_yaw = math.atan2(
                float(camera_forward[1]), float(camera_forward[0])
            )
            candidate_yaw = math.atan2(
                float(center_world[1] - camera_world[1]),
                float(center_world[0] - camera_world[0]),
            )
            relative_bearing = math.atan2(
                math.sin(candidate_yaw - camera_yaw),
                math.cos(candidate_yaw - camera_yaw),
            )
            if width < self.minimum_panel_width:
                gap = self.minimum_panel_width - width
                if gap > 0.20:
                    width_relation = "below_min_over_0.20"
                elif gap > 0.10:
                    width_relation = "below_min_0.10_to_0.20"
                elif gap > 0.05:
                    width_relation = "below_min_0.05_to_0.10"
                else:
                    width_relation = "below_min_under_0.05"
            elif width > self.maximum_panel_width:
                width_relation = "above_max"
            else:
                width_relation = "within_limits"
            record(f"panel_width_{width_relation}")
            if panel_width_samples is not None:
                panel_width_samples.append(
                    {
                        "stamp": (
                            None
                            if resolved_stamp is None
                            else round(resolved_stamp, 3)
                        ),
                        "width": round(width, 3),
                        "relation": width_relation,
                        "pixel_area": round(float(cv2.contourArea(contour)), 1),
                        "valid_depth_count": valid_depth_count,
                        "plane_candidate_count": int(len(optical_points)),
                        "inlier_count": inlier_count,
                        "median_depth": round(median_depth, 3),
                        "plane_rmse": round(plane_rmse, 4),
                        "bbox": [int(x), int(y), int(width_px), int(height_px)],
                        "world_xy": [
                            round(float(center_world[0]), 3),
                            round(float(center_world[1]), 3),
                        ],
                        "outward_yaw": round(float(outward_yaw), 3),
                        "distance": round(distance, 3),
                        "relative_bearing": round(relative_bearing, 3),
                    }
                )
            if not self.minimum_panel_width <= width <= self.maximum_panel_width:
                record("rejected_panel_width")
                continue
            record("passed_panel_width")
            if height < self.minimum_panel_height:
                record("rejected_panel_height")
                continue
            record("passed_panel_height")

            center_roi = _center_roi((x, y, width_px, height_px), dark_mask.shape)
            center_dark_fraction = _mask_fraction(dark_mask, center_roi)
            observations.append(
                DoorPanelObservation(
                    stamp=resolved_stamp,
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
            record("accepted_panel")
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
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
            camera_k = camera_info_msg.K
        except Exception:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        return self.assess_registered_opening_arrays(
            rgb,
            depth,
            camera_k,
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
        if not _valid_rgbd_arrays(rgb_bgr, depth_raw):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        try:
            matrix = np.asarray(camera_to_world, dtype=float)
        except (TypeError, ValueError):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        if (
            matrix.shape != (4, 4)
            or not np.all(np.isfinite(matrix))
            or not _is_rigid_transform(matrix)
        ):
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
        # 返程电梯门可能只留下局部轿厢深处回波；当整块 ROI 都有有效深度且
        # 几乎没有暗门板像素时，这是强开口证据，不能因单帧回波占比略低于
        # 常规阈值而丢失确认。低有效覆盖或仍呈暗门板外观不走该容错。
        partial_deeper_confirmed = bool(
            valid_fraction >= 0.90
            and dark_fraction <= min(0.10, self.maximum_opening_dark_ratio)
            and deeper_fraction
            >= max(0.15, 0.75 * self.minimum_opening_deeper_fraction)
        )
        confirmed = bool(
            valid_count >= self.minimum_depth_pixels
            and valid_fraction >= self.minimum_opening_valid_depth_fraction
            and (
                deeper_fraction >= self.minimum_opening_deeper_fraction
                or partial_deeper_confirmed
            )
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
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
        except Exception:
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        return self.assess_opening_change_arrays(baseline, rgb, depth)

    def assess_opening_change_arrays(
        self,
        baseline: DoorPanelObservation,
        rgb_bgr: np.ndarray,
        depth_raw: np.ndarray,
    ) -> DoorOpeningChange:
        if cv2 is None or not _valid_rgbd_arrays(rgb_bgr, depth_raw):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        try:
            median_depth = float(baseline.median_depth)
            baseline_dark_fraction = float(baseline.center_dark_fraction)
            roi = _center_roi(baseline.bbox, rgb_bgr.shape[:2])
        except (AttributeError, TypeError, ValueError, OverflowError):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
        if (
            not math.isfinite(median_depth)
            or median_depth <= 0.0
            or not math.isfinite(baseline_dark_fraction)
            or not 0.0 <= baseline_dark_fraction <= 1.0
        ):
            return DoorOpeningChange(False, 1.0, 0.0, 0.0)
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
                median_depth + self.minimum_opening_depth_change
            )
            deeper_fraction = float(np.count_nonzero(deeper) / valid_count)
        else:
            deeper_fraction = 0.0
        baseline_dark = max(baseline_dark_fraction, 1e-6)
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


def _valid_rgbd_arrays(rgb_bgr, depth_raw) -> bool:
    return bool(
        isinstance(rgb_bgr, np.ndarray)
        and isinstance(depth_raw, np.ndarray)
        and rgb_bgr.ndim == 3
        and rgb_bgr.shape[2] == 3
        and rgb_bgr.dtype == np.uint8
        and depth_raw.ndim == 2
        and rgb_bgr.shape[:2] == depth_raw.shape[:2]
        and np.issubdtype(depth_raw.dtype, np.number)
    )


def _is_rigid_transform(matrix: np.ndarray) -> bool:
    rotation = matrix[:3, :3]
    return bool(
        np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6)
        and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
        and np.linalg.det(rotation) > 0.0
    )


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
    if not callable(to_sec):
        return None
    try:
        value = float(to_sec())
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value >= 0.0 else None

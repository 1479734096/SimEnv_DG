"""基于合法 RGB-D 输入的红色球形危险源检测。

检测链路：HSV 红色分割 -> 轮廓初筛 -> 深度反投影 -> RANSAC 球拟合。
颜色只用于生成候选，最终必须通过物理半径和三维拟合误差约束，从而拒绝红色
方块等干扰源。输出仍是单帧候选，必须交给 ``DetectionTracker`` 多帧确认。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Sequence

import numpy as np

from danger_search.detection_tracker import DetectionCandidate

try:
    import cv2
except ImportError:  # pragma: no cover - 仅在缺少 OpenCV 的离线环境触发
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - 仅在缺少 ROS 的离线环境触发
    CvBridge = None


@dataclass(frozen=True)
class SphereFit:
    """一次三维球面拟合的中心、半径、误差和内点数量。"""
    center: np.ndarray
    radius: float
    rmse: float
    inlier_count: int


class RedSphereDetector:
    """从同步 RGB-D 图像中检测红球并输出 world/camera 坐标候选。

    若正式调用要求 world 变换而对应图像时间戳没有匹配位姿，则直接返回空集合，
    不用最新位姿代替。随机采样器使用固定种子，使同一输入的 RANSAC 结果可复现。
    """

    def __init__(
        self,
        *,
        min_area: int = 35,
        min_circularity: float = 0.55,
        min_aspect: float = 0.65,
        max_aspect: float = 1.55,
        min_radius: float = 0.10,
        max_radius: float = 0.22,
        max_rmse: float = 0.035,
        min_depth: float = 0.35,
        max_depth: float = 8.0,
        ransac_iterations: int = 80,
        ransac_sample_size: int = 4,
        random_seed: int = 202602,
    ):
        self.min_area = int(min_area)
        self.min_circularity = float(min_circularity)
        self.min_aspect = float(min_aspect)
        self.max_aspect = float(max_aspect)
        self.min_radius = float(min_radius)
        self.max_radius = float(max_radius)
        self.max_rmse = float(max_rmse)
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.ransac_iterations = int(ransac_iterations)
        self.ransac_sample_size = int(ransac_sample_size)
        if (
            self.min_area <= 0
            or not math.isfinite(self.min_circularity)
            or not 0.0 <= self.min_circularity <= 1.0
            or not all(
                math.isfinite(value) and value > 0.0
                for value in (
                    self.min_aspect,
                    self.max_aspect,
                    self.min_radius,
                    self.max_radius,
                    self.max_rmse,
                    self.min_depth,
                    self.max_depth,
                )
            )
            or self.min_aspect > self.max_aspect
            or self.min_radius >= self.max_radius
            or self.min_depth >= self.max_depth
            or self.ransac_iterations <= 0
            or self.ransac_sample_size < 4
        ):
            raise ValueError("红球检测的形状、尺度、深度或 RANSAC 参数无效")
        self._rng = random.Random(random_seed)
        self._bridge = CvBridge() if CvBridge is not None else None

    def detect_messages(
        self,
        rgb_msg,
        depth_msg,
        camera_info_msg,
        camera_to_world: np.ndarray | None,
        *,
        require_world_transform: bool = False,
    ) -> list[DetectionCandidate]:
        """将 ROS 图像转换为数组后进入无 ROS 依赖的检测核心。"""
        if self._bridge is None:
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
            return []
        return self.detect_arrays(
            rgb,
            depth,
            camera_k,
            camera_to_world,
            stamp=stamp,
            require_world_transform=require_world_transform,
        )

    def detect_arrays(
        self,
        rgb_bgr: np.ndarray,
        depth_raw: np.ndarray,
        camera_k: Sequence[float],
        camera_to_world: np.ndarray | None = None,
        *,
        stamp: float | None = None,
        require_world_transform: bool = False,
    ) -> list[DetectionCandidate]:
        """执行一帧完整检测并返回所有通过几何校验的候选。

        深度点先位于相机光学坐标；提供 ``camera_to_world`` 时才转换到 world。
        ``require_world_transform`` 用于生产链 fail-closed，离线算法测试可省略变换。
        """
        if require_world_transform and camera_to_world is None:
            return []
        if cv2 is None:
            return []
        if rgb_bgr is None or depth_raw is None:
            return []
        if (
            not isinstance(rgb_bgr, np.ndarray)
            or not isinstance(depth_raw, np.ndarray)
            or rgb_bgr.ndim != 3
            or rgb_bgr.shape[2] != 3
            or rgb_bgr.dtype != np.uint8
            or depth_raw.ndim != 2
            or rgb_bgr.shape[:2] != depth_raw.shape[:2]
            or not np.issubdtype(depth_raw.dtype, np.number)
        ):
            return []
        try:
            fx, fy = float(camera_k[0]), float(camera_k[4])
            cx, cy = float(camera_k[2]), float(camera_k[5])
        except (IndexError, TypeError, ValueError):
            return []
        if (
            not all(math.isfinite(value) for value in (fx, fy, cx, cy))
            or fx <= 0.0
            or fy <= 0.0
        ):
            return []
        try:
            resolved_stamp = None if stamp is None else float(stamp)
        except (TypeError, ValueError):
            return []
        if resolved_stamp is not None and (
            not math.isfinite(resolved_stamp) or resolved_stamp < 0.0
        ):
            return []
        matrix = None
        if camera_to_world is not None:
            try:
                matrix = np.asarray(camera_to_world, dtype=float)
            except (TypeError, ValueError):
                return []
            if (
                matrix.shape != (4, 4)
                or not np.all(np.isfinite(matrix))
                or not _is_rigid_transform(matrix)
            ):
                return []

        depth = _depth_to_meters(depth_raw)
        mask = self._red_mask(rgb_bgr)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: list[DetectionCandidate] = []
        for contour in contours:
            if cv2.contourArea(contour) < self.min_area:
                continue
            if not self._passes_2d_shape_filter(contour):
                continue
            points = self._contour_points_3d(
                contour,
                mask,
                depth,
                (fx, 0.0, cx, 0.0, fy, cy),
            )
            if len(points) < 12:
                continue
            sphere = self._fit_sphere_ransac(points)
            if sphere is None:
                continue
            if not self._passes_sphere_filter(sphere):
                continue

            center = sphere.center
            if matrix is not None:
                center = _transform_point(matrix, center)
            if not np.all(np.isfinite(center)):
                continue
            confidence = self._score_detection(contour, sphere)
            candidates.append(
                DetectionCandidate.from_sequence(
                    center.tolist(),
                    confidence=confidence,
                    rmse=sphere.rmse,
                    radius=sphere.radius,
                    stamp=resolved_stamp,
                )
            )
        return candidates

    def _red_mask(self, rgb_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)
        lower_1 = np.array([0, 80, 70], dtype=np.uint8)
        upper_1 = np.array([12, 255, 255], dtype=np.uint8)
        lower_2 = np.array([165, 80, 70], dtype=np.uint8)
        upper_2 = np.array([179, 255, 255], dtype=np.uint8)
        return cv2.bitwise_or(cv2.inRange(hsv, lower_1, upper_1), cv2.inRange(hsv, lower_2, upper_2))

    def _passes_2d_shape_filter(self, contour) -> bool:
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            return False
        area = cv2.contourArea(contour)
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        _, _, width, height = cv2.boundingRect(contour)
        aspect = float(width) / max(float(height), 1.0)
        # 部分遮挡时圆度会下降，因此圆度和长宽比采用联合初筛。
        return circularity >= self.min_circularity or self.min_aspect <= aspect <= self.max_aspect

    def _contour_points_3d(
        self,
        contour,
        mask: np.ndarray,
        depth: np.ndarray,
        camera_k: Sequence[float],
    ) -> np.ndarray:
        contour_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], contourIdx=-1, color=255, thickness=-1)
        contour_mask = cv2.erode(contour_mask, np.ones((3, 3), np.uint8), iterations=1)
        ys, xs = np.where(contour_mask > 0)
        if len(xs) == 0:
            return np.empty((0, 3), dtype=float)

        z = depth[ys, xs].astype(float)
        valid = np.isfinite(z) & (z >= self.min_depth) & (z <= self.max_depth)
        xs = xs[valid]
        ys = ys[valid]
        z = z[valid]
        if len(xs) == 0:
            return np.empty((0, 3), dtype=float)

        fx = float(camera_k[0])
        fy = float(camera_k[4])
        cx = float(camera_k[2])
        cy = float(camera_k[5])
        x = (xs.astype(float) - cx) * z / fx
        y = (ys.astype(float) - cy) * z / fy
        return np.column_stack((x, y, z))

    def _fit_sphere_ransac(self, points: np.ndarray) -> SphereFit | None:
        """用最小样本提出球模型，再以残差内点做最小二乘精炼。"""
        if len(points) < self.ransac_sample_size:
            return None
        best: SphereFit | None = None
        indices = list(range(len(points)))
        for _ in range(self.ransac_iterations):
            sample_indices = self._rng.sample(indices, self.ransac_sample_size)
            fit = _fit_sphere_least_squares(points[sample_indices])
            if fit is None:
                continue
            distances = np.linalg.norm(points - fit.center, axis=1)
            residuals = np.abs(distances - fit.radius)
            inliers = points[residuals <= max(self.max_rmse * 2.0, 0.05)]
            if len(inliers) < self.ransac_sample_size:
                continue
            refined = _fit_sphere_least_squares(inliers)
            if refined is None:
                continue
            if best is None or (refined.inlier_count, -refined.rmse) > (best.inlier_count, -best.rmse):
                best = refined
        return best

    def _passes_sphere_filter(self, sphere: SphereFit) -> bool:
        return (
            self.min_radius <= sphere.radius <= self.max_radius
            and sphere.rmse <= self.max_rmse
            and sphere.inlier_count >= 8
        )

    def _score_detection(self, contour, sphere: SphereFit) -> float:
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1e-6)
        radius_center = (self.min_radius + self.max_radius) / 2.0
        radius_error = abs(sphere.radius - radius_center) / max(radius_center, 1e-6)
        rmse_score = max(0.0, 1.0 - sphere.rmse / max(self.max_rmse, 1e-6))
        score = 0.45 * min(circularity, 1.0) + 0.35 * rmse_score + 0.20 * max(0.0, 1.0 - radius_error)
        return float(max(0.0, min(1.0, score)))


def _fit_sphere_least_squares(points: np.ndarray) -> SphereFit | None:
    if len(points) < 4:
        return None
    a = np.column_stack((2.0 * points[:, 0], 2.0 * points[:, 1], 2.0 * points[:, 2], np.ones(len(points))))
    b = np.sum(points * points, axis=1)
    try:
        solution, *_ = np.linalg.lstsq(a, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    center = solution[:3]
    radius_sq = float(np.dot(center, center) + solution[3])
    if radius_sq <= 0.0 or not np.isfinite(radius_sq):
        return None
    radius = math.sqrt(radius_sq)
    residuals = np.linalg.norm(points - center, axis=1) - radius
    rmse = math.sqrt(float(np.mean(residuals * residuals)))
    return SphereFit(center=center.astype(float), radius=float(radius), rmse=float(rmse), inlier_count=len(points))


def _depth_to_meters(depth_raw: np.ndarray) -> np.ndarray:
    if depth_raw.dtype == np.uint16:
        return depth_raw.astype(float) / 1000.0
    return depth_raw.astype(float)


def _transform_point(matrix: np.ndarray, point: np.ndarray) -> np.ndarray:
    homogeneous = np.array([point[0], point[1], point[2], 1.0], dtype=float)
    transformed = matrix @ homogeneous
    return transformed[:3]


def _is_rigid_transform(matrix: np.ndarray) -> bool:
    rotation = matrix[:3, :3]
    return bool(
        np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6)
        and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
        and np.linalg.det(rotation) > 0.0
    )


def _stamp_to_float(header) -> float | None:
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    if hasattr(stamp, "to_sec"):
        try:
            value = float(stamp.to_sec())
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value >= 0.0 else None
    return None

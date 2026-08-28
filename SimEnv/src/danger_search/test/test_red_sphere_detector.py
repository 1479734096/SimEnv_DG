import math
import unittest

import numpy as np

from danger_search.red_sphere_detector import RedSphereDetector


class RedSphereDetectorTest(unittest.TestCase):
    def setUp(self):
        self.width = 160
        self.height = 120
        self.fx = 300.0
        self.fy = 300.0
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        self.camera_k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            RedSphereDetector(min_radius=0.3, max_radius=0.2)
        with self.assertRaises(ValueError):
            RedSphereDetector(max_rmse=math.nan)

    def test_invalid_intrinsics_transform_and_array_shape_fail_closed(self):
        detector = RedSphereDetector()
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        depth = np.ones((4, 4), dtype=np.float32)

        self.assertEqual(
            detector.detect_arrays(
                rgb,
                depth,
                [0.0, 0.0, 2.0, 0.0, 1.0, 2.0],
                np.eye(4),
            ),
            [],
        )
        invalid_transform = np.eye(4)
        invalid_transform[0, 0] = math.nan
        self.assertEqual(
            detector.detect_arrays(
                rgb,
                depth,
                [1.0, 0.0, 2.0, 0.0, 1.0, 2.0],
                invalid_transform,
            ),
            [],
        )
        self.assertEqual(
            detector.detect_arrays(
                rgb,
                depth[:3],
                [1.0, 0.0, 2.0, 0.0, 1.0, 2.0],
                np.eye(4),
            ),
            [],
        )

    def test_requires_world_transform_when_requested(self):
        detector = RedSphereDetector()
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        depth = np.ones((4, 4), dtype=np.float32)
        camera_k = [1.0, 0.0, 2.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0]

        detections = detector.detect_arrays(
            rgb,
            depth,
            camera_k,
            camera_to_world=None,
            require_world_transform=True,
        )

        self.assertEqual(detections, [])

    def test_detects_synthetic_red_sphere(self):
        detector = RedSphereDetector(min_area=20)
        rgb, depth = self._synthetic_sphere(color_bgr=(0, 0, 255))

        detections = detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            camera_to_world=np.eye(4),
            require_world_transform=True,
        )

        self.assertEqual(len(detections), 1)
        self.assertAlmostEqual(detections[0].position[0], 0.0, places=2)
        self.assertAlmostEqual(detections[0].position[1], 0.0, places=2)
        self.assertAlmostEqual(detections[0].position[2], 2.0, places=2)
        self.assertGreaterEqual(detections[0].confidence, 0.85)
        self.assertAlmostEqual(detections[0].radius, 0.15, places=2)

    def test_world_transform_applies_floor_offset_exactly_once(self):
        detector = RedSphereDetector(min_area=20)
        rgb, depth = self._synthetic_sphere(color_bgr=(0, 0, 255))
        camera_to_world = np.eye(4)
        camera_to_world[2, 3] = 2.6

        detections = detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            camera_to_world=camera_to_world,
            require_world_transform=True,
        )

        self.assertEqual(len(detections), 1)
        self.assertAlmostEqual(detections[0].position[2], 4.6, places=2)

    def test_filters_green_sphere_and_red_flat_square(self):
        detector = RedSphereDetector(min_area=20)
        green_rgb, green_depth = self._synthetic_sphere(color_bgr=(0, 255, 0))
        square_rgb, square_depth = self._flat_red_square()

        green_detections = detector.detect_arrays(
            green_rgb,
            green_depth,
            self.camera_k,
            camera_to_world=np.eye(4),
            require_world_transform=True,
        )
        square_detections = detector.detect_arrays(
            square_rgb,
            square_depth,
            self.camera_k,
            camera_to_world=np.eye(4),
            require_world_transform=True,
        )

        self.assertEqual(green_detections, [])
        self.assertEqual(square_detections, [])

    def test_detects_candidate_with_distance_and_far_field_decay(self):
        detector = RedSphereDetector(min_area=10)
        # 近场红球 (2.0m)
        rgb_near, depth_near = self._synthetic_sphere(color_bgr=(0, 0, 255), center=(0.0, 0.0, 2.0))
        near_detections = detector.detect_arrays(
            rgb_near,
            depth_near,
            self.camera_k,
            camera_to_world=np.eye(4),
            require_world_transform=True,
        )
        self.assertEqual(len(near_detections), 1)
        self.assertAlmostEqual(near_detections[0].distance, 2.0, places=1)
        self.assertGreaterEqual(near_detections[0].confidence, 0.8)

        # 远场红球 (5.0m)
        rgb_far, depth_far = self._synthetic_sphere(color_bgr=(0, 0, 255), center=(0.0, 0.0, 5.0))
        far_detections = detector.detect_arrays(
            rgb_far,
            depth_far,
            self.camera_k,
            camera_to_world=np.eye(4),
            require_world_transform=True,
        )
        self.assertEqual(len(far_detections), 1)
        self.assertAlmostEqual(far_detections[0].distance, 5.0, places=1)
        # 远场置信度受衰减因子降级
        self.assertLess(far_detections[0].confidence, near_detections[0].confidence)

    def _synthetic_sphere(self, color_bgr, center=(0.0, 0.0, 2.0), radius=0.15):
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.zeros((self.height, self.width), dtype=np.float32)
        center_vec = np.array(center, dtype=float)
        for y in range(self.height):
            for x in range(self.width):
                ray = np.array([(x - self.cx) / self.fx, (y - self.cy) / self.fy, 1.0], dtype=float)
                a = float(np.dot(ray, ray))
                b = float(-2.0 * np.dot(ray, center_vec))
                c = float(np.dot(center_vec, center_vec) - radius * radius)
                discriminant = b * b - 4.0 * a * c
                if discriminant < 0.0:
                    continue
                distance = (-b - np.sqrt(discriminant)) / (2.0 * a)
                if distance <= 0.0:
                    continue
                depth[y, x] = distance
                rgb[y, x] = color_bgr
        return rgb, depth

    def _flat_red_square(self, size=60, depth_value=2.0):
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.zeros((self.height, self.width), dtype=np.float32)
        x0 = self.width // 2 - size // 2
        y0 = self.height // 2 - size // 2
        rgb[y0 : y0 + size, x0 : x0 + size] = (0, 0, 255)
        depth[y0 : y0 + size, x0 : x0 + size] = depth_value
        return rgb, depth

    def test_fit_sphere_geometric_accuracy_on_hemisphere(self):
        """验证单侧球冠在固定半径几何高斯-牛顿拟合下球心误差 < 0.02m。"""
        from danger_search.red_sphere_detector import _fit_sphere_geometric

        true_center = np.array([0.4, -0.3, 1.8], dtype=float)
        radius = 0.15
        # 仅采样朝向相机的单侧球冠点 (z 较小的凸表面)
        thetas = np.linspace(0, 2 * np.pi, 20)
        phis = np.linspace(0, np.pi / 3, 10)  # 仅前向 60 度球冠
        points = []
        for phi in phis:
            for theta in thetas:
                p = true_center + radius * np.array([
                    np.sin(phi) * np.cos(theta),
                    np.sin(phi) * np.sin(theta),
                    -np.cos(phi),  # 朝向相机 (z 变小)
                ])
                points.append(p)
        points = np.array(points)
        fit = _fit_sphere_geometric(points, target_radius=radius)
        self.assertIsNotNone(fit)
        error = np.linalg.norm(fit.center - true_center)
        self.assertLess(error, 0.01)  # 误差小于 1 厘米
        self.assertAlmostEqual(fit.radius, radius, places=3)


if __name__ == "__main__":
    unittest.main()

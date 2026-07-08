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


if __name__ == "__main__":
    unittest.main()

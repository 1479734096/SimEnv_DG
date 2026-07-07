import unittest

import numpy as np

from danger_search.red_sphere_detector import RedSphereDetector


class RedSphereDetectorTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()

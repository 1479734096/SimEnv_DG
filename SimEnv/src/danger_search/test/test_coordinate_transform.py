import unittest

import numpy as np

from danger_search.perception_mapping_runtime import ros_link_from_optical_matrix


class CoordinateTransformTest(unittest.TestCase):
    def test_converts_opencv_optical_to_ros_link_axes(self):
        matrix = ros_link_from_optical_matrix()

        optical_forward = np.array([0.0, 0.0, 1.0, 1.0])
        optical_right = np.array([1.0, 0.0, 0.0, 1.0])
        optical_down = np.array([0.0, 1.0, 0.0, 1.0])

        np.testing.assert_allclose((matrix @ optical_forward)[:3], [1.0, 0.0, 0.0])
        np.testing.assert_allclose((matrix @ optical_right)[:3], [0.0, -1.0, 0.0])
        np.testing.assert_allclose((matrix @ optical_down)[:3], [0.0, 0.0, -1.0])


if __name__ == "__main__":
    unittest.main()

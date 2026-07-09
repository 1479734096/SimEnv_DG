import unittest

from danger_search.navigator import compute_tracking_command


class LocalControllerTest(unittest.TestCase):
    def test_large_yaw_error_keeps_forward_component(self):
        linear_x, linear_y, angular = compute_tracking_command(
            1.5,
            max_linear=0.35,
            max_lateral=0.3,
            max_angular=0.9,
            min_linear_fraction=0.25,
        )

        self.assertGreater(linear_x, 0.0)
        self.assertGreater(linear_y, 0.0)
        self.assertAlmostEqual(angular, 0.9)

    def test_straight_target_uses_full_linear_speed(self):
        linear_x, linear_y, angular = compute_tracking_command(
            0.0,
            max_linear=0.35,
            max_lateral=0.3,
            max_angular=0.9,
            min_linear_fraction=0.25,
        )

        self.assertAlmostEqual(linear_x, 0.35)
        self.assertAlmostEqual(linear_y, 0.0)
        self.assertAlmostEqual(angular, 0.0)

    def test_min_linear_fraction_is_clamped(self):
        linear_x, _, _ = compute_tracking_command(
            3.14,
            max_linear=0.4,
            max_lateral=0.3,
            max_angular=1.0,
            min_linear_fraction=2.0,
        )

        self.assertAlmostEqual(linear_x, 0.4)


if __name__ == "__main__":
    unittest.main()

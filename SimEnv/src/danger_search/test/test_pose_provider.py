#!/usr/bin/env python3
"""统一位姿入口的纯 Python 边界测试。"""
import math
import unittest
from types import SimpleNamespace

from danger_search.pose_provider import POSE_EXPIRED, POSE_MATCHED, POSE_PENDING, PoseProvider


def _vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _odometry(stamp, *, x=1.0, quaternion=(0.0, 0.0, 0.0, 1.0)):
    qx, qy, qz, qw = quaternion
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=_vector(x=x, y=2.0, z=0.39),
                orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(linear=_vector(), angular=_vector())
        ),
    )


class PoseProviderTest(unittest.TestCase):
    def test_only_accepts_internal_odometry_topic(self):
        provider = PoseProvider("/danger_search/localization/odom")
        self.assertEqual(provider.odom_topic, "/danger_search/localization/odom")
        with self.assertRaises(ValueError):
            PoseProvider("/Odometry_gazebo")

    def test_lookup_distinguishes_matched_pending_and_expired(self):
        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        for stamp in (1.0, 1.02, 1.04):
            provider._on_odometry(_odometry(stamp))

        self.assertEqual(provider.lookup(1.021).status, POSE_MATCHED)
        self.assertEqual(provider.lookup(1.20).status, POSE_PENDING)
        provider._on_odometry(_odometry(1.30))
        self.assertEqual(provider.lookup(1.20).status, POSE_EXPIRED)

    def test_invalid_and_out_of_order_samples_are_rejected(self):
        provider = PoseProvider("/danger_search/localization/odom", max_samples=8)
        provider._on_odometry(_odometry(2.0))
        provider._on_odometry(_odometry(1.0))
        provider._on_odometry(_odometry(math.nan))
        provider._on_odometry(_odometry(3.0, x=math.inf))
        provider._on_odometry(_odometry(4.0, quaternion=(0.0, 0.0, 0.0, 0.0)))

        self.assertEqual(len(provider._samples), 1)
        self.assertEqual(provider.latest().stamp, 2.0)

    def test_lookup_rejects_non_finite_contract_values(self):
        provider = PoseProvider("/danger_search/localization/odom")
        provider._on_odometry(_odometry(1.0))
        with self.assertRaises(ValueError):
            provider.lookup(math.nan)
        with self.assertRaises(ValueError):
            provider.lookup(1.0, max_delta=math.inf)


if __name__ == "__main__":
    unittest.main()

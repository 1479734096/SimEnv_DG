#!/usr/bin/env python3
"""定位运行时边界的行为刻画测试。"""
import unittest
from types import SimpleNamespace

from danger_search.localization_runtime import LocalizationRuntime
from danger_search.pose_provider import (
    POSE_EXPIRED,
    POSE_MATCHED,
    POSE_PENDING,
    PoseLookup,
    PoseSample,
)


def _sample(stamp: float) -> PoseSample:
    return PoseSample(
        stamp=stamp,
        xyz=(1.0, 2.0, 0.39),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        linear_xyz=(0.0, 0.0, 0.0),
        angular_xyz=(0.0, 0.0, 0.0),
    )


class LocalizationRuntimeTest(unittest.TestCase):
    def test_health_stale_failure_and_recovery_keep_existing_timing(self):
        runtime = LocalizationRuntime(
            SimpleNamespace(observed_period=lambda **_kwargs: 0.1),
            stale_minimum=0.4,
            unavailable_timeout=3.0,
            start_time=0.0,
        )

        self.assertEqual(
            runtime.lookup_stats()["cloud"],
            {POSE_MATCHED: 0, POSE_PENDING: 0, POSE_EXPIRED: 0},
        )

        stale = runtime.assess_health(_sample(10.0), now=10.5)
        self.assertFalse(stale.allows_progress)
        self.assertTrue(stale.stale_started)
        self.assertFalse(stale.failure_started)
        self.assertAlmostEqual(stale.stale_limit, 0.4)

        failed = runtime.assess_health(_sample(10.0), now=13.6)
        self.assertTrue(failed.failure_started)
        self.assertAlmostEqual(failed.unavailable_duration, 3.1)

        repeated = runtime.assess_health(_sample(10.0), now=13.7)
        self.assertFalse(repeated.failure_started)

        recovered = runtime.assess_health(_sample(13.75), now=13.8)
        self.assertTrue(recovered.allows_progress)
        self.assertTrue(recovered.recovered)
        self.assertIsNone(runtime.unavailable_since)

    def test_pending_lookup_is_counted_once_then_matched(self):
        lookups = [
            PoseLookup(POSE_PENDING, None, 0.2),
            PoseLookup(POSE_PENDING, None, 0.1),
            PoseLookup(POSE_MATCHED, _sample(2.0), 0.0),
        ]
        provider = SimpleNamespace(lookup=lambda _stamp: lookups.pop(0))
        runtime = LocalizationRuntime(
            provider,
            stale_minimum=0.4,
            unavailable_timeout=3.0,
            start_time=0.0,
        )

        first = runtime.lookup(2.0)
        second = runtime.lookup(2.0)
        matched = runtime.lookup(2.0)

        self.assertTrue(runtime.record_lookup("rgbd", first, stamp=2.0))
        self.assertFalse(runtime.record_lookup("rgbd", second, stamp=2.0))
        self.assertTrue(runtime.record_lookup("rgbd", matched, stamp=2.0))
        self.assertEqual(
            runtime.lookup_stats()["rgbd"],
            {POSE_MATCHED: 1, POSE_PENDING: 1, POSE_EXPIRED: 0},
        )

    def test_legacy_provider_behavior_remains_explicit(self):
        provider = SimpleNamespace(at=lambda stamp: _sample(float(stamp)))
        runtime = LocalizationRuntime(
            provider,
            stale_minimum=0.4,
            unavailable_timeout=3.0,
            start_time=0.0,
        )

        self.assertEqual(runtime.lookup(1.0).status, POSE_MATCHED)
        self.assertEqual(
            runtime.lookup(1.0, allow_legacy_at=False).status,
            POSE_MATCHED,
        )
        self.assertIsNone(
            runtime.lookup(1.0, allow_legacy_at=False).sample,
        )


if __name__ == "__main__":
    unittest.main()

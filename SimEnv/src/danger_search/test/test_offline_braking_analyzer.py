"""离线制动测量口径测试。"""

import importlib.util
from pathlib import Path
import sys
import unittest


SIMENV_ROOT = Path(__file__).resolve().parents[3]
ANALYZER_PATH = SIMENV_ROOT / "tools" / "offline" / "analyze_braking_bag.py"


def _load_analyzer_module():
    spec = importlib.util.spec_from_file_location("braking_analyzer", ANALYZER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载制动分析脚本: {ANALYZER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class OfflineBrakingAnalyzerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.analyzer = _load_analyzer_module()

    def motion(self, stamp, x, speed, angular=0.0, y=0.0, yaw=0.0):
        return self.analyzer.MotionSample(stamp, x, y, yaw, speed, angular)

    def command(self, stamp, linear, angular=0.0):
        return self.analyzer.CommandSample(stamp, linear, 0.0, angular)

    def test_extracts_traceable_path_forward_and_confirmation_metrics(self):
        commands = [
            self.command(0.0, 0.6, 0.2),
            self.command(0.1, 0.0),
            self.command(0.2, 0.0),
            self.command(0.3, 0.0),
        ]
        truths = [
            self.motion(0.0, 0.00, 0.58),
            self.motion(0.1, 0.02, 0.50),
            self.motion(0.2, 0.07, 0.20, y=0.01),
            self.motion(0.3, 0.09, 0.04, y=0.01),
        ]
        localization = [
            self.motion(0.10, 0.0, 0.04, 0.04),
            self.motion(0.20, 0.0, 0.03, 0.03),
            self.motion(0.30, 0.0, 0.02, 0.02),
        ]

        events = self.analyzer.extract_braking_events(
            commands, localization, truths
        )

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event.route_kind, "弧线")
        self.assertAlmostEqual(event.stop_path_length, 0.070990195, places=7)
        self.assertAlmostEqual(event.forward_displacement, 0.07)
        self.assertAlmostEqual(event.stop_time, 0.2)
        self.assertAlmostEqual(event.localization_confirmation_delay, 0.2)
        self.assertAlmostEqual(
            event.truth_speed_at_localization_confirmation, 0.04
        )

    def test_implicit_low_speed_events_are_excluded_by_declared_threshold(self):
        commands = [self.command(0.0, 0.29), self.command(0.1, 0.0)]
        truths = [self.motion(0.0, 0.0, 0.2), self.motion(0.1, 0.0, 0.04)]

        events = self.analyzer.extract_braking_events(commands, [], truths)

        self.assertEqual(events, [])

    def test_recommand_before_truth_stop_rejects_event(self):
        commands = [
            self.command(0.0, 0.6),
            self.command(0.1, 0.0),
            self.command(0.2, 0.2),
        ]
        truths = [
            self.motion(0.0, 0.0, 0.5),
            self.motion(0.1, 0.0, 0.4),
            self.motion(0.2, 0.04, 0.2),
            self.motion(0.3, 0.06, 0.04),
        ]

        events = self.analyzer.extract_braking_events(commands, [], truths)

        self.assertEqual(events, [])

    def test_localization_confirmation_requires_three_consecutive_samples(self):
        commands = [self.command(0.0, 0.6), self.command(0.1, 0.0)]
        truths = [
            self.motion(0.0, 0.0, 0.5),
            self.motion(0.1, 0.0, 0.04),
            self.motion(0.2, 0.0, 0.04),
            self.motion(0.3, 0.0, 0.04),
            self.motion(0.4, 0.0, 0.04),
            self.motion(0.5, 0.0, 0.04),
        ]
        localization = [
            self.motion(0.1, 0.0, 0.04, 0.04),
            self.motion(0.2, 0.0, 0.06, 0.04),
            self.motion(0.3, 0.0, 0.04, 0.04),
            self.motion(0.4, 0.0, 0.04, 0.04),
            self.motion(0.5, 0.0, 0.04, 0.04),
        ]

        event = self.analyzer.extract_braking_events(
            commands, localization, truths
        )[0]

        self.assertAlmostEqual(event.localization_confirmation_stamp, 0.5)
        self.assertAlmostEqual(event.localization_confirmation_delay, 0.4)

    def test_summary_keeps_empty_measurements_explicit(self):
        summary = self.analyzer.summarize_events([])

        self.assertEqual(summary["event_count"], 0)
        self.assertIsNone(summary["maximum_stop_path_length"])
        self.assertEqual(summary["route_counts"], {"直行": 0, "弧线": 0})

    def test_confirmation_cannot_leak_into_a_later_motion_segment(self):
        commands = [
            self.command(0.0, 0.6),
            self.command(0.1, 0.0),
            self.command(0.2, 0.3),
        ]
        truths = [
            self.motion(0.0, 0.0, 0.5),
            self.motion(0.1, 0.0, 0.04),
            self.motion(0.2, 0.0, 0.04),
            self.motion(0.3, 0.0, 0.04),
            self.motion(0.4, 0.0, 0.04),
        ]
        localization = [
            self.motion(0.2, 0.0, 0.04, 0.04),
            self.motion(0.3, 0.0, 0.04, 0.04),
            self.motion(0.4, 0.0, 0.04, 0.04),
        ]

        event = self.analyzer.extract_braking_events(
            commands, localization, truths
        )[0]

        self.assertIsNone(event.localization_confirmation_stamp)


if __name__ == "__main__":
    unittest.main()

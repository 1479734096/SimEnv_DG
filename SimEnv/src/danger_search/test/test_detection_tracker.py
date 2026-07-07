import unittest

from danger_search.detection_tracker import CONFIRMED, DetectionCandidate, DetectionTracker


class DetectionTrackerTest(unittest.TestCase):
    def test_confirms_stable_target(self):
        tracker = DetectionTracker(
            min_hits=4,
            confirmation_window=10.0,
            min_confidence=0.75,
            max_position_std=0.25,
            max_average_rmse=0.04,
        )

        for index in range(4):
            tracker.update(
                [
                    DetectionCandidate.from_sequence(
                        [1.0 + index * 0.01, 2.0, 0.15],
                        confidence=0.9,
                        rmse=0.02,
                    )
                ],
                now=float(index),
            )

        positions = tracker.confirmed_positions(now=4.0)
        self.assertEqual(len(positions), 1)
        self.assertAlmostEqual(positions[0][0], 1.015, places=3)
        self.assertEqual(tracker.tracks[0].status, CONFIRMED)

    def test_nms_removes_duplicate_confirmed_tracks(self):
        tracker = DetectionTracker(min_hits=1, nms_radius=0.9)
        tracker.update([DetectionCandidate.from_sequence([1.0, 1.0, 0.15], confidence=1.0)], now=0.0)
        tracker.update([DetectionCandidate.from_sequence([1.5, 1.0, 0.15], confidence=1.0)], now=0.0)

        positions = tracker.confirmed_positions(now=1.0)
        self.assertEqual(len(positions), 1)


if __name__ == "__main__":
    unittest.main()

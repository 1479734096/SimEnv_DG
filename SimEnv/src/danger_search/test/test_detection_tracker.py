import math
import unittest

from danger_search.detection_tracker import CONFIRMED, DetectionCandidate, DetectionTracker


class DetectionTrackerTest(unittest.TestCase):
    def test_invalid_candidate_and_configuration_are_rejected(self):
        with self.assertRaises(ValueError):
            DetectionCandidate.from_sequence([math.nan, 0.0, 0.0])
        with self.assertRaises(ValueError):
            DetectionCandidate.from_sequence([0.0, 0.0, 0.0], confidence=1.1)
        with self.assertRaises(ValueError):
            DetectionTracker(association_radius=math.nan)

    def test_multiple_candidates_in_one_frame_do_not_fake_multi_frame_hits(self):
        tracker = DetectionTracker(
            association_radius=0.45,
            min_hits=4,
            nms_radius=0.9,
        )

        tracker.update(
            [
                DetectionCandidate.from_sequence(
                    [1.0 + 0.01 * index, 2.0, 0.15],
                    confidence=0.9,
                    rmse=0.02,
                )
                for index in range(4)
            ],
            now=1.0,
        )

        self.assertEqual(tracker.confirmed_positions(now=1.0), [])
        self.assertTrue(all(len(track.hit_times) == 1 for track in tracker.tracks))

    def test_update_time_cannot_go_backwards(self):
        tracker = DetectionTracker(min_hits=1)
        candidate = DetectionCandidate.from_sequence([1.0, 2.0, 0.15])
        tracker.update([candidate], now=2.0)

        with self.assertRaises(ValueError):
            tracker.update([candidate], now=1.0)
        with self.assertRaises(ValueError):
            tracker.confirmed_positions(now=1.0)

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

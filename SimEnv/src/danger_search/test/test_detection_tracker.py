import math
import unittest

from danger_search.detection_tracker import (
    CONFIRMED,
    TENTATIVE,
    DetectionCandidate,
    DetectionTracker,
)


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

    def test_high_quality_two_frame_tentative_track_can_shortly_protect_path(self):
        """预保护必须早于四帧确认，但不能让同一时间戳重复伪造双帧。"""
        tracker = DetectionTracker(
            min_hits=4,
            obstacle_min_hits=2,
            obstacle_min_confidence=0.90,
            obstacle_max_average_rmse=0.025,
            obstacle_max_age=0.75,
            obstacle_min_observation_span=0.08,
        )
        candidate = DetectionCandidate.from_sequence(
            [1.0, 2.0, 0.15], confidence=0.95, rmse=0.01, distance=1.0
        )

        tracker.update([candidate], now=1.0)
        self.assertEqual(tracker.obstacle_tracks(now=1.0), [])
        tracker.update([candidate], now=1.1)

        protected = tracker.obstacle_tracks(now=1.1)
        self.assertEqual([track.track_id for track in protected], [1])
        self.assertEqual(protected[0].status, TENTATIVE)
        # 未收到新帧时临时保护必须有界撤销，不能永久占住地图。
        self.assertEqual(tracker.obstacle_tracks(now=1.86), [])

        repeated_stamp = DetectionTracker(min_hits=4)
        repeated_stamp.update([candidate], now=2.0)
        repeated_stamp.update([candidate], now=2.0)
        self.assertEqual(repeated_stamp.obstacle_tracks(now=2.0), [])

    def test_nms_removes_duplicate_confirmed_tracks(self):
        tracker = DetectionTracker(min_hits=1, nms_radius=0.9)
        tracker.update([DetectionCandidate.from_sequence([1.0, 1.0, 0.15], confidence=1.0)], now=0.0)
        tracker.update([DetectionCandidate.from_sequence([1.5, 1.0, 0.15], confidence=1.0)], now=0.0)

        positions = tracker.confirmed_positions(now=1.0)
        self.assertEqual(len(positions), 1)

    def test_weighted_position_converges_to_near_field_high_precision_hits(self):
        tracker = DetectionTracker(
            min_hits=4,
            association_radius=0.6,
            min_confidence=0.5,
            max_average_rmse=0.05,
            max_position_std=0.5,
        )
        # 模拟 3 次远场粗测（距离 5m，rmse 0.03，位置偏到 1.3）
        for index in range(3):
            tracker.update(
                [
                    DetectionCandidate.from_sequence(
                        [1.3, 2.0, 0.15],
                        confidence=0.7,
                        rmse=0.03,
                        distance=5.0,
                    )
                ],
                now=float(index),
            )
        # 模拟 3 次近场精测（距离 1.0m，rmse 0.005，位置精准在 1.0）
        for index in range(3, 6):
            tracker.update(
                [
                    DetectionCandidate.from_sequence(
                        [1.0, 2.0, 0.15],
                        confidence=0.95,
                        rmse=0.005,
                        distance=1.0,
                    )
                ],
                now=float(index),
            )

        positions = tracker.confirmed_positions(now=6.0)
        self.assertEqual(len(positions), 1)
        # 简单算术平均为 (3*1.3 + 3*1.0)/6 = 1.15，而加权融合使质心极度收敛在 1.0 附近（误差 < 0.02m）
        self.assertLess(abs(positions[0][0] - 1.0), 0.02)

    def test_invalid_candidate_distance_is_rejected(self):
        with self.assertRaises(ValueError):
            DetectionCandidate.from_sequence([1.0, 2.0, 0.15], distance=-0.5)

    def test_planar_correction_moves_track_mean_and_observation_history(self):
        tracker = DetectionTracker(min_hits=1)
        tracker.update(
            [
                DetectionCandidate.from_sequence(
                    [1.0, 2.0, 0.15], confidence=0.9, distance=2.0
                )
            ],
            now=1.0,
        )
        tracker.update(
            [
                DetectionCandidate.from_sequence(
                    [1.1, 2.1, 0.15], confidence=0.9, distance=2.0
                )
            ],
            now=2.0,
        )

        translated = tracker.translate_planar(0.3, -0.2)

        self.assertEqual(translated, 1)
        track = tracker.tracks[0]
        self.assertAlmostEqual(track.mean_position[0], 1.35)
        self.assertAlmostEqual(track.mean_position[1], 1.85)
        self.assertAlmostEqual(track.positions[0][0], 1.3)
        self.assertAlmostEqual(track.positions[0][1], 1.8)
        self.assertAlmostEqual(track.positions[1][0], 1.4)
        self.assertAlmostEqual(track.positions[1][1], 1.9)
        confirmed = tracker.confirmed_positions(now=2.0)
        self.assertEqual(len(confirmed), 1)
        self.assertAlmostEqual(confirmed[0][0], 1.35)
        self.assertAlmostEqual(confirmed[0][1], 1.85)
        self.assertAlmostEqual(confirmed[0][2], 0.15)

    def test_pending_inspection_target_and_mark_inspected(self):
        tracker = DetectionTracker(min_hits=1)
        # 添加一个距离 3.0m 的候选目标 (0.0, 3.0)
        tracker.update(
            [
                DetectionCandidate.from_sequence(
                    [0.0, 3.0, 0.15],
                    confidence=0.85,
                    distance=3.0,
                )
            ],
            now=1.0,
        )
        # 机器狗在 (0, 0)，距离为 3.0m >= 2.0m，且置信度 0.85 >= 0.70
        target = tracker.get_pending_inspection_target(robot_xy=(0.0, 0.0), min_observe_dist=2.0, min_confidence=0.70)
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target[0], 0.0)
        self.assertAlmostEqual(target[1], 3.0)

        # 标记为已核验
        tracker.mark_inspected(target, radius=0.5)
        self.assertTrue(tracker.tracks[0].inspected)

        # 再次查询应返回 None
        target_after = tracker.get_pending_inspection_target(robot_xy=(0.0, 0.0), min_observe_dist=2.0)
        self.assertIsNone(target_after)


if __name__ == "__main__":
    unittest.main()

import unittest

from danger_search.map_evidence import (
    HIGH_HARD,
    LOW_CANDIDATE,
    ObstacleEvidenceLedger,
)


class _ItemsForbiddenDict(dict):
    def items(self):
        raise AssertionError("反证射线不得枚举全部障碍核心")


class ObstacleEvidenceLedgerTest(unittest.TestCase):
    def test_counter_ray_only_queries_cells_on_ray(self):
        ledger = ObstacleEvidenceLedger(
            floor_index=0,
            size_x=100,
            size_y=100,
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
        )
        target_cell = (10, 10)
        target_xy = ledger.grid_to_world(*target_cell)
        ledger.record_observation(
            *target_xy,
            0.0,
            kind=LOW_CANDIDATE,
            cloud_generation=1,
            robot_xy=(0.05, 1.05),
            map_epoch=1,
            localization_epoch=1,
            low_confirm_generations=3,
            low_confirm_window_generations=10,
            pose_separation=0.15,
        )
        for gx in range(100):
            for gy in range(20, 100):
                ledger.record_observation(
                    *ledger.grid_to_world(gx, gy),
                    0.0,
                    kind=HIGH_HARD,
                    cloud_generation=1,
                    robot_xy=(0.05, 1.05),
                    map_epoch=1,
                    localization_epoch=1,
                    low_confirm_generations=3,
                    low_confirm_window_generations=10,
                    pose_separation=0.15,
                )
        ledger.cores = _ItemsForbiddenDict(ledger.cores)

        removed = ledger.record_counter_ray(
            (0, 10),
            (20, 10),
            hit_cells=frozenset(),
            cloud_generation=2,
            robot_xy=(0.05, 1.05),
            map_epoch=1,
            localization_epoch=1,
            candidate_generations=1,
            candidate_positions=1,
            confirmed_generations=2,
            confirmed_positions=2,
            pose_separation=0.15,
        )

        self.assertEqual(removed, {target_cell})
        self.assertNotIn(target_cell, ledger.cores)
        self.assertEqual(len(ledger.cores), 8000)


if __name__ == "__main__":
    unittest.main()

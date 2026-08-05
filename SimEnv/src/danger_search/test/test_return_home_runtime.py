import unittest
from types import SimpleNamespace

from danger_search.mapper import MultiFloorGridMap
from danger_search.return_home_runtime import ReturnHomeRuntime


class ReturnHomeRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.grid = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        for gy in range(self.grid.size_y):
            for gx in range(self.grid.size_x):
                self.grid.mark_free(gx, gy)

    def test_stuck_feedback_keeps_only_four_distinct_segments(self):
        runtime = ReturnHomeRuntime()
        obstacle = SimpleNamespace(
            distance=0.2,
            world_xyz=(1.0, 2.0, 0.3),
            forward=0.2,
            lateral=0.0,
            relative_height=0.3,
        )

        for index in range(6):
            runtime.record_stuck(
                (1.0, 1.0, 0.0),
                ((1.0, 1.0), (2.0, 1.0 + 0.1 * index)),
                obstacle,
            )

        self.assertEqual(runtime.stuck_count, 6)
        self.assertEqual(len(runtime.failed_segments), 4)
        self.assertEqual(runtime.last_stuck_obstacle["distance"], 0.2)

    def test_departing_stuck_area_resets_counts_but_keeps_veto_segments(self):
        runtime = ReturnHomeRuntime(
            stuck_anchor=(1.0, 1.0),
            stuck_count=3,
            candidate_reject_count=2,
            failed_segments=[(1.0, 1.0, 2.0, 1.0)],
        )

        runtime.clear_stuck_if_departed((2.1, 1.0, 0.0))

        self.assertIsNone(runtime.stuck_anchor)
        self.assertEqual(runtime.stuck_count, 0)
        self.assertEqual(runtime.candidate_reject_count, 0)
        self.assertEqual(runtime.failed_segments, [(1.0, 1.0, 2.0, 1.0)])

    def test_retreat_selection_and_completion_are_runtime_owned(self):
        runtime = ReturnHomeRuntime()
        selection = runtime.select_retreat(
            pose=(4.0, 3.0, 0.0),
            grid=self.grid,
            navigation_trace=[(2.0, 3.0), (3.0, 3.0), (4.0, 3.0)],
            rotation_min_clearance=0.4,
            clearance_margin=0.1,
            maximum_distance=2.0,
        )

        self.assertIsNotNone(selection)
        self.assertEqual(selection.target, (3.0, 3.0))
        runtime.begin_retreat(selection, now=10.0, reverse_speed=-0.15)
        self.assertEqual(
            runtime.tick_retreat((4.0, 3.0, 0.0), now=10.1).action,
            "reverse",
        )
        self.assertEqual(
            runtime.tick_retreat((3.2, 3.0, 0.0), now=11.0).action,
            "completed",
        )
        self.assertIsNone(runtime.retreat_target)

    def test_retreat_timeout_and_heading_error_fail_closed(self):
        runtime = ReturnHomeRuntime(
            retreat_anchor=(4.0, 3.0),
            retreat_target=(3.0, 3.0),
            retreat_deadline=12.0,
        )
        self.assertEqual(
            runtime.tick_retreat((4.0, 3.0, 0.8), now=10.0).action,
            "heading_invalid",
        )

        runtime.retreat_target = (3.0, 3.0)
        runtime.retreat_deadline = 12.0
        self.assertEqual(
            runtime.tick_retreat((4.0, 3.0, 0.0), now=12.0).action,
            "timeout",
        )

    def test_completed_plan_from_old_floor_is_discarded(self):
        runtime = ReturnHomeRuntime()
        runtime.planner = SimpleNamespace(
            floor=0,
            goal_xy=(1.0, 2.0),
            poll=lambda: ("reached", [(0.0, 0.0), (1.0, 2.0)]),
            error=None,
        )

        result = runtime.poll_plan(current_floor=1, goal_xy=(1.0, 2.0))

        self.assertEqual(result.status, "stale")
        self.assertIsNone(runtime.planner)


if __name__ == "__main__":
    unittest.main()

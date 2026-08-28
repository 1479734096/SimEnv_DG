import math
import threading
import time
import unittest

from danger_search.frontier import Frontier, FrontierExplorer
from danger_search.frontier_planning_runtime import (
    MODE_NORMAL,
    MODE_OPPORTUNISTIC,
    FrontierPlanScope,
    FrontierPlanningRuntime,
    compute_frontier_plan,
    live_frontier_failure_deltas,
    rebase_and_validate_frontier_path,
)
from danger_search.mapper import FREE, FloorGrid
from danger_search.planner import PlanningConstraints


class FrontierPlanningRuntimeTest(unittest.TestCase):
    def test_background_plan_uses_isolated_grid_and_explorer(self):
        grid = FloorGrid(0, 1.0, 7.0, 3.0, 0.0, 0.0)
        for gx in range(1, 6):
            grid.set_base_state(gx, 1, FREE)
        start = grid.grid_to_world(1, 1)
        goal = grid.grid_to_world(5, 1)
        frontier = Frontier(((5, 1),), goal, 4.0)
        explorer = FrontierExplorer(min_cluster_size=1, fail_limit=1)
        worker = explorer.planning_copy()
        worker._candidate_targets_for_frontier = lambda *_args: [goal]
        worker._select_free_space_goal = lambda *_args, **_kwargs: (None, [])

        outcome = compute_frontier_plan(
            worker,
            grid.copy(),
            (start[0], start[1], 0.0),
            (frontier,),
            selection_context=None,
            planning_constraints=PlanningConstraints(revision=3),
        )

        self.assertEqual(outcome.status, "selected")
        self.assertEqual(outcome.frontier.centroid, goal)
        self.assertTrue(outcome.path)
        self.assertIsNone(explorer.last_selection_debug)
        self.assertEqual(explorer._blacklist, {})
        self.assertEqual(explorer._fail_counts, {})

    def test_single_worker_and_scope_are_preserved(self):
        runtime = FrontierPlanningRuntime(
            timeout=5.0,
            preplan_min_lead=0.5,
            preplan_max_lead=3.0,
        )
        scope = FrontierPlanScope(
            request_id=1,
            mode=MODE_NORMAL,
            floor=0,
            map_epoch=2,
            localization_epoch=4,
            constraint_revision=6,
            path_sequence=8,
            active_target=None,
        )
        runtime.start(scope, compute=lambda: ("ok",))
        with self.assertRaises(RuntimeError):
            runtime.start(scope, compute=lambda: ("duplicate",))

        deadline = time.monotonic() + 1.0
        poll = runtime.poll()
        while poll.status == "pending" and time.monotonic() < deadline:
            time.sleep(0.001)
            poll = runtime.poll()

        self.assertEqual(poll.status, "ready")
        self.assertEqual(poll.scope, scope)
        self.assertEqual(poll.outcome, ("ok",))

    def test_timeout_keeps_late_result_isolated_until_worker_returns(self):
        runtime = FrontierPlanningRuntime(
            timeout=0.01,
            preplan_min_lead=0.5,
            preplan_max_lead=3.0,
        )
        scope = FrontierPlanScope(1, MODE_NORMAL, 0, 1, 1, 0, 0, None)
        release = threading.Event()
        runtime.start(scope, compute=lambda: (release.wait(1.0), "late"))

        time.sleep(0.02)
        self.assertEqual(runtime.poll().status, "timed_out")
        self.assertTrue(runtime.busy)

        release.set()
        deadline = time.monotonic() + 1.0
        poll = runtime.poll()
        while poll.status not in {"ready", "late"} and time.monotonic() < deadline:
            time.sleep(0.001)
            poll = runtime.poll()
        self.assertEqual(poll.scope, scope)
        self.assertEqual(poll.status, "late")
        self.assertEqual(poll.outcome, (True, "late"))

    def test_dynamic_lead_uses_p90_margin_and_bounds(self):
        runtime = FrontierPlanningRuntime(
            timeout=5.0,
            preplan_min_lead=0.5,
            preplan_max_lead=3.0,
        )
        self.assertEqual(runtime.preplan_lead(), 1.0)

        for duration in (0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 4.0):
            runtime.record_duration(duration)
        self.assertAlmostEqual(runtime.preplan_lead(), 2.05)

        runtime.record_duration(20.0)
        self.assertLessEqual(runtime.preplan_lead(), 3.0)

    def test_wall_eta_requires_recent_positive_route_progress(self):
        runtime = FrontierPlanningRuntime(
            timeout=5.0,
            preplan_min_lead=0.5,
            preplan_max_lead=3.0,
        )
        runtime.observe_route_progress(2.0, wall_time=10.0)
        runtime.observe_route_progress(1.5, wall_time=11.0)
        self.assertAlmostEqual(runtime.route_eta(1.0, wall_time=11.1), 2.0)
        self.assertIsNone(runtime.route_eta(1.0, wall_time=12.1))

        runtime.reset_route_progress()
        self.assertIsNone(runtime.route_eta(0.2, wall_time=12.2))

    def test_opportunistic_period_is_three_seconds_and_reserves_preplan_window(self):
        runtime = FrontierPlanningRuntime(
            timeout=5.0,
            preplan_min_lead=0.5,
            preplan_max_lead=3.0,
            opportunistic_period=3.0,
        )
        runtime.mark_opportunistic_started(10.0)
        self.assertFalse(runtime.opportunistic_due(12.99, route_eta=None))
        self.assertTrue(runtime.opportunistic_due(13.0, route_eta=None))
        self.assertFalse(
            runtime.opportunistic_due(
                13.0,
                route_eta=runtime.preplan_lead() * 2.0,
            )
        )

        disabled_runtime = FrontierPlanningRuntime(
            timeout=5.0,
            preplan_min_lead=0.5,
            preplan_max_lead=3.0,
            opportunistic_period=0.0,
        )
        self.assertFalse(disabled_runtime.opportunistic_due(100.0, route_eta=None))

    def test_live_path_is_rebased_and_fail_closed_when_blocked(self):
        grid = FloorGrid(0, 1.0, 7.0, 3.0, 0.0, 0.0)
        for gx in range(1, 6):
            grid.set_base_state(gx, 1, FREE)
        points = [grid.grid_to_world(gx, 1) for gx in range(1, 6)]

        rebased = rebase_and_validate_frontier_path(
            grid,
            (points[1][0], points[1][1], 0.0),
            tuple(points),
            planning_constraints=None,
        )
        self.assertEqual(rebased[0], points[1])
        self.assertEqual(rebased[-1], points[-1])

        grid.mark_black(3, 1, owner="test:stale_async_path")
        self.assertIsNone(
            rebase_and_validate_frontier_path(
                grid,
                (points[1][0], points[1][1], 0.0),
                tuple(points),
                planning_constraints=None,
            )
        )

    def test_failure_delta_is_dropped_after_frontier_is_observed(self):
        grid = FloorGrid(0, 1.0, 3.0, 3.0, 0.0, 0.0)
        grid.set_base_state(1, 1, FREE)
        deltas = (((1, 1), 1),)
        self.assertEqual(live_frontier_failure_deltas(grid, deltas), deltas)

        for gy in range(grid.size_y):
            for gx in range(grid.size_x):
                grid.set_base_state(gx, gy, FREE)
        self.assertEqual(live_frontier_failure_deltas(grid, deltas), ())


if __name__ == "__main__":
    unittest.main()

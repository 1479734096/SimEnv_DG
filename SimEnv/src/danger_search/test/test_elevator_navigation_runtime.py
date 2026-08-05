import unittest
from types import SimpleNamespace

from danger_search.elevator_navigation_runtime import (
    elevator_approach_trace_candidates,
    elevator_start_retreat_target,
)
from danger_search.mapper import FREE, OCC, MultiFloorGridMap


class ElevatorNavigationRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.grid = MultiFloorGridMap(
            resolution=0.1,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        ).floor(0)
        self.grid.data[:, :] = FREE

    def test_approach_candidates_only_use_traversable_hall_side_trace(self):
        elevator = SimpleNamespace(lobby_poses={0: (0.5, 2.0, 0.0)})
        trace = [(0.0, 2.0), (0.6, 2.3), (2.0, 2.0), (3.8, 2.0)]

        candidates = elevator_approach_trace_candidates(
            elevator=elevator,
            floor_index=0,
            navigation_trace=trace,
            grid=self.grid,
            visual_max_distance=4.0,
            max_candidates=4,
        )

        self.assertEqual(candidates, [(2.0, 2.0)])

    def test_retreat_only_exits_initial_occ_band_along_trace(self):
        start = self.grid.world_to_grid(2.0, 2.0)
        for offset in range(3):
            self.grid.data[start[1], start[0] - offset] = OCC

        target = elevator_start_retreat_target(
            pose=(2.0, 2.0, 0.0),
            grid=self.grid,
            navigation_trace=[(1.0, 2.0), (1.8, 2.0), (2.0, 2.0)],
            rotation_min_clearance=0.40,
            retreat_clearance_margin=0.10,
        )

        self.assertEqual(target, (1.0, 2.0))
        self.assertEqual(int(self.grid.data[start[1], start[0]]), OCC)


if __name__ == "__main__":
    unittest.main()

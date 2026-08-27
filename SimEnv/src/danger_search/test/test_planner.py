"""A* 路径规划相关测试。"""
from __future__ import annotations

import math
import unittest

import numpy as np

from danger_search.map_evidence import HIGH_HARD, LOW_CANDIDATE, LOW_CONFIRMED
from danger_search.mapper import FREE, FloorGrid
from danger_search.planner import (
    PlanningConstraints,
    SupportVetoRegion,
    BUDGET_EXHAUSTED,
    REACHED,
    UNREACHABLE,
    astar_path,
    astar_plan,
    path_length,
    simplify_path_line_of_sight,
)


def test_planning_constraints_apply_to_start_goal_search_and_los_without_map_write():
    grid = FloorGrid(
        floor_index=0,
        resolution=1.0,
        width=7.0,
        length=5.0,
        origin_x=0.0,
        origin_y=0.0,
    )
    grid.fill_base(FREE)
    original = grid.data.copy()
    blocked = frozenset({(3, 1), (3, 2), (3, 3)})
    constraints = PlanningConstraints(blocked_cells=blocked)

    result = astar_plan(
        grid,
        grid.grid_to_world(1, 2),
        grid.grid_to_world(5, 2),
        constraints=constraints,
    )

    assert result.reached
    assert all(grid.world_to_grid(*point) not in blocked for point in result.path)
    assert np.array_equal(grid.data, original)


def test_start_veto_requires_complete_monotonic_exit_prefix():
    grid = FloorGrid(
        floor_index=0,
        resolution=1.0,
        width=8.0,
        length=5.0,
        origin_x=0.0,
        origin_y=0.0,
    )
    grid.fill_base(FREE)
    blocked = frozenset({(1, 2), (2, 2), (3, 2)})
    region = SupportVetoRegion("r1", blocked, frozenset({(0, 2)}))
    constraints = PlanningConstraints.from_regions((region,), revision=1)

    result = astar_plan(
        grid,
        grid.grid_to_world(1, 2),
        grid.grid_to_world(6, 2),
        constraints=constraints,
    )

    assert result.reached
    cells = [grid.world_to_grid(*point) for point in result.path]
    assert cells[:4] == [(1, 2), (2, 2), (3, 2), (4, 2)]
    assert all(cell not in blocked for cell in cells[4:])


def test_start_veto_cannot_exempt_cell_owned_by_foreign_region():
    grid = FloorGrid(
        floor_index=0, resolution=1.0, width=7.0, length=3.0,
        origin_x=0.0, origin_y=0.0,
    )
    grid.fill_base(FREE)
    owner = SupportVetoRegion(
        "owner", frozenset({(1, 1), (2, 1)}), frozenset({(0, 1)})
    )
    foreign = SupportVetoRegion(
        "foreign", frozenset({(2, 1), (3, 1)}), frozenset({(3, 0)})
    )
    result = astar_plan(
        grid, grid.grid_to_world(1, 1), grid.grid_to_world(5, 1),
        constraints=PlanningConstraints.from_regions((owner, foreign), revision=1),
        max_iterations=None,
    )
    assert result.unreachable
    assert result.reason_code == "support_veto_escape_unreachable"


def _record_hard_core(grid, cell, *, radius=1.1, map_epoch=1):
    wx, wy = grid.grid_to_world(*cell)
    grid.record_obstacle_observation(
        wx,
        wy,
        radius,
        kind=HIGH_HARD,
        cloud_generation=1,
        robot_xy=(wx, wy),
        map_epoch=map_epoch,
        localization_epoch=1,
        low_confirm_generations=3,
        low_confirm_window_generations=20,
        pose_separation=0.15,
    )


def test_inflated_occ_start_uses_same_monotonic_escape_search():
    grid = FloorGrid(
        floor_index=0, resolution=1.0, width=8.0, length=5.0,
        origin_x=0.0, origin_y=0.0,
    )
    grid.fill_base(FREE)
    _record_hard_core(grid, (1, 2))
    start = grid.grid_to_world(2, 2)
    result = astar_plan(grid, start, grid.grid_to_world(6, 2))

    assert result.reached
    assert [grid.world_to_grid(*point) for point in result.path][:2] == [
        (2, 2), (3, 2)
    ]
    assert result.blocker["obstacle_owner_cells"] == ((1, 2),)


def _record_low_confirmed_core(grid, cell, *, radius=1.1):
    """构造多帧低回波确认的 LOW_CONFIRMED 核心（3 代次 + 分离位姿）。"""
    wx, wy = grid.grid_to_world(*cell)
    for gen in (1, 2, 3):
        grid.record_obstacle_observation(
            wx,
            wy,
            radius,
            kind=LOW_CANDIDATE,
            cloud_generation=gen,
            robot_xy=(wx + gen * 0.2, wy),
            map_epoch=1,
            localization_epoch=1,
            low_confirm_generations=3,
            low_confirm_window_generations=20,
            pose_separation=0.15,
        )
    assert grid.occ_cores[cell].kind == LOW_CONFIRMED


def test_obstacle_core_start_is_never_exempted():
    # HIGH_HARD（可靠实体障碍，如贴墙）起点本体保持 fail-closed，不豁免。
    grid = FloorGrid(
        floor_index=0, resolution=1.0, width=8.0, length=5.0,
        origin_x=0.0, origin_y=0.0,
    )
    grid.fill_base(FREE)
    _record_hard_core(grid, (2, 2))
    result = astar_plan(
        grid, grid.grid_to_world(2, 2), grid.grid_to_world(6, 2)
    )

    assert result.unreachable
    assert result.reason_code == "start_cell_obstacle_core"


def test_low_confirmed_start_core_self_exempted_when_base_traversable():
    # LOW_CONFIRMED（多帧低回波确认，seed=8127 链 B 自腿伪影）起点本体自证豁免：
    # 机器人 pose 落在此格=物理可通行，放开起点格及其 base 可通行膨胀邻域后可达。
    grid = FloorGrid(
        floor_index=0, resolution=1.0, width=8.0, length=5.0,
        origin_x=0.0, origin_y=0.0,
    )
    grid.fill_base(FREE)
    _record_low_confirmed_core(grid, (2, 2))
    result = astar_plan(
        grid, grid.grid_to_world(2, 2), grid.grid_to_world(6, 2)
    )

    assert result.reached
    assert result.blocker.get("start_core_self") is True


def test_low_confirmed_start_core_still_blocked_when_base_untraversable():
    # base 层非 FREE/VISITED（从未探测的 UNKNOWN）时自证不成立，仍不可达。
    grid = FloorGrid(
        floor_index=0, resolution=1.0, width=8.0, length=5.0,
        origin_x=0.0, origin_y=0.0,
    )
    _record_low_confirmed_core(grid, (2, 2))
    result = astar_plan(
        grid, grid.grid_to_world(2, 2), grid.grid_to_world(6, 2)
    )

    assert result.unreachable


def test_inflated_start_rejects_cross_epoch_owner():
    grid = FloorGrid(
        floor_index=0, resolution=1.0, width=8.0, length=5.0,
        origin_x=0.0, origin_y=0.0,
    )
    grid.fill_base(FREE)
    _record_hard_core(grid, (1, 2))
    grid.evidence_scope = (0, 2, 1)
    result = astar_plan(
        grid, grid.grid_to_world(2, 2), grid.grid_to_world(6, 2)
    )

    assert result.unreachable
    assert result.reason_code == "start_obstacle_owner_scope_mismatch"


class PlannerTest(unittest.TestCase):
    def _make_long_corridor(self, resolution: float = 0.5) -> FloorGrid:
        grid = FloorGrid(
            floor_index=0,
            resolution=resolution,
            width=20.0,
            length=2.0,
            origin_x=-10.0,
            origin_y=-1.0,
        )
        for gx in range(grid.size_x):
            for gy in range(1, grid.size_y - 1):
                grid.mark_free(gx, gy)
        return grid

    def test_astar_finds_full_path_when_reachable(self):
        grid = self._make_long_corridor()
        start = grid.grid_to_world(2, 2)
        goal = grid.grid_to_world(grid.size_x - 3, 2)

        path = astar_path(grid, start, goal)

        self.assertTrue(path)
        self.assertGreater(path_length(path), 0.0)
        self.assertAlmostEqual(path[0][0], start[0], places=5)
        self.assertAlmostEqual(path[-1][0], goal[0], places=5)

    def test_line_of_sight_simplifies_grid_staircase_to_straight_segment(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        path = [
            grid.grid_to_world(10, 10),
            grid.grid_to_world(11, 11),
            grid.grid_to_world(11, 12),
            grid.grid_to_world(12, 13),
            grid.grid_to_world(12, 14),
        ]

        simplified = simplify_path_line_of_sight(grid, path)

        self.assertEqual(simplified, [path[0], path[-1]])

    def test_line_of_sight_keeps_bend_around_obstacle(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        obstacle = (grid.size_x // 2, grid.size_y // 2)
        grid.mark_occupied(*obstacle)
        start = grid.grid_to_world(obstacle[0] - 3, obstacle[1])
        goal = grid.grid_to_world(obstacle[0] + 3, obstacle[1])

        result = astar_plan(grid, start, goal, max_iterations=None)

        self.assertTrue(result.reached)
        self.assertGreater(len(result.path), 2)

    def test_line_of_sight_does_not_cut_blocked_diagonal_corner(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        grid.mark_occupied(3, 2)
        path = [
            grid.grid_to_world(2, 2),
            grid.grid_to_world(2, 3),
            grid.grid_to_world(3, 3),
        ]

        simplified = simplify_path_line_of_sight(grid, path)

        self.assertEqual(simplified, path)

    def test_astar_returns_empty_when_budget_exhausted_without_progress(self):
        grid = self._make_long_corridor()
        start = grid.grid_to_world(2, 2)
        goal = grid.grid_to_world(grid.size_x - 3, 2)

        # 只让算法展开一步，best_node 仍然是 start，必须返回 [] 而不是零长度路径
        path = astar_path(grid, start, goal, max_iterations=1)

        self.assertEqual(path, [])

    def test_astar_returns_empty_when_budget_exhausted_with_progress(self):
        grid = self._make_long_corridor(resolution=0.2)
        start = grid.grid_to_world(5, 5)
        goal = grid.grid_to_world(grid.size_x - 5, 5)

        # 预算足够走一段但到不了终点，仍必须返回 []，避免上层把部分路径误判为已到达目标。
        path = astar_path(grid, start, goal, max_iterations=50)

        self.assertEqual(path, [])

    def test_astar_returns_empty_for_truly_unreachable(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        # 在中间竖一堵墙
        wall_x = grid.size_x // 2
        for gy in range(grid.size_y):
            grid.mark_occupied(wall_x, gy)

        start = grid.grid_to_world(1, grid.size_y // 2)
        goal = grid.grid_to_world(grid.size_x - 2, grid.size_y // 2)

        path = astar_path(grid, start, goal)

        self.assertEqual(path, [])

    def test_path_length_empty_or_single(self):
        self.assertEqual(path_length([]), 0.0)
        self.assertEqual(path_length([(0.0, 0.0)]), 0.0)

    def test_astar_plan_status_reached_for_reachable_goal(self):
        grid = self._make_long_corridor()
        start = grid.grid_to_world(2, 2)
        goal = grid.grid_to_world(grid.size_x - 3, 2)

        result = astar_plan(grid, start, goal)

        self.assertEqual(result.status, REACHED)
        self.assertTrue(result.reached)
        self.assertGreater(path_length(result.path), 0.0)

    def test_astar_plan_status_budget_exhausted(self):
        grid = self._make_long_corridor(resolution=0.2)
        start = grid.grid_to_world(5, 5)
        goal = grid.grid_to_world(grid.size_x - 5, 5)

        result = astar_plan(grid, start, goal, max_iterations=50)

        self.assertEqual(result.status, BUDGET_EXHAUSTED)
        self.assertTrue(result.budget_exhausted)
        self.assertFalse(result.unreachable)
        self.assertEqual(result.path, [])

    def test_astar_plan_status_unreachable(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        wall_x = grid.size_x // 2
        for gy in range(grid.size_y):
            grid.mark_occupied(wall_x, gy)

        start = grid.grid_to_world(1, grid.size_y // 2)
        goal = grid.grid_to_world(grid.size_x - 2, grid.size_y // 2)

        result = astar_plan(grid, start, goal, max_iterations=None)

        self.assertEqual(result.status, UNREACHABLE)
        self.assertTrue(result.unreachable)
        self.assertFalse(result.budget_exhausted)

    def test_astar_rejects_occupied_start_cell(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=0.5,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        start_cell = (2, 2)
        grid.mark_occupied(*start_cell)

        result = astar_plan(
            grid,
            grid.grid_to_world(*start_cell),
            grid.grid_to_world(6, 6),
            max_iterations=None,
        )

        self.assertEqual(result.status, UNREACHABLE)
        self.assertEqual(result.path, [])

    def test_blocked_goal_uses_geometrically_nearest_free_cell(self):
        grid = FloorGrid(
            floor_index=0,
            resolution=1.0,
            width=5.0,
            length=5.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        for gx in range(grid.size_x):
            for gy in range(grid.size_y):
                grid.mark_free(gx, gy)
        goal_cell = (2, 2)
        grid.mark_occupied(*goal_cell)

        result = astar_plan(
            grid,
            grid.grid_to_world(2, 0),
            grid.grid_to_world(*goal_cell),
            max_iterations=None,
        )

        self.assertTrue(result.reached)
        self.assertEqual(result.path[-1], grid.grid_to_world(2, 1))

    def test_astar_prefers_corridor_centerline_over_wall_hugging(self):
        # 验证在 3 格宽 (0.3m 分辨率) 的走廊中，A* 规划严格沿中央轴线行进，不贴墙走
        grid = FloorGrid(
            floor_index=0,
            resolution=0.1,
            width=3.0,
            length=10.0,
            origin_x=0.0,
            origin_y=0.0,
        )
        grid.fill_base(FREE)
        # 两侧为墙体 (x < 0.8 或 x > 2.2)
        for y in range(grid.size_y):
            for x in range(8):
                grid.planning_layer[y, x] = 100
            for x in range(22, grid.size_x):
                grid.planning_layer[y, x] = 100

        result = astar_plan(
            grid,
            (1.5, 1.0),
            (1.5, 8.0),
        )
        self.assertTrue(result.reached)
        # 路径上所有点的 X 坐标都应严格保持在走廊中心 1.5 附近
        for pt in result.path:
            self.assertAlmostEqual(pt[0], 1.55, delta=0.1)


if __name__ == "__main__":
    unittest.main()


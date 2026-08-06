"""换层前路线就绪门（route gate）单测。

覆盖 _floor_change_route_ready 的五条分支：
- 未到检查点：放行；
- 电梯等待点 A* 可达：放行换层；
- UNKNOWN 阻断（乐观可达）：延后换层继续探索（预算足够时不写失败）；
- UNKNOWN 阻断且预算裁剪 <15s：显式失败 elevator_route_not_ready；
- 硬障碍围堵（乐观也不可达）：fail-closed 显式失败并记录 blocker。
"""
from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

from danger_search.mapper import FloorGrid
from danger_search.map_evidence import HIGH_HARD
from danger_search.mission_manager import MissionManager, WRITE_RESULT


class FakeElevator:
    def __init__(self, lobby_poses):
        self.lobby_poses = lobby_poses


def _make_manager(grid, *, floor_timeout=280.0, route_check_ratio=0.75,
                  max_runtime=620.0, return_safety_buffer=60.0,
                  elapsed=300.0, elevator_serves=(0, 1)):
    manager = MissionManager.__new__(MissionManager)
    manager.current_floor = 0
    manager.max_floor = 2
    manager.floor_timeout = float(floor_timeout)
    manager.route_check_ratio = float(route_check_ratio)
    manager.max_runtime = float(max_runtime)
    manager.return_safety_buffer = float(return_safety_buffer)
    lobby = {floor: (3.0, 3.0, 0.0) for floor in elevator_serves}
    client = SimpleNamespace(
        elevators={"elev": FakeElevator(lobby)},
        get_elevator_points=lambda eid, floor, **kw: ((3.8, 3.0), (2.05, 3.0)),
    )
    manager.building_client = client
    manager._map_runtime = SimpleNamespace(
        mapper=SimpleNamespace(floor=lambda floor_index: grid)
    )
    manager._elapsed = lambda: float(elapsed)
    manager._estimate_floor_change_time = lambda target, pose: 40.0
    manager._estimate_return_time = lambda pose: 60.0
    manager._planning_constraints = lambda: None
    manager._failure_reasons = []
    manager._record_failure = manager._failure_reasons.append
    manager._transitions = []
    manager._transition_state = (
        lambda state, reason_code=None, call_site=None, **kw: manager._transitions.append(
            (state, reason_code)
        )
    )
    manager._record_floor_exit_certificate = lambda *a, **kw: None
    return manager


def _free_grid():
    grid = FloorGrid(
        floor_index=0, resolution=0.5, width=10.0, length=10.0,
        origin_x=0.0, origin_y=0.0,
    )
    grid.fill_base(0)  # FREE
    return grid


def _unknown_grid():
    # base 全 UNKNOWN（默认）
    return FloorGrid(
        floor_index=0, resolution=0.5, width=10.0, length=10.0,
        origin_x=0.0, origin_y=0.0,
    )


def _blocked_grid():
    grid = _free_grid()
    # 贯穿全宽的 HIGH_HARD 墙（y=2 整行），把起点与电梯等待点完全隔开
    for gx in range(0, 20):
        wx, wy = grid.grid_to_world(gx, 4)
        grid.record_obstacle_observation(
            wx, wy, 0.0, kind=HIGH_HARD, cloud_generation=1,
            robot_xy=(wx, wy), map_epoch=1, localization_epoch=1,
            low_confirm_generations=1, low_confirm_window_generations=1,
            pose_separation=0.0,
        )
    return grid


class RouteReadyGateTest(unittest.TestCase):
    def test_gate_passes_before_checkpoint(self):
        # floor_elapsed=150 < 280*0.75=210：未到检查点，直接放行
        manager = _make_manager(_unknown_grid())
        self.assertTrue(
            manager._floor_change_route_ready(
                150.0, (1.0, 1.0, 0.0), decision_reason="floor_timeout_change_floor"
            )
        )
        self.assertEqual(manager._transitions, [])

    def test_gate_passes_when_route_reachable(self):
        manager = _make_manager(_free_grid())
        self.assertTrue(
            manager._floor_change_route_ready(
                250.0, (1.0, 1.0, 0.0), decision_reason="floor_timeout_change_floor"
            )
        )
        self.assertEqual(manager._transitions, [])

    def test_gate_defers_on_unknown_block_when_budget_ok(self):
        # 全 UNKNOWN：正式 A* 不可达，但乐观（UNKNOWN 可穿）可达 → unknown_blocked
        manager = _make_manager(_unknown_grid())
        result = manager._floor_change_route_ready(
            250.0, (1.0, 1.0, 0.0), decision_reason="floor_timeout_change_floor"
        )
        self.assertFalse(result)
        # 延后探索：不写 WRITE_RESULT、不记录失败
        self.assertEqual(manager._transitions, [])
        self.assertEqual(manager._failure_reasons, [])

    def test_gate_fails_when_budget_exhausted_after_unknown_block(self):
        # max_runtime=320、elapsed=300：remaining=20，裁剪后 <15s → 显式失败
        manager = _make_manager(
            _unknown_grid(), max_runtime=320.0, elapsed=300.0
        )
        result = manager._floor_change_route_ready(
            250.0, (1.0, 1.0, 0.0), decision_reason="floor_timeout_change_floor"
        )
        self.assertFalse(result)
        self.assertEqual(manager._transitions[-1][0], WRITE_RESULT)
        self.assertEqual(manager._transitions[-1][1], "elevator_route_not_ready")

    def test_gate_fails_on_hard_block(self):
        # HIGH_HARD 墙围堵：正式不可达且乐观也不可达 → fail-closed 显式失败
        manager = _make_manager(_blocked_grid())
        result = manager._floor_change_route_ready(
            250.0, (1.0, 1.0, 0.0), decision_reason="floor_timeout_change_floor"
        )
        self.assertFalse(result)
        self.assertEqual(manager._transitions[-1][0], WRITE_RESULT)
        self.assertEqual(manager._transitions[-1][1], "elevator_route_not_ready")
        self.assertTrue(manager._failure_reasons)

    def test_gate_no_usable_elevator_fails_closed(self):
        # 电梯未注册（lobby_poses 不含当前层）→ no_usable_elevator → 显式失败
        manager = _make_manager(_free_grid(), elevator_serves=(1,))
        result = manager._floor_change_route_ready(
            250.0, (1.0, 1.0, 0.0), decision_reason="floor_timeout_change_floor"
        )
        self.assertFalse(result)
        self.assertEqual(manager._transitions[-1][0], WRITE_RESULT)
        self.assertEqual(manager._transitions[-1][1], "elevator_route_not_ready")


if __name__ == "__main__":
    unittest.main()

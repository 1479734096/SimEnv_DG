"""任务层最小进度监督器单元测试。"""
from __future__ import annotations

import math
import unittest
from dataclasses import dataclass

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from danger_search.progress_monitor import ProgressMonitor


@dataclass
class _FakeFrontier:
    centroid: tuple[float, float]


class ProgressMonitorTests(unittest.TestCase):
    def _make_monitor(
        self,
        *,
        goal_tolerance: float = 0.35,
        window: float = 5.0,
        repeat: int = 3,
        min_displacement: float = 0.20,
        cooldown: float = 10.0,
    ) -> ProgressMonitor:
        return ProgressMonitor(
            goal_tolerance=goal_tolerance,
            window=window,
            repeat=repeat,
            min_displacement=min_displacement,
            cooldown=cooldown,
        )

    def test_same_region_repeated_goal_reached_no_movement_triggers_cooldown(self):
        """同一区域近邻目标反复被选中且机器人不移动时应触发 cooldown。"""
        monitor = self._make_monitor()
        pose = (0.0, 0.0, 0.0)
        target = (0.30, 0.0)
        for t in [0.0, 0.1, 0.2, 0.3, 0.4]:
            monitor.record_pose(t, pose)
            monitor.record_selection(t, target)

        is_degenerate, repeat_count, displacement, reason = monitor.check_degenerate(0.4, target)
        self.assertTrue(is_degenerate)
        self.assertGreaterEqual(repeat_count, 3)
        self.assertLess(displacement, 0.20)
        self.assertEqual(reason, "degenerate_goal_reached_loop")

        monitor.start_cooldown(0.4, target, reason)
        self.assertTrue(monitor.is_cooled(0.4, target))
        self.assertTrue(monitor.is_cooled(0.4, (0.32, 0.02)))  # 同一区域内略有变化

    def test_slightly_different_coordinates_same_region(self):
        """同一区域内坐标略有变化的目标应被识别为同一区域。"""
        monitor = self._make_monitor()
        pose = (1.0, 2.0, 0.0)
        base_target = (1.35, 2.05)
        variants = [(1.35, 2.05), (1.38, 2.02), (1.32, 2.08), (1.36, 1.98)]
        for i, target in enumerate(variants):
            t = i * 0.1
            monitor.record_pose(t, pose)
            monitor.record_selection(t, target)

        is_degenerate, _, _, _ = monitor.check_degenerate(0.4, base_target)
        self.assertTrue(is_degenerate)

    def test_robot_moving_does_not_trigger(self):
        """机器人持续移动时不应误判为退化循环。"""
        monitor = self._make_monitor()
        target = (0.30, 0.0)
        poses = [(0.0, 0.0, 0.0), (0.15, 0.0, 0.0), (0.30, 0.0, 0.0), (0.45, 0.0, 0.0)]
        for i, pose in enumerate(poses):
            t = i * 0.1
            monitor.record_pose(t, pose)
            monitor.record_selection(t, target)

        is_degenerate, _, displacement, _ = monitor.check_degenerate(0.4, target)
        self.assertFalse(is_degenerate)
        self.assertGreaterEqual(displacement, 0.20)

    def test_normal_adjacent_frontiers_do_not_trigger(self):
        """正常连续到达多个相邻 frontier 时不应误判。"""
        monitor = self._make_monitor()
        pose = (0.0, 0.0, 0.0)
        targets = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        for i, target in enumerate(targets):
            t = i * 2.0
            # 机器人也同步移动，避免位移过小
            monitor.record_pose(t, (target[0] - 0.3, target[1], 0.0))
            monitor.record_selection(t, target)

        is_degenerate, _, _, _ = monitor.check_degenerate(4.0, targets[-1])
        self.assertFalse(is_degenerate)

    def test_cooldown_allows_other_regions(self):
        """cooldown 后可以选择其他区域的 frontier。"""
        monitor = self._make_monitor()
        now = 1.0
        cooled_target = (0.30, 0.0)
        monitor.start_cooldown(now, cooled_target, "test")

        frontiers = [
            _FakeFrontier((0.30, 0.0)),
            _FakeFrontier((5.0, 0.0)),
            _FakeFrontier((0.32, 0.02)),
        ]
        filtered = monitor.filter_frontiers(now, frontiers)
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0].centroid, (5.0, 0.0))

    def test_cooldown_expires_and_region_re_evaluated(self):
        """cooldown 到期后原区域可以重新评估。"""
        monitor = self._make_monitor(cooldown=1.0)
        target = (0.30, 0.0)
        monitor.start_cooldown(0.0, target, "test")
        self.assertTrue(monitor.is_cooled(0.0, target))
        self.assertTrue(monitor.is_cooled(0.5, target))
        self.assertFalse(monitor.is_cooled(1.1, target))

    def test_local_loop_does_not_directly_return_home(self):
        """局部循环检测只返回 cooldown 信号，不直接改变任务状态或触发返航。"""
        monitor = self._make_monitor()
        pose = (0.0, 0.0, 0.0)
        target = (0.30, 0.0)
        for i in range(5):
            t = i * 0.1
            monitor.record_pose(t, pose)
            monitor.record_selection(t, target)

        is_degenerate, _, _, reason = monitor.check_degenerate(0.4, target)
        self.assertTrue(is_degenerate)
        self.assertEqual(reason, "degenerate_goal_reached_loop")
        # ProgressMonitor 本身不应包含 RETURN_HOME 等状态切换逻辑
        self.assertFalse(hasattr(monitor, "state"))


if __name__ == "__main__":
    unittest.main()

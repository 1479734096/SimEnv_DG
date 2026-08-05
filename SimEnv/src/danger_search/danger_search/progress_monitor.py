"""前沿选择与位姿进度的诊断状态机。"""
from __future__ import annotations

from collections import deque
import math


class ProgressMonitor:
    """检测同一区域目标反复选中且机器人无实际位移的退化循环。

    该状态机只产生诊断和短期 cooldown，不修改地图、不切换任务状态。
    """

    def __init__(
        self,
        *,
        goal_tolerance: float,
        window: float,
        repeat: int,
        min_displacement: float,
        cooldown: float,
    ) -> None:
        self.region_radius = max(0.30, float(goal_tolerance) * 1.5)
        self.window = float(window)
        self.repeat = max(2, int(repeat))
        self.min_displacement = float(min_displacement)
        self.cooldown = float(cooldown)
        self._selections: deque[tuple[float, tuple[float, float]]] = deque()
        self._poses: deque[tuple[float, tuple[float, float, float]]] = deque()
        self._cooldowns: list[tuple[float, tuple[float, float], str]] = []
        self._loop_count = 0
        self._last_log_time = 0.0
        self._last_reason_code: str | None = None

    def record_pose(self, timestamp: float, pose: tuple[float, float, float]) -> None:
        self._poses.append((float(timestamp), pose))
        self._expire(timestamp)

    def record_selection(self, timestamp: float, target_xy: tuple[float, float]) -> None:
        self._selections.append((float(timestamp), target_xy))
        self._expire(timestamp)

    def check_degenerate(
        self,
        timestamp: float,
        target_xy: tuple[float, float],
    ) -> tuple[bool, int, float, str | None]:
        self._expire(timestamp)
        repeat_count = sum(
            1
            for selected_at, xy in self._selections
            if self._same_region(xy, target_xy)
            and (timestamp - selected_at) <= self.window
        )
        displacement = self._displacement(timestamp)
        if repeat_count >= self.repeat and displacement < self.min_displacement:
            self._loop_count += 1
            self._last_reason_code = "degenerate_goal_reached_loop"
            return True, repeat_count, displacement, self._last_reason_code
        self._loop_count = 0
        self._last_reason_code = None
        return False, repeat_count, displacement, None

    def start_cooldown(
        self,
        timestamp: float,
        center_xy: tuple[float, float],
        reason_code: str,
    ) -> None:
        self._cooldowns.append(
            (float(timestamp) + self.cooldown, center_xy, reason_code)
        )
        self._loop_count = 0

    def is_cooled(self, timestamp: float, target_xy: tuple[float, float]) -> bool:
        self._expire(timestamp)
        return any(
            self._same_region(center, target_xy)
            for _, center, _ in self._cooldowns
        )

    def filter_frontiers(self, timestamp: float, frontiers: list) -> list:
        self._expire(timestamp)
        return [
            frontier
            for frontier in frontiers
            if not self.is_cooled(timestamp, frontier.centroid)
        ]

    def should_log(self, timestamp: float, interval: float = 5.0) -> bool:
        if timestamp - self._last_log_time >= interval:
            self._last_log_time = timestamp
            return True
        return False

    def cooldown_reason(
        self,
        timestamp: float,
        target_xy: tuple[float, float],
    ) -> str | None:
        self._expire(timestamp)
        for _, center, reason in self._cooldowns:
            if self._same_region(center, target_xy):
                return reason
        return None

    def _same_region(
        self,
        a: tuple[float, float],
        b: tuple[float, float],
    ) -> bool:
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= self.region_radius

    def _displacement(self, timestamp: float) -> float:
        recent = [
            (sample_at, pose)
            for sample_at, pose in self._poses
            if (timestamp - sample_at) <= self.window
        ]
        if len(recent) < 2:
            return 0.0
        first = recent[0][1]
        last = recent[-1][1]
        return math.hypot(last[0] - first[0], last[1] - first[1])

    def _expire(self, timestamp: float) -> None:
        while self._selections and (timestamp - self._selections[0][0]) > self.window:
            self._selections.popleft()
        while self._poses and (timestamp - self._poses[0][0]) > self.window:
            self._poses.popleft()
        self._cooldowns = [
            (expires, center, reason)
            for expires, center, reason in self._cooldowns
            if expires > timestamp
        ]

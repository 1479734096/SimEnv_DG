"""探索前沿异步规划、动态提前量与实时路径复核。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from threading import Lock, Thread
import time

from danger_search.frontier import (
    Frontier,
    FrontierExplorer,
    FrontierSelectionContext,
    FrontierSelectionDebug,
)
from danger_search.mapper import FloorGrid, UNKNOWN
from danger_search.planner import PlanningConstraints, segment_is_traversable


MODE_NORMAL = "normal"
MODE_OPPORTUNISTIC = "opportunistic"
MODE_PREFETCH = "prefetch"
_VALID_MODES = frozenset({MODE_NORMAL, MODE_OPPORTUNISTIC, MODE_PREFETCH})


@dataclass(frozen=True)
class FrontierPlanScope:
    """一次后台结果能够生效的连续性作用域。"""

    request_id: int
    mode: str
    floor: int
    map_epoch: int
    localization_epoch: int
    constraint_revision: int
    path_sequence: int
    active_target: tuple[float, float] | None

    def __post_init__(self) -> None:
        if self.mode not in _VALID_MODES:
            raise ValueError("未知前沿规划模式")


@dataclass(frozen=True)
class FrontierPlanOutcome:
    status: str
    frontier: Frontier | None = None
    path: tuple[tuple[float, float], ...] = ()
    frontiers: tuple[Frontier, ...] = ()
    debug: FrontierSelectionDebug | None = None
    failure_deltas: tuple[tuple[tuple[int, int], int], ...] = ()


@dataclass(frozen=True)
class FrontierPlanPoll:
    status: str
    scope: FrontierPlanScope | None = None
    outcome: object | None = None
    duration: float | None = None
    error: str | None = None


class _AsyncFrontierPlanner:
    """在冻结输入上执行一次计算，结果只允许主循环轮询。"""

    def __init__(self, scope: FrontierPlanScope, compute):
        self.scope = scope
        self._compute = compute
        self.started_wall = time.monotonic()
        self._lock = Lock()
        self._done = False
        self._outcome = None
        self._error: str | None = None
        self._duration: float | None = None
        self._thread = Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def poll(self):
        with self._lock:
            if not self._done:
                return None
            return self._outcome, self._duration, self._error

    def _run(self) -> None:
        try:
            outcome = self._compute()
        except Exception as exc:  # pragma: no cover - 由运行时错误测试覆盖语义
            outcome = None
            error = repr(exc)
        else:
            error = None
        duration = max(0.0, time.monotonic() - self.started_wall)
        with self._lock:
            self._outcome = outcome
            self._duration = duration
            self._error = error
            self._done = True


class FrontierPlanningRuntime:
    """持有单个探索规划线程、耗时分位数和路线速度估计。"""

    def __init__(
        self,
        *,
        timeout: float,
        preplan_min_lead: float,
        preplan_max_lead: float,
        opportunistic_period: float = 3.0,
    ):
        if (
            not math.isfinite(float(opportunistic_period))
            or float(opportunistic_period) < 0.0
            or not all(
                math.isfinite(value) and value > 0.0
                for value in (float(timeout), float(preplan_min_lead), float(preplan_max_lead))
            )
            or preplan_min_lead > preplan_max_lead
        ):
            raise ValueError("异步前沿规划时间参数无效")
        self.timeout = float(timeout)
        self.preplan_min_lead = float(preplan_min_lead)
        self.preplan_max_lead = float(preplan_max_lead)
        self.opportunistic_period = float(opportunistic_period)
        self._planner: _AsyncFrontierPlanner | None = None
        self._durations: deque[float] = deque(maxlen=20)
        self._last_opportunistic_time = -math.inf
        self._last_route_remaining: float | None = None
        self._last_route_wall: float | None = None
        self._last_positive_progress_wall: float | None = None
        self._route_speed_ema: float | None = None

    @property
    def busy(self) -> bool:
        return self._planner is not None

    @property
    def active_scope(self) -> FrontierPlanScope | None:
        planner = self._planner
        return planner.scope if planner is not None else None

    def start(self, scope: FrontierPlanScope, *, compute) -> None:
        if self._planner is not None:
            raise RuntimeError("前沿规划线程已存在")
        planner = _AsyncFrontierPlanner(scope, compute)
        self._planner = planner
        planner.start()

    def poll(self, *, wall_time: float | None = None) -> FrontierPlanPoll:
        planner = self._planner
        if planner is None:
            return FrontierPlanPoll("idle")
        result = planner.poll()
        now = time.monotonic() if wall_time is None else float(wall_time)
        if result is None:
            status = (
                "timed_out"
                if now - planner.started_wall >= self.timeout
                else "pending"
            )
            return FrontierPlanPoll(status, scope=planner.scope)
        self._planner = None
        outcome, duration, error = result
        if (
            duration is not None
            and duration <= self.timeout
            and error is None
        ):
            self.record_duration(duration)
        return FrontierPlanPoll(
            (
                "error"
                if error is not None
                else ("late" if duration is not None and duration > self.timeout else "ready")
            ),
            scope=planner.scope,
            outcome=outcome,
            duration=duration,
            error=error,
        )

    def record_duration(self, duration: float) -> None:
        parsed = float(duration)
        if math.isfinite(parsed) and parsed >= 0.0:
            self._durations.append(parsed)

    def preplan_lead(self) -> float:
        if not self._durations:
            estimate = 1.0
        else:
            ordered = sorted(self._durations)
            index = max(0, int(math.ceil(0.90 * len(ordered))) - 1)
            estimate = ordered[index] + 0.25
        return min(
            self.preplan_max_lead,
            max(self.preplan_min_lead, estimate),
        )

    def observe_route_progress(
        self,
        remaining_distance: float | None,
        *,
        wall_time: float | None = None,
    ) -> None:
        now = time.monotonic() if wall_time is None else float(wall_time)
        if remaining_distance is None or not math.isfinite(float(remaining_distance)):
            self._last_route_remaining = None
            self._last_route_wall = now
            return
        remaining = max(0.0, float(remaining_distance))
        if self._last_route_remaining is not None and self._last_route_wall is not None:
            elapsed = now - self._last_route_wall
            progress = self._last_route_remaining - remaining
            if elapsed > 1e-6 and progress > 1e-4:
                speed = progress / elapsed
                self._route_speed_ema = (
                    speed
                    if self._route_speed_ema is None
                    else 0.3 * speed + 0.7 * self._route_speed_ema
                )
                self._last_positive_progress_wall = now
        self._last_route_remaining = remaining
        self._last_route_wall = now

    def reset_route_progress(self) -> None:
        """路径代次变化时清除旧路线速度，避免跨路径推导虚假 ETA。"""
        self._last_route_remaining = None
        self._last_route_wall = None
        self._last_positive_progress_wall = None
        self._route_speed_ema = None

    def route_eta(
        self,
        remaining_distance: float | None,
        *,
        wall_time: float | None = None,
    ) -> float | None:
        now = time.monotonic() if wall_time is None else float(wall_time)
        if (
            remaining_distance is None
            or self._route_speed_ema is None
            or self._last_positive_progress_wall is None
            or now - self._last_positive_progress_wall > 1.0
        ):
            return None
        speed = max(0.02, self._route_speed_ema)
        return max(0.0, float(remaining_distance)) / speed

    def mark_opportunistic_started(self, now: float) -> None:
        self._last_opportunistic_time = float(now)

    def opportunistic_due(self, now: float, *, route_eta: float | None) -> bool:
        if self.opportunistic_period <= 0.0:
            return False
        if float(now) - self._last_opportunistic_time < self.opportunistic_period:
            return False
        if route_eta is not None and route_eta <= 2.0 * self.preplan_lead():
            return False
        return True


def compute_frontier_plan(
    explorer: FrontierExplorer,
    grid: FloorGrid,
    pose: tuple[float, float, float],
    frontiers: tuple[Frontier, ...],
    *,
    selection_context: FrontierSelectionContext | None,
    planning_constraints: PlanningConstraints | None,
) -> FrontierPlanOutcome:
    """在 Explorer 与地图副本上完成一次有限规划及预算未决确认。"""
    before_failures = dict(explorer._fail_counts)
    frontier, path = explorer.select_frontier(
        grid,
        pose,
        selection_context=selection_context,
        frontiers=list(frontiers),
        planning_constraints=planning_constraints,
    )
    debug = explorer.last_selection_debug
    if (
        (frontier is None or not path)
        and debug is not None
        and debug.budget_exhausted_count > 0
    ):
        frontier, path = explorer.confirm_budget_exhausted_frontier(
            grid,
            pose,
            selection_context=selection_context,
            planning_constraints=planning_constraints,
        )
    deltas = tuple(
        sorted(
            (
                cell,
                count - before_failures.get(cell, 0),
            )
            for cell, count in explorer._fail_counts.items()
            if count > before_failures.get(cell, 0)
        )
    )
    return FrontierPlanOutcome(
        status="selected" if frontier is not None and path else "no_target",
        frontier=frontier,
        path=tuple((float(point[0]), float(point[1])) for point in path),
        frontiers=tuple(frontiers),
        debug=debug,
        failure_deltas=deltas,
    )


def frontier_still_relevant(grid: FloorGrid, frontier: Frontier) -> bool:
    """确认快照中的真实前沿在实时地图上仍邻接未知区域。"""
    if frontier.information_gain <= 0.0:
        return False
    for gx, gy in frontier.cells:
        if not (0 <= gx < grid.size_x and 0 <= gy < grid.size_y):
            continue
        for ny in range(max(0, gy - 1), min(grid.size_y, gy + 2)):
            for nx in range(max(0, gx - 1), min(grid.size_x, gx + 2)):
                if int(grid.data[ny, nx]) == UNKNOWN:
                    return True
    return False


def live_frontier_failure_deltas(
    grid: FloorGrid,
    deltas: tuple[tuple[tuple[int, int], int], ...],
) -> tuple[tuple[tuple[int, int], int], ...]:
    """只保留实时地图上仍邻接 UNKNOWN 的失败格，避免合并过期失败。"""
    accepted = []
    for (gx, gy), increment in deltas:
        if not (0 <= gx < grid.size_x and 0 <= gy < grid.size_y):
            continue
        still_frontier = any(
            int(grid.data[ny, nx]) == UNKNOWN
            for ny in range(max(0, gy - 1), min(grid.size_y, gy + 2))
            for nx in range(max(0, gx - 1), min(grid.size_x, gx + 2))
        )
        if still_frontier:
            accepted.append(((int(gx), int(gy)), int(increment)))
    return tuple(accepted)


def rebase_and_validate_frontier_path(
    grid: FloorGrid,
    pose: tuple[float, float, float],
    path: tuple[tuple[float, float], ...],
    *,
    planning_constraints: PlanningConstraints | None,
    maximum_rebase_distance: float = 0.75,
) -> list[tuple[float, float]] | None:
    """把快照路径接到实时位姿，并在实时地图上复核全部线段。"""
    if len(path) < 2:
        return None
    current = (float(pose[0]), float(pose[1]))
    nearest_index = min(
        range(len(path)),
        key=lambda index: math.hypot(
            path[index][0] - current[0],
            path[index][1] - current[1],
        ),
    )
    nearest_distance = math.hypot(
        path[nearest_index][0] - current[0],
        path[nearest_index][1] - current[1],
    )
    if nearest_distance > float(maximum_rebase_distance):
        return None
    suffix = list(path[nearest_index + 1 :])
    if not suffix:
        return None
    candidate = [current, *suffix]
    if not all(
        segment_is_traversable(
            grid,
            start,
            end,
            constraints=planning_constraints,
        )
        for start, end in zip(candidate, candidate[1:])
    ):
        return None
    return candidate

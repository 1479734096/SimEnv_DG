"""探索预算与无前沿恢复状态机，不负责楼层切换等顶层任务决策。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from danger_search.mapper import BLACK, FREE, NEGATIVE, OCC, UNKNOWN, VISITED, FloorGrid
from danger_search.planner import astar_plan, path_length


@dataclass(frozen=True)
class ExplorationBudgetInput:
    home_recorded: bool
    mission_elapsed: float
    floor_elapsed: float
    max_runtime: float
    return_time: float
    return_safety_buffer: float
    floor_timeout: float
    floor_finish_time_ratio: float
    known_cells_growth: float
    exploration_progress_window: float
    minimum_progress_cells: int
    frontier_count: int
    minimum_remaining_frontiers: int
    current_floor: int
    maximum_floor: int
    next_floor_change_time: float
    last_floor_uses_global_budget: bool


@dataclass(frozen=True)
class ExplorationBudgetDecision:
    action: str
    reason_code: str


@dataclass(frozen=True)
class FrontierArrivalGainResult:
    """一次真实前沿到达后的实际地图收益及区域冷却决定。"""

    floor: int
    map_epoch: int
    target: tuple[float, float]
    expected_unknown_gain: float
    actual_known_gain: int
    low_gain_count: int
    cooldown_center: tuple[float, float] | None

    def diagnostic(self) -> dict[str, object]:
        return {
            "event": "frontier_arrival_gain",
            "floor": self.floor,
            "map_epoch": self.map_epoch,
            "target": [round(value, 3) for value in self.target],
            "expected_unknown_gain": round(self.expected_unknown_gain, 3),
            "actual_known_gain": self.actual_known_gain,
            "low_gain_count": self.low_gain_count,
            "cooled": self.cooldown_center is not None,
            "cooldown_center": (
                [round(value, 3) for value in self.cooldown_center]
                if self.cooldown_center is not None
                else None
            ),
        }


@dataclass
class _FrontierGainRegion:
    center: tuple[float, float]
    low_gain_count: int = 0


class FrontierArrivalGainRuntime:
    """按楼层区域记录成功但低收益的真实前沿到达。"""

    def __init__(
        self,
        *,
        region_radius: float = 1.0,
        minimum_known_gain: int = 30,
        low_gain_limit: int = 2,
    ) -> None:
        self.region_radius = max(0.0, float(region_radius))
        self.minimum_known_gain = max(0, int(minimum_known_gain))
        self.low_gain_limit = max(1, int(low_gain_limit))
        self._context: tuple[int, int] | None = None
        self._active: dict[str, object] | None = None
        self._regions: list[_FrontierGainRegion] = []

    def begin(
        self,
        *,
        floor: int,
        map_epoch: int,
        target: tuple[float, float],
        known_cells: int,
        expected_gain: float,
        is_real_frontier: bool,
    ) -> None:
        self._sync_context(floor, map_epoch)
        if not is_real_frontier:
            self._active = None
            return
        self._active = {
            "floor": int(floor),
            "map_epoch": int(map_epoch),
            "target": (float(target[0]), float(target[1])),
            "known_cells": int(known_cells),
            "expected_gain": max(0.0, float(expected_gain)),
        }

    def cancel(self) -> None:
        self._active = None

    def reset(self) -> None:
        """清除当前楼层的活动目标和区域收益账本。"""
        self._context = None
        self._active = None
        self._regions = []

    def arrive(
        self,
        *,
        floor: int,
        map_epoch: int,
        known_cells: int,
    ) -> FrontierArrivalGainResult | None:
        context = (int(floor), int(map_epoch))
        if context != self._context:
            self._sync_context(*context)
            return None
        active = self._active
        self._active = None
        if active is None:
            return None

        target = active["target"]
        assert isinstance(target, tuple)
        region = self._region_for(target)
        actual_gain = max(0, int(known_cells) - int(active["known_cells"]))
        if region is None:
            region = _FrontierGainRegion(center=target)
            self._regions.append(region)
        else:
            region.center = target

        cooldown_center = None
        if actual_gain < self.minimum_known_gain:
            region.low_gain_count += 1
            low_gain_count = region.low_gain_count
            if low_gain_count >= self.low_gain_limit:
                cooldown_center = target
                # 冷却到期后允许该区域重新积累证据，避免永久封禁。
                region.low_gain_count = 0
        else:
            region.low_gain_count = 0
            low_gain_count = 0

        return FrontierArrivalGainResult(
            floor=int(active["floor"]),
            map_epoch=int(active["map_epoch"]),
            target=target,
            expected_unknown_gain=float(active["expected_gain"]),
            actual_known_gain=actual_gain,
            low_gain_count=low_gain_count,
            cooldown_center=cooldown_center,
        )

    def _sync_context(self, floor: int, map_epoch: int) -> None:
        context = (int(floor), int(map_epoch))
        if context == self._context:
            return
        self._context = context
        self._active = None
        self._regions = []

    def _region_for(
        self,
        target: tuple[float, float],
    ) -> _FrontierGainRegion | None:
        return next(
            (
                region
                for region in self._regions
                if math.hypot(
                    region.center[0] - target[0],
                    region.center[1] - target[1],
                )
                <= self.region_radius
            ),
            None,
        )


def decide_exploration_budget(
    facts: ExplorationBudgetInput,
) -> ExplorationBudgetDecision:
    """依据冻结事实决定继续探索、结束本层或返航。"""
    if not facts.home_recorded:
        return ExplorationBudgetDecision("CONTINUE", "home_pose_not_recorded")

    remaining = facts.max_runtime - facts.mission_elapsed
    if (
        facts.mission_elapsed
        + facts.return_time
        + facts.return_safety_buffer
        >= facts.max_runtime
    ):
        return ExplorationBudgetDecision(
            "RETURN_HOME", "mission_insufficient_time_for_return"
        )

    has_next_floor = facts.current_floor + 1 < facts.maximum_floor
    if facts.floor_elapsed >= facts.floor_timeout:
        if has_next_floor:
            if (
                facts.next_floor_change_time
                + facts.return_time
                + facts.return_safety_buffer
                < remaining
            ):
                return ExplorationBudgetDecision(
                    "FINISH_FLOOR", "floor_timeout_change_floor"
                )
            return ExplorationBudgetDecision(
                "RETURN_HOME", "floor_timeout_return_home"
            )
        if not facts.last_floor_uses_global_budget:
            return ExplorationBudgetDecision(
                "RETURN_HOME", "floor_timeout_return_home"
            )
        known_growth = (
            facts.known_cells_growth * facts.exploration_progress_window
        )
        if known_growth < facts.minimum_progress_cells:
            return ExplorationBudgetDecision(
                "RETURN_HOME", "floor_low_progress_return_home"
            )

    if (
        facts.floor_elapsed
        >= facts.floor_timeout * facts.floor_finish_time_ratio
    ):
        known_growth = (
            facts.known_cells_growth * facts.exploration_progress_window
        )
        has_progress = known_growth >= facts.minimum_progress_cells
        has_frontiers = (
            facts.frontier_count >= facts.minimum_remaining_frontiers
        )
        if not has_progress and not has_frontiers:
            if has_next_floor and (
                facts.next_floor_change_time
                + facts.return_time
                + facts.return_safety_buffer
                < remaining
            ):
                return ExplorationBudgetDecision(
                    "FINISH_FLOOR", "floor_low_progress_change_floor"
                )
            return ExplorationBudgetDecision(
                "RETURN_HOME", "floor_low_progress_return_home"
            )

    if facts.floor_elapsed >= facts.floor_timeout and not has_next_floor:
        return ExplorationBudgetDecision(
            "CONTINUE", "last_floor_uses_global_budget"
        )
    return ExplorationBudgetDecision("CONTINUE", "budget_allows_continue")


@dataclass(frozen=True)
class NoFrontierRecoveryConfig:
    scan_duration: float
    rotation_min_clearance: float
    retreat_clearance_margin: float
    retreat_max_distance: float


@dataclass(frozen=True)
class ExplorationRuntimeResult:
    completed: bool = False
    failure_reason: str | None = None
    info_message: tuple[object, ...] | None = None
    warning_message: tuple[object, ...] | None = None
    structured_events: tuple[dict[str, object], ...] = ()


def grid_cell_has_clearance(
    grid: FloorGrid,
    cell: tuple[int, int],
    required_clearance: float,
) -> bool:
    """保守检查目标格周围旋转包络，地图边界同样视为不可用。"""
    cx, cy = cell
    cells = int(math.ceil(required_clearance / grid.resolution))
    for gy in range(cy - cells, cy + cells + 1):
        for gx in range(cx - cells, cx + cells + 1):
            distance = math.hypot(gx - cx, gy - cy) * grid.resolution
            if distance > required_clearance:
                continue
            if not (0 <= gx < grid.size_x and 0 <= gy < grid.size_y):
                return False
            if int(grid.data[gy, gx]) in (OCC, BLACK, NEGATIVE):
                return False
    return True


def initial_occ_escape_segment_is_safe(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    target_xy: tuple[float, float],
    *,
    maximum_initial_occ_distance: float,
) -> bool:
    """只豁免起点附近连续 OCC 膨胀带，脱离后不允许再次进入障碍。"""
    distance = math.hypot(
        target_xy[0] - start_xy[0],
        target_xy[1] - start_xy[1],
    )
    if distance <= 1e-9:
        return False
    step_count = max(
        1,
        int(math.ceil(distance / max(0.5 * grid.resolution, 1e-6))),
    )
    reached_traversable = False
    for index in range(1, step_count + 1):
        progress = distance * index / step_count
        ratio = index / step_count
        cell = grid.world_to_grid(
            start_xy[0] + (target_xy[0] - start_xy[0]) * ratio,
            start_xy[1] + (target_xy[1] - start_xy[1]) * ratio,
        )
        if cell is None:
            return False
        state = int(grid.data[cell[1], cell[0]])
        if state in (BLACK, NEGATIVE, UNKNOWN):
            return False
        if state == OCC:
            if reached_traversable or progress > maximum_initial_occ_distance:
                return False
            continue
        if state not in (FREE, VISITED):
            return False
        reached_traversable = True
    return reached_traversable


@dataclass
class ExplorationRuntime:
    no_frontier_scan_until: float = 0.0
    no_reachable_frontier_confirmations: int = 0
    last_no_frontier_signature: object | None = None
    no_frontier_escape_state: str = "idle"
    no_frontier_escape_target: tuple[float, float] | None = None
    no_frontier_escape_deadline: float = 0.0
    no_frontier_escape_start_time: float = 0.0
    no_frontier_escape_start_pose: tuple[float, float, float] | None = None
    floor_timeout_logged: bool = False
    probe_attempt: int = 0
    last_frontier_target: tuple[float, float] | None = None
    last_frontier_score: float | None = None
    last_frontier_used_fallback: bool = False
    progress_history: list[dict[str, object]] = field(default_factory=list)
    last_budget_decision: tuple[str, str] | None = None
    last_budget_decision_time: float = 0.0
    no_frontier_rotation_blocked_logged: bool = False
    post_door_frontier_debug: dict[str, object] | None = None

    def reset_no_frontier_escape(self) -> None:
        self.no_frontier_escape_state = "idle"
        self.no_frontier_escape_target = None
        self.no_frontier_escape_deadline = 0.0
        self.no_frontier_escape_start_time = 0.0
        self.no_frontier_escape_start_pose = None

    @staticmethod
    def rotation_clearance_is_safe(
        clearance: float | None,
        required_clearance: float,
    ) -> bool:
        return (
            clearance is not None
            and math.isfinite(float(clearance))
            and float(clearance) >= float(required_clearance)
        )

    def tick_no_frontier_scan(
        self,
        *,
        now: float,
        frontiers_present: bool,
        pose: tuple[float, float, float] | None,
        grid: FloorGrid,
        navigation_trace: list[tuple[float, float]],
        controller: object,
        radial_clearance: float | None,
        current_floor: int,
        config: NoFrontierRecoveryConfig,
        record_failure: bool = True,
        planning_constraints=None,
    ) -> ExplorationRuntimeResult:
        """推进一次无前沿扫描；楼层结束由调用方根据 completed 决定。"""
        escape_state = self.no_frontier_escape_state
        clearance_safe = self.rotation_clearance_is_safe(
            radial_clearance,
            config.rotation_min_clearance,
        )
        if self.no_frontier_scan_until <= 0.0 and not clearance_safe:
            if escape_state == "idle" and pose is not None:
                return self._start_no_frontier_retreat(
                    now=now,
                    pose=pose,
                    grid=grid,
                    navigation_trace=navigation_trace,
                    controller=controller,
                    current_floor=current_floor,
                    config=config,
                    planning_constraints=planning_constraints,
                )
            if escape_state == "scan_once":
                self.no_frontier_escape_state = "exhausted"
                controller.stop(reason="no_frontier_escape_scan_blocked")
                event = self._rotation_blocked_event(
                    source="no_frontier_escape_scan",
                    current_floor=current_floor,
                    radial_clearance=radial_clearance,
                    required_clearance=config.rotation_min_clearance,
                )
                return ExplorationRuntimeResult(
                    completed=True,
                    structured_events=(event,) if event is not None else (),
                )
            if escape_state == "exhausted":
                return ExplorationRuntimeResult(completed=True)

        info_message = None
        if self.no_frontier_scan_until <= 0.0:
            self.no_frontier_scan_until = now + config.scan_duration
            info_message = (
                "当前无可达前沿，原地扫描 %.1f 秒后再判断"
                if frontiers_present
                else "当前无前沿，原地扫描 %.1f 秒后再判断",
                config.scan_duration,
            )
        if now < self.no_frontier_scan_until:
            event = self.rotate_for_no_frontier_scan(
                controller=controller,
                radial_clearance=radial_clearance,
                current_floor=current_floor,
                required_clearance=config.rotation_min_clearance,
            )
            return ExplorationRuntimeResult(
                info_message=info_message,
                structured_events=(event,) if event is not None else (),
            )

        controller.stop()
        self.no_frontier_scan_until = 0.0
        self.no_frontier_rotation_blocked_logged = False
        if escape_state == "scan_once":
            self.no_frontier_escape_state = "exhausted"
        return ExplorationRuntimeResult(
            completed=True,
            failure_reason=(
                "无可达前沿" if frontiers_present and record_failure else None
            ),
            warning_message=(
                ("扫描后仍无可达前沿，结束当前楼层",)
                if frontiers_present
                else None
            ),
        )

    def _start_no_frontier_retreat(
        self,
        *,
        now: float,
        pose: tuple[float, float, float],
        grid: FloorGrid,
        navigation_trace: list[tuple[float, float]],
        controller: object,
        current_floor: int,
        config: NoFrontierRecoveryConfig,
        planning_constraints=None,
    ) -> ExplorationRuntimeResult:
        """沿本层已走轨迹选择一次有界直线退避。"""
        start = grid.world_to_grid(float(pose[0]), float(pose[1]))
        if start is None or not grid.is_traversable(*start):
            return self._exhaust_no_frontier_escape(
                controller=controller,
                current_floor=current_floor,
                reason="start_not_traversable",
            )

        min_distance = max(0.5, 2.0 * float(grid.resolution))
        required_clearance = (
            config.rotation_min_clearance + config.retreat_clearance_margin
        )
        selected_path: list[tuple[float, float]] | None = None
        selected_target: tuple[float, float] | None = None
        for trace_point in reversed(navigation_trace):
            target = (float(trace_point[0]), float(trace_point[1]))
            distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
            if distance < min_distance or distance > config.retreat_max_distance:
                continue
            target_cell = grid.world_to_grid(*target)
            if target_cell is None or not grid.is_traversable(*target_cell):
                continue
            if not grid_cell_has_clearance(grid, target_cell, required_clearance):
                continue
            plan = astar_plan(
                grid,
                (pose[0], pose[1]),
                target,
                constraints=planning_constraints,
            )
            if not plan.reached or len(plan.path) != 2:
                continue
            target_direction = math.atan2(target[1] - pose[1], target[0] - pose[0])
            backward_error = math.atan2(
                math.sin(target_direction - pose[2] - math.pi),
                math.cos(target_direction - pose[2] - math.pi),
            )
            if abs(backward_error) > 0.35:
                continue
            selected_path = plan.path
            selected_target = target
            break

        if selected_path is None or selected_target is None:
            return self._exhaust_no_frontier_escape(
                controller=controller,
                current_floor=current_floor,
                reason="no_safe_trace_candidate",
            )

        self.no_frontier_escape_state = "moving"
        self.no_frontier_escape_target = selected_target
        self.no_frontier_escape_start_time = now
        self.no_frontier_escape_start_pose = (
            float(pose[0]),
            float(pose[1]),
            float(pose[2]),
        )
        reverse_speed = abs(float(getattr(controller, "reverse_speed", -0.35)))
        retreat_distance = math.hypot(
            selected_target[0] - pose[0], selected_target[1] - pose[1]
        )
        self.no_frontier_escape_deadline = now + max(
            3.0, retreat_distance / max(0.05, reverse_speed) + 1.5
        )
        self.no_frontier_scan_until = 0.0
        self.last_no_frontier_signature = None
        return ExplorationRuntimeResult(
            structured_events=(
                {
                    "event": "no_frontier_retreat_started",
                    "current_floor": int(current_floor),
                    "target": [round(value, 3) for value in selected_target],
                    "path_length": round(path_length(selected_path), 3),
                    "required_clearance": round(required_clearance, 3),
                },
            )
        )

    def tick_no_frontier_escape_motion(
        self,
        *,
        now: float,
        pose: tuple[float, float, float],
        controller: object,
        current_floor: int,
    ) -> ExplorationRuntimeResult:
        """沿已验证直线低速倒退，不在贴墙位置掉头。"""
        target = self.no_frontier_escape_target
        if target is None:
            return self._exhaust_no_frontier_escape(
                controller=controller,
                current_floor=current_floor,
                reason="target_missing",
            )
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        if distance <= 0.25:
            self.no_frontier_escape_state = "scan_once"
            self.no_frontier_escape_target = None
            self.no_frontier_escape_deadline = 0.0
            self.no_frontier_escape_start_time = 0.0
            self.no_frontier_escape_start_pose = None
            self.no_frontier_scan_until = 0.0
            self.last_no_frontier_signature = None
            controller.stop(reason="no_frontier_retreat_completed")
            return ExplorationRuntimeResult(
                structured_events=(
                    {
                        "event": "no_frontier_retreat_completed",
                        "current_floor": int(current_floor),
                        "target": list(target),
                    },
                )
            )
        if (
            self.no_frontier_escape_start_pose is not None
            and now - self.no_frontier_escape_start_time >= 2.5
        ):
            moved = math.hypot(
                pose[0] - self.no_frontier_escape_start_pose[0],
                pose[1] - self.no_frontier_escape_start_pose[1],
            )
            if moved < 0.05:
                return self._exhaust_no_frontier_escape(
                    controller=controller,
                    current_floor=current_floor,
                    reason="retreat_stalled",
                )
        if now >= self.no_frontier_escape_deadline:
            return self._exhaust_no_frontier_escape(
                controller=controller,
                current_floor=current_floor,
                reason="retreat_timeout",
            )
        target_direction = math.atan2(target[1] - pose[1], target[0] - pose[0])
        backward_error = math.atan2(
            math.sin(target_direction - pose[2] - math.pi),
            math.cos(target_direction - pose[2] - math.pi),
        )
        if abs(backward_error) > 0.45:
            return self._exhaust_no_frontier_escape(
                controller=controller,
                current_floor=current_floor,
                reason="reverse_heading_diverged",
            )
        if not controller.reverse_straight(reason="no_frontier_retreat"):
            return self._exhaust_no_frontier_escape(
                controller=controller,
                current_floor=current_floor,
                reason="reverse_command_rejected",
            )
        return ExplorationRuntimeResult()

    def rotate_for_no_frontier_scan(
        self,
        *,
        controller: object,
        radial_clearance: float | None,
        current_floor: int,
        required_clearance: float,
    ) -> dict[str, object] | None:
        if self.rotation_clearance_is_safe(radial_clearance, required_clearance):
            self.no_frontier_rotation_blocked_logged = False
            controller.rotate_in_place(0.45)
            return None
        controller.stop(reason="unsafe_no_frontier_rotation_blocked")
        return self._rotation_blocked_event(
            source="no_frontier_scan",
            current_floor=current_floor,
            radial_clearance=radial_clearance,
            required_clearance=required_clearance,
        )

    def _rotation_blocked_event(
        self,
        *,
        source: str,
        current_floor: int,
        radial_clearance: float | None,
        required_clearance: float,
    ) -> dict[str, object] | None:
        if self.no_frontier_rotation_blocked_logged:
            return None
        self.no_frontier_rotation_blocked_logged = True
        return {
            "event": "unsafe_rotation_blocked",
            "source": str(source),
            "current_floor": int(current_floor),
            "clearance": (
                round(float(radial_clearance), 4)
                if radial_clearance is not None
                and math.isfinite(float(radial_clearance))
                else None
            ),
            "required_clearance": round(float(required_clearance), 4),
        }

    def _exhaust_no_frontier_escape(
        self,
        *,
        controller: object,
        current_floor: int,
        reason: str,
    ) -> ExplorationRuntimeResult:
        self.no_frontier_escape_state = "exhausted"
        self.no_frontier_escape_target = None
        self.no_frontier_escape_deadline = 0.0
        controller.stop(reason="no_frontier_retreat_unavailable")
        return ExplorationRuntimeResult(
            completed=True,
            structured_events=(
                {
                    "event": "no_frontier_retreat_unavailable",
                    "current_floor": int(current_floor),
                    "reason": str(reason),
                },
            )
        )

    def record_progress(
        self,
        *,
        now: float,
        known_cells: int,
        free_or_visited_cells: int,
        confirmed_dangers: int,
        retention_seconds: float,
    ) -> None:
        self.progress_history.append(
            {
                "time": float(now),
                "known_cells": int(known_cells),
                "free_or_visited_cells": int(free_or_visited_cells),
                "confirmed_dangers": int(confirmed_dangers),
            }
        )
        cutoff = float(now) - max(0.0, float(retention_seconds))
        while self.progress_history and float(self.progress_history[0]["time"]) < cutoff:
            self.progress_history.pop(0)

    def recent_progress(self, *, now: float, window_seconds: float) -> dict[str, float]:
        cutoff = float(now) - max(0.0, float(window_seconds))
        samples = [
            sample
            for sample in self.progress_history
            if float(sample["time"]) >= cutoff
        ]
        if len(samples) < 2:
            return {
                "known_cells_growth": 0.0,
                "free_or_visited_growth": 0.0,
                "danger_growth": 0.0,
            }
        first, last = samples[0], samples[-1]
        elapsed = max(1e-6, float(last["time"]) - float(first["time"]))
        return {
            "known_cells_growth": (
                int(last["known_cells"]) - int(first["known_cells"])
            ) / elapsed,
            "free_or_visited_growth": (
                int(last["free_or_visited_cells"])
                - int(first["free_or_visited_cells"])
            ) / elapsed,
            "danger_growth": (
                int(last["confirmed_dangers"]) - int(first["confirmed_dangers"])
            ) / elapsed,
        }


def compute_danger_approach_pose(
    grid: FloorGrid,
    robot_pose: tuple[float, float, float],
    sphere_xy: tuple[float, float],
    standoff_distance: float = 1.3,
) -> tuple[float, float, float] | None:
    """计算前往远距离危险源近场核验的合法安全停靠位姿 (x, y, yaw)。"""
    rx, ry, _ = robot_pose
    sx, sy = sphere_xy
    dx = sx - rx
    dy = sy - ry
    dist = math.hypot(dx, dy)
    if dist <= standoff_distance or not math.isfinite(dist) or dist <= 1e-3:
        return None

    # 沿机器人到红球方向，在距红球 standoff_distance 处停机
    ratio = (dist - standoff_distance) / dist
    target_x = rx + dx * ratio
    target_y = ry + dy * ratio
    target_yaw = math.atan2(dy, dx)

    # 校验栅格地图合法性与可通行性
    target_cell = grid.world_to_grid(target_x, target_y)
    if target_cell is None:
        return None
    cell_val = grid.data[target_cell[1], target_cell[0]]
    if cell_val not in (FREE, VISITED):
        best_pos = None
        min_d = float("inf")
        for ox in (-0.2, 0.0, 0.2):
            for oy in (-0.2, 0.0, 0.2):
                cx, cy = target_x + ox, target_y + oy
                c_cell = grid.world_to_grid(cx, cy)
                if c_cell is not None and grid.data[c_cell[1], c_cell[0]] in (FREE, VISITED):
                    d_cur = math.hypot(cx - rx, cy - ry)
                    if d_cur < min_d:
                        min_d = d_cur
                        best_pos = (cx, cy)
        if best_pos is None:
            return None
        target_x, target_y = best_pos

    return (float(target_x), float(target_y), float(target_yaw))

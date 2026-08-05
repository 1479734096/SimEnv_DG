"""返航规划与卡滞恢复运行时，不负责控制发布或顶层状态转换。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from danger_search.exploration_runtime import grid_cell_has_clearance
from danger_search.planner import astar_plan, path_length
from danger_search.return_home_planner import (
    AsyncReturnPlanner,
    compute_return_home_plan,
    copy_floor_grid,
    return_segment_signature,
)


@dataclass(frozen=True)
class ReturnHomeStuckFeedback:
    stuck_count: int
    failed_segment: tuple[float, float, float, float] | None
    obstacle_payload: dict[str, object] | None


@dataclass(frozen=True)
class ReturnHomeRetreatSelection:
    target: tuple[float, float]
    path: tuple[tuple[float, float], ...]
    required_clearance: float

    @property
    def path_length(self) -> float:
        return float(path_length(list(self.path)))


@dataclass(frozen=True)
class ReturnHomeRetreatStep:
    action: str
    target: tuple[float, float] | None


@dataclass(frozen=True)
class ReturnHomePlanPoll:
    status: str
    path: tuple[tuple[float, float], ...] = ()
    planner_error: str | None = None


@dataclass
class ReturnHomeRuntime:
    entry_trace_attempted: bool = False
    next_plan_time: float = 0.0
    planner: object | None = None
    reason: str | None = None
    stuck_anchor: tuple[float, float] | None = None
    stuck_count: int = 0
    failed_segments: list[tuple[float, float, float, float]] = field(
        default_factory=list
    )
    last_failed_signature: tuple[int, int] | None = None
    candidate_reject_count: int = 0
    last_stuck_obstacle: dict[str, object] | None = None
    retreat_target: tuple[float, float] | None = None
    retreat_deadline: float = 0.0
    retreat_anchor: tuple[float, float] | None = None

    def reset_feedback(self) -> None:
        self.stuck_anchor = None
        self.stuck_count = 0
        self.failed_segments.clear()
        self.last_failed_signature = None
        self.candidate_reject_count = 0
        self.last_stuck_obstacle = None
        self.retreat_target = None
        self.retreat_deadline = 0.0
        self.retreat_anchor = None

    def clear_stuck_if_departed(
        self,
        pose: tuple[float, float, float],
        *,
        distance: float = 1.0,
    ) -> None:
        if self.stuck_anchor is None:
            return
        if math.hypot(
            float(pose[0]) - self.stuck_anchor[0],
            float(pose[1]) - self.stuck_anchor[1],
        ) <= float(distance):
            return
        # 失败首段继续保留，避免返航后段重新选择同一不可执行方向。
        self.stuck_anchor = None
        self.stuck_count = 0
        self.candidate_reject_count = 0

    def start_plan(
        self,
        *,
        floor: int,
        goal_xy: tuple[float, float],
        grid,
        pose: tuple[float, float, float],
        navigation_trace,
        visited_radius: float,
        veto_lookahead: float,
    ) -> None:
        if self.planner is not None:
            raise RuntimeError("返航规划线程已存在")
        grid_snapshot = copy_floor_grid(grid)
        trace_snapshot = tuple(
            tuple(float(value) for value in point)
            for point in navigation_trace
        )
        veto_segments = tuple(
            tuple(float(value) for value in segment)
            for segment in self.failed_segments
        )
        failed_signature = self.last_failed_signature
        start_pose = tuple(float(value) for value in pose)
        frozen_goal = (float(goal_xy[0]), float(goal_xy[1]))
        planner = AsyncReturnPlanner(
            floor=int(floor),
            goal_xy=frozen_goal,
            compute=lambda: compute_return_home_plan(
                grid_snapshot,
                start_pose,
                frozen_goal,
                trace_snapshot=trace_snapshot,
                visited_radius=float(visited_radius),
                veto_segments=veto_segments,
                failed_signature=failed_signature,
                veto_lookahead=float(veto_lookahead),
            ),
        )
        self.planner = planner
        planner.start()

    def poll_plan(
        self,
        *,
        current_floor: int,
        goal_xy: tuple[float, float],
    ) -> ReturnHomePlanPoll:
        planner = self.planner
        if planner is None:
            return ReturnHomePlanPoll("idle")
        outcome = planner.poll()
        if outcome is None:
            return ReturnHomePlanPoll("pending")
        self.planner = None
        frozen_goal = (float(goal_xy[0]), float(goal_xy[1]))
        if planner.floor != int(current_floor) or planner.goal_xy != frozen_goal:
            return ReturnHomePlanPoll("stale")
        plan_status, path = outcome
        if plan_status == "error":
            return ReturnHomePlanPoll(
                "error",
                planner_error=planner.error,
            )
        return ReturnHomePlanPoll(
            str(plan_status),
            tuple((float(point[0]), float(point[1])) for point in path),
        )

    def record_stuck(
        self,
        pose: tuple[float, float, float],
        segment: tuple[tuple[float, float], tuple[float, float]] | None,
        obstacle,
    ) -> ReturnHomeStuckFeedback:
        if self.stuck_anchor is None or math.hypot(
            float(pose[0]) - self.stuck_anchor[0],
            float(pose[1]) - self.stuck_anchor[1],
        ) > 0.75:
            self.stuck_anchor = (float(pose[0]), float(pose[1]))
            self.stuck_count = 1
        else:
            self.stuck_count += 1

        failed_segment = None
        if segment is not None:
            _segment_start, segment_end = segment
            failed_segment = (
                float(pose[0]),
                float(pose[1]),
                float(segment_end[0]),
                float(segment_end[1]),
            )
            if not self.failed_segments or any(
                abs(a - b) > 1e-6
                for a, b in zip(self.failed_segments[-1], failed_segment)
            ):
                self.failed_segments.append(failed_segment)
                del self.failed_segments[:-4]
            self.last_failed_signature = return_segment_signature(segment_end)

        obstacle_payload = None
        if obstacle is not None:
            obstacle_payload = {
                "distance": round(float(obstacle.distance), 4),
                "world_xyz": [
                    round(float(value), 4) for value in obstacle.world_xyz
                ],
                "forward": round(float(obstacle.forward), 4),
                "lateral": round(float(obstacle.lateral), 4),
                "relative_height": round(float(obstacle.relative_height), 4),
            }
        self.last_stuck_obstacle = obstacle_payload
        return ReturnHomeStuckFeedback(
            stuck_count=int(self.stuck_count),
            failed_segment=failed_segment,
            obstacle_payload=obstacle_payload,
        )

    def select_retreat(
        self,
        *,
        pose: tuple[float, float, float],
        grid,
        navigation_trace,
        rotation_min_clearance: float,
        clearance_margin: float,
        maximum_distance: float,
    ) -> ReturnHomeRetreatSelection | None:
        if self.retreat_anchor is not None and math.hypot(
            float(pose[0]) - self.retreat_anchor[0],
            float(pose[1]) - self.retreat_anchor[1],
        ) <= 0.75:
            return None

        start = grid.world_to_grid(float(pose[0]), float(pose[1]))
        if start is None or not grid.is_traversable(*start):
            return None
        required_clearance = float(rotation_min_clearance) + float(
            clearance_margin
        )
        selection = None
        for trace_point in reversed(list(navigation_trace)):
            target = (float(trace_point[0]), float(trace_point[1]))
            distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
            if distance < 0.5 or distance > float(maximum_distance):
                continue
            target_cell = grid.world_to_grid(*target)
            if target_cell is None or not grid.is_traversable(*target_cell):
                continue
            if not grid_cell_has_clearance(grid, target_cell, required_clearance):
                continue
            plan = astar_plan(grid, (pose[0], pose[1]), target)
            if not plan.reached or len(plan.path) != 2:
                continue
            target_direction = math.atan2(target[1] - pose[1], target[0] - pose[0])
            backward_error = math.atan2(
                math.sin(target_direction - pose[2] - math.pi),
                math.cos(target_direction - pose[2] - math.pi),
            )
            if abs(backward_error) > 0.35:
                continue
            selection = ReturnHomeRetreatSelection(
                target=target,
                path=tuple(plan.path),
                required_clearance=required_clearance,
            )
            break
        self.retreat_anchor = (float(pose[0]), float(pose[1]))
        return selection

    def begin_retreat(
        self,
        selection: ReturnHomeRetreatSelection,
        *,
        now: float,
        reverse_speed: float,
    ) -> None:
        self.planner = None
        self.retreat_target = selection.target
        retreat_distance = math.hypot(
            selection.target[0] - self.retreat_anchor[0],
            selection.target[1] - self.retreat_anchor[1],
        )
        self.retreat_deadline = float(now) + max(
            4.0,
            retreat_distance / max(0.05, abs(float(reverse_speed))) + 2.0,
        )

    def tick_retreat(
        self,
        pose: tuple[float, float, float],
        *,
        now: float,
    ) -> ReturnHomeRetreatStep:
        target = self.retreat_target
        if target is None:
            return ReturnHomeRetreatStep("inactive", None)
        if math.hypot(target[0] - pose[0], target[1] - pose[1]) <= 0.25:
            self.retreat_target = None
            self.retreat_deadline = 0.0
            self.next_plan_time = 0.0
            return ReturnHomeRetreatStep("completed", target)
        if float(now) >= self.retreat_deadline:
            self.retreat_target = None
            return ReturnHomeRetreatStep("timeout", target)
        target_direction = math.atan2(target[1] - pose[1], target[0] - pose[0])
        backward_error = math.atan2(
            math.sin(target_direction - pose[2] - math.pi),
            math.cos(target_direction - pose[2] - math.pi),
        )
        if abs(backward_error) > 0.45:
            self.retreat_target = None
            return ReturnHomeRetreatStep("heading_invalid", target)
        return ReturnHomeRetreatStep("reverse", target)

    def abort_retreat(self) -> None:
        self.retreat_target = None

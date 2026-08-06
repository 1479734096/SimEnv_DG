"""普通动态门的候选资格、主动接近与任务构造。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from danger_search.navigation_tasks import OpenDoorAndPass
from danger_search.planner import astar_plan, path_length


@dataclass(frozen=True)
class DoorCandidateDecision:
    allowed: bool
    failure_reason: str | None = None


@dataclass
class DoorNavigationRuntime:
    approach_blacklist: dict[str, float] = field(default_factory=dict)
    last_door_id: str | None = None
    last_door_xy: tuple[float, float] | None = None
    last_pass_vector: tuple[float, float] | None = None
    transition_until: float = 0.0
    context_until: float = 0.0
    completion_pose: tuple[float, float, float] | None = None
    opening_sensor_confirmed: bool = False
    frontier_debug: dict[str, object] | None = None

    def candidate_decision(
        self,
        candidate,
        *,
        exploring: bool,
        current_task_present: bool,
        opened_doors,
        failed_doors,
        active_door: str | None,
        building_client,
        maximum_distance: float,
    ) -> DoorCandidateDecision:
        if (
            candidate is None
            or not exploring
            or current_task_present
            or candidate.kind != "dynamic_door"
            or candidate.door_id in opened_doors
            or candidate.door_id in failed_doors
            or candidate.door_id == active_door
            or building_client.is_elevator_door(candidate.door_id)
            or candidate.distance >= float(maximum_distance)
            or not candidate.visible
        ):
            return DoorCandidateDecision(False)
        if not building_client.door_service_ready():
            return DoorCandidateDecision(False, "门服务不可用")
        return DoorCandidateDecision(True)

    def unopened_doors(
        self,
        *,
        floor_index: int,
        opened_doors,
        failed_doors,
        building_client,
        now: float,
    ) -> list:
        result = []
        for door in building_client.controllable_doors(
            floor_index=int(floor_index),
            include_elevators=False,
        ):
            if door.door_id in opened_doors or door.door_id in failed_doors:
                continue
            if self.approach_blacklist.get(door.door_id, 0.0) > float(now):
                continue
            result.append(door)
        return result

    def select_approach(
        self,
        pose: tuple[float, float, float],
        *,
        floor_index: int,
        grid,
        opened_doors,
        failed_doors,
        building_client,
        pass_offset: float,
        now: float,
        planning_constraints=None,
    ) -> str | None:
        doors = self.unopened_doors(
            floor_index=floor_index,
            opened_doors=opened_doors,
            failed_doors=failed_doors,
            building_client=building_client,
            now=now,
        )
        robot_xy = (float(pose[0]), float(pose[1]))
        best_id = None
        best_length = math.inf
        for door in doors:
            try:
                wait_point, _ = building_client.get_door_points(
                    door.door_id,
                    robot_xy,
                    offset=float(pass_offset),
                )
            except ValueError:
                continue
            result = astar_plan(
                grid, robot_xy, wait_point, constraints=planning_constraints
            )
            if result.reached:
                length = path_length(result.path)
                if length < best_length:
                    best_length = length
                    best_id = door.door_id
            elif result.unreachable:
                # 只有确定不可达才冷却；规划预算不足保留给下一轮。
                self.approach_blacklist[door.door_id] = float(now) + 20.0
        return best_id

    def record_completed(
        self,
        task: OpenDoorAndPass,
        pose: tuple[float, float, float],
        *,
        now: float,
        transition_duration: float,
        context_duration: float,
    ) -> bool:
        if task.wait_point is None or task.pass_point is None:
            self.frontier_debug = None
            return False
        center = (
            0.5 * (task.wait_point[0] + task.pass_point[0]),
            0.5 * (task.wait_point[1] + task.pass_point[1]),
        )
        pass_x = task.pass_point[0] - center[0]
        pass_y = task.pass_point[1] - center[1]
        norm = math.hypot(pass_x, pass_y)
        self.last_pass_vector = (
            (pass_x / norm, pass_y / norm) if norm > 1e-6 else None
        )
        self.last_door_id = str(task.door_id)
        self.last_door_xy = center
        self.completion_pose = tuple(float(value) for value in pose)
        self.opening_sensor_confirmed = bool(
            getattr(task, "opening_sensor_confirmed", False)
        )
        self.transition_until = float(now) + float(transition_duration)
        self.context_until = float(now) + max(
            float(transition_duration),
            float(context_duration),
        )
        self.frontier_debug = {
            "door_id": task.door_id,
            "wait_point": task.wait_point,
            "pass_point": task.pass_point,
            "commit_point": task.commit_point,
            "completion_pose": self.completion_pose,
        }
        return True

    def progress_distance(self, pose: tuple[float, float, float]) -> float:
        if self.last_door_xy is None or self.last_pass_vector is None:
            return 0.0
        dx = float(pose[0]) - self.last_door_xy[0]
        dy = float(pose[1]) - self.last_door_xy[1]
        return max(0.0, dx * self.last_pass_vector[0] + dy * self.last_pass_vector[1])

    def known_area(self, grid, *, minimum_distance: float) -> float:
        if self.last_door_xy is None or self.last_pass_vector is None:
            return 0.0
        door_cell = grid.world_to_grid(*self.last_door_xy)
        if door_cell is None:
            return 0.0
        radius = int((float(minimum_distance) + 2.0) / grid.resolution)
        known = 0
        for gy in range(max(0, door_cell[1] - radius), min(grid.size_y, door_cell[1] + radius + 1)):
            for gx in range(max(0, door_cell[0] - radius), min(grid.size_x, door_cell[0] + radius + 1)):
                if int(grid.data[gy, gx]) == -1:
                    continue
                wx, wy = grid.grid_to_world(gx, gy)
                side = (
                    (wx - self.last_door_xy[0]) * self.last_pass_vector[0]
                    + (wy - self.last_door_xy[1]) * self.last_pass_vector[1]
                )
                if side >= -0.05:
                    known += 1
        return float(known * grid.resolution * grid.resolution)

    def context_satisfied(
        self,
        pose: tuple[float, float, float],
        *,
        grid,
        minimum_distance: float,
        minimum_known_area: float,
    ) -> bool:
        if self.last_door_xy is None or self.last_pass_vector is None:
            return True
        area = (
            self.known_area(grid, minimum_distance=minimum_distance)
            if minimum_known_area > 0.0
            else 0.0
        )
        return (
            self.progress_distance(pose) >= float(minimum_distance)
            and area >= float(minimum_known_area)
        )

    def side_label(self, target_xy: tuple[float, float]) -> str:
        if self.last_door_xy is None or self.last_pass_vector is None:
            return "无门上下文"
        signed = (
            (float(target_xy[0]) - self.last_door_xy[0]) * self.last_pass_vector[0]
            + (float(target_xy[1]) - self.last_door_xy[1]) * self.last_pass_vector[1]
        )
        if signed > 0.05:
            return "门后侧"
        if signed < -0.05:
            return "门前侧"
        return "门中线附近"

    @staticmethod
    def classify_goal_side(
        wait_point: tuple[float, float],
        pass_point: tuple[float, float],
        target_xy: tuple[float, float],
    ) -> str:
        center_x = 0.5 * (wait_point[0] + pass_point[0])
        center_y = 0.5 * (wait_point[1] + pass_point[1])
        direction_x = pass_point[0] - wait_point[0]
        direction_y = pass_point[1] - wait_point[1]
        norm = math.hypot(direction_x, direction_y)
        if norm <= 1e-6:
            return "无法判断"
        signed = (
            (target_xy[0] - center_x) * direction_x
            + (target_xy[1] - center_y) * direction_y
        ) / norm
        if signed > 0.05:
            return "门后侧(通过点一侧)"
        if signed < -0.05:
            return "门前侧(等待点一侧)"
        return "门中线附近"

    @staticmethod
    def create_task(
        door_id: str,
        *,
        pass_offset: float,
        commit_distance: float,
    ) -> OpenDoorAndPass:
        return OpenDoorAndPass(
            str(door_id),
            pass_offset=float(pass_offset),
            commit_distance=float(commit_distance),
        )

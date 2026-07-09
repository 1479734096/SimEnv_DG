"""局部语义检测。

当前版本只做轻量规则检测：用局部点云确认近距离门状结构，再把候选
匹配到可控动态门服务 ID。metadata 只用于 ID 匹配，不生成导航目标。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from danger_search.building_control_client import BuildingControlClient, DoorInfo


@dataclass(frozen=True)
class DoorCandidate:
    door_id: str
    distance: float
    visible: bool
    kind: str
    floor_index: int
    world_xy: tuple[float, float]
    source: str
    evidence_count: int = 0


class LocalSemanticDetector:
    """检测机器人局部可见范围内的可控动态门。"""

    def __init__(
        self,
        *,
        max_distance: float = 3.0,
        fov_half_angle: float = 0.85,
        evidence_radius: float = 0.75,
        min_evidence_points: int = 6,
        min_evidence_height: float = 0.15,
        max_evidence_height: float = 1.80,
        use_truth_proximity_detector: bool = False,
    ):
        self.max_distance = float(max_distance)
        self.fov_half_angle = float(fov_half_angle)
        self.evidence_radius = float(evidence_radius)
        self.min_evidence_points = int(min_evidence_points)
        self.min_evidence_height = float(min_evidence_height)
        self.max_evidence_height = float(max_evidence_height)
        self.use_truth_proximity_detector = bool(use_truth_proximity_detector)

    def detect_controllable_door(
        self,
        *,
        robot_pose: tuple[float, float, float],
        floor_index: int,
        cloud_points: Iterable[tuple[float, float, float]],
        building_client: BuildingControlClient,
        floor_height: float = 2.6,
        allow_elevator: bool = False,
    ) -> DoorCandidate | None:
        candidates: list[DoorCandidate] = []
        points = list(cloud_points)
        for door in building_client.controllable_doors(
            floor_index=floor_index,
            include_elevators=allow_elevator,
        ):
            if door.kind == "elevator" and not allow_elevator:
                continue
            distance = math.hypot(door.x - robot_pose[0], door.y - robot_pose[1])
            if distance > self.max_distance:
                continue
            visible = self._in_sensor_view(robot_pose, door)
            if not visible:
                continue
            evidence_count = self._count_local_structure_points(door, points, floor_index, floor_height)
            if evidence_count >= self.min_evidence_points:
                candidates.append(
                    DoorCandidate(
                        door_id=door.door_id,
                        distance=distance,
                        visible=True,
                        kind="dynamic_door" if door.kind != "elevator" else "elevator",
                        floor_index=door.floor_index,
                        world_xy=(door.x, door.y),
                        source="local_pointcloud",
                        evidence_count=evidence_count,
                    )
                )
            elif self.use_truth_proximity_detector:
                candidates.append(
                    DoorCandidate(
                        door_id=door.door_id,
                        distance=distance,
                        visible=True,
                        kind="dynamic_door" if door.kind != "elevator" else "elevator",
                        floor_index=door.floor_index,
                        world_xy=(door.x, door.y),
                        source="truth_proximity_debug",
                        evidence_count=evidence_count,
                    )
                )
        if not candidates:
            return None
        return min(candidates, key=lambda candidate: candidate.distance)

    def _in_sensor_view(self, robot_pose: tuple[float, float, float], door: DoorInfo) -> bool:
        dx = door.x - robot_pose[0]
        dy = door.y - robot_pose[1]
        bearing = math.atan2(dy, dx)
        return abs(_angle_diff(bearing, robot_pose[2])) <= self.fov_half_angle

    def _count_local_structure_points(
        self,
        door: DoorInfo,
        points: list[tuple[float, float, float]],
        floor_index: int,
        floor_height: float,
    ) -> int:
        count = 0
        floor_z = floor_index * floor_height
        for px, py, pz in points:
            if math.hypot(px - door.x, py - door.y) > self.evidence_radius:
                continue
            z_rel = pz - floor_z
            if self.min_evidence_height <= z_rel <= self.max_evidence_height:
                count += 1
        return count


def _angle_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi

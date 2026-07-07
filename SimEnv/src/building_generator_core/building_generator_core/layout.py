"""楼栋生成器的数据模型。

本文件只定义“楼栋是什么”，不负责随机生成，也不负责导出 SDF。
generator.py 负责填充这些 dataclass，exporter.py 负责把它们翻译成 Gazebo 文件。
这些结构最终会同时出现在 layout_metadata.json、door_config.yaml、
elevator_config.yaml 和比赛真值文件中。
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class Rect2D:
    """二维轴对齐矩形，用于描述房间、走廊、楼梯间、电梯井等平面范围。"""
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def length(self) -> float:
        return self.y_max - self.y_min

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)

    def as_dict(self) -> dict[str, float]:
        return {
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
        }


@dataclass(frozen=True)
class FurnitureSpec:
    """房间内家具的抽象描述，导出时会转成 Gazebo 中的静态 box。"""
    id: str
    kind: str
    pose: tuple[float, float, float, float, float, float]
    size: tuple[float, float, float]

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "kind": self.kind,
            "pose": list(self.pose),
            "size": list(self.size),
        }


@dataclass(frozen=True)
class RoomSpec:
    """单个房间的布局信息。

    door_pose 用于生成房门和危险源避让区域，goal_pose 用于给导航/评估提供
    房间内部的代表性目标点。
    """
    id: str
    floor_index: int
    room_type: str
    bounds: Rect2D
    side: str
    door_pose: tuple[float, float, float, float, float, float]
    goal_pose: tuple[float, float, float, float, float, float]
    furniture: list[FurnitureSpec] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "floor_index": self.floor_index,
            "room_type": self.room_type,
            "bounds": self.bounds.as_dict(),
            "side": self.side,
            "door_pose": list(self.door_pose),
            "goal_pose": list(self.goal_pose),
            "furniture": [item.as_dict() for item in self.furniture],
        }


@dataclass(frozen=True)
class DoorSpec:
    """动态门的抽象状态。

    exporter.py 会根据这里的 pose/width/height 生成 Gazebo 门模型和
    door_config.yaml；control_server.py 再读取配置来开关门。
    """
    id: str
    floor_index: int
    kind: str
    pose: tuple[float, float, float, float, float, float]
    width: float
    height: float
    initial_open: bool
    dynamic: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "floor_index": self.floor_index,
            "kind": self.kind,
            "pose": list(self.pose),
            "width": self.width,
            "height": self.height,
            "initial_open": self.initial_open,
            "dynamic": self.dynamic,
        }


@dataclass(frozen=True)
class ElevatorSpec:
    """电梯抽象状态。

    served_floors 和 lobby_positions 用于生成每层电梯门、电梯轿厢目标位置，
    后续 /call_elevator 服务会根据这些位置移动轿厢模型。
    """
    id: str
    shaft_bounds: Rect2D
    served_floors: list[int]
    current_floor: int
    car_size: tuple[float, float, float]
    lobby_positions: dict[int, tuple[float, float, float, float, float, float]]

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "shaft_bounds": self.shaft_bounds.as_dict(),
            "served_floors": self.served_floors,
            "current_floor": self.current_floor,
            "car_size": list(self.car_size),
            "lobby_positions": {
                str(key): list(value) for key, value in self.lobby_positions.items()
            },
        }


@dataclass(frozen=True)
class FloorLayout:
    """单层楼的完整布局。"""
    floor_index: int
    elevation: float
    lobby_bounds: Rect2D
    corridor_bounds: Rect2D
    stair_bounds: Rect2D
    elevator_bounds: Rect2D
    rooms: list[RoomSpec]
    reachability: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return {
            "floor_index": self.floor_index,
            "elevation": self.elevation,
            "lobby_bounds": self.lobby_bounds.as_dict(),
            "corridor_bounds": self.corridor_bounds.as_dict(),
            "stair_bounds": self.stair_bounds.as_dict(),
            "elevator_bounds": self.elevator_bounds.as_dict(),
            "rooms": [room.as_dict() for room in self.rooms],
            "reachability": self.reachability,
        }


@dataclass(frozen=True)
class BuildingLayout:
    """整栋楼的完整抽象布局，是 generator.py 的核心输出。"""
    model_name: str
    footprint: dict[str, float]
    floor_height: float
    wall_height: float
    entrance_pose: tuple[float, float, float, float, float, float]
    floors: list[FloorLayout]
    door_specs: list[DoorSpec]
    elevator_specs: list[ElevatorSpec]
    signature: str
    target_points: dict[str, Any]
    metadata: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return {
            "model_name": self.model_name,
            "footprint": self.footprint,
            "floor_height": self.floor_height,
            "wall_height": self.wall_height,
            "entrance_pose": list(self.entrance_pose),
            "floors": [floor.as_dict() for floor in self.floors],
            "door_specs": [door.as_dict() for door in self.door_specs],
            "elevator_specs": [elevator.as_dict() for elevator in self.elevator_specs],
            "signature": self.signature,
            "target_points": self.target_points,
            "metadata": self.metadata,
        }


@dataclass(frozen=True)
class ArtifactPaths:
    """export_sdf 写出的关键产物路径，供上层脚本继续组装比赛场景。"""
    world_sdf: str
    model_sdf: str
    layout_metadata: str
    elevator_config: str
    door_config: str
    validation_report: str

    def as_dict(self) -> dict[str, str]:
        return asdict(self)

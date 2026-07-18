"""公开场景信息的唯一读取与解释入口。

初始接口文档只允许读取 ``team_scene_info.json``。该文件只给出 ID、起点、
服务楼层和结果文件，不包含门、电梯的空间真值；本模块刻意不提供空间查询。

调用关系为 ``MissionManager._load_params -> BuildingFeatureProvider.load``。
加载在任何订阅、发布或服务连接之前完成；schema、ID、楼层或接口列表一旦非法，
节点会 fail-closed，而不是退回布局元数据或根据服务响应猜测建筑结构。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from danger_search.interface_policy import (
    ALLOWED_SERVICES,
    ALLOWED_TOPICS,
    require_public_scene_info,
)


@dataclass(frozen=True)
class PublicDoorFeature:
    """公开门的非空间属性。

    ``floor_index`` 只用于校验服务 ID 所属楼层，不能推导门的位置或朝向。
    空间几何只能由运行期间的合法传感器在本地注册。
    """

    door_id: str
    kind: str
    floor_index: int
    initial_open: bool


@dataclass(frozen=True)
class PublicElevatorFeature:
    """公开电梯的非空间属性，用于限制可请求的目标楼层集合。"""

    elevator_id: str
    current_floor: int
    served_floors: frozenset[int]


@dataclass(frozen=True)
class BuildingFeatureProvider:
    """经完整校验、可安全传给业务层的公开场景能力描述。

    该不可变对象是正式运行时关于建筑公开能力的唯一事实源。门/电梯客户端、
    楼层预算和结果写入都使用这里的字段，避免不同模块重复解析 JSON 后产生
    schema 语义差异。
    """

    path: Path
    robot_start: tuple[float, float, float, float]
    door_ids: frozenset[str]
    elevators: dict[str, frozenset[int]]
    doors: dict[str, PublicDoorFeature]
    elevator_features: dict[str, PublicElevatorFeature]
    allowed_topics: frozenset[str]
    allowed_services: frozenset[str]
    result_file: Path

    @classmethod
    def load(cls, path: str | Path, *, purpose: str) -> "BuildingFeatureProvider":
        """读取并严格验证公开 schema，成功后返回不可变能力快照。

        校验顺序从文件身份、根 schema、起点数值，逐步进入门、电梯和接口集合；
        任一步失败都会抛出 ``ValueError``，不会返回部分可用对象。输出路径也被
        固定在工作区 ``results/detected_danger.json``，防止配置诱导任意文件写入。
        """
        scene_path = require_public_scene_info(path, purpose=purpose)
        with scene_path.open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
        if not isinstance(raw, dict):
            raise ValueError("公开场景信息根节点必须是对象")
        if raw.get("schema") != "team_scene_info_v1":
            raise ValueError("公开场景信息 schema 必须为 team_scene_info_v1")
        if raw.get("coordinate_frame") != "world":
            raise ValueError("公开场景信息坐标系必须是 world")
        start = raw.get("robot_start", {})
        if not isinstance(start, dict):
            raise ValueError("公开场景信息的 robot_start 必须是对象")
        try:
            start_values = tuple(start[key] for key in ("x", "y", "z", "yaw"))
            if any(type(value) not in (int, float) for value in start_values):
                raise ValueError("robot_start 必须使用 JSON 数值")
            robot_start = tuple(float(value) for value in start_values)
            if not all(math.isfinite(value) for value in robot_start):
                raise ValueError("robot_start 包含非有限数值")
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("公开场景信息缺少合法 robot_start") from exc
        public_scene = raw.get("public_scene", {})
        if not isinstance(public_scene, dict):
            raise ValueError("公开场景信息的 public_scene 必须是对象")
        door_ids = []
        door_features: dict[str, PublicDoorFeature] = {}
        raw_doors = public_scene.get("door_ids")
        if not isinstance(raw_doors, list):
            raise ValueError("door_ids 必须是列表")
        for item in raw_doors:
            if not isinstance(item, dict):
                raise ValueError("door_ids 包含非法条目")
            door_id = item.get("id")
            kind = item.get("kind")
            floor_index = item.get("floor_index")
            initial_open = item.get("initial_open")
            if not isinstance(door_id, str) or not door_id.strip():
                raise ValueError("door_ids 包含非法 ID")
            if not isinstance(kind, str) or not kind.strip():
                raise ValueError("door_ids 缺少合法 kind")
            if type(floor_index) is not int or floor_index < 0:
                raise ValueError("door_ids 的 floor_index 必须为非负整数")
            if type(initial_open) is not bool:
                raise ValueError("door_ids 的 initial_open 必须为布尔值")
            door_ids.append(door_id)
            door_features[door_id] = PublicDoorFeature(
                door_id=door_id,
                kind=kind,
                floor_index=floor_index,
                initial_open=initial_open,
            )
        if len(door_ids) != len(set(door_ids)):
            raise ValueError("door_ids 存在重复 ID")
        doors = frozenset(door_ids)
        elevator_ids = []
        elevators: dict[str, frozenset[int]] = {}
        elevator_features: dict[str, PublicElevatorFeature] = {}
        raw_elevators = public_scene.get("elevators")
        if not isinstance(raw_elevators, list):
            raise ValueError("elevators 必须是列表")
        for item in raw_elevators:
            if not isinstance(item, dict):
                raise ValueError("elevators 包含非法条目")
            elevator_id = item.get("id")
            current_floor = item.get("current_floor")
            raw_floors = item.get("served_floors")
            if not isinstance(elevator_id, str) or not elevator_id.strip():
                raise ValueError("elevators 包含非法 ID")
            if type(current_floor) is not int or current_floor < 0:
                raise ValueError("elevators 的 current_floor 必须为非负整数")
            if not isinstance(raw_floors, list) or not raw_floors:
                raise ValueError("served_floors 必须为非空列表")
            if any(type(floor) is not int or floor < 0 for floor in raw_floors):
                raise ValueError("served_floors 必须为非负整数")
            floors = frozenset(raw_floors)
            if len(floors) != len(raw_floors):
                raise ValueError("served_floors 存在重复楼层")
            if current_floor not in floors:
                raise ValueError("current_floor 必须位于 served_floors")
            elevator_ids.append(elevator_id)
            elevators[elevator_id] = floors
            elevator_features[elevator_id] = PublicElevatorFeature(
                elevator_id=elevator_id,
                current_floor=current_floor,
                served_floors=floors,
            )
        if len(elevator_ids) != len(set(elevator_ids)):
            raise ValueError("elevators 存在重复 ID")
        interfaces = raw.get("allowed_interfaces", {})
        if not isinstance(interfaces, dict):
            raise ValueError("公开场景信息的 allowed_interfaces 必须是对象")
        raw_topics = interfaces.get("topics", [])
        raw_services = interfaces.get("services", [])
        if (
            not isinstance(raw_topics, list)
            or not isinstance(raw_services, list)
            or any(not isinstance(value, str) or not value.strip() for value in raw_topics)
            or any(not isinstance(value, str) or not value.strip() for value in raw_services)
        ):
            raise ValueError("公开场景信息的接口列表必须包含非空字符串")
        topics = frozenset(raw_topics)
        services = frozenset(raw_services)
        if not topics or not topics.issubset(ALLOWED_TOPICS):
            raise ValueError("公开场景信息包含未授权的话题")
        if not services.issubset(ALLOWED_SERVICES):
            raise ValueError("公开场景信息包含未授权的服务")
        result_name = interfaces.get("result_file")
        if not isinstance(result_name, str) or not result_name:
            raise ValueError("公开场景信息缺少结果文件路径")
        result_file = Path(result_name)
        if not result_file.is_absolute():
            result_file = (scene_path.parent / result_file).resolve()
        # 结果文件必须位于本工作区 results/detected_danger.json，防止公开文件把输出指向任意系统路径。
        workspace_root = scene_path.parent.parent
        expected_result = (workspace_root / "results" / "detected_danger.json").resolve()
        if result_file.resolve() != expected_result:
            raise ValueError(
                f"公开场景信息指定的结果文件必须是 {expected_result}，而不是 {result_file}"
            )
        return cls(
            path=scene_path,
            robot_start=robot_start,
            door_ids=doors,
            elevators=elevators,
            doors=door_features,
            elevator_features=elevator_features,
            allowed_topics=topics,
            allowed_services=services,
            result_file=result_file,
        )

    def require_topic(self, topic: str, *, purpose: str) -> str:
        """要求话题同时出现在本次场景公布的接口集合中。"""
        normalized = str(topic).strip()
        if normalized not in self.allowed_topics:
            raise ValueError(f"{purpose} 未在本场景公开接口内: {normalized}")
        return normalized

    def require_service(self, service: str, *, purpose: str) -> str:
        """要求服务同时出现在本次场景公布的接口集合中。"""
        normalized = str(service).strip()
        if normalized not in self.allowed_services:
            raise ValueError(f"{purpose} 未在本场景公开服务内: {normalized}")
        return normalized

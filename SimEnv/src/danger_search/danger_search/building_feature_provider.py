"""公开场景信息的唯一读取入口。

初始接口文档只允许读取 ``team_scene_info.json``。该文件只给出 ID、起点、
服务楼层和结果文件，不包含门、电梯的空间真值；本模块刻意不提供空间查询。
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
class BuildingFeatureProvider:
    """经白名单校验后的公开场景能力描述。"""

    path: Path
    robot_start: tuple[float, float, float, float]
    door_ids: frozenset[str]
    elevators: dict[str, frozenset[int]]
    allowed_topics: frozenset[str]
    allowed_services: frozenset[str]
    result_file: Path

    @classmethod
    def load(cls, path: str | Path, *, purpose: str) -> "BuildingFeatureProvider":
        scene_path = require_public_scene_info(path, purpose=purpose)
        with scene_path.open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
        if raw.get("schema") != "team_scene_info_v1":
            raise ValueError("公开场景信息 schema 必须为 team_scene_info_v1")
        if raw.get("coordinate_frame") != "world":
            raise ValueError("公开场景信息坐标系必须是 world")
        start = raw.get("robot_start", {})
        try:
            robot_start = tuple(float(start[key]) for key in ("x", "y", "z", "yaw"))
            if not all(math.isfinite(value) for value in robot_start):
                raise ValueError("robot_start 包含非有限数值")
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("公开场景信息缺少合法 robot_start") from exc
        public_scene = raw.get("public_scene", {})
        door_ids = []
        for item in public_scene.get("door_ids", []):
            if not isinstance(item, dict) or not item.get("id"):
                raise ValueError("door_ids 包含非法条目")
            door_ids.append(str(item["id"]))
        if len(door_ids) != len(set(door_ids)):
            raise ValueError("door_ids 存在重复 ID")
        doors = frozenset(door_ids)
        elevator_ids = []
        elevators: dict[str, frozenset[int]] = {}
        for item in public_scene.get("elevators", []):
            if not isinstance(item, dict) or not item.get("id"):
                raise ValueError("elevators 包含非法条目")
            elevator_id = str(item["id"])
            elevator_ids.append(elevator_id)
            floors = frozenset(int(floor) for floor in item.get("served_floors", []))
            if any(floor < 0 for floor in floors):
                raise ValueError("served_floors 必须为为非负整数")
            elevators[elevator_id] = floors
        if len(elevator_ids) != len(set(elevator_ids)):
            raise ValueError("elevators 存在重复 ID")
        interfaces = raw.get("allowed_interfaces", {})
        topics = frozenset(str(value) for value in interfaces.get("topics", []))
        services = frozenset(str(value) for value in interfaces.get("services", []))
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
            allowed_topics=topics,
            allowed_services=services,
            result_file=result_file,
        )

    def require_topic(self, topic: str, *, purpose: str) -> str:
        normalized = str(topic).strip()
        if normalized not in self.allowed_topics:
            raise ValueError(f"{purpose} 未在本场景公开接口内: {normalized}")
        return normalized

    def require_service(self, service: str, *, purpose: str) -> str:
        normalized = str(service).strip()
        if normalized not in self.allowed_services:
            raise ValueError(f"{purpose} 未在本场景公开服务内: {normalized}")
        return normalized

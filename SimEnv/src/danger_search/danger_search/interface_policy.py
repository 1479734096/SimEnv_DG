"""正式比赛接口准入规则。

规则唯一依据为 ``SimEnv初始.zip`` 中的
``docs/algorithm-interfaces.md``。生产代码接入外部 ROS 话题、服务或
场景文件前必须通过本模块；未列入白名单的接口不能以“本地可用”为由进入
正式运行链。
"""
from __future__ import annotations

from pathlib import Path


# 初始文档“允许状态与传感器接口”与“最小控制接口”。
ALLOWED_TOPICS = frozenset(
    {
        "/clock",
        "/cmd_vel",
        "/scan",
        "/livox/Pointcloud2",
        "/livox/lidar2",
        "/livox/imu",
        "/trunk_imu",
        "/real_sense/rgb/image_raw",
        "/real_sense/rgb/camera_info",
        "/real_sense/depth/image_raw",
        "/real_sense/depth/camera_info",
        "/real_sense/depth/points",
    }
)

# 初始文档“门与电梯交互”。
ALLOWED_SERVICES = frozenset({"/set_door_state", "/call_elevator"})

# 初始文档“裁判与调试专用接口”。显式列出，便于配置错误立即失败。
FORBIDDEN_TOPICS = frozenset(
    {
        "/Odometry_gazebo",
        "/ground_truth/base_w",
        "/ground_truth/base_trunk",
        "/ground_truth/FL_foot",
        "/ground_truth/FR_foot",
        "/ground_truth/RL_foot",
        "/ground_truth/RR_foot",
    }
)

FORBIDDEN_SCENE_FILE_NAMES = frozenset(
    {
        "layout_metadata.json",
        "building_config.json",
        "scene_manifest.json",
        "danger_truth.json",
        "detected_danger.json",
    }
)

PUBLIC_SCENE_INFO_NAME = "team_scene_info.json"


def require_allowed_topic(topic: str, *, purpose: str) -> str:
    """确认生产外部话题已由初始接口文档授权。"""
    normalized = str(topic).strip()
    if normalized in FORBIDDEN_TOPICS:
        raise ValueError(f"{purpose} 禁止使用 Gazebo 真值话题: {normalized}")
    if normalized not in ALLOWED_TOPICS:
        raise ValueError(f"{purpose} 未在初始比赛接口白名单中: {normalized}")
    return normalized


def require_internal_topic(topic: str, *, purpose: str) -> str:
    """确认参赛方内部总线不会伪装成赛方外部接口。"""
    normalized = str(topic).strip()
    if not normalized.startswith(("/danger_search/", "/nav/")):
        raise ValueError(
            f"{purpose} 必须使用 /danger_search/ 或 /nav/ 命名空间内部话题: {normalized}"
        )
    return normalized


def require_allowed_service(service: str, *, purpose: str) -> str:
    """确认生产服务调用已由初始接口文档授权。"""
    normalized = str(service).strip()
    if normalized not in ALLOWED_SERVICES:
        raise ValueError(f"{purpose} 未在初始比赛服务白名单中: {normalized}")
    return normalized


def require_public_scene_info(path: str | Path, *, purpose: str) -> Path:
    """确认读取的是唯一允许的公开场景信息文件。"""
    result = Path(path)
    if result.name in FORBIDDEN_SCENE_FILE_NAMES:
        raise ValueError(f"{purpose} 禁止读取裁判/环境文件: {result}")
    if result.name != PUBLIC_SCENE_INFO_NAME:
        raise ValueError(f"{purpose} 只能读取 {PUBLIC_SCENE_INFO_NAME}: {result}")
    return result

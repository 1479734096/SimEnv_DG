"""正式比赛接口准入规则。

规则唯一依据为 ``SimEnv初始.zip`` 中的
``docs/algorithm-interfaces.md``。生产代码接入外部 ROS 话题、服务或
场景文件前必须通过本模块；未列入白名单的接口不能以“本地可用”为由进入
正式运行链。
"""
from __future__ import annotations

import os
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


def _lexical_absolute(path: str | Path) -> Path:
    """生成不解析符号链接的规范词法绝对路径。"""
    return Path(os.path.normpath(os.path.abspath(os.fspath(path))))


def official_scene_info_path() -> Path:
    """返回唯一允许的公开场景信息的词法绝对路径。

    路径基于 ROS 包 ``danger_search`` 所在工作区推导，避免依赖当前工作目录。
    根据 ``docs/algorithm-interfaces.md``，正式算法只能读取
    ``<SimEnv>/generated_building/team_scene_info.json``。
    """
    import rospkg

    package_dir = _lexical_absolute(rospkg.RosPack().get_path("danger_search"))
    workspace_dir = package_dir.parents[1]
    return _lexical_absolute(
        workspace_dir / "generated_building" / PUBLIC_SCENE_INFO_NAME
    )


def require_public_scene_info(path: str | Path, *, purpose: str) -> Path:
    """确认读取的是唯一允许的公开场景信息文件。

    校验顺序：词法路径 -> 文件类型/符号链接 -> 规范真实路径。
    ROS 包管理器返回的工作区词法根可以是部署别名；除此之外的同名路径，
    以及公开文件或 ``generated_building`` 自身的符号链接均拒绝。
    """
    supplied = Path(path)
    if supplied.parts and supplied.parts[0].startswith("~"):
        raise ValueError(f"{purpose} 正式场景文件路径不支持 shell 缩写: {supplied}")
    if supplied.name in FORBIDDEN_SCENE_FILE_NAMES:
        raise ValueError(f"{purpose} 禁止读取裁判/环境文件: {supplied}")
    if supplied.name != PUBLIC_SCENE_INFO_NAME:
        raise ValueError(f"{purpose} 只能读取 {PUBLIC_SCENE_INFO_NAME}: {supplied}")

    # 先拿到不解析符号链接的词法绝对路径，防止同名文件或 symlink 绕过。
    supplied_lexical = _lexical_absolute(supplied)
    expected_lexical = official_scene_info_path()
    if supplied_lexical != expected_lexical:
        raise ValueError(
            f"{purpose} 公开场景文件路径不合法: {supplied_lexical} != {expected_lexical}"
        )
    if supplied_lexical.parent.is_symlink() or supplied_lexical.is_symlink():
        raise ValueError(f"{purpose} 公开场景文件及其目录不能是符号链接: {supplied}")
    if not supplied_lexical.is_file():
        raise ValueError(f"{purpose} 公开场景文件必须是真实普通文件: {supplied}")

    supplied_real = supplied_lexical.resolve(strict=True)
    expected_real = expected_lexical.resolve(strict=True)
    if supplied_real != expected_real:
        raise ValueError(
            f"{purpose} 公开场景文件规范路径不一致: {supplied_real} != {expected_real}"
        )
    return supplied_real


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

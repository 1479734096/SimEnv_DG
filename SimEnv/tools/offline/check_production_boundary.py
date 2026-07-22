#!/usr/bin/env python3
"""检查离线工具是否与正式 danger_search 运行闭包隔离。"""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import re
from typing import Iterable


FORBIDDEN_LAUNCH_TOKENS = (
    "/Odometry_gazebo",
    "/ground_truth/",
    "layout_metadata.json",
    "building_config.json",
    "scene_manifest.json",
    "danger_truth.json",
    ".world",
    "rosbag",
    "evaluate_danger",
    "batch_evaluate",
)

FORBIDDEN_IMPORT_PREFIXES = (
    "building_obstacles",
    "danger_search.evaluation_tools",
    "evaluation_tools",
    "tools.offline",
)


def _without_comments(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def installed_python_programs(cmake_text: str) -> tuple[str, ...]:
    """提取 catkin_install_python 的 PROGRAMS 清单。"""
    clean = _without_comments(cmake_text)
    matches = re.findall(
        r"catkin_install_python\s*\(\s*PROGRAMS\s+(.*?)\s+DESTINATION",
        clean,
        flags=re.DOTALL,
    )
    programs: list[str] = []
    for match in matches:
        programs.extend(match.split())
    return tuple(programs)


def imported_modules(path: Path) -> tuple[str, ...]:
    """返回 Python 文件的显式 import 模块名。"""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)
    return tuple(modules)


def _has_forbidden_prefix(module: str, prefixes: Iterable[str]) -> bool:
    return any(module == prefix or module.startswith(prefix + ".") for prefix in prefixes)


def audit_workspace(workspace: Path) -> list[str]:
    """返回隔离违规说明；空列表表示通过。"""
    root = workspace.resolve()
    package = root / "src" / "danger_search"
    python_package = package / "danger_search"
    offline_dir = root / "tools" / "offline"
    errors: list[str] = []

    if not offline_dir.is_dir():
        errors.append("缺少根目录 tools/offline 隔离区")
    if package in offline_dir.parents:
        errors.append("tools/offline 不得位于 danger_search catkin 包内")

    cmake_path = package / "CMakeLists.txt"
    programs = installed_python_programs(cmake_path.read_text(encoding="utf-8"))
    expected_programs = ("scripts/danger_search_node",)
    if programs != expected_programs:
        errors.append(
            "danger_search Python 安装清单必须严格等于 "
            f"{expected_programs}，实际为 {programs}"
        )

    setup_text = (package / "setup.py").read_text(encoding="utf-8")
    if "evaluation_tools" in setup_text or "tools.offline" in setup_text:
        errors.append("setup.py 不得安装离线评测模块")

    production_python = [
        *sorted(python_package.glob("*.py")),
        package / "scripts" / "danger_search_node",
    ]
    for path in production_python:
        lowered_name = path.name.lower()
        if "evaluation" in lowered_name or "offline" in lowered_name:
            errors.append(f"生产 Python 包中存在离线模块: {path.relative_to(root)}")
        for module in imported_modules(path):
            if _has_forbidden_prefix(module, FORBIDDEN_IMPORT_PREFIXES):
                errors.append(
                    f"生产模块 {path.relative_to(root)} 导入离线依赖 {module}"
                )

    launch_dirs = (
        package / "launch",
        root / "src" / "danger_search_localization" / "launch",
    )
    for launch_dir in launch_dirs:
        for path in sorted(launch_dir.glob("*.launch")):
            content = path.read_text(encoding="utf-8")
            for token in FORBIDDEN_LAUNCH_TOKENS:
                if token in content:
                    errors.append(
                        f"launch {path.relative_to(root)} 引用了禁用离线标识 {token}"
                    )

    for path in root.glob("src/**/CMakeLists.txt"):
        content = _without_comments(path.read_text(encoding="utf-8"))
        if "tools/offline" in content:
            errors.append(f"CMake 不得安装 tools/offline: {path.relative_to(root)}")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="工作区根目录，默认由脚本位置推导",
    )
    args = parser.parse_args()
    errors = audit_workspace(args.workspace)
    if errors:
        print("生产/离线边界审计失败：")
        for error in errors:
            print(f"- {error}")
        return 1
    print("生产/离线边界审计通过。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

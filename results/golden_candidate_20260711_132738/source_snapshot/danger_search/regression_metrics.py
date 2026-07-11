"""danger_search 系统级回归指标提取。

该模块离线读取 run 归档目录，计算用于版本对照的指标，不依赖运行中的节点。
"""
from __future__ import annotations

import json
import math
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from danger_search.evaluation_tools import read_json


@dataclass
class TrajectoryMetrics:
    """从 mission_runtime 日志位姿流计算的运动指标。"""

    sample_count: int = 0
    actual_distance: float = 0.0
    max_home_distance: float = 0.0
    repeat_ratio: float = 0.0
    region_count: int = 0
    long_stop_ratio: float = 0.0
    long_stop_count: int = 0
    median_speed: float = 0.0


@dataclass
class RunMetrics:
    """单个 run 的回归指标。"""

    run_id: str = ""
    seed: str = ""
    ref: str = ""
    completed: bool = False
    return_home_success: bool = False
    terminal_reason: str = ""
    failure_reasons: list[str] = field(default_factory=list)

    # 硬失败标志
    hard_failure: bool = False
    return_failed: bool = False
    severe_stuck: bool = False
    wrong_exit: bool = False
    state_loop: bool = False
    door_pass_failed: bool = False

    # 探索与识别
    exploration_time: float = 0.0
    max_home_distance: float = 0.0
    known_area: float = 0.0
    visited_area: float = 0.0
    coverage_ratio: float = 0.0
    confirmed_count: int = 0
    correct_count: int = 0
    missed_count: int = 0
    false_alarm_count: int = 0

    # 导航健康
    stuck_count: int = 0
    recover_count: int = 0
    stuck_per_minute: float = 0.0
    recover_per_minute: float = 0.0

    # 轨迹效率（第一版用实际轨迹/最大离家距离作为深度效率代理）
    trajectory: TrajectoryMetrics = field(default_factory=TrajectoryMetrics)

    # 辅助原始数据
    raw: dict[str, Any] = field(default_factory=dict)


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def load_trajectory(log_path: str | Path) -> list[dict[str, Any]]:
    """从 danger_search.log 解析 mission_runtime 事件位姿流。

    返回每个采样点的 dict：time, x, y, yaw, state, task, phase。
    """
    trajectory: list[dict[str, Any]] = []
    path = Path(log_path)
    if not path.exists():
        return trajectory
    for line in path.read_text(encoding="utf-8").splitlines():
        # 日志格式: [LEVEL] [wall_time, sim_time]: <json>
        if "]: " not in line:
            continue
        json_part = line.split("]: ", 1)[1]
        try:
            data = json.loads(json_part)
        except json.JSONDecodeError:
            continue
        if data.get("event") != "mission_runtime":
            continue
        pose = data.get("pose")
        if not isinstance(pose, (list, tuple)) or len(pose) < 2:
            continue
        trajectory.append(
            {
                "time": float(data.get("elapsed", 0.0)),
                "x": float(pose[0]),
                "y": float(pose[1]),
                "yaw": float(pose[2]) if len(pose) > 2 else 0.0,
                "state": data.get("state"),
                "task": data.get("current_task"),
                "phase": data.get("current_task_phase"),
            }
        )
    return trajectory


def compute_trajectory_metrics(
    trajectory: list[dict[str, Any]],
    home_pose: tuple[float, float, float] | None = None,
    spatial_bin_size: float = 0.5,
    long_stop_speed_threshold: float = 0.03,
    long_stop_window_seconds: float = 3.0,
) -> TrajectoryMetrics:
    """计算轨迹效率指标。

    - actual_distance：累计欧氏里程。
    - max_home_distance：离家的最远距离。
    - repeat_ratio：对 odom 降采样到 spatial_bin_size 网格后，重访网格采样点占比。
    - region_count：轨迹降采样后按连通性（相邻网格）划分的区域数，作为分支/区域代理。
    - long_stop_ratio：长时间近乎静止的采样点占比。
    """
    metrics = TrajectoryMetrics()
    if len(trajectory) < 2:
        return metrics

    distances: list[float] = []
    home_xy = (home_pose[0], home_pose[1]) if home_pose else (0.0, 0.0)
    max_home_distance = 0.0

    prev = trajectory[0]
    for current in trajectory[1:]:
        dx = current["x"] - prev["x"]
        dy = current["y"] - prev["y"]
        dt = max(1e-6, current["time"] - prev["time"])
        distance = math.hypot(dx, dy)
        distances.append(distance)
        max_home_distance = max(max_home_distance, math.hypot(current["x"] - home_xy[0], current["y"] - home_xy[1]))
        prev = current

    metrics.sample_count = len(trajectory)
    metrics.actual_distance = sum(distances)
    metrics.max_home_distance = max_home_distance

    # 速度中位数
    speeds = [d / max(1e-6, trajectory[i + 1]["time"] - trajectory[i]["time"]) for i, d in enumerate(distances)]
    metrics.median_speed = float(sorted(speeds)[len(speeds) // 2]) if speeds else 0.0

    # 降采样到网格，统计重复访问
    bin_counts: dict[tuple[int, int], int] = Counter()
    for point in trajectory:
        gx = int(math.floor(point["x"] / spatial_bin_size))
        gy = int(math.floor(point["y"] / spatial_bin_size))
        bin_counts[(gx, gy)] += 1

    total_samples = len(trajectory)
    revisit_samples = sum(count for count in bin_counts.values() if count > 1)
    metrics.repeat_ratio = revisit_samples / total_samples if total_samples > 0 else 0.0

    # 区域数：用网格连通分量（4-邻域）近似
    occupied = set(bin_counts.keys())
    region_count = 0
    visited: set[tuple[int, int]] = set()
    for cell in occupied:
        if cell in visited:
            continue
        region_count += 1
        stack = [cell]
        while stack:
            cx, cy = stack.pop()
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (cx + dx, cy + dy)
                if neighbor in occupied and neighbor not in visited:
                    stack.append(neighbor)
    metrics.region_count = region_count

    # 长时停留：滑动窗口内平均速度低于阈值
    long_stop_samples = 0
    window_size = max(1, int(round(long_stop_window_seconds)))
    for i in range(len(trajectory)):
        start = max(0, i - window_size // 2)
        end = min(len(trajectory), i + window_size // 2 + 1)
        window_speeds = speeds[start:end]
        if window_speeds and sum(window_speeds) / len(window_speeds) < long_stop_speed_threshold:
            long_stop_samples += 1
    metrics.long_stop_ratio = long_stop_samples / total_samples if total_samples > 0 else 0.0
    metrics.long_stop_count = long_stop_samples

    return metrics


def detect_state_loop(transitions: list[dict[str, Any]]) -> bool:
    """检测状态循环：EXPLORE_FLOOR / RETURN_HOME 之间反复切换超过阈值。"""
    state_sequence = [t.get("new_state") for t in transitions if t.get("new_state")]
    loop_patterns = (
        ("EXPLORE_FLOOR", "RETURN_HOME", "EXPLORE_FLOOR"),
        ("RETURN_HOME", "EXPLORE_FLOOR", "RETURN_HOME"),
    )
    loop_count = 0
    for i in range(len(state_sequence) - 2):
        if tuple(state_sequence[i : i + 3]) in loop_patterns:
            loop_count += 1
    return loop_count >= 2


def detect_wrong_exit(trajectory: list[dict[str, Any]], bounds: dict[str, float] | None = None) -> bool:
    """检测机器人是否离开建筑边界。"""
    if bounds is None:
        # 默认边界为单层典型范围；调用方应传入 layout_metadata 中的楼层 bounds
        bounds = {"x_min": -15.0, "x_max": 15.0, "y_min": -10.0, "y_max": 45.0}
    for point in trajectory:
        if (
            point["x"] < bounds["x_min"]
            or point["x"] > bounds["x_max"]
            or point["y"] < bounds["y_min"]
            or point["y"] > bounds["y_max"]
        ):
            return True
    return False


def extract_floor_bounds(layout_metadata_path: str | Path) -> dict[str, float]:
    """从 layout_metadata.json 读取建筑外框，包含入口区域。"""
    path = Path(layout_metadata_path)
    default = {"x_min": -20.0, "x_max": 20.0, "y_min": -15.0, "y_max": 50.0}
    if not path.exists():
        return default
    data = read_json(path, default={})
    floors = data.get("floors", [])
    x_values: list[float] = []
    y_values: list[float] = []
    for floor in floors:
        for key in ("corridor_bounds", "lobby_bounds", "elevator_bounds"):
            bounds = floor.get(key, {})
            x_values.extend([bounds.get("x_min", 0.0), bounds.get("x_max", 0.0)])
            y_values.extend([bounds.get("y_min", 0.0), bounds.get("y_max", 0.0)])
    entrance = data.get("entrance_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    if len(entrance) >= 2:
        x_values.append(float(entrance[0]))
        y_values.append(float(entrance[1]))
    if not x_values or not y_values:
        return default
    # 入口可能在楼层边界之外，额外放宽余量
    margin = 5.0
    return {
        "x_min": min(x_values) - margin,
        "x_max": max(x_values) + margin,
        "y_min": min(y_values) - margin,
        "y_max": max(y_values) + margin,
    }


def extract_run_metrics(run_dir: str | Path, ref: str = "") -> RunMetrics:
    """从 run 归档目录提取完整回归指标。"""
    run_path = Path(run_dir)
    metrics = RunMetrics(run_id=run_path.name, ref=ref)

    mission = read_json(run_path / "mission_summary.json", default={}) or {}
    runtime = read_json(run_path / "mission_runtime_summary.json", default={}) or {}
    evaluation = read_json(run_path / "evaluation.json", default={}) or {}

    metrics.seed = str(mission.get("seed", ""))
    metrics.completed = bool(mission.get("completed", False))
    metrics.return_home_success = bool(mission.get("return_home_success", False))
    metrics.terminal_reason = str(mission.get("runtime_terminal_reason", ""))
    metrics.failure_reasons = list(_as_list(mission.get("failure_reason", [])))

    metrics.exploration_time = float(mission.get("exploration_time", 0.0))
    metrics.max_home_distance = float(runtime.get("max_home_distance", 0.0))
    metrics.known_area = float(runtime.get("known_area", 0.0))
    metrics.visited_area = float(runtime.get("visited_area", 0.0))
    metrics.coverage_ratio = float(runtime.get("coverage_ratio", 0.0))
    metrics.confirmed_count = int(runtime.get("confirmed_count", 0) or 0)

    eval_metrics = evaluation.get("metrics", {})
    metrics.correct_count = int(eval_metrics.get("correct", 0) or 0)
    metrics.missed_count = int(eval_metrics.get("missed", 0) or 0)
    metrics.false_alarm_count = int(eval_metrics.get("false_alarms", 0) or 0)

    controller_debug = runtime.get("controller_debug", {}) or {}
    metrics.stuck_count = int(controller_debug.get("stuck_count", 0) or 0)
    metrics.recover_count = int(controller_debug.get("recover_count", 0) or 0)
    exploration_minutes = max(metrics.exploration_time / 60.0, 1e-6)
    metrics.stuck_per_minute = metrics.stuck_count / exploration_minutes
    metrics.recover_per_minute = metrics.recover_count / exploration_minutes

    # 轨迹指标
    trajectory = load_trajectory(run_path / "logs" / "danger_search.log")
    home_pose = runtime.get("home_pose")
    if home_pose is None:
        home_pose = (0.0, 0.0, 0.0)
    metrics.trajectory = compute_trajectory_metrics(trajectory, tuple(home_pose))

    # 硬失败判定
    layout_path = run_path.parents[2] / "generated_building" / "layout_metadata.json"
    bounds = extract_floor_bounds(layout_path)

    metrics.return_failed = not metrics.return_home_success and metrics.terminal_reason != ""
    # 严重卡死以控制器显式 STUCK 计数为主；长时静止可能包含正常转向/等待，不作为硬失败阈值
    metrics.severe_stuck = metrics.stuck_count >= 10
    metrics.wrong_exit = detect_wrong_exit(trajectory, bounds)
    metrics.state_loop = detect_state_loop(runtime.get("transition_history", []))

    # 门任务失败：日志中没有成功过门记录且存在 OpenDoorAndPass 任务但返航失败或探索异常短
    log_text = ""
    log_path = run_path / "logs" / "danger_search.log"
    if log_path.exists():
        log_text = log_path.read_text(encoding="utf-8", errors="ignore")
    door_pass_success = "完成过门" in log_text
    has_door_task = "OpenDoorAndPass" in log_text
    metrics.door_pass_failed = has_door_task and not door_pass_success and not metrics.return_home_success

    metrics.hard_failure = (
        metrics.return_failed
        or metrics.severe_stuck
        or metrics.wrong_exit
        or metrics.state_loop
        or metrics.door_pass_failed
    )

    metrics.raw = {
        "mission": mission,
        "runtime": runtime,
        "evaluation": evaluation,
    }
    return metrics

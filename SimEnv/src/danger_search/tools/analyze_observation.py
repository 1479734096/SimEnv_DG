#!/usr/bin/env python3
"""将观察 rosbag 转为可对时的轨迹、路径、地图和运动统计。"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    import rosbag
except ImportError:  # pragma: no cover - 纯离线函数测试不要求 ROS
    rosbag = None


TOPICS = {
    "/clock",
    "/cmd_vel",
    "/Odometry_gazebo",
    "/danger_search/status",
    "/nav/map_debug",
    "/danger_search/path_raw",
    "/danger_search/path_followed",
    "/danger_search/local_target",
    "/junior_ctrl/rl_status",
    "/gazebo/performance_metrics",
    "/rosout_agg",
}

ELEVATOR_THRESHOLD_HALF_HEIGHT = 0.03
ELEVATOR_THRESHOLD_HALF_LENGTH = 0.11
A1_FOOT_COLLISION_RADIUS = 0.02
A1_TRUNK_COLLISION_SIZE = (0.267, 0.194, 0.114)
A1_LEGS = ("FR", "FL", "RR", "RL")
A1_CALF_TO_FOOT_OFFSET = 0.2
A1_FOOT_LINK_MAX_RESIDUAL = 0.08


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--exit-diagnostics", type=Path)
    parser.add_argument("--long-stop-threshold", type=float, default=3.0)
    return parser.parse_args()


def rpy_from_quaternion(quaternion) -> tuple[float, float, float]:
    sinr = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    cosr = 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return (roll, pitch, math.atan2(siny, cosy))


def yaw_from_quaternion(quaternion) -> float:
    return rpy_from_quaternion(quaternion)[2]


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
    return float(ordered[index])


def path_length(points: list[list[float]]) -> float:
    return sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in zip(points, points[1:])
    )


def path_sinuosity(points: list[list[float]]) -> float | None:
    if len(points) < 2:
        return None
    direct = math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1])
    return path_length(points) / direct if direct > 1e-6 else None


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def read_jsonl(path: Path) -> list[dict[str, object]]:
    if not path.is_file():
        return []
    rows: list[dict[str, object]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number} 不是合法 JSONL：{exc}") from exc
            if isinstance(payload, dict):
                rows.append(payload)
    return rows


def _link_with_suffix(
    links: dict[str, object],
    suffix: str,
) -> dict[str, object]:
    candidates = _link_candidates(links, suffix)
    return candidates[0][1] if candidates else {}


def _link_candidates(
    links: dict[str, object],
    suffix: str,
) -> list[tuple[str, dict[str, object]]]:
    pattern = re.compile(rf"{re.escape(suffix)}(?:#\d+)?$")
    return [
        (str(name), state)
        for name, state in links.items()
        if pattern.search(str(name)) and isinstance(state, dict)
    ]


def _calf_derived_foot_state(calf: dict[str, object]) -> dict[str, float]:
    """按官方 URDF 的 calf→foot 固定关节推导足端世界状态。"""
    required = ("x", "y", "z")
    if any(calf.get(field) is None for field in required):
        return {}
    roll = float(calf.get("roll") or 0.0)
    pitch = float(calf.get("pitch") or 0.0)
    yaw = float(calf.get("yaw") or 0.0)
    cosine_roll = math.cos(roll)
    sine_roll = math.sin(roll)
    cosine_pitch = math.cos(pitch)
    sine_pitch = math.sin(pitch)
    cosine_yaw = math.cos(yaw)
    sine_yaw = math.sin(yaw)
    # 旋转矩阵第三列乘以局部偏移 (0, 0, -0.2)。
    offset = (
        -A1_CALF_TO_FOOT_OFFSET
        * (cosine_yaw * sine_pitch * cosine_roll + sine_yaw * sine_roll),
        -A1_CALF_TO_FOOT_OFFSET
        * (sine_yaw * sine_pitch * cosine_roll - cosine_yaw * sine_roll),
        -A1_CALF_TO_FOOT_OFFSET * cosine_pitch * cosine_roll,
    )
    state = {
        "x": float(calf["x"]) + offset[0],
        "y": float(calf["y"]) + offset[1],
        "z": float(calf["z"]) + offset[2],
    }
    if all(calf.get(f"linear_{axis}") is not None for axis in ("x", "y", "z")):
        angular = tuple(float(calf.get(f"angular_{axis}") or 0.0) for axis in ("x", "y", "z"))
        cross = (
            angular[1] * offset[2] - angular[2] * offset[1],
            angular[2] * offset[0] - angular[0] * offset[2],
            angular[0] * offset[1] - angular[1] * offset[0],
        )
        for index, axis in enumerate(("x", "y", "z")):
            state[f"linear_{axis}"] = float(calf[f"linear_{axis}"]) + cross[index]
    return state


def _select_foot_state(
    links: dict[str, object],
    leg: str,
) -> tuple[dict[str, object], str, float | str]:
    calf = _link_with_suffix(links, f"a1_gazebo::{leg}_calf")
    derived = _calf_derived_foot_state(calf)
    candidates = _link_candidates(links, f"a1_gazebo::{leg}_foot")
    if not derived:
        return (candidates[0][1], "link_state", "") if candidates else ({}, "missing", "")

    def residual(candidate: tuple[str, dict[str, object]]) -> float:
        state = candidate[1]
        try:
            return math.sqrt(
                sum((float(state[axis]) - float(derived[axis])) ** 2 for axis in ("x", "y", "z"))
            )
        except (KeyError, TypeError, ValueError):
            return math.inf

    if candidates:
        best = min(candidates, key=residual)
        best_residual = residual(best)
        if best_residual <= A1_FOOT_LINK_MAX_RESIDUAL:
            return best[1], "link_state", best_residual
        return derived, "calf_kinematic_fallback", best_residual
    return derived, "calf_kinematic_fallback", ""


def _oriented_box_bottom_z(
    state: dict[str, object],
    size_xyz: tuple[float, float, float],
) -> float | None:
    if state.get("z") is None:
        return None
    roll = float(state.get("roll") or 0.0)
    pitch = float(state.get("pitch") or 0.0)
    length, width, height = size_xyz
    vertical_half_extent = (
        abs(math.sin(pitch)) * length * 0.5
        + abs(math.cos(pitch) * math.sin(roll)) * width * 0.5
        + abs(math.cos(pitch) * math.cos(roll)) * height * 0.5
    )
    return float(state["z"]) - vertical_half_extent


def _relative_height(value: float | None, reference: float | None) -> float | str:
    if value is None or reference is None:
        return ""
    return value - reference


def flatten_exit_sample(sample: dict[str, object]) -> dict[str, object]:
    status = sample.get("status") if isinstance(sample.get("status"), dict) else {}
    odom = sample.get("odom") if isinstance(sample.get("odom"), dict) else {}
    command = sample.get("cmd_vel") if isinstance(sample.get("cmd_vel"), dict) else {}
    models = sample.get("model_poses") if isinstance(sample.get("model_poses"), dict) else {}
    links_value = sample.get("link_states", sample.get("link_poses"))
    links = links_value if isinstance(links_value, dict) else {}
    contacts = sample.get("foot_contacts") if isinstance(sample.get("foot_contacts"), dict) else {}
    cloud = sample.get("forward_cloud") if isinstance(sample.get("forward_cloud"), dict) else {}
    depth = sample.get("depth_roi") if isinstance(sample.get("depth_roi"), dict) else {}
    controller = status.get("controller") if isinstance(status.get("controller"), dict) else {}
    followed_path = (
        sample.get("followed_path") if isinstance(sample.get("followed_path"), dict) else {}
    )
    local_target = (
        sample.get("local_target") if isinstance(sample.get("local_target"), dict) else {}
    )
    floors = sample.get("elevator_floors")
    target_floor = floors[1] if isinstance(floors, (list, tuple)) and len(floors) >= 2 else None
    elevator_pose = models.get("dynamic_elevator_main", {})
    target_door_prefix = (
        f"dynamic_elevator_floor_{target_floor}::" if target_floor is not None else ""
    )
    target_panels = {
        side: next(
            (
                pose
                for name, pose in links.items()
                if target_door_prefix
                and str(name).startswith(target_door_prefix)
                and str(name).endswith(f"{side}_panel")
            ),
            {},
        )
        for side in ("left", "right")
    }
    threshold = _link_with_suffix(links, f"elevator_threshold_floor_{target_floor}")
    threshold_top_z = (
        float(threshold["z"]) + ELEVATOR_THRESHOLD_HALF_HEIGHT
        if threshold.get("z") is not None
        else None
    )
    body_link_name = next(
        (str(name) for name in links if str(name).endswith("a1_gazebo::trunk")),
        "",
    )
    if not body_link_name:
        body_link_name = next(
            (str(name) for name in links if str(name).endswith("a1_gazebo::base")),
            "",
        )
    trunk_value = links.get(body_link_name, {}) if body_link_name else {}
    trunk = trunk_value if isinstance(trunk_value, dict) else {}
    trunk_bottom_z = _oriented_box_bottom_z(trunk, A1_TRUNK_COLLISION_SIZE)
    path_goal = followed_path.get("goal_xy")
    if not isinstance(path_goal, (list, tuple)) or len(path_goal) < 2:
        path_goal = None
    row: dict[str, object] = {
        "sim_time": sample.get("sim_time", ""),
        "task": status.get("current_task", ""),
        "phase": status.get("task_phase", ""),
        "current_floor": status.get("floor", ""),
        "target_floor": target_floor if target_floor is not None else "",
        "robot_x": odom.get("x", ""),
        "robot_y": odom.get("y", ""),
        "robot_z": odom.get("z", ""),
        "robot_roll": odom.get("roll", ""),
        "robot_pitch": odom.get("pitch", ""),
        "robot_yaw": odom.get("yaw", ""),
        "odom_linear_x": odom.get("linear_x", ""),
        "odom_linear_y": odom.get("linear_y", ""),
        "cmd_linear_x": command.get("linear_x", ""),
        "cmd_linear_y": command.get("linear_y", ""),
        "cmd_angular_z": command.get("angular_z", ""),
        "exit_target_x": path_goal[0] if path_goal is not None else "",
        "exit_target_y": path_goal[1] if path_goal is not None else "",
        "local_target_x": local_target.get("x", ""),
        "local_target_y": local_target.get("y", ""),
        "recover_state": controller.get("recover_state", ""),
        "recover_reason": controller.get("recover_reason", ""),
        "recover_remaining": controller.get("recover_remaining", ""),
        "stuck_count": controller.get("stuck_count", ""),
        "recover_count": controller.get("recover_count", ""),
        "elevator_car_z": elevator_pose.get("z", "")
        if isinstance(elevator_pose, dict)
        else "",
        "target_door_panel_count": sum(bool(pose) for pose in target_panels.values()),
        "target_left_panel_x": target_panels["left"].get("x", ""),
        "target_left_panel_y": target_panels["left"].get("y", ""),
        "target_right_panel_x": target_panels["right"].get("x", ""),
        "target_right_panel_y": target_panels["right"].get("y", ""),
        "threshold_x": threshold.get("x", ""),
        "threshold_y": threshold.get("y", ""),
        "threshold_center_z": threshold.get("z", ""),
        "threshold_top_z": threshold_top_z if threshold_top_z is not None else "",
        "body_link_name": body_link_name,
        "trunk_x": trunk.get("x", ""),
        "trunk_y": trunk.get("y", ""),
        "trunk_z": trunk.get("z", ""),
        "trunk_roll": trunk.get("roll", ""),
        "trunk_pitch": trunk.get("pitch", ""),
        "trunk_yaw": trunk.get("yaw", ""),
        "trunk_linear_x": trunk.get("linear_x", ""),
        "trunk_linear_y": trunk.get("linear_y", ""),
        "trunk_linear_z": trunk.get("linear_z", ""),
        "trunk_angular_x": trunk.get("angular_x", ""),
        "trunk_angular_y": trunk.get("angular_y", ""),
        "trunk_angular_z": trunk.get("angular_z", ""),
        "trunk_bottom_z": trunk_bottom_z if trunk_bottom_z is not None else "",
        "trunk_bottom_relative_threshold_top": _relative_height(
            trunk_bottom_z,
            threshold_top_z,
        ),
        "forward_point_count": cloud.get("point_count", ""),
        "forward_world_z_span": cloud.get("world_z_span", ""),
        "depth_valid_count": depth.get("valid_count", ""),
        "depth_median_m": depth.get("median_m", ""),
        "exit_outcome": sample.get("exit_outcome") or "",
    }
    for foot in ("FR", "FL", "RR", "RL"):
        contact = contacts.get(foot, {}) if isinstance(contacts, dict) else {}
        row[f"{foot}_force_norm"] = contact.get("force_norm", "") if isinstance(contact, dict) else ""
        force = contact.get("force", []) if isinstance(contact, dict) else []
        row[f"{foot}_force_z"] = force[2] if isinstance(force, list) and len(force) >= 3 else ""
        foot_state, foot_source, foot_residual = _select_foot_state(links, foot)
        calf_state = _link_with_suffix(links, f"a1_gazebo::{foot}_calf")
        foot_bottom_z = (
            float(foot_state["z"]) - A1_FOOT_COLLISION_RADIUS
            if foot_state.get("z") is not None
            else None
        )
        for axis in ("x", "y", "z"):
            row[f"{foot}_foot_{axis}"] = foot_state.get(axis, "")
            row[f"{foot}_foot_linear_{axis}"] = foot_state.get(f"linear_{axis}", "")
            row[f"{foot}_calf_{axis}"] = calf_state.get(axis, "")
        row[f"{foot}_foot_position_source"] = foot_source
        row[f"{foot}_foot_link_residual"] = foot_residual
        row[f"{foot}_foot_bottom_z"] = foot_bottom_z if foot_bottom_z is not None else ""
        row[f"{foot}_foot_bottom_relative_threshold_top"] = _relative_height(
            foot_bottom_z,
            threshold_top_z,
        )
    return row


def add_exit_derivatives(rows: list[dict[str, object]]) -> None:
    previous_time: float | None = None
    previous_velocity: float | None = None
    for row in rows:
        row["trunk_linear_ax"] = ""
        try:
            current_time = float(row["sim_time"])
            current_velocity = float(row["trunk_linear_x"])
        except (KeyError, TypeError, ValueError):
            previous_time = None
            previous_velocity = None
            continue
        if previous_time is not None and previous_velocity is not None:
            delta_time = current_time - previous_time
            if delta_time > 1e-6:
                row["trunk_linear_ax"] = (current_velocity - previous_velocity) / delta_time
        previous_time = current_time
        previous_velocity = current_velocity


def parse_rl_status_command(message: str) -> dict[str, object] | None:
    """解析 RL 控制器实际消费的速度命令，而非仅解析上游配置值。"""
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    match = re.search(rf"cmd=\(\s*({number})\s*,\s*({number})\s*,\s*({number})\s*\)", message)
    if match is None:
        return None

    def flag(name: str) -> bool | None:
        value = re.search(rf"\b{re.escape(name)}=([01])\b", message)
        return bool(int(value.group(1))) if value is not None else None

    return {
        "linear_x": float(match.group(1)),
        "linear_y": float(match.group(2)),
        "angular_z": float(match.group(3)),
        "timed_out": flag("cmd_timed_out"),
        "clamped": flag("cmd_clamped"),
    }


def _float_or_none(value) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def summarize_exit_ab_metrics(
    rows: list[dict[str, object]],
    received_commands: list[dict[str, object]],
) -> dict[str, object]:
    """汇总首次 EXIT_CABIN 通过阶段的连续 A/B 指标。"""
    first_attempt: list[dict[str, object]] = []
    started = False
    for row in rows:
        if str(row.get("phase") or "") == "EXIT_CABIN":
            started = True
            first_attempt.append(row)
        elif started:
            break

    if not first_attempt:
        return {
            "exit_first_attempt_seen": False,
            "metric_semantics": "仅统计首次连续 EXIT_CABIN 窗口",
        }

    start_time = _float_or_none(first_attempt[0].get("sim_time"))
    end_time = _float_or_none(first_attempt[-1].get("sim_time"))
    targets = [
        value
        for value in (_float_or_none(row.get("exit_target_x")) for row in first_attempt)
        if value is not None
    ]
    thresholds = [
        value
        for value in (_float_or_none(row.get("threshold_x")) for row in first_attempt)
        if value is not None
    ]
    target_x = statistics.median(targets) if targets else None
    threshold_x = statistics.median(thresholds) if thresholds else None
    exit_direction = None
    if target_x is not None and threshold_x is not None and abs(target_x - threshold_x) > 1e-6:
        exit_direction = 1.0 if target_x > threshold_x else -1.0

    threshold_duration = 0.0
    no_x_motion_duration = 0.0
    rebound_distance = 0.0
    rebound_count = 0
    in_rebound = False
    for row, next_row in zip(first_attempt, first_attempt[1:]):
        current_time = _float_or_none(row.get("sim_time"))
        next_time = _float_or_none(next_row.get("sim_time"))
        if current_time is None or next_time is None:
            continue
        delta_time = next_time - current_time
        if delta_time <= 0.0 or delta_time > 1.0:
            continue
        robot_x = _float_or_none(row.get("robot_x"))
        if robot_x is not None and 1.35 <= robot_x <= 1.55:
            threshold_duration += delta_time
        command_x = _float_or_none(row.get("cmd_linear_x")) or 0.0
        odom_x = _float_or_none(row.get("odom_linear_x")) or 0.0
        if abs(command_x) > 0.1 and abs(odom_x) < 0.02:
            no_x_motion_duration += delta_time

        next_x = _float_or_none(next_row.get("robot_x"))
        if exit_direction is not None and robot_x is not None and next_x is not None:
            progress_delta = exit_direction * (next_x - robot_x)
            reversing = progress_delta < -0.005
            if reversing:
                rebound_distance += -progress_delta
                if not in_rebound:
                    rebound_count += 1
            in_rebound = reversing

    def crossing_time(legs: tuple[str, ...]) -> float | None:
        if exit_direction is None or threshold_x is None:
            return None
        far_edge = threshold_x + exit_direction * ELEVATOR_THRESHOLD_HALF_LENGTH
        for row in first_attempt:
            positions = [_float_or_none(row.get(f"{leg}_foot_x")) for leg in legs]
            if all(
                position is not None and exit_direction * (position - far_edge) >= 0.0
                for position in positions
            ):
                return _float_or_none(row.get("sim_time"))
        return None

    front_crossing_time = crossing_time(("FR", "FL"))
    rear_crossing_time = crossing_time(("RR", "RL"))
    received = [
        row
        for row in received_commands
        if start_time is not None
        and end_time is not None
        and (stamp := _float_or_none(row.get("timestamp"))) is not None
        and start_time <= stamp <= end_time
    ]
    received_linear = [
        abs(value)
        for value in (_float_or_none(row.get("linear_x")) for row in received)
        if value is not None
    ]
    rolls = [
        abs(value)
        for value in (_float_or_none(row.get("robot_roll")) for row in first_attempt)
        if value is not None
    ]
    pitches = [
        abs(value)
        for value in (_float_or_none(row.get("robot_pitch")) for row in first_attempt)
        if value is not None
    ]
    lateral_positions = [
        value
        for value in (_float_or_none(row.get("robot_y")) for row in first_attempt)
        if value is not None
    ]
    outcome = next(
        (str(row.get("exit_outcome")) for row in reversed(first_attempt) if row.get("exit_outcome")),
        None,
    )
    return {
        "exit_first_attempt_seen": True,
        "exit_outcome": outcome,
        "exit_first_attempt_start": start_time,
        "exit_first_attempt_end": end_time,
        "exit_first_attempt_duration": (
            end_time - start_time if start_time is not None and end_time is not None else None
        ),
        "threshold_region_duration": threshold_duration,
        "commanded_no_x_motion_duration": no_x_motion_duration,
        "front_feet_crossing_time": front_crossing_time,
        "rear_feet_crossing_time": rear_crossing_time,
        "front_to_rear_crossing_duration": (
            rear_crossing_time - front_crossing_time
            if front_crossing_time is not None and rear_crossing_time is not None
            else None
        ),
        "max_abs_roll": max(rolls, default=None),
        "max_abs_pitch": max(pitches, default=None),
        "max_lateral_deviation": (
            max(abs(value - lateral_positions[0]) for value in lateral_positions)
            if lateral_positions
            else None
        ),
        "rebound_distance": rebound_distance,
        "rebound_count": rebound_count,
        "received_cmd_sample_count": len(received_linear),
        "received_cmd_linear_mean": (
            statistics.fmean(received_linear) if received_linear else None
        ),
        "received_cmd_linear_max": max(received_linear, default=None),
        "received_cmd_timed_out_count": sum(row.get("timed_out") is True for row in received),
        "received_cmd_clamped_count": sum(row.get("clamped") is True for row in received),
        "metric_semantics": (
            "仅统计首次连续 EXIT_CABIN；门槛区为机身 x∈[1.35,1.55]；"
            "有命令无 x 位移为 |cmd_x|>0.1 且 |odom_vx|<0.02；反弹按出口反方向位移>0.005m 分段"
        ),
    }


def summarize_time_breakdown(
    trajectory: list[dict[str, object]],
    structured_events: list[dict[str, object]],
) -> dict[str, object]:
    durations = {
        "exit_cabin_duration": 0.0,
        "rotate_in_place_duration": 0.0,
        "fully_stationary_duration": 0.0,
        "commanded_no_motion_duration": 0.0,
        "elevator_door_wait_duration": 0.0,
        "recover_duration": 0.0,
        "effective_translation_duration": 0.0,
    }
    phase_durations: dict[str, float] = {}
    wait_phases = {
        "CALL_ELEVATOR_TO_CURRENT_FLOOR",
        "OPEN_CURRENT_FLOOR_DOOR",
        "WAIT_CURRENT_DOOR_OPEN",
        "CLOSE_CURRENT_FLOOR_DOOR",
        "CALL_ELEVATOR_TO_TARGET_FLOOR",
        "WAIT_FOR_ODOM_STABLE",
        "OPEN_TARGET_FLOOR_DOOR",
        "WAIT_TARGET_DOOR_OPEN",
        "CLOSE_TARGET_FLOOR_DOOR",
    }
    for row, next_row in zip(trajectory, trajectory[1:]):
        delta = float(next_row["timestamp"]) - float(row["timestamp"])
        if delta <= 0.0 or delta > 1.0:
            continue
        phase = str(row.get("task_phase") or "")
        task = str(row.get("task") or "")
        actual_linear = float(row.get("actual_linear_speed") or 0.0)
        actual_angular = abs(float(row.get("actual_angular_speed") or 0.0))
        command_linear = math.hypot(
            float(row.get("cmd_linear_x") or 0.0),
            float(row.get("cmd_linear_y") or 0.0),
        )
        command_angular = abs(float(row.get("cmd_angular_z") or 0.0))
        recover_state = str(row.get("recover_state") or "idle")
        if task.startswith("TakeElevator(") and phase:
            phase_durations[phase] = phase_durations.get(phase, 0.0) + delta
        if phase == "EXIT_CABIN" or phase.startswith("EXIT_BACKOFF_"):
            durations["exit_cabin_duration"] += delta
        if command_linear < 0.03 and command_angular >= 0.08:
            durations["rotate_in_place_duration"] += delta
        if (
            actual_linear < 0.03
            and actual_angular < 0.05
            and command_linear < 0.03
            and command_angular < 0.05
        ):
            durations["fully_stationary_duration"] += delta
        if command_linear >= 0.05 and actual_linear < 0.03:
            durations["commanded_no_motion_duration"] += delta
        if task.startswith("TakeElevator(") and phase in wait_phases:
            durations["elevator_door_wait_duration"] += delta
        if recover_state not in {"", "idle"} or phase.startswith("EXIT_BACKOFF_"):
            durations["recover_duration"] += delta
        if actual_linear >= 0.05:
            durations["effective_translation_duration"] += delta
    planning_durations = [
        float(event.get("selection_wall_duration") or 0.0)
        for event in structured_events
        if event.get("event") == "frontier_selection"
    ]
    return {
        **durations,
        "planning_wall_duration": sum(planning_durations),
        "planning_wall_duration_max": max(planning_durations, default=0.0),
        "elevator_phase_durations": dict(sorted(phase_durations.items())),
        "duration_semantics": "各类别独立统计，允许重叠；轨迹时间为仿真时间，规划为 wall time",
    }


def wall_duration(manifest_path: Path | None) -> float | None:
    if manifest_path is None or not manifest_path.is_file():
        return None
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not payload.get("start_wall") or not payload.get("end_wall"):
        return None
    def parse_wall_stamp(value: str) -> datetime:
        # Python 3.8 只接受微秒，GNU date --iso-8601=ns 输出 9 位纳秒。
        normalized = re.sub(r"([.,]\d{6})\d+", r"\1", value).replace(",", ".", 1)
        return datetime.fromisoformat(normalized)

    start = parse_wall_stamp(payload["start_wall"])
    end = parse_wall_stamp(payload["end_wall"])
    return max(0.0, (end - start).total_seconds())


def nearest_row(rows: list[dict[str, object]], stamp: float) -> dict[str, object] | None:
    if not rows:
        return None
    return min(rows, key=lambda row: abs(float(row["timestamp"]) - stamp))


def main() -> int:
    args = parse_args()
    if rosbag is None:
        raise RuntimeError("当前环境没有 rosbag，无法分析观察数据")
    output_dir = args.output_dir or args.bag.parent / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.manifest or args.bag.parent / "manifest.json"
    exit_diagnostics_path = (
        args.exit_diagnostics or args.bag.parent / "exit_cabin_diagnostics.jsonl"
    )
    exit_samples = read_jsonl(exit_diagnostics_path)
    exit_timeline = [flatten_exit_sample(sample) for sample in exit_samples]
    add_exit_derivatives(exit_timeline)

    trajectory: list[dict[str, object]] = []
    commands: list[dict[str, object]] = []
    statuses: list[dict[str, object]] = []
    maps: list[dict[str, object]] = []
    local_targets: list[dict[str, object]] = []
    raw_paths: list[dict[str, object]] = []
    followed_paths: list[dict[str, object]] = []
    structured_events: list[dict[str, object]] = []
    performance: list[dict[str, object]] = []
    received_rl_commands: list[dict[str, object]] = []
    latest_status: dict[str, object] = {}
    latest_command = (0.0, 0.0, 0.0)
    first_clock: float | None = None
    last_clock: float | None = None
    last_map_message = None

    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, receive_time in bag.read_messages(topics=TOPICS):
            stamp = receive_time.to_sec()
            if topic == "/clock":
                clock = message.clock.to_sec()
                first_clock = clock if first_clock is None else first_clock
                last_clock = clock
            elif topic == "/cmd_vel":
                latest_command = (
                    float(message.linear.x),
                    float(message.linear.y),
                    float(message.angular.z),
                )
                commands.append(
                    {
                        "timestamp": stamp,
                        "linear_x": latest_command[0],
                        "linear_y": latest_command[1],
                        "angular_z": latest_command[2],
                        "linear_speed": math.hypot(latest_command[0], latest_command[1]),
                    }
                )
            elif topic == "/danger_search/status":
                try:
                    latest_status = json.loads(message.data)
                except (json.JSONDecodeError, TypeError):
                    latest_status = {"parse_error": str(message.data)}
                statuses.append({"timestamp": stamp, **latest_status})
            elif topic == "/junior_ctrl/rl_status":
                command = parse_rl_status_command(str(message.data))
                if command is not None:
                    received_rl_commands.append({"timestamp": stamp, **command})
            elif topic == "/Odometry_gazebo":
                position = message.pose.pose.position
                twist = message.twist.twist
                roll, pitch, yaw = rpy_from_quaternion(message.pose.pose.orientation)
                controller = latest_status.get("controller") or {}
                goal = latest_status.get("current_goal")
                trajectory.append(
                    {
                        "timestamp": stamp,
                        "x": float(position.x),
                        "y": float(position.y),
                        "z": float(position.z),
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "state": latest_status.get("state", ""),
                        "task": latest_status.get("current_task") or "",
                        "task_phase": latest_status.get("task_phase") or "",
                        "goal_x": goal[0] if isinstance(goal, list) and len(goal) >= 2 else "",
                        "goal_y": goal[1] if isinstance(goal, list) and len(goal) >= 2 else "",
                        "actual_linear_speed": math.hypot(float(twist.linear.x), float(twist.linear.y)),
                        "actual_angular_speed": float(twist.angular.z),
                        "cmd_linear_x": latest_command[0],
                        "cmd_linear_y": latest_command[1],
                        "cmd_angular_z": latest_command[2],
                        "zero_command_reason": controller.get("zero_command_reason") or "",
                        "recover_state": controller.get("recover_state") or "",
                        "heading_error": controller.get("heading_error", ""),
                        "path_curvature": controller.get("path_curvature", ""),
                        "nearest_obstacle_distance": controller.get(
                            "nearest_obstacle_distance", ""
                        ),
                    }
                )
            elif topic in ("/danger_search/path_raw", "/danger_search/path_followed"):
                points = [
                    [float(pose.pose.position.x), float(pose.pose.position.y)]
                    for pose in message.poses
                ]
                row = {
                    "timestamp": stamp,
                    "frame_id": message.header.frame_id,
                    "points": points,
                    "length": path_length(points),
                    "sinuosity": path_sinuosity(points),
                }
                (raw_paths if topic.endswith("path_raw") else followed_paths).append(row)
            elif topic == "/danger_search/local_target":
                local_targets.append(
                    {
                        "timestamp": stamp,
                        "x": float(message.pose.position.x),
                        "y": float(message.pose.position.y),
                    }
                )
            elif topic == "/nav/map_debug":
                data = np.asarray(message.data)
                resolution = float(message.info.resolution)
                maps.append(
                    {
                        "timestamp": stamp,
                        "known_cells": int(np.count_nonzero(data != -1)),
                        "unknown_cells": int(np.count_nonzero(data == -1)),
                        "free_cells": int(np.count_nonzero(data == 0)),
                        "visited_cells": int(np.count_nonzero(data == 80)),
                        "occupied_cells": int(np.count_nonzero(data == 100)),
                        "known_area": float(np.count_nonzero(data != -1) * resolution * resolution),
                    }
                )
                last_map_message = message
            elif topic == "/gazebo/performance_metrics":
                performance.append(
                    {
                        "timestamp": stamp,
                        "real_time_factor": float(getattr(message, "real_time_factor", math.nan)),
                        "sim_time": getattr(message, "sim_time", None).to_sec()
                        if hasattr(getattr(message, "sim_time", None), "to_sec")
                        else None,
                        "real_time": getattr(message, "real_time", None).to_sec()
                        if hasattr(getattr(message, "real_time", None), "to_sec")
                        else None,
                    }
                )
            elif topic == "/rosout_agg":
                text = str(message.msg).strip()
                if not text.startswith("{"):
                    continue
                try:
                    event = json.loads(text)
                except json.JSONDecodeError:
                    continue
                structured_events.append({"timestamp": stamp, **event})

    long_stops: list[dict[str, object]] = []
    stop_start: int | None = None
    for index, row in enumerate(trajectory):
        stopped = (
            row["state"] == "EXPLORE_FLOOR"
            and float(row["actual_linear_speed"]) < 0.03
            and abs(float(row["actual_angular_speed"])) < 0.05
        )
        if stopped and stop_start is None:
            stop_start = index
        if (not stopped or index == len(trajectory) - 1) and stop_start is not None:
            end_index = index if stopped else index - 1
            duration = float(trajectory[end_index]["timestamp"]) - float(
                trajectory[stop_start]["timestamp"]
            )
            if duration >= args.long_stop_threshold:
                start_row = trajectory[stop_start]
                long_stops.append(
                    {
                        "start": start_row["timestamp"],
                        "end": trajectory[end_index]["timestamp"],
                        "duration": duration,
                        "x": start_row["x"],
                        "y": start_row["y"],
                        "reason": start_row["zero_command_reason"] or "UNKNOWN",
                        "state": start_row["state"],
                    }
                )
            stop_start = None

    actual_distance = 0.0
    repeated_distance = 0.0
    visited_bins: set[tuple[int, int]] = set()
    for start, end in zip(trajectory, trajectory[1:]):
        segment = math.hypot(float(end["x"]) - float(start["x"]), float(end["y"]) - float(start["y"]))
        if segment > 1.0:
            continue
        actual_distance += segment
        cell = (round(float(end["x"]) / 0.5), round(float(end["y"]) / 0.5))
        if cell in visited_bins:
            repeated_distance += segment
        visited_bins.add(cell)

    cmd_speeds = [float(row["linear_speed"]) for row in commands]
    odom_speeds = [float(row["actual_linear_speed"]) for row in trajectory]
    raw_sinuosity = [float(row["sinuosity"]) for row in raw_paths if row["sinuosity"] is not None]
    followed_sinuosity = [
        float(row["sinuosity"]) for row in followed_paths if row["sinuosity"] is not None
    ]
    time_breakdown = summarize_time_breakdown(trajectory, structured_events)
    exit_ab_metrics = summarize_exit_ab_metrics(exit_timeline, received_rl_commands)
    sim_duration = (
        last_clock - first_clock
        if first_clock is not None and last_clock is not None
        else None
    )
    recording_wall_duration = wall_duration(manifest_path)
    measured_rtf = (
        sim_duration / recording_wall_duration
        if sim_duration is not None and recording_wall_duration and recording_wall_duration > 0.0
        else None
    )
    reported_rtf = [
        float(row["real_time_factor"])
        for row in performance
        if math.isfinite(float(row["real_time_factor"]))
    ]
    metrics = {
        "bag": str(args.bag),
        "sim_duration": sim_duration,
        "wall_recording_duration": recording_wall_duration,
        "measured_real_time_factor": measured_rtf,
        "reported_real_time_factor_median": statistics.median(reported_rtf)
        if reported_rtf
        else None,
        "trajectory_samples": len(trajectory),
        "actual_distance": actual_distance,
        "repeat_distance_ratio_0_5m_bins": repeated_distance / actual_distance
        if actual_distance > 1e-6
        else None,
        "cmd_linear_mean": statistics.fmean(cmd_speeds) if cmd_speeds else None,
        "cmd_linear_median": statistics.median(cmd_speeds) if cmd_speeds else None,
        "cmd_linear_p95": percentile(cmd_speeds, 0.95),
        "odom_linear_mean": statistics.fmean(odom_speeds) if odom_speeds else None,
        "odom_linear_median": statistics.median(odom_speeds) if odom_speeds else None,
        "odom_linear_p95": percentile(odom_speeds, 0.95),
        "odom_stopped_sample_ratio": (
            sum(speed < 0.03 for speed in odom_speeds) / len(odom_speeds)
            if odom_speeds
            else None
        ),
        "raw_path_count": sum(bool(row["points"]) for row in raw_paths),
        "followed_path_count": sum(bool(row["points"]) for row in followed_paths),
        "raw_path_sinuosity_median": statistics.median(raw_sinuosity)
        if raw_sinuosity
        else None,
        "followed_path_sinuosity_median": statistics.median(followed_sinuosity)
        if followed_sinuosity
        else None,
        "long_stop_count": len(long_stops),
        "long_stop_total_duration": sum(float(row["duration"]) for row in long_stops),
        "long_stop_over_12s_count": sum(float(row["duration"]) >= 12.0 for row in long_stops),
        "state_transition_count": sum(
            event.get("event") == "mission_state_transition" for event in structured_events
        ),
        "frontier_selection_count": sum(
            event.get("event") == "frontier_selection" for event in structured_events
        ),
        "exit_diagnostic_sample_count": len(exit_samples),
        "exit_cabin_seen": any(bool(sample.get("seen_exit_cabin")) for sample in exit_samples),
        "exit_cabin_outcome": next(
            (
                str(sample.get("exit_outcome"))
                for sample in reversed(exit_samples)
                if sample.get("exit_outcome")
            ),
            None,
        ),
        "exit_diagnostic_has_door_panels": any(
            int(row.get("target_door_panel_count") or 0) > 0 for row in exit_timeline
        ),
        "exit_diagnostic_has_foot_contacts": any(
            any(row.get(f"{foot}_force_norm") != "" for foot in ("FR", "FL", "RR", "RL"))
            for row in exit_timeline
        ),
        "exit_diagnostic_has_nonzero_foot_force": any(
            any(float(row.get(f"{foot}_force_norm") or 0.0) > 1e-6 for foot in ("FR", "FL", "RR", "RL"))
            for row in exit_timeline
        ),
        "exit_diagnostic_has_forward_cloud": any(
            row.get("forward_point_count") != "" for row in exit_timeline
        ),
        "exit_diagnostic_has_depth": any(
            row.get("depth_valid_count") != "" for row in exit_timeline
        ),
        "time_breakdown": time_breakdown,
        "exit_ab_metrics": exit_ab_metrics,
    }

    write_csv(output_dir / "trajectory.csv", trajectory)
    write_csv(output_dir / "cmd_vel.csv", commands)
    write_csv(output_dir / "status.csv", statuses)
    write_csv(output_dir / "map_changes.csv", maps)
    write_csv(output_dir / "local_targets.csv", local_targets)
    write_csv(output_dir / "long_stops.csv", long_stops)
    write_csv(output_dir / "exit_cabin_timeline.csv", exit_timeline)
    write_csv(output_dir / "rl_cmd_received.csv", received_rl_commands)
    write_jsonl(output_dir / "path_raw.jsonl", raw_paths)
    write_jsonl(output_dir / "path_followed.jsonl", followed_paths)
    write_jsonl(output_dir / "structured_events.jsonl", structured_events)
    (output_dir / "time_breakdown.json").write_text(
        json.dumps(time_breakdown, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_dir / "exit_ab_metrics.json").write_text(
        json.dumps(exit_ab_metrics, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_dir / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    figure, axis = plt.subplots(figsize=(12, 8))
    if last_map_message is not None:
        map_data = np.asarray(last_map_message.data).reshape(
            int(last_map_message.info.height),
            int(last_map_message.info.width),
        )
        extent = [
            last_map_message.info.origin.position.x,
            last_map_message.info.origin.position.x
            + last_map_message.info.width * last_map_message.info.resolution,
            last_map_message.info.origin.position.y,
            last_map_message.info.origin.position.y
            + last_map_message.info.height * last_map_message.info.resolution,
        ]
        axis.imshow(
            map_data,
            origin="lower",
            extent=extent,
            cmap="Greys",
            alpha=0.28,
            vmin=-1,
            vmax=100,
        )
    for row in raw_paths:
        if len(row["points"]) >= 2:
            points = np.asarray(row["points"])
            axis.plot(points[:, 0], points[:, 1], color="#d97706", alpha=0.18, linewidth=0.8)
    for row in followed_paths:
        if len(row["points"]) >= 2:
            points = np.asarray(row["points"])
            axis.plot(points[:, 0], points[:, 1], color="#2563eb", alpha=0.28, linewidth=1.0)
            axis.scatter(points[-1, 0], points[-1, 1], marker="x", color="#dc2626", s=18)
    if trajectory:
        xs = [float(row["x"]) for row in trajectory]
        ys = [float(row["y"]) for row in trajectory]
        axis.plot(xs, ys, color="#111827", linewidth=1.4, label="实际轨迹")
        axis.scatter(xs[0], ys[0], color="#16a34a", s=70, marker="o", label="起点")
        axis.scatter(xs[-1], ys[-1], color="#dc2626", s=70, marker="s", label="终点")
    if long_stops:
        axis.scatter(
            [float(row["x"]) for row in long_stops],
            [float(row["y"]) for row in long_stops],
            color="#facc15",
            edgecolor="#713f12",
            s=55,
            marker="o",
            label="长时间停留",
        )
    recover_rows = [row for row in trajectory if row["recover_state"] not in ("", "idle")]
    if recover_rows:
        axis.scatter(
            [float(row["x"]) for row in recover_rows[:: max(1, len(recover_rows) // 80)]],
            [float(row["y"]) for row in recover_rows[:: max(1, len(recover_rows) // 80)]],
            color="#db2777",
            s=18,
            marker="^",
            label="recover",
        )
    for event in structured_events:
        if event.get("event") != "mission_state_transition":
            continue
        row = nearest_row(trajectory, float(event["timestamp"]))
        if row is None:
            continue
        axis.annotate(
            str(event.get("new_state", "")),
            (float(row["x"]), float(row["y"])),
            fontsize=7,
            color="#7c3aed",
        )
    axis.set_title("danger_search 轨迹、规划路径与停留点")
    axis.set_xlabel("x / m")
    axis.set_ylabel("y / m")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.18)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_dir / "trajectory.png", dpi=160)
    plt.close(figure)

    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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
import rosbag


TOPICS = {
    "/clock",
    "/cmd_vel",
    "/Odometry_gazebo",
    "/danger_search/status",
    "/nav/map_debug",
    "/danger_search/path_raw",
    "/danger_search/path_followed",
    "/danger_search/local_target",
    "/gazebo/performance_metrics",
    "/rosout_agg",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--long-stop-threshold", type=float, default=3.0)
    return parser.parse_args()


def yaw_from_quaternion(quaternion) -> float:
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


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
    output_dir = args.output_dir or args.bag.parent / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.manifest or args.bag.parent / "manifest.json"

    trajectory: list[dict[str, object]] = []
    commands: list[dict[str, object]] = []
    statuses: list[dict[str, object]] = []
    maps: list[dict[str, object]] = []
    local_targets: list[dict[str, object]] = []
    raw_paths: list[dict[str, object]] = []
    followed_paths: list[dict[str, object]] = []
    structured_events: list[dict[str, object]] = []
    performance: list[dict[str, object]] = []
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
            elif topic == "/Odometry_gazebo":
                position = message.pose.pose.position
                twist = message.twist.twist
                controller = latest_status.get("controller") or {}
                goal = latest_status.get("current_goal")
                trajectory.append(
                    {
                        "timestamp": stamp,
                        "x": float(position.x),
                        "y": float(position.y),
                        "yaw": yaw_from_quaternion(message.pose.pose.orientation),
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
    }

    write_csv(output_dir / "trajectory.csv", trajectory)
    write_csv(output_dir / "cmd_vel.csv", commands)
    write_csv(output_dir / "status.csv", statuses)
    write_csv(output_dir / "map_changes.csv", maps)
    write_csv(output_dir / "local_targets.csv", local_targets)
    write_csv(output_dir / "long_stops.csv", long_stops)
    write_jsonl(output_dir / "path_raw.jsonl", raw_paths)
    write_jsonl(output_dir / "path_followed.jsonl", followed_paths)
    write_jsonl(output_dir / "structured_events.jsonl", structured_events)
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

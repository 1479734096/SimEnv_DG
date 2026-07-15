#!/usr/bin/env python3
"""从已有 ground_support rosbag 离线提取自然停止事件的完整制动包络。

只读 bag 和分析 JSON，不运行仿真，不修改生产代码。
输出：每个事件的 roll/pitch、减速曲线、停止位移、回弹、停后迈步。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

try:
    import rosbag
    import rospy
    from nav_msgs.msg import Odometry
except ImportError:
    rosbag = None
    rospy = None
    Odometry = None


def _quaternion_rpy(q):
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _stamp_sec(msg) -> float:
    s = getattr(getattr(msg, "header", None), "stamp", None)
    fn = getattr(s, "to_sec", None)
    return float(fn()) if callable(fn) else 0.0


def _load_natural_stops(analysis_json: Path) -> list[dict]:
    data = json.loads(analysis_json.read_text(encoding="utf-8"))
    ns = data.get("natural_stop_measurements", {})
    if not ns.get("available"):
        return []
    events = ns.get("events", [])
    for e in events:
        e["source_run"] = str(analysis_json.parent.parent.parent)
    return events


def _extract_odom_window(
    bag_path: Path,
    t_start: float,
    t_end: float,
) -> list[dict]:
    samples = []
    bag = rosbag.Bag(str(bag_path), "r")
    try:
        for topic, msg, recorded_at in bag.read_messages(
            topics=["/Odometry_gazebo"],
            start_time=rospy.Time.from_sec(max(0.0, t_start)),
            end_time=rospy.Time.from_sec(t_end),
        ):
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            tw = msg.twist.twist.linear
            roll, pitch, yaw = _quaternion_rpy(o)
            stamp = float(recorded_at.to_sec())
            header_stamp = _stamp_sec(msg)
            speed = math.hypot(float(tw.x), float(tw.y))
            samples.append({
                "stamp": stamp,
                "header_stamp": header_stamp,
                "header_to_bag_delay": (
                    stamp - header_stamp if header_stamp > 0.0 else None
                ),
                "x": float(p.x),
                "y": float(p.y),
                "z": float(p.z),
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "vx": float(tw.x),
                "vy": float(tw.y),
                "speed": speed,
            })
    finally:
        bag.close()
    samples.sort(key=lambda s: s["stamp"])
    return samples


def _analyze_event(event: dict, bag_path: Path) -> dict:
    rl_zero = float(event["rl_zero_time"])
    full_stop = float(event["full_stop_time"])
    start_speed = float(event["start_actual_speed"])
    t_start = rl_zero - 0.6
    t_end = full_stop + 2.0

    samples = _extract_odom_window(bag_path, t_start, t_end)
    if not samples:
        return {**event, "extraction_error": "no_odom_in_window", "odom_sample_count": 0}

    zero_idx = min(
        range(len(samples)),
        key=lambda i: abs(samples[i]["stamp"] - rl_zero),
    )
    stop_idx = min(
        range(len(samples)),
        key=lambda i: abs(samples[i]["stamp"] - full_stop),
    )

    # 这里使用可复查的诊断定义，而不把单帧速度抖动当成开始减速：
    # 从 RL 收到零线速度后，速度相对该时刻至少下降 0.02 m/s，且后续
    # 50 ms 内不再越过该阈值，才记为 odom 开始持续减速。
    zero_speed = samples[zero_idx]["speed"]
    deceleration_threshold = max(0.0, zero_speed - 0.02)
    deceleration_start_time = None
    for index_value in range(zero_idx, len(samples)):
        sample = samples[index_value]
        if sample["stamp"] > full_stop:
            break
        window = [
            later["speed"]
            for later in samples[index_value:]
            if later["stamp"] <= sample["stamp"] + 0.05
        ]
        if (
            window
            and samples[min(len(samples) - 1, index_value + len(window) - 1)]["stamp"]
            - sample["stamp"]
            >= 0.045
            and max(window) <= deceleration_threshold
        ):
            deceleration_start_time = sample["stamp"]
            break

    start_xy = (samples[zero_idx]["x"], samples[zero_idx]["y"])
    stop_xy = (samples[stop_idx]["x"], samples[stop_idx]["y"])

    displacements = []
    for s in samples[zero_idx:]:
        d = math.hypot(s["x"] - start_xy[0], s["y"] - start_xy[1])
        displacements.append((s["stamp"], d))
    max_displacement = max(d for _, d in displacements) if displacements else 0.0

    post_stop = samples[stop_idx:]
    post_stop_displacements = []
    for s in post_stop:
        d = math.hypot(s["x"] - stop_xy[0], s["y"] - stop_xy[1])
        post_stop_displacements.append((s["stamp"], d))
    post_stop_max = max(d for _, d in post_stop_displacements) if post_stop_displacements else 0.0

    rebound = False
    rebound_distance = 0.0
    if len(post_stop_displacements) >= 2:
        peak_d = 0.0
        peak_idx = 0
        for i, (_, d) in enumerate(post_stop_displacements):
            if d > peak_d:
                peak_d = d
                peak_idx = i
        if peak_idx < len(post_stop_displacements) - 1:
            final_d = post_stop_displacements[-1][1]
            if final_d < peak_d - 0.005:
                rebound = True
                rebound_distance = peak_d - final_d

    continued_stepping = post_stop_max > 0.02

    decel_samples = samples[zero_idx:stop_idx + 1]
    roll_values = [s["roll"] for s in decel_samples]
    pitch_values = [s["pitch"] for s in decel_samples]
    post_roll = [s["roll"] for s in post_stop]
    post_pitch = [s["pitch"] for s in post_stop]

    z_values = [s["z"] for s in decel_samples]
    post_z = [s["z"] for s in post_stop]
    z_drop = (max(z_values) - min(z_values)) if z_values else 0.0
    post_z_change = (max(post_z) - min(post_z)) if post_z else 0.0

    speed_at_end = samples[-1]["speed"] if samples else 0.0
    stop_confirmation = [
        sample
        for sample in samples
        if full_stop <= sample["stamp"] <= full_stop + 0.101
    ]
    fully_stopped_confirmed = bool(
        stop_confirmation
        and stop_confirmation[-1]["stamp"] - full_stop >= 0.095
        and max(sample["speed"] for sample in stop_confirmation) <= 0.02
    )
    observation_after_stop = max(0.0, samples[-1]["stamp"] - full_stop)

    return {
        "rl_zero_time": rl_zero,
        "cmd_zero_time": float(event.get("cmd_zero_time", rl_zero)),
        "rl_receive_delay": float(event.get("rl_receive_delay_from_cmd", 0.0)),
        "start_actual_speed": start_speed,
        "full_stop_time": full_stop,
        "full_stop_delay": float(event["full_stop_delay"]),
        "full_stop_net_displacement": float(event["full_stop_net_displacement"]),
        "maximum_displacement_before_stop": float(event.get("maximum_displacement_before_stop", max_displacement)),
        "start_xy": list(start_xy),
        "stop_xy": list(stop_xy),
        "odom_sample_count": len(samples),
        "odom_speed_at_rl_zero": zero_speed,
        "deceleration_start_time": deceleration_start_time,
        "deceleration_start_delay_from_rl_zero": (
            deceleration_start_time - rl_zero
            if deceleration_start_time is not None
            else None
        ),
        "roll_during_deceleration": {
            "minimum": min(roll_values) if roll_values else None,
            "maximum": max(roll_values) if roll_values else None,
            "abs_maximum": max(abs(v) for v in roll_values) if roll_values else None,
        },
        "pitch_during_deceleration": {
            "minimum": min(pitch_values) if pitch_values else None,
            "maximum": max(pitch_values) if pitch_values else None,
            "abs_maximum": max(abs(v) for v in pitch_values) if pitch_values else None,
        },
        "roll_after_stop": {
            "minimum": min(post_roll) if post_roll else None,
            "maximum": max(post_roll) if post_roll else None,
            "abs_maximum": max(abs(v) for v in post_roll) if post_roll else None,
        },
        "pitch_after_stop": {
            "minimum": min(post_pitch) if post_pitch else None,
            "maximum": max(post_pitch) if post_pitch else None,
            "abs_maximum": max(abs(v) for v in post_pitch) if post_pitch else None,
        },
        "z_drop_during_deceleration": z_drop,
        "z_change_after_stop": post_z_change,
        "post_stop_max_displacement": post_stop_max,
        "rebound": rebound,
        "rebound_distance": rebound_distance,
        "continued_stepping": continued_stepping,
        "speed_at_observation_end": speed_at_end,
        "fully_stopped_confirmed": fully_stopped_confirmed,
        "observation_after_stop_seconds": observation_after_stop,
        "distance_is_lower_bound_only": not fully_stopped_confirmed,
        "minimum_linear_zero_hold_seconds": float(event.get("minimum_linear_zero_hold_seconds", 0.0)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        action="append",
        type=Path,
        default=[],
        help="包含 observation/ground_support_analysis 和 bag 的运行目录，可重复指定",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if rosbag is None:
        raise SystemExit("当前环境没有 rosbag / nav_msgs")

    run_dirs = args.run or [
        Path("results/runs/20260714_230842_seed_42"),
        Path("results/runs/20260714_235118_seed_42"),
        Path("results/runs/20260715_001918_seed_42"),
    ]
    all_events = []
    for run_dir in run_dirs:
        analysis = run_dir / "observation" / "ground_support_analysis" / "ground_support_analysis.json"
        bag = run_dir / "observation" / "ground_support_telemetry.bag"
        if not analysis.exists() or not bag.exists():
            print("跳过（缺文件）: %s" % run_dir)
            continue
        events = _load_natural_stops(analysis)
        print("%s: %d 个自然停止事件" % (run_dir.name, len(events)))
        for ev in events:
            result = _analyze_event(ev, bag)
            result["source_run"] = run_dir.name
            all_events.append(result)

    in_range = [e for e in all_events if 0.50 <= e["start_actual_speed"] <= 0.60]
    print("\n0.50-0.60 m/s 区间事件: %d / %d" % (len(in_range), len(all_events)))
    for e in in_range:
        print("  speed=%.3f disp=%.4f rebound=%s continued_step=%s roll_abs=%.4f pitch_abs=%.4f stopped=%s" % (
            e["start_actual_speed"], e["full_stop_net_displacement"],
            e["rebound"], e["continued_stepping"],
            e["roll_during_deceleration"]["abs_maximum"] or 0,
            e["pitch_during_deceleration"]["abs_maximum"] or 0,
            e["fully_stopped_confirmed"],
        ))

    complete_events = [e for e in all_events if not e["distance_is_lower_bound_only"]]
    complete_in_range = [
        e for e in complete_events if 0.50 <= e["start_actual_speed"] <= 0.60
    ]
    all_displacements = [e["maximum_displacement_before_stop"] for e in complete_events]
    in_range_displacements = [
        e["maximum_displacement_before_stop"] for e in complete_in_range
    ]
    conservative_upper = max(all_displacements) if all_displacements else None
    conservative_in_range = (
        max(in_range_displacements) if in_range_displacements else None
    )
    print("\n完整事件的保守停止距离上界（所有速度）: %s" % (
        "%.4f m" % conservative_upper if conservative_upper is not None else "无",
    ))
    print("0.50-0.60 m/s 完整事件的保守停止距离上界: %s" % (
        "%.4f m" % conservative_in_range if conservative_in_range is not None else "无",
    ))

    output = {
        "schema": "braking_envelope_offline_v1",
        "event_count": len(all_events),
        "events_in_0.50_0.60_range": len(in_range),
        "complete_event_count": len(complete_events),
        "complete_events_in_0.50_0.60_range": len(complete_in_range),
        "conservative_upper_bound_stop_distance_m": conservative_upper,
        "conservative_upper_bound_stop_distance_0.50_0.60_mps_m": conservative_in_range,
        "events": all_events,
        "scope": "仅从已有 ground_support rosbag 的 /Odometry_gazebo 离线提取；非专用制动采集",
        "method_note": "回弹=停后位移先增后减超5mm；继续迈步=停后最大位移超20mm；完全停止确认=停后最后5帧速度均<0.05m/s；持续减速=RL零命令后速度下降至少0.02m/s并保持50ms",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("写出: %s" % args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""离线提取零速换挡后的真实制动距离，不得被生产节点导入。"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class CommandSample:
    stamp: float
    linear: float
    lateral: float
    angular: float


@dataclass(frozen=True)
class MotionSample:
    stamp: float
    x: float
    y: float
    yaw: float
    linear_speed: float
    angular_speed: float


@dataclass(frozen=True)
class BrakingEvent:
    zero_command_stamp: float
    requested_linear: float
    requested_angular: float
    route_kind: str
    initial_truth_speed: float
    truth_stop_stamp: float
    stop_time: float
    stop_path_length: float
    forward_displacement: float
    localization_confirmation_stamp: float | None
    localization_confirmation_delay: float | None
    truth_speed_at_localization_confirmation: float | None


def _nearest(
    samples: Sequence[MotionSample],
    stamp: float,
    maximum_gap: float,
) -> MotionSample | None:
    if not samples:
        return None
    stamps = [sample.stamp for sample in samples]
    index = bisect_left(stamps, stamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    closest = min(candidates, key=lambda sample: abs(sample.stamp - stamp))
    return closest if abs(closest.stamp - stamp) <= maximum_gap else None


def _is_zero(command: CommandSample, threshold: float) -> bool:
    return (
        abs(command.linear) <= threshold
        and abs(command.lateral) <= threshold
        and abs(command.angular) <= threshold
    )


def _localization_confirmation(
    localization: Sequence[MotionSample],
    truths: Sequence[MotionSample],
    start_stamp: float,
    end_stamp: float | None,
    *,
    sample_count: int,
    linear_threshold: float,
    angular_threshold: float,
    maximum_sync_gap: float,
) -> tuple[float | None, float | None]:
    consecutive = 0
    for sample in localization:
        if sample.stamp < start_stamp:
            continue
        if end_stamp is not None and sample.stamp >= end_stamp:
            break
        if (
            sample.linear_speed <= linear_threshold
            and sample.angular_speed <= angular_threshold
        ):
            consecutive += 1
        else:
            consecutive = 0
        if consecutive < sample_count:
            continue
        truth = _nearest(truths, sample.stamp, maximum_sync_gap)
        return sample.stamp, None if truth is None else truth.linear_speed
    return None, None


def extract_braking_events(
    commands: Sequence[CommandSample],
    localization: Sequence[MotionSample],
    truths: Sequence[MotionSample],
    *,
    minimum_pre_stop_command: float = 0.30,
    zero_command_threshold: float = 0.01,
    truth_stop_threshold: float = 0.05,
    localization_linear_threshold: float = 0.05,
    localization_angular_threshold: float = 0.05,
    localization_confirmation_samples: int = 3,
    maximum_sync_gap: float = 0.10,
    straight_angular_threshold: float = 0.05,
) -> list[BrakingEvent]:
    """按预注册口径提取“运动命令直接切换为持续全零”的制动事件。"""
    if localization_confirmation_samples <= 0:
        raise ValueError("localization_confirmation_samples 必须为正整数")
    if not commands or not truths:
        return []

    ordered_commands = sorted(commands, key=lambda sample: sample.stamp)
    ordered_localization = sorted(localization, key=lambda sample: sample.stamp)
    ordered_truths = sorted(truths, key=lambda sample: sample.stamp)
    events: list[BrakingEvent] = []

    for previous, current in zip(ordered_commands, ordered_commands[1:]):
        if previous.linear < minimum_pre_stop_command:
            continue
        if _is_zero(previous, zero_command_threshold):
            continue
        if not _is_zero(current, zero_command_threshold):
            continue

        start_truth = _nearest(ordered_truths, current.stamp, maximum_sync_gap)
        initial_truth = _nearest(ordered_truths, previous.stamp, maximum_sync_gap)
        if start_truth is None or initial_truth is None:
            continue

        stop_truth = next(
            (
                sample
                for sample in ordered_truths
                if sample.stamp >= current.stamp
                and sample.linear_speed <= truth_stop_threshold
            ),
            None,
        )
        if stop_truth is None:
            continue
        if any(
            not _is_zero(command, zero_command_threshold)
            for command in ordered_commands
            if current.stamp < command.stamp <= stop_truth.stamp
        ):
            continue
        next_motion_command_stamp = next(
            (
                command.stamp
                for command in ordered_commands
                if command.stamp > current.stamp
                and not _is_zero(command, zero_command_threshold)
            ),
            None,
        )

        path_samples = [
            sample
            for sample in ordered_truths
            if start_truth.stamp <= sample.stamp <= stop_truth.stamp
        ]
        path_length = sum(
            math.hypot(following.x - sample.x, following.y - sample.y)
            for sample, following in zip(path_samples, path_samples[1:])
        )
        dx = stop_truth.x - start_truth.x
        dy = stop_truth.y - start_truth.y
        forward_displacement = (
            dx * math.cos(start_truth.yaw) + dy * math.sin(start_truth.yaw)
        )
        confirmation_stamp, confirmation_truth_speed = _localization_confirmation(
            ordered_localization,
            ordered_truths,
            current.stamp,
            next_motion_command_stamp,
            sample_count=localization_confirmation_samples,
            linear_threshold=localization_linear_threshold,
            angular_threshold=localization_angular_threshold,
            maximum_sync_gap=maximum_sync_gap,
        )
        events.append(
            BrakingEvent(
                zero_command_stamp=current.stamp,
                requested_linear=previous.linear,
                requested_angular=previous.angular,
                route_kind=(
                    "直行"
                    if abs(previous.angular) <= straight_angular_threshold
                    else "弧线"
                ),
                initial_truth_speed=initial_truth.linear_speed,
                truth_stop_stamp=stop_truth.stamp,
                stop_time=stop_truth.stamp - current.stamp,
                stop_path_length=path_length,
                forward_displacement=forward_displacement,
                localization_confirmation_stamp=confirmation_stamp,
                localization_confirmation_delay=(
                    None
                    if confirmation_stamp is None
                    else confirmation_stamp - current.stamp
                ),
                truth_speed_at_localization_confirmation=confirmation_truth_speed,
            )
        )
    return events


def summarize_events(events: Sequence[BrakingEvent]) -> dict[str, object]:
    by_route = {
        route: sum(event.route_kind == route for event in events)
        for route in ("直行", "弧线")
    }
    confirmed = [
        event
        for event in events
        if event.truth_speed_at_localization_confirmation is not None
    ]
    return {
        "event_count": len(events),
        "route_counts": by_route,
        "maximum_stop_path_length": (
            max((event.stop_path_length for event in events), default=None)
        ),
        "maximum_forward_displacement": (
            max((event.forward_displacement for event in events), default=None)
        ),
        "maximum_stop_time": max((event.stop_time for event in events), default=None),
        "localization_confirmation_count": len(confirmed),
        "maximum_truth_speed_at_localization_confirmation": (
            max(
                (
                    event.truth_speed_at_localization_confirmation
                    for event in confirmed
                    if event.truth_speed_at_localization_confirmation is not None
                ),
                default=None,
            )
        ),
        "events": [asdict(event) for event in events],
    }


def _stamp(message, bag_stamp) -> float:
    header = getattr(message, "header", None)
    header_stamp = getattr(header, "stamp", None)
    if header_stamp is not None and header_stamp.to_sec() > 0.0:
        return header_stamp.to_sec()
    return bag_stamp.to_sec()


def _yaw(quaternion) -> float:
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def _read_bag(
    path: Path,
    command_topic: str,
    localization_topic: str,
    truth_topic: str,
) -> tuple[list[CommandSample], list[MotionSample], list[MotionSample]]:
    import rosbag  # 仅离线命令入口允许读取诊断真值。

    commands: list[CommandSample] = []
    localization: list[MotionSample] = []
    truths: list[MotionSample] = []
    topics = [command_topic, localization_topic, truth_topic]
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            stamp = _stamp(message, bag_stamp)
            if topic == command_topic:
                commands.append(
                    CommandSample(
                        stamp,
                        float(message.linear.x),
                        float(message.linear.y),
                        float(message.angular.z),
                    )
                )
                continue
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            twist = message.twist.twist
            sample = MotionSample(
                stamp=stamp,
                x=float(position.x),
                y=float(position.y),
                yaw=_yaw(orientation),
                linear_speed=math.hypot(float(twist.linear.x), float(twist.linear.y)),
                angular_speed=abs(float(twist.angular.z)),
            )
            (localization if topic == localization_topic else truths).append(sample)
    return commands, localization, truths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--command-topic", default="/cmd_vel")
    parser.add_argument(
        "--localization-topic", default="/danger_search/localization/odom"
    )
    parser.add_argument("--truth-topic", default="/ground_truth/base_w")
    parser.add_argument("--minimum-pre-stop-command", type=float, default=0.30)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    commands, localization, truths = _read_bag(
        args.bag,
        args.command_topic,
        args.localization_topic,
        args.truth_topic,
    )
    summary = summarize_events(
        extract_braking_events(
            commands,
            localization,
            truths,
            minimum_pre_stop_command=args.minimum_pre_stop_command,
        )
    )
    summary["inputs"] = {
        "bag": str(args.bag),
        "command_topic": args.command_topic,
        "localization_topic": args.localization_topic,
        "truth_topic": args.truth_topic,
        "command_count": len(commands),
        "localization_count": len(localization),
        "truth_count": len(truths),
    }
    output = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output is not None:
        args.output.write_text(output + "\n", encoding="utf-8")
    else:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""从 rosout 结构化日志分解最近一轮任务的状态、运动和停顿耗时。"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
from typing import Iterable


ACTIVE_STATES = {"EXPLORE_FLOOR", "RETURN_HOME", "CHANGE_FLOOR"}


def load_latest_runtime_samples(path: Path) -> list[dict[str, object]]:
    """读取最近一次 INIT 之后的 mission_runtime 样本。"""
    samples: list[dict[str, object]] = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            payload_start = line.find("{")
            if payload_start < 0:
                continue
            try:
                payload = json.loads(line[payload_start:])
            except (json.JSONDecodeError, TypeError):
                continue
            if not isinstance(payload, dict):
                continue
            if (
                payload.get("event") == "mission_state_transition"
                and payload.get("old_state") == "INIT"
            ):
                samples = []
            if payload.get("event") == "mission_runtime":
                samples.append(payload)
    return samples


def _finite_number(value: object, default: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def _pose_xy(sample: dict[str, object]) -> tuple[float, float] | None:
    pose = sample.get("pose")
    if not isinstance(pose, list) or len(pose) < 2:
        return None
    x = _finite_number(pose[0], math.nan)
    y = _finite_number(pose[1], math.nan)
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return x, y


def _motion_category(sample: dict[str, object]) -> str:
    controller = sample.get("controller")
    if not isinstance(controller, dict):
        return "无控制诊断"
    linear = math.hypot(
        _finite_number(controller.get("linear_command")),
        _finite_number(controller.get("lateral_command")),
    )
    angular = abs(_finite_number(controller.get("angular_command")))
    requested_linear = math.hypot(
        _finite_number(controller.get("requested_linear_command")),
        _finite_number(controller.get("requested_lateral_command")),
    )
    if str(controller.get("recover_state") or "idle") != "idle":
        return "恢复动作"
    if linear > 0.03:
        return "前进命令"
    if angular > 0.08:
        return "原地转向"
    if requested_linear > 0.03:
        return "安全门停车"
    if bool(controller.get("path_active")):
        return "路径内零速"
    return "规划或空闲"


def analyze_runtime_samples(samples: Iterable[dict[str, object]]) -> dict[str, object]:
    """按相邻样本的任务时间差累计耗时，返回可序列化摘要。"""
    ordered = list(samples)
    if len(ordered) < 2:
        raise ValueError("至少需要两份 mission_runtime 样本")

    state_seconds: dict[str, float] = defaultdict(float)
    motion_seconds: dict[str, float] = defaultdict(float)
    task_phase_seconds: dict[str, float] = defaultdict(float)
    zero_reason_seconds: dict[str, float] = defaultdict(float)
    low_displacement_command_seconds = 0.0

    for current, following in zip(ordered, ordered[1:]):
        elapsed = _finite_number(current.get("elapsed"), math.nan)
        next_elapsed = _finite_number(following.get("elapsed"), math.nan)
        if not math.isfinite(elapsed) or not math.isfinite(next_elapsed):
            continue
        duration = next_elapsed - elapsed
        if duration <= 0.0 or duration > 5.0:
            continue

        state = str(current.get("state") or "UNKNOWN")
        state_seconds[state] += duration
        motion_seconds[
            _motion_category(current) if state in ACTIVE_STATES else "初始化或终态"
        ] += duration

        phase = current.get("current_task_phase")
        if phase:
            task_phase_seconds[str(phase)] += duration

        controller = current.get("controller")
        if not isinstance(controller, dict):
            continue
        zero_reason = controller.get("zero_command_reason")
        if zero_reason:
            zero_reason_seconds[str(zero_reason)] += duration

        requested_linear = math.hypot(
            _finite_number(controller.get("requested_linear_command")),
            _finite_number(controller.get("requested_lateral_command")),
        )
        current_pose = _pose_xy(current)
        next_pose = _pose_xy(following)
        if requested_linear <= 0.05 or current_pose is None or next_pose is None:
            continue
        displacement = math.hypot(
            next_pose[0] - current_pose[0],
            next_pose[1] - current_pose[1],
        )
        if displacement < 0.02:
            low_displacement_command_seconds += duration

    first_elapsed = _finite_number(ordered[0].get("elapsed"), math.nan)
    last_elapsed = _finite_number(ordered[-1].get("elapsed"), math.nan)
    return {
        "sample_count": len(ordered),
        "sample_elapsed_range": [first_elapsed, last_elapsed],
        "sampled_duration": max(0.0, last_elapsed - first_elapsed),
        "state_seconds": dict(sorted(state_seconds.items())),
        "motion_seconds": dict(sorted(motion_seconds.items())),
        "task_phase_seconds": dict(sorted(task_phase_seconds.items())),
        "zero_reason_seconds": dict(sorted(zero_reason_seconds.items())),
        "low_displacement_command_seconds": low_displacement_command_seconds,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rosout_log", type=Path, help="待分析的 rosout.log")
    args = parser.parse_args()
    samples = load_latest_runtime_samples(args.rosout_log)
    try:
        summary = analyze_runtime_samples(samples)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

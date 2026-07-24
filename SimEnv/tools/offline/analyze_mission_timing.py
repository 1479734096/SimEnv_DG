#!/usr/bin/env python3
"""从 rosout 结构化日志分解最近一轮任务的状态、运动和停顿耗时。"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
from typing import Iterable


ACTIVE_STATES = {"EXPLORE_FLOOR", "RETURN_HOME", "CHANGE_FLOOR"}
DEFAULT_MAX_TRAJECTORY_JUMP_METERS = 1.0
EFFECTIVE_DISPLACEMENT_METERS = 0.02
COMMANDED_MOTION_METERS_PER_SECOND = 0.05
SAFETY_EVENT_KEYWORDS = ("ground_support", "unsafe", "drop", "safety")


def load_latest_run_records(path: Path) -> list[dict[str, object]]:
    """读取最近一次 INIT 之后的全部结构化任务事件。"""
    records: list[dict[str, object]] = []
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
                records = []
            records.append(payload)
    return records


def load_latest_runtime_samples(path: Path) -> list[dict[str, object]]:
    """读取最近一次 INIT 之后的 mission_runtime 样本。"""
    return [
        payload
        for payload in load_latest_run_records(path)
        if payload.get("event") == "mission_runtime"
    ]


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


def _floor_index(sample: dict[str, object]) -> int | None:
    value = _finite_number(sample.get("current_floor"), math.nan)
    if not math.isfinite(value) or not value.is_integer():
        return None
    return int(value)


def _counter_increase(current: float | None, previous: float | None) -> int:
    """计算单调诊断计数器的增量，兼容进程内计数器意外重置。"""
    if current is None:
        return 0
    rounded = max(0, int(current))
    if previous is None:
        return rounded
    return rounded - int(previous) if rounded >= previous else rounded


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


def _controller_counters(
    samples: Iterable[dict[str, object]],
) -> tuple[int, int]:
    """统计完整样本序列中的累计 STUCK/恢复次数，不能漏掉末样本。"""
    stuck_episode_count = 0
    recovery_episode_count = 0
    previous_stuck_count: float | None = None
    previous_recovery_count: float | None = None
    previous_recovering = False
    observed_recovery_counter = False
    for sample in samples:
        controller = sample.get("controller")
        if not isinstance(controller, dict):
            continue
        stuck_value = _finite_number(controller.get("stuck_count"), math.nan)
        if math.isfinite(stuck_value):
            stuck_episode_count += _counter_increase(stuck_value, previous_stuck_count)
            previous_stuck_count = stuck_value
        recovery_value = _finite_number(controller.get("recover_count"), math.nan)
        if math.isfinite(recovery_value):
            observed_recovery_counter = True
            recovery_episode_count += _counter_increase(
                recovery_value, previous_recovery_count
            )
            previous_recovery_count = recovery_value
        recovering = str(controller.get("recover_state") or "idle") != "idle"
        if not observed_recovery_counter and recovering and not previous_recovering:
            recovery_episode_count += 1
        previous_recovering = recovering
    return stuck_episode_count, recovery_episode_count


def _coverage_summary(
    samples: Iterable[dict[str, object]],
) -> tuple[list[float], dict[str, dict[str, float]]]:
    all_coverages: list[float] = []
    by_floor: dict[int, list[float]] = defaultdict(list)
    for sample in samples:
        coverage = _finite_number(sample.get("coverage_ratio"), math.nan)
        if not math.isfinite(coverage):
            continue
        all_coverages.append(coverage)
        floor = _floor_index(sample)
        if floor is not None:
            by_floor[floor].append(coverage)
    floor_summary = {
        str(floor): {
            "initial": values[0],
            "final": values[-1],
            "maximum": max(values),
        }
        for floor, values in sorted(by_floor.items())
    }
    return all_coverages, floor_summary


def _incomplete_runtime_summary(
    samples: Iterable[dict[str, object]],
) -> dict[str, object]:
    """为启动前失败保留明确缺失语义，同时仍输出可用的离散计数。"""
    ordered = list(samples)
    coverages, coverage_by_floor = _coverage_summary(ordered)
    stuck_count, recovery_count = _controller_counters(ordered)
    elapsed_values = [
        _finite_number(sample.get("elapsed"), math.nan) for sample in ordered
    ]
    finite_elapsed = [value for value in elapsed_values if math.isfinite(value)]
    return {
        "sample_count": len(ordered),
        "sample_elapsed_range": (
            [finite_elapsed[0], finite_elapsed[-1]] if finite_elapsed else [None, None]
        ),
        "sampled_duration": 0.0 if finite_elapsed else None,
        "state_seconds": {},
        "motion_seconds": {},
        "task_phase_seconds": {},
        "zero_reason_seconds": {},
        "active_seconds": 0.0,
        "effective_motion_seconds": None,
        "effective_motion_ratio": None,
        "commanded_motion_seconds": None,
        "low_displacement_command_seconds": None,
        "raw_trajectory_distance": None,
        "trajectory_distance": None,
        "excluded_trajectory_segments": {},
        "excluded_trajectory_distance": {},
        "average_commanded_speed": None,
        "peak_commanded_speed": None,
        "stuck_episode_count": stuck_count,
        "recovery_episode_count": recovery_count,
        "initial_coverage_ratio": coverages[0] if coverages else None,
        "final_coverage_ratio": coverages[-1] if coverages else None,
        "maximum_coverage_ratio": max(coverages) if coverages else None,
        "coverage_by_floor": coverage_by_floor,
    }


def analyze_runtime_samples(
    samples: Iterable[dict[str, object]],
    *,
    max_trajectory_jump_meters: float = DEFAULT_MAX_TRAJECTORY_JUMP_METERS,
) -> dict[str, object]:
    """按相邻样本的任务时间差累计耗时，返回可序列化摘要。"""
    ordered = list(samples)
    if len(ordered) < 2:
        raise ValueError("至少需要两份 mission_runtime 样本")

    state_seconds: dict[str, float] = defaultdict(float)
    motion_seconds: dict[str, float] = defaultdict(float)
    task_phase_seconds: dict[str, float] = defaultdict(float)
    zero_reason_seconds: dict[str, float] = defaultdict(float)
    low_displacement_command_seconds = 0.0
    effective_motion_seconds = 0.0
    commanded_motion_seconds = 0.0
    active_seconds = 0.0
    raw_trajectory_distance = 0.0
    trajectory_distance = 0.0
    excluded_trajectory_segments: dict[str, int] = defaultdict(int)
    excluded_trajectory_distance: dict[str, float] = defaultdict(float)
    weighted_linear_command = 0.0
    peak_commanded_speed = 0.0
    stuck_episode_count, recovery_episode_count = _controller_counters(ordered)

    if not math.isfinite(max_trajectory_jump_meters) or max_trajectory_jump_meters <= 0.0:
        raise ValueError("max_trajectory_jump_meters 必须为正的有限数")

    for sample in ordered:
        controller = sample.get("controller")
        if not isinstance(controller, dict):
            continue
        peak_commanded_speed = max(
            peak_commanded_speed,
            math.hypot(
                _finite_number(controller.get("linear_command")),
                _finite_number(controller.get("lateral_command")),
            ),
        )

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
        active = state in ACTIVE_STATES
        if active:
            active_seconds += duration
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
        commanded_linear = math.hypot(
            _finite_number(controller.get("linear_command")),
            _finite_number(controller.get("lateral_command")),
        )
        if commanded_linear > COMMANDED_MOTION_METERS_PER_SECOND:
            commanded_motion_seconds += duration
        if active:
            weighted_linear_command += commanded_linear * duration

        current_pose = _pose_xy(current)
        next_pose = _pose_xy(following)
        if current_pose is None or next_pose is None:
            continue
        displacement = math.hypot(
            next_pose[0] - current_pose[0],
            next_pose[1] - current_pose[1],
        )
        raw_trajectory_distance += displacement
        current_floor = _floor_index(current)
        next_floor = _floor_index(following)
        # 楼层变更与超过可配置上限的定位跳变不能归因于机器人行走。
        if current_floor is not None and next_floor is not None and current_floor != next_floor:
            excluded_trajectory_segments["floor_change"] += 1
            excluded_trajectory_distance["floor_change"] += displacement
        elif displacement > max_trajectory_jump_meters:
            excluded_trajectory_segments["localization_jump"] += 1
            excluded_trajectory_distance["localization_jump"] += displacement
        else:
            trajectory_distance += displacement
            if active and displacement >= EFFECTIVE_DISPLACEMENT_METERS:
                effective_motion_seconds += duration
        if requested_linear <= COMMANDED_MOTION_METERS_PER_SECOND:
            continue
        if displacement < EFFECTIVE_DISPLACEMENT_METERS:
            low_displacement_command_seconds += duration

    first_elapsed = _finite_number(ordered[0].get("elapsed"), math.nan)
    last_elapsed = _finite_number(ordered[-1].get("elapsed"), math.nan)
    finite_coverages, coverage_by_floor = _coverage_summary(ordered)
    return {
        "sample_count": len(ordered),
        "sample_elapsed_range": [first_elapsed, last_elapsed],
        "sampled_duration": max(0.0, last_elapsed - first_elapsed),
        "state_seconds": dict(sorted(state_seconds.items())),
        "motion_seconds": dict(sorted(motion_seconds.items())),
        "task_phase_seconds": dict(sorted(task_phase_seconds.items())),
        "zero_reason_seconds": dict(sorted(zero_reason_seconds.items())),
        "active_seconds": active_seconds,
        "effective_motion_seconds": effective_motion_seconds,
        "effective_motion_ratio": (
            effective_motion_seconds / active_seconds if active_seconds > 0.0 else 0.0
        ),
        "commanded_motion_seconds": commanded_motion_seconds,
        "low_displacement_command_seconds": low_displacement_command_seconds,
        "raw_trajectory_distance": raw_trajectory_distance,
        "trajectory_distance": trajectory_distance,
        "excluded_trajectory_segments": dict(sorted(excluded_trajectory_segments.items())),
        "excluded_trajectory_distance": dict(
            sorted(excluded_trajectory_distance.items())
        ),
        "average_commanded_speed": (
            weighted_linear_command / active_seconds if active_seconds > 0.0 else 0.0
        ),
        "peak_commanded_speed": peak_commanded_speed,
        "stuck_episode_count": stuck_episode_count,
        "recovery_episode_count": recovery_episode_count,
        "initial_coverage_ratio": finite_coverages[0] if finite_coverages else None,
        "final_coverage_ratio": finite_coverages[-1] if finite_coverages else None,
        "maximum_coverage_ratio": max(finite_coverages) if finite_coverages else None,
        "coverage_by_floor": coverage_by_floor,
    }


def classify_run_failure(
    records: Iterable[dict[str, object]],
    auto_log_text: str = "",
    test_summary: dict[str, str] | None = None,
) -> dict[str, object]:
    """区分生产任务终态与控制器/启动环境失败。"""
    latest_result = None
    for payload in records:
        if payload.get("event") == "mission_result_write":
            latest_result = payload
    if latest_result is not None:
        outcome = str(latest_result.get("terminal_outcome") or "unknown")
        reason = str(latest_result.get("result_write_reason") or "unknown")
        if outcome == "success":
            category = "任务成功"
        elif reason.startswith("localization_"):
            category = "定位健康失败"
        elif reason.startswith("ground_support_"):
            category = "地面支撑失败"
        elif reason.startswith("return_home_"):
            category = "返航失败"
        elif (
            "elevator" in reason
            or "floor_transition" in reason
            or "floor_change" in reason
        ):
            category = "电梯或换层失败"
        elif reason.startswith("initial_entry_") or "main_entrance" in reason:
            category = "主入口失败"
        elif reason.startswith("external_"):
            category = "外部中止"
        else:
            category = "其他算法失败"
        return {"category": category, "terminal_outcome": outcome, "reason": reason}

    if "段错误" in auto_log_text or "Segmentation fault" in auto_log_text:
        return {
            "category": "控制器进程崩溃",
            "terminal_outcome": "environment_failed",
            "reason": "controller_segmentation_fault",
        }
    if "机器人站立失败" in auto_log_text or "站立状态超时" in auto_log_text:
        return {
            "category": "站立失败",
            "terminal_outcome": "environment_failed",
            "reason": "controller_stand_failed",
        }
    if "主循环首帧" in auto_log_text and "超时" in auto_log_text:
        return {
            "category": "控制器未就绪",
            "terminal_outcome": "environment_failed",
            "reason": "controller_not_ready",
        }
    summary = test_summary or {}
    detail = str(summary.get("detail") or "")
    result = str(summary.get("result") or "")
    if any(
        marker in detail
        for marker in ("FIFO", "stdin", "catkin", "ROS master", "测试夹具")
    ):
        return {
            "category": "测试夹具错误",
            "terminal_outcome": "environment_failed",
            "reason": "test_fixture_failed",
        }
    if "Gazebo" in detail and ("未" in detail or "超时" in detail):
        return {
            "category": "模型或控制器反馈未就绪",
            "terminal_outcome": "environment_failed",
            "reason": "gazebo_or_controller_not_ready",
        }
    if "墙钟" in detail or "wall timeout" in detail.lower():
        return {
            "category": "墙钟超时",
            "terminal_outcome": "environment_failed",
            "reason": "wall_timeout",
        }
    if result == "中止":
        return {
            "category": "外部中止",
            "terminal_outcome": "aborted",
            "reason": "runner_aborted",
        }
    return {"category": "无终态证据", "terminal_outcome": "unknown", "reason": None}


def detected_source_count(path: Path | None) -> int | None:
    if path is None or not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return None
    sources = payload.get("detected_danger_sources") if isinstance(payload, dict) else None
    return len(sources) if isinstance(sources, list) else None


def _load_json_object(path: Path | None) -> dict[str, object] | None:
    if path is None or not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return None
    return payload if isinstance(payload, dict) else None


def localization_error_summary(path: Path | None) -> dict[str, object] | None:
    payload = _load_json_object(path)
    if payload is None:
        return None
    fields = (
        "sample_count",
        "duration",
        "truth_distance",
        "final_position_error",
        "maximum_position_error",
        "rms_position_error",
        "p95_position_error",
        "final_yaw_error",
        "maximum_absolute_yaw_error",
    )
    summary: dict[str, object] = {}
    for field in fields:
        value = payload.get(field)
        if field == "sample_count":
            parsed = _finite_number(value, math.nan)
            if math.isfinite(parsed):
                summary[field] = max(0, int(parsed))
            continue
        parsed = _finite_number(value, math.nan)
        if math.isfinite(parsed):
            summary[field] = parsed
    return summary if summary else None


def evaluation_summary(path: Path | None) -> dict[str, object] | None:
    payload = _load_json_object(path)
    if payload is None:
        return None
    output: dict[str, object] = {}
    for section_name, fields in {
        "metrics": (
            "truth_count",
            "detected_count",
            "correct",
            "missed",
            "false_alarms",
            "exploration_time",
            "threshold_used",
        ),
        "scores": (
            "exploration_time_score",
            "recognition_probability_score",
            "false_alarm_rate_score",
            "technical_objective_total",
        ),
    }.items():
        section = payload.get(section_name)
        if not isinstance(section, dict):
            continue
        values = {}
        for field in fields:
            parsed = _finite_number(section.get(field), math.nan)
            if math.isfinite(parsed):
                values[field] = parsed
        if values:
            output[section_name] = values
    return output if output else None


def load_test_summary(path: Path | None) -> dict[str, str]:
    if path is None or not path.is_file():
        return {}
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return {}
    for line in lines:
        key, separator, value = line.partition("=")
        if separator and key:
            values[key] = value
    return values


def analyze_run(
    records: list[dict[str, object]],
    *,
    auto_log_text: str = "",
    test_summary: dict[str, str] | None = None,
    result_file: Path | None = None,
    localization_report: Path | None = None,
    evaluation_file: Path | None = None,
    max_trajectory_jump_meters: float = DEFAULT_MAX_TRAJECTORY_JUMP_METERS,
) -> dict[str, object]:
    if not math.isfinite(max_trajectory_jump_meters) or max_trajectory_jump_meters <= 0.0:
        raise ValueError("max_trajectory_jump_meters 必须为正的有限数")
    samples = [payload for payload in records if payload.get("event") == "mission_runtime"]
    summary = (
        analyze_runtime_samples(
            samples, max_trajectory_jump_meters=max_trajectory_jump_meters
        )
        if len(samples) >= 2
        else _incomplete_runtime_summary(samples)
    )
    event_counts = Counter(
        str(payload.get("event")) for payload in records if payload.get("event")
    )
    summary["event_counts"] = dict(sorted(event_counts.items()))
    summary["safety_event_counts"] = {
        event: count
        for event, count in sorted(event_counts.items())
        if any(keyword in event.lower() for keyword in SAFETY_EVENT_KEYWORDS)
    }
    summary["run_outcome"] = classify_run_failure(
        records, auto_log_text, test_summary
    )
    summary["detected_source_count"] = detected_source_count(result_file)
    summary["localization_error"] = localization_error_summary(localization_report)
    summary["evaluation"] = evaluation_summary(evaluation_file)
    summary["run_metadata"] = {
        key: value
        for key, value in (test_summary or {}).items()
        if key
        in {
            "result",
            "detail",
            "seed",
            "floor_timeout",
            "min_floor_explore_time",
            "max_runtime",
            "full_cycle",
            "offline_truth",
        }
    }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rosout_log", type=Path, help="待分析的 rosout.log")
    parser.add_argument("--auto-log", type=Path, help="可选的控制器启动日志")
    parser.add_argument("--test-summary", type=Path, help="可选的运行器 TEST_SUMMARY.txt")
    parser.add_argument("--result-file", type=Path, help="可选的正式检测结果文件")
    parser.add_argument(
        "--localization-report",
        type=Path,
        help="可选的任务结束后 localization_drift.json",
    )
    parser.add_argument(
        "--evaluation-file",
        type=Path,
        help="可选的任务结束后 evaluation_result.json",
    )
    parser.add_argument(
        "--max-trajectory-jump-meters",
        type=float,
        default=DEFAULT_MAX_TRAJECTORY_JUMP_METERS,
        help="轨迹单段超过该值视为定位跳变，不计入里程（默认：1.0）",
    )
    args = parser.parse_args()
    test_summary = load_test_summary(args.test_summary)
    records: list[dict[str, object]] = []
    if args.rosout_log.is_file():
        records = load_latest_run_records(args.rosout_log)
    auto_log_text = ""
    auto_log_path = args.auto_log
    if auto_log_path is None and test_summary.get("auto_log"):
        auto_log_path = Path(test_summary["auto_log"])
    if auto_log_path is not None and auto_log_path.is_file():
        auto_log_text = auto_log_path.read_text(encoding="utf-8", errors="replace")
    localization_report = args.localization_report
    if localization_report is None and test_summary.get("drift_summary"):
        localization_report = Path(test_summary["drift_summary"])
    if not records and not auto_log_text and not test_summary:
        parser.error("任务日志不存在或没有可用记录，且未提供启动/运行器证据")
    try:
        summary = analyze_run(
            records,
            auto_log_text=auto_log_text,
            test_summary=test_summary,
            result_file=args.result_file,
            localization_report=localization_report,
            evaluation_file=args.evaluation_file,
            max_trajectory_jump_meters=args.max_trajectory_jump_meters,
        )
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

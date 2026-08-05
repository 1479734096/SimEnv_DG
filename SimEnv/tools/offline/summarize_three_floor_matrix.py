#!/usr/bin/env python3
"""汇总三层完整闭环 seed 矩阵的运行经过、效果和问题。"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Iterable


FINAL_ATTEMPT_CLASSES = {"success", "algorithm_failure"}


def _finite(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _load_json(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _load_key_values(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, separator, value = line.partition("=")
        if separator and key.strip():
            values[key.strip()] = value.strip()
    return values


def _text(path: Path, default: str = "missing") -> str:
    if not path.is_file():
        return default
    return path.read_text(encoding="utf-8", errors="replace").strip() or default


def _attempt_number(path: Path) -> int:
    try:
        return int(path.name[len("attempt_") :])
    except ValueError:
        return -1


def _selected_attempt(seed_root: Path) -> Path | None:
    attempts = sorted(
        (path for path in seed_root.glob("attempt_*") if path.is_dir()),
        key=_attempt_number,
    )
    finalized = [
        path
        for path in attempts
        if _text(path / "ATTEMPT_CLASS.txt", "") in FINAL_ATTEMPT_CLASSES
    ]
    if finalized:
        return finalized[-1]
    return attempts[-1] if attempts else None


def _score_fields(evaluation: dict[str, object]) -> dict[str, object]:
    metrics = evaluation.get("metrics")
    scores = evaluation.get("scores")
    if not isinstance(metrics, dict):
        metrics = {}
    if not isinstance(scores, dict):
        scores = {}
    truth_count = metrics.get("truth_count")
    correct = metrics.get("correct")
    detected_count = metrics.get("detected_count")
    false_alarms = metrics.get("false_alarms")
    recall = None
    false_alarm_rate = None
    if isinstance(truth_count, int) and truth_count > 0 and isinstance(correct, int):
        recall = correct / truth_count
    if (
        isinstance(detected_count, int)
        and detected_count > 0
        and isinstance(false_alarms, int)
    ):
        false_alarm_rate = false_alarms / detected_count
    return {
        "truth_count": truth_count if isinstance(truth_count, int) else None,
        "detected_count": detected_count if isinstance(detected_count, int) else None,
        "correct": correct if isinstance(correct, int) else None,
        "missed": metrics.get("missed") if isinstance(metrics.get("missed"), int) else None,
        "false_alarms": false_alarms if isinstance(false_alarms, int) else None,
        "recall": recall,
        "false_alarm_rate": false_alarm_rate,
        "objective_score": _finite(scores.get("technical_objective_total")),
    }


def analyze_seed(root: Path, seed: int) -> dict[str, object]:
    run_dir = _selected_attempt(root / f"seed_{seed}")
    if run_dir is None:
        return {
            "seed": seed,
            "attempt": None,
            "attempt_class": "missing",
            "evidence_complete": False,
            "issues": ["尚未运行"],
            "run_dir": None,
        }

    attempt_class = _text(run_dir / "ATTEMPT_CLASS.txt")
    runner_summary = _load_key_values(run_dir / "TEST_SUMMARY.txt")
    metadata = _load_key_values(run_dir / "RUN_METADATA.txt")
    timing = _load_json(run_dir / "mission_timing.json")
    evaluation = _load_json(run_dir / "evaluation_result.json")
    outcome = timing.get("run_outcome")
    if not isinstance(outcome, dict):
        outcome = {}
    safety_counts = timing.get("safety_event_counts")
    if not isinstance(safety_counts, dict):
        safety_counts = {}
    safety_counts = {
        str(key): int(value)
        for key, value in safety_counts.items()
        if isinstance(value, int) and value > 0
    }
    excluded_segments = timing.get("excluded_trajectory_segments")
    if not isinstance(excluded_segments, dict):
        excluded_segments = {}

    required_files = [
        "RUNNER_EXIT_CODE.txt",
        "ATTEMPT_CLASS.txt",
        "RUN_METADATA.txt",
        "TEST_SUMMARY.txt",
        "mission_timing.json",
    ]
    if attempt_class in FINAL_ATTEMPT_CLASSES:
        required_files.extend(
            (
                "detected_danger.json",
                "danger_truth.json",
                "evaluation_result.json",
                "result_files.sha256",
            )
        )
    missing_files = [name for name in required_files if not (run_dir / name).is_file()]
    issues: list[str] = []
    if attempt_class == "algorithm_failure":
        issues.append(
            f"算法失败：{outcome.get('category', '未分类')} / "
            f"{outcome.get('reason') or '无终态原因'}"
        )
    elif attempt_class == "infrastructure_failure":
        issues.append(
            f"基础设施失败：{outcome.get('category', '未分类')} / "
            f"{outcome.get('reason') or runner_summary.get('detail', '无原因')}"
        )
    elif attempt_class == "interrupted":
        issues.append("运行被外部中止")
    elif attempt_class not in FINAL_ATTEMPT_CLASSES:
        issues.append(f"未知尝试分类：{attempt_class}")
    if missing_files:
        issues.append("缺少证据：" + "、".join(missing_files))
    if safety_counts:
        rendered = "、".join(f"{key}={value}" for key, value in sorted(safety_counts.items()))
        issues.append("安全事件：" + rendered)
    stuck_count = timing.get("stuck_episode_count")
    recovery_count = timing.get("recovery_episode_count")
    if isinstance(stuck_count, int) and stuck_count > 0:
        issues.append(f"STUCK episode={stuck_count}")
    if isinstance(recovery_count, int) and recovery_count > 0:
        issues.append(f"恢复 episode={recovery_count}")
    excluded_count = sum(
        value for value in excluded_segments.values() if isinstance(value, int) and value > 0
    )
    if excluded_count:
        issues.append(f"定位跳变剔除段={excluded_count}")
    if not evaluation:
        issues.append("缺少同轮离线评分")

    return {
        "seed": seed,
        "attempt": _attempt_number(run_dir),
        "attempt_class": attempt_class,
        "runner_exit_code": metadata.get("runner_exit_code"),
        "runner_result": runner_summary.get("result"),
        "runner_detail": runner_summary.get("detail"),
        "wall_elapsed_seconds": _finite(metadata.get("wall_elapsed_seconds")),
        "terminal_outcome": outcome.get("terminal_outcome"),
        "category": outcome.get("category"),
        "reason": outcome.get("reason"),
        "sampled_duration": _finite(timing.get("sampled_duration")),
        "trajectory_distance": _finite(timing.get("trajectory_distance")),
        "effective_motion_ratio": _finite(timing.get("effective_motion_ratio")),
        "maximum_coverage_ratio": _finite(timing.get("maximum_coverage_ratio")),
        "stuck_episode_count": stuck_count if isinstance(stuck_count, int) else None,
        "recovery_episode_count": recovery_count if isinstance(recovery_count, int) else None,
        "safety_event_counts": safety_counts,
        "evidence_complete": not missing_files,
        "issues": issues,
        "run_dir": str(run_dir.resolve()),
        **_score_fields(evaluation),
    }


def summarize(rows: Iterable[dict[str, object]], expected_count: int) -> dict[str, object]:
    runs = list(rows)
    class_counts = Counter(str(row.get("attempt_class")) for row in runs)
    scored = [row for row in runs if _finite(row.get("objective_score")) is not None]
    scores = [float(row["objective_score"]) for row in scored]
    recalls = [
        float(row["recall"])
        for row in scored
        if _finite(row.get("recall")) is not None
    ]
    false_alarm_rates = [
        float(row["false_alarm_rate"])
        for row in scored
        if _finite(row.get("false_alarm_rate")) is not None
    ]
    safety_totals: Counter[str] = Counter()
    for row in runs:
        counts = row.get("safety_event_counts")
        if isinstance(counts, dict):
            safety_totals.update(
                {
                    str(key): int(value)
                    for key, value in counts.items()
                    if isinstance(value, int)
                }
            )
    completed = class_counts["success"] + class_counts["algorithm_failure"]
    return {
        "expected_run_count": expected_count,
        "completed_formal_run_count": completed,
        "success_count": class_counts["success"],
        "algorithm_failure_count": class_counts["algorithm_failure"],
        "infrastructure_failure_count": class_counts["infrastructure_failure"],
        "interrupted_count": class_counts["interrupted"],
        "missing_count": class_counts["missing"],
        "evidence_complete_count": sum(bool(row.get("evidence_complete")) for row in runs),
        "scored_run_count": len(scored),
        "average_objective_score": sum(scores) / len(scores) if scores else None,
        "minimum_objective_score": min(scores) if scores else None,
        "average_recall": sum(recalls) / len(recalls) if recalls else None,
        "average_false_alarm_rate": (
            sum(false_alarm_rates) / len(false_alarm_rates)
            if false_alarm_rates
            else None
        ),
        "safety_event_totals": dict(sorted(safety_totals.items())),
        "matrix_complete": (
            completed == expected_count
            and sum(bool(row.get("evidence_complete")) for row in runs)
            == expected_count
        ),
        "runs": runs,
    }


def _show(value: object, digits: int = 3) -> str:
    number = _finite(value)
    return f"{number:.{digits}f}" if number is not None else "-"


def write_markdown(path: Path, result: dict[str, object], config: dict[str, str]) -> None:
    lines = [
        "# 三层完整闭环 Seed 矩阵摘要",
        "",
        "## 冻结条件",
        "",
        f"- Git HEAD：`{config.get('head', '缺失')}`",
        f"- Seed：`{config.get('seeds', '缺失')}`",
        "- 流程：`auto.sh → fixed stand → RL /cmd_vel → formal danger_search.launch`",
        "- 预算：每层 `280 s`，最短探索 `60 s`，总仿真 `840 s`，墙钟 `9000 s`",
        "- 在线真值：关闭；评分只读取每轮结束后归档的裁判文件",
        "",
        "## 整体效果",
        "",
        f"- 正式结果：`{result['completed_formal_run_count']}/{result['expected_run_count']}`",
        f"- 完整成功：`{result['success_count']}`",
        f"- 算法失败：`{result['algorithm_failure_count']}`",
        f"- 基础设施失败：`{result['infrastructure_failure_count']}`",
        f"- 已评分：`{result['scored_run_count']}`",
        f"- 平均/最低客观分：`{_show(result['average_objective_score'], 2)}` / "
        f"`{_show(result['minimum_objective_score'], 2)}`（满分 37）",
        f"- 平均召回率：`{_show(result['average_recall'])}`",
        f"- 平均虚警率：`{_show(result['average_false_alarm_rate'])}`",
        "",
        "## 逐轮经过",
        "",
        "| seed | 尝试 | 分类 | 终态类别 | 原因 | 仿真时长 | 墙钟时长 | 路程 | 覆盖峰值 | 检测/真值 | 正确 | 虚警 | 得分 |",
        "|---:|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in result["runs"]:
        detected = row.get("detected_count")
        truth = row.get("truth_count")
        detected_truth = (
            f"{detected}/{truth}"
            if isinstance(detected, int) and isinstance(truth, int)
            else "-"
        )
        lines.append(
            "| {seed} | {attempt} | {attempt_class} | {category} | {reason} | "
            "{sim} | {wall} | {distance} | {coverage} | {detected_truth} | "
            "{correct} | {false_alarms} | {score} |".format(
                seed=row["seed"],
                attempt=row.get("attempt") or "-",
                attempt_class=row.get("attempt_class", "-"),
                category=row.get("category") or "-",
                reason=row.get("reason") or "-",
                sim=_show(row.get("sampled_duration"), 1),
                wall=_show(row.get("wall_elapsed_seconds"), 0),
                distance=_show(row.get("trajectory_distance")),
                coverage=_show(row.get("maximum_coverage_ratio")),
                detected_truth=detected_truth,
                correct=row.get("correct") if isinstance(row.get("correct"), int) else "-",
                false_alarms=(
                    row.get("false_alarms")
                    if isinstance(row.get("false_alarms"), int)
                    else "-"
                ),
                score=_show(row.get("objective_score"), 2),
            )
        )
    lines.extend(["", "## 出现的问题", ""])
    issue_found = False
    for row in result["runs"]:
        issues = row.get("issues")
        if not isinstance(issues, list) or not issues:
            continue
        issue_found = True
        lines.append(f"- seed `{row['seed']}`：" + "；".join(str(item) for item in issues))
    if not issue_found:
        lines.append("- 当前结构化证据未发现问题；仍需复核人工观察记录。")
    lines.extend(
        [
            "",
            "## 人工观察",
            "",
            "画面中的卡门、接触、姿态异常等现象记录在同目录 `OPERATOR_NOTES.md`；",
            "这些观察不替代结构化终态、日志和离线评分。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="矩阵证据根目录")
    parser.add_argument("--seeds", nargs="+", required=True, type=int)
    args = parser.parse_args()
    if len(args.seeds) < 2 or len(args.seeds) > 20 or len(set(args.seeds)) != len(args.seeds):
        parser.error("必须提供 2～20 个互不重复的 seed")
    config = _load_key_values(args.root / "MATRIX_CONFIG.txt")
    rows = [analyze_seed(args.root, seed) for seed in args.seeds]
    result = summarize(rows, len(args.seeds))
    result["configuration"] = config
    (args.root / "matrix_summary.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_markdown(args.root / "matrix_summary.md", result, config)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

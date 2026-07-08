"""danger_search 离线评测、归档和调参辅助函数。

该模块只服务离线工具；运行中的 `danger_search_node` 不会导入它，也不会读取真值文件。
"""
from __future__ import annotations

import csv
import datetime as _datetime
import json
import math
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - ROS Noetic 常规环境包含 PyYAML
    yaml = None


SUMMARY_FIELDS = [
    "run_id",
    "timestamp",
    "seed",
    "scene_type",
    "param_version",
    "total_time",
    "exploration_time",
    "recognition_rate",
    "false_alarm_rate",
    "truth_count",
    "detected_count",
    "correct_count",
    "missed_count",
    "false_alarm_count",
    "duplicate_count",
    "return_home_success",
    "elevator_success",
    "completed",
    "total_score",
    "time_score",
    "recognition_score",
    "far_score",
    "max_floor",
    "timeout",
    "failure_reason",
]


PARAM_ALIASES = {
    "detection/nms_radius": "tracker/nms_radius",
    "detection/confirm_hits": "tracker/min_hits",
    "detection/confirm_window_sec": "tracker/confirmation_window",
    "detection/min_confidence": "tracker/min_confidence",
    "detection/max_position_std": "tracker/max_position_std",
    "detector/min_radius": "detection/min_radius",
    "detector/max_radius": "detection/max_radius",
    "detector/max_rmse": "detection/max_rmse",
    "explorer/blacklist_ttl": "frontier/blacklist_ttl",
    "explorer/no_progress_timeout": "frontier/no_progress_timeout",
    "mission/total_timeout": "mission/max_runtime",
    "controller/max_linear_vel": "controller/max_linear",
    "controller/max_angular_vel": "controller/max_angular",
}


def find_workspace_dir(start: str | Path | None = None) -> Path:
    """从给定路径向上寻找 SimEnv 工作区根目录。"""
    candidates: list[Path] = []
    if start is not None:
        path = Path(start).resolve()
        candidates.append(path if path.is_dir() else path.parent)
    candidates.append(Path.cwd().resolve())
    candidates.append(Path(__file__).resolve())

    seen: set[Path] = set()
    for candidate in candidates:
        for parent in [candidate, *candidate.parents]:
            if parent in seen:
                continue
            seen.add(parent)
            if (parent / "auto.sh").exists() and (parent / "src" / "danger_search").exists():
                return parent
    raise FileNotFoundError("无法定位 SimEnv 工作区根目录，请使用 --workspace 显式指定")


def package_dir(workspace_dir: str | Path) -> Path:
    return Path(workspace_dir) / "src" / "danger_search"


def default_config_path(workspace_dir: str | Path) -> Path:
    return package_dir(workspace_dir) / "config" / "danger_search.yaml"


def create_run_dir(workspace_dir: str | Path, seed: int | str, timestamp: str | None = None) -> Path:
    stamp = timestamp or _datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(workspace_dir) / "results" / "runs" / f"{stamp}_seed_{seed}"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "logs").mkdir(exist_ok=True)
    return run_dir


def read_json(path: str | Path, default: Any = None) -> Any:
    file_path = Path(path)
    if not file_path.exists():
        return default
    return json.loads(file_path.read_text(encoding="utf-8"))


def write_json(path: str | Path, data: Any) -> Path:
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return file_path


def read_yaml(path: str | Path, default: Any = None) -> Any:
    if yaml is None:
        raise RuntimeError("缺少 PyYAML，无法读取 YAML 配置")
    file_path = Path(path)
    if not file_path.exists():
        return default
    with file_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return default if data is None else data


def write_yaml(path: str | Path, data: Any) -> Path:
    if yaml is None:
        raise RuntimeError("缺少 PyYAML，无法写入 YAML 配置")
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    with file_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
    return file_path


def flatten_mapping(data: Mapping[str, Any], parent_key: str = "", sep: str = "/") -> dict[str, Any]:
    items: dict[str, Any] = {}
    for key, value in data.items():
        new_key = f"{parent_key}{sep}{key}" if parent_key else str(key)
        if isinstance(value, Mapping):
            items.update(flatten_mapping(value, new_key, sep=sep))
        else:
            items[new_key] = value
    return items


def unflatten_mapping(data: Mapping[str, Any], sep: str = "/") -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in data.items():
        cursor = result
        parts = str(key).split(sep)
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = value
    return result


def normalize_param_key(key: str) -> str:
    normalized = str(key).strip().replace(".", "/").lstrip("/")
    return PARAM_ALIASES.get(normalized, normalized)


def load_param_overrides(path: str | Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    file_path = Path(path)
    if file_path.suffix.lower() == ".json":
        raw = read_json(file_path, default={})
    else:
        raw = read_yaml(file_path, default={})
    if not isinstance(raw, Mapping):
        raise ValueError(f"参数覆盖文件必须是对象结构: {file_path}")
    return {normalize_param_key(k): v for k, v in flatten_mapping(raw).items()}


def apply_param_overrides(base_config: Mapping[str, Any], overrides: Mapping[str, Any]) -> dict[str, Any]:
    flat = flatten_mapping(base_config)
    for key, value in overrides.items():
        flat[normalize_param_key(key)] = value
    return unflatten_mapping(flat)


def write_effective_config(
    *,
    base_config_path: str | Path,
    override_path: str | Path | None,
    output_path: str | Path,
) -> dict[str, Any]:
    base_config = read_yaml(base_config_path, default={})
    overrides = load_param_overrides(override_path)
    effective = apply_param_overrides(base_config, overrides)
    write_yaml(output_path, effective)
    return effective


def parse_run_dir_name(name: str) -> tuple[str, str]:
    parts = name.split("_")
    timestamp = "_".join(parts[:2]) if len(parts) >= 2 else name
    seed = ""
    if "seed" in parts:
        index = parts.index("seed")
        if index + 1 < len(parts):
            seed = parts[index + 1]
    return timestamp, seed


def summarize_run(run_dir: str | Path) -> dict[str, Any]:
    run_path = Path(run_dir)
    timestamp, seed = parse_run_dir_name(run_path.name)
    mission = read_json(run_path / "mission_summary.json", default={}) or {}
    evaluation = read_json(run_path / "evaluation.json", default={}) or {}
    metrics = evaluation.get("metrics", {})
    scores = evaluation.get("scores", {})

    truth_count = int(metrics.get("truth_count", 0) or 0)
    detected_count = int(metrics.get("detected_count", 0) or 0)
    correct = int(metrics.get("correct", 0) or 0)
    missed = int(metrics.get("missed", 0) or 0)
    false_alarms = int(metrics.get("false_alarms", 0) or 0)
    exploration_time = safe_float(
        metrics.get("exploration_time", mission.get("exploration_time", mission.get("elapsed_wall_time", 0.0)))
    )
    threshold = safe_float(metrics.get("threshold_used", 1.0), default=1.0)
    recognition_rate = correct / truth_count if truth_count > 0 else 0.0
    false_alarm_rate = false_alarms / detected_count if detected_count > 0 else 0.0

    duplicate_count = count_duplicate_detections(
        truth_file=run_path / "danger_truth.json",
        detected_file=run_path / "detected_danger.json",
        threshold=threshold,
    )

    failure_reasons = list(_as_list(mission.get("failure_reason", [])))
    if not evaluation:
        failure_reasons.append("缺少评测结果")
    if mission.get("completed") is False:
        failure_reasons.append("任务未完成")
    if mission.get("external_timeout") is True:
        failure_reasons.append("外部超时")
    if false_alarm_rate > 0.10:
        failure_reasons.append("虚警偏高")
    if truth_count > 0 and recognition_rate < 0.60:
        failure_reasons.append("漏检偏高")
    timeout = safe_float(mission.get("timeout", 0.0))
    if mission.get("external_timeout") is True or (timeout > 0 and exploration_time >= max(0.0, timeout - 5.0)):
        failure_reasons.append("探索超时")

    return {
        "run_id": run_path.name,
        "timestamp": str(mission.get("timestamp", timestamp)),
        "seed": str(mission.get("seed", seed)),
        "scene_type": str(mission.get("scene_type", "")),
        "param_version": str(mission.get("param_version", "")),
        "total_time": round(exploration_time, 3),
        "exploration_time": round(exploration_time, 3),
        "recognition_rate": round(recognition_rate, 6),
        "false_alarm_rate": round(false_alarm_rate, 6),
        "truth_count": truth_count,
        "detected_count": detected_count,
        "correct_count": correct,
        "missed_count": missed,
        "false_alarm_count": false_alarms,
        "duplicate_count": duplicate_count,
        "return_home_success": str(mission.get("return_home_success", "")),
        "elevator_success": str(mission.get("elevator_success", "")),
        "completed": str(bool(mission.get("completed", False))),
        "total_score": safe_float(scores.get("technical_objective_total", 0.0)),
        "time_score": safe_float(scores.get("exploration_time_score", 0.0)),
        "recognition_score": safe_float(scores.get("recognition_probability_score", 0.0)),
        "far_score": safe_float(scores.get("false_alarm_rate_score", 0.0)),
        "max_floor": int(mission.get("max_floor", 0) or 0),
        "timeout": round(timeout, 3),
        "failure_reason": ";".join(dict.fromkeys(reason for reason in failure_reasons if reason)),
    }


def write_summary_csv(runs_dir: str | Path, output_file: str | Path) -> list[dict[str, Any]]:
    runs_path = Path(runs_dir)
    rows = [summarize_run(path) for path in runs_path.iterdir() if path.is_dir()] if runs_path.exists() else []
    rows.sort(key=lambda item: item["run_id"], reverse=True)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in SUMMARY_FIELDS})
    return rows


def read_summary_csv(path: str | Path) -> list[dict[str, str]]:
    file_path = Path(path)
    if not file_path.exists():
        return []
    with file_path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def count_duplicate_detections(truth_file: str | Path, detected_file: str | Path, threshold: float = 1.0) -> int:
    truth = _positions_from_truth(read_json(truth_file, default={}) or {})
    detected = _positions_from_detected(read_json(detected_file, default={}) or {})
    if not truth or not detected:
        return 0

    candidate_pairs: list[tuple[float, int, int]] = []
    for truth_index, truth_pos in enumerate(truth):
        for detected_index, detected_pos in enumerate(detected):
            distance = _distance(truth_pos, detected_pos)
            if distance <= threshold:
                candidate_pairs.append((distance, truth_index, detected_index))
    candidate_pairs.sort(key=lambda item: item[0])

    matched_truth: set[int] = set()
    matched_detected: set[int] = set()
    for _, truth_index, detected_index in candidate_pairs:
        if truth_index in matched_truth or detected_index in matched_detected:
            continue
        matched_truth.add(truth_index)
        matched_detected.add(detected_index)

    duplicate_count = 0
    for detected_index, detected_pos in enumerate(detected):
        if detected_index in matched_detected:
            continue
        if any(_distance(truth_pos, detected_pos) <= threshold for truth_pos in truth):
            duplicate_count += 1
    return duplicate_count


def build_optimization_report(
    *,
    results_dir: str | Path,
    tuning_report_path: str | Path | None = None,
    summary_csv_path: str | Path | None = None,
) -> str:
    results_path = Path(results_dir)
    tuning_path = Path(tuning_report_path) if tuning_report_path else results_path / "tuning" / "tuning_report.json"
    summary_path = Path(summary_csv_path) if summary_csv_path else results_path / "summary.csv"
    rows = read_summary_csv(summary_path)
    tuning_report = read_json(tuning_path, default={}) or {}

    lines = ["# danger_search 参数优化报告", ""]
    if tuning_report:
        lines.extend(_format_tuning_section(tuning_report))
    else:
        lines.extend(["## 最佳参数组合", "", "未找到 `tuning_report.json`，暂无多组参数对比结果。", ""])
    lines.extend(_format_failure_section(rows))
    return "\n".join(lines).rstrip() + "\n"


def safe_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return default
    try:
        if text.endswith("%"):
            return float(text[:-1]) / 100.0
        return float(text)
    except ValueError:
        return default


def _positions_from_truth(data: Mapping[str, Any]) -> list[tuple[float, float, float]]:
    return [_coerce_position(item.get("position")) for item in data.get("danger_sources", []) if isinstance(item, Mapping)]


def _positions_from_detected(data: Mapping[str, Any]) -> list[tuple[float, float, float]]:
    result = []
    for item in data.get("detected_danger_sources", []):
        if isinstance(item, Mapping):
            result.append(_coerce_position(item.get("position")))
        else:
            result.append(_coerce_position(item))
    return result


def _coerce_position(value: Any) -> tuple[float, float, float]:
    if not isinstance(value, Iterable):
        raise ValueError(f"位置必须是三维坐标: {value}")
    values = list(value)
    if len(values) != 3:
        raise ValueError(f"位置必须是三维坐标: {value}")
    return (float(values[0]), float(values[1]), float(values[2]))


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def _as_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, list):
        return [str(item) for item in value]
    return [str(value)]


def _format_tuning_section(report: Mapping[str, Any]) -> list[str]:
    lines = ["## 最佳参数组合", ""]
    lines.append(f"综合得分: {safe_float(report.get('best_score', 0.0)):.2f}")
    lines.extend(["", "```yaml"])
    best_params = report.get("best_params", {})
    if yaml is not None:
        lines.append(yaml.safe_dump(best_params, allow_unicode=True, sort_keys=False).rstrip())
    else:
        for key, value in dict(best_params).items():
            lines.append(f"{key}: {value}")
    lines.extend(["```", "", "## 候选参数表现", "", "| 参数组合 | 平均分 | 最差分 | 运行数 |", "|---|---:|---:|---:|"])
    for result in report.get("all_results", []):
        params = result.get("params", {})
        params_text = ", ".join(f"{key}={value}" for key, value in params.items())
        lines.append(
            f"| `{params_text}` | {safe_float(result.get('avg_score')):.2f} | "
            f"{safe_float(result.get('worst_score')):.2f} | {len(result.get('runs', []))} |"
        )
    lines.append("")
    return lines


def _format_failure_section(rows: list[Mapping[str, Any]]) -> list[str]:
    lines = ["## 失败归因与优化方向", ""]
    total = len(rows)
    if total == 0:
        lines.append("没有找到汇总运行记录，先执行批量评测生成 `summary.csv`。")
        return lines

    timeout_runs = 0
    high_far_runs = 0
    low_rec_runs = 0
    return_fail_runs = 0
    elevator_fail_runs = 0
    duplicate_runs = 0
    planner_or_stuck_runs = 0
    for row in rows:
        timeout = safe_float(row.get("timeout"))
        total_time = safe_float(row.get("total_time", row.get("exploration_time")))
        if timeout > 0 and total_time >= max(0.0, timeout - 5.0):
            timeout_runs += 1
        if safe_float(row.get("false_alarm_rate")) > 0.10:
            high_far_runs += 1
        if safe_float(row.get("recognition_rate")) < 0.60:
            low_rec_runs += 1
        if str(row.get("return_home_success", "")).lower() == "false":
            return_fail_runs += 1
        if str(row.get("elevator_success", "")).lower() == "false":
            elevator_fail_runs += 1
        if int(safe_float(row.get("duplicate_count"))) > 0:
            duplicate_runs += 1
        failure_reason = str(row.get("failure_reason", ""))
        if "规划" in failure_reason or "卡住" in failure_reason:
            planner_or_stuck_runs += 1

    lines.extend(
        [
            f"- 总运行次数：{total}",
            f"- 探索超时：{timeout_runs}/{total} ({timeout_runs / total:.1%})",
            f"- 虚警率高于 10%：{high_far_runs}/{total} ({high_far_runs / total:.1%})",
            f"- 识别率低于 60%：{low_rec_runs}/{total} ({low_rec_runs / total:.1%})",
            f"- 重复检测：{duplicate_runs}/{total} ({duplicate_runs / total:.1%})",
            f"- 返航失败：{return_fail_runs}/{total} ({return_fail_runs / total:.1%})",
            f"- 电梯失败：{elevator_fail_runs}/{total} ({elevator_fail_runs / total:.1%})",
            f"- 规划或卡住问题：{planner_or_stuck_runs}/{total} ({planner_or_stuck_runs / total:.1%})",
            "",
            "### 下一轮候选方向",
        ]
    )
    suggestions = []
    if high_far_runs / total > 0.25 or duplicate_runs / total > 0.25:
        suggestions.append("提高 `tracker/min_hits`、`tracker/min_confidence`，并把 `tracker/nms_radius` 维持在 0.9 到 1.0。")
    if low_rec_runs / total > 0.25:
        suggestions.append("适度降低 `tracker/min_hits` 或 `detection/min_circularity`，同时观察虚警率是否突破 10%。")
    if timeout_runs / total > 0.25:
        suggestions.append("缩短 `mission/floor_timeout` 和 `mission/no_frontier_scan_duration`，或小幅提高控制速度上限。")
    if planner_or_stuck_runs / total > 0.0:
        suggestions.append("调大黑名单 TTL 或障碍膨胀余量，并检查无效前沿是否被重复选择。")
    if elevator_fail_runs / total > 0.0:
        suggestions.append("优先核对 `floor/elevator_lobby_xy`、`floor/elevator_inside_xy` 和门服务响应时间。")
    if not suggestions:
        suggestions.append("当前汇总没有明显单一瓶颈，下一轮应增加 seed 数量并关注最差场景。")
    lines.extend(f"- {item}" for item in suggestions)
    return lines

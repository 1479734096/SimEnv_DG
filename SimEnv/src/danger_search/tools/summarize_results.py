#!/usr/bin/env python3
import csv
import json
from pathlib import Path
import sys

def parse_run_dir(run_dir: Path) -> dict:
    # 提取目录名中的时间戳和 seed
    # 例如：20260707_120000_seed_42
    name_parts = run_dir.name.split('_')
    timestamp = "_".join(name_parts[:2])
    
    seed = "unknown"
    if "seed" in name_parts:
        seed_idx = name_parts.index("seed")
        if seed_idx + 1 < len(name_parts):
            seed = name_parts[seed_idx + 1]

    summary = {
        "run_id": run_dir.name,
        "timestamp": timestamp,
        "seed": seed,
        "completed": False,
        "exploration_time": 0.0,
        "truth_count": 0,
        "detected_count": 0,
        "correct": 0,
        "missed": 0,
        "false_alarms": 0,
        "recognition_rate": 0.0,
        "false_alarm_rate": 0.0,
        "time_score": 0.0,
        "prob_score": 0.0,
        "far_score": 0.0,
        "total_score": 0.0,
        "max_floor": 0,
        "timeout": 0
    }

    # 读取 mission_summary
    mission_path = run_dir / "mission_summary.json"
    if mission_path.exists():
        with open(mission_path, 'r') as f:
            mission_data = json.load(f)
            summary["completed"] = mission_data.get("completed", False)
            summary["max_floor"] = mission_data.get("max_floor", 0)
            summary["timeout"] = mission_data.get("timeout", 0)

    # 读取 evaluation.json
    eval_path = run_dir / "evaluation.json"
    if eval_path.exists():
        with open(eval_path, 'r') as f:
            eval_data = json.load(f)
            metrics = eval_data.get("metrics", {})
            scores = eval_data.get("scores", {})
            
            summary["exploration_time"] = metrics.get("exploration_time", 0.0)
            summary["truth_count"] = metrics.get("truth_count", 0)
            summary["detected_count"] = metrics.get("detected_count", 0)
            summary["correct"] = metrics.get("correct", 0)
            summary["missed"] = metrics.get("missed", 0)
            summary["false_alarms"] = metrics.get("false_alarms", 0)
            
            if summary["truth_count"] > 0:
                summary["recognition_rate"] = summary["correct"] / summary["truth_count"]
            
            if summary["detected_count"] > 0:
                summary["false_alarm_rate"] = summary["false_alarms"] / summary["detected_count"]
                
            summary["time_score"] = scores.get("exploration_time_score", 0.0)
            summary["prob_score"] = scores.get("recognition_probability_score", 0.0)
            summary["far_score"] = scores.get("false_alarm_rate_score", 0.0)
            summary["total_score"] = scores.get("technical_objective_total", 0.0)

    return summary

def main():
    results_dir = Path("/home/xbh/tzb26/codeDG/SimEnv/results")
    runs_dir = results_dir / "runs"
    
    if not runs_dir.exists():
        print(f"找不到 runs 目录: {runs_dir}")
        sys.exit(1)

    summaries = []
    for run_dir in runs_dir.iterdir():
        if run_dir.is_dir():
            summaries.append(parse_run_dir(run_dir))

    if not summaries:
        print("没有找到任何运行记录。")
        sys.exit(0)

    # 按 timestamp 降序排序
    summaries.sort(key=lambda x: x["timestamp"], reverse=True)

    csv_path = results_dir / "summary.csv"
    
    fieldnames = [
        "run_id", "timestamp", "seed", "completed", "exploration_time",
        "truth_count", "detected_count", "correct", "missed", "false_alarms",
        "recognition_rate", "false_alarm_rate",
        "time_score", "prob_score", "far_score", "total_score",
        "max_floor", "timeout"
    ]

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for summary in summaries:
            # 格式化浮点数
            s = summary.copy()
            s["recognition_rate"] = f'{s["recognition_rate"]:.2%}'
            s["false_alarm_rate"] = f'{s["false_alarm_rate"]:.2%}'
            s["exploration_time"] = f'{s["exploration_time"]:.2f}'
            writer.writerow(s)

    print(f"成功汇总 {len(summaries)} 条记录至 {csv_path}")

if __name__ == "__main__":
    main()

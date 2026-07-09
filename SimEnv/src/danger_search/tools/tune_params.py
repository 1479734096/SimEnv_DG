#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from danger_search.evaluation_tools import find_workspace_dir, read_summary_csv, safe_float, write_yaml


def generate_param_combinations(param_grid: dict[str, list[object]]) -> list[dict[str, object]]:
    keys = list(param_grid.keys())
    values = list(param_grid.values())
    return [dict(zip(keys, combination)) for combination in itertools.product(*values)]


def get_run_scores(csv_path: Path, run_ids: set[str]) -> list[float]:
    rows = read_summary_csv(csv_path)
    scores = []
    for row in rows:
        if row.get("run_id") in run_ids:
            scores.append(safe_float(row.get("total_score")))
    return scores


def default_param_grid(test_mode: bool) -> dict[str, list[object]]:
    if test_mode:
        return {
            "tracker/nms_radius": [0.85, 0.95, 1.0],
        }
    return {
        "tracker/nms_radius": [0.85, 0.9, 1.0],
        "tracker/min_hits": [3, 4],
        "tracker/min_confidence": [0.72, 0.78],
        "detection/max_rmse": [0.03, 0.035],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="网格搜索 danger_search 参数")
    parser.add_argument("--workspace", type=str, help="SimEnv 工作区路径，默认自动寻找")
    parser.add_argument("--seeds", type=int, nargs="+", default=[10, 42, 100], help="用于评测的 seed 集合")
    parser.add_argument("--timeout", type=int, default=6200, help="单次运行超时，单位秒")
    parser.add_argument("--max-floor", type=int, default=3, help="最大探索楼层数")
    parser.add_argument("--test-mode", action="store_true", help="只跑 3 组轻量候选")
    parser.add_argument("--dry-run", action="store_true", help="只生成候选参数文件，不启动仿真")
    parser.add_argument("--max-candidates", type=int, help="最多评测多少组候选参数")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    workspace_dir = find_workspace_dir(args.workspace or Path(__file__))
    tools_dir = workspace_dir / "src" / "danger_search" / "tools"
    scripts_dir = workspace_dir / "src" / "danger_search" / "scripts"
    batch_evaluate_script = scripts_dir / "batch_evaluate.py"
    report_script = tools_dir / "generate_optimization_report.py"

    combinations = generate_param_combinations(default_param_grid(args.test_mode))
    if args.max_candidates is not None:
        combinations = combinations[: max(0, args.max_candidates)]
    print(f"共有 {len(combinations)} 组参数候选")

    tune_dir = workspace_dir / "results" / "tuning"
    tune_dir.mkdir(parents=True, exist_ok=True)
    runs_dir = workspace_dir / "results" / "runs"
    summary_csv = workspace_dir / "results" / "summary.csv"

    best_score = -1.0
    best_params: dict[str, object] | None = None
    all_results = []

    for index, params in enumerate(combinations):
        param_file = tune_dir / f"params_{index:03d}.yaml"
        write_yaml(param_file, params)
        print(f"\n[{index + 1}/{len(combinations)}] 参数候选: {params}")
        if args.dry_run:
            continue

        before_runs = {path.name for path in runs_dir.iterdir() if path.is_dir()} if runs_dir.exists() else set()
        cmd = [
            "python3",
            str(batch_evaluate_script),
            "--workspace",
            str(workspace_dir),
            "--seeds",
            *[str(seed) for seed in args.seeds],
            "--params",
            str(param_file),
            "--timeout",
            str(args.timeout),
            "--max-floor",
            str(args.max_floor),
            "--param-version",
            param_file.stem,
        ]
        try:
            subprocess.run(cmd, check=True, cwd=str(workspace_dir))
        except subprocess.CalledProcessError as exc:
            print(f"当前参数组合执行失败: {exc}")
            continue

        after_runs = {path.name for path in runs_dir.iterdir() if path.is_dir()} if runs_dir.exists() else set()
        new_runs = after_runs - before_runs
        scores = get_run_scores(summary_csv, new_runs)
        if not scores:
            print("未能从 summary.csv 获取当前参数组合得分")
            continue

        avg_score = sum(scores) / len(scores)
        worst_score = min(scores)
        combined_score = 0.7 * avg_score + 0.3 * worst_score
        print(f"平均分 {avg_score:.2f}，最差分 {worst_score:.2f}，综合指标 {combined_score:.2f}")
        all_results.append(
            {
                "params": params,
                "avg_score": avg_score,
                "worst_score": worst_score,
                "combined_score": combined_score,
                "runs": sorted(new_runs),
            }
        )
        if combined_score > best_score:
            best_score = combined_score
            best_params = params

    if args.dry_run:
        print(f"候选参数文件已生成到 {tune_dir}")
        return 0

    if best_params is None:
        print("调参结束，但没有产生有效评分")
        return 1

    best_params_path = workspace_dir / "results" / "best_params.yaml"
    write_yaml(best_params_path, best_params)
    report = {
        "best_params": best_params,
        "best_score": best_score,
        "all_results": all_results,
    }
    tuning_report = tune_dir / "tuning_report.json"
    tuning_report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"最佳参数已保存至 {best_params_path}，综合得分 {best_score:.2f}")

    subprocess.run(["python3", str(report_script), "--workspace", str(workspace_dir)], check=True, cwd=str(workspace_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

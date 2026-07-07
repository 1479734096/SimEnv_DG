#!/usr/bin/env python3
import argparse
import itertools
import json
import os
import subprocess
import sys
import yaml
from pathlib import Path
import csv

def generate_param_combinations(param_grid: dict):
    keys = list(param_grid.keys())
    values = list(param_grid.values())
    for combination in itertools.product(*values):
        yield dict(zip(keys, combination))

def save_yaml(data: dict, path: str):
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)

def get_run_scores(csv_path: Path, run_ids: set) -> list:
    scores = []
    if not csv_path.exists():
        return scores
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['run_id'] in run_ids:
                scores.append(float(row['total_score']))
    return scores

def main():
    parser = argparse.ArgumentParser(description="网格搜索参数调优")
    parser.add_argument("--seeds", type=int, nargs="+", default=[10, 42, 100], help="用于评测的种子集合")
    parser.add_argument("--test-mode", action="store_true", help="测试模式，减少候选参数组合")
    args = parser.parse_args()

    workspace_dir = Path("/home/xbh/tzb26/codeDG/SimEnv")
    tools_dir = workspace_dir / "src" / "danger_search" / "tools"
    scripts_dir = workspace_dir / "src" / "danger_search" / "scripts"
    batch_evaluate_script = scripts_dir / "batch_evaluate.py"
    
    if not batch_evaluate_script.exists():
        print(f"找不到 batch_evaluate.py: {batch_evaluate_script}")
        sys.exit(1)

    # 定义网格搜索参数
    if args.test_mode:
        param_grid = {
            "tracker/nms_radius": [0.8, 1.0],
            "detection/min_confidence": [0.70]
        }
    else:
        param_grid = {
            "tracker/nms_radius": [0.8, 0.9, 1.0],
            "tracker/min_hits": [3, 4, 5],
            "detection/min_confidence": [0.65, 0.75, 0.80]
        }

    combinations = list(generate_param_combinations(param_grid))
    print(f"共有 {len(combinations)} 组参数候选。")

    tune_dir = workspace_dir / "results" / "tuning"
    tune_dir.mkdir(parents=True, exist_ok=True)
    
    csv_path = workspace_dir / "results" / "summary.csv"
    
    best_score = -1
    best_params = None
    all_results = []

    # 记录已有的 runs，以便区分本次执行的新 runs
    runs_dir = workspace_dir / "results" / "runs"
    
    for i, params in enumerate(combinations):
        print(f"\n[{i+1}/{len(combinations)}] 正在评估参数组合: {params}")
        
        param_file = tune_dir / f"params_{i}.yaml"
        save_yaml(params, str(param_file))
        
        # 记录执行前的 runs
        existing_runs = set(p.name for p in runs_dir.iterdir() if p.is_dir()) if runs_dir.exists() else set()
        
        cmd = [
            "python3", str(batch_evaluate_script),
            "--seeds"
        ] + [str(s) for s in args.seeds] + [
            "--params", str(param_file)
        ]
        
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError:
            print("当前参数组合执行失败。")
            continue
            
        # 记录执行后的 runs，找出新增的 runs
        current_runs = set(p.name for p in runs_dir.iterdir() if p.is_dir()) if runs_dir.exists() else set()
        new_runs = current_runs - existing_runs
        
        scores = get_run_scores(csv_path, new_runs)
        if scores:
            avg_score = sum(scores) / len(scores)
            worst_score = min(scores)
            print(f"参数组合得分 -> 平均: {avg_score:.2f}, 最差: {worst_score:.2f}")
            
            all_results.append({
                "params": params,
                "avg_score": avg_score,
                "worst_score": worst_score,
                "runs": list(new_runs)
            })
            
            # 使用平均分和最差分的综合指标
            combined_score = avg_score * 0.7 + worst_score * 0.3
            if combined_score > best_score:
                best_score = combined_score
                best_params = params
        else:
            print("未能获取当前参数组合的得分。")

    if best_params:
        best_params_path = workspace_dir / "results" / "best_params.yaml"
        save_yaml(best_params, str(best_params_path))
        print(f"\n调优完成！最佳参数已保存至 {best_params_path}")
        print(f"最佳参数综合得分: {best_score:.2f}")
        print(f"参数内容: {best_params}")
        
        # 写入调优报告 JSON
        report = {
            "best_params": best_params,
            "best_score": best_score,
            "all_results": all_results
        }
        with open(tune_dir / "tuning_report.json", 'w') as f:
            json.dump(report, f, indent=2)
    else:
        print("\n调优完成，但未能产生有效得分。")

if __name__ == "__main__":
    main()

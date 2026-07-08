#!/usr/bin/env python3
import argparse
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from danger_search.evaluation_tools import find_workspace_dir


def main():
    parser = argparse.ArgumentParser(description="批量评测多个 seed")
    parser.add_argument("--seeds", type=int, nargs="+", required=True, help="要评测的随机种子列表")
    parser.add_argument("--params", type=str, help="参数重载 JSON 或 YAML 文件路径")
    parser.add_argument("--timeout", type=int, default=620, help="算法运行最大超时时间(秒)")
    parser.add_argument("--max-floor", type=int, default=3, help="最大探索楼层数")
    parser.add_argument("--workspace", type=str, help="SimEnv 工作区路径，默认自动寻找")
    parser.add_argument("--scene-type", default="", help="写入归档的场景类型标签")
    parser.add_argument("--param-version", default="", help="写入归档的参数版本标签")
    parser.add_argument("--no-report", action="store_true", help="只生成 summary.csv，不生成 optimization_report.md")
    args = parser.parse_args()

    workspace_dir = find_workspace_dir(args.workspace or Path(__file__))
    tools_dir = workspace_dir / "src" / "danger_search" / "tools"
    run_seed_script = tools_dir / "run_seed.py"
    summarize_script = tools_dir / "summarize_results.py"
    report_script = tools_dir / "generate_optimization_report.py"

    if not run_seed_script.exists():
        print(f"找不到 run_seed.py: {run_seed_script}")
        sys.exit(1)

    print(f"开始批量评测，共 {len(args.seeds)} 个 seed: {args.seeds}")

    success_count = 0
    for i, seed in enumerate(args.seeds):
        print(f"\n[{i+1}/{len(args.seeds)}] 正在评测 Seed: {seed}")
        cmd = [
            "python3", str(run_seed_script),
            "--seed", str(seed),
            "--timeout", str(args.timeout),
            "--max-floor", str(args.max_floor),
            "--workspace", str(workspace_dir),
        ]
        if args.params:
            cmd.extend(["--params", args.params])
        if args.scene_type:
            cmd.extend(["--scene-type", args.scene_type])
        if args.param_version:
            cmd.extend(["--param-version", args.param_version])

        try:
            subprocess.run(cmd, check=True)
            success_count += 1
        except subprocess.CalledProcessError:
            print(f"Seed {seed} 评测失败！继续下一个...")
            continue

    print(f"\n批量评测完成！成功: {success_count}/{len(args.seeds)}")
    
    if summarize_script.exists():
        print("正在生成汇总报告...")
        subprocess.run(["python3", str(summarize_script), "--workspace", str(workspace_dir)], check=True)
    else:
        print("未找到 summarize_results.py，无法生成汇总报告。")

    if not args.no_report and report_script.exists():
        print("正在生成优化报告...")
        subprocess.run(["python3", str(report_script), "--workspace", str(workspace_dir)], check=True)


if __name__ == "__main__":
    main()

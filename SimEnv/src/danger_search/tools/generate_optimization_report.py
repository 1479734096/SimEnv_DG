#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from danger_search.evaluation_tools import build_optimization_report, find_workspace_dir


def main() -> int:
    parser = argparse.ArgumentParser(description="生成 danger_search 参数优化报告")
    parser.add_argument("--workspace", type=str, help="SimEnv 工作区路径，默认自动寻找")
    parser.add_argument("--summary", type=str, help="summary.csv 路径")
    parser.add_argument("--tuning-report", type=str, help="tuning_report.json 路径")
    parser.add_argument("--output", type=str, help="输出 Markdown，默认 results/optimization_report.md")
    args = parser.parse_args()

    workspace_dir = find_workspace_dir(args.workspace or Path(__file__))
    results_dir = workspace_dir / "results"
    output_path = Path(args.output) if args.output else results_dir / "optimization_report.md"
    content = build_optimization_report(
        results_dir=results_dir,
        tuning_report_path=args.tuning_report,
        summary_csv_path=args.summary,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(content, encoding="utf-8")
    print(f"优化报告已生成至 {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

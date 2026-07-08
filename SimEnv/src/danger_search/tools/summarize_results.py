#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from danger_search.evaluation_tools import find_workspace_dir, write_summary_csv


def main() -> int:
    parser = argparse.ArgumentParser(description="汇总 danger_search 历史运行记录")
    parser.add_argument("--workspace", type=str, help="SimEnv 工作区路径，默认自动寻找")
    parser.add_argument("--runs-dir", type=str, help="运行归档目录，默认 results/runs")
    parser.add_argument("--output", type=str, help="汇总 CSV，默认 results/summary.csv")
    args = parser.parse_args()

    workspace_dir = find_workspace_dir(args.workspace or Path(__file__))
    results_dir = workspace_dir / "results"
    runs_dir = Path(args.runs_dir) if args.runs_dir else results_dir / "runs"
    output_file = Path(args.output) if args.output else results_dir / "summary.csv"

    rows = write_summary_csv(runs_dir, output_file)
    print(f"成功汇总 {len(rows)} 条记录至 {output_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

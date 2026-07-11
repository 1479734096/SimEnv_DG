"""danger_search 系统级回归运行器。

支持在多个 Git ref（HEAD、当前工作树、任意分支/SHA）之间横向对比同一组 seed。
用法示例：
    python3 src/danger_search/tools/regression_runner.py \
        --refs HEAD,WORKTREE \
        --seeds 42,123 \
        --params config/medium_time_budget_test.yaml \
        --timeout 1500
"""
from __future__ import annotations

import argparse
import datetime
import json
import shutil
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path
from typing import Any

# 允许作为脚本直接运行：把 src/danger_search/ 加入路径
_SRC_DIR = Path(__file__).resolve().parents[1]
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from danger_search.evaluation_tools import find_workspace_dir, write_json
from danger_search.regression_metrics import RunMetrics, extract_run_metrics


def run_git(workspace: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", *args],
        cwd=str(workspace),
        check=check,
        capture_output=True,
        text=True,
    )


def resolve_ref(workspace: Path, ref: str) -> str:
    """将用户指定的 ref 解析为 commit SHA。"""
    ref = ref.strip()
    if ref.upper() == "WORKTREE":
        result = run_git(workspace, "rev-parse", "HEAD")
        return f"WORKTREE@{result.stdout.strip()}"
    result = run_git(workspace, "rev-parse", ref)
    return result.stdout.strip()


def create_worktree(workspace: Path, ref: str, worktree_dir: Path) -> None:
    """为指定 ref 创建独立工作树，并共享 devel/build 以减少构建时间。"""
    resolved = ref
    if ref.upper() == "WORKTREE":
        # 当前工作树直接使用主目录，不需要额外 worktree
        return
    worktree_dir.mkdir(parents=True, exist_ok=True)
    try:
        run_git(workspace, "worktree", "add", "--force", str(worktree_dir), resolved)
    except subprocess.CalledProcessError as exc:
        # 可能已存在，尝试 reset 到目标 ref
        run_git(worktree_dir, "reset", "--hard", resolved)

    # 共享构建产物：符号链接 devel 和 build
    import os
    for name in ("devel", "build"):
        source = workspace / name
        target = worktree_dir / name
        if target.exists() or target.is_symlink():
            if target.is_symlink() or target.is_dir():
                shutil.rmtree(target, ignore_errors=True)
            else:
                target.unlink(missing_ok=True)
        if source.exists():
            os.symlink(str(source.resolve()), str(target), target_is_directory=True)


def remove_worktree(workspace: Path, worktree_dir: Path) -> None:
    if worktree_dir == workspace:
        return
    shutil.rmtree(worktree_dir, ignore_errors=True)
    try:
        run_git(workspace, "worktree", "prune")
    except subprocess.CalledProcessError:
        pass


def run_seed_in_worktree(
    worktree: Path,
    seed: int,
    params: Path,
    timeout: int,
    max_floor: int,
    gui: bool,
) -> Path | None:
    """在指定工作树中调用 run_seed.py，返回 run 目录。"""
    # worktree 是 git 根目录，里面包含 SimEnv 子目录；需要定位到 SimEnv 工作区
    inner_workspace = find_workspace_dir(worktree)
    run_seed_script = inner_workspace / "src" / "danger_search" / "tools" / "run_seed.py"
    if not run_seed_script.exists():
        print(f"[错误] 找不到 {run_seed_script}", file=sys.stderr)
        return None

    cmd = [
        "python3",
        str(run_seed_script),
        "--seed",
        str(seed),
        "--params",
        str(params),
        "--timeout",
        str(timeout),
        "--max-floor",
        str(max_floor),
        "--workspace",
        str(inner_workspace),
    ]
    if gui:
        cmd.append("--gui")
    else:
        cmd.append("--headless")

    print(f"[运行] {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True, cwd=str(inner_workspace))
    except subprocess.CalledProcessError as exc:
        print(f"[警告] run_seed 异常退出: {exc}", file=sys.stderr)

    # run_seed.py 会自动创建 results/runs/<timestamp>_seed_<seed>
    # 通过查找最新目录确定本次 run
    runs_dir = inner_workspace / "results" / "runs"
    if not runs_dir.exists():
        return None
    candidates = sorted(
        (d for d in runs_dir.iterdir() if d.is_dir() and f"_seed_{seed}" in d.name),
        key=lambda d: d.stat().st_mtime,
        reverse=True,
    )
    return candidates[0] if candidates else None


def copy_run_to_report_dir(run_dir: Path, report_dir: Path, ref: str, seed: int) -> Path:
    """将 run 目录复制到报告目录，避免工作树被清理后丢失。"""
    dest = report_dir / "runs" / ref / f"seed_{seed}"
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(run_dir, dest)
    return dest


def format_metrics_table(results: dict[str, dict[int, RunMetrics]]) -> str:
    """生成 Markdown 指标对比表。"""
    lines: list[str] = []
    lines.append("| ref | seed | 完成 | 返航成功 | 探索时间 | 最大离家距离 | 已知面积 | 识别数 | STUCK | recover | 重访比例 | 区域数 | 硬失败 |")
    lines.append("|-----|------|------|----------|----------|--------------|----------|--------|-------|---------|----------|--------|--------|")
    for ref in sorted(results.keys()):
        for seed in sorted(results[ref].keys()):
            m = results[ref][seed]
            lines.append(
                f"| {ref} | {seed} | {m.completed} | {m.return_home_success} | "
                f"{m.exploration_time:.1f} | {m.max_home_distance:.2f} | {m.known_area:.1f} | "
                f"{m.correct_count} | {m.stuck_count} | {m.recover_count} | "
                f"{m.trajectory.repeat_ratio:.2f} | {m.trajectory.region_count} | {m.hard_failure} |"
            )
    return "\n".join(lines)


def generate_report(
    report_dir: Path,
    results: dict[str, dict[int, RunMetrics]],
    config_path: Path,
    timeout: int,
    max_floor: int,
) -> Path:
    """生成回归报告 JSON 和 Markdown。"""
    report_dir.mkdir(parents=True, exist_ok=True)

    # 序列化结果
    serializable: dict[str, dict[str, dict[str, Any]]] = {}
    for ref, seed_map in results.items():
        serializable[ref] = {}
        for seed, metrics in seed_map.items():
            serializable[ref][str(seed)] = {
                "run_id": metrics.run_id,
                "seed": metrics.seed,
                "ref": metrics.ref,
                "completed": metrics.completed,
                "return_home_success": metrics.return_home_success,
                "terminal_reason": metrics.terminal_reason,
                "hard_failure": metrics.hard_failure,
                "return_failed": metrics.return_failed,
                "severe_stuck": metrics.severe_stuck,
                "wrong_exit": metrics.wrong_exit,
                "state_loop": metrics.state_loop,
                "door_pass_failed": metrics.door_pass_failed,
                "exploration_time": metrics.exploration_time,
                "max_home_distance": metrics.max_home_distance,
                "known_area": metrics.known_area,
                "visited_area": metrics.visited_area,
                "coverage_ratio": metrics.coverage_ratio,
                "confirmed_count": metrics.confirmed_count,
                "correct_count": metrics.correct_count,
                "missed_count": metrics.missed_count,
                "false_alarm_count": metrics.false_alarm_count,
                "stuck_count": metrics.stuck_count,
                "recover_count": metrics.recover_count,
                "stuck_per_minute": metrics.stuck_per_minute,
                "recover_per_minute": metrics.recover_per_minute,
                "trajectory": {
                    "sample_count": metrics.trajectory.sample_count,
                    "actual_distance": metrics.trajectory.actual_distance,
                    "max_home_distance": metrics.trajectory.max_home_distance,
                    "repeat_ratio": metrics.trajectory.repeat_ratio,
                    "region_count": metrics.trajectory.region_count,
                    "long_stop_ratio": metrics.trajectory.long_stop_ratio,
                    "long_stop_count": metrics.trajectory.long_stop_count,
                    "median_speed": metrics.trajectory.median_speed,
                },
                "failure_reasons": metrics.failure_reasons,
            }

    json_path = report_dir / "regression_report.json"
    write_json(json_path, serializable)

    # Markdown 报告
    md_lines: list[str] = []
    md_lines.append("# danger_search 回归报告")
    md_lines.append("")
    md_lines.append(f"生成时间：{datetime.datetime.now().isoformat()}")
    md_lines.append(f"配置文件：{config_path}")
    md_lines.append(f"运行超时：{timeout}s | 最大楼层：{max_floor}")
    md_lines.append("")
    md_lines.append("## 指标对比")
    md_lines.append("")
    md_lines.append(format_metrics_table(results))
    md_lines.append("")

    # 硬失败汇总
    md_lines.append("## 硬失败汇总")
    md_lines.append("")
    failures = [
        (ref, seed, m)
        for ref, seed_map in results.items()
        for seed, m in seed_map.items()
        if m.hard_failure
    ]
    if failures:
        for ref, seed, m in failures:
            reasons = []
            if m.return_failed:
                reasons.append("返航失败")
            if m.severe_stuck:
                reasons.append("严重卡死")
            if m.wrong_exit:
                reasons.append("错误出门")
            if m.state_loop:
                reasons.append("状态循环")
            if m.door_pass_failed:
                reasons.append("门任务失败")
            md_lines.append(f"- **{ref} / seed {seed}**: {', '.join(reasons)}")
            if m.failure_reasons:
                md_lines.append(f"  - 失败原因：{'; '.join(m.failure_reasons)}")
    else:
        md_lines.append("- 无硬失败")
    md_lines.append("")

    # 关键指标说明
    md_lines.append("## 指标说明")
    md_lines.append("")
    md_lines.append(textwrap.dedent(
        """
        - **重访比例**：对 odom 位姿按 0.5m 网格降采样后，落在被访问过多次的网格中的采样点占比。
        - **区域数**：轨迹降采样后按 4-邻域连通分量划分，作为探索分支/区域的代理指标。
        - **硬失败**：返航失败、严重卡死（STUCK≥10 或长停>30%）、错误出门、状态循环、门任务失败任一满足。
        - **coverage_ratio**：仅作辅助观察，不作为主要评分。
        """
    ).strip())
    md_lines.append("")

    md_path = report_dir / "regression_report.md"
    md_path.write_text("\n".join(md_lines), encoding="utf-8")

    print(f"报告已保存: {json_path}")
    print(f"Markdown 报告: {md_path}")
    return json_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="danger_search 系统级回归运行器")
    parser.add_argument(
        "--refs",
        type=str,
        required=True,
        help="逗号分隔的 Git ref 列表，支持 HEAD、WORKTREE、分支名或 SHA",
    )
    parser.add_argument(
        "--seeds",
        type=str,
        required=True,
        help="逗号分隔的 seed 列表",
    )
    parser.add_argument(
        "--params",
        type=str,
        default="config/medium_time_budget_test.yaml",
        help="参数覆盖 YAML 文件路径",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=1500,
        help="每个 seed 的运行超时（秒）",
    )
    parser.add_argument(
        "--max-floor",
        type=int,
        default=1,
        help="最大探索楼层数",
    )
    parser.add_argument(
        "--workspace",
        type=str,
        help="SimEnv 工作区路径，默认自动寻找",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="强制启动 Gazebo GUI（默认 headless）",
    )
    parser.add_argument(
        "--keep-worktrees",
        action="store_true",
        help="运行结束后保留临时 worktree",
    )
    parser.add_argument(
        "--report-dir",
        type=str,
        help="报告输出目录，默认 results/regression/<timestamp>",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    workspace = find_workspace_dir(args.workspace)
    refs = [r.strip() for r in args.refs.split(",") if r.strip()]
    seeds = [int(s.strip()) for s in args.seeds.split(",") if s.strip()]
    params_path = Path(args.params)
    if not params_path.is_absolute():
        params_path = workspace / params_path

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    report_dir = Path(args.report_dir) if args.report_dir else workspace / "results" / "regression" / timestamp
    report_dir.mkdir(parents=True, exist_ok=True)
    print(f"回归报告目录: {report_dir}")

    # 解析并准备 worktree
    worktrees: dict[str, Path] = {}
    temp_dirs: list[Path] = []
    try:
        for ref in refs:
            resolved = resolve_ref(workspace, ref)
            if ref.upper() == "WORKTREE":
                worktrees[ref] = workspace
            else:
                wt_dir = Path(tempfile.mkdtemp(prefix=f"regression_{ref.replace('/', '_')}_", dir=str(report_dir)))
                temp_dirs.append(wt_dir)
                print(f"为 ref={ref} ({resolved}) 创建工作树: {wt_dir}")
                create_worktree(workspace, resolved, wt_dir)
                worktrees[ref] = wt_dir

        results: dict[str, dict[int, RunMetrics]] = {ref: {} for ref in refs}
        for ref in refs:
            wt = worktrees[ref]
            for seed in seeds:
                print(f"\n===== ref={ref}, seed={seed} =====")
                run_dir = run_seed_in_worktree(
                    wt,
                    seed,
                    params_path,
                    args.timeout,
                    args.max_floor,
                    args.gui,
                )
                if run_dir is None:
                    print(f"[警告] 未找到 ref={ref} seed={seed} 的 run 目录", file=sys.stderr)
                    continue
                archived = copy_run_to_report_dir(run_dir, report_dir, ref, seed)
                metrics = extract_run_metrics(archived, ref=ref)
                results[ref][seed] = metrics
                print(f"归档: {archived}")

        generate_report(report_dir, results, params_path, args.timeout, args.max_floor)

    finally:
        if not args.keep_worktrees:
            for ref in refs:
                wt = worktrees.get(ref)
                if wt and wt != workspace:
                    print(f"清理 worktree: {wt}")
                    remove_worktree(workspace, wt)
            for td in temp_dirs:
                shutil.rmtree(td, ignore_errors=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

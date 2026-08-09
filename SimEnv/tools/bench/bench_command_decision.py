#!/usr/bin/env python3
"""控制命令决策热路径微基准。

用途
----
命令写入权重构（R1）会把散落的写入点收口到统一裁决入口，链路上多一层
间接调用。本基准固化重构前的决策热路径耗时，用于每阶段验收时判断中位数
回归是否 < 10%。

方法
----
- 固定输入：所有目标使用写死的输入，不含随机数、不读文件、不联网。
- 预热 5 批，测量 40 批（要求 >= 20）。
- 每批内跑 ``inner`` 次，用批耗时除以 ``inner`` 得到单次耗时，抵消计时器
  分辨率噪声（单次调用只有几微秒，直接计时不可靠）。
- 记录 median 与 p95；median 用于回归判定，p95 用于观察长尾。

确定性守卫
----------
带状态的目标（PathFollower、地面支撑门）可能在测量期间因超时、锁存或路径
推进而改变行为，此时耗时数字不可比。每个目标提供 ``signature``，基准在预热
后与全部测量后各取一次；两次不一致即判为 NON-DETERMINISTIC 并以非零码退出，
不输出会被误用的数字。

用法
----
    # 建立基线
    python3 tools/bench/bench_command_decision.py --json reports/bench/r0_baseline.json

    # 阶段验收：与基线比对，中位数回归 >= 10% 时退出码非零
    python3 tools/bench/bench_command_decision.py --baseline reports/bench/r0_baseline.json
"""
from __future__ import annotations

import argparse
import json
import math
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

from danger_search.ground_support_gate import GroundSupportSafetyGate  # noqa: E402
from danger_search.navigator import PathFollower, compute_tracking_command  # noqa: E402

WARMUP_BATCHES = 5
DEFAULT_REPS = 40
REGRESSION_THRESHOLD = 0.10

# 固定输入：跟踪决策的典型工况（有航向误差、有曲率、前方有障碍、有实测角速度）。
_TRACK_KWARGS = dict(
    max_linear=0.6,
    max_angular=0.8,
    path_curvature=0.1,
    nearest_obstacle_distance=1.5,
    measured_yaw_rate=0.05,
)
# 固定路径与位姿：位姿离终点足够远，重复 tick 进入稳态且不会走完路径。
_FIXED_PATH = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.5), (3.0, 1.0)]
_FIXED_POSE = (0.2, 0.0, 0.0)
_FIXED_COMMAND = (0.4, 0.0, 0.1)


def _new_follower(with_gate: bool) -> PathFollower:
    follower = PathFollower(cmd_vel_topic="/cmd_vel")
    if with_gate:
        follower.set_command_gate(GroundSupportSafetyGate())
    return follower


# ---------------------------------------------------------------- 目标定义
# 每个目标: setup() -> state; run(state) -> None; signature(state) -> 可比对象


def _t_compute_tracking_command():
    def run(_state):
        compute_tracking_command(0.25, **_TRACK_KWARGS)

    def signature(_state):
        return tuple(round(v, 9) for v in compute_tracking_command(0.25, **_TRACK_KWARGS))

    return None, run, signature


def _t_publish_locked_no_gate():
    state = _new_follower(with_gate=False)

    def run(follower):
        follower._publish_locked(_FIXED_COMMAND[0], _FIXED_COMMAND[2], linear_y=_FIXED_COMMAND[1])

    def signature(follower):
        return (
            tuple(round(float(v), 9) for v in follower._last_emitted_command),
            follower._debug_zero_command_reason,
            follower._publish_failure_latched,
        )

    return state, run, signature


def _t_publish_locked_with_gate():
    state = _new_follower(with_gate=True)

    def run(follower):
        follower._publish_locked(_FIXED_COMMAND[0], _FIXED_COMMAND[2], linear_y=_FIXED_COMMAND[1])

    def signature(follower):
        gate = follower._command_gate
        return (
            tuple(round(float(v), 9) for v in follower._last_emitted_command),
            follower._debug_zero_command_reason,
            getattr(gate, "state", None),
            getattr(gate, "reason_code", None),
        )

    return state, run, signature


def _t_publish_with_lock():
    """``_publish``：含事务锁获取与直接命令时间戳，是任务层直接控制的入口。"""
    state = _new_follower(with_gate=True)

    def run(follower):
        follower._publish(_FIXED_COMMAND[0], _FIXED_COMMAND[2], linear_y=_FIXED_COMMAND[1])

    def signature(follower):
        gate = follower._command_gate
        return (
            tuple(round(float(v), 9) for v in follower._last_emitted_command),
            follower._debug_zero_command_reason,
            getattr(gate, "state", None),
        )

    return state, run, signature


def _t_gate_filter_command():
    state = GroundSupportSafetyGate()

    def run(gate):
        gate.filter_command(
            _FIXED_COMMAND,
            pose_yaw=0.0,
            wall_time=time.monotonic(),
            control_time=time.monotonic(),
        )

    def signature(gate):
        out = gate.filter_command(
            _FIXED_COMMAND, pose_yaw=0.0, wall_time=time.monotonic(), control_time=time.monotonic()
        )
        return (
            tuple(round(float(v), 9) for v in out),
            getattr(gate, "state", None),
            getattr(gate, "reason_code", None),
        )

    return state, run, signature


def _t_path_follower_tick():
    state = _new_follower(with_gate=False)
    state.set_path(list(_FIXED_PATH))

    def run(follower):
        follower.tick(_FIXED_POSE)

    def signature(follower):
        return (
            follower.tick(_FIXED_POSE),
            follower.has_path(),
            tuple(round(float(v), 9) for v in follower._last_emitted_command),
        )

    return state, run, signature


TARGETS = {
    # name: (factory, inner) —— inner 取值使单批耗时落在 ~10ms 量级
    "compute_tracking_command": (_t_compute_tracking_command, 2000),
    "publish_locked_no_gate": (_t_publish_locked_no_gate, 5000),
    "publish_locked_with_gate": (_t_publish_locked_with_gate, 2000),
    "publish_with_lock": (_t_publish_with_lock, 2000),
    "gate_filter_command": (_t_gate_filter_command, 2000),
    "path_follower_tick": (_t_path_follower_tick, 300),
}


# ---------------------------------------------------------------- 测量


def _percentile(values, fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = int(math.ceil(fraction * len(ordered))) - 1
    return ordered[max(0, min(len(ordered) - 1, index))]


def _measure(name: str, reps: int) -> dict:
    factory, inner = TARGETS[name]
    state, run, signature = factory()

    for _ in range(WARMUP_BATCHES):
        for _ in range(inner):
            run(state)

    sig_before = signature(state)

    samples = []
    for _ in range(reps):
        start = time.perf_counter()
        for _ in range(inner):
            run(state)
        samples.append((time.perf_counter() - start) / inner * 1e6)

    sig_after = signature(state)
    deterministic = sig_before == sig_after

    return {
        "inner": inner,
        "reps": reps,
        "median_us": round(statistics.median(samples), 4),
        "p95_us": round(_percentile(samples, 0.95), 4),
        "min_us": round(min(samples), 4),
        "mean_us": round(statistics.mean(samples), 4),
        "deterministic": deterministic,
        "signature": repr(sig_before),
        "signature_after": None if deterministic else repr(sig_after),
    }


def _git_head() -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        return out.stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="控制命令决策热路径微基准")
    parser.add_argument("--reps", type=int, default=DEFAULT_REPS, help="测量批数（>=20）")
    parser.add_argument("--json", type=Path, default=None, help="结果写入路径")
    parser.add_argument("--baseline", type=Path, default=None, help="基线 JSON，用于回归比对")
    parser.add_argument(
        "--threshold",
        type=float,
        default=REGRESSION_THRESHOLD,
        help="中位数回归阈值，默认 0.10",
    )
    parser.add_argument("--only", action="append", default=None, help="只跑指定目标，可重复")
    args = parser.parse_args(argv)

    if args.reps < 20:
        print(f"[错误] reps={args.reps} < 20，不满足最小测量次数要求", file=sys.stderr)
        return 2

    names = args.only or list(TARGETS)
    unknown = [n for n in names if n not in TARGETS]
    if unknown:
        print(f"[错误] 未知目标: {unknown}；可选: {list(TARGETS)}", file=sys.stderr)
        return 2

    result = {
        "git_head": _git_head(),
        "python": platform.python_version(),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "warmup_batches": WARMUP_BATCHES,
        "targets": {},
    }

    print(f"git_head={result['git_head'][:12]}  python={result['python']}  预热={WARMUP_BATCHES} 批  测量={args.reps} 批")
    print(f"{'target':<28}{'median_us':>12}{'p95_us':>12}{'min_us':>12}{'inner':>8}{'det':>6}")
    non_deterministic = []
    for name in names:
        entry = _measure(name, args.reps)
        result["targets"][name] = entry
        flag = "ok" if entry["deterministic"] else "NO"
        if not entry["deterministic"]:
            non_deterministic.append(name)
        print(
            f"{name:<28}{entry['median_us']:>12.3f}{entry['p95_us']:>12.3f}"
            f"{entry['min_us']:>12.3f}{entry['inner']:>8}{flag:>6}"
        )

    if non_deterministic:
        print("\n[NON-DETERMINISTIC] 以下目标在测量期间行为签名发生变化，数字不可比：", file=sys.stderr)
        for name in non_deterministic:
            entry = result["targets"][name]
            print(f"  {name}\n    before={entry['signature']}\n    after ={entry['signature_after']}", file=sys.stderr)

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"\n结果已写入 {args.json}")

    exit_code = 1 if non_deterministic else 0

    if args.baseline:
        baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
        base_targets = baseline.get("targets", {})
        print(f"\n与基线比对（{args.baseline}，git_head={baseline.get('git_head','?')[:12]}，阈值 {args.threshold:.0%}）")
        print(f"{'target':<28}{'base_us':>12}{'now_us':>12}{'delta':>10}{'verdict':>10}")
        regressed = []
        missing = []
        for name in names:
            base = base_targets.get(name)
            if base is None:
                missing.append(name)
                print(f"{name:<28}{'-':>12}{result['targets'][name]['median_us']:>12.3f}{'-':>10}{'NO-BASE':>10}")
                continue
            base_us = float(base["median_us"])
            now_us = float(result["targets"][name]["median_us"])
            delta = (now_us - base_us) / base_us if base_us > 0 else float("inf")
            verdict = "REGRESS" if delta >= args.threshold else "pass"
            if verdict == "REGRESS":
                regressed.append((name, base_us, now_us, delta))
            print(f"{name:<28}{base_us:>12.3f}{now_us:>12.3f}{delta:>9.1%}{verdict:>10}")
        if missing:
            print(f"\n[警告] 基线缺少目标: {missing}", file=sys.stderr)
        if regressed:
            print(f"\n[回归] {len(regressed)} 个目标中位数回归 >= {args.threshold:.0%}：", file=sys.stderr)
            for name, base_us, now_us, delta in regressed:
                print(f"  {name}: {base_us:.3f} -> {now_us:.3f} us ({delta:+.1%})", file=sys.stderr)
            exit_code = 1
        elif not missing:
            print("\n全部目标中位数回归 < 阈值")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())

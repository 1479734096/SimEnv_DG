#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime
import os
import shutil
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from danger_search.evaluation_tools import (
    create_run_dir,
    default_config_path,
    find_workspace_dir,
    read_json,
    write_effective_config,
    write_json,
    write_yaml,
)


PROCESS_PATTERNS = [
    "roslaunch unitree_guide multi_floor_gazeboSim.launch",
    "building_generator_classic_control",
    "gzserver|gzclient|gazebo",
    "junior_ctrl",
    "danger_search_node",
]


DEFAULT_HEALTH_CHECK_TIMEOUT = 120


def cleanup() -> None:
    print("清理残留进程...")
    for pattern in PROCESS_PATTERNS:
        subprocess.run(
            ["pkill", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    time.sleep(2.0)


def resolve_gui_enabled(args: argparse.Namespace) -> bool:
    if args.gui:
        if not os.environ.get("DISPLAY"):
            raise RuntimeError(
                "当前环境没有 DISPLAY，不能使用 --gui 启动 Gazebo 图形界面；"
                "请改用 --headless，或使用 xvfb-run -a python3 src/danger_search/tools/run_seed.py ..."
            )
        return True
    if args.headless:
        return False
    return bool(os.environ.get("DISPLAY"))


def run_environment(seed: int, workspace_dir: Path, log_file, *, gui_enabled: bool, health_log_path: Path):
    try:
        import pexpect
    except ImportError as exc:
        raise RuntimeError("缺少 pexpect，无法自动给 junior_ctrl 输入 2/6") from exc

    env = os.environ.copy()
    env.update(
        {
            "SEED": str(seed),
            "GUI": "true" if gui_enabled else "false",
            "PAUSED": "false",
            "CONTROLLER_FOREGROUND": "1",
            "START_CONTROLLER": "1",
            "START_BUILDING_CONTROL": "1",
        }
    )

    print(f"[{datetime.datetime.now().time()}] 启动仿真环境 auto.sh ...")
    child = pexpect.spawn(
        str(workspace_dir / "auto.sh"),
        env=env,
        encoding="utf-8",
        logfile=log_file,
        cwd=str(workspace_dir),
    )
    try:
        child.expect("Use keyboard input in this terminal", timeout=120)
        print(f"[{datetime.datetime.now().time()}] 控制器进程已启动，等待 RL 模型加载完成")
        child.expect("load model to device!", timeout=120)
        print(f"[{datetime.datetime.now().time()}] RL 模型已加载，发送 2 进入站立")
        child.send("2")
        child.expect("Switched from passive to fixed stand", timeout=30)
        print(f"[{datetime.datetime.now().time()}] 已进入站立，等待起立完成后发送 6 进入 RL 模式")
        time.sleep(5.0)
        child.send("6")
        child.expect("Switched from fixed stand to RL", timeout=30)
        print(f"[{datetime.datetime.now().time()}] 控制器已进入 RL，开始启动健康检查")
        check_startup_health(workspace_dir, health_log_path)
        time.sleep(1.0)
        print(f"[{datetime.datetime.now().time()}] 仿真环境准备完毕，控制器已进入 RL 模式")
    except pexpect.TIMEOUT:
        child.close(force=True)
        raise RuntimeError("启动仿真环境超时")
    except pexpect.EOF:
        child.close(force=True)
        raise RuntimeError("auto.sh 意外退出")
    return child


def wait_for_ros_topic(
    workspace_dir: Path,
    topic: str,
    timeout: int = 30,
    log_file=None,
    *,
    field: str | None = None,
) -> None:
    echo_target = topic if field is None else f"{topic}/{field.lstrip('/')}"
    cmd = " ".join(
        [
            "source /opt/ros/noetic/setup.bash",
            "&&",
            f"source {shlex.quote(str(workspace_dir / 'devel' / 'setup.bash'))}",
            "&&",
            f"rostopic echo -n 1 {shlex.quote(echo_target)}",
        ]
    )
    try:
        subprocess.run(
            ["bash", "-lc", cmd],
            cwd=str(workspace_dir),
            stdout=log_file if log_file is not None else subprocess.DEVNULL,
            stderr=subprocess.STDOUT if log_file is not None else subprocess.DEVNULL,
            timeout=timeout,
            check=True,
        )
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"等待 {topic} 数据超时") from exc


def check_startup_health(workspace_dir: Path, health_log_path: Path) -> None:
    required_topics = (
        ("/Odometry_gazebo", "里程计"),
        ("/livox/Pointcloud2", "点云"),
    )
    health_log_path.parent.mkdir(parents=True, exist_ok=True)
    with health_log_path.open("w", encoding="utf-8") as health_log:
        health_log.write(f"启动健康检查开始: {datetime.datetime.now().isoformat()}\n")
        for topic, label in required_topics:
            health_log.write(f"\n检查 {label} 话题 {topic}\n")
            health_log.flush()
            try:
                wait_for_ros_topic(
                    workspace_dir,
                    topic,
                    timeout=DEFAULT_HEALTH_CHECK_TIMEOUT,
                    log_file=health_log,
                    field="header",
                )
                health_log.write(f"{topic} 已收到首帧数据\n")
                health_log.flush()
            except RuntimeError as exc:
                health_log.write(f"{topic} 检查失败: {exc}\n")
                health_log.flush()
                if topic == "/livox/Pointcloud2":
                    raise RuntimeError(
                        "启动健康检查失败：未收到 /livox/Pointcloud2 点云；"
                        "请检查 Gazebo/Livox 插件和 pointcloud2livox.py，详见 logs/health_check.log"
                    ) from exc
                raise RuntimeError(
                    f"启动健康检查失败：未收到 {topic} 数据，详见 logs/health_check.log"
                ) from exc
        health_log.write(f"\n启动健康检查通过: {datetime.datetime.now().isoformat()}\n")


def run_danger_search(
    workspace_dir: Path,
    run_config_path: Path,
    result_file: Path,
    mission_summary_file: Path,
    max_floor: int,
    timeout: int,
    log_path: Path,
) -> tuple[int, bool]:
    launch_cmd = " ".join(
        [
            "source /opt/ros/noetic/setup.bash",
            "&&",
            f"source {shlex.quote(str(workspace_dir / 'devel' / 'setup.bash'))}",
            "&&",
            "roslaunch danger_search danger_search.launch",
            f"config:={shlex.quote(str(run_config_path))}",
            f"max_floor:={int(max_floor)}",
            f"result_file:={shlex.quote(str(result_file))}",
            f"mission_summary_file:={shlex.quote(str(mission_summary_file))}",
        ]
    )
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            ["bash", "-lc", launch_cmd],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            cwd=str(workspace_dir),
        )
        external_timeout = False
        try:
            process.wait(timeout=timeout)
            print(f"[{datetime.datetime.now().time()}] danger_search 节点退出，返回码 {process.returncode}")
        except subprocess.TimeoutExpired:
            external_timeout = True
            print(f"[{datetime.datetime.now().time()}] 达到超时限制 {timeout}s，发送 SIGINT")
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        return int(process.returncode or 0), external_timeout


def evaluate_run(workspace_dir: Path, run_dir: Path, result_file: Path) -> bool:
    truth_file = workspace_dir / "results" / "danger_truth.json"
    eval_output = run_dir / "evaluation.json"
    if not result_file.exists() or not truth_file.exists():
        print("未找到 detected_danger.json 或 danger_truth.json，跳过离线评测")
        return False

    eval_script = workspace_dir / "src" / "building_obstacles" / "scripts" / "evaluate_danger.py"
    cmd = [
        "python3",
        str(eval_script),
        "--truth-file",
        str(truth_file),
        "--detected-file",
        str(result_file),
        "--output-file",
        str(eval_output),
    ]
    subprocess.run(cmd, check=True, cwd=str(workspace_dir))
    shutil.copy2(truth_file, run_dir / "danger_truth.json")
    print(f"评测完成，结果保存在 {eval_output}")
    return True


def copy_runtime_logs(workspace_dir: Path, run_dir: Path) -> None:
    logs_dir = run_dir / "logs"
    logs_dir.mkdir(exist_ok=True)
    for name in ("competition_gazebo.log", "building_control.log", "junior_ctrl.log", "virtual_joy.log"):
        source = workspace_dir / "logs" / name
        if source.exists():
            shutil.copy2(source, logs_dir / name)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="运行单一 seed 评测并归档结果")
    parser.add_argument("--seed", type=int, required=True, help="随机种子")
    parser.add_argument("--params", type=str, help="参数重载 JSON 或 YAML 文件路径")
    parser.add_argument("--timeout", type=int, default=10000, help="算法运行最大超时时间，单位秒")
    parser.add_argument("--max-floor", type=int, default=3, help="最大探索楼层数")
    parser.add_argument("--workspace", type=str, help="SimEnv 工作区路径，默认自动寻找")
    parser.add_argument("--scene-type", default="", help="归档中的场景类型标签")
    parser.add_argument("--param-version", default="", help="归档中的参数版本标签")
    gui_group = parser.add_mutually_exclusive_group()
    gui_group.add_argument("--gui", action="store_true", help="强制启动 Gazebo GUI；无 DISPLAY 时会报错")
    gui_group.add_argument("--headless", action="store_true", help="强制关闭 Gazebo GUI")
    parser.add_argument("--skip-cleanup-before", action="store_true", help="启动前不清理旧仿真进程")
    parser.add_argument("--skip-cleanup-after", action="store_true", help="结束后不清理仿真进程")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    workspace_dir = find_workspace_dir(args.workspace or Path(__file__))
    try:
        gui_enabled = resolve_gui_enabled(args)
    except RuntimeError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2
    if not args.skip_cleanup_before:
        cleanup()

    run_dir = create_run_dir(workspace_dir, args.seed)
    print(f"运行归档目录: {run_dir}")
    logs_dir = run_dir / "logs"

    run_config_path = run_dir / "params.yaml"
    effective_config = write_effective_config(
        base_config_path=default_config_path(workspace_dir),
        override_path=args.params,
        output_path=run_config_path,
    )
    # 兼容早期脚本命名，方便人工 diff。
    write_yaml(run_dir / "danger_search.yaml", effective_config)

    result_file = run_dir / "detected_danger.json"
    runtime_summary_file = run_dir / "mission_runtime_summary.json"
    env_process = None
    failure_reason: list[str] = []
    algo_return_code: int | None = None
    external_timeout = False
    evaluation_completed = False
    started_at = datetime.datetime.now()

    try:
        with (logs_dir / "auto_sh.log").open("w", encoding="utf-8") as env_log:
            env_process = run_environment(
                args.seed,
                workspace_dir,
                env_log,
                gui_enabled=gui_enabled,
                health_log_path=logs_dir / "health_check.log",
            )
            algo_return_code, external_timeout = run_danger_search(
                workspace_dir=workspace_dir,
                run_config_path=run_config_path,
                result_file=result_file,
                mission_summary_file=runtime_summary_file,
                max_floor=args.max_floor,
                timeout=args.timeout,
                log_path=logs_dir / "danger_search.log",
            )
    except Exception as exc:
        failure_reason.append(str(exc))
        print(f"运行失败: {exc}")
    finally:
        if env_process is not None:
            env_process.close(force=True)
        if not args.skip_cleanup_after:
            cleanup()
        copy_runtime_logs(workspace_dir, run_dir)

    try:
        evaluation_completed = evaluate_run(workspace_dir, run_dir, result_file)
    except subprocess.CalledProcessError as exc:
        failure_reason.append(f"评测脚本失败: {exc}")

    write_json(run_dir / "detection_debug.json", {"status": "not_collected", "source": "logs/danger_search.log"})
    write_json(run_dir / "planner_debug.json", {"status": "not_collected", "source": "logs/danger_search.log"})

    runtime_summary = read_json(runtime_summary_file, default={}) or {}
    runtime_failures = runtime_summary.get("failure_reason", [])
    if isinstance(runtime_failures, str):
        runtime_failures = [runtime_failures]
    failure_reason.extend(str(item) for item in runtime_failures if item)
    if external_timeout:
        failure_reason.append("外部超时")
    failure_reason = list(dict.fromkeys(failure_reason))
    result_written = result_file.exists()
    mission_completed = result_written and not external_timeout and algo_return_code == 0

    mission_summary = {
        "run_id": run_dir.name,
        "seed": args.seed,
        "scene_type": args.scene_type,
        "param_version": args.param_version,
        "timeout": args.timeout,
        "max_floor": args.max_floor,
        "gui": gui_enabled,
        "gui_mode": "gui" if args.gui else "headless" if args.headless else "auto",
        "completed": mission_completed,
        "result_written": result_written,
        "external_timeout": external_timeout,
        "evaluation_completed": evaluation_completed,
        "algo_return_code": algo_return_code,
        "return_home_success": runtime_summary.get("return_home_success", ""),
        "elevator_success": runtime_summary.get("elevator_success", ""),
        "exploration_time": runtime_summary.get("exploration_time", ""),
        "timeout_triggered": runtime_summary.get("timeout_triggered", ""),
        "current_floor": runtime_summary.get("current_floor", ""),
        "confirmed_count": runtime_summary.get("confirmed_count", ""),
        "track_count": runtime_summary.get("track_count", ""),
        "started_at": started_at.isoformat(),
        "timestamp": datetime.datetime.now().isoformat(),
        "elapsed_wall_time": round((datetime.datetime.now() - started_at).total_seconds(), 3),
        "failure_reason": failure_reason,
        "result_file": str(result_file),
        "params_file": str(run_config_path),
    }
    write_json(run_dir / "mission_summary.json", mission_summary)
    print(f"所有任务完成。归档保存在: {run_dir}")
    return 0 if result_file.exists() else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime
from dataclasses import dataclass
import math
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


EXACT_CONFLICT_NAMES = {"rosmaster", "rosout", "gzserver", "gzclient", "junior_ctrl"}
COMMAND_CONFLICT_MARKERS = (
    "roslaunch unitree_guide multi_floor_gazeboSim.launch",
    "building_generator_classic_control",
    "danger_search_node",
)
OWNED_ENVIRONMENT_NAMES = {
    "rosmaster",
    "rosout",
    "gzserver",
    "gzclient",
    "junior_ctrl",
}


DEFAULT_HEALTH_CHECK_TIMEOUT = 120

# 启动姿态门禁阈值：只读采样 /Odometry_gazebo，超限即判定 startup_posture_failure。
# 正常起立 run 实测 max_abs_roll≈0.066、max_abs_pitch≈0.041、z∈[0.18,0.34]，
# 翻倒 run 实测 max_abs_roll≈3.14、z<0.1，阈值在两者间留充足余量。
POSTURE_MAX_ABS_ROLL = 0.5
POSTURE_MAX_ABS_PITCH = 0.5
POSTURE_MIN_Z = 0.10
POSTURE_SAMPLE_SECONDS = 5.0


@dataclass(frozen=True)
class RunTermination:
    return_code: int
    kind: str
    reason: str
    component: str


def _running_processes() -> list[tuple[int, str, str]]:
    processes: list[tuple[int, str, str]] = []
    proc_root = Path("/proc")
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            name = (entry / "comm").read_text(encoding="utf-8").strip()
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        processes.append((int(entry.name), name, command))
    return processes


def find_conflicting_processes() -> list[str]:
    conflicts: list[str] = []
    own_pid = os.getpid()
    for pid, name, command in _running_processes():
        if pid == own_pid:
            continue
        if name in EXACT_CONFLICT_NAMES or any(marker in command for marker in COMMAND_CONFLICT_MARKERS):
            conflicts.append(f"pid={pid} name={name} command={command}")
    return conflicts


def process_name_is_running(name: str) -> bool:
    return any(process_name == name for _, process_name, _ in _running_processes())


def drain_environment_output(child) -> int:
    """非阻塞排空 pexpect PTY，防止控制器因终端缓冲区写满而停转。"""
    try:
        import pexpect
    except ImportError:
        return 0
    drained = 0
    while True:
        try:
            chunk = child.read_nonblocking(size=4096, timeout=0)
        except (pexpect.TIMEOUT, pexpect.EOF):
            break
        if not chunk:
            break
        drained += len(chunk)
    return drained


def capture_environment_processes(auto_pid: int) -> list[dict[str, object]]:
    """在无冲突启动前提下快照本轮环境 PID/PGID，供结束时精确清理。"""
    captured: list[dict[str, object]] = []
    for pid, name, command in _running_processes():
        if (
            pid != auto_pid
            and name not in OWNED_ENVIRONMENT_NAMES
            and not any(marker in command for marker in COMMAND_CONFLICT_MARKERS[:2])
        ):
            continue
        try:
            process_group = os.getpgid(pid)
        except ProcessLookupError:
            continue
        captured.append(
            {
                "pid": pid,
                "pgid": process_group,
                "name": name,
                "command": command,
            }
        )
    return captured


def stop_owned_process(process: subprocess.Popen, *, label: str) -> None:
    """按本轮创建的进程组温和退出，超时后逐级升级信号。"""
    if process.poll() is not None:
        return
    try:
        process_group = os.getpgid(process.pid)
    except ProcessLookupError:
        return
    if process_group == os.getpgrp():
        raise RuntimeError(f"拒绝终止共享进程组：{label} pgid={process_group}")
    for sig, timeout in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(process_group, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def stop_environment(child) -> None:
    """仅停止 pexpect 本轮启动的 auto.sh 进程组。"""
    if child is None:
        return
    # pexpect 为 PTY 子进程建立独立会话。roslaunch/Gazebo 还会建立子进程组，
    # 因此必须使用健康检查后保存的 PGID 快照，不能只依赖退出时的父子关系。
    process_groups = {int(child.pid)}
    for process in getattr(child, "owned_environment_processes", []):
        process_groups.add(int(process["pgid"]))
    process_groups.discard(os.getpgrp())
    if not process_groups:
        raise RuntimeError("拒绝清理：本轮没有可验证的独立仿真进程组")
    for sig, delay in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
        for process_group in sorted(process_groups):
            try:
                os.killpg(process_group, sig)
            except ProcessLookupError:
                continue
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline:
            alive_groups = []
            for process_group in process_groups:
                try:
                    os.killpg(process_group, 0)
                    alive_groups.append(process_group)
                except ProcessLookupError:
                    continue
            if not alive_groups:
                break
            time.sleep(0.1)
        process_groups = set(alive_groups)
        if not process_groups:
            break
    child.close(force=True)


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


def run_environment(
    seed: int,
    workspace_dir: Path,
    log_file,
    *,
    gui_enabled: bool,
    health_log_path: Path,
    max_floor: int = 3,
    run_pid_dir: Path | None = None,
):
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
            "FLOOR_COUNT": str(max_floor),
            "RUN_PID_DIR": str(run_pid_dir or (workspace_dir / "logs" / "run_pids")),
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
        child.owned_environment_processes = capture_environment_processes(child.pid)
        if run_pid_dir is not None:
            write_json(
                run_pid_dir / "owned_environment_processes.json",
                child.owned_environment_processes,
            )
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


def _quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def _probe_robot_posture(sample_seconds: float) -> dict:
    """只读采样 /Odometry_gazebo 姿态统计；绝不发布任何命令、不调用 set_model_state。

    使用墙钟窗口采样（use_sim_time 下 rospy.sleep 会受 RTF 拖慢），disable_signals
    避免干扰父进程已有的信号处理；采样后立即注销订阅并关闭临时节点。
    """
    import rospy
    from nav_msgs.msg import Odometry

    stats = {
        "max_abs_roll": 0.0,
        "max_abs_pitch": 0.0,
        "min_z": None,
        "samples": 0,
        "sample_seconds": float(sample_seconds),
    }
    finished = {"done": False}

    def _on_odom(msg) -> None:
        if finished["done"]:
            return
        pose = msg.pose.pose
        roll, pitch = _quaternion_to_rpy(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        stats["max_abs_roll"] = max(stats["max_abs_roll"], abs(roll))
        stats["max_abs_pitch"] = max(stats["max_abs_pitch"], abs(pitch))
        z = float(pose.position.z)
        if stats["min_z"] is None or z < stats["min_z"]:
            stats["min_z"] = z
        stats["samples"] += 1

    rospy.init_node("startup_posture_probe", anonymous=True, disable_signals=True)
    subscriber = rospy.Subscriber("/Odometry_gazebo", Odometry, _on_odom, queue_size=200)
    try:
        deadline = time.monotonic() + float(sample_seconds)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
    finally:
        finished["done"] = True
        subscriber.unregister()
        rospy.signal_shutdown("posture_probe_done")
    return stats


def _posture_breach_detail(stats: dict) -> str | None:
    """姿态统计超阈值时返回 startup_posture_failure 详情字符串，否则返回 None。"""
    samples = int(stats.get("samples", 0))
    roll = float(stats.get("max_abs_roll", 0.0))
    pitch = float(stats.get("max_abs_pitch", 0.0))
    min_z_raw = stats.get("min_z")
    min_z = float(min_z_raw) if min_z_raw is not None else None
    breached = (
        samples == 0
        or roll > POSTURE_MAX_ABS_ROLL
        or pitch > POSTURE_MAX_ABS_PITCH
        or (min_z is not None and min_z < POSTURE_MIN_Z)
    )
    if not breached:
        return None
    min_z_text = "" if min_z is None else f" min_z={min_z:.4f}"
    return (
        f"startup_posture_failure: samples={samples}"
        f" max_abs_roll={roll:.4f} max_abs_pitch={pitch:.4f}{min_z_text}"
    )


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
        health_log.write("\n检查机器人姿态（只读采样 /Odometry_gazebo）\n")
        health_log.flush()
        posture: dict | None = None
        try:
            posture = _probe_robot_posture(POSTURE_SAMPLE_SECONDS)
        except Exception as exc:
            health_log.write(f"姿态采样异常（跳过姿态门禁）: {exc}\n")
            health_log.flush()
        if posture is not None:
            health_log.write(
                f"姿态采样: samples={posture['samples']} "
                f"max_abs_roll={posture['max_abs_roll']:.4f} "
                f"max_abs_pitch={posture['max_abs_pitch']:.4f} "
                f"min_z={posture['min_z']:.4f}\n"
            )
            health_log.flush()
            detail = _posture_breach_detail(posture)
            if detail is not None:
                health_log.write(f"姿态门禁失败：{detail}\n")
                health_log.flush()
                raise RuntimeError(detail)
            health_log.write("姿态门禁通过\n")
            health_log.flush()
        health_log.write(f"\n启动健康检查通过: {datetime.datetime.now().isoformat()}\n")


def run_danger_search(
    workspace_dir: Path,
    run_config_path: Path,
    result_file: Path,
    mission_summary_file: Path,
    max_floor: int,
    timeout: int,
    log_path: Path,
    environment_process,
    gui_enabled: bool,
) -> RunTermination:
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
            start_new_session=True,
        )
        deadline = time.monotonic() + timeout
        termination: RunTermination | None = None
        try:
            while termination is None:
                drain_environment_output(environment_process)
                return_code = process.poll()
                if return_code is not None:
                    reason = "algorithm_exit" if return_code == 0 else "algorithm_exit_nonzero"
                    termination = RunTermination(return_code, "algorithm_exit", reason, "danger_search")
                    break
                if not environment_process.isalive():
                    termination = RunTermination(-1, "environment_exit", "auto_sh_exited", "auto.sh")
                    break
                if not process_name_is_running("gzserver"):
                    termination = RunTermination(-1, "environment_exit", "gzserver_exited", "gzserver")
                    break
                if gui_enabled and not process_name_is_running("gzclient"):
                    termination = RunTermination(-1, "environment_exit", "gui_exited", "gzclient")
                    break
                if time.monotonic() >= deadline:
                    termination = RunTermination(-1, "runner_timeout", "wall_timeout", "run_seed.py")
                    break
                time.sleep(1.0)
        except KeyboardInterrupt:
            termination = RunTermination(-1, "runner_interrupt", "keyboard_interrupt", "run_seed.py")

        assert termination is not None
        if process.poll() is None:
            print(
                f"[{datetime.datetime.now().time()}] 本轮终止：kind={termination.kind} "
                f"reason={termination.reason} component={termination.component}，停止算法"
            )
            stop_owned_process(process, label="danger_search")
            termination = RunTermination(
                int(process.returncode or 0),
                termination.kind,
                termination.reason,
                termination.component,
            )
        else:
            print(f"[{datetime.datetime.now().time()}] danger_search 节点退出，返回码 {process.returncode}")
        return termination


def runtime_completed_successfully(runtime_summary: dict) -> bool:
    return bool(
        runtime_summary.get("completed") is True
        and runtime_summary.get("terminal_outcome") == "success"
        and runtime_summary.get("terminal_state") == "FINISH"
        and runtime_summary.get("terminal_reason") == "return_home_completed"
        and runtime_summary.get("return_home_started") is True
        and runtime_summary.get("return_home_completed") is True
        and runtime_summary.get("return_home_success") is True
        and runtime_summary.get("timeout_triggered") is False
    )


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
    parser.add_argument("--max-floor", type=int, default=1, help="最大探索楼层数")
    parser.add_argument("--workspace", type=str, help="SimEnv 工作区路径，默认自动寻找")
    parser.add_argument("--scene-type", default="", help="归档中的场景类型标签")
    parser.add_argument("--param-version", default="", help="归档中的参数版本标签")
    gui_group = parser.add_mutually_exclusive_group()
    gui_group.add_argument("--gui", action="store_true", help="强制启动 Gazebo GUI；无 DISPLAY 时会报错")
    gui_group.add_argument("--headless", action="store_true", help="强制关闭 Gazebo GUI")
    parser.add_argument(
        "--skip-cleanup-before",
        action="store_true",
        help="兼容旧命令；当前版本不会主动清理未知进程",
    )
    parser.add_argument(
        "--skip-cleanup-after",
        action="store_true",
        help="调试时保留本轮启动的仿真进程",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    workspace_dir = find_workspace_dir(args.workspace or Path(__file__))
    try:
        gui_enabled = resolve_gui_enabled(args)
    except RuntimeError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2
    conflicts = find_conflicting_processes()
    if conflicts:
        print("检测到已有 ROS/Gazebo 仿真进程，拒绝启动以避免实例冲突：", file=sys.stderr)
        for conflict in conflicts:
            print(f"  {conflict}", file=sys.stderr)
        return 2

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
    termination: RunTermination | None = None
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
                max_floor=args.max_floor,
                run_pid_dir=logs_dir / "processes",
            )
            termination = run_danger_search(
                workspace_dir=workspace_dir,
                run_config_path=run_config_path,
                result_file=result_file,
                mission_summary_file=runtime_summary_file,
                max_floor=args.max_floor,
                timeout=args.timeout,
                log_path=logs_dir / "danger_search.log",
                environment_process=env_process,
                gui_enabled=gui_enabled,
            )
    except Exception as exc:
        failure_reason.append(str(exc))
        print(f"运行失败: {exc}")
    finally:
        if env_process is not None and not args.skip_cleanup_after:
            stop_environment(env_process)
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
    if termination is not None and termination.kind == "runner_timeout":
        failure_reason.append("外部超时")
    if termination is not None and termination.kind not in ("algorithm_exit",):
        failure_reason.append(
            f"运行器终止: {termination.reason} ({termination.component})"
        )
    failure_reason = list(dict.fromkeys(failure_reason))
    result_written = result_file.exists()
    normal_runtime_completion = runtime_completed_successfully(runtime_summary)
    mission_completed = bool(
        result_written
        and termination is not None
        and termination.kind == "algorithm_exit"
        and termination.return_code == 0
        and normal_runtime_completion
    )
    if result_written and not normal_runtime_completion:
        failure_reason.append(
            f"任务未正常完成: {runtime_summary.get('terminal_reason', 'missing_runtime_outcome')}"
        )
    failure_reason = list(dict.fromkeys(failure_reason))

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
        "external_timeout": bool(termination and termination.kind == "runner_timeout"),
        "termination_kind": termination.kind if termination is not None else "startup_failure",
        "termination_reason": termination.reason if termination is not None else "startup_failure",
        "termination_component": termination.component if termination is not None else "run_seed.py",
        "evaluation_completed": evaluation_completed,
        "algo_return_code": termination.return_code if termination is not None else None,
        "runtime_completed": normal_runtime_completion,
        "runtime_terminal_state": runtime_summary.get("terminal_state", ""),
        "runtime_terminal_outcome": runtime_summary.get("terminal_outcome", ""),
        "runtime_terminal_reason": runtime_summary.get("terminal_reason", ""),
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
    return 0 if mission_completed else 1


if __name__ == "__main__":
    raise SystemExit(main())

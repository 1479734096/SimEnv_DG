#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime
from collections import deque
from dataclasses import dataclass
import json
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

# 启动姿态门禁阈值：只读采样初始接口允许的 /trunk_imu，超限即判定 startup_posture_failure。
# 正常起立 run 实测 max_abs_roll≈0.066、max_abs_pitch≈0.041、z∈[0.18,0.34]，
# 翻倒 run 实测 max_abs_roll≈3.14、z<0.1，阈值在两者间留充足余量。
POSTURE_MAX_ABS_ROLL = 0.5
POSTURE_MAX_ABS_PITCH = 0.5
POSTURE_SETTLED_MAX_ABS_ROLL = 0.25
POSTURE_SETTLED_MAX_ABS_PITCH = 0.25
POSTURE_MIN_Z = 0.10
POSTURE_SAMPLE_SECONDS = 5.0
# fixed-stand 自身有约 3.5 秒的起立/增益过渡。传感器观察使用仿真时间，
# wall time 只承担 /clock 或进程冻结的 watchdog，避免低 RTF 下误判。
FIXED_STAND_MIN_SECONDS = 4.0
FIXED_STAND_MAX_WAIT_SECONDS = 75.0
FIXED_STAND_STABLE_WINDOW_SECONDS = 1.0
FIXED_STAND_STABLE_MAX_ABS_ROLL = 0.20
FIXED_STAND_STABLE_MAX_ABS_PITCH = 0.20
FIXED_STAND_MAX_IMU_GAP_SECONDS = 0.20
# Livox 外参与定位节点保持一致。该距离来自允许的 /scan 楼面回波，不是真值高度。
LIVOX_BASE_PITCH = 0.785
LIVOX_BASE_X = 0.2
LIVOX_BASE_Z = 0.08
FIXED_STAND_MIN_FLOOR_CLEARANCE = 0.18
FIXED_STAND_MIN_CLEARANCE_FRAMES = 3
FIXED_STAND_MAX_CLEARANCE_GAP_SECONDS = 0.30
LANDING_MAX_ABS_ROLL = 0.25
LANDING_MAX_ABS_PITCH = 0.25
LANDING_FREE_FALL_MAX_ACCEL = 3.0
LANDING_SUPPORTED_MIN_ACCEL = 6.0
LANDING_SUPPORTED_CONFIRMATIONS = 3
LANDING_FALLBACK_SUPPORT_SECONDS = 0.10


@dataclass(frozen=True)
class RunTermination:
    return_code: int
    kind: str
    reason: str
    component: str


def configure_workspace_ros_package_path(workspace_dir: Path) -> None:
    """让直接运行的 Python runner 与其子 shell 使用同一 ROS 工作区。

    README 允许直接执行本脚本，不要求调用者预先 source ``devel/setup.bash``。
    runner 自身也会通过 rospkg 校验公开场景文件，因此必须先显式加入当前
    工作区源码路径；保留调用者已有路径，且重复调用不产生重复项。
    """
    source_path = str(Path(workspace_dir) / "src")
    existing = [
        item for item in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep)
        if item
    ]
    os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join(
        [source_path, *(item for item in existing if item != source_path)]
    )


def environment_launch_overrides(
    *,
    seed: int,
    gui_enabled: bool,
    max_floor: int,
    run_pid_dir: Path,
) -> dict[str, str]:
    """构造本轮仿真环境变量。

    只在 A1 与 Livox ray 插件初始化期间暂停物理，避免 ODE 在扫描模式
    装载和活动碰撞查询并发时崩溃。runner 看到 ``/scan`` 发布器后立即
    解锁，后续起立和比赛运行仍使用初始环境的正常物理语义。
    """
    return {
        "SEED": str(seed),
        "GUI": "true" if gui_enabled else "false",
        "PAUSED": "true",
        "AUTO_UNPAUSE": "false",
        "CONTROLLER_FOREGROUND": "1",
        "START_CONTROLLER": "1",
        "START_BUILDING_CONTROL": "1",
        "FLOOR_COUNT": str(max_floor),
        "RUN_PID_DIR": str(run_pid_dir),
        # 赛方转换节点默认会订阅 /Odometry_gazebo；正式 runner 必须显式关闭，
        # 不能依赖调用者恰好设置了同名环境变量。
        "POINTCLOUD_USE_GROUND_TRUTH_ODOM": "false",
        "ENABLE_REFEREE_ODOM": "false",
        # 初始环境的受保护 RL 控制器以 referee-only 的 /ground_truth/base_w
        # 构造策略观测。规则禁止参赛算法订阅这些话题，不禁止裁判环境发布；
        # 关闭发布会让 RL 收到 /cmd_vel 却无法形成正常步态。
        "ENABLE_GROUND_TRUTH": "true",
    }


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


def abort_environment_startup(child) -> None:
    """启动失败时按本轮 PID/PGID 快照清理，避免只关闭 PTY 后遗留 ROS 进程。"""
    child.owned_environment_processes = capture_environment_processes(child.pid)
    stop_environment(child)


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


def unpause_environment_physics(workspace_dir: Path, *, timeout: int = 15) -> None:
    """在 Livox 注册发布器后通过赛方已有 Gazebo 服务解除启动暂停。"""
    command = " ".join(
        [
            "source /opt/ros/noetic/setup.bash",
            "&&",
            f"source {shlex.quote(str(workspace_dir / 'devel' / 'setup.bash'))}",
            "&&",
            "rosservice call /gazebo/unpause_physics",
        ]
    )
    try:
        subprocess.run(
            ["bash", "-lc", command],
            cwd=str(workspace_dir),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=True,
        )
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as exc:
        raise RuntimeError("Gazebo 物理解除暂停失败") from exc


def wait_for_ros_topic_publisher(topic: str, *, timeout: float = 120.0) -> None:
    """等待 ROS master 上出现指定话题发布器，不要求话题已经产生数据帧。

    暂停状态下 Livox 不会发布点云帧，但其插件完成初始化后会注册 ``/scan``。
    这里仅查询 ROS master，不订阅任何话题，也不会初始化 ``rospy`` 节点。
    """
    try:
        from rosgraph.masterapi import Master
    except ImportError as exc:
        raise RuntimeError("缺少 rosgraph，无法检查 Livox 启动状态") from exc

    master = Master("/danger_search_runner_startup")
    deadline = time.monotonic() + float(timeout)
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            publishers, _, _ = master.getSystemState()
            if any(name == topic and nodes for name, nodes in publishers):
                return
            last_error = None
        except Exception as exc:  # ROS master 在 auto.sh 启动初期尚不可达
            last_error = exc
        time.sleep(0.05)

    detail = ""
    if last_error is not None:
        detail = f"，最后错误：{last_error}"
    raise RuntimeError(f"等待 {topic} 发布器超时{detail}")


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

    effective_pid_dir = run_pid_dir or (workspace_dir / "logs" / "run_pids")
    env = os.environ.copy()
    env.update(
        environment_launch_overrides(
            seed=seed,
            gui_enabled=gui_enabled,
            max_floor=max_floor,
            run_pid_dir=effective_pid_dir,
        )
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
        # 初始 auto.sh 已提供 paused 参数；仅把解锁时机从固定 wall sleep
        # 改为 Livox 插件实际注册完成，避免修改任何赛方模型或传感器代码。
        print(f"[{datetime.datetime.now().time()}] 等待 Livox 发布器完成装载")
        wait_for_ros_topic_publisher("/scan", timeout=120.0)
        print(f"[{datetime.datetime.now().time()}] Livox 已就绪，解除 Gazebo 暂停")
        unpause_environment_physics(workspace_dir)
        child.expect("Use keyboard input in this terminal", timeout=120)
        print(f"[{datetime.datetime.now().time()}] 控制器进程已启动，等待 RL 模型加载完成")
        child.expect("load model to device!", timeout=120)
        # 与初始环境的人工启动流程及早期稳定 runner 保持一致：RL 模型一旦
        # 就绪便进入 fixed-stand，不能在 passive 状态额外等待传感器判定，
        # 否则非暂停物理会先让机器人趴到地面。
        print(f"[{datetime.datetime.now().time()}] RL 模型已加载，发送 2 进入站立")
        child.send("2")
        child.expect("Switched from passive to fixed stand", timeout=30)
        # 保持赛方初始启动时序：fixed-stand 过渡 5 秒后只发送一次 6。
        # 部分启动中 fixed-stand 净空偏低，但 RL 接管后可以正常站稳；不能
        # 把 fixed-stand 净空作为进入 RL 的前置条件，否则会形成启动自锁。
        print(f"[{datetime.datetime.now().time()}] 已进入 fixed-stand，等待 5 秒后发送 6 进入 RL 模式")
        time.sleep(5.0)
        child.send("6")
        child.expect("Switched from fixed stand to RL", timeout=30)
        print(f"[{datetime.datetime.now().time()}] 控制器已进入 RL，确认机器人实际站稳")
        posture = wait_for_fixed_stand_posture(
            min_stand_seconds=2.0,
            max_wait_seconds=30.0,
        )
        if not posture["stable"]:
            raise RuntimeError(_fixed_stand_failure_detail(posture))
        print(f"[{datetime.datetime.now().time()}] RL 姿态已稳定，开始启动健康检查")
        check_startup_health(workspace_dir, health_log_path, posture=posture)
        time.sleep(1.0)
        child.owned_environment_processes = capture_environment_processes(child.pid)
        if run_pid_dir is not None:
            write_json(
                run_pid_dir / "owned_environment_processes.json",
                child.owned_environment_processes,
            )
        print(f"[{datetime.datetime.now().time()}] 仿真环境准备完毕，控制器已进入 RL 模式")
    except pexpect.TIMEOUT:
        abort_environment_startup(child)
        raise RuntimeError("启动仿真环境超时")
    except pexpect.EOF:
        abort_environment_startup(child)
        raise RuntimeError("auto.sh 意外退出")
    except Exception:
        abort_environment_startup(child)
        raise
    return child


def wait_for_ros_topic(
    workspace_dir: Path,
    topic: str,
    timeout: int = 30,
    log_file=None,
    *,
    field: str | None = None,
    message_count: int = 1,
) -> None:
    if int(message_count) < 1:
        raise ValueError("message_count 必须大于等于 1")
    echo_target = topic if field is None else f"{topic}/{field.lstrip('/')}"
    cmd = " ".join(
        [
            "source /opt/ros/noetic/setup.bash",
            "&&",
            f"source {shlex.quote(str(workspace_dir / 'devel' / 'setup.bash'))}",
            "&&",
            f"rostopic echo -n {int(message_count)} {shlex.quote(echo_target)}",
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
    """只读采样初始文档允许的 /trunk_imu 姿态统计。

    使用墙钟窗口采样（use_sim_time 下 rospy.sleep 会受 RTF 拖慢），disable_signals
    避免干扰父进程已有的信号处理；采样后立即注销订阅并关闭临时节点。
    """
    import rospy
    from sensor_msgs.msg import Imu

    stats = {
        "max_abs_roll": 0.0,
        "max_abs_pitch": 0.0,
        "last_abs_roll": 0.0,
        "last_abs_pitch": 0.0,
        "samples": 0,
        "sample_seconds": float(sample_seconds),
    }
    finished = {"done": False}

    def _on_imu(msg) -> None:
        if finished["done"]:
            return
        roll, pitch = _quaternion_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        stats["max_abs_roll"] = max(stats["max_abs_roll"], abs(roll))
        stats["max_abs_pitch"] = max(stats["max_abs_pitch"], abs(pitch))
        stats["last_abs_roll"] = abs(roll)
        stats["last_abs_pitch"] = abs(pitch)
        stats["samples"] += 1

    rospy.init_node("startup_posture_probe", anonymous=True, disable_signals=True)
    subscriber = rospy.Subscriber("/trunk_imu", Imu, _on_imu, queue_size=200)
    try:
        deadline = time.monotonic() + float(sample_seconds)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
    finally:
        finished["done"] = True
        subscriber.unregister()
        rospy.signal_shutdown("posture_probe_done")
    return stats


def _is_stable_posture_window(
    samples: list[tuple[float, float, float]],
    *,
    now: float,
    window_seconds: float = FIXED_STAND_STABLE_WINDOW_SECONDS,
) -> tuple[bool, int]:
    """判断最近连续 IMU 窗口是否稳定，保持为纯函数以便离线单元测试。"""
    window_start = now - float(window_seconds)
    start_indices = [index for index, sample in enumerate(samples) if sample[0] <= window_start]
    if not start_indices:
        return False, len([sample for sample in samples if sample[0] >= window_start])
    window = samples[start_indices[-1]:]
    if not window or now - window[-1][0] > FIXED_STAND_MAX_IMU_GAP_SECONDS:
        return False, len(window)
    if any(
        current[0] - previous[0] > FIXED_STAND_MAX_IMU_GAP_SECONDS
        for previous, current in zip(window, window[1:])
    ):
        return False, len(window)
    stable = all(
        roll <= FIXED_STAND_STABLE_MAX_ABS_ROLL
        and pitch <= FIXED_STAND_STABLE_MAX_ABS_PITCH
        for _, roll, pitch in window
    )
    return stable, len(window)


def _measure_livox_floor_clearance(points) -> tuple[float | None, dict[str, object]]:
    """从允许的 Livox 点云估计 base 到当前楼面的距离。

    这里只用于 runner 起立握手。采用主平面高度众数，避免把家具底部或少量
    离群点当成楼面；同时返回最小诊断计数，以区分未收到点云、空帧和筛选
    失败。生产定位仍使用 C++ RANSAC 实现。
    """
    cosine = math.cos(LIVOX_BASE_PITCH)
    sine = math.sin(LIVOX_BASE_PITCH)
    candidate_z: list[float] = []
    diagnostics: dict[str, object] = {
        "point_count": len(points),
        "finite_points": 0,
        "radial_points": 0,
        "z_band_points": 0,
        "mode_inliers": 0,
        "mode_z": None,
        "rejection": "empty_frame" if not points else "unknown",
    }
    for point in points:
        x = float(point.x)
        y = float(point.y)
        z = float(point.z)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        diagnostics["finite_points"] = int(diagnostics["finite_points"]) + 1
        base_x = cosine * x + sine * z + LIVOX_BASE_X
        base_z = -sine * x + cosine * z + LIVOX_BASE_Z
        if not (0.25 <= math.hypot(base_x, y) <= 6.0):
            continue
        diagnostics["radial_points"] = int(diagnostics["radial_points"]) + 1
        if -0.80 <= base_z <= 0.20:
            candidate_z.append(base_z)
    diagnostics["z_band_points"] = len(candidate_z)
    if len(candidate_z) < 300:
        if diagnostics["point_count"]:
            diagnostics["rejection"] = "insufficient_z_band_points"
        return None, diagnostics

    bin_width = 0.02
    histogram: dict[int, int] = {}
    for value in candidate_z:
        key = int(round(value / bin_width))
        histogram[key] = histogram.get(key, 0) + 1
    mode_key, _ = max(histogram.items(), key=lambda item: item[1])
    mode_z = mode_key * bin_width
    diagnostics["mode_z"] = mode_z
    inliers = [value for value in candidate_z if abs(value - mode_z) <= 0.04]
    diagnostics["mode_inliers"] = len(inliers)
    if len(inliers) < 300 or len(inliers) < 0.15 * len(candidate_z):
        diagnostics["rejection"] = "insufficient_mode_inliers"
        return None, diagnostics
    inliers.sort()
    median_z = inliers[len(inliers) // 2]
    clearance = -median_z
    # passive 落地姿态的 Livox 净空实测可低至约 0.02 m；这里仅判断测量
    # 是否有效。是否完成站立仍由 FIXED_STAND_MIN_FLOOR_CLEARANCE=0.18 m
    # 单独判定，不能把量程下限误当成站立门槛。
    if not math.isfinite(clearance) or not 0.01 <= clearance <= 0.80:
        diagnostics["rejection"] = "clearance_out_of_range"
        return None, diagnostics
    diagnostics["rejection"] = "accepted"
    return clearance, diagnostics


def _estimate_livox_floor_clearance(points) -> float | None:
    """兼容现有调用方，仅返回 Livox 楼面距离估计。"""
    clearance, _ = _measure_livox_floor_clearance(points)
    return clearance


def _stable_floor_clearance(
    samples: list[tuple[float, float]],
    *,
    now: float,
) -> tuple[float | None, int]:
    """返回连续新鲜 Livox 楼面帧的保守距离和帧数。"""
    recent = [
        sample
        for sample in samples
        if 0.0 <= now - sample[0] <= FIXED_STAND_MAX_CLEARANCE_GAP_SECONDS
    ]
    if len(recent) < FIXED_STAND_MIN_CLEARANCE_FRAMES:
        return None, len(recent)
    if any(
        current[0] - previous[0] > FIXED_STAND_MAX_CLEARANCE_GAP_SECONDS
        for previous, current in zip(recent, recent[1:])
    ):
        return None, len(recent)
    # 使用近期最小值，避免一次偏高估计掩盖腹部贴地。
    return min(value for _, value in recent), len(recent)


def _aligned_floor_clearance(
    samples: list[tuple[float, float]],
    *,
    imu_stamp: float | None,
    scan_stamp: float | None,
) -> tuple[float | None, int, float | None]:
    """以点云自身时间判断连续帧，并独立确认其与 IMU 仍属同一时刻。"""
    if imu_stamp is None or scan_stamp is None:
        return None, 0, None
    stamp_delta = abs(float(imu_stamp) - float(scan_stamp))
    if stamp_delta > FIXED_STAND_MAX_CLEARANCE_GAP_SECONDS:
        return None, 0, stamp_delta
    clearance, frame_count = _stable_floor_clearance(
        samples,
        now=float(scan_stamp),
    )
    return clearance, frame_count, stamp_delta


def _landing_contact_detected(
    imu_samples: list[tuple[float, float, float, float]],
) -> tuple[bool, dict[str, float | int | bool | None]]:
    """从自由落体到承重比力的变化确认首次触地，不等待被动趴姿稳定。"""
    diagnostics: dict[str, float | int | bool | None] = {
        "imu_samples": len(imu_samples),
        "free_fall_seen": False,
        "supported_samples": 0,
        "last_acceleration": None,
    }
    if len(imu_samples) < LANDING_SUPPORTED_CONFIRMATIONS:
        return False, diagnostics
    diagnostics["free_fall_seen"] = any(
        acceleration <= LANDING_FREE_FALL_MAX_ACCEL
        for _, _, _, acceleration in imu_samples
    )
    diagnostics["last_acceleration"] = imu_samples[-1][3]
    supported = 0
    for _, roll, pitch, acceleration in reversed(imu_samples):
        if (
            roll <= LANDING_MAX_ABS_ROLL
            and pitch <= LANDING_MAX_ABS_PITCH
            and acceleration >= LANDING_SUPPORTED_MIN_ACCEL
        ):
            supported += 1
        else:
            break
    diagnostics["supported_samples"] = supported
    if supported < LANDING_SUPPORTED_CONFIRMATIONS:
        return False, diagnostics
    supported_start = imu_samples[-supported][0]
    supported_duration = imu_samples[-1][0] - supported_start
    contact = bool(diagnostics["free_fall_seen"]) or (
        supported_duration >= LANDING_FALLBACK_SUPPORT_SECONDS
    )
    return contact, diagnostics


def wait_for_fixed_stand_posture(
    *,
    min_stand_seconds: float = FIXED_STAND_MIN_SECONDS,
    max_wait_seconds: float = FIXED_STAND_MAX_WAIT_SECONDS,
    stable_window_seconds: float = FIXED_STAND_STABLE_WINDOW_SECONDS,
    stand_callback=None,
) -> dict:
    """先确认落地，再在 fixed-stand 阶段等待姿态与楼面距离正常。

    这是 runner 的启动前健康握手，不进入参赛算法。落地和站立稳定时长均
    使用传感器 stamp，``max_wait_seconds`` 仅是 wall-time watchdog。
    """
    import rospy
    from sensor_msgs.msg import Imu, PointCloud

    stats = {
        "max_abs_roll": 0.0,
        "max_abs_pitch": 0.0,
        "last_abs_roll": 0.0,
        "last_abs_pitch": 0.0,
        "samples": 0,
        "sample_seconds": 0.0,
        "stable": False,
        "stable_window_samples": 0,
        "floor_clearance": None,
        "floor_clearance_frames": 0,
        "imu_scan_stamp_delta": None,
        "stand_retry_count": 0,
        "landing_confirmed": stand_callback is None,
        "landing_imu_samples": 0,
        "landing_free_fall_seen": False,
        "landing_supported_samples": 0,
        "landing_last_acceleration": None,
        "scan_messages": 0,
        "scan_nonempty_frames": 0,
        "scan_last_point_count": 0,
        "scan_max_point_count": 0,
        "scan_last_finite_points": 0,
        "scan_last_radial_points": 0,
        "scan_last_z_band_points": 0,
        "scan_last_mode_inliers": 0,
        "scan_last_mode_z": None,
        "scan_last_rejection": "not_received",
    }
    samples: deque[tuple[float, float, float]] = deque()
    landing_samples: deque[tuple[float, float, float, float]] = deque()
    clearance_samples: deque[tuple[float, float]] = deque()
    finished = {"done": False}

    def _on_imu(msg) -> None:
        if finished["done"]:
            return
        roll, pitch = _quaternion_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        sensor_stamp = float(msg.header.stamp.to_sec())
        if not math.isfinite(sensor_stamp) or sensor_stamp <= 0.0:
            return
        abs_roll, abs_pitch = abs(roll), abs(pitch)
        samples.append((sensor_stamp, abs_roll, abs_pitch))
        acceleration = math.sqrt(
            float(msg.linear_acceleration.x) ** 2
            + float(msg.linear_acceleration.y) ** 2
            + float(msg.linear_acceleration.z) ** 2
        )
        landing_samples.append(
            (sensor_stamp, abs_roll, abs_pitch, acceleration)
        )
        while samples and samples[0][0] < sensor_stamp - stable_window_seconds - 1.0:
            samples.popleft()
        while (
            landing_samples
            and landing_samples[0][0]
            < sensor_stamp - 2.0
        ):
            landing_samples.popleft()
        stats["max_abs_roll"] = max(stats["max_abs_roll"], abs_roll)
        stats["max_abs_pitch"] = max(stats["max_abs_pitch"], abs_pitch)
        stats["last_abs_roll"] = abs_roll
        stats["last_abs_pitch"] = abs_pitch
        stats["samples"] += 1
        if "first_sensor_stamp" not in stats:
            stats["first_sensor_stamp"] = sensor_stamp
        stats["last_sensor_stamp"] = sensor_stamp
        stats["sensor_elapsed"] = max(
            0.0, sensor_stamp - float(stats["first_sensor_stamp"])
        )

    def _on_scan(msg) -> None:
        if finished["done"]:
            return
        stats["scan_messages"] = int(stats["scan_messages"]) + 1
        point_count = len(msg.points)
        stats["scan_last_point_count"] = point_count
        stats["scan_max_point_count"] = max(
            int(stats["scan_max_point_count"]), point_count
        )
        if point_count:
            stats["scan_nonempty_frames"] = int(stats["scan_nonempty_frames"]) + 1
        sensor_stamp = float(msg.header.stamp.to_sec())
        if not math.isfinite(sensor_stamp) or sensor_stamp <= 0.0:
            stats["scan_last_rejection"] = "invalid_stamp"
            return
        stats["scan_last_stamp"] = sensor_stamp
        clearance, diagnostics = _measure_livox_floor_clearance(msg.points)
        for source, target in (
            ("finite_points", "scan_last_finite_points"),
            ("radial_points", "scan_last_radial_points"),
            ("z_band_points", "scan_last_z_band_points"),
            ("mode_inliers", "scan_last_mode_inliers"),
            ("mode_z", "scan_last_mode_z"),
            ("rejection", "scan_last_rejection"),
        ):
            stats[target] = diagnostics[source]
        if clearance is None:
            return
        clearance_samples.append((sensor_stamp, clearance))
        while (
            clearance_samples
            and clearance_samples[0][0]
            < sensor_stamp - FIXED_STAND_MAX_CLEARANCE_GAP_SECONDS - 0.5
        ):
            clearance_samples.popleft()

    rospy.init_node("fixed_stand_posture_probe", anonymous=True, disable_signals=True)
    imu_subscriber = rospy.Subscriber("/trunk_imu", Imu, _on_imu, queue_size=200)
    scan_subscriber = rospy.Subscriber("/scan", PointCloud, _on_scan, queue_size=2)
    started = time.monotonic()
    try:
        deadline = started + float(max_wait_seconds)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            sensor_elapsed = float(stats.get("sensor_elapsed", 0.0))
            sensor_now = stats.get("last_sensor_stamp")
            if sensor_now is not None and not bool(stats["landing_confirmed"]):
                landed, landing = _landing_contact_detected(list(landing_samples))
                stats["landing_imu_samples"] = landing["imu_samples"]
                stats["landing_free_fall_seen"] = landing["free_fall_seen"]
                stats["landing_supported_samples"] = landing["supported_samples"]
                stats["landing_last_acceleration"] = landing["last_acceleration"]
                if landed:
                    stand_callback()
                    stats["landing_confirmed"] = True
                    samples.clear()
                    landing_samples.clear()
                    clearance_samples.clear()
                    stats.pop("first_sensor_stamp", None)
                    stats.pop("last_sensor_stamp", None)
                    stats["sensor_elapsed"] = 0.0
                time.sleep(0.05)
                continue
            if sensor_now is not None and sensor_elapsed >= float(min_stand_seconds):
                stable, window_samples = _is_stable_posture_window(
                    list(samples),
                    now=float(sensor_now),
                    window_seconds=stable_window_seconds,
                )
                stats["stable_window_samples"] = window_samples
                clearance, clearance_frames, stamp_delta = _aligned_floor_clearance(
                    list(clearance_samples),
                    imu_stamp=float(sensor_now),
                    scan_stamp=stats.get("scan_last_stamp"),
                )
                stats["floor_clearance"] = clearance
                stats["floor_clearance_frames"] = clearance_frames
                stats["imu_scan_stamp_delta"] = stamp_delta
                if (
                    stable
                    and clearance is not None
                    and clearance >= FIXED_STAND_MIN_FLOOR_CLEARANCE
                ):
                    stats["stable"] = True
                    break
            time.sleep(0.05)
    finally:
        stats["sample_seconds"] = time.monotonic() - started
        finished["done"] = True
        imu_subscriber.unregister()
        scan_subscriber.unregister()
        rospy.signal_shutdown("fixed_stand_posture_probe_done")
    return stats


def _posture_breach_detail(stats: dict) -> str | None:
    """姿态统计超阈值时返回 startup_posture_failure 详情字符串，否则返回 None。"""
    samples = int(stats.get("samples", 0))
    roll = float(stats.get("max_abs_roll", 0.0))
    pitch = float(stats.get("max_abs_pitch", 0.0))
    settled_roll = float(stats.get("last_abs_roll", 0.0))
    settled_pitch = float(stats.get("last_abs_pitch", 0.0))
    breached = (
        samples == 0
        or roll > POSTURE_MAX_ABS_ROLL
        or pitch > POSTURE_MAX_ABS_PITCH
        or settled_roll > POSTURE_SETTLED_MAX_ABS_ROLL
        or settled_pitch > POSTURE_SETTLED_MAX_ABS_PITCH
    )
    if not breached:
        return None
    return (
        f"startup_posture_failure: samples={samples}"
        f" max_abs_roll={roll:.4f} max_abs_pitch={pitch:.4f}"
        f" last_abs_roll={settled_roll:.4f} last_abs_pitch={settled_pitch:.4f}"
    )


def _fixed_stand_failure_detail(stats: dict) -> str:
    """固定站立未形成稳定窗口时给出可审计的启动失败原因。"""
    return (
        "startup_posture_failure: fixed_stand_not_stable"
        f" samples={int(stats.get('samples', 0))}"
        f" sample_seconds={float(stats.get('sample_seconds', 0.0)):.2f}"
        f" sensor_elapsed={float(stats.get('sensor_elapsed', 0.0)):.2f}"
        f" max_abs_roll={float(stats.get('max_abs_roll', 0.0)):.4f}"
        f" max_abs_pitch={float(stats.get('max_abs_pitch', 0.0)):.4f}"
        f" last_abs_roll={float(stats.get('last_abs_roll', 0.0)):.4f}"
        f" last_abs_pitch={float(stats.get('last_abs_pitch', 0.0)):.4f}"
        f" stable_window_samples={int(stats.get('stable_window_samples', 0))}"
        f" floor_clearance={stats.get('floor_clearance')}"
        f" floor_clearance_frames={int(stats.get('floor_clearance_frames', 0))}"
        f" imu_scan_stamp_delta={stats.get('imu_scan_stamp_delta')}"
        f" stand_retry_count={int(stats.get('stand_retry_count', 0))}"
        f" landing_confirmed={bool(stats.get('landing_confirmed', False))}"
        f" landing_imu_samples={int(stats.get('landing_imu_samples', 0))}"
        f" landing_free_fall_seen={bool(stats.get('landing_free_fall_seen', False))}"
        f" landing_supported_samples={int(stats.get('landing_supported_samples', 0))}"
        f" landing_last_acceleration={stats.get('landing_last_acceleration')}"
        f" scan_messages={int(stats.get('scan_messages', 0))}"
        f" scan_nonempty_frames={int(stats.get('scan_nonempty_frames', 0))}"
        f" scan_last_point_count={int(stats.get('scan_last_point_count', 0))}"
        f" scan_max_point_count={int(stats.get('scan_max_point_count', 0))}"
        f" scan_last_finite_points={int(stats.get('scan_last_finite_points', 0))}"
        f" scan_last_radial_points={int(stats.get('scan_last_radial_points', 0))}"
        f" scan_last_z_band_points={int(stats.get('scan_last_z_band_points', 0))}"
        f" scan_last_mode_inliers={int(stats.get('scan_last_mode_inliers', 0))}"
        f" scan_last_mode_z={stats.get('scan_last_mode_z')}"
        f" scan_last_rejection={stats.get('scan_last_rejection', 'unknown')}"
    )


def check_startup_health(
    workspace_dir: Path,
    health_log_path: Path,
    *,
    require_localization: bool = False,
    posture: dict | None = None,
) -> None:
    """分两阶段检查，避免在定位节点尚未启动时错误判定环境失败。"""
    required_topics = [("/scan", "Livox 原始点云")]
    if require_localization:
        required_topics.append(("/danger_search/localization/odom", "参赛方定位"))
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
                if topic == "/scan":
                    raise RuntimeError(
                        "启动健康检查失败：未收到 /scan 原始点云；"
                        "请检查 Gazebo/Livox 插件，详见 logs/health_check.log"
                    ) from exc
                raise RuntimeError(
                    f"启动健康检查失败：未收到 {topic} 数据，详见 logs/health_check.log"
                ) from exc
        if require_localization:
            # 同一 runner 进程中的 rospy 节点在 signal_shutdown 后不能重新初始化。
            # 姿态已在环境阶段检查一次；算法阶段只验证新启动的内部定位链。
            health_log.write("\n机器人姿态已在环境启动阶段检查，本阶段不重复采样\n")
            health_log.flush()
        else:
            health_log.write("\n检查机器人姿态（只读采样 /trunk_imu）\n")
            health_log.flush()
            if posture is None:
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
                    f"last_abs_roll={posture['last_abs_roll']:.4f} "
                    f"last_abs_pitch={posture['last_abs_pitch']:.4f} "
                    f"stable={posture.get('stable', 'n/a')} "
                    f"livox_floor_clearance={posture.get('floor_clearance')} "
                    f"stand_retry_count={posture.get('stand_retry_count', 0)} "
                    "高度真值不参与正式健康检查\n"
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
    result_file: Path | None,
    mission_summary_file: Path | None,
    max_floor: int,
    timeout: int,
    log_path: Path,
    environment_process,
    gui_enabled: bool,
) -> RunTermination:
    launch_args = [
        "source /opt/ros/noetic/setup.bash",
        "&&",
        f"source {shlex.quote(str(workspace_dir / 'devel' / 'setup.bash'))}",
        "&&",
        "roslaunch danger_search danger_search.launch",
        f"config:={shlex.quote(str(run_config_path))}",
    ]
    if mission_summary_file is not None:
        launch_args.append(f"mission_summary_file:={shlex.quote(str(mission_summary_file))}")
    launch_cmd = " ".join(launch_args)
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
            check_startup_health(
                workspace_dir,
                log_path.with_name("algorithm_health_check.log"),
                require_localization=True,
            )
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
        except Exception:
            # 健康检查异常发生在正常终止循环之前，也必须回收本轮算法进程组，
            # 否则会遗留 roslaunch/节点并污染下一次测试。
            if process.poll() is None:
                stop_owned_process(process, label="danger_search")
            raise

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


def read_and_validate_summary(path: Path) -> dict | None:
    """读取并校验 mission_runtime_summary.json 是否包含判定闭环完成所需的字段。"""
    if not path.exists():
        return None
    try:
        raw = path.read_text(encoding="utf-8").strip()
        if not raw:
            return None
        summary = json.loads(raw)
    except (json.JSONDecodeError, OSError):
        return None
    if not isinstance(summary, dict):
        return None
    required_fields = (
        "completed",
        "terminal_outcome",
        "terminal_state",
        "terminal_reason",
        "return_home_started",
        "return_home_completed",
        "return_home_success",
        "timeout_triggered",
    )
    for field in required_fields:
        if field not in summary:
            return None
    return summary


def archive_existing_public_result(public_result: Path, run_dir: Path) -> None:
    """若存在上轮参赛方输出，先归档再只删除该文件；不得触碰 truth、metadata 等环境文件。"""
    if not public_result.exists():
        return
    if public_result.name != "detected_danger.json" or public_result.parent.name != "results":
        raise ValueError(f"拒绝归档或删除非参赛方结果文件: {public_result}")
    archive_path = run_dir / "preexisting_detected_danger.json"
    shutil.copy2(public_result, archive_path)
    public_result.unlink()


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


def public_result_file(workspace_dir: Path) -> Path:
    """读取唯一允许的公开 scene info，并返回裁判指定的结果路径。"""
    from danger_search.building_feature_provider import BuildingFeatureProvider

    provider = BuildingFeatureProvider.load(
        workspace_dir / "generated_building" / "team_scene_info.json",
        purpose="正式运行归档",
    )
    return provider.result_file


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
    parser.add_argument(
        "--offline-evaluate",
        action="store_true",
        help="运行结束后显式读取裁判真值进行离线评估；不属于正式算法运行",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    workspace_dir = find_workspace_dir(args.workspace or Path(__file__))
    configure_workspace_ros_package_path(workspace_dir)
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
    production_result_file: Path | None = None
    public_result_was_archived = False
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
            production_result_file = public_result_file(workspace_dir)
            # 在算法启动前归档并删除上轮参赛方输出，避免旧结果污染本轮判断。
            archive_existing_public_result(production_result_file, run_dir)
            public_result_was_archived = True
            termination = run_danger_search(
                workspace_dir=workspace_dir,
                run_config_path=run_config_path,
                result_file=None,
                mission_summary_file=runtime_summary_file,
                max_floor=args.max_floor,
                timeout=args.timeout,
                log_path=logs_dir / "danger_search.log",
                environment_process=env_process,
                gui_enabled=gui_enabled,
            )
            # 结果由公开 scene info 指定；在停止环境前先归档，避免清理脚本影响证据。
            if production_result_file.exists():
                shutil.copy2(production_result_file, result_file)
    except Exception as exc:
        failure_reason.append(str(exc))
        print(f"运行失败: {exc}")
    finally:
        if env_process is not None and not args.skip_cleanup_after:
            stop_environment(env_process)
        copy_runtime_logs(workspace_dir, run_dir)

    if production_result_file is not None and production_result_file.exists():
        shutil.copy2(production_result_file, result_file)

    if args.offline_evaluate:
        try:
            evaluation_completed = evaluate_run(workspace_dir, run_dir, result_file)
        except subprocess.CalledProcessError as exc:
            failure_reason.append(f"评测脚本失败: {exc}")

    write_json(run_dir / "detection_debug.json", {"status": "not_collected", "source": "logs/danger_search.log"})
    write_json(run_dir / "planner_debug.json", {"status": "not_collected", "source": "logs/danger_search.log"})

    runtime_summary = read_and_validate_summary(runtime_summary_file)
    runtime_summary_valid = runtime_summary is not None
    if not runtime_summary_valid:
        failure_reason.append("缺少或损坏 mission_runtime_summary.json")
    else:
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
    # 若本轮已归档并删除了旧结果，则赛后该文件存在即代表算法本轮新写，不依赖 mtime。
    result_written_this_run = (
        public_result_was_archived
        and production_result_file is not None
        and production_result_file.exists()
    )
    normal_runtime_completion = (
        runtime_summary_valid and runtime_completed_successfully(runtime_summary)
    )
    # 算法零退出码只代表进程正常退出，不代表任务成功；必须同时有本轮新结果和成功返航闭环。
    mission_completed = bool(
        result_written_this_run
        and termination is not None
        and termination.kind == "algorithm_exit"
        and termination.return_code == 0
        and normal_runtime_completion
    )
    if result_written and not normal_runtime_completion:
        failure_reason.append(
            f"任务未正常完成: {runtime_summary.get('terminal_reason', 'missing_runtime_outcome') if runtime_summary else 'missing_runtime_summary'}"
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
        "runtime_exploration_completed": (
            runtime_summary.get("exploration_completed", False) if runtime_summary else False
        ),
        "runtime_terminal_state": runtime_summary.get("terminal_state", "") if runtime_summary else "",
        "runtime_terminal_outcome": runtime_summary.get("terminal_outcome", "") if runtime_summary else "",
        "runtime_terminal_reason": runtime_summary.get("terminal_reason", "") if runtime_summary else "",
        "normal_budget_return": bool(
            normal_runtime_completion
            and runtime_summary is not None
            and runtime_summary.get("exploration_completed") is not True
        ),
        "return_home_success": runtime_summary.get("return_home_success", "") if runtime_summary else "",
        "elevator_success": runtime_summary.get("elevator_success", "") if runtime_summary else "",
        "exploration_time": runtime_summary.get("exploration_time", "") if runtime_summary else "",
        "timeout_triggered": runtime_summary.get("timeout_triggered", "") if runtime_summary else "",
        "current_floor": runtime_summary.get("current_floor", "") if runtime_summary else "",
        "confirmed_count": runtime_summary.get("confirmed_count", "") if runtime_summary else "",
        "track_count": runtime_summary.get("track_count", "") if runtime_summary else "",
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

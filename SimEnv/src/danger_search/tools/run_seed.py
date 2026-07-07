#!/usr/bin/env python3
import argparse
import datetime
import json
import os
import pexpect
import shutil
import subprocess
import sys
import time
import yaml
from pathlib import Path

def generate_run_dir(seed: int) -> Path:
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(f"results/runs/{timestamp}_seed_{seed}")
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir

def load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def save_yaml(data: dict, path: str):
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)

def update_params(base: dict, override: dict) -> dict:
    for k, v in override.items():
        if isinstance(v, dict) and k in base and isinstance(base[k], dict):
            update_params(base[k], v)
        else:
            base[k] = v
    return base

def flatten_dict(d: dict, parent_key='', sep='.') -> dict:
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)

def nest_dict(flat: dict, sep='.') -> dict:
    nested = {}
    for k, v in flat.items():
        parts = k.split(sep)
        d = nested
        for part in parts[:-1]:
            if part not in d:
                d[part] = {}
            d = d[part]
        d[parts[-1]] = v
    return nested

def run_environment(seed: int, workspace_dir: Path, log_file):
    env = os.environ.copy()
    env["SEED"] = str(seed)
    env["GUI"] = "false"
    env["CONTROLLER_FOREGROUND"] = "1"
    
    print(f"[{datetime.datetime.now().time()}] 启动仿真环境 (auto.sh) ...")
    auto_sh_path = str(workspace_dir / "auto.sh")
    child = pexpect.spawn(auto_sh_path, env=env, encoding='utf-8', logfile=log_file, cwd=str(workspace_dir))
    
    try:
        child.expect("Use keyboard input in this terminal", timeout=120)
        print(f"[{datetime.datetime.now().time()}] 控制器已启动，发送 '2' (Stand) ...")
        time.sleep(3.0)
        child.sendline("2")
        print(f"[{datetime.datetime.now().time()}] 等待起立完成，发送 '6' (RL mode) ...")
        time.sleep(6.0)
        child.sendline("6")
        time.sleep(2.0)
        print(f"[{datetime.datetime.now().time()}] 仿真环境准备完毕！")
    except pexpect.TIMEOUT:
        print("错误：启动仿真环境超时！")
        return None
    except pexpect.EOF:
        print("错误：auto.sh 意外退出！")
        return None
        
    return child

def cleanup():
    print("清理残留进程...")
    os.system('pkill -f "roslaunch unitree_guide multi_floor_gazeboSim.launch" 2>/dev/null || true')
    os.system('pkill -f "building_generator_classic_control" 2>/dev/null || true')
    os.system('pkill -f "gzserver|gzclient|gazebo" 2>/dev/null || true')
    os.system('pkill -f "junior_ctrl" 2>/dev/null || true')
    os.system('pkill -f "danger_search_node" 2>/dev/null || true')
    time.sleep(2.0)

def main():
    parser = argparse.ArgumentParser(description="运行单一 seed 评测")
    parser.add_argument("--seed", type=int, required=True, help="随机种子")
    parser.add_argument("--params", type=str, help="参数重载 JSON 或 YAML 文件路径")
    parser.add_argument("--timeout", type=int, default=620, help="算法运行最大超时时间(秒)")
    parser.add_argument("--max-floor", type=int, default=3, help="最大探索楼层数")
    args = parser.parse_args()

    workspace_dir = Path("/home/xbh/tzb26/codeDG/SimEnv")
    danger_search_dir = workspace_dir / "src" / "danger_search"
    
    # 确保没有残留进程
    cleanup()

    # 创建归档目录
    run_dir = workspace_dir / generate_run_dir(args.seed)
    print(f"运行归档目录: {run_dir}")

    # 处理配置
    base_config_path = danger_search_dir / "config" / "danger_search.yaml"
    config_data = load_yaml(str(base_config_path))
    
    if args.params:
        with open(args.params, 'r') as f:
            if args.params.endswith('.json'):
                override_data = json.load(f)
            else:
                override_data = yaml.safe_load(f)
        # 支持平铺结构的参数覆盖
        flat_config = flatten_dict(config_data, sep='/')
        for k, v in override_data.items():
            flat_config[k] = v
        config_data = nest_dict(flat_config, sep='/')
        
    run_config_path = run_dir / "danger_search.yaml"
    save_yaml(config_data, str(run_config_path))

    # 启动环境
    env_log_path = run_dir / "auto_sh.log"
    with open(env_log_path, 'w') as env_log:
        env_process = run_environment(args.seed, workspace_dir, env_log)
        if env_process is None:
            cleanup()
            sys.exit(1)

        # 启动 danger_search
        print(f"[{datetime.datetime.now().time()}] 启动 danger_search 节点 ...")
        result_file_path = run_dir / "detected_danger.json"
        launch_cmd = [
            "bash", "-c",
            f"source {workspace_dir}/devel/setup.bash && roslaunch danger_search danger_search.launch "
            f"config:={run_config_path} max_floor:={args.max_floor} result_file:={result_file_path}"
        ]
        
        algo_log_path = run_dir / "danger_search.log"
        with open(algo_log_path, 'w') as algo_log:
            algo_process = subprocess.Popen(launch_cmd, stdout=algo_log, stderr=subprocess.STDOUT)
            try:
                algo_process.wait(timeout=args.timeout)
                print(f"[{datetime.datetime.now().time()}] danger_search 节点已正常退出。")
            except subprocess.TimeoutExpired:
                print(f"[{datetime.datetime.now().time()}] 达到最大超时限制 ({args.timeout}s)，发送 SIGINT。")
                algo_process.send_signal(subprocess.signal.SIGINT)
                try:
                    algo_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    algo_process.kill()
                    algo_process.wait()

    # 环境清理
    print(f"[{datetime.datetime.now().time()}] 正在清理环境 ...")
    env_process.close(force=True)
    cleanup()

    # 执行评测
    truth_file_path = workspace_dir / "results" / "danger_truth.json"
    eval_output_path = run_dir / "evaluation.json"
    
    if result_file_path.exists() and truth_file_path.exists():
        print("开始进行离线评测 ...")
        eval_script = workspace_dir / "src" / "building_obstacles" / "scripts" / "evaluate_danger.py"
        eval_cmd = [
            "python3", str(eval_script),
            "--truth-file", str(truth_file_path),
            "--detected-file", str(result_file_path),
            "--output-file", str(eval_output_path)
        ]
        try:
            subprocess.run(eval_cmd, check=True)
            print(f"评测完成，结果保存在 {eval_output_path}")
        except subprocess.CalledProcessError as e:
            print(f"评测脚本执行失败: {e}")
    else:
        print("未找到 detected_danger.json 或 danger_truth.json，无法评测。")

    # 保存一些附加信息
    mission_summary = {
        "seed": args.seed,
        "timeout": args.timeout,
        "max_floor": args.max_floor,
        "completed": result_file_path.exists(),
        "timestamp": datetime.datetime.now().isoformat()
    }
    with open(run_dir / "mission_summary.json", 'w') as f:
        json.dump(mission_summary, f, indent=2)
        
    # 拷贝真值文件作为备份
    if truth_file_path.exists():
        shutil.copy(truth_file_path, run_dir / "danger_truth.json")

    print(f"所有任务完成。归档保存在: {run_dir}")

if __name__ == "__main__":
    main()

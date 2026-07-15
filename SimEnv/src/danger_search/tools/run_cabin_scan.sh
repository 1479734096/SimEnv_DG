#!/usr/bin/env bash
set -euo pipefail

# 任务一：电梯轿厢原地多朝向扫描
# 启动 run_observation.sh（floor=2，kill 边界 floor=1 max_x=1.5），
# 等待 ROS 就绪后启动 cabin_scan_goals.py 在轿厢内发不同朝向目标。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"

SEED=42
TIMEOUT=2400
PARAMS="$SIM_DIR/config/three_floor_time_budget_test.yaml"
STAMP="$(date +%Y%m%d_%H%M%S)"
WRAPPER_LOG="$SIM_DIR/logs/cabin_scan_${STAMP}.log"

export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
source /opt/ros/noetic/setup.bash
source "$SIM_DIR/devel/setup.bash"

mkdir -p "$SIM_DIR/logs"

echo "=== 任务一：电梯轿厢原地扫描 ===" | tee "$WRAPPER_LOG"
echo "时间: $STAMP" | tee -a "$WRAPPER_LOG"

OBS_PID=""
GOAL_PID=""
OBS_DIR=""

cleanup() {
  trap - EXIT INT TERM
  if [ -n "$GOAL_PID" ] && kill -0 "$GOAL_PID" 2>/dev/null; then
    kill -INT "$GOAL_PID" 2>/dev/null || true
    wait "$GOAL_PID" 2>/dev/null || true
  fi
  if [ -n "$OBS_PID" ] && kill -0 "$OBS_PID" 2>/dev/null; then
    kill -INT "$OBS_PID" 2>/dev/null || true
    wait "$OBS_PID" 2>/dev/null || true
  fi
  echo "=== 封装退出 ===" | tee -a "$WRAPPER_LOG"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# 启动 run_observation.sh
# kill 边界：floor=1 时 x<=1.5 触发（机器人离开轿厢进入空带）
# 姿态守卫：z<=0.2（跌落地面层）、|roll|>=0.55、|pitch|>=0.6
# posture-active-after-sim=10 跳过出生下落稳定期
# sim-time 上限 200s（足够电梯换层 + 6 个朝向扫描）
"$SCRIPT_DIR/run_observation.sh" \
  --seed "$SEED" \
  --max-floor 2 \
  --params "$PARAMS" \
  --timeout "$TIMEOUT" \
  --ground-support-capture \
  --ground-support-shadow \
  --kill-only-floor 1 \
  --kill-only-max-x 1.5 \
  --kill-only-minimum-z 0.2 \
  --kill-only-maximum-abs-roll 0.55 \
  --kill-only-maximum-abs-pitch 0.6 \
  --kill-only-posture-active-after-sim 10 \
  --kill-only-sim-time 350 \
  --diagnostic-rate 2.0 \
  > "$WRAPPER_LOG.obs" 2>&1 &
OBS_PID=$!

echo "run_observation.sh PID=$OBS_PID" | tee -a "$WRAPPER_LOG"

# 等待观察采集就绪（ROS master + rosbag + 录屏启动）
echo "等待观察采集就绪..." | tee -a "$WRAPPER_LOG"
READY=0
for _ in $(seq 1 300); do
  if ! kill -0 "$OBS_PID" 2>/dev/null; then
    echo "run_observation.sh 在就绪前退出" | tee -a "$WRAPPER_LOG"
    cat "$WRAPPER_LOG.obs" | tail -30 >> "$WRAPPER_LOG"
    exit 1
  fi
  if grep -q "观察采集已启动" "$WRAPPER_LOG.obs" 2>/dev/null; then
    READY=1
    break
  fi
  sleep 1
done

if [ "$READY" != "1" ]; then
  echo "等待观察采集就绪超时" | tee -a "$WRAPPER_LOG"
  exit 1
fi

# 提取 observation 目录
OBS_DIR="$(grep -m1 '^observation_dir=' "$WRAPPER_LOG.obs" | cut -d= -f2)"
if [ -z "$OBS_DIR" ]; then
  echo "无法提取 observation_dir" | tee -a "$WRAPPER_LOG"
  exit 1
fi
echo "observation_dir=$OBS_DIR" | tee -a "$WRAPPER_LOG"

# 等待 ROS master 完全就绪
sleep 3

# 启动轿厢扫描目标发布器
GOAL_OUTPUT="$OBS_DIR/cabin_scan_goals.json"
echo "启动轿厢扫描目标发布器: $GOAL_OUTPUT" | tee -a "$WRAPPER_LOG"
python3 "$SCRIPT_DIR/cabin_scan_goals.py" \
  --output "$GOAL_OUTPUT" \
  --dwell-seconds 4.0 \
  --goal-timeout 20.0 \
  >> "$WRAPPER_LOG.goal" 2>&1 &
GOAL_PID=$!

echo "cabin_scan_goals.py PID=$GOAL_PID" | tee -a "$WRAPPER_LOG"
echo "等待 run_observation.sh 和 goal 发布器完成..." | tee -a "$WRAPPER_LOG"

# 等待 run_observation.sh 完成
wait "$OBS_PID"
OBS_STATUS=$?
echo "run_observation.sh 退出，状态=$OBS_STATUS" | tee -a "$WRAPPER_LOG"

# 等待 goal 发布器退出（最多 10 秒）
if kill -0 "$GOAL_PID" 2>/dev/null; then
  for _ in $(seq 1 10); do
    if ! kill -0 "$GOAL_PID" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if kill -0 "$GOAL_PID" 2>/dev/null; then
    kill -INT "$GOAL_PID" 2>/dev/null || true
    wait "$GOAL_PID" 2>/dev/null || true
  fi
fi

echo "=== 任务一运行完成 ===" | tee -a "$WRAPPER_LOG"
echo "observation_dir=$OBS_DIR" | tee -a "$WRAPPER_LOG"
if [ -f "$GOAL_OUTPUT" ]; then
  echo "目标日志:" | tee -a "$WRAPPER_LOG"
  python3 -c 'import json; d=json.load(open(sys.argv[1])); print("  朝向数:", len(d.get("entries",[]))); [print("  %s: yaw=%.2f (%.0f°)" % (e["label"], e.get("yaw_median",0) or 0, __import__("math").degrees(e.get("yaw_median",0) or 0))) for e in d.get("entries",[])]' "$GOAL_OUTPUT" 2>/dev/null | tee -a "$WRAPPER_LOG" || true
fi
exit "$OBS_STATUS"

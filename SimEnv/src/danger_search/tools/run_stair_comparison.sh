#!/usr/bin/env bash
set -euo pipefail

# 任务三：楼梯接近与通过诊断
# floor=0 运行，姿态守卫防跌落，stair_approach_goals.py 引导：
# 1. 安全预备点 (-1.8, 1.0)
# 2. 楼梯上方目标 (-2.5, 4.0)，让生产控制链自行接近

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"

SEED=42
TIMEOUT=1200
PARAMS="$SIM_DIR/config/three_floor_time_budget_test.yaml"
STAMP="$(date +%Y%m%d_%H%M%S)"
WRAPPER_LOG="$SIM_DIR/logs/stair_comparison_${STAMP}.log"

export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
source /opt/ros/noetic/setup.bash
source "$SIM_DIR/devel/setup.bash"

mkdir -p "$SIM_DIR/logs"

echo "=== 任务三：楼梯接近与通过诊断 ===" | tee "$WRAPPER_LOG"
echo "时间: $STAMP" | tee -a "$WRAPPER_LOG"

OBS_PID=""
GOAL_PID=""

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

# floor=0，姿态守卫：z<=0.1（楼层0跌落）、|roll|>=0.55、|pitch|>=0.6
# posture-active-after-sim=10 跳过出生稳定期
# sim-time 150s（足够预备点 + 楼梯尝试）
"$SCRIPT_DIR/run_observation.sh" \
  --seed "$SEED" \
  --max-floor 1 \
  --params "$PARAMS" \
  --timeout "$TIMEOUT" \
  --ground-support-capture \
  --ground-support-shadow \
  --kill-only-minimum-z 0.1 \
  --kill-only-maximum-abs-roll 0.55 \
  --kill-only-maximum-abs-pitch 0.6 \
  --kill-only-posture-active-after-sim 10 \
  --kill-only-sim-time 150 \
  --diagnostic-rate 2.0 \
  > "$WRAPPER_LOG.obs" 2>&1 &
OBS_PID=$!

echo "run_observation.sh PID=$OBS_PID" | tee -a "$WRAPPER_LOG"

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

OBS_DIR="$(grep -m1 '^observation_dir=' "$WRAPPER_LOG.obs" | cut -d= -f2)"
if [ -z "$OBS_DIR" ]; then
  echo "无法提取 observation_dir" | tee -a "$WRAPPER_LOG"
  exit 1
fi
echo "observation_dir=$OBS_DIR" | tee -a "$WRAPPER_LOG"

sleep 3

GOAL_OUTPUT="$OBS_DIR/stair_approach.json"
echo "启动楼梯接近目标发布器: $GOAL_OUTPUT" | tee -a "$WRAPPER_LOG"
python3 "$SCRIPT_DIR/stair_approach_goals.py" \
  --output "$GOAL_OUTPUT" \
  >> "$WRAPPER_LOG.goal" 2>&1 &
GOAL_PID=$!

echo "stair_approach_goals.py PID=$GOAL_PID" | tee -a "$WRAPPER_LOG"
echo "等待完成..." | tee -a "$WRAPPER_LOG"

wait "$OBS_PID"
OBS_STATUS=$?
echo "run_observation.sh 退出，状态=$OBS_STATUS" | tee -a "$WRAPPER_LOG"

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

echo "=== 任务三运行完成 ===" | tee -a "$WRAPPER_LOG"
echo "observation_dir=$OBS_DIR" | tee -a "$WRAPPER_LOG"
if [ -f "$GOAL_OUTPUT" ]; then
  python3 -c 'import json; d=json.load(open(sys.argv[1])); print("outcome:", d.get("outcome")); [print("  %s" % e.get("event","?")) for e in d.get("log",[])]' "$GOAL_OUTPUT" 2>/dev/null | tee -a "$WRAPPER_LOG" || true
fi
exit "$OBS_STATUS"

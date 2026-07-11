#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$ROOT_DIR/SimEnv"
cd "$SIM_DIR"

source /opt/ros/noetic/setup.bash
source "$SIM_DIR/devel/setup.bash"

export DANGER_COUNT=1
export DISTRACTOR_COUNT=0
export FLOOR_COUNT=1
export SEED=42
export GUI=true
export PAUSED=false

RUN_PID=""
BAG_PID=""

cleanup_owned() {
  trap - EXIT INT TERM
  if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [ -n "$RUN_PID" ] && kill -0 "$RUN_PID" 2>/dev/null; then
    kill -INT "$RUN_PID" 2>/dev/null || true
    wait "$RUN_PID" 2>/dev/null || true
  fi
}

trap cleanup_owned EXIT INT TERM

# 先让 run_seed 启动 ROS；rosbag 在 master 可用后再启动，避免启动即失败。
python3 src/danger_search/tools/run_seed.py \
  --seed 42 \
  --gui \
  --max-floor 1 \
  --timeout 10000 &
RUN_PID=$!

for _ in $(seq 1 180); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$RUN_PID" 2>/dev/null; then
    echo "run_seed 在 ROS master 就绪前退出" >&2
    wait "$RUN_PID"
  fi
  sleep 1
done

if ! rosparam list >/dev/null 2>&1; then
  echo "等待 ROS master 超时" >&2
  exit 1
fi

mkdir -p "$ROOT_DIR/results"
BAG_PATH="$ROOT_DIR/results/baseline_$(date +%Y%m%d_%H%M%S).bag"
rosbag record -O "$BAG_PATH" /cmd_vel /danger_search/status /Odometry_gazebo &
BAG_PID=$!

echo "Started rosbag record with PID $BAG_PID"

RUN_STATUS=0
wait "$RUN_PID" || RUN_STATUS=$?
RUN_PID=""

echo "Stopping rosbag record..."
kill -INT "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true
BAG_PID=""

echo "Baseline execution finished. Bag: $BAG_PATH"
exit "$RUN_STATUS"

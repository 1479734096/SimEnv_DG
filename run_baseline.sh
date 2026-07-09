#!/usr/bin/env bash
set -e

cd /home/xbh/tzb26/codeDG/SimEnv

export DANGER_COUNT=1
export DISTRACTOR_COUNT=0
export FLOOR_COUNT=1
export SEED=42
export GUI=false
export PAUSED=false

# Start rosbag record in the background
mkdir -p /home/xbh/tzb26/codeDG/results
rosbag record -O /home/xbh/tzb26/codeDG/results/baseline.bag /cmd_vel /danger_search/status /Odometry_gazebo &
BAG_PID=$!

echo "Started rosbag record with PID $BAG_PID"

# Run the environment and algorithm using the existing tool
python3 src/danger_search/tools/run_seed.py --seed 42 --max-floor 1 --timeout 1200

# Kill rosbag record safely
echo "Stopping rosbag record..."
kill -INT $BAG_PID
wait $BAG_PID || true

echo "Baseline execution finished."

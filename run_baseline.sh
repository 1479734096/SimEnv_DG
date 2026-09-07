#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$ROOT_DIR/SimEnv"

# 加载 ROS 与工作空间环境
source /opt/ros/noetic/setup.bash
if [ -f "$SIM_DIR/devel/setup.bash" ]; then
  source "$SIM_DIR/devel/setup.bash"
fi

# 默认参数配置（支持外部环境变量覆盖）
export SEED="${SEED:-42}"
export GUI="${GUI:-true}"
export ELEVATOR_TEST_FLOOR_COUNT="${FLOOR_COUNT:-1}"
export ELEVATOR_TEST_FULL_CYCLE="${FULL_CYCLE:-1}"

echo "================================================================"
echo "启动基线评测任务 (Seed: $SEED, Floors: $ELEVATOR_TEST_FLOOR_COUNT, GUI: $GUI)"
echo "调用后端专项运行器: SimEnv/tools/sim/run_elevator_gui_test.sh"
echo "================================================================"

exec bash "$SIM_DIR/tools/sim/run_elevator_gui_test.sh" "$@"

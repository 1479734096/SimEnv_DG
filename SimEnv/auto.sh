#!/usr/bin/env bash
# 一键启动比赛仿真环境。
# 这个脚本负责把“生成随机比赛场景、启动 Gazebo、启动门/电梯服务、
# 启动 Unitree A1 控制器”串成一个固定流程。

# set -e：任意命令失败时立即退出，避免带着错误状态继续启动。
# set -u：使用未定义变量时报错，避免变量名写错后静默变成空值。
# set -o pipefail：管道中任意一段失败都视为失败，便于发现隐藏错误。
set -euo pipefail

# 无论从哪个目录执行 ./auto.sh，都先切换到脚本所在的工作区根目录。
# 这样后面的相对路径（devel、generated_building、results、logs）都稳定可用。
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# ==============================
# 启动参数
# ==============================
# 所有参数都支持通过环境变量覆盖，例如：
#   SEED=77 GUI=false FLOOR_COUNT=1 ./auto.sh
#
# 场景生成参数：
# - SEED 为空时使用随机种子；指定后可以复现实验。
# - FLOOR_COUNT、ROOMS_PER_FLOOR 控制楼栋复杂度，也是影响 Gazebo 性能的关键参数。
# - DANGER_COUNT、DISTRACTOR_COUNT 支持固定值或 min:max 范围。
SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-3}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"

# Gazebo 启动参数：
# - GUI=false 可关闭 Gazebo 图形界面，通常能显著提升仿真速度。
# - PAUSED=true 表示 Gazebo 启动后先暂停，便于检查场景和机器人状态。
GUI="${GUI:-true}"
PAUSED="${PAUSED:-true}"

# 附加进程开关：
# - START_CONTROLLER 控制是否启动 junior_ctrl。
# - START_VIRTUAL_JOY 控制是否启动虚拟手柄，通常比赛算法不需要。
# - CONTROLLER_FOREGROUND=1 时 junior_ctrl 在当前终端前台运行，便于键盘输入 2/6 切换状态。
# - START_BUILDING_CONTROL 控制是否启动门/电梯服务。
START_CONTROLLER="${START_CONTROLLER:-1}"
START_VIRTUAL_JOY="${START_VIRTUAL_JOY:-0}"
CONTROLLER_FOREGROUND="${CONTROLLER_FOREGROUND:-1}"
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}"
RUN_PID_DIR="${RUN_PID_DIR:-$WORKSPACE_DIR/logs/run_pids}"

OWNED_PIDS=()

record_owned_pid() {
  local name="$1"
  local pid="$2"
  OWNED_PIDS+=("$pid")
  mkdir -p "$RUN_PID_DIR"
  printf '%s\n' "$pid" > "$RUN_PID_DIR/$name.pid"
}

cleanup_owned_processes() {
  local pid
  local _
  local any_alive
  trap - EXIT INT TERM
  for pid in "${OWNED_PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for _ in $(seq 1 50); do
    any_alive=0
    for pid in "${OWNED_PIDS[@]:-}"; do
      if kill -0 "$pid" 2>/dev/null; then
        any_alive=1
      fi
    done
    if [ "$any_alive" = "0" ]; then
      break
    fi
    sleep 0.2
  done
  for pid in "${OWNED_PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${OWNED_PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}

trap cleanup_owned_processes EXIT INT TERM

# Unitree 控制器周期，默认 0.004 秒，即 250 Hz。
# 周期太小会更接近原始控制频率，但大场景下更容易出现控制循环超时。
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.004}"

# 机器人初始位姿，会同时传给场景生成器和 Gazebo spawn_model。
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:--2.2}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"

# ==============================
# 检查 libtorch 运行库
# ==============================
# junior_ctrl 链接 GPU 版 libtorch。部分 libtorch 包内 CUDA 库文件名带哈希，
# 但可执行文件运行期按标准 SONAME 查找，因此启动前先补齐安全的同目录软链接。
if [ "$START_CONTROLLER" = "1" ]; then
  LIBTORCH_RUNTIME_SCRIPT="$WORKSPACE_DIR/scripts/prepare_libtorch_runtime.sh"
  JUNIOR_CTRL_BIN="$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl"
  DETECTED_LIBTORCH_ROOT="${LIBTORCH_ROOT:-}"

  if [ -z "$DETECTED_LIBTORCH_ROOT" ] && [ -f "$JUNIOR_CTRL_BIN" ] && command -v readelf >/dev/null 2>&1; then
    DETECTED_LIBTORCH_ROOT="$(
      readelf -d "$JUNIOR_CTRL_BIN" 2>/dev/null \
        | sed -n 's/.*Library runpath: \[\(.*\)\].*/\1/p' \
        | tr ':' '\n' \
        | sed -n 's#^\(.*\)/libtorch/lib$#\1/libtorch#p' \
        | head -n 1
    )"
  fi

  if [ -n "$DETECTED_LIBTORCH_ROOT" ]; then
    bash "$LIBTORCH_RUNTIME_SCRIPT" "$DETECTED_LIBTORCH_ROOT"
  else
    bash "$LIBTORCH_RUNTIME_SCRIPT"
  fi
fi

# ==============================
# 启动冲突检查
# ==============================
# 不主动终止未知进程；已有仿真时直接拒绝启动，避免破坏其他用户的任务。
CONFLICTS="$(
  {
    pgrep -a -x rosmaster 2>/dev/null || true
    pgrep -a -x rosout 2>/dev/null || true
    pgrep -a -x gzserver 2>/dev/null || true
    pgrep -a -x gzclient 2>/dev/null || true
    pgrep -a -x junior_ctrl 2>/dev/null || true
    pgrep -af '[r]oslaunch unitree_guide multi_floor_gazeboSim.launch' 2>/dev/null || true
    pgrep -af '[b]uilding_generator_classic_control' 2>/dev/null || true
  } | sort -u
)"
if [ -n "$CONFLICTS" ]; then
  echo "检测到已有 ROS/Gazebo 仿真进程，拒绝启动以避免实例冲突：" >&2
  printf '%s\n' "$CONFLICTS" >&2
  exit 2
fi

# ==============================
# 加载 ROS 和工作区环境
# ==============================
# /opt/ros/noetic/setup.bash 提供系统 ROS 包。
# devel/setup.bash 提供当前工作区 catkin_make 后生成的包、节点和消息类型。
# 如果 devel/setup.bash 不存在，说明还没有成功编译，需要先运行 catkin_make。
echo "Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"

# 通过 rospack 获取包路径，避免把 src 下的绝对路径写死。
# building_obstacles 提供比赛场景生成脚本。
# unitree_gazebo/models 提供 Gazebo 需要解析的 Unitree 模型资源。
BUILDING_OBSTACLES_DIR="$(rospack find building_obstacles)"
UNITREE_GAZEBO_MODELS="$(rospack find unitree_gazebo)/models"

# 生成文件、结果文件和日志的固定输出目录。
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR" "$WORKSPACE_DIR/logs"

# ==============================
# 生成比赛场景
# ==============================
# generate_competition_scene.py 会生成：
# - generated_building/competition_scene.world：Gazebo 加载的 world 文件
# - generated_building/layout_metadata.json：楼栋、房间、门、电梯等元数据
# - generated_building/door_config.yaml：门控制配置
# - generated_building/elevator_config.yaml：电梯控制配置
# - generated_building/scene_manifest.json：本次场景 manifest
# - results/danger_truth.json：危险源真值，供裁判评估使用
echo "Generating competition scene..."
GENERATOR_ARGS=(
  --output-dir "$SCENE_OUTPUT_DIR"
  --results-dir "$RESULTS_DIR"
  --floor-count "$FLOOR_COUNT"
  --rooms-per-floor "$ROOMS_PER_FLOOR"
  --width "$BUILDING_WIDTH"
  --length "$BUILDING_LENGTH"
  --danger-count "$DANGER_COUNT"
  --distractor-count "$DISTRACTOR_COUNT"
  --robot-x "$ROBOT_X"
  --robot-y "$ROBOT_Y"
  --robot-z "$ROBOT_Z"
  --robot-yaw "$ROBOT_YAW"
)

# 只有用户显式设置 SEED 时才传入 --seed。
# 不传 seed 时，生成器内部会使用随机种子。
if [ -n "$SEED" ]; then
  GENERATOR_ARGS+=(--seed "$SEED")
fi

# stdout 额外保存到 scene_manifest.stdout.json，便于排查生成器输出。
# 真正被 Gazebo 使用的是下面导出的 competition_scene.world。
python3 "$BUILDING_OBSTACLES_DIR/scripts/generate_competition_scene.py" "${GENERATOR_ARGS[@]}" \
  > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"

# ==============================
# 导出给 launch / 控制器使用的环境变量
# ==============================
# multi_floor_gazeboSim.launch 会读取 BUILDING_WORLD_FILE 加载刚生成的 world。
export BUILDING_WORLD_FILE="$SCENE_OUTPUT_DIR/competition_scene.world"

# 机器人出生点也通过环境变量传给 launch，保持场景生成和 Gazebo spawn 位置一致。
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"

# junior_ctrl 会读取该控制周期。
export UNITREE_CTRL_DT

# Gazebo 通过 GAZEBO_MODEL_PATH 查找模型资源。
# 这里追加生成场景目录和 Unitree 模型目录，保留用户已有的 GAZEBO_MODEL_PATH。
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"

# 打印本次启动生成的关键文件，方便用户定位 world、真值和结果文件。
echo "=========================================="
echo "Competition scene is ready"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Truth:   $RESULTS_DIR/danger_truth.json"
echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "=========================================="

# ==============================
# 可选：启动虚拟手柄
# ==============================
# 默认不启动。比赛算法一般直接发布 /cmd_vel，不需要虚拟手柄。
# 开启后日志写入 logs/virtual_joy.log。
if [ "$START_VIRTUAL_JOY" = "1" ]; then
  echo "Starting virtual joystick. This may require uinput permissions."
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  record_owned_pid "virtual_joy" "$!"
fi

# ==============================
# 启动 Gazebo、A1 模型、传感器和 ROS 接口
# ==============================
# multi_floor_gazeboSim.launch 内部会做这些事：
# - 使用 gazebo_ros/empty_world.launch 加载 BUILDING_WORLD_FILE
# - 通过 xacro 生成 A1 的 robot_description
# - spawn A1 到 Gazebo
# - 启动 state_from_gazebo，发布 /Odometry_gazebo
# - 启动 12 个腿部关节控制器
# - 启动 robot_state_publisher
# - 启动 joy_node
# - 启动 pointcloud2livox.py，把 Livox 点云转换成算法使用的话题
echo "Launching Gazebo, Unitree A1 model, sensors, and ROS interfaces..."
roslaunch unitree_guide multi_floor_gazeboSim.launch \
  gui:="$GUI" \
  paused:="$PAUSED" \
  user_debug:=False \
  rname:=a1 \
  robot_x:="$ROBOT_X" \
  robot_y:="$ROBOT_Y" \
  robot_z:="$ROBOT_Z" \
  robot_yaw:="$ROBOT_YAW" \
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &

# 记录 roslaunch 进程号，便于后续人工排查或停止。
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$WORKSPACE_DIR/logs/competition_gazebo.pid"
record_owned_pid "competition_gazebo" "$LAUNCH_PID"

# 给 Gazebo、模型和 ROS 服务一点初始化时间。
# 后面的门/电梯控制服务依赖 Gazebo 服务已经起来。
sleep 6

# ==============================
# 启动门 / 电梯控制服务
# ==============================
# building_generator_classic_control 会读取场景生成阶段写出的门和电梯配置，
# 并通过 Gazebo 服务移动门模型、电梯轿厢等对象。
if [ "$START_BUILDING_CONTROL" = "1" ]; then
  echo "Starting building door/elevator control service..."
  rosrun building_generator_classic building_generator_classic_control \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  BUILDING_CONTROL_PID=$!
  echo "$BUILDING_CONTROL_PID" > "$WORKSPACE_DIR/logs/building_control.pid"
  record_owned_pid "building_control" "$BUILDING_CONTROL_PID"
fi

# ==============================
# 启动 Unitree A1 控制器
# ==============================
# junior_ctrl 是机器狗运动控制核心。
# 默认在前台运行，因为原始控制流程需要在终端输入：
# - 2：进入站立状态
# - 6：进入 RL 模式
# 进入 RL 模式后，算法发布 geometry_msgs/Twist 到 /cmd_vel 才会生效。
# 等待 ROS Master 和 Gazebo 核心完全启动，防止 junior_ctrl 报 Connection refused 和触发 SIGTRAP。
echo "Waiting for ROS and Gazebo to initialize..."
sleep 5

if [ "$START_CONTROLLER" = "1" ]; then
  if [ "$CONTROLLER_FOREGROUND" = "1" ]; then
    echo "Starting junior_ctrl controller in the foreground."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    echo "Use keyboard input in this terminal: 2 = stand, 6 = RL mode."
    "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl"
  else
    # 后台模式适合无人值守启动，但通常无法方便地通过当前终端输入 2/6。
    # 如果要切换状态，建议保持默认前台模式。
    echo "Starting junior_ctrl controller in the background. Keyboard state switching may not be available."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl" \
      > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
    record_owned_pid "junior_ctrl" "$!"
  fi
fi

# 如果 START_CONTROLLER=1 且 CONTROLLER_FOREGROUND=1，脚本通常会停留在 junior_ctrl。
# 只有控制器退出，才会执行到下面两行。
echo "Simulation startup command completed."
echo "Controller mode remains governed by unitree_guide keyboard/joy input; publish geometry_msgs/Twist to /cmd_vel after RL mode is enabled."

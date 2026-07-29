#!/usr/bin/env bash
# 电梯专项 GUI 测试：默认验证两层，可显式扩展到三层完整往返。

set -Eeuo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BASE_CONFIG="$WORKSPACE_ROOT/src/danger_search/config/danger_search.yaml"
DEVEL_SETUP="$WORKSPACE_ROOT/devel/setup.bash"

SEED="${SEED:-42}"
GUI="${GUI:-true}"
ELEVATOR_TEST_FLOOR_COUNT="${ELEVATOR_TEST_FLOOR_COUNT:-2}"
ELEVATOR_TEST_FLOOR_TIMEOUT="${ELEVATOR_TEST_FLOOR_TIMEOUT:-33.0}"
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME="${ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME:-$ELEVATOR_TEST_FLOOR_TIMEOUT}"
ELEVATOR_TEST_MAX_RUNTIME="${ELEVATOR_TEST_MAX_RUNTIME:-180.0}"
ELEVATOR_TEST_WALL_TIMEOUT="${ELEVATOR_TEST_WALL_TIMEOUT:-1800}"
ELEVATOR_GO_TO_DOOR_STALL_WALL="${ELEVATOR_GO_TO_DOOR_STALL_WALL:-120}"
ELEVATOR_TEST_KEEP_RUNNING="${ELEVATOR_TEST_KEEP_RUNNING:-0}"
ELEVATOR_TEST_FULL_CYCLE="${ELEVATOR_TEST_FULL_CYCLE:-0}"
ELEVATOR_TEST_RECORD_DRIFT="${ELEVATOR_TEST_RECORD_DRIFT:-0}"
ELEVATOR_TEST_RECORD_RGBD="${ELEVATOR_TEST_RECORD_RGBD:-0}"
ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH="${ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH:-0}"
ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS="${ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS:-0}"
ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS="${ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS:-0}"
ELEVATOR_TEST_LOCALIZATION_BACKEND="${ELEVATOR_TEST_LOCALIZATION_BACKEND:-adaptive_shared_map}"
LOG_DIR="${ELEVATOR_TEST_LOG_DIR:-/tmp/danger_search_elevator_test_$(date +%Y%m%d_%H%M%S)}"

AUTO_PID=""
MISSION_PID=""
KEEPER_PID=""
MASTER_PID=""
BAG_PID=""
RGBD_BAG_PID=""
RUNTIME_DIR=""
RESULT="未完成"
DETAIL="脚本提前退出"

usage() {
  cat <<'EOF'
用法：
  tools/sim/run_elevator_gui_test.sh

目标：
  复用正式定位和 MissionManager，只缩短专项楼层预算；成功条件是电梯门板
  完成传感器注册，并到达显式楼层数的最高层；可选继续验证从最高层返回 0 层、
  主入口退出和结果写出。

常用环境变量：
  SEED=42                              场景种子
  GUI=true                             是否打开 Gazebo GUI
  ELEVATOR_TEST_FLOOR_COUNT=2          楼层数，仅支持 2 或 3
  ELEVATOR_TEST_FLOOR_TIMEOUT=33.0     楼层 0 专项预算（仿真秒）
  ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5.0  每层最小驻留（仿真秒）
  ELEVATOR_TEST_MAX_RUNTIME=180.0      专项总预算（仿真秒）
  ELEVATOR_TEST_WALL_TIMEOUT=1800      墙钟总超时（秒）
  ELEVATOR_GO_TO_DOOR_STALL_WALL=120   厅外导航无路径墙钟超时（秒）
  ELEVATOR_TEST_FULL_CYCLE=1            继续验证返程电梯与主入口退出
  ELEVATOR_TEST_RECORD_DRIFT=1          录制自身定位、控制和姿态诊断 bag
  ELEVATOR_TEST_RECORD_RGBD=1           额外录制 RGB-D 供失败后离线重放
  ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1  同时录制真值供运行结束后离线漂移评估
  ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1  启用逐帧定位日志并在诊断 bag 录制 /scan
  ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS=1  启用只读电梯地标漂移诊断
  ELEVATOR_TEST_LOCALIZATION_BACKEND=adaptive_shared_map  选择定位后端，默认 adaptive_shared_map
  ELEVATOR_TEST_KEEP_RUNNING=1         成功后保留 GUI，按 Ctrl-C 清理
  ELEVATOR_TEST_LOG_DIR=/tmp/...       证据目录

返回码：
  0  电梯视觉注册和目标换层链成功
  1  算法失败或专项超时
  2  启动/环境前置失败

合规边界：
  默认显式关闭真值话题、裁判里程计和真值点云转换；生产节点仍以 formal
  profile 启动。只有显式设置 ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 时才发布
  真值，并且只由独立 rosbag 诊断进程记录，运行结束后离线分析，不参与决策。
  脚本不读取真值文件，不修改正式 YAML，也不修改官方文件。
EOF
}

log() {
  printf '[电梯专项] %s\n' "$*"
}

is_positive_number() {
  [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk -v value="$1" 'BEGIN { exit !(value > 0.0) }'
}

is_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

terminate_group() {
  local pid="${1:-}"
  local signal="${2:-TERM}"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill "-$signal" -- "-$pid" 2>/dev/null || true
  fi
}

write_summary() {
  {
    printf 'result=%s\n' "$RESULT"
    printf 'detail=%s\n' "$DETAIL"
    printf 'seed=%s\n' "$SEED"
    printf 'floor_count=%s\n' "$ELEVATOR_TEST_FLOOR_COUNT"
    printf 'floor_timeout=%s\n' "$ELEVATOR_TEST_FLOOR_TIMEOUT"
    printf 'min_floor_explore_time=%s\n' "$ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME"
    printf 'max_runtime=%s\n' "$ELEVATOR_TEST_MAX_RUNTIME"
    printf 'full_cycle=%s\n' "$ELEVATOR_TEST_FULL_CYCLE"
    printf 'record_drift=%s\n' "$ELEVATOR_TEST_RECORD_DRIFT"
    printf 'record_rgbd=%s\n' "$ELEVATOR_TEST_RECORD_RGBD"
    printf 'offline_truth=%s\n' "$ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH"
    printf 'localization_diagnostics=%s\n' "$ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS"
    printf 'landmark_drift_diagnostics=%s\n' "$ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS"
    printf 'localization_backend=%s\n' "$ELEVATOR_TEST_LOCALIZATION_BACKEND"
    printf 'diagnostic_bag=%s/elevator_diagnostic.bag\n' "$LOG_DIR"
    printf 'rgbd_bag=%s/elevator_rgbd.bag\n' "$LOG_DIR"
    printf 'drift_summary=%s/localization_drift.json\n' "$LOG_DIR"
    printf 'localization_quality=%s/localization_quality.json\n' "$LOG_DIR"
    printf 'localization_quality_samples=%s/localization_quality.csv\n' "$LOG_DIR"
    printf 'localization_motion_windows=%s/localization_motion_windows.csv\n' "$LOG_DIR"
    printf 'localization_sensor_windows=%s/localization_sensor_windows.csv\n' "$LOG_DIR"
    printf 'landmark_drift_summary=%s/landmark_drift.json\n' "$LOG_DIR"
    printf 'mission_log=%s/mission.log\n' "$LOG_DIR"
    printf 'auto_log=%s/auto.log\n' "$LOG_DIR"
    printf 'result_file=%s/results/detected_danger.json\n' "$WORKSPACE_ROOT"
  } > "$LOG_DIR/TEST_SUMMARY.txt"
}

cleanup() {
  local exit_code=$?
  trap - EXIT
  terminate_group "$MISSION_PID" INT
  terminate_group "$RGBD_BAG_PID" INT
  terminate_group "$BAG_PID" INT
  sleep 1
  if [[ "$RGBD_BAG_PID" =~ ^[0-9]+$ ]]; then
    wait "$RGBD_BAG_PID" 2>/dev/null || true
  fi
  if [[ "$BAG_PID" =~ ^[0-9]+$ ]]; then
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ "$ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH" == "1" && \
        -f "$LOG_DIR/elevator_diagnostic.bag" ]]; then
    python3 "$WORKSPACE_ROOT/tools/offline/analyze_localization_drift_bag.py" \
      "$LOG_DIR/elevator_diagnostic.bag" \
      --csv "$LOG_DIR/localization_drift.csv" \
      --json "$LOG_DIR/localization_drift.json" \
      > "$LOG_DIR/localization_drift.log" 2>&1 || true
  fi
  if [[ "$ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS" == "1" && \
        "$ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH" == "1" && \
        -f "$LOG_DIR/elevator_diagnostic.bag" ]]; then
    python3 "$WORKSPACE_ROOT/tools/offline/analyze_localization_quality_bag.py" \
      "$LOG_DIR/elevator_diagnostic.bag" \
      --csv "$LOG_DIR/localization_quality.csv" \
      --windows-csv "$LOG_DIR/localization_motion_windows.csv" \
      --sensor-windows-csv "$LOG_DIR/localization_sensor_windows.csv" \
      --json "$LOG_DIR/localization_quality.json" \
      > "$LOG_DIR/localization_quality.log" 2>&1 || true
  fi
  if [[ "$ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS" == "1" && \
        -f "$LOG_DIR/mission.log" && \
        -f "$LOG_DIR/localization_drift.csv" && \
        -f "$LOG_DIR/localization_drift.json" ]]; then
    python3 "$WORKSPACE_ROOT/tools/offline/analyze_landmark_drift.py" \
      "$LOG_DIR/mission.log" \
      "$LOG_DIR/localization_drift.csv" \
      "$LOG_DIR/localization_drift.json" \
      --json "$LOG_DIR/landmark_drift.json" \
      > "$LOG_DIR/landmark_drift.log" 2>&1 || true
  fi
  terminate_group "$MISSION_PID" TERM
  terminate_group "$AUTO_PID" TERM
  if [[ "$AUTO_PID" =~ ^[0-9]+$ ]]; then
    for _attempt in 1 2 3 4 5; do
      if ! kill -0 "$AUTO_PID" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    terminate_group "$AUTO_PID" KILL
    wait "$AUTO_PID" 2>/dev/null || true
  fi
  terminate_group "$KEEPER_PID" TERM
  if [[ "$KEEPER_PID" =~ ^[0-9]+$ ]]; then
    for _attempt in 1 2 3; do
      if ! kill -0 "$KEEPER_PID" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    terminate_group "$KEEPER_PID" KILL
    wait "$KEEPER_PID" 2>/dev/null || true
  fi
  if [[ "$MASTER_PID" =~ ^[0-9]+$ ]] && kill -0 "$MASTER_PID" 2>/dev/null; then
    kill -TERM "$MASTER_PID" 2>/dev/null || true
  fi
  if [[ -n "$RUNTIME_DIR" && -d "$RUNTIME_DIR" ]]; then
    rm -rf -- "$RUNTIME_DIR"
  fi
  write_summary
  log "$RESULT：$DETAIL"
  log "证据目录：$LOG_DIR"
  exit "$exit_code"
}

fail() {
  RESULT="失败"
  DETAIL="$1"
  return "${2:-1}"
}

wait_for_log() {
  local file="$1"
  local pattern="$2"
  local timeout="$3"
  local watched_pid="$4"
  local started=$SECONDS
  while (( SECONDS - started < timeout )); do
    if [[ -f "$file" ]] && grep -Fq -- "$pattern" "$file"; then
      return 0
    fi
    if [[ -n "$watched_pid" ]] && ! kill -0 "$watched_pid" 2>/dev/null; then
      return 1
    fi
    sleep 1
  done
  return 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if (( $# > 0 )); then
  usage >&2
  exit 2
fi
if [[ "$ELEVATOR_TEST_FLOOR_COUNT" != "2" && "$ELEVATOR_TEST_FLOOR_COUNT" != "3" ]]; then
  printf 'ELEVATOR_TEST_FLOOR_COUNT 只能是 2 或 3\n' >&2
  exit 2
fi

for value in "$ELEVATOR_TEST_FLOOR_TIMEOUT" "$ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME" "$ELEVATOR_TEST_MAX_RUNTIME"; do
  if ! is_positive_number "$value"; then
    printf '非法正数参数：%s\n' "$value" >&2
    exit 2
  fi
done
for value in "$ELEVATOR_TEST_WALL_TIMEOUT" "$ELEVATOR_GO_TO_DOOR_STALL_WALL"; do
  if ! is_positive_integer "$value"; then
    printf '墙钟超时必须是正整数秒：%s\n' "$value" >&2
    exit 2
  fi
done
if [[ "$ELEVATOR_TEST_FULL_CYCLE" != "0" && "$ELEVATOR_TEST_FULL_CYCLE" != "1" ]]; then
  printf 'ELEVATOR_TEST_FULL_CYCLE 只能是 0 或 1\n' >&2
  exit 2
fi
case "$ELEVATOR_TEST_LOCALIZATION_BACKEND" in
  kiss_icp|imu_constrained|livox_imu_local_map|adaptive_shared_map|rgbd_livox) ;;
  *)
    printf 'ELEVATOR_TEST_LOCALIZATION_BACKEND 仅支持 kiss_icp、imu_constrained、livox_imu_local_map、adaptive_shared_map 或 rgbd_livox\n' >&2
    exit 2
    ;;
esac
for flag in "$ELEVATOR_TEST_RECORD_DRIFT" "$ELEVATOR_TEST_RECORD_RGBD" "$ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH" "$ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS" "$ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS"; do
  if [[ "$flag" != "0" && "$flag" != "1" ]]; then
    printf '诊断开关只能是 0 或 1：%s\n' "$flag" >&2
    exit 2
  fi
done
if [[ "$ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH" == "1" && "$ELEVATOR_TEST_RECORD_DRIFT" != "1" ]]; then
  printf '启用离线真值时必须同时设置 ELEVATOR_TEST_RECORD_DRIFT=1\n' >&2
  exit 2
fi
if [[ "$ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS" == "1" && "$ELEVATOR_TEST_RECORD_DRIFT" != "1" ]]; then
  printf '启用逐帧定位诊断时必须同时设置 ELEVATOR_TEST_RECORD_DRIFT=1\n' >&2
  exit 2
fi
if [[ "$ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS" == "1" && \
      ( "$ELEVATOR_TEST_RECORD_DRIFT" != "1" || \
        "$ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH" != "1" ) ]]; then
  printf '启用电梯地标漂移诊断时必须同时录制定位和独立离线真值\n' >&2
  exit 2
fi

if [[ ! -f "$BASE_CONFIG" || ! -f "$DEVEL_SETUP" ]]; then
  printf '缺少基础配置或已构建工作空间：%s\n' "$WORKSPACE_ROOT" >&2
  exit 2
fi
for command_name in awk find grep mkfifo roslaunch setsid sed ss tail; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    printf '缺少命令：%s\n' "$command_name" >&2
    exit 2
  fi
done
if [[ "$ELEVATOR_TEST_RECORD_DRIFT" == "1" ]]; then
  for command_name in rosbag rosnode; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
      printf '缺少诊断命令：%s\n' "$command_name" >&2
      exit 2
    fi
  done
fi
if [[ "$ELEVATOR_TEST_RECORD_RGBD" == "1" ]]; then
  for command_name in rosbag rosnode; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
      printf '缺少 RGB-D 诊断命令：%s\n' "$command_name" >&2
      exit 2
    fi
  done
fi
if [[ -n "$(find "$LOG_DIR" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null || true)" ]]; then
  printf '证据目录非空，拒绝覆盖：%s\n' "$LOG_DIR" >&2
  exit 2
fi

mkdir -p "$LOG_DIR"
RUNTIME_DIR="$(mktemp -d /tmp/danger_search_elevator_runtime.XXXXXX)"
FIFO="$RUNTIME_DIR/auto.stdin"
TEST_CONFIG="$LOG_DIR/elevator_test.yaml"
AUTO_LOG="$LOG_DIR/auto.log"
MISSION_LOG="$LOG_DIR/mission.log"
mkfifo "$FIFO"
trap cleanup EXIT
trap 'RESULT="中止"; DETAIL="收到用户或外部终止信号"; exit 130' INT TERM

# 仅生成证据目录中的专项副本；精确匹配可以在正式配置字段改名时 fail-closed。
LANDMARK_DIAGNOSTIC_VALUE=false
if [[ "$ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS" == "1" ]]; then
  LANDMARK_DIAGNOSTIC_VALUE=true
fi
sed \
  -e "s/^  max_runtime: 900[.]0$/  max_runtime: $ELEVATOR_TEST_MAX_RUNTIME/" \
  -e "s/^  runtime_by_floor: \[\[1, 300[.]0\], \[2, 620[.]0\], \[3, 840[.]0\]\]$/  runtime_by_floor: [[1, $ELEVATOR_TEST_MAX_RUNTIME], [2, $ELEVATOR_TEST_MAX_RUNTIME], [3, $ELEVATOR_TEST_MAX_RUNTIME]]/" \
  -e "s/^  floor_timeout: 280[.]0$/  floor_timeout: $ELEVATOR_TEST_FLOOR_TIMEOUT/" \
  -e "s/^  min_floor_explore_time: 60[.]0$/  min_floor_explore_time: $ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME/" \
  -e "s/^  landmark_drift_diagnostic_enabled: false$/  landmark_drift_diagnostic_enabled: $LANDMARK_DIAGNOSTIC_VALUE/" \
  "$BASE_CONFIG" > "$TEST_CONFIG"

if ! grep -Fq "  floor_timeout: $ELEVATOR_TEST_FLOOR_TIMEOUT" "$TEST_CONFIG" || \
   ! grep -Fq "  min_floor_explore_time: $ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME" "$TEST_CONFIG" || \
   ! grep -Fq "  runtime_by_floor: [[1, $ELEVATOR_TEST_MAX_RUNTIME], [2, $ELEVATOR_TEST_MAX_RUNTIME], [3, $ELEVATOR_TEST_MAX_RUNTIME]]" "$TEST_CONFIG" || \
   ! grep -Fq "  landmark_drift_diagnostic_enabled: $LANDMARK_DIAGNOSTIC_VALUE" "$TEST_CONFIG"; then
  fail "专项配置覆盖失败，正式配置字段可能已变化" 2
  exit $?
fi

source "$DEVEL_SETUP"
if [[ -n "$(ss -ltn 2>/dev/null | awk '$4 ~ /:11311$/ {print; exit}')" ]]; then
  fail "ROS master 端口 11311 已被占用，拒绝干扰现有仿真" 2
  exit $?
fi

log "证据目录：$LOG_DIR"
log "启动 seed=$SEED、floor_count=$ELEVATOR_TEST_FLOOR_COUNT、floor_timeout=$ELEVATOR_TEST_FLOOR_TIMEOUT 的 GUI 环境"

setsid bash -c 'exec 3<>"$1"; sleep 1000000' _ "$FIFO" \
  > "$LOG_DIR/fifo_keeper.log" 2>&1 &
KEEPER_PID=$!

setsid env \
  SEED="$SEED" \
  FLOOR_COUNT="$ELEVATOR_TEST_FLOOR_COUNT" \
  GUI="$GUI" \
  PAUSED=true \
  AUTO_UNPAUSE=1 \
  CONTROLLER_FOREGROUND=1 \
  ENABLE_REFEREE_ODOM=false \
  ENABLE_GROUND_TRUTH="$ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH" \
  ENABLE_POINTCLOUD_CONVERTER=false \
  POINTCLOUD_USE_GROUND_TRUTH_ODOM=false \
  WRITE_GENERATED_TRUTH_COPY=false \
  "$WORKSPACE_ROOT/auto.sh" > "$AUTO_LOG" 2>&1 < "$FIFO" &
AUTO_PID=$!

if ! wait_for_log "$AUTO_LOG" "12 路 Gazebo 关节状态和机身 IMU 均已有实时反馈。" 180 "$AUTO_PID"; then
  fail "Gazebo 或关节控制链未在 180 秒内就绪" 2
  exit $?
fi
MASTER_PID="$(ss -ltnp 2>/dev/null | awk '$4 ~ /:11311$/ {if (match($0, /pid=[0-9]+/)) {print substr($0, RSTART + 4, RLENGTH - 4); exit}}')"
if [[ ! "$MASTER_PID" =~ ^[0-9]+$ ]]; then
  fail "无法记录本轮 rosmaster PID，拒绝继续以免留下残余进程" 2
  exit $?
fi

printf '2' > "$FIFO"
if ! wait_for_log "$AUTO_LOG" "Switched from passive to fixed stand" 90 "$AUTO_PID"; then
  fail "机器人未进入 fixed stand" 2
  exit $?
fi

printf '6' > "$FIFO"
if ! wait_for_log "$AUTO_LOG" "Entered RL /cmd_vel mode." 30 "$AUTO_PID"; then
  fail "控制器未进入 RL /cmd_vel 模式" 2
  exit $?
fi

log "机器人已站立并进入 /cmd_vel 模式，启动正式任务节点"
setsid roslaunch danger_search danger_search.launch \
  controller_ready_ack:=true \
  config:="$TEST_CONFIG" \
  localization_backend:="$ELEVATOR_TEST_LOCALIZATION_BACKEND" \
  localization_diagnostic_frame_details:="$([[ "$ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS" == "1" ]] && printf true || printf false)" \
  > "$MISSION_LOG" 2>&1 &
MISSION_PID=$!

if [[ "$ELEVATOR_TEST_RECORD_DRIFT" == "1" ]]; then
  bag_topics=(
    /danger_search/localization/odom
    /danger_search/localization/health
    /cmd_vel
    /trunk_imu
    /a1_gazebo/joint_states
    /rosout
  )
  if [[ "$ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH" == "1" ]]; then
    bag_topics+=(/ground_truth/base_w)
  fi
  if [[ "$ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS" == "1" ]]; then
    bag_topics+=(/scan)
  fi
  setsid rosbag record --buffsize=64 \
    -O "$LOG_DIR/elevator_diagnostic.bag" \
    "${bag_topics[@]}" \
    > "$LOG_DIR/rosbag.log" 2>&1 &
  BAG_PID=$!
  log "已启动隔离诊断录包；真值仅供运行结束后离线评估"
fi

sleep 2
{
  rosnode info /danger_search 2>&1 || true
  rosnode info /scan_localization 2>&1 || true
} > "$LOG_DIR/production_node_info.txt"
if grep -Eq '/ground_truth/|/Odometry_gazebo' "$LOG_DIR/production_node_info.txt"; then
  fail "生产节点订阅审计发现禁用真值输入" 2
  exit $?
fi

started=$SECONDS
highest_floor=$((ELEVATOR_TEST_FLOOR_COUNT - 1))
registration_reported=0
change_floor_reported=0
go_to_door_stall_started=0
upward_complete=0
rgbd_recording_started=0

while (( SECONDS - started < ELEVATOR_TEST_WALL_TIMEOUT )); do
  if [[ "$ELEVATOR_TEST_RECORD_RGBD" == "1" ]] && \
     (( rgbd_recording_started == 0 )) && \
     grep -Fq "开始一次全向 RGB-D 电梯门地标扫描" "$MISSION_LOG"; then
    # 只录扫描窗口，避免把全程原始深度写入磁盘；独立 rosbag 不进入生产输入链。
    setsid rosbag record --buffsize=256 \
      -O "$LOG_DIR/elevator_rgbd.bag" \
      /real_sense/rgb/image_raw/compressed \
      /real_sense/depth/image_raw \
      /real_sense/rgb/camera_info \
      /real_sense/depth/camera_info \
      /danger_search/localization/odom \
      > "$LOG_DIR/rgbd_rosbag.log" 2>&1 &
    RGBD_BAG_PID=$!
    rgbd_recording_started=1
    log "电梯扫描已开始，启动隔离 RGB-D 诊断录包"
  fi
  if [[ "$ELEVATOR_TEST_FULL_CYCLE" == "1" ]] && \
     grep -F '"event": "mission_result_write"' "$MISSION_LOG" 2>/dev/null | \
     grep -F '"terminal_outcome": "success"' | \
     grep -Fq '"return_home_completed": true'; then
    RESULT="通过"
    if [[ "$ELEVATOR_TEST_FLOOR_COUNT" == "3" ]]; then
      DETAIL="电梯视觉注册、0→1→2 换层、2→0 返程和主入口返航闭环成功"
    else
      DETAIL="电梯视觉注册、0→1 换层、1→0 返程和主入口返航闭环成功"
    fi
    log "$DETAIL"
    exit 0
  fi
  if ! kill -0 "$MISSION_PID" 2>/dev/null; then
    fail "任务进程在专项成功前退出"
    exit $?
  fi

  if (( registration_reported == 0 )) && \
     grep -Fq "局部传感器连续确认电梯候选并注册运行期竖井几何" "$MISSION_LOG"; then
    registration_reported=1
    if [[ "$RGBD_BAG_PID" =~ ^[0-9]+$ ]]; then
      terminate_group "$RGBD_BAG_PID" INT
      wait "$RGBD_BAG_PID" 2>/dev/null || true
      RGBD_BAG_PID=""
      log "电梯注册完成，停止 RGB-D 扫描窗口录包"
    fi
    log "电梯视觉注册成功"
  fi

  if (( registration_reported == 0 )) && \
     grep -F "全向 RGB-D 电梯门地标扫描结束" "$MISSION_LOG" 2>/dev/null | \
     grep -Fq "coverage_complete=1 timed_out=1"; then
    fail "全向扫描已覆盖且超时，仍未注册电梯视觉地标"
    exit $?
  fi

  if (( change_floor_reported == 0 )) && grep -Fq "准备换层：0 -> 1" "$MISSION_LOG"; then
    change_floor_reported=1
    log "已创建 TakeElevator(0->1)"
  fi

  if grep -F '"event": "mission_state_transition"' "$MISSION_LOG" 2>/dev/null | \
     grep -F "\"current_floor\": $highest_floor" | \
     grep -F '"new_state": "EXPLORE_FLOOR"' | \
     grep -Fq '"old_state": "CHANGE_FLOOR"'; then
    if (( registration_reported == 0 )); then
      fail "换层完成但缺少电梯视觉注册证据"
      exit $?
    fi
    if [[ "$ELEVATOR_TEST_FULL_CYCLE" == "1" ]]; then
      if (( upward_complete == 0 )); then
        upward_complete=1
        if [[ "$ELEVATOR_TEST_FLOOR_COUNT" == "3" ]]; then
          log "0→1→2 换层成功，继续验证三层探索、返程电梯和主入口退出"
        else
          log "0→1 换层成功，继续验证二层探索、返程电梯和主入口退出"
        fi
      fi
    else
      RESULT="通过"
      if [[ "$ELEVATOR_TEST_FLOOR_COUNT" == "3" ]]; then
        DETAIL="电梯视觉注册与 0→1→2 换层闭环成功"
      else
        DETAIL="电梯视觉注册与 0→1 换层闭环成功"
      fi
      log "$DETAIL"
      if [[ "$ELEVATOR_TEST_KEEP_RUNNING" == "1" ]]; then
        log "GUI 保持运行；按 Ctrl-C 清理"
        while kill -0 "$MISSION_PID" 2>/dev/null; do
          sleep 2
        done
      fi
      exit 0
    fi
  fi

  if grep -Fq '"event": "mission_result_write"' "$MISSION_LOG" || \
     grep -Fq "换层任务失败" "$MISSION_LOG"; then
    fail "任务在完成所需电梯闭环前进入失败结果"
    exit $?
  fi

  latest_runtime="$(grep -F '"event": "mission_runtime"' "$MISSION_LOG" 2>/dev/null | tail -1 || true)"
  if [[ "$latest_runtime" == *'"current_task_phase": "GO_TO_ELEVATOR_DOOR"'* && \
        "$latest_runtime" == *'"path_active": false'* ]]; then
    if (( go_to_door_stall_started == 0 )); then
      go_to_door_stall_started=$SECONDS
    elif (( SECONDS - go_to_door_stall_started >= ELEVATOR_GO_TO_DOOR_STALL_WALL )); then
      fail "GO_TO_ELEVATOR_DOOR 持续无路径且零速，触发专项有界停止"
      exit $?
    fi
  else
    go_to_door_stall_started=0
  fi

  sleep 2
done

fail "超过专项墙钟总超时，尚未完成所需电梯闭环"
exit $?

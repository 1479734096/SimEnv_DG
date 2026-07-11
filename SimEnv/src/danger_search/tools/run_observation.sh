#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"

SEED=42
TIMEOUT=10000
MAX_FLOOR=1
while [ "$#" -gt 0 ]; do
  case "$1" in
    --seed)
      SEED="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT="$2"
      shift 2
      ;;
    --max-floor)
      MAX_FLOOR="$2"
      shift 2
      ;;
    *)
      echo "未知参数: $1" >&2
      exit 2
      ;;
  esac
done

if [ -z "${DISPLAY:-}" ]; then
  echo "缺少 DISPLAY，不能执行 GUI 观察测试" >&2
  exit 2
fi

export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
source /opt/ros/noetic/setup.bash
source "$SIM_DIR/devel/setup.bash"

STAMP="$(date +%Y%m%d_%H%M%S)"
BOOTSTRAP_LOG="$SIM_DIR/logs/observation_${STAMP}_seed_${SEED}.log"
RUN_PID=""
BAG_PID=""
RECORD_PID=""
RUN_DIR=""
OBS_DIR=""
ROS_RUN_ID=""
WINDOW_ID=""
MODEL_NAME=""
CAMERA_NAME=""
START_WALL=""
END_WALL=""
RUN_STATUS=1
STOP_REASON="run_seed_exit"

find_gazebo_window() {
  local client_id
  local geometry
  local height
  local properties
  local root_clients
  local width
  root_clients="$(xprop -root _NET_CLIENT_LIST 2>> "$OBS_DIR/window_probe.log" || true)"
  while read -r client_id; do
    [ -n "$client_id" ] || continue
    properties="$(
      xprop -id "$client_id" WM_CLASS _NET_WM_NAME WM_NAME 2>> "$OBS_DIR/window_probe.log" || true
    )"
    geometry="$(xwininfo -id "$client_id" 2>> "$OBS_DIR/window_probe.log" || true)"
    width="$(printf '%s\n' "$geometry" | awk '/Width:/ {print $2; exit}')"
    height="$(printf '%s\n' "$geometry" | awk '/Height:/ {print $2; exit}')"
    printf '%s id=%s width=%s height=%s %s\n' \
      "$(date --iso-8601=ns)" "$client_id" "$width" "$height" "$properties" \
      >> "$OBS_DIR/window_probe.log"
    if printf '%s\n' "$properties" | grep -q '_NET_WM_NAME(UTF8_STRING) = "Gazebo"' \
      && [ "${width:-0}" -ge 800 ] \
      && [ "${height:-0}" -ge 500 ]; then
      printf '%d\n' "$client_id"
      return 0
    fi
  done < <(printf '%s\n' "$root_clients" | grep -o '0x[0-9a-fA-F]\+')
  return 1
}

stop_process() {
  local pid="$1"
  local signal_name="$2"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill "-$signal_name" "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

stop_owned_algorithm() {
  if [ -z "$RUN_DIR" ]; then
    return
  fi
  local expected_marker="mission_summary_file:=$RUN_DIR/mission_runtime_summary.json"
  local pid
  local pgid
  local command
  while read -r pid pgid command; do
    if [[ "$command" != *"$expected_marker"* ]]; then
      continue
    fi
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pgid" 2>/dev/null || true
    fi
  done < <(ps -eo pid=,pgid=,args=)
  for _ in $(seq 1 100); do
    if ! ps -eo args= | grep -F "$expected_marker" | grep -v grep >/dev/null; then
      return
    fi
    sleep 0.1
  done
  while read -r pid pgid command; do
    if [[ "$command" == *"$expected_marker"* ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    fi
  done < <(ps -eo pid=,pgid=,args=)
}

write_manifest() {
  if [ -z "$OBS_DIR" ]; then
    return
  fi
  END_WALL="$(date --iso-8601=ns)"
  local abort_reason=""
  if [ -f "$OBS_DIR/abort_reason.txt" ]; then
    abort_reason="$(tr '\n' ' ' < "$OBS_DIR/abort_reason.txt")"
  fi
  printf '%s\n' \
    '{' \
    "  \"seed\": $SEED," \
    "  \"max_floor\": $MAX_FLOOR," \
    "  \"wall_timeout\": $TIMEOUT," \
    "  \"start_wall\": \"$START_WALL\"," \
    "  \"end_wall\": \"$END_WALL\"," \
    "  \"run_dir\": \"$RUN_DIR\"," \
    "  \"ros_run_id\": \"$ROS_RUN_ID\"," \
    "  \"gazebo_window_id\": \"$WINDOW_ID\"," \
    "  \"model_name\": \"$MODEL_NAME\"," \
    "  \"camera_name\": \"$CAMERA_NAME\"," \
    "  \"stop_reason\": \"$STOP_REASON\"," \
    "  \"abort_reason\": \"$abort_reason\"," \
    "  \"run_status\": $RUN_STATUS," \
    '  "path_followed_semantics": "collinear_compression_only"' \
    '}' > "$OBS_DIR/manifest.json"
}

cleanup() {
  trap - EXIT INT TERM
  if [ -n "$RUN_PID" ] && kill -0 "$RUN_PID" 2>/dev/null; then
    kill -INT "$RUN_PID" 2>/dev/null || true
    wait "$RUN_PID" 2>/dev/null || true
  fi
  stop_owned_algorithm
  stop_process "$BAG_PID" INT
  stop_process "$RECORD_PID" INT
  if [ -n "$OBS_DIR" ] && [ -f "$BOOTSTRAP_LOG" ]; then
    cp "$BOOTSTRAP_LOG" "$OBS_DIR/run_seed.log"
  fi
  write_manifest
}

handle_signal() {
  STOP_REASON="operator_interrupt_$1"
  RUN_STATUS=130
  exit 130
}

trap cleanup EXIT
trap 'handle_signal INT' INT
trap 'handle_signal TERM' TERM

cd "$SIM_DIR"
(
  trap - INT TERM
  exec env PYTHONUNBUFFERED=1 python3 src/danger_search/tools/run_seed.py \
    --seed "$SEED" \
    --gui \
    --timeout "$TIMEOUT" \
    --max-floor "$MAX_FLOOR"
) > "$BOOTSTRAP_LOG" 2>&1 &
RUN_PID=$!

for _ in $(seq 1 240); do
  RUN_DIR="$(sed -n 's/^运行归档目录: //p' "$BOOTSTRAP_LOG" | head -n 1)"
  if [ -n "$RUN_DIR" ]; then
    break
  fi
  if ! kill -0 "$RUN_PID" 2>/dev/null; then
    echo "run_seed 在创建 run 目录前退出" >&2
    exit 1
  fi
  sleep 1
done
if [ -z "$RUN_DIR" ]; then
  echo "等待 run 目录超时" >&2
  exit 1
fi

OBS_DIR="$RUN_DIR/observation"
mkdir -p "$OBS_DIR"
cp "$BOOTSTRAP_LOG" "$OBS_DIR/run_seed.log"

for _ in $(seq 1 240); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$RUN_PID" 2>/dev/null; then
    echo "run_seed 在 ROS master 就绪前退出" >&2
    exit 1
  fi
  sleep 1
done
if ! rosparam list >/dev/null 2>&1; then
  echo "等待 ROS master 超时" >&2
  exit 1
fi
ROS_RUN_ID="$(rosparam get /run_id 2>/dev/null || true)"

for _ in $(seq 1 60); do
  WINDOW_ID="$(find_gazebo_window || true)"
  if [ -n "$WINDOW_ID" ]; then
    break
  fi
  if ! kill -0 "$RUN_PID" 2>/dev/null; then
    echo "run_seed 在 Gazebo 窗口出现前退出" >&2
    exit 1
  fi
  sleep 1
done
if [ -z "$WINDOW_ID" ]; then
  echo "没有找到 Gazebo GUI 窗口，拒绝生成无视频观察结果" >&2
  exit 1
fi
xwininfo -id "$WINDOW_ID" > "$OBS_DIR/gazebo_window_geometry.txt"
xprop -id "$WINDOW_ID" WM_CLASS _NET_WM_NAME WM_NAME > "$OBS_DIR/gazebo_window_properties.txt"

MODEL_NAME="${GAZEBO_TRACK_MODEL:-a1_gazebo}"
for _ in $(seq 1 60); do
  if gz model -m "$MODEL_NAME" -p > "$OBS_DIR/gazebo_model_check.txt" 2>&1; then
    model_ready=1
  else
    model_ready=0
  fi
  gz camera -l > "$OBS_DIR/gazebo_cameras.txt" 2>&1 || true
  CAMERA_NAME="$(grep -m1 '^gzclient_camera$' "$OBS_DIR/gazebo_cameras.txt" || true)"
  if [ -z "$CAMERA_NAME" ]; then
    CAMERA_NAME="$(grep -m1 '^user_camera$' "$OBS_DIR/gazebo_cameras.txt" || true)"
  fi
  if [ "$model_ready" = "1" ] && [ -n "$CAMERA_NAME" ]; then
    break
  fi
  sleep 1
done
if [ "${model_ready:-0}" != "1" ] || [ -z "$CAMERA_NAME" ]; then
  echo "无法确认 Gazebo 模型或 user camera，拒绝开始无跟踪录屏" >&2
  exit 1
fi
if ! gz camera -c "$CAMERA_NAME" -f "$MODEL_NAME" > "$OBS_DIR/gazebo_follow.log" 2>&1; then
  echo "Gazebo 相机跟踪命令失败" >&2
  exit 1
fi

if ! gst-launch-1.0 -q \
  ximagesrc display-name="$DISPLAY" xid="$WINDOW_ID" remote=true use-damage=false show-pointer=false num-buffers=1 \
  ! videoconvert \
  ! pngenc snapshot=true \
  ! filesink location="$OBS_DIR/gazebo_start.png" \
  > "$OBS_DIR/screenshot.log" 2>&1; then
  echo "Gazebo 窗口快照失败，拒绝开始无可视证据测试" >&2
  exit 1
fi

START_WALL="$(date --iso-8601=ns)"
(
  trap - INT TERM
  exec gst-launch-1.0 -e \
    ximagesrc display-name="$DISPLAY" xid="$WINDOW_ID" remote=true use-damage=false show-pointer=false do-timestamp=true \
    ! videoconvert \
    ! videorate \
    ! video/x-raw,framerate=10/1 \
    ! queue \
    ! vp8enc deadline=1 cpu-used=8 threads=4 target-bitrate=2500000 keyframe-max-dist=50 \
    ! matroskamux \
    ! filesink location="$OBS_DIR/gazebo.mkv"
) > "$OBS_DIR/recording.log" 2>&1 &
RECORD_PID=$!

(
  trap - INT TERM
  exec rosbag record --buffsize=1024 -O "$OBS_DIR/telemetry.bag" \
    /clock \
    /cmd_vel \
    /Odometry_gazebo \
    /danger_search/status \
    /nav/map_debug \
    /danger_search/path_raw \
    /danger_search/path_followed \
    /danger_search/local_target \
    /junior_ctrl/rl_status \
    /gazebo/performance_metrics \
    /rosout_agg
) > "$OBS_DIR/rosbag.log" 2>&1 &
BAG_PID=$!

sleep 2
if ! kill -0 "$RECORD_PID" 2>/dev/null || ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "录屏或 rosbag 进程未保持运行，拒绝继续测试" >&2
  exit 1
fi
if [ ! -s "$OBS_DIR/gazebo.mkv" ] || [ ! -f "$OBS_DIR/telemetry.bag.active" ]; then
  echo "录屏或 rosbag 文件没有正常产生，拒绝继续测试" >&2
  exit 1
fi

printf '%s\n' "$START_WALL" > "$OBS_DIR/start_wall.txt"
printf '%s\n' "ready" > "$OBS_DIR/capture_ready"
echo "观察采集已启动"
echo "run_dir=$RUN_DIR"
echo "observation_dir=$OBS_DIR"
echo "model=$MODEL_NAME camera=$CAMERA_NAME window=$WINDOW_ID"

set +e
wait "$RUN_PID"
RUN_STATUS=$?
set -e
RUN_PID=""
exit "$RUN_STATUS"

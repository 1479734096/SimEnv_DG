#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"

SEED=42
TIMEOUT=10000
MAX_FLOOR=1
PARAMS=""
STOP_AFTER_EXIT=0
DIAGNOSTIC_RATE=2.0
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
    --params)
      PARAMS="$2"
      shift 2
      ;;
    --stop-after-exit)
      STOP_AFTER_EXIT=1
      shift
      ;;
    --diagnostic-rate)
      DIAGNOSTIC_RATE="$2"
      shift 2
      ;;
    *)
      echo "未知参数: $1" >&2
      exit 2
      ;;
  esac
done

if [ -n "$PARAMS" ]; then
  if [ ! -f "$PARAMS" ]; then
    echo "参数覆盖文件不存在: $PARAMS" >&2
    exit 2
  fi
  PARAMS="$(realpath "$PARAMS")"
fi

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
DIAGNOSTIC_PID=""
RUN_DIR=""
OBS_DIR=""
EXIT_MARKER=""
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
    for _ in $(seq 1 100); do
      if ! kill -0 "$pid" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
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

archive_scene_evidence() {
  local archive_dir="$OBS_DIR/offline_scene"
  local source
  mkdir -p "$archive_dir"
  for source in \
    scene_manifest.json \
    scene_manifest.stdout.json \
    generation_checks.json \
    building_config.json \
    layout_metadata.json \
    door_config.yaml \
    elevator_config.yaml \
    team_scene_info.json; do
    if [ -f "$SIM_DIR/generated_building/$source" ]; then
      cp "$SIM_DIR/generated_building/$source" "$archive_dir/$source"
    fi
  done
  (
    cd "$archive_dir"
    sha256sum ./* > sha256sums.txt 2>/dev/null || true
  )
  git -C "$SIM_DIR" rev-parse HEAD > "$OBS_DIR/source_head.txt"
  git -C "$SIM_DIR" status --short > "$OBS_DIR/source_status.txt"
  if [ -f "$SIM_DIR/../SimEnv初始.zip" ]; then
    sha256sum "$SIM_DIR/../SimEnv初始.zip" > "$OBS_DIR/initial_archive_sha256.txt"
  fi
  cat > "$archive_dir/USAGE_BOUNDARY.txt" <<'EOF'
本目录仅用于离线复现、场景覆盖分析和诊断归档。
layout_metadata、scene_manifest、building_config、door/elevator 配置不得由生产算法读取或影响决策。
若存在 team_scene_info.json，是否可作运行时输入必须以赛方正式接口文档为准。
危险源真值不由本观察脚本归档；run_seed 只会在算法退出后的离线评估阶段读取真值。
EOF
}

wait_for_current_scene() {
  local manifest="$SIM_DIR/generated_building/scene_manifest.json"
  for _ in $(seq 1 240); do
    if [ -f "$manifest" ] \
      && [ "$manifest" -nt "$BOOTSTRAP_LOG" ] \
      && python3 -c \
        'import json,sys; raise SystemExit(0 if int(json.load(open(sys.argv[1], encoding="utf-8")).get("seed", -1)) == int(sys.argv[2]) else 1)' \
        "$manifest" "$SEED" 2>/dev/null; then
      return 0
    fi
    if ! kill -0 "$RUN_PID" 2>/dev/null; then
      echo "run_seed 在生成当前场景前退出" >&2
      return 1
    fi
    sleep 1
  done
  echo "等待当前 seed 的 scene manifest 超时" >&2
  return 1
}

write_manifest() {
  if [ -z "$OBS_DIR" ]; then
    return
  fi
  END_WALL="$(date --iso-8601=ns)"
  local abort_reason=""
  local abort_reason_json='""'
  local params_json="null"
  if [ -n "$PARAMS" ]; then
    params_json="$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$PARAMS")"
  fi
  if [ -f "$OBS_DIR/abort_reason.txt" ]; then
    abort_reason="$(tr '\n' ' ' < "$OBS_DIR/abort_reason.txt")"
    abort_reason_json="$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$abort_reason")"
  fi
  printf '%s\n' \
    '{' \
    "  \"seed\": $SEED," \
    "  \"max_floor\": $MAX_FLOOR," \
    "  \"wall_timeout\": $TIMEOUT," \
    "  \"params\": $params_json," \
    "  \"stop_after_exit\": $STOP_AFTER_EXIT," \
    "  \"diagnostic_rate_hz\": $DIAGNOSTIC_RATE," \
    "  \"start_wall\": \"$START_WALL\"," \
    "  \"end_wall\": \"$END_WALL\"," \
    "  \"run_dir\": \"$RUN_DIR\"," \
    "  \"ros_run_id\": \"$ROS_RUN_ID\"," \
    "  \"gazebo_window_id\": \"$WINDOW_ID\"," \
    "  \"model_name\": \"$MODEL_NAME\"," \
    "  \"camera_name\": \"$CAMERA_NAME\"," \
    "  \"stop_reason\": \"$STOP_REASON\"," \
    "  \"abort_reason\": $abort_reason_json," \
    "  \"run_status\": $RUN_STATUS," \
    '  "path_followed_semantics": "collinear_compression_only"' \
    '}' > "$OBS_DIR/manifest.json"
}

cleanup() {
  trap - EXIT INT TERM
  set +e
  if [ -n "$OBS_DIR" ]; then
    printf '%s cleanup_start run_pid=%s diagnostic_pid=%s bag_pid=%s record_pid=%s\n' \
      "$(date --iso-8601=ns)" "$RUN_PID" "$DIAGNOSTIC_PID" "$BAG_PID" "$RECORD_PID" \
      >> "$OBS_DIR/cleanup.log"
    # 先落盘运行终止语义；即使后续某个外部进程拒绝退出，归档也不会完全丢失。
    write_manifest
  fi
  stop_process "$DIAGNOSTIC_PID" INT
  DIAGNOSTIC_PID=""
  stop_process "$BAG_PID" INT
  BAG_PID=""
  stop_process "$RECORD_PID" INT
  RECORD_PID=""
  if [ -n "$RUN_PID" ] && kill -0 "$RUN_PID" 2>/dev/null; then
    kill -INT "$RUN_PID" 2>/dev/null || true
    wait "$RUN_PID" 2>/dev/null || true
  fi
  stop_owned_algorithm
  if [ -n "$OBS_DIR" ] && [ -f "$BOOTSTRAP_LOG" ]; then
    cp "$BOOTSTRAP_LOG" "$OBS_DIR/run_seed.log"
  fi
  write_manifest
  if [ -n "$OBS_DIR" ]; then
    printf '%s cleanup_end\n' "$(date --iso-8601=ns)" >> "$OBS_DIR/cleanup.log"
  fi
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
PARAM_ARGS=()
if [ -n "$PARAMS" ]; then
  PARAM_ARGS=(--params "$PARAMS")
fi
(
  trap - INT TERM
  exec env PYTHONUNBUFFERED=1 python3 src/danger_search/tools/run_seed.py \
    --seed "$SEED" \
    --gui \
    --timeout "$TIMEOUT" \
    --max-floor "$MAX_FLOOR" \
    "${PARAM_ARGS[@]}"
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
wait_for_current_scene
archive_scene_evidence
EXIT_MARKER="$OBS_DIR/exit_cabin_outcome.json"
(
  trap - INT TERM
  exec python3 "$SCRIPT_DIR/capture_exit_cabin_diagnostics.py" \
    --output "$OBS_DIR/exit_cabin_diagnostics.jsonl" \
    --summary "$OBS_DIR/exit_cabin_diagnostics_summary.json" \
    --stop-marker "$EXIT_MARKER" \
    --rate "$DIAGNOSTIC_RATE"
) > "$OBS_DIR/exit_cabin_recorder.log" 2>&1 &
DIAGNOSTIC_PID=$!
sleep 2
if ! kill -0 "$DIAGNOSTIC_PID" 2>/dev/null; then
  echo "EXIT_CABIN 诊断采集器启动失败" >&2
  exit 1
fi

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
while kill -0 "$RUN_PID" 2>/dev/null; do
  if [ "$STOP_AFTER_EXIT" = "1" ] && [ -s "$EXIT_MARKER" ]; then
    STOP_REASON="exit_cabin_outcome"
    cp "$EXIT_MARKER" "$OBS_DIR/abort_reason.txt"
    stop_process "$DIAGNOSTIC_PID" INT
    DIAGNOSTIC_PID=""
    kill -INT "$RUN_PID" 2>/dev/null || true
    break
  fi
  sleep 1
done
wait "$RUN_PID"
RUN_STATUS=$?
set -e
RUN_PID=""
exit "$RUN_STATUS"

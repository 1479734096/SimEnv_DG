#!/usr/bin/env bash
# 串行运行预登记的三层完整闭环 seed 矩阵（底层调用 run_elevator_gui_test.sh），并在每轮结束后归档离线证据。

set -Eeuo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNNER="$WORKSPACE_ROOT/tools/sim/run_elevator_gui_test.sh"
TIMING_ANALYZER="$WORKSPACE_ROOT/tools/offline/analyze_mission_timing.py"
MATRIX_SUMMARIZER="$WORKSPACE_ROOT/tools/offline/summarize_three_floor_matrix.py"
EVALUATOR="$WORKSPACE_ROOT/src/building_obstacles/scripts/evaluate_danger.py"
OUTPUT_ROOT="${THREE_FLOOR_MATRIX_OUTPUT_ROOT:-/tmp/three_floor_formal_matrix}"
MAX_INFRASTRUCTURE_ATTEMPTS=2
MATRIX_INTERRUPTED=0
RUN_ACTIVE=0

handle_signal() {
  MATRIX_INTERRUPTED=1
  if [[ "$RUN_ACTIVE" != "1" ]]; then
    exit 130
  fi
}

usage() {
  cat <<'EOF'
用法：
  THREE_FLOOR_MATRIX_OUTPUT_ROOT=/tmp/three_floor_20260729 \
    tools/sim/run_three_floor_seed_matrix.sh SEED [SEED ...]

示例：
  THREE_FLOOR_MATRIX_OUTPUT_ROOT=/tmp/three_floor_20260729 \
    tools/sim/run_three_floor_seed_matrix.sh 8127 6946 1612

固定条件：
  - 2～20 个互不重复、运行前显式给出的 seed；
  - GUI=true，3 层，280 s 单层预算，60 s 最短探索，840 s 总仿真预算；
  - 9000 s 墙钟上限，1200 s 厅外导航墙钟停滞上限；
  - 完整 0→1→2→0→home 闭环，正式 adaptive_shared_map 后端；
  - 正式运行关闭诊断录包和真值话题，任务结束后才归档并读取裁判真值评分。

执行语义：
  - 算法成功或失败都保留为该 seed 的正式结果，然后继续下一个 seed；
  - 基础设施失败最多同配置尝试两次，仍失败则暂停整个矩阵；
  - Ctrl-C 立即中止；再次使用相同目录、HEAD 和 seed 列表可从断点继续；
  - 已完成的算法结果不会重跑，防止用重复运行掩盖失败。
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
if (( $# < 2 || $# > 20 )); then
  printf '必须预登记 2～20 个 seed；使用 --help 查看示例\n' >&2
  exit 2
fi

SEEDS=("$@")
declare -A SEEN_SEEDS=()
for seed in "${SEEDS[@]}"; do
  if [[ ! "$seed" =~ ^[0-9]+$ ]]; then
    printf 'seed 必须是非负整数：%s\n' "$seed" >&2
    exit 2
  fi
  if [[ -n "${SEEN_SEEDS[$seed]:-}" ]]; then
    printf 'seed 不得重复：%s\n' "$seed" >&2
    exit 2
  fi
  SEEN_SEEDS[$seed]=1
done

for path in "$RUNNER" "$TIMING_ANALYZER" "$MATRIX_SUMMARIZER" "$EVALUATOR"; do
  if [[ ! -f "$path" ]]; then
    printf '缺少矩阵依赖：%s\n' "$path" >&2
    exit 2
  fi
done
if [[ ! -f "$WORKSPACE_ROOT/devel/setup.bash" ]]; then
  printf '工作区尚未构建，缺少：%s\n' "$WORKSPACE_ROOT/devel/setup.bash" >&2
  exit 2
fi
if [[ -n "$(git -C "$WORKSPACE_ROOT" status --porcelain --untracked-files=no)" ]]; then
  printf '正式矩阵要求已跟踪文件工作树干净，请先提交或明确整理改动\n' >&2
  exit 2
fi

HEAD_COMMIT="$(git -C "$WORKSPACE_ROOT" rev-parse HEAD)"
mkdir -p "$OUTPUT_ROOT"
CONFIG_PATH="$OUTPUT_ROOT/MATRIX_CONFIG.txt"
if [[ ! -f "$CONFIG_PATH" && \
      -n "$(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null || true)" ]]; then
  printf '新矩阵证据目录必须为空，拒绝接管已有内容：%s\n' "$OUTPUT_ROOT" >&2
  exit 2
fi
TEMPORARY_CONFIG="$(mktemp /tmp/three_floor_matrix_config.XXXXXX)"
trap 'rm -f -- "$TEMPORARY_CONFIG"' EXIT
trap handle_signal INT TERM
{
  printf 'head=%s\n' "$HEAD_COMMIT"
  printf 'seeds=%s\n' "${SEEDS[*]}"
  printf 'gui=true\n'
  printf 'floor_count=3\n'
  printf 'floor_timeout=280\n'
  printf 'min_floor_explore_time=60\n'
  printf 'max_runtime=840\n'
  printf 'wall_timeout=9000\n'
  printf 'go_to_door_stall_wall=1200\n'
  printf 'full_cycle=1\n'
  printf 'localization_backend=adaptive_shared_map\n'
  printf 'record_drift=0\n'
  printf 'record_rgbd=0\n'
  printf 'offline_truth=0\n'
  printf 'localization_diagnostics=0\n'
  printf 'landmark_drift_diagnostics=0\n'
  printf 'maximum_infrastructure_attempts=%s\n' "$MAX_INFRASTRUCTURE_ATTEMPTS"
} > "$TEMPORARY_CONFIG"
if [[ -f "$CONFIG_PATH" ]] && ! cmp -s "$TEMPORARY_CONFIG" "$CONFIG_PATH"; then
  printf '已有矩阵配置与当前 HEAD、seed 或预算不一致，拒绝混合证据：%s\n' \
    "$CONFIG_PATH" >&2
  exit 2
fi
if [[ ! -f "$CONFIG_PATH" ]]; then
  cp "$TEMPORARY_CONFIG" "$CONFIG_PATH"
  date --iso-8601=seconds > "$OUTPUT_ROOT/MATRIX_STARTED_AT.txt"
  {
    printf '# 三层闭环矩阵人工观察\n\n'
    printf '只记录画面中可直接观察的现象；算法结论以同轮日志和结构化摘要为准。\n'
    for seed in "${SEEDS[@]}"; do
      printf '\n## seed %s\n\n- 观察：\n- 异常位置/阶段：\n- 是否需要赛后复核：\n' "$seed"
    done
  } > "$OUTPUT_ROOT/OPERATOR_NOTES.md"
fi

summarize() {
  python3 "$MATRIX_SUMMARIZER" "$OUTPUT_ROOT" --seeds "${SEEDS[@]}" \
    > "$OUTPUT_ROOT/matrix_summary.log"
}

archive_if_newer() {
  local source_path="$1"
  local destination_path="$2"
  local marker_path="$3"
  if [[ -f "$source_path" && "$source_path" -nt "$marker_path" ]]; then
    cp "$source_path" "$destination_path"
  fi
}

write_timing_summary() {
  local run_dir="$1"
  local arguments=(
    "$TIMING_ANALYZER"
    "$run_dir/mission.log"
    --auto-log "$run_dir/auto.log"
    --test-summary "$run_dir/TEST_SUMMARY.txt"
  )
  if [[ -f "$run_dir/detected_danger.json" ]]; then
    arguments+=(--result-file "$run_dir/detected_danger.json")
  fi
  if [[ -f "$run_dir/evaluation_result.json" ]]; then
    arguments+=(--evaluation-file "$run_dir/evaluation_result.json")
  fi
  if ! python3 "${arguments[@]}" > "$run_dir/mission_timing.json" 2> "$run_dir/mission_timing.err"; then
    rm -f -- "$run_dir/mission_timing.json"
  elif [[ ! -s "$run_dir/mission_timing.err" ]]; then
    rm -f -- "$run_dir/mission_timing.err"
  fi
}

classify_attempt() {
  local runner_status="$1"
  local run_dir="$2"
  if [[ "$runner_status" == "0" ]]; then
    printf 'success\n'
    return
  fi
  if [[ "$runner_status" == "130" ]]; then
    printf 'interrupted\n'
    return
  fi
  if [[ -f "$run_dir/mission_timing.json" ]]; then
    local terminal_outcome
    terminal_outcome="$(python3 - "$run_dir/mission_timing.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    payload = json.load(stream)
print(payload.get("run_outcome", {}).get("terminal_outcome", "unknown"))
PY
)"
    if [[ "$terminal_outcome" == "failed" ]]; then
      printf 'algorithm_failure\n'
      return
    fi
  fi
  printf 'infrastructure_failure\n'
}

for index in "${!SEEDS[@]}"; do
  seed="${SEEDS[$index]}"
  seed_root="$OUTPUT_ROOT/seed_$seed"
  mkdir -p "$seed_root"
  finalized=""
  highest_attempt=0
  infrastructure_attempts=0
  for attempt_dir in "$seed_root"/attempt_*; do
    [[ -d "$attempt_dir" ]] || continue
    attempt_name="${attempt_dir##*/attempt_}"
    if [[ "$attempt_name" =~ ^[0-9]+$ ]] && (( attempt_name > highest_attempt )); then
      highest_attempt="$attempt_name"
    fi
    attempt_class="$(<"$attempt_dir/ATTEMPT_CLASS.txt" 2>/dev/null || true)"
    case "$attempt_class" in
      success|algorithm_failure) finalized="$attempt_dir" ;;
      infrastructure_failure) infrastructure_attempts=$((infrastructure_attempts + 1)) ;;
    esac
  done
  if [[ -n "$finalized" ]]; then
    printf '[三层矩阵] %d/%d seed=%s 已有正式结果，跳过：%s\n' \
      "$((index + 1))" "${#SEEDS[@]}" "$seed" "$finalized"
    continue
  fi
  if (( infrastructure_attempts >= MAX_INFRASTRUCTURE_ATTEMPTS )); then
    printf '[三层矩阵] seed=%s 已连续两次基础设施失败，矩阵保持暂停\n' "$seed" >&2
    summarize
    exit 2
  fi

  attempt=$((highest_attempt + 1))
  while (( infrastructure_attempts < MAX_INFRASTRUCTURE_ATTEMPTS )); do
    run_dir="$seed_root/attempt_$attempt"
    console_log="$OUTPUT_ROOT/seed_${seed}_attempt_${attempt}.console.log"
    marker="$OUTPUT_ROOT/.seed_${seed}_attempt_${attempt}_started"
    touch "$marker"
    started_at="$(date --iso-8601=seconds)"
    started_monotonic="$SECONDS"
    printf '[三层矩阵] 开始 %d/%d seed=%s attempt=%s，证据=%s\n' \
      "$((index + 1))" "${#SEEDS[@]}" "$seed" "$attempt" "$run_dir"

    # 显式覆盖底层 runner 参数：使用三层矩阵紧凑预算（280s/840s/60s探索），并显式关闭耗时的高负荷诊断与 RGB-D 录包
    set +e
    RUN_ACTIVE=1
    env \
      SEED="$seed" \
      GUI=true \
      ELEVATOR_TEST_FLOOR_COUNT=3 \
      ELEVATOR_TEST_FLOOR_TIMEOUT=280 \
      ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60 \
      ELEVATOR_TEST_MAX_RUNTIME=840 \
      ELEVATOR_TEST_WALL_TIMEOUT=9000 \
      ELEVATOR_GO_TO_DOOR_STALL_WALL=1200 \
      ELEVATOR_TEST_FULL_CYCLE=1 \
      ELEVATOR_TEST_RECORD_DRIFT=0 \
      ELEVATOR_TEST_RECORD_RGBD=0 \
      ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=0 \
      ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=0 \
      ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS=0 \
      ELEVATOR_TEST_LOCALIZATION_BACKEND=adaptive_shared_map \
      ELEVATOR_TEST_LOG_DIR="$run_dir" \
      "$RUNNER" 2>&1 | tee "$console_log"
    runner_status=${PIPESTATUS[0]}
    RUN_ACTIVE=0
    if [[ "$MATRIX_INTERRUPTED" == "1" ]]; then
      runner_status=130
    fi
    set -e

    mkdir -p "$run_dir"
    printf '%s\n' "$runner_status" > "$run_dir/RUNNER_EXIT_CODE.txt"
    archive_if_newer "$WORKSPACE_ROOT/results/detected_danger.json" \
      "$run_dir/detected_danger.json" "$marker"
    archive_if_newer "$WORKSPACE_ROOT/results/danger_truth.json" \
      "$run_dir/danger_truth.json" "$marker"
    if [[ -f "$run_dir/detected_danger.json" && -f "$run_dir/danger_truth.json" ]]; then
      if ! python3 "$EVALUATOR" \
        --truth-file "$run_dir/danger_truth.json" \
        --detected-file "$run_dir/detected_danger.json" \
        --output-file "$run_dir/evaluation_result.json" \
        > "$run_dir/evaluation.log" 2>&1; then
        printf '离线评分失败，详见 evaluation.log\n' > "$run_dir/EVALUATION_ERROR.txt"
      fi
    fi
    if compgen -G "$run_dir/*.json" >/dev/null; then
      sha256sum "$run_dir"/*.json > "$run_dir/result_files.sha256"
    fi
    write_timing_summary "$run_dir"
    attempt_class="$(classify_attempt "$runner_status" "$run_dir")"
    printf '%s\n' "$attempt_class" > "$run_dir/ATTEMPT_CLASS.txt"
    {
      printf 'seed=%s\n' "$seed"
      printf 'attempt=%s\n' "$attempt"
      printf 'head=%s\n' "$HEAD_COMMIT"
      printf 'started_at=%s\n' "$started_at"
      printf 'finished_at=%s\n' "$(date --iso-8601=seconds)"
      printf 'wall_elapsed_seconds=%s\n' "$((SECONDS - started_monotonic))"
      printf 'runner_exit_code=%s\n' "$runner_status"
      printf 'attempt_class=%s\n' "$attempt_class"
    } > "$run_dir/RUN_METADATA.txt"
    rm -f -- "$marker"
    summarize

    case "$attempt_class" in
      success)
        printf '[三层矩阵] seed=%s 完成闭环，继续下一 seed\n' "$seed"
        break
        ;;
      algorithm_failure)
        printf '[三层矩阵] seed=%s 记录为算法失败，不重跑，继续下一 seed\n' "$seed"
        break
        ;;
      interrupted)
        printf '[三层矩阵] seed=%s 被中止，矩阵暂停\n' "$seed" >&2
        exit 130
        ;;
      infrastructure_failure)
        infrastructure_attempts=$((infrastructure_attempts + 1))
        if (( infrastructure_attempts >= MAX_INFRASTRUCTURE_ATTEMPTS )); then
          printf '[三层矩阵] seed=%s 连续两次基础设施失败，矩阵暂停\n' "$seed" >&2
          exit 2
        fi
        printf '[三层矩阵] seed=%s 基础设施失败，同配置仅重试一次\n' "$seed" >&2
        attempt=$((attempt + 1))
        ;;
    esac
  done
done

summarize
date --iso-8601=seconds > "$OUTPUT_ROOT/MATRIX_FINISHED_AT.txt"
printf '[三层矩阵] 全部预登记 seed 已运行；摘要：%s\n' \
  "$OUTPUT_ROOT/matrix_summary.md"

#!/usr/bin/env bash
# 串行运行预登记定位 seed 矩阵（底层调用 run_elevator_gui_test.sh）；生产算法不读取本脚本生成的离线证据。

set -Eeuo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNNER="$WORKSPACE_ROOT/tools/sim/run_elevator_gui_test.sh"
SUMMARIZER="$WORKSPACE_ROOT/tools/offline/summarize_localization_matrix.py"
BACKEND="${LOCALIZATION_MATRIX_BACKEND:-adaptive_shared_map}"
OUTPUT_ROOT="${LOCALIZATION_MATRIX_OUTPUT_ROOT:-/tmp/stage17_adaptive_shared_map_matrix}"
SEEDS=(8127 1612 8072 6946 4962 42 137 911 2027 3141 4099 5273 6203 7331 8459 9511 10657 11777 12907 14029)

usage() {
  cat <<'EOF'
用法：
  tools/sim/run_localization_seed_matrix.sh

环境变量：
  LOCALIZATION_MATRIX_BACKEND=adaptive_shared_map
  LOCALIZATION_MATRIX_OUTPUT_ROOT=/tmp/stage17_adaptive_shared_map_matrix

矩阵固定为 20 个预登记 seed、2 层、33 秒单层预算、5 秒最小探索、
180 秒任务预算、完整返程、离线真值和逐帧定位诊断。脚本串行运行，
已存在且配置匹配的有效证据会跳过；无效证据最多保留两次尝试。
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  printf '不支持位置参数；使用 --help 查看固定矩阵配置\n' >&2
  exit 2
fi
if [[ "$BACKEND" != "adaptive_shared_map" ]]; then
  printf 'Stage17 矩阵只允许 adaptive_shared_map，实际为：%s\n' "$BACKEND" >&2
  exit 2
fi
for path in "$RUNNER" "$SUMMARIZER"; do
  if [[ ! -f "$path" ]]; then
    printf '缺少矩阵依赖：%s\n' "$path" >&2
    exit 2
  fi
done

mkdir -p "$OUTPUT_ROOT"
config_path="$OUTPUT_ROOT/MATRIX_CONFIG.txt"
temporary_config="$(mktemp /tmp/stage17_matrix_config.XXXXXX)"
trap 'rm -f -- "$temporary_config"' EXIT
{
  printf 'backend=%s\n' "$BACKEND"
  printf 'seeds=%s\n' "${SEEDS[*]}"
  printf 'gui=false\n'
  printf 'floor_count=2\n'
  printf 'floor_timeout=33\n'
  printf 'min_floor_explore_time=5\n'
  printf 'max_runtime=180\n'
  printf 'wall_timeout=1800\n'
  printf 'full_cycle=1\n'
  printf 'record_drift=1\n'
  printf 'record_rgbd=0\n'
  printf 'offline_truth=1\n'
  printf 'localization_diagnostics=1\n'
} > "$temporary_config"
if [[ -f "$config_path" ]] && ! cmp -s "$temporary_config" "$config_path"; then
  printf '已有矩阵配置不一致，拒绝混合证据：%s\n' "$config_path" >&2
  exit 2
fi
if [[ ! -f "$config_path" ]]; then
  cp "$temporary_config" "$config_path"
fi

summarize() {
  python3 "$SUMMARIZER" "$OUTPUT_ROOT" --backend "$BACKEND" \
    --seeds "${SEEDS[@]}" > "$OUTPUT_ROOT/matrix_summary.log"
}

for index in "${!SEEDS[@]}"; do
  seed="${SEEDS[$index]}"
  seed_dir="$OUTPUT_ROOT/seed_$seed"
  mkdir -p "$seed_dir"
  valid_attempt=""
  for attempt_dir in "$seed_dir"/attempt_*; do
    [[ -d "$attempt_dir" ]] || continue
    if python3 "$SUMMARIZER" --validate-run "$attempt_dir" \
      --seed "$seed" --backend "$BACKEND" >/dev/null 2>&1; then
      valid_attempt="$attempt_dir"
    fi
  done
  if [[ -n "$valid_attempt" ]]; then
    printf '[定位矩阵] seed=%s 已有有效证据，跳过：%s\n' "$seed" "$valid_attempt"
    continue
  fi

  existing_attempts=0
  highest_attempt=0
  for attempt_dir in "$seed_dir"/attempt_*; do
    [[ -d "$attempt_dir" ]] || continue
    attempt_name="${attempt_dir##*/attempt_}"
    if [[ "$attempt_name" =~ ^[0-9]+$ ]] && (( attempt_name > highest_attempt )); then
      highest_attempt="$attempt_name"
    fi
    # 用户中断和全局前置失败没有产生算法证据，不得消耗同 seed 的两次运行额度。
    if [[ -f "$attempt_dir/RUNNER_EXIT_CODE.txt" ]]; then
      prior_status="$(<"$attempt_dir/RUNNER_EXIT_CODE.txt")"
      if [[ "$prior_status" != "130" && "$prior_status" != "2" ]]; then
        existing_attempts=$((existing_attempts + 1))
      fi
    fi
  done
  if (( existing_attempts >= 2 )); then
    printf '[定位矩阵] seed=%s 已有两次无效证据，保留阻断并继续\n' "$seed"
    summarize
    continue
  fi

  attempt=$((highest_attempt + 1))
  while (( existing_attempts < 2 )); do
    attempt_dir="$seed_dir/attempt_$attempt"
    mkdir -p "$attempt_dir"
    printf '[定位矩阵] 开始 %d/20 seed=%s attempt=%s\n' "$((index + 1))" "$seed" "$attempt"
    # 显式传递定位评估专用参数：无GUI、2层极短驻留（33s单层/5s探索/180s总预算）、记录漂移与离线真值、关闭大容量RGB-D
    set +e
    env \
      SEED="$seed" \
      GUI=false \
      ELEVATOR_TEST_FLOOR_COUNT=2 \
      ELEVATOR_TEST_FLOOR_TIMEOUT=33 \
      ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5 \
      ELEVATOR_TEST_MAX_RUNTIME=180 \
      ELEVATOR_TEST_WALL_TIMEOUT=1800 \
      ELEVATOR_TEST_FULL_CYCLE=1 \
      ELEVATOR_TEST_RECORD_DRIFT=1 \
      ELEVATOR_TEST_RECORD_RGBD=0 \
      ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 \
      ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1 \
      ELEVATOR_TEST_LOCALIZATION_BACKEND="$BACKEND" \
      ELEVATOR_TEST_LOG_DIR="$attempt_dir" \
      "$RUNNER"
    runner_status=$?
    set -e
    printf '%s\n' "$runner_status" > "$attempt_dir/RUNNER_EXIT_CODE.txt"
    summarize
    if [[ "$runner_status" == "130" || "$runner_status" == "2" ]]; then
      printf '[定位矩阵] 收到中断或全局前置失败，暂停矩阵：seed=%s status=%s\n' \
        "$seed" "$runner_status" >&2
      exit "$runner_status"
    fi
    existing_attempts=$((existing_attempts + 1))
    if python3 "$SUMMARIZER" --validate-run "$attempt_dir" \
      --seed "$seed" --backend "$BACKEND" >/dev/null 2>&1; then
      printf '[定位矩阵] seed=%s 证据有效，runner_status=%s\n' "$seed" "$runner_status"
      break
    fi
    printf '[定位矩阵] seed=%s attempt=%s 证据无效，runner_status=%s\n' \
      "$seed" "$attempt" "$runner_status"
    attempt=$((attempt + 1))
  done
  if (( index == 4 )); then
    printf '[定位矩阵] 五个试点 seed 已完成，继续剩余十五个固定 seed\n'
  fi
done

summarize
printf '[定位矩阵] 完成；摘要：%s\n' "$OUTPUT_ROOT/matrix_summary.md"

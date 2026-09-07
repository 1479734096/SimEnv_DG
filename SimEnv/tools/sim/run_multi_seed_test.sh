#!/usr/bin/env bash
# 多 Seed 连续仿真自动化测试脚本
# 支持命令行传入多个 seed 连续运行完整闭环仿真测试，默认 3 层无头模式，支持自定义楼层与 GUI。

set -Eeuo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNNER="$WORKSPACE_ROOT/tools/sim/run_elevator_gui_test.sh"
EVALUATOR="$WORKSPACE_ROOT/src/building_obstacles/scripts/evaluate_danger.py"
DEVEL_SETUP="$WORKSPACE_ROOT/devel/setup.bash"

DEFAULT_EVIDENCE_ROOT="/home/xbh/tzb26/evidence/simenv"
if [ ! -d "$DEFAULT_EVIDENCE_ROOT" ]; then
  DEFAULT_EVIDENCE_ROOT="/tmp"
fi

FLOOR_COUNT="${FLOOR_COUNT:-3}"
GUI="${GUI:-false}"
RECORD_RGBD="${RECORD_RGBD:-0}"
OUTPUT_ROOT="${OUTPUT_ROOT:-}"
SEEDS=()

usage() {
  cat <<'EOF'
用法：
  tools/sim/run_multi_seed_test.sh [选项] SEED [SEED ...]

参数：
  SEED                        一个或多个场景随机种子（非负整数），必须至少提供一个

选项：
  -f, --floors <2|3>          测试楼层数（默认 3）
  --gui                       开启 Gazebo GUI（默认无头模式关闭 GUI）
  --no-gui                    关闭 Gazebo GUI（默认）
  --record-rgbd               开启全任务 RGB-D 录包（默认关闭，防止多 seed 磁盘耗尽）
  -o, --output-dir <DIR>      测试证据输出根目录，默认自动按时间戳生成
  -h, --help                  显示本帮助信息并退出

环境变量覆盖支持：
  FLOOR_COUNT=3               楼层数（支持 2 或 3）
  GUI=false                   是否开启 GUI（true/false）
  RECORD_RGBD=0               是否录制 RGB-D（0/1）
  OUTPUT_ROOT=/path/...       输出证据根目录

示例：
  # 默认 3 层无头模式连续测试 3 个 seed
  tools/sim/run_multi_seed_test.sh 8127 1612 6946

  # 指定 2 层、带 GUI 连续测试
  tools/sim/run_multi_seed_test.sh -f 2 --gui 8127 1612

  # 指定输出目录并开启 RGB-D 录包
  tools/sim/run_multi_seed_test.sh -o /tmp/batch_eval --record-rgbd 8127 1612

返回码：
  0  所有 seed 测试全部成功
  1  存在算法失败或超时的 seed
  2  参数或前置依赖错误
  130 用户主动中断（Ctrl-C）
EOF
}

# 命令行参数解析
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -f|--floors)
      if [[ $# -lt 2 ]]; then
        printf '错误：选项 %s 缺少楼层数值\n' "$1" >&2
        exit 2
      fi
      FLOOR_COUNT="$2"
      shift 2
      ;;
    --gui)
      GUI="true"
      shift
      ;;
    --no-gui)
      GUI="false"
      shift
      ;;
    --record-rgbd)
      RECORD_RGBD="1"
      shift
      ;;
    --no-record-rgbd)
      RECORD_RGBD="0"
      shift
      ;;
    -o|--output-dir)
      if [[ $# -lt 2 ]]; then
        printf '错误：选项 %s 缺少目录路径\n' "$1" >&2
        exit 2
      fi
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --)
      shift
      while [[ $# -gt 0 ]]; do
        SEEDS+=("$1")
        shift
      done
      break
      ;;
    -*)
      printf '错误：未知选项 %s；使用 --help 查看用法\n' "$1" >&2
      exit 2
      ;;
    *)
      SEEDS+=("$1")
      shift
      ;;
  esac
done

if (( ${#SEEDS[@]} == 0 )); then
  printf '错误：必须传入至少 1 个 seed。示例：tools/sim/run_multi_seed_test.sh 8127 1612\n' >&2
  printf '使用 --help 查看完整帮助\n' >&2
  exit 2
fi

if [[ "$FLOOR_COUNT" != "2" && "$FLOOR_COUNT" != "3" ]]; then
  printf '错误：FLOOR_COUNT 仅支持 2 或 3，当前为：%s\n' "$FLOOR_COUNT" >&2
  exit 2
fi

if [[ "$GUI" != "true" && "$GUI" != "false" ]]; then
  printf '错误：GUI 只能是 true 或 false，当前为：%s\n' "$GUI" >&2
  exit 2
fi

if [[ "$RECORD_RGBD" != "0" && "$RECORD_RGBD" != "1" ]]; then
  printf '错误：RECORD_RGBD 只能是 0 或 1，当前为：%s\n' "$RECORD_RGBD" >&2
  exit 2
fi

declare -A SEEN_SEEDS=()
FILTERED_SEEDS=()
for seed in "${SEEDS[@]}"; do
  if [[ ! "$seed" =~ ^[0-9]+$ ]]; then
    printf '错误：seed 必须是非负整数，非法输入：%s\n' "$seed" >&2
    exit 2
  fi
  if [[ -n "${SEEN_SEEDS[$seed]:-}" ]]; then
    printf '提示：跳过重复 seed：%s\n' "$seed"
    continue
  fi
  SEEN_SEEDS["$seed"]=1
  FILTERED_SEEDS+=("$seed")
done
SEEDS=("${FILTERED_SEEDS[@]}")

if [[ ! -f "$RUNNER" ]]; then
  printf '错误：缺少底层运行器：%s\n' "$RUNNER" >&2
  exit 2
fi

if [[ ! -f "$DEVEL_SETUP" ]]; then
  printf '错误：工作空间尚未构建，缺少：%s\n' "$DEVEL_SETUP" >&2
  exit 2
fi

if [[ -z "$OUTPUT_ROOT" ]]; then
  OUTPUT_ROOT="${DEFAULT_EVIDENCE_ROOT}/batch_${FLOOR_COUNT}floor_$(date +%Y%m%d_%H%M%S)"
fi

if [[ -d "$OUTPUT_ROOT" && -n "$(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null || true)" ]]; then
  printf '错误：输出根目录非空，拒绝覆盖：%s\n' "$OUTPUT_ROOT" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"

CURRENT_CHILD_PID=""
INTERRUPTED=0
cleanup_all_processes() {
  killall -9 gzserver gzclient rosmaster roscore junior_ctrl danger_search 2>/dev/null || true
}
handle_signal() {
  printf '\n[多Seed测试] 收到中断信号，正在安全终止当前轮次……\n'
  INTERRUPTED=1
  if [[ -n "$CURRENT_CHILD_PID" ]] && kill -0 "$CURRENT_CHILD_PID" 2>/dev/null; then
    kill -TERM "$CURRENT_CHILD_PID" 2>/dev/null || true
  fi
  cleanup_all_processes
}
trap handle_signal INT TERM
trap cleanup_all_processes EXIT

printf '==========================================================\n'
printf '[多Seed测试] 启动批量仿真测试\n'
printf '  测试楼层: %s 层\n' "$FLOOR_COUNT"
printf '  界面模式: %s\n' "$([[ "$GUI" == "true" ]] && printf "开启 GUI" || printf "无头模式 (GUI=false)")"
printf '  RGB-D录包: %s\n' "$([[ "$RECORD_RGBD" == "1" ]] && printf "开启 (注意磁盘容量)" || printf "关闭 (推荐)")"
printf '  种子列表: %s (共 %d 个)\n' "${SEEDS[*]}" "${#SEEDS[@]}"
printf '  证据输出: %s\n' "$OUTPUT_ROOT"
printf '==========================================================\n'

declare -A RUN_RESULTS=()
declare -A RUN_DETAILS=()
declare -A RUN_DURATIONS=()
declare -A RUN_DRIFTS=()
declare -A RUN_SCORES=()

total_seeds=${#SEEDS[@]}
completed_count=0
success_count=0

for index in "${!SEEDS[@]}"; do
  if (( INTERRUPTED == 1 )); then
    break
  fi

  seed="${SEEDS[$index]}"
  current_idx=$((index + 1))
  seed_dir="$OUTPUT_ROOT/seed_${seed}"
  mkdir -p "$seed_dir"

  # 启动前建立时间戳标记，用于判断 results 是否由本轮测试生成
  marker="$seed_dir/.seed_started_marker"
  touch "$marker"

  # 如果公共 results/ 中存在未归档的历史遗留文件，安全移动至 backup 目录，绝不直接删除
  if compgen -G "$WORKSPACE_ROOT/results/*.json" >/dev/null; then
    legacy_backup="$OUTPUT_ROOT/legacy_results_backup"
    mkdir -p "$legacy_backup"
    mv -f "$WORKSPACE_ROOT/results/"*.json "$legacy_backup/" 2>/dev/null || true
  fi

  printf '\n>>> [%d/%d] 开始测试 seed=%s (输出目录: %s)\n' \
    "$current_idx" "$total_seeds" "$seed" "$seed_dir"

  start_wall=$SECONDS
  set +e

  env \
    SEED="$seed" \
    ELEVATOR_TEST_FLOOR_COUNT="$FLOOR_COUNT" \
    GUI="$GUI" \
    ELEVATOR_TEST_RECORD_DRIFT=1 \
    ELEVATOR_TEST_RECORD_RGBD="$RECORD_RGBD" \
    ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 \
    ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1 \
    ELEVATOR_TEST_LOG_DIR="$seed_dir" \
    "$RUNNER" &
  CURRENT_CHILD_PID=$!
  wait "$CURRENT_CHILD_PID"
  runner_exit=$?
  CURRENT_CHILD_PID=""
  set -e
  cleanup_all_processes

  elapsed_wall=$((SECONDS - start_wall))
  RUN_DURATIONS["$seed"]="${elapsed_wall}s"

  # 将本轮生成的选手结果文件与裁判真值完整移动归档保存至当前 seed 目录
  if [[ -f "$WORKSPACE_ROOT/results/danger_truth.json" ]]; then
    mv -f "$WORKSPACE_ROOT/results/danger_truth.json" "$seed_dir/danger_truth.json" 2>/dev/null || true
  fi
  if [[ -f "$WORKSPACE_ROOT/results/detected_danger.json" && ( ! -f "$marker" || "$WORKSPACE_ROOT/results/detected_danger.json" -nt "$marker" ) ]]; then
    mv -f "$WORKSPACE_ROOT/results/detected_danger.json" "$seed_dir/detected_danger.json" 2>/dev/null || true
  fi
  if compgen -G "$WORKSPACE_ROOT/results/*.json" >/dev/null; then
    mv -f "$WORKSPACE_ROOT/results/"*.json "$seed_dir/" 2>/dev/null || true
  fi
  rm -f "$marker" 2>/dev/null || true

  score_str="N/A"
  if [[ -f "$EVALUATOR" && -f "$seed_dir/danger_truth.json" && -f "$seed_dir/detected_danger.json" ]]; then
    python3 "$EVALUATOR" \
      --truth-file "$seed_dir/danger_truth.json" \
      --detected-file "$seed_dir/detected_danger.json" \
      --output-file "$seed_dir/evaluation_result.json" \
      > "$seed_dir/evaluate_danger.log" 2>&1 || true
    if [[ -f "$seed_dir/evaluation_result.json" ]]; then
      score_str="$(python3 -c '
import json, sys
try:
    with open(sys.argv[1]) as f:
        d = json.load(f)
    m = d.get("metrics", {})
    s = d.get("scores", {})
    correct = m.get("correct", 0)
    truth = m.get("truth_count", 0)
    total_score = s.get("technical_objective_total", 0.0)
    print(f"{correct}/{truth} ({total_score:.1f}分)")
except Exception:
    print("解析异常")
' "$seed_dir/evaluation_result.json" 2>/dev/null || printf 'N/A')"
    fi
  fi
  RUN_SCORES["$seed"]="$score_str"

  # 读取漂移指标
  drift_str="N/A"
  if [[ -f "$seed_dir/localization_drift.json" ]]; then
    drift_str="$(python3 -c '
import json, sys
try:
    with open(sys.argv[1]) as f:
        d = json.load(f)
    max_pos = d.get("maximum_position_error")
    final_pos = d.get("final_position_error")
    if max_pos is not None:
        print(f"最大{max_pos:.2f}m / 最终{final_pos:.2f}m")
    else:
        print("未计算")
except Exception:
    print("N/A")
' "$seed_dir/localization_drift.json" 2>/dev/null || printf 'N/A')"
  fi
  RUN_DRIFTS["$seed"]="$drift_str"

  # 读取 TEST_SUMMARY.txt
  summary_result="未知"
  summary_detail="无详情"
  if [[ -f "$seed_dir/TEST_SUMMARY.txt" ]]; then
    summary_result="$(grep -E '^result=' "$seed_dir/TEST_SUMMARY.txt" | cut -d= -f2- || true)"
    summary_detail="$(grep -E '^detail=' "$seed_dir/TEST_SUMMARY.txt" | cut -d= -f2- || true)"
  fi

  if (( INTERRUPTED == 1 )); then
    summary_result="中断"
    summary_detail="用户按 Ctrl-C 中断"
  elif (( runner_exit == 0 )); then
    summary_result="通过"
    success_count=$((success_count + 1))
  else
    if [[ -z "$summary_result" || "$summary_result" == "未完成" ]]; then
      summary_result="失败"
    fi
  fi

  RUN_RESULTS["$seed"]="$summary_result"
  RUN_DETAILS["$seed"]="$summary_detail"
  completed_count=$((completed_count + 1))

  printf '<<< [%d/%d] seed=%s 完成: 结果=[%s], 耗时=%s, 评分=[%s], 漂移=[%s]\n' \
    "$current_idx" "$total_seeds" "$seed" "$summary_result" "${elapsed_wall}s" "$score_str" "$drift_str"
done

# 生成 BATCH_SUMMARY.md
SUMMARY_MD="$OUTPUT_ROOT/BATCH_SUMMARY.md"
{
  printf '# 多 Seed 连续测试汇总报告\n\n'
  printf '- **运行时间**: %s\n' "$(date '+%Y-%m-%d %H:%M:%S')"
  printf '- **测试楼层**: %s 层\n' "$FLOOR_COUNT"
  printf '- **界面模式**: %s\n' "$([[ "$GUI" == "true" ]] && printf "GUI 开启" || printf "无头模式 (GUI 关闭)")"
  printf '- **RGB-D 录包**: %s\n' "$([[ "$RECORD_RGBD" == "1" ]] && printf "开启" || printf "关闭")"
  printf '- **完成进度**: %d / %d 个 Seed\n' "$completed_count" "$total_seeds"
  printf '- **通过率**: %d / %d (%s%%)\n' \
    "$success_count" "$completed_count" \
    "$([ "$completed_count" -gt 0 ] && awk -v s="$success_count" -v c="$completed_count" 'BEGIN { printf "%.1f", (s/c)*100 }' || printf '0.0')"
  printf '\n## 明细结果\n\n'
  printf '| Seed | 状态 | 耗时 | 危险源得分 (正确/总数) | 定位漂移 (最大/最终) | 详细说明 |\n'
  printf '|---|---|---|---|---|---|\n'
  for seed in "${SEEDS[@]}"; do
    if [[ -n "${RUN_RESULTS[$seed]:-}" ]]; then
      status_icon="❌ 失败"
      if [[ "${RUN_RESULTS[$seed]}" == "通过" ]]; then
        status_icon="✅ 通过"
      elif [[ "${RUN_RESULTS[$seed]}" == "中断" ]]; then
        status_icon="⏹️ 中断"
      fi
      printf '| `%s` | %s | %s | %s | %s | %s |\n' \
        "$seed" \
        "$status_icon" \
        "${RUN_DURATIONS[$seed]:-N/A}" \
        "${RUN_SCORES[$seed]:-N/A}" \
        "${RUN_DRIFTS[$seed]:-N/A}" \
        "${RUN_DETAILS[$seed]:-无}"
    else
      printf '| `%s` | ⏳ 未执行 | - | - | - | 批次提前终止 |\n' "$seed"
    fi
  done
  printf '\n## 证据目录\n\n'
  printf '- 根目录: `%s`\n' "$OUTPUT_ROOT"
} > "$SUMMARY_MD"

printf '\n==========================================================\n'
printf '[多Seed测试] 批次测试结束\n'
printf '  总计: %d | 完成: %d | 通过: %d | 失败: %d\n' \
  "$total_seeds" "$completed_count" "$success_count" "$((completed_count - success_count))"
printf '  汇总报告已写出: %s\n' "$SUMMARY_MD"
printf '==========================================================\n'

cat "$SUMMARY_MD"

if (( INTERRUPTED == 1 )); then
  exit 130
fi

if (( success_count == completed_count && completed_count == total_seeds )); then
  exit 0
else
  exit 1
fi

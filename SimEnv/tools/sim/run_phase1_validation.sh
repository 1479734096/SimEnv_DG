#!/usr/bin/env bash
# Phase 1: 验证 PATH 死区补偿修复效果（底层调用 run_elevator_gui_test.sh）
# 运行 5 个 seed 的 2 层闭环测试，专用于历史 commit 1096603 的回归测试

set -Eeuo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNNER="$WORKSPACE_ROOT/tools/sim/run_elevator_gui_test.sh"
OUTPUT_ROOT="${PHASE1_OUTPUT_ROOT:-/tmp/phase1_deadzone_validation}"
REQUIRED_COMMIT="1096603"

usage() {
  cat <<'EOF'
用法：
  PHASE1_OUTPUT_ROOT=/tmp/phase1_validation \
    tools/sim/run_phase1_validation.sh

功能：
  - 验证 commit 1096603 的 PATH 死区补偿修复
  - 运行 5 个 seed 的 2 层闭环测试
  - seed 列表：8127（已知失败）+ 4 个随机 seed
  - GUI=true，2 层，280 s 单层预算，60 s 最短探索
  - 自动生成验证报告

固定参数：
  - 楼层：2 层（0→1→0→home）
  - 后端：adaptive_shared_map
  - 单层预算：280 秒
  - 总预算：560 秒
  - 墙钟超时：9000 秒
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

# 验证依赖
for path in "$RUNNER"; do
  if [[ ! -f "$path" ]]; then
    printf '缺少运行器：%s\n' "$path" >&2
    exit 2
  fi
done

if [[ ! -f "$WORKSPACE_ROOT/devel/setup.bash" ]]; then
  printf '工作区尚未构建，缺少：%s\n' "$WORKSPACE_ROOT/devel/setup.bash" >&2
  exit 2
fi

# 验证 Git 状态
HEAD_COMMIT="$(git -C "$WORKSPACE_ROOT" rev-parse --short HEAD)"
if [[ "$HEAD_COMMIT" != "$REQUIRED_COMMIT" ]]; then
  printf '警告：当前 HEAD (%s) 不是预期的 %s\n' "$HEAD_COMMIT" "$REQUIRED_COMMIT" >&2
  printf '是否继续？[y/N] '
  read -r response
  if [[ "$response" != "y" && "$response" != "Y" ]]; then
    exit 1
  fi
fi

# Seed 列表：已知失败 seed + 4 个随机 seed
SEEDS=(8127 1234 5678 9101 3456)

mkdir -p "$OUTPUT_ROOT"
REPORT_FILE="$OUTPUT_ROOT/PHASE1_VALIDATION_REPORT.md"

# 初始化报告
{
  printf '# Phase 1 验证报告\n\n'
  printf '## 测试配置\n\n'
  printf '- **提交**: %s\n' "$HEAD_COMMIT"
  printf '- **日期**: %s\n' "$(date --iso-8601=seconds)"
  printf '- **楼层**: 2 层\n'
  printf '- **后端**: adaptive_shared_map\n'
  printf '- **Seeds**: %s\n\n' "${SEEDS[*]}"
  printf '## 测试结果\n\n'
} > "$REPORT_FILE"

success_count=0
total_count="${#SEEDS[@]}"

for seed in "${SEEDS[@]}"; do
  run_dir="$OUTPUT_ROOT/seed_$seed"
  console_log="$OUTPUT_ROOT/seed_${seed}.console.log"

  printf '\n[Phase1] 开始 seed=%s (%d/%d)\n' "$seed" "$((success_count + 1))" "$total_count"

  started_at="$(date --iso-8601=seconds)"
  started_monotonic="$SECONDS"

  # 显式传递 Phase 1 固定参数（2层、280s单层/60s探索/560s总预算），底层 runner 自动按新默认值录制完整诊断与真值
  set +e
  env \
    SEED="$seed" \
    GUI=true \
    ELEVATOR_TEST_FLOOR_COUNT=2 \
    ELEVATOR_TEST_FLOOR_TIMEOUT=280 \
    ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60 \
    ELEVATOR_TEST_MAX_RUNTIME=560 \
    ELEVATOR_TEST_WALL_TIMEOUT=9000 \
    ELEVATOR_TEST_FULL_CYCLE=1 \
    ELEVATOR_TEST_LOCALIZATION_BACKEND=adaptive_shared_map \
    ELEVATOR_TEST_LOG_DIR="$run_dir" \
    "$RUNNER" 2>&1 | tee "$console_log"
  runner_status=${PIPESTATUS[0]}
  set -e

  wall_elapsed=$((SECONDS - started_monotonic))
  finished_at="$(date --iso-8601=seconds)"

  # 分析结果
  if [[ "$runner_status" == "0" ]]; then
    result="✅ SUCCESS"
    success_count=$((success_count + 1))
    failure_mode="N/A"
  else
    result="❌ FAILED"
    # 尝试提取失败模式
    if [[ -f "$run_dir/mission.log" ]]; then
      failure_mode="$(grep -oP '(?<=reason=")[^"]+' "$run_dir/mission.log" | tail -1 || echo "unknown")"
    else
      failure_mode="unknown"
    fi
  fi

  printf '[Phase1] seed=%s 结果=%s (耗时=%d秒)\n' "$seed" "$result" "$wall_elapsed"

  # 写入报告
  {
    printf '### Seed %s\n\n' "$seed"
    printf '- **结果**: %s\n' "$result"
    printf '- **开始时间**: %s\n' "$started_at"
    printf '- **结束时间**: %s\n' "$finished_at"
    printf '- **墙钟耗时**: %d 秒\n' "$wall_elapsed"
    printf '- **退出码**: %d\n' "$runner_status"
    if [[ "$result" == "❌ FAILED" ]]; then
      printf '- **失败模式**: %s\n' "$failure_mode"
    fi
    printf '- **日志目录**: `%s`\n' "$run_dir"
    printf '- **控制台日志**: `%s`\n\n' "$console_log"
  } >> "$REPORT_FILE"
done

# 生成总结
{
  printf '## 总结\n\n'
  printf '- **成功**: %d/%d\n' "$success_count" "$total_count"
  printf '- **失败**: %d/%d\n' "$((total_count - success_count))" "$total_count"
  printf '- **成功率**: %.1f%%\n\n' "$(awk "BEGIN {printf \"%.1f\", ($success_count / $total_count) * 100}")"

  if (( success_count == total_count )); then
    printf '### 结论\n\n'
    printf '✅ **全部通过**：PATH 死区补偿修复有效，4/5 seed 卡滞问题已解决。\n\n'
    printf '**建议**: 关闭架构重构计划，问题已解决。\n'
  elif (( success_count >= 3 )); then
    printf '### 结论\n\n'
    printf '⚠️ **部分成功**：成功率提升，修复有效但可能存在其他问题。\n\n'
    printf '**建议**: 分析剩余失败 seed 的失败模式，决定是否需要额外修复。\n'
  else
    printf '### 结论\n\n'
    printf '❌ **修复失败**：成功率无明显提升，根因诊断可能错误。\n\n'
    printf '**建议**: 回退到 b15b5b4 基线，重新诊断或执行完整重构计划。\n'
  fi

  printf '\n## 下一步\n\n'
  if (( success_count == total_count )); then
    printf '1. 归档验证报告\n'
    printf '2. 更新 `/home/xbh/tzb26/cc/README.md` 标记问题已解决\n'
    printf '3. 继续其他开发任务\n'
  elif (( success_count >= 3 )); then
    printf '1. 委托 Opus 分析剩余失败模式\n'
    printf '2. 决策：小修复 vs 部分重构\n'
  else
    printf '1. 回退代码: `git reset --hard b15b5b4`\n'
    printf '2. 委托 Opus 重新诊断\n'
    printf '3. 考虑执行 `/home/xbh/tzb26/cc/03_EXECUTION_PLAN.md`\n'
  fi
} >> "$REPORT_FILE"

printf '\n[Phase1] 验证完成，报告位于: %s\n' "$REPORT_FILE"
printf '[Phase1] 成功率: %d/%d (%.1f%%)\n' \
  "$success_count" "$total_count" \
  "$(awk "BEGIN {printf \"%.1f\", ($success_count / $total_count) * 100}")"

exit 0

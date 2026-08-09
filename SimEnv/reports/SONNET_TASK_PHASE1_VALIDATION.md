# Sonnet 任务规格：Phase 1 Seed 验证

## 任务目标

验证 commit 1096603 的 PATH 死区补偿修复是否解决 4/5 seed 失败问题。

## 背景

- **问题**: 4/5 seed 失败，失败模式为 `return_home_stuck_exhausted`
- **根因**: PATH 模式缺少死区补偿，A1 电机在小角速度下存在执行死区
- **修复**: navigator.py 添加死区补偿逻辑（commit 1096603）
- **验证**: 707/707 单元测试通过，需要 Gazebo seed 测试验证实际效果

## 执行步骤

### 第一阶段：快速验证（seed 8127）

运行已知失败 seed 进行快速验证：

```bash
cd /home/xbh/tzb26/codeDG_rebuild

# 确认 Git 状态
git log -1 --oneline  # 应显示 1096603

# 运行 seed 8127
SEED=8127 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=2 \
ELEVATOR_TEST_FLOOR_TIMEOUT=280 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60 \
ELEVATOR_TEST_MAX_RUNTIME=560 \
ELEVATOR_TEST_WALL_TIMEOUT=9000 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_LOCALIZATION_BACKEND=adaptive_shared_map \
ELEVATOR_TEST_LOG_DIR=/tmp/phase1_quick_8127 \
./tools/sim/run_elevator_gui_test.sh

# 记录退出码
echo "Exit code: $?" > /tmp/phase1_quick_8127/exit_code.txt
```

**预计时间**: 15-20 分钟

**成功判定**:
- 退出码 = 0
- 日志中显示完成 2 层闭环（0→1→0→home）
- 无 `return_home_stuck_exhausted` 错误

**失败判定**:
- 退出码 ≠ 0
- 仍然出现 `return_home_stuck_exhausted`
- 或出现新的失败模式

### 第二阶段：完整验证（如果第一阶段成功）

运行剩余 4 个 seed：

```bash
cd /home/xbh/tzb26/codeDG_rebuild

# 使用完整验证脚本
PHASE1_OUTPUT_ROOT=/tmp/phase1_deadzone_validation \
  ./tools/sim/run_phase1_validation.sh
```

**注意**: 脚本会跳过已完成的 seed 8127，只运行剩余的 seed。

**预计时间**: 1-1.5 小时

### 第三阶段：生成报告

无论成功或失败，都生成验证报告：

```bash
cd /home/xbh/tzb26/codeDG_rebuild
python3 ./tools/orchestrator.py
```

编排器会自动分析结果并生成相应报告。

## 决策逻辑

### 场景 A: seed 8127 成功 ✅
→ 继续运行剩余 4 个 seed
→ 如果 5/5 成功：生成成功报告，任务完成
→ 如果 3-4/5 成功：创建 Opus 决策请求
→ 如果 0-2/5 成功（包括 8127）：创建 Opus 决策请求

### 场景 B: seed 8127 失败 ❌
→ **立即停止**，不运行剩余 seed
→ 分析失败日志
→ 创建 Opus 决策请求，包含：
  - 失败模式（是否仍是 return_home_stuck_exhausted）
  - 日志关键片段
  - 建议下一步（回退/重新诊断/深入分析）

## 输出要求

### 成功时输出

在 `/home/xbh/tzb26/codeDG_rebuild/reports/SONNET_PHASE1_REPORT.md` 中记录：

```markdown
# Phase 1 验证报告

## 执行摘要

- **seed 8127 快速验证**: ✅ 成功
- **完整验证**: ✅ 5/5 成功
- **结论**: PATH 死区补偿修复有效，问题已解决

## 详细结果

### Seed 8127
- 结果: ✅ SUCCESS
- 耗时: XX 秒
- 日志: /tmp/phase1_quick_8127/

### Seed 1234
...（其他 seed）

## 下一步

任务完成，无需进一步操作。
```

### 失败时输出

在 `/home/xbh/tzb26/codeDG_rebuild/reports/SONNET_PHASE1_REPORT.md` 中记录：

```markdown
# Phase 1 验证报告

## 执行摘要

- **seed 8127 快速验证**: ❌ 失败
- **失败模式**: [具体模式]
- **结论**: 修复未生效或根因诊断错误

## 失败分析

### 失败日志片段
[关键日志]

### 可能原因
1. [原因1]
2. [原因2]

## 下一步

已创建 Opus 决策请求，等待分析。
```

## 约束条件

1. **只读操作**: 除了生成报告外，不修改任何代码
2. **Git 状态**: 确认在 commit 1096603
3. **环境检查**: 运行前检查 ROS/Gazebo 可用性
4. **日志保留**: 保留所有 seed 的完整日志
5. **中断处理**: 如果被中断，记录当前状态便于恢复

## 成功标准

- [ ] seed 8127 完成验证（成功或失败都记录）
- [ ] 如果 8127 成功，完成 5/5 验证
- [ ] 生成验证报告
- [ ] 运行编排器分析结果
- [ ] 根据结果创建相应的后续任务（如需要）

## 预计总时间

- **最快**: 20 分钟（8127 失败，立即停止）
- **正常**: 1.5-2 小时（8127 成功，完成 5/5 验证）
- **最慢**: 2.5 小时（包括失败重试和分析）

## 开始执行

请按照上述步骤执行 Phase 1 验证，并根据结果生成相应报告。

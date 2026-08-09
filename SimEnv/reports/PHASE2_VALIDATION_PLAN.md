# Phase 2: Seed 8127 重新验证

## 目标

验证 IDLE 模式死区补偿修复（commit 26b176f）是否解决了原地摆动问题。

## 背景

**已完成的修复**:
1. Commit 1096603: PATH 模式死区补偿
2. Commit 26b176f: IDLE 模式（rotate_in_place）死区补偿

**Phase 1 验证结果**:
- PATH 模式大部分场景正常 ✅
- 返航时在 `rotate_in_place()` 处原地摆动 ❌
- 电梯导航失败（独立问题，本次不处理）

**核心修复**:
在 `rotate_in_place()` 方法中添加死区补偿，小角速度（< 0.55 rad/s）自动提升到最小可执行转速。

## 验证策略

### 快速验证（推荐）

只运行 seed 8127，重点观察返航阶段：

```bash
cd /home/xbh/tzb26/codeDG_rebuild

SEED=8127 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=2 \
ELEVATOR_TEST_FLOOR_TIMEOUT=280 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60 \
ELEVATOR_TEST_MAX_RUNTIME=560 \
ELEVATOR_TEST_WALL_TIMEOUT=9000 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_LOCALIZATION_BACKEND=adaptive_shared_map \
ELEVATOR_TEST_LOG_DIR=/tmp/phase2_quick_8127 \
./tools/sim/run_elevator_gui_test.sh
```

### 关键观察点

1. **电梯导航阶段**（预期仍会失败，这是已知独立问题）
   - 是否仍然 `elevator_navigation_unreachable`
   - 记录但不视为阻塞问题

2. **返航阶段**（核心验证点）
   - 接近目标点（< 0.2m）时是否还有原地摆动
   - `rotate_in_place()` 调用时角速度是否被补偿到 ≥ 0.55 rad/s
   - 机器人是否能实际转动并到达目标

3. **探索停滞**（次要观察点）
   - 是否仍有 simtime 120s, 151s 等停滞时刻
   - 记录但不视为本次修复范围

### 成功标准

**完全成功** (预期概率 60%):
- ✅ 返航顺利完成，无原地摆动
- ✅ 到达 home 点
- ⚠️ 电梯导航失败（预期，不阻塞）
- 失败模式：`floor_change_failed_elevator_navigation_unreachable`（而非 `return_home_stuck_exhausted`）

**部分成功** (预期概率 30%):
- ✅ 返航有改善（摆动时间 < 10 秒）
- ❌ 仍然卡住但不如之前严重
- 需要微调死区补偿参数

**失败** (预期概率 10%):
- ❌ 返航仍然原地摆动 60+ 秒
- 说明问题更深层，需要系统化诊断

## 执行计划

### 阶段 1: 运行 seed 8127 验证（1-1.5 小时）

Sonnet 启动测试并监控：
- 每 60 秒检查状态
- 重点监控返航阶段
- 检测到明确失败立即停止

### 阶段 2: 分析结果（15 分钟）

根据测试结果分类：

**场景 A: 返航成功** ✅
- 电梯失败但返航成功
- 说明死区补偿修复完全有效
- 下一步：标记电梯问题为独立缺陷，运行 5 seed 完整验证

**场景 B: 返航有改善但仍卡住**
- 摆动时间 < 10 秒但仍未到达
- 说明补偿有效但参数不足
- 下一步：微调死区补偿参数（增大到 0.65 或 0.75 rad/s）

**场景 C: 返航仍然失败**
- 与 Phase 1 相同，摆动 60+ 秒
- 说明 `rotate_in_place()` 补偿未生效或问题更深
- 下一步：回退到 b15b5b4，使用 systematic-debugging 系统化诊断

### 阶段 3: 根据结果推进

**如果场景 A**:
1. 更新电梯问题状态（独立缺陷，不阻塞闭环目标）
2. 运行完整 5 seed 验证
3. 预计 3-4/5 成功（电梯问题可能影响 1-2 个 seed）
4. 如果 ≥3/5 → 评估是否达成目标
5. 如果 <3/5 → 分析新的失败模式

**如果场景 B**:
1. 微调死区补偿参数
2. 重新验证 seed 8127
3. 如果成功 → 转场景 A 流程
4. 如果仍失败 → 转场景 C 流程

**如果场景 C**:
1. 回退到 b15b5b4
2. 使用 systematic-debugging skill 分析
3. 重新制定修复方案

## 时间估算

- 阶段 1: 1-1.5 小时（测试运行）
- 阶段 2: 15 分钟（结果分析）
- 阶段 3a: 1.5-2 小时（场景 A，5 seed 验证）
- 阶段 3b: 1-2 小时（场景 B，参数调优）
- 阶段 3c: 4-6 小时（场景 C，系统化诊断）

**总计**: 2-10 小时（取决于结果）

## 监控配置

Sonnet 应实现：
1. 每 60 秒检查 dog 状态
2. 返航阶段重点监控 `rotate_in_place()` 调用
3. 检测到以下情况立即报告：
   - 原地摆动 > 30 秒
   - 返航成功到达 home
   - 测试结束（任何原因）

## 日志位置

- 测试日志: `/tmp/phase2_quick_8127/mission.log`
- 监控日志: `/tmp/phase2_monitor.log`
- 验证报告: `/home/xbh/tzb26/codeDG_rebuild/reports/PHASE2_VALIDATION_REPORT.md`

## 成功定义

**最小成功**: 返航不再出现 60+ 秒原地摆动，能到达 home 点

**完全成功**: 5/5 seed 完成 2 层闭环（电梯问题不影响所有 seed）

---
创建时间: $(date --iso-8601=seconds)
状态: 待执行

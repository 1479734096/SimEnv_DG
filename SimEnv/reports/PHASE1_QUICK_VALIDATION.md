# Phase 1 快速验证任务

## 目标

运行单个 seed (8127) 快速验证 PATH 死区补偿修复是否有效。

## 执行计划

### 快速验证（约 15-20 分钟）
运行 seed=8127（已知失败 seed）测试 2 层闭环。

**成功标志**:
- seed 8127 完成 2 层闭环（之前失败）
- 无 `return_home_stuck_exhausted` 错误

**失败标志**:
- 仍然出现 `return_home_stuck_exhausted`
- 或新的失败模式

### 完整验证（如快速验证成功）
运行剩余 4 个 seed 完成 5/5 验证矩阵。

## 决策树

```
运行 seed 8127
    │
    ├─ ✅ 成功 → 运行剩余 4 个 seed → 5/5 验证
    │
    └─ ❌ 失败 → 立即停止，委托 Opus 分析
```

## 命令

```bash
# 快速验证
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
ELEVATOR_TEST_LOG_DIR=/tmp/quick_validation_8127 \
./tools/sim/run_elevator_gui_test.sh
```

## 结果收集

- 退出码: $?
- 日志目录: /tmp/quick_validation_8127/
- 关键日志: mission.log, auto.log

## 时间估算

- 单个 seed: 15-20 分钟
- 成功后完整验证: 额外 1 小时
- **总计**: 1.5 小时（最坏情况）

## 风险评估

**低风险**: 仅运行测试，不修改代码
**可中断**: Ctrl-C 随时停止
**可重复**: 失败可以重跑

## 委托给 Sonnet

由于我是 Opus 后台会话，应该委托给 Sonnet 执行此长时间任务。

**Sonnet 任务**:
1. 运行 seed 8127 快速验证
2. 如果成功，继续运行剩余 4 个 seed
3. 生成验证报告
4. 根据结果决定下一步

# 自动化验证与推进流程

## 当前状态

- ✅ Phase 0 完成：诊断与最小化修复
- ✅ 代码修复已提交（commit 1096603）
- ✅ 单元测试通过（707/707）
- ⏳ Phase 1 待执行：Gazebo seed 验证

## 自动化架构

```
┌─────────────────────────────────────────────────────────────┐
│                   Phase 1: Seed 验证                         │
│  运行 5 个 seed 的 2 层闭环测试（需要 Gazebo 环境）          │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ↓
         ┌────────────────────────┐
         │  tools/orchestrator.py  │  ← 自动分析结果
         └────────────┬───────────┘
                      │
         ┌────────────┴───────────┬───────────────┐
         ↓                        ↓               ↓
    5/5 成功                  3-4/5 成功      0-2/5 失败
         │                        │               │
         ↓                        ↓               ↓
   生成成功报告              委托 Opus 决策    委托 Opus 决策
   任务完成                  │                 │
                            ↓                 ↓
                      小修复/重构       回退/重新诊断
                            │                 │
                            ↓                 ↓
                      委托 Sonnet 执行  委托 Sonnet 执行
```

## 执行步骤

### 第一步：运行 Phase 1 验证（需要用户在 Gazebo 环境中执行）

```bash
cd /home/xbh/tzb26/codeDG_rebuild

# 运行 2 层闭环验证（5 个 seed）
PHASE1_OUTPUT_ROOT=/tmp/phase1_deadzone_validation \
  ./tools/sim/run_phase1_validation.sh
```

**预计时间**: 约 1.5-2 小时（5 个 seed，每个约 15-20 分钟）

**输出位置**:
- 验证报告：`/tmp/phase1_deadzone_validation/PHASE1_VALIDATION_REPORT.md`
- 各 seed 日志：`/tmp/phase1_deadzone_validation/seed_*/`

### 第二步：自动分析结果并决策

验证完成后，运行编排器：

```bash
cd /home/xbh/tzb26/codeDG_rebuild
python3 ./tools/orchestrator.py
```

编排器会：
1. 解析 Phase 1 验证结果
2. 根据成功率自动决策：
   - **5/5 成功** → 生成成功报告，任务完成
   - **3-4/5 成功** → 创建 Opus 决策请求
   - **0-2/5 失败** → 创建 Opus 决策请求

### 第三步：根据编排器输出推进

#### 场景 A: 5/5 成功 ✅

编排器自动生成 `reports/FINAL_SUCCESS_SUMMARY.md`，任务完成。

**输出示例**:
```
✅ Phase 1 验证成功：5/5 seed 完成二层闭环
🎉 任务完成！二层闭环目标已达成。
```

**后续动作**: 无需进一步操作

#### 场景 B: 3-4/5 成功 ⚠️

编排器创建 `reports/OPUS_DECISION_REQUEST.md`，等待 Opus 分析。

**Opus 将**:
1. 分析剩余失败模式
2. 决策：小修复 vs 部分重构
3. 创建 `OPUS_DECISION_RESPONSE.md`

**Sonnet 将**:
- 根据 Opus 决策执行具体修复或重构
- 运行测试验证
- 提交代码

#### 场景 C: 0-2/5 失败 ❌

编排器创建 `reports/OPUS_DECISION_REQUEST.md`，等待 Opus 分析。

**Opus 将**:
1. 分析失败原因（根因错误 vs 新引入问题）
2. 决策：回退 + 重新诊断 vs 启动完整重构
3. 创建 `OPUS_DECISION_RESPONSE.md`

**Sonnet 将**:
- 执行回退或重构任务
- 重新验证

## 关键文件位置

### 已完成
- `/home/xbh/tzb26/codeDG_rebuild/reports/PHASE0_COMPLETION_SUMMARY.md` - Phase 0 总结
- `/home/xbh/tzb26/codeDG_rebuild/src/danger_search/danger_search/navigator.py` - 修复代码
- `/home/xbh/tzb26/codeDG_rebuild/reports/SEED_VERIFICATION_GUIDE.md` - 验证指南

### 待生成
- `/tmp/phase1_deadzone_validation/PHASE1_VALIDATION_REPORT.md` - Phase 1 结果
- `/home/xbh/tzb26/codeDG_rebuild/reports/OPUS_DECISION_REQUEST.md` - Opus 决策请求（如需要）
- `/home/xbh/tzb26/codeDG_rebuild/reports/FINAL_SUCCESS_SUMMARY.md` - 最终成功报告（如成功）

### 脚本
- `/home/xbh/tzb26/codeDG_rebuild/tools/sim/run_phase1_validation.sh` - Phase 1 验证脚本
- `/home/xbh/tzb26/codeDG_rebuild/tools/orchestrator.py` - 自动化编排器

## 当前阻塞点

**阻塞**: Phase 1 需要 Gazebo 环境运行 seed 测试

**解除方式**: 用户在 Gazebo 环境中执行 `run_phase1_validation.sh`

**为什么不能自动化**:
- Gazebo 需要图形界面（GUI=true）
- ROS1 + 物理仿真环境
- 硬件依赖（GPU、内存）

## 时间估算

| 阶段 | 时间 | 执行者 |
|------|------|--------|
| Phase 1 验证 | 1.5-2h | 用户（Gazebo 环境）|
| 结果分析 | 1min | 编排器（自动）|
| Opus 决策 | 5-10min | Opus（如需要）|
| Sonnet 执行 | 30min-2h | Sonnet（如需要）|
| **总计** | **2-5h** | **混合** |

## 成功标准

**最终目标**: 5/5 seed 完成 2 层闭环探索

**当前进度**:
- [x] Phase 0: 诊断与修复
- [ ] Phase 1: Seed 验证
- [ ] Phase N: 根据结果决定（可能不需要）

## 快速开始

```bash
# 1. 验证当前状态
cd /home/xbh/tzb26/codeDG_rebuild
git log -1 --oneline  # 应显示 1096603

# 2. 运行 Phase 1 验证（Gazebo 环境）
PHASE1_OUTPUT_ROOT=/tmp/phase1_deadzone_validation \
  ./tools/sim/run_phase1_validation.sh

# 3. 分析结果并自动推进
python3 ./tools/orchestrator.py

# 4. 根据输出继续（如果需要）
```

## 注意事项

1. **Git 状态**: Phase 1 验证要求工作区干净（无未提交改动）
2. **环境要求**: 需要 ROS1 Noetic + Gazebo 环境
3. **日志保留**: 所有 seed 日志都会保存，便于失败分析
4. **可中断**: Ctrl-C 可中断验证，重新运行会从断点继续
5. **可重复**: 使用相同输出目录会跳过已完成的 seed

## 联系信息

如有问题，查看：
- Phase 0 诊断报告：`reports/PHASE0_DIAGNOSTIC_REPORT.md`
- Phase 0 完成总结：`reports/PHASE0_COMPLETION_SUMMARY.md`
- 验证指南：`reports/SEED_VERIFICATION_GUIDE.md`

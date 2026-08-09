# Phase 1 结果应对方案

## 场景矩阵

### 场景 1: Seed 8127 成功 ✅

**意义**: 修复有效，已知失败 seed 现在能通过

**下一步**:
1. Sonnet 继续运行剩余 4 个 seed (1234, 5678, 9101, 3456)
2. 预计额外时间: 1-1.5 小时

**子场景**:
- **1a: 5/5 成功** → 生成最终成功报告，任务完成 🎉
- **1b: 4/5 成功** → Opus 分析唯一失败的 seed，决定是否可接受
- **1c: 3/5 成功** → Opus 分析 2 个失败 seed 的共同点，决定小修复还是重构
- **1d: ≤2/5 成功** → 说明修复不稳定或有其他问题，Opus 深入分析

---

### 场景 2: Seed 8127 失败 ❌

**意义**: 修复未生效或根因诊断错误

**立即动作**:
1. Sonnet 停止后续测试
2. 分析失败日志，提取关键信息

**失败模式分析**:

#### 2a: 仍然是 `return_home_stuck_exhausted`
**结论**: 死区补偿未生效或参数不正确

**可能原因**:
- 死区补偿触发条件太严格（deadband_error=0.10 太大？）
- 最小角速度 0.55 rad/s 仍不足
- 补偿逻辑被其他代码路径绕过
- 配置参数覆盖了默认值

**Opus 决策**:
- 选项 A: 调整死区补偿参数（放宽触发条件或增大最小角速度）
- 选项 B: 检查是否有其他调用路径绕过了补偿
- 选项 C: 回退并重新诊断根因

#### 2b: 新的失败模式（非 return_home_stuck_exhausted）
**结论**: 修复引入了新问题或暴露了其他问题

**可能原因**:
- 死区补偿过于激进，导致震荡或不稳定
- 改变了某些边界行为
- 测试用例更新有误

**Opus 决策**:
- 选项 A: 分析新失败模式，判断是否与修复相关
- 选项 B: 如果是修复引入的，调整补偿策略
- 选项 C: 如果是独立问题，记录为新缺陷

#### 2c: 基础设施失败（Infrastructure failure）
**结论**: Gazebo/ROS 环境问题，非算法问题

**Opus 决策**:
- 选项 A: 重试一次（infrastructure_attempts < 2）
- 选项 B: 检查环境配置
- 选项 C: 如果持续失败，暂停验证

---

## 响应矩阵

| Seed 8127 结果 | 剩余 seed 结果 | 总成功率 | 决策者 | 下一步 |
|---------------|---------------|---------|--------|--------|
| ✅ 成功 | 4/4 成功 | 5/5 | 自动 | 生成成功报告，完成 |
| ✅ 成功 | 3/4 成功 | 4/5 | Opus | 分析唯一失败，决定可接受性 |
| ✅ 成功 | 2/4 成功 | 3/5 | Opus | 分析 2 个失败，小修复或重构 |
| ✅ 成功 | ≤1/4 成功 | ≤2/5 | Opus | 深入分析，修复不稳定 |
| ❌ 失败 (stuck) | N/A | 0/5 | Opus | 调整参数或重新诊断 |
| ❌ 失败 (新) | N/A | 0/5 | Opus | 分析新失败，判断相关性 |
| ❌ 失败 (infra) | N/A | 0/5 | Sonnet | 重试或检查环境 |

---

## 自动化触发器

### 触发器 1: 完全成功（5/5）
```python
if success_count == 5:
    generate_final_success_report()
    notify_user("🎉 任务完成！5/5 seed 通过，二层闭环目标达成。")
    exit(0)
```

### 触发器 2: 部分成功（3-4/5）
```python
if 3 <= success_count <= 4:
    create_opus_decision_request(
        context="部分成功",
        failures=get_failure_details(),
        question="如何处理剩余失败？"
    )
    notify_user("⚠️ 部分成功，等待 Opus 决策...")
```

### 触发器 3: 失败（0-2/5）
```python
if success_count <= 2:
    create_opus_decision_request(
        context="验证失败",
        failures=get_failure_details(),
        question="应该回退、重新诊断还是调整修复？"
    )
    notify_user("❌ 验证失败，等待 Opus 分析...")
```

---

## 准备工作

已准备的文件和工具：
- ✅ 验证脚本：`tools/sim/run_phase1_validation.sh`
- ✅ 编排器：`tools/orchestrator.py`
- ✅ 工作流文档：`reports/AUTOMATION_WORKFLOW.md`
- ✅ Sonnet 任务规格：`reports/SONNET_TASK_PHASE1_VALIDATION.md`

等待中：
- ⏳ Seed 8127 测试结果（Sonnet 监控中）
- ⏳ 完整 5/5 验证结果（如果 8127 成功）

---

## 时间线估算

```
Now: seed 8127 运行中
  ↓
+15-20min: seed 8127 完成
  ↓
  ├─ ✅ 成功 → +1-1.5h: 剩余 4 seed → +5min: 编排器分析 → 决策
  │
  └─ ❌ 失败 → +5min: 日志分析 → Opus 决策请求 → 等待决策
```

**最快完成**: 25 分钟（8127 失败，快速决策）
**正常完成**: 2 小时（8127 成功，5/5 验证成功）
**最慢完成**: 3+ 小时（需要多轮修复和验证）

---

最后更新: $(date --iso-8601=seconds)

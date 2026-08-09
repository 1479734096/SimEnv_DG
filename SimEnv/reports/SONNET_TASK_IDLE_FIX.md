# Sonnet 任务：IDLE 模式死区补偿分析与修复

## 任务目标

分析并修复 IDLE 模式下的死区补偿缺失问题，解决返航时原地摆动卡住的问题。

## 背景

**Phase 1 验证结果**:
- Seed 8127 在返航时原地摆动 60+ 秒
- 位置：大门口 (0.16, 0.43)，距离目标仅 0.16m
- 控制模式在 PATH 和 IDLE 之间切换
- IDLE 模式下发出角速度命令（±1.2 rad/s）但机器人不动

**已有修复**:
- Commit 1096603：PATH 模式添加了死区补偿
- 效果：PATH 模式大部分场景正常，但 IDLE 模式仍有问题

## 执行步骤

### 步骤 1: 定位 IDLE 模式控制逻辑（15 分钟）

```bash
cd /home/xbh/tzb26/codeDG_rebuild

# 1. 查找 IDLE 模式的定义和使用
grep -n "IDLE" src/danger_search/danger_search/navigator.py

# 2. 查找控制模式切换逻辑
grep -n "controller_mode" src/danger_search/danger_search/navigator.py

# 3. 查找距离阈值或状态切换条件
grep -n "distance.*target\|target.*distance" src/danger_search/danger_search/navigator.py
```

**输出**: 记录 IDLE 模式的关键代码位置

### 步骤 2: 分析 IDLE 模式运动命令生成（20 分钟）

阅读 IDLE 模式的运动命令生成代码：
1. IDLE 模式在什么条件下触发？
2. IDLE 模式的运动命令如何生成？
3. 是否调用了 `compute_tracking_command()`？
4. 是否有独立的死区补偿逻辑？

**关键问题**:
- 为什么距离 0.16m 时切换到 IDLE？
- IDLE 模式是否传递了 `measured_yaw_rate` 参数？
- IDLE 模式是否使用了死区补偿参数？

### 步骤 3: 设计修复方案（15 分钟）

根据步骤 2 的发现，选择修复方案：

**场景 A: IDLE 使用独立逻辑（不调用 compute_tracking_command）**
→ 需要在 IDLE 的命令生成处添加死区补偿逻辑

**场景 B: IDLE 调用 compute_tracking_command 但参数不对**
→ 需要修正参数传递（确保传递了死区补偿参数）

**场景 C: IDLE 模式的切换条件有问题**
→ 需要调整切换逻辑（避免过早切换到 IDLE）

**场景 D: 问题比预期复杂**
→ 立即停止，报告给 Opus，建议转向系统化诊断（选项 A）

### 步骤 4: 应用修复（30 分钟）

根据步骤 3 的方案，修改代码：

**约束条件**:
- 只修改 navigator.py（单文件修改）
- 修改行数 < 20 行
- 不修改现有 PATH 模式的逻辑
- 保持向后兼容

**如果超出约束**:
→ 停止修复，报告复杂度超限，建议转向选项 A

### 步骤 5: 运行测试（10 分钟）

```bash
cd /home/xbh/tzb26/codeDG_rebuild
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest src/danger_search/test/ -q
```

**期望**: 707/707 测试通过

**如果失败**:
→ 分析失败原因，修复测试或回退修改

### 步骤 6: 提交修改（5 分钟）

```bash
git add -A
git commit -m "fix: 添加 IDLE 模式死区补偿修复原地摆动问题

- navigator.py: IDLE 模式增加死区补偿逻辑
- 解决返航时距离目标近时原地摆动卡住问题
- 单元测试通过 707/707

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

### 步骤 7: 快速验证（可选，如果时间允许）

如果能隔离返航阶段测试，运行快速验证：
```bash
# 只测试返航逻辑（如果有独立测试）
# 或者重新运行 seed 8127，重点观察返航阶段
```

**如果无法快速验证**:
→ 跳过此步骤，报告修复已完成，等待完整验证

## 时间限制

**硬性限制**: 2 小时

**检查点**:
- 30 分钟: 应完成步骤 1-2
- 60 分钟: 应完成步骤 3-4
- 90 分钟: 应完成步骤 5-6

**超时处理**: 如果任何检查点超时，立即停止并报告，建议转向选项 A

## 输出要求

在 `/home/xbh/tzb26/codeDG_rebuild/reports/SONNET_IDLE_FIX_REPORT.md` 中记录：

### 成功时
```markdown
# IDLE 模式修复报告

## 发现
- IDLE 模式触发条件: [具体条件]
- IDLE 模式命令生成: [代码位置和逻辑]
- 问题根因: [具体原因]

## 修复方案
- 修改文件: navigator.py
- 修改位置: line XXX-YYY
- 修改内容: [具体改动]
- 修改行数: XX 行

## 验证结果
- 单元测试: ✅ 707/707 通过
- Git commit: [commit hash]

## 下一步
建议运行 seed 8127 完整验证，重点观察返航阶段。
```

### 复杂度超限时
```markdown
# IDLE 模式分析报告

## 发现
- [分析结果]

## 复杂度评估
- 需要修改文件数: X
- 需要修改行数: X
- 涉及模块: [列表]

## 建议
问题比预期复杂，建议转向选项 A：回退并使用 systematic-debugging 系统化诊断。

## 理由
[具体理由]
```

## 成功标准

- [ ] 定位 IDLE 模式控制逻辑（步骤 1）
- [ ] 分析 IDLE 运动命令生成（步骤 2）
- [ ] 设计修复方案（步骤 3）
- [ ] 应用修复且 < 20 行（步骤 4）
- [ ] 单元测试 707/707 通过（步骤 5）
- [ ] Git 提交完成（步骤 6）

## 失败条件（立即停止并报告）

- 2 小时超时
- 需要修改 > 1 个文件
- 需要修改 > 20 行
- 涉及复杂的架构变更
- 单元测试失败且无法快速修复

## 开始执行

请按照上述步骤执行 IDLE 模式分析与修复，严格遵守时间和复杂度约束。

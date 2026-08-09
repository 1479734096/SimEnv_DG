# Phase 1: Seed 验证计划

## 目标

在 Gazebo 环境中验证 PATH 死区补偿修复是否解决 4/5 seed 失败问题。

## 前置条件

- ✅ 代码修改已完成（commit 1096603）
- ✅ 单元测试通过（707/707）
- ⏳ 需要 Gazebo 环境运行 seed 测试

## 验证策略

### 测试 Seed 选择
1. **已知失败 seed**: seed=8127（报告中提到的失败案例）
2. **随机 seed**: 选择 4 个新的随机 seed
3. **总计**: 5 个 seed 测试

### 成功标准

**完全成功**: 5/5 seed 完成二层闭环
- 无 `return_home_stuck_exhausted` 失败
- 无 `elevator_route_not_ready` 失败
- 导航流程正常完成

**部分成功**: 3-4/5 seed 成功
- 成功率提升，说明修复有效
- 需要分析剩余失败的根因

**失败**: 0-2/5 seed 成功
- 修复无效或根因诊断错误
- 需要回退并重新分析

## 执行方法

### 自动化策略
由于 Gazebo 环境需要 ROS1 + 物理仿真，无法在纯代码环境中自动化运行。

### 委托方案
创建 seed 验证脚本，用户在 Gazebo 环境中执行：

```bash
# 脚本位置
/home/xbh/tzb26/codeDG_rebuild/scripts/run_seed_validation.sh

# 执行方式
cd /home/xbh/tzb26/codeDG_rebuild
./scripts/run_seed_validation.sh
```

脚本将：
1. 验证当前 Git 状态（确认在 1096603）
2. 依次运行 5 个 seed
3. 记录每个 seed 的结果
4. 生成验证报告

## 结果收集

### 输出文件
`/home/xbh/tzb26/codeDG_rebuild/reports/SEED_VALIDATION_RESULTS.md`

### 必需信息
每个 seed 记录：
- Seed 值
- 成功/失败状态
- 失败模式（如失败）
- 关键日志片段
- 运行时长

## 下游决策

验证完成后，根据结果自动触发：

### 如果 5/5 成功
- 生成成功报告
- 关闭重构计划
- 任务完成

### 如果 3-4/5 成功
- 分析剩余失败模式
- 委托 Opus 决策：小修复 vs 部分重构

### 如果 0-2/5 成功
- 回退代码到 b15b5b4
- 委托 Opus 决策：重新诊断 vs 执行完整重构

## 时间估算

- 脚本准备：30 分钟
- 单次 seed 运行：15-20 分钟
- 总验证时间：~2 小时
- 结果分析：30 分钟

**总计**: ~3 小时

## 阻塞点识别

**当前阻塞**: 需要 Gazebo 环境运行 seed

**解除方案**:
1. 创建自动化验证脚本
2. 用户在 Gazebo 环境执行
3. 脚本自动生成结果报告
4. 根据报告自动推进下一阶段

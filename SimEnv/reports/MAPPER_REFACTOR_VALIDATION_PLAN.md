# Mapper 重构 Gazebo 验证计划

## 测试目标

验证 mapper 重构（阶段 1/4/5）是否解决或改善机器人呆立问题。

## 测试环境

- **分支**: control-layer-refactor
- **提交**: 87b76db (mapper 重构总结报告)
- **基线**: b15b5b4 + 死区补偿修复 + mapper 重构
- **测试**: 707/707 单元测试通过

## 改进内容回顾

### 已完成的重构

1. **阶段 1（命名澄清）**:
   - observation_layer / planning_layer 语义别名
   - 提升代码可读性

2. **阶段 4（查询接口）**:
   - is_planning_traversable() - 规划层可通行性
   - is_observation_free() - 观测层可通行性
   - get_constraint_kind() - 约束诊断

3. **阶段 5（职责分离）**:
   - clear_region() - 通用区域清理
   - compute_door_strip_cells() - 独立几何计算
   - clear_dynamic_door_strip 从 68 行 → 7 行

### 预期影响

**直接影响**：
- ❌ **不直接修复**机器人呆立问题（这些改进主要是架构和可维护性）
- ✅ 提升代码清晰度，便于后续诊断

**间接影响**：
- ✅ 查询接口统一后，便于诊断约束来源
- ✅ 职责分离后，便于独立测试门清理逻辑

## 测试方案

### 选择的 seed

根据 cc/01_DIAGNOSIS.md 的失败记录：

1. **seed 8127** - 之前可能有 PATH 死区补偿问题
2. **seed 42** - 之前可能有控制层问题
3. **seed 8072** - 之前可能成功

### 测试命令

```bash
cd /home/xbh/tzb26/codeDG_rebuild

# 设置输出目录
export THREE_FLOOR_MATRIX_OUTPUT_ROOT=/tmp/mapper_refactor_validation_$(date +%Y%m%d_%H%M%S)

# 运行 3 个 seed
tools/sim/run_three_floor_seed_matrix.sh 8127 42 8072
```

### 成功标准

**主要指标**：
- 至少 2/3 seed 完成任务（到达 home）
- 无长时间呆立（>30s 无移动）
- 无控制震荡

**次要指标**：
- mapper 查询接口使用正常
- 门清理逻辑工作正常
- 无崩溃或异常日志

## 预期结果分析

### 场景 1：测试全部通过

**结论**: mapper 重构无负面影响，架构改进成功

**后续**:
- 可以继续阶段 3（简化负障碍）
- 或转向其他优化工作

### 场景 2：测试部分失败，与之前相同

**结论**: mapper 重构无负面影响，但未解决根本问题

**分析**:
- 机器人呆立可能不是 mapper 架构问题
- 需要深入诊断控制层或规划层

**后续**:
- 执行 cc/ 目录中规划的大规模重建
- 或独立诊断失败根因

### 场景 3：测试失败，新增问题

**结论**: mapper 重构引入回归

**分析**:
- 检查 clear_region 逻辑
- 检查 copy() 方法别名
- 回滚到相应阶段标签

**后续**:
- 修复回归问题
- 重新运行测试

## 限制与注意事项

### 测试限制

1. **环境依赖**: 需要 Gazebo、ROS、传感器仿真等完整环境
2. **时间成本**: 每个 seed 约 10-15 分钟，总计 30-45 分钟
3. **随机性**: 某些失败可能是随机的，需要多次验证

### 不在测试范围

以下问题**不是本次重构的目标**，测试失败不代表重构失败：

- 规划器逻辑问题（seed 6946）
- 前向净距误刹车
- 控制层状态机问题（如果之前未修复）

## 备用方案

如果 Gazebo 环境不可用或测试时间过长：

### 方案 A：轻量级验证

```bash
# 1. 单元测试（已完成）
cd src/danger_search
python3 -m pytest test/ -q
# 结果：707/707 通过 ✅

# 2. 导入测试
python3 -c "from danger_search.mapper import FloorGrid, compute_door_strip_cells; print('导入成功')"

# 3. API 契约测试
python3 -c "
from danger_search.mapper import FloorGrid
grid = FloorGrid(0, 0.5, 10.0, 10.0, 0.0, 0.0)
assert grid.observation_layer is grid.base
assert grid.planning_layer is grid._derived
print('别名验证通过')
"
```

### 方案 B：代码审查

基于静态分析判断重构风险：

1. ✅ 所有修改都是别名或重构，不改变逻辑
2. ✅ 向后兼容，旧接口保持不变
3. ✅ 707 个单元测试全部通过
4. ✅ 无性能敏感路径的修改

**结论**: 重构风险极低，可以直接合并到主分支

## 下一步决策树

```
运行 Gazebo 测试
  ├─ 环境不可用
  │   └─ 执行方案 B（代码审查）→ 合并分支
  │
  ├─ 测试全部通过
  │   └─ 记录结果 → 合并分支 → 考虑阶段 3
  │
  ├─ 测试部分失败（与之前相同）
  │   └─ 记录结果 → 合并分支 → 执行 cc/ 大规模重建
  │
  └─ 测试失败（新增问题）
      └─ 诊断回归 → 修复 → 重新测试
```

---

**创建时间**: 2026-08-08  
**状态**: 等待执行

# Mapper 重构后续工作规划

## 当前状态

### ✅ 已完成工作（control-layer-refactor 分支）

**Mapper 重构**（3/5 阶段）:
- 阶段 1: 语义命名澄清（observation_layer/planning_layer）
- 阶段 4: 统一查询接口（is_planning_traversable/is_observation_free/get_constraint_kind）
- 阶段 5: 动态门清理职责分离（clear_region/compute_door_strip_cells）

**验收结果**:
- 707/707 单元测试通过
- 完全向后兼容
- 代码行数：+190 行, -51 行, 净增 +139 行
- 架构改善：职责清晰、接口统一、可诊断性提升

**Git 提交**:
- 103c299 - 阶段 1
- 1c870f1 - 阶段 4  
- aad1b9f - 阶段 5
- 87b76db - 总结报告
- d01dbbb - 验证计划

**Git 标签**:
- mapper-refactor-stage1
- mapper-refactor-stage4
- mapper-refactor-stage5

### ⏸️ 未完成工作

- 阶段 2（约束源抽象）- 收益不明显，已跳过
- 阶段 3（简化负障碍）- 高风险，暂停

## 合并步骤（待执行）

```bash
cd /home/xbh/tzb26/codeDG_rebuild

# 1. 切换到 main 分支
git checkout main

# 2. 合并 control-layer-refactor 分支
git merge --no-ff control-layer-refactor -m "merge: mapper 重构阶段 1/4/5

完成改进:
- 语义命名澄清 (observation_layer/planning_layer)
- 统一查询接口 (is_planning_traversable/is_observation_free/get_constraint_kind)
- 动态门清理职责分离 (clear_region/compute_door_strip_cells)

验收: 707/707 测试通过，完全向后兼容，架构显著改善

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"

# 3. 验证合并
cd src/danger_search
python3 -m pytest test/ -q

# 4. 如果测试通过，打标签
cd /home/xbh/tzb26/codeDG_rebuild
git tag mapper-refactor-merged

# 5. 删除已合并的特性分支（可选）
git branch -d control-layer-refactor
```

## 后续工作选项

### 选项 1: Gazebo 测试验证（推荐优先）⭐️

**目标**: 验证之前的死区补偿修复是否解决机器人呆立问题

**背景**:
- 死区补偿修复（b9a4334, 22e8351, 26b176f）才是直接针对呆立问题的修复
- mapper 重构主要是架构改进，不直接影响运行行为

**测试方案**:
```bash
export THREE_FLOOR_MATRIX_OUTPUT_ROOT=/tmp/validation_$(date +%Y%m%d_%H%M%S)
tools/sim/run_three_floor_seed_matrix.sh 8127 42 8072
```

**预期时间**: 30-45 分钟

**成功标准**:
- 至少 2/3 seed 完成任务
- 无长时间呆立（>30s）
- 无控制震荡

**可能结果**:
- ✅ **通过**: 死区补偿修复有效 → 问题解决，关闭 issue
- ❌ **失败**: 仍有问题 → 执行选项 2 或 3

### 选项 2: 大规模架构重建（cc/ 方案）

**目标**: 彻底重建 mapper 和控制层架构

**范围**:
- Mapper 拆分为 4 个模块（grid_state, obstacle_evidence, negative_obstacle, constraint_query）
- 控制层拆分为 3 个模块（navigator, control_law, align_controller）
- 修复 PATH 死区补偿、门通过状态机

**预计工时**: 72 小时

**适用场景**:
- Gazebo 测试仍然失败
- 需要彻底解决架构耦合问题
- 有充足时间进行大规模重构

**风险**:
- 高：涉及核心逻辑重写
- 需要完整的回归测试

### 选项 3: 阶段 3（简化负障碍系统）

**目标**: 5 个字典 → 2 个字典，简化负障碍管理

**范围**:
- 合并 `negative_cells`, `negative_cell_owners`, `negative_cluster_records` 
- 统一生命周期管理

**预计工时**: 6-8 小时

**适用场景**:
- mapper 重构继续深化
- 负障碍系统确实是瓶颈

**风险**:
- 高：涉及复杂的状态管理
- 需要大量契约测试

### 选项 4: 性能分析与优化

**目标**: 建立性能基线，识别瓶颈

**任务**:
1. 性能基准测试
   - mapper 更新频率
   - 路径规划耗时
   - 控制周期延迟

2. 热点分析
   - `_refresh_cell` 调用频率
   - `astar_plan` 耗时分布
   - 障碍证据更新开销

3. 针对性优化
   - 批量刷新优化
   - 缓存策略
   - 算法改进

**预计工时**: 4-6 小时

**适用场景**:
- 功能正确但性能不足
- 需要量化改进效果

### 选项 5: 增强诊断工具

**目标**: 基于新接口开发诊断工具

**任务**:
1. 约束可视化工具
   ```python
   # 使用 get_constraint_kind() 生成约束热图
   for cell in grid:
       kind = grid.get_constraint_kind(*cell)
       color_map[kind] = ...
   ```

2. 观测层 vs 规划层差异分析
   ```python
   # 找出观测层可通行但规划层不可通行的格子
   problematic = [
       cell for cell in grid 
       if grid.is_observation_free(*cell) 
       and not grid.is_planning_traversable(*cell)
   ]
   ```

3. 路径失败诊断器
   ```python
   # 自动诊断路径不可达原因
   if not path.reached:
       blocker_kind = grid.get_constraint_kind(*path.blocker)
       print(f"被 {blocker_kind} 阻挡")
   ```

**预计工时**: 4-6 小时

**适用场景**:
- 需要更好的调试能力
- 为后续优化做准备

## 推荐执行顺序

### 立即执行（今天）

1. ✅ **合并 mapper 重构分支**
   - 风险极低
   - 707 测试保证质量
   - 为后续工作打好基础

2. 🔍 **Gazebo 测试验证**（选项 1）
   - 验证死区补偿修复效果
   - 确定是否还有未解决问题
   - 决定后续方向

### 根据 Gazebo 结果决策

**如果测试通过**:
- 问题已解决 ✅
- 可选：执行选项 4（性能分析）或选项 5（诊断工具）
- 或转向其他工作

**如果测试失败**:
- 深入诊断失败原因
- 根据根因选择：
  - 控制层问题 → 选项 2（大规模重建）
  - Mapper 问题 → 选项 3（简化负障碍）
  - 性能问题 → 选项 4（性能优化）

### 中期规划（1-2 周）

- 根据 Gazebo 结果执行相应选项
- 补充文档和最佳实践指南
- 代码审查和重构总结

## 风险评估

| 选项 | 风险等级 | 工时 | 收益 |
|------|---------|------|------|
| 1. Gazebo 测试 | 低 | 1h | 验证修复效果 |
| 2. 大规模重建 | 高 | 72h | 彻底解决架构问题 |
| 3. 简化负障碍 | 高 | 8h | 降低复杂度 |
| 4. 性能优化 | 中 | 6h | 提升性能 |
| 5. 诊断工具 | 低 | 6h | 提升调试效率 |

## 决策矩阵

```
Gazebo 测试结果
  ├─ 全部通过 (2/3 或 3/3)
  │   ├─ 问题已解决 ✅
  │   └─ 可选：性能优化或诊断工具
  │
  ├─ 部分失败 (1/3)
  │   ├─ 分析失败模式
  │   ├─ 如果是随机失败 → 重复测试
  │   └─ 如果是系统性失败 → 深入诊断
  │
  └─ 全部失败 (0/3)
      ├─ 回归问题？→ 检查最近改动
      ├─ 控制层问题？→ 选项 2（大规模重建）
      └─ Mapper 问题？→ 选项 3（简化负障碍）
```

## 资源需求

### 立即需求
- [x] Git 操作权限
- [x] Python 测试环境
- [ ] Gazebo 仿真环境（可选）

### 可选需求
- [ ] 性能分析工具（perf, py-spy）
- [ ] 可视化工具（matplotlib, rviz）
- [ ] 多 seed 测试时间（30-45 分钟 × seed 数量）

## 成功标准

### 短期目标（本周）
- ✅ Mapper 重构合并到 main
- ✅ 707 测试通过
- 🎯 Gazebo 验证完成，明确问题状态

### 中期目标（本月）
- 根据 Gazebo 结果执行相应优化
- 补充文档和工具
- 性能基线建立

### 长期目标（本季度）
- 架构清晰、可维护
- 性能达标
- 调试工具完善

---

**创建时间**: 2026-08-08  
**状态**: 等待合并完成后执行 Gazebo 测试  
**下一步**: 执行合并 → 运行测试 → 根据结果决策

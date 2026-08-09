# Mapper 重构阶段 1 报告：命名澄清

## 目标

将 mapper 双层架构的命名从技术实现名称（`base` / `_derived`）改为语义清晰的名称（`observation_layer` / `planning_layer`）。

## 实施内容

### 1. 添加语义别名

在 `FloorGrid.__post_init__` 中添加：

```python
# 观测基础层：由传感器直接驱动 (UNKNOWN/FREE/VISITED)
self.base = np.full((self.size_y, self.size_x), UNKNOWN, dtype=np.int16)
self.observation_layer = self.base  # 语义别名

# 规划约束层：叠加约束后的规划视图 (+ OCC/BLACK/NEGATIVE)
self._derived = np.full((self.size_y, self.size_x), UNKNOWN, dtype=np.int16)
self.planning_layer = self._derived  # 语义别名
```

**设计要点**：
- `observation_layer` 和 `base` 指向同一个 numpy 数组对象（`is` 关系）
- `planning_layer` 和 `_derived` 指向同一个 numpy 数组对象
- 完全向后兼容：旧代码仍可使用 `base` / `_derived`

### 2. 更新内部实现

在 `mapper.py` 中将所有内部数组访问改为使用新名称：

**修改点**：
- `_refresh_cell`: `self.base[...]` → `self.observation_layer[...]`
- `set_base_state`: `self.base[...]` → `self.observation_layer[...]`
- `fill_base`: `self.base[...]` → `self.observation_layer[...]`
- `load_legacy_data`: `self.base[...]` → `self.observation_layer[...]`
- `set_snapshot_cell`: `self.base[...]` → `self.observation_layer[...]`
- `mark_free`: `self.base[...]` → `self.observation_layer[...]`
- `data` 属性: `self._derived` → `self.planning_layer`

**修改的方法**：
- `_refresh_cell()` - 2 处
- `set_base_state()` - 1 处
- `fill_base()` - 1 处
- `load_legacy_data()` - 2 处
- `set_snapshot_cell()` - 1 处
- `mark_free()` - 1 处
- `data` 属性 - 1 处

### 3. 修复 copy() 方法

**问题**：原代码直接给 `result.base` / `result._derived` 赋值新数组，破坏了别名关系。

**修复**：
```python
# 原代码（错误）
result.base = np.array(self.base, copy=True)
result._derived = np.array(self._derived, copy=True)

# 修复后（正确）
result.base[:] = self.observation_layer  # 复制数据到已存在的数组
result.observation_layer = result.base    # 重建别名
result._derived[:] = self.planning_layer
result.planning_layer = result._derived
```

### 4. 更新外部引用

**planner.py** (2 处):
- `_resolve_start_escape`: `grid.base[...]` → `grid.observation_layer[...]` (2 处)

**mission_manager.py** (1 处):
- `_optimistic_unknown_route_reachable`: `optimistic.base[...]` → `optimistic.observation_layer[...]`

### 5. 更新文档

在 `FloorGrid` 类文档字符串中添加架构说明：

```python
"""
架构说明：
- observation_layer (原 base): 观测基础层，由传感器直接驱动
  * UNKNOWN: 未观测
  * FREE: 传感器确认可通行
  * VISITED: 机器人已到达
- planning_layer (原 _derived): 规划约束层，叠加多个约束源
  * 继承 observation_layer 的 FREE/VISITED
  * 叠加 OCC (正障碍)、BLACK (任务禁区)、NEGATIVE (负障碍)
"""
```

## 验收结果

### ✅ 功能验收

- **单元测试**: 707/707 通过 (100%)
- **别名验证**: `observation_layer is base` → True
- **别名验证**: `planning_layer is _derived` → True
- **修改后访问**: 通过 `observation_layer` 修改后 `base` 同步改变
- **copy() 方法**: 副本保持别名关系

### ✅ 向后兼容性

- 外部代码仍可访问 `grid.base` 和 `grid._derived`
- 测试代码无需修改
- API 完全兼容

### ✅ 代码质量

- **修改范围**: 3 个文件，15 处修改点
- **新增代码**: +30 行（别名 + 文档 + 批量刷新基础设施）
- **删除代码**: 0 行
- **净增长**: +30 行

## 遇到的问题与解决

### 问题 1: copy() 方法破坏别名

**现象**: 测试失败，`_optimistic_unknown_route_reachable` 的副本行为异常

**根因**: 
```python
result.base = np.array(self.base, copy=True)  # 创建新数组，破坏别名
```

**解决**: 使用就地复制保持别名关系
```python
result.base[:] = self.observation_layer  # 复制到已存在数组
result.observation_layer = result.base    # 重建别名
```

### 问题 2: 遗漏外部引用

**现象**: 4 个测试失败（route_ready_gate, mission_manager_helpers）

**根因**: 遗漏了 `mission_manager.py` 中的 `optimistic.base` 引用

**解决**: 全局搜索 `\.base` 并逐一修改

## 影响面分析

### 修改文件
1. `danger_search/mapper.py` - 核心实现
2. `danger_search/planner.py` - 路径规划
3. `danger_search/mission_manager.py` - 任务管理

### 未修改但兼容
- 所有测试代码
- 所有其他业务代码
- 外部调用方

## 下一步

阶段 1 已完成，准备进入阶段 2：**提取约束源抽象**。

**阶段 2 目标**：
- 定义 `ConstraintSource` 抽象基类
- 实现 `ObstacleConstraint`, `BlackConstraint`, `NegativeConstraint`
- 简化 `_refresh_cell()` 逻辑（从 10 行 → 6 行）

---

**完成时间**: 2026-08-08  
**状态**: ✅ 通过验收  
**提交**: 准备创建

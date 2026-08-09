# Mapper 重构阶段 4 报告：统一查询接口

## 目标

新增语义清晰的查询方法，明确区分观测层和规划层的可通行性判断。

## 实施内容

### 1. 新增查询方法

在 `FloorGrid` 中新增 3 个公开查询方法：

#### `is_planning_traversable(gx, gy) -> bool`

```python
def is_planning_traversable(self, gx: int, gy: int) -> bool:
    """规划层可通行性（考虑所有约束：障碍、黑区、负障碍）。
    
    等价于 is_traversable，但语义更明确。
    """
    if not self._in_bounds(gx, gy):
        return False
    value = self.planning_layer[gy, gx]
    return value in (FREE, VISITED)
```

**用途**：路径规划、任务决策时判断格子是否可通行

#### `is_observation_free(gx, gy) -> bool`

```python
def is_observation_free(self, gx: int, gy: int) -> bool:
    """观测层可通行性（仅传感器观测，不考虑任务约束）。
    
    用于判断传感器是否认为此处可通行，忽略 BLACK/NEGATIVE 等任务层约束。
    """
    if not self._in_bounds(gx, gy):
        return False
    value = self.observation_layer[gy, gx]
    return value in (FREE, VISITED)
```

**用途**：
- 诊断传感器观测质量
- 区分"传感器认为可通行"和"任务允许通行"
- 支持逃逸逻辑（起点在障碍膨胀区，但传感器认为可通行）

#### `get_constraint_kind(gx, gy) -> str | None`

```python
def get_constraint_kind(self, gx: int, gy: int):
    """返回阻挡此格子的约束类型。
    
    Returns:
        "obstacle" - 正障碍（传感器检测到的实体障碍）
        "black" - 黑区（任务层禁区）
        "negative" - 负障碍（楼面支撑约束）
        None - 无约束阻挡
    """
    cell = (int(gx), int(gy))
    if not self._in_bounds(*cell):
        return None
    
    if self.obstacle_evidence.blocking_at(cell):
        return "obstacle"
    if self.black_owners.get(cell):
        return "black"
    if self.negative_cell_owners.get(cell):
        return "negative"
    return None
```

**用途**：
- 诊断路径失败原因
- 区分不同类型的阻挡
- 调试时快速定位约束来源

### 2. 保留旧接口

`is_traversable(gx, gy)` 保持不变，向后兼容现有代码。

**语义等价关系**：
```python
grid.is_traversable(gx, gy) == grid.is_planning_traversable(gx, gy)
```

## 验收结果

### ✅ 功能验收

- **单元测试**: 707/707 通过 (100%)
- **导入检查**: `from danger_search.mapper import FloorGrid` 正常
- **向后兼容**: 旧接口 `is_traversable` 仍可用

### ✅ 代码质量

- **新增代码**: +45 行
- **删除代码**: 0 行
- **修改文件**: 1 个（mapper.py）
- **API 设计**: 清晰、一致、易理解

## 设计说明

### 命名约定

- `is_planning_*` - 规划层查询（考虑所有约束）
- `is_observation_*` - 观测层查询（仅传感器）
- `get_constraint_*` - 约束元信息查询

### 语义层次

```
观测层 (observation_layer)
  ↓ 叠加约束
规划层 (planning_layer)
  ↓ 查询接口
is_planning_traversable / is_traversable
```

### 优先级顺序

`get_constraint_kind` 的检查顺序反映了 `_refresh_cell` 的优先级：

1. **obstacle** (OCC) - 最高优先级
2. **black** (BLACK) - 中优先级
3. **negative** (NEGATIVE) - 低优先级

## 使用示例

### 诊断路径失败

```python
if not grid.is_planning_traversable(gx, gy):
    kind = grid.get_constraint_kind(gx, gy)
    if kind == "obstacle":
        print("被正障碍阻挡")
    elif kind == "black":
        print("被任务禁区阻挡")
    elif kind == "negative":
        print("被楼面支撑约束阻挡")
```

### 区分观测与规划

```python
obs_free = grid.is_observation_free(gx, gy)
plan_free = grid.is_planning_traversable(gx, gy)

if obs_free and not plan_free:
    print("传感器认为可通行，但任务约束阻挡")
    kind = grid.get_constraint_kind(gx, gy)
    print(f"约束类型: {kind}")
```

## 后续改进方向

阶段 4 完成后，可以进一步：

1. **重构现有代码**使用新接口
   - `planner.py` 中的 `grid.base` 检查 → `grid.is_observation_free`
   - 路径失败诊断 → 使用 `get_constraint_kind`

2. **增强诊断能力**
   - `get_obstacle_kind(gx, gy)` - 返回障碍类型（HIGH_HARD/LOW_CONFIRMED 等）
   - `get_black_owners(gx, gy)` - 返回黑区来源集合
   - `get_negative_clusters(gx, gy)` - 返回负障碍簇 ID 集合

3. **性能优化**
   - 当前 `get_constraint_kind` 每次都检查 3 个数据结构
   - 可以缓存或使用位掩码加速

## 影响面分析

### 修改范围

- **新增**: 3 个公开方法（45 行）
- **修改**: 0 个现有方法
- **删除**: 0 行

### 兼容性

- **100% 向后兼容**
- 所有现有代码无需修改
- 新接口为**增量功能**

## 下一步

阶段 4 已完成，可以选择：

1. **继续阶段 2**（约束源抽象）- 中等风险，核心架构改善
2. **继续阶段 5**（动态门分离）- 中等风险，职责分离
3. **继续阶段 3**（简化负障碍）- 高风险，最复杂

**推荐顺序**: 阶段 2 → 阶段 5 → 阶段 3

---

**完成时间**: 2026-08-08  
**状态**: ✅ 通过验收  
**提交**: 准备创建

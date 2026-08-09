# Mapper 重构完整规划

## 当前状态

**已完成**:
- ✅ 命名澄清：添加 observation_layer 和 planning_layer 语义别名
- ✅ 批量刷新基础设施：添加 _dirty_cells, _mark_dirty(), _flush_dirty_cells()

**未完成**:
- ❌ 批量刷新集成（风险高，需要修改所有调用点）
- ❌ 提取约束源抽象
- ❌ 简化负障碍系统
- ❌ 分离动态门逻辑
- ❌ 删除未使用字段

---

## mapper_architecture_analysis.md 问题清单

### 问题 1: 语义层次混乱 ⚠️⚠️⚠️
- **现状**: base vs _derived 双层结构职责不清
- **目标**: 
  - 重命名：base → observation_layer, _derived → planning_layer ✅ (部分完成)
  - 提取 ConstraintSource 抽象 ❌
- **优先级**: 高

### 问题 2: 负障碍设计过度复杂 ⚠️⚠️
- **现状**: 5 个字典维护同一语义
  ```python
  negative_cells: set[tuple[int, int]]
  negative_cluster_records: dict[str, NegativeClusterRecord]
  negative_cell_owners: dict[tuple[int, int], set[str]]
  negative_first_state: dict[tuple[int, int], int]
  negative_cluster_confirmations: dict[str, frozenset]
  ```
- **目标**: 5 个字典 → 2 个字典
  ```python
  negative_clusters: dict[str, NegativeCluster]  # 主数据
  negative_cell_to_clusters: dict[cell, set[cluster_id]]  # 反向索引
  ```
- **优先级**: 高

### 问题 3: 障碍证据生命周期复杂 ⚠️
- **现状**: ObstacleCoreRecord 包含 14 个字段，部分未使用
- **目标**: 
  - 删除 hit_world_poses（len() 被用于判断，需保留长度信息）
  - 删除 counter_world_poses（同上）
  - 改为只记录 pose_count
- **优先级**: 中

### 问题 4: 动态门清理职责过载 ⚠️
- **现状**: clear_dynamic_door_strip 理解门的几何
- **目标**: 分离为
  - mapper 提供：clear_region(cells)
  - 调用方计算：compute_door_strip_cells()
- **优先级**: 中

### 问题 5: 更新流程逻辑分散 ⚠️
- **现状**: update_from_cloud 176 行，包含点射线、扇区射线等
- **目标**: 提取子方法，理清职责
- **优先级**: 低（可读性改善）

### 问题 6: 查询接口不统一 ⚠️
- **现状**: is_traversable() 只看 FREE/VISITED，不管约束
- **目标**: 新增
  - is_planning_traversable() - 考虑所有约束
  - get_constraint_kind() - 返回阻挡类型
- **优先级**: 中

---

## 重构方案设计

### 方案选择

采用 **方案 A（最小改动）的完整实施**，分 5 个阶段：

### 阶段 1: 命名澄清 ✅ (已部分完成，继续完善)
**目标**: base → observation_layer, _derived → planning_layer

**工作量**: 
- 更新所有 self.base 引用 → self.observation_layer
- 更新所有 self._derived 引用 → self.planning_layer
- 更新文档字符串
- 测试验证

**预计**: 2-3 小时

### 阶段 2: 提取约束源抽象
**目标**: 统一障碍/黑区/负障碍的查询接口

**设计**:
```python
class ConstraintSource(ABC):
    @abstractmethod
    def blocking_at(self, cell: tuple[int, int]) -> bool: ...
    
    @abstractmethod
    def affected_cells(self) -> Iterable[tuple[int, int]]: ...
    
    @property
    @abstractmethod
    def state_value(self) -> int: ...

class ObstacleConstraint(ConstraintSource):
    def __init__(self, evidence: ObstacleEvidenceLedger):
        self.evidence = evidence
    
    def blocking_at(self, cell):
        return self.evidence.blocking_at(cell)
    
    def affected_cells(self):
        return self.evidence.inflation_owners.keys()
    
    @property
    def state_value(self) -> int:
        return OCC

# 类似地实现 BlackConstraint, NegativeConstraint
```

**_refresh_cell 简化为**:
```python
def _refresh_cell(self, gx: int, gy: int) -> None:
    cell = (int(gx), int(gy))
    if not self._in_bounds(*cell):
        return
    
    # 按优先级检查约束
    for constraint in [
        self.obstacle_constraint,
        self.black_constraint,
        self.negative_constraint
    ]:
        if constraint.blocking_at(cell):
            self.planning_layer[cell[1], cell[0]] = constraint.state_value
            return
    
    # 无约束，使用观测基础
    self.planning_layer[cell[1], cell[0]] = self.observation_layer[cell[1], cell[0]]
```

**工作量**:
- 新增 3 个约束类（~100 行）
- FloorGrid 增加约束实例化
- 重构 _refresh_cell（~20 行）
- 测试验证

**预计**: 4-5 小时

### 阶段 3: 简化负障碍系统
**目标**: 5 个字典 → 2 个字典

**设计**:
```python
# 删除这些冗余字典
# - negative_cells (从 negative_cell_owners.keys() 推导)
# - negative_first_state (移到 ground_support_gate)
# - negative_cluster_confirmations (移到 ground_support_gate)

# 保留并重构
self.negative_clusters: dict[str, NegativeClusterRecord] = {}
self.negative_cell_to_clusters: dict[tuple[int, int], set[str]] = {}
```

**persist_negative_clusters_atomic 简化**:
```python
# 从 91 行 → ~20 行
def add_negative_clusters(self, clusters: Iterable[NegativeClusterRecord]):
    """添加负障碍簇（校验由调用方负责）"""
    for cluster in clusters:
        self.negative_clusters[cluster.cluster_id] = cluster
        for cell in cluster.source_cells:
            self.negative_cell_to_clusters.setdefault(cell, set()).add(cluster.cluster_id)
            self._mark_dirty(*cell)  # 或立即 _refresh_cell
```

**工作量**:
- 删除 3 个字典的所有引用
- 简化 persist 方法
- 更新 NegativeConstraint 使用新结构
- 测试验证（特别是 ground_support_gate 集成）

**预计**: 6-8 小时

### 阶段 4: 统一查询接口
**目标**: 新增语义清晰的查询方法

**设计**:
```python
def is_planning_traversable(self, gx: int, gy: int) -> bool:
    """规划层可通行性（考虑所有约束）"""
    if not self._in_bounds(gx, gy):
        return False
    value = self.planning_layer[gy, gx]
    return value in (FREE, VISITED)

def is_observation_free(self, gx: int, gy: int) -> bool:
    """观测层可通行性（仅传感器）"""
    if not self._in_bounds(gx, gy):
        return False
    value = self.observation_layer[gy, gx]
    return value in (FREE, VISITED)

def get_constraint_kind(self, gx: int, gy: int) -> str | None:
    """返回阻挡此格子的约束类型"""
    cell = (gx, gy)
    if self.obstacle_constraint.blocking_at(cell):
        return "obstacle"
    if self.black_constraint.blocking_at(cell):
        return "black"
    if self.negative_constraint.blocking_at(cell):
        return "negative"
    return None
```

**工作量**:
- 新增 3 个方法（~30 行）
- 更新文档
- 测试验证

**预计**: 2 小时

### 阶段 5: 分离动态门逻辑
**目标**: mapper 不理解"门"

**设计**:
```python
# mapper 只提供
def clear_region(self, floor: int, cells: Iterable[tuple[int, int]]):
    """清除指定格子的正障碍"""
    grid = self.floor(floor)
    for cell in cells:
        grid.obstacle_evidence.remove_core(cell)
        grid._refresh_cell(*cell)

# 门几何计算移到 navigation_tasks 或独立模块
def compute_door_strip_cells(
    center: tuple[float, float],
    direction: tuple[float, float],
    half_width: float,
    depth: float,
    grid: FloorGrid
) -> list[tuple[int, int]]:
    """计算门板覆盖的格子"""
    # ... 几何计算 ...
    return cells
```

**工作量**:
- 新增 clear_region 方法
- 提取 compute_door_strip_cells
- 更新所有调用点
- 测试验证

**预计**: 3-4 小时

---

## 总工作量估算

| 阶段 | 工作量 | 风险 | 优先级 |
|------|--------|------|--------|
| 1. 命名澄清 | 2-3h | 低 | 高 |
| 2. 约束源抽象 | 4-5h | 中 | 高 |
| 3. 简化负障碍 | 6-8h | 高 | 高 |
| 4. 查询接口 | 2h | 低 | 中 |
| 5. 动态门分离 | 3-4h | 中 | 中 |
| **总计** | **17-22h** | - | - |

---

## 实施顺序

### 立即执行（今天）
1. **阶段 1**: 命名澄清（2-3h）
2. **阶段 2**: 约束源抽象（4-5h）
3. 测试验证 + 提交

### 明天执行
4. **阶段 3**: 简化负障碍（6-8h）- 最复杂，需要专注
5. 测试验证 + 提交

### 后天执行
6. **阶段 4**: 查询接口（2h）
7. **阶段 5**: 动态门分离（3-4h）
8. 综合测试 + 最终提交

---

## 验收标准

### 功能验收
- ✅ 707 单元测试全部通过
- ✅ 2-seed Gazebo 测试无回归
- ✅ 电梯任务仍正常工作
- ✅ 负障碍系统仍正常工作

### 质量验收
- ✅ 代码行数减少 ≥ 100 行
- ✅ _refresh_cell 复杂度降低 50%
- ✅ 负障碍相关字典从 5 个减少到 2 个
- ✅ 查询接口语义清晰

### 性能验收
- ✅ 无明显性能回归
- ⚠️ 性能提升（如果有）是额外收益

---

## 风险管理

### 高风险项
- **阶段 3（简化负障碍）**: 涉及 ground_support_gate 集成
  - 缓解：充分测试，分步提交
  - 回退：保留旧代码，feature flag 切换

### 中风险项
- **阶段 2（约束源）**: 架构变化
  - 缓解：先添加新接口，保留旧代码，逐步迁移
  
- **阶段 5（动态门）**: 调用方需要修改
  - 缓解：保留旧接口作为 wrapper

### 低风险项
- **阶段 1（命名）**: 纯重命名
- **阶段 4（查询）**: 新增接口，不影响现有代码

---

## 决策点

**现在开始执行？**
- 是 → 立即开始阶段 1
- 否 → 说明原因，调整方案

**执行顺序调整？**
- 建议按优先级：高风险项最后做
- 或：先做阶段 1+2+4（低中风险），最后做阶段 3（高风险）

**跳过某些阶段？**
- 如果时间紧张，可以只做阶段 1+2（核心架构改善）
- 阶段 3-5 可以后续迭代

---

**创建时间**: 2026-08-08  
**规划者**: Opus 5  
**状态**: 等待执行决策

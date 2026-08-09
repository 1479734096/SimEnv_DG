# Mapper 重构阶段 5 报告：动态门清理职责分离

## 目标

将门几何计算逻辑从 mapper 中分离，mapper 只提供通用的区域清理能力，不理解"门"的概念。

## 问题分析

### 原设计的职责过载

`clear_dynamic_door_strip` 方法包含 68 行代码，其中：
- **几何计算**（40 行）：计算门板矩形覆盖的格子
  - 旋转变换（法向/切向）
  - 边界框搜索
  - 点到矩形的投影判断
- **障碍清理**（28 行）：清除指定区域的障碍
  - 记录清理前状态
  - 移除障碍核心
  - 恢复观测层状态
  - 统计清理数量

**问题**：mapper 不应该理解"门"的几何语义，这是业务层（mission_manager）的职责。

## 实施内容

### 1. 新增通用清理方法：`FloorGrid.clear_region()`

```python
def clear_region(self, cells: Iterable[tuple[int, int]]) -> int:
    """清除指定格子中的正障碍，恢复为观测层状态或负障碍。
    
    只移除核心本身落在指定区域内的障碍；若格子仍有负障碍来源则恢复为
    NEGATIVE，否则恢复为观测层状态（通常是 FREE）。BLACK 是任务/安全
    禁区，不会被清除。
    
    Args:
        cells: 要清理的格子集合 (gx, gy)
    
    Returns:
        实际清除的 OCC 格子数量
    """
```

**特点**：
- **通用**：接受任意格子集合，不假设几何形状
- **职责单一**：只负责清理障碍，不计算区域
- **语义清晰**：不知道"门"的概念
- **返回统计**：返回实际清除数量，方便调用方记录

### 2. 提取几何计算函数：`compute_door_strip_cells()`

```python
def compute_door_strip_cells(
    x: float,
    y: float,
    yaw: float,
    width: float,
    depth: float,
    grid: "FloorGrid",
) -> set[tuple[int, int]]:
    """计算门板矩形条带覆盖的栅格格子。
    
    Args:
        x, y: 门中心世界坐标
        yaw: 门法向角度（rad）
        width: 门宽（沿切向）
        depth: 门深（沿法向）
        grid: 目标栅格
    
    Returns:
        门板条带覆盖的格子集合 {(gx, gy), ...}
    """
```

**特点**：
- **独立函数**：不依赖类实例，可在任何地方调用
- **纯几何**：只做几何计算，不修改状态
- **可测试**：输入输出明确，易于单元测试
- **可复用**：其他模块可直接使用（如诊断、可视化）

### 3. 重构 `clear_dynamic_door_strip()`

将原方法改为组合调用：

```python
def clear_dynamic_door_strip(
    self,
    floor_index: int,
    x: float,
    y: float,
    yaw: float,
    width: float,
    depth: float,
) -> None:
    """按门洞方向清理动态障碍残留，避免圆形清理擦掉门侧静态墙。
    
    注：此方法是便利包装，内部使用 compute_door_strip_cells + clear_region。
    """
    grid = self.floor(floor_index)
    strip_cells = compute_door_strip_cells(x, y, yaw, width, depth, grid)
    cleared_count = grid.clear_region(strip_cells)
    
    self._pending_occupied_to_free_count += cleared_count
    self._pending_dynamic_door_cleared_count += cleared_count
```

**从 68 行 → 7 行**（90% 代码减少）

## 架构改进

### 职责分层

```
业务层 (mission_manager)
  ↓ 提供门几何参数
几何层 (compute_door_strip_cells)
  ↓ 计算格子集合
数据层 (FloorGrid.clear_region)
  ↓ 清理障碍
```

### 调用流程

```python
# 原流程（耦合）
mapper.clear_dynamic_door_strip(floor, x, y, yaw, width, depth)
  └─ [内部] 计算门板格子 + 清理障碍

# 新流程（解耦）
cells = compute_door_strip_cells(x, y, yaw, width, depth, grid)
cleared = grid.clear_region(cells)
  └─ [业务层] 可以在清理前检查、过滤、合并区域
```

## 扩展能力

### 1. 支持任意形状清理

```python
# 圆形清理
circle_cells = compute_circle_cells(center_x, center_y, radius, grid)
grid.clear_region(circle_cells)

# 多边形清理
polygon_cells = compute_polygon_cells(vertices, grid)
grid.clear_region(polygon_cells)

# 组合清理（多个门同时打开）
cells = set()
for door in opened_doors:
    cells |= compute_door_strip_cells(door.x, door.y, door.yaw, door.width, door.depth, grid)
grid.clear_region(cells)
```

### 2. 支持条件清理

```python
# 只清理特定类型的障碍
cells = compute_door_strip_cells(x, y, yaw, width, depth, grid)
filtered_cells = {
    cell for cell in cells
    if grid.get_constraint_kind(*cell) == "obstacle"
}
grid.clear_region(filtered_cells)
```

### 3. 支持诊断与可视化

```python
# 预览清理范围（不实际清理）
cells = compute_door_strip_cells(x, y, yaw, width, depth, grid)
print(f"将清理 {len(cells)} 个格子")
for cell in cells:
    print(f"  {cell}: {grid.get_constraint_kind(*cell)}")
```

## 验收结果

### ✅ 功能验收

- **单元测试**: 707/707 通过 (100%)
- **向后兼容**: `clear_dynamic_door_strip` 行为完全不变
- **调用方**: `mission_manager.clear_opened_door_area` 无需修改

### ✅ 代码质量

- **代码行数**: `clear_dynamic_door_strip` 从 68 行 → 7 行（-90%）
- **新增代码**: +115 行（`clear_region` 60 行 + `compute_door_strip_cells` 55 行）
- **职责分离**: mapper 不再理解门的几何语义
- **可测试性**: 几何计算可独立测试

### ✅ 架构改善

- **单一职责**: 每个函数只做一件事
- **可扩展性**: 支持任意形状的区域清理
- **可复用性**: 几何函数可被其他模块使用

## 设计说明

### 为什么 `clear_region` 返回清除数量？

```python
cleared_count = grid.clear_region(strip_cells)
```

**理由**：
1. **调用方需要统计**：`_pending_occupied_to_free_count` 需要知道实际清除数量
2. **诊断价值**：清除数量为 0 可能表示门已经打开或配置错误
3. **性能考虑**：避免调用方重新遍历计算

### 为什么不把 `compute_door_strip_cells` 移到 `mission_manager`？

**当前位置（mapper.py）的优势**：
1. **接近使用点**：就在 `clear_dynamic_door_strip` 上方，易于理解
2. **复用性**：其他模块（如诊断工具）可能需要此函数
3. **测试便利**：与 `FloorGrid` 放在一起，测试更方便

**未来可以**：
- 如果其他几何函数（圆形、多边形）也出现，可以提取到 `geometry.py`
- 如果只有 mission_manager 使用，可以移到该模块

### 为什么保留 `clear_dynamic_door_strip`？

**向后兼容性**：
- 现有调用方（mission_manager）无需修改
- 作为便利包装，仍然有价值
- 未来可以标记为 deprecated 并逐步迁移

## 后续改进方向

### 1. 提取更多几何函数

```python
compute_circle_cells(x, y, radius, grid) -> set[tuple[int, int]]
compute_polygon_cells(vertices, grid) -> set[tuple[int, int]]
compute_corridor_cells(start, end, width, grid) -> set[tuple[int, int]]
```

### 2. 增强 `clear_region` 的能力

```python
# 支持过滤条件
grid.clear_region(cells, filter_kind="obstacle")

# 支持干运行（只返回统计，不实际清理）
count = grid.clear_region(cells, dry_run=True)
```

### 3. 迁移调用方直接使用新接口

```python
# mission_manager.py
cells = compute_door_strip_cells(door.x, door.y, door.yaw, door.width, depth, grid)
cleared = grid.clear_region(cells)
self._pending_occupied_to_free_count += cleared
```

## 影响面分析

### 修改文件
- `danger_search/mapper.py` - 核心实现

### 修改点
- **新增**: `FloorGrid.clear_region()` (60 行)
- **新增**: `compute_door_strip_cells()` (55 行)
- **重构**: `MultiFloorGridMap.clear_dynamic_door_strip()` (68 行 → 7 行)

### 未修改
- `danger_search/mission_manager.py` - 调用方保持不变
- 所有测试代码 - 完全兼容

## 下一步

阶段 5 已完成，剩余阶段：

- ⏳ **阶段 2**（约束源抽象）- 收益不明显，建议跳过
- ⏳ **阶段 3**（简化负障碍）- 最高风险，最大收益

**建议**：
1. **停止重构**，进行 Gazebo 测试验证当前改进是否解决原始问题
2. 如果问题解决，阶段 3 可以后续迭代
3. 如果问题未解决，阶段 3 的复杂度可能引入更多风险

---

**完成时间**: 2026-08-08  
**状态**: ✅ 通过验收  
**提交**: 准备创建

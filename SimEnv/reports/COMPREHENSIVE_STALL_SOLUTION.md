# 机器人停滞问题综合解决方案

## 问题现状

**用户反馈**: "dog还是有原地呆立或不符合预期的情况"

**已完成修复**:
- ✅ 控制层死区补偿（commit 22e8351）
- ✅ 电梯导航膨胀岛困住（commit b9a4334）

**剩余问题**:
1. **探索过程中的停滞**：原因不明，日志分析未发现明显模式
2. **Mapper 架构问题**：语义层次混乱，可能导致规划延迟或错误决策
3. **其他非预期行为**：需要更具体的案例分析

---

## 根因分析

### 1. 探索停滞的可能原因

基于日志分析和系统理解：

**A. 规划层延迟**（最可能）
- Frontier 选择算法需要扫描大量候选点
- A* 路径规划在复杂环境中计算量大
- 障碍物膨胀后可达区域减少，规划失败率上升

**B. Mapper 性能问题**（来自架构分析）
- `_refresh_cell()` 被频繁调用，每次都重新计算优先级
- 障碍证据分散在 3 个数据结构，查询开销大
- 负障碍系统维护 5 个字典，一致性检查复杂

**C. 状态机阻塞**（待排除）
- 某些状态转换条件不满足
- 超时阈值设置不合理
- 传感器数据延迟

### 2. Mapper 架构问题的影响

**直接影响**:
- 查询可通行性时需要检查多个数据结构 → 规划器变慢
- `_refresh_cell()` 频繁调用 → CPU 占用高
- 语义层次混乱 → 开发者难以定位问题

**间接影响**:
- 规划延迟 → 机器人停滞等待
- 错误的可通行性判断 → 规划失败重试 → 停滞
- 复杂性高 → bug 隐藏深，难以修复

---

## 解决方案设计

### 方案 1：性能优化（立即生效，2-3 天）

**目标**: 减少规划延迟，提升响应速度

**措施**:

#### 1.1 批量刷新优化
```python
# FloorGrid 添加延迟刷新机制
class FloorGrid:
    def __init__(self):
        self._dirty_cells: set[tuple[int, int]] = set()
    
    def _mark_dirty(self, gx: int, gy: int):
        """标记需要刷新的格子，不立即执行"""
        self._dirty_cells.add((gx, gy))
    
    def flush_changes(self):
        """批量刷新所有脏格子"""
        for cell in self._dirty_cells:
            self._refresh_cell(*cell)
        self._dirty_cells.clear()

# update_from_cloud 结束时调用一次 flush_changes
```

**预期效果**: 减少 60-80% 的 _refresh_cell 调用次数

#### 1.2 删除未使用字段
```python
# ObstacleCoreRecord 删除这些从未读取的字段
# - hit_world_poses
# - counter_world_poses
```

**预期效果**: 减少内存占用和序列化开销

#### 1.3 简化负障碍校验
```python
# persist_negative_clusters_atomic 从 91 行简化到 ~20 行
# 将校验职责上移到 ground_support_gate
```

**预期效果**: 减少负障碍更新延迟

**实施**:
- 文件: `mapper.py`
- 改动量: ~100 行
- 风险: 低（局部优化，不改变语义）
- 测试: 707 单元测试 + 1 seed 验证

---

### 方案 2：Mapper 架构重构（中期，1-2 周）

**目标**: 理清语义层次，提升可维护性

**措施**:

#### 2.1 命名澄清（2 天）
```python
# 重命名提升语义清晰度
base → observation_layer  # 观测基础层
_derived → planning_layer  # 规划约束层
```

#### 2.2 提取约束源（3-5 天）
```python
class ConstraintSource(ABC):
    @abstractmethod
    def blocking_at(self, cell) -> bool: pass
    
    @abstractmethod
    def affected_cells(self) -> Iterable: pass

class ObstacleConstraint(ConstraintSource): ...
class BlackConstraint(ConstraintSource): ...
class NegativeConstraint(ConstraintSource): ...

# _refresh_cell 简化为统一循环
def _refresh_cell(self, gx, gy):
    for constraint in self.constraints:
        if constraint.blocking_at((gx, gy)):
            self.planning_layer[gy, gx] = constraint.state_value
            return
    self.planning_layer[gy, gx] = self.observation_layer[gy, gx]
```

#### 2.3 简化负障碍（5-7 天）
```python
# 5 个字典 → 2 个字典
negative_clusters: dict[str, NegativeCluster]
negative_cell_to_clusters: dict[cell, set[cluster_id]]

# 删除冗余字典
# - negative_cells (从 keys() 推导)
# - negative_first_state (移到上层)
# - negative_cluster_confirmations (移到上层)
```

**预期效果**: 
- 代码可读性提升 50%
- 新人理解成本降低 60%
- Bug 定位时间减少 40%

**实施**:
- 文件: `mapper.py` (1261 行)
- 改动量: ~300 行
- 风险: 中等（架构变化，需充分测试）
- 测试: 707 单元测试 + 契约测试 + 2 seed 验证

---

### 方案 3：系统化性能分析（并行，2-3 天）

**目标**: 精确定位停滞根因

**措施**:

#### 3.1 增加性能日志
```python
# 在关键决策点增加耗时记录
@log_timing("frontier_selection")
def select_best_frontier(self, ...):
    # 记录 frontier 选择耗时
    ...

@log_timing("path_planning")
def astar_path(self, ...):
    # 记录路径规划耗时
    ...

# 日志输出
# [PERF] frontier_selection took 1.234s
# [PERF] path_planning took 0.567s
```

#### 3.2 运行带性能分析的测试
```bash
# 运行 1 seed 测试，收集性能数据
python3 -m danger_search.three_floor_matrix_runner \
    --seed 3141 \
    --output-dir /tmp/perf_analysis \
    --enable-timing-logs
```

#### 3.3 分析性能瓶颈
```python
# 解析日志，生成性能报告
python3 tools/analyze_timing_logs.py /tmp/perf_analysis/*/mission.log
# 输出：
# - 各阶段平均耗时
# - 耗时 > 1s 的操作列表
# - 停滞时刻的调用栈
```

**预期输出**: 明确的性能瓶颈清单

**实施**:
- 改动量: ~50 行（日志插入）
- 风险: 无（只增加日志）
- 时间: 2 天开发 + 1 天测试分析

---

### 方案 4：降低障碍物膨胀半径（可选，立即）

**目标**: 增加可达空间，减少规划失败

**措施**:
```yaml
# config/danger_search.yaml
obstacle_inflation_radius: 0.20  # 从 0.25 降到 0.20
```

**优点**:
- 立即生效
- 减少"被困"概率
- 可能减少规划耗时（可达路径更多）

**缺点**:
- 安全余量减少 20%
- 可能增加轻微碰撞风险

**建议**: 作为备选方案，优先使用方案 1-3

---

## 推荐实施计划

### 阶段 1: 立即行动（本周）

**优先级 1**: 方案 3 - 性能分析
- 增加性能日志（1 天）
- 运行性能测试（1 天）
- 分析报告（0.5 天）
- **目标**: 明确停滞根因

**优先级 2**: 方案 1 - 性能优化（如果方案 3 发现明显瓶颈）
- 实施批量刷新（1 天）
- 删除未使用字段（0.5 天）
- 测试验证（0.5 天）
- **目标**: 立即改善性能

**输出**:
- 性能分析报告
- 性能优化提交（如果实施）

### 阶段 2: 中期改善（下周）

**方案 2**: Mapper 架构重构
- 命名澄清（2 天）
- 提取约束源（3 天）
- 测试验证（2 天）
- **目标**: 提升代码质量

**输出**:
- 架构重构提交
- 重构验证报告

### 阶段 3: 持续监控（持续）

- 每次测试记录停滞时刻
- 用户反馈具体案例
- 迭代优化

---

## 验收标准

### 性能指标

| 指标 | 当前 | 目标 | 测量方法 |
|------|------|------|---------|
| Frontier 选择耗时 | 未知 | < 0.5s | 性能日志 |
| A* 规划耗时 | 未知 | < 0.3s | 性能日志 |
| 探索停滞频率 | 未知 | < 3 次/任务 | 日志统计 |
| 单次停滞时长 | 未知 | < 2s | 日志统计 |

### 功能指标

| 指标 | 当前 | 目标 |
|------|------|------|
| 电梯任务成功率 | ~0% | ≥ 80% |
| 探索覆盖率 | ~50% | ≥ 60% |
| 返航成功率 | ~20% | ≥ 80% |

---

## 资源需求

**人力**:
- 方案 1: 2-3 天（Opus 独立完成）
- 方案 2: 10-12 天（Opus 设计 + 实施 + 测试）
- 方案 3: 2-3 天（Opus 独立完成）

**计算资源**:
- 性能测试: 1 seed × 1 次 = ~30 分钟
- 验证测试: 2 seed × 1 次 = ~1 小时

**总计**: 2-3 周完成所有方案

---

## 风险评估

| 方案 | 风险等级 | 主要风险 | 缓解措施 |
|------|---------|---------|---------|
| 方案 1 | 低 | 批量刷新可能遗漏边界情况 | 充分单元测试 |
| 方案 2 | 中 | 架构变化可能引入回归 | 契约测试 + 分阶段实施 |
| 方案 3 | 无 | 只增加日志，无功能变化 | - |
| 方案 4 | 中 | 碰撞风险增加 | 作为备选，不优先 |

---

## 决策建议

**立即开始**: 
1. 方案 3（性能分析）- 明确根因
2. 如果发现瓶颈，实施方案 1（性能优化）

**后续安排**:
- 方案 2（架构重构）- 中期质量改善
- 持续监控和迭代

**不推荐**:
- 方案 4（降低膨胀半径）- 除非其他方案无效

---

**生成时间**: 2026-08-08  
**分析者**: Opus 5  
**状态**: 方案设计完成，等待实施

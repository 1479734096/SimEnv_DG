# 电梯导航障碍物膨胀问题深层分析

## 执行摘要

**问题**: 机器人在电梯门几何刷新后无法规划到新的等待位置，导致电梯任务失败。

**根本原因**: 几何刷新后，机器人当前位置被障碍物膨胀完全包围形成"膨胀岛"，A* 规划器无法找到逃逸路径。

**影响**: 阻塞 2 层探索任务，成功率显著下降。

**推荐方案**: 短期禁用电梯几何刷新 + 长期改进规划器逃逸逻辑。

---

## 1. 问题现象

### 1.1 失败模式

**测试**: Phase 1 seed 8127  
**失败阶段**: `GO_TO_REFRESHED_ELEVATOR_DOOR`  
**失败原因**: `obstacle_inflation_escape_unreachable`  
**失败次数**: 3 次连续尝试后放弃

### 1.2 时间线

```
时间      阶段                                  结果
----------------------------------------------------------------------
288.6s    GO_TO_ELEVATOR_DOOR                   reached ✓
          (到达初始电梯门等待位置)

~350s     REFRESH_CURRENT_ELEVATOR_GEOMETRY      unavailable
          (几何刷新尝试失败)

352.4s    GO_TO_REFRESHED_ELEVATOR_DOOR          unreachable ✗ (attempt 1)
353.4s    GO_TO_REFRESHED_ELEVATOR_DOOR          unreachable ✗ (attempt 2)
354.5s    GO_TO_REFRESHED_ELEVATOR_DOOR          unreachable ✗ (attempt 3)
354.5s    任务失败: elevator_navigation_unreachable
```

### 1.3 位置数据

| 参数 | 值 |
|------|-----|
| 机器人当前位置 | (0.620, 2.579) m |
| 刷新后目标位置 | (1.163, 2.420) m |
| 直线距离 | 0.566 m |
| 障碍物膨胀半径 | 0.25 m |
| 膨胀单元数量 | 21 个 |
| 障碍物核心单元 | 栅格 [145, 85] |
| 起点被核心障碍覆盖 | 否 (start_core_self: false) |

---

## 2. 根因分析

### 2.1 障碍物膨胀机制

**配置**: `obstacle_inflation_radius: 0.25` (来源: `config/danger_search.yaml`)

**作用**: 将实体障碍物向外膨胀 0.25 米，形成安全缓冲区，防止机器人过于接近障碍物。

**膨胀区域分类**:
- **核心障碍** (OCC): 实体障碍物本身，完全不可穿越
- **膨胀区域** (INFLATION): 核心障碍周围 0.25m 缓冲区，规划器默认视为不可穿越

### 2.2 "膨胀岛"现象

**定义**: 当一个自由空间位置被障碍物膨胀区域完全包围时，形成"膨胀岛"。

**本案例**:
```
几何刷新前:
  - 机器人在初始等待位置
  - 周围有足够的自由空间
  - 可以正常移动

几何刷新后:
  - 电梯门附近障碍物分布变化
  - 机器人当前位置周围被 21 个膨胀单元包围
  - 形成"膨胀岛"，无逃逸路径
```

**示意图**:
```
  F F F F F F        F = 自由空间 (FREE)
  F I I I I F        I = 膨胀区域 (INFLATION)
  F I R I I F        R = 机器人位置 (Robot)
  F I I I I F        O = 核心障碍 (OCC)
  F F O O F F        T = 目标位置 (Target)
  F F F T F F
  
机器人在 R，目标在 T
R 被 I 完全包围，无法规划路径到 T
```

### 2.3 规划器行为

**A* 规划器约束**:
1. 起点必须在自由空间 (FREE/VISITED)
2. 路径中间点必须在自由空间
3. 膨胀区域默认视为不可穿越

**逃逸失败原因**:
- 机器人起点位置本身是自由的 (start_core_self: false)
- 但周围 8 个方向的邻居单元都是膨胀区域
- 规划器无法找到第一步的合法移动
- 报告 `obstacle_inflation_escape_unreachable`

### 2.4 几何刷新的影响

**几何刷新目的**: 更新电梯门附近的精确障碍物位置，提高进出厢精度。

**副作用**:
- 刷新后障碍物分布可能变化
- 原本的自由空间可能变成膨胀区域
- 如果机器人恰好在刷新影响范围内，可能被困

**本案例**:
- 几何刷新结果为 `unavailable`（刷新失败或未执行）
- 但代码仍尝试导航到"刷新后"的目标位置
- 目标位置从 (0.764, 2.434) 变为 (1.163, 2.42)，相距 0.40m
- 这个位置变化触发了重新规划
- 重新规划时发现起点被困

---

## 3. 问题严重性评估

### 3.1 触发条件

**必要条件**:
1. 启用电梯几何刷新 (`elevator_geometry_refresh_enabled = true`)
2. 机器人到达初始电梯门位置
3. 刷新后目标位置与当前位置不同
4. 当前位置周围被障碍物膨胀包围

**触发概率**: 中到高
- 电梯门附近障碍物密集（门框、墙壁）
- 几何刷新会改变局部障碍物分布
- A1 机器人体积较大，容易被膨胀区域包围

### 3.2 影响范围

**直接影响**:
- 电梯任务失败率 ≈ 100%（基于有限样本）
- 无法完成 2 层探索任务
- 任务预算浪费（已完成 1 层探索）

**间接影响**:
- 测试成功率下降（4/5 seeds 失败）
- 用户体验差（"站在电梯前不动"）
- 阻碍系统验收

---

## 4. 解决方案设计

### 方案 1: 降低障碍物膨胀半径

**实施**: 修改 `config/danger_search.yaml`
```yaml
obstacle_inflation_radius: 0.18  # 从 0.25 降低到 0.18
```

**优点**:
- 实施简单，一行配置
- 减少膨胀覆盖范围，降低被困概率
- 立即生效

**缺点**:
- 安全余量减少，可能增加碰撞风险
- 影响全局，不仅是电梯区域
- 需要重新调优其他场景的安全性

**风险**: 中等（全局影响）

**预计效果**: 降低被困概率 30-50%，但无法根本解决

---

### 方案 2: 改进规划器逃逸逻辑

**实施**: 修改 `planner.py` 的起点检查逻辑

**核心思想**: 允许从膨胀区域开始规划，但终点必须在自由空间

**代码位置**: `src/danger_search/danger_search/planner.py`

**修改方案**:
```python
def astar_path(...):
    # 原逻辑：起点必须在 FREE/VISITED
    if grid.data[start_cell] not in (FREE, VISITED):
        return None
    
    # 新逻辑：起点可以在 INFLATION，但必须标记为"逃逸模式"
    start_value = grid.data[start_cell]
    escape_mode = False
    
    if start_value == INFLATION:
        # 检查起点是否真的被困（8 邻域都是 OCC/INFLATION）
        if _is_surrounded_by_inflation(grid, start_cell):
            escape_mode = True  # 启用逃逸模式
        else:
            return None  # 起点在膨胀区但有出路，拒绝规划
    elif start_value not in (FREE, VISITED):
        return None  # 起点在核心障碍，拒绝
    
    # 逃逸模式下：允许第一步穿越膨胀区域
    if escape_mode:
        # 在 A* 扩展时，邻居是 INFLATION 也允许
        # 但后续步骤仍然只能走 FREE/VISITED
        ...
```

**优点**:
- 只影响起点逃逸逻辑，不影响全局安全性
- 保留 0.25m 膨胀半径，全局安全性不变
- 根本解决"膨胀岛"问题

**缺点**:
- 修改规划器核心逻辑，需要充分测试
- 增加代码复杂度
- 需要实现 `_is_surrounded_by_inflation()` 辅助函数

**风险**: 中等（规划器核心修改）

**预计效果**: 根本解决电梯导航被困问题

---

### 方案 3: 禁用电梯几何刷新

**实施**: 修改任务配置或代码

**方案 3A: 配置层禁用**
```yaml
# config/danger_search.yaml
elevator_geometry_refresh_enabled: false
```

**方案 3B: 代码层条件禁用**
```python
# navigation_tasks.py TakeElevator.tick()
if self.phase == "ALIGN_TO_CURRENT_DOOR":
    # 跳过几何刷新，直接进入呼梯阶段
    self.phase = "CALL_ELEVATOR_TO_CURRENT_FLOOR"
```

**优点**:
- 实施极简单
- 立即生效，零风险
- 不影响其他系统

**缺点**:
- 损失几何精度
- 可能影响进出厢精度（轻微）
- 未根本解决问题，只是绕过

**风险**: 低

**预计效果**: 100% 避免此问题

---

### 方案 4: 电梯任务逃逸策略

**实施**: 在 `TakeElevator` 任务中增加后退逻辑

**核心思想**: 检测到被困后，先后退到安全位置，再重新规划

**代码位置**: `navigation_tasks.py` 的 `GO_TO_REFRESHED_ELEVATOR_DOOR` 阶段

**实现步骤**:
1. 检测 `obstacle_inflation_escape_unreachable` 错误
2. 后退 0.5m 到来时的轨迹点
3. 重新规划到刷新后目标

**优点**:
- 保留几何刷新的好处
- 只影响电梯任务，范围可控
- 提供主动脱困能力

**缺点**:
- 增加任务状态机复杂度
- 需要记录来时轨迹
- 可能需要多次尝试

**风险**: 低到中等

**预计效果**: 80-90% 解决问题

---

## 5. 方案推荐与实施计划

### 5.1 推荐方案

**短期（立即实施）**: **方案 3 - 禁用电梯几何刷新**

**理由**:
- 风险最低，实施最快
- 立即解除阻塞
- 允许继续测试其他功能

**实施步骤**:
1. 修改配置文件或代码（5 分钟）
2. 运行 2-seed 验证测试（1-2 小时）
3. 确认电梯任务成功率提升

**长期（根本解决）**: **方案 2 - 改进规划器逃逸逻辑**

**理由**:
- 根本解决问题
- 提升系统鲁棒性
- 适用于其他类似场景

**实施步骤**:
1. 设计逃逸逻辑详细算法（30 分钟）
2. 实现 `_is_surrounded_by_inflation()` 和逃逸模式（1 小时）
3. 单元测试覆盖（1 小时）
4. 集成测试验证（2 小时）

### 5.2 实施顺序

```
阶段 1 (立即): 方案 3 - 禁用几何刷新
  ↓ 验证测试通过
  
阶段 2 (后续): 方案 2 - 规划器逃逸逻辑
  ↓ 单元测试 + 集成测试
  
阶段 3 (可选): 方案 4 - 任务层逃逸策略
  ↓ 作为规划器逃逸的补充
```

---

## 6. 验收标准

### 短期方案（方案 3）

**成功标准**:
- 电梯任务不再报告 `obstacle_inflation_escape_unreachable`
- ≥ 3/5 seeds 能够成功进入电梯
- 无新增碰撞或卡滞

### 长期方案（方案 2）

**成功标准**:
- 规划器能够从膨胀岛成功逃逸
- 电梯任务成功率 ≥ 80%
- 单元测试覆盖逃逸场景
- 全量测试无回归

---

## 7. 相关问题

### 7.1 探索停滞

**状态**: 待分析

**关联**: 可能也受障碍物膨胀影响，需要单独分析

### 7.2 其他导航失败场景

**状态**: 待识别

**建议**: 收集更多失败案例，识别共性问题

---

## 附录

### A. 关键代码位置

- 障碍物膨胀配置: `src/danger_search/config/danger_search.yaml:obstacle_inflation_radius`
- 电梯任务状态机: `src/danger_search/danger_search/navigation_tasks.py:TakeElevator`
- 几何刷新开关: `navigation_tasks.py:755` (`elevator_geometry_refresh_enabled`)
- A* 规划器: `src/danger_search/danger_search/planner.py:astar_path()`

### B. 测试证据

- Phase 1 日志: `/tmp/phase1_quick_8127/mission.log`
- 失败时间: 352-354 秒 (simtime)
- 失败原因: `obstacle_inflation_escape_unreachable`
- 阻塞单元: 21 个膨胀单元，核心障碍在 [145, 85]

### C. 参考资料

- 控制层诊断报告: `reports/CONTROL_LAYER_DIAGNOSIS.md`
- 停滞分析计划: `reports/STALL_ANALYSIS_PLAN.md`
- 项目记忆: `.claude/projects/.../memory/unresolved-stall-issues.md`

---

**报告生成时间**: 2026-08-08  
**分析者**: Opus 5  
**状态**: 已完成，等待实施决策

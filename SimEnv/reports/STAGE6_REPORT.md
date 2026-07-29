# 第 6 层报告：建筑控制、门与电梯导航任务

## 阶段结论

第 6 层已完成。新工作区形成了“公开服务权限 → 运行时传感器几何 → 本地点云语义 → 非阻塞门/电梯任务 → 第 4 层唯一速度出口”的独立闭包。顶层 MissionManager 尚未迁入，但 3 个生产模块可通过 fake manager 和注入式服务完整验证状态转换。

本阶段直接测试 59 项全部通过，`danger_search` 累计 254 项通过。未知服务 ID/楼层、授权请求字段、门洞与轿厢证据、过门对齐、进出厢、服务失败、A* 三态和非法几何均有自动化覆盖。

## 迁移闭包

- `building_control_client.py`：公开 ID/served floors、运行时门/电梯几何、异步 `/set_door_state` 与 `/call_elevator` 请求。
- `local_semantic_detector.py`：局部点云门洞、轿厢侧壁/后壁和已注册门附近结构证据。
- `navigation_tasks.py`：`NavigateToPose`、`OpenDoorAndPass` 和 `TakeElevator` 非阻塞子状态机。
- 直接迁移并扩展对应 3 组测试；不迁移 MissionManager 或正式 launch。

## 已确认并修复的缺陷

### NAV-001：A* 失败后普通直连

旧 `NavigateToPose` 在 `astar_plan` 不是 reached 时，不区分 `UNREACHABLE` 与 `BUDGET_EXHAUSTED`，直接设置当前位置到目标的两点路径。这绕过占据栅格，可能让局部控制器朝 OCC/BLACK 目标行驶，也破坏第 3 层三态契约。

现确定不可达时统一发零速并返回 `FAILED`；预算耗尽发零速并返回 `RUNNING`，等待下一轮重新规划，不设置任何路径。专用过门/进出厢直线没有删除：它们已有段阻塞检查、门法向对齐、严格端点容差和位姿进度监督，语义不同于普通导航兜底。

### BUILD-001：运行时空间几何缺少边界

旧 `DoorInfo`、`ElevatorInfo`、服务超时、过门偏移和普通导航目标可以含 NaN/Inf，随后生成非有限等待点或服务生命周期异常。现构造和几何计算入口要求非空公开 ID、有限 XY-yaw、正宽度、有效厅姿态、正服务超时及有限偏移；非法任务目标在产生路径前失败。

### 服务权限与异步请求证据

保留旧有双重边界：空间字典中存在对象不等于获得服务权限，`open_door`、`close_door` 和 `call_elevator` 仍先检查公开 ID 与 served floors。新增测试注入实际服务函数，确认异步任务完成且请求字段精确保留；未知 ID/楼层继续 fail-closed。

## 合规与架构结论

- `BuildingControlClient` 不读取布局文件，不从公开场景信息制造空间坐标。
- `LocalSemanticDetector` 的正式路径只用运行时点云；`truth_proximity_debug` 兼容分支必须由第 7 层 formal profile 强制关闭。
- 服务 accepted 不直接清地图或确认换层；门开视觉、局部开口和目标层门确认仍是任务状态机必要条件。
- 所有旋转、直线跨越、恢复和停车都调用 `manager.controller`，最终复用 `PathFollower._publish`；本层没有第二个 `/cmd_vel` publisher。

## 自动化验证

| 验证项 | 结果 |
|---|---|
| 第 6 层直接测试 | 59 项通过 |
| `danger_search` 全测试 | 254 项通过 |
| 生产构建 | `catkin_make -j2` 成功 |
| 官方文件完整性 | 1109 个文件 SHA-256 全部一致 |
| 补丁格式 | `git diff --check` 通过 |

## Gazebo 决策与后续风险

本层没有正式节点和 MissionManager 调度入口，真实 Gazebo 服务调用无法在不提前迁移第 7/8 层的情况下形成完整闭环；因此本阶段采用确定性的服务注入和状态机测试，不新增仿真。第 8/10 层必须在真实服务上验证门动画、视觉开口、轿厢运输、目标层确认和唯一速度出口。

下一层进入 MissionManager。优先事项是修复 `MISSION-001` 电梯成功误报、`FLOOR-001` 隐式楼层对齐超时死锁、`TIME-001` 时间域混用和 `PERF-001` 同步长 A*；第 6 层状态机测试必须作为回归基线保留。

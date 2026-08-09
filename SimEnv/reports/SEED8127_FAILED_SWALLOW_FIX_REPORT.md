# seed 8127 电梯 ENTER_CABIN 停滞修复报告

- 日期：2026-08-08
- 分支：`fix/seed8127-task-control`（基线提交 `975bf4f`）
- 范围：任务层 `navigation_tasks.py`
- 类型：行为修复（非纯重构），不含建图层、控制层改动
- 验证状态：单元测试 **716 passed**；负对照有效；**Gazebo 闭环未跑**

## 1. 背景

上一轮（`975bf4f`，NAV-004 / NAV-005 / CONTROL-005）修复后，seed 8127 二层闭环复测
（证据目录 `/tmp/seed8127_ele2f_jTjwBf8s/`）结论仍为**算法失败**，但失败点前移：

- 硬门通过：`Entered RL /cmd_vel mode.`、`INIT→WAIT_CONTROLLER_READY reason home_pose_recorded`
- 路径进展：进门 ✓ → 0 层探索 ✓（coverage 0.457，`floor_timeout_change_floor_change_floor`）
  → `GO_TO_ELEVATOR_DOOR` ✓ → `OPEN_CURRENT_FLOOR_DOOR` ✓ → `ALIGN_TO_CURRENT_DOOR`
  → **`ENTER_CABIN` ✗**
- 停滞现象：sim t=330.0→385.6（**55.6 s**）原地不动，pose=(0.779, 2.33, -0.091)，
  `controller_mode=IDLE`，`active_motion_command=(0,0,0)`，`path_size=0`
- `cabin_transit_stage` 全程只有 `POSITION`（24→38 号记录），**CROSS 从未进入**
- 上一轮新增的 `logerr` 字符串**一条都没出现**

最后一条是关键线索：新增的失败日志没打，说明失败判定根本没有走到上抛路径。

## 2. 根因（两层，缺一不可）

### 层一：为什么规划失败（顺序死锁）

刷新后的等待点 (1.37, 2.35) 从起点 (0.773, 2.339) 不可达。全部 44 条记录的 blocker
完全一致：

```
active_cell_count: 51
obstacle_owner_cells: [[146,81],[147,81],[147,85],[149,82]]
start_core_self: false
unreachable_reason: obstacle_inflation_escape_unreachable
```

起点格落在电梯门框膨胀 OCC 内，`_resolve_start_escape` 无法脱困。

而 `_clear_opened_door_area` 此前**只在 `_commit_cabin_cross`（ALIGN→CROSS 提交时）
被调用**。但 `POSITION` 阶段先于 `ALIGN` 执行，于是形成闭环：

> 门区膨胀要等 POSITION 完成才清 → POSITION 又被没清的门区挡住 → 永不完成。

`mission_manager.clear_opened_door_area` 的清理深度本就按"门两侧等待点 + 占据膨胀
半径"设计（见其注释），正是为此场景准备，只是调用点接错了阶段。

### 层二：为什么是 55.6 s 而不是约 3 s（判决被吞）

界**存在且触发了**，但结论被调用方丢弃。

`_navigate_to_with_status` 在 unreachable/budget/timeout 达上限时调
`_fail_navigation` 返回 `FAILED`；`POSITION` 分支用裸 `return status` 正确上抛；
但 `ENTER_CABIN` 只判 `DONE` / `STUCK`，`FAILED` 落进函数末尾的 `return RUNNING`
被吞掉。计数器在失败时不重置，于是**每 tick 重新触发判决、每 tick 重新被吞**，
形成零速无界活锁。`EXIT_CABIN` 有完全相同的缺陷。

日志算术印证（87 条记录）：

| 项 | 值 | 含义 |
|---|---|---|
| `attempt` 3–44 各出现 **2 次** | 84 | 一次来自 unreachable 分支，一次来自 `_fail_navigation` |
| `attempt` 0/1/2 各 1 次 | 3 | 未达上限，只有 unreachable 分支 |
| `target` = (1.37, 2.35) | 44 | 正常规划记录 |
| `target` 为空 | **42** | `_fail_navigation(None, None)` |
| `target` = (0.972, 2.383) | 1 | 一次几何刷新 |

**42 次空 target = 界触发了 42 次、被丢弃了 42 次。**

顺带纠正三处此前的错误判断：`_DEFAULT_NAVIGATION_TIMEOUT` 是 120.0 s 不是 60 s
（55.6 s 只走了 46%，不存在"差 4.4 s 就自己失败"）；`attempt` 并非恒为 1，会爬到 44，
看门狗只在阶段入口重置一次；这不是"无界缺陷"，界存在且生效，是判决被调用方丢弃。

## 3. 修改清单

| 文件 | 变更 | 性质 |
|---|---|---|
| `src/danger_search/danger_search/navigation_tasks.py` | **+39 / −0** | 行为修复 |
| `src/danger_search/test/test_navigation_tasks.py` | **+145 / −0** | 契约测试 |

合计 +184 / −0。**零删除行**：未改写、未弱化任何既有断言或逻辑。

6 处 hunk 全部 grep 逐点核对已落地：

```
@@ -755,0  +756     __init__：新增 _cabin_transit_position_door_cleared
@@ -994,0  +996,15  修复A：ENTER_CABIN 显式判 FAILED
@@ -1166,0 +1183,11  修复A：EXIT_CABIN 显式判 FAILED（对称）
@@ -1339,0 +1367,10  修复B：POSITION 首次规划前清理门区
@@ -1553,0 +1591     _begin_cabin_transit 重置一次性标志
@@ -1569,0 +1608     _reset_cabin_transit 重置一次性标志
```

### 修复 A：确定性失败必须显式上抛（NAV-006）

`ENTER_CABIN` / `EXIT_CABIN` 各补 `elif status == FAILED:` 分支，`rospy.logerr`
输出 `stage` 与 `failure_reason` 后 `return FAILED`。

### 修复 B：POSITION 首次规划前清理已确认开放的门区（NAV-007）

`_tick_cabin_transit` 的 POSITION 分支，在**首次** `_navigate_to_with_status` 调用
**之前**插入 `_clear_opened_door_area(context, door_id)`，由一次性标志
`_cabin_transit_position_door_cleared` 保证每次 transit 只清一次（5 处触点齐全）。

## 4. 定向审计（用户批准范围内）

按用户批准，对任务层全部"可能吞掉 FAILED 的 `return RUNNING`"做 AST 审计。

审计器本身先做了负向标定：在修复前的 HEAD 上必须能标出全部 4 个疑点
（971 / 1125 / 1340 / 1947），否则审计器无效。第一版规则有致命盲区——
`_tick_cabin_transit` 因为经 `return status` 间接上抛而未被纳入 can-fail 集合，
会漏掉正在修的这个缺陷本身；`_navigate_to_with_status` 因为返回
`self._fail_navigation(...)`（调用而非 `FAILED` 字面量）也被漏掉。修正规则后标定通过。

结论：4 处疑点 → **2 处真缺陷已修**（971 ENTER、1125 EXIT），
**2 处确认为假阳性**：

- 1377：裸 `return status`，本身就是正确的向上传递，不吞
- 1986：`_navigate_to` 返回 bool，7 个调用方全部由 `failure_reason` 或
  `_last_navigation_blocked` 兜住

另一处早先误报也已更正：GO_COMMIT_POINT（276 行）实际有
`_last_navigation_stuck → _last_navigation_blocked → _handle_commit_blocked` 三重兜底，
是我的 grep 上下文窗口截断在它们之前造成的误判。

## 5. 验证结果

| 项 | 结果 |
|---|---|
| 全量单元测试（修复后工作区） | **716 passed**（基线 712 + 新增 4），1 warning，2.40 s |
| **负对照**（4 个新测试跑在未修复 HEAD 上） | **4 failed / 4**，有效 |
| 删除行数 | **0**，无既有断言被删或弱化 |
| `_cabin_transit_position_tolerance` 与 HEAD 一致 | 是（diff 中零改动，约束遵守） |
| 6 处 hunk 触点 grep 核对 | 全部落地 |

### 负对照方法（这是本轮唯一能证明测试有效的环节）

通过测试不能证明任何事，除非它在未修复代码上会失败。前两次尝试都无效：
`PYTHONPATH` 被 devel-space 的同名包遮蔽，实际导入的仍是工作区已修复版本
（自检打出 `含修复A标记: True`）。改为在隔离副本中直接操作 `sys.path`，并把
**HEAD 版 `navigation_tasks.py` 与工作区版测试文件配对**（前一次连测试文件也是 HEAD 版，
4 个新方法根本不存在，加载数为 0）。

最终自检确认无误后的结果：

```
实际导入的模块路径: /tmp/negctl_QwbF4aEL/danger_search/navigation_tasks.py
含修复A标记: False    含修复B标记: False    已加载测试数: 4
run=4  failures=4  errors=0
```

4 条失败断言与缺陷语义精确对应：

| 测试 | 未修复时的失败断言 |
|---|---|
| `test_enter_cabin_propagates_deterministic_failed_instead_of_running` | `'running' != 'failed'` |
| `test_exit_cabin_propagates_deterministic_failed_instead_of_running` | `'running' != 'failed'` |
| `test_enter_cabin_unreachable_position_fails_within_retry_limit` | `'running' != 'failed'` |
| `test_enter_clears_confirmed_door_area_before_first_position_plan` | `('plan_path', None) != ('clear_door', 'lift_door_ground')` |

其中端到端那条走完整 `task.tick` 路径（UNREACHABLE 规划器 + 推进时钟），
修复后 `plan_calls=3`、`unreachable_count=3`、
`failure_reason='elevator_navigation_unreachable'`，在 tick=11（t=1.1 s）有界 FAILED
—— 对比实测的 55.6 s。

## 6. 未验证部分与风险

**修复 B 仍是假设，必须由 Gazebo 判定。** 日志里的 51 个膨胀格在"门板"与"固定门框"
之间的归属无法从 `obstacle_owner_cells` 反推。若这些格主要来自**固定门框**而非门板，
清理门区带不动起点脱困，(0.773 → 1.37) 依旧不可达。届时的正确方向是把 NAV-003 的
脱困机制（`_begin_trace_approach` / `_begin_start_retreat_or_trace_approach`，
目前只接在 `GO_TO_ELEVATOR_DOOR`）延伸到 `ENTER_CABIN`——即本轮刻意搁置的修复 C。

**修复 A 是确定的，但它改变可观测行为**：原来的 55.6 s 静默停滞会变成约 3 s 内的
显式 FAILED + `logerr`。若修复 B 的假设不成立，下一轮会**更快地失败**而不是成功。
这是预期，也正是判断修复 B 是否有效的最快信号。

**回放能力边界**（回答用户"能否用接口输入回放替代完整测试"）：可以用于任务层
活性/有界性缺陷——本轮根因用回放几秒定位，对比完整仿真 57 分钟。不能替代：
膨胀为何吞掉门洞（感知层）、物理可行性（0.55 rad/s 死区、触地、RL 响应）、
未预料的实时竞争、时序抖动。且回放是**开环**的：它只能证明旧逻辑在录制输入流上会卡，
不能证明新逻辑会成功——行为一改，录制的位姿流就失效了。建议两级：
回放用于调试迭代，完整 Gazebo 跑一次用于验收。

## 7. 文档同步说明

本轮全部是内部失败处理行为修复，未变更接口、命令用法或配置项，故用户手册与
配置示例无需改动。已更新：本报告 + `docs/team/BUG_LEDGER.md`（NAV-006 / NAV-007 两条）。

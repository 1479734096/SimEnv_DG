# R1 阶段报告：命令写入权收口

阶段目标：把散落在 `_publish_locked` 内联的零速归因优先级链收口为唯一仲裁点，
并给唯一的非零命令入口 `set_path` 补上归因。

阶段禁止事项（全部守住）：不改支撑门阈值、不改既有 reason code 字面值、
不改 `_cabin_transit_position_tolerance`、保持 `navigator → mapper/planner = 0`、
不混行为修复。

---

## 1. 基线与结果

| 项 | R0 (`605a130`) | R1 |
|---|---|---|
| 全量测试 | 751 passed（本阶段新增 19：13 等价 + 6 归因） | 751 passed, 0 failed |
| 生产文件改动 | — | `navigator.py` +106/−42（纯提取）、+归因写入 |
| 调用点改动 | — | 14 处 `set_path` 补 reason |
| 无归因写入点 | 50 | **36**（`set_path` 14/14 清零） |
| reason code 清单 | 90 | 104（新增 14，删除 0，改名 0） |

## 2. R1a：仲裁收口为纯函数

`_publish_locked` 原本内联五级归因优先级链。提取为模块级纯函数：

- `sanitize_command_request(linear_x, angular_z, linear_y, zero_reason)` → `(requested, zero_reason)`
- `arbitrate_command(requested, *, zero_reason, gate, publish_failure_latched,
  publish_failure_reason, pose_yaw, wall_time, control_time_source)` → `(emitted, resolved_zero_reason)`

拆成两个而非一个：调用方必须在消毒之后、仲裁之前提交 `_last_requested_command`。
该诊断字段语义是「任务层请求的命令」，与「门过滤后的命令」不同，合并会让请求
快照被门结果污染。

`_publish_locked` 现在只负责事务性副作用：提交请求快照、发布、以及只描述
「已成功交给 publisher 的命令」的那批统计字段。

### 2.1 提取过程中查明的三条隐藏语义

均由 396 场景表格实测确认，非代码阅读推断。这三条是重构最容易静默破坏的部分：

**归因遮蔽。** 非法请求（nan/inf/类型错误）叠加畸形门输出时，最终归因是
`invalid_command_gate_output`，而不是 `invalid_requested_command`。门是用消毒后的
`(0,0,0)` 调用的，它的畸形输出覆盖了更早的非法请求归因。推论：`invalid_command_gate_output`
只在请求本身已是零速时可观测，其他情况会被 `ground_support_gate:<code>` 盖掉。

**epsilon 边界含等号。** 请求 `1e-4` 会被原样发出（不归零），但因判据是
`abs(v) <= 1e-4`，仍被归因为 `zero_command`。

**门被错误归责。** 把 `1e-4` 减半到 `5e-5` 的门并未归零，但因结果跌破 epsilon
且 `emitted != requested`，归因变成 `ground_support_gate:halved`。

### 2.2 提取时避开的一个新失败面

原实现只在「有门 + filter 可调用 + 未锁存」时才调用 `get_time()`；无门场景 0 次
（已实测）。而 `get_time()` 在 ROS 已初始化却读取失败时会抛
`RuntimeError("ROS 任务时间不可用，拒绝切换到墙钟")`。

若提取时把 `control_time` 作为求值结果传入，无门路径会新增一次任务时间读取，
等于给它凭空引入一个新的失败面。因此参数设计为 `control_time_source` 传 callable。
`ArbitrationLazinessTest` 五条断言把这个惰性钉死。

## 3. R1b：`set_path` 归因

### 3.1 修复的缺陷

`_debug_replan_reason` 此前只由 `clear_path` / `goal_reached` / `route_progress_timeout`
/ `alignment_done` 写入，**装载新路径不写**。后果：装上新路径后，快照里的
`replan_reason` 仍停留在上一次清理的旧值 —— 路径已经换了一条，归因还指向前一个原因。

实测确认（修复前后对比）：`clear_path("door_pass_completed")` 后再
`set_path(reason="probe_unknown_space_path")`，修复前 `replan_reason` 保持
`door_pass_completed`，修复后正确翻到 `probe_unknown_space_path`。

### 3.2 归因写入点的位置

写在两道校验（类型解析、有限性）**之后**。被拒绝的路径不得声称自己已装载，
否则快照会留下一条从未生效的 reason。校验失败路径的归因由
`stop(reason="invalid_path")` 经 `_debug_zero_command_reason` 记录，两条通道不混。

### 3.3 14 处调用点的 reason

| 文件 | 行 | 所属方法 | reason |
|---|---|---|---|
| `navigation_tasks.py` | 72 | `NavigateToPose.tick` | `navigate_to_pose_path` |
| `navigation_tasks.py` | 367 | `OpenDoorAndPass._tick_active_pass` | `door_pass_point_path` |
| `navigation_tasks.py` | 530 | `OpenDoorAndPass._navigate_to` | `door_navigate_planned_path` |
| `navigation_tasks.py` | 538 | `OpenDoorAndPass._navigate_to` | `door_navigate_direct_path` |
| `navigation_tasks.py` | 1321 | `TakeElevator._commit_cabin_cross` | `elevator_cabin_cross_path` |
| `navigation_tasks.py` | 1891 | `TakeElevator._navigate_to_with_status` | `elevator_navigate_planned_path` |
| `mission_manager.py` | 2442 | `_tick_explore` | `frontier_confirmed_path` |
| `mission_manager.py` | 2485 | `_tick_explore` | `frontier_selected_path` |
| `mission_manager.py` | 2863 | `_try_probe_unknown_space` | `probe_unknown_space_path` |
| `mission_manager.py` | 3415 | `_tick_return_home` | `return_home_entry_trace_path` |
| `mission_manager.py` | 3459 | `_tick_return_home` | `return_home_planned_path` |
| `initial_entry_runtime.py` | 139 | `tick_clearance` | `initial_entry_clearance_path` |
| `initial_entry_runtime.py` | 271 | `tick` | `initial_entry_corridor_path` |
| `initial_entry_runtime.py` | 300 | `tick` | `initial_entry_reentry_path` |

14 个新 code 全部以 `_path` 结尾，与既有 90 个无重名（跨方法复用断言仍只有
`return_home_retreat_unavailable` 一例）。

## 4. 等价性证明

### 4.1 表驱动等价（396 场景）

`test_command_arbitration_equivalence.py` + `data/command_arbitration_equivalence.json`。
输入是 9 种请求 × 11 种门 × 2 种调用方 reason × 2 种锁存态的完整叉积。
固化数据由**重构前**的 `_publish_locked` 实测生成，20 种去重 `(emitted, resolved)` 形态。

三条断言路径：

1. `PublishLockedMatchesFixtureTest` —— 方法行为逐场景等于固化数据（重构前证明
   fixture 准确，重构后证明无漂移，同一份断言两次都过）。
2. `PureArbitrationMatchesFixtureTest` —— 纯函数逐场景等于固化数据。
3. 同类中的 `test_pure_and_method_paths_agree_scenario_by_scenario` —— 双路逐场景互等。

另有 `ArbitrationFixtureIntegrityTest` 先验证固化数据本身覆盖完整叉积（396 条），
防止等价测试因数据缺失而空洞通过。

### 4.2 变异负对照

R1a（等价测试，4 组全部被捕获）：

| 变异 | 结果 |
|---|---|
| epsilon `1e-4` → `1e-3` | 1 failed |
| 去掉门归因覆盖 | 1 failed |
| 默认 `zero_command` 改名 | 1 failed |
| 门畸形输出归因改名 | 1 failed |

R1b（`PathInstallAttributionTest`，4 组全部被捕获）：

| 变异 | 结果 |
|---|---|
| `set_path` 不写归因 | 4 failed（6 个测试中，另 2 个走 AST 读源码，正确不受运行时影响） |
| 归因写在校验之前 | 1 failed |
| 默认 reason 改名 | 2 failed |
| 调用点去掉归因 | 3 failed |

负对照方法论修正：第一次跑变异 A 时补丁未生效（匹配串漏了三行注释），
脚本报 `fail=0`，那是**对未变异代码的测量**，不能算通过。第二次用行号精确删除，
并先验证 `navigator.__file__` 指向 `/tmp` 隔离副本且归因写入确认已消失，
才采信结果。

## 5. 性能：一个超阈值回归，判定为豁免

| 目标 | R0 | R1 | Δ | 判定 |
|---|---|---|---|---|
| `compute_tracking_command` | 1.672 | 1.681 | +0.5% | pass |
| `publish_locked_no_gate` | 0.855 | 1.030 | **+20.6%** | REGRESS |
| `publish_locked_with_gate` | 2.793 | 2.924 | +4.7% | pass |
| `publish_with_lock` | 3.197 | 3.251 | +1.7% | pass |
| `gate_filter_command` | 0.875 | 0.863 | −1.4% | pass |
| `path_follower_tick` | 11.093 | 11.348 | +2.3% | pass |

回归稳定复现四次（+21.8% / +22.2% / +21.2% / +20.6%），非噪声。

**瓶颈分析。** 绝对增量 0.185 µs。实测两次 Python 函数调用（含关键字绑定）
开销 0.109 µs，可解释 59%；余量来自返回元组分配与 8 个关键字参数绑定。
无门路径原本几乎不做事，调用开销占比因此被放大。

**豁免依据：该路径在生产不可达。**

- `GroundSupportCoordinator.__init__` **无条件**构造 `GroundSupportSafetyGate(gate_config)`。
  类型标注写 `| None`，但实际永不为 None。
- `mission_components.build()` 在 `controller.set_command_gate(ground_support.gate)`（:115）
  装门；`controller.start()`（`mission_node_runtime.py`:86，创建控制定时器）在其之后。
  装门早于任何命令发布的可能。

生产实际热路径是 `publish_locked_with_gate`（+4.7%）与 `path_follower_tick`（+2.3%），
均在阈值内。控制决策 50 Hz，0.185 µs 相当于 0.0009% CPU。

**同时记录我在 R0 的选型缺陷**：`publish_locked_no_gate` 作为基准目标不反映生产
配置。保留它是因为它是纯粹的调用开销探针（有诊断价值），但**不应作为回归门禁**。
后续阶段判定回归时以带门目标为准。

## 6. 交付物

| 文件 | 变化 |
|---|---|
| `src/danger_search/danger_search/navigator.py` | 提取 2 个纯函数；`set_path`/`_set_path_locked` 加 `reason`；归因写入 |
| `src/danger_search/danger_search/navigation_tasks.py` | 6 处调用点补 reason |
| `src/danger_search/danger_search/mission_manager.py` | 5 处调用点补 reason |
| `src/danger_search/danger_search/initial_entry_runtime.py` | 3 处调用点补 reason |
| `src/danger_search/test/test_command_arbitration_equivalence.py` | 新增，13 测试 |
| `src/danger_search/test/data/command_arbitration_equivalence.json` | 新增，396 场景固化数据 |
| `src/danger_search/test/test_command_authority_baseline.py` | 新增 `PathInstallAttributionTest`（6 测试）；更新 2 处基线 |
| `src/danger_search/test/test_mission_manager_helpers.py` | 6 处 `set_path` 替身补 `**kwargs` |
| `reports/bench/r1_command_authority.json` | 新增 |

### 6.1 正当更新的两处 R0 基线

- `FROZEN_REASON_DEFAULTS["set_path"]`：`{}` → `{"reason": "path_set"}`。R1 的目标
  就是给它加归因，签名变更是有意为之。
- `UNATTRIBUTED_CALLSITE_BASELINE`：50 → 36。断言仍是 `<=`，只允许继续减少。

两处都不是为了让测试通过而弱化断言：清单冻结（子集断言）、跨方法复用（精确字典）、
零速归因优先级（7 条）全部未动且仍通过。

### 6.2 测试替身的修改说明

`test_mission_manager_helpers.py` 有 6 处 `set_path` 替身只接受位置参数
（`lambda p: ...`、`paths.append`），生产签名新增关键字参数后会 `TypeError`。
补 `**kwargs` 与本文件既有 13 处替身写法一致，断言仍落在 recorded path 上，
未弱化任何断言。

## 7. 验收对照

| 项 | 状态 |
|---|---|
| 定向测试通过 | ✅ 19 新增测试全过 |
| 契约测试通过 | ✅ R0 的 22 条全部仍过（含未改动的优先级链 7 条） |
| 等价测试通过 | ✅ 396 场景 × 三条断言路径 |
| 微基准回归 <10% | ⚠️ 生产路径 ✅（+4.7% / +2.3%）；无门路径 +20.6% 豁免（生产不可达，§5） |
| 全量测试无新增失败 | ✅ 751 passed，0 failed |
| 阶段报告 | ✅ 本文件 |
| 提交与标签 | `command-authority-layer-1` |

## 8. 剩余归因空洞（后续事项）

36 处无归因写入点仍在，按方法：`stop` 13、`clear_path` 10、`rotate_in_place` 4、
`poll_result` 4、`recover` 3、`_publish_locked` 1、`_publish` 1。

其中三个 API 仍无 `reason` 参数：

- `rotate_in_place`（4/4 无归因）—— 直接发 cmd_vel，非零命令入口。
- `poll_result`（4/4）—— 名为「轮询读」却调用 `_clear_path_locked` 有写副作用，
  这个命名与副作用不符本身应先解决。
- `end_heading_alignment` —— 有签名但 0 调用点，无任何契约保护，改动无网。

`stop` / `clear_path` / `recover` 的 26 处吃默认值（`'stop'` / `'path_cleared'` /
`'stuck'`），不是缺参数而是调用方没传。这些默认值让日志无法区分具体是哪一处停车，
是下一步收口的对象。

## 9. 口径保留

- 微基准是离线单进程测量，不含 ROS 消息序列化、网络与 Gazebo 负载，只能检出
  算法级回归，不能推断实机时序。
- Gazebo 闭环未验证。R1 是纯结构重构（行为等价已由 396 场景表格证明），
  但 seed 8127 的 A/B 修复仍未在实机验证。
- `set_path` 归因复用 `_debug_replan_reason` 而非新增字段：语义统一为
  「最近一次改变路径状态的原因」。代价是无法同时观测「上一次清理原因」与
  「本次装载原因」，只保留最新一条。

# R0 报告：命令写入权重构基线固化

阶段目标：在不改动任何生产代码的前提下，固化 R1（命令写入权收口）必须保持的行为契约、
测量准确的结构基线、建立性能基线。

阶段禁止事项（本阶段实际遵守情况）：**不改任何生产代码，只新增测试与工具** —— 已遵守，
`git status` 显示生产代码零改动。

---

## 1. 基线信息

| 项目 | 值 |
| --- | --- |
| Git HEAD | `8470f6a35027c6d6c5b09864c698a57c77d42d34` |
| 分支 | `fix/seed8127-task-control` |
| 工作区（R0 开始前） | 生产代码零改动；仅存在未跟踪的 `reports/NEXT_STEPS_PLAN.md`（用户既有文件，未触碰） |
| Python | 3.8.10（`ast.unparse` 不可用，AST 分析改用 `ast.Constant` 取值） |
| 全量测试（R0 前） | 716 passed |
| 全量测试（R0 后） | 732 passed, 0 failed（+16 为本阶段新增） |

### R1 目标文件哈希与行数

| 文件 | 行数 | sha256(前16) |
| --- | --- | --- |
| `navigator.py` | 1691 | `9f86d2030a4c4ebb` |
| `navigation_tasks.py` | 2250 | `24388dd0bc25bd1d` |
| `mission_manager.py` | 4218 | `27721f9b5120476f` |
| `initial_entry_runtime.py` | 485 | `d4dadde2481d8ca2` |
| `exploration_runtime.py` | 565 | `f804e5d5d78c9a3d` |
| `elevator_observation_runtime.py` | 522 | `ae8f973a20e12a8c` |
| `ground_support_coordinator.py` | 843 | `d6e3e960df41899f` |
| `ground_support_gate.py` | 1427 | `c5f851cb0d1d1d90` |

---

## 2. 对既有基线记录的修正

R0 用 AST 重测后，发现此前用 `rg` 正则得到的几项基线数字不准确。**修正如下**，
后续阶段以本节为准。

### 2.1 「129 处命令写入权 / 7 文件」——总数偶然正确，分布错误

正则基线记为：`navigation_tasks` 56、`mission_manager` 38、`initial_entry` 16、
`navigator` 8、`exploration` 5、`elevator_observation` 4、`ground_support_coordinator` 2。

AST 实测（10 个命令写入方法，共 **168** 个调用点）：

| 方法 | 调用点 | 类别 |
| --- | --- | --- |
| `stop` | 71 | 直接写 cmd_vel |
| `clear_path` | 42 | 改路径状态 |
| `_publish_locked` | 20 | 发布原语（navigator 内部） |
| `set_path` | 14 | 改路径状态 |
| `recover` | 5 | 直接写 cmd_vel |
| `rotate_in_place` | 4 | 直接写 cmd_vel |
| `reverse_straight` | 4 | 直接写 cmd_vel |
| `poll_result` | 4 | 改路径状态（见 3.2） |
| `_publish` | 4 | 发布原语（navigator 内部） |
| `end_heading_alignment` | 0 | 直接写 cmd_vel（有签名、无调用点） |

拆分：**168 = 外部模块 140 + navigator 内部 28**（28 = 24 个发布原语调用 + 4 个 public 方法自调用）。

外部 140 按文件：`navigation_tasks` 66（direct 38 / state 28）、`mission_manager` 44（25/19）、
`initial_entry_runtime` 16（6/10）、`exploration_runtime` 7（7/0）、
`elevator_observation_runtime` 5（4/1）、`ground_support_coordinator` 2（1/1）。

正则把 `navigator.py` 记成 8（实际内部 28），同时少算了 `navigation_tasks` 和 `mission_manager`，
总数凑巧仍落在 129。**「129」这个数字不可再引用。**

### 2.2 「137 个 reason code（134 字面值 + 3 f-string）」——统计的是混合面

那次统计把三个互不相干的 reason 族混在一起：控制器写入的 `reason=` / `zero_reason=`、
任务失败的 `reason_code=` / `failure_reason=`、前沿筛选的 `rejection_reason=`。

R1 真正会碰到的是**控制器命令写入面**，AST 实测为 **90 个去重字面 reason code**
（100 处字面出现去重后得 90；`return_home_retreat_unavailable` 在 `stop` 和 `clear_path` 下各出现一次）。

### 2.3 「控制器写入出口 5 个」——漏了 5 个

正则记为 `set_path:267`、`clear_path:374`、`stop:1052`、`_publish:1208`、`_publish_locked:1233`。
漏掉的 public 写入 API：`poll_result:435`、`end_heading_alignment:522`、`recover:980`、
`rotate_in_place:1026`、`reverse_straight:1042`。

### 2.4 「`record_failure` 5 文件 37 处」——含同名异义方法

其中 `frontier.py:373/714/1010` 是 `FrontierExplorer.record_failure(grid, world_xy)`
（标记不可达前沿），与任务失败记录器 `NavigationTaskContext.record_failure(reason)` 无关。
正则按方法名匹配无法区分族属。

---

## 3. R0 新发现的结构事实（R1 的直接依据）

### 3.1 归因空洞集中在「让 dog 动起来」的路径上

168 个调用点中：**100 处显式字面 reason、50 处无任何 reason 归因、18 处动态 reason**。

50 处无归因按方法：`set_path` 14/14、`stop` 13/71、`clear_path` 10/42、
`rotate_in_place` 4/4、`poll_result` 4/4、`recover` 3/5、`_publish` 1/4、`_publish_locked` 1/20。

关键点：**`set_path` 完全没有 `reason` 参数**（14 个调用点全部无归因），
而它是下发非零运动路径的主入口；`rotate_in_place` 同样无 `reason` 参数。
`stop` / `clear_path` / `recover` / `reverse_straight` 带静默默认值
（`'stop'` / `'path_cleared'` / `'stuck'` / `'reverse_straight'`），
因此「dog 停下了，是谁停的」在默认值调用点上不可区分。

### 3.2 两类写入被混为一类

`set_path` / `clear_path` / `poll_result` **本身不发布 cmd_vel**，它们只改路径状态，
实际命令由控制定时器 `_on_control_timer` 发出。真正直接发 cmd_vel 的是
`stop` / `recover` / `rotate_in_place` / `reverse_straight` / `end_heading_alignment`。
R1 的仲裁入口必须区分这两类，否则「写入权收口」会把状态变更和命令发布混在一个仲裁点上。

另外 `poll_result` 名为轮询读取，却调用 `_clear_path_locked` 产生写副作用。

### 3.3 零速归因的优先级链（`_publish_locked`，已冻结）

1. 请求值非有限或类型错误 → `invalid_requested_command`（最高）
2. `_publish_failure_latched` 为真 → `command_publish_failure_latched`，**跳过门**
3. 门抛异常或输出非法 → `invalid_command_gate_output`
4. 门把非零请求改为零 → `ground_support_gate:<gate.reason_code>`
5. 调用方自己请求零速 → **保留调用方 reason**（门未致零，不归因给门）
6. 兜底 → `zero_command`

隐藏语义：**`invalid_command_gate_output` 只在请求本身已是零速时可观测**。若请求非零而门输出非法，
第 4 条会把它覆盖成 `ground_support_gate:<code>`（现有 `test_command_gate_exception_is_replaced_with_zero`
断言的正是后者）。这是 R1 最容易静默改坏的一条。

### 3.4 控制定时器优先级：`recover` > `align` > `path` > `keepalive`

`recover` 分支不做位姿检查（`_control_step_locked(None)`）；`align` 与 `path` 分支都先校验
位姿存在与新鲜度，失败即零速并归因 `controller_pose_missing` / `controller_pose_stale`。

### 3.5 `_publish_failure_latched` 无复位路径

全文仅 `navigator.py:1295` 置为 `True`，**无任何复位点**。一次 `/cmd_vel` 发布异常后，
控制器永久输出零速。这是 fail-closed 设计，R0 已用测试固化，R1 不得引入复位。

### 3.6 keepalive 会重放门过滤后的值

`_keepalive_replay_locked` 重放 `_last_emitted_command`，该字段存的是**门过滤后**的输出。
因此门一旦致零，keepalive 会持续重放零速（非零命令另有 `direct_command_max_age` 超龄归零，
归因 `direct_command_expired`）。

### 3.7 45/90 个 reason code 此前无任何测试引用

R0 前，90 个控制器写入 reason code 中恰好 45 个在其他测试文件中被引用过，另 45 个
完全无引用（45 + 45 = 90，已校验）。这 45 个无引用 code 现已被新增的清单冻结测试覆盖，
R1 若改名或删除会立即失败。

---

## 4. R0 交付物

| 文件 | 行数 | 说明 |
| --- | --- | --- |
| `src/danger_search/test/test_command_authority_baseline.py` | 536 | 契约测试，16 项 |
| `tools/bench/bench_command_decision.py` | 344 | 命令决策热路径微基准 |
| `reports/bench/r0_baseline.json` | 74 | 微基准基线数据 |

### 4.1 契约测试构成（16 项 / 22 条断言）

- `CommandReasonInventoryTest`（3）：90 个 reason code 全部仍在；清单规模与实测一致；
  跨方法复用显式声明。
- `CommandWriteApiSurfaceTest`（3）：10 个方法的 reason 参数默认值冻结；写入方法在
  `PathFollower` 上存在；无归因调用点数不得增长（基线 50）。
- `ZeroCommandAttributionPriorityTest`（7）：3.3 的六级优先级逐条固化，含
  `invalid_command_gate_output` 的可观测条件、锁存优先于门、锁存无复位路径。
- `ControlTimerPriorityTest`（3）：3.4 的四级优先级。

清单提取用 AST 在测试运行时从源码现算，而非硬编码副本，因此 R1 移动调用点不会误报；
断言方向为「冻结集 ⊆ 实测集」，允许 R1 新增 reason code，但删除或改名即失败。

### 4.2 变异负对照（确认测试有约束力）

16 项一次通过后，按项目既有纪律做了变异验证，确认不是空洞通过：

| 变异 | 结果 |
| --- | --- |
| 未变异基线 | 16 run / 0 fail |
| A：冻结清单含一个已删除的 code | 16 run / **2 fail** |
| B：`stop` 默认 reason 被改 | 16 run / **1 fail** |
| C：无归因基线改为 0 | 16 run / **1 fail** |

三个变异全部被捕获。

### 4.3 微基准基线

固定输入、预热 5 批、测量 40 批（要求 ≥20）、每批内循环 300–5000 次，取中位数与 p95。
内置确定性守卫：若目标在测量期间输出或门状态漂移，报 NON-DETERMINISTIC 而非给出数字。
本次 6 个目标全部 `det=ok`。

| 目标 | median (µs) | p95 (µs) | min (µs) | 内循环 |
| --- | --- | --- | --- | --- |
| `compute_tracking_command` | 1.672 | 2.177 | 1.618 | 2000 |
| `publish_locked_no_gate` | 0.855 | 0.866 | 0.848 | 5000 |
| `publish_locked_with_gate` | 2.793 | 2.842 | 2.725 | 2000 |
| `publish_with_lock` | 3.197 | 3.241 | 3.122 | 2000 |
| `gate_filter_command` | 0.875 | 0.894 | 0.845 | 2000 |
| `path_follower_tick` | 11.093 | 11.216 | 10.832 | 300 |

R1 验收口径：以上 median 回归 <10%。

---

## 5. A/B 行为契约（留待 Gazebo 验证）

按用户决策「直接进 R0，A/B 留待重构后验证」，`8470f6a` 的两个修复
（进出轿厢确定性失败被吞、门区清理时序）仍只有离线契约测试覆盖，未经 Gazebo 闭环验证。
四个测试确认在位：

- `test_enter_cabin_propagates_deterministic_failed_instead_of_running`
- `test_exit_cabin_propagates_deterministic_failed_instead_of_running`
- `test_enter_cabin_unreachable_position_fails_within_retry_limit`
- `test_enter_clears_confirmed_door_area_before_first_position_plan`

它们已在 `975bf4f`（修复前）上做过负对照，4/4 失败于预期断言。

---

## 6. 已知失败签名（重构前既存，非本阶段引入）

- `/clock` 冻结导致无界空转：`gzserver` 退出 → `/clock` 停 → `get_time()` 冻结
  （`time_utils` 明确拒绝墙钟回退）→ `_elapsed()` 不增长 → `max_runtime` 兜底永不触发。
  三次运行分别空转 276 s / 548 s / 123 s，最终被外部 kill，退出码 1 且无 terminal 状态转移。
- `_run_service` 无界轮询（`navigation_tasks.py:648` / `:2229`），理论缺陷，三次运行均未触发。
- 探索层：`no_frontier_escape_exhausted_return_home`、前沿否决抖动。

以上均记录在 `cc/next.md` §6「重构之后的待办」，不在 R1 范围内。

---

## 7. 验收结论

| 项 | 状态 |
| --- | --- |
| 定向测试通过 | ✅ 16/16 |
| 契约测试通过 | ✅ 16/16，变异负对照 3/3 被捕获 |
| 等价测试 | 不适用（本阶段无模块拆分） |
| 微基准 | ✅ 基线建立，6/6 确定性通过 |
| 全量测试无新增失败 | ✅ 732 passed（716 + 16），0 failed |
| 阶段报告 | ✅ 本文件 |
| 生产代码零改动 | ✅ `git status` 确认 |
| 提交与标签 | ⏳ 待用户确认后执行 |

## 8. 口径保留

- 本报告所有结构数字来自 AST 分析，已排除注释与字符串中的同名出现；与 §2 的正则数字冲突时以 AST 为准。
- 微基准是**离线单进程**测量，不含 ROS 消息序列化、网络与 Gazebo 负载，只用于检出重构引入的
  算法级回归，不能推断实机时序。
- `end_heading_alignment` 有签名但 0 调用点，其行为未被任何测试覆盖，R1 若改动它无契约保护。
- 3.7 的「45 个已被引用」按字面量在测试文件中出现判定（已排除本冻结测试自身），可能把跨族同名
  计入，绝对值略偏高；「45 个无引用」则是可信下界。

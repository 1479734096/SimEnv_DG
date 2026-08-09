# R3 阶段报告：乘梯阶段机强制穷举（修复 fail-open 假成功）

## 1. 基线

| 项 | 值 |
| --- | --- |
| 起始 Git HEAD | `ccac10d`（R1 完成态，标签 `command-authority-layer-1`） |
| 分支 | `fix/seed8127-task-control` |
| 全量测试（R3 前） | 751 passed |
| 全量测试（R3 后） | 754 passed, 0 failed（+3 为本阶段新增） |

## 2. 阶段目标的调整（与原计划不同，记录原因）

原 R3 计划是「把 `TakeElevator.tick`（426 行 / 32 分支）改成派发表」。测量后调整为
**只补强制穷举、不做派发表改造**，原因是测量结果改变了对收益与风险的判断：

`tick` 的真实结构不是嵌套 if-elif 链，而是 **20 个平铺、彼此独立、各自 return 的
`if` 语句**（AST 实测顶层语句序列）。这个形态已经具备派发表的主要优点（分支间无
耦合、无 fallthrough 到下一分支），改成派发表要把 20 个分支体各自提取成方法，
而分支体共享 `client`、`refresh_wait`、`cabin_center`、`current_wait`、
`target_wait` 等局部变量，提取时必须逐个决定传参或重算 —— 这是 426 行的大范围
搬动，等价性论证成本高，而收益主要是「可读性」。

派发表真正不可替代的收益只有一条：**强制穷举**。而这条可以独立、局部地实现。
所以本阶段只取这条收益，跳过搬动。派发表改造作为后续候选记录在第 6 节。

## 3. 修复的缺陷（NAV-008，行为修复非纯重构）

### 3.1 缺陷

`tick` 末尾是裸 `return DONE` 兜底。phase 全集与分支覆盖的实测对账：

| 集合 | 数量 | 说明 |
| --- | --- | --- |
| 全类 `self.phase = ` 赋值的 phase | 23 | AST 提取字面值 |
| 加上 `__init__` 初始值 `GO_TO_ELEVATOR_DOOR` | 24 | 已知 phase 全集 |
| `tick` 内显式比较的 phase | 19 | 20 个 `if` 中 19 个是等值比较 |
| `startswith("EXIT_BACKOFF_")` 覆盖 | 4 | `EXIT_BACKOFF_{ALIGN,REALIGN,REVERSE,SETTLE}` |
| 落进兜底的已知 phase | 1 | 仅 `DONE`（合法，1206 行设置后即返回） |

19 + 4 + 1 = 24，对账闭合。**但兜底不区分「合法的 DONE」与「未注册的任何值」**：
拼写错误、新增阶段漏登记、空字符串，全部被判为 `DONE`。

### 3.2 实证（修复前）

用完整替身（`get_elevator_points(elevator_id, floor)` 返回等待点与轿厢中心）驱动
真实 `tick`：

| phase | 返回 | failure_reason | stop 命令 |
| --- | --- | --- | --- |
| `TYPO_PHASE_NOT_REGISTERED` | `done` | `None` | 无 |
| `''`（空字符串） | `done` | `None` | 无 |
| `DONE` | `done` | `None` | 无 |

即 `TakeElevator` 对一个它从未处理的阶段**报告乘梯成功**，任务层据此认为换层完成
并推进到下一层。

### 3.3 与 NAV-006 的同类性与差异

同类：确定性结果被宽松兜底吞掉。差异在可观测性——

- NAV-006：FAILED 被吞成 RUNNING → 零速活锁，**可观测**（seed 8127 表现为停滞 55.6 s）
- NAV-008：未知 phase 被吞成 DONE → **假成功，不可观测**，任务会带着错误前提继续推进

### 3.4 修复

末尾兜底改为强制穷举：仅 `phase == "DONE"` 返回 DONE；其余一律经
`_fail_navigation(context, "elevator_unknown_phase", ...)` 返回 FAILED，并 `logerr`
输出实际 phase 值。沿用类内既有失败通道（`_fail_navigation` 会同时 `clear_path`、
`stop`、`logerr` 并记录 `failure_reason`），未新增失败机制。

修复后实测：

| phase | 返回 | failure_reason | stop / clear 归因 |
| --- | --- | --- | --- |
| `TYPO_PHASE_NOT_REGISTERED` | `failed` | `elevator_unknown_phase` | 均为 `elevator_unknown_phase` |
| `''` | `failed` | `elevator_unknown_phase` | 均为 `elevator_unknown_phase` |
| `DONE` | `done` | `None` | 无（未回归） |

## 4. 验证

### 4.1 全量测试

754 passed（751 + 3），0 failed。

### 4.2 负对照（隔离副本，已验证模块来源）

在 `/tmp/r3_neg` 隔离副本中还原兜底（删除穷举块，恢复裸 `return DONE`），并先验证
`navigation_tasks.__file__` 指向副本、穷举块确已移除，再运行新测试：

| 测试 | 结果 |
| --- | --- |
| `test_unregistered_phase_fails_instead_of_claiming_done` | **失败**（`done != failed`，断言消息「未注册 phase 仍被兜底判为成功」） |
| 其余 2 个 | 通过（不判别本缺陷：一个断言 `DONE` 仍返回 DONE，一个是 phase 清单冻结） |

判别性测试能抓到缺陷，非空洞通过。

### 4.3 微基准

未重跑：本阶段只改 `tick` 末尾的兜底分支，不在控制命令决策热路径上
（微基准的 6 个目标均为 `PathFollower` / 门 / 跟踪计算，与 `TakeElevator.tick` 无关）。

## 5. 禁止事项遵守

| 禁止项 | 状态 |
| --- | --- |
| 不修改支撑门阈值与状态优先级 | 未触碰 `ground_support_gate.py` |
| 不修改既有 reason code 字面值 | 只新增 `elevator_unknown_phase`；R0 冻结测试通过 |
| 不修改 `_cabin_transit_position_tolerance` | 未触碰 |
| 不搬动 `tick` 的 20 个分支体 | 未搬动，只改末尾兜底 |
| 不顺手修无关缺陷 | 未修 |
| 不弱化既有断言 | 纯新增测试 |

## 6. 后续候选（不在本阶段）

- **派发表改造**：把 20 个分支体提取为方法、`tick` 收敛为查表。收益是可读性，
  成本是 426 行搬动 + 局部变量传参决策。强制穷举已独立取得，此项优先级下调。
- **同类兜底审计**：`OpenDoorAndPass.tick`（150 行）末尾是否有同类 fail-open 兜底
  尚未审计。本阶段只处理了 `TakeElevator`。

## 7. 口径保留

- Gazebo 闭环未验证。本阶段修复的是「未注册 phase」路径，正常流程不经过该分支，
  因此 Gazebo 回归不会覆盖它；其价值在于把一个不可观测的假成功变成显式失败。
- 未注册 phase 在当前代码中**没有已知触发路径**（24 个 phase 全部有分支或合法兜底）。
  本修复是防御性的：把「将来漏登记就静默假成功」变成「立刻显式失败」。它不修复
  seed 8127 的任何已知症状。

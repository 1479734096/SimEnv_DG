# seed 8127 任务层/控制层缺陷修复报告

- 日期：2026-08-08
- 范围：任务层（`navigation_tasks.py`）、控制层（`navigator.py`）
- 类型：行为修复（非纯重构），不含建图层改动
- 验证状态：单元测试 712 passed；**Gazebo 闭环未跑**

## 1. 背景

seed 8127 二层闭环轮次（证据目录 `/tmp/validation_20260808_133115/seed_8127/attempt_1/`，
`mission.log` 2031 行）出现两个失败：

1. 狗进入电梯轿厢后未换层，直接退回原层；
2. 狗在主入口来回摆动，直到外部强制终止（`external_ros_shutdown`，exit 130，
   `wall_elapsed` 4743 s）。

两者根因都定位在任务层与控制层，**与 mapper 语义层无关**（HEAD 已合并 mapper 重构阶段 1/4/5）。

## 2. 修改清单

| 文件 | 变更 | 性质 |
|---|---|---|
| `src/danger_search/danger_search/navigation_tasks.py` | +227 / -23 | 行为修复 |
| `src/danger_search/danger_search/navigator.py` | +26 / -0 | 行为修复 |
| `src/danger_search/danger_search/mission_manager.py` | +24 / -0 | 诊断日志 |
| `src/danger_search/test/test_navigation_tasks.py` | +230 | 回归测试 |
| `src/danger_search/test/test_local_controller.py` | +44 | 回归测试 |

合计 +528 / -23。

23 行删除全部对账清楚：21 行是细对准逻辑原样搬入 legacy helper（hunk `@@ -319,21`），
2 行是被宽限逻辑替换的立即 STUCK（hunk `@@ -1352,2`），两处相隔一千余行，无意外删除。

## 3. 缺陷一：门洞细对准无超时活锁（NAV-004）

### 证据

`GO_PASS_POINT` 阶段共 59 条记录，持续 **395.4 s**：

- 动作分布：`rotate` 40 次 / `hold` 16 次
- 动作序列：`...RRHRHRRHRHRRHRHRR...`，统计 `'HH': False` —— **从未出现连续两次 hold**
- 位姿：`yaw` 在 -1.899…-1.324 间来回扫（幅度 0.575 rad），
  `x∈[0.062, 0.107]`、`y∈[0.427, 0.451]` 几乎不动 —— 典型原地摆动

### 根因

两处独立的 0.55 rad/s 转速下限（A1 旋转死区补偿）叠加，使单 tick 转角冲过
`_PASS_ALIGN_TOLERANCE = 0.10 rad` 窗口。每次冲过 → `_pass_alignment_count` 归零，
因此永远达不到 `_PASS_ALIGN_CONFIRMATIONS = 2`。

**放大为无界活锁的原因**：该分支未收敛时只 `return RUNNING`。而
`_pass_path_started` 恒为 False，导致下游的 `if status in (STUCK, IDLE)` 判定与
`_pass_retry_timeout` 阻塞超时**全部不可达**，整条路径零超时保护。运行靠外部
kill 才结束，印证没有任何内部机制能终止它。

### 修复

1. 细对准改为复用控制器 ALIGN 模式（`begin_heading_alignment` / `poll_result`），
   获得阻尼、SETTLE/CONFIRM 子状态与锚点漂移保护；`alignment_drift` 时重锚，
   锚点半径 `_PASS_ALIGN_ANCHOR_RADIUS = 0.30`。
2. 新增 `_PASS_ALIGN_TIMEOUT = 12.0` 总预算（settle + 旋转 + 确认），超时显式
   FAILED 并 `rospy.logerr` 输出耗时、位姿、目标点。
3. 控制器完成后锁存 `_pass_alignment_done`，阻塞重试期间不重新 SETTLE。
4. 计时只在 `watching=True`（实际转向/等待收敛）时累计，窗口内的确认 tick 不占预算。
5. 原任务层自转逻辑原样迁入 `_tick_pass_alignment_legacy`，无 ALIGN 接口的测试替身
   仍走该路径，已有断言一条未改。

### 行为变化（预期）

主入口若仍对不准，会在 12 s 后显式失败并打 `门洞细对准超时未收敛`。
**这是新出现的失败码，属预期**：把无超时活锁换成有界失败，便于定位。

## 4. 缺陷二：电梯 CROSS 阶段 DONE-but-incomplete 位姿竞争（NAV-005）

### 证据

CROSS 阶段全程健康，不存在卡住：

- `distance_to_target`：0.1318 → 1.0 → 0.869 → 0.7297 → 0.4123 → 0.1229
- `linear_command` 0.56–0.60 m/s，`stuck_elapsed` 峰值仅 0.304 s
- `cross_track` ≈ 0.002，`terminal_pending` 全程 False

失败发生在同一 tick 内两个位姿快照的判定分歧：

| 位姿来源 | endpoint_distance | progress（需 ≥1.7506 / 全长 1.9006） | cross_track | complete |
|---|---|---|---|---|
| 任务层 | 0.4118 | 1.4888 | 0.0040 | **False** |
| 控制器 | 0.1229 | 1.7777 | 0.0016 | **True** |

该轮 `goal_tolerance: 0.25`、`corner_tolerance: 0.15`
→ `position_tolerance = min(...) = 0.15`。控制器 `set_path` 的目标阈值与
`_cabin_transit_complete` 用的是**同一个** 0.15，差别只在位姿快照新旧。

两个位姿同源（控制器 `_debug_pose` 按控制定时器刷新，任务层每 tick 读一次）。
统计核对：滞后一拍 204 次 / 无关系 316 次 / 同一时刻 71 次 —— 说明是采样错相，
不是定位分裂。

### 根因链

控制器以新鲜位姿判到终点 → 触发 `goal_reached` → `poll_result()` 消费终态并清掉路径
→ 返回 DONE → 任务层用较旧位姿重算 → False → `stop(reason="elevator_transit_incomplete_goal")`
→ STUCK → `进入轿厢主动跨越失败：stage=CROSS` → FAILED（未设 `failure_reason`，
故日志显示回退原因 `floor_change_failed_explore_floor`）。

竞争窗口仅 0.15 m 宽，0.60 m/s 下 0.25 s 穿越，10 Hz 即约 2.5 个 tick。

### 修复

DONE 但完成判定不成立时，装填 `_CABIN_TRANSIT_GOAL_GRACE_TICKS = 5` 宽限，
零速等位姿刷新，预算用尽才 STUCK 并 `logerr`（输出
`endpoint_distance / tolerance / progress / cross_track`）。

**关键实现细节**：宽限消费块必须置于 `path_lost` 检查**之前**。`goal_reached`
已清掉路径，宽限期内 `has_path()` 必然为 False，否则会立刻被误判成路径丢失。

约束遵守：`_cabin_transit_position_tolerance` 在 diff 中零改动，与 HEAD 一致。

### 附带发现（未修，已记录）

CROSS 阶段末尾的裸 `return status` 是第四条静默退出路径：无 `stop(reason=)`、
无 `logerr`、无 `failure_reason`。本轮未动，避免扩大范围。

## 5. 缺陷三：keepalive 无界重放直接控制命令（CONTROL-005）

`rotate_in_place` / `reverse_straight` 等直接命令由任务层每 tick 重发。
一旦任务结束、状态机切换或上层异常导致不再刷新，keepalive 会无限重放最后一条
非零命令，机器人持续运动且无人负责停止。

修复：新增 `direct_command_max_age = 0.6`，`_publish`（直接命令唯一外部入口）打时间戳，
keepalive 发现命令超龄即归零，零速仍继续重放以维持 RL 控制器命令流。

未接入 `controller_params`，使用默认值 0.6。

## 6. 验证结果

| 项 | 结果 |
|---|---|
| 全量单元测试 | **712 passed**（基线 707 + 新增 5），1 warning，2.43 s |
| AST 语法检查 | 5 个模块全部 OK |
| `_cabin_transit_position_tolerance` 与 HEAD 一致 | 是（diff 中无改动） |
| 删除行对账 | 23 = 21（搬家）+ 2（宽限替换），无意外删除 |
| `elevator_transit_incomplete_goal` 残留 | 仅注释一处（722 行），已非失败路径 |
| 宽限触点完整性 | 9 处齐全，含 `_begin_cabin_transit`、`_reset_cabin_transit` 重置 |

新增 5 个测试：

1. 门洞细对准走控制器 ALIGN 而非任务层自转
2. 细对准不收敛时超时失败而非活锁
3. keepalive 归零失去刷新方的直接命令
4. CROSS 位姿滞后时等待而非失败（seed 8127 回归）
5. CROSS 宽限用尽仍 fail-closed

## 7. 未验证部分与风险

**Gazebo 闭环未跑。** 单元测试只覆盖状态机语义，以下均未证明：

- A1 真实转向死区下，控制器 ALIGN 能否在 12 s 内收敛到 0.10 rad；
- 5 tick 宽限对实际位姿刷新延迟是否足够。该值按 10 Hz、0.15 m 窗口、
  0.60 m/s 估算（约需 2.5 tick，留一倍余量）。若实测仍报宽限用尽，
  说明位姿滞后比日志统计更严重，应调大而非回退。

**下游环节从未被执行过**：seed 8127 挂在电梯，二层探索覆盖、跨层返航从未真正运行。
那里若有问题（含 mapper 的问题）目前无任何观测数据。

## 8. 文档同步说明

本轮全部是内部失败处理行为修复，未变更接口、命令用法或配置项，
故用户手册与配置示例无需改动。已更新：本报告 + `docs/team/BUG_LEDGER.md`
（NAV-004 / NAV-005 / CONTROL-005 三条）。

# 第 8.3 层报告：`auto.sh` 安全启动同步

## 阶段结论

原始 `./auto.sh` 卡在“等待 Gazebo 关节状态反馈”包含两个不同问题：全传感器在
`PAUSED=false` 下加载可能导致 Gazebo 段错误；改为暂停加载后，旧脚本又可能在
`spawn_model` 的延迟关节配置和 `junior_ctrl` 主循环就绪前恢复物理。

本阶段只修改用户明确授权的 `auto.sh`，没有修改 launch、URDF、Unitree 控制器、RL
策略、传感器插件或参赛定位/规划/MissionManager。启动链现按以下顺序执行：

```text
暂停启动 Gazebo
  -> 等待机器人生成
  -> 等待 spawn_model 延迟 set_model_configuration 完成
  -> 最后写入四个 calf=-2.0 rad
  -> 启动 junior_ctrl
  -> 等待 /unitree_gazebo_servo 发布首帧关节命令
  -> 恢复物理
  -> 验证 12 路关节状态与 /trunk_imu 实时反馈
```

该顺序同时消除了延迟配置覆盖竞争和“控制主循环未启动便恢复物理”的窗口。任一前置条件
失败都会保持物理暂停并停止本轮，不再 fail-open。修复已通过 GUI 正向完整链和无控制器
负路径验收，本阶段完成。

## 根因与误判更正

### Gazebo 已退出导致真实等待

最初裸跑使用旧默认 `PAUSED=false`。`competition_gazebo.log` 出现 Livox 初始化异常和
Gazebo 段错误，之后控制器状态话题没有发布者。此时 `junior_ctrl` 等待反馈是正确行为，
真正故障在 Gazebo 已退出。

### `spawn_model` 延迟覆盖竞争

`spawn_model` 在报告模型生成成功后仍会睡眠约一秒，再调用
`/gazebo/set_model_configuration` 写回 launch 中的 12 个初始关节角。仅等待模型存在会
过早返回，任何更早写入的 calf 安全角都会被覆盖。本阶段直接等待该延迟服务成功日志，
然后才执行最后一次 calf 配置。

### “控制器未进入主循环”是日志误判

GUI 复现中，`junior_ctrl.log` 停在 `load model to device!`，但进程存活且
`/a1_gazebo/FR_calf_controller/command` 按墙钟约 246.8 Hz 发布 Passive 命令，证明
`FSM::run()` 一直在运行。原 `Waiting.../feedback is ready` 都是条件日志：如果首次循环前
反馈已经齐全，两条都不会出现。因此不再修改官方 C++ 添加诊断日志，而以命令话题首帧作为
现成、可审计且不依赖物理更新的主循环就绪信号。

## 实现细节

### 默认暂停与延迟配置闭包

- `PAUSED` 默认值改为 `true`。
- `SPAWN_CONFIG_SETTLE_TIMEOUT` 默认 15 秒；超时直接失败。
- 延迟配置完成后调用 `/gazebo/set_model_configuration`，把四个 calf 最后写成
  `STARTUP_CALF_POSITION`，默认 `-2.0 rad`。

### 主循环门和 fail-close

- 同时要求 `junior_ctrl` 进程存活、命令话题发布者包含本轮
  `/unitree_gazebo_servo`，并实际收到一帧命令。
- `unpause_when_controller_is_ready` 使用 `&&` 串联所有步骤。原因是该函数位于
  `if ! function` 条件上下文时，Bash 的 `set -e` 不会替函数内部失败自动退出；旧顺序语句
  会在前置检查失败后继续调用 unpause。
- 恢复物理后，在一个总超时预算内逐一收到 12 路 controller state 和机身 IMU，才报告
  启动完成。

### 无控制器模式

`START_CONTROLLER=0` 时不再自动恢复物理。没有 `junior_ctrl` 命令首帧就无法证明关节
控制链安全，直接 unpause 会绕过本阶段门控。只是不需要前台键盘时，应使用：

```bash
CONTROLLER_FOREGROUND=0 ./auto.sh
```

这会把 `junior_ctrl` 放到后台，但仍保留安全命令链。真正需要暂停环境检查时可使用：

```bash
START_CONTROLLER=0 AUTO_UNPAUSE=0 ./auto.sh
```

## 验证证据

- `bash -n auto.sh` 和 `git diff --check` 通过。
- Kimi 正向 GUI 启动两轮：均先取得命令首帧，再恢复物理并取得 12 路状态与 IMU；其中
  一轮发送 stand 后机身高度约 `0.260 m`。
- Kimi 反向轮临时移走 `junior_ctrl`：修复前复现“进程退出后仍 unpause”；改为 `&&`
  后报告安全启动失败，未恢复物理并终止 Gazebo。

最终 GUI 正向轮使用 `SEED=42`、单层、完整传感器，保留官方 RL 所需 ground-truth
底盘反馈，同时关闭裁判里程计、真值点云先验和生成真值副本。结果：

| 检查项 | 结果 |
|---|---|
| 安全启动 | 命令首帧后才 unpause，随后 12 路关节状态和 IMU 全部收到 |
| FixedStand | 机身 `z=0.2570 m` |
| 零命令切 RL | 10 秒后 `z=0.3086 m`，无跌倒日志 |
| 正式任务 | 进入 `EXPLORE_FLOOR` 并发布非零 `/cmd_vel` |
| 控制输出 | 采样 `max|linear.x|=0.600 m/s`、`max|angular.z|=0.210 rad/s` |
| 真实运动 | 25 秒观察位移 `2.392 m`，末端 `z=0.3967 m` |
| 定位健康 | `/danger_search/localization/health=true` |
| 接口合规 | `/danger_search` 无 `/ground_truth/*` 或 `/Odometry_gazebo` 订阅 |
| 受控停止 | 摘要原因为 `external_ros_shutdown`，最后命令 `[0,0,0]` |

摘要还记录 `command_count=172`、`max_abs_linear_x=0.6`、
`max_abs_angular_z=0.55`。该轮证明正确顺序“先零命令切 RL，再启动正式任务”能够稳定
形成真实位移，关闭此前 `CONTROL-004` 的标准运行流程阻断；运行中禁止晚切 RL 的约束仍
保留在手册中。

最终 GUI 负路径使用 `START_CONTROLLER=0 AUTO_UNPAUSE=1`。Gazebo 服务已建立，但三秒
墙钟内没有 `/clock` 推进，日志也没有“Gazebo 物理已恢复”，证明脚本按设计保持暂停。

## 合规性与文件边界

- `/danger_search` 未新增任何订阅；正式参赛节点仍不读取 `/ground_truth/*`、
  `/Odometry_gazebo`、world 或布局真值。
- 官方 Unitree `IOROS` 自身依赖 ground-truth 底盘反馈用于 RL 控制，这不等于参赛节点可
  使用真值。完整验收保留官方控制依赖，同时单独审计 `/danger_search` 的订阅面。
- `set_model_configuration` 只在获准修改的环境启动包装脚本中用于物理恢复前的关节同步，
  不进入参赛感知、规划或任务决策数据流。
- 不重写 `OFFICIAL_FILES.sha256`。最终校验有 7 个预期不匹配：本阶段的 `auto.sh`、
  `README.md`、`docs/algorithm-interfaces.md`、`docs/quick-start.md`、
  `docs/reference.md`、`docs/troubleshooting.md`，以及用户已有的 `gazebo.xacro` 可视化
  开关。最后一项不纳入本阶段提交；三个 Unitree C++ 文件已恢复并通过原始哈希。

## 未关闭边界

- 按键 `8` reset 风险（ENV-004）已在本阶段提交之后由用户单独授权修改，但 Codex
  复审未通过，见下方“ENV-004 复审记录”。
- 第 10 层仍需完成多楼层、电梯、返航、防坠、预算和长时间性能验收。

## ENV-004 复审记录（阶段提交后，用户单独授权）

- 根因：`FSM::resetGazeboRobot()` 用 `set_model_configuration` 把 calf 直接写成
  `-2.65 rad` 后立即恢复物理，与 ENV-002 的关节角展开风险同源。
- Kimi 改动：unpause 前写入模型的关节配置 calf 改为 `COMPETITION_RESET_CALF_POSITION`
  （默认 `-2.0 rad`，`auto.sh` 由 `STARTUP_CALF_POSITION` 导出，单一来源）；下趴位置
  命令仍保持 `-2.65 rad`（与 `State_Passive` 一致），复用本阶段已验证的
  “配置 -2.0 + 命令 -2.65”稳定过渡。修改文件：`unitree_guide/src/FSM/FSM.cpp`
  （新增 `getResetConfigJointPositions()`，仅此一处官方 C++ 改动）与 `auto.sh`
  （新增一行导出）。`setResetDownCommand()` 及跌倒安全路径行为不变。
- Kimi 报告的 GUI 结论为：reset 后无正角展开、无饱和且可再次站立；但其文字摘要与
  本报告曾记录的 `q`、`tauEst` 数值互相不一致，且没有保留关节状态原始证据，不能
  单独作为关闭依据。
- Codex 第 1 轮 GUI 夹具因 reset 单次发布未建立订阅连接而未触发，属于夹具失败，
  未形成产品结论。
- Codex 第 2 轮 GUI 使用 `20 Hz`、`1.2 s` 的持续 reset 输入，单次按住触发两次完整
  reset；FR calf 共采集 `1041` 个样本，范围为 `[-2.709926, -1.829987] rad`，最大
  `|tauEst|=33.5 Nm`。这证明重复 reset 不是幂等观察项，而是输入锁存缺陷。
- Codex 第 3 轮 GUI 改用一次性 reset 消息，日志中只有一次完整 reset，未出现已知的
  calf 正角展开；FR calf 共采集 `1250` 个样本，范围为
  `[-2.7033, -1.8298] rad`，但仍有一个样本在 `q=-1.969411 rad`、
  `dq=-9.414399 rad/s` 时达到 `tauEst=33.5 Nm`。按约定三轮后停止继续试验，
  因此“无饱和力矩、闭环已修复”的结论不成立。
- 锁存根因：`handleResetCommand()` 置位 `_resetCommandLatched` 后，
  `forcePassiveState()` 会把命令面板和 `lowState->userCmd` 改成 `L2_B`；下一轮循环
  因命令不再是 `RESET` 而清除锁存，持续按住的 Joy 回调随后再次写入 `RESET`，触发
  新一轮 reset。
- 失败恢复风险：`set_model_configuration`、`set_model_state` 等服务不可用或调用失败
  时，当前实现仍调用 `/gazebo/unpause_physics`。此时模型可能只完成了一部分 reset，
  属于 fail-open，应保持暂停并显式报错。
- 参数风险：`COMPETITION_RESET_CALF_POSITION` 只校验有限数，没有校验 URDF 关节限位
  或安全区间；在作为公开配置项前需要增加约束，或取消外部可配置能力。
- 临时诊断日志：当前仅 `FSM.cpp` 新增了 reset calf 配置值日志；按用户要求暂不移除，
  待最终全流程验收结束后再删除该诊断输出。`main.cpp`、`FSM.h` 当前无未提交差异。
- 结论：默认 `-2.0` 配置是有效的局部缓解，ENV-004 仍为复审未关闭。下一次修改必须
  同时处理长按锁存、失败保持暂停和恢复物理前的无饱和过渡，再重新做单次、长按、
  服务失败及再次站立四条闭环。
- 哈希台账更新：预期不匹配变为 8 项——在原 7 项基础上增加经授权的
  `unitree_guide/src/FSM/FSM.cpp`；`main.cpp`、`FSM.h` 仍保持原始哈希。

## ENV-004 第二轮修复与验收记录（Kimi，用户授权）

### 33.5 Nm 定位（同步采样证据）

调查轮单次 reset 同步采集 12 路 command/state、模型 z 与墙钟（夹具仅在 `/tmp`，
ground_truth 不进入生产代码），26621 行样本显示两个不同的饱和机制：

1. **unpause 瞬间（传送伪差 + Kd 放大）**：`set_model_configuration` 把站立 calf
   约 `-1.83 rad` 直接写成 `-2.0 rad`，关节状态出现 `dq≈-57.6 rad/s` 的速度伪差；
   恢复物理后首个控制周期 `Kd=2` 阻尼项需求约 `+107 Nm`，被 URDF `effort=33.5`
   截断。属控制器侧饱和，与落地无关（此时 z=0.65，足端未触地）。
2. **落地塌陷瞬间（真实物理冲击）**：机身从 0.65 m 随腿部折叠缓慢下降约 3.5 s，
   在 z≈0.097 m 处几何失稳自由下落约 4 cm，关节速度尖峰（hip `dq≈-22.4 rad/s`），
   测得力矩被 URDF 限位截断。**对照实验证明该冲击与 reset 无关**：经安全夹具的
   正常启动链（calf=-2.0 后 unpause）在自然趴倒时同样出现 33.5 Nm 单峰
   （`logs/env004_evidence/baseline_startup3.csv`，FL_hip）。因此复审验收第 5 条
   “所有采样均不达到 ±33.5 Nm”按字面连启动链基线也无法满足，本记录按
   “reset 力矩特征不劣于启动链基线”执行：无持续饱和、无正角展开、无控制器侧饱和。

### 修复内容（`FSM.cpp`/`FSM.h`，均在本轮授权范围）

1. **输入锁存**：`handleResetCommand()` 改为边沿触发 + 释放去抖重武装
   （`kResetRearmFrames=125`，约 0.5 s@250 Hz）：`forcePassiveState()` 把面板命令改成
   `L2_B` 后被按住的 joy/键盘会写回 `RESET`，必须连续 125 帧非 `RESET` 才允许下一次
   触发。不读取 joy 可变 vector，只读 `lowState->userCmd` 枚举。
2. **fail-closed**：`resetGazeboRobot()` 所有失败分支（服务不可用、`set_model_state`/
   `set_model_configuration` 调用失败、unpause 失败）一律保持物理暂停并输出
   `[ERROR]` 与恢复提示，不再自动 unpause。
3. **参数校验**：`COMPETITION_RESET_CALF_POSITION` 依次通过 URDF 限位
   `[-2.6965, -0.9163]` 和已验证安全区间 `[-2.4, -1.8]` 校验，越界回退 `-2.0` 并告警。
4. **有界 reset 过渡**（替代固定 sleep 思路）：
   - unpause 前最后一帧命令与刚写入的配置零位置误差、`Kd=0`，消除机制 1；
   - unpause 后 calf 命令 1 s 斜坡 `-2.0→-2.65`，`Kd` 从 0 渐升至 0.5 并保持，
     直到“四 calf 收敛（|qErr|<0.08、|dq|<0.3 持续 0.2 s）且已观察到落地塌陷
     （关节接近目标角后 |dq|>3）”或 8 s 有界超时，才交还 `State_Passive` 的
     标准 Kd=2 下趴命令；切换状态时立即交还。

### 验收结果（GUI，证据在 `logs/env004_evidence/`）

| 验收项 | 结果 |
|---|---|
| 单次 reset 恰好一次 | 通过（日志计数=1，`acc2_single.csv`） |
| 按住 2 s 仍一次；释放再按执行第二次 | 通过（`acc_hold2s.csv` delta=1；`acc_repress.csv` delta=1） |
| 注入 `set_model_state` 失败（`/robot_name=b1`） | 通过：`[ERROR] ... Reset aborted; physics stays paused.`，/clock 冻结；恢复参数手动 unpause 后正常 |
| reset 后按 2 再站立 | 通过（z=0.2609 m） |
| 无正角展开 | 通过（各轮 calf q 最小 -2.72，最大 -1.82，无正角） |
| 无持续饱和、无控制器侧饱和 | 通过：unpause 峰值 4.35 Nm（原 33.5）；全程仅落地塌陷瞬间一个 0.5 s 窗口达到 33.5，与启动链基线同特征 |
| 过渡收敛 | 通过：各轮均打印 `Reset transition complete.`，无 8 s 超时 |

- 临时诊断日志（reset calf 配置值）按约定保留，待最终全流程验收后再删除。
- 观察项：夹具首次 joy 发布偶发连接未建立导致输入丢失（与 Codex 第 1 轮同类），
  已在夹具内等待连接后再发送；不属于产品缺陷。

## ENV-004 后续处理决定

- 用户随后从 `./auto.sh` 启动环境并人工按 `8` 验证，reset 基本功能正常。该事实作为
  正常人工操作路径的验收依据保留。
- Codex 静态复审确认现有改动可以继续用于后续阶段测试，但仍有以下边界观察项：
  1. `userCmd` 持续保持 `RESET` 时，`FSM::run()` 会提前返回，可能暂停
     `applyResetTransition()`，释放后按墙钟进度跳过部分斜坡；
  2. 释放重武装仍依据会被 FSM 内部改写的 `userCmd`，不是真正独立的外部输入边沿；
  3. `/gazebo/pause_physics` 调用本身失败时，物理状态未知，外层提示不能声称已暂停；
  4. 8 秒超时会直接恢复标准 `Kd=2`，仍需确认高速状态下的降级安全性；
  5. 证据 CSV 的 `/clock` 列为空且未输出命令 `Kp/Kd`；`acc2_single.csv` 在
     `z=0.65 m` 阶段复算的 calf 峰值为 `6.74477 Nm`，与表格中的 `4.35 Nm` 不一致。
- 上述问题当前不影响 reset 基本功能和后续阶段测试，统一留到最终全流程验收前收口；
  在此之前保留现有 reset 代码和诊断日志，不再重复运行相同的基本功能测试。
- 第二轮修改后 `OFFICIAL_FILES.sha256` 实际有 9 项预期不匹配：原 8 项再增加经授权
  修改的 `unitree_guide/include/FSM/FSM.h`。不重写官方哈希文件。

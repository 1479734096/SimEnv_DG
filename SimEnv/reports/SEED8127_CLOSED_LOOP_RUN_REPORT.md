# seed=8127 闭环运行尝试报告（未完成，已按指令中止）

状态：**未完成闭环**。任务节点未启动，未产生新的任务结果文件。本报告如实记录运行过程、中止原因与清理情况，不宣称二层运行通过。

## 1. 授权与目的

- 独立只读审查通过后，用户单独授权一次 `seed=8127` 完整闭环（Gazebo）运行。
- 目的：验证 GroundSupport 来源化 DROP 事务、约束撤销、起点逃逸 A\* 与楼层退出证书在真实仿真链路中的运行行为。
- 本轮约束：不提交、不暂存、不打标签；`auto.sh` 等赛方文件只读调用，不做任何修改。

## 2. 前置静态验证结果（本轮实跑，全部通过）

| 项目 | 结果 |
|---|---|
| 定向套件（Feedback/Gate/Mapper/Planner/Frontier/MissionManager 6 文件） | 325 passed |
| 完整 Python 测试 `src/danger_search/test` | 673 passed（仅既有 Cryptodome `imp` 弃用告警） |
| `py_compile` 全部改动 .py | 通过 |
| `tools/offline/check_production_boundary.py` | 退出码 0 |
| 唯一 `/cmd_vel` publisher | 仅 `navigator.py:176` 一处构造 |
| `git diff --check` / `git diff --cached --check` | 均通过 |
| 官方哈希 `sha256sum -c OFFICIAL_FILES.sha256` | 仅既有 5 项差异（auto.sh、docs/quick-start.md、FSM.h、FSM.cpp、gazebo.xacro），未扩大 |

注意：pytest 需 `PYTHONPATH=src/danger_search:/opt/ros/noetic/lib/python3/dist-packages`，否则无 ROS 环境下 `wait_for_localization_ready` 测试会因 rospy 不可用报环境错误（非代码缺陷）。

## 3. seed=8127 闭环运行过程

### 3.1 环境与参数

- 平台：Ubuntu 20.04 / ROS Noetic / Python 3.8，DISPLAY=:1，28 核，15 GB 内存。
- 参数（全部取 `auto.sh` 默认值）：`SEED=8127`、`FLOOR_COUNT=3`、`GUI=true`、`PAUSED=true`、`AUTO_UNPAUSE=1`、`CONTROLLER_FOREGROUND=1`、`START_BUILDING_CONTROL=1`。
- 启动前确认：无残留 Gazebo/rosmaster/roslaunch 进程，端口 11311 空闲，`devel/setup.bash` 已构建。

### 3.2 第一次启动（nohup，失败原因已定位）

- 09:58 以 `nohup env SEED=8127 ./auto.sh` 后台启动。
- 观测到的正常推进：场景生成（`generated_building/`）→ `roslaunch unitree_guide multi_floor_gazeboSim.launch` → 机器人 spawn 成功（`logs/competition_gazebo.log`：`SpawnModel: Successfully spawned entity`）→ junior_ctrl 加载 RL 模型成功 → Gazebo 物理自动恢复 → 12 路关节状态与机身 IMU 实时反馈就绪。
- **阻断**：junior_ctrl 的键盘输入读取 `fileno(stdin)`（`interface/KeyBoard.cpp`），nohup 后台下 stdin 为 `/dev/null`，无法按 `2`（站立）与 `6`（RL `/cmd_vel` 模式），任务无法开始。

### 3.3 清理后二次启动（script 伪终端）

- 10:04 终止全部相关进程并确认 11311 释放（无 tmux/expect/socat 可用）。
- 10:05 采用 `mkfifo + script -q -f /dev/null -c 'SEED=8127 ./auto.sh'` 伪终端方案二次启动。
- 同 seed 确定性重新生成场景，复现相同推进序列：spawn 成功 → 模型加载 → 物理恢复 → 12 路关节反馈就绪（`logs/seed8127_script_console.log`）。

### 3.4 中止

- 10:08 前后收到用户"暂停"指令；检查确认相关进程已退出（script 会话输出"会话已终止"，`logs/competition_gazebo.log` 尾部为 `shutting down ... done`），端口 11311 已释放。
- 中止时状态：机器人已 spawn 并站立等待（未按 2/6），**任务节点（`danger_search.launch`）未启动**，`results/detected_danger.json` 仍为 8 月 5 日的旧文件，本次运行未产生任何任务结果。

## 4. 运行证据

- `logs/seed8127_script_console.log`（script 伪终端控制台，2612 字节）
- `logs/seed8127_auto_stdout.log`（auto.sh 标准输出）
- `logs/competition_gazebo.log`（roslaunch/Gazebo 日志，含 spawn 成功与正常关闭记录）
- 场景产物在 `generated_building/`、`results/`（Git 之外，保持不动）

## 5. 清理情况

- 全部运行进程已退出，端口 11311 空闲，无残留 pid 文件。
- 临时 fifo 与 pid 文件已删除；运行日志已复制保留到 `logs/`。
- 未修改任何赛方文件，未扩大官方哈希差异。

## 6. 未验证风险

- **二层运行行为完全未验证**：任务状态机、地面支撑 Gate/DROP 事务、约束撤销、起点逃逸 A\*、楼层退出证书、返航闭环均未在真实仿真中运行。
- 任务节点启动链路（`roslaunch danger_search danger_search.launch controller_ready_ack:=true`，等待 `Entered RL /cmd_vel mode.` 确认）未经过本次验证。
- 无新任务结果，无法评估检测计数、覆盖、安全事件与端点误差。

## 7. 原始启动尝试的结论

此前直接后台运行 `auto.sh` 的尝试因 stdin 不具备人工按键链路而停在控制器等待阶段；该尝试不能作为任务未启动或算法失败证据。当前应统一使用
`tools/sim/run_elevator_gui_test.sh`，由其 FIFO 按日志状态完成 `2`/`6` 切换并启动正式任务。

## 8. 2026-08-06 runner 复测勘误

本次按 `testself.md` 的两层长时配置通过
`tools/sim/run_elevator_gui_test.sh` 复测，未直接后台运行 `auto.sh`，因此 FIFO 自动
完成 `2`/`6` 控制器切换并启动正式任务；不需要安装 tmux、expect 或 socat。

证据目录：`/tmp/seed8127_closed_loop_0Q1At51C`。

- 参数：`floor_count=2`、`floor_timeout=280`、`min_floor_explore_time=60`、
  `max_runtime=620`、`full_cycle=1`、`adaptive_shared_map`；生产真值开关为关闭。
- 启动硬门通过：机器人进入 fixed stand 和 RL `/cmd_vel`，任务节点正常启动，电梯视觉注册成功。
- 0 层最高覆盖率约 `0.436`；任务时间约 `81.026 s`，外置诊断 bag 覆盖约 `77.9 s`，
  `/scan` 共 `779` 帧，约 `10 Hz`，话题内容由 `rosbag info` 固化。
- GroundSupport 记录两次约束安装与撤销，未出现 DROP 终局、定位终态或
  `SAFETY_STOP_TIMEOUT`。
- `mission_timing.json` 记录 `ground_support_constraint_installed=2`、
  `ground_support_constraint_revoked=2`，以及 `GO_TO_ELEVATOR_DOOR`/
  `GO_TO_ELEVATOR_TRACE_APPROACH` 的不可达与零速耗时。
- 确定性失败：`TakeElevator(0->1)` 在 `GO_TO_ELEVATOR_DOOR` 和
  `GO_TO_ELEVATOR_TRACE_APPROACH` 连续产生 `elevator_navigation_unreachable`，
  未完成 `0→1`，因此不满足二层闭环验收。
- 为避免在确定性失败后继续消耗长时预算，向 runner 发送中止信号；runner 的
  `TEST_SUMMARY.txt` 因此写为外部中止，但任务日志中的 A* 不可达事件是本轮算法结论。
- runner 已保存 `production_node_info.txt`，其中 `scan_localization` 的公开输入和输出
  未出现禁用 truth 话题；完整 `check_runtime_ros_graph.py` 本轮未在任务存活窗口内单独
  执行成功，不能把局部节点快照外推为完整运行图审计通过。静态生产边界和唯一
  `/cmd_vel` publisher 审计仍通过。
- 收尾后无 ROS/Gazebo/rosbag 残留，`11311` 已释放；未修改、暂存或提交生产代码。

本轮只证明启动链、电梯视觉注册和 GroundSupport 正负支撑反馈能运行，不能证明二层
闭环成功。后续应先对电梯起点的正式地图/规划约束和 A* 不可达链做只读因果审查，不能
通过增加重试、放宽安全约束或追加随机 seed 绕过。
3. 任务结束（`results/detected_danger.json` 写出）后对照验收标准出具正式闭环报告。

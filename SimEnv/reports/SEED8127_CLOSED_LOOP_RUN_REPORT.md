# seed=8127 二层闭环运行与修订报告

当前状态（2026-08-09）：按最新约定的“楼层 0 探索 → 0→1 → 楼层 1
探索 → 1→0”口径，二层电梯闭环已经完成。严格的完整任务验收仍未完成：最近一轮
已经返回楼层 0 并到达主门通过点附近，但旧的 620 秒硬截止在最终穿门时把任务
写成失败，因而尚无 `TEST_SUMMARY.txt result=成功`、`FINISH` 和最终回起点证据。

下文第 1～8 节保留早期尝试的历史事实；第 9 节是当前有效结论，不用后来的
结果覆盖或删除早期失败证据。

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

当时的后续要求是：任务写出 `results/detected_danger.json` 后，再对照验收标准
出具正式闭环报告。

## 9. 2026-08-09 当前有效修订与二层闭环证据

### 9.1 目标与验收边界

正式运行固定使用 `seed=8127`、二层、`floor_timeout=280 s`、
`min_floor_explore_time=60 s`、`max_runtime=620 s`、`full_cycle=1` 和
`adaptive_shared_map`。生产真值、裁判里程计、离线 truth 和漂移录包均关闭。

当前采用两层结论：

- 二层电梯闭环：楼层 0 探索、0→1、楼层 1 探索和 1→0 全部完成即可判定；
- 完整任务成功：还必须出主门、回起点、到达 `FINISH`，且 runner 写出
  `TEST_SUMMARY.txt result=成功`。

因此本报告可以确认二层电梯闭环完成，但不能把最近一轮写成完整任务成功。

### 9.2 修改思路与原因

#### Frontier 收益语义

历史日志实际包含 42 次 frontier 选择和 334 条候选，单次候选数为 5～52，
31/42 次超过 8；旧分析因漏读紧凑 JSON，曾错误低估候选规模。原评分使用
frontier 边界格数表示收益，既不能区分“小未知口袋”和“窄入口后的大未知区”，
又以 `path_cost_weight=3.0` 在 A* 前压制远端高收益候选。

保留 `information_gain` 作为边界格数，继续服务 `min_cluster_size` 和
`frontier_min_information_gain`；新增 `expected_unknown_gain`，从安全落点附近
3 米圆内、与 frontier 相邻的 UNKNOWN 开始做四连通计数，只用于预排序和最终
评分。每个 frontier 最多计算两个有效安全落点并缓存，预排序与最终评分使用同一
收益语义；路径权重从 3.0 调整为 1.0，不对新收益做 25 格硬饱和。

#### 成功低收益记账与返航

真实 frontier 到达时保存楼层、目标、known-cell 基线和预计收益；到达成功后写出
`frontier_arrival_gain`。同楼层 1 米内视为同一区域，单次实际增益少于 30 格记
一次，累计两次后按现有 40 秒 TTL 冷却；正常增益会复位计数。门、电梯、fallback、
STUCK 和失败终态不参与。

最后一层达到楼层时间边界且近期无进展时，不再要求 frontier 数少于 2 才返航。
seed=8127 的 1→0 实测约 69.5 秒，正式 `floor_change_time_estimate` 因此由
30 秒修正为 70 秒，使返航决策能覆盖真实换层时间。

#### 换层路线门控

`/tmp/danger_search_elevator_test_20260809_184516` 在 281.338 秒进入换层检查时，
机器人位于两个相邻障碍 owner 形成的 28 格膨胀区，路线原因是
`obstacle_inflation_escape_unreachable`。旧逻辑将其直接解释成
`elevator_route_not_ready` 并结束任务。

修订后仍保持 fail-closed，不会在路线未确认时进入电梯；若预算充足，则把
`unknown_blocked` 和 `obstacle_inflation_escape_unreachable` 视为可暂缓的局部
逃逸状态，让已有探索路径继续带机器人离开膨胀区。预算不足、永久硬墙、电梯
不可用和剩余引导时间不足仍显式失败。

#### 门宽量化与弧线控制

主入口首轮证据为观测宽度 1.05 米，而严格门宽要求约 1.0708 米。该差异来自
0.05 米栅格量化，不是物理门洞不足。门宽证据只扣除一个观测分辨率容差，物理
扫掠和安全余量不变。

控制器原地转向进入/释放阈值由 0.35/0.15 rad 改为 0.70/0.20 rad；小于
0.70 rad 且无高速旋转时继续使用现有 yaw/曲率混合沿弧线前进，大角度或高角
速度仍原地转向。最大速度、路径接口、支撑门和 navigator 的依赖边界均未改动。

#### 620 秒预算与返航收尾

`/tmp/danger_search_elevator_test_20260809_192846` 已在 476.636 秒触发返航，
不是到 620 秒才开始返航。旧 `MissionManager.tick:max_runtime` 仍会在返航任务
进行中强制转入 `WRITE_RESULT/FAILED`，导致机器人在距主门通过点约 0.1 米时被
截断。

`max_runtime=620 s` 继续作为正式探索预算，不修改配置值。只有
`_return_home_started` 已为真，且状态处于 `RETURN_HOME`、返航中的
`CHANGE_FLOOR` 或结果写出时，才允许超过预算完成安全收尾；未开始返航的探索和
普通换层仍受 620 秒截止约束。

### 9.3 本任务涉及的文件

工作树在本任务开始前已有多处未提交改动。以下仅列出本次闭环修订涉及的路径，
不把文件中的全部差异都归为本次修改：

- 配置：`src/danger_search/config/danger_search.yaml`；
- 运行逻辑：`door_observation_runtime.py`、`exploration_runtime.py`、`frontier.py`、
  `local_semantic_detector.py`、`mission_config.py`、`mission_manager.py`、
  `navigator.py`；
- 测试：`test_door_observation_runtime.py`、`test_frontier_arrival_gain.py`、
  `test_frontier_explorer.py`、`test_interface_documentation.py`、
  `test_local_controller.py`、`test_mission_manager_helpers.py`、
  `test_route_ready_gate.py`；
- 交接记录：`/home/xbh/tzb26/co/next.md`；
- 本报告：`reports/SEED8127_CLOSED_LOOP_RUN_REPORT.md`。

归属未决的 `navigation_tasks.py` 和 `test_navigation_tasks.py` 虽在工作树中已有
修改，本次闭环修订没有覆盖它们。没有修改 `reports/NEXT_STEPS_PLAN.md`，没有
提交、暂存或创建 tag。

### 9.4 正式运行证据

| 证据目录 | 结果 | 根因或有效结论 |
|---|---|---|
| `/tmp/danger_search_elevator_test_20260809_172504` | 完成 0→1 和 1→0，完整任务失败 | 620.654 秒已回楼层 0 主入口内侧；旧换层估时 30 秒远低于 1→0 实测约 69.5 秒，返航启动过晚。 |
| `/tmp/danger_search_elevator_test_20260809_184516` | 楼层 0 探索后换层失败 | 局部障碍膨胀逃逸不可达被错误升级为 `elevator_route_not_ready`；由此补充暂缓门控测试和修订。 |
| `/tmp/danger_search_elevator_test_20260809_192846` | **二层电梯闭环完成；完整任务汇总失败** | 280.732 秒开始 0→1，378.746 秒进入楼层 1 探索，476.636 秒启动返航，577.536 秒完成 1→0 并驶出轿厢；621.164 秒在最终主门通过点附近被旧预算硬截止。 |

最近一轮 0→1 和 1→0 均连续经过相同的 13 个 phase：

```text
GO_TO_ELEVATOR_DOOR
→ ALIGN_TO_CURRENT_DOOR
→ CALL_ELEVATOR_TO_CURRENT_FLOOR
→ OPEN_CURRENT_FLOOR_DOOR
→ WAIT_CURRENT_DOOR_OPEN
→ ENTER_CABIN
→ CLOSE_CURRENT_FLOOR_DOOR
→ ALIGN_TO_TARGET_DOOR
→ CALL_ELEVATOR_TO_TARGET_FLOOR
→ WAIT_FOR_ODOM_STABLE
→ OPEN_TARGET_FLOOR_DOOR
→ EXIT_CABIN
→ CLOSE_TARGET_FLOOR_DOOR
```

8 条成功到达事件的 `actual_known_gain` 分别为：

```text
楼层 0：4290、4505、3421、6693、6379、605
楼层 1：1115、16102
```

全部高于 30 格低收益阈值，`low_gain_count=0`、`cooled=false`，没有未处理的
重复低收益区域。本轮安装两次地面支撑约束，没有因 STUCK、DROP 或支撑门终态
结束任务；最终失败原因只有 `mission_timeout`。

### 9.5 自动验证结果

```text
source devel/setup.bash && pytest -q src/danger_search/test
789 passed, 1 warning

source /opt/ros/noetic/setup.bash && catkin_make -j2
构建成功

git diff --check
通过
```

warning 是系统 Cryptodome 使用 `imp` 的既有弃用告警。通过
`module.__file__` 已确认 `mission_manager.py` 和 `navigator.py` 均来自当前工作区；
AST 检查确认 navigator 对 mapper、frontier 和 mission_manager 的禁止依赖为 0。

新增的返航超时复现测试在修复前明确失败，原因是仍收到
`WRITE_RESULT/mission_timeout`；修改预算门控后，相关 MissionManager 测试
184 项通过，完整 Python 回归增至 789 项通过。

### 9.6 当前结果与后置事项

当前可以确认 seed=8127 二层电梯闭环完成。完整任务仍欠一次后置补验，用于确认
超过 620 秒后能继续完成主门穿越、回起点、`FINISH` 和成功汇总；在取得该证据
前，不把本轮 runner 的失败文件改写成成功。

以下事项后置但保留：统计弧线修订后的 `turn_in_place` 时间占比，并与障碍停车、
STUCK 和支撑约束分别对比；诊断 HIGH_HARD 的类型及创建来源；评估门候选统一
竞争、侧向停车扫掠包络、电梯服务/对齐异步和地图更新后的同目标路径刷新。

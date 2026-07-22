# 第 11 层第 7 轮检查点：探索卡滞自封起点修复

## 1. 结论

`seed=42` 两层完整电梯专项第三轮确认：机器人在二层看似长期不动并非腿控、定位线程或安全门失效，而是普通探索 `STUCK` 分支把近处 frontier 周围 `0.6 m` 写为永久 `BLACK`，同时覆盖了距离目标仅 `0.374 m` 的机器人当前位置。此后所有 A* 都从不可通行起点开始，最终使返程 `TakeElevator(1->0)` 的精确等待点和已走轨迹粗接近点全部报告不可达。

已删除普通探索卡滞对地图的永久写黑，只保留 `FrontierExplorer.record_stuck_failure()` 的临时目标区域黑名单。相关 454 个 Python 测试通过，合规静态检查通过。由于同一返程问题已经完成两轮 GUI 取证，本轮按约束不继续堆叠仿真；修复后的双向电梯闭环仍需下一轮验证。

## 2. 本轮 GUI 配置

- 证据目录：`/tmp/stage11_elevator_return_drift_round3_20260722`
- `SEED=42`
- `FLOOR_COUNT=2`
- `GUI=true`
- `ELEVATOR_TEST_FLOOR_TIMEOUT=33.0`
- `ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5.0`
- `ELEVATOR_TEST_MAX_RUNTIME=180.0`
- `ELEVATOR_TEST_FULL_CYCLE=1`
- `ELEVATOR_TEST_RECORD_DRIFT=1`
- `ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1`
- 生产节点没有订阅真值；真值仅由独立 bag 进程记录并在运行结束后离线评估。

## 3. 关键时间线

| 任务时间 | 事件 |
|---:|---|
| `3.75 s` 左右 | 主入口连续确认并启动过门任务 |
| `6.35 s` 左右 | 主入口近场重观测成功，门洞宽约 `1.20 m` |
| `21.75 s` 左右 | RGB-D 电梯候选注册成功，中心约 `(1.82, 2.28)`、宽 `0.88 m` |
| `33.16 s` | 一层预算触发 `TakeElevator(0->1)` |
| `73.96 s` | 去程换层完成，进入二层探索 |
| `83~88 s` | 机器人接近 frontier 后无进展，随后触发 `STUCK` |
| `88.81 s` | 旋转净空仅 `0.3179 m < 0.40 m`，安全拒绝原地恢复 |
| `93~109 s` | 无可达前沿扫描同样因净空不足保持零速 |
| `111.06 s` | 全局返航预算触发，创建 `TakeElevator(1->0)` |
| `111.26~113.66 s` | 精确等待点三次不可达，唯一历史粗接近点三次不可达 |
| `113.66 s` | `elevator_trace_approach_exhausted`，fail-closed 写失败结果 |

## 4. 根因证据链

1. 卡滞前路径首航点约为 `(1.35, 0.85)`，机器人约为 `(1.16, 1.19)`，两者距离约 `0.374 m`。
2. 原 `MissionManager._tick_explore()` 在 `STUCK` 时先调用 `record_stuck_failure()`，随后当机器人距 frontier 小于 `1.5 m` 时，又以 frontier 为中心写入半径 `0.6 m` 的 `BLACK`。
3. 机器人距离目标小于写黑半径，因此当前位置所在格被一并永久封锁。`mark_visited_radius()` 按设计不能覆盖 `BLACK`。
4. A* 对不可通行起点只尝试 `mark_free()`；该操作也不能覆盖 `BLACK`，所以直接返回 `UNREACHABLE`。
5. 返程任务先对精确候梯点连续三次规划失败，再选择已走轨迹点 `(0.951, 1.561)`；该候选自身通过了可通行筛选，但规划仍从已写黑的机器人起点开始，因此同样连续三次失败。
6. 本轮没有 `ground_support_map_update` 或 `ground_support_boundary_persisted` 事件，排除新增 `NEGATIVE` 负障碍层误封本次起点。
7. 原代码注释明确要求“只改规划黑名单，不把远处拐角写成永久地图障碍”，实际紧随其后的 `mark_black()` 与注释和设计意图矛盾。

## 5. 修改文件与行为

- `src/danger_search/danger_search/mission_manager.py`
  - 删除普通探索 `STUCK` 分支按 frontier 写永久 `BLACK` 的逻辑。
  - 保留已有临时 frontier 黑名单、旋转净空检查和安全零速行为。
  - 真正负障碍仍由 Livox 支撑证据写 `NEGATIVE`；任务级明确禁区和返航失败段机制不受影响。
- `src/danger_search/test/test_mission_manager_helpers.py`
  - 把旧的“近目标卡滞应写黑”断言改为实测回归：frontier 距机器人 `0.4 m` 时只登记临时失败区域，不调用 `mark_black()`，机器人所在格保持可通行。
- `tools/sim/run_elevator_gui_test.sh`
  - 本阶段此前新增二层最短驻留独立参数、可选隔离漂移录包及生产节点订阅审计；正式配置文件不被测试脚本覆盖。
- `tools/offline/analyze_localization_drift_bag.py`
  - 本阶段此前新增离线定位/真值轨迹对齐分析；不进入生产启动链。

未修改官方 world、生成器、门/电梯服务、Unitree 控制器、传感器模型、既有说明文档或既有报告。

## 6. 测试结果

```bash
source devel/setup.bash
python3 -m pytest \
  src/danger_search/test/test_mission_manager_helpers.py \
  src/danger_search/test/test_navigation_tasks.py \
  src/danger_search/test/test_multi_floor_mapper.py \
  src/danger_search/test/test_support_shadow_online_map.py -q
```

结果：`225 passed`。

```bash
source devel/setup.bash
python3 -m pytest src/danger_search/test -q
```

结果：`454 passed`。

```bash
python3 tools/offline/check_production_boundary.py
git diff --check
```

结果：生产/离线边界审计通过，补丁格式检查通过。

## 7. 定位漂移结论

本轮离线 bag 统计：

- 行程：`15.242 m`
- 最终平移误差：`0.254 m`
- 最大平移误差：`0.258 m`
- RMS 平移误差：`0.155 m`
- 最大 yaw 误差：`0.0030 rad`
- 最后 10 秒平移误差波动：`0.0062 m`

IMU 姿态约束使 yaw 基本无漂移；主要误差仍是平面平移偏置。最后 10 秒机器人静止期间误差几乎不增长，排除“静止积分持续漂移”是本次返程不可达的原因。下一阶段应继续按运动分段分析 RGB-D/Livox GICP 的平移偏置，但不能与本次地图自封修复混在同一验证结论中。

## 8. 合规检查

- 生产输入没有新增话题、文件、TF、服务或回退源。
- GUI 运行中真值仅供独立离线 bag 记录，不进入 MissionManager、定位、建图、规划或控制决策。
- `check_production_boundary.py` 通过。
- 官方哈希仍恰有 4 个历史已知差异：`auto.sh`、`FSM.h`、`FSM.cpp`、用户修改的 `gazebo.xacro`；本轮没有新增官方差异。
- 仿真结束后 Gazebo、ROS master、任务节点和控制器进程均已清理，11311 端口空闲。

## 9. 未验证与下一最小实验

本轮静态修复尚未做 GUI 回归。下一轮只需复用相同 `seed=42`、33 秒楼层预算和完整闭环脚本，重点判定：

1. 二层近 frontier `STUCK` 后，当前机器人格不再被永久写黑；
2. `TakeElevator(1->0)` 至少能规划到候梯点或历史粗接近点；
3. 若进入 `ENTER_CABIN/CROSS`，继续验证第 6 轮记录的门槛/门板近场阻断是否仍存在；
4. 完成 `1->0` 后继续验证主入口退出和最终返航；
5. 同步保留离线漂移 bag，但不得让真值影响生产决策。

在该 GUI 回归通过前，第 11 层不能封口，也不创建提交或标签。


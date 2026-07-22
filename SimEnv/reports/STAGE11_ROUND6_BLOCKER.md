# 第 11 层第 6 轮：在线负障碍建图通过，返程电梯入口阻塞

## 1. 结论

本轮针对二层楼板空洞的根因修复已经实现，并由单元测试和两轮 `seed=42` GUI 仿真验证：低于本层地面的 Livox 回波不再把空洞射线刷成 `FREE`，稳定下降回波和支撑边界会写入独立的 `NEGATIVE` 禁行层；同一位置重新出现连续直接支撑时，动态负障碍可以恢复为 `FREE`。第二轮中机器人在旧版本倾倒位置前安全停止，持久化 4 个负障碍单元，随后绕开空洞继续探索，未再倾倒。

第 11 层仍不能封口。第二轮完整两层闭环在返程 `TakeElevator(1->0)` 的 `ENTER_CABIN/CROSS` 阶段失败：任务层持续发布约 `0.19~0.21 m/s` 前进命令，但定位速度和位姿连续 6 秒无进展，最终按既有有界看门狗进入 `WRITE_RESULT`。该失败发生时支撑安全门为 `CLEAR`，附近没有新增负障碍，属于独立的电梯近场跨越问题，不是本轮负障碍建图造成的回归。

按“确定性失败后停止堆仿真”和“同一阶段最多两轮自修复”的约束，本轮不再启动第三次 GUI，不修改赛方门服务，不放宽障碍停车或门洞确认条件，也不提交代码。

## 2. 根因与设计

### 2.1 原缺陷

原二维地图仅把 `z_rel in [0.15, 1.40]` 的点作为正障碍。二层空洞中的 Livox 射线会落到下一层，回波约为 `z_rel=-2.6 m`：该点不属于障碍，却仍执行自由射线和扇区清扫，导致没有本层楼板的区域被错误写成 `FREE`。规划器只看到可通行二维栅格，直到足端临近边缘，支撑门才事后停车。

### 2.2 本轮最小实现

1. 在既有单层 `FloorGrid` 中新增 `NEGATIVE` 状态，不引入三维地图。`FREE/VISITED` 仍是唯一可通行状态，`UNKNOWN/OCC/BLACK/NEGATIVE` 全部不可通行。
2. `z_rel < -minimum_drop` 的下降回波不进入普通自由射线与扇区清扫，避免把边缘之后刷成自由空间。
3. `SupportShadowEstimator` 对下降回波执行命中次数、连续帧和连通簇门控；稳定证据写入 `NEGATIVE` 并按既有安全半径膨胀。
4. 同格本层支撑回波必须连续出现后才能清除动态 `NEGATIVE`。普通自由射线、开门条带和瞬时单帧回波均不能清除它；`OCC/BLACK` 的更高优先级保持不变。
5. 支撑边界增加 `0.30 m` 前视余量和连续一致性锁存。锁存达到上限时，任务层在主定时器线程持久化边界单元、清路径并触发一次重规划。
6. 位于真实空洞边缘时，定位可能同时失去地平面估计。姿态健康监控改为：当地面高度和法向同时缺失、但 IMU 明确竖直且角速度稳定时保持健康；任何明确倾斜、异常高度或异常法向仍按原规则失败。缺地面由支撑安全门处理，不再被误解释为“已经摔倒”。

## 3. 改动文件与行为

- `src/danger_search/danger_search/mapper.py`
  - 新增 `NEGATIVE` 栅格语义、来源集合、稳定负障碍写入和直接支撑恢复。
  - 截断下降射线；可视扇区不再越过负障碍清图。
- `src/danger_search/danger_search/support_shadow.py`
  - 输出稳定负障碍单元、稳定直接支撑单元和第一条支撑边界截面。
  - 支撑与下降证据均采用连续帧门控，允许电梯轿厢离开后重新形成负障碍。
- `src/danger_search/danger_search/ground_support_gate.py`
  - 新增支撑边界余量、空间一致性和连续确认锁存。
- `src/danger_search/danger_search/mission_manager.py`
  - 将支撑线程结果排队到 10 Hz 主线程更新地图。
  - 稳定边界触发禁行持久化和有界重规划。
  - 规划、返航、探针和栅格副本统一识别 `NEGATIVE`。
- `src/danger_search/danger_search/frontier.py`
  - 前沿净空计算识别 `NEGATIVE`。
- `src/danger_search/danger_search/navigation_tasks.py`
  - 门和电梯主动跨越前的段阻塞检查识别 `NEGATIVE`。
- `src/danger_search/config/danger_search.yaml`
  - 新增边界余量、连续确认和一致性半径参数；负障碍下降阈值复用 `ground_support/minimum_drop`，避免双重配置漂移。
- `src/danger_search_localization/include/danger_search_localization/stance_health_monitor.hpp`
  - 区分“IMU 已证明稳定但地面暂时不可见”和“明确姿态异常”。
- 测试：
  - `test_multi_floor_mapper.py`
  - `test_support_shadow_online_map.py`
  - `test_ground_support_gate.py`
  - `test_mission_manager_helpers.py`
  - `test_stance_health_monitor.cpp`

没有修改赛方 world、楼板、门/电梯服务、场景生成器、Unitree 控制器或正式启动链。本轮也没有改动任何既有说明文档和既有报告。

## 4. 自动测试

### 4.1 Python

```bash
source devel/setup.bash
python3 -m pytest src/danger_search/test -q
```

结果：`454 passed in 1.37s`。

核心新增覆盖包括：

- 下层回波不会产生穿过空洞的自由射线或扇区清扫；
- 连续下降回波形成 `NEGATIVE`，孤立噪声不形成禁区；
- `NEGATIVE` 不被普通自由射线或开门条带清除；
- 连续直接支撑可恢复动态负障碍；
- `OCC/BLACK` 对动态负障碍保持优先级；
- 支撑边界的提前余量、连续锁存和几何跳变重置；
- 地图更新只在 MissionManager 主线程应用；
- IMU 明确稳定而地面同时不可见时不误报跌倒，明确倾斜仍失败。

### 4.2 定位 C++/ROS 测试

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_danger_search_localization
catkin_test_results build/test_results/danger_search_localization --all
```

结果：`88 tests, 0 errors, 0 failures, 0 skipped`。

### 4.3 静态检查

```bash
python3 tools/offline/check_production_boundary.py
git diff --check
sha256sum -c OFFICIAL_FILES.sha256
```

结果：

- 生产/离线边界审计通过；
- `git diff --check` 通过；
- 官方哈希仍恰有 4 个历史预期差异：`auto.sh`、`FSM.h`、`FSM.cpp`、用户修改的 `gazebo.xacro`；本轮没有新增赛方文件差异。

## 5. GUI 仿真

共同配置：

- `SEED=42`
- `FLOOR_COUNT=2`
- `GUI=true`
- `PAUSED=true`
- `AUTO_UNPAUSE=1`
- `ENABLE_GROUND_TRUTH=false`
- `ENABLE_REFEREE_ODOM=false`
- `POINTCLOUD_USE_GROUND_TRUTH_ODOM=false`
- 单层最小探索预算 `33 s`
- 最大任务时间 `300 s`
- 完整闭环模式开启

### 5.1 第一轮

目录：`/tmp/stage11_negative_map_round1_20260722`

- `0->1` 电梯换层成功，任务在 `75.962 s` 回到二层探索。
- 二层空洞边缘检测到 4 个稳定边界单元，连续确认 18 帧，机器人零速停止，没有物理倾倒。
- 随后定位健康监控因地面高度和法向同时缺失而误判 `localization_stance_unhealthy`，在 `96.062 s` fail-closed。
- 该轮促成上述姿态健康语义修复；没有放松明确跌倒条件。

### 5.2 第二轮

目录：`/tmp/stage11_negative_map_round2_20260722`

关键时间线：

| 任务时间 | 事件 |
|---:|---|
| `33.162 s` | 一层预算结束，进入 `CHANGE_FLOOR` |
| `76.764 s` | `TakeElevator(0->1)` 完成，进入二层探索 |
| `87.064 s` | 空洞边缘持久化 4 个 `NEGATIVE` 单元并重规划 |
| `198.262 s` | 二层覆盖约 `47.5%`，开始返航 |
| `198.362 s` | 启动 `TakeElevator(1->0)` |
| `259.462 s` | `ENTER_CABIN/CROSS` 无进展超时，进入 `WRITE_RESULT` |

本轮对当前修复的确认：

- 机器人越过旧版约 `101~108 s` 的倾倒窗口，持续探索到 `198 s` 以上；
- 空洞边缘事件为 `ground_support_boundary_persisted`，`cell_count=4`；
- 没有 `localization_stance_unhealthy`，没有 Unitree 跌倒日志；
- 返回电梯附近出现两次 `negative_cleared_by_support=1`，证明轿厢/楼板直接支撑恢复链在真实仿真中生效；
- 所有测试进程已清理，ROS 11311 端口空闲。

## 6. 新阻塞：返程进入电梯轿厢

失败证据：

- 返程近场电梯几何刷新超时，任务按现有策略保留旧的连续确认几何；
- 电梯门服务从仿真时刻 `251.414 s` 开始动作，赛方控制日志记录 25 秒墙钟动画，最后一次面板更新在约 `255.382 s`；主动跨越在 `258.112 s` 开始，因此“固定再等几秒”不是有证据的修复；
- 跨越起点约 `(0.938, 2.363)`，轿厢目标约 `(2.724, 2.284)`；
- 卡住时位姿约 `(1.034, 2.331, -0.052)`，前进请求约 `0.19~0.21 m/s`，实际线速度为 `0`；
- 最近前向实体回波稳定在门平面附近 `(1.64~1.66, 2.08)`，相对高度约 `0.16~0.22 m`，距离约 `0.66 m`；
- 安全门为 `CLEAR`、`drop_confirmed=false`、无新负障碍写入，说明速度没有被本轮安全逻辑归零；
- 去程同一 `ENTER_CABIN/CROSS` 成功，跨门阶段没有同类前向回波。

赛后只读核对 `competition_scene.world`、`door_config.yaml` 和控制日志，仅用于解释仿真结果，不进入生产链：门口存在 6 cm 高门槛，门板下沿也落在当前障碍高度带内；当前证据尚不足以区分“门板实际残留/几何刷新漂移”与“门槛接触后控制器无法跨越”。下一轮必须采集同一时刻的合法 RGB-D/Livox 局部几何和控制器关节/姿态诊断，不能仅靠离线布局选择固定 seed 坐标或清空障碍。

## 7. 合规性

- 新增生产信息全部派生自公开 `/scan`、`/trunk_imu` 和参赛方自身定位；没有新增话题、服务、文件、TF 或 relay。
- 正式运行显式关闭真值、裁判里程计和真值点云转换；生产边界审计通过。
- `layout_metadata.json`、world 和门内部配置只在运行结束后用于本报告的离线归因，不被生产节点读取，也没有把其坐标、状态或参数写入生产逻辑。
- 没有修改官方 SimEnv 文件；保留用户已有 `gazebo.xacro visualize=false` 变更。
- 失败被明确写为 `floor_change_failed_return_home`，没有伪装为成功或继续无界重试。

## 8. Git 状态与封口判断

工作区保留第 11 层此前未提交改动和本轮改动；按当前检查点要求未执行 `commit/tag/push/reset/checkout`，也未改变暂存状态。

本阶段不可封口：在线负障碍建图子目标完成，但完整两层仍缺 `1->0` 返程电梯、主入口退出和最终返航成功证据。

## 9. 下一最小实验

先做一个只关注 `TakeElevator(1->0)` 进入轿厢的定向 GUI 实验，不重复长时间全楼层探索：

1. 使用生产允许的 RGB-D/Livox 记录开门完成后门平面的实际剩余结构、开口有效宽度和连续帧变化；
2. 同步记录 `/cmd_vel`、自身定位、IMU 姿态和 Unitree 关节状态，区分“生产位姿未动”与“物理机体已动但定位未更新”；
3. 对照去程成功和返程失败的门洞宽度、障碍高度、门槛接触位置与跨越速度；
4. 若证实开口未达到机身安全宽度，修复门洞确认的完整宽度语义；若证实是 6 cm 门槛导致步态失效，在参赛任务层采用有界、低速、正向跨门槛策略；若证实是定位漂移，则修复近场几何刷新/任务局部相对量，而不是写入固定坐标。

在该定向实验得到唯一根因前，不应继续放宽门洞阈值、障碍停车距离或地图清理范围。

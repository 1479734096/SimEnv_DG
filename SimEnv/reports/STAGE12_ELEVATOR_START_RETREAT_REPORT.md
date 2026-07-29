# 第 12 层电梯返程 OCC 起点恢复报告

## 1. 结论

已修复 `TakeElevator(1→0)` 在机器人起点格被墙体膨胀 `OCC` 覆盖时，反复更换电梯厅外目标仍全部 A* 不可达的问题。修复不修改地图、不放宽 A*、不吸附邻近自由格，只允许机器人沿本层实际走过的直线轨迹低速倒退一次，脱离起点膨胀带后重新执行标准 A*。

固定 `seed=42` 的两轮 GUI 完整双层仿真均成功完成电梯视觉注册、`0→1`、二层探索、`1→0`、主入口退出、回起点和结果写出。两轮返程起点均位于远离旧阻塞点的位置，标准 A* 直接 `reached`，因此运行期没有自然触发新增直退分支。结论边界是：分支正确性由定向单测覆盖，两轮 GUI 证明正常路径无回归；尚不能宣称旧贴墙场景的真实直退动作已完成在线闭环。

## 2. 根因链

上一轮失败时机器人约位于 `(0.973, 3.15)`，最近径向实体约 `0.25 m`，与地图障碍膨胀半径相同。普通 A* 按设计只接受 `FREE/VISITED` 起点，起点被 `OCC` 覆盖后会立即返回 `UNREACHABLE`。

原兜底只把目标依次换成精确候梯点和历史粗接近点，起点没有变化，所以六次规划都从同一不可通行起点失败，最终显式退出为 `elevator_trace_approach_exhausted`。这不是目标选择错误，也不是需要放宽 A*；缺口是任务层没有从当前墙体膨胀带安全退出的动作。

## 3. 实现行为

- 只有当前起点格严格等于 `OCC` 时才考虑直退；`BLACK`、`NEGATIVE`、`UNKNOWN` 和越界均拒绝。
- 目标只从当前楼层真实行驶轨迹中选择，距离限制为 `0.5～1.5 m`，且必须位于车体正后方。
- 目标格必须是 `FREE/VISITED`，周围保留现有旋转净空要求。
- 路段只豁免起点附近最多 `0.35 m` 的连续 `OCC` 膨胀带；一旦进入可通行区，后续再次遇到 `OCC` 即拒绝，`BLACK/NEGATIVE/UNKNOWN` 全程拒绝。
- 使用已有统一控制出口 `PathFollower.reverse_straight()`，不原地掉头，不新增 `/cmd_vel` 发布者。
- 每个 `TakeElevator` 最多尝试一次；完成后回到 `GO_TO_ELEVATOR_DOOR` 并重新运行标准 A*。
- 目标缺失、超时、倒车朝向偏差超过 `0.45 rad` 或命令被拒绝时，明确失败为 `elevator_start_retreat_failed`。

## 4. 修改文件

- `src/danger_search/danger_search/mission_manager.py`
  - 新增合法轨迹退让目标选择和起点连续 OCC 路段校验。
- `src/danger_search/danger_search/navigation_tasks.py`
  - 新增一次性 `ELEVATOR_START_RETREAT` 子状态、超时与结构化事件。
- `src/danger_search/test/test_mission_manager_helpers.py`
  - 覆盖连续起点膨胀带成功、`BLACK` 起点拒绝和脱离后 OCC 重入拒绝。
- `src/danger_search/test/test_navigation_tasks.py`
  - 覆盖直退后重走标准 A*，以及朝向发散 fail-closed。

未修改定位、地图写入规则、A*、视觉阈值、Unitree、`auto.sh`、生成器、world 或赛方文档。

## 5. 静态与单元验证

- `python3 -m pytest -q src/danger_search/test/test_navigation_tasks.py src/danger_search/test/test_mission_manager_helpers.py`
  - `214 passed`。
- `python3 -m pytest -q src/danger_search/test`
  - `473 passed`，仅有既有第三方弃用警告。
- `python3 tools/offline/check_production_boundary.py`
  - 通过。
- `git diff --check`
  - 通过。
- `sha256sum -c OFFICIAL_FILES.sha256`
  - 仍恰好 4 项历史差异：`auto.sh`、`FSM.h`、`FSM.cpp`、用户保留的 `gazebo.xacro`；本轮无新增赛方文件差异。

## 6. GUI 仿真验证

两轮共同配置：

- `SEED=42`
- `GUI=true`
- `FLOOR_COUNT=2`
- `ELEVATOR_TEST_FLOOR_TIMEOUT=33.0`
- `ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5.0`
- `ELEVATOR_TEST_MAX_RUNTIME=180.0`
- `ELEVATOR_TEST_FULL_CYCLE=1`
- 真值、裁判里程计、真值点云转换和定位逐帧诊断均关闭。

第一轮证据目录：`/tmp/stage12_elevator_start_retreat_round1_20260723`

- `0→1` 于仿真时间 `78.412 s` 完成。
- `TakeElevator(1→0)` 从 `(1.429, 6.485)` 规划，标准 A* 直接 `reached`。
- `1→0` 于 `150.414 s` 完成。
- 于 `167.514 s` 进入 `FINISH`，`reason_code=normal_result_written`。

第二轮证据目录：`/tmp/stage12_elevator_start_retreat_round2_20260723`

- `0→1` 于仿真时间 `81.212 s` 完成。
- `TakeElevator(1→0)` 从 `(0.849, 6.103)` 规划，标准 A* 直接 `reached`。
- `1→0` 于 `151.716 s` 完成。
- 于 `169.616 s` 进入 `FINISH`，`reason_code=normal_result_written`。

两轮均无 `localization_stance_unhealthy`、机器人倾倒、`elevator_start_retreat_failed` 或禁用输入订阅。测试结束后 ROS master、Gazebo、控制器和 runner 均已清理，端口 `11311` 空闲。

## 7. 合规性

新增决策只消费合法 `/scan` 派生地图、自身定位和运行期真实行驶轨迹。没有读取布局、world、Gazebo model state、裁判里程计或真值；没有通过地图清障、传送、扶正或伪造输入绕过问题。离线/生产边界审计通过，生产节点运行期订阅中不存在 `/ground_truth/*` 或 `/Odometry_gazebo`。

## 8. 残余风险与下一步

- 两轮随机执行轨迹都没有复现旧 `OCC` 起点，真实直退动作仍缺在线证据；按两轮停止规则不继续堆第三轮。
- 若未来自然触发，应核对事件严格为一次 `elevator_start_retreat_started`、一次 `completed`，且随后 `elevator_navigation_plan` 变为 `reached`。
- 若返回 `elevator_start_retreat_failed`，先检查轨迹目标净空和路段状态，不得扩大起点豁免、清除 OCC 或降低安全门槛。
- 当前主线可回到第 12 层定位漂移：继续做旋转不变的跨里程计一致性诊断，不使用固定世界 X/Y 符号，也不实施电梯真值重锚定。

## 9. Git 状态

工作区保留既有大量未提交迁移改动，本轮未执行 commit、tag、push、reset、restore 或 clean。阶段是否统一提交仍等待用户指示。

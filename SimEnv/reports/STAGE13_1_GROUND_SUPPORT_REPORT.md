# 第 13.1 阶段：地面支撑重试预算隔离报告

## 1. 结论

已修复地面支撑边界写入 `NEGATIVE` 后，不同探索前沿错误共享同一重试预算的问题。修复只改变失败归属键，不调整停车包络、安全距离、观察超时、确认帧数或最大重试次数。

生产防坠的纯逻辑闭环和正常两层运行无回归已经验证；真实空带上的 `DROP_CONFIRMED` 尚未在本轮 Gazebo 中自然触发，不能宣称生产防坠端到端证据已经封板。

## 2. 根因与修改

`MissionManager._handle_ground_support_control()` 在稳定边界写入 `NEGATIVE` 时，会先清空 `_last_frontier_target`，随后才生成 `_ground_support_path_retry_key`。因此重试键退化为 `("state", EXPLORE_FLOOR)`，多个不同前沿的独立边界事件会累计同一计数，可能误触发 `ground_support_path_replan_exhausted`。

修复在清理当前前沿之前保存失败目标快照，并用该快照生成 `("frontier", x, y)` 重试键。相同前沿仍受既有上限约束，不同前沿各自计数。

修改文件：

- `src/danger_search/danger_search/mission_manager.py`
- `src/danger_search/test/test_mission_manager_helpers.py`

## 3. 测试

- 地面支撑与 MissionManager 定向测试：`202 passed`。
- 参赛 Python 全包：`489 passed`，仅一条既有第三方弃用警告。
- 新增测试验证稳定边界写入后仍保留原前沿重试键，以及两个不同边界不会共享预算。
- 既有测试继续覆盖：显式 DROP 零速、停稳、局部禁行写入、路径清理、门任务父状态保留、同相位有界失败和跨层证据失效。

## 4. GUI 仿真

证据目录：`/tmp/stage13_1_ground_support_seed6946`

- `SEED=6946`、两层、GUI、正式 formal profile。
- 单层预算 `33 s`、最短探索 `5 s`、总预算 `180 s`。
- 真值、裁判里程计和真值点云转换关闭。
- 主入口、电梯视觉注册和 `0→1` 换层成功。
- 地面支撑门全程保持 `CLEAR`，没有 hold、边界持久化、DROP 或安全门终止；正常门槛和去程电梯无回归。
- 返程在 `TakeElevator(1→0)` 的局部开口确认阶段失败：开门服务完成，但传感器未确认注册位置开口，最终 `floor_change_failed_return_home`。

返程失败与本阶段修改无关：失败前后安全门均为 `CLEAR`，没有 `ground_support_*` 状态转换。该问题转入第 13.2 阶段处理，本轮不追加随机重跑。

## 5. 合规与清理

- 生产/离线边界审计通过。
- `git diff --check` 通过。
- 官方文件仍只有 `auto.sh`、`FSM.h`、`FSM.cpp`、`gazebo.xacro` 四项历史差异，本阶段未修改赛方文件。
- 测试结束后 ROS master、Gazebo、控制器和任务节点均已清理，端口 `11311` 空闲。

## 6. 残余风险

- 仍缺真实空带上的 `DROP_CONFIRMED → 零速 → 停稳 → 路径失效 → 有界重规划/失败` Gazebo 证据。
- 当前稳定边界持久化需要三帧一致观测；本阶段没有放宽该约束。
- 不应通过增加超时、提高重试上限或降低安全余量掩盖后续失败。


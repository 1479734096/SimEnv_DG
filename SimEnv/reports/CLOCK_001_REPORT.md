# CLOCK-001 阶段报告

## 目标

为 `/clock` 冻结、ROS 定时器停止回调时提供独立墙钟诊断，并修复电梯专项
runner 把 roslaunch 包装进程误认为任务节点的生命周期判断。

## 根因证据

- ROS Noetic `rospy.Timer` 使用 ROS 时间；`MissionManager.tick()` 的入口和
  `_elapsed()` 也分别依赖仿真时间。
- 原墙钟线程只在 `ground_support_gate` 存在时启动，且支撑门 `CLEAR` 时已有
  看门狗早退。仿真器停止时可能既没有 tick，也没有支撑门超时。
- roslaunch 为子节点建立独立 session，`MISSION_PID` 只是包装进程，不能代表
  `/danger_search` 和 `/scan_localization` 的存活状态。

## 修改

- 新增 `mission/tick_wall_stall_timeout`，默认 `60.0` 秒；有限的 `<= 0` 值关闭。
- 每次 `MissionManager.tick()` 入口记录 `time.monotonic()`；新增 tick 墙钟检查和
  显式失败处理，首个 tick 前不武装。
- 墙钟线程无条件启动，先检查 tick 活性，再按原逻辑检查支撑门；未改动支撑门
  `state == CLEAR` 早退。
- runner 通过 `rosnode info` 记录任务节点 PID，区分包装进程消失与任务节点退出，
  清理实际节点、`/gazebo` 和 rosmaster，并处理终态日志与退出竞态。

## 验证

- 修复前新增 CLOCK-001、runtime AST 和 runner 契约测试均按预期失败。
- 修复后全量测试：`764 passed, 1 warning`。
- 多轮 seed 8127 Gazebo 观察到 tick elapsed 持续推进，未出现
  `mission_tick_wall_stalled` 或新的包装进程误判；一轮日志确认旧节点同名注册会
  触发外部 shutdown，清理旧孤儿后该现象消失。
- 尚未注入真实 `/clock` 冻结来证明运行期告警触发，当前测试只覆盖判据和线程边界。

## 未完成风险

本阶段不等于二层闭环成功。seed 8127 仍需继续验证 0 层探索、0→1、1 层探索、
1→0、主入口返航和结果写出。

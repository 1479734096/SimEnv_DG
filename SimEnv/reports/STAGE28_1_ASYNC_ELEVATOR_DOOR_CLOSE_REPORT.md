# STAGE28.1：出电梯关门异步化（Fire-and-Forget 极速放行）技术报告

## 一、 背景与根因

在多层自主探索任务中，机器狗乘坐电梯到达目标楼层并完成出厢跨越（`EXIT_CABIN`）到达厅外安全等待点后，原状态机进入 `CLOSE_TARGET_FLOOR_DOOR` 阶段：
1. **同步阻塞调用**：复用通用 `_run_service` 调用 ROS 服务 `/set_door_state` 关闭目标层电梯门，并在服务未返回期间强制执行 `controller.stop()` 保持零速；
2. **无谓时间等待**：Gazebo 仿真中门板插值运动约需 2.5 秒，实测采样阻塞耗时约 **3.3 秒**；
3. **探索时延积累**：机器狗在已完全离开轿厢、进入走廊安全区的情况下原地死等，双楼层闭环累计浪费 **~6 秒**，三楼层闭环累计浪费 **~10 秒**。

---

## 二、 核心修改

### 1. 出厢非阻塞即刻放行（Fire-and-Forget）
在 [`navigation_tasks.py`](../src/danger_search/danger_search/navigation_tasks.py) 的 `TakeElevator.tick()` 中：
- `EXIT_CABIN` 完成时，主线程调用 `client.close_door(target_door_id)`，由后台 Daemon 线程异步发送 `/set_door_state` 服务请求；
- 主状态机不再执行 `controller.stop()`，清理跨越几何后直接标记并返回 `DONE`；
- `MissionManager` 瞬间感知换层完成，无缝切换到 `EXPLORE_FLOOR` 并立即启动 Frontier 前沿评估与 A* 路径规划。

### 2. 安全保障与向下兼容
- **防回头约束**：结合 STAGE28 建立的 `_record_elevator_exit_context` 机制，在 `TakeElevator` 完成时向 `_door_navigation` 注入防回头半平面，规划器绝不向后规划穿入正在关闭的电梯门；
- **配置开关与回退**：新增 `elevator_async_target_door_close_enabled`（默认 `true`），若设为 `false` 则平滑回退至原有同步阻塞关门模式。

---

## 三、 测试与验证

1. **边界隔离审计**：
   ```bash
   python3 tools/offline/check_production_boundary.py
   ```
   结果：参赛 Python/离线边界审计通过。
2. **全量 Python 单元测试**：
   ```bash
   PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest src/danger_search/test/ -q
   ```
   结果：**864 passed** 全绿通过（包含新增异步放行及同步回退覆盖测试）。

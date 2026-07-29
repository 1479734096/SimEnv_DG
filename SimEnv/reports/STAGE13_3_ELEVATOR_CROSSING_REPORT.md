# 第 13.3 层：电梯门平面跨越停车收口报告

## 1. 结论

本阶段关闭了电梯出厢直线跨越时被已打开门板/门框回波误停的问题。修复只在电梯任务已经由合法传感器确认门打开、进入 `CROSS` 直线跨越阶段后生效，并且只忽略门平面附近的薄层回波；门后实体障碍仍进入统一前向停车链。

固定 `seed=6946` 的同场景复测中，上一阶段在二层 `EXIT_CABIN` 停住的路径已成功完成，随后机器人继续二层探索约 50 秒。完整返程未完成：返程候梯对准时定位健康门因三帧地面法向低于阈值而失败，但同期 `base_up_z=1.000`、足印受支撑、控制器没有跌倒日志。该事件属于独立的健康门误报候选，不是本阶段跨门过滤回归。

## 2. 根因

上一轮出厢路径从约 `(2.62, 2.26)` 指向 `(0.82, 2.31)`。机器人到达约 `(1.97, 2.28)` 时，统一前向障碍查询报告约 `(1.80, 2.04)` 的高回波，前向距离 `0.17 m`、横向距离 `0.24 m`。该点恰好落入通用 `0.25 m` 前向走廊和 `0.50 m` 停车距离，但位于已经确认打开的电梯门平面边缘。

旧实现只为 `OpenDoorAndPass` 提供已开门洞过滤，`TakeElevator` 的 `ENTER_CABIN`/`EXIT_CABIN` 仍使用普通走廊障碍语义。恢复旧的出厢倒车不能消除同一门平面回波，重试仍会走相同直线，因此没有启用已证伪的倒车策略。

## 3. 修改

- `TakeElevator.active_opened_door_id()` 只在进厢或出厢的 `CROSS` 阶段返回已经通过传感器确认打开的门 ID。
- `MissionManager._active_opened_door_obstacle_exclusion()` 复用统一门洞过滤入口，为上述电梯阶段构造薄门平面棱柱。
- 电梯视觉宽度表示单扇滑动门板，因此门洞半宽按一扇门板宽度减去栅格边缘余量估计；没有修改视觉注册阈值。
- 过滤深度限制为门平面附近约 `0.35 m`；门后更深的实体障碍仍可触发停车。
- 任务、门确认、公开客户端或注册几何缺失时返回 `None`，保持原来的 fail-closed 行为。

修改文件：

- `src/danger_search/danger_search/mission_manager.py`
- `src/danger_search/danger_search/navigation_tasks.py`
- `src/danger_search/test/test_mission_manager_helpers.py`
- `src/danger_search/test/test_navigation_tasks.py`

## 4. 自动化验证

- `source devel/setup.bash && python3 -m pytest src/danger_search/test/test_navigation_tasks.py src/danger_search/test/test_mission_manager_helpers.py -q`：`221 passed`。
- `source devel/setup.bash && python3 -m pytest src/danger_search/test -q`：`495 passed`，仅一条第三方弃用警告。
- 新测试覆盖：未确认/非跨越阶段不启用过滤、确认后返回正确门 ID、门平面点被过滤、门后实体仍被观察。
- `python3 tools/offline/check_production_boundary.py`：通过。
- `git diff --check`：通过。
- `sha256sum -c OFFICIAL_FILES.sha256`：仍恰有 4 个历史差异，即 `auto.sh`、`FSM.h`、`FSM.cpp`、`gazebo.xacro`；本阶段无新增。

## 5. GUI 验证

配置：

```bash
SEED=6946 GUI=true \
ELEVATOR_TEST_FLOOR_TIMEOUT=33.0 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5.0 \
ELEVATOR_TEST_MAX_RUNTIME=180.0 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_LOG_DIR=/tmp/stage13_3_elevator_plane_seed6946_retry \
tools/sim/run_elevator_gui_test.sh
```

第一轮在控制器模型加载后发生 `junior_ctrl` 段错误，物理恢复前即由 runner 安全终止，归类为环境启动失败，不形成算法结论。按约束同配置只重试一次。

重试轮关键事实：

- 主入口、电梯视觉注册、当前层门确认、进厢和换层均成功；
- 二层 `EXIT_CABIN` 完成，控制器写出 `elevator_transit_completed`，任务随后回到 `EXPLORE_FLOOR`；
- 机器人继续二层探索约 50 秒，证明修复没有把门后区域永久屏蔽；
- 返程 `TakeElevator(1->0)` 已到 `ALIGN_TO_REFRESHED_ELEVATOR_DOOR`，仿真 `125.25 s` 健康门报告 `base_up_z=1.000`、`angular=0.407`、`base_z=0.483`、`ground_nz=0.797`，三帧后输出 `health=false`；
- 同期地面支撑门为 `CLEAR`、`current_footprint_supported=true`，`auto.log` 无 Unitree 跌倒/切换被动态日志。MissionManager 按现有策略显式进入 `WRITE_RESULT → FAILED`，失效安全行为正确，但健康判定需另行复核。

运行目录：`/tmp/stage13_3_elevator_plane_seed6946_retry`。runner 已清理 ROS master、Gazebo、控制器和任务进程。

## 6. 合规性

本阶段没有新增 ROS 话题、服务、文件输入、TF、launch 参数或控制出口。门 ID、开门确认和门几何均来自既有公开服务与合法传感器派生状态；没有使用真值、布局、world、Gazebo model state 或裁判里程计。赛方文件和既有赛方说明文档均未修改。

## 7. 残余风险与下一入口

- 本轮只实测了 `0→1` 出厢，完整 `1→0→主入口→home` 被独立健康门事件阻断。
- 健康门将地面法向短时退化与真实倾倒合并为同一坏帧条件；应先用现有公开 `/scan` 与 `/trunk_imu` 日志复核状态机语义，不得直接放宽姿态阈值或绕过失败。
- 门洞半宽由单扇门板宽度推导，单测已证明门后实体不被过滤；仍需在后续多层闭环中观察不同电梯门宽。
- 下一阶段先建立时间账本和失败分类，再决定效率修改；本阶段不顺带修改速度、规划或防坠参数。

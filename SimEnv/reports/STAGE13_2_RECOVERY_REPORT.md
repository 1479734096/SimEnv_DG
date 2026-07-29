# 第 13.2 层：入口与近墙恢复报告

## 1. 结论

本阶段关闭了三个证据充分的状态机缺陷：主入口初态声明与物理门态不一致时永久等待、返航贴墙 STUCK 时原地旋转、以及电梯近场几何刷新失败后跳过精确候梯点。实现只使用公开门服务、合法点云派生地图和本轮真实轨迹，没有新增外部输入。

固定 `seed=6946` GUI 轮验证了主入口进入、电梯视觉注册、近场刷新失败后的精确候梯点导航、当前层与目标层开门传感器确认以及 `0→1` 运输。完整闭环未通过：机器人在二层 `EXIT_CABIN` 直线跨越阶段卡住，既有策略禁用了已证伪的倒车恢复，随后在门区直接创建返程任务又遇到 A* 起点不可达。该问题与本阶段三项修复不同，保留为下一阶段阻断。

## 2. 根因与修改

### 2.1 主入口幂等开门

`team_scene_info.json` 的 `initial_open` 表示公开期望初态，不能替代运行期物理确认。旧代码在该值为 `true` 时跳过开门服务，若生成或启动时序使门实体仍关闭，入口状态机会永久等待 FREE 走廊。

修复后，每轮初始进入只异步调用一次公开、幂等的 `open_door("main_entrance")`；任务未完成时保持零速，服务失败显式记录失败。没有读取 Gazebo 门态或内部配置。

### 2.2 返航贴墙直退

旧返航 STUCK 在记录失败段后无条件调用旋转恢复，未复用已有的旋转净空门控。修复后：

- 净空充足时保留原恢复行为；
- 净空不足时，仅从当前楼层实际行驶轨迹选择 `0.5~2.0 m` 后方候选；
- 候选必须位于可通行格、周围满足旋转净空、A* 压缩结果为单一直线，并与当前朝向构成后退方向；
- 低速直退到宽处后重新规划；每个卡滞锚点最多尝试一次；
- 无安全候选、朝向发散、命令拒绝或超时均保持零速并进入已有有界失败路径。

### 2.3 电梯候梯点状态跳转

返程失败日志显示，A* 不可达后任务使用粗接近轨迹点；近场几何刷新返回 `IDLE` 后，旧状态机直接调用开门服务，跳过了精确候梯点。机器人停在距候梯点约 `0.60 m`、距注册门中心约 `1.14 m` 的墙边，RGB-D 有效深度为 0，Livox 开口确认也为 0。

修复后，刷新 `IDLE` 只保留既有注册几何，随后仍进入 `GO_TO_REFRESHED_ELEVATOR_DOOR`，到候梯点并重新对门后才调用服务。视觉、RMSE、宽度、连续帧和 Livox 开口阈值均未放宽。

## 3. 修改文件

- `src/danger_search/danger_search/mission_manager.py`
- `src/danger_search/danger_search/navigation_tasks.py`
- `src/danger_search/test/test_mission_manager_helpers.py`
- `src/danger_search/test/test_navigation_tasks.py`

## 4. 测试

- `python3 -m pytest src/danger_search/test/test_navigation_tasks.py src/danger_search/test/test_mission_manager_helpers.py -q`：`218 passed`。
- `python3 -m pytest src/danger_search/test -q`：`492 passed`，仅 1 条第三方弃用警告。
- `python3 tools/offline/check_production_boundary.py`：通过。
- `git diff --check`：通过。
- `sha256sum -c OFFICIAL_FILES.sha256`：仍恰有 4 个历史差异：`auto.sh`、`FSM.h`、`FSM.cpp`、`gazebo.xacro`；本阶段无新增。

## 5. GUI 验证

命令：

```bash
SEED=6946 GUI=true \
ELEVATOR_TEST_FLOOR_TIMEOUT=33.0 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5.0 \
ELEVATOR_TEST_MAX_RUNTIME=180.0 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_LOG_DIR=/tmp/stage13_2_recovery_seed6946 \
tools/sim/run_elevator_gui_test.sh
```

关键证据：

- 主入口在仿真 `8.61 s` 连续确认并进入；
- 电梯在 `27.91 s` 完成运行期视觉注册；
- 近场刷新未获得新几何后，机器人继续从约 `(0.22, 2.30)` 到精确候梯点约 `(0.85, 2.35)`，没有在粗接近点直接开门；
- `elevator_floor_0` 在 `60.01 s` 由 `registered_door_plane` 连续确认打开；
- `elevator_floor_1` 在 `75.81 s` 由同一合法传感器链确认打开；
- `0→1` 运输和楼层确认完成；
- `85.31 s` 在二层 `EXIT_CABIN` 卡住，进入显式失败；随后返程任务因门区 A* 起点不可达而失败。

运行目录：`/tmp/stage13_2_recovery_seed6946`。runner 已清理 Gazebo、ROS master 和任务节点，端口 `11311` 空闲。

## 6. 合规性

本阶段没有修改赛方 world、生成器、Unitree/A1、传感器模型、电梯服务、`auto.sh` 或既有官方说明文档。生产输入仍为公开传感器、公开门/电梯服务和参赛方内部派生状态；没有订阅真值、读取布局或调用 Gazebo model state。

## 7. 未验证与下一入口

- GUI 轮未自然触发普通 `RETURN_HOME` 的贴墙直退，当前只有合成栅格和控制器级回归证据。
- 完整双向电梯闭环被二层出厢卡滞阻断，不能宣称本阶段完整返航通过。
- 下一阶段应先修复 `EXIT_CABIN` 的有界安全恢复：仅在轿厢内、门开传感器确认仍有效、后退段可通行且姿态可控时允许一次直退重试；否则保持现有 fail-closed。通过后再验证 `1→0` 和主入口返航。

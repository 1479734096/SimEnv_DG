# danger_search 正式运行入口

本包负责比赛任务编排、建图规划、感知、建筑交互、路径执行和结果写出。
定位由 `danger_search_localization` 提供；正式模式只消费比赛公开接口和参赛方内部
派生话题。

## 构建与测试

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
python3 -m pytest -q src/danger_search/test
```

## 正式启动

先按团队运行手册启动官方 Gazebo 底座并让机器人稳定站立。底盘进入可消费
`/cmd_vel` 的 RL 模式后，再启动正式系统：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch danger_search danger_search.launch controller_ready_ack:=true
```

该确认参数只能在 `junior_ctrl` 已输出 `Entered RL /cmd_vel mode.` 后传入；
缺省值为 `false`，正式任务会拒绝启动。随后节点等待
`/danger_search/localization/ready=true` 后才创建 `MissionManager`，
因此定位未建立世界系时不会开始任务预算，也不会建立控制发布器。结果路径由
`generated_building/team_scene_info.json` 公布，formal 模式拒绝改写到其他位置。
运行期间定位持续用公开 `/scan` 和 `/trunk_imu` 发布姿态 health；连续异常会撤销
ready、停止输出新位姿，MissionManager 优先停车并写出失败结果。formal 模式不允许
关闭该门控。

短超时诊断入口不依赖 Gazebo 站立就绪：

```bash
roslaunch danger_search danger_search_smoke.launch \
  result_file:=/tmp/danger_search_smoke_detected.json \
  mission_summary_file:=/tmp/danger_search_smoke_summary.json
```

该入口使用 diagnostic profile，只验证启动、零速失效安全、超时和结果写出，
不能作为比赛正式入口。

## 离线评测

离线工具不属于本 catkin 包，也不会随 `danger_search` 安装。正式任务退出后才能按
工作区根目录 `tools/offline/README.md` 调用评分器。提交前可执行：

```bash
python3 tools/offline/check_production_boundary.py
```

## 当前运行边界

第 8.3 层已通过获准修改的 `auto.sh` 关闭默认启动竞争：延迟关节配置和控制主循环首帧
均确认后才恢复物理。第 8 层还观察到：若正式任务已发布非零命令后才把底盘切入 RL
模式，机器人可能在命令交接时跌倒。姿态健康逻辑已通过 ROS 合成和 Gazebo 可控侧倾
闭环；完整控制交接仍需验收。问题证据见 `reports/STAGE8_REPORT.md`、
`reports/STAGE8_1_REPORT.md`、`reports/STAGE8_2_REPORT.md` 和
`reports/STAGE8_3_REPORT.md`。

第 10.1 层已通过 GUI 仿真验证主入口闭环：远场门洞只建立入场轴，公开服务开门后以
两帧近场点云重新估计门洞，再限速直线通过。传感器确认的主入口通过阶段、电梯
`ENTER_CABIN/EXIT_CABIN` 阶段及有界离门上下文只关闭地面支撑 `UNKNOWN`
临时停车，显式 `DROP` 始终保持生效。

## 当前速度与预算规范

- **巡航限速与防坠硬保护**：巡航与探索线速度统一稳健限速为 `0.65 m/s`。即使关闭未知区域保持（`unknown_support_hold_enabled=false`），四足足端一旦出现真实悬空，地面支撑门立即输出零速并在超时后转入 `SUPPORT_PATH_UNAVAILABLE` 重规划，杜绝坠楼。
- **楼层预算**：单层探索超时为 `360.0 s`，单层最短探索时间为 `150.0 s`；测试运行脚本按 1/2/3 层将总预算覆盖为 `540.0 s`、`840.0 s` 与 `1280.0 s`。直接使用 YAML 启动时仍受 `mission/max_runtime` 的专项上限约束。
- **启动稳定等待**：启动脚本在切 stand 与切 RL 之间加入 2.0s 物理阻尼等待，降低四足接地冲击对传感器和控制器的影响。

## 当前电梯开门时序

开始换层时允许以 `call_elevator(..., open_doors=false)` 异步预呼梯，但不得在远距离阶段
调用 `open_door`。机器狗须先利用关闭的深色门板完成电梯近场几何刷新、回到刷新后的候梯点
并完成对准，随后才在 `OPEN_CURRENT_FLOOR_DOOR` 显式开源层门。目标层同样以
`open_doors=false` 升梯，在轿厢内调头完成后由 `OPEN_TARGET_FLOOR_DOOR` 显式开门；两侧均
仍须取得局部传感器开口确认后才能清理门区或跨越。

## 当前危险源与前沿策略

- 红球检测流程先由 RGB-D 完成颜色分割、深度反投影和固定半径 `R=0.15m` 几何拟合；若当前帧有时间相近的 Livox 配准点云，则在进入 `DetectionTracker` 之前执行视锥内深度修正。该修正作用于 RGB-D 候选，不等到多帧 `CONFIRMED` 状态；随后仍须经过多帧质量融合和 NMS 确认，正式结果只输出确认
  轨迹；为防止机器人在确认前压过近场红球，两个跨越至少 `0.08s` 的高质量帧会临时写入
  语义 `OCC`，若 `0.75s` 没有新帧即撤销。该覆盖按 `danger:<track_id>` 独立管理，半径为
  目标半径加现有地图膨胀半径，不能被自由射线、开门区域清理或定位 epoch 切换擦除。
- 稀疏地标纠偏回执会同步平移全部危险源历史观测与语义 `OCC`。若确认危险源安全圆已
  覆盖机器人当前格，任务立即停车、写出已有结果并显式失败，不会尝试穿越该区域。
- 前沿最终评分以真实 A* 路径首段和累计拐角计算转向代价；正式
  `yaw_cost_weight=0.40`、`path_turn_cost_weight=0.70`，并将最高角速度限制为
  `0.90rad/s`。距离超过 `3.5m` 且预期收益低于 `4` 格的前沿在 A* 前后均被跳过，不能
  重新进入候选。

运行日志中的 `danger_obstacle_sync`、`danger_obstacle_reprojected`、
`danger_source_overlap` 与 `frontier_selection` 候选分项用于复核上述策略；它们均不读取
真值数据。

## 当前验证优先级

- 下一轮运行优先检查地面支撑与运动安全，重点确认真实悬空、支撑边界和 `SUPPORT_PATH_UNAVAILABLE`
  的停车及重规划行为。
- 前沿探索与路径转弯、电梯门时序、时间预算与配置当前暂不作为优先修改项；只有出现明确的
  可复现问题或回归证据时才调整。
- Livox 候选级深度修正仍需结合真实 RGB-D/Livox 对齐数据复核，单元测试通过不等于已完成
  真实场景验证。

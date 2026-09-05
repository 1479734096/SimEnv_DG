# danger_search 正式运行入口

本包负责比赛任务编排、建图规划、感知、建筑交互、路径执行和结果写出。
定位由 `danger_search_localization` 提供；正式模式只消费比赛公开接口和参赛方内部
派生话题。

## 修改流程

本项目后续不安排自动测试或回归测试。修改前先理清调用链、状态转换、输入输出和失败降级，
再直接实施最小范围修改；测试源码和历史命令仅保留作查阅资料，不作为当前工作流程。必要的
编译、语法或生产边界检查只能作为静态确认，不得扩展成自动回归测试。

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

当前入口定位保护还包括固定外墙几何的有界平面 SE(2) 纠偏：开局和开门后过门前分别最多尝试一次，
仅使用当前 `/scan` 注册的多帧 Livox 点云，固定合同不等于读取场景 metadata。
至少三帧一致、墙体两侧支持、XY/yaw 残差和 yaw 改善量通过后才向合法 `PoseProvider` 请求纠偏；任何陈旧、
质量不足、请求或回执超时都会在 3 秒内回退原流程。成功纠偏会清路径并同步危险源语义轨迹，
下一轮必须同时检查 `residual_p70`、仅平移基线残差、`residual_improvement`、`yaw_spread`、
左右墙支持点和 `vector.z` 回执，并用带时间戳的 Livox、RGB 和位姿证据确认点云时空对应；还要比较
纠偏前后的门前姿态、足端支撑、路径首段和门区误障碍。质量门拒绝时应记录明确原因并有界回退，不能
为了产生 `applied` 事件而降低阈值。

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

- 红球检测流程先由 RGB-D 完成颜色分割、深度反投影和固定半径 `R=0.15m` 几何拟合；当前实现使用最新缓存 Livox 配准点云，在进入 `DetectionTracker` 之前执行视锥内深度修正，尚未按 RGB 时间戳匹配最近云帧。该修正作用于 RGB-D 候选，不等到多帧 `CONFIRMED` 状态；随后仍须经过多帧质量融合和 NMS 确认，正式结果只输出确认
  轨迹；为防止机器人在确认前压过近场红球，两个跨越至少 `0.08s` 的高质量帧会临时写入
  语义 `OCC`，若 `0.75s` 没有新帧即撤销。该覆盖按 `danger:<track_id>` 独立管理，半径为
  目标半径加现有地图膨胀半径，不能被自由射线、开门区域清理或定位 epoch 切换擦除。
- 稀疏地标纠偏回执会同步平移全部危险源历史观测与语义 `OCC`。若确认危险源安全圆已
  覆盖机器人当前格，任务立即停车、写出已有结果并显式失败，不会尝试穿越该区域。
- 已确认、未过期且属于当前楼层的危险源候选，在普通前沿路径执行期间也可以抢占该路径，
  优先进入 `danger_approach_verification`；前沿收益、预取路径和旧到达状态会同步清除。
  接近完成后必须原地对准并取得核验阶段的新鲜多帧证据，接近失败不会把候选永久标记为已核验。
- 前沿最终评分以真实 A* 路径首段和累计拐角计算转向代价；正式
  `yaw_cost_weight=0.40`、`path_turn_cost_weight=0.70`，并将最高角速度限制为
  `0.90rad/s`。距离超过 `3.5m` 且预期收益低于 `4` 格的前沿在 A* 前后均被跳过，不能
  重新进入候选。

运行日志中的 `danger_obstacle_sync`、`danger_obstacle_reprojected`、
`danger_source_overlap` 与 `frontier_selection` 候选分项用于复核上述策略；它们均不读取
真值数据。

## 当前验证优先级

- 下一轮运行优先检查地面支撑与运动安全：普通导航重点确认真实悬空、支撑边界和
  `SUPPORT_PATH_UNAVAILABLE` 的停车及重规划；电梯 `ENTER_CABIN/EXIT_CABIN` 的 `CROSS` 阶段应跳过
  已知无空洞门槛的支撑边界判定，跨越完成后恢复检测，同时确认显式 DROP 和速度异常仍能停车。
- 危险源核验优先于普通前沿、换层和软返航决策；已提交的过门/电梯/返航任务以及安全闭锁
  不得被抢占。换层和返航的原有动态预算不扣除危险源接近或核验耗时。
- Livox 候选级深度修正仍需结合真实 RGB-D/Livox 对齐日志分析；代码逻辑成立不等于真实
  场景行为已经确认。

## 接手修复与剩余限制（2026-09-05）

已修复核验期间OCC变格无条件清路径和预算暂缓阻断terminal_pending消费。
核验按当前剩余折线检查真实冲突，保留无冲突路径；预算暂缓仍处理DONE/STUCK。
失败只取消核验自己拥有的控制，不清除接管者路径或恢复。

每轨迹最多3次尝试，规划失败也计次；5秒冷却后重试，耗尽后保留Confirmed输出资格。
单次接近、对准、核验共30秒上限；驻留证据最多等待5秒，至少2个核验开始后采集的独立RGB
时间戳，跨度至少0.08秒。定位ready等待另有120秒墙钟上限，超时不创建任务。

这些局部上限不能保证完整返航预算。当前max_runtime对已启动返航豁免；CROSS豁免的阶段
同步窗口、新鲜度、入口纠偏回执和固定层高依据仍有未闭合项。入口的观测与回执分别计3秒，
不能把此前“3秒”描述理解为整个纠偏阶段的严格上限。
本次仅语法、差异及历史日志/bag只读分析，未构建或运行；详细文件清单和四轮撞门对照见
[接手审查报告](../../reports/DANGER_PRIORITY_STATIC_REVIEW_20260905.md)。

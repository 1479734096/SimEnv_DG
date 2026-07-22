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
roslaunch danger_search danger_search.launch
```

正式节点等待 `/danger_search/localization/ready=true` 后才创建 `MissionManager`，
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
两帧近场点云重新估计门洞，再限速直线通过。只有服务成功且传感器确认的门槛及其有界
离门阶段会关闭地面支撑 `UNKNOWN` 临时停车，显式 `DROP` 始终保持生效。证据见
`reports/STAGE10_1_REPORT.md`。

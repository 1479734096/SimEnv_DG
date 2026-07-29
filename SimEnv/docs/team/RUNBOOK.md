# 开发与验证手册

## 构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
```

## 官方文件保护

```bash
sha256sum -c OFFICIAL_FILES.sha256
git diff --check
```

## 第 1 层 Python 契约测试

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/danger_search/test/test_interface_policy.py \
  src/danger_search/test/test_result_writer.py \
  src/danger_search/test/test_time_utils.py
```

## 第 2 层定位测试

```bash
source /opt/ros/noetic/setup.bash
catkin_make run_tests_danger_search run_tests_danger_search_localization -j2
catkin_test_results build/test_results
```

单独启动定位节点时必须显式选择正式后端：

```bash
source devel/setup.bash
roslaunch danger_search_localization scan_localization.launch \
  localization_backend:=adaptive_shared_map
```

`livox_imu_local_map` 仅用于固定后端诊断，不是正式默认：

```bash
source devel/setup.bash
roslaunch danger_search_localization scan_localization.launch \
  localization_backend:=livox_imu_local_map
```

该候选只用 `/scan`、`/trunk_imu` 和自身 `/cmd_vel` 做定位；RGB-D 仍由任务节点用于
危险源、门和电梯感知。Stage15 固定 seed 闭环未通过，禁止在正式运行中选择该候选，
也禁止根据任务阶段或实时输入在它与正式 `adaptive_shared_map` 之间自动切换。

正式启动默认使用 `adaptive_shared_map`；也可用下面命令独立诊断：

```bash
source devel/setup.bash
roslaunch danger_search_localization scan_localization.launch \
  localization_backend:=adaptive_shared_map
```

该后端使用 `/trunk_imu` 定姿、Livox 维护唯一局部地图，并让 RGB-D 与 Livox 针对同一
地图逐帧竞争平移候选。它不是任务阶段切源：启动后后端保持不变，PoseProvider 仍只有
一条正式位姿。两路都无效时会停止发布新 odom/cloud，调用方必须保留既有陈旧定位和
3 秒不可用门，不能用旧位姿伪造新时间戳。Stage17 当前 `8/20` 个有效 seed 全部完成
闭环且没有定位诱发失败，但矩阵尚未完成，seed `1612` 的末端误差也超过冻结硬门。
正式默认是按用户明确决策直接切换，不代表矩阵已经通过；禁止现场修改
`shared_map_*` 阈值。Stage17 工程验收状态为通过，剩余矩阵属于后续风险量化。

需要回退旧后端时必须显式指定，避免把诊断行为误当成正式默认：

```bash
roslaunch danger_search danger_search.launch \
  controller_ready_ack:=true \
  localization_backend:=rgbd_livox
```

该确认只能在 `junior_ctrl` 已输出 `Entered RL /cmd_vel mode.` 后传入。

节点启动后先锁存发布 `ready=false`。机器人站立稳定、连续楼面证据满足门槛后才发布 `ready=true`、里程计和配准点云；调用方不得用固定等待替代该就绪信号。

持续姿态健康的确定性回归不依赖 Gazebo：测试先用合成 `/scan` 楼面和竖直
`/trunk_imu` 建立 health/ready，再发布倾倒 IMU，要求两者均撤销：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_danger_search_localization -j2
```

该用例用于日常确定性回归。第 8.2 层另以 Gazebo 可控侧倾验证了正式定位和 MissionManager
的失效安全；证据见 `reports/STAGE8_2_REPORT.md`。正式模式强制启用 MissionManager
health 门控；诊断 smoke 才允许关闭。

## 第 3 层建图与规划测试

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/danger_search/test/test_multi_floor_mapper.py \
  src/danger_search/test/test_planner.py \
  src/danger_search/test/test_frontier_explorer.py
```

阶段性能测量使用默认 `280×480` 栅格。普通 A* 必须保留迭代预算；`BUDGET_EXHAUSTED` 不等于不可达，不得据此写 BLACK 或前沿黑名单。无预算长墙绕行约 170 ms；第 7 层已要求返航穷举只在地图和历史轨迹快照的后台线程运行，主 tick 在结果就绪前持续零速。

## 第 4 层路径执行与安全门测试

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/danger_search/test/test_local_controller.py \
  src/danger_search/test/test_ground_support_gate.py
```

ROS 发布闭环使用独立 master：先给安全门一帧新鲜、充足支撑，确认 `/cmd_vel` 可出现非零跟踪命令；再给出支撑边界不足评估，确认当前 tick 和后续保活均只发布零速。测试结束必须关闭独立 `roscore`，不得连接正在运行的比赛 master。

定位默认速度死区是线速度 `0.05 m/s`、角速度 `0.03 rad/s`。`GroundSupportGateConfig` 必须显式保持停稳阈值不小于这两个值；若后续调整定位死区，必须同步修改安全门配置并重跑动态停车验证。

## 第 5 层 RGB-D 感知测试

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/danger_search/test/test_detection_tracker.py \
  src/danger_search/test/test_red_sphere_detector.py \
  src/danger_search/test/test_rgbd_door_detector.py
```

测试覆盖合成红球、红色平面干扰、暗色门板、门板消失、双侧滑门开口、稀疏远端深度、同帧多候选和非法刚体变换。`camera_to_world` 必须使用右手刚体变换；正式光学坐标转换是 world `Y=-optical X`，不得使用镜像矩阵。

`floor_local` 模式只允许在上层构造 `camera_to_world` 时增加一次 `floor_index * floor_height`；检测器直接应用传入矩阵，不再自行修正 z。第 6/7 层接线后必须以两层场景验证最终结果坐标。

## 第 6 层建筑交互与导航任务测试

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/danger_search/test/test_building_control_client.py \
  src/danger_search/test/test_local_semantic_detector.py \
  src/danger_search/test/test_navigation_tasks.py
```

测试使用 fake manager 和注入式服务函数验证异步协议，不连接真实比赛服务。重点断言：未知 ID/楼层拒绝、授权请求字段保持、门洞/轿厢必须有点云证据、A* 不可达与预算耗尽均不生成直线路径，以及过门/进出厢的对齐和阻塞失败边界。

`use_truth_proximity_detector` 只保留历史诊断兼容，正式 profile 必须为 false。真实 `/set_door_state`、`/call_elevator` 与视觉开口闭环留到第 8/10 层正式 runner 验收。

第 10.1 层主入口 GUI 回归必须确认：机器人原地请求开门，近场连续两帧确认净空后再
低速直线通过；首个前沿位于门后侧，离门期间不得出现 `STALE_DATA_HOLD` 或
`ground_support_path_unavailable`。完整证据见 `reports/STAGE10_1_REPORT.md`。

## 第 7 层任务编排测试

```bash
source devel/setup.bash
python3 -m pytest -q \
  src/danger_search/test/test_time_utils.py \
  src/danger_search/test/test_coordinate_transform.py \
  src/danger_search/test/test_progress_monitor.py \
  src/danger_search/test/test_mission_manager_helpers.py
```

返航规划第一次 tick 只创建后台任务并调用 `controller.stop(reason="return_home_plan_pending")`；测试必须等待该任务完成后再推进下一 tick，不能恢复成同步 A*。楼层对齐超时是终止故障，必须进入 `WRITE_RESULT`。电梯摘要保持三态：未完成为 `null`、完成为 `true`、失败为 `false`。

## 官方底座冒烟

```bash
GUI=false SEED=42 FLOOR_COUNT=1 \
PAUSED=true AUTO_UNPAUSE=1 \
./auto.sh
```

键盘 `2` 站立，稳定后使用 `6` 进入 RL `/cmd_vel` 模式。不要使用 `PAUSED=false` 全传感器组合。

第 8.3 层已在获准修改的 `auto.sh` 中关闭启动竞争：等待 `spawn_model` 延迟配置完成，
最后写入 calf 安全角，再等待 `junior_ctrl` 命令首帧后恢复物理。不要再使用固定延时或从
参赛节点调用 Gazebo 配置服务。`START_CONTROLLER=0` 会保持物理暂停；无键盘运行应使用
`CONTROLLER_FOREGROUND=0`。

## Gazebo 证据记录

涉及运行时行为的阶段至少记录：Git 提交、seed、楼层数、关键环境变量、ROS/Gazebo 隔离端口、仿真与墙钟时长、传感器频率、状态转换、结果摘要和退出后的残留进程。一次运行足以证明确定性边界时不扩大矩阵；随机性使结论不清时再做固定 seed 复测。

Gazebo 日志、bag、生成场景和真值不提交到 Git，只把与结论直接相关的短摘要写入对应 `reports/STAGE<N>_REPORT.md`。

## 第 8 层正式入口与结果闭包

机器人稳定站立后，应先让底盘进入可消费 `/cmd_vel` 的 RL 模式，再启动正式节点：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch danger_search danger_search.launch controller_ready_ack:=true
```

该参数只能在 `junior_ctrl` 已输出 `Entered RL /cmd_vel mode.` 后传入；缺省时
正式入口拒绝创建任务。不得为 formal 模式传入任意结果路径。正式结果文件只能使用
`generated_building/team_scene_info.json` 的 `allowed_interfaces.result_file`；路径不一致时
节点会在建立任何任务控制前显式失败。节点还会等待内部定位 ready，等待期间不创建
`MissionManager`，也不计入任务预算。

不依赖 Gazebo 的短超时闭包检查：

```bash
source devel/setup.bash
roslaunch danger_search danger_search_smoke.launch \
  result_file:=/tmp/danger_search_smoke_detected.json \
  mission_summary_file:=/tmp/danger_search_smoke_summary.json
```

禁止在正式任务已经发布非零命令后再按 `6` 切入 RL 模式；标准流程必须先以零命令切入
RL、等待稳定，再启动任务。该流程已在第 8.3 层通过 GUI 移动闭环。跌倒后的失效安全
已完成单元测试、ROS 合成和 Gazebo 可控侧倾闭环。证据见 `reports/STAGE8_REPORT.md`、
`reports/STAGE8_1_REPORT.md`、`reports/STAGE8_2_REPORT.md` 和
`reports/STAGE8_3_REPORT.md`。

## 第 9 层离线工具

提交或正式验收前执行生产闭包审计：

```bash
python3 tools/offline/check_production_boundary.py
```

只有正式任务和相关 ROS 进程退出后，才可按 `tools/offline/README.md` 调用环境自带的
评分器读取 `danger_truth.json`。评分输出不得被同一轮 MissionManager、定位、感知、
规划或控制读取。

本机若执行 catkin install 时出现 `--install-layout` 不识别，可只对该命令设置：

```bash
SETUPTOOLS_USE_DISTUTILS=stdlib catkin_make install
```

该变量只修正 Python 安装后端选择，不改变生产运行依赖。

# danger_search

`danger_search` 是比赛仿真环境中的自主搜索算法包，面向 `ROS1 Noetic + Gazebo Classic + Unitree A1`。

## 目标

- 使用公开 ROS 接口完成自主探索、危险源红球识别、返航和结果输出。
- 输出 `results/detected_danger.json`，格式遵循 `docs/evaluation.md`。
- 运行时不读取 `results/danger_truth.json`，不修改随机场景生成逻辑。

## 主要节点

```text
danger_search_node
```

该节点内部由 `rospy.Timer` 推进状态机，订阅里程计、点云和 RGB-D 图像，发布 `/cmd_vel`。
节点同时发布 `/danger_search/status`，用于查看当前状态、楼层、已确认目标数量和是否正在跟踪路径。

## 启动方式

在比赛环境启动、`junior_ctrl` 已进入 RL 模式后运行：

```bash
source ./devel/setup.bash
roslaunch danger_search danger_search.launch
```

如果只调试单层闭环，保持默认 `max_floor=1`。需要启用多楼层电梯流程时：

```bash
roslaunch danger_search danger_search.launch max_floor:=3
```

短超时自检可使用：

```bash
roslaunch danger_search danger_search_smoke.launch
```

自检会把结果写到 `/tmp/danger_search_smoke_detected.json`，用于确认节点能启动、超时退出并写出合法结果文件。

## 关键约束

- 检测结果使用多帧确认后才输出，优先降低虚警率。
- 楼层探索至少持续 `mission/min_floor_explore_time`，避免启动后已知区域过小却被误判为覆盖完成。
- 楼层结束还要求最大离家距离达到 `mission/min_floor_travel_distance`，避免机器人只在起点附近抖动就结束任务。
- 无前沿且无远处 FREE 目标时，会从机器人当前位置按 `mission/probe_distance` 做短距离探测前进，再重新建图；默认 `mission/probe_yaw=1.5708`，沿比赛入口到房间的全局 +Y 方向探测，避免起立后的机体 yaw 抖动把探测方向带反。探测段如果穿过已知障碍或黑名单栅格，会跳过该目标。
- 路径跟踪默认允许有限转向和侧移：`controller/min_linear_fraction=0.15`、`controller/max_angular=0.8`、`controller/max_lateral=0.25`，目标偏角较大时会降低前进速度，避免目标在侧后方时仍满速直顶墙体。
- 卡住恢复默认等待 `frontier/no_progress_timeout=12.0` 秒，避免 A1 刚起步就被频繁判定为卡住并后退。
- 默认使用 `/livox/Pointcloud2` 做 2.5D 栅格建图。
- 点云命中障碍会按 `map/obstacle_inflation_radius` 做栅格膨胀，给 A1 留出安全余量。
- 建图默认启用可视扇区自由空间清空：`map/free_clear_*` 会按角度桶聚合可见方向，只在该方向点数足够时把机器人到最近障碍前的连续区域标成 FREE；遇到已知 `OCC/BLACK` 会停止，避免把静态墙体后方擦成可通行。
- 前沿目标默认先找更安全的内部 free cell，而不是直接贴着 frontier 边缘落点；`frontier/frontier_obstacle_clearance`、`frontier/frontier_unknown_standoff` 和 `frontier/frontier_min_information_gain` 分别控制离障碍余量、离 unknown 边缘退让和最小信息增益。
- 默认使用 `/real_sense/rgb/image_raw` 和 `/real_sense/depth/image_raw` 做红球识别。
- Mapper 只维护 free / occupied / unknown 占据栅格，不编码门、电梯或房间入口语义；普通点云射线不会擦除已知静态障碍。
- 动态门只在局部传感器检测到近距离可见门候选后才匹配服务 ID 并执行 `OpenDoorAndPass`；开门成功后按 `mission/door_clear_radius` 清理门附近动态障碍残留，并按 `mission/door_pass_offset`、`mission/door_commit_distance` 继续深入门后，再在 `mission/door_transition_duration` 和 `mission/door_transition_hold_distance` 约束下短时抑制回头选回门前侧目标，不会把 `layout_metadata.json` 中的门坐标作为探索目标。
- 房间入口不视为动态门，不调用 `/set_door_state`；普通探索依靠 Frontier 自然进入。
- 电梯门不走普通开门逻辑，只在 `CHANGE_FLOOR` 状态下由 `TakeElevator` 状态机处理。
- `local_semantic/use_truth_proximity_detector` 仅用于调试，默认关闭。
- 输出坐标使用 Gazebo `world/odom` 数值坐标。
- RGB-D 反投影默认按 OpenCV 光学坐标计算，再转换到 ROS link 坐标后使用 TF。若实际相机 TF 已经是光学坐标，可把 `detection/depth_frame_is_optical` 设为 `false`。

## 批量评测与调参

单个 seed 运行并归档：

```bash
python3 src/danger_search/tools/run_seed.py --seed 42 --timeout 10000 --max-floor 3
```

批量 seed 评测：

```bash
python3 src/danger_search/scripts/batch_evaluate.py --seeds 10 42 100 --timeout 10000 --max-floor 3
```

网格搜索参数：

```bash
python3 src/danger_search/tools/tune_params.py --seeds 10 42 100 --timeout 10000 --max-floor 3
```

每次运行会创建独立目录：

```text
results/runs/<时间戳>_seed_<seed>/
├── detected_danger.json
├── evaluation.json
├── params.yaml
├── mission_summary.json
├── mission_runtime_summary.json
├── detection_debug.json
├── planner_debug.json
└── logs/
```

如果外层 `run_seed.py` 达到 `--timeout` 后中断算法，归档中的 `mission_summary.json` 会写入 `external_timeout=true`、`completed=false`，并用 `result_written` 单独表示是否已经落盘 `detected_danger.json`。

汇总和报告文件：

```text
results/summary.csv
results/best_params.yaml
results/optimization_report.md
```

参数覆盖文件支持扁平 key，例如：

```yaml
tracker/nms_radius: 0.95
tracker/min_hits: 4
detection/max_rmse: 0.035
controller/max_linear: 0.35
mission/door_clear_radius: 0.9
mission/door_transition_duration: 2.5
mission/door_transition_hold_distance: 2.0
map/obstacle_inflation_radius: 0.35
map/free_clear_max_range: 6.0
map/free_clear_angle_step: 0.087
frontier/frontier_obstacle_clearance: 0.3
frontier/frontier_unknown_standoff: 0.25
```

也兼容计划里的旧 key，例如 `detection.confirm_hits`、`mission.total_timeout`、`controller.max_linear_vel`，工具会映射到当前配置结构。

如果出现穿墙、漏房间或门后抑制过强，优先小幅回调以下参数，而不是继续叠加新逻辑：

- `map/free_clear_max_range`
- `map/free_clear_obstacle_margin`
- `map/free_clear_angle_step` 与 `map/free_clear_min_points_per_bin`
- `mission/door_transition_duration`
- `mission/door_transition_hold_distance`
- `frontier/door_front_half_plane_penalty`

注意：`danger_search_node` 运行期间不会读取 `danger_truth.json`。`run_seed.py` 只在算法退出后调用 `evaluate_danger.py` 做离线评测，此阶段才读取真值并写入 `evaluation.json`。

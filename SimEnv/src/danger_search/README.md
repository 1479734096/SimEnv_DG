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

上述命令沿用默认单层时限。三层完整闭环回归应通过 `run_seed.py` 合并专用时间预算覆盖：

```bash
python3 src/danger_search/tools/run_seed.py --seed 42 \
  --params config/three_floor_time_budget_test.yaml \
  --gui --timeout 6000 --max-floor 3
```

该配置只调整总任务时间、单层时限和最小探索时间，已用于验证 0/1/2 层探索、换层、返回 0 层和最终返航。它是工程回归配置，不代表危险源识别率等官方比赛就绪标准。

短超时自检可使用：

```bash
roslaunch danger_search danger_search_smoke.launch
```

自检会把结果写到 `/tmp/danger_search_smoke_detected.json`，用于确认节点能启动、超时退出并写出合法结果文件。

## 关键约束

- 检测结果使用多帧确认后才输出，优先降低虚警率。
- 楼层结束需要同时满足最小探索时间、最大离家距离、最小已知面积、覆盖率和连续无可达前沿确认次数；不会再因为 `len(frontiers) <= 1` 直接结束，避免启动后小已知区域 coverage 虚高导致没搜走廊/房间就返航。
- 楼层探索至少持续 `mission/min_floor_explore_time`，最大离家距离至少达到 `mission/min_floor_travel_distance`，已知面积至少达到 `mission/min_known_area`；`mission/no_reachable_frontier_confirmations` 控制无可达前沿需连续确认的次数，`mission/min_corridor_progress_distance` 可选限制走廊主方向已知范围。
- 无前沿且无远处 FREE 目标时，会从机器人当前位置按 `mission/probe_distance` 做短距离探测前进，再重新建图；默认 `mission/probe_yaw=1.5708`，沿比赛入口到房间的全局 +Y 方向探测，避免起立后的机体 yaw 抖动把探测方向带反。探测段如果穿过已知障碍或黑名单栅格，会跳过该目标。
- Frontier 选择带短期惯性：当前区域仍可达时，新目标分数需要高出 `frontier/switch_score_margin` 才会切换；日志会记录每次选择的分数、距离、门侧、fallback 和切换原因。
- 过门后除了短时间抑制回头，还会在门后推进距离和门后侧已知面积达到 `mission/post_door_min_distance`、`mission/post_door_min_known_area` 前持续优先门后侧目标，减少刚进门又回头；临时和永久禁区都只在门口有限半径内完全跳过，远处仅按距离衰减降分，避免用无限半平面把整片可达区域永久屏蔽。
- 路径跟踪默认允许有限转向和侧移：`controller/min_linear_fraction=0.35`、`controller/max_angular=1.2`、`controller/max_lateral=0.25`，目标偏角较大时按余弦平方衰减前进速度，避免旋转中满速撞墙；转角处使用 `controller/corner_tolerance=0.10` 限制提前切换到下一段，防止在 U 型/平行走廊中切角或跳到未来折返段。
- 卡住恢复默认等待 `frontier/no_progress_timeout=8.0` 秒后原地停止 `controller/recover_duration=0.5` 秒，不盲目后退，避免贴墙时推压导致倒地。
- 默认使用 `/livox/Pointcloud2` 做 2.5D 栅格建图。
- 诊断话题 `/danger_search/path_raw`、`/danger_search/path_followed` 和 `/danger_search/local_target` 分别发布原始 A* 路径、控制器实际跟踪路径和局部前视目标；`path_followed` 当前只做共线点压缩，不应误解为几何平滑器输出。
- 点云命中障碍会按 `map/obstacle_inflation_radius` 做栅格膨胀，给 A1 留出安全余量。
- 建图默认启用可视扇区自由空间清空：`map/free_clear_*` 会按角度桶聚合可见方向，只在该方向点数足够时把机器人到最近障碍前的连续区域标成 FREE；遇到已知 `OCC/BLACK` 会停止，避免把静态墙体后方擦成可通行；无障碍点的方向只清到 1.5m 保守距离，不按 `free_clear_max_range` 全距离清空，防止点云稀疏时穿墙。
- 前沿目标默认先找更安全的内部 free cell，而不是直接贴着 frontier 边缘落点；`frontier/frontier_obstacle_clearance`、`frontier/frontier_unknown_standoff` 和 `frontier/frontier_min_information_gain` 分别控制离障碍余量、离 unknown 边缘退让和最小信息增益。
- 默认使用 `/real_sense/rgb/image_raw` 和 `/real_sense/depth/image_raw` 做红球识别。
- Mapper 只维护 free / occupied / unknown 占据栅格，不编码门、电梯或房间入口语义；普通点云射线不会擦除已知静态障碍。
- 动态门只在局部传感器检测到近距离可见门候选后才匹配服务 ID 并执行 `OpenDoorAndPass`；开门成功后按门 yaw、metadata 门宽和 `mission/door_clear_radius` 条带深度清理门洞内动态障碍残留，并按 `mission/door_pass_offset`、`mission/door_commit_distance` 继续深入门后，再在 `mission/door_transition_duration` 和 `mission/door_transition_hold_distance` 约束下短时抑制回头选回门前侧目标，不会把 `layout_metadata.json` 中的门坐标作为探索目标。
- 房间入口不视为动态门，不调用 `/set_door_state`；普通探索依靠 Frontier 自然进入。
- 电梯门不走普通开门逻辑，只在 `CHANGE_FLOOR` 状态下由 `TakeElevator` 状态机处理。
- `local_semantic/use_truth_proximity_detector` 仅用于调试，默认关闭。
- 输出坐标使用 Gazebo `world/odom` 数值坐标。
- RGB-D 反投影默认按 OpenCV 光学坐标计算，再转换到 ROS link 坐标后使用 TF。若实际相机 TF 已经是光学坐标，可把 `detection/depth_frame_is_optical` 设为 `false`。

## 批量评测与调参

单个 seed 运行并归档：

```bash
python3 src/danger_search/tools/run_seed.py --seed 42 \
  --params config/three_floor_time_budget_test.yaml \
  --timeout 6000 --max-floor 3
```

`run_seed.py` 默认使用 GUI auto 模式：检测到 `DISPLAY` 时启动 Gazebo GUI，否则自动使用 headless。也可以显式指定：

```bash
python3 src/danger_search/tools/run_seed.py --seed 42 --headless --timeout 6000 --max-floor 3 \
  --params config/three_floor_time_budget_test.yaml
python3 src/danger_search/tools/run_seed.py --seed 42 --gui --timeout 6000 --max-floor 3 \
  --params config/three_floor_time_budget_test.yaml
```

无 `DISPLAY` 时不能使用 `--gui`，请改用 `--headless`，或通过虚拟显示运行：

```bash
xvfb-run -a python3 src/danger_search/tools/run_seed.py --seed 42 --gui --timeout 6000 --max-floor 3 \
  --params config/three_floor_time_budget_test.yaml
```

批量 seed 评测：

```bash
python3 src/danger_search/scripts/batch_evaluate.py --seeds 10 42 100 --headless --timeout 10000 --max-floor 3
```

网格搜索参数：

```bash
python3 src/danger_search/tools/tune_params.py --seeds 10 42 100 --headless --timeout 10000 --max-floor 3
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
    ├── auto_sh.log
    ├── danger_search.log
    ├── health_check.log
    └── competition_gazebo.log
```

算法启动前会检查 `/Odometry_gazebo` 和 `/livox/Pointcloud2` 是否收到首帧数据；若没有点云，会提前失败并归档 `logs/health_check.log`、Gazebo 和控制器日志。

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
mission/min_known_area: 45.0
mission/no_reachable_frontier_confirmations: 2
mission/post_door_min_distance: 2.0
mission/post_door_min_known_area: 8.0
mission/door_clear_radius: 0.9
mission/door_transition_duration: 2.5
mission/door_transition_hold_distance: 2.0
map/obstacle_inflation_radius: 0.35
map/free_clear_max_range: 6.0
map/free_clear_angle_step: 0.087
frontier/frontier_obstacle_clearance: 0.3
frontier/frontier_unknown_standoff: 0.25
frontier/switch_score_margin: 2.0
frontier/current_region_radius: 1.2
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

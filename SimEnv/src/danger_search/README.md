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
- `mission/floor_timeout` 作为非最后一层的探索配额，为后续换层保留时间；默认启用 `mission/last_floor_uses_global_budget`，最后一层仍有可达前沿或地图收益时继续探索，并由预计返航时间、`mission/return_safety_buffer` 与 `mission/max_runtime` 共同决定预算退出。关闭该开关可恢复所有楼层到达配额即结束的旧语义。
- 无前沿且无远处 FREE 目标时，会从机器人当前位置按 `mission/probe_distance` 做短距离探测前进，再重新建图；默认 `mission/probe_yaw=1.5708`，沿比赛入口到房间的全局 +Y 方向探测，避免起立后的机体 yaw 抖动把探测方向带反。探测段如果穿过已知障碍或黑名单栅格，会跳过该目标。
- Frontier 选择带短期惯性：当前区域仍可达时，新目标分数需要高出 `frontier/switch_score_margin` 才会切换；日志会记录每次选择的分数、距离、门侧、fallback 和切换原因。
- 过门后除了短时间抑制回头，还会在门后推进距离和门后侧已知面积达到 `mission/post_door_min_distance`、`mission/post_door_min_known_area` 前持续优先门后侧目标，减少刚进门又回头；临时和永久禁区都只在门口有限半径内完全跳过，远处仅按距离衰减降分，避免用无限半平面把整片可达区域永久屏蔽。
- 路径跟踪默认允许有限转向和侧移：`controller/min_linear_fraction=0.35`、`controller/max_angular=1.2`、`controller/max_lateral=0.25`，目标偏角较大时按余弦平方衰减前进速度，避免旋转中满速撞墙；转角处使用 `controller/corner_tolerance=0.10` 限制提前切换到下一段，防止在 U 型/平行走廊中切角或跳到未来折返段。
- ENTER/EXIT 不再直接复用普通导航的完成半径和栅格折线路径：机器人先在厅外等待点或轿厢中心精确就位，再原地对准最终目标，随后只跟踪一条两点直线，直到机身到达具有后足净空余量的最终端点才完成。Gazebo 已证伪跨骑卡死后的倒车恢复，因此 `mission/exit_cabin_backoff_enabled` 默认关闭；旧 backoff 代码仅保留为显式诊断能力。
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

## EXIT_CABIN 外部诊断

需要复现电梯出厢卡住时，可在 GUI 环境运行低频外部观察：

```bash
src/danger_search/tools/run_observation.sh \
  --seed 42 \
  --max-floor 2 \
  --params results/runs/<既有运行目录>/params.yaml \
  --timeout 900 \
  --diagnostic-rate 10 \
  --stop-after-exit
```

观察器不修改导航、控制或电梯状态。它仅在 `TakeElevator` 活跃后低频汇总机器人真实高度、轿厢/门板姿态、A1 trunk/足端/腿部 link 世界状态、跟踪目标、门区点云和深度 ROI；同名 Gazebo link 会保留为带序号的独立样本，足端 link 明显偏离官方 URDF 的小腿固定关节时，离线分析会标记并采用运动学推导值。观察到 `EXIT_CABIN` 成功或失败后，`--stop-after-exit` 会终止本次工程诊断。足端力话题只作为辅助记录，零值不能作为接触证据。输出位于运行目录的 `observation/`，可用以下命令生成对时时间线：

```bash
python3 src/danger_search/tools/analyze_observation.py \
  results/runs/<运行目录>/observation/telemetry.bag
```

`observation/offline_scene/` 中的 layout、manifest、门和电梯配置只用于离线复现及覆盖分析，不得接入生产算法决策。Gazebo model/link state、足端接触等裁判侧信息同样只属于外部诊断证据。

## 负障碍外部诊断

负障碍诊断必须先做短平地采集，确认原始点云、Livox、深度、TF 和地图录制没有明显降低控制频率或 RTF：

```bash
src/danger_search/tools/run_observation.sh \
  --seed 42 \
  --max-floor 2 \
  --params results/runs/<既有运行目录>/params.yaml \
  --timeout 240 \
  --ground-support-capture \
  --ground-support-shadow \
  --kill-only-sim-time 15
```

`--ground-support-shadow` 默认关闭，且必须与原始证据采集同时启用。它启动独立只订阅进程，按参赛配置的点云抽样方式维护稀疏支撑置信网格，并写出：

- `observation/ground_support_shadow.jsonl`：低频状态变化、当前路径足迹扫掠、候选反事实告警和相邻阈值结果；
- `observation/ground_support_shadow_summary.json`：负载、消息计数、方案 G 跳帧及各楼层状态汇总；
- 退出前的 `ground_support_shadow_final_grid_v1`：完整稀疏网格，避免最后一个输出周期内的证据丢失。

该进程没有 ROS publisher，`hypothetical_alert` 不会进入 Mapper、Planner 或控制器。方案 G 歧义期间收到的 cloud stamp 会被消费但不更新支撑层，对齐后只处理新 cloud。

安全静止扫描需要让假想扫掠区覆盖待观测区域时，可单独运行 shadow 进程并传入
`--minimum-lookahead`、`--maximum-lookahead`。这两个参数只改变诊断输出中的假想
扫掠长度，默认仍为 0.5～0.8 m，不得作为生产控制或固定场景坐标输入。

通过负载检查后，才允许在已知安全边界前采集。空间边界只属于本次外部诊断，不能写入参赛算法：

```bash
src/danger_search/tools/run_observation.sh \
  --seed <seed> \
  --max-floor <楼层数> \
  --params <参数文件> \
  --timeout <wall秒数> \
  --ground-support-capture \
  --kill-only-floor <楼层> \
  --kill-only-min-x <边界> \
  --kill-only-min-y <边界> \
  --kill-only-min-world-x-velocity <朝空带方向的最小世界速度>
```

kill-only 工具只订阅状态和 odom，命中边界后向 runner 发送 `SIGINT`；它不发布速度、不修改路径，也不扶正机器人。自然闭环中应同时约束朝危险区域的世界速度，避免机器人沿安全边界平行行驶时被误杀。若历史危险区域无法在安全距离前终止，应停止采集，不得再次实际坠落。

平地制动包络可额外使用 `--kill-only-min-speed` 约束触发速度。独立楼梯诊断可配置 `--kill-only-minimum-z`、`--kill-only-maximum-abs-roll`、`--kill-only-maximum-abs-pitch` 和 `--kill-only-posture-active-after-sim`；任一姿态危险条件成立即只终止 runner。手动 `/nav/goal` 只能作为明确归档的非生产测试输入，不能写入比赛策略或由真值在正式运行时触发。

需要测量正常导航链路的制动距离时，可让诊断锁存器中继生产算法命令，并在 odom 实测平移速度达到阈值后保持零速：

```bash
src/danger_search/tools/run_observation.sh \
  --seed 42 \
  --max-floor 1 \
  --params src/danger_search/config/danger_search.yaml \
  --timeout 180 \
  --ground-support-capture \
  --diagnostic-brake-speed 0.70 \
  --diagnostic-brake-relay \
  --kill-only-sim-time 20
```

该模式只属于非生产制动采集：它不注入平移或转向命令，触发前原样中继 Planner、PathFollower 和 RL 的现有命令，触发后仅保持零速。未达到阈值、姿态保护提前终止或 bag 未记录到完全停止的运行都不能扩展停车包络。

离线分析命令如下：

```bash
python3 src/danger_search/tools/analyze_ground_support.py \
  results/runs/<安全采集目录> \
  --reference-run results/runs/<无原始采集参考目录> \
  --evaluation-path-run results/runs/<历史坠落目录> \
  --independent-normal-run results/runs/<独立正常采集目录>
```

分析器区分 `SUPPORTED`、`UNKNOWN_SUPPORT` 和 `NO_SUPPORT_EVIDENCE`。视场外、遮挡、点数不足、从未观测以及单纯“没有点”均保持 `UNKNOWN_SUPPORT`；只有明确穿过当前楼面并命中更低表面的回波才作为缺失支撑正证据。历史路径和 `offline_scene/` 几何只用于离线真值对照，不得成为运行时输入。

Mapper 回放按 10 Hz 推断 tick、最新 cloud/odom 快照、cloud stamp 消费、点云抽样、当前楼层和楼层歧义跳帧近似生产语义。只要回放与实时 `map_debug` 不完全一致，就必须标记为近似回放，并禁止归因某个栅格的具体写入来源。

## 负障碍首版生产停车门

`ground_support/production_gate_enabled` 默认是 `false`。关闭时不创建支撑网格、不查询额外 TF，也不改变命令、任务状态或超时语义。工程回归可通过以下覆盖文件显式启用：

```bash
python3 src/danger_search/tools/run_seed.py --seed 42 --gui --max-floor 1 \
  --params src/danger_search/config/ground_support_gate_test.yaml \
  --timeout <wall秒数>
```

启用后的首版只在 `PathFollower` 的最终 `/cmd_vel` 出口钳制速度，不修改 `FREE/VISITED`、A*、Frontier、目标和全局速度：

- 新鲜且充分的低层回波连续确认与未来足迹走廊相交，并且命令朝危险方向平移时，锁存零速；完全停止后以 `negative_obstacle` 结束本次运行。
- `UNKNOWN_SUPPORT` 不等同于悬崖。只有即将驶出最近确认支撑范围时才临时观察停车；连续新鲜支撑恢复后自动继续，超时只报告当前支撑路径不可用。
- 支撑数据陈旧也只在接近确认支撑边界时临时停车，不按固定点云帧间隔全局停走。
- 正支撑前缀按左右足迹带的覆盖率和纵向连续性计算；完整 `sweep_half_width` 仍只用于明确 DROP 检测。外缘单个 UNKNOWN 格不会截断前缀，短内部 UNKNOWN 也只有在下游重新出现正支撑且整体覆盖率满足要求时才桥接，末端 UNKNOWN 不桥接。
- `support_path_unavailable` 只清除当前 follower 路径，保留同一个 `OpenDoorAndPass` 实例、phase、方向、门状态和目标；同一 phase 重规划有界，耗尽后以专用原因结束，不进入普通 recover。
- 停车后三份新 odom 达到停止阈值即完成。正常观察和停车上限都按仿真时间推进；wall time 仅监视 `/clock`/odom 停止推进并负责进程退出 watchdog。

安全门开启时，用独立的 0.02 s 仿真周期及时消费每个新 Livox cloud，不改变 10 Hz 任务状态机和 PathFollower 节拍。`drop_evidence_max_age=0.05` 约束的是评估时的 cloud/odom 采集年龄，评估后的等待由 `decision_period=0.10` 单独计入停车包络，二者不会重复计费。正支撑只稀疏积累机器人周围 `local_support_evidence_radius=3.0` m 内的明确楼面回波；负支撑更新和门控消费仍严格限定在最长 0.8 m 的未来足迹走廊，不扫描全地图。

`ground_support/unknown_support_hold_enabled` 只控制 UNKNOWN 边界的临时观察停车，不影响明确低层回波 DROP 的锁存停车。默认保持开启；完整任务回归若需要隔离门口 UNKNOWN 误停，可在测试覆盖中关闭并验证 explicit-DROP-only 模式，不能把 UNKNOWN 记成负障碍。

明确 DROP 停稳后，`ground_support/negative_obstacle_replan_enabled` 控制是否把触发点周围按现有物理扫掠半宽写成局部 `BLACK`，仅清除当前 follower 路径并交回原有 A* 重规划；门、电梯父任务、停车超时和速度包络异常仍保持 fail-closed。`maximum_negative_obstacle_replans` 为单次任务提供有界保护，关闭该开关即可恢复“发现 DROP 后结束任务”的旧行为。

停车包络使用 `ground_support/stop_distance_table` 的速度上界保守查表，不插值，有效速度上限直接取表格最后一档。现有表包含平地 0.60/0.651 m/s 制动样本，以及正常 EXIT 中 0.7944 m/s 实际路径方向峰值到完全停车的 0.125 m 实测上界（保守记为 0.80→0.13 m）；超过最后一档时不会外推，而是锁存零速并报告 `support_speed_out_of_envelope`。这不会修改全局 `controller/max_linear`；未来提速只需在补测停车距离、前视距离、数据年龄、横向误差和物理扫掠宽度后扩展配置表，不修改门控状态机。

生产门回归时不得同时启用 `ground_support_stop_latch.py`。后者是外部诊断实验的独立 `/cmd_vel` 中继，会破坏“生产算法只有一个最终速度发布源”的验收前提。

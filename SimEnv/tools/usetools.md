# tools 使用手册

本文介绍 `codeDG_rebuild/tools/` 中现有工具的用途、输入、典型命令、输出和安全边界。
所有命令默认从工作区根目录执行：

```bash
cd /home/xbh/tzb26/codeDG_rebuild
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

离线分析器通常使用 `python3 tools/offline/xxx.py` 调用；定位矩阵汇总器和仿真 runner
可以直接执行。

## 1. 目录与工具总览

```text
tools/
├── usetools.md
├── sim/
│   ├── run_elevator_gui_test.sh
│   └── run_localization_seed_matrix.sh
└── offline/
    ├── README.md
    ├── check_production_boundary.py
    ├── summarize_localization_matrix.py
    ├── analyze_mission_timing.py
    ├── analyze_localization_drift_bag.py
    ├── analyze_localization_quality_bag.py
    ├── analyze_localization_burst_windows.py
    ├── analyze_localization_frame_counterfactual.py
    ├── analyze_braking_bag.py
    ├── analyze_landmark_drift.py
    └── analyze_rgbd_door_bag.py
```

| 工具 | 主要用途 | 是否需要离线真值 |
|---|---|---|
| `sim/run_elevator_gui_test.sh` | 启动两层/三层 GUI 电梯专项并归档证据 | 默认不需要；可显式录制 |
| `sim/run_localization_seed_matrix.sh` | 串行运行并恢复预登记的 20-seed 定位矩阵 | 是，仅离线裁决 |
| `offline/README.md` | 说明离线工具与生产闭包的隔离原则 | 否 |
| `offline/check_production_boundary.py` | 检查生产包没有导入离线/真值工具 | 否 |
| `offline/summarize_localization_matrix.py` | 汇总矩阵证据并按冻结门槛裁决 | 是，读取 runner 产物 |
| `offline/analyze_mission_timing.py` | 统计任务耗时、运动、覆盖、安全事件和失败类型 | 否；定位误差/评分为可选 sidecar |
| `offline/analyze_localization_drift_bag.py` | 计算生产定位相对真值的漂移 | 是 |
| `offline/analyze_localization_quality_bag.py` | 分析 GICP、scan-to-map 和传感器短窗质量 | 是 |
| `offline/analyze_localization_burst_windows.py` | 跨运行验证 RGB-D/Livox 短窗分歧 | 是，读取前一工具生成的 CSV |
| `offline/analyze_localization_frame_counterfactual.py` | 逐 RGB-D 帧复算替换/保持位置的累计反事实 | 是 |
| `offline/analyze_braking_bag.py` | 提取零速换挡后的停车距离和停稳确认 | 是 |
| `offline/analyze_landmark_drift.py` | 比较电梯门板地标残差与真实定位漂移 | 是 |
| `offline/analyze_rgbd_door_bag.py` | 诊断 RGB-D 门板候选在哪一级被拒绝 | 否 |

## 2. 必须遵守的边界

`tools/offline/` 只用于任务结束后的分析：

- 不得被 `danger_search.launch`、生产 Python 模块、CMake 或 `setup.py` 导入；
- `/ground_truth/*`、Gazebo state、world、truth 和布局信息不得回流到同一轮任务；
- 分析器输出只能用于人工复核和后续预注册实验，不能修改正在运行的定位、规划或控制；
- 不用历史工作区当前结果文件冒充某一旧轮结果；sidecar 必须和对应任务同轮归档；
- 后续 Gazebo 测试必须显式设置 `GUI=true`；
- runner 会拒绝覆盖非空证据目录，应为每轮使用新的目录。

修改生产代码后至少执行：

```bash
python3 tools/offline/check_production_boundary.py
git diff --check
sha256sum -c OFFICIAL_FILES.sha256
```

当前官方哈希存在项目总账已记录的 5 项已知差异。检查时应确认没有新增差异，不能为让
命令全绿而恢复或覆盖用户保留修改。

## 3. 电梯 GUI 专项 runner

### 3.1 查看帮助

```bash
bash tools/sim/run_elevator_gui_test.sh --help
```

runner 使用正式定位和正式 `MissionManager`，只通过证据目录中的临时 YAML 调整专项
预算。正式配置文件不会被覆盖。

### 3.2 两层定向上行

到达最高层后即可结束，不要求返程：

```bash
SEED=42 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=2 \
ELEVATOR_TEST_FLOOR_TIMEOUT=33 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5 \
ELEVATOR_TEST_MAX_RUNTIME=180 \
ELEVATOR_TEST_FULL_CYCLE=0 \
ELEVATOR_TEST_LOG_DIR=/tmp/elevator_seed42_up \
tools/sim/run_elevator_gui_test.sh
```

`ELEVATOR_TEST_FULL_CYCLE=0` 只验证电梯注册和到达最高层。runner 达到目标后会主动结束
任务，所以日志中的 `external_ros_shutdown` 不应解释为生产算法失败。

### 3.3 两层完整往返

```bash
SEED=6946 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=2 \
ELEVATOR_TEST_FLOOR_TIMEOUT=33 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5 \
ELEVATOR_TEST_MAX_RUNTIME=180 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_LOG_DIR=/tmp/elevator_seed6946_full \
tools/sim/run_elevator_gui_test.sh
```

完整模式要求完成目标层、返回 0 层、退出主入口、回到 home 并成功写出结果。

### 3.4 三层完整闭环

三层正式闭环应给足正式量级预算，不能把短诊断预算的返程结果当成三层失败：

```bash
SEED=8127 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=3 \
ELEVATOR_TEST_FLOOR_TIMEOUT=280 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60 \
ELEVATOR_TEST_MAX_RUNTIME=840 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_WALL_TIMEOUT=1800 \
ELEVATOR_TEST_LOG_DIR=/tmp/elevator_seed8127_three_floor \
tools/sim/run_elevator_gui_test.sh
```

预算应按当层预注册方案调整，不能在看到运行结果后临时延长。

### 3.5 录制定位和 RGB-D 证据

```bash
SEED=8127 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=3 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_RECORD_DRIFT=1 \
ELEVATOR_TEST_RECORD_RGBD=1 \
ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 \
ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1 \
ELEVATOR_TEST_LOG_DIR=/tmp/elevator_seed8127_diagnostic \
tools/sim/run_elevator_gui_test.sh
```

开关关系：

- `ELEVATOR_TEST_RECORD_DRIFT=1`：录制内部 odom、`/cmd_vel`、IMU、关节状态和 `/rosout`；
- `ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1`：额外把 `/ground_truth/base_w` 交给独立 rosbag；
- `ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1`：启用逐帧定位日志并额外录制 `/scan`；
- `ELEVATOR_TEST_RECORD_RGBD=1`：只在电梯扫描窗口录制 RGB、深度、内参和内部 odom；
- `ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS=1`：启用默认关闭的只读地标日志。该开关
  强制要求同时录制定位和独立真值。

真值只由独立 rosbag 保存。runner 启动后还会检查生产节点信息，若发现生产节点订阅
`/ground_truth/*` 或 `/Odometry_gazebo`，会以环境前置失败退出。

### 3.6 常见输出

证据目录可能包含：

| 文件 | 内容 |
|---|---|
| `mission.log` | 正式任务和定位 `/rosout` |
| `auto.log` | Gazebo、Unitree 控制器和启动链日志 |
| `TEST_SUMMARY.txt` | runner 结果、seed、预算和证据路径 |
| `elevator_diagnostic.bag` | 定位、控制、姿态及可选真值、`/scan` |
| `elevator_rgbd.bag` | 电梯扫描窗口的 RGB-D 数据 |
| `localization_drift.csv/json` | runner 清理阶段生成的离线定位误差 |
| `localization_quality.csv/json` | 逐帧 scan-to-map 质量 |
| `localization_motion_windows.csv` | 运动窗口统计 |
| `localization_sensor_windows.csv` | RGB-D/Livox 同窗增量 |
| `landmark_drift.json` | 电梯地标诊断准入结果 |
| `production_node_info.txt` | 生产订阅边界快照 |

runner 返回码：

- `0`：达到本次定向或完整闭环目标；
- `1`：算法失败或专项超时；
- `2`：参数、端口、启动或环境前置失败；
- `130`：用户或外部信号中止。

## 4. 生产边界检查

```bash
python3 tools/offline/check_production_boundary.py
```

指定其他工作区：

```bash
python3 tools/offline/check_production_boundary.py \
  --workspace /home/xbh/tzb26/codeDG_rebuild
```

输出“生产/离线边界审计通过。”且返回 `0` 表示通过；发现生产安装、import 或 launch
闭包包含离线工具时返回 `1` 并逐条列出问题。

## 5. 任务时间、覆盖和失败账本

最小调用：

```bash
python3 tools/offline/analyze_mission_timing.py \
  /tmp/elevator_seed6946_full/mission.log \
  > /tmp/elevator_seed6946_full/mission_timing.json
```

绑定同轮完整 sidecar：

```bash
python3 tools/offline/analyze_mission_timing.py \
  /tmp/elevator_seed6946_full/mission.log \
  --auto-log /tmp/elevator_seed6946_full/auto.log \
  --test-summary /tmp/elevator_seed6946_full/TEST_SUMMARY.txt \
  --result-file /tmp/elevator_seed6946_full/detected_danger.json \
  --localization-report /tmp/elevator_seed6946_full/localization_drift.json \
  --evaluation-file /tmp/elevator_seed6946_full/evaluation_result.json \
  > /tmp/elevator_seed6946_full/mission_timing.json
```

主要输出：

- 正式终态和失败分类；
- 各状态、任务阶段、零速原因和运动类型耗时；
- 有效移动、命令移动、平均/峰值命令速度；
- 原始里程和过滤定位跳变后的里程；
- 每层初始、最大和最终覆盖代理；
- STUCK、恢复和安全事件；
- 可选识别数、定位误差和评分。

没有 `mission.log` 时仍可通过 `--auto-log` 或 `--test-summary` 分类启动失败。缺失
sidecar 的字段保持 `null`，不会猜测。

## 6. 定位漂移分析

输入 bag 必须同时含内部 odom 和独立真值：

```bash
python3 tools/offline/analyze_localization_drift_bag.py \
  /tmp/elevator_seed8127_diagnostic/elevator_diagnostic.bag \
  --csv /tmp/elevator_seed8127_diagnostic/localization_drift.csv \
  --json /tmp/elevator_seed8127_diagnostic/localization_drift.json
```

默认话题：

- 定位：`/danger_search/localization/odom`
- 真值：`/ground_truth/base_w`
- 最大时间同步间隔：`0.10 s`

输出包括 RMSE、P95、最大/最终平移误差、yaw 误差、真值路程和首个对齐 stamp。CSV
中的 `elapsed` 以首个对齐定位 header stamp 为原点，不是 ROS `/clock` 绝对时间。

## 7. 定位质量和短窗分析

### 7.1 单轮逐帧质量

该工具需要 bag 中同时存在：

- `/danger_search/localization/odom`
- `/ground_truth/base_w`
- `/rosout` 中的逐帧定位诊断
- 分析结构主轴时需要 `/scan`

典型调用：

```bash
python3 tools/offline/analyze_localization_quality_bag.py \
  /tmp/elevator_seed8127_diagnostic/elevator_diagnostic.bag \
  --csv /tmp/elevator_seed8127_diagnostic/localization_quality.csv \
  --windows-csv /tmp/elevator_seed8127_diagnostic/localization_motion_windows.csv \
  --sensor-windows-csv /tmp/elevator_seed8127_diagnostic/localization_sensor_windows.csv \
  --json /tmp/elevator_seed8127_diagnostic/localization_quality.json
```

该工具用于分析 GICP、scan-to-map、局部运动轴、RGB-D/Livox 同窗增量和高误差窗口。
如果运行时没有设置 `ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1`，bag 可能缺少逐帧日志，
分析器会明确失败。

### 7.2 跨运行短窗分歧

输入是上一工具生成的 `localization_sensor_windows.csv`：

```bash
python3 tools/offline/analyze_localization_burst_windows.py \
  --validation-run seed1612=/tmp/seed1612/localization_sensor_windows.csv \
  --validation-run seed6946=/tmp/seed6946/localization_sensor_windows.csv \
  --confirmation-run route8072=/tmp/route8072/localization_sensor_windows.csv \
  --json /tmp/localization_burst_validation.json
```

至少需要一轮 validation 和一轮 confirmation。输出包含每轮高/低样本、AUC、bootstrap
区间和 Livox 局部改善。脚本成功完成分析时返回 `0`；是否达到研究门槛应读取 JSON
中的 `passed`，不能只看进程返回码。

### 7.3 逐 RGB-D 帧累计反事实

输入是带逐帧 `/rosout`、内部 odom 和独立真值的历史 bag：

```bash
python3 tools/offline/analyze_localization_frame_counterfactual.py \
  --validation-run seed1612=/tmp/seed1612/elevator_diagnostic.bag \
  --validation-run seed6946=/tmp/seed6946/elevator_diagnostic.bag \
  --confirmation-run route8072=/tmp/route8072/elevator_diagnostic.bag \
  --frames-csv-directory /tmp/frame_counterfactual \
  --json /tmp/frame_counterfactual.json
```

输出比较：

- 异常帧使用同窗 Livox 增量；
- 异常帧保持位置；
- 两者的局部误差与全程累计 RMSE/P95/最终误差。

脚本返回 `0` 表示分析过程完成；生产候选是否通过必须查看 `passed` 和
`action_passes_all_runs`。

## 8. 制动距离分析

bag 必须包含 `/cmd_vel`、内部定位和独立真值：

```bash
python3 tools/offline/analyze_braking_bag.py \
  /tmp/elevator_seed6946_full/elevator_diagnostic.bag \
  --output /tmp/elevator_seed6946_full/braking.json
```

默认只提取停车前请求不低于 `0.30 m/s` 的事件。每个事件记录：

- 停车前请求和直行/弧线分类；
- 真值初速、停车时间、二维路径长和沿初始航向前向位移；
- 三个连续定位低速样本的确认时刻和当时真值速度；
- 停稳前重新出现非零命令等无效原因。

`--minimum-pre-stop-command` 是分析口径，不应用来看到结果后降低门槛凑事件。正式制动
矩阵应沿用预注册档位。

## 9. 电梯语义地标漂移分析

该工具依赖同一轮的三个文件：

```bash
python3 tools/offline/analyze_landmark_drift.py \
  /tmp/elevator_landmark/mission.log \
  /tmp/elevator_landmark/localization_drift.csv \
  /tmp/elevator_landmark/localization_drift.json \
  --json /tmp/elevator_landmark/landmark_drift.json
```

`mission.log` 必须由显式开启
`ELEVATOR_TEST_LANDMARK_DRIFT_DIAGNOSTICS=1` 的运行生成。分析器把门板残差与锚点时刻
之后的真值漂移增量对齐，并检查高漂移复现、低漂移负对照、方向、向量误差、确认延迟
和任务后果提前量。

返回码：

- `0`：所有冻结准入门槛通过；
- `1`：分析完成，但准入门槛未通过；
- `2`：参数或输入文件错误。

返回 `1` 是有效实验结论，不应当作工具崩溃，也不能通过降低阈值重跑消除。

## 10. RGB-D 电梯门板诊断

输入是 runner 在电梯扫描窗口生成的 `elevator_rgbd.bag`。先从 `mission.log` 找到待
分析的传感器时间窗，再运行：

```bash
python3 tools/offline/analyze_rgbd_door_bag.py \
  /tmp/elevator_rgbd/elevator_rgbd.bag \
  --start 20.0 \
  --end 38.0 \
  --csv /tmp/elevator_rgbd/door_candidates.csv \
  --export-dir /tmp/elevator_rgbd/frames
```

bag 需要包含：

- `/real_sense/rgb/image_raw/compressed`
- `/real_sense/depth/image_raw`
- `/real_sense/rgb/camera_info`
- `/danger_search/localization/odom`

工具会统计暗色候选、深度配对、平面拟合和最终门板候选，并可导出代表 RGB 帧。可用
`--minimum-panel-width`、`--minimum-panel-height` 做离线对照，但这些参数不会修改正式
配置，离线对照结果也不能直接授权放宽生产门槛。

## 11. 官方识别评分

评分器不在 `tools/` 中。必须先结束正式任务，再运行：

```bash
python3 src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file results/danger_truth.json \
  --detected-file results/detected_danger.json \
  --output-file results/evaluation_result.json
```

随后把同轮 `evaluation_result.json` 显式传给 `analyze_mission_timing.py`。正式任务不得
读取 `danger_truth.json` 或 `evaluation_result.json`。

## 12. 推荐的完整工作流

### 运行前

```bash
python3 tools/offline/check_production_boundary.py
git diff --check
sha256sum -c OFFICIAL_FILES.sha256
```

确认 11311 没有其他 ROS master：

```bash
ss -ltnp | rg ':11311\b' || true
```

### 运行

1. 预先声明 seed、楼层、预算、成功条件和证据目录；
2. 使用 `GUI=true` 启动 runner；
3. 环境失败最多同配置重试一次，算法失败不通过重跑消除；
4. 不在运行中读取真值或修改阈值。

### 运行后

1. 先查看 `TEST_SUMMARY.txt`、`mission.log` 和正式结果；
2. 按录包内容运行对应离线分析器；
3. 把同轮结果、定位摘要和评分传给时间账本；
4. 在新阶段报告中记录命令、输入、输出、返回码和缺失字段；
5. 检查进程和 ROS master 已清理。

```bash
ps -eo pid,ppid,state,cmd | \
  rg 'roscore|rosmaster|roslaunch|gazebo|gzserver|gzclient|junior_ctrl|rosbag' | \
  rg -v 'rg ' || true
ss -ltnp | rg ':11311\b' || true
```

## 13. 常见问题

### `ModuleNotFoundError: rosbag`

当前 shell 没有加载 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### runner 提示证据目录非空

runner 为防止覆盖历史证据会拒绝启动。换一个新目录，不要删除或覆盖旧证据：

```bash
ELEVATOR_TEST_LOG_DIR=/tmp/elevator_$(date +%Y%m%d_%H%M%S)
```

### 定位质量工具提示缺少逐帧诊断

录包时没有启用：

```bash
ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1
```

不能从普通 bag 凭空恢复未记录的逐帧日志，应重新预注册下一轮，而不是伪造输入。

### 定位漂移工具提示缺少真值

必须在运行前同时设置：

```bash
ELEVATOR_TEST_RECORD_DRIFT=1
ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1
```

真值只能由独立 rosbag 录制，不能为补数据临时接入生产节点。

### runner 返回失败，但任务日志写 `return_home_completed`

runner 和生产任务可能使用不同验收目标。例如三层定向 runner 要求到达 2 层，而短预算
可能使生产任务提前安全返航。应分别记录：

- runner 是否达到本次专项目标；
- 生产任务最终是成功、显式失败还是外部中止。

不得把两者合并成单一“成功/失败”。

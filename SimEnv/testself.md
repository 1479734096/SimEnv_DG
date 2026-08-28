以后建议始终按“独立证据目录 → GUI runner → 离线分析”的流程测试。

## 1. 测试前准备

cd /home/xbh/tzb26/codeDG/SimEnv

source /opt/ros/noetic/setup.bash
source devel/setup.bash

python3 tools/offline/check_production_boundary.py
git diff --check
ss -ltnp | rg ':11311\b' || true

如果修改了 C++，先构建：

catkin_make --pkg danger_search danger_search_localization -j4

Python 回归测试：

PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:src/danger_search \
pytest -o cache_dir=/tmp/.pytest_cache src/danger_search/test/

## 2. 启动常用测试

### 方式一：一键自动化测试（推荐，runner 自动交互与监控）

#### 两层快速上行

只验证入口、电梯注册和 0→1：

```bash
EVIDENCE_DIR="/home/xbh/tzb26/evidence/simenv/elevator_seed42_up_$(date +%Y%m%d_%H%M%S)"

SEED=42 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=2 \
ELEVATOR_TEST_FLOOR_TIMEOUT=33 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5 \
ELEVATOR_TEST_MAX_RUNTIME=180 \
ELEVATOR_TEST_FULL_CYCLE=0 \
ELEVATOR_TEST_LOG_DIR="$EVIDENCE_DIR" \
tools/sim/run_elevator_gui_test.sh
```

#### 当前固定验收：seed=8127 两层全周期闭环（STAGE33 标准验收）

目标：`进门 → 0 层探索与危险源识别 → 0→1 → 1 层探索与危险源识别 → 1→0 → 出门 → 原路返航回起点 → 写出 detected_danger.json → success`。
专项 runner 会通过 FIFO 自动发送 `2`（站立）和 `6`（RL `/cmd_vel`），并在日志确认控制器就绪后才启动正式任务。

**推荐标准测试命令（证据持久化至 `/home/xbh/tzb26/evidence/simenv/`）**：

```bash
EVIDENCE_DIR="/home/xbh/tzb26/evidence/simenv/seed8127_2floor_new_$(date +%Y%m%d_%H%M%S)"

SEED=8127 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=2 \
ELEVATOR_TEST_FLOOR_TIMEOUT=280.0 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60.0 \
ELEVATOR_TEST_MAX_RUNTIME=620.0 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_RECORD_DRIFT=1 \
ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 \
ELEVATOR_TEST_LOCALIZATION_MAX_RANGE=15.0 \
ELEVATOR_TEST_ADAPTIVE_RGBD_LOCALIZATION_ENABLED=false \
ELEVATOR_TEST_LOG_DIR="$EVIDENCE_DIR" \
tools/sim/run_elevator_gui_test.sh
```

**单行复制即用**：

```bash
EVIDENCE_DIR="/home/xbh/tzb26/evidence/simenv/seed8127_2floor_new_$(date +%Y%m%d_%H%M%S)" && SEED=8127 GUI=true ELEVATOR_TEST_FLOOR_COUNT=2 ELEVATOR_TEST_FLOOR_TIMEOUT=280.0 ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60.0 ELEVATOR_TEST_MAX_RUNTIME=620.0 ELEVATOR_TEST_FULL_CYCLE=1 ELEVATOR_TEST_RECORD_DRIFT=1 ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 ELEVATOR_TEST_LOCALIZATION_MAX_RANGE=15.0 ELEVATOR_TEST_ADAPTIVE_RGBD_LOCALIZATION_ENABLED=false ELEVATOR_TEST_LOG_DIR="$EVIDENCE_DIR" tools/sim/run_elevator_gui_test.sh
```

双锚点、开口重试和返航闭环的结构化事件验收口径见
[`docs/team/RUNBOOK.md`](docs/team/RUNBOOK.md) 的“STAGE33 双锚点与电梯重试验收”。

#### 三层完整闭环

验证 0→1→2→0→home：

```bash
EVIDENCE_DIR="/home/xbh/tzb26/evidence/simenv/elevator_seed8127_three_floor_$(date +%Y%m%d_%H%M%S)"

SEED=8127 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=3 \
ELEVATOR_TEST_FLOOR_TIMEOUT=330.0 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60.0 \
ELEVATOR_TEST_MAX_RUNTIME=840.0 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_WALL_TIMEOUT=1800 \
ELEVATOR_TEST_LOG_DIR="$EVIDENCE_DIR" \
tools/sim/run_elevator_gui_test.sh
```

---

### 方式二：赛方原生 auto.sh 双终端测试（手动/分布式调试）

#### 终端 1：启动仿真环境与物理底盘
```bash
cd /home/xbh/tzb26/codeDG/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 启动 2 层仿真场景；正式生产链不需要发布真值
SEED=8127 FLOOR_COUNT=2 GUI=true ENABLE_GROUND_TRUTH=0 ./auto.sh
```
> **终端 1 键盘操作**：
> 1. Gazebo 机器人加载后，输入 **`2`** 并回车（进入 fixed stand 站立姿态）；
> 2. 站立平稳后，输入 **`6`** 并回车（进入 RL `/cmd_vel` 控制模式）。

#### 终端 2：启动自主搜索算法
```bash
cd /home/xbh/tzb26/codeDG/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 启动参赛核心算法节点
roslaunch danger_search danger_search.launch \
  controller_ready_ack:=true \
  localization_max_range:=15.0 \
  adaptive_rgbd_localization_enabled:=false
```

---

证据目录必须不存在或为空。不要重复使用旧目录。

## 3. 需要定位诊断时

EVIDENCE_DIR="/home/xbh/tzb26/evidence/simenv/elevator_seed8127_diagnostic_$(date +%Y%m%d_%H%M%S)"

SEED=8127 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=3 \
ELEVATOR_TEST_MAX_RUNTIME=840 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_RECORD_DRIFT=1 \
ELEVATOR_TEST_RECORD_RGBD=1 \
ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 \
ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1 \
ELEVATOR_TEST_LOG_DIR="$EVIDENCE_DIR" \
tools/sim/run_elevator_gui_test.sh

这些开关会生成定位 bag、真值、逐帧质量和 RGB-D 扫描证据。真值只由独立 rosbag 录制，不进入生产决策。

## 4. 运行时怎么看日志

另开终端，设置同一个目录（以实际生成的 EVIDENCE_DIR 为准）：

EVIDENCE_DIR="/home/xbh/tzb26/evidence/simenv/seed8127_2floor_new_xxx"

看启动、Gazebo 和控制器：

tail -f "$EVIDENCE_DIR/auto.log"

看 MissionManager 和定位：

tail -f "$EVIDENCE_DIR/mission.log"

只看任务状态转换：

tail -f "$EVIDENCE_DIR/mission.log" |
rg --line-buffered 'mission_state_transition|mission_result_write|reason_code'

只看电梯过程：

tail -f "$EVIDENCE_DIR/mission.log" |
rg --line-buffered 'elevator_|floor_transition|TakeElevator'

只看安全门和恢复：

tail -f "$EVIDENCE_DIR/mission.log" |
rg --line-buffered 'ground_support|DROP|SAFETY_|STUCK|unsafe_rotation'

只看定位异常：

tail -f "$EVIDENCE_DIR/mission.log" |
rg --line-buffered 'localization_|GICP|scan_to_map|stance_unhealthy'

检查启动崩溃：

rg -n 'ERROR|FATAL|Segmentation|core dumped|Failed|失败' \
"$EVIDENCE_DIR/auto.log"

### 运行时只读监控协议

监控全程只读：不修改参数、不重启 Gazebo、不追加 seed；不暂存、提交、打标签；
不用 pkill、killall 或广域清理；不删除 RUN_DIR 中任何数据。

每 60 秒轮询一次：

- 主 runner 及关键子进程是否存活；
- mission.log 最新 mission_runtime 字段：elapsed、state、current_floor、
  target_floor、coverage_ratio、pose、terminal_outcome、return_home_completed；
- 新增关键事件：mission_result_write、floor_change_completed、elevator*、
  ENTER_CABIN / EXIT_CABIN、ground_support_constraint_installed/revoked、
  ground_support_path_replan_exhausted、DROP / HOLD / STUCK / replan；
- elevator_diagnostic.bag.active 是否持续增长；
- /clock 是否继续推进。

日志提取必须有界：JSON 解析只取关键字段，禁止整段输出 mission.log 及
localization_map_registration_frame 等高频日志。不因短时覆盖率平台、HOLD、
单次重规划或较低 RTF 提前终止。

runner 自然结束时等待其自行完成 rosbag 停止、reindex 和清理。只有已经出现
明确 mission_result_write 失败且 runner 长时间不退出时，才可向主 runner
进程组发送一次 SIGINT；发送前先记录证据（会触发 runner 的 130 号退出并写
result=中止，属人为终止，不是算法结论）。

失败分类：

- 算法确定性失败/专项超时：mission_result_write 非 success，或 runner
  exit=1（exit=1 含专项超时，需结合 mission.log 区分，不以退出码为唯一依据）；
- 启动/环境前置失败：runner exit=2；
- 基础设施失败/外部终止：master、Gazebo 或 rosbag 异常退出，或 runner
  exit=130（收到 INT/TERM）；
- wall timeout 与 mission timeout 分开记录；
- 确定性失败后不重跑，只还原最短因果链；基础设施启动失败清理后最多用
  完全相同配置重试一次。

## 5. 运行结束后先看什么

### 第一项：runner 总结

cat "$EVIDENCE_DIR/TEST_SUMMARY.txt"

这里显示 runner 判定、seed、预算和各证据路径。

### 第二项：正式任务终态

rg -n 'mission_result_write|return_home_completed|floor_change_failed|normal_result_written' \
"$EVIDENCE_DIR/mission.log" |
tail -n 30

注意区分：

- runner 是否达到本次专项目标；
- 正式任务是否成功完成；
- 是否由 runner 主动结束。

例如 runner 要求到达 2 层，但任务因预算安全返航，可能出现“runner 失败、生产任务成功”。

### 第三项：生成时间账本

python3 tools/offline/analyze_mission_timing.py \
"$EVIDENCE_DIR/mission.log" \
--auto-log "$EVIDENCE_DIR/auto.log" \
--test-summary "$EVIDENCE_DIR/TEST_SUMMARY.txt" \
--localization-report "$EVIDENCE_DIR/localization_drift.json" \
> "$EVIDENCE_DIR/mission_timing.json"

查看：

python3 -m json.tool "$EVIDENCE_DIR/mission_timing.json" | less

重点字段：

- run_outcome
- state_seconds
- task_phase_seconds
- zero_reason_seconds
- effective_motion_ratio
- coverage_by_floor
- stuck_episode_count
- safety_event_counts
- localization_error

### 运行结束后收尾

1. 等待 runner 自然返回并记录退出码；
2. 确认 elevator_diagnostic.bag 已结束 active 状态；必要时只对副本 reindex，
   不破坏原文件；
3. 执行 rosbag info，确认包含 /scan，记录起止时间、帧数、频率和大小；
4. 确认端口 11311 释放，且无 roscore/rosmaster/roslaunch/gazebo/gzserver/
   gzclient/junior_ctrl/rosbag 残留进程；不得杀其他会话进程；
5. 在 RUN_DIR 内生成 RUN_ANALYSIS.md，至少包括：
   - 配置、HEAD、运行时长、退出码；
   - 完整状态转换时间线；
   - 每层最高/最终覆盖率；
   - 电梯进出舱与换层结果；
   - GroundSupport 安装/撤销/耗尽；
   - DROP/HOLD/STUCK/重规划统计；
   - 地图证据统计：LOW_CANDIDATE、LOW_CONFIRMED、撤销数、HIGH_HARD；
   - 最短成功链或失败因果链；
   - 证据缺口与未验证风险。

## 6. 各类证据怎么看

### 定位误差

python3 -m json.tool "$EVIDENCE_DIR/localization_drift.json" | less

重点看：

- rms_position_error
- p95_position_error
- maximum_position_error
- final_position_error
- maximum_absolute_yaw_error

### 定位漂移测量方法（规范，勿混用）

**唯一正式口径：离线 RMSE 相对真值**（`analyze_localization_drift_bag.py`，
`ELEVATOR_TEST_RECORD_DRIFT=1` 录制真值后生成 `localization_drift.json`）。
历史记录均为该口径（如 fixed.md 中 RMSE 0.056-0.089 m）。报告定位漂移一律引用
rms/p95/maximum_position_error。

**无真值时的金标准替代**：稳定特征拟合（如走廊墙线 x 坐标的 std 与趋势）。
实测走廊墙线 std≈0.05 m 且前后半均值一致 = 定位稳定，与 RMSE 8 cm 同量级。

**禁止使用（本次已踩坑）**：准静止帧对的"点云质心偏移"或"网格互相关平移峰值"。
机器人微动/遮挡/稀疏会改变点云覆盖，互相关会锁定错误局部峰值（假阳性
0.2-0.3 m），与 pose 波动（0.02 m）和墙线 std（0.05 m）矛盾，不能作为漂移证据。
判断"定位是否漂移"以 pose/稳定特征为准；判断"点云为何错位"另查时间匹配与外参，
不要用互相关平移反推漂移。

### 定位逐帧质量

python3 -m json.tool "$EVIDENCE_DIR/localization_quality.json" | less

逐帧数据：

less "$EVIDENCE_DIR/localization_quality.csv"
less "$EVIDENCE_DIR/localization_sensor_windows.csv"

### bag 内容

rosbag info "$EVIDENCE_DIR/elevator_diagnostic.bag"
rosbag info "$EVIDENCE_DIR/elevator_rgbd.bag"

### 正式识别结果

任务结束后立即保存同轮结果，避免下一轮覆盖：

cp results/detected_danger.json \
"$EVIDENCE_DIR/detected_danger.json"

查看：

python3 -m json.tool "$EVIDENCE_DIR/detected_danger.json"

### 离线评分

只能在任务完全结束后运行：

python3 src/building_obstacles/scripts/evaluate_danger.py \
--truth-file results/danger_truth.json \
--detected-file "$EVIDENCE_DIR/detected_danger.json" \
--output-file "$EVIDENCE_DIR/evaluation_result.json"

然后重新生成完整时间账本：

python3 tools/offline/analyze_mission_timing.py \
"$EVIDENCE_DIR/mission.log" \
--auto-log "$EVIDENCE_DIR/auto.log" \
--test-summary "$EVIDENCE_DIR/TEST_SUMMARY.txt" \
--result-file "$EVIDENCE_DIR/detected_danger.json" \
--localization-report "$EVIDENCE_DIR/localization_drift.json" \
--evaluation-file "$EVIDENCE_DIR/evaluation_result.json" \
> "$EVIDENCE_DIR/mission_timing.json"

## 7. 测试结束后的清理检查

ps -eo pid,ppid,state,cmd |
rg 'roscore|rosmaster|roslaunch|gazebo|gzserver|gzclient|junior_ctrl|rosbag' |
rg -v 'rg ' || true

ss -ltnp | rg ':11311\b' || true

正常情况下两条命令都不应显示相关进程或端口。

运行结束后先生成 `mission_timing.json`、`key_events.log`、`scan_bag_info.txt` 和
`evidence.sha256`。不删除 RUN_DIR 中任何原始数据；如失败原因仍需点云离线裁决，
只在副本上执行 reindex 或裁决，不破坏原文件、不清理原始 bag 与高频原始日志。
报告只保留关键时间、状态、计数、哈希和证据目录路径。

不要把 `TEST_SUMMARY.txt` 的 `result=中止` 单独当作算法结论：若人工停止，必须同时
读取 `mission.log` 的最后一个算法事件，区分“确定性算法失败后中止”和“未到失败点的
基础设施中止”。

完整工具说明已写在 tools/usetools.md。

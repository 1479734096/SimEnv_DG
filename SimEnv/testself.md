本文件不再规定或推荐任何自动测试、回归测试、固定 seed 矩阵、pytest/gtests、测试 runner 或
长时 Gazebo 闭环。以后先理清代码逻辑、调用链和失败边界，再直接实施最小修改；测试源码和
历史命令仅保留作查阅资料，不作为当前工作流程。

## 1. 人工启动与观察

### 赛方原生 auto.sh 双终端人工运行

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

## 2. 运行时怎么看日志

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

### 人工运行只读观察协议

监控全程只读：不修改参数、不重启 Gazebo、不追加 seed；不暂存、提交、打标签；
不用 pkill、killall 或广域清理；不删除 RUN_DIR 中任何数据。

每 60 秒轮询一次：

- 仿真、控制器和任务进程是否存活；
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

人工运行结束时等待 rosbag 停止、reindex 和清理。只有已经出现明确
`mission_result_write` 失败且进程长时间不退出时，才可向该进程组发送一次 SIGINT；
发送前先记录证据，人工终止不作为算法结论。

失败分类：

- 算法问题：`mission_result_write` 非 `success`，结合 `mission.log` 还原最短因果链；
- 启动或基础设施问题：记录 master、Gazebo、控制器和 rosbag 的实际状态；
- wall timeout 与 mission timeout 分开记录；
- 出现确定性问题后不追加运行，不通过改变 seed 或参数掩盖问题。

## 3. 运行结束后先看什么

### 第一项：正式任务终态

rg -n 'mission_result_write|return_home_completed|floor_change_failed|normal_result_written' \
"$EVIDENCE_DIR/mission.log" |
tail -n 30

注意区分正式任务终态和人工停止原因，不把人工中止直接归因于算法。

### 第二项：生成时间账本

python3 tools/offline/analyze_mission_timing.py \
"$EVIDENCE_DIR/mission.log" \
--auto-log "$EVIDENCE_DIR/auto.log" \
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

1. 等待人工运行自然结束并记录退出状态；
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

## 4. 各类证据怎么看

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
--result-file "$EVIDENCE_DIR/detected_danger.json" \
--localization-report "$EVIDENCE_DIR/localization_drift.json" \
--evaluation-file "$EVIDENCE_DIR/evaluation_result.json" \
> "$EVIDENCE_DIR/mission_timing.json"

## 5. 人工运行结束后的清理检查

ps -eo pid,ppid,state,cmd |
rg 'roscore|rosmaster|roslaunch|gazebo|gzserver|gzclient|junior_ctrl|rosbag' |
rg -v 'rg ' || true

ss -ltnp | rg ':11311\b' || true

正常情况下两条命令都不应显示相关进程或端口。

运行结束后先生成 `mission_timing.json`、`key_events.log`、`scan_bag_info.txt` 和
`evidence.sha256`。不删除 RUN_DIR 中任何原始数据；如失败原因仍需点云离线裁决，
只在副本上执行 reindex 或裁决，不破坏原文件、不清理原始 bag 与高频原始日志。
报告只保留关键时间、状态、计数、哈希和证据目录路径。

人工停止时必须同时读取 `mission.log` 的最后一个算法事件，区分“算法失败”与“尚未到失败点
的基础设施中止”。

完整工具说明已写在 tools/usetools.md。

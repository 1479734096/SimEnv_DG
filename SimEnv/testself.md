以后建议始终按“独立证据目录 → GUI runner → 离线分析”的流程测试。

## 1. 测试前准备

cd /home/xbh/tzb26/codeDG_rebuild

source /opt/ros/noetic/setup.bash
source devel/setup.bash

python3 tools/offline/check_production_boundary.py
git diff --check
ss -ltnp | rg ':11311\b' || true

如果修改了 C++，先构建：

catkin_make --pkg danger_search danger_search_localization -j4

Python 回归：

python3 -m pytest src/danger_search/test -q

## 2. 启动常用测试

### 两层快速上行

只验证入口、电梯注册和 0→1：

EVIDENCE_DIR=/tmp/elevator_seed42_up

SEED=42 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=2 \
ELEVATOR_TEST_FLOOR_TIMEOUT=33 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5 \
ELEVATOR_TEST_MAX_RUNTIME=180 \
ELEVATOR_TEST_FULL_CYCLE=0 \
ELEVATOR_TEST_LOG_DIR="$EVIDENCE_DIR" \
tools/sim/run_elevator_gui_test.sh

### 两层完整往返

验证 0→1→0→home：

EVIDENCE_DIR=/tmp/elevator_seed6946_full

SEED=6946 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=2 \
ELEVATOR_TEST_FLOOR_TIMEOUT=33 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5 \
ELEVATOR_TEST_MAX_RUNTIME=180 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_LOG_DIR="$EVIDENCE_DIR" \
tools/sim/run_elevator_gui_test.sh

### 三层完整闭环

验证 0→1→2→0→home：

EVIDENCE_DIR=/tmp/elevator_seed8127_three_floor

SEED=8127 \
GUI=true \
ELEVATOR_TEST_FLOOR_COUNT=3 \
ELEVATOR_TEST_FLOOR_TIMEOUT=280 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60 \
ELEVATOR_TEST_MAX_RUNTIME=840 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_WALL_TIMEOUT=1800 \
ELEVATOR_TEST_LOG_DIR="$EVIDENCE_DIR" \
tools/sim/run_elevator_gui_test.sh

证据目录必须不存在或为空。不要重复使用旧目录。

## 3. 需要定位诊断时

EVIDENCE_DIR=/tmp/elevator_seed8127_diagnostic

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

另开终端，设置同一个目录：

EVIDENCE_DIR=/tmp/elevator_seed8127_diagnostic

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

## 6. 各类证据怎么看

### 定位误差

python3 -m json.tool "$EVIDENCE_DIR/localization_drift.json" | less

重点看：

- rms_position_error
- p95_position_error
- maximum_position_error
- final_position_error
- maximum_absolute_yaw_error

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

完整工具说明已写在 codeDG_rebuild/tools/usetools.md:1。


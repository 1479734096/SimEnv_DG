# 第 11 层电梯专项完整测试报告

## 1. 结论

2026-07-21 在 `seed=42`、两层、Gazebo GUI 环境中完成了一次生产链 0→1 层电梯闭环测试。测试通过以下完整动作链：

`RGB-D 电梯注册 → 厅外导航 → 呼叫当前层 → 打开当前层门 → 进入轿厢 → 关闭当前层门 → 呼叫目标层 → 等待定位稳定 → 打开目标层门 → 离开轿厢 → 关闭目标层门 → 确认楼层 1 → 恢复探索`

任务在仿真任务时间 `66.760s` 从 `CHANGE_FLOOR` 返回楼层 1 的 `EXPLORE_FLOOR`，专项脚本返回码为 0。

本次专项预算最终设置为：

- `floor_timeout=33.0s`
- `min_floor_explore_time=33.0s`
- `max_runtime=180.0s`

## 2. 预算问题与修正

第一次尝试只把 `floor_timeout` 改为 `20.0s`，但正式配置仍有 `min_floor_explore_time=60.0s`。日志在楼层超时后明确给出：

```text
action=continue_until_min_floor_time
reason_code=floor_timeout_before_min_explore_time
floor_timeout=20.0
min_floor_explore_time=60.0
```

因此“预算结束后仍不去电梯”的原因不是电梯任务没有响应，而是任务预算决策在进入 `_decide_mission_action()` 之前仍受最小探索时间约束。专项脚本现将 `floor_timeout` 与 `min_floor_explore_time` 同步设置为同一个测试预算，并按用户要求把默认值调整为 `33.0s`。

20 秒尝试保留了有效的视觉注册证据，但因预算门槛不一致主动停止，不计为完整电梯验收。

## 3. 测试环境与启动边界

- 工作区：`/home/xbh/tzb26/codeDG_rebuild`
- 分支：`main`
- 场景：`SEED=42`、`FLOOR_COUNT=2`
- Gazebo：`GUI=true`、暂停启动后由既有安全同步恢复物理
- 控制器：先进入 fixed stand，再进入 RL `/cmd_vel` 模式
- 正式任务：`roslaunch danger_search danger_search.launch`
- 专项配置：仅在证据目录生成临时 YAML，不修改正式配置
- 证据目录：`/tmp/danger_search_elevator_test_20260721_215714`

测试脚本显式关闭：

- `ENABLE_REFEREE_ODOM=false`
- `ENABLE_GROUND_TRUTH=false`
- `ENABLE_POINTCLOUD_CONVERTER=false`
- `POINTCLOUD_USE_GROUND_TRUTH_ODOM=false`
- `WRITE_GENERATED_TRUTH_COPY=false`

## 4. 电梯视觉注册结果

全向扫描在仿真时间 `19.612s` 开始，在 `28.510s` 获得连续确认，耗时约 `8.898s`，没有等到 18 秒超时：

```text
elevator_id=elevator_main
door_id=elevator_floor_0
floor=0
center=(1.82, 2.33)
outward_yaw=3.10
width=0.74
evidence=5000
source=rgbd_dark_panel_pending_service_confirmation
```

候选宽度 `0.74m` 位于当前任务层允许的 `[0.65m, 1.80m]` 范围内，与单扇滑动门板语义一致。注册对象的位置、朝向和门 ID 均指向楼层 0 电梯门，不是主入口门框。

## 5. 预算切换结果

任务时间 `33.160s` 触发：

```text
old_state=EXPLORE_FLOOR
new_state=CHANGE_FLOOR
current_floor=0
target_floor=1
reason_code=floor_timeout_change_floor_change_floor
remaining_time=146.84
estimated_change_time=39.38
estimated_return_time=13.18
```

随后创建 `TakeElevator(0->1)`。本轮没有再次出现此前完整探索测试中的 `GO_TO_ELEVATOR_DOOR` 无路径零速活锁；厅外路径有效且机器人持续向电梯移动。

## 6. 电梯子状态机逐阶段结果

下表时间为任务诊断中的 `elapsed`，阶段边界按 0.5 秒诊断采样近似：

| 阶段 | 首次观测时间 | 关键结果 |
| --- | ---: | --- |
| `GO_TO_ELEVATOR_DOOR` | 33.460s | 从约 `(-1.88, 2.48)` 沿有效路径到达厅外 `(0.93, 2.36)` |
| `CALL_ELEVATOR_TO_CURRENT_FLOOR` | 39.760s | 当前层呼梯成功，机器人保持零速 |
| `OPEN_CURRENT_FLOOR_DOOR` | 40.360s | 门服务完成后进入进厢阶段 |
| `ENTER_CABIN / ALIGN` | 43.964s | 在厅外完成朝向对齐 |
| `ENTER_CABIN / CROSS` | 44.460s | 有效前向穿门，约在 48.560s 到达轿厢内 `(2.80, 2.28)` |
| `CLOSE_CURRENT_FLOOR_DOOR` | 48.560s | 在轿厢内停稳并关门 |
| `CALL_ELEVATOR_TO_TARGET_FLOOR` | 52.060s | 调用目标楼层 1 |
| `WAIT_FOR_ODOM_STABLE` | 52.560s | 换层期间阻断楼层相关地图更新 |
| `OPEN_TARGET_FLOOR_DOOR` | 54.560s | 目标层门打开 |
| `EXIT_CABIN / ALIGN` | 58.560s | `current_floor` 已确认切换为 1 |
| `EXIT_CABIN / CROSS` | 59.060s | 反向穿门离开轿厢，到达约 `(1.01, 2.38)` |
| `CLOSE_TARGET_FLOOR_DOOR` | 63.460s | 出厢后关门 |
| `DONE → EXPLORE_FLOOR` | 66.760s | `current_floor=1`，换层闭环成功 |

电梯任务从状态切换到完成约耗时 `33.600s`。全过程未记录换层任务失败、无路径持续等待、控制器 STUCK 或安全门终止。

## 7. 楼层对齐与地图隔离

显式楼层过渡保护在调用目标层时进入模糊期，并在目标层传感器与新鲜里程计共同确认后退出：

```text
enter_phase=CALL_ELEVATOR_TO_TARGET_FLOOR
duration=6.298
source_floor=0
target_floor=1
mapper_estimated_floor=1
alignment_odom_samples=2
skipped_cloud_count=62
exit_reason=service_sensor_transition_and_fresh_odom_confirmed
```

这证明换层期间的 62 帧点云没有误写到楼层 0/1 地图，且不是仅凭服务返回立即切换楼层。

## 8. 验证命令与结果

```bash
bash -n tools/sim/run_elevator_gui_test.sh
git diff --check
python3 tools/offline/check_production_boundary.py
sha256sum -c OFFICIAL_FILES.sha256
tools/sim/run_elevator_gui_test.sh
```

结果：

- 电梯专项脚本：通过，返回码 0
- Shell 语法检查：通过
- `git diff --check`：通过
- 生产/离线边界审计：通过
- 仿真退出清理：通过，11311 端口空闲，无 `gzserver`、`gzclient`、`junior_ctrl`、任务或定位残留进程
- 相关 Python 测试沿用本检查点结果：`428 passed`
- 官方文件哈希仍只有 4 个历史预期差异：`auto.sh`、`FSM.h`、`FSM.cpp`、用户修改的 `gazebo.xacro`；本次没有新增官方文件差异

## 9. 合规性结论

本次生产链没有引入布局、危险源真值、Gazebo model state、裁判里程计或真值里程计依赖。专项脚本只修改临时测试预算，并显式关闭环境真值发布与真值点云转换。`check_production_boundary.py` 审计通过。

本轮成功仿真结束前没有单独保存 `rosnode info` 快照，这是证据采集上的不足；同一代码检查点此前运行时订阅审计未发现 `/ground_truth/*`、`/Odometry_gazebo` 或裁判里程计订阅。后续专项脚本应在成功退出前自动保存节点订阅快照，避免依赖人工抽查。

## 10. 未覆盖范围与残余风险

- 本报告验证的是完整的 0→1 单次乘梯闭环，不包含 1→0 返航乘梯。
- 未在本专项中验证主入口返航近场重观测和最终离开建筑；它们属于完整两层任务验收。
- GUI 运行实时率较低，仿真约 67 秒耗费数分钟墙钟时间，但状态机使用仿真时间，未出现因墙钟慢导致的逻辑超时。
- 电梯视觉注册此前出现过一轮全向覆盖超时，说明注册仍有运行间波动；本轮在 8.898 秒内成功，但后续完整两层回归仍需保留 fail-closed 监控。

## 11. Git 状态与阶段判断

本次新增/修改仅限新测试工具及本报告，没有提交、暂存、标签或推送操作。工作区中原有暂存/未暂存改动均保持不变。

电梯 0→1 动作链可以判定通过；第 11 层整体仍不能封口，下一步应运行完整两层任务，重点验证二层探索、1→0 返航乘梯、主入口安全退出和最终结果写入。

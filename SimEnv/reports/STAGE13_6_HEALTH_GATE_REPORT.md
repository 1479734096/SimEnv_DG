# STAGE13.6 定位健康门误判修复与两层闭环报告

## 1. 本层目标与边界

本层只处理 STAGE13.5 已由历史证据定位的单一问题：机器人 IMU 明确直立且运动稳定时，
局部地面 RANSAC 偶发拟合到斜面，`ground_normal_z` 与同一平面派生的
`base_height` 不应单独把定位健康状态判为跌倒。

本层没有修改定位融合、GICP、scan-to-map、任务预算、电梯参数、传感器模型或赛方文件，
也没有调整任何健康阈值。Gazebo 真值仅由独立诊断录制，并在任务结束后离线评估，没有
进入生产决策。

## 2. 最小实现

修改：

- `src/danger_search_localization/include/danger_search_localization/stance_health_monitor.hpp`
- `src/danger_search_localization/test/test_stance_health_monitor.cpp`

健康证据顺序调整为：

1. IMU `base_up_z` 明确低于阈值时仍直接累计 `BAD`；
2. 只有地面法向达到既有可靠阈值时，才用同一平面派生的机身高度判定 `BAD`；
3. 地面平面缺失，或地面法向与直立 IMU 明显不一致时，不把该平面的法向和高度当作
   跌倒证据；
4. 角速度过大仍是有界 `UNKNOWN`，部分字段缺失仍是 `UNKNOWN`；
5. 前方路径和当前足印支撑仍由独立 `GroundSupportSafetyGate` fail-closed，本层没有
   放宽该安全门。

这保持了 STAGE13.5 预注册的单一机制，没有新增参数，也没有针对 seed 或固定坐标分支。

## 3. 定向测试与构建

新增三组回归：

- 历史误判值 `height=0.483, ground_nz=0.797, base_up_z=1.0,
  angular=0.407` 连续三帧不再累计 `BAD`；
- 相同地面拟合下，`base_up_z=0.60` 仍在第三帧判不健康；
- 可靠水平地面下，`base_height=0.058` 仍在第三帧判不健康。

执行：

```bash
catkin_make --pkg danger_search_localization -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make run_tests_danger_search_localization -j2
catkin_test_results --verbose build/test_results/danger_search_localization
```

结果为 `106 tests, 0 errors, 0 failures`。其中姿态健康监控共 14 项测试，既有 ROS
受控倾倒测试 `health_and_ready_revoke_after_fall` 也继续通过。

## 4. 同 seed 两层定向 Gazebo

只运行一次与 STAGE13.3 失败轮相同的 seed：

```bash
SEED=6946 GUI=true \
ELEVATOR_TEST_FLOOR_TIMEOUT=33 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5 \
ELEVATOR_TEST_MAX_RUNTIME=180 \
ELEVATOR_TEST_FULL_CYCLE=1 \
ELEVATOR_TEST_RECORD_DRIFT=1 \
ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 \
ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1 \
ELEVATOR_TEST_LOG_DIR=/tmp/stage13_6_health_seed6946 \
tools/sim/run_elevator_gui_test.sh
```

结果：

- 控制器 ready、主入口通过、电梯视觉注册均成功；
- 完成 `0→1`、一层探索、返程对门、`1→0`、主入口退出和 home；
- 正式终态 `success`，reason 为 `return_home_completed`；
- 总任务耗时 `165.06 s`；
- STUCK `0`，恢复 `0`；
- 原 STAGE13.3 在约 125 秒出现的 `localization_stance_unhealthy` 未复现，机器人继续
  进入返程电梯并完成闭环。

离线计时账本：

- 状态耗时：换层 `93.7 s`、探索 `53.8 s`、返航 `15.2 s`、等待控制器
  `1.498 s`；
- 有效运动 `99.0 s`，占 active 时间 `60.85%`；
- 原始里程 `33.445 m`，排除换层跳变后的里程 `33.431 m`；
- 平均命令速度 `0.238 m/s`，峰值命令速度 `0.600 m/s`；
- 楼层 0 的 bbox fill ratio 为初始 `0.461`、最大 `0.461`、最终 `0.326`；
- 楼层 1 为初始 `0.413`、最大 `0.467`、最终 `0.321`。

上述覆盖字段是当前地图已知包围盒填充率，会随包围盒扩张下降，不能解释成官方累计
覆盖得分。本轮短探索预算下正式识别数为 `0`，且没有同轮评分 sidecar，因此本轮只验证
健康门和换层返航，不据此评价识别或评分收益。

## 5. 定位误差与安全事件

任务结束后离线真值结果：

- 真值路程 `33.182 m`，同步样本 `1652`；
- 平移 RMSE `0.0520 m`，P95 `0.1045 m`；
- 最大及终点平移误差均为 `0.2125 m`；
- 最大绝对 yaw 误差 `0.00287 rad`，终点 yaw 误差 `0.00034 rad`。

这证明本轮闭环期间定位可用，但不能证明平移漂移机制已经改善；终点误差仍高于部分历史
短路线结果，定位短窗分歧仍按原计划单独做跨 seed、跨路线诊断。

安全账本记录 13 次门状态迁移和 1 次路径重规划：

- 94.46–96.86 秒间为 `OBSERVATION_HOLD/STALE_DATA_HOLD` 与 `CLEAR` 往返，
  自动恢复 5 次；
- 累计 hold `3.4 s`，没有速度包络违规；
- `true_drop_stop_count=0`，没有 `DROP_CONFIRMED`；
- 97.26 秒再次进入 `STALE_DATA_HOLD`，99.16 秒短暂进入终止态
  `SUPPORT_PATH_UNAVAILABLE`，随即触发一次 `ground_support_path_replan`；
- 当时当前足印仍受支撑、没有确认落差，之后任务正常返航成功。

因此本层只能认定“定位健康误判修复通过一次定向闭环”，不能认定生产防坠闭环已经完成。
上述短暂停顿与无落差情况下的路径不可用是下一层必须审计的独立问题。

证据目录：`/tmp/stage13_6_health_seed6946`。

## 6. 合规检查

```bash
python3 tools/offline/check_production_boundary.py
git diff --check
sha256sum -c OFFICIAL_FILES.sha256
```

结果：

- 生产/离线边界审计通过；
- `git diff --check` 通过；
- 官方哈希仍恰有 4 项历史差异：`auto.sh`、`FSM.h`、`FSM.cpp`、
  `gazebo.xacro`，本层无新增；
- 测试结束后 ROS master、Gazebo、控制器和任务进程均为空；
- 没有提交、tag 或 push。

## 7. 下一层入口

下一层先只读审计 `GroundSupportSafetyGate` 的观测时序、速度来源、路径失效与重规划
闭环，并回答：

1. 正常楼面为何连续产生 `STALE_DATA_HOLD`，是否由控制时钟与传感器 stamp 的语义混用
   导致；
2. 没有 `drop_confirmed` 时为什么能进入终止态 `SUPPORT_PATH_UNAVAILABLE`；
3. 真实已知空带能否稳定形成
   `DROP_CONFIRMED → 零速 → 路径失效 → 有界重规划或失败`；
4. 电梯门槛和门边是否会形成假终止。

在上述证据闭环前不调安全阈值，不通过关闭未知支撑 hold 或放宽停止距离换取成功。

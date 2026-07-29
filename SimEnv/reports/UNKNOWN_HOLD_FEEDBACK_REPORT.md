# UNKNOWN HOLD 语义反馈环修复报告

## 1. 结论

`GroundSupportSafetyGate` 的释放证据已从“当前已提交零速命令”改为绑定触发
HOLD 的完整候选命令。固定 `seed=6946` 的双层 GUI 闭环中，旧轮 13 次短时
HOLD/自动恢复振荡收敛为 1 个真实 HOLD episode：静止补观测未满足原命令扫掠，
因此没有自动释放，而是 2 仿真秒后显式进入 `SUPPORT_PATH_UNAVAILABLE`，经一次
有界重规划后完成 `0→1→0→home`。

本轮没有增加 probe、没有调整 YAML、没有改变 DROP 命令源，也没有新增话题、服务、
依赖或 `/cmd_vel` publisher。

## 2. 根因

旧实现的 RequiredSupportSweep 使用 committed command。Gate 输出安全零速后，
keepalive 持续提交零命令，支撑扫掠从非零候选对应的最长 `0.80 m` 缩短为静止
`0.50 m`。Gate 却仍按原非零命令计算释放距离，导致扫描末端之外的边界暂时消失，
零命令评估可错误释放原命令，随后边界重新出现并再次 HOLD。

评估对象原先不携带 HOLD 上下文和实际扫掠命令，Gate 无法拒绝这种异源释放证据。

## 3. 实现

- `GroundSupportAssessment` 增加 HOLD 上下文 ID、完整支撑扫掠命令、实际前视距离、
  边界可观测性和原因计数。
- Gate 在进入临时 HOLD 时冻结完整候选命令并创建上下文 ID；零速、keepalive 和
  安全停车不改写该上下文。
- HOLD 期间 RequiredSupportSweep 始终使用冻结命令。只有上下文 ID、完整命令和
  前视元数据匹配的评估才能累计连续支撑并参与释放。
- 定位健康状态真实切换或显式 assessment invalidation 会更新 HOLD 上下文，旧评估
  不能跨健康状态释放。
- DROP 仍使用 `active_motion_command`，不检查 HOLD 上下文 ID；上下文不匹配的
  新鲜 DROP 仍可进入既有停车与锁存链。
- 导出 `boundary_observable` 和 `boundary_reason_counts` 仅用于诊断，不参与新增
  运动许可或参数放宽。

## 4. 自动验证

```text
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -q \
  src/danger_search/test/test_ground_support_gate.py \
  src/danger_search/test/test_mission_manager_helpers.py \
  src/danger_search/test/test_support_shadow_online_map.py -x
219 passed

PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -q src/danger_search/test
587 passed, 1 warning

python3 tools/offline/check_production_boundary.py
参赛 Python/离线边界审计通过。

python3 -m py_compile <本轮三个生产 Python 文件>
bash -n tools/sim/run_elevator_gui_test.sh
git diff --check
全部通过。
```

系统 Cryptodome 的 `imp` 弃用 warning 为既有环境警告。官方文件哈希仍只有总账中
记录的 5 个既有差异：`auto.sh`、`docs/quick-start.md`、`FSM.h`、`FSM.cpp` 和
`gazebo.xacro`；本轮没有新增受保护文件差异。

新增回归覆盖：零命令评估不能释放非零 HOLD、零 keepalive 不改变 HOLD 上下文、
assessment invalidation 后旧上下文不能释放、MissionManager 在 HOLD 时使用冻结命令
而 DROP 继续使用 active motion，以及边界可观测性遥测导出。

## 5. Gazebo 定向验证

两轮均使用 `seed=6946`、`GUI=true`、两层、楼层预算 `33 s`、最少探索 `5 s`、
总仿真预算 `180 s`、`adaptive_shared_map`。truth、裁判里程计、离线 truth 和诊断
录包均关闭。

| 证据目录 | 结果 | 说明 |
|---|---|---|
| `/tmp/unknown_hold_context_seed6946_round1` | 失败 | 主入口门槛刚体接触卡滞；Gate 全程 CLEAR，UNKNOWN hold 关闭，非零命令持续但 odom 无位移，仿真 `23.426 s` 以 `main_entrance_crossing_failed` 结束。该轮未到达目标场景。 |
| `/tmp/unknown_hold_context_seed6946_round2` | 通过 | 主入口正常通过，完成 `0→1→0→home`，仿真 `170.518 s` 写出成功结果。 |

第一轮近场重测门中心 `(-0.01,-0.09)`，卡住位姿横向仅偏约 `3.2 cm`；第二轮同一
代码和 seed 正常通过，说明它是非确定性刚体接触，不是 Gate 停车或门中心目标错误。

第二轮唯一 HOLD episode 的关键证据：

- `107.312 s`：`CLEAR→OBSERVATION_HOLD`，`context_id=1`；
- 全 episode 的 `support_command` 固定为
  `(0.54398,-0.00030,-0.05225)`，`support_lookahead_distance=0.80 m`；
- 边界原因主要为 `explicit_current_floor_returns` 和
  `visible_without_explicit_support_or_drop_return`，静止补观测没有满足释放条件；
- `109.212 s`：进入 `SUPPORT_PATH_UNAVAILABLE`，总 HOLD `2.0 s`；
- `automatic_resume_count=0`，随后只触发一次 `ground_support_path_replan`；
- 最终 `observation_hold_count=1`、`true_drop_stop_count=0`、任务成功。

因此本轮既消除了零扫掠假释放，也没有造成不可恢复死锁；证据不足时通过既有有界
重规划改变路径，而不是棘轮式推进未验证区域。

## 6. 残余风险

- 只验证了固定 seed 的一次有效目标 episode，尚不能代表所有边界几何；但架构不变量
  已由单测覆盖。
- 主入口门槛存在非确定性刚体接触卡滞，属于独立可用性风险。本轮不把门恢复或速度
  调整混入安全语义修复。
- 当前 `SUPPORT_PATH_UNAVAILABLE` 会按既有任务线程路径写动态 NEGATIVE 并有界
  重规划；本轮没有改变 BLACK 语义。
- 未增加 bounded probe。只有未来出现“有界重规划耗尽且静止不可观测”的确定性失败，
  才应基于本报告的可观测性原因另行设计，不得恢复零命令假释放。

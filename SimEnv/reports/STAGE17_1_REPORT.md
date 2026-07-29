# Stage 17.1 可用性回退与双时钟复核报告

## 1. 结论

本轮目标已收口为“恢复可用的两层闭环”，不再进入 Stage 17.2 及后续
迁移。固定 `seed=6946` 最终完成电梯视觉注册、`0->1`、二层探索、
`1->0`、主入口退出和 home 返航，runner 返回 0。

实现不是整体关闭防坠：明确 DROP、消息新鲜度、定位健康、统一
`/cmd_vel` 事务、发布失败锁存和刹车期 active motion 全部保留。回退的是
`2092803` 中未闭环的一次性门槛许可和命令代次竞态，恢复历史已验证的
任务上下文 UNKNOWN 语义。

## 2. 根因与修复

### 2.1 低 RTF 下的评估饥饿

`PathFollower` 原实现每次成功 publish 都增加命令代次，包括零速 keepalive。
MissionManager 在点云评估前后要求代次完全相等，低 RTF 时几乎每个评估
窗口都被 keepalive 打断，且无日志丢弃。

已删除这个跨回调代次判定。点云计算后在统一命令事务锁内提交评估，
并对当时最新 active motion 重新检查 DROP。单测覆盖点云计算期间命令
变化，评估仍必须提交，DROP 必须按最新运动检查。

### 2.2 未闭环的受控门槛许可

一次性许可依赖旧 active command 进行签发和消费，旋转命令可以误消费许可，
8 秒时限还会在低 RTF 的电梯进厢过程中到期。这与已验证的主入口、电梯
任务状态机重复，并没有形成更可靠的物理闭环。

已移除 `ControlledSupportPermit` 及速度特例，恢复原有语义：传感器确认的主入口
`GO_PASS_POINT`、电梯 `ENTER_CABIN/EXIT_CABIN` 及有界门后上下文只局部关闭
UNKNOWN hold；明确 DROP、OCC/BLACK、定位和输出故障不受影响。

### 2.3 DROP 与 RequiredSupportSweep 的命令语义

Stage 17.1 曾把停稳前保留的 active motion 同时用于 DROP 和 RequiredSupportSweep。
现已拆成两个显式输入：

- DropSweep 使用 `active_motion_command`，已发零速后仍保持到三帧新鲜 odom
  确认停稳；
- RequiredSupportSweep 使用最近一条 publisher 正常返回的 committed command，
  Gate 已输出零速后可按静止/重规划上下文重新获取支撑。

这个拆分避免用零速缩短 DROP 刹车扫掠，也避免把刹车前的高速长前视
永久施加给支撑恢复评估。

## 3. 双时钟裁决

不全局换成任意一种时钟，固定以下边界：

- ROS `/clock` / 仿真时间：消息 header stamp、点云/定位新鲜度、时间同步、
  证据 TTL、连续 odom 停稳样本、状态超时、楼层和任务预算；
- `time.monotonic()` / 单调墙钟：`/clock` 或 odom 不再推进、进程/回调活性、
  publish 间隔诊断和外部测试总超时；
- `time.time()` 不用于 elapsed/deadline，避免 NTP 或系统时钟跳变。

`get_time()` 在 ROS 节点初始化后必须返回 ROS time，读取失败显式报错；只有无 ROS
的离线路径回退到 `time.monotonic()`。Gate 的仿真停车上限保持 3 仿真秒，
原先复用同一数值的墙钟冻结 watchdog 已拆为独立 `wall_stall_timeout=30 s`。
Gate 在非 CLEAR 时已持续输出零速，这 30 秒只决定何时显式结束故障进程。

定位 ROS 集成测试的连接等待也从 ROS time/`rospy.sleep` 改为单调墙钟，
使 `/clock` 停止时测试仍能有界退出；合成消息 stamp 仍是 ROS time。

## 4. 验证

### 4.1 自动测试与静态审计

```text
python3 -m pytest -q src/danger_search/test
584 passed, 1 warning

python3 tools/offline/check_production_boundary.py
参赛 Python/离线边界审计通过。

git diff --check
bash -n tools/sim/run_elevator_gui_test.sh
python3 -m py_compile src/danger_search_localization/test/test_stance_health_ros.py
全部通过。

rostest danger_search_localization stance_health_ros.test
1 test, 0 errors, 0 failures
```

warning 为系统 Cryptodome 使用 `imp` 的既有弃用警告。生产源码静态搜索只有
`PathFollower.start()` 一个 `/cmd_vel` publisher 构造点；禁用 truth/metadata 字符只出现在
`interface_policy.py` 拒绝列表，不是生产读取。

官方哈希审计仍有历史既有的 5 项差异：`auto.sh`、`docs/quick-start.md`、
`FSM.h`、`FSM.cpp`、`gazebo.xacro`。本轮未修改这些受保护文件。

### 4.2 Gazebo

三轮均使用 `seed=6946`、两层、`GUI=false`、楼层预算 `33 s`、最少探索
`5 s`、`adaptive_shared_map`，truth、裁判里程计、离线 truth 和原始录包均关闭。

| 证据目录 | 结果 | 关键事实 |
|---|---|---|
| `/tmp/usable_rollback_seed6946_run1` | 测试基础设施超时 | 墙钟 900 s 时仿真只到 `139.914 s`；已完成 `0->1` 并创建 `1->0`，不记为算法失败。 |
| `/tmp/usable_rollback_seed6946_run2` | 算法失败 | 到达返程电梯门外后，刷新候梯点 A* 连续三次 `unreachable`，仿真 `150.918 s` 显式失败。 |
| `/tmp/usable_rollback_seed6946_run3` | **通过** | `0->1` 在 `71.814 s` 确认，`1->0` 在 `142.214 s` 确认，`162.276 s` 写出成功结果并完成 home。 |

最终轮返程初始与视觉刷新后的 A* 都一次 `reached`。Gate 有 13 次短暂
UNKNOWN hold，13 次均由后续新鲜支撑自动恢复；最终无 DROP、无控制输出失败、
无时钟/odom 冻结、无定位终态。runner 已清理 ROS、Gazebo、控制器和任务进程。

## 5. 残余风险与边界

- 本轮证明同 seed 可用闭环恢复，不等于新防坠逻辑已完成自然 DROP 验收。
- 最终轮仍有 13 次可恢复 hold，高于历史成功轮的 3 次；它们未破坏本轮任务，
  但应作为效率风险保留，不通过放宽 DROP 或删除新鲜度检查消除。
- 本轮不创建 Stage 17.1 完成标签，不开始 Stage 17.2；当前结论仅为可用性
  回退与两层闭环通过。

## 6. 工作树边界

- 用户已有 `.codex/config.toml` 和 `fix.md` 改动保留，未覆盖。
- 本轮修改限于参赛算法、配置、测试、包说明和本报告。
- 未新增依赖、生产输入、服务或 `/cmd_vel` publisher；未修改赛方 world、模型、
  传感器、服务、生成器或原始启动流程。

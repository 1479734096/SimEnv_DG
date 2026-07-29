# 第 8 层报告：正式 launch、runner 与结果闭包

## 阶段结论

第 8 层生产代码闭包已完成：正式配置、正式/诊断 launch、ROS 节点入口、定位 ready
前门、公开结果路径约束、唯一 `/cmd_vel` 出口和失败写出均已接通。正式节点只有在
`/danger_search/localization/ready=true` 后才创建 `MissionManager`，等待期间不开始任务
预算，也不建立控制发布器。

本阶段不宣称完整动力学验收通过。固定 seed Gazebo 联调确认生产链能进入
`EXPLORE_FLOOR` 并生成路径，但在任务已经发布非零速度后再切换底盘到 RL 模式，
机器人快速跌倒。针对跌倒后定位仍保持 ready、任务链没有立即失效安全的问题，
已实现 `LOCALIZATION-002` 代码修复：定位节点持续发布姿态 health，MissionManager
在 health=false 时立即停车并进入失败写出。第 8.1 层审计后又补齐真实 `/scan` 楼面
证据、非法证据 fail-closed、失败优先级和 ROS 合成站立→倾倒闭环；Gazebo 跌倒仍待
复测。详见 `reports/STAGE8_1_REPORT.md`。`CONTROL-004` 仍待隔离命令交接次序、速度
阶跃与 RL 模式自身稳定性。

官方 `auto.sh`、launch、URDF、Unitree 控制器和传感器插件仍未修改。生产代码没有
调用 Gazebo 模型配置服务。

## 迁移与整理范围

- `config/danger_search.yaml`：形成正式唯一配置面；formal 强制地面支撑门开启，线速度
  上限 `0.60 m/s` 不超过停车包络声明的 `0.651 m/s`。
- `launch/danger_search.launch`：组合正式 RGB-D/Livox 定位与任务节点，两个关键进程均
  `required=true`，公开场景路径显式传入定位。
- `launch/danger_search_smoke.launch`：短超时 diagnostic 闭包，不要求 Gazebo 站立就绪。
- `scripts/danger_search_node`：仅保留生产入口；离线工具不进入本阶段安装清单。
- `mission_manager.py`：新增定位 ready 等待门，formal 拒绝关闭生产地面支撑门。
- `sensor_buffer.py`：区分公开传感器输入和参赛方内部配准点云输入。
- `CMakeLists.txt`：只安装正式节点、配置和 launch。
- `README.md`、团队手册和接口文档测试：同步正式启动、结果路径与当前运行边界。

## 本阶段确认并修复的缺陷

### STARTUP-001：定位建立前任务预算已开始

旧入口在 `rospy.init_node()` 后立即构造并启动 `MissionManager`。定位若等待机器人站立或
世界系初始化，任务预算会被无效消耗，控制发布器也会提前存在。

现使用持久订阅等待锁存的内部 ready。收到 true 后注销临时订阅，再创建任务编排器；
ROS shutdown 会中断等待。单测覆盖 false→true 锁存、订阅注销和内部话题约束。

### CONFIG-001：正式安全配置没有强制闭合

正式配置现显式开启 `production_gate_enabled`，同步定位速度死区与停稳阈值，并将默认
控制上限收在已声明停车包络内。除静态配置测试外，formal 构造期还会拒绝关闭安全门，
避免只改 YAML 即绕过生产安全边界。

### RUNTIME-001：内部配准点云被误判为非法外部输入

首次 ROS smoke 中，`SensorBuffer` 对
`/danger_search/localization/cloud_registered` 仍执行外部公开话题白名单校验，任务节点
启动失败。现只对 `/danger_search/*`、`/nav/*` 派生点云使用内部话题校验；其他点云继续
必须通过公开输入白名单。新增回归测试覆盖内部成功和 `/cmd_vel` 等非法输入拒绝。

### LOCALIZATION-002：跌倒后定位仍保持 ready 且任务继续

问题根因是世界系建立后缺少持续姿态健康检查：Gazebo 中机器人跌倒时，`/trunk_imu`
与楼面法向已明显偏离站立范围，但定位仍按固定高度和零平面速度发布位姿并维持
`ready=true`，MissionManager 与 GroundSupportGate 继续恢复与转向命令。

修复方案完全基于参赛方已允许的公开输入，不读取任何 Gazebo 真值：

1. `danger_search_localization` 新增 `StanceHealthMonitor`，输入仅使用
   `StanceObservation` 中的 base 高度、楼面法向 `z`、机身竖直分量 `z` 和角速度。
   连续多帧正常才判为健康，连续多帧异常判为不健康，默认启用不健康锁存，避免
   跌倒后短暂抖动触发重锚定导致世界系跳变。
2. `scan_localization_node` 初始化成功后持续更新 health，不健康时撤销
   `/danger_search/localization/ready`，并发布 `/danger_search/localization/health=false`。
3. `mission_manager.py` 订阅 health，在 `tick()` 中 health=false 时立即停车、
   取消成功资格并进入 `WRITE_RESULT`，失败原因码为 `localization_stance_unhealthy`。
4. 诊断模式 `danger_search_smoke.launch` 可关闭 `localization/stance_health_required`，
   保持旧版本定位节点或不需姿态验证的 ROS smoke 兼容。

新增 C++ 单元测试覆盖健康/不健康计数、锁存、非锁存恢复、非法配置和非有限观测；
新增 Python 单元测试覆盖 MissionManager 在 health=true/false/unknown 三种情况下的
推进与失败行为。第 8.1 层进一步增加真实 ROS 节点合成闭环，避免只验证孤立谓词。

## ROS smoke 结果闭包

独立 ROS master、不启动 Gazebo，使用 diagnostic profile 和约 1 秒任务预算：

- 首轮暴露并修复 `RUNTIME-001`；
- 修复后定位与任务节点正常启动，任务超时进入失败终态并干净退出；
- 官方结果 schema 写出 `exploration_time` 与空的 `detected_danger_sources`；
- 诊断摘要记录 `terminal_reason=mission_timeout`、`terminal_state=FAILED`、
  `timeout=true`；
- 控制器共记录 19 条命令，全部为零速；
- launch 因 required 任务节点退出而联动停止定位，没有残留进程。

## Gazebo 三轮验证

共同条件：单层、`SEED=42`、`GUI=false`、独立 ROS/Gazebo master、独立 `ROS_HOME`，
关闭真值、裁判里程计、点云真值里程计和生成真值副本。退出后只清理本轮进程。

### 第 1 轮：官方暂停启动链

使用未修改的 `auto.sh`，`PAUSED=true`、`AUTO_UNPAUSE=1`。四个 calf 再次进入约
`+3.5866 rad` 分支，力矩均约 `-33.5 Nm`，机身高度约 `0.12 m`，无法站立。
`ENV-002` 稳定复现，正式任务没有启动。

### 第 2 轮：只用于隔离的站立夹具

使用 `AUTO_UNPAUSE=0`，等待 `spawn_model` 延迟的
`set_model_configuration` 完成后，仅由测试命令将四个 calf 临时设为 `-2.0 rad`，再恢复
物理。按 `2` 后 calf 收敛到 `-1.831～-1.836 rad`，`/scan` 为 10 Hz，站立成功。

随后第一次 formal 启动传入 `/tmp` 结果路径，被公开场景路径校验正确拒绝。定位在
ready 后才启动任务入口，拒绝发生在建立任务控制之前。该失败属于错误测试参数，反而
验证了 formal 结果路径不可任意覆盖。

### 第 3 轮：正式路径与动态交接

改用 `team_scene_info.json` 公布的结果路径：

- 定位在仿真时间约 `41.256 s` ready，随后才开始任务计时；
- MissionManager 进入 `INIT → WAIT_CONTROLLER_READY → EXPLORE_FLOOR`；
- 点云/RGB-D 时间匹配、建图、入口自由走廊和两点路径均正常产生；
- `/cmd_vel` 只有 `/danger_search` 一个发布者，订阅者为底盘和定位；
- 初始入口跟踪请求约 `linear.x=0.598 m/s`，任务启动时底盘仍处于 FixedStand；
- 之后按 `6` 切换到 RL `/cmd_vel` 模式，采样命令曾为
  `linear.x=0.03 m/s`、`angular.z=-1.2 rad/s`，机器人很快失稳；
- Gazebo 机身 z 从站立约 `0.258 m` 降至 `0.058 m`，junior_ctrl 最终报告跌倒并切回
  passive/down；
- 跌倒后参赛定位仍报告约 `z=0.207 m`，ready 不撤销；安全门看到的里程计平面速度为
  0，MissionManager 继续 stuck 恢复和转向；
- 外部停止时结果按公开路径写出，终态明确标记为失败，没有静默退出。

当前事实只能证明“非零命令已存在后切换 RL”与跌倒同时发生，不能直接断言根因就是
`0.60 m/s` 上限。需要分别隔离 RL 零命令切换、命令阶跃、模式切换次序和持续姿态健康。
本阶段不继续试参。

## 给 Kimi 的直接提示词

```text
请审阅 codeDG_rebuild/reports/STAGE8_REPORT.md 和当前代码树，重点协助隔离
CONTROL-004 并验证 LOCALIZATION-002 的 Gazebo 跌倒闭环效果。

环境与已知事实：
1. 官方 auto.sh 未修改；PAUSED=true/AUTO_UNPAUSE=1 会复现 calf +3.5866 rad、
   -33.5 Nm 的 ENV-002。诊断夹具用 AUTO_UNPAUSE=0，等待 spawn_model 延迟配置后把
   四个 calf 设为 -2.0，再 unpause，可稳定 FixedStand，calf 约 -1.83 rad。
2. 正式 danger_search.launch 会等 localization ready 后才创建 MissionManager。正式链已
   进入 EXPLORE_FLOOR，/cmd_vel 唯一发布者是 /danger_search。
3. 本次顺序是：先正式任务运行并已发布约 0.598 m/s 的入口命令，底盘仍 FixedStand；
   后按 6 切到 RL /cmd_vel。切换后机器人快速跌倒，trunk/model z 从约 0.258 降到
   0.058 m，junior_ctrl 报 fall safety。
4. LOCALIZATION-002 已完成代码、单元测试和 ROS 合成闭环修复；Gazebo 跌倒闭环仍待
   复测。第 8.1 层发现并修正的初版问题见 `reports/STAGE8_1_REPORT.md`。

请先回答并设计不超过 3 轮的最小验证：
A. 区分“RL 模式本身不稳”“进入 RL 时已有缓存非零命令”“0→0.6 速度阶跃”和
   “角速度/恢复命令”四种原因。优先建议零 /cmd_vel 下先切 RL、稳定后再启动任务，
   以及在切换前后记录 /cmd_vel、12 个关节 state/effort、trunk_imu 和 FSM 状态。
B. 检查 RL 入口是否读取切换前缓存命令、是否有命令限幅/归一化/超时，以及
   FixedStand→RL 的首个推理周期是否存在观测或动作突变。
C. 在 Gazebo 中使用诊断夹具稳定站立后启动任务，复现跌倒并确认：
   `/danger_search/localization/health` 是否变为 false，`ready` 是否撤销，
   MissionManager 是否进入 `WRITE_RESULT` 并写出失败，终态 reason_code 是否为
   `localization_stance_unhealthy`。禁用真值 topic 不得被参赛节点订阅。
D. 判断 ENV-002 是否应由参赛 wrapper 等待 spawn_model 的延迟配置后再写 calf=-2.0，
   还是必须由官方环境修复；不要建议修改受 OFFICIAL_FILES.sha256 保护的文件，除非先明确
   说明这是需要用户批准的规则边界变更。

可参考临时 ROS 日志：
/tmp/codeDG_rebuild_stage8_fixture_ros_home/log/7aff35dc-843d-11f1-9010-dd08b796070a/
原始 Gazebo 日志：codeDG_rebuild/logs/competition_gazebo.log
```

## 自动化验证

| 验证项 | 结果 |
|---|---|
| `danger_search` Python 全测试 | 398 项通过 |
| 定位 C++ 测试 | 10 个测试程序、38 个用例通过（含 `test_stance_health_monitor`） |
| 定位 ROS 合成闭环 | 1 项通过：站立 health/ready=true，倾倒后两者撤销 |
| Catkin 生产构建 | `catkin_make -j2` 成功 |
| ROS smoke | 超时失败写出成功；19 条控制命令全部为零 |
| 正式 Gazebo 接线 | ready 后启动；唯一 `/cmd_vel` 发布者；进入探索 |
| 完整任务动力学 | 未通过：切换 RL 后跌倒，按 3 轮上限停止 |
| 姿态健康代码 | 已通过单测与 ROS 合成闭环；第 8.2 层后续完成 Gazebo 侧倾闭环 |
| 官方文件完整性 | 1109 个 SHA-256 全部一致 |
| 补丁格式 | `git diff --check` 通过 |

## 下一层边界

- 第 9 层只隔离离线工具，不得改变生产控制、定位或官方环境。
- 第 10 层动力学验收前必须关闭或获得明确处置：`ENV-002`、`CONTROL-004`。
  `LOCALIZATION-002` 已由第 8.2 层 Gazebo 侧倾闭环关闭；第 10 层只回归正常运行不误报。
- `COORD-001` 的跨层绝对 z、电梯服务与真实视觉门洞闭环仍待多层验收。
- 未经用户明确批准，不修改任何 `OFFICIAL_FILES.sha256` 保护文件。

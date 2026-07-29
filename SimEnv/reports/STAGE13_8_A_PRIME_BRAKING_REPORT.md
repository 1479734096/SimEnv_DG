# STAGE13.8 A′ 速度职责拆分与制动测量入口报告

## 1. 阶段结论

本层完成了 A′ 的生产语义拆分、离线制动分析器和合成 DROP 闭环测试，但两轮制动
测量尚未封板。

已完成：

- 请求速度成为停车表和支撑 guard 的唯一速度索引；
- 定位差分速度只用于三帧停稳确认和独立的有界异常观察；
- 请求速度超过停车表有效域仍显式失败；
- 定位速度单次超过 `1.0 m/s` 时立即零速，恢复正常新样本后自动恢复；
- 定位速度持续异常满 `2.0 s` 时以 `localization_speed_abnormal` 显式失败；
- formal 地面支撑安全门、UNKNOWN fail-closed、DROP 锁存、BLACK 局部禁行和有界重规划
  均保持开启；
- 当前 `controller.max_linear` 冻结为 `0.60 m/s`，停车表明确标注为“工程保守包络，
  待两轮独立测量封板”。

未完成：

- 预注册制动矩阵要求每轮同时覆盖足够的 `0.30/0.60 m/s`、直行和弧线事件。普通任务
  路线在两个独立 seed 的证据中均不能自然生成足够的接近 `0.60 m/s` 停车事件；
- 因此不更新停车表，不宣称 `0.651 m/s` 已验证，不提速，也不宣称生产防坠已经封板。

## 2. A′ 最小实现

修改：

- `src/danger_search/danger_search/ground_support_gate.py`
- `src/danger_search/danger_search/mission_manager.py`
- `src/danger_search/config/danger_search.yaml`
- `src/danger_search/test/test_ground_support_gate.py`
- `src/danger_search/test/test_mission_manager_helpers.py`

生产职责现在为：

```text
请求速度
  → 查询工程保守停车表
  → 计算时延余量、机身余量和边界余量
  → 决定支撑前缀是否足够

定位差分速度
  → 三个连续新样本确认停稳
  → 单次 > 1.0 m/s：立即零速观察
  → 恢复正常：自动恢复
  → 持续 2.0 s：localization_speed_abnormal 显式失败
```

保留 `validated_max_speed` 字段名仅为兼容既有内部配置结构；文档和快照不再把现行表称为
已封板实测结果。运行快照新增：

- `stop_envelope_index_source=requested_command`
- `localization_speed_role=stop_confirmation_and_anomaly_observation`
- `localization_speed_anomaly_threshold=1.0`
- `localization_speed_anomaly_timeout=2.0`

没有修改停车表数值、UNKNOWN 时序、`boundary_hold_margin`、DROP 几何、停稳阈值、formal
构造检查或状态名。

## 3. 离线制动分析器

新增：

- `tools/offline/analyze_braking_bag.py`
- `src/danger_search/test/test_offline_braking_analyzer.py`

工具只在任务结束后读取诊断 bag，生产节点不导入它。默认输入为：

- `/cmd_vel`
- `/danger_search/localization/odom`
- `/ground_truth/base_w`

冻结事件口径：

1. 前一条前进请求不低于 `0.30 m/s`；
2. 下一条线、横移和角命令均进入零命令；
3. 到真值线速度首次不高于 `0.05 m/s` 前，命令必须持续全零；
4. 停车距离为真值二维路径长，同时报告沿零命令起点航向的前向位移；
5. 定位停稳要求三个连续新定位样本的线、角速度均不高于 `0.05 m/s`；
6. 定位确认不得跨入下一段非零运动，避免把后续片段误配给当前制动事件；
7. 每个事件输出前置请求、直行/弧线分类、真值初速、停车时间、路径长、前向位移、
   定位确认时刻和该时刻真值速度。

合成测试覆盖成功提取、隐式低速事件过滤、停稳前重新给运动命令、三帧连续性、跨运动段
误配和空数据缺失语义，共 `6/6` 通过。

## 4. 合成 DROP 与异常链

现有真实 `GroundSupportSafetyGate`、`PathFollower`、`MissionManager` 和地图接线新增或
强化以下断言：

- 新鲜 DROP 且命令朝向缺口时进入 `DROP_STOPPING`，reason 为
  `negative_obstacle`，计数只增加一次；
- DROP 锁存后，普通跟踪、原地转向、倒车、恢复和 keepalive 的全部发布均为零；
- 停稳确认后进入 `SAFETY_BLOCKED`；
- 已确认缺口写入持久 `BLACK` 禁行区，普通建图更新不能擦除；
- 清路径和前沿后允许有界局部重规划，并复位为 `CLEAR`；
- 第 4 次负障重规划不再复位，任务显式进入 `WRITE_RESULT`；
- 定位速度短异常自动恢复且不产生终态；
- 定位速度持续异常满 `2.0 s` 后进入
  `SUPPORT_PATH_UNAVAILABLE/localization_speed_abnormal`，任务层不得把它当普通路径失败
  重规划。

这里沿用项目既有 `BLACK` 持久禁行语义。外部复核建议中的“写入 NEGATIVE”与当前实现
不符；`NEGATIVE` 是 ShadowSupportGrid 高频动态证据，明确 DROP 的重规划禁区必须使用
不会被普通建图擦除的 `BLACK`。

## 5. 历史 bag 可追溯复算

输入：

`/tmp/stage13_6_health_seed6946/elevator_diagnostic.bag`

新工具复现：

- 有效事件 `13`；
- 直行 `5`、弧线 `8`；
- 最大停车二维路径长 `0.0802662404 m`；
- 最大前向位移 `0.0797062723 m`；
- 最大停车时间 `0.3300 s`；
- 按“确认不得跨入下一段非零运动”的冻结口径，定位三帧确认可配对 `5` 次，
  确认时真值速度最大 `0.0137597 m/s`。

这精确关闭了 STAGE13.7 中“13 个事件、最大约 `0.0803 m`”缺少固定工具与过滤口径的
可复现性缺口。但该历史轮发生在本次预注册之前，不能冒充正式测量轮。

即使把请求不低于 `0.55 m/s` 作为“接近 `0.60 m/s`”的宽松代理，该轮也只有 `4` 个
事件，仍达不到预注册的每轮至少 `5` 个要求。

## 6. seed 8127 定向 Gazebo

本层在 Kimi 补充“持续定位异常必须显式失败”语义后，才执行有效定向运行：

```bash
SEED=8127 GUI=false \
ELEVATOR_TEST_FLOOR_TIMEOUT=33 \
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5 \
ELEVATOR_TEST_MAX_RUNTIME=180 \
ELEVATOR_TEST_FULL_CYCLE=0 \
ELEVATOR_TEST_RECORD_DRIFT=1 \
ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1 \
ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=0 \
ELEVATOR_TEST_LOG_DIR=/tmp/stage13_8_braking_seed8127_targeted_r1 \
tools/sim/run_elevator_gui_test.sh
```

运行结果：

- 传感器完成电梯视觉注册；
- 完成主入口、`0→1` 电梯换层并重新进入楼层 1；
- 专项运行器判定通过；
- STUCK `0`，恢复 `0`，安全门事件 `0`，定位异常事件 `0`；
- 峰值请求速度 `0.59834 m/s`；
- 采样时长 `74.704 s`，有效运动 `41.096 s`；
- 原始里程 `14.0676 m`，过滤楼层切换后的里程 `14.0646 m`；
- 楼层 0 的 bbox fill ratio 初始 `0.493`、最大 `0.497`、结束 `0.334`；
- 楼层 1 在专项停止时为 `0.461`；
- 正式识别数 `0`，没有评分 sidecar，评分为 `null`。

专项脚本在确认 `0→1` 后主动关闭正式任务节点，因此离线账本把 formal 终态记为
`external_ros_shutdown`；这不是算法失败，也不能算完整任务成功。它只证明本层改动没有
破坏定向 `0→1` 路径。

离线定位：

- 真值路程 `13.9700 m`；
- 平移 RMSE `0.10040 m`，P95 `0.18288 m`；
- 最大平移误差 `0.18738 m`，终点 `0.16039 m`；
- 最大绝对 yaw 误差 `0.00323 rad`。

制动事件：

- 有效事件 `3`；
- 直行 `1`、弧线 `2`；
- 前置请求分别为约 `0.3490/0.5402/0.5552 m/s`；
- 接近 `0.60 m/s` 的宽松代理事件仅 `1`；
- 最大停车二维路径长 `0.0696032 m`；
- 最大前向位移 `0.0693550 m`；
- 最大停车时间 `0.2900 s`；
- 两次可配对定位停稳确认的真值速度最大 `0.01146 m/s`；
- 定位差分速度全轮峰值 `0.57111 m/s`，未接近独立 `1.0 m/s` 故障阈值。

数值方向没有出现安全倒退，但事件矩阵明确不足，因此该轮不能封板 I3/I4/I5。

## 7. 停止条件与后续裁决

普通任务自然路线这一测量方法已有两个独立 seed 的证据：

- seed 6946：总事件较多，但接近 `0.60 m/s` 的宽松代理仍只有 `4`；
- seed 8127：只有 `3` 个总事件，接近 `0.60 m/s` 的宽松代理只有 `1`。

继续换 seed 跑相同任务属于重复碰运气，不能保证预注册的档位和直/弧线配额。按“同一
逻辑假设连续两轮无改善即停止”规则，本层停止自然任务采样，不降低事件门槛、不把历史
轮升级为正式测量、不扫 seed。

下一步存在一个需要外部架构复核的选择：是否允许新增一个诊断专用、仍经过
`PathFollower → GroundSupportSafetyGate → 唯一 /cmd_vel` 出口的受控制动测量 harness，
在合法生成场景的已确认支撑区域内执行固定的 `0.30/0.60 m/s` 直行与弧线序列。该选择
会新增运行入口并改变测量组织方式，本层不擅自实施。

在裁决前：

- 保持 formal 安全门开启；
- 保持 `max_linear=0.60` 冻结；
- 保持现行停车表“工程保守、待封板”状态；
- 不进入提速或防坠封板；
- 可继续不依赖该测量裁决的代码审计，但不得以本层结果关闭生产防坠证据缺口。

## 8. 验证与合规

执行：

```bash
python3 -m pytest \
  src/danger_search/test/test_ground_support_gate.py \
  src/danger_search/test/test_local_controller.py \
  src/danger_search/test/test_mission_manager_helpers.py \
  src/danger_search/test/test_offline_braking_analyzer.py -q
python3 -m pytest src/danger_search/test -q
python3 -m py_compile tools/offline/analyze_braking_bag.py
python3 tools/offline/check_production_boundary.py
git diff --check
sha256sum -c OFFICIAL_FILES.sha256
```

结果：

- 定向测试 `254 passed`；
- 全 Python 包测试 `522 passed, 1 warning`；
- `py_compile` 通过；
- 生产/离线边界检查通过；
- `git diff --check` 通过；
- 官方哈希仍只有 `auto.sh`、`FSM.h`、`FSM.cpp`、`gazebo.xacro` 四项既有不一致，
  本层无新增；
- 有效运行结束后 ROS master、Gazebo、控制器、任务和 rosbag 进程均为空；
- 未修改官方 world、生成器、Unitree/A1、传感器模型、电梯服务、`auto.sh` 或赛方文档；
- 真值只由独立诊断进程录制并在任务结束后离线使用；
- 没有提交、tag 或 push。

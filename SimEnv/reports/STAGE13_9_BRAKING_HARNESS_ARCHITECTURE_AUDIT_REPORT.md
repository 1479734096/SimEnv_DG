# STAGE13.9 制动测量 Harness 架构审计报告

## 1. 审计结论

外部复核已经允许新增诊断专用制动测量 harness，但其建议的“只发布 `/nav/goal`、
生产代码零改动、单轮完成 `0.30/0.60 m/s` 直行和弧线矩阵”在当前实现中不可直接成立。

本层只做只读代码审计、STAGE13.8 数字复算和计划落账，没有实现 harness、没有启动
Gazebo，也没有修改生产行为。

需要先裁决两个互斥架构方向：

1. 增加严格受 `runtime/profile=diagnostic` 限制的每目标速度上限接口；
2. 保持生产代码零改动，每个 seed 分别以 `max_linear=0.30/0.60` 启动两个诊断
   MissionManager 会话，再把同 seed 的两个会话合并为一轮测量。

在裁决前不能直接编码或运行 smoke。

## 2. STAGE13.8 勘误复算

使用当前交付工具重新执行：

```bash
python3 tools/offline/analyze_braking_bag.py \
  /tmp/stage13_6_health_seed6946/elevator_diagnostic.bag \
  --output /tmp/stage13_8_braking_seed6946_recheck.json
```

真实输出为：

- 有效事件 `13`；
- 直行 `5`、弧线 `8`；
- 最大二维停车路径 `0.0802662404 m`；
- 最大前向位移 `0.0797062723 m`；
- 最大停车时间 `0.3300 s`；
- 按“定位确认不得跨入下一段非零运动”的冻结口径，可配对确认 `5` 次；
- 确认时真值速度最大 `0.0137596538 m/s`。

因此 STAGE13.8 原报告中的 `11 次 / 0.02705 m/s` 沿用了允许跨运动段配对的旧口径，
已经修正为 `5 次 / 0.0137597 m/s`。该勘误不改变事件矩阵不足和未封板结论。

外部复核文本中出现的最大路径 `0.88027`、停车时间 `0.87971/0.83` 属转录错误，
不写入项目结论。

## 3. `/nav/goal` 的真实能力边界

当前诊断入口调用链为：

```text
/nav/goal PoseStamped
  → MissionManager._goal_callback()
  → NavigateToPose(target_xy)
  → A* 路径
  → PathFollower.tick()
  → GroundSupportSafetyGate
  → /cmd_vel
```

它满足“conductor 不发布 Twist、所有命令经过唯一安全出口”，但只携带 world XY：

- `_goal_callback()` 只读取 `msg.pose.position.x/y`；
- `NavigateToPose` 只保存目标 XY 和可选容差；
- `PathFollower.tick()` 的速度上限来自构造期全局 `controller.max_linear`；
- 没有每目标速度、曲率或驻留字段；
- 发布当前位置目标会调用 `clear_path()`，可以经统一出口直接产生零命令，这一部分可行。

因此同一个 MissionManager 实例中：

- 直行且航向误差很小时，请求速度趋近全局 `max_linear`；
- 不能只靠目标距离稳定生成可审计的 `0.30 m/s` 直行档；
- 横向偏置目标只能间接改变 pure-pursuit 航向误差，不能保证指定弧线角速度；
- A* 还可能按局部地图改变路径折线，conductor 无法预先承诺曲率。

## 4. 支撑前缀约束的不可实现部分

正式同级配置中：

```yaml
ground_support:
  maximum_lookahead: 0.80
```

`ShadowSupportGrid` 输出的 `supported_distance` 被该上限限制。外部复核建议的
“启动前确认 3–4 m 支撑走廊”无法由现有 gate 快照证明。

可实现且不放宽安全语义的约束应是：

1. 段落启动时 gate 为 `CLEAR`；
2. assessment 新鲜、观测充分、当前足印受支撑；
3. 当前 `supported_distance` 大于该请求档位的 guard 和释放滞回；
4. 行驶期间每个生产控制周期继续由 gate 重新评估；
5. 任意 UNKNOWN、DROP、定位异常或健康失败立即零速并中止本轮。

这利用的是生产门的连续 fail-closed 保护，不能改写成“一次观测证明整段 3–4 m 安全”。

## 5. 待裁决架构

### 方案 A：诊断专用每目标速度接口

最小方向：

- 新增独立诊断目标消息或服务，显式携带目标 XY、`max_linear` 和段落类型；
- MissionManager 仅在 `runtime/profile=diagnostic` 且显式配置开启时创建接口；
- formal 构造必须继续拒绝该接口；
- `NavigateToPose.tick()` 把段落速度上限传给既有 `PathFollower.tick(max_linear=...)`；
- conductor 仍不发布 `/cmd_vel`，停车仍通过当前位置目标触发 `clear_path()`。

优点：一个 seed、一个连续会话可以完成两档矩阵，档位与事件逐条可审计。

风险：需要修改生产仓库中的 MissionManager/NavigationTask 接口，并新增 ROS 接口面；
必须证明 formal 不可达、生产 launch 不导入、边界检查覆盖。

### 方案 B：同 seed 双诊断会话

最小方向：

- 不改生产代码；
- runner 在同一合法生成场景中，先后以 `controller.max_linear=0.30` 和 `0.60`
  启动两个 diagnostic MissionManager 会话；
- 每个会话只组织对应档位事件；
- 独立 bag 和 sidecar 保留会话边界，离线工具按 seed 合并为一轮。

优点：继续复用现有 `/nav/goal`，没有新增控制接口。

风险：

- 两档不在同一 MissionManager/地图/安全门生命周期内；
- 第二会话从第一会话结束位置启动，地图和任务状态重建；
- “一轮”的定义需要修改为“同 seed 两个会话的合并”，必须在运行前重新预注册；
- 弧线只能通过目标偏置和实际命令验收，不能由 conductor 预先保证。

## 6. 无论选择哪一方案都必须保留的验收

- conductor 源码不得导入或构造 `Twist`，不得发布 `/cmd_vel`；
- 所有非零和零命令都经过 `PathFollower._publish()` 与
  `GroundSupportSafetyGate.filter_command()`；
- 真值仅由独立 rosbag 录制，conductor 和生产节点不得订阅；
- 目标只由当前内部定位姿态、当前航向和新鲜支撑状态推导；
- 每个事件必须以实际 `/cmd_vel` 判断档位和直行/弧线，不以计划标签冒充；
- 零命令后驻留至少 `2.5 s`；
- 任意 DROP、`SAFETY_*`、`localization_speed_abnormal`、健康失败或支撑不足立即中止；
- harness 不进入 formal launch/import；
- smoke 失败最多按同一方案修正一次，第二次仍失败即停止并形成新检查点；
- harness 只能补制动动力学证据，不能替代真实空带 DROP 端到端证据。

## 7. 当前检查点

- STAGE13.8 数字勘误已完成；
- 未修改停车表、速度上限、安全阈值或状态机；
- 未新增 harness 文件；
- 未启动 Gazebo；
- 下一步等待用户或替代复核者裁决方案 A/B，或提出第三个同时满足速度可审计和
  formal 隔离的方案。

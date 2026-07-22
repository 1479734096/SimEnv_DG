# 第 8.2 层报告：Gazebo 姿态健康闭环验收

## 阶段结论

`LOCALIZATION-002` 已通过真实 Gazebo 传感器链和正式任务入口验收。机器人先稳定站立，
正式 `rgbd_livox` 定位建立 health/ready；外部测试夹具随后把模型侧倾。定位连续三帧
发现姿态/楼面证据异常后发布 health=false、撤销 ready，MissionManager 保留姿态异常
原因，零速完成 `WRITE_RESULT → FAILED` 和官方结果写出。

本结论只关闭“跌倒后任务仍继续”的失效安全缺陷，不关闭 `ENV-002` 官方暂停启动竞争，
也不关闭 `CONTROL-004` RL 控制交接跌倒。

## 测试条件

- 提交基线：`d08abe8`（`migration-layer-8-health`）。
- 单层、seed 42、无 GUI；关闭真值话题、裁判里程计、真值点云先验和生成真值副本。
- 官方 `auto.sh`、launch、URDF、Unitree/A1/RL 和传感器文件保持不变。
- 因 `ENV-002` 尚未关闭，使用既有诊断夹具：`PAUSED=true`、`AUTO_UNPAUSE=0`，等待
  `spawn_model` 延迟配置完成后，从测试端把四个 calf 设为 `-2.0 rad`，再恢复物理。
- 跌倒由测试端调用 Gazebo `set_model_state` 制造 90° 侧倾；该服务不被任何参赛节点
  调用，只用于确定性验证失效安全。

## 有效轮时间线

| 仿真时刻/证据 | 结果 |
|---|---|
| 站立后 | Gazebo 机身 `z=0.2569 m` |
| `6.830 s` | 楼面 `base_z=0.249 m`、`normal_z=0.998`，health/ready=true |
| `7.710 s` | 正式入口在 ready 后创建 MissionManager，进入 `WAIT_CONTROLLER_READY` |
| 侧倾后第 3 个异常帧 | `base_up_z=-0.393`、角速度 `1.680 rad/s`，楼面证据不可用并按异常累计 |
| `8.836 s` | 定位发布 health=false 并撤销 ready |
| `8.910 s` | MissionManager 以 `localization_stance_unhealthy` 进入 `WRITE_RESULT` |
| `9.012 s` | 写出结果并进入 `FAILED` |

`mission_runtime_summary.json` 记录：

- `terminal_state=FAILED`；
- `terminal_reason=result_write_reason=localization_stance_unhealthy`；
- `mission_success_eligible=false`；
- `controller_debug.max_abs_linear_x=0.0`；
- `controller_debug.last_command=[0.0, 0.0, 0.0]`。

## 合规性与接口核对

有效轮中 `/danger_search` 的外部订阅只有 `/clock`、RGB-D 公开话题和参赛方内部定位
话题；没有 `/Odometry_gazebo` 或 `/ground_truth/*`。生产 `/cmd_vel` 仍只有
`/danger_search` 发布，定位只把它作为运动先验消费。

测试夹具使用 Gazebo 服务和模型高度只做测试判定，不进入生产 launch、包、参数、结果
或控制决策。日志与生成场景不提交 Git，只保留本报告中的必要事实。

## 试验轮次说明

前两次未进入算法验证：第一次临时脚本文件名含 `gazebo`，被官方启动清理规则误伤；
第二次误读旧 `competition_gazebo.log`，在本轮 master 就绪前提前调用服务。清空旧日志并
同时等待 master、模型和本轮延迟配置后，有效轮完成上述闭环。按用户要求未开启第四轮。

## 后续边界

- 第 10 层正常长任务需回归姿态 health 不误报，但无需重复可控侧倾证明。
- `ENV-002` 必须由合法启动边界解决，不能把诊断 calf 写入正式参赛脚本。
- `CONTROL-004` 仍按零命令切 RL、首帧动作、速度阶跃和角速度分组验证。

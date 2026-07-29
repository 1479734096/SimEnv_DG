# 第 8.1 层报告：定位持续姿态健康闭环

## 阶段结论

本阶段完成 `LOCALIZATION-002` 的代码、单元测试和 ROS 节点级合成闭环：定位节点只用
公开 `/scan` 与 `/trunk_imu` 判断持续站立健康，正常证据满足连续门槛后发布
health/ready=true；连续倾倒或非法证据会锁存 health=false、撤销 ready，并停止发布该帧
里程计和配准点云。MissionManager 对明确 health=false 优先停车、取消成功资格并进入
`WRITE_RESULT`，不会再被随后出现的里程计陈旧原因遮蔽。

本报告提交时 Gazebo 真实跌倒尚未验收。后续第 8.2 层已用隔离站立夹具完成正式
RGB-D/Livox + MissionManager 的可控侧倾闭环，结论见 `reports/STAGE8_2_REPORT.md`。
`ENV-002` 官方暂停启动竞争和 `CONTROL-004` RL 控制交接仍是独立阻断。

## 对未完成实现的审计

接手时已有 health monitor、话题、launch 参数和 MissionManager 订阅，但存在三处不能
直接封板的问题：

1. 节点传给 health monitor 的 `base_height=0.35`、`ground_normal_z=1.0` 是常量，实际
   只有 IMU 参与判断，文档所称 `/scan` 楼面健康证据并未接通。
2. 非有限观测被忽略，可能一直保留上一帧健康值，属于安全边界 fail-open。
3. Python 测试只断言健康布尔谓词，没有验证停车、成功资格、状态转换和失败原因；同时
   tick 先判断里程计陈旧，可能遮蔽已经收到的 health=false。
4. 持续健康角速度上限 `0.60 rad/s` 低于控制器正常转向包络 `1.20 rad/s`，可能把连续
   正常转弯误判为跌倒。

## 实施内容

- `StanceHealthMonitor` 将缺失/非有限观测按异常帧累计，仍用连续帧门槛过滤单帧抖动。
- `scan_localization_node` 每个有效 Livox 帧都把点云变换到 base 系并重新估计楼面；用
  时间匹配 IMU 的机身竖直分量和角速度共同更新 health。
- health 未通过时发布 health=false、ready=false，并在 odom/cloud 发布前返回。默认
  `latch_unhealthy=true`，避免跌倒后抖动造成世界系重新放行。
- launch 暴露开关、站立高度、楼面法向、机身竖直、角速度、连续帧数和锁存参数。
- 默认健康角速度上限调整为 `1.50 rad/s`，并用接口测试约束它不低于正式控制器角速度
  包络；高度和方向证据仍负责识别倾倒。
- formal 配置和构造期均强制 MissionManager health 门控开启；只有 diagnostic smoke
  显式关闭任务侧门控。
- health 未收到时先安全停车，定位已可用但持续缺失超过定位不可用上限后显式失败；明确
  false 则优先于里程计陈旧立即失败，原因码为 `localization_stance_unhealthy`。
- 新增 ROS 合成测试：发布平面点云与竖直 IMU 建立 health/ready，再发布 90° 倾倒 IMU，
  验证两者均撤销。测试不启动 Gazebo、不读取模型状态或真值。

## 合规性复核

- 外部输入仅为官方接口文档允许的 `/scan`（`sensor_msgs/PointCloud`）和
  `/trunk_imu`（`sensor_msgs/Imu`）。
- `/danger_search/localization/health` 是参赛定位从公开传感器派生的内部锁存输出，不扩大
  信息边界。
- 生产代码没有订阅 `/Odometry_gazebo`、`/ground_truth/*`、Gazebo model/link state，
  没有读取 world、truth 或 layout metadata。
- 未修改 `auto.sh`、官方 launch/URDF、A1/Unitree/RL、传感器插件或其他
  `OFFICIAL_FILES.sha256` 保护文件。

## 自动化验证

| 验证项 | 结果 |
|---|---|
| Catkin 生产构建 | `catkin_make -j2` 成功 |
| 定位 C++ 测试 | 10 个测试程序、38 个用例全部通过 |
| 定位 ROS 合成闭环 | 1 项通过，站立后为真、倾倒后 health/ready 均撤销 |
| MissionManager/接口定向测试 | 135 项通过 |
| `danger_search` Python 全测试 | 398 项通过 |
| diagnostic ROS smoke | 约 1 秒超时失败写出成功；19 条控制命令全部为零 |
| Gazebo 跌倒闭环 | 本报告提交时未运行；第 8.2 层后续通过 |
| 官方文件完整性 | 1109 个 SHA-256 全部一致 |
| 补丁格式 | `git diff --check` 通过 |

## 未关闭风险与后续边界

- `LOCALIZATION-002` 的真实动力学侧倾验收已在第 8.2 层完成；第 10 层只需在正常长任务
  回归不误报。
- `ENV-002` 与按键 `8` reset 路径属于官方保护边界；未经用户确认不得把原工作区
  `auto.sh` 或 Unitree 修改迁入新仓库。
- `CONTROL-004` 先隔离零命令切 RL、首帧动作、速度阶跃和角速度，不在本阶段猜测参数。
- `fix.md` 中 P2～P5、F2～F5 及多楼层坐标/返航验收已汇总到
  `docs/team/MIGRATION_LEDGER.md`，按依赖顺序在后续阶段执行。

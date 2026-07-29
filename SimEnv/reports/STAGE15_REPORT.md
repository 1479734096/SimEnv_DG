# STAGE15 定位后端根因验证与候选实现报告

## 1. 结论

本阶段实现了可独立选择、默认关闭的 `livox_imu_local_map` 后端，并用同一固定
`seed=8127` 对纯 KISS-ICP 和新候选各做一次双层定向闭环。两者都未达到正式切换条件，
因此正式 `danger_search.launch` 仍默认使用 `rgbd_livox`。

- 纯 KISS-ICP：平移 RMSE `0.08805 m`，最大航向误差 `0.05801 rad`，在二层
  `92.612 s` 左右失败；淘汰为正式候选。
- Livox-IMU 局部地图：平移 RMSE `0.05598 m`，最大航向误差 `0.00294 rad`，成功
  完成 `0→1` 换层，但在二层 `102.714 s` 失败；IMU 定姿有效，平移与闭环仍未通过。
- 当前正式 `rgbd_livox` 的已记录基线为 RMSE `0.0520 m`、P95 `0.1045 m`。新候选
  没有形成可证明的净改善，不能切默认。

本阶段没有加入 FAST-LIO2、LIO-SAM、GTSAM、Ceres 或 g2o，也没有新增依赖。原因不是
实现成本，而是当前点云缺少可信逐点采样时间，无法满足逐点去畸变的输入前提。

## 2. 现状事实

### 2.1 IMU 当前如何使用

正式 `rgbd_livox` 使用公开 `/trunk_imu`：

1. 与 `/scan`、深度点云按时间戳匹配；
2. 以公开起点航向锚定 IMU 相对姿态，直接确定每帧 roll、pitch、yaw；
3. 参与站立初始化和持续定位 health；
4. 不积分线加速度得到正式位置。

旧 `imu_constrained` 后端会积分加速度，但其漂移不可控，只保留诊断兼容。新候选也不
积分加速度：IMU 只定旋转，平移必须由 Livox 地图配准产生。

### 2.2 Livox 与 RGB-D 当前如何合作

正式 `rgbd_livox` 中，RGB-D 帧间 GICP 是连续平移主先验；深度陈旧时 Livox 帧间
GICP 补位；Livox scan-to-map 每帧校正慢漂；IMU 统一确定姿态。相机还承担危险源、
房门和电梯视觉，因此“定位不用 RGB-D”不等于“停用相机”。

新候选只把 RGB-D 从定位闭包移除。定位由 `/scan + /trunk_imu + 自身 /cmd_vel` 完成，
RGB-D 仍完整用于识别和建筑交互感知，两条职责链不重叠。

## 3. 原因、方案、目的

| 原因 | 实施方案 | 目的 |
|---|---|---|
| 纯 KISS 同时估计旋转和平移，转弯时航向漂移会投影成全局平移误差 | 增加固定后端测试入口，先用同 seed 量化纯 KISS | 判断是否能用最小现成方案，避免先引入复杂融合 |
| `/trunk_imu` 姿态稳定，线加速度直接积分却会积累偏置 | 新候选只用 IMU 定姿，不积分加速度 | 保留 IMU 的强观测，避开位置积分弱项 |
| Livox 安装臂长在原地转向时会产生传感器原点运动 | 点先转换成世界方向、以 base 为中心的向量，优化变量固定为 base 平移 | 消除安装臂长造成的伪平移 |
| 弱结构错误帧会同时污染当前位姿和后续局部地图 | 位姿与地图事务提交；运动学超限时返回上一位姿且不写地图 | 阻断一次坏匹配演化成持续漂移 |
| 单一相机兼顾定位与视觉会形成资源和故障耦合 | 候选从定位闭包移除深度点云，RGB-D 继续只做感知 | 明确 Livox/IMU 定位与 RGB-D 识别职责 |
| 任务阶段切换定位源会产生新坐标系和状态转移 | 每次启动固定一个后端，禁止运行时自动切换 | 保持 PoseProvider 单源、输出连续且可审计 |

## 4. 实现

- 正式 launch 和电梯定向 runner 增加白名单后端参数；默认仍为 `rgbd_livox`。
- 新增 `livox_imu_local_map` 后端：
  - `/trunk_imu` 锚定世界姿态；
  - `/scan` 只保留楼面以上 `0.22～1.90 m` 的室内结构；
  - `/cmd_vel` 只作为配准初值，不能直接生成输出位姿；
  - 动态位移上界为 `min(0.30 m, 0.70 m/s × dt + 0.02 m)`；
  - 拒绝帧不发布新平移，也不更新地图。
- `ImuConstrainedOdometry` 增加 `translation_limit` 状态和事务拒绝测试。
- 没有改变正式 MissionManager、Mapper、PathFollower、安全门、识别或服务逻辑。

## 5. 固定 seed 证据

两轮均使用：`seed=8127`、2 层、单层超时 `33 s`、最小探索 `5 s`、任务预算
`180 s`、完整返程、定位逐帧诊断和运行结束后离线真值评估。

| 指标 | `kiss_icp` | `livox_imu_local_map` |
|---|---:|---:|
| 任务结果 | 失败 | 失败 |
| 成功换层 | 是 | 是 |
| 真值路程 | `14.636 m` | `19.048 m` |
| 平移 RMSE | `0.08805 m` | `0.05598 m` |
| 平移 P95 | `0.11496 m` | `0.11669 m` |
| 最大平移误差 | `0.19390 m` | `0.12112 m` |
| 末端平移误差 | `0.10866 m` | `0.11790 m` |
| 最大绝对 yaw 误差 | `0.05801 rad` | `0.00294 rad` |
| 末端 yaw 误差 | `-0.00708 rad` | `0.00029 rad` |

新候选 bag 时长 `99.99 s`，包含 `/scan` 1000 帧、`/trunk_imu` 38779 帧、内部 odom
980 帧；约为 `10 Hz`、`388 Hz`、`9.8 Hz`。生产进程检查未发现真值订阅。真值只由
runner 隔离录制并在任务退出后分析。

## 6. 根因判断

1. 纯 KISS 的主要问题包含航向漂移；IMU 定姿把最大 yaw 误差从 `0.05801 rad` 降到
   `0.00294 rad`，该假设成立。
2. 新候选仍有平移漂移。误差主要在二层行走的 `80～92 s` 从约 `0.054 m` 增至
   `0.118 m`，不是电梯升降瞬间跳变。
3. 1026 个地图配准帧全部被接受，现有验收只挡几何跳变，没有挡住统计上合理但方向
   有害的更新。离线数据中较高 `residual_p70` 与 Y 向有害更新有关，但单阈值同时会拒绝
   大量有益更新，证据不足以固化生产门槛。
4. 电梯后重置地图的离线反事实只把该段 RMSE 从 `0.08487 m` 改为 `0.08347 m`，改善
   约 `1.4 mm`；按任务阶段停用或重置某一定位量不是主要解法。
5. 两个后端都在二层因 `ground_support_path_replan_exhausted` 失败。新候选晚约 10 秒，
   但仍未证明定位变化能闭合安全路径问题，不能把该任务失败隐去。

## 7. 验证与合规

已完成：

```text
catkin_make --pkg danger_search_localization -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_danger_search_localization
catkin_test_results build/test_results/danger_search_localization
python3 -m pytest -q test_interface_documentation.py test_elevator_runner_contract.py
bash -n tools/sim/run_elevator_gui_test.sh
git diff --check
```

- 定位包：`110 tests, 0 errors, 0 failures`；
- Python 契约：`14 passed`；
- 两轮 Gazebo 均正常清理，结束后无 ROS master、Gazebo、控制器、任务或 rosbag 进程；
- 修改仅位于参赛方定位、launch、测试、工具和团队文档范围；
- 正式默认、合法输入缺失 fail-closed、PoseProvider 单源和统一 `/cmd_vel` 出口均未改变。
- 官方哈希仍有此前报告记录的 `auto.sh`、`FSM.h`、`FSM.cpp`、`gazebo.xacro` 四项历史
  差异，以及用户已有的 `docs/quick-start.md` 差异；本阶段没有修改这五个文件，也没有
  新增官方文件差异。

## 8. 下一阶段入口

本阶段不继续扫描残差阈值，也不按楼层、任务阶段或固定 seed 增加特例。后续只有两个
合规入口：

1. 先让官方授权点云提供真实逐点采样时间，再评估 FAST-LIO2 的去畸变、IMU 传播与
   ESKF；这涉及消息语义和新增依赖，实施前必须单独确认。
2. 若保持现有传感器消息不变，先做至少跨 seed 的只读残差门可分性验证；只有冻结阈值
   在训练外 seed 同时改善 RMSE、P95、末端误差且不降低闭环成功率，才允许加入生产门。

在局部里程计稳定前不引入 LIO-SAM/Pose Graph。因子图和回环只能校正全局累积漂移，
不能替代错误的逐点时间语义或不可靠的局部帧验收。

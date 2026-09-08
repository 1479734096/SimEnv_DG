# 正式接口闭包

依据仅来自官方 `README.md`、`docs/algorithm-interfaces.md`、`docs/reference.md` 和 `team_scene_info_v1` 生成代码。

## 话题

| 名称 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 输入 | ROS 仿真时间 |
| `/scan` | `sensor_msgs/PointCloud` | 输入 | Livox 原始点云 |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | 输入 | Livox 转换点云 |
| `/livox/lidar2` | `unitree_guide/CustomMsg` | 输入 | Livox 风格点云 |
| `/livox/imu` | `sensor_msgs/Imu` | 输入 | Livox IMU |
| `/trunk_imu` | `sensor_msgs/Imu` | 输入 | 机体 IMU |
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | 输入 | RGB 图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | 输入 | RGB 标定 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | 输入 | 深度图 |
| `/real_sense/depth/camera_info` | `sensor_msgs/CameraInfo` | 输入 | 深度标定 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 输入 | 深度点云 |
| `/cmd_vel` | `geometry_msgs/Twist` | 输出 | 唯一运动控制出口 |

## 服务和文件

| 名称 | 类型/格式 | 方向 | 约束 |
|---|---|---|---|
| `/set_door_state` | `building_generator_interfaces/SetDoorState` | 调用 | ID 必须来自公开文件 |
| `/call_elevator` | `building_generator_interfaces/CallElevator` | 调用 | ID 与楼层必须在公开集合内 |
| `generated_building/team_scene_info.json` | `team_scene_info_v1` | 读取 | 唯一公开场景文件 |
| `results/detected_danger.json` | 官方评估 schema | 写入 | 只能写公开声明的固定路径 |

`team_scene_info.json` 只提供起点、门的非空间属性、电梯 ID/served floors、允许接口和结果路径，不提供空间真值。

## 绝对禁用

- `/Odometry_gazebo`、`/ground_truth/*`、Gazebo model/link state；
- `layout_metadata.json`、`building_config.json`、`scene_manifest.json`、world、truth；
- 将上述数据改名、转发或缓存后作为生产输入；
- 参赛方 runner 调用 Gazebo 模型配置或传送服务改变机器人状态；
- 通过 ROS launch 或命令行 `<remap>` 将白名单话题重定向至禁用真值。

`interface_policy`（Python）与 `scan_localization_node`（C++）现已全面内置 ROS 话题解析（`rospy.names.resolve_name` 与 `ros::names::resolve`），在白名单比对前先解析实际通信话题名，双重防御原始名与重映射名，彻底封堵 `<remap>` 重定向漏洞；未初始化时安全兼顾纯离线环境。

## 空间先验消除与在线感知规范

1. **主入口定位与门洞检测**：初始进入观察点不再依赖全局坐标原点 `(0,0)` 先验，改为基于机器狗出生位姿 $(x_0, y_0, \theta_0)$ 沿初始朝向正前方延伸安全接近距离 $\text{target} = (x_0 + d_{\text{approach}}\cos\theta_0, y_0 + d_{\text{approach}}\sin\theta_0)$；彻底清除硬编码 `MAIN_ENTRANCE_CENTER = (0,0)` 常量，门洞候选过滤改为仅基于出生点横向中轴 $x_0$ 约束横向偏差 $|x_{\text{door}} - x_0| \le 0.60\text{m}$，纵向坐标完全由雷达在线探测决定，不依赖任何空间先验。
2. **高层层高获取**：电梯在仿真中通过 Gazebo `set_model_state` 瞬间传送（0 秒瞬移，无加减速），IMU 积分物理失效；系统采用“空洞激光在线测高为主 + 标称 2.6m 兜底”机制。机器狗在 1 层巡航时前下倾 45° 雷达穿透楼梯间空洞下视 0 层地表，提取双水平地面高差 $\Delta z$；严格执行 $[2.5\text{m}, 2.7\text{m}]$ 物理安全门限拦截（过滤半层 1.3m 楼梯平台），超出门限或未观测时安全回退至标称 2.6m。
3. **仿真真值解耦**：Gazebo 仿真中保持 `ENABLE_GROUND_TRUTH=1` 确保四足底层控制器（`IOROS.cpp`）足端物理反馈正常运行；`ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH` 仅用于独立控制测试脚本后台是否启动 `rosbag record` 录制离线分析包，两者彻底解耦。
4. **外墙几何先验彻底脱敏**：历史大门外墙纠偏模块已完全停用（硬锁死为 disabled），YAML 配置与代码参数中彻底删除 27 项硬编码外墙尺寸（如 `wall_thickness`、`wall_half_length` 等），杜绝未公开建筑参数暴露；机器狗过门与建图完全依赖在线传感器实测，不依赖任何静态几何模板。

## 第 2 层内部定位接口

这些话题由参赛定位节点从上述公开输入派生，只允许参赛方内部消费，不扩大外部信息边界：

| 名称 | 类型 | 方向 | 约束 |
|---|---|---|---|
| `/danger_search/localization/ready` | `std_msgs/Bool` | 定位输出 | 启动即锁存 false，站立证据通过后才为 true；持续监控异常时会重新置 false |
| `/danger_search/localization/health` | `std_msgs/Bool` | 定位输出 | 锁存；初始化/热身为 false，建立后由真实 `/scan` 楼面与时间匹配 IMU 持续更新；连续异常为 false，正式 MissionManager 应停车失败 |
| `/danger_search/localization/odom` | `nav_msgs/Odometry` | 定位输出 | `world → base`，只由 `PoseProvider` 统一订阅 |
| `/danger_search/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 定位输出 | 与里程计同时间戳、同世界系，供后续 Mapper 使用 |
| `/danger_search/path_raw` | `nav_msgs/Path` | 路径诊断输出 | 只记录规划器原始折线，不参与控制反馈 |
| `/danger_search/path_followed` | `nav_msgs/Path` | 路径诊断输出 | 只记录压缩后的跟踪折线 |
| `/danger_search/local_target` | `geometry_msgs/PoseStamped` | 路径诊断输出 | 只记录当前 pure-pursuit 前视目标 |

正式 `adaptive_shared_map` 定位节点采用 G3 默认，外部订阅闭包为 `/scan`、
`/trunk_imu` 和本系统已发布的 `/cmd_vel`。它不订阅 TF、Gazebo 状态、RGB-D 深度点云
或任何真值话题；MissionManager 仍按公开接口使用 RGB-D 完成感知。

默认关闭的 `livox_imu_local_map` 候选只订阅 `/scan`、`/trunk_imu` 和本系统已发布的
`/cmd_vel`；构造时不要求 `/real_sense/depth/points`。这只缩小定位输入闭包，不影响
MissionManager 对 RGB-D 图像、深度和内参的合法感知订阅。候选没有通过 Stage15
闭环验收，运行期间禁止自动切换到它。

正式 `adaptive_shared_map` 后端默认只让 Livox 针对唯一局部地图产生平移候选，最终只有
一条 `world → base` 位姿输出；显式设置 `adaptive_rgbd_localization_enabled:=true` 时才
恢复 `/real_sense/depth/points` 订阅及双候选选择。这不允许 MissionManager 按楼层、任务
阶段或状态切换后端。Livox 候选无效时不提交地图、不发布该帧 odom/cloud，也不回退到
真值、TF 或禁用文件。Stage17 当前只有 `8/20` 个有效 seed，冻结裁决仍为
`insufficient_evidence`，seed `1612` 的末端误差超过硬门；正式 launch 是按用户明确
决策直接切换，Stage17 工程验收状态为通过，但不表示矩阵已通过。`rgbd_livox` 保留为
显式回退，运行期间不得按任务阶段自动切换后端。

姿态 health 不增加外部输入：它只复用 `/scan` 与 `/trunk_imu`。缺失、非有限或无法
估计楼面的证据按异常帧累计；health=false 时同帧不再发布 odom/cloud，避免下游继续
消费已知不可信位姿。正式 profile 不允许关闭 MissionManager 的 health 门控。

第 4 层开始，生产 `/cmd_vel` 只能由 `PathFollower._publish` 建立的 publisher 发送；路径跟踪、旋转、倒车、恢复、停车与保活均先经过同一 `GroundSupportSafetyGate`。MissionManager 和导航任务不得新增第二个生产 publisher。

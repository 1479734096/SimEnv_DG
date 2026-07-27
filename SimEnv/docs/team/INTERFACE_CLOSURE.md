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
- 参赛方 runner 调用 Gazebo 模型配置或传送服务改变机器人状态。

`interface_policy` 保留兼容的无方向校验函数，但新增生产代码必须使用 `require_allowed_input_topic` 或 `require_allowed_output_topic`。

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

正式 `rgbd_livox` 定位节点的外部订阅闭包为 `/scan`、`/trunk_imu`、`/real_sense/depth/points` 和本系统已发布的 `/cmd_vel`。它不订阅 TF、Gazebo 状态或任何真值话题。

默认关闭的 `livox_imu_local_map` 候选只订阅 `/scan`、`/trunk_imu` 和本系统已发布的
`/cmd_vel`；构造时不要求 `/real_sense/depth/points`。这只缩小定位输入闭包，不影响
MissionManager 对 RGB-D 图像、深度和内参的合法感知订阅。候选没有通过 Stage15
闭环验收，正式 launch 默认仍是 `rgbd_livox`，运行期间禁止自动切换后端。

默认关闭的 `adaptive_shared_map` 实验后端订阅 `/scan`、`/trunk_imu`、
`/real_sense/depth/points` 和本系统已发布的 `/cmd_vel`，没有扩大正式后端已有的外部
输入闭包。Livox 和 RGB-D 只在同一个固定后端内部对同一地图产生候选，最终仍只有一条
`world → base` 位姿输出；这不允许 MissionManager 按楼层、任务阶段或状态切换后端。
候选 generation 不一致或两路都无效时不提交地图、不发布该帧 odom/cloud，也不回退到
真值、TF 或禁用文件。Stage16 只完成三组 seed，且曾使用不同运动进度的短窗提前停止
矩阵，证据不足以晋级；正式 launch 默认保持 `rgbd_livox`，等待冻结门槛的 20-seed
完整闭环裁决。

姿态 health 不增加外部输入：它只复用 `/scan` 与 `/trunk_imu`。缺失、非有限或无法
估计楼面的证据按异常帧累计；health=false 时同帧不再发布 odom/cloud，避免下游继续
消费已知不可信位姿。正式 profile 不允许关闭 MissionManager 的 health 门控。

第 4 层开始，生产 `/cmd_vel` 只能由 `PathFollower._publish` 建立的 publisher 发送；路径跟踪、旋转、倒车、恢复、停车与保活均先经过同一 `GroundSupportSafetyGate`。MissionManager 和导航任务不得新增第二个生产 publisher。

# 目标架构

## 依赖方向

```text
官方公开接口
  → runtime 适配层（SensorBuffer、PoseProvider、BuildingFeatureProvider）
  → 纯逻辑层（mapping、planning、perception、safety）
  → building/navigation 任务层
  → mission 编排层
  → CommandGateway 与 ResultWriter
```

上层可以依赖下层，下层不得导入任务编排、离线评估或真值工具。ROS 订阅、服务和文件访问集中在边缘适配层；算法核心优先保持无 ROS 副作用。

## ROS 包边界

- `danger_search`：Python 主系统。第 2 层已包含传感器缓存、统一位姿入口和定位健康/时间匹配决策。
- `danger_search_localization`：C++ 定位。第 2 层迁入固定版本 KISS-ICP C++ 分发、定位组件和定向测试，不引入上游 ROS wrapper、Python 包、评测 notebook 或 CI；上游自带的 `metrics` 源码不进入默认构建、链接、安装和启动链。
- `danger_search.mapper`：每层独立 2.5D 占据栅格，只接受第 2 层已配准世界系点云；UNKNOWN、BLACK、OCC 一律不可通行。
- `danger_search.planner`：单层八邻域 A*，显式区分到达、确定不可达和预算耗尽，禁止对角切角。
- `danger_search.frontier`：前沿提取、安全落点、有限候选规划、评分和短期失败黑名单；不发布速度或切换任务状态。
- `danger_search.navigator`：纯跟踪计算加 ROS 边缘发送器；路径、恢复、停车和保活命令都经过同一个 `_publish` 出口。
- `danger_search.ground_support_gate`：最终速度出口前的停车包络和支撑状态机；不改地图、不重规划，也不自行发布速度。
- `danger_search.red_sphere_detector`：同步 RGB-D 的红色分割、深度反投影和球面拟合；只有上层提供同时间匹配的刚体 `camera_to_world` 才输出逻辑 world 候选。
- `danger_search.detection_tracker`：危险源候选的多帧关联、质量确认和空间 NMS；同一输入帧对每条轨迹最多贡献一次命中。
- `danger_search.rgbd_door_detector`：深色门板的平面/尺寸验证和开门视觉变化；只输出匿名几何观测，不解释门 ID、不调用服务。
- `danger_search.building_control_client`：公开 door/elevator ID、服务权限和运行时传感器几何的唯一结合点；异步服务响应不直接改地图。
- `danger_search.local_semantic_detector`：从局部世界系点云提取门洞与轿厢结构；正式配置禁止真值邻近回退。
- `danger_search.navigation_tasks`：普通导航、过门和乘梯的非阻塞子状态机；只通过 Mapper、PathFollower、感知确认和建筑客户端推进。
- `danger_search.mission_manager`：10 Hz 顶层状态机；按定位健康、感知更新、安全门控、子任务和顶层决策的固定顺序推进，耗时返航规划只轮询后台快照任务。

## 离线工具边界

- 真值评分、bag 回放、Gazebo 状态分析和批量调参只能位于工作区根目录
  `tools/offline/`，不得进入任何 catkin 包。
- `danger_search` 的 Python 安装清单严格只有 `scripts/danger_search_node`；Python 包、
  setup、CMake 和 launch 均不得导入或安装离线工具。
- 环境自带的 `building_obstacles/evaluate_danger.py` 只允许在正式任务退出后单独执行，
  其真值输入和评测输出不得回流到同一轮生产状态。
- `tools/offline/check_production_boundary.py` 在提交和最终验收前重复检查上述边界。

迁移期保留旧模块导入路径；模块完成行为等价验证后再移动到内部子包，旧路径只保留兼容导出。

## 不变量

- 位姿只能来自统一 `PoseProvider`。
- 建筑公开信息只能来自 `BuildingFeatureProvider`。
- 生产 `/cmd_vel` 只能有一个最终发送入口。
- 生产包不能导入离线评估或真值工具。
- 缺失、陈旧、非法或未授权输入必须 fail-closed。

## 定位就绪契约

- 定位节点启动后立即锁存发布 `/danger_search/localization/ready=false`。
- 只有连续 8 帧公开 `/scan` 与 `/trunk_imu` 同时满足站立高度、楼面法向、机身朝向和角速度约束，才建立世界系并开始发布里程计与配准点云。
- 腹部着地时即便楼面法向和 IMU 朝向近似竖直，只要 `base_z` 不在 `0.20～0.55 m` 内就不得初始化。
- 世界系建立后，定位节点逐帧从公开 `/scan` 重新估计楼面高度和法向，并与时间匹配的 `/trunk_imu` 共同监控姿态健康；连续异常时撤销 `ready`、发布 health=false，并停止发布该帧里程计和配准点云。
- 缺失或非有限的楼面/IMU 证据按异常帧累计，不能沿用上一帧健康值放行。
- 不健康状态默认锁存，避免重锚定导致世界系跳变；诊断模式可通过参数关闭该锁存或 health 检查。
- 里程计速度使用一秒位姿窗口和死区估计；原始位姿不做静止锁定，避免把真实低速运动误判为零速。
- Python `PoseProvider` 只接受有限、四元数有效且时间严格递增的内部里程计样本；任务层必须通过 `LocalizationRuntime` 判断陈旧、待匹配和过期状态。
- MissionManager 在 health=false 时优先于里程计陈旧判定立即停车、取消成功资格并进入失败写出；定位可用但 health 持续缺失时也必须超时失败。
- 定位线/角速度死区分别为 `0.05 m/s`、`0.03 rad/s`；地面支撑门的停稳阈值不得小于对应死区，否则构造时失败。

## 定位后端与传感器职责

- 正式启动仍固定使用 `rgbd_livox`，不按任务阶段或单帧质量在不同定位后端之间切换。
- `/trunk_imu` 在正式后端中提供时间匹配的绝对姿态增量，并参与初始化与持续 health；
  不对加速度做位置积分。旧 `imu_constrained` 诊断后端仍保留加速度积分，但不作为正式
  方案。
- 正式 `rgbd_livox` 用 RGB-D 帧间 GICP 提供连续平移，用 Livox 帧间 GICP 在深度陈旧时
  补位，并始终用 Livox 局部结构地图校正慢漂。RGB-D 同时继续承担危险源、门和电梯
  视觉感知。
- `livox_imu_local_map` 是默认关闭的候选后端：IMU 固定旋转，`/scan` 局部结构地图估计
  平移，`/cmd_vel` 仅提供优化初值。位姿和地图以同一事务提交，结构不足或运动学超限时
  保持上一已接受位姿且不污染地图；RGB-D 不参与该后端的定位，但仍完整用于感知。
- `adaptive_shared_map` 是默认关闭的实验后端：IMU 始终确定姿态，Livox 维护唯一局部
  地图和约 `10 Hz` 输出时钟，RGB-D 与 Livox 只读同一地图生成平移候选。选择器只依据
  候选新鲜度、有限数、地图代次、配准结果和运动学上界逐帧选源；这属于固定后端内部的
  测量选择，不是按任务阶段切换定位后端。只有当前 Livox 帧可以按最终正式位姿提交地图；
  两路都无效时不发布新 odom/cloud，让既有陈旧检测 fail-closed。
- Stage15 固定 seed 验证中，该候选的最大航向误差为 `0.00294 rad`，但平移 RMSE 为
  `0.05598 m`、P95 为 `0.11669 m`，且二层闭环失败。因此不得把它改成正式默认，也
  不得据单轮日志增加残差阈值。
- Stage16 的 `adaptive_shared_map` 在 seed `8127/1612/8072` 上都完成闭环。seed `8072`
  按基线实际走过的同一 `1.529 m` 路程比较，RMSE、P95 和末端误差分别改善 `24.3%`、
  `15.8%` 和 `1.9%`；原相同时间比较混入了不同运动进度，不能作为退化证据。三轮
  RGB-D 选中占比仅约 `3.7%～12.5%`，样本也不足以晋级，正式默认暂时保持
  `rgbd_livox`，等待冻结门槛的 20-seed 完整闭环矩阵裁决。
- 当前 `/scan` 每帧只有统一时间戳，没有逐点采样时间；现有 `/livox/lidar2` 的
  `offset_time` 不是传感器采样时间。未修正公开输入语义前，FAST-LIO2 的逐点去畸变
  前提不成立，不能直接接入生产链。

## 建图与规划不变量

- 点云写障碍必须先于同帧自由射线；自由射线遇到 OCC/BLACK 立即停止。
- VISITED 只能覆盖 UNKNOWN/FREE/VISITED，不得擦除 OCC 或安全 BLACK。
- A* 允许把 UNKNOWN 起点修正为当前机器人自由格，但 OCC/BLACK 起点必须直接判定不可达。
- 阻塞目标的替代格按几何距离选择；规划预算耗尽不能累计成不可达或黑名单。
- 前沿候选必须先找到满足障碍净距与未知退让的已知自由落点，再由 A* 确认可达。

## 路径执行与安全不变量

- `PathFollower` 只能向白名单输出 `/cmd_vel`；原始路径、跟踪路径和局部目标只能写入 `/danger_search/*` 内部诊断话题。
- 路径点、当前位姿、算法请求命令或安全门返回值出现非有限数时，最终输出必须为零；非法位姿同时清除当前路径并报告 `STUCK`。
- 单点路径在目标容差外仍是有效活动路径，必须朝该点跟踪，不能访问不存在的下一段。
- 地面支撑门开启且尚无有效评估时，非零命令进入 `STALE_DATA_HOLD`，不得以“尚未发现缺口”为由放行。
- 支撑不足后的定时保活只能重发已经过门控的零速；恢复放行必须等待新的控制 tick，不能重放停车前旧命令。

## RGB-D 感知不变量

- RGB、深度、内参和 `camera_to_world` 任一缺失、非有限、尺寸不匹配或不是合法刚体变换时，检测返回空集合，不退回最新位姿、TF 或真值。
- 红色外观只能产生候选；危险源必须同时通过物理半径、球面 RMSE 和多帧轨迹确认。
- `min_hits` 表示不同 `DetectionTracker.update` 帧的命中数，同一帧多个相邻轮廓不得提前确认目标。
- 感知核心不自行添加楼层高度。若定位垂直参考为 `floor_local`，任务适配层只在构造 `camera_to_world` 时增加一次逻辑楼层 z 偏移；`absolute` 模式不增加。
- 门板观测只有几何位置、朝向、尺寸和开闭证据；与公开 door ID 的关联属于第 6 层任务边界。

## 建筑交互与导航任务不变量

- 服务调用前必须再次校验 door/elevator ID 和 served floors 属于公开场景集合；运行时几何不能给未知 ID 扩权。
- 门、电梯厅的 XY-yaw 和宽度必须来自有限的运行时传感器注册值，公开场景文件不提供空间目标。
- 普通 `NavigateToPose` 的 A* 确定不可达返回 `FAILED`；预算耗尽零速等待下一轮，二者都不得退化成直线穿越 OCC/BLACK。
- 过门和进出轿厢的直线跨越只允许在专用子状态机内，经段碰撞检查、朝向对齐、严格端点容差和真实位姿进度监督后执行。
- 主入口远场门洞只建立入场轴和门面；公开服务开门后必须以新鲜近场点云连续确认净空并更新等待/通过点。该刷新不改变普通房门状态机。
- 服务返回 accepted 只表示交互请求成功；清门区、确认开口和切换逻辑楼层仍需本地传感器证据。

## 任务编排不变量

- ROS 节点初始化后任务预算只使用 ROS 时间；读取异常显式失败，不得回退墙钟混用时间域。
- 电梯结果只在任务 `DONE` 后记真，`FAILED` 记假，发起和执行中保持未知。
- 楼层估计与逻辑楼层持续不一致时阻止楼层数据写入；对齐超时必须停车并进入结果写出，不能永久零速等待。
- 无预算返航 A* 只读取栅格与历史轨迹快照并在后台运行；主 tick 在结果就绪前持续发送安全零速，过期楼层或目标的结果直接丢弃。

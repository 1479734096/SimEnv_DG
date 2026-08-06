# MissionManager 职责分离报告

## 1. 目标与边界

本阶段以提交 `5051111` 的 seed=8127 双层短闭环为行为基线，逐批把配置、ROS
边缘、感知与地图更新、电梯、探索和返航实现迁出 `MissionManager`。迁移只改变职责
所有权，不改变地图语义、安全阈值、规划策略、服务语义或统一 `/cmd_vel` 出口。

## 2. 第一批：配置、ROS 边缘与报告输出

新增三个模块：

- `mission_config.py`：完整保留原参数默认值、校验顺序、公开场景能力校验和正式接口
  白名单；`MissionManager` 只安装已经校验的 `MissionConfig`。
- `mission_node_runtime.py`：持有 publisher/subscriber、主定时器、GroundSupport 定时器、
  墙钟 watchdog、定位 ready 等待和 ROS shutdown。
- `mission_reporting.py`：持有结果 writer、mission summary、状态/地图 publisher 和
  结构化日志。

已从 `MissionManager` 删除 `_load_params`、`start/shutdown` ROS 生命周期、ROS 回调、
publisher 字段、`ResultWriter` 所有权和 ROS 参数服务器访问。节点脚本改由
`mission_node_runtime.main` 启动。业务层通过显式方法接收手动目标、定位 health、墙钟
超时和 shutdown 事件。

### 2.1 验证

- 相关测试：`246 passed`；
- 完整 Python：`605 passed`；
- `py_compile`、`git diff --check`、`git diff --cached --check`：通过；
- `tools/offline/check_production_boundary.py`：通过；
- 禁用输入审计：未新增 truth、world、metadata 生产依赖；
- `/cmd_vel`：仍只有 `navigator.py` 创建和发布。

### 2.2 seed=8127 短闭环

配置：2 层、`floor_timeout=33`、`min_floor_explore_time=5`、
`max_runtime=180`、`full_cycle=1`、`adaptive_shared_map`。结果：

1. `2.054 s` 进入 0 层探索；
2. `33.156 s` 开始 `0→1`，`73.358 s` 完成；
3. `103.558 s` 开始返航，`148.160 s` 完成 `1→0`；
4. `167.960 s` 出门并回到起点后写结果；
5. `168.062 s` 正常进入 `FINISH`，`terminal_outcome=success`、
   `return_home_completed=true`。

外置 `/scan` 为 `1676` 帧、`167.510 s`，约 `10 Hz`。一次 GroundSupport 路径失效
按既有反馈链恢复，无 Python traceback、定位终态或 DROP 终局。证据摘要位于
`/tmp/refactor_batch1_seed8127_SykIHW/`。按用户磁盘约束，分析后已删除大型 bag 和高频
原始日志，仅保留 52 KiB 的摘要、配置、节点信息与哈希。

## 3. 后续批次

## 3. 第二批：感知、地图、楼层作用域与 GroundSupport 所有权

新增四个边界模块：

- `perception_runtime.py`：拥有 `SensorBuffer` 和点云/RGB-D 两套消费游标；读取不等于
  消费，只有上层明确处理或拒绝后才提交 generation；
- `map_update_runtime.py`：拥有 `MultiFloorGridMap`、危险源检测器、门板检测器和
  `DetectionTracker`，主线程提交点云/RGB-D 后只返回冻结摘要；
- `floor_transition_runtime.py`：拥有唯一换层守卫，显式换层必须完成服务/传感器确认并
  收到两帧更新的里程计才解除楼层写入锁存；
- `ground_support_coordinator.py`：拥有 Gate、ShadowSupport 与反馈队列，安全决策和地图
  语义没有迁入任务编排层。

`MissionManager` 不再构造上述具体组件，也删除了重复的障碍距离实现和
`_configure_ground_support_gate` 旧入口。测试中绕过构造器的对象统一安装新运行时所有者，
不再通过给生产类增加兼容 setter 来保留旧字段写法。伪传感器帧使用明确 generation，避免
Python 对象地址复用造成虚假的重复帧。

### 3.1 静态与单元验证

- 新边界及 MissionManager 定向测试：`175 passed`；
- 完整 Python：`610 passed`；
- `py_compile`、生产边界、禁用输入、唯一 `/cmd_vel` publisher 和两类 diff check 在
  本批提交前执行；
- 本批未改变 Mapper、ShadowSupport、Gate、A*、服务参数或安全阈值。

### 3.2 运行验证

同第一批配置完成 seed=8127 双层短闭环，runner 返回 `0`：

1. `2.044 s` 进入 0 层探索，`33.244 s` 开始 `0→1`，`71.544 s` 完成；
2. `100.846 s` 进入返航，`100.946 s` 开始 `1→0`，`145.248 s` 完成；
3. `163.244 s` 确认返航完成，`163.344 s` 写出结果并进入 `FINISH`；
4. 两次运输均依次记录 `ambiguous_enter → sensor_confirmed → 2 帧新 odom →
   ambiguous_exit`，无对齐超时、旧云重放或 `cloud_rejected`；
5. `/scan` 为 `1701` 帧、`169.978 s`，约 `10.007 Hz`，覆盖完整任务时段；
6. 一次 GroundSupport observation hold 自动恢复，总 hold 约 `2.098 s`，无 DROP
   终局或不可恢复锁存。

关键摘要保留在 `/tmp/refactor_batch2_seed8127_kdxXO4/`，共约 `64 KiB`。在冻结
`RUN_SUMMARY.md`、`TEST_SUMMARY.txt`、配置、节点信息、bag 元信息和三项 SHA-256 后，
已永久删除约 `490 MB` `/scan` bag、约 `85 MB` 诊断 bag 和逐帧原始日志。另清理了一
个第一批遗留的只读 `rostopic hz /scan` 监控进程；未终止任何非本轮 ROS/Gazebo 进程。

## 4. 第三批：导航上下文与任务运行状态

新增四个纯 Python 边界：

- `navigation_context.py`：冻结每次子任务 tick 可见的组件、配置和回调。`navigation_tasks.py`
  不再接收 `MissionManager`，也不再通过 `getattr` 探测编排器私有方法；门洞和电梯几何
  刷新的 `RUNNING/DONE/FAILED` 状态原样传递，禁止误转成布尔值。
- `exploration_runtime.py`：持有无前沿扫描/沿轨迹逃逸、前沿惯性和有界进展历史；纯函数
  计算近期增长，不切换顶层任务状态。
- `elevator_observation_runtime.py`：持有电梯 RGB-D/Livox 多帧确认、近场刷新、开门确认和
  视觉准备状态；计数和局部 reset 不再散落在编排器字段中。
- `return_home_runtime.py`：持有异步规划器、失败首段、卡住计数和受控后退状态；任务是否
  成功、何时写结果仍由顶层编排器决定。

测试同步删除了对 `mission_manager_module.rospy` 的无效全局伪造；绕过构造器的用例必须
安装真实运行时所有者，导航任务用例通过真实 `NavigationTaskContext` 冻结依赖。没有为旧
字段增加兼容 property 或 setter。

### 4.1 静态与单元验证

- 导航、编排器、运行时边界和地标集成定向测试：`237 passed`；
- 完整 Python：`614 passed`；
- `py_compile`、生产边界、禁用输入、唯一 `/cmd_vel` publisher 和两类 diff check：通过；
- 本批没有修改 Mapper、ShadowSupport、GroundSupport Gate、A*、服务参数或安全阈值。

### 4.2 运行验证

同第一、二批配置完成 seed=8127 双层短闭环，runner 返回 `0`：

1. `2.046 s` 进入 0 层探索，`33.146 s` 开始 `0→1`，`74.746 s` 完成；
2. `97.046 s` 进入返航，`97.146 s` 开始 `1→0`，`143.546 s` 完成；
3. `161.846 s` 确认返航完成，`161.946 s` 写出成功结果并进入 `FINISH`；
4. 0 层和 1 层最高覆盖率分别为 `0.502`、`0.461`；该短测验证职责迁移后的机械
   闭环，不作为长时间探索质量或随机 seed 验收；
5. 外置 `/scan` 为 `1692` 帧、`169.100 s`，约 `10.006 Hz`，运行图审计通过；
6. 一次 GroundSupport 暂停按既有约束反馈链重规划恢复，无 DROP、STUCK/recovery、
   定位终局、Python traceback 或任务失败事件。

关键摘要位于 `/tmp/refactor_batch3_seed8127_tY7Sgd/`。摘要、配置、节点信息和 bag 哈希
冻结后，外置 `/scan`、诊断 bag 与逐帧原始日志按精确路径永久清理。

## 5. 后续边界

`MissionManager` 仍保留顶层任务策略：进入建筑、探索决策、换层触发、返航编排和结果状态
转换。后续只按完整策略族迁移，不再拆散单个字段或建立第二个总管；每批先删除旧接口并
更新测试，再做静态验证和一次同配置短闭环。

## 6. 第四批：冻结配置、唯一装配根与真实测试构造

新增 `mission_components.py`，由 `MissionRuntimeComponents.build()` 按固定顺序创建定位、
感知、地图、探索、控制、GroundSupport、建筑服务和进展监督组件。`MissionManager`
只绑定冻结组件引用，测试可以注入同形状替身，但不能再绕过构造器制造缺字段的半对象。

`MissionConfig` 按运行时、话题、定位、建图、入场、电梯、探索、返航和输出分成九个只读
分组。分组函数会拒绝重复、遗漏或未归属字段，防止参数在职责迁移时静默丢失。电梯运行
时 ID 改为独立状态，不再回写冻结配置字典。

定位推进只保留 `LocalizationRuntime.decide_progress()` 一条组合决策路径，显式姿态异常、
里程计陈旧和姿态状态等待采用固定优先级。删除了编排器内重复的里程计判断和姿态判断
入口，对应测试全部改为验证唯一生产入口。

初始入场的可变状态和两项纯几何计算迁入 `initial_entry_runtime.py`。本批尚未迁移完整入场
策略；它将与门任务一起作为完整策略族处理，不再继续拆单个字段。

### 6.1 测试与边界

- `test/` 完整 Python 包：`619 passed`；
- 配置分组、组件注入、定位组合决策和真实构造边界相关测试：`181 passed`；
- `test/` 中 `MissionManager.__new__`、旧定位双入口生产/测试引用均为零；
- 本批没有修改 Mapper、ShadowSupport、GroundSupport Gate、A*、服务参数或安全阈值；
- 本批只改变职责和测试构造边界，未运行 Gazebo；最终职责迁移完成后统一执行 seed=8127
  双层闭环，避免每个纯结构批次重复生成大型日志。

## 7. 第五批：楼层坐标、地图帧状态与 GroundSupport 证据处理

新增冻结 `FloorCoordinateModel`，由组件装配根依据定位垂直参考和公开地图楼层高度构造。
绝对 Z 与 `floor_local` 的逻辑坐标换算、楼层估计和点云 Z 偏移只保留这一份实现；点云、
RGB-D、GroundSupport 和诊断不再各自读取管理器配置推导楼层坐标。

云/图像最近时间戳、最新合法点集、最近前向/径向障碍摘要及楼层不匹配去重键全部归
`MapUpdateRuntime`。删除 `MissionManager` 中对应状态字段和旧测试写法，任务层只读取地图
运行时摘要。

`GroundSupportCoordinator` 新增完整证据处理入口：时间匹配点云经过 ShadowSupport 后构造
`GroundSupportAssessment`，在统一控制事务内更新 Gate；已确认负障碍的 BLACK 落图、Gate
复位、反馈计数和 BLACK 重施也由协调器原子执行。`MissionManager` 只根据冻结重规划记录
清理任务目标、记录诊断或进入顶层失败状态。旧的评估、负障碍恢复和 BLACK 重施私有方法
已删除。

### 7.1 测试与边界

- `test/` 完整 Python 包：`622 passed`；
- 楼层坐标、地图状态、GroundSupport 并发命令变化、禁用 Gate、单帧单消费和负障碍恢复
  相关定向测试：`240 passed`；
- `py_compile`、生产边界、禁用输入、唯一 `/cmd_vel` publisher 和两类 diff check 在提交前
  执行；
- 本批未改变支撑阈值、Gate 状态机、Mapper 栅格语义或任务失败优先级，未运行 Gazebo。

## 8. 第六批：完整主入口进入策略

`InitialEntryRuntime` 从单纯状态容器扩展为完整主入口进入子状态机。它现在独立拥有：

- 公开主入口开门服务的一次性请求与异步等待；
- 初始朝向锁存、滚动自由走廊目标和单次卡滞重试；
- 两帧点云门洞一致性确认、沿初始轴投影的入口注册；
- `OpenDoorAndPass` 子任务构造和明确失败结果。

冻结 `InitialEntryConfig` 只从 `MissionConfig.entry` 构造。子状态机直接依赖控制器、合法
局部检测器、门观测账本和建筑服务组件，返回 `InitialEntryTickResult`；`MissionManager`
只应用“是否截断本 tick、是否清理探索目标、安装哪个任务、记录哪个失败”四类顶层结果。
删除 `_tick_initial_entry_probe`、`_tick_initial_entry_heading_alignment` 和
`_try_start_initial_entry_door_task` 旧实现及全部测试引用。

### 8.1 测试与边界

- `test/` 完整 Python 包：`622 passed`；
- 入场服务、连续门洞确认、距离拒绝、朝向恢复、导航任务与编排器定向测试：`225 passed`；
- 本批未修改门/电梯服务接口、控制器发送链、地图语义或安全阈值，未运行 Gazebo；
- 门近场刷新与电梯注册仍待按完整策略族迁移，因此第三批总任务尚未完成。

## 9. 第七批：门近场刷新与电梯视觉准备

`DoorObservationRuntime.refresh_geometry()` 现在完整拥有主入口近场刷新：超时、候选距离、
宽度、门轴、中心偏移、连续确认、门几何替换和活动 `OpenDoorAndPass` 的传感器确认均在
同一模块执行。编排器只提交当前点云事实并记录冻结的失败/诊断结果，原约 130 行刷新
实现已删除。

`ElevatorObservationRuntime` 完整拥有视觉准备服务的启动、幂等锁存、pending 查询和终态
日志。公开 `MissionManager.start_elevator_visual_preparation()` 仅负责提交当前公开场景与
组件引用；旧私有启动和结果处理入口已删除，导航任务继续通过公开 pending 合同等待，
没有新增服务或改变门命令语义。

### 9.1 测试与边界

- `test/` 完整 Python 包：`622 passed`；
- 门刷新、电梯视觉准备、导航任务和编排器定向测试：`237 passed`；
- 本批没有修改电梯注册几何算法、门服务参数、地图/安全语义或控制出口，未运行 Gazebo；
- 电梯扫描、注册、近场刷新和开门确认仍待整体迁入 `ElevatorMissionRuntime`。

## 10. 第八批：无前沿扫描与沿迹退避状态机

`ExplorationRuntime` 现在完整拥有无前沿恢复的有界状态机：原地扫描、旋转净空门控、沿本层
已走轨迹选择单次直线退避、低速倒退、一次复扫和耗尽终态均在同一模块推进。状态机只返回
冻结的 `ExplorationRuntimeResult`；`MissionManager` 负责记录任务失败，并依据 `completed`
决定是否结束当前楼层，不把换层或任务成功策略下沉。

旋转目标格净空和起点连续 OCC 膨胀带退出判定迁为模块级纯函数。电梯起点退避与返航贴墙
退避复用同一实现，不再依赖 `MissionManager` 内的静态方法。旧 `_tick_no_frontier_scan`、
`_try_start_no_frontier_escape`、`_tick_no_frontier_escape_motion`、栅格几何辅助方法和扫描旋转
入口已经删除；测试直接调用新状态机合同，没有建立兼容别名。

### 10.1 测试与边界

- `test/` 完整 Python 包：`622 passed`；
- 编排器、运行时边界和导航任务定向测试：`237 passed`；
- 贴墙零速、净空旋转、沿迹退避不改地图、无安全轨迹耗尽、仅一次复扫和倒退完成等旧行为
  均由新状态机接口覆盖；
- 本批不改变栅格状态、规划通行掩码、退避距离、旋转净空、安全 Gate 或 `/cmd_vel` 出口，
  未运行 Gazebo；电梯注册/扫描和返航执行仍是后续完整职责族。

## 11. 第九批：电梯全向视觉扫描状态机

`ElevatorObservationRuntime.tick_visual_scan()` 现在完整拥有主入口内的一次性 RGB-D 电梯
地标扫描：准备任务门控、启动清路径、累计实际正向旋转量、候选确认时停车、超时结束和已注册
地标提前结束均在同一状态机内。冻结 `ElevatorVisualScanConfig` 与
`ElevatorVisualScanResult` 分别承载配置和顶层效果；`MissionManager` 只清理探索目标、记录日志
并依据 `handled` 截断当前探索 tick。

扫描证据汇总同时迁入运行时。电梯近场刷新超时直接调用运行时只读诊断，不再依赖已删除的
`_elevator_visual_scan_diagnostics`。旧 `_tick_elevator_visual_scan` 和诊断私有方法已删除；新增
独立 `test_elevator_observation_runtime.py`，避免继续由编排器巨型测试锁定旧接口。

### 11.1 测试与边界

- `test/` 完整 Python 包：`624 passed`；
- 电梯观测、编排器、导航任务和运行时边界定向测试：`239 passed`；
- 独立覆盖“已转一圈仍观察到有界超时”和“运行期地标注册后立即结束”两条核心路径；
- 本批未改变 RGB-D 检测阈值、门服务、电梯服务、路径规划、地图或安全控制语义，未运行
  Gazebo；Livox/RGB-D 注册与近场刷新仍待迁入同一观测所有者。

## 12. 第十批：电梯开门传感器确认

电梯门基线锁存、RGB-D 关闭门板变化、已注册门平面变化、Livox 连续开口确认和终态摘要全部
迁入 `ElevatorObservationRuntime`。`MissionManager` 保留导航任务公开需要的
`begin/confirm/finish` 三个薄适配入口，只提供当前合法点云、楼层过渡状态和组件引用，并记录
冻结结果；图像更新流水线直接调用运行时，不再经过编排器私有处理函数。

依赖按实际分支惰性读取：RGB-D 已达到确认次数时不要求 Mapper 或 Livox 检测器存在；只有
Livox 备选确认才使用楼层高度、当前点云和局部门洞检测。这一约束由独立运行时测试固化，
防止适配层重新引入与当前证明无关的组件依赖。

### 12.1 测试与边界

- `test/` 完整 Python 包：`625 passed`；
- 电梯观测、编排器、导航任务和运行时边界定向测试：`240 passed`；
- 旧 `_update_elevator_door_visual_change` 已删除，测试直接验证运行时公开合同；
- 本批未改变两帧确认次数、几何容差、Livox 备选语义、服务调用或控制路径，未运行 Gazebo。

## 13. 第十一批：电梯候选注册与近场刷新

新增 `ElevatorRegistrationRuntime`，独立拥有 Livox 开口候选、RGB-D 门板候选、时间连续性、
关联拒绝计数、近场刷新序列和运行期几何安装。`ElevatorObservationRuntime` 只嵌套一个注册
所有者供视觉扫描和开门确认读取，不再复制候选字段；地图图像更新也只写嵌套注册对象。

正式注册入口一次性校验公开电梯、楼层门 ID 与 served floors，连续证据满足后生成运行期
`ElevatorInfo`/`DoorInfo`。近场刷新由同一模块验证中心漂移、朝向变化和宽度，并在超时时
保留既有几何。`MissionManager` 只构造冻结配置、提交当前合法事实、应用日志和保存活动电梯
ID；旧 `_try_register_local_elevator`、`_try_register_rgbd_elevator` 和
`_register_elevator_geometry` 已删除。

### 13.1 测试与边界

- `test/` 完整 Python 包：`627 passed`；
- 电梯注册/观测、地标漂移、编排器、导航任务和运行时边界定向测试：`245 passed`；
- 新增独立测试覆盖三帧 RGB-D 连续注册与刷新超时不覆盖既有几何；所有旧注册测试改为直接
  驱动新运行时，无兼容私有入口；
- 本批未改变注册确认次数、候选阈值、公开 ID 规则、服务接口、地图或安全控制语义，未运行
  Gazebo；电梯导航退避几何仍待迁入导航服务模块。

## 14. 第十二批：电梯导航轨迹几何

新增 `elevator_navigation_runtime.py`，把电梯大厅侧已走轨迹候选筛选和起点实体膨胀带退避
目标选择迁为无状态纯函数。两个函数只消费冻结的电梯几何、楼层栅格和导航轨迹，不读取
`MissionManager` 状态，也不修改正式地图；候选仍必须由调用方经过标准 A* 和统一安全发送链。

`MissionManager.elevator_approach_trace_candidates()` 与
`MissionManager.elevator_start_retreat_target()` 保留为导航任务需要的公开薄适配入口，旧的排序、
净空、倒车朝向和初始 OCC 连续带判断实现已经删除。退避仅允许沿已走轨迹离开起始 OCC 带，
目标格仍须正式可通行并满足既有净空约束，没有引入盲倒车、坐标或 seed 特例。

### 14.1 测试与边界

- `test/` 完整 Python 包：`629 passed`；
- 电梯导航/注册/观测、编排器、导航任务和运行时边界定向测试：`244 passed`；
- 独立测试覆盖大厅侧候选过滤和初始 OCC 带沿迹退避，且确认退避选择不修改正式栅格；
- 本批不改变候选距离、退避距离、旋转净空、规划掩码、安全 Gate 或 `/cmd_vel` 出口，未运行
  Gazebo；返航规划/卡滞处理和感知建图提交仍是后续主要职责族。

## 15. 第十三批：返航卡滞反馈与沿迹退避

`ReturnHomeRuntime` 从字段容器扩展为返航恢复子状态机，独立拥有卡滞区域计数、失败首段去重与
四段有界账本、最近障碍摘要、离开卡滞区后的计数复位、沿本层已走轨迹选择安全直线退避、
退避时限和倒车朝向判定。规划仍由既有 `return_home_planner.py` 在冻结快照上执行，运行时不
发布控制、不写地图，也不决定任务成功或失败。

`MissionManager` 的返航恢复入口现在只提交当前位姿、控制段、合法障碍摘要和配置，随后应用
冻结结果：恢复旋转、直线倒车、停车或进入明确失败终态。顶层换层、主入口任务、到家成功和
结果写入继续留在编排层。旧的失败段构造、退避搜索和退避子状态判断已从编排器删除。

### 15.1 测试与边界

- `test/` 完整 Python 包：`633 passed`；
- 返航运行时、编排器、运行时边界和导航任务定向测试：`241 passed`；
- 独立测试覆盖四段上限、离开卡滞区只清计数不清禁行首段、安全退避选择，以及完成、超时、
  朝向错误三类退避终态；
- 本批不改变卡滞距离、退避距离、净空、A*、旋转恢复、安全 Gate 或控制发送语义，未运行
  Gazebo；返航异步规划轮询与主入口交接仍待进一步收敛。

## 16. 第十四批：感知帧时间匹配与消费准备

`PerceptionRuntime` 新增冻结的 `CloudPreparation`/`ImagePreparation`，统一负责新 generation
识别、消息时间戳校验、RGB-D 三消息同步检查和时间匹配位姿状态分类。准备阶段不推进消费
游标、不写地图；待匹配帧可以在后续 tick 继续匹配，过期或非法帧仍由编排器显式消费并记录
原因。

`MissionManager._update_mapping_and_detection()` 改为消费准备结果，只保留楼层过渡门控、写图
授权、任务诊断和检测结果应用。楼层是否允许提交仍由 `FloorTransitionRuntime` 决定，没有下沉
到感知模块；这样既避免感知适配器知道电梯任务，又消除了编排器自行解释时间戳和同步窗口的
重复逻辑。

### 16.1 测试与边界

- `test/` 完整 Python 包：`636 passed`；
- 感知运行时与编排器完整 helper 测试：`185 passed`；
- 新增反例覆盖待匹配点云不消费、匹配点云采用同时间位姿、RGB-D 不同步在位姿查询前拒绝；
- 本批不改变云点解码、楼层坐标、Mapper、检测阈值、合法输入或消费时机，未运行 Gazebo；
  楼层门控后的地图/检测提交仍可继续迁入独立提交协调器。

## 17. 第十五批：楼层门控后的地图与检测提交

新增 `PerceptionMappingRuntime`，组合 `PerceptionRuntime`、`MapUpdateRuntime`、楼层坐标模型和
统一位姿提供者，完整拥有楼层守卫授权后的点云/RGB-D 提交事务。点云事务依次完成解码、逻辑
楼层 Z 转换、generation 消费、Mapper 更新和前向/径向障碍摘要锁存；图像事务完成游标推进、
同时间位姿外参、光学坐标转换、楼层 Z 偏移和 Detector/Tracker 更新。

非法、过期和楼层过渡期帧通过同一协调器显式丢弃，待匹配帧仍不消费。`MissionManager` 只判断
楼层写入授权、记录任务诊断、重放 GroundSupport BLACK owner，并把冻结的门板观测提交给电梯
观测运行时。旧相机变换辅助函数已从编排器删除，坐标变换测试改为直接验证提交模块。

### 17.1 测试与边界

- `test/` 完整 Python 包：`638 passed`；
- 提交协调器、感知边界、编排器、组件装配与坐标转换定向测试：`191 passed`；
- 独立覆盖成功点云提交同时推进游标和派生障碍状态，以及未匹配点云不能绕过准备阶段写图；
- 本批不改变合法输入、点云/图像时间匹配、楼层过渡授权、地图分类、检测阈值或安全 Gate，
  未运行 Gazebo；门板观测的任务侧分发和漂移诊断仍留在编排层。

## 18. 第十六批：普通门候选与主动接近

新增 `DoorNavigationRuntime`，独立拥有动态门候选资格、公开可控门筛选、候门点 A* 排序、
确定不可达门的 20 秒冷却账本和 `OpenDoorAndPass` 任务构造。门服务不可用通过冻结
`DoorCandidateDecision.failure_reason` 返回，由编排器记录；预算耗尽仍不加入黑名单。

`MissionManager` 只在探索 tick 中提交当前候选和合法公开门状态，安装返回的门任务并清理探索
目标。旧 `_should_open_door()`、`_unopened_controllable_doors()`、
`_try_select_door_approach()`、`_start_door_approach()` 及管理器内黑名单均已删除；相关测试改为
直接驱动门导航运行时，不保留兼容私有入口。

### 18.1 测试与边界

- `test/` 完整 Python 包：`638 passed`；
- 可见动态门、电梯门拒绝、任务不可抢占、最近可达门、不可达冷却和预算耗尽定向测试：
  `8 passed`；
- 本批不改变门距离、冷却时间、A*、公开门 ID 校验、服务调用或控制发送语义，未运行 Gazebo；
- 主入口过门后的机身净空交接仍是编排器中的独立策略族，后续应整体迁移而非并入本运行时。

## 19. 第十七批：过门上下文与前沿约束

`DoorNavigationRuntime` 进一步成为过门上下文的唯一状态所有者：最近门 ID、门中心、通过方向、
传感器开门确认、完成位姿、临时约束时限和首个前沿诊断不再镜像存放于 `MissionManager` 或
`ExplorationRuntime`。运行时从成功 `OpenDoorAndPass` 的等待点/通过点计算门轴，并独立计算
轴向推进、门后已知面积、上下文完成条件和目标相对门侧。

`MissionManager` 只在门任务成功时提交一次完成事实，并把运行时上下文组装进
`FrontierSelectionContext`；GroundSupport 的受控过门窗口也读取同一确认标志和绝对到期时间。
旧 `_remember_completed_door()`、进度/面积实现、目标侧分类和管理器字段均已删除。地图只在
已启用 `post_door_min_known_area` 时读取，关闭该证明项不会产生无关 Mapper 依赖。

### 19.1 测试与边界

- `test/` 完整 Python 包：`638 passed`；
- 门、过门上下文、前沿约束和 GroundSupport 上下文定向测试：`28 passed`；
- 旧测试改为直接设置/验证 `DoorNavigationRuntime`，不再调用已删除的完成门私有入口；
- 本批不改变门轴算法、距离/面积阈值、临时前沿惩罚、GroundSupport 或控制语义，未运行 Gazebo；
  主入口轴向离门控制仍待补全到 `InitialEntryRuntime`。

## 20. 第十八批：主入口轴向离门子状态机

`InitialEntryRuntime` 现在完整拥有主入口成功过门后的机身净空交接：依据已确认门中心/通过方向
锁存轴向目标，安装直线路径，以受限线速度推进，并判定完成、卡滞或控制器空闲失败。冻结
`InitialEntryClearanceResult` 只返回 handled/failed/target，不写顶层结果。

`MissionManager` 在门任务成功时调用 `schedule_clearance()`，主 tick 调用 `tick_clearance()`；仅当
返回 failed 时记录任务失败并转换到 `WRITE_RESULT`。旧
`_schedule_main_entrance_clearance()`/`_tick_main_entrance_clearance()` 已删除，测试改为直接推进
运行时并验证编排器的失败映射。

### 20.1 测试与边界

- `test/` 完整 Python 包：`638 passed`；
- 编排器、导航任务与运行时边界相关套件：`242 passed`；
- 独立覆盖沿门轴路径、0.12 m 目标容差、线速度上限和 STUCK 立即 fail-closed；
- 本批不改变离门距离、速度、门轴、控制器、安全 Gate 或任务成功判定，未运行 Gazebo；
  主入口进入/通过/离门三段现已由同一 `InitialEntryRuntime` 状态族持有。

## 21. 第十九批：返航异步规划事务

`ReturnHomeRuntime` 现在拥有返航规划的完整异步事务：在主线程冻结 FloorGrid、导航轨迹、失败
首段和目标，创建后台规划器，并把轮询结果收敛为 `ReturnHomePlanPoll`。完成结果会校验当前楼层
和目标；旧楼层/旧目标结果返回 stale 并丢弃，线程异常只以冻结错误摘要返回。

`MissionManager._tick_return_home()` 不再复制地图、构造 lambda/线程或读取规划器内部字段，只按
`pending/stale/error/reached/candidate_rejected/unreachable` 执行顶层控制与终局策略。规划算法仍
位于纯 `return_home_planner.py`，运行时只负责快照和生命周期。

### 21.1 测试与边界

- `test/` 完整 Python 包：`639 passed`；
- 返航运行时、编排器和导航任务相关套件：`230 passed`；
- 新增旧楼层完成结果必须丢弃的直接测试；异步非阻塞、主入口 wait-point、轨迹回退、候选拒绝
  和卡滞退避旧反例均继续通过；
- 本批不改变返航 A*、轨迹候选、失败预算、线程模型、控制器或结果语义，未运行 Gazebo。

## 22. 收尾职责审计与停止边界

本轮迁移后 `MissionManager` 为 3867 行。行数仍较大，但剩余方法已按所有权复核：

- 顶层任务编排：`tick`、`_tick_explore`、`_tick_return_home`、换层发起和楼层完成；
- 事务授权：楼层过渡期间是否允许提交点云、GroundSupport 动作如何映射为任务动作；
- 子状态机结果应用：门/电梯/入场/返航运行时返回值如何安装为当前任务或显式终局；
- 任务语义：状态转换、成功资格、结果原因、预算返航与探索完成的区分；
- 跨组件只读聚合：状态发布、结构化诊断和最终摘要。

以下状态和功能已具有唯一外部所有者：配置装配、ROS 生命周期、合法感知缓冲、时间匹配、
地图/检测提交、楼层坐标、定位健康、GroundSupport 证据、探索预算/无前沿恢复、普通门导航、
门观测、过门上下文、主入口进入/离门、电梯视觉/注册/开门确认/导航几何、返航规划与卡滞恢复、
报告 IO。删除的旧私有接口在生产和测试中均无调用。

因此本阶段停止继续按行数拆分。若把楼层授权、任务终局或跨组件结果应用迁入功能模块，会迫使
子模块反向读取任务状态，形成第二个 `MissionManager`，违反本轮目标。下一阶段入口是固定
`seed=8127` 的单次 Gazebo 闭环验证；运行失败时应按模块公开合同定位，不恢复已删除接口，也不
在编排器重新实现功能。

## 23. GroundSupport 生命周期合同收敛

本轮针对长时测试中重复出现的“约束安装后永久存在、换层与健康事件误增 epoch、起点 veto
按出口重复 A*、临时约束隔离被误写成探索完成”四个根因做替换式修正，不调整安全阈值，也不
运行 Gazebo。

### 23.1 证据与 epoch

- `ConfirmedSupportBatch` 只逐格确认并撤销 `SupportConstraintRegion`，不再调用 Mapper 的
  NEGATIVE 写入或清除接口；部分确认不撤销，全部来源格都由晚于最近失败 witness 的帧确认后
  才原子撤销并递增 constraints revision。
- `GroundSupportCoordinator` 以单一 `RLock` 串行 scope 提交、约束安装、ACK/reset、pending
  batch 和 ShadowSupport 失效。普通换层只切 active floor，不递增 map/localization epoch。
- ShadowSupport 的有状态帧处理也在 scope 临界区内完成，但不持 controller 锁；scope 事务
  必须等待旧帧处理结束并随后清空旧证据，旧帧不能在失效后重新写回。
- 每层 map epoch 只允许显式地图重建入口递增；localization epoch 由 `PoseProvider` 的 odom
  frame identity 变化或明确时间基重置推进，健康布尔变化不再伪造连续性中断。
- ShadowSupport 的时间高水位改为按楼层保存，scope 失效会清相应证据和高水位。
- 同一空间 region 出现更新的失败 witness 时，全部逐格确认集随
  `last_witness_generation` 一起重置；早于新失败的局部确认不能与后续帧拼接撤销约束。
- Gate 的 scope 失效先清旧评估、witness 和 HOLD 上下文，再回到无评估 `CLEAR`；下一条
  非零命令通过正常入口建立带控制时间的 fail-closed HOLD，避免伪造无命令、无进入时间的
  `STALE_DATA_HOLD` 导致永久零速。

### 23.2 规划与楼层完成

- 删除“枚举每个出口并递归完整 A*”路径，改为一次 `(cell, escaped)` 状态化 A*。起点 owner
  只在单调退出前缀内豁免，其他 region 的重叠格始终阻挡；退出后禁止重入。整个搜索共享一份
  迭代预算，认证前缀不做 LOS 压缩。
- Frontier 在同一地图快照上各计算一次 formal/effective 连通域，只有 effective 候选占用有限
  A* 名额。过程性隔离只记录诊断；只有楼层准备结束时 formal 仍有目标、effective 为零、没有
  预算未决且阻挡全部来自 support ledger，才记录无效完成证据并禁止最终 success。
- 楼层终止证书基于当时的 `FloorGrid`、位姿和冻结 constraints 重新计算，不复用较早一次
  frontier 选择的 debug；地图已变化而 constraints revision 未变化时不会误用陈旧证明。
- 新增 AST 边界测试，生产中的 A* 调用必须显式传递冻结 `PlanningConstraints`；临时支撑反馈
  模块不得出现正式 NEGATIVE 地图修改 API。

### 23.3 验证与边界

- GroundSupport、Planner、Frontier、MissionManager、PoseProvider 和 ShadowSupport 定向套件
  全部通过；完整 Python 包 `656 passed`。
- `py_compile`、生产边界、离线隔离、唯一 `/cmd_vel` publisher 和两类 diff check 通过。
- 官方哈希仍只有既有 5 项允许差异：`auto.sh`、`docs/quick-start.md`、`FSM.h`、`FSM.cpp`、
  `gazebo.xacro`；本轮未扩大。
- 本轮未运行 Gazebo，不能据静态测试宣称 seed `8127` 长时二层闭环通过。后续须先独立只读
  复核，再单独授权一次固定 seed 验证。

## 24. 楼层退出证明与来源化 DROP 事务

本轮继续收敛 GroundSupport 到任务编排、规划约束和正式地图之间的所有权边界，不调整 Gate、
DROP、建图或控制阈值，也不恢复单点 BLACK、匿名 NEGATIVE 或开放式 recovery。

### 24.1 楼层退出与重试

- Frontier 新增冻结 `FloorTerminationEvidence`。它在同一次当前地图提取中计算 formal/effective
  连通分量、支撑约束 revision、region IDs 和完全隔离结论；楼层退出不再混用较早的
  `last_selection_debug`。
- `FloorExitCertificate` 对 invalid 采用粘滞语义：同楼层首次无效证书保留，后续结果不能升级
  或覆盖。正常 success 仍要求每个实际探索楼层都有有效证书。
- 支撑失败按空间 region 的 `failure_count` 消费 `maximum_same_phase_replans`：EXPLORE 丢弃
  当前 frontier，关键任务第三次同 region 通过下一任务 tick 显式返回 `FAILED`，RETURN_HOME
  进入既有有界候选拒绝链；约束安装与精确 ACK 始终先于任务层分派。
- MissionManager 只通过 Coordinator 的带锁 Gate transition 和冻结 region 查询入口访问反馈
  状态，测试不再直接修改内部 feedback 或读取 `_regions`。

### 24.2 来源化 DROP

- ShadowSupport 输出完整连通 DROP source cluster；Gate 只锁存 scope、stamp、generation 和
  几何完全匹配的簇。源簇缺失时保持 `SAFETY_BLOCKED`，不能退化为单点 fallback。
- DROP 恢复事务固定为 `controller transaction -> scope lock -> Mapper atomic commit -> clear path
  -> Gate exact ACK/reset`。只有无关键任务的 `EXPLORE_FLOOR`/`RETURN_HOME` 可以提交；门、
  电梯等任务内 DROP 保持锁存并走显式终局。
- `maximum_negative_obstacle_replans` 只统计首次新增簇且 ACK 成功的事务。预算耗尽在 Mapper
  原子提交内拒绝新增并保持地图零修改；全幂等重放不递增 revision 或预算，但仍允许精确 ACK。
- Mapper 批次要求统一 floor/frame/双 epoch/origin/resolution，任一异常整批零修改。正支撑按
  簇跨帧累积且必须晚于 witness generation，完整覆盖全部 source cells 后才撤销 owner；OCC/
  BLACK 仍保持显示优先级，owner 清空时同步移除隐藏 underlay。
- 新楼层 DROP 批次先按 Mapper 固定几何和边界完成结构校验，再允许创建 `FloorGrid`；未知楼层的
  正支撑确认只返回空结果，不会因无效输入惰性创建地图。簇 ID 必须是非空字符串，簇时间戳
  必须为有限数。
- 已删除 `FloorGrid.mark_negative()`、`FloorGrid.clear_negative_with_support()`、单点 BLACK DROP
  返回类型和旧重规划接口；`confirmed_negative_cells` 只保留为只读诊断载荷，生产无消费者。
- 返航完成但楼层退出证书缺失或无效时，结果原因分别写为
  `floor_exit_certificate_missing` / `floor_exit_certificate_invalid`；已有定位、DROP 等更高优先级
  失败原因保持不变，不再出现失败结果仍标记 `return_home_completed`。

### 24.3 验证与剩余边界

- 指定定向套件：`308 passed`，覆盖 Feedback、Gate、Mapper、Frontier 和 MissionManager；
- 完整 Python 测试包：`673 passed`，仅有系统 `Cryptodome` 使用弃用 `imp` 模块的既有告警；
- `py_compile`、`git diff --check`、`git diff --cached --check` 通过，生产/测试均无已删除旧接口
  的调用；
- `tools/offline/check_production_boundary.py` 退出码为 0；唯一 `/cmd_vel` publisher 静态审计仍为
  `navigator.py`，定位模块只订阅该话题；
- 官方文件哈希仍仅有既有 5 项不匹配：`auto.sh`、`docs/quick-start.md`、`FSM.h`、`FSM.cpp`、
  `gazebo.xacro`，本轮未扩大；
- 本轮未运行 Gazebo。上述结果证明 Python 与生产边界静态合同回归闭合；固定 `seed=8127`
  的运行时闭环仍需独立审查后另行授权。

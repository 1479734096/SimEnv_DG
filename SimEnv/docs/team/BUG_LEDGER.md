# 缺陷台账

| ID | 层级 | 结论 | 状态 |
|---|---|---|---|
| ENV-001 | 0 | `PAUSED=false` 全传感器启动触发 `libgazebo_ode` 段错误 | 已规避：官方参数暂停加载后恢复物理 |
| ENV-002 | 0/8.3 | 暂停期 ros_control 初始化与 `spawn_model` 延迟配置竞争，复测出现 calf `+3.586 rad` 与饱和力矩 | 已修复默认启动：延迟配置、最终 calf 角和控制命令首帧完成后才恢复物理 |
| ENV-003 | 8.3 | ROS 接口注册早于 FSM 主循环，条件日志缺失曾被误判为控制器卡死 | 已修复：以 `/unitree_gazebo_servo` 的命令首帧作为无物理依赖的主循环门 |
| ENV-004 | 8.3/10 | 按键 `8` reset 的关节配置、输入锁存与失败恢复存在安全风险 | 基本功能通过、边界待最终收口（不阻断后续阶段）：用户实测 `./auto.sh` 后按 `8` reset 正常；默认 `-2.0` 配置、参数校验、配置失败保持暂停和有界过渡已生效。持续 `RESET` 时过渡推进、真实输入边沿、pause 失败状态、过渡超时降级及证据字段一致性已登记，最终全流程验收前复核 |
| ENV-005 | 9 | 当前 setuptools 接管 distutils 后不识别 catkin 生成的 `--install-layout=deb` | 已规避：安装命令局部设置 `SETUPTOOLS_USE_DISTUTILS=stdlib` 后隔离安装成功；普通 devel-space 构建不受影响 |
| CONTRACT-001 | 1/7 | 旧话题白名单及 MissionManager 接线不区分输入与输出 | 已修复：增加方向校验，正式编排按输入/输出函数接线 |
| RESULT-001 | 1 | 结果写入接受 NaN、Inf、负耗时和字符串数值 | 已修复：有限数校验与原子替换 |
| SENSOR-001 | 2 | 点云读取器声称只返回有限 XYZ，实际会保留 Inf | 已修复：显式有限数过滤与读取上限校验 |
| POSE-001 | 2 | 乱序、非有限或零范数四元数可污染统一位姿历史 | 已修复：入口校验并保持时间严格递增 |
| TIME-001 | 2/7 | ROS 时间异常后回退墙钟会混用时间域 | 已修复：节点初始化后严格使用 ROS 时间，异常显式失败 |
| DATA-TIME-001 | 当前 | 点云或 RGB-D 缺失/非法时间戳时，同一缓存帧会被重复视为新帧；点云还会通过 `PoseProvider.lookup(None)` 反复绑定调用时的最新位姿 | 已修复：唯一接收 generation；非法、零、非有限及不同步时间戳只拒绝一次，PENDING 可重试，不回退最新 pose |
| COORD-001 | 2/5 | `floor_local` 与绝对 z 契约需要实测确认 | 待两层定位验证 |
| LOCALIZATION-001 | 2 | 趴地首帧可错误建立 `base_z≈0.077 m` 的世界系 | 已修复：连续站立证据通过前保持 ready=false 且不发布位姿 |
| SAFETY-001 | 2/4 | 单帧定位差分速度尖峰会导致地面支撑门误停 | 已修复：一秒窗口、速度死区及安全门死区契约；静止仿真速度为零 |
| MAP-001 | 3 | 访问区标记会把 BLACK 安全禁区改写为 VISITED | 已修复：OCC 与 BLACK 均拥有更高优先级 |
| MAP-002 | 当前 | 定位漂移会让远距回波改写已经近距离通过的自由路径，并形成无法返回的假障碍 | 已修复（确定性回归与 seed=8127 G3 长轮地图行为通过，完整闭环因既有下行出厢 CROSS 失败未通过）：按楼层持久保护实际轨迹 `0.25m` 核心带；`>2.0m` 正障碍不覆盖，`<=2.0m` 需 3 代次和 2 个相隔 `0.15m` 的位姿确认；BLACK/NEGATIVE、严格反证和动态门清除不放宽；定位跳变不连接轨迹。长轮完成 0→1、1 层约 25m 旧路返航和 1→0 到达，门关闭后可重新确认、开门后可重复清理；最终失败点不在地图规划链。 |
| MAP-003 | 当前 | `max_range=8m` 人工中止轮仍出现大量疑似假障碍；轨迹保护只能守住 `0.25m` 已走核心带，不能清除带外漂移墙影。Mapper 将 `0.30～1.40m` 高度回波首帧直接记为 `HIGH_HARD`，普通反证射线只处理 LOW 或已覆盖保护带的核心，因此漂移位姿下重复投影出的 HIGH_HARD 会永久累积 | 已暂停，根因链已由代码和运行日志确认，尚未修复：本轮 `hard_core_count` 从 elapsed `89.92s` 的 `3304` 增至结束时 `4347`，`occupied_to_free_count=0`；下一步应让高障碍保持立即阻挡，但允许同帧命中之前的自由射线经多代次、多位姿严格反证撤销旧核心。不得扩大永久 FREE 半径或降低全局近障停车距离。当前 bag 未录 `/nav/map_debug` 和注册点云，无法逐格证明截图中每个黑格的来源，修复前先补只读证据。 |
| PLAN-001 | 3 | A* 可从 OCC/BLACK 起点直接穿出并返回成功路径 | 已修复：UNKNOWN 只允许保守修正，阻塞起点直接不可达 |
| PLAN-002 | 3 | 阻塞目标的“最近”替代格受行优先遍历顺序影响，可能先选对角远格 | 已修复：按欧氏距离和确定性次序选择 |
| PLAN-003 | 3 | 非有限建图/前沿参数和非法 A* 预算可能形成异常或无限搜索 | 已修复：构造与规划入口显式校验 |
| PERF-001 | 3/7 | 默认栅格长墙绕行的无预算 A* 确认约 170 ms，会跨过 10 Hz tick | 已修复：返航穷举在不可变快照后台执行，主 tick 零速轮询 |
| CONTROL-001 | 4 | 路径执行器未区分 `/cmd_vel` 输出与诊断话题方向，非法话题可能延迟到运行期 | 已修复：构造期分别校验唯一输出和内部诊断话题 |
| CONTROL-002 | 4 | 非有限路径、位姿、算法命令或安全门返回值可能进入控制/诊断链 | 已修复：统一零速失效安全；非法位姿清路径并报告 `STUCK` |
| CONTROL-003 | 4 | 单点路径在目标容差外访问第二个点并抛出越界异常 | 已修复：单点前视目标直接使用唯一目标点 |
| SAFETY-002 | 4 | 安全门开启但缺少评估，或收到 NaN 评估时仍可能放行非零速度 | 已修复：清除非法证据并进入 `STALE_DATA_HOLD` |
| SAFETY-003 | 2/4/7 | 定位线速度死区 `0.05 m/s` 大于旧停稳阈值 `0.02 m/s`，且旧编排仍传入冲突默认值 | 已修复：配置显式声明死区、拒绝更小阈值，并同步 MissionManager 默认接线 |
| PERCEPTION-001 | 5 | 同一 RGB-D 帧内多个相邻候选会重复命中同一轨迹，一帧即可满足多帧确认 | 已修复：每次 update 每条轨迹最多关联一个候选 |
| PERCEPTION-002 | 5 | 红球/门板检测缺少完整参数、数组、内参和刚体变换边界校验 | 已修复：非法输入统一返回空观测，非法配置构造失败 |
| TEST-001 | 5 | 旧门板单测用 world `Y=+optical X` 的镜像矩阵，与正式光学转换不一致 | 已修复：夹具改为 world `Y=-optical X` 的右手刚体变换 |
| NAV-001 | 6 | 普通导航把 A* 不可达和预算耗尽都降级为直线路径，可能穿越 OCC/BLACK | 已修复：不可达失败、预算耗尽零速重试，禁止普通直连 |
| BUILD-001 | 6 | 运行时门/电梯几何、服务超时和任务目标可接受 NaN/Inf | 已修复：构造和几何计算入口显式有限性校验 |
| MISSION-001 | 7 | `_elevator_success` 在发起时提前置真、失败后可能保留真值 | 已修复：发起保持未知，仅 DONE 置真、FAILED 置假 |
| FLOOR-001 | 7 | 隐式楼层对齐超时缺少终止路径 | 已修复：停车、取消成功资格并进入 `WRITE_RESULT` |
| SMELL-001 | 7 | 局部 `normal_completion` 名称与实例属性混淆 | 已修复：重命名为成功资格语义，不改变终态计算 |
| STARTUP-001 | 8 | 正式 runner 在定位 ready 前创建任务编排器并提前消耗任务预算 | 已修复：锁存 ready 后才创建 `MissionManager` 和控制发布器 |
| CONFIG-001 | 8 | 旧正式配置可能关闭生产地面支撑门，控制上限也未与停车包络形成强约束 | 已修复：formal 强制开启门控，默认速度不超过已声明包络 |
| RUNTIME-001 | 8/当前 | 正式配准点云属于内部派生输入；旧策略只按 `/danger_search`、`/nav` 前缀信任，无法拒绝改名 relay | 已修复：内部输入、输出和手动控制分别使用精确话题白名单，未知前缀话题构造期拒绝 |
| CONTROL-004 | 8/8.3/当前 | 正式任务已发布非零速度后再切入 RL 模式，实测机器人快速跌倒 | 已修复现有接口范围：formal 默认要求 `controller_ready_ack=false` 并拒绝创建任务；runner 观察到 `Entered RL /cmd_vel mode.` 后才传入确认 |
| CONTROL-TXN-001 | 当前 | 最终发送在 `publish()` 前提交输出状态，DROP 锁存与零速分离，支撑扫掠使用 requested command，且 Gate/keepalive/watchdog/点云/发送出口没有共享事务锁 | 已修复：发布成功后提交、失败锁存零速、active motion 保持至停稳；事务锁和成功命令 generation 覆盖 DROP、keepalive、watchdog 与发送出口 |
| LOCALIZATION-002 | 8/10 | 世界系建立后机器人跌倒，定位仍保持 ready 和固定高度，任务与安全门继续运行 | 已修复并通过 Gazebo：持续使用公开 IMU/楼面证据发布 health；侧倾后撤销 ready，MissionManager 零速进入 `WRITE_RESULT → FAILED`，原因为 `localization_stance_unhealthy` |
| LOCALIZATION-003 | 当前 | G3 Livox-only 将 `max_range` 从 `15m` 改为 `8m` 后仍在重复走廊积累平移误差；8m 只裁剪当前帧，不改变 `12m` 滚动地图、对应点规则或每帧提交行为，无法阻止小幅有偏校正写回同一地图 | 已暂停，根因类别已确认、具体调参项待逐帧证据：8m 轮在 `153.5s/38.24m` 人工结束，RMSE/P95/最大/末端为 `0.1219/0.1668/0.2013/0.1731m`，yaw 最大仅 `0.00463rad`。同机 15m 轮按 `38.24m` 截取为 RMSE/P95/最大/末端 `0.1271/0.1827/0.1973/0.1499m`，8m 只在 RMSE/P95 略优，不能由单轮认定最优或无效。下一步应先用同一原始 bag 跨电脑重放，再按单变量测试 `maximum_map_distance=8m`、`minimum_residual_cutoff=0.06`、`maximum_iterations=8`；不得用命令速度尺度补偿或恢复无证据的共同模式门。 |
| HEALTH-001 | 8.1 | 初版姿态健康接线把楼面高度/法向写成常量，缺失或非有限观测还会保持旧健康值，不能证明依赖真实 `/scan` | 已修复：逐帧估计真实楼面证据，非法/缺失证据按异常帧累计，健康异常时停止发布里程计和配准点云 |
| HEALTH-002 | 8.1 | 初版持续健康角速度上限 `0.60 rad/s` 低于正常控制包络 `1.20 rad/s`，连续转向可能误判跌倒 | 已修复：默认健康上限为 `1.50 rad/s`，接口测试约束其不低于控制上限；倾倒仍由高度和方向证据共同判定 |
| TEST-002 | 8.1 | 初版 Python 测试只调用健康布尔谓词，没有验证停车、失败资格和终态转换；定位陈旧还可能遮蔽显式 health=false | 已修复：覆盖真实副作用及 tick 优先级，并增加 ROS 合成站立→倾倒闭环 |
| NAV-002 | 10.1 | 主入口远场门洞横向误差被固化，且旧流程先走等待点再开门，导致顶门或开门前横向绕行 | 已修复并通过 GUI：远场中心投影到入场轴、原地开门、两帧近场重观测后低速直线过门 |
| NAV-003 | 10.2 | 返航起点落在膨胀 OCC 内时，真实栅格 A* 不可达；轨迹回退把历史 OCC 改写为 VISITED，未校验的贴墙首段被局部层反复判为不可执行，形成活锁 | 已修复：STUCK 反馈记录失败首段并加入有限否决；轨迹候选相对原图校验 OCC/BLACK，同段和候选拒绝均有界，耗尽后零速失败；合成回归及正确起点 GUI 成功返航通过 |
| HEALTH-003 | 10.2 | 主入口约 `7.5 cm` 门槛使持续楼面高度短暂降至 `0.175 m`，低于旧健康下限 `0.18 m`，姿态和楼面法向正常仍被误判跌倒 | 已修复并通过 GUI：持续健康下限改为 `0.14 m`，初始化下限仍为 `0.20 m`；`0.175 m` 通过、趴地 `0.058 m` 失败的测试通过 |
| CONFIG-002 | 10.2/当前 | 此前错误地把 ZIP 默认 `ROBOT_Y=-3.2` 认作错误值，并把当前工作树改为 `-2.2` | 已修复：`auto.sh` 默认恢复 `-3.2`；明确使用 `-2.2` 的旧轮次只算非官方默认条件工程证据 |
| COMPLIANCE-001 | 10.3 | `auto.sh` 默认发布裁判/真值话题且点云转换使用真值 odom；更关键的是官方 `junior_ctrl` 的 RL 仿真观测直接读取 `/ground_truth/base_w` | 待收口：点云转换关闭真值后已有 `laser_livox` 合法局部坐标语义，正式定位直接消费 `/scan`；但关闭全部真值前必须先把 RL 姿态/角速度改接公开 `/trunk_imu` 并完成运动回归，不能只改开关 |
| COMPLIANCE-002 | 当前 | 公开文档和 `team_scene_info_v1` schema 未提供层高，正式算法却使用生成器内部同值常量 `2.6 m` 合成点云、相机、检测和结果的绝对世界 Z | 待收口：按项目严格标准不得回退隐藏默认值；只可使用合法传感器层高证据，质量不足时禁止输出未经证明的绝对楼层 Z 并显式失败或声明能力受限 |
| SAFETY-004 | 10.1/当前 | 已开且由传感器确认的门槛会因支撑 `UNKNOWN` 停车；旧实现按 `TakeElevator` phase 整体关闭 UNKNOWN hold，许可范围过宽 | 已修复：一次性许可绑定传感器确认、区域、方向、速度、位移、有效期、任务实例、楼层及 map/localization epoch；显式 DROP 不可绕过 |
| FRAME-CONTRACT-001 | 当前 | 外部传感器和内部配准点云缺少完整 frame_id 输入契约 | 已修复：Python 缓存与 C++ 定位入口校验固定公开 frame，空/异常 frame fail-closed |
| FRAME-DIAG-001 | 当前 | 定位 odom/配准点云使用 `world`，但地图、路径和局部目标硬编码为 `odom`；ROS 图又没有有效 `world↔odom` TF，RViz 因错误标签产生假错位 | 已修复（离线合同测试通过，Gazebo 待重启后验证）：新增唯一 `localization/global_frame_id`，formal 固定 `world`、relative smoke 固定 `danger_search_relative`，装配层统一注入地图和路径诊断；不添加虚假静态 TF，不改变真实定位漂移 |
| STARTUP-CTRL-001 | 当前 | 普通 formal 入口未确认底盘进入 RL `/cmd_vel` 模式，非零缓存命令可能在切换后立即生效 | 已修复现有接口范围：formal 默认拒绝未确认启动；runner 观察控制器模式日志后才传入确认 |
| AUDIT-CLOSURE-001 | 当前 | Python 离线隔离脚本和局部节点订阅报告被外推为完整生产闭包合规 | 已修复工具范围：静态脚本明确只证明 Python/离线隔离；新增完整 ROS graph 审计。`COMPLIANCE-001` 未关闭时完整审计应失败 |
| ENV-006 | 10.1 | 一次 GUI 调参轮 Gazebo 卡住，用户强制终止，当前无足够日志定位 | 已记录、不阻断；最终固定种子轮未复现，转第 10 层长时间稳定性统计 |
| ENV-007 | 10.1 | 停止 `auto.sh` 时 `junior_ctrl` 偶发在 libtorch 清理阶段异常或段错误 | 已记录、不阻断运行；最终全流程验收结束时复核退出顺序和官方日志 |
| NAV-004 | 当前 | `GO_PASS_POINT` 门洞细对准两处 `0.55 rad/s` 死区下限叠加冲过 `0.10 rad` 窗口，确认计数永远到不了 2；该分支只返回 RUNNING 且 `_pass_path_started` 恒为 False，使 STUCK/IDLE 判定与阻塞超时全部不可达，形成无界活锁（seed 8127 实测主入口原地摆动 395 s，靠外部 kill 结束） | 已修复（单测 712 passed，Gazebo 未验证）：细对准复用控制器 ALIGN 模式（阻尼、SETTLE/CONFIRM、锚点漂移重锚），新增 12 s 有界预算，超时显式 FAILED 并 `logerr`；原任务层自转原样保留为 legacy 分支 |
| NAV-005 | 当前 | 电梯 CROSS 阶段控制器以新鲜位姿判到终点（0.1229 m ≤ 0.15）触发 `goal_reached` 并消费终态清掉路径后返回 DONE，任务层用同一 tick 较旧位姿重算（0.4118 m）判未到达，立即 STUCK → 进轿厢失败 → 不换层。同一容差、不同位姿快照，竞争窗口 0.15 m，0.60 m/s 下仅约 2.5 个 tick | 已修复（单测 712 passed，Gazebo 未验证）：DONE 但完成判定不成立时装填 5 tick 宽限，零速等位姿刷新，用尽才 STUCK 并输出 `endpoint_distance/tolerance/progress/cross_track`；宽限消费置于 `path_lost` 检查之前（路径已被 `goal_reached` 清掉）。`_cabin_transit_position_tolerance` 保持与 HEAD 一致。CROSS 末尾裸 `return status` 仍是静默退出路径，已记录未修 |
| CONTROL-005 | 当前 | `rotate_in_place`/`reverse_straight` 等直接命令依赖任务层每 tick 重发；任务结束、状态机切换或上层异常导致停止刷新后，keepalive 会无限重放最后一条非零命令，机器人持续运动且无人负责停止 | 已修复（单测，Gazebo 未验证）：新增 `direct_command_max_age=0.6`，`_publish` 作为直接命令唯一入口打时间戳，超龄归零；零速继续重放以维持 RL 控制器命令流。未接入 `controller_params`，使用默认值 |
| NAV-006 | 当前 | `_navigate_to_with_status` 在 unreachable/budget/timeout 达上限时经 `_fail_navigation` 返回 FAILED，`POSITION` 分支已正确上抛，但 `ENTER_CABIN`/`EXIT_CABIN` 只判 DONE/STUCK，FAILED 落进函数末尾 `return RUNNING` 被吞。计数器失败时不重置，形成每 tick 触发判决又每 tick 丢弃的零速活锁（seed 8127 实测电梯门外停滞 55.6 s，`controller_mode=IDLE`、指令全零、`cabin_transit_stage` 只到 POSITION）。日志算术：`attempt` 爬到 44，其中 3–44 各出现两次，`target` 为空的记录 42 条 = 界触发 42 次、被丢弃 42 次 | 已修复（单测 716 passed + 负对照 4/4 失败，Gazebo 未验证）：两处各补 `elif status == FAILED` 显式上抛并 `logerr` 输出 stage/failure_reason。定向 AST 审计 4 处疑点：2 处真缺陷已修，1377 裸 `return status`（正确传递）与 1986 `_navigate_to`→bool（7 个调用方均有兜底）确认为假阳性 |
| NAV-007 | 当前 | `_clear_opened_door_area` 此前只在 `_commit_cabin_cross`（ALIGN→CROSS 提交）调用，而 `POSITION` 先于 `ALIGN` 执行，形成顺序死锁：门区膨胀要等 POSITION 完成才清、POSITION 又被没清的门区挡住。起点 (0.773, 2.339) 困在电梯门框膨胀 OCC 内，A* 恒返回 `obstacle_inflation_escape_unreachable`，44 条记录 blocker 完全一致（`active_cell_count: 51`、`start_core_self: false`） | 已修复（单测 716 passed + 负对照，**Gazebo 未验证，仍属假设**）：POSITION 首次规划前调 `_clear_opened_door_area`，一次性标志 `_cabin_transit_position_door_cleared` 保证每次 transit 只清一次（5 处触点齐全）。`clear_opened_door_area` 清理深度本按门两侧等待点加膨胀半径设计，原本就为此场景准备，只是调用点接错阶段。**风险**：51 个膨胀格在门板与固定门框间的归属无法从 `obstacle_owner_cells` 反推；若主要来自固定门框则清门区带不动起点脱困，届时应把 NAV-003 的 `_begin_trace_approach` 脱困机制从 `GO_TO_ELEVATOR_DOOR` 延伸到 `ENTER_CABIN` |
| NAV-008 | 当前 | `TakeElevator.tick` 是 20 个平铺 `if self.phase == ...` 的序列，末尾以裸 `return DONE` 兜底。已知 phase 全集 24 个，其中 `DONE` 与 4 个 `EXIT_BACKOFF_*`（由 `startswith` 统一处理）之外的 19 个各有分支；任何未注册 phase（拼写错误、新增阶段漏登记、空字符串）都会落进兜底并被判为 `DONE`，即**谎报乘梯成功**，任务层据此认为换层完成并推进到下一层。与 NAV-006 同类（确定性结果被宽松兜底吞掉），但危害更重：NAV-006 吞成 RUNNING 是活锁（可观测为停滞），本缺陷吞成 DONE 是假成功（不可观测）。实证：`phase='TYPO_PHASE_NOT_REGISTERED'` 与 `phase=''` 均返回 `done`、`failure_reason=None`、无 stop 命令 | 已修复（单测 754 passed + 负对照：隔离副本还原兜底后判别性测试以 `done != failed` 失败，Gazebo 未验证）：末尾兜底改为强制穷举 —— 仅 `phase == "DONE"` 返回 DONE，其余一律经 `_fail_navigation(context, "elevator_unknown_phase", ...)` 返回 FAILED，并 `logerr` 输出实际 phase 值。新增 3 个回归测试（未注册 phase 判 FAILED、`DONE` 仍判 DONE、phase 注册表清单冻结） |
| NAV-009 | 当前 | seed=8127 G3 r2 下行旧注册几何因平移漂移把真实侧墙纳入出厢方向；后续 post-reboot `15m` 专项又发现目标层局部门洞检测把距机体约 `0.35m` 的轿厢内部结构稳定误认为门框，门中心相对旧值跳 `0.783m`，错误轿厢中心从约 `2.816m` 移到 `3.581m`，POSITION 将机器人送到后墙，斜向 CROSS 再由横向 `0.434m` 的真实侧墙点关闭失败。该短轮定位 RMSE `0.111m`，不能归因于定位漂移 | 修复后待 GUI 复测：保留 3 代次局部几何和当前点云 CROSS 门控；新增站位距离窗口，进厢仅接受 `0.45～1.15m`、出厢仅接受 `0.60～1.30m` 的门框，检测器跳过近处假结构继续搜索真实门平面。没有降低全局近障停车距离，也不读取真值；定向回归覆盖“近假门+远真门” |
| DIAG-001 | 当前 | `_publish_status` 读控制层私有量 `controller._last_goal_distance` 算 `path_length`，而该字段全仓库只有两处出现（声明 `default=math.inf` 与 `_set_path_locked` 重置为 `math.inf`），从无第三个写入点，是死字段。`has_path()` 为真时 `path_length` 恒为 `Infinity`；`json.dumps` 默认 `allow_nan=True` 不抛错而是静默写出裸 `Infinity`，使 `/mission_status` 话题上的载荷成为非法 JSON。同处还读 `controller._recover_until` 算 `is_stuck`。实证：`set_path` + `tick`×3 后 `_last_goal_distance` 仍为 `inf`，`json.dumps(payload, allow_nan=False)` 抛 `ValueError` | 已修复（757 passed + 负对照，Gazebo 未验证）：改用同函数 3810 行已持有的 `tracking_debug_snapshot()` 的 `route_progress`（由 `_remaining_path_distance` 每控制周期真实更新，经 `_finite_or_none` 兜底非有限值）与 `recover_remaining > 0.0`（与 `_recover_until > now` 精确等价）。实测 `inf` → 1.8，严格 JSON 通过。新增 `CrossLayerPrivateAccessTest` 做 AST 全包扫描，禁止状态机层读取 `controller._x`，修复后全仓库该类访问计数为 0。**注意**：该缺陷此前从未在 `mission.log` 中暴露，因为状态载荷只发 ROS 话题不落日志，全日志 `Infinity`/`NaN` 计数为 0 |

`fix.md` SHA-256：`bbaad1824a16bdbd0ba226b0edf268b91a2458e6e4cdb7ef14cbb8eb0c84ccb3`。其中结论必须重新由新仓库测试或代码事实确认后才能关闭。

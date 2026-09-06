# 开发与验证手册

2026-09-05用户单独授权最多三次计次二层闭环，验收与每轮记录见
`reports/AUTONOMOUS_TWO_FLOOR_20260905.md`，不构成长期自动回归许可。
启用定位诊断录包时，从stand/RL切换前开始，包含配准点云、地图、ready和纠偏回执。
源层开门后若地图持续阻塞，应以`elevator_current_entry_blocked_timeout`有界退出。

## 逻辑优先，停止自动回归测试

本手册不再规定或推荐自动测试、回归测试、固定 seed 矩阵、pytest/gtests、测试 runner 或
长时 Gazebo 闭环。以后先理清代码逻辑、调用链、输入输出和失败降级，再直接实施最小范围
修改。测试源码、历史命令和既有运行证据仅供查阅，不作为当前修改门槛；运行时验证如需取得，
由用户另行安排。

## 构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
```

## 官方文件保护

```bash
sha256sum -c OFFICIAL_FILES.sha256
git diff --check
```

## 定位与接口运行约束

单独启动定位节点时必须显式选择正式后端：

```bash
source devel/setup.bash
roslaunch danger_search_localization scan_localization.launch \
  localization_backend:=adaptive_shared_map
```

`livox_imu_local_map` 仅用于固定后端诊断，不是正式默认：

```bash
source devel/setup.bash
roslaunch danger_search_localization scan_localization.launch \
  localization_backend:=livox_imu_local_map
```

该候选只用 `/scan`、`/trunk_imu` 和自身 `/cmd_vel` 做定位；RGB-D 仍由任务节点用于
危险源、门和电梯感知。历史阶段曾记录该候选的失败，正式运行禁止选择它，也禁止根据任务
阶段或实时输入在它与正式 `adaptive_shared_map` 之间自动切换。

正式启动默认使用 `adaptive_shared_map`、`max_range=15m`，并关闭 RGB-D 定位候选；也可用
下面命令独立诊断：

```bash
source devel/setup.bash
roslaunch danger_search_localization scan_localization.launch \
  localization_backend:=adaptive_shared_map
```

该后端使用 `/trunk_imu` 定姿、Livox 维护唯一局部地图并提供平移候选。定位节点默认不
订阅 RGB-D 深度点云；任务节点仍使用 RGB-D 完成危险源、门和电梯感知。Livox 候选无效
时会停止发布新 odom/cloud，调用方必须保留既有陈旧定位和 3 秒不可用门，不能用旧位姿
伪造新时间戳。历史参数对照和 Stage17 seed 结果仅供追溯，不作为当前修改依据。正式默认按
当前配置执行，禁止现场修改 `shared_map_*` 阈值。

需要回退旧后端时必须显式指定，避免把诊断行为误当成正式默认：

```bash
roslaunch danger_search danger_search.launch \
  controller_ready_ack:=true \
  localization_backend:=rgbd_livox
```

该确认只能在 `junior_ctrl` 已输出 `Entered RL /cmd_vel mode.` 后传入。

节点启动后先锁存发布 `ready=false`。机器人站立稳定、连续楼面证据满足门槛后才发布 `ready=true`、里程计和配准点云；调用方不得用固定等待替代该就绪信号。

正式模式强制启用 MissionManager health 门控；诊断模式才允许关闭。定位和接口行为通过逻辑
审阅、配置核对与已有运行日志分析，不再自动执行回归用例。

## 建图与规划运行约束

正式配置默认开启分楼层轨迹保护：`trail_protection_radius=0.25`、
`trail_protection_near_range=2.0`、`trail_protection_confirm_generations=3`、
`trail_protection_confirm_positions=2`。验收时必须同时检查：远距漂移障碍不能堵塞历史轨迹，
近场真实障碍经多代次、多位姿后仍会阻挡，开门清除和严格反证能撤销该阻挡。需要回退旧优先级
时只设置 `map/trail_protection_enabled:=false`，不得联动修改定位、规划或控制参数。

阶段性能测量使用默认 `280×480` 栅格。普通 A* 必须保留迭代预算；`BUDGET_EXHAUSTED` 不等于不可达，不得据此写 BLACK 或前沿黑名单。无预算长墙绕行约 170 ms；第 7 层已要求返航穷举只在地图和历史轨迹快照的后台线程运行，主 tick 在结果就绪前持续零速。

## 路径执行与安全门运行约束

高层普通导航保留地面支撑门。按用户保证，0 层及 `ENTER_CABIN/EXIT_CABIN` 的 `CROSS`
不检测空洞，也不依据显式 DROP 停车；高层 CROSS 结束后必须用新证据恢复检测。
实体障碍、速度包络、定位与站立健康不豁免；换层歧义不能仅凭旧 current_floor=0 放行。

定位默认速度死区是线速度 `0.05 m/s`、角速度 `0.03 rad/s`。`GroundSupportGateConfig` 必须显式保持停稳阈值不小于这两个值；若后续调整定位死区，必须同步审阅动态停车逻辑。

## RGB-D 感知运行约束

感知逻辑必须覆盖合成红球、红色平面干扰、暗色门板、门板消失、双侧滑门开口、稀疏远端深度、同帧多候选和非法刚体变换。`camera_to_world` 必须使用右手刚体变换；正式光学坐标转换是 world `Y=-optical X`，不得使用镜像矩阵。

`floor_local` 模式只允许在上层构造 `camera_to_world` 时增加一次 `floor_index * floor_height`；检测器直接应用传入矩阵，不再自行修正 z。已有两层运行日志用于核对最终结果坐标。

## 建筑交互与导航运行约束

异步协议使用 fake manager 和注入式服务函数建模，不连接真实比赛服务。运行逻辑必须保证：未知 ID/楼层拒绝、授权请求字段保持、门洞/轿厢必须有点云证据、A* 不可达与预算耗尽均不生成直线路径，以及过门/进出厢的对齐和阻塞失败边界。

正式默认要求 3 个不同 Livox 点云代次，最大间隔 `0.5s`；4s 内无法确认时返回
`elevator_open_crossing_geometry_unavailable`，不得回退旧注册中心。目标层位姿距新轿厢中心
超过 `0.80m` 返回 `elevator_cabin_center_offset_exceeded`。当前点云中位于有限穿越走廊内的
实体仍返回 `elevator_crossing_corridor_blocked`；不得因此降低全局近障停车距离。
局部门洞还按 POSITION 站位限制距离：进厢 `0.45～1.15m`、出厢 `0.60～1.30m`。
若近处稳定结构和远处真实门框同时存在，必须跳过近处结构继续搜索距离窗口内的门框；
不能把“三帧稳定”单独当作语义正确。

`use_truth_proximity_detector` 只保留历史诊断兼容，正式 profile 必须为 false。真实 `/set_door_state`、`/call_elevator` 与视觉开口闭环由人工运行时观察。

第 10.1 层主入口运行记录应确认：机器人原地请求开门，近场连续两帧确认净空后再
低速直线通过；首个前沿位于门后侧，离门期间不得出现 `STALE_DATA_HOLD` 或
`ground_support_path_unavailable`。完整证据见 `reports/STAGE10_1_REPORT.md`。

## 任务编排运行约束

返航规划第一次 tick 只创建后台任务并调用 `controller.stop(reason="return_home_plan_pending")`；不能恢复成同步 A*。楼层对齐超时是终止故障，必须进入 `WRITE_RESULT`。电梯摘要保持三态：未完成为 `null`、完成为 `true`、失败为 `false`。

## 运行证据与历史记录

本手册不再安排自动 runner、固定 seed 矩阵或回归复测。已有 Gazebo 日志、bag、参数对照和
阶段报告只作为历史证据，用于在用户提供新运行结果后分析因果链；不得据此自动追加运行、扫
参数或修改前沿、预算、电梯时序等逻辑。

## 第 8 层正式入口与结果闭包

机器人稳定站立后，应先让底盘进入可消费 `/cmd_vel` 的 RL 模式，再启动正式节点：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch danger_search danger_search.launch controller_ready_ack:=true
```

该参数只能在 `junior_ctrl` 已输出 `Entered RL /cmd_vel mode.` 后传入；缺省时
正式入口拒绝创建任务。不得为 formal 模式传入任意结果路径。正式结果文件只能使用
`generated_building/team_scene_info.json` 的 `allowed_interfaces.result_file`；路径不一致时
节点会在建立任何任务控制前显式失败。节点还会等待内部定位 ready，等待期间不创建
`MissionManager`，也不计入任务预算。

禁止在正式任务已经发布非零命令后再按 `6` 切入 RL 模式；标准流程必须先以零命令切入
RL、等待稳定，再启动任务。标准启动流程中，机器狗由 fixed stand (按 2) 切换至 RL /cmd_vel (按 6)
之间必须保留至少 2.0s 的物理接触阻尼等待时间，以平息
四足落地接地冲击。定位节点在姿态门控稳定确认后发布 ready。该流程已在第 8.3 层通过 GUI
移动闭环。跌倒后的失效安全证据保留在历史阶段报告中，不构成当前自动回归要求。

## 第 9 层离线工具

提交或正式验收前执行生产闭包审计：

```bash
python3 tools/offline/check_production_boundary.py
```

只有正式任务和相关 ROS 进程退出后，才可按 `tools/offline/README.md` 调用环境自带的
评分器读取 `danger_truth.json`。评分输出不得被同一轮 MissionManager、定位、感知、
规划或控制读取。

本机若执行 catkin install 时出现 `--install-layout` 不识别，可只对该命令设置：

```bash
SETUPTOOLS_USE_DISTUTILS=stdlib catkin_make install
```

该变量只修正 Python 安装后端选择，不改变生产运行依赖。

## 2026-09-05 接手后的运行前提

本次没有启动仿真或回归，结论见[静态审查报告](../../reports/DANGER_PRIORITY_STATIC_REVIEW_20260905.md)。
风险1/2和后续确定性缺陷已做代码修复；预算仍依赖返航估计，门区定位和实际出梯未验证，
不能据此宣称全链可运行或直接安排下一轮自动runner。

核验参数：`mission/danger_inspection_timeout=30.0`（单次总计）、
`mission/danger_verification_timeout=5.0`、`mission/danger_verification_min_hits=2`、
`mission/danger_verification_min_observation_span=0.08`、
`mission/danger_inspection_retry_cooldown=5.0`、`tracker/inspection_max_attempts=3`。
尝试上限包括规划失败，耗尽后保留正式确认结果，继续普通任务。探索核验准入另预留单次
总上限、接近路线折返时间、返航估计和缓冲；返航途中核验不预扣，仍有单次上限和尝试上限。
定位ready在120秒墙钟未到达时启动失败，释放订阅、不创建任务，不要将此出口记为探索失败闭环。

若用户后续安排人工运行，按同轮证据核对：

1. OCC变格时，核验安全路径的generation/reason保持；真冲突记录
   `danger_obstacle_path_blocked`。也查`approach_idle`，不能只查`approach_path_replaced`。
2. 暂缓预算时DONE/STUCK是否消费、到达记账/恢复是否发生；不能长时间保持terminal_pending。
3. APPROACHING→ALIGNING→VERIFYING及新鲜RGB stamp跨度；失败日志中的attempts和
   retry_exhausted应与最多3次一致，单次超过30秒退出，核验不取消外部恢复。
4. 比较入口绝对odom与离线真值时，不使用已首帧对齐的drift CSV替代绝对误差；同时保留
   初始化前数据、位姿epoch、控制模式时间和stand→RL真实间隔。
5. 同时归档RGB/深度/Livox、配准点云和地图；`record_rgbd=1`只表示意图，须核对文件存在与
   话题清单。轮4没有独立RGB-D bag，旧diagnostic bag也没有注册点云/地图，不能据此证明时空融合。
6. 结果必须同时检查Confirmed+NMS、failure reason、outcome、return_home与楼层退出证书；
   门口/轿厢内失败保存JSON是部分结果，不是完成搜索返航。

新增配置与观察合同：

- `localization/entrance_geometry_apply_correction=false`：默认只观察候选，不等待、不注入。
  `entrance_geometry_correction_evaluated` 的 accepted 不等于 applied；缺少帧会记录 insufficient_frames。
- 危险源候选不再使用 Livox 球心修正。查看限频事件 `danger_perception_summary` 的
  RGB/深度/pose stamp、RGB-D 球心、RMSE、置信度和 Tracker 状态；不要期待
  `rgbd_livox_time_match` 或候选修正前后字段。Livox 仍须按定位、地图和门/电梯用途取证。
- `mission/return_cleanup_timeout=180.0`、`return_cleanup_wall_timeout=600.0`：名义总期限到达
  后停止新核验，完成当前流程再返航；独立上限耗尽则停车写失败部分结果。到家但已超时仍失败，
  不允许通过放宽结果判定掩盖 `mission_timeout`。
- GroundSupport 日志增加 `hole_exemption_reason` 和 `hole_scope_revision`；高层 POSITION、ALIGN、
  EXIT_BACKOFF、普通导航不得因之前的 CROSS 沿用豁免，速度异常锁存不随阶段改变消失。

## 9.5.20.25 后的人工取证约定

本轮结论见[证据报告](../../reports/RUN_20260905_2025_REVIEW.md)。本次未执行仿真或验证脚本。
用户另行安排运行时：

1. 保存 `localization_world_initialized`，核对 scan/IMU stamp、orientation_source、初始 XY/yaw。
   正式路径应为 `trunk_imu_world`；入口默认观察，不能期待 applied 或 epoch 跳变。
2. 观察 `elevator_service_started/finished/timeout`、`elevator_cabin_turn_completed/failed`、
   `elevator_door_open_confirmed`。目标开门请求必须在换层服务成功后发起，不再等调头完成；
   出梯仍要开口和对准均通过。异步不等于同时发起互相冲突的门/换层请求。
3. `danger_inspection_phase` 应呈现 IDLE→APPROACHING→ALIGNING→VERIFYING→IDLE，
   失败进入 IDLE 时保留原因。结合 `danger_perception_summary` 的修正前后坐标、质量及时间戳
   分析跳变；摘要限频且截断，不代表全部候选或完整图像证据。
4. `ELEVATOR_TEST_RECORD_RGBD=1` 现在从任务启动持续录包，默认
   `ELEVATOR_TEST_RGBD_MAX_SIZE_MB=16384`、`ELEVATOR_TEST_RGBD_MAX_WALL_SECONDS=7200`。
   达到任一上限只停 RGB-D 录包，不循环分片或删除旧包；容量可能超出最后一条消息大小，
   墙钟检查有 runner 轮询延迟。`TEST_SUMMARY.txt` 记录 `rgbd_recording_state`；强制停止可能
   留下 `.active`，保留原件。录包启用不代表已经覆盖全程，仍须检查起止时间和退出原因。
5. 漂移 JSON 原有顶层字段继续表示首帧 SE(2) 对齐后的相对漂移；新增 `absolute_world`
   表示原始 world XY/yaw 误差，角度单位弧度。CSV 新增 `absolute_*` 列；不要混用两种指标。
   这些真值比较只能在任务结束后离线进行，结果不得用于当次定位或重写危险源坐标。

### 续审修复与构建记录

主入口诊断时不要把 `entrance_geometry_center_x=0` 理解为机器人定位目标；该参数仅是
初始 yaw 观察合同。确认门洞时检查左右门框、内部中高位占用和扫掠净宽；开门后的地图
清理不应导致新鲜正障碍距离变为 `null`。若净空证据不足，停车并在有限时限内失败降级。

已构建 `scan_localization_node`（未执行任何测试目标）：

```bash
source /opt/ros/noetic/setup.bash
cmake --build build --target scan_localization_node -- -j2
```

以上命令复用本工作区既有 CMake 配置，产物为
`devel/lib/danger_search_localization/scan_localization_node`。本次编译和链接成功，未启动节点。

- 主门请求查看 `door_service_started/finished/timeout`；超时30秒任务时间或180秒墙钟，
  `request_cancelled=false` 表示不能据客户端超时推定门动画已取消。返航不能重新打开失败门。
- 门前对准看 `elevator_door_alignment_timeout` 或 `elevator_door_alignment_retry_exhausted`；
  单阶段12秒任务时间/180秒墙钟，漂移重启最多3次，不因重启而刷新计时。
- 关闭局部穿越几何时，出梯直线预检仅忽略负障碍；应仍能被实际 OCC/BLACK 挡住。
- `TEST_SUMMARY.txt` 同时保存 `rgbd_recording_state` 和 `drift_recording_state`。
  `_forced_stop` 表示使用过 TERM/KILL，`_exit_unconfirmed` 表示仍无法确认退出；不要把这类
  包当作完整证据，也不要删除 `.active`。每个停止过程约12秒内结束轮询，不保证操作系统能
  立即终止不可中断 I/O。旧离线分析工具本身的耗时不包含在这12秒内。

## 主入口等待点

主入口首段目标固定为world坐标`(0,-1.6)`，接近上限`0.44m/s`；这是基于初始生成器
公开的入口原点合同设置的导航目标，不是定位纠偏。远场门洞候选上限为3.0m，中心还须
位于原点0.6m内。提前发现不放行：到点必须停车，只用停车0.5秒后的新点云重新完成两帧
确认，随后才按实时观测中心执行门外1.6m等待、门内0.8m通过和深入。

检查日志应依次出现`initial_entry_observation_path`、`主入口固定观察点已停车`和
`点云连续确认主入口门洞`。固定点接近最多30秒任务时间/180秒墙钟；停车观察每轮最多
5秒，只重试一次且每轮另限60秒墙钟，耗尽后记录失败并返航。电梯厅外等待点0.8m、
轿厢内目标0.95m保持独立，不跟随主入口参数改变。

控制到点容差最多0.12m，停车漂移超0.30m显式失败；这些是合法定位下的判据，不保证
真实机身或足端净距。首次对准结束后开始每轮5秒观察；后续重对准和稳定等待计入该预算，
不会续期或重置次数。路线不可用等待独立计时，首次对准仍有12秒任务时间/180秒墙钟上限。
开放门洞刷新后，检查`main_entrance`的实时等待点、通过点和机器人位姿：横向中心线误差应
不大于0.25m；超限必须先出现重新站位日志，不能直接出现`door_pass_point_path`。刷新时距离
不超过0.30m且横向合格可直接ALIGN；需要重新站位时进入等待点0.25m站位窗后再ALIGN。
整个站位与对准阶段最多30秒任务时间/180秒墙钟。普通门仍保留
原流程，不使用本入口专用中心线门控。

电梯进出厢日志必须依次出现`POSITION`、`ALIGN`、`CROSS`。进梯POSITION目标是门外等待点，
ALIGN目标是轿厢中心；出梯POSITION目标是轿厢中心，ALIGN目标是厅外等待点。不得把电梯门
中心误当朝向目标。POSITION目标距离和横向误差均允许0.25m；之后必须经过ALIGN，不得从
POSITION直接跳到CROSS。对准漂移应退回POSITION，站位与对准总计最多30秒。
首次ALIGN开始后的12秒截止时间不会因重新站位续期，漂移回退最多四次；准备过程另有180秒
墙钟上限。ALIGN仍用原控制器0.10rad进入确认、0.15rad释放；CROSS完成点仍用原0.15m容差。
两类站位路径的控制器到点容差会扣除规划终点偏差；这不是缩小任务站位窗。若A*替代终点
已在窗外，应明确报不可达或`elevator_position_goal_outside_acceptance`，不能反复消耗DONE。
这些是合法定位下的任务容差，不等于机身真实净空保证；门洞误识别和定位漂移仍需分别检查。
主入口门识别现行单侧高度跨度阈值为0.30m（适配Livox 45°俯仰下倾雷达视场），已彻底移除blocked_interior内部占用一票否决；主入口过门阶段已废除1.8m避障排除盒，正障碍检测全量生效，若机身偏航擦碰门板将触发急停保护。
本次只做调用链静态审查、Python语法编译和`git diff --check`，未运行测试、节点或仿真。

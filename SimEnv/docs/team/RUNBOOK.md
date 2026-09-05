# 开发与验证手册

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

普通导航先由安全门确认新鲜、充足支撑；支撑边界不足时当前 tick 和后续保活均发布零速。电梯
`ENTER_CABIN/EXIT_CABIN` 的 `CROSS` 阶段是已确认无空洞的受控门槛通道，允许暂时跳过支撑边界
判定，但速度包络和显式 DROP 仍必须停车，CROSS 结束后恢复检测。

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
风险1/2已做代码修复，但完整预算、CROSS阶段切换/支撑新鲜度及入口纠偏回执边界尚未闭合；
不能据此宣称全链可运行或直接安排下一轮自动runner。

核验参数：`mission/danger_inspection_timeout=30.0`（单次总计）、
`mission/danger_verification_timeout=5.0`、`mission/danger_verification_min_hits=2`、
`mission/danger_verification_min_observation_span=0.08`、
`mission/danger_inspection_retry_cooldown=5.0`、`tracker/inspection_max_attempts=3`。
尝试上限包括规划失败，耗尽后保留正式确认结果，继续普通任务；该上限不构成返航时间预留。
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

# 危险源优先级接手静态审查与历史门区因果分析

日期：2026-09-05。起始 HEAD：`4f81070`（未完成的危险源优先级提升），起始工作树干净。
本报告对应未提交的项目内修改，不代表 Gazebo 验收通过。

## 范围与证据规则

已读 `SimEnv/AGENTS.md`、根 README、团队架构/台账/运行手册、danger_search README
与人工测试说明。仓库顶层没有实体 AGENTS.md、README，本次同时遵守用户提供的顶层约束。
检查了 `HEAD~1..HEAD` 和近期历史，未恢复旧提交、修改测试断言或执行 Git 写操作。
用户指定的 grill-me 文件只要求调用 `Skill("grilling")`；本环境没有对应工具或技能，无法执行
该转调。systematic-debugging 仅用于根因追踪，复现及测试步骤按用户约束跳过。

证据目录前缀为 `/home/xbh/tzb26/evidence/simenv/`，以下四轮均为
`stage35_truth_diagnostic_seed8127_2floor_20260901` 加空后缀、`_2`、`_3`、`_4`。
只读解析了既有 mission.log、auto.log、参数清单、漂移 CSV、诊断 bag 的话题清单及
odom、truth、cmd_vel、rosout 消息；没有回放、启动节点或生成新的运行数据。
真值只用于本次离线分析，未写回任何生产配置、地图或算法缓存。

## 结论与交付

当前无法确认系统能在比赛总时限内完整完成多层搜索和返航。核验抢占的方向合理，但原实现
存在确定性路径失效和终态饥饿。修复了这两条代码路径，同时补齐目标级重试、核验时间证据、
单次核验时限、核验失败的控制权归属，以及定位 ready 无界等待。
门区碰撞、误障碍和完整时间预留仍未闭合，不能据此直接宣布下一轮仿真具备完整验收条件。

| 修改文件（相对 SimEnv） | 改动及解决的问题 |
|---|---|
| `src/danger_search/danger_search/mission_manager.py` | 核验 OCC 更新检查剩余路段；预算暂缓消费控制器终态；抢占复核与装载共享事务；核验总时限及失败归属；规划失败记账和结构化降级日志 |
| `src/danger_search/danger_search/navigator.py` | 增加当前跟踪段的剩余折线快照，包含当前位置到段投影点的接入段，不改跟踪算法 |
| `src/danger_search/danger_search/detection_tracker.py` | 每轨迹最多 3 次尝试；保留 RGB 采集时间，核验按独立时间戳与最小跨度判断 |
| `src/danger_search/danger_search/mission_config.py` | 接线并校验核验时限、时间跨度、尝试上限 |
| `src/danger_search/danger_search/mission_node_runtime.py` | ready 等待最多 120 秒墙钟，超时释放订阅并以失败退出，不创建任务控制器 |
| `src/danger_search/config/danger_search.yaml` | 明确单次核验 30 秒、跨度 0.08 秒、每轨迹 3 次尝试 |
| `docs/team/BUG_LEDGER.md`、`ARCHITECTURE.md`、`RUNBOOK.md` | 同步缺陷状态、控制权、证据与未闭合边界 |
| `src/danger_search/README.md`、`tools/人工测试.md` | 更新核验规则、实际 Livox 选帧语义和人工观察项 |
| 本报告 | 保存审查依据、四轮差异、因果链和剩余风险 |

未调整 Frontier 评分、路径转弯、门/电梯时序、地图反证策略、定位算法、层高或任务预算。
未回退历史优化；保留 fixed stand→RL 的 `sleep 2.0`。

## 风险 1：OCC 更新破坏核验路径

生产顺序是 `tick → _update_mapping_and_detection → commit_image → update_images →
synchronize_confirmed_danger_obstacles → changed_owner_ids → clear_path`。
确认轨迹的加权球心跨栅格、NMS owner 变化或临时保护撤销都能改变 owner 覆盖。
原代码不检查变化是否与路径相交，直接清空路径，递增 path_generation 并清掉 path_reason。
随后 APPROACHING 的 `poll_result` 得到 IDLE，或代次/reason 不符而失败。
因此确属确定性缺陷，但不保证每次都打印 `approach_path_replaced`：IDLE 分支也会记录
`approach_idle` 等原因，人工检查不能只搜索一个字符串。

修改后，普通任务原有 OCC 失效策略保留；活动核验只在当前代次、当前核验路径且没有待消费
终态时检查剩余折线。复用 A* 的线段可通行合同，同时检查 OCC、BLACK、UNKNOWN 和安全约束；
不忽略目标 OCC、不降低膨胀半径。真实阻塞以 `danger_obstacle_path_blocked` 停车失败，
安全路径保持代次。ALIGNING/VERIFYING 不因 OCC 变格撤销对准或驻留；当前机器人危险源
覆盖判定仍优先失败。控制器终态保持给核验消费者处理。

核验失败还修正了另一处归属错误：原来的无条件 `has_path → clear_path` 会清掉接管者的
路径或恢复。现在只取消自己仍拥有的核验路径/对准，不取消 RECOVER 或代次已变化的路径。
这修复代码级自我中断，不证明 RGB-D/Livox 融合位置稳定。

## 风险 2：预算暂缓导致终态无人消费

原循环为：普通前沿到达并锁存 DONE → `terminal_pending=true` → 拒绝危险源抢占 →
软预算动作被 pending 核验推迟 → 提前 return → 尾部 poll_status 不可达。
在这个状态组合持续成立时，每帧重复，直到其他终止条件介入。

把原有探索终态处理原样集中为 `_poll_exploration_controller`，由正常尾部和预算暂缓分支
共同调用：DONE 消费后仍执行到达收益记账，STUCK 仍执行黑名单和受限恢复，IDLE 清理旧意图。
没有把 terminal_pending 当成可抢占状态，也没有只清锁存而丢弃终态语义。
规划前先检查不可抢占状态，避免等待恢复时反复做接近 A*；规划后在控制事务内再次复核再装载。
正式配置仍关闭异步 Frontier；已有异步模式的迟到结果继续按代次失效，不据此宣称异步分支实测通过。

## 风险 3～5与额外等待边界

- 完整核验时间预留仍未修复：仲裁仅使用当前位置返航估计加安全缓冲；没有纳入接近、对准、
  核验、失败后重试和接近点返航/换层的完整耗时。活动核验先于新预算判断返回。
  新增 30 秒单次上限只能限制局部停留，不能保证核验后仍有足够返航时间。
- 原 cooldown 不限制次数。现在每轨迹 3 次，接近位姿缺失或 A* 失败也计一次，5 秒冷却后
  才可重试。A* 预算耗尽只消耗核验尝试，不写不可达黑名单、不改地图。
  耗尽后不再抢占该轨迹，仍保留 Confirmed 结果资格，未冒充 inspected。
- `danger_inspection_failed` 输出 reason、attempts、max_attempts、retry_exhausted，
  可以区分规划、路径替换、真实阻塞、单次超时和证据超时。
- 原核验只统计处理时刻 `hit_times >= since`。现在保留候选 RGB stamp，要求采集与处理
  均严格位于核验开始后、不晚于查询时间，采集时间去重后至少 2 帧且首末相隔至少 0.08 秒。
  5 秒证据等待与 30 秒总时限均有界。正式确认的 4 次/10 秒质量门槛未被弱化。
- 定位 ready 在任务创建前等待，原有 while 只有外部 shutdown 出口，任务 max_runtime 与
  watchdog 尚不存在。现在最多 120 秒墙钟，超时 fatal 并返回非零；不是假写任务完成。

尝试上限按 track_id，不是物理对象身份。严重定位跳变产生新 track_id 时，同一实体仍可能
重新获得额度；这属于关联稳定性风险，未引入新的空间黑名单。

## 四轮主入口对比

下表“首帧”取诊断 bag 中第一份已录制 odom，同 header stamp 匹配真值；并非定位发布的
真正第一帧，初始化之前的变化不在证据覆盖内。z 是定位 floor_local 固定参考，不能把它与
真值瞬时机身高度之差直接当作全轴定位误差。

| 轮次 | 首帧 odom x/y/yaw | 同刻 truth x/y/yaw | 开门后局部门洞中心/宽度 | 结果 |
|---|---|---|---|---|
| 1 | −0.023/−3.199/1.576 | −0.023/−3.189/1.607 | (0.084,−0.117)/1.60m | 主门通过，后在楼层 1 姿态不健康失败 |
| 2 | −0.003/−3.200/1.574 | −0.020/−3.192/1.583 | (−0.204,−0.128)/1.50m | 主门通过，后在楼层 1 姿态不健康失败 |
| 3 | 0.000/−3.200/1.573 | −0.027/−3.199/1.617 | (−0.106,−0.087)/1.55m | 主门通过，55.5 秒人工中止 |
| 4 | 0.412/−3.186/1.708 | −0.003/−3.246/1.706 | (0.096,−0.014)/2.05m | GO_PASS_POINT 撞停，18.5 秒人工中止 |

轮 4 具体因果证据：

1. bag stamp `6.030` 已存在约 `+0.414m` 的 x 绝对定位偏差，真实 yaw 也偏离公开出生 yaw。
   到 stamp `14.026`，odom `(0.055,−0.611)`、truth `(−0.583,−0.624)`，横向差约 `0.638m`。
   控制器追踪的“居中”坐标并不代表机器人真实居中。
2. mission.log:525 首次门洞确认距离只有 `0.60m`，前三轮有记录的轮 2/3 约 `1.34/1.39m`。
   :544/:559 近场刷新通过，但宽度为 `2.05m`、50 个支持点，并出现一次 inconsistent_candidate。
   前三轮刷新距离约 `0.95～1.02m`，宽度 `1.50～1.60m`。多帧通过仅证明自洽，未证明
   检测对象是正确净空，更未校正全局平移偏差。
3. stamp `20.026`，odom `(0.084,−0.245)`、truth `(−0.541,−0.255)`；到 `24.526`
   基本未移动。mission.log:612～676 仍为 `PATH/door_pass_point_path`，Gate=CLEAR，
   无 DROP，forward obstacle 为 null，末次径向最近距离约 `0.348m`。
   bag 末段 cmd_vel 仍约 `vx=0.428～0.429m/s`、`wz=−0.033～−0.038rad/s`。
4. 已坐实“定位横向错位、门洞近场观测不同、持续前进但无真实位移”。具体撞到门扇、门框
   还是门槛，以及哪一束点被过滤，仍不能只靠这些字段判定：没有接触证据和同刻注册地图。
   不能把“碰撞后没有位移”写成“地面支撑清路径”。
5. 中止前最后一次 `stuck_elapsed=4.8s`，并未观察到控制器耗尽全部进度超时后的恢复。
   这轮证明撞停，不能单独证明碰撞恢复必然无限等待。

`analyze_localization_drift_bag.py::_compute_drift` 先用首对位姿消去初始 XY/yaw 差，因此
旧 CSV 首行误差为 0 不代表绝对初始位姿准确。本报告直接对照已录制原始坐标。

## 启动与版本差异

- 四轮 auto.log 的公开出生参数均为 `(0,−3.2,0.6,1.5708)`，calf=−2.0，
  physics=`0.002/500/40`，都有 passive→fixed stand→RL 和 RL /cmd_vel 确认。
  模式日志没有足够时间戳，不能证明每轮 stand→RL 的实际间隔都是 2 秒。
- 轮 1/2 巡航上限 `0.75`、lookahead `1.0`、支撑 lookahead `0.6～1.0`；轮 3/4
  为 `0.65`、`0.85`、`0.5～0.8`。楼层/总预算从 `310/720` 改成 `360/840`。
  轮 3/4 的 mission 参数清单一致，但初始定位和门洞观测明显不同，不能归因于预算变化。
- 轮 1 landmark 漂移诊断开启，其余关闭。四轮定位均为 adaptive_shared_map、15m、
  RGB-D 定位关闭；没有当前入口 SE(2) evaluated/applied 事件。
- 当前 HEAD 及前一提交的提交日期在这些运行之后；没有每轮源文件、二进制哈希和完整工作树
  快照，不能由 Git 提交时间断言轮 4 使用了哪一份未提交强制对齐代码，也不能把其初始偏差
  确定归因于已删除的 initializer 或随机接触冲量。它们需要初始化前证据才能区分。

## 大量疑似误障碍

轮 3 elapsed≈54.8s 的 hard_core_count=3243，末尾 55.3s 为 3256；轮 1/2
在约54.8s分别为2283/2364。所取样本 occupied_to_free_count 都为0。
轨迹与观测覆盖并不相同，数量差不能单独证明多出来的格都是误障碍。

代码支持 MAP-003 的累积机制：较高障碍回波在有效距离内直接写 HIGH_HARD，普通清除
主要针对低障碍/保护轨迹反证，带外高核心不会因缺少新命中自动消失。定位偏差把墙重复投影后，
0.25m 轨迹保护只能保住已走核心带。轮 3 首帧后14秒已有约0.21m绝对 x 偏差，末帧约0.53m，
与墙影扩散相容，但尚无逐格 owner 与同刻注册点云证明每格来源。
四轮 diagnostic bag 都没有 `/nav/map_debug` 或注册点云，不能恢复当时内部地图提交状态。
本轮不修改 Mapper、清障门槛或原始数据。

## 电梯、安全与全局时间的剩余边界

代码确实仅按 `TakeElevator + ENTER_CABIN/EXIT_CABIN + cabin_transit_stage=CROSS`
设置支撑边界豁免，位置对准、普通导航和出梯恢复不满足此布尔条件。
Gate 在 CROSS 时跳过支撑未知/边界，并继续检查速度包络和显式 DROP。
因此“CROSS 不检测空洞”在当前代码中的实际含义是跳过支撑不足推断，并非屏蔽一切显式负障碍。

尚有边界需收口：

- `_sync_ground_support_task_context` 主要在任务 tick 前执行，阶段在任务 tick 内改变时，
  flag 要到后续同步才变化；不能声称 CROSS 退出瞬间已原子恢复全部检测。
- CROSS 放行分支在普通 assessment 新鲜度门之前返回，只靠 `_fresh_drop_blocks` 不能
  证明陈旧支撑帧关闭失败；主循环点云/定位门是另外的保护，不能代替这个局部契约。
- 原阶段等待有导航、门确认、局部几何和恢复超时，但底层 AsyncServiceCall 的实际 RPC
  线程无法由上层超时取消，迟到服务的物理副作用不能按“上层已失败”假定撤销。
- `max_runtime` 只限制尚未启动返航的阶段，已启动 RETURN_HOME/CHANGE_FLOOR 的闭环
  被排除；因此“总比赛时间预算”与代码“探索预算”的含义不一致。本轮未修改这项策略。

这些是后续仿真前应评估的剩余阻断，不能用风险1/2已修复掩盖。

## 感知、坐标与结果闭包

已追踪生产链：同步 RGB-D + 时间匹配 PoseProvider → HSV/轮廓 → 深度反投影 →
固定 R=0.15m 的 RANSAC/几何精炼 → 候选 Livox 距离修正 → Tracker → Confirmed → NMS →
结果。Livox 不独立识别颜色。红方块/绿球的过滤具有颜色与球面几何门，但仍需真实误检证据。

- `MapUpdateRuntime.update_images` 确实取 `recent_cloud_frames[-1]`，缺少按 RGB stamp
  的最近帧选择和时间差门；最后缓存回退也没有时差门。候选改球心后沿用原 RGB-D 球拟合
  RMSE，不能用这个 RMSE 证明被 Livox 修正后的中心仍与原球面一致。PERCEPTION-003 未关闭。
- 正式 Confirmed 门槛仍为10秒4次、平均置信度≥0.75、平均RMSE≤0.04m、位置标准差≤0.25m。
  Confirmed 一旦取得不会因离开视野自动撤销，最终位置是确认轨迹的持续加权估计；
  `inspected` 是导航核验标记，不是最终结果的额外过滤条件。
- 临时双帧语义 OCC 不进入 confirmed_positions，核验失败不会删除已经正式确认的目标。
  失败/外部中止也会保存已有 Confirmed+NMS 结果；文件存在不表示返航成功。
  轮3/4日志实际以 external_ros_shutdown 写出 failed，未假装正常完成。
- floor_local 的世界 z 在适配层加一次层高，当前层高兜底2.6m仍是 COMPLIANCE-002；
  未取得公开或合法感知层高证据前，不能声称多层结果绝对 z 已合规准确。
- 入口 SE(2) 通过合法 PoseProvider 请求/回执修改位姿与局部定位地图，轨迹同步在回执后做。
  `_current_landmark_frames` 过滤楼层/epoch但交给观察器时丢弃各帧 stamp，仅最新云做年龄检查。
  等待观测和等待回执各有3秒计时，不能概括为整个阶段绝不超过3秒；匹配计算耗时另计。
  回执分支构造 robot_xy 只加 dx/dy，未按 yaw 旋转；且丢弃语义同步的 overlap 结果。
  当前纠偏不可视为已经证明闭合，必须先收口坐标与回执边界再用真实数据评价收益。
- 入口固定外墙常量“不读 metadata”只证明文件边界，不等于常量依据已自动合法。
  初始 ZIP 接口文档只公开起点和 ID，不提供门/电梯空间目标；固定几何依据仍需单独记录。

已核对 ZIP 内 `docs/algorithm-interfaces.md`，并搜索正式 Python、C++、launch 与脚本入口：
本次未新增外部输入、订阅或文件读取，禁用字符串命中为接口拒绝清单，未发现本次范围内
参赛节点直接订阅真值/metadata。官方环境内部真值依赖不应与参赛算法直接输入混为一谈。
这不是完整 ROS graph 实测或所有官方文件哈希验收。

## 已执行检查与后续入口

执行了 Git 只读状态/历史/差异、`rg` 调用链与禁用输入搜索、ZIP 文档读取、历史文本/参数/
CSV/bag 消息解析、5个修改 Python 文件的 `ast.parse` 与配置 YAML 语法解析、`git diff --check`。
上述语法与差异检查均完成且未报错，只证明语法/差异格式，不证明实际行为。
没有执行 pytest、gtest、catkin run_tests、构建、lint、仿真、回放、轨迹评估 runner 或浏览器。
没有新增测试文件，也没有宣称测试、编译或运行通过。

下次取得人工运行证据前，应先收口完整核验预算、CROSS 原子恢复与新鲜度、入口纠偏坐标/回执
边界；MAP-003 和 PERCEPTION-003 则需同步传感器和地图证据进一步区分成因。
运行期间同时保存合法位姿、RGB/深度/Livox stamp、定位 epoch、注册点云、地图 owner、
核验阶段/代次与控制命令；若另行授权录真值，仍只用于运行结束后的离线对照。
最终按 result_write_reason、terminal_outcome、return_home、楼层证书与 Confirmed+NMS
共同判读，不能以写出了 JSON 或成功上过一次电梯代替完整闭环。

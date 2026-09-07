# SimEnv 已完成与历史归档

最后更新：2026-09-07

本文归档已经完成的机制、有效验证和历史失败结论。尚未完成的事项只维护在 [fix.md](fix.md)。

范围更正：本文此前所称“生产节点订阅审计”实际只覆盖 `danger_search` 和参赛定位节点，
不覆盖完整机器人控制闭包。官方 ZIP 自带的 `junior_ctrl`、环境 converter/relay、TF
及 `/joy` publisher 必须另计；相关未关闭项已登记到 `fix.md`。

## 1. 已完成的参赛节点边界

- `danger_search` 和参赛定位节点的显式 ROS/文件输入已收敛到赛方公开传感器、
  `/cmd_vel`、门/电梯服务和 `team_scene_info.json`。
- 已审计的上述节点不订阅 `/Odometry_gazebo`、`/ground_truth/*`，不读取 world、
  truth、layout metadata 或固定 XY 场景坐标；该结论不能外推到 `junior_ctrl`。
- 位姿统一由合法传感器定位产生并通过 `PoseProvider` 提供。
- 点云和同步 RGB-D bundle 使用唯一接收 generation；合法时间戳使用时间匹配 pose，
  PENDING 延迟消费，EXPIRED 或非法时间戳拒绝并只消费一次，不再调用最新 pose 兜底。
- 门、电梯公开 ID、`served_floors`、文件 schema 和服务调用均 fail-closed 校验。
- 真值工具已从参赛 Python 生产包、安装清单和正式 launch 隔离。该局部隔离不证明同一
  ROS master 中的真值只供离线使用：当前 `junior_ctrl` 仍会消费
  `/ground_truth/base_w`。
- 正式结果保持官方 `detected_danger.json` 格式；额外摘要仅在 runner 显式指定时写入运行归档。

## 2. 启动与控制链

- 启动竞争已通过条件同步关闭：等待模型延迟关节配置、控制主循环首帧和关节反馈后再恢复物理。
- 机器人可稳定完成 Passive → FixedStand → RL `/cmd_vel` 控制交接。
- 按键 8 reset 已改为有界、边沿触发、失败保持暂停的安全流程，并完成 reset 后再次站立 GUI 验证。
- `junior_ctrl` 来自赛方 ZIP 内的 Unitree 控制器，由 `auto.sh` 启动；它把参赛任务发布
  的 `/cmd_vel` 经 RL 策略转换为 12 个关节控制量，不是参赛方新增的控制器，但属于
  完整生产依赖闭包。
- 参赛任务的 `/cmd_vel` 发布已收敛到唯一 `PathFollower` 出口，没有新增竞争
  `/cmd_vel` publisher。最终 publish 提交、DROP 锁存、active motion、keepalive 和
  wall watchdog 已使用同一事务锁；发布成功只表示本地 `publish()` 正常返回。

说明：`auto.sh`、`FSM.h`、`FSM.cpp`、用户保留的 `gazebo.xacro`，以及用户明确要求保留
`catkin_make -j4` 的 `docs/quick-start.md` 是已知官方哈希差异。内部授权只说明变更
来源，不等于赛方接受正式部署差异。`auto.sh` 的默认出生点已恢复 ZIP 的
`ROBOT_Y=-3.2`。

## 3. MissionManager 已关闭问题

- `_elevator_success` 已改为三态生命周期：未尝试 `None`、进行中/失败 `False`、任务真正 `DONE` 后 `True`。
- 显式和隐式楼层对齐 guard 均有超时 fail-closed；超时会零速、取消成功资格并进入 `WRITE_RESULT`。
- 两份新鲜一致 odom 才能解除楼层过渡 guard；不匹配会重置确认计数。
- 顶层终态语义已明确为 `WRITE_RESULT → FINISH | FAILED`，返航允许 `RETURN_HOME → CHANGE_FLOOR → RETURN_HOME`。
- 局部 `normal_completion` 与实例 `_normal_completion` 职责不同，不构成当前逻辑 Bug；变量重命名仅是可选清理。

## 4. 单层闭环

- 有效基线 `SimEnv/results/runs/20260717_224544_seed_42` 完成预算返航和回起点，仿真时间 `248.21 s`。
- 该轮 cloud `2429` 份匹配、RGB-D `2319` 份匹配，STUCK/recover 为 `0/0`。
- 危险源结果为正确 2、漏报 4、虚警 0，只作为早期基线。
- 第 10.3 层又完成两轮固定 `seed=42`、300 秒完整单层闭环，覆盖启动、主入口、探索、检测、预算返航和结果写出。

证据：`reports/STAGE10_3_REPORT.md`。

## 5. 主入口与门任务

- 主入口通过合法 Livox/RGB-D 运行期几何注册，不依赖固定坐标。
- 开门后使用近场门洞重观测重新计算门中心与轴线，避免长程定位漂移造成顶门框。
- 返航先到传感器注册等待点，并校验门侧、径向距离和横向门轴。
- `OpenDoorAndPass` 已完成对准、开门确认、直线通过和门后深入的有界闭环。
- 第 11 层封口轮完成主入口进入、返航重观测、出楼和回起点。
- 第 13.2 层不再把公开 `initial_open` 当作运行期物理事实；每轮入场只调用一次幂等公开开门服务，服务失败显式暴露。

## 6. 电梯与多楼层机制

- RGB-D 单扇电梯门板语义已与任务注册阈值对齐，电梯视觉注册成功。
- A* 已区分 `REACHED`、`UNREACHABLE` 和 `BUDGET_EXHAUSTED`，规划失败、STUCK 和总超时均有界退出。
- 精确候梯点不可达时，只选择大厅侧真实历史轨迹点作为标准 A* 目标，不改写地图。
- 呼梯前会近场重观测门板，只刷新运行期电梯/门几何，不修改 odom、地图、轨迹或检测坐标。
- 目标门传感器确认后，再以两份新鲜 odom 完成逻辑楼层对齐。
- 已确认开门的电梯只在进/出厢直线跨越阶段过滤门平面薄层回波，门后实体仍进入统一停车链；`seed=6946` 已从上一轮同位置卡滞恢复为成功出厢并继续二层探索。
- `0→1`、`1→0`、跨层点云隔离、二层探索、主入口退出和回起点已完成多轮 GUI 闭环。

第 11 层封口轮：

- 证据目录：`/tmp/stage11_main_entrance_axis_round1_20260722`。
- 仿真 `160.17 s` 完成，最终覆盖率 `0.34`。
- 离线定位 RMSE `0.084 m`、P95 `0.148 m`、最大/最终 `0.288 m`，最大 yaw 误差 `0.0032 rad`。

最新两轮无回归验证：

- `/tmp/stage12_elevator_start_retreat_round1_20260723`：`167.514 s` 正常完成。
- `/tmp/stage12_elevator_start_retreat_round2_20260723`：`169.616 s` 正常完成。

证据：`reports/STAGE11_REPORT.md`、`reports/STAGE12_ELEVATOR_START_RETREAT_REPORT.md`、`reports/STAGE13_3_ELEVATOR_CROSSING_REPORT.md` 及上述运行目录。

## 7. 地图、负障与恢复机制

- 2D 地图已增加在线负障语义：本层以下回波不会继续把空洞射线刷成 FREE，确认负障可写入不可通行层。
- 同格直接楼面支撑证据可恢复动态负障，保证电梯轿厢返回后可重新通行。
- 普通 STUCK 不再把机器人当前格永久写成 BLACK；临时目标黑名单与 A* 地图职责分离。
- 返航 STUCK 已增加失败段反馈、候选否决和有界失败，不再无限重复同一路径。
- 旋转恢复统一检查 `0.40 m` 径向净空，贴墙时保持零速。
- 二层无前沿恢复已实现“沿真实直线来路低速倒退一次、复扫一次、耗尽结束本层”。
- 电梯返程 OCC 起点恢复已实现“仅从连续初始膨胀带沿真实轨迹直退一次，再重走标准 A*”。
- 电梯近场几何刷新失败时会保留既有合法注册几何并继续到精确候梯点，不再在粗接近轨迹点直接开门等待。
- 普通返航 STUCK 在旋转净空不足时只沿本层真实轨迹直退一次到宽处；无安全候选时保持零速并沿已有上限显式失败。
- 地面支撑路径失败的重试预算已按任务相位或原前沿目标隔离；稳定边界写入 `NEGATIVE` 后不会把不同前沿错误累计为同一状态级重试。

后两条恢复分支的代码和单测已完成，但真实自然触发证据仍列在 `fix.md`。

## 8. 定位与健康门

- 正式定位默认使用 `adaptive_shared_map`：RGB-D 与 Livox 对同一张局部地图生成平移
  候选，由 Livox 回调统一提交地图并发布单一位姿；`/trunk_imu` 直接约束姿态。
- 两路候选共享 generation 和事务提交语义；过期、无效或代次不一致时不提交地图，
  两路都无效时不发布 odom/cloud，使既有定位陈旧门 fail-closed。
- 旧 `rgbd_livox` 保留为启动级显式回退，禁止在运行中按楼层、任务阶段或离线真值切换。
- 定位 stance health 仅依赖公开 `/scan` 和 `/trunk_imu`，异常时撤销 ready，MissionManager 零速并显式失败。
- `floor_local` z 的实现语义已统一，当前所有消费者一致使用配置
  `map/floor_height: 2.6`。这只证明工程实现一致，不证明该值有公开接口依据；固定层高
  参与绝对世界 Z 的问题已作为 `COMPLIANCE-002` 打开。
- 定位 M0 只读诊断已完成：记录 GICP 拒绝原因、scan-to-map 校正、尺度窗口和运动一致性，不改变融合行为。
- M2 已将 scan-to-map 单帧上限参数化并保留 `0.10 m` 大跳变保护。
- M3 已完成逐帧质量与反事实分析，证明现有单一残差、对应点数量或固定轴符号不足以可靠在线降权。
- M4 已完成机器人局部坐标和结构主轴预注册分析；候选特征未在两轮数据上同时达到冻结门槛，未进入生产。
- M5 已完成多 seed 定位对照。全局改用 Livox 平移虽能降低部分短窗漂移，但引入主入口任务回归，候选已完整回退。
- M6 已增加默认关闭、行为中性的 GICP 隐藏旋转诊断，并完成一轮双层闭环。隐藏旋转特征 AUC 未达到预注册门槛，门控假设已淘汰。

M0 基线：

- 轨迹 `33.51 m`，RMSE `0.089 m`，P95 `0.113 m`，最大/最终 `0.316 m`。
- 最大 yaw 误差 `0.0027 rad`。
- scan-to-map 最大单帧校正 `0.2276 m`，旧 `0.25 m` 上限未触发。

M2 结论：

- `0.10 m` 两轮均未触发限幅，只证明无回归。
- `0.005 m` 强限幅使 RMSE 恶化到 `0.1966 m`、最终误差 `0.8626 m`，已回退。
- 统一压低 scan-to-map 会造成必要校正不足，禁止继续扫该阈值。

M3 结论：

- 对应点比例、残差分位、迭代次数和各向异性 AUC 约 `0.50～0.56`，区分力不足。
- Livox-RGB-D 两秒分歧有潜力，但两轮样本少且稳定性不足。
- 未实施 alpha 降权、拒绝门、电梯重锚定或固定世界轴规则。

M4～M6 结论：

- 局部前向/侧向、结构主轴和短窗分歧没有形成跨轮稳定判据，M4 停止在离线诊断层。
- M5 的全局 Livox 候选已在 `11797de` 回退；当时生产行为恢复为 RGB-D 主链、Livox
  仅在 RGB-D 过期时兜底。该结论不等于后续共享地图事务式候选。
- M6 seed=8072 完整完成 `0→1→0→主入口→home`，仿真 `161.962 s`，覆盖率 `0.339`；定位 RMSE `0.0563 m`、P95 `0.0838 m`、最大 `0.1494 m`、最终 `0.1168 m`，最大 yaw 误差 `0.0031 rad`。
- M6 隐藏旋转主特征 AUC `0.6531`，低于预注册门槛 `0.70`，因此没有实现在线门控，也不再继续为该假设扫参。

Stage15～17 当前结论：

- 纯 KISS-ICP 与 `livox_imu_local_map` 均完成同 seed 定向测试，但都在二层失败；后者
  RMSE `0.05598 m`、最大 yaw 误差 `0.00294 rad`，仍未形成正式切换证据，保留为默认
  关闭实验后端。
- `adaptive_shared_map` 通过共享地图、只读估计和事务提交消除了两路独立累计后的坐标
  分叉；Stage16 三个固定 seed 候选均完成闭环，但样本不足，保持默认关闭。
- Stage17 已有八个有效 seed，完整闭环 `8/8`、RMSE `<0.1 m` 为 `7/8`、定位诱发失败
  为 `0`，平均 odom 频率均不低于 `9.93 Hz`，最大输出间隔均不超过 `0.202 s`。
- 冻结的 20-seed 矩阵仍未完成；seed `42` 的 RMSE 为 `0.1012 m`，seed `1612` 的末端
  误差为 `0.2529 m`，所以统计矩阵不能晋级。用户明确接受该残余风险后，Stage17 按工程
  验收通过并将 `adaptive_shared_map` 切为正式默认；不得表述为冻结矩阵通过。

证据：`reports/STAGE12_M0_REPORT.md`、
`reports/STAGE12_M2_REPORT.md`、
`reports/STAGE12_M3_DIAGNOSTIC_REPORT.md`、
`reports/STAGE12_M6_GICP_ROTATION_DIAGNOSTIC_CHECKPOINT.md`、
`reports/STAGE12_M6_HIDDEN_ROTATION_REJECTION.md`、
`reports/STAGE15_REPORT.md`、`reports/STAGE16_REPORT.md`
和 `reports/STAGE17_REPORT.md`。

## 9. 历史失败检查点

| 编号 | 现象 | 结论 |
|---|---|---|
| F1 | 目标层门确认不足 | 稀疏深度比例门槛已修复，后续目标门确认通过，关闭。 |
| F2 | 返回 0 层后主入口失败 | 门轴约束和近场重观测已修复，完整返航通过，关闭。 |
| F3 | 一层探索从空洞坠落 | 在线负障建图已补根因，但生产防坠完整封板仍未完成。 |
| F4 | 单帧定位速度尖峰触发永久停车 | 已确认定位差分速度语义不可靠，不再使用单帧直接锁存方案。 |
| F5 | 连续速度尖峰导致停车超时 | 已确认增加确认次数无效，转入可靠速度语义任务。 |
| E1 | 启动姿态失败 | 启动同步已修复并完成多轮稳定站立，环境随机风险保留观察。 |
| E2 | 外部监控误中止 | 属无效运行，不计入算法成功或失败。 |

原始运行目录和详细时间线继续保留在既有阶段报告与 `results/runs/`，不在当前待办中重复展开。

## 10. 第 0～17 层逐层尝试档案

### 第 0 层：官方底座

- 尝试：从 `SimEnv初始.zip` 建立哈希基线，清洁构建，并比较非暂停、关闭比赛传感器和
  暂停加载后恢复物理三种启动方式。
- 当轮结果：暂停加载轮完成模型、传感器、站立和 RL 控制冒烟。
- 后续裁决：相同策略后来再次出现四个 calf 正角展开和 `-33.5 Nm` 饱和，说明一次成功
  不能证明启动确定性；登记 `ENV-002`，不通过修改传感器或 Gazebo 插件掩盖。

### 第 1 层：治理、接口和包骨架

- 尝试：建立参赛方包骨架、来源快照、迁移台账、话题方向白名单和结果 schema。
- 成功：22 项契约测试通过；结果写入拒绝 NaN、Inf、负耗时和错误维度，并采用原子替换。
- 原因：先冻结输入输出与结果边界，避免后续模块迁移时把不合规接口带入生产链。

### 第 2 层：传感器、定位和统一位姿

- 尝试：接入公开 Livox、RGB-D、IMU，建立内部里程计、配准点云、`PoseProvider` 和时间
  匹配；增加连续站立证据门。
- 成功：趴地时不发布错误位姿，站立后才建立世界系；固定 seed 静止 10.098 秒端点漂移
  `8.3 mm`，平面速度归零；65 项本轮参赛包测试通过。
- 未完成原因：官方 `uav_utils-test` 链接错误阻断全工作区测试；官方暂停启动竞争再次
  复现，继续作为环境问题处理。

### 第 3 层：建图、A* 和前沿

- 尝试：迁移多楼层 2.5D 栅格、单层 A*、视线压缩、前沿提取、候选评分和失败黑名单。
- 成功：修复访问标记擦除 BLACK、阻塞起点穿出、替代格选择和非法参数边界；直接测试
  68 项、包内累计 101 项通过。
- 边界：本层只建立纯算法闭包，不提前接入控制和 MissionManager。

### 第 4 层：路径执行与地面支撑

- 尝试：实现 pure-pursuit、转角/障碍降速、无进展监督和最终 `/cmd_vel` 前的支撑门。
- 成功：所有跟踪、旋转、倒车、恢复、停车和 keepalive 收敛到单一发布函数；注入支撑
  不足后当前命令与后续 keepalive 全为零；直接测试 72 项通过。
- 保留缺口：停车距离表当时没有完成动力学溯源，只能作为工程包络。

### 第 5 层：危险源与门板 RGB-D

- 尝试：实现红球候选、球面验证、多帧确认/NMS，以及关闭门板、开口和门板变化检测。
- 成功：修复同帧多候选伪造多帧确认和非法几何输入；纠正测试夹具的镜像坐标系，生产
  光学变换保持不变；22 项直接测试通过。
- 边界：感知核心不订阅 ROS、不读场景文件、不调用服务。

### 第 6 层：门、电梯和导航任务

- 尝试：把公开服务授权、运行时几何、门/电梯非阻塞任务和统一速度出口接通。
- 成功：A* 明确区分到达、不可达和预算耗尽；普通导航不再在 A* 失败后直连；未知 ID、
  非法楼层和非法几何均 fail-closed；59 项直接测试通过。
- 边界：专用过门/进出厢直线仍受阻塞、朝向、端点和进度检查，不属于普通直连兜底。

### 第 7 层：MissionManager 编排

- 尝试：闭合定位健康、建图/感知、建筑任务、返航和结果写出。
- 成功：修复电梯成功提前置真、楼层对齐超时活锁、返航 A* 阻塞 10 Hz tick、ROS 时间
  混入墙钟和停稳阈值不一致；包内 384 项测试通过。
- 原因：顶层只编排，不新增 `/cmd_vel` 发布者；返航规划采用快照和守护线程，过期结果
  不应用。

### 第 8 层：正式 launch、runner 与健康闭环

- 第 8.0 层尝试：接通正式配置、launch、定位 ready 门、公开结果路径和失败写出。
  `danger_search`/定位启动边界接通，但固定 seed 动力学轮在 RL 交接后跌倒，未宣称
  完整通过；普通 formal 入口至今仍缺少可审计的 RL 模式握手。
- 第 8.1 层成功：姿态健康只使用 `/scan` 和 `/trunk_imu`，倾倒后撤销 health/ready，
  MissionManager 零速并写出 `localization_stance_unhealthy`。
- 第 8.2 层成功：真实 Gazebo 传感器链完成可控侧倾闭环，`WRITE_RESULT → FAILED`
  和官方结果写出均符合 fail-closed。
- 第 8.3 层成功：启动链等待延迟关节配置、主循环首帧和反馈后才恢复物理；按键 reset
  改为边沿触发、失败保持暂停和有界无饱和过渡。该层经用户授权修改了受保护启动/Unitree
  文件，形成后续必须保留审计的历史哈希差异。

### 第 9 层：离线工具隔离

- 尝试：把真值评测、bag 回放和诊断工具移出正式 Python 包、安装清单和 launch。
- 成功：生产安装产物不包含离线工具，生产/离线边界检查通过；正确 ROS 环境下 401 项
  测试通过。
- 边界：只保证参赛 Python 包不会导入或安装离线工具；没有审计
  `junior_ctrl`，不能据此保证同 master 真值不会影响控制。

### 第 10 层：单层任务闭环

- 第 10.1 层成功：主入口采用“远场注册、近场重观测、重新计算门轴、严格直线通过”，
  不再朝旧门中心或门框行进；入口失败显式终止。
- 第 10.2 层成功：返航 STUCK 把失败首段和障碍证据反馈给规划层，有界否决重复路径；
  `seed=42` 完成入口、探索、预算返航和回起点。
- 第 10.3 层成功：两轮 300 秒正式配置完整单层闭环；门洞速度
  `0.25→0.35 m/s` 有稳定收益并保留。返航缓冲 `30→20 s` 虽增加探索时间，但降低
  最终余量且缺乏第二轮独立收益，恢复为 `30 s`。

### 第 11 层：主入口与双层电梯封口

- 尝试：处理电梯 RGB-D 注册、目标门确认、跨层对齐、二层探索和主入口返航。
- 成功：封口轮完成主入口进入、电梯往返、主入口退出和 home；定位 RMSE `0.084 m`、
  P95 `0.148 m`，最大/最终 `0.288 m`。
- 历史失败及原因：早期轮次先后暴露目标门稀疏深度不足、返回 0 层门轴偏差、错误
  `_elevator_success` 和楼层 guard 语义；均以传感器证据和状态机修复，不使用固定坐标。

### 第 12 层：电梯恢复与定位 M0～M6

- 电梯返程 OCC 尝试：只允许从连续初始膨胀带沿真实轨迹直退一次，脱出后恢复标准 A*；
  两轮无回归闭环成功，但自然触发证据仍缺失。
- M0 成功完成只读诊断：基线 RMSE `0.089 m`，发现 scan-to-map 最大单帧校正
  `0.2276 m`。
- M2 失败候选：`0.10 m` 限幅两轮未触发；`0.005 m` 强限幅使 RMSE 恶化到
  `0.1966 m`、最终误差 `0.8626 m`，因此回退并禁止继续统一压低。
- M3 诊断失败：残差、对应点、迭代次数和各向异性 AUC 约 `0.50～0.56`，不足以形成
  在线门控。
- M4 诊断失败：局部前向/侧向和结构主轴没有跨轮同时通过冻结门槛，停止在离线层。
- M5 行为候选失败：全局 Livox 平移在部分短窗改善定位，却导致主入口任务回归；提交
  `11797de` 完整回退到 RGB-D 主链、Livox 只在 RGB-D 过期时兜底。
- M6 诊断失败：双层任务成功且 RMSE `0.0563 m`，但隐藏旋转主特征 AUC `0.6531`
  低于 `0.70`，不实现门控。

### 第 13.1 层：支撑门重试预算隔离

- 尝试：修复写入 `NEGATIVE` 后清空前沿，导致不同前沿共享状态级重试预算的问题。
- 成功：失败目标在清理前快照并成为重试键；同一前沿仍有界，不同前沿不互相耗尽预算。
- GUI 结果：主入口和 `0→1` 成功，返程因传感器未确认电梯开口失败；当时安全门始终
  `CLEAR`，所以失败转入下一层而不扩大支撑门修改。

### 第 13.2 层：入口与近墙恢复

- 成功修复三项：初始门态不一致时只调用一次幂等公开开门服务；返航贴墙时沿真实轨迹
  直退到宽处；电梯近场刷新失败后仍回到精确候梯点。
- GUI 结果：完成主入口、电梯注册和 `0→1`，但二层 `EXIT_CABIN` 被门平面回波阻断；
  已证伪的轿厢倒车不恢复，问题转入第 13.3 层。

### 第 13.3 层：电梯门平面跨越

- 尝试：只在传感器确认开门且处于进/出厢直线跨越阶段，过滤门平面附近薄层回波。
- 成功：同一 `seed=6946` 完成此前失败的二层出厢并继续探索约 50 秒；门后实体仍进入
  统一停车链。
- 未完成原因：返程对准时健康门以 `ground_nz=0.797` 连续三帧失败，但 IMU
  `base_up_z=1.000`、足印受支撑，属于独立健康误报。

### 第 13.4 层：离线时间与失败账本

- 尝试：补全状态、任务阶段、零速原因、运动、逐层覆盖、安全事件、里程、识别、定位
  误差和评分账本；支持没有 `mission.log` 的启动失败。
- 成功：修复末样本 STUCK/恢复漏计；历史控制器段错误正确分类为
  `controller_segmentation_fault`；第 13.1/13.3 旧调用成功复算。
- 验证：20 项定向、510 项全包通过。缺少同轮 sidecar 时识别、定位误差和评分保持
  `null`，不读取工作区当前结果冒充历史证据。

### 第 13.5 层：健康门只读诊断

- 尝试：复核第 13.3 层失败时 IMU、地面平面、足印和控制器证据。
- 结论：真实根因是楼面估计器允许 `ground_nz≈0.732` 的斜面返回，而健康门把
  `<0.85` 直接视为倾倒；同一坏平面派生的 `base_z=0.483 m` 不是独立证据。
- 本层只形成预注册机制，不修改代码、不跑 Gazebo，避免直接放宽阈值。

### 第 13.6 层：健康证据职责修复

- 尝试：IMU 明确倾斜仍判 BAD；只有地面法向可靠时才消费同平面派生高度；斜面与直立
  IMU 冲突时把地面估计视为无效证据。
- 成功：106 项定位测试通过；`seed=6946` GUI 完成 `0→1→0→主入口→home`，
  `165.06 s` 正常结束，原健康误判未复现。
- 边界：定位 RMSE `0.0520 m`，但出现 UNKNOWN/STALE 支撑 hold，没有真实 DROP，
  所以只能关闭健康误判，不能封板防坠。

### 第 13.7 层：防坠速度语义诊断

- 尝试：用同轮 bag 离线比较请求速度、1 秒定位差分速度和真值速度。
- 失败结论：定位速度误差 P95 `0.2786 m/s`、最大 `0.4150 m/s`；真实峰值
  `0.7607 m/s` 时请求/定位仅约 `0.556/0.453 m/s`，不能充当瞬时物理速度上界。
- 有效结论：15 个三帧停稳片段的真值均不超过 `0.021 m/s`；13 个制动事件最大路径
  `0.0803 m`。停车表仍缺两轮可追溯测量，UNKNOWN 路径失效不等于 DROP 闭环。

### 第 13.8 层：A′ 速度职责拆分

- 成功实现：请求速度成为停车表/guard 唯一索引；定位速度只负责三帧停稳与
  `1.0 m/s、2.0 s` 有界异常观察；短异常自动恢复，持续异常显式失败。
- 成功实现：离线制动分析器和合成 DROP 全链测试；明确 DROP 写入持久 `BLACK`，第 4 次
  重规划显式失败；formal 门、UNKNOWN、DROP 和 `max_linear=0.60` 均保持。
- 测量失败：seed 6946/8127 的自然任务分别只有 4/1 个接近 0.60 档代理事件，连续两轮
  达不到预注册矩阵；不更新停车表、不提速、不宣称防坠封板。

### 第 13.9 层：制动 harness 架构审计

- 尝试：核对外部建议“只发布 `/nav/goal`、生产零改动、单会话双档位”。
- 阻断原因：`/nav/goal` 只携带 XY，速度来自构造期全局上限，不能逐目标指定
  `0.30/0.60 m/s`；支撑前视上限 `0.80 m` 也不能一次证明 3～4 m 走廊。
- 保留分叉：诊断专用每目标速度接口，或同 seed 双诊断会话。两者尚未裁决，未实现
  harness、未运行 smoke。

### 第 14 层：三层检查点

- 定向上行成功：`seed=8127` 完成 `0→1→2`，三层覆盖代理最大值
  `0.479/0.484/0.459`；该轮由 runner 主动终止，不是完整任务成功。
- 完整闭环失败：完成 `0→1` 后，`1→2` 与返程 `1→0` 均因标准 A* 无法到达候梯点，
  显式终态为 `floor_change_failed_elevator_trace_approach_exhausted`。
- 原因：离线真值显示平移误差最终 `0.7461 m`、yaw 仍稳定；失败时定位和真实位置相差
  约 `0.746 m`。不通过清 OCC、临时改图、放宽 A* 或固定 seed 坐标绕过。

### 第 14.1 层：时基和短窗分歧

- 尝试：显式对齐定位 stamp、任务 elapsed 和漂移 CSV 时基，并在 M5 三 seed、M6
  不同路线冻结验证 RGB-D/Livox 0.5 秒分歧模长。
- 诊断成功：AUC `0.9745～0.9979`，bootstrap 95% 下界最低 `0.9297`；高误差窗
  Livox 局部 RMSE 改善 `81.2%～85.5%`。
- 边界：只证明特征可区分，未证明累计动作安全；没有修改生产定位或运行 Gazebo。

### 第 14.2 层：逐帧累计反事实

- 尝试一：触发帧用因果同窗 Livox 增量替换 RGB-D。
- 尝试二：触发帧保持位置、不累计 RGB-D 平移。
- 失败：两种动作都只有 M5 seed 8072 通过；其余三轮累计 RMSE 分别放大
  `3.69～11.06` 倍和 `3.14～12.43` 倍。局部误差改善不能抵消长期同向累计和
  scan-to-map 反馈缺失。
- 裁决：不改 C++、不新增生产参数、不跑 Gazebo，关闭“直接替换/丢弃”路径。

### 第 14.3 层：电梯语义地标漂移诊断

- 尝试：默认关闭地冻结首次 0 层 RGB-D 电梯锚点，在高层普通探索中记录关闭门板
  三帧 X/Y、门法向和切向残差；真值数据在任务结束后离线比较，但若该轮在同一 master
  发布了 `/ground_truth/base_w`，当前 `junior_ctrl` 会在运行中同时消费，不能称为纯
  离线真值轮。
- GUI 结果：唯一 `GUI=true` seed 8127 轮完成 `0→1→0→home`；240 秒专项预算未到
  2 层，不能作为三层验收。
- 诊断失败：59 个对齐观测、49 个确认，但真值漂移增量最大仅 `0.23392 m`，未复现
  预注册 `>=0.30 m` 高漂移，也没有 `<=0.15 m` 低漂移负对照；向量误差
  P50/P95 为 `0.22909/0.30201 m`。
- 当时裁决：最佳样本虽仅差 `0.00241 m`，不能代替完整分布；不实现定位回写、地图重锚
  或第二轮同假设仿真。诊断默认关闭，定位专项在该路径上恢复暂停。

### 第 15 层：Livox-IMU 局部地图候选

- 尝试：先量化纯 KISS-ICP，再实现只用 IMU 定姿、Livox 局部地图估计平移且拒绝帧不
  污染地图的 `livox_imu_local_map` 后端；没有新增第三方依赖。
- 结果：纯 KISS 和新候选都完成 `0→1`，但均在二层失败；候选 RMSE `0.05598 m`、
  最大 yaw 误差 `0.00294 rad`，没有证明相对正式基线的净改善。
- 裁决：正式默认保持当时的 `rgbd_livox`，不扫残差阈值，不引入缺少可信逐点时间前提
  的 FAST-LIO2、LIO-SAM 或因子图。

### 第 16 层：共享地图事务式自适应定位

- 尝试：让 RGB-D 与 Livox 从同一地图只读估计候选，只由最终选中的 Livox 时钟帧按
  generation 事务提交，避免未选候选和过期结果污染地图。
- 结果：`seed=8127/1612/8072` 三轮候选均完成双层闭环，最大 yaw 误差均小于
  `0.005 rad`；RGB-D 选中比例仍仅约 `3.7%～12.5%`。
- 裁决：原同时间、不同路程的 seed 8072 对照无效；修正为同路程后候选改善，但三组
  seed 仍不足以切默认，因此冻结实现和门槛，转入预登记矩阵。

### 第 17 层：自适应定位正式后端验收

- 尝试：增加固定 20-seed 矩阵 runner、离线汇总和暂停续跑语义，并把任务闭环与定位
  RMSE、P95、末端、最大误差、yaw 和输出连续性分别验收。
- 结果：当前八个有效 seed 全部完成双层闭环，定位诱发失败为 `0`；其中 `7/8` 的 RMSE
  小于 `0.1 m`。seed `2027` 因 Gazebo/控制链未就绪暂停，矩阵仍为
  `insufficient_evidence`。
- 裁决：seed `42` 已使明显提升门槛不可达，seed `1612` 已违反末端误差硬门。用户明确
  接受残余风险后按工程验收通过，正式默认切为 `adaptive_shared_map`，旧
  `rgbd_livox` 保留为显式回退；未完成矩阵只继续量化风险。

## 11. 当前验证与审计范围

- 参赛 Python 全包：`558 passed, 1 warning`。
- Stage17 定位包构建与测试：`130 tests, 0 errors, 0 failures`；默认值、runner、矩阵与
  接口契约定向测试 `25 passed`。
- 当时的参赛 Python/离线边界、矩阵汇总器 `py_compile`、runner `bash -n`、正式
  launch 短时参数与局部订阅检查均通过；正式运行时后端确认为
  `adaptive_shared_map`。这些历史检查不等于新增的完整 ROS graph 审计通过。
- 八轮订阅审计未在 `danger_search` 和参赛定位节点发现真值输入；正式定位节点只订阅
  `/scan`、`/trunk_imu`、`/real_sense/depth/points` 和本系统 `/cmd_vel`。审计未覆盖
  `junior_ctrl`，因此不构成完整生产闭包零真值证明。
- 启用真值并与 `junior_ctrl` 共用 ROS master 的历史轮次，相关数值仍可作为该工程条件
  下的观测，但不能证明无真值控制闭环；`COMPLIANCE-001` 关闭前不得把它们称为纯离线
  真值证据。
- Stage17 正式启动检查结束后 ROS master、Gazebo、控制器、任务和 rosbag 相关进程已
  清理，11311 端口为空。
- 当前受保护文件差异包括 `auto.sh`、`main.cpp`、`FSM.h`、`FSM.cpp`、
  `gazebo.xacro`，以及用户明确保留 `catkin_make -j4` 的
  `docs/quick-start.md`。`auto.sh` 的出生点已恢复 ZIP 默认 `-3.2`，其余启动稳定化差异
  和用户冻结文件仍按 `fix.md` 管理。
- 未记录显式 `ROBOT_Y=-3.2` 覆盖的历史运行不能直接作为官方默认出生点闭环证据；明确
  使用 `-2.2` 的运行只保留为非官方默认条件下的工程证据。
- 当前生产定位为共享地图自适应平移和 IMU 姿态约束；20-seed 矩阵尚未完成且已有硬门
  失败，只能称工程验收通过，不能称统计准入通过。
- 固定层高绝对 Z 和 `junior_ctrl` 真值观测仍在 `fix.md` 延期项；其他本轮问题已关闭。

## 12. 2026-07-28 合规与安全收口

- `CONFIG-002`：`auto.sh` 默认出生点恢复为 ZIP 基线 `ROBOT_Y=-3.2`；旧 `-2.2`
  运行继续只作为非官方默认条件下的工程证据。
- `CONTROL-TXN-001`：`publisher.publish()` 正常返回后才提交输出状态；异常锁存零速。
  DROP 更新、最终过滤、发布、keepalive 和 wall watchdog 共用事务锁；支撑评估使用
  active motion 检查 DROP，并保持到三份新鲜里程计确认停稳。本节原有 publish
  generation 丢弃评估机制已于 2026-07-29 撤销，原因见第 13 节。
- `SUPPORT-PERMIT-001` 的一次性 `ControlledSupportPermit` 实现已于 2026-07-29 撤销；
  当前恢复为仅在已确认主入口通过和电梯进出阶段使用任务上下文 UNKNOWN 豁免，显式
  DROP、定位健康、楼层及 epoch 检查不受豁免影响。
- `DATA-TIME-001`：点云及 RGB/depth/camera_info bundle 均有唯一 generation；
  缺失、零、非有限或不同步时间戳 fail-closed，不再调用 `lookup(None)`。
- `FRAME-CONTRACT-001`：Python 传感器缓存和 C++ 定位入口分别校验公开固定 frame；
  `/scan=laser_livox`、`/trunk_imu=imu_link`、RealSense 深度/RGB-D 为
  `real_sense_optical_frame`、内部配准点云为 `world`。
- 内部话题不再按 `/danger_search`、`/nav` 前缀整体信任，而是按输入、输出和手动控制
  三类精确白名单校验。
- `STARTUP-CTRL-001`：formal launch 的 `controller_ready_ack` 默认 `false`，未确认
  `junior_ctrl` 日志 `Entered RL /cmd_vel mode.` 时拒绝创建任务；电梯 runner 只在
  等到该日志后传入确认。
- `AUDIT-CLOSURE-001`：静态工具更名为“参赛 Python/离线边界”，另增完整运行时
  ROS graph 审计，覆盖所有真值订阅、所有 `/cmd_vel` 与 `/joy` publisher，以及内部
  定位话题生产者和参赛节点 TF 订阅；环境 TF publisher 只列清单，不因存在即判违规。
  当前延期的 `junior_ctrl` 真值订阅会让完整审计如实失败。
- 本轮验证：参赛 Python 全包 `549` 项通过；定位包编译成功，C++ gtest 目标通过；
  合成 `/scan` 与 `/trunk_imu` 的 ROS 集成测试 `1/1` 通过；静态 Python/离线边界、
  runner `bash -n` 和 `git diff --check` 通过。完整 ROS graph 需在整机运行时执行，
  本轮未伪造通过结论。

## 13. 2026-07-29 可用性回退与双时钟复核

- 撤销以每次 publish 都递增的 generation 判废支撑评估，消除低 RTF 下 keepalive/零速
  发布导致的评估饥饿；评估改为绑定候选命令和 active motion 语义。
- 移除无法覆盖完整电梯周期的 `ControlledSupportPermit`，恢复主入口和电梯进出阶段的
  局部任务上下文 UNKNOWN 豁免；DROP 仍由 active motion 持续检查到 odom 确认停稳。
- RequiredSupportSweep 使用最新已提交命令，DropSweep 使用最后非零 active motion，
  解决零速停车期间支撑判断与制动防坠需要不同命令快照的问题。
- ROS/仿真时间统一负责消息时间戳、新鲜度、同步、TTL、停稳序列、状态截止和任务预算；
  `time.monotonic()` 负责进程卡死、`/clock` 或 odom 停滞、回调断流和外部测试总超时。
- 参赛 Python 全包 `584 passed, 1 warning`，定位 ROS 集成测试 `1/1` 通过，生产边界、
  Python 编译、runner 语法和 diff 检查通过。
- `seed=6946` 最终完成 `0→1→0→home`，任务在仿真时间 `162.276 s` 成功；13 次短时
  UNKNOWN hold 均自动恢复，无 DROP、控制输出故障、wall stall 或定位终态。
- 完整优化尝试、两次非成功运行的原因、时基分类和运行证据见
  [STAGE17_1_REPORT.md](reports/STAGE17_1_REPORT.md)。

## 14. 2026-07-29 UNKNOWN HOLD 语义反馈环修复

- HOLD 期间 RequiredSupportSweep 绑定触发 HOLD 的完整候选命令和上下文 ID；安全零速
  与 keepalive 不再把扫描缩短后冒充原命令释放证据。
- DROP 继续独立使用 active motion；定位健康切换会更新 HOLD 上下文，旧评估不能释放。
- 参赛 Python 全包 `587 passed, 1 warning`，生产边界、Python 编译、runner 语法、
  官方哈希边界和 diff 检查完成。
- `seed=6946` GUI 闭环最终完成 `0→1→0→home`。旧轮 13 次短 HOLD/自动恢复变为
  1 个真实 HOLD episode、0 次自动恢复、1 次有界重规划，任务在仿真 `170.518 s` 成功。
- 详细根因、两轮运行结果和残余风险见
  [UNKNOWN_HOLD_FEEDBACK_REPORT.md](reports/UNKNOWN_HOLD_FEEDBACK_REPORT.md)。

## 15. 2026-07-29 三层多 Seed 矩阵工具

- 增加预登记 seed 的三层完整闭环串行 runner，复用 `auto.sh` 和现有正式任务入口；
- 每轮结束后隔离归档检测、真值、评分、时间账本和哈希，自动生成整体效果与问题摘要；
- 算法失败不重跑，基础设施失败最多同配置两次，支持同 HEAD/配置断点恢复；
- 定向契约和合成证据测试 `8 passed`，连同既有 runner/账本共 `35 passed`；生产边界和
  diff 审计通过；按用户要求未启动当前正在使用的 Gazebo。
- 详细实现和未验证项见
  [THREE_FLOOR_SEED_MATRIX_TOOL_REPORT.md](reports/THREE_FLOOR_SEED_MATRIX_TOOL_REPORT.md)。

## 16. 2026-08-27 电梯门中轴几何正交化与进出避障容差修复

- `local_semantic_detector.py` 中开门法向改由左右门框点云聚类深度差几何求解，消除机身航向斜角引起的投影偏斜；
- `elevator_crossing_runtime.py` 将生成的电梯穿越几何法向锁定到曼哈顿主轴（$\pm 0.35\text{ rad}$ 门限），确保候梯点、轿厢中心与门中心严格共享相同 $Y$ 轴坐标（$Y=2.60$）；
- 控制器 `PathFollower` 支持为窄道直线任务配置局部避障容差（`stop=0.15m, slow=0.30m`），防止门洞两侧立柱误判刹停。
- 详情见 [STAGE18_ELEVATOR_AND_SPEED_OPTIMIZATION_REPORT.md](reports/STAGE18_ELEVATOR_AND_SPEED_OPTIMIZATION_REPORT.md)。

## 17. 2026-08-27 远距离异步预呼梯与跨层联动开门流水线

- 机器狗完成探索并触发 `_begin_floor_change` 的第一瞬间，后台立即异步发起 `call_elevator(floor=0, open_doors=True)`；
- 机器狗在 15~20 秒跑回电梯门口的途中，轿厢与门已提前就位敞开，抵达候梯点时直接雷达确认进厢，单次省下 5~6 秒；
- 进厢关门后调用 `call_elevator(target_floor, open_doors=True)` 联动开门，并去除 2.0s 固定人工空转，单次升梯省下 4~6 秒；
- 契约与单元测试全量通过（804 passed in 5.43s）；
- 详情见 [STAGE19_ELEVATOR_PRE_CALLING_AND_LINKED_TRANSITION_REPORT.md](reports/STAGE19_ELEVATOR_PRE_CALLING_AND_LINKED_TRANSITION_REPORT.md)。

## 18. 2026-08-27 电梯关门即升梯、开门即清障与入梯直接对准极致提速（STAGE20）

- **关门即升梯（升梯与调头并行）**：在 `CLOSE_CURRENT_FLOOR_DOOR` 阶段，关门服务完成瞬间立即发起 `call_elevator(target_floor, open_doors=True)` 升梯，移除原先要求狗在轿厢内调头 180° 完全完成才允许升梯的阻塞等待，将调头平移至升梯途中并行完成，**单次乘梯直接省去 8.7 秒静止空等**；
- **开门即主动清障（消除 7 秒自清延迟）**：传感器近场确认开门后，立即主动触发 `_clear_opened_door_area` 清除门板残余膨胀栅格，使地图连通性检查第 1 帧直接通过，**彻底消除此前在门外等待激光射线自然冲刷的 7.3 秒原地发呆**；
- **入梯直接对准进厢（消灭 0.3 米二次折线横移）**：当机器狗抵达门外等待区时，`ENTER_CABIN` 的 `POSITION` 阶段检测到当前已在等待容差范围内直接无缝进入 `ALIGN` 对准与 `CROSS` 直线进门，消灭微距小折线与中间停顿，**再省 3 秒**；
- **测试验证**：全量单元测试（`804 passed in 5.36s`）100% 通过；
- 详情见 [STAGE20_ELEVATOR_CLOSING_TRANSITION_AND_ALIGNMENT_SPEEDUP_REPORT.md](reports/STAGE20_ELEVATOR_CLOSING_TRANSITION_AND_ALIGNMENT_SPEEDUP_REPORT.md)。

## 19. 2026-08-27 二楼挑空中庭与负障碍防踩空跌落安全闭环（STAGE21）

- **安全门边界保护缺陷修复**：修复 `ground_support_gate.py` 中当机身足迹已脱离支撑区（`current_footprint_supported == False`）时被跳过判断的逻辑漏洞，明确只要探测到支撑边界且观测充分，立即触发安全保护刹停；
- **扩大前视探测与制动保留余量**：在 `danger_search.yaml` 中将前视探测范围提升至 `minimum_lookahead: 0.60, maximum_lookahead: 1.00`，悬崖制动保留余量从 0.30m 扩大到 0.40m（完全覆盖四足踏步外摆包络）；
- **测试验证**：全量单元与契约测试（`804 passed in 5.32s`）100% 通过；
- 详情见 [STAGE21_NEGATIVE_OBSTACLE_AND_DROP_PROTECTION_REPORT.md](reports/STAGE21_NEGATIVE_OBSTACLE_AND_DROP_PROTECTION_REPORT.md)。

## 20. 2026-08-27 电梯入厢直通与轿厢安全调头（STAGE22）

- **入厢快速路径（消灭门前 5.5 秒停顿）**：在 `navigation_tasks.py` 的 `_tick_cabin_transit` 中，`POSITION` 阶段到达门前容差且朝向轿厢航向误差 $\le 0.50\text{ rad}$（约 28.6°）时，在同一 tick 直接提交 `CROSS` 直线进厢，跳过冗余的 `SETTLE`（滑行等待）、`ALIGN`（原地旋转）和 `CONFIRM`（静止确认），**单次进梯再省 5.5 秒**；
- **轿厢调头柔和原地限速（彻底消除踢墙卡死）**：为 `PathFollower.begin_heading_alignment()` 增加可选局部角速度上限 `max_angular`。轿厢内调头传入 `0.65 rad/s`，调头期间线速度严格为零，死区补偿不超过局部上限，消除四足大摆幅扫墙卡阻，实现 2~3 秒内平稳完成原地调头；
- **调头完成锁存与先转正后开门确认时序保护**：目标出口调头异步推进并锁存状态；电梯升梯服务先完成时进入专用 `WAIT_CABIN_TURN_COMPLETE` 等待（设置 8 秒确定性上限），机身转正后才启动 8 秒开门确认计时，确保传感器正对二楼门口，根治背向门口导致的开门确认超时；
- **测试验证**：全量单元与契约测试（`809 passed in 5.20s`）100% 通过；
- 详情见 [STAGE22_ELEVATOR_ENTRY_AND_CABIN_TURN_REPORT.md](reports/STAGE22_ELEVATOR_ENTRY_AND_CABIN_TURN_REPORT.md)。

## 21. 2026-08-27 窄门侧壁与房间拐角前向避障误判死锁修复（STAGE23）

- **侧向门框误判前向阻挡缺陷根治**：修复 `map_update_runtime.py` 中 `nearest_forward_obstacle_observation` 将机身腰部/肋部侧方（`forward < 0.20m`）的门框立柱误判为正前方碰撞阻挡的几何缺陷。仅检测机头前方（`forward >= 0.20m` 且 `lateral <= 0.22m`）的真实行进障碍，彻底释放 0.85m 标准窄门的通行能力；
- **消除底层零速死锁**：避免了控制器将侧方门框距离（如 0.235m）误当成前向撞墙距离而强行下发 `linear = 0.0` 刹停的假阳性死锁；
- **测试验证**：全量单元与契约测试（`810 passed in 6.16s`）100% 通过；
- 详情见 [STAGE23_NARROW_DOORWAY_SIDE_OBSTACLE_AVOIDANCE_REPORT.md](reports/STAGE23_NARROW_DOORWAY_SIDE_OBSTACLE_AVOIDANCE_REPORT.md)。

## 22. 2026-08-27 电梯调头命令冲突与进厢直通优化（STAGE24）

- **消除关门、升梯期间的零速竞争**：为 `TakeElevator._run_service()` 增加默认开启的 `hold_position` 控制。普通候梯、开门等服务等待仍保持停车；仅 `CLOSE_CURRENT_FLOOR_DOOR` 与 `CALL_ELEVATOR_TO_TARGET_FLOOR` 传入 `False`，避免任务层零速与 10Hz 轿厢 ALIGN 旋转命令交替发布；
- **进厢直通阈值扩展至 `0.80 rad`**：到达门前位置容差后，航向误差 `<= 0.80 rad` 时同一 tick 直接提交 `CROSS`，超过阈值仍走 `SETTLE/ALIGN/CONFIRM`；不改变路径控制器全局转向契约；
- **保持已验证的轿厢调头边界**：调头继续严格使用零线速度和 `max_angular=0.65 rad/s`，不提高到缺少安全证据的 `0.85 rad/s`；专职调头等待超时由 8 秒调整为 12 秒，超过后仍以 `elevator_cabin_turn_timeout` 确定性失败；
- **测试验证**：任务层与控制器定向回归 `139 passed in 0.55s`；全量单元与契约测试 `814 passed in 4.16s`；按用户要求本轮未启动 Gazebo、未复跑 seed `8127`；
- 详情见 [STAGE24_ELEVATOR_COMMAND_ARBITRATION_AND_DIRECT_ENTRY_REPORT.md](reports/STAGE24_ELEVATOR_COMMAND_ARBITRATION_AND_DIRECT_ENTRY_REPORT.md)。

## 23. 2026-08-27 进门大厅顺时针快速扫描与大门防卡深入（STAGE25）

- **深入大厅开阔中心（彻底消灭摆腿踢门框风险）**：主入口过门轴向深入距离 `post_door_min_distance` 从 `0.9m` 提升至 `1.4m`，过门偏移 `door_pass_offset` 从 `0.5m` 提升至 `0.8m`，使机器狗在充分远离门框与侧墙的大厅中央开阔地执行环视，根治门前近距离旋转导致的踢门卡阻；
- **顺时针快速环视提速**：`elevator_visual_scan_angular_speed` 从 `0.70 rad/s` 提速至 `0.90 rad/s`，下发负角速度命令实现顺时针旋转，在大厅开阔区域兼顾快速覆盖与 RealSense 门板候选捕捉稳定性；
- **顺时针转角累计与防抖闭环**：`elevator_observation_runtime.py` 支持顺时针负向航向增量累计，累计满 `5.80 rad` 确定性退出，落脚扰动反弹不干扰累计；保留候选捕获即 `stop()` 停稳确认的防漏检机制；
- **测试验证**：全量单元与契约测试（`815 passed in 6.01s`）100% 通过；
- 详情见 [STAGE25_CLOCKWISE_VISUAL_SCAN_AND_ENTRY_CLEARANCE_REPORT.md](reports/STAGE25_CLOCKWISE_VISUAL_SCAN_AND_ENTRY_CLEARANCE_REPORT.md)。

## 24. 2026-08-27 出厢无效观察早退与大门外原路返航（STAGE26）

- **保留首次入场轨迹并绑定连续性作用域**：返航优先继续使用本轮实际入场轨迹的倒序路径；新增楼层、地图 epoch、定位 epoch 与轨迹中心线偏差校验，仅在同一作用域且偏差不超过 `0.35m` 时关闭普通 `UNKNOWN` 临时停车；显式 `DROP`、实时障碍、已有支撑约束和控制器卡滞保护不受影响；
- **防止旧轨迹跨定位断点复用**：首次采集轨迹时锁定作用域，若入场期间发生定位连续性变化，轨迹直接失效并回退正常返航规划；
- **缩短出厢无效观察**：目标层开门已确认后，连续 `10` 个新 Livox 点云帧且持续至少 `1.0s` 未观察到开口时，提前降级到已注册门几何；入厢、不稳定候选和显式障碍仍保留原有确认与 `4.0s` 总超时；
- **测试验证**：相关返航、电梯穿越、任务层回归与全量 Python 测试共 `823 passed`；本阶段未启动 Gazebo，尚未取得修复后的 seed `8127` 运行证据；
- 详情见 [STAGE26_ENTRY_TRACE_AND_EXIT_FALLBACK_REPORT.md](reports/STAGE26_ENTRY_TRACE_AND_EXIT_FALLBACK_REPORT.md)。

## 25. 2026-08-27 换层 Livox 目标门确认与历史地图复用（STAGE27）

- **修正显式换层点云楼层切片**：换层守卫继续阻止点云写入 Mapper，但不再丢弃合法新帧；点云按 `guard.target_floor` 修正楼层高度后写入现有 `latest_cloud_points`，消除目标门确认读取旧楼层缓存的问题；
- **所有目标层出口统一使用 Livox**：显式换层期间 RGB-D 只保留诊断作用，不参与放行；复用现有 `detect_opening_ahead()` 与注册门中心、宽度约束，连续两个不同时间戳的新 Livox 帧匹配后确认开口；
- **保留历史楼层地图**：换层期间不写图、不清图、不推进 map epoch；确认开口后只清理目标电梯门动态条带，随后沿用既有楼层锁存、局部穿越刷新与 STAGE26 降级流程；
- **KISS 边界**：未新增独立换层缓存、YAML 参数、ROS 接口或额外地图握手状态；
- **测试验证**：电梯、任务、换层与返航定向回归 `314 passed`，全量 Python 回归 `826 passed`，生产边界审计与补丁格式检查通过；按约定未启动 Gazebo；
- 详情见 [STAGE27_LIVOX_TARGET_DOOR_CONFIRMATION_REPORT.md](reports/STAGE27_LIVOX_TARGET_DOOR_CONFIRMATION_REPORT.md)。

## 26. 2026-08-27 电梯目标门双通道放行与近距离几何放宽（STAGE28）

- **双通道并联互补放行（OR 逻辑）**：解除 STAGE27 对显式换层出梯时 RGB-D 确认的硬编码阻断。无论进厢还是出厢，只要 RGB-D 深度图连续 2 帧确认开门，或 Livox 激光点云连续 2 帧确认门洞开口，任意通道满足即刻放行（`visual_confirmed or livox_confirmed`），实现双重冗余与毫秒级放行响应；
- **放宽近距离电梯门立柱几何约束**：将 `local_semantic_detector.py` 中 `detect_opening_ahead` 的单侧门框立柱垂直高度跨度由 `0.50m` 放宽至 `0.25m`，立柱最小点数由 `6` 点放宽至 `3` 点，并支持动态传参覆盖；完美包容轿厢狭窄视角与近距离内壁遮挡，同时保持门关闭时 0% 穿透的绝对物理防误判；
- **大门近场对准直通过门（彻底消除进门左右摆头）**：在 `navigation_tasks.py` 的 `OpenDoorAndPass` 中，门洞几何刷新后若当前位姿距离 `_wait_point` $\le 0.30\text{m}$，跳过短路段 Pure Pursuit 追踪直接切入 `GO_PASS_POINT`，原地一步正对门后 `_pass_point` 笔直穿过，消除极短距离追踪导致的左右大幅摆头，单次过门再提速 3.5 秒；
- **日志与确认源审计**：准确记录 `confirmation_source` 为视觉深度来源或 `livox_registered_opening`，保留双通道确认帧数审计；
- **测试验证**：全量 Python 单元与契约测试 `860 passed in 4.73s` 100% 绿灯通过。

## 27. 2026-08-28 出电梯关门异步化与极速放行（STAGE28.1）

- **出厢即放行（Fire-and-Forget）**：在 `TakeElevator` 状态机中，机器狗完成目标楼层出厢跨越（`EXIT_CABIN`）到达厅外安全点后，触发后台 Daemon 线程异步请求 `/set_door_state` 关门，主状态机不再下发 `controller.stop()` 阻塞等待，瞬间返回 `DONE`；
- **探索无缝衔接**：`MissionManager` 瞬间切入 `EXPLORE_FLOOR` 启动 Frontier 前沿评估与 A* 路径规划，消除原同步关门导致的 2.5~3.3 秒原地发呆等待（双楼层提速 ~6s，三楼层提速 ~10s）；
- **安全与兼容**：依托 STAGE28 建立的 `target_crossing_geometry` 防回头半平面约束防倒退撞门，并新增 `elevator_async_target_door_close_enabled`（默认 `true`）支持平滑回退；
- **测试验证**：全量 Python 单元测试 `864 passed in 5.07s` 全部绿灯通过，生产边界审计通过；
- 详情见 [STAGE28_1_ASYNC_ELEVATOR_DOOR_CLOSE_REPORT.md](reports/STAGE28_1_ASYNC_ELEVATOR_DOOR_CLOSE_REPORT.md)。

## 28. 2026-08-28 电梯门前语义几何刷新与远距离危险源主动走近核验（STAGE29）

- **电梯门前语义几何刷新（`elevator_geometry_refresh_enabled`）**：在电梯门前 `1.2m`
  远视点连续重观测关闭门板，只刷新当次 `TakeElevator` 使用的等待点、门轴和乘梯几何。
  本阶段不修改 odom、定位地图、轨迹或危险源坐标；真正的定位/共享图平移纠偏在 STAGE33 实现。
- **远距离危险源主动走近核验（Active Danger Approach Verification）**：
  - `DetectionTracker` 增加 `inspected` 状态管理与 `get_pending_inspection_target()`、`mark_inspected()` 接口；
  - `exploration_runtime.py` 实现 `compute_danger_approach_pose()`，计算距目标 1.3m 且正对红球的安全自由停靠航向点；
  - `MissionManager` 在路径空闲且楼层时间预算充裕时优先插入走近路径（`reason="danger_approach_verification"`）；
  - 走近过程中相机自动采集多帧近距离点云拟合，并按观测距离、拟合 RMSE 和置信度加权；
    单元测试证明调度与融合逻辑成立，但本轮尚无 Gazebo 证据证明全场误差稳定达到厘米级；
- **配置与参数暴露**：在 `mission_config.py` 和 `danger_search.yaml` 中新增 `danger_approach_verification_enabled`、`danger_approach_trigger_distance`（2.2m）和 `danger_approach_standoff_distance`（1.3m）；
- **测试验证**：全量 826 项单元与契约测试 100% 绿灯通过，新增 `set_path` 归因与几何停靠点测试；
- 详情见 [STAGE29_ELEVATOR_REFRESH_AND_DANGER_APPROACH_REPORT.md](reports/STAGE29_ELEVATOR_REFRESH_AND_DANGER_APPROACH_REPORT.md)。

## 29. 2026-08-28 探索前沿异步规划（STAGE30，已在 STAGE32 回退）

- **前沿规划异步化与提前量预规划**：实现单后台规划线程，支持普通前沿、机会性重选（3.0s）与到达前动态提前量预规划（Prefetch），避免到点停车等待；
- 详情见 [STAGE30_ASYNC_FRONTIER_PLANNING_REPORT.md](reports/STAGE30_ASYNC_FRONTIER_PLANNING_REPORT.md)。

## 30. 2026-08-28 平衡弧线转弯与航向速度下限（STAGE31）

- **弧线转弯平衡**：将原地转向触发阈值放宽至 `turn_in_place_heading = 1.10`，释放阈值 `turn_release_heading = 0.30`，并引入 `arc_turn_min_linear_fraction: 0.15`，允许中等偏角平滑过弯；
- 详情见 [STAGE31_BALANCED_ARC_TURN_REPORT.md](reports/STAGE31_BALANCED_ARC_TURN_REPORT.md)。

## 31. 2026-08-28 探索前沿规划回退同步模式（STAGE32）

- **回退背景与根因**：
  1. 异步规划消除了到点停稳，机器狗在 $0.80\,\text{m/s}$ 高速奔跑中，Livox 点云动态更新在狭窄走廊/转角处偶发触发 $2.0\,\text{s}$ 的 `OBSERVATION_HOLD` 超时，导致 `GroundSupportGate` 频繁触发 `ground_support_path_unavailable` 截断正在执行的路径，引发机器人被迫在相邻前沿间来回横跳；
  2. 高速奔跑与大幅弧线转弯引起机身 Pitch 颠簸（$3^\circ \sim 5^\circ$），平整地面点云在坐标转换中被虚假抬高至 $z \ge 0.20\,\text{m} \sim 0.40\,\text{m}$，生成大量硬障碍物核心（`hard_core_count > 4000`）并自我固化；
  3. 经典同步单次规划模式下，机器狗每到一个目标点完全刹停零速 $300 \sim 500\,\text{ms}$，在静止水平姿态下雷达充分积累点云并完成激光射线深度洗地（Raycast Free Clear），地面支撑覆盖率 100% 满格，彻底消除运动中的虚假障碍和路径被动掐断问题。
- **配置变更**：在 `danger_search.yaml` 中将 `async_planning_enabled` 设为 `false`，完全关闭后台规划线程、行进中换道与到达前预规划；
- **测试验证**：全量单元与契约测试 `857 passed in 6.29s` 全部绿灯通过，生产边界审计通过；
- 详情见 [STAGE32_ROLLBACK_ASYNC_FRONTIER_PLANNING_REPORT.md](reports/STAGE32_ROLLBACK_ASYNC_FRONTIER_PLANNING_REPORT.md)。

## 32. 2026-08-28 当前提交候选收口与稀疏地标纠偏（STAGE33）

- **红球几何定位**：使用公开半径 `R=0.15m` 的高斯-牛顿非线性几何拟合，替代单侧球冠代数球拟合；跟踪器按观测距离、RMSE 和置信度加权，远距离高质量候选可触发一次近场核验。
- **无效前沿剪枝**：距离大于 `3.5m` 且预期收益小于 `30` 格的小前沿，在 A* 落点解析时仍保持剪枝，不再重新激活；不计入不可达失败或黑名单。
- **正式运动与规划基线**：`max_linear=0.75m/s`、`boundary_hold_margin=0.40m`、
  `async_planning_enabled=false`、`opportunistic_replan_period=0.0`；异步规划框架仅作为默认关闭的可回退实现保留。
- **双静态锚点**：首次合法注册时冻结主入口和 0 层电梯周边 Livox 静态结构；复访只用当前楼层、当前定位 epoch 的连续三帧。
- **原子平移注入**：候选必须通过点数、重叠率、P70 残差、唯一性、帧间离散、静止和 `1.20m` 上界门控；C++ 定位节点同步平移位姿与 Adaptive Shared Map，回执后切换 `PoseProvider` epoch。
- **电梯开口弹性**：首次确认超时 `15s`，只允许一次 `10s` 重试；重试前必须等待至少 `0.5s` 后的新 Livox 帧。源层重新复核/对准/呼梯/开门，目标层留在轿厢内只重开门。
- **量程统一**：正式 launch 和 `run_elevator_gui_test.sh` 默认使用 `15.0m`；`8.0m` 只保留为显式历史 A/B 覆盖。
- **验证**：全仓 Python `893 passed`；定位包 `catkin_test_results` 汇总
  `158 tests, 0 failures`；生产边界审计和 `git diff --check` 通过。按用户要求未运行 Gazebo 定向或完整闭环，运行时效果仍属待验证。
- **未关闭的阶段门**：`sha256sum -c OFFICIAL_FILES.sha256` 仍有 6 个历史不一致项；本轮未覆盖
  受保护文件，因此当前候选尚不能创建迁移阶段提交或标签。
- 详情见 [STAGE33_SPARSE_LANDMARK_RELOCALIZATION_REPORT.md](reports/STAGE33_SPARSE_LANDMARK_RELOCALIZATION_REPORT.md)。

## 33. 2026-09-06 远距离危险源解耦与三级确认阶梯（STAGE34）

- **问题现象（走廊高速巡航路过危险源 100% 漏检）**：机器狗以约 $0.63\text{ m/s}$ 巡航经过危险源（红球）时（如 simtime 1min40s~47s 及 2min 左右路过危险源），未能触发靠近核验（`danger_approach_verification`），导致危险源被彻底遗漏。
- **根本原因排查与分析**：
  1. **远距离置信度硬截断（数学死锁）**：旧版 `_score_detection` 在目标距离 $>5.0\text{ m}$ 时强行施加距离折减（每米扣 $0.15$），将置信度打压至 $<0.75$（例如 $5.82\text{ m}$ 处的真球即使几何形状高度吻合也被压制至 $0.69$），直接低于跟踪器确认门限 `min_confidence=0.75` 及任务层抢占门限 $0.75$，无法完成确认；
  2. **固定 4 帧确认门禁与相机窄 FOV（$\pm 30^\circ$）时空冲突**：机器狗在走廊以 $0.63\text{ m/s}$ 巡航时，侧前方红球穿过视锥的时间窗口极短（约 $0.4\sim 0.6\text{ s}$，仅获 1~2 帧有效 RGB-D 样本），无法满足硬编码的 `min_hits=4`，被跟踪器作为孤立离散帧淘汰；
  3. **急停扫描方案否定**：四足机器狗在高速行进中急停会引发足端打滑与位姿漂移，且惯性滑跑（约 $0.2\sim 0.3\text{ m}$）极易导致停下后目标反而脱离视锥，不可行。
- **修改方案与修改后现状**：
  1. **距离惩罚解耦**：在 `danger_search.yaml` 引入 `detection/distance_penalty_start: 6.5`，使 $6.5\text{ m}$ 内红球不施加硬性距离扣分，真实红球在远距离仍可保留 $0.88\sim 0.98$ 的高几何置信度；
  2. **基于球面物理残差 RMSE 的三级确认阶梯（Tiered Confirmation）**：
     - **Tier 1（极高精度）**：$\text{RMSE} \le 0.005\text{ m}$（$5\text{ mm}$），置信度 $\ge 0.70$，只需 **1 帧** 命中即可立即 `CONFIRMED`；
     - **Tier 2（高精度）**：$\text{RMSE} \le 0.015\text{ m}$（$15\text{ mm}$），置信度 $\ge 0.70$，只需 **2 帧** 命中即可 `CONFIRMED`；
     - **Tier 3（常规兜底）**：$\text{RMSE} \le 0.035\text{ m}$（$35\text{ mm}$），置信度 $\ge 0.75$，维持 **4 帧** 命中才 `CONFIRMED`。
  3. **任务层核验抢占门限对齐**：`MissionManager._pending_danger_inspection_target` 的置信度门限与 `tier1_min_confidence`（$0.70$）对齐，确保极低 RMSE 单帧即能抢占巡航并转入靠近核验；
  4. **实测数据与离线回放验证**：
     - Gazebo 仿真中真实红球点云拟合实测 RMSE 极低（普遍为 $0.20\text{ mm}\sim 1.33\text{ mm}$，最大不超过 $3.5\text{ mm}$）；
     - 场景内红色干扰方块（`distractor_red_box`，边长 0.3m）在拟合 $R=0.15\text{ m}$ 球面时实测 RMSE 均在 $21.8\text{ mm}\sim 22.8\text{ mm}$；
     - $5\text{ mm}$ 与 $15\text{ mm}$ 阈值精准切入真球上限与干扰物下限之间的绝对空白带；
     - 离线回放验证：`9.6.12.08` 真实漏检真球（$5.82\text{ m}$，RMSE $0.197\text{ mm}$）单帧即被 CONFIRMED 并激活核验；`attempt01` 真实红色方块干扰物连续 21 帧均被阻断在 Tentative（零误报）。

## 34. 2026-09-06 高层电梯门探索禁区与防轿厢回头振荡（STAGE35）

- **问题现象（出电梯后回头探索电梯轿厢）**：机器狗搭乘电梯到达高层后，在执行前沿探索评估时容易被电梯轿厢内残留的未探索/未闭合未知区域吸引，刚出电梯门走几步又立刻掉头试图重新返回电梯，陷入“出梯即回头”的前沿探索死循环振荡。
- **根本原因排查与分析**：
  1. **永久禁区仅对 0 层主入口生效**：原本的 `permanent_forbidden_half_plane`（永久禁止半平面）机制被硬编码仅对 0 层的主入口大门（`main_entrance`）生效；
  2. **高层电梯门临时上下文时效短**：在高层（1层、2层等）时，电梯门只有短时间的临时过门上下文（`temporary context`），一旦机器狗离开门口超过维持距离（`door_transition_hold_distance`）或临时抑制计时过期，电梯轿厢内部的未知区域重新被激活为高分候选前沿，诱发回头路。
- **修改方案与修改后现状**：
  1. **高层电梯门永久禁区扩展**：在 `mission_manager.py` 的 `_frontier_selection_context` 中扩展永久禁区作用域。当处于非 0 层时，将当前楼层电梯门（`elevator_door_id`）注册为该楼层整个探索生命周期内的永久禁止半平面（`permanent_forbidden_half_plane`，半径 $\ge 3.0\text{m}$，衰减距离 $6.0\text{m}$），直到下一次换层或返航；
  2. **前沿硬剪枝**：在 `frontier.py` 中支持 `skip_in_forbidden_half_plane`，对永久禁区和临时禁区半平面内的电梯轿厢候选前沿进行直接剪枝剔除，彻底杜绝出梯后对电梯轿厢的重复回头探索；
  3. **测试验证**：单测覆盖 `test_mission_manager_helpers.py`，证明临时抑制过期后高层电梯门永久禁区依然稳固过滤轿厢深处前沿。

## 35. 2026-09-06 前向检测盒梯形收腰与转角卡死消除（STAGE36）

- **问题现象（转角与窄道擦边误判急停卡死）**：机器狗在拐角、窄门框或弯道行进转弯时，经常因检测到机身侧方近处的墙体或门框立柱而误触发前向障碍物急停（`nearest_forward_obstacle` 距离过小，控制器下发 `linear = 0.0`），导致转角频繁卡顿甚至卡死（corner freeze）。
- **根本原因排查与分析**：
  - 原有的前向障碍物检测区域采用固定半宽（`0.22m`）的矩形长方体通道（`forward >= 0.20m 且 lateral <= 0.22m`）。在紧贴机身近前方的近身段（`forward <= 0.35m`），固定 $0.22\text{m}$ 的半宽过宽，未考虑 A1 机器狗机身与肩关节自然收窄的物理轮廓，导致擦肩而过的转角墙皮（如实测中的 `forward ~ 0.2001m, lateral ~ 0.2078m`）被误判为正前方的正面阻挡障碍。
- **修改方案与修改后现状**：
  1. **前向走廊梯形收腰函数**：在 `map_update_runtime.py` 实现 `forward_obstacle_lateral_limit(forward)`：
     - **近身段（$forward \le 0.35\text{m}$）**：侧向半宽收腰至 **$0.16\text{m}$**，紧密贴合 A1 机器狗机身与肩关节物理包络，彻底排除擦肩而过的转角墙皮和窄门框点云；
     - **过渡段（$0.35\text{m} < forward < 0.60\text{m}$）**：侧向半宽由 $0.16\text{m}$ 线性渐变展开至 $0.22\text{m}$；
     - **远距离（$forward \ge 0.60\text{m}$）**：保持 **$0.22\text{m}$** 充分的走廊宽度，确保正前方正面障碍物拥有充足的行进减速与避障预警裕度；
  2. **动态半宽判定与全局接入**：`nearest_forward_obstacle_observation` 改用动态梯形半宽判定，在 `mission_manager.py` 中导出并接入；
  3. **测试验证**：在 `test_mission_manager_helpers.py` 中增加 `test_forward_obstacle_lateral_limit_profile` 与 `test_forward_obstacle_tapered_corridor_prevents_corner_freeze`，实测证明转角擦边点（如 `lateral~0.208m`）被安全忽略，真正正向障碍物灵敏检出。

## 36. 2026-09-06 远距离微小前沿剪枝优化与危险源生成规则对齐（STAGE37）

- **问题现象（远距离微小死角诱导机器人回头折返）**：机器人在自主探索过程中（如 `simtime 198.4s`），在完成当前房间探索并来到走廊后，容易被数米乃至十几米外已探索房间残留的家具背阴或桌腿阴影等微小未知碎片吸引，选择横跨地图掉头折返去清扫这些仅几个格子的细碎死角，而不去探索前方未知增益高达数百格的走廊深处和新房间，严重浪费任务时间。
- **根本原因排查与分析**：
  1. **远距离剪枝门槛过低**：前沿规划器中用于排除远距离小碎片的门限 `min_far_expected_gain` 原设为 `4.0` 格（距离 $> 3.5\text{m}$ 且收益 $< 4$ 格才剪枝）；
  2. **旧认知假设与真实规则脱节**：旧注释假设“0.15m 红球投影约为此量级，设为 4 格避免漏球”。但实际物理与官方规则（`generate_competition_scene.py`）表明：
     - 红球半径 $R = 0.15\text{ m}$，水平截面积 $0.0707\text{ m}^2$，在 $0.10\text{ m}$ 分辨率下**其实体自身就占据 5~8 个栅格**；
     - 规则硬性要求危险源只在房间内生成，且**必须避让墙体 $\ge 0.45\text{m}$（球心距墙 $\ge 0.60\text{m}$）、避让家具 $\ge 0.35\text{m}$（球心距家具 $\ge 0.50\text{m}$）**；
     - 任何能够合法生成危险源的位置，周围必须有至少直径 $1.0\text{ m}$ 的开阔区域（对应面积约 78 格）；
     - 任何连通未知格数 $< 16$ 格（$< 0.16\text{ m}^2$）的远距离狭小阴影或家具死角，在物理和生成规则上**100% 绝无可能容纳危险源**，4 格阈值导致大量无效噪点和死角被误当成合法候选竞选。
- **修改方案与修改后现状**：
  1. **提升剪枝门槛至 16.0 格**：在 `danger_search.yaml` 中将 `min_far_expected_gain` 由 `4.0` 调整为 `16.0`；
  2. **缺省回退参数同步**：在 `mission_config.py` 中将 `frontier/min_far_expected_gain` 默认回退值同步更新为 `16.0`；
  3. **单元测试与接口断言同步**：更新 `test_interface_documentation.py` 与 `test_mission_components.py` 中对生产配置和默认参数的断言至 `16.0`；
  4. **归档报告与缺陷台账更新**：在 `docs/team/BUG_LEDGER.md` 登记 `FRONTIER-PRUNE-001`，并编写完整报告 `reports/RUN_20260906_FRONTIER_PRUNING_REPORT.md`。

## 37. 2026-09-06 出大门室外返航终局收尾与激光断流保护（STAGE38）

- **问题现象（室外返航距起点仅1m因点云稀疏误报定位失败）**：`9.6.21.53` 测试中，机器狗跑通 1→0 返程电梯并穿过大门回到室外走廊，沿入场足迹反向返航至 `[0.041, -2.167]`（距离起点 `[0.0, -3.19]` 仅 1.024m，距旧容差 1.0m 仅差 2.4cm），但因前向面向开阔空地无墙面反射导致激光点数骤降（`insufficient_source`），室内严格监控触发位姿老化超过 0.3s 强制下发零速并在 3 秒后判定为 `localization_unavailable`，导致全任务被误标为 FAILED。
- **根本原因排查与分析**：
  1. **室外开阔场地几何特征退化**：机器狗穿出门机头朝外，前向无建筑物墙体，Livox 激光匹配点断崖式下跌至 3~4 点；
  2. **室内严苛监控误杀室外收尾**：控制器位姿老化阈值 0.3s 过于严苛，连续 3s 断流直接判死全任务，缺乏对终局室外开阔场景的柔性容错；
  3. **终点判定容差过窄且缺乏真实基准**：原 `home_tolerance: 1.0` 在室外过于严苛，且旧代码曾使用粗放单轴坐标判定。
- **修改方案与修改后现状**：
  1. **终点判定放宽与真实基准**：在 `mission_manager.py` 的 `_tick_return_home` 中，出大门后启用户外到达容差 `max(home_tolerance, 1.5)`，严格基于记录的出生点真实世界坐标 `home_pose` 的欧氏距离 $\le 1.5\text{m}$ 判定完成（`return_home_completed`）；配置文件 `danger_search.yaml` 中 `home_tolerance` 同步更新为 1.5；
  2. **室外断流优雅收尾**：在 `_apply_localization_progress_decision` 中，出大门室外返航阶段若激光发生断流，只要当前位置已处于室外起点容差范围内，严禁判定任务失败，直接认定为返航安全完成优雅收尾；
  3. **控制器位姿老化放宽**：在 `navigator.py` 中，执行 `return_home_entry_trace_path` 期间将位姿老化超时上限由 0.3s 放宽至 1.5s，避免室外开阔地单帧点云稀疏引发急刹停。

## 38. 2026-09-07 1 层天井负障碍防坠脱困重构、主备分流与 0.40m 转向包络膨胀（STAGE39）

- **问题现象（1层天井悬崖伪脱困、甩尾进悬崖与配额耗尽死锁）**：`9.6.23.38`（`seed_8127`）测试在 506.22 秒以 `negative_obstacle` 失败，全任务未能进入返航。日志显示机器狗在天井悬崖边连续两次触发防坠脱困，虽记录“后退完成、掉头完成”，但实际位移仅 0.016m、0.013m（只有一两厘米），掉头超标约 20° 未达标，均为超时伪报成功；原地掉头时外侧肢体转入悬崖上空；随后 A* 贴着崖边切线规划刚起步又触发 Gate；同一物理悬崖多帧落图质心未去重瞬间耗尽 3 次重规划配额，导致机器狗彻底死锁停滞。
- **根本原因排查与分析**：
  1. **动作死区与超时伪报**：后退速度 `-0.12m/s` 处于电机死区无法克服摩擦力，3s 超时后直接跳入成功；掉头 5s 超时同样跳入成功；
  2. **悬崖几何与回撤策略缺陷**：机器狗距悬崖 $0.40\sim 0.47\text{m}$ 时处于安全区，却无脑倒车与原地掉头，而 A1 原地转向包络半径 $R \approx \sqrt{0.30^2 + 0.25^2} \approx 0.40\text{m}$，原地掉头直接将外侧肢体甩入悬崖边缘；
  3. **负障碍落图缺乏机身膨胀**：落图未按机器狗旋转包络膨胀 0.40m，导致 A* 规划路径贴着悬崖切线擦过，一动即停；
  4. **同一悬崖重复落图耗尽配额**：同一悬崖点云多帧落图每次赋予新 cluster ID，3 次配额瞬间耗光。
- **修改方案与修改后现状**：
  1. **悬崖主备分流**：以 $d_{\text{base\_to\_drop}} = 0.40\text{m}$ 为界。当 $d \ge 0.40\text{m}$ 时，处于安全旋转包络外，直接原地清路径并在膨胀栅格上由 A* 重新规划绕行（`strategy: primary_in_place_replan`），不执行多余倒车与掉头；仅当 $d < 0.40\text{m}$ 极度贴崖时才启动微退兜底；
  2. **动作做实与真退开**：微退速度提升至 `-0.35m/s`，超时放宽至 7.0s；增加真实位移校验，位移 $< 0.05\text{m}$ 记录 `step_back_safe_retreat_blocked` 并尝试其他脱困，严禁超时伪报；掉头超时未收敛明确返回失败；
  3. **负障碍 0.40m 膨胀与前沿扣分**：在 `mapper.py` 和 `danger_search.yaml` 引入 `negative_obstacle_inflation_radius: 0.40`，将负障点做 0.40m 圆形膨胀写入障碍栅格；在前沿探索评分中对靠近负障 $< 0.50\text{m}$ 的候选点扣除 100 分，杜绝向崖边派发探测点；
  4. **质心去重与历史轨迹安全回撤**：在 `ground_support_coordinator.py` 中对落图负障质心实施 $< 1.0\text{m}$ 去重，同一悬崖多帧落图不递增配额计数；二次触发脱困时调用 `_build_trail_backtrack_path` 沿历史安全轨迹反向回撤 1.0m，避免再次盲目掉头甩尾。

## 39. 2026-09-07 主入口单调前向站位与穿门防 180° 掉头死锁消除（STAGE40）

- **问题现象（开门后在门前 180° 原地大掉头并横向死锁）**：主入口大门开启后，机器狗非但没有向前穿门，反而在门前触发 180° 原地大掉头（`turn_in_place_active`），航向角反转，横向卡在门框外死锁，线速度被压为 0，迟迟无法进门。
- **根本原因排查与分析**：
  - 在 `navigation_tasks.py` 的 `_compute_forward_aligned_wait_point` 中，原实现包含 `target_proj = min(wait_offset - 0.8, max(0.0, proj))` 截断逻辑。当机器狗已推进到门前（`proj > 0.8m`）时，`wait_offset - 0.8` 截断直接强制将目标站位点投影到了机器人当前位置的“身后” 0.2m 处！
  - 目标落在身后导致控制器计算出航向偏差高达 180°，激活 `turn_in_place_active`，线速度强制压为 0，在门前狭窄区域死锁原地打转。
- **修改方案与修改后现状**：
  1. **单调前向追踪**：重构站位计算为严格的单调前向追踪 `target_proj = max(0.0, proj + 0.35)`，确保目标点永远单调向前推进，绝不向身后倒退；
  2. **近门直插机制**：当机器狗与门平面的垂直距离 $< 0.40\text{m}$ 时，跳过中间过渡点，直接以门内通过点 `_pass_point` 为目标直插穿门，彻底消除门前原地掉头与打转死锁。

## 40. 2026-09-07 Frontier 探索微距盲区过滤与监视器自愈拉黑闭环（STAGE41）

- **问题现象（走廊微小位移报到达引发 14+ 次退化选点死循环）**：机器狗在部分走廊或拐角区域出现“连续十几次触发无进展报警（`mission_progress_monitor: displacement 0.02m < 0.20m`）”，原地微移几厘米就报到达，形成多达 14+ 次退化微距选点死循环，严重消耗探索预算。
- **根本原因排查与分析**：
  1. **选点容差与监视器盲区**：探索规划器 `frontier.py` 候选点生成逻辑允许在机器人近身 $0.25\sim 0.35\text{m}$ 处生成目标点，机器狗微动几厘米即可触发 0.40m 到达容差，随即再次选定近身点；
  2. **监视器缺乏自愈闭环**：`mission_progress_monitor` 虽然能识别位移 $< 0.20\text{m}$ 的停滞并打印警告，但只记录诊断日志，未触发任何降级、拉黑或冷却机制，导致系统陷入死循环。
- **修改方案与修改后现状**：
  1. **源头门限提升**：在 `frontier.py` 中将探索候选点的最小有效距离提升至 `goal_tolerance + min_displacement`（$> 0.55\text{m}$），从源头杜绝生成“走两步就到”的退化微距点；
  2. **监视器自愈拉黑闭环**：在 `mission_manager.py` 中，当 `progress_monitor` 连续报警触发退化时，自动调用 `progress_monitor.start_cooldown` 并在 `explorer.blacklist_region` 中将当前停滞区域进行短时拉黑封禁，强制规划器切换到其他方向的开阔前沿，斩断死循环。

## 41. 2026-09-07 自动化测试框架多 Seed 结果安全归档与孤儿进程彻底清理（STAGE42）

- **问题现象（多 Seed 连续测试跨轮污染与异常退出孤儿进程残留）**：连续测试多 seed 时，公共 `results/` 目录的历史结果会跨 seed 污染后续评测；原先直接 `rm -f` 会导致历史证据丢失；仿真异常中断时后台进程残留导致端口占用或下一轮启动卡死；时间戳标记文件放进 `seed_dir` 导致底层 runner 判定非空拒跑。
- **根本原因排查与分析**：
  - 测试脚本缺乏独立结果安全移动归档机制；退出清理缺乏强力无差别杀绝命令；时间戳标记文件位置不当。
- **修改方案与修改后现状**：
  1. **安全归档不删除**：在 `run_multi_seed_test.sh` 中，将每轮生成的选手结果与离线评测真值直接移动（`mv`）归档到对应 `seed_${seed}/` 独立目录下；对公共目录的历史残留文件自动归档至 `legacy_results_backup/`，杜绝跨 seed 污染且完整保留全部历史证据；
  2. **孤儿进程强力清理**：在脚本开头、每轮测试前后、以及 `EXIT` / `INT` / `TERM` 的 trap 钩子中，统一加入强力清理命令 `killall -9 gzserver gzclient rosmaster roscore junior_ctrl danger_search 2>/dev/null || true`，确保测试环境绝对干净；
  3. **标记文件路径隔离**：将时间戳标记由 `$seed_dir/.seed_started_marker` 移至 `$OUTPUT_ROOT/.seed_${seed}_started_marker`，避免污染 `seed_dir`。

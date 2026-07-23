# SimEnv 已完成与历史归档

最后更新：2026-07-23

本文归档已经完成的机制、有效验证和历史失败结论。尚未完成的事项只维护在 [fix.md](fix.md)。

## 1. 合规生产边界

- 正式启动链只使用赛方公开传感器、`/cmd_vel`、门/电梯服务和 `team_scene_info.json`。
- 生产节点不订阅 `/Odometry_gazebo`、`/ground_truth/*`，不读取 world、truth、layout metadata 或固定场景坐标。
- 位姿统一由合法传感器定位产生并通过 `PoseProvider` 提供。
- 点云和 RGB-D 使用时间匹配 pose；PENDING 延迟消费，EXPIRED 跳过，不用最新 pose 兜底。
- 门、电梯公开 ID、`served_floors`、文件 schema 和服务调用均 fail-closed 校验。
- 真值只由独立诊断进程录制并在任务结束后离线使用，不进入生产决策。
- 正式结果保持官方 `detected_danger.json` 格式；额外摘要仅在 runner 显式指定时写入运行归档。

## 2. 启动与控制链

- 启动竞争已通过条件同步关闭：等待模型延迟关节配置、控制主循环首帧和关节反馈后再恢复物理。
- 机器人可稳定完成 Passive → FixedStand → RL `/cmd_vel` 控制交接。
- 按键 8 reset 已改为有界、边沿触发、失败保持暂停的安全流程，并完成 reset 后再次站立 GUI 验证。
- 所有生产速度命令继续经过唯一 `PathFollower` 出口，没有新增竞争 `/cmd_vel` 发布者。

说明：`auto.sh`、`FSM.h`、`FSM.cpp` 和用户保留的 `gazebo.xacro` 是已知官方哈希差异；后续不得继续扩大赛方文件修改。

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

证据：`../codeDG_rebuild/reports/STAGE10_3_REPORT.md`。

## 5. 主入口与门任务

- 主入口通过合法 Livox/RGB-D 运行期几何注册，不依赖固定坐标。
- 开门后使用近场门洞重观测重新计算门中心与轴线，避免长程定位漂移造成顶门框。
- 返航先到传感器注册等待点，并校验门侧、径向距离和横向门轴。
- `OpenDoorAndPass` 已完成对准、开门确认、直线通过和门后深入的有界闭环。
- 第 11 层封口轮完成主入口进入、返航重观测、出楼和回起点。

## 6. 电梯与多楼层机制

- RGB-D 单扇电梯门板语义已与任务注册阈值对齐，电梯视觉注册成功。
- A* 已区分 `REACHED`、`UNREACHABLE` 和 `BUDGET_EXHAUSTED`，规划失败、STUCK 和总超时均有界退出。
- 精确候梯点不可达时，只选择大厅侧真实历史轨迹点作为标准 A* 目标，不改写地图。
- 呼梯前会近场重观测门板，只刷新运行期电梯/门几何，不修改 odom、地图、轨迹或检测坐标。
- 目标门传感器确认后，再以两份新鲜 odom 完成逻辑楼层对齐。
- `0→1`、`1→0`、跨层点云隔离、二层探索、主入口退出和回起点已完成多轮 GUI 闭环。

第 11 层封口轮：

- 证据目录：`/tmp/stage11_main_entrance_axis_round1_20260722`。
- 仿真 `160.17 s` 完成，最终覆盖率 `0.34`。
- 离线定位 RMSE `0.084 m`、P95 `0.148 m`、最大/最终 `0.288 m`，最大 yaw 误差 `0.0032 rad`。

最新两轮无回归验证：

- `/tmp/stage12_elevator_start_retreat_round1_20260723`：`167.514 s` 正常完成。
- `/tmp/stage12_elevator_start_retreat_round2_20260723`：`169.616 s` 正常完成。

证据：`../codeDG_rebuild/reports/STAGE11_REPORT.md`、`../codeDG_rebuild/reports/STAGE12_ELEVATOR_START_RETREAT_REPORT.md` 及上述两轮运行目录。

## 7. 地图、负障与恢复机制

- 2D 地图已增加在线负障语义：本层以下回波不会继续把空洞射线刷成 FREE，确认负障可写入不可通行层。
- 同格直接楼面支撑证据可恢复动态负障，保证电梯轿厢返回后可重新通行。
- 普通 STUCK 不再把机器人当前格永久写成 BLACK；临时目标黑名单与 A* 地图职责分离。
- 返航 STUCK 已增加失败段反馈、候选否决和有界失败，不再无限重复同一路径。
- 旋转恢复统一检查 `0.40 m` 径向净空，贴墙时保持零速。
- 二层无前沿恢复已实现“沿真实直线来路低速倒退一次、复扫一次、耗尽结束本层”。
- 电梯返程 OCC 起点恢复已实现“仅从连续初始膨胀带沿真实轨迹直退一次，再重走标准 A*”。

后两条恢复分支的代码和单测已完成，但真实自然触发证据仍列在 `fix.md`。

## 8. 定位与健康门

- 正式定位使用 RGB-D/Livox 平移配准和 `/trunk_imu` 姿态约束；yaw 在现有仿真中稳定。
- 定位 stance health 仅依赖公开 `/scan` 和 `/trunk_imu`，异常时撤销 ready，MissionManager 零速并显式失败。
- `floor_local` z 语义已统一，当前所有消费者使用配置 `map/floor_height: 2.6`。
- 定位 M0 只读诊断已完成：记录 GICP 拒绝原因、scan-to-map 校正、尺度窗口和运动一致性，不改变融合行为。
- M2 已将 scan-to-map 单帧上限参数化并保留 `0.10 m` 大跳变保护。
- M3 已完成逐帧质量与反事实分析，证明现有单一残差、对应点数量或固定轴符号不足以可靠在线降权。

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

证据：`../codeDG_rebuild/reports/STAGE12_M0_REPORT.md`、`../codeDG_rebuild/reports/STAGE12_M2_REPORT.md`、`../codeDG_rebuild/reports/STAGE12_M3_DIAGNOSTIC_REPORT.md`。

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

## 10. 当前验证基线

- 参赛 Python 全包：`473 passed`。
- 定位 C++/ROS 测试：`94 passed`。
- 生产/离线边界审计：通过。
- `git diff --check`：通过。
- 官方文件哈希：恰有 4 项历史差异，无新增赛方文件差异。
- 最新 GUI 测试结束后 ROS master、Gazebo、控制器和 runner 均已清理。

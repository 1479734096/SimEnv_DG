# 迁移台账

| 层级 | 内容 | 状态 | 提交/证据 |
|---|---|---|---|
| 0 | 官方源码、构建、全传感器与站立基线 | 完成 | `migration-layer-0`、`reports/STAGE0_REPORT.md` |
| 1 | 治理、接口契约与包骨架 | 完成 | `migration-layer-1`、`reports/STAGE1_REPORT.md` |
| 2 | 传感器、定位、PoseProvider | 完成 | `migration-layer-2`、`reports/STAGE2_REPORT.md` |
| 3 | 建图、A*、前沿 | 完成 | `migration-layer-3`、`reports/STAGE3_REPORT.md` |
| 4 | 路径执行与地面支撑安全 | 完成 | `migration-layer-4`、`reports/STAGE4_REPORT.md` |
| 5 | 危险源与门板感知 | 完成 | `migration-layer-5`、`reports/STAGE5_REPORT.md` |
| 6 | 门、电梯和导航任务 | 完成 | `migration-layer-6`、`reports/STAGE6_REPORT.md` |
| 7 | MissionManager 编排 | 完成 | `migration-layer-7`、`reports/STAGE7_REPORT.md` |
| 8 | 正式 launch、runner 和结果闭包 | 完成；动力学风险转第 10 层 | `migration-layer-8`、`reports/STAGE8_REPORT.md` |
| 8.1 | 定位持续姿态健康闭环 | 完成代码、ROS 合成与 Gazebo 侧倾验收 | `migration-layer-8-health`、`migration-layer-8-health-gazebo`、`reports/STAGE8_1_REPORT.md`、`reports/STAGE8_2_REPORT.md` |
| 8.3 | `auto.sh` 安全启动同步 | 完成；GUI 正向完整链与无控制器负路径通过 | `migration-layer-8-auto-startup`、`reports/STAGE8_3_REPORT.md` |
| 9 | 离线工具隔离 | 完成 | `migration-layer-9`、`reports/STAGE9_REPORT.md` |
| 10.1 | 主入口开门、近场重观测、过门和离门闭环 | 完成 | `reports/STAGE10_1_REPORT.md`；固定种子 GUI 仿真通过 |
| 10.2 | 单层探索、检测与返航闭环 | 完成；正确起点 GUI 返回 home 并写出成功结果 | `migration-layer-10-single-floor-return`、`reports/STAGE10_2_REPORT.md` |
| 10.3 | 完整进程接口合规收口 | 待开始：移除 RL 真值观测依赖并关闭环境默认真值输入 | `COMPLIANCE-001` |
| 10 | 单层、多层、安全和性能验收 | 进行中 | 最终集成 |

每层遵循“等价迁移 → 特征测试 → 缺陷修复 → 架构整理 → 定向验证 → 报告与提交”。

当前第 10 层进行中：第 10.1 层已完成主入口闭环；第 10.2 层已完成单层探索、返航、
主入口出门、到达 home 和结果写出。下一步先关闭完整进程中的 RL 真值观测依赖，再按
`fix.md` 的 P4 建议量化移动/零速/恢复耗时并分场景提速，随后进入两层/三层电梯往返。
第 9 层已用根目录 `tools/offline/`、严格生产安装清单和自动审计隔离离线工具；
旧工作区混合启动与真值评测的工具不直接迁入。
第 8.3 层已关闭默认启动竞争和标准控制交接阻断；
`LOCALIZATION-002` 已完成代码、单元测试、ROS 合成和 Gazebo 侧倾闭环验收，
不得以诊断夹具的单次成功代替正式启动确定性。

## 未关闭问题与后续实施顺序

以下清单合并 `fix.md` 的 P2～P5、F2～F5、原工作区后续说明及新仓库实测。
“代码已修复”不等于“物理闭环已验收”；每项仍按依赖顺序单独形成证据。

1. **启动与控制前置（第 10 层入口）**
   - `ENV-002` 已通过用户授权的 `auto.sh` 安全同步关闭；`ENV-004` 按键 `8` reset 路径
     已完成基本功能修复，用户从 `./auto.sh` 启动后的人工 reset 实测通过。静态复审发现
     的持续输入、pause 失败状态、过渡超时和证据一致性问题已登记为最终验收前收口项，
     当前不阻断后续迁移与仿真测试；不得据此删除最终边界回归。
   - `CONTROL-004` 已按“零命令切 RL → 稳定等待 → 启动任务”的顺序通过 GUI 完整链；
     运行中晚切 RL 仍属禁止操作，不作为正常流程。
   - `LOCALIZATION-002` 已完成可控侧倾闭环；后续正常长任务只需回归它不误报，不再作为
     第 10 层前置阻断。
2. **P2/F3～F5 最低可用防坠**
   - 新仓库已用窗口速度估计、死区契约和安全门输入校验修复旧的差分尖峰路径，但仍需
     真实物理速度对照和长时间零误停证据。
   - 在已知空带取得 `DROP_CONFIRMED → 零速 → 停稳 → 当前路径失效/局部重规划` 的
     生产闭环，测量正安全余量；不得靠放宽阈值或虚构停车表封板。
   - 验证一层探索不会再次从无支撑区域坠落，普通 STUCK/recover 不得抢占防坠语义。
3. **P3/F2 多楼层与返航**
   - 完成一次两层 `0→1→0→主入口→home`，再完成三层 `0→1→2→0→home`。
   - 复验目标层门视觉确认、电梯往返、返回 0 层后的主入口轴向接近，以及防坠不会在
     电梯阶段误终止。
   - 关闭 `COORD-001`：核对 `floor_local` 的逻辑 z 偏移与最终危险源绝对 z。
4. **P4 运行效率**
   - 分解规划、静止、有命令无位移、转向、门、电梯和有效移动时间。
   - 基于证据改进无可达目标过滤、路径简化/跟踪、走廊折线、分场景速度、弧线跟踪、
     轿厢内提前对准和退出后的非必要等待；仅在规划停车占比足够时考虑异步化。
5. **P5 覆盖、识别与预算**
   - 减少重复探测、重复路径和无收益扫描，验证分楼层地图与门/电梯语义缓存。
   - 比较 frontier 收益、换层成本和返航成本；完成 300/620/840 秒预算闭环。
   - 量化漏报在覆盖、定位、外参和检测器之间的主因，并完成单/双/三层得分评估。

已在第 7 层关闭且只需物理回归的 `fix.md` 项：`MISSION-001` 电梯成功三态、
`FLOOR-001` 隐式楼层对齐超时终止、`SMELL-001` 局部变量命名。若电梯任务在全局预算
到期时仍处于进行中，摘要保持 `null` 表示“未完成”，不得误记成功或强行归为服务失败。

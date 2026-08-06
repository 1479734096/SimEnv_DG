# 地图证据生命周期与二层闭环报告

## 1. 当前阶段目标

在不使用真值、固定坐标或 seed 特例的前提下，使正式地图不再被单帧低高度回波永久
污染，并让同一份来源化地图证据同时服务探索、电梯和返航规划。验收顺序为：先固定
`seed=8127` 完成两层长时闭环，再登记有限随机 seed；短时机械闭环不等于本阶段通过。

本报告从回退后的可运行基线重新建账。历史成功与回退边界见
`ROLLBACK_BASELINE_REPORT.md`，最近一次长时失败见
`SEED8127_CLOSED_LOOP_RUN_REPORT.md`。

## 2. 2026-08-06 地图证据生命周期实现

### 2.1 已确认根因

最近一次 `seed=8127` 长时运行在 0 层探索后无法接近电梯。GroundSupport 的两个临时
约束均已撤销，楼层退出证书也冻结为 `revision=0`、`region_ids=[]`；电梯任务收到的
却只有泛化 `unreachable`。代码审查确认回退基线的 Mapper 仍把任意
`0.15m <= z_rel <= 1.40m` 单点立即写为永久 OCC，并让自由射线在该 OCC 处停止。
历史诊断已证明 `0.155m~0.191m` 的擦边低回波可沿已走走廊累计成阻断带。因此本轮
修复对象是正式地图正障碍证据生命周期，不是继续增加电梯重试或放宽 GroundSupport。

### 2.2 实现合同

1. `FloorGrid` 分离基础层与只读派生规划视图。派生优先级固定为
   `OCC > BLACK > NEGATIVE > VISITED > FREE > UNKNOWN`，生产与测试不再直接写
   `data`。
2. 新增 `ObstacleEvidenceLedger`，持有障碍核心、膨胀反向 owner、有界命中代次、
   有界观测位姿和严格射线反证。`LOW_CANDIDATE` 不阻挡；不同点云代次且至少两个
   分离观测位姿确认后晋升 `LOW_CONFIRMED`；`HIGH_HARD` 立即阻挡。
3. 反证必须来自不同代次和分离位姿，射线终点严格越过核心，且同帧不得存在该核心
   命中。候选和已确认低核心采用不同撤销门槛，不使用 TTL 自动遗忘。
4. DROP 继续以 `NegativeClusterRecord` 原子持久化；障碍、DROP 在双 epoch 改变时
   一并撤销，BLACK 与基础走过轨迹不混入该生命周期。
5. 合法点云的 generation、map epoch、localization epoch 从感知提交边界原样传到
   Mapper。生产 generation 为正，因此不会退回 legacy 单帧 OCC 路径。
6. A* 使用一个 `(cell, escaped)` 有状态搜索统一处理 Support veto 与 OCC 膨胀起点。
   退出前每条边必须远离全部失败来源/障碍核心；其他 owner 始终硬阻挡；核心格、
   owner 缺失和跨 epoch owner fail-closed；退出后禁止重入。所有出口共享同一预算，
   不新增盲倒车或第二条 fallback。
7. `PlanResult` 增加有界 blocker 摘要，电梯规划日志保留 `reason_code` 与 blocker，
   不再把所有失败压扁为 `unreachable`。
8. 返航失败首段可能是一个栅格长的认证退出前缀。快照 veto 现沿失败方向延伸至既有
   有界 lookahead，并显式跳过起点格，避免短 waypoint 使 veto 失效。

### 2.3 验证

- `python3 -m pytest -q test`：`679 passed`，仅既有 Cryptodome 弃用告警；
- Mapper/Planner 定向反例覆盖：单帧低候选不阻挡、多代次/多位姿晋升、严格反证
  撤销、重叠膨胀 owner、双 epoch 撤销、OCC 膨胀起点逃逸、核心格拒绝、跨 epoch
  owner 拒绝；
- 受影响 Python 文件 `py_compile`：通过；
- `tools/offline/check_production_boundary.py`：通过；
- `/cmd_vel` publisher 仍只有 `navigator.py` 一处；
- 禁用输入静态审计未发现新增生产依赖；
- `git diff --check` 与 `git diff --cached --check`：通过；
- 官方哈希仍仅既有 5 项差异，未扩大。

尚未验证：本轮新地图生命周期和统一起点逃逸的 Gazebo 长时闭环行为。下一步只运行
一次正式配置的 `seed=8127`；确定性失败后停止，不追加 seed 或现场调参。

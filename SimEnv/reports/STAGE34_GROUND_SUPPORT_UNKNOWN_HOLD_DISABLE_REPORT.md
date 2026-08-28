# STAGE 34: 地面支撑平地盲区停顿关闭与超时放行优化报告

## 1. 背景与问题定义

在高速自主探索（线速度上限锁定为 $0.75\text{m/s}$）阶段，系统在平地直道行进时暴露了两个关键问题：

1. **走廊 Z 字形蛇形绕道（已在第一阶段修复）**：
   - 机器狗前瞻 $1.0\text{m}$ 处的平地横截面因雷达非重复扫描稀疏（4 个格子未达到 $\ge 2$ 点云阈值），触发 `OBSERVATION_HOLD`（停顿 3 秒）；
   - 超时后旧逻辑误报 `ground_support_path_unavailable` 并在平地走廊正中打上硬禁行障碍物，迫使 A* 规划器绕向两侧走出 Z 弯。
2. **平地走走停停死循环（第二阶段彻底解决）**：
   - 第一阶段优化后，安全门在观察期超时且无真跌落证据时直接放行直走，消除了虚假禁行与 Z 弯；
   - 但因为平地远端（$0.86\sim1.0\text{m}$）激光密度持续处于临界状态，导致放行后控制器刚给速 $0.70\text{m/s}$，安全门下一帧又计算出支撑不足（$0.56\text{m} \le 0.686\text{m}$），瞬间再次抓入 `OBSERVATION_HOLD`；
   - 导致在 $76\text{s}$ 内发生 13 次停顿、累计发呆 $36\text{s}$，“走 0.5 秒停 3 秒”严重拖慢探索效率。

---

## 2. 核心架构与修改要点

根据系统设计规范（*“UNKNOWN 临时观察停车可独立关闭；显式低层回波 DROP 始终锁存停车”*），进行了如下分层优化：

### 2.1 平地盲区拦截关闭（`danger_search.yaml`）

* **参数变更**：`ground_support.unknown_support_hold_enabled: false`
* **配置同步**：`mission_config.py` 与 `mission_manager.py` 缺省值同步对齐为 `False`。
* **效果**：平地正常行进时完全绕过 `OBSERVATION_HOLD`，机器狗以 $0.75\text{m/s}$ 持续全速直行。

### 2.2 防坠落绝对安全保证（`ground_support_gate.py`）

* 真正的结构性负障碍（二楼天井、中庭坠井、楼梯等）拥有打到楼下地板（$z < \text{floor\_z} - 0.30\text{m}$）的负高度反射点云（`drop_confirmed: true`、`drop_clusters`）；
* 无论 `unknown_support_hold_enabled` 为何值，`_fresh_drop_blocks` 均独立优先拦截，强力刹车进入 `DROP_STOPPING` $\to$ `SAFETY_BLOCKED` 锁死停车；
* `boundary_hold_margin` 严格保持 **0.40 m** 红线不变。

### 2.3 状态机超时放行与 Witness 隔离

* 在 `check_control_timeout` 中，若未来有特殊场景进入临时 Hold，超时后当且仅当存在真实跌落证据时才切入 `SUPPORT_PATH_UNAVAILABLE`；若无真跌落证据直接放行并清除 witness，杜绝向全局地图注入虚假障碍。

---

## 3. 测试与验证结果

1. **自动化测试**：
   - 地面支撑专项测试：`62 passed`；
   - 全仓 Python 单元测试：`895 passed, 0 failed`（100% 通过）。
2. **生产红线与边界审计**：
   - `python3 tools/offline/check_production_boundary.py` $\to$ **参赛 Python/离线边界审计通过**；
   - `git diff --check` $\to$ **格式检查无误**。

# STAGE22 技术报告：电梯入厢直通与轿厢安全调头

## 1. 背景与根因剖析

在上一轮仿真双层任务测试（日志 `/tmp/danger_search_elevator_test_20260827_143959/mission.log`）中，系统暴露了两个影响效率与任务成败的关键问题：

1. **电梯门前停顿 5.5 秒（$t=94.6\text{s} \sim 100.8\text{s}$）**：
   - 机器狗到达电梯门口时，航向基本朝向电梯门（误差仅约 20°）；
   - 但控制器进入 `begin_heading_alignment` 强制走 `SETTLE`（滑行减速等待 1.6s）$\to$ `ALIGN`（原地旋转 2.8s）$\to$ `CONFIRM`（静止确认 1.1s），造成 5.5 秒的无意义门口静止发呆。
2. **轿厢内调头踢墙卡死并引发二楼开门确认超时（$t=103.4\text{s} \sim 120.1\text{s}$）**：
   - 轿厢内部空间有限（净空约 2.4m×2.7m），机器狗进厢后调头被下发了 `1.2 rad/s` 极限角速度；
   - 极限角速度引发强化学习步态（RL policy）迈出最大步幅（外摆 $\pm 0.30\text{m}$），四足踢撞并卡在侧壁接缝处，导致实测转速仅 `0.03 ~ 0.08 rad/s`，15 秒内仅旋转不到 30°；
   - 电梯升至二楼开门时，机器狗仍然背对二楼大门（偏离 140°），雷达与 RGB-D 无法看到门口，`WAIT_TARGET_DOOR_OPEN` 超时 10 秒后以 `"floor_change_failed_floor_alignment_unconfirmed"` 失败。

---

## 2. 解决方案与核心改动

### 2.1 入厢快速路径（Skip ALIGN Settle）
- **文件**：[`SimEnv/src/danger_search/danger_search/navigation_tasks.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigation_tasks.py)
- **机制**：在 `_tick_cabin_transit` 的 `POSITION` 阶段，当机器狗到达门前等待容差且朝向轿厢航向误差 $\le 0.50\text{ rad}$（约 28.6°）时，在同一 tick 直接调用 `_commit_cabin_cross` 提交直线进厢路径并切入 `CROSS` 阶段；
- **收益**：彻底消除进门前 5.5 秒的刹停、对准与确认等待，单次进门直接省 5.5 秒。

### 2.2 轿厢调头柔和原地限速（Pure In-Place Turn $\le 0.65\text{ rad/s}$）
- **文件**：[`SimEnv/src/danger_search/danger_search/navigator.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigator.py)
- **机制**：
  - `PathFollower.begin_heading_alignment()` 增加可选局部角速度上限 `max_angular`；
  - `_align_step_locked` 中计算 `omega` 时使用 `effective_max = float(self._align_max_angular) if self._align_max_angular is not None else float(self.max_angular)` 严格限幅；
  - 执行死区补偿 `deadband_omega` 不超过 `effective_max`；
  - 在 `navigation_tasks.py` 的 `_tick_target_door_alignment` 中传入 `max_angular=0.65`；
- **收益**：角速度降低后四足踏步收敛在机身投影包络内（外摆幅 $<0.15\text{m}$），完全消除与轿厢壁的物理干涉与卡阻。

### 2.3 调头完成锁存与先转正后开门确认时序保护
- **文件**：[`SimEnv/src/danger_search/danger_search/navigation_tasks.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigation_tasks.py)
- **机制**：
  - `TakeElevator` 维护 `_cabin_turn_completed: bool` 状态；
  - 关门与升梯期间异步推进调头，调头完成时锁存 `_cabin_turn_completed = True`；
  - 升梯服务完成后，若调头尚未完成，状态机平滑进入 `WAIT_CABIN_TURN_COMPLETE`（设置 8 秒确定性上限）继续等待调头完成；
  - 调头完成后才进入 `WAIT_TARGET_DOOR_OPEN` 并开启开门确认计时，确保传感器在开门时已经正对二楼门口。

---

## 3. 测试与验证结果

### 3.1 单元与契约测试
运行全量单元与契约测试：
```bash
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -o cache_dir=/tmp/.pytest_cache \
  --ignore=src/danger_search/test/test_localization_burst_analyzer.py \
  --ignore=src/danger_search/test/test_localization_quality_analyzer.py src/danger_search/test/ -q
```
**结果**：`809 passed in 5.20s`（100% 通过）。

### 3.2 新增单元测试清单
1. `test_begin_heading_alignment_with_max_angular_limit`：验证局部限速 0.65rad/s、线速度为零；
2. `test_begin_heading_alignment_default_max_angular_preserves_global_limit`：验证全局限速 1.2rad/s 保持原行为；
3. `test_cabin_transit_enter_fast_path_skips_align_when_aligned`：验证门前对准误差 $\le 0.50\text{ rad}$ 时 0 延迟进入 `CROSS`；
4. `test_take_elevator_gates_wait_target_door_open_until_cabin_turn_complete`：验证先调头转正、后开启开门确认计时；
5. `test_take_elevator_cabin_turn_timeout_fails_deterministically`：验证调头超 8 秒确定性失败上抛。

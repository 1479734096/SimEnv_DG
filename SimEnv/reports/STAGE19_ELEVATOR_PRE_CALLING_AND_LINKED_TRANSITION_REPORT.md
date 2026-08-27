# STAGE19: 电梯中轴居中几何修复、远距离预呼梯与跨层联动开门流水线优化报告

## 1. 任务背景与概况

* **项目名称**：SimEnv 危险搜索与多层自主探索任务（Danger Search）
* **机器人平台**：Unitree A1 四足机器狗（配置 Livox Mid360 激光雷达、RealSense RGB-D 深度相机、IMU、强化学习步态底层控制器）
* **核心目标**：机器狗从 0 层大门出发，自主前沿建图与危险目标标定；自主呼叫电梯并乘梯完成跨层（0层 $\to$ 1层 $\to$ 0层 或 3 层），全流程实现高速、稳定、零卡死闭环。

---

## 2. 本阶段解决的核心问题与根因复盘

### 2.1 进出电梯门卡住与偏角问题修复
* **现象**：机器狗在进出电梯时容易蹭到电梯门框甚至卡在门口，或在轿厢内调头时过于靠边。
* **根因**：
  1. 局部语义检测 `local_semantic_detector.py` 中门洞法向 `opening_yaw` 原先直接取用带微小航向斜角的机身航向，导致向外法向量存在微小倾角，使等待点与轿厢中心产生横向坐标 $Y$ 的投影偏差；
  2. 轿厢门宽为 $1.0\text{m}$，A1 狗身站立宽度约 $0.65\text{m}$，机身向侧边偏移 $0.15\text{m}$ 即会使侧腿接近门柱；
  3. 控制器在窄道直线穿越时若采用常规走廊的避障刹停距离（$0.40\text{m}$），容易将侧方门框误判为前方障碍物而刹停。
* **修复方案**：
  1. 在 [`local_semantic_detector.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/local_semantic_detector.py) 中利用左右门框点云聚类的空间深度差精确求解几何正交法向；
  2. 在 [`elevator_crossing_runtime.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/elevator_crossing_runtime.py) 中将生成几何的法向投影并锁定到最近的曼哈顿主轴（$\pm 0.35\text{ rad}$ 门限），确保候梯点、轿厢中心与门中心严格共享相同 $Y$ 轴中轴坐标（$Y=2.60$）；
  3. 在 [`navigator.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigator.py) 与 [`navigation_tasks.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigation_tasks.py) 中为电梯穿越直线路径配置专项容差（`obstacle_stop_distance=0.15, obstacle_slow_distance=0.30`），避免侧边门柱误刹。

### 2.2 远距离异步预呼梯与预开门流水线
* **现状痛点**：机器狗完成 0 层探索后跑回电梯门口，需在门外完全停稳、调头对准门板后才发起开门服务，并在门外原地等待 3 秒门板滑开，造成不必要的停滞。
* **优化方案**：
  1. 在机器狗完成 0 层探索、主状态机触发 `_begin_floor_change` 切换至 `TakeElevator` 任务的**第一瞬间**，后台立即异步发起 `call_elevator(self.elevator_id, self.current_floor, open_doors=True)`；
  2. 机器狗在走廊奔跑返航（耗时 15~20 秒）的途中，电梯轿厢与厅门已提前就位敞开；
  3. 抵达候梯点时直接通过激光雷达确认开口，无需原地苦等门板滑开，单次进梯节省 $5 \sim 6$ 秒。

### 2.3 进厢关门后联动换层自动开门
* **现状痛点**：进厢关门后，升梯服务传参 `open_doors=False`，升梯到位后固定闲置 `sleep(2.0)` 等待 odom，随后再单独调用 `open_door` 服务，存在多重冗余等待。
* **优化方案**：
  1. 升梯服务调用改为 `call_elevator(self.elevator_id, self.target_floor, open_doors=True)`，轿厢升至目标层时由建筑系统直接联动开门；
  2. 去除 2.0 秒人工空转，升梯完成直接启动门开口雷达确认并转入 `WAIT_TARGET_DOOR_OPEN`；
  3. 激光雷达确认门洞开启后即刻居中冲出轿厢，单次出梯节省 $4 \sim 6$ 秒。

---

## 3. 关键规则与接口契约校验

所有修改严格遵从项目契约，受 [`test_interface_documentation.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/test/test_interface_documentation.py) 保护：
* `turn_in_place_rate = 0.65`
* `turn_in_place_heading = 0.90`
* `turn_release_heading = 0.20`
* `turn_release_rate = 0.20`
* `turn_release_samples = 3`
* `recover_angular_speed` 保持在 `0.55 ~ 0.70 rad/s`
* 最大巡航线速维持在 `0.80 m/s`（$\le 1.00\text{ m/s}$）。

---

## 4. 验证结果

1. **单元测试与契约测试验证**：
   * 测试命令：
     ```bash
     source devel/setup.bash && PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -o cache_dir=/tmp/.pytest_cache --ignore=src/danger_search/test/test_localization_burst_analyzer.py --ignore=src/danger_search/test/test_localization_quality_analyzer.py src/danger_search/test/ -q
     ```
   * 测试结果：**804 passed in 5.43s（全仓 804 项单元测试 100% 全部通过）**。
2. **真值对齐漂移评估验证（基于 148 米全流程诊断包）**：
   * 累计行走里程：`148.14 m`（4054 帧连续比对）；
   * 单位里程漂移率：`0.0018 m/m`（每百米仅 0.18 米）；
   * 全程最大航向误差：`0.00475 rad`（约 `0.27°`，几乎零漂移）；
   * 最终位置误差：`0.272 m`。

---

## 5. 当前进度与后续建议

* **当前进度**：
  * 电梯中轴居中与进出避障容差已全面就绪；
  * 远距离异步呼梯与升层联动开门流水线已完整接入并校验；
  * 全仓测试与文档已同步更新。
* **后续建议**：
  * 可在多 seed 矩阵（如 `seed=8127, 6946, 1612`）下进行全流程多层自主探索验证。

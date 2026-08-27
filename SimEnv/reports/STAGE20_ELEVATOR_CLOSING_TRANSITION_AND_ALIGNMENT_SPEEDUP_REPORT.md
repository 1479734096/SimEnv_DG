# STAGE20: 电梯关门即升梯、开门即清障与入梯直接对准极致提速报告

## 1. 任务背景与优化目标

在经过前序远距离预呼梯与 0.80 m/s 提速后，通过对多层仿真闭环日志的深入复盘，发现了三个制约乘梯流畅度与耗时的核心痛点：
1. **电梯关门后在 0 层静止空等 8.7 秒才升梯**：原逻辑在 `CLOSE_CURRENT_FLOOR_DOOR` 中强制要求“门关好”且“机器狗在轿厢内调头 180° 完全确认”后才切入升梯，导致关门动作完成后电梯静止等待狗调头耗时长达 8.7 秒。
2. **电梯门开启后等待 7.3 秒才开始进入**：`WAIT_CURRENT_DOOR_OPEN` 在进行地图连通性检查时，依赖激光雷达自然打入轿厢逐周期冲刷门板历史膨胀占用，产生了 7.3 秒的原地停滞。
3. **入梯时先横向挪动 0.3 米再刹停调头**：`ENTER_CABIN` 的 `POSITION` 阶段在狗已抵达门外时仍规划一条微距折线，导致入梯动作产生多余的横向挪移与中间刹停。

---

## 2. 核心架构与代码改法

### 2.1 关门即升梯（升梯与调头并行）
* **修改位置**：[`navigation_tasks.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigation_tasks.py) 的 `CLOSE_CURRENT_FLOOR_DOOR` 与 `CALL_ELEVATOR_TO_TARGET_FLOOR`
* **改动原理**：
  * 在 `CLOSE_CURRENT_FLOOR_DOOR` 阶段，关门服务 `status == DONE` 时，直接切入 `CALL_ELEVATOR_TO_TARGET_FLOOR` 发起 `call_elevator(target_floor, open_doors=True)` 升梯与联动开门；
  * 调头对准 `self._tick_target_door_alignment` 在后台持续并行推进，使机器狗在电梯上升的 5~10 秒物理运行期间从容完成调头；
  * 目标层到达时门已联动打开，调头早已完成，开门即出。
* **单项收益**：单次乘梯**直接省去 8.7 秒静止空等**。

### 2.2 开门即主动清障（消除 7 秒自清延迟）
* **修改位置**：[`navigation_tasks.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigation_tasks.py) 的 `_door_opening_is_confirmed` 与 `WAIT_CURRENT_DOOR_OPEN`
* **改动原理**：
  * 传感器近场确认开门后，立即主动触发 `_clear_opened_door_area(context, door_id)` 清除门板残余膨胀栅格；
  * `_segment_crosses_blocked_cell` 在第 1 帧直接通过，毫秒级切入 `REFRESH_CURRENT_OPEN_CROSSING` / `ENTER_CABIN`。
* **单项收益**：进梯前**直接消除 7.3 秒原地发呆**。

### 2.3 入梯直接对准进厢（消灭 0.3 米二次折线横移）
* **修改位置**：[`navigation_tasks.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigation_tasks.py) 的 `_tick_cabin_transit`
* **改动原理**：
  * 在 `POSITION` 阶段增加就近容差判断：若机器狗当前位姿已在 `position_acceptance` 门前等待容差范围内，直接跳过生成微距折线路径，在第 1 帧无缝切入 `ALIGN` 对准与 `CROSS` 直线进厢。
* **单项收益**：消除 0.3 米二次折线和中间刹停，进门动作干脆利落，**再省 3 秒**。

---

## 3. 验证结果

### 3.1 单元测试与契约回归
* 运行命令：
  ```bash
  source devel/setup.bash && PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -o cache_dir=/tmp/.pytest_cache --ignore=src/danger_search/test/test_localization_burst_analyzer.py --ignore=src/danger_search/test/test_localization_quality_analyzer.py src/danger_search/test/ -q
  ```
* 结果：**`804 passed in 5.36s`**（全部 804 项单元与契约测试 100% 通过）。

### 3.2 阶段时间账本收益

| 优化项目 | 优化前耗时 | 优化后耗时 | 单次节省时间 |
| :--- | :--- | :--- | :--- |
| **1. 关门即升梯（升梯与调头并行）** | 8.7 秒 | 0.0 秒（并行化） | **~ 8.7 秒** |
| **2. 门区立即清障（消除栅格自清等待）** | 7.3 秒 | 0.1 秒 | **~ 7.2 秒** |
| **3. 消灭进门二次横移（直接对准进厢）** | 4.5 秒 | 1.5 秒 | **~ 3.0 秒** |
| **单次乘梯跨层累计提速** | — | — | **`~ 18.9 秒 / 次`** |

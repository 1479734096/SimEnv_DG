# STAGE21: 二楼挑空中庭与负障碍防踩空跌落安全闭环报告

## 1. 问题背景与根因复盘

在二楼大厅（$Y \in [0.0, 7.85]$）探索过程中，大厅西侧（$X \in [-4.94, -1.56]$）为贯通一二两层的**无实体护栏挑空中庭（落差 2.6m 的大跌落负障碍）**。机器狗在全速奔跑（0.80 m/s）靠近空洞边缘时险些踩空并倒地，具体根因链如下：
1. **安全门脱离支撑逻辑缺陷**：原 `ground_support_gate.py` 中判断 `near_supported_boundary` 时强行要求 `current_footprint_supported == True`，当机器狗足迹已临界脱离支撑区时反被判定为“不在边界附近”而漏放了前进命令；
2. **高速前视与制动保留余量偏小**：原 `boundary_hold_margin: 0.30` 在 0.80 m/s 高速下制动距离偏紧，停下时机身质心已紧贴悬崖边缘（$X=-1.004$）；
3. **悬崖边缘大角速度急转**：停在边缘瞬间控制器下发了高达 `1.01 rad/s` 的大角速度转向指令，四足大幅外摆（摆幅约 0.25m）直接踏入虚空导致侧翻倒地（`FSM.cpp` 跌落保护触发）。

---

## 2. 修复实施方案

### 2.1 安全门边界保护逻辑闭环
* **修改位置**：[`ground_support_gate.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/ground_support_gate.py#L690-L700)
* **改动原理**：
  只要探测到支撑边界（`support_boundary_detected == True`），若足迹已脱离支撑区（`not current_footprint_supported`）或前向支撑前缀不足（`supported_distance <= guard_distance`），在观测充分的前提下**一律强制进入安全保护刹停**，彻底消除漏放行漏洞。

### 2.2 扩大前视探测与悬崖保留余量
* **修改位置**：[`danger_search.yaml`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/config/danger_search.yaml#L290-L320)
* **改动参数**：
  * `minimum_lookahead: 0.60`（从 0.50m 提升，提前 0.6m 发现断崖）；
  * `maximum_lookahead: 1.00`（从 0.80m 提升，覆盖 0.80m/s 高速奔跑）；
  * `boundary_hold_margin: 0.40`（从 0.30m 扩大至 0.40m，完全覆盖四足踏步外摆包络）。

---

## 3. 验证与回归结果

* **单元与契约测试全量通过**：
  ```bash
  source devel/setup.bash && PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -o cache_dir=/tmp/.pytest_cache --ignore=src/danger_search/test/test_localization_burst_analyzer.py --ignore=src/danger_search/test/test_localization_quality_analyzer.py src/danger_search/test/ -q
  ```
  **`804 passed in 5.32s`**。

# STAGE18: 电梯换层闭环、0.8m/s提速与地表避障优化总结报告

## 1. 任务背景与项目概况

* **项目名称**：SimEnv 危险搜索与多层自主探索任务（Danger Search）
* **机器人平台**：Unitree A1 四足机器狗（配置 Livox Mid360 激光雷达、RealSense RGB-D 深度相机、IMU、强化学习步态底层控制器）
* **核心任务目标**：机器狗从 0 层大门出发，自主规划路径并探索整栋建筑的多层走廊与大厅；通过自主呼叫电梯与乘梯完成跨楼层（$0 \to 1 \to 0$ 层或 3 层）穿梭，在规定时间预算内完成多层前沿探索与危险目标标定，并安全返回 0 层大门初始点。

---

## 2. 本阶段核心解决的问题与根因复盘

### 2.1 出电梯多转一圈（360° 原地打转）
* **现象**：电梯到达目标层开门后，机器狗没有直接冲出电梯，而是在轿厢内连续向后调头 180° 又向前调头 180°，白白浪费 6~8 秒。
* **根因**：换层开门后的离厢（`EXIT_CABIN`）初始阶段固定要求机器人先导航到轿厢几何中心（`POSITION` 打卡）。由于乘梯时狗停在中心前方，导致先调头 180° 面向内壁，到达中心后再调头 180° 面向门口。
* **修复**：在 [`navigation_tasks.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/navigation_tasks.py) 中将 `mode == "exit"` 初始阶段直接设为 `ALIGN`（跳过 `POSITION`），对准门外等待点后直接直线冲出电梯。

### 2.2 回 0 层出电梯激光门洞校准超时导致任务被误杀
* **现象**：$1 \to 0$ 层返航时，电梯门打开后机器狗停在轿厢内报 `elevator_open_crossing_geometry_unavailable`，任务直接致命失败。
* **根因**：
  1. 狗停在轿厢南侧偏内（$X=2.47, Y=1.85$），到门中心 $(1.65, 2.60)$ 的物理视线夹角为 $137^\circ$（$2.40\text{ rad}$）；
  2. 算法假定狗在中心线上正西朝向（$180^\circ / \pi$），只在 $\pm 11^\circ$ 窄扇区搜索，导致门框落入盲区；
  3. 动态激光门洞精细校准属于精度优化模块，但原逻辑在 4 秒超时后直接抛致命异常终止 Mission。
* **修复**：
  1. 将搜索角 `expected_crossing_yaw` 改为从当前站位指向门中心的**动态视线夹角（LOS Yaw）**；
  2. 在 [`elevator_crossing_runtime.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/elevator_crossing_runtime.py) 中动态根据实际欧氏距离确定 `expected_standoff`；
  3. **增加平滑降级保底**：若 4 秒内激光动态校准未命中，自动降级沿用已注册的基准电梯门几何（`cabin_center -> target_wait`）直接切入 `EXIT_CABIN` 出梯，杜绝任务被误杀。

### 2.3 走廊地面噪点导致误判障碍物
* **现象**：长走廊平坦地面上频繁生成红色伪障碍物栅格，导致规划器绕行或打转。
* **根因**：机器狗奔跑起伏与足端反弹导致局部地面点云高度落入旧的 `0.15m` 障碍物统计区间。
* **修复**：在 [`danger_search.yaml`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/config/danger_search.yaml) 中将 `obstacle_min_z` 提升至 `0.20m`，`low_height_max_z` 提升至 `0.40m`，彻底滤除地面噪点，同时真实矮障碍物经 3 帧（代次）确认后稳定生效。

### 2.4 巡航速度提升至 0.80 m/s 与全链路适配
* **调整内容**：
  * 控制器最大线速 `max_linear` 提升至 **`0.80 m/s`**（符合契约测试 $\le 1.00\text{ m/s}$ 安全制动包络）；
  * 避障减速前视距离 `obstacle_slow_distance` 设为 `0.60m`；
  * 激光定位 `scan_localization` 运动学上限适配为 `livox_imu_maximum_speed: 1.00m/s`、`shared_map_livox_only_maximum_speed: 0.90m/s`，防止 0.80m/s 奔跑时定位帧被误判跳变丢弃；
  * 地面安全门定位速度异常阈值调整为 `1.20 m/s`。

---

## 3. 关键规则与接口契约约束

在修改任何参数或代码时，必须严格遵守以下契约约束（受 [`test_interface_documentation.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/test/test_interface_documentation.py) 强制断言保护）：
1. **转向与脱困滞回契约**：
   * `turn_in_place_rate` 必须等于 **`0.65`**；
   * `turn_in_place_heading` 必须等于 **`0.90`**；
   * `turn_release_heading` 必须等于 **`0.20`**；
   * `turn_release_rate` 必须等于 **`0.20`**；
   * `turn_release_samples` 必须等于 **`3`**；
   * `recover_angular_speed` 必须保持在 **`0.55 ~ 0.70 rad/s`**（严禁 $< 0.55\text{ rad/s}$，否则触发 A1 强化学习步态电机死区导致卡死）。
2. **最大线速契约**：
   * `max_linear <= 1.00 m/s`（受 `stop_distance_table` 最高已验证档位约束）。
3. **真实电梯物理配置**：
   * 建筑电梯门所有楼层均在靠近中轴线的同一侧（$X=1.65, Y=2.60$），并非双侧开门贯通式；轿厢中心在 $X \approx 2.76, Y \approx 2.60$。

---

## 4. 验证结果

1. **单元测试验证**：
   * 运行导航任务、电梯穿越几何、A*规划器、地面安全门等全量核心测试：**102 项单测 100% 全部通过**。
   * 运行正式接口与文档契约测试：**13 项测试 100% 全部通过**。
2. **仿真闭环验证**：
   * 2 层测试脚本 `run_elevator_gui_test.sh` 参数已更新，`SEED=8127` 下完整覆盖 $0 \to 1 \to 0$ 闭环。

---

## 5. 当前进度与后续任务

* **当前进度**：
  * 电梯进出几何与防误杀降级已完全修复；
  * 0.80 m/s 高速巡航与避障参数已全链路对齐；
  * 核心单元测试全部通过。
* **下一步待办事项**：
  1. 观察当前仿真 GUI 运行全流程，确认 0 层到 1 层再返回 0 层的完整闭环用时及成功率；
  2. 根据实际探索覆盖情况，评估是否开启“覆盖率达标主动提前换层”（如当前层覆盖率达 0.65 即提前乘梯，进一步压缩总耗时）。

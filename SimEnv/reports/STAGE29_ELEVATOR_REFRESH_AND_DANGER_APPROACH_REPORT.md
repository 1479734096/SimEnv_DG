# STAGE29 阶段报告：电梯门前近场重定位恢复与远距离危险源主动走近核验（历史命名，见勘误）

**日期**：2026-08-28  
**责任模块**：`mission_manager.py`, `detection_tracker.py`, `exploration_runtime.py`, `mission_config.py`, `danger_search.yaml`  
**验证状态**：全量单元与契约测试 826 项 100% 通过  

> **2026-08-28 后续勘误**：本报告当时将 `elevator_geometry_refresh_enabled`
> 称为“近场重定位”并描述为“清零漂移”，该表述超出代码事实。STAGE29 只刷新
> `TakeElevator` 的运行期语义门几何，不修改 odom、定位地图、轨迹或危险源坐标。
> 位姿与 Adaptive Shared Map 的有界原子平移在 STAGE33 才实现。后文“彻底消除远距误差”
> 和“收敛至厘米级”同样只有算法与单元测试依据，没有 Gazebo 全场验证，不能作为已达成指标。
> 本报告后文保留为历史原记录。

---

## 一、 背景与根因分析

在 2026-08-27 进行的 `seed=8127` 双楼层大循环测试（总里程 214.13m）中，发现了两项直接影响闭环成功率与目标精度的关键问题：

1. **返程 1→0 电梯开门确认超时死锁**：
   - 机器狗在 1 楼探索并全速折返电梯厅后，沿长走廊 Y 轴产生了 `0.793m` 的 SLAM 里程计累积漂移；
   - 机器狗实际停在距电梯门洞右侧 `35cm` 的实体水泥墙前；
   - 排查发现，原本用于在门前 `1.2m` 远视点扫描门框深色面板并清零漂移的 `REFRESH_CURRENT_ELEVATOR_GEOMETRY`，在 `mission_manager.py:433` 中被硬编码写死了 `elevator_geometry_refresh_enabled=False`（早期为避开膨胀岛卡死临时关闭，但膨胀岛问题已在 `planner.py` 的 Escape Mode 中根本解决）；
   - 导致机器狗直接绕过门前重定位走到墙前，前向传感器直视墙壁，开门检查超时报错。

2. **远距离检测危险源坐标偏差放大**：
   - 在 0 楼房间门口检测到红色球体危险源时，虽然分类与干扰排除准确率达 100%，但输出世界坐标与真值偏差达到 `0.71m`；
   - 经数据链路分析，由于机器狗在房间门口眺望（距离目标 3.2m），RGB-D 深度相机在远距离的反投影几何放大贡献了约 `0.47m` 误差，叠加当时机器狗自身的 `0.24m` SLAM 漂移，放大了全局坐标误差；
   - 探索机制为被动感知，门口激光扫亮房间未知栅格后直接掉头去下一个房间，没有主动走近精测。

---

## 二、 核心改进与技术实现

### 1. 恢复电梯门前近场静态重定位 (`elevator_geometry_refresh_enabled`)
- **配置与上下文解耦**：在 `mission_manager.py:433`、`mission_config.py` 和 `danger_search.yaml` 中将 `elevator_geometry_refresh_enabled` 恢复为可配置的 `true`；
- **重定位闭环**：机器狗导航到达距电梯门 `1.2m` 的观察点后，对准门框执行点云拟合，修正全场累积的走廊纵向滑移与横向漂移，再根据校正后的物理门中心对准近门点（0.8m），确保正对门洞开门。

### 2. 远距离危险源主动走近核验 (`Active Danger Approach Verification`)
- **目标跟踪与状态管理** (`detection_tracker.py`)：
  - `Track` 数据结构增加 `inspected: bool = False` 标记；
  - 提供 `get_pending_inspection_target(robot_xy, min_observe_dist=2.2, min_confidence=0.70)` 查询远距离高质量候选；
  - 提供 `mark_inspected(target_xy, radius=1.2)` 标记已发起核验，防止反复打断常规探索；
  - 走近后采集的大量近距离样本（<1.5m）通过现有的 `_calculate_weighted_position()` 逆距离平方加权算法，自动将整体坐标加权收敛至厘米级精度。
- **近场停靠航向点计算** (`exploration_runtime.py`)：
  - 实现 `compute_danger_approach_pose(grid, robot_pose, sphere_xy, standoff_distance=1.3)`；
  - 沿射线方向计算距红球 1.3 米的停靠点与正对红球的航向角 `yaw = atan2(dy, dx)`，并确保落在自由可通行的 FREE 栅格。
- **探索主循环调度与安全预算拦截** (`mission_manager.py`)：
  - 在 `_tick_explore` 中：路径空闲且楼层时间预算充裕（`floor_elapsed < floor_timeout * 0.75`）时，优先规划前往近场核验点的 A* 路径（`reason="danger_approach_verification"`）；
  - 走近多帧近距离采样完成后，无缝切回常规 Frontier 前沿探索。

---

## 三、 测试与验证结果

1. **单元测试与回归**：
   - `test_detection_tracker.py`：新增待核验目标检索与标记流转测试通过；
   - `test_frontier_arrival_gain.py`：新增停靠位姿几何解算测试通过；
   - `test_command_authority_baseline.py`：更新 `set_path` 调用点唯一归因断言为 15 处，全部通过；
   - **全量 826 项单元测试全部绿灯通过（0 失败，0 回归）**。

2. **交付工件**：
   - `SimEnv/src/danger_search/danger_search/detection_tracker.py`
   - `SimEnv/src/danger_search/danger_search/exploration_runtime.py`
   - `SimEnv/src/danger_search/danger_search/mission_manager.py`
   - `SimEnv/src/danger_search/danger_search/mission_config.py`
   - `SimEnv/src/danger_search/config/danger_search.yaml`
   - `SimEnv/src/danger_search/test/test_detection_tracker.py`
   - `SimEnv/src/danger_search/test/test_frontier_arrival_gain.py`
   - `SimEnv/src/danger_search/test/test_command_authority_baseline.py`

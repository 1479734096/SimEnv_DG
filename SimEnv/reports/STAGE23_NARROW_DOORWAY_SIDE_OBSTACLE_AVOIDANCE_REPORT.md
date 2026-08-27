# STAGE23 技术报告：窄门侧壁与房间拐角前向避障误判死锁修复

## 1. 背景与根因定位

在双层全流程仿真测试（日志 `/tmp/danger_search_elevator_test_20260827_152040/mission.log`）中，机器狗完成一楼探索后，从一楼西北角最深处房间穿过门洞（$X=-1.40\text{m}, Y=14.33\text{m}$，门宽约 0.85m）返回电梯时发生死锁停滞：

1. **上层 A\* 全局规划正常通过**：
   - 栅格地图中机身周围 0.25m 保护轨迹（`trail_protection_radius = 0.25`）正常生效，A\* 成功规划出穿门返回电梯的有效路径（`result: "reached"`）。
2. **底层激光雷达实时前向避障发生几何误判**：
   - 机器狗机身宽 0.30m，穿过 0.85m 门洞时身体右侧距离门框立柱约 `0.235m`；
   - 门框点云位于机身坐标系中的 $\text{forward} = 0.005\text{m}$，$\text{lateral} = 0.235\text{m}$（实际位于狗身体右侧腰部/肋骨处）；
   - 在 `map_update_runtime.py` 的 `nearest_forward_obstacle_observation` 中，旧逻辑判定 `forward >= 0.0` 且 `lateral <= 0.25m` 即视为“前向障碍”，并将欧氏距离 $\text{distance} = \sqrt{0.005^2 + 0.235^2} = 0.235\text{m}$ 误报为前向障碍物距离；
   - 控制器底层根据 `obstacle_stop_distance = 0.40m` 判定当前距离 $0.235\text{m} \le 0.40\text{m}$，以为正前方即将撞墙，强制切断线速度下发 `linear = 0.0`，引发持续 0 速死锁。

---

## 2. 解决方案与实现

### 2.1 纯粹几何过滤（KISS 原则）
- **文件**：[`SimEnv/src/danger_search/danger_search/map_update_runtime.py`](file:///home/xbh/tzb26/codeDG/SimEnv/src/danger_search/danger_search/map_update_runtime.py)
- **改动**：
  ```python
  dx = px - pose[0]
  dy = py - pose[1]
  forward = dx * cos_yaw + dy * sin_yaw
  lateral = abs(-dx * sin_yaw + dy * cos_yaw)
  distance = math.hypot(dx, dy)
  if forward < 0.20 or lateral > 0.22 or distance > max_distance:
      continue
  ```
- **核心逻辑**：
  - 四足机器狗机头/前胸位于 `forward ≈ +0.25m`；
  - 只有真正位于机身前方的点（`forward >= 0.20m`）且在行进横向碰撞包络内（`lateral <= 0.22m`）才属于前向行进阻挡；
  - 位于机身侧面腰部/肋部（`forward < 0.20m`）的点云一律跳过，不参与前向避障刹停计算，彻底根除把侧方门框当成前方障碍物的假阳性。

---

## 3. 测试与验证结果

### 3.1 单元与契约测试
```bash
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -o cache_dir=/tmp/.pytest_cache \
  --ignore=src/danger_search/test/test_localization_burst_analyzer.py \
  --ignore=src/danger_search/test/test_localization_quality_analyzer.py src/danger_search/test/ -q
```
**结果**：`810 passed in 6.16s`（100% 通过）。

### 3.2 新增测试用例
- `test_forward_obstacle_ignores_side_doorposts_near_waist`：验证腰部侧方门框点云（`forward = 0.01m, lateral = 0.23m`）被正确忽略，不产生前向障碍物观测；
- `test_nearest_forward_obstacle_observation_filters_to_corridor_and_height`：验证正前方行进碰撞区内的障碍物依然能被可靠探测并输出精确制动测距。

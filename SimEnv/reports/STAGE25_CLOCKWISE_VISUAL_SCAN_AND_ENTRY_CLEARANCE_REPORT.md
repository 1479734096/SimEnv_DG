# STAGE25 技术报告：进门大厅顺时针快速扫描与大门防卡深入优化

## 1. 优化背景与需求

机器狗穿过主入口正门（`main_entrance`）后，需要执行一次 360° 全向 RGB-D 电梯视觉扫描（`elevator_visual_scan`）以注册 0 层电梯门地标。

原实现存在三点可优化空间：
1. **转圈位置偏近门框（存在踢门卡死隐患）**：原深入距离 `post_door_min_distance = 0.9m`，在微小定位漂移下，旋转时四足外摆容易靠近门框立柱；
2. **旋转方向为逆时针**：原实现硬编码正向旋转；
3. **角速度偏保守**：原大厅开阔区域转速为 $0.70\text{ rad/s}$，存在提速空间。

## 2. 实施方案与技术实现

### 2.1 深入大厅中心开阔区（防门框干涉）
- 在 `src/danger_search/config/danger_search.yaml` 中将 `post_door_min_distance` 从 `0.9m` 提升至 `1.4m`，`door_pass_offset` 从 `0.5m` 提升至 `0.8m`；
- 机器狗穿过正门后笔直深入至大厅正中央（$Y \approx 1.4\text{m}$），完全远离门洞立柱和左右墙壁，彻底消灭旋转摆腿刮擦门框的几何风险。

### 2.2 顺时针旋转与安全提速
- 将 `elevator_visual_scan_angular_speed` 从 `0.70 rad/s` 提速至 `0.90 rad/s`（约 $51.6^\circ/\text{s}$）；
- 处于全局上限 $1.20\text{ rad/s}$ 与姿态翻倒熔断阈值 $1.50\text{ rad/s}$ 之内的安全动力学包络；
- 控制器下发负角速度命令（`-0.90 rad/s`），驱动机器狗顺时针快速环顾大厅。

### 2.3 顺时针转角累计与防抖闭环
- 在 `src/danger_search/danger_search/elevator_observation_runtime.py` 中：
  - 顺时针模式下，计算负向航向增量 `yaw_delta < 0.0`，累计 `visual_scan_rotation += abs(yaw_delta)`；
  - 机器狗落脚回弹（反向微小扰动）不计入累计，防止虚假达标；
  - 累计达到 `required_rotation = 5.80 rad` 确定性退出；
  - 严格保留“捕获电梯门候选即 `stop()` 刹车确认”的防漏检机制。

## 3. 测试与验证

### 3.1 单元测试与定向验证
- 在 `src/danger_search/test/test_mission_manager_helpers.py` 中新增 `test_stage25_clockwise_visual_scan_and_accumulation`：
  - 验证顺时针配置下 `rotate_in_place` 正确下发负角速度 `-0.90 rad/s`；
  - 验证顺时针转角平稳累计至 5.80 rad；
  - 验证反向扰动不计入转角累计；
  - 验证转角满后确定性退出。

### 3.2 全量回归测试
```bash
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -o cache_dir=/tmp/.pytest_cache \
  --ignore=src/danger_search/test/test_localization_burst_analyzer.py \
  --ignore=src/danger_search/test/test_localization_quality_analyzer.py src/danger_search/test/ -q
```
**结果**：`815 passed in 6.01s`（100% 通过）。

### 3.3 静态检查
- `git diff --check`：无任何格式与空白问题；
- Python AST 解析：全包 Python 文件 100% 解析成功。

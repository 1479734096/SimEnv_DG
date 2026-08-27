# STAGE27 技术报告：换层 Livox 目标门确认与历史地图复用

## 1. 问题与证据

seed `8127` 的完整往返测试在 `297.416s` 终止：
`TakeElevator(1->0)` 已完成目标层电梯门服务，但目标门确认在 8 秒后超时。
结束事件显示 `visual_confirmations=0`、`livox_confirmations=0`，RGB-D 最后一帧的
有效深度比例为 `0.897`，更深回波比例仅为 `0.0467`。

源码核对表明，`livox_confirmations=0` 并非 Livox 数据中断，而是显式换层期间
`confirm_door_open()` 主动禁止了 Livox 分支。同期日志仍持续选择 Livox 定位源，点云匹配
计数继续增长。

第二个根因位于点云生命周期：换层守卫累计丢弃了 156 帧点云，原逻辑只更新时间戳，
不更新 `latest_cloud_points`。因此仅解除 Livox 禁用会把运输前的旧楼层点云交给目标门
检测器，必须同时修正点云的目标楼层切片。

## 2. KISS 修改

### 2.1 复用现有点云缓存

`PerceptionMappingRuntime.cache_cloud()` 复用正式建图相同的点云解码、限点、步进和楼层坐标
转换。显式换层被门控的合法点云按 `guard.target_floor` 转换后，直接更新现有
`last_cloud_stamp` 与 `latest_cloud_points`，但不调用 Mapper。

非显式楼层歧义仍直接消费并丢弃点云，避免扩大行为范围。本轮没有新增独立换层缓存、
临时帧类型或生命周期状态。

### 2.2 目标层只使用 Livox 确认

显式换层时，目标门检测的 `floor_index` 使用 `guard.target_floor`。目标层出口不再接受
RGB-D 确认，只复用现有 `detect_opening_ahead()`：

- 检测朝向由当前位姿指向注册门中心；
- 开口中心须落在既有几何容差内；
- 开口宽度与注册门宽差不得超过 `0.50m`；
- 连续两个不同时间戳的新帧匹配后放行，失配帧清零，重复时间戳不计数。

Livox 确认来源记录为 `livox_registered_opening`。当前层候梯、开门和进厢仍保留原有
RGB-D/Livox 行为，8 秒目标门确认超时也保持不变。

### 2.3 地图处理

多楼层历史地图继续独立持久保存。换层期间不写图、不清图、不推进 map epoch；目标门确认
成功后，任务只清理目标楼层电梯门的动态条带，再调用既有楼层确认和两帧里程计对齐。
后续继续使用现有 Livox 局部穿越几何；空观测时沿用 STAGE26 的 1 秒/10 帧早退并降级到
已注册几何。一层历史地图、主入口轨迹和大门外原路返航均不重置。

## 3. 验证

定向回归覆盖电梯观测、局部穿越、任务状态机、换层守卫、感知提交边界和返航：

```bash
source devel/setup.bash
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest \
  -o cache_dir=/tmp/.pytest_cache \
  src/danger_search/test/test_elevator_observation_runtime.py \
  src/danger_search/test/test_elevator_crossing_runtime.py \
  src/danger_search/test/test_navigation_tasks.py \
  src/danger_search/test/test_mission_manager_helpers.py \
  src/danger_search/test/test_mission_runtime_boundaries.py \
  src/danger_search/test/test_return_home_runtime.py -q
```

结果：`314 passed`。

全量 Python 回归结果：`826 passed in 4.03s`。此外：

- `python3 tools/offline/check_production_boundary.py`：通过；
- `git diff --check` 与 `git diff --cached --check`：通过；
- 未新增禁用真值、metadata、ROS 话题、服务或控制出口。

## 4. 未验证事项与风险

本轮按约定未启动 Gazebo，因此尚未取得修改后 seed `8127` 的完整往返证据。

`detect_opening_ahead()` 要求 Livox 同时观察到门洞两侧竖直结构。STAGE26 的历史运行曾在
已确认开门后的出厢视角连续返回空开口观察，因此本次 KISS 方案仍可能受轿厢视角和点云
稀疏性影响。若复测仍超时，应先记录 `/scan` 并核对两帧候选，不在缺少运行证据时继续
放宽几何门槛或绕过传感器确认。

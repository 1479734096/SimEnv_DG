# STAGE26 技术报告：出厢无效观察早退与大门外原路返航

## 1. 问题与证据

seed `8127` 的运行记录显示，主入口外返航在任务层已经生成并下发
`return_home_entry_trace_path`。初始倒序轨迹共 7 个点，首段实际轨迹长度与直线距离比约
`1.001`，说明原路返回能力本身存在且路径几乎笔直。

返程变曲的直接原因是地面支撑安全门，而不是返航规划器：门外约 1.3m 处连续进入
`OBSERVATION_HOLD`，随后产生 `ground_support_path_unavailable`，安装临时支撑约束并清除
原路径，才改用 A* 绕行。该轮共有 13 次观察停车、11 次支撑约束不可用，
`true_drop_stop_count=0`，没有显式坑洞证据。

目标层出厢也没有发生 CROSS 卡滞。开门确认后约 40 个点云帧都返回
`opening_not_observed`，任务仍等待原 4 秒几何刷新超时，然后才使用已注册门几何继续；
CROSS 实际约 2.8 秒且无持续零速卡住证据。

## 2. 实施内容

### 2.1 受限原路返航作用域

- `InitialEntryRuntime` 在首次轨迹采样时记录楼层、地图 epoch、定位 epoch；连续性发生变化时
  轨迹作用域失效，不允许把旧轨迹当作当前安全来路。
- `ReturnHomeRuntime` 新增冻结的 `TrustedEntryTraceScope` 和折线偏差判断。只有返航状态、主入口
  已通过、路径仍为 `return_home_entry_trace_path`、scope 完全匹配且距轨迹中心线不超过
  `0.35m` 时，MissionManager 才关闭普通 `UNKNOWN` 观察停车。
- 该开关只影响 UNKNOWN。安全门的显式 `DROP`、速度包络、实时穿越障碍、已有约束和返航
  STUCK/失败预算继续生效；轨迹不满足规划约束时也不会启用该作用域。

### 2.2 出厢空观测早退

- `ElevatorCrossingConfig` 增加 `exit_empty_fallback_timeout=1.0s` 与
  `exit_empty_fallback_frames=10`。
- 仅 `mode="exit"` 统计连续新点云的空开口观察；两项阈值同时满足后返回
  `elevator_open_crossing_geometry_not_observed`，任务层按既有逻辑降级到已注册几何并继续
  EXIT 的 POSITION/ALIGN/CROSS 安全流程。
- 入厢不触发该早退；出现候选会清零空帧计数；候选不稳定仍受原 4 秒总超时约束。

## 3. 验证

定向测试：

```bash
source devel/setup.bash
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest -o cache_dir=/tmp/.pytest_cache \
  src/danger_search/test/test_return_home_runtime.py \
  src/danger_search/test/test_elevator_crossing_runtime.py \
  src/danger_search/test/test_navigation_tasks.py \
  src/danger_search/test/test_mission_manager_helpers.py -q
```

结果：`288 passed`。

全量 Python 回归：

```bash
source devel/setup.bash
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest \
  -o cache_dir=/tmp/.pytest_cache \
  --ignore=src/danger_search/test/test_localization_burst_analyzer.py \
  --ignore=src/danger_search/test/test_localization_quality_analyzer.py \
  src/danger_search/test/ -q
```

结果：`823 passed in 3.91s`。

本阶段按要求未启动 Gazebo，尚未复跑 seed `8127`，因此原路返航曲线和出厢等待缩短的
运行时效果仍需下一轮仿真确认。

## 4. 风险与下一步

- `0.35m` 轨迹偏差是受限的 UNKNOWN 豁免，不是无条件忽略地面支撑；若实际存在显式 DROP，
  仍必须停车。
- 若复测仍在直线路径上发生定位输出中断，应单独建立定位稀疏区连续性阶段，不在本阶段放宽
  `home_tolerance` 或使用无界 IMU 推算冒充定位。
- 当前测试未覆盖真实 Gazebo 时序；验收重点是原路路径保持、无 UNKNOWN 约束绕行、出厢早退
  小于约 1.5 秒，以及最终 `return_home_completed`。

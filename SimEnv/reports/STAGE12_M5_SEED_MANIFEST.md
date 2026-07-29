# 第 12 层 M5 随机 seed 预登记

## 1. 冻结 seed

2026-07-23 在任何新仿真运行前使用 `shuf -i 100-9999 -n 4` 一次性抽取：

| 顺序 | seed | 用途 |
|---:|---:|---|
| 1 | 1612 | 正式样本 |
| 2 | 8072 | 正式样本 |
| 3 | 6946 | 正式样本 |
| 4 | 4962 | 仅用于替补环境启动失败或 bag 损坏的轮次 |

不得因定位误差大、任务失败、覆盖率低或候选不通过而替换 seed。算法确定性失败保留为
有效结果；只有 Gazebo、ROS master、传感器、控制器启动失败或证据文件损坏才允许使用
第四个 seed，并在报告中记录替补原因。

## 2. 冻结运行配置

```text
FLOOR_COUNT=2
GUI=true
ELEVATOR_TEST_FLOOR_TIMEOUT=33.0
ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5.0
ELEVATOR_TEST_MAX_RUNTIME=180.0
ELEVATOR_TEST_WALL_TIMEOUT=1800
ELEVATOR_TEST_FULL_CYCLE=1
ELEVATOR_TEST_RECORD_DRIFT=1
ELEVATOR_TEST_RECORD_RGBD=0
ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1
ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1
```

生产参数在各轮之间保持不变。证据目录固定为
`logs/stage12_m5/seed_<seed>/`，不放在会被会话清理的 `/tmp`。

## 3. 每轮记录

- 任务是否完成 `0→1→0→返航`，失败时的状态、reason code 和安全事件；
- 平移 RMSE、P95、最大误差、最终误差和 yaw 最大误差；
- RGB-D/Livox 公共同窗整体、低转角、高转角、电梯过渡和出厢窗口指标；
- 最大漂移增长窗口的仿真时间、楼层、任务阶段、定位位置、局部误差方向、转角、
  scan-to-map 累计校正和候选原因；
- 生产节点订阅边界、官方文件哈希、进程清理和证据完整性。

## 4. 候选门槛

Livox 主链候选沿用 M5 检查点冻结门槛。三个正式 seed 中至少两个方向一致才允许进入
生产设计；若不同 seed 结论冲突，则优先研究 GICP 隐含旋转或跨楼层地图生命周期，
不按 seed 写特例，也不继续扫描固定权重。

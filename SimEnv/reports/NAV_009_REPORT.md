# NAV-009 阶段报告

## 问题

Gazebo seed 8127 的主入口 `OpenDoorAndPass` 偶发在通过点附近报告“控制器提前
结束过门路径”。日志显示控制器固定频率回调已经以最新位姿完成终点判定，而任务
10 Hz tick 使用较旧位姿，二者处于不同时间快照。

## 根因

过门直线路径首次提交时没有携带门洞通过阶段的 `corner_tolerance`，控制器可能先
按较宽的默认 `goal_tolerance` 锁存 `DONE`。即使补上提交容差，控制器仍可能比任务
层多执行一帧；任务层若只使用旧 pose，就会把真实完成误判为失败。

## 修改

- 首次提交过门直线路径时原子携带 `goal_tolerance=crossing_tolerance` 和既有
  `door_pass_max_linear`。
- 控制器返回 `DONE` 时，若提供公开 `tracking_debug_snapshot()`，优先用其最新
  有限位姿核验通过点；确实超出通过容差仍保持原失败行为。

## 验证

- 新增测试在未修复代码上分别复现路径提交缺少容差、任务旧 pose 导致 `DONE` 被
  误判为失败。
- 修复后门任务定向测试通过，全量测试 `764 passed, 1 warning`。
- Gazebo seed 8127 已观察到 `OpenDoorAndPass(main_entrance) 完成过门`，随后
  进入电梯视觉扫描并成功注册 `elevator_main/elevator_floor_0`。

## 未完成风险

本阶段只处理主入口通过时序，尚未证明电梯进轿厢和完整二层往返成功。

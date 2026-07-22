# 第 11 层第 4 轮阻塞报告

## 1. 结论

电梯视觉注册修复已连续通过两次 GUI 实测，但完整两层闭环仍不能封口。

- 短测在扫描开始后约 9.7 秒注册电梯门板，注册结果为 `center=(1.52, 2.42)`、`yaw=3.13`、`width=0.74`。
- 完整轮在扫描开始后约 9.3 秒再次注册电梯门板，注册结果为 `center=(1.73, 2.52)`、`yaw=3.14`、`width=0.72`。
- 楼层 0 探索在任务耗时约 280.5 秒后进入 `CHANGE_FLOOR`，但 `TakeElevator(0->1)` 永久停在 `GO_TO_ELEVATOR_DOOR`：A* 未产出路径时，现有实现只零速返回 `RUNNING`，没有超时、失败或替代路径。
- 本轮楼层 0 覆盖率约 0.622，最终确认 1 个危险源；离线真值对照显示楼层 0 有 3 个危险源，因此本层召回为 1/3。

本轮按立即停止条件人工终止，未进入电梯呼叫、进厢、楼层 1 探索和返航闭环。

## 2. 确认事实

### 2.1 电梯视觉注册

短测日志 `/tmp/stage11_mission.log`：

```text
sim 22.810  开始一次全向 RGB-D 电梯门地标扫描
sim 32.512  注册 center=(1.52, 2.42) yaw=3.13 width=0.74
```

完整轮日志 `/tmp/stage11_full_mission.log`：

```text
sim 27.010  开始一次全向 RGB-D 电梯门地标扫描
sim 36.312  注册 center=(1.73, 2.52) yaw=3.14 width=0.72
```

两次注册都发生在 18 秒扫描窗口内，宽度均处于 `[0.65, 0.90)`，来源均为 `rgbd_dark_panel_pending_service_confirmation`。注册后控制器恢复前沿探索，没有扫描超时。

### 2.2 楼层 0 探索

- 主入口检测、近场重观测、动态障碍清理、过门和离开门框均成功。
- 机器人从入口走廊探索到左右两侧区域，覆盖率由约 0.28 增长到 0.622。
- 运行中发生两次局部 `STUCK`，均未形成重复恢复活锁；第二次之后任务层进入有界无前沿扫描，随后重新发现前沿并继续移动。
- 仿真 `293.828` 秒进入 `EXPLORE_FLOOR -> CHANGE_FLOOR`，原因为 `floor_timeout_change_floor_change_floor`。

### 2.3 换层阻塞

视觉注册的门中心为 `(1.73, 2.52)`，朝外方向约为 `pi`，按生产公式推导当前层厅外等待点约为 `(0.93, 2.52)`。进入换层时机器人位于约 `(8.50, 10.46)`。

从仿真 `303.014` 到 `312.928` 秒的 20 个连续 `mission_runtime` 诊断周期均满足：

```text
state=CHANGE_FLOOR
current_task=TakeElevator(0->1)
current_task_phase=GO_TO_ELEVATOR_DOOR
path_active=false
path_size=0
requested_linear_command=0.0
pose≈(8.50, 10.46)
```

停止前继续观察到仿真 `320.842` 秒，状态仍未变化。

### 2.4 危险源结果

本轮停止时输出：

```json
{
  "exploration_time": 307.19,
  "detected_danger_sources": [
    {"position": [5.802, 16.462, 0.074]}
  ]
}
```

生产链停止后才进行离线真值对照。对应楼层 0 真值约为 `(5.342, 16.295, 0.15)`，平面误差约 0.49 米；楼层 0 另有两个目标位于更北侧房间，本轮未覆盖或未确认。

## 3. 根因链

`TakeElevator.GO_TO_ELEVATOR_DOOR` 调用 `_navigate_to()`，后者调用 `_navigate_to_with_status()`。

当 `astar_plan()` 没有返回 `reached` 时，当前实现把“明确不可达”和“规划预算耗尽”合并处理：

```python
if result.reached:
    manager.controller.set_path(result.path)
else:
    manager.controller.stop()
    return RUNNING
```

下一 tick 会以相同地图、相同起点和相同目标重新执行同一逻辑。该分支没有：

- 规划结果分类日志；
- 连续失败计数；
- 超时或 `FAILED` 出口；
- 对已走轨迹的安全回退；
- 给任务层的不可达反馈。

因此，本轮能确定的直接根因是“未产出路径时无界零速重试”。现有日志不能进一步区分该次 A* 是 `unreachable` 还是 `budget_exhausted`，不能在没有新证据时断言具体栅格原因。

## 4. 本阶段累计改动与行为

本轮没有修改生产代码，只验证当前已暂存/未暂存检查点中的既有改动：

- `src/danger_search/config/danger_search.yaml`：18 秒按实际转角扫描，电梯视觉最小宽度 0.65 米。
- `src/danger_search/danger_search/mission_manager.py`：按实际转角扫描、三态电梯摘要、主入口返航近场重观测、电梯视觉宽度默认值同步。
- `src/danger_search/danger_search/rgbd_door_detector.py`：只读拒绝计数诊断。
- `src/danger_search/test/test_mission_manager_helpers.py`：单扇门板注册等回归测试。
- `tools/offline/analyze_rgbd_door_bag.py`：只读录包分析工具。

本轮只新增本报告，没有修改既有报告或说明文档。

## 5. 明确未改动模块

- 未修改 `navigation_tasks.py`，因为该文件不在本检查点允许扩张的修复范围内。
- 未修改官方 `auto.sh`、`unitree_guide`、生成器、赛题文档或既有报告。
- 保留用户已有 `gazebo.xacro visualize=false` 改动。
- 未修改颜色、圆度、深度、RMSE、门板最大宽度或其他视觉阈值。

## 6. 测试与结果

```text
python3 -m pytest src/danger_search/test -q
428 passed in 1.53s

python3 tools/offline/check_production_boundary.py
生产/离线边界审计通过。

git diff --check
通过，无输出。
```

运行期节点订阅抽查保存在：

- `/tmp/stage11_full_danger_node_info.txt`
- `/tmp/stage11_full_localization_node_info.txt`

`danger_search` 与 `scan_localization` 均未订阅 `/ground_truth/*`、`/Odometry_gazebo`、裁判里程计或其转发话题。

官方文件哈希仍只有 4 个既有预期不匹配：

- `auto.sh`
- `src/unitree_guide/unitree_guide/unitree_guide/include/FSM/FSM.h`
- `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/FSM.cpp`
- `src/unitree_guide/unitree_ros/robots/a1_description/xacro/gazebo.xacro`

## 7. 仿真目录与关键日志

本轮采用：

```text
SEED=42
FLOOR_COUNT=2
GUI=true
PAUSED=true
AUTO_UNPAUSE=1
CONTROLLER_FOREGROUND=1
ROBOT_Y=-2.2
```

关键文件：

- 完整轮环境日志：`/tmp/stage11_full_auto_console.log`
- 完整轮任务日志：`/tmp/stage11_full_mission.log`
- 短测任务日志：`/tmp/stage11_mission.log`
- Gazebo 日志：`logs/competition_gazebo.log`
- 结果：`results/detected_danger.json`

场景 `competition_scene.world` 在本阶段前后哈希均为：

```text
bf118195fe38327c331c02798f3cb7bc43d7dfba3dc69055aa27e961f3c3225d
```

检查点描述中的 `87c0ddc611960aad` 没有在仓库脚本、报告或当前产物中找到对应计算方法，不能把它冒充为已验证签名。

## 8. 与基线指标对比

| 指标 | 本轮 | 既有证据/目标 | 结论 |
|---|---:|---:|---|
| 电梯视觉注册 | 2/2 GUI 轮成功 | 旧阈值 0.90 时失败 | 明显改善，修复有效 |
| 注册耗时 | 9.3 至 9.7 秒 | 18 秒上限 | 有界通过 |
| 楼层 0 覆盖率 | 0.622 | 配置完成阈值 0.80 | 未达阈值，靠楼层预算换层 |
| 楼层 0 检测 | 1/3 | 离线真值 3 个 | 召回不足 |
| 换层 | 未开始机械动作 | 目标为 0→1 完整闭环 | 被厅外导航阻塞 |
| 两层返航 | 未进入 | 目标为 1→0→主入口→起点 | 未验证 |

## 9. 残余风险与新问题

1. `GO_TO_ELEVATOR_DOOR` 未产出路径时无界零速重试，是当前首要阻塞。
2. A* 未产出路径的具体分类没有诊断，需先区分 `unreachable` 与 `budget_exhausted`，不能直接放宽地图或改成无校验直连。
3. 楼层 0 覆盖率只有 0.622，且仅检出 1/3 目标；换层修复后仍需单独审计房间覆盖和红球视角。
4. 唯一检测的平面误差约 0.49 米，需按官方评分容差确认是否有效命中。
5. 楼层 1、电梯返回、主入口返航近场重观测仍无本轮实证。
6. ROS 日志目录超过 1 GB，但未自动清理，避免未经确认删除用户日志。

## 10. Git 状态

本轮开始前的既有暂存/未暂存状态保持不变；新增本报告为未跟踪文件。未执行 `git add`、`commit`、`tag`、`push`、`reset` 或 `checkout`。

## 11. 阶段是否可封口

不可封口。

电梯视觉注册子问题可以视为通过，但第 11 层的完整两层闭环被 `GO_TO_ELEVATOR_DOOR` 无界零速重试阻断。

## 12. 下一个最小实验

先做节点/单元级最小复现，不立即重跑 30 多分钟 GUI：

1. 给 `TakeElevator` 的厅外导航补充可观测的规划结果分类和有界失败语义。
2. 构造 A* `unreachable` 与 `budget_exhausted` 两种测试，证明任务不会永久返回 `RUNNING`。
3. 在不穿越 `OCC/BLACK` 的前提下评估是否复用已走轨迹回到电梯；若无安全路径，必须显式 `FAILED` 并进入结果写出，而不是零速活锁。
4. 相关单测通过后，先跑短预算 GUI：让机器人远离电梯后直接触发换层，验证能回到厅外等待点或有界失败。
5. 短预算通过后再重跑完整两层闭环；同时记录楼层 0 漏检房间的覆盖轨迹。

如果交由其他分析者协助，问题描述应保持开放：提供上述运行事实、已做尝试和所需输出，要求其报告 A* 分类证据、最小安全方案、测试设计与合规风险，不预先指定必须采用直连、轨迹回放或降低障碍阈值。

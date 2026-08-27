# STAGE24 技术报告：电梯调头命令仲裁与进厢直通优化

## 1. 问题与根因

前序 seed `8127` 电梯运行证据显示，轿厢调头并不是 `0.65 rad/s` 角速度不足，而是任务层和控制层同时争用 `/cmd_vel`：

- `PathFollower` 的 ALIGN 定时器以 10Hz 发布原地旋转命令；
- `TakeElevator._run_service()` 在异步服务 pending 的每个任务 tick 无条件调用 `controller.stop()`；
- `CLOSE_CURRENT_FLOOR_DOOR` 与 `CALL_ELEVATOR_TO_TARGET_FLOOR` 又在同一阶段并行推进 ALIGN，因此零速和旋转命令持续交替。

前序 bag 统计中，关门阶段零速占比为 `61.1%`，升梯阶段为 `48.2%`。作为对照，专职调头等待阶段没有服务轮询零速干扰，同样的 `0.65 rad/s` 能连续旋转 `2.522 rad`。这说明当前阻塞点是控制权冲突，无需把角速度冒险提高到 `0.85 rad/s`。

历史样本的进厢航向误差最高约为 `0.788 rad`；原 `0.50 rad` 快速路径会把仍可安全直通的样本送入耗时的 `SETTLE/ALIGN/CONFIRM`。因此本阶段采用 `0.80 rad` 作为任务层直通阈值。

## 2. 实现

### 2.1 服务等待的局部控制权仲裁

在 `src/danger_search/danger_search/navigation_tasks.py` 中，将 `TakeElevator._run_service()` 扩展为：

```python
def _run_service(
    self,
    context: NavigationTaskContext,
    task_factory,
    *,
    hold_position: bool = True,
) -> str:
```

默认值保持原有失效安全语义：呼梯到当前层、开门和其他服务 pending 时仍调用 `stop()`。仅以下两个本来就与轿厢调头并行的阶段传入 `hold_position=False`：

- `CLOSE_CURRENT_FLOOR_DOOR`；
- `CALL_ELEVATOR_TO_TARGET_FLOOR`。

这样 ALIGN 成为这两个阶段唯一的运动命令来源。调头完成后，控制器的 ALIGN 终态自行发布并保持零速，服务轮询无需重复干预。关门、升梯和调头仍保持并行，没有退回串行换层流程。

### 2.2 进厢快速直通边界

新增任务层常量 `_CABIN_TRANSIT_DIRECT_ENTRY_HEADING_TOLERANCE = 0.80`：

- 航向误差 `<= 0.80 rad`：在 POSITION 到位的同一 tick 提交直线 `CROSS`；
- 航向误差 `> 0.80 rad`：继续进入 `ALIGN`，完成 `SETTLE/ALIGN/CONFIRM` 后再跨越；
- `CROSS` 本身不启动 heading alignment。

没有修改 `navigator.py`，也没有改变全局 `turn_in_place_heading=0.90` 等路径跟踪契约。

### 2.3 轿厢调头安全边界与超时

轿厢 ALIGN 继续调用 `begin_heading_alignment(..., max_angular=0.65)`。已有控制器测试覆盖：

- `linear.x == 0.0`；
- `linear.y == 0.0`；
- `abs(angular.z) <= 0.65 rad/s`。

专职 `WAIT_CABIN_TURN_COMPLETE` 超时由 `8.0s` 调整为 `12.0s`，为连续控制下约 9～10 秒的最坏收敛时间留出余量。12 秒边界内保持 `RUNNING`，超过后仍记录并返回 `elevator_cabin_turn_timeout`，不会形成活锁。目标门开启确认仍只能在调头完成锁存后启动。

## 3. 自动化验证

### 3.1 失败优先回归

新增测试后、修改生产代码前，定向运行得到 `4 failed, 3 passed`，失败项分别准确复现：

1. 关门 pending 服务插入 `stop()`；
2. 升梯 pending 服务插入 `stop()`；
3. `0.80 rad` 边界未进入 `CROSS`；
4. 原 8 秒超时在 12 秒边界前提前失败。

实施修改后，同一组测试得到 `7 passed`。

### 3.2 覆盖项

- 普通 pending 服务仍调用 `stop()`；
- 关门和升梯并行调头时，pending 服务不调用 `stop()`；
- 调头完成前不启动目标门开启确认；
- `0.80 rad` 边界直接进入 `CROSS`；
- `0.801 rad` 回退 `ALIGN`；
- 快速 `CROSS` 不启动 heading alignment；
- 轿厢调头以 `max_angular=0.65` 启动，控制器保持零线速度和局部角速度上限；
- 12 秒边界内保持 `RUNNING`，超过后确定性失败。

任务层与控制器定向回归：

```bash
source devel/setup.bash && PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest \
  -o cache_dir=/tmp/.pytest_cache \
  src/danger_search/test/test_navigation_tasks.py \
  src/danger_search/test/test_local_controller.py -q
```

结果：`139 passed in 0.55s`。

全量 Python 测试：

```bash
source devel/setup.bash && PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest \
  -o cache_dir=/tmp/.pytest_cache \
  --ignore=src/danger_search/test/test_localization_burst_analyzer.py \
  --ignore=src/danger_search/test/test_localization_quality_analyzer.py \
  src/danger_search/test/ -q
```

结果：`814 passed in 4.16s`。

## 4. 仿真状态与待验收项

按用户要求，本轮未启动 Gazebo，也未复跑 seed `8127`。因此以下运行时结论仍待后续仿真确认，不能由单元测试替代：

- 门前误差约 `0.516 rad` 时运行态直接进入 `CROSS`；
- 关门、升梯期间的旋转命令之间不再出现任务层零速；
- 运行态调头线速度始终为零、角速度不超过 `0.65 rad/s`；
- 不出现 `elevator_cabin_turn_timeout`；
- 完成目标门确认、逻辑楼层更新和二楼出厢。

本阶段没有改动门视觉确认逻辑：现有证据表明失败先发生在调头超时，尚无证据支持扩大视觉容差。本次也未新增依赖、外部话题、服务、参数或真值输入。

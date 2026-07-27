# Stage17 定位矩阵暂停交接

## 1. 当前状态

Stage17 在完成 seed `6946` 后按用户要求暂停。矩阵实现与门槛已经冻结，证据根目录为：

```text
/tmp/stage17_adaptive_shared_map_matrix
```

已完成且证据有效的 seed：`8127`、`1612`、`8072`、`6946`。剩余 seed 按固定顺序为：

```text
4962 42 137 911 2027 3141 4099 5273 6203 7331 8459 9511 10657 11777 12907 14029
```

不要删除或移动现有证据目录，不要新建另一套配置混跑。runner 会校验
`MATRIX_CONFIG.txt`，自动跳过已有有效证据，从 seed `4962` 继续。暂停时曾生成一批
状态码 `130/2` 或没有状态码的空尝试目录；修正后的 runner 会保留这些审计痕迹但不把
它们计入每个 seed 的两次运行额度，因此新的 attempt 编号可能从 `3` 开始。

当前四轮结果：

| seed | 完整闭环 | RMSE | P95 | 末端误差 | 最大误差 | 最大 yaw |
|---:|:---:|---:|---:|---:|---:|---:|
| 8127 | 是 | `0.0920 m` | `0.1242 m` | `0.0345 m` | `0.1665 m` | `0.00264 rad` |
| 1612 | 是 | `0.0694 m` | `0.0960 m` | `0.0917 m` | `0.1559 m` | `0.00283 rad` |
| 8072 | 是 | `0.1077 m` | `0.1320 m` | `0.0750 m` | `0.1586 m` | `0.00343 rad` |
| 6946 | 是 | `0.0814 m` | `0.1277 m` | `0.0499 m` | `0.1583 m` | `0.00334 rad` |

阶段性结论：完整闭环 `4/4`，RMSE `<0.1 m` 为 `3/4`，定位诱发失败为 `0`。
seed `8072` 已使“明显提升”门槛无法满足；“可接受”仍可能满足，剩余 16 轮至少
15 轮 RMSE 必须 `<0.1 m`，至少 14 轮必须完整闭环。

## 2. 固定配置

禁止在续跑中修改以下配置：

```text
backend=adaptive_shared_map
gui=false
floor_count=2
floor_timeout=33
min_floor_explore_time=5
max_runtime=180
wall_timeout=1800
full_cycle=1
record_drift=1
record_rgbd=0
offline_truth=1
localization_diagnostics=1
```

20 个预登记 seed 的完整顺序由 `tools/sim/run_localization_seed_matrix.sh` 固定。必须串行
运行，不能并行启动多个 Gazebo/ROS master。离线真值只用于任务退出后的误差分析，
不得反馈给同轮定位、规划、控制或任务状态。

## 3. 续跑步骤

从工作区根目录执行：

```bash
cd /home/xbh/tzb26/codeDG_rebuild
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ss -ltnp | rg ':11311\b' || true
df -h /tmp
LOCALIZATION_MATRIX_OUTPUT_ROOT=/tmp/stage17_adaptive_shared_map_matrix \
  tools/sim/run_localization_seed_matrix.sh
```

启动前要求 11311 没有残留 ROS master。若存在上一轮残留，先确认它确实属于本矩阵，
再正常终止对应 runner；不要使用宽泛的 `pkill` 或删除证据目录。状态码 `130`（中断）
或 `2`（全局前置失败）会立即暂停整个矩阵且不消耗 seed 尝试额度；其他无效环境证据
最多保留两次尝试。有效算法失败只计结果，不通过反复重跑消除失败。

每轮结束后 runner 会更新：

```text
/tmp/stage17_adaptive_shared_map_matrix/matrix_summary.json
/tmp/stage17_adaptive_shared_map_matrix/matrix_summary.csv
/tmp/stage17_adaptive_shared_map_matrix/matrix_summary.md
```

运行中可查看摘要，不要修改它：

```bash
sed -n '1,80p' /tmp/stage17_adaptive_shared_map_matrix/matrix_summary.md
```

## 4. 跑完后的分析

runner 正常结束后重新生成一次汇总：

```bash
python3 tools/offline/summarize_localization_matrix.py \
  /tmp/stage17_adaptive_shared_map_matrix
```

必须确认 `valid_run_count=20`。不足 20 时结论只能是 `insufficient_evidence`，先从
`matrix_summary.md` 找无效 seed，再检查对应 `attempt_*/TEST_SUMMARY.txt`、`auto.log`、
`mission.log`、`rosbag.log` 和 `RUNNER_EXIT_CODE.txt`，不能用替代 seed 填补。

冻结裁决门槛如下：

- `acceptable`：至少 18/20 RMSE `<0.1 m`，至少 18/20 完整闭环，定位诱发失败为 0；
- `clear_improvement`：在 `acceptable` 基础上，至少 16/20 RMSE `<0.05 m`，且 20/20
  RMSE `<0.1 m`；
- 所有有效轮还必须同时满足 RMSE `<0.20 m`、P95 `<0.20 m`、末端误差 `<0.20 m`、
  最大误差 `<0.35 m`、最大 yaw `<0.005 rad`、平均 odom 频率 `>=8 Hz`、最大输出间隔
  `<=0.5 s`；
- 任一硬门失败或出现定位诱发任务失败，矩阵不得晋级。

由于 seed `8072` 的 RMSE 为 `0.1077 m`，本矩阵已经不可能得到
`clear_improvement`。最终仍需跑完 20 轮，判断能否得到 `acceptable`；不要提前修改
正式 launch 默认值。

## 5. 需要回传的结果

跑完后向主执行者提供以下信息即可：

1. `matrix_summary.md` 全文；
2. `matrix_summary.json` 中的 `verdict`、四个计数和全部 `guards`；
3. 无效、RMSE `>=0.1 m`、未闭环或任一硬门失败的 seed 列表；
4. 每个异常 seed 的 `terminal_outcome`、`reason`、RMSE、P95、末端/最大误差、最大 yaw、
   odom 频率和最大输出间隔；
5. runner 最终退出码，以及确认 11311、Gazebo、rosbag、定位和任务进程均已清理。

原始 bag、真值和生成场景继续保留在 Git 之外。不要提交 `/tmp` 证据，也不要在没有
完整矩阵裁决前修改 `danger_search.launch` 的正式默认后端。

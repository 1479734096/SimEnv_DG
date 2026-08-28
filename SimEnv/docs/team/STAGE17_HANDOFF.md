# Stage17 后续定位矩阵交接

## 1. 当前状态

Stage17 定位漂移优化已按用户决策完成工程验收，正式默认后端为
`adaptive_shared_map`。20-seed 统计矩阵在 seed `2027` 的环境启动失败后暂停；该矩阵
现在用于量化残余风险，不再作为 Stage17 验收或默认切换的前置条件。矩阵实现与门槛保持
冻结，证据根目录为：

```text
/tmp/stage17_adaptive_shared_map_matrix
```

已完成且证据有效的 seed：`8127`、`1612`、`8072`、`6946`、`4962`、`42`、`137`、
`911`。剩余 seed 按固定顺序为：

```text
2027 3141 4099 5273 6203 7331 8459 9511 10657 11777 12907 14029
```

不要删除或移动现有证据目录，不要新建另一套配置混跑。runner 会校验
`MATRIX_CONFIG.txt`，自动跳过已有有效证据，从 seed `2027` 继续。暂停时曾生成一批
状态码 `130/2` 或没有状态码的空尝试目录；修正后的 runner 会保留这些审计痕迹但不把
它们计入每个 seed 的两次运行额度，因此新的 attempt 编号可能较大。

当前八轮结果：

| seed | 完整闭环 | RMSE | P95 | 末端误差 | 最大误差 | 最大 yaw |
|---:|:---:|---:|---:|---:|---:|---:|
| 8127 | 是 | `0.0869 m` | `0.1379 m` | `0.0211 m` | `0.2063 m` | `0.00281 rad` |
| 1612 | 是 | `0.0892 m` | `0.1222 m` | `0.2529 m` | `0.2723 m` | `0.00327 rad` |
| 8072 | 是 | `0.0760 m` | `0.1345 m` | `0.1993 m` | `0.3315 m` | `0.00283 rad` |
| 6946 | 是 | `0.0624 m` | `0.0932 m` | `0.1420 m` | `0.1751 m` | `0.00354 rad` |
| 4962 | 是 | `0.0578 m` | `0.0874 m` | `0.0861 m` | `0.1158 m` | `0.00284 rad` |
| 42 | 是 | `0.1012 m` | `0.1290 m` | `0.0305 m` | `0.1539 m` | `0.00353 rad` |
| 137 | 是 | `0.0693 m` | `0.0974 m` | `0.0101 m` | `0.1309 m` | `0.00292 rad` |
| 911 | 是 | `0.0888 m` | `0.1199 m` | `0.1036 m` | `0.1703 m` | `0.00260 rad` |

工程验收结论：**通过**。现有数据为完整闭环 `8/8`，RMSE `<0.1 m` 为 `7/8`，
定位诱发失败为 `0`。
seed `42` 已使“明显提升”门槛无法满足；seed `1612` 的末端误差 `0.2529 m` 超过
`0.20 m` 硬门，因此原冻结矩阵即使补齐也不能晋级。用户已明确决定接受当前风险，正式
默认直接切换为 `adaptive_shared_map`；继续矩阵只用于量化残余风险，不再作为切换前置。

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
cd /home/xbh/tzb26/codeDG/SimEnv
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

由于 seed `42` 的 RMSE 为 `0.1012 m`，本矩阵已经不可能得到
`clear_improvement`；seed `1612` 的末端硬门也已失败，所以原冻结裁决最终不能晋级。
正式 launch 已按用户决策切为 `adaptive_shared_map`。最终仍建议跑完 20 轮，用于评估
失败率和决定是否需要显式回退到 `rgbd_livox`，不要据离线真值在同一轮动态切换后端。

## 5. 需要回传的结果

跑完后向主执行者提供以下信息即可：

1. `matrix_summary.md` 全文；
2. `matrix_summary.json` 中的 `verdict`、四个计数和全部 `guards`；
3. 无效、RMSE `>=0.1 m`、未闭环或任一硬门失败的 seed 列表；
4. 每个异常 seed 的 `terminal_outcome`、`reason`、RMSE、P95、末端/最大误差、最大 yaw、
   odom 频率和最大输出间隔；
5. runner 最终退出码，以及确认 11311、Gazebo、rosbag、定位和任务进程均已清理。

原始 bag、真值和生成场景继续保留在 Git 之外。不要提交 `/tmp` 证据，也不要用离线
真值结果影响当轮生产定位、规划或控制。

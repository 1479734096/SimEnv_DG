# 第 13.4 层：离线时间与失败账本报告

## 1. 结论

**PASS（仅离线分析层）**。账本工具保持在 `tools/offline`，不进入 catkin 包、launch
或生产依赖。旧命令
`python3 tools/offline/analyze_mission_timing.py <mission.log>` 继续可用，并增加可选的
运行器摘要、正式结果、离线定位误差和离线评分输入。

本层不修改生产节点、任务行为、配置或启动链，也不运行新的 Gazebo。所有真值相关定位
误差和评分只读取任务结束后显式提供的离线摘要，不会回流到同一轮生产决策。

## 2. 修改

- `tools/offline/analyze_mission_timing.py`
  - 按状态、任务阶段、零速原因、运动类型和事件统计耗时。
  - 正式终态优先于启动日志和运行器摘要；没有 `mission.log`、零份或一份运行样本时，
    仍可输出启动/环境失败分类，缺失的连续指标显式为 `null`。
  - STUCK/恢复累计计数独立遍历完整样本序列，不再漏掉末样本增量。
  - 同时输出有效移动、实际已发送平移命令时间、平均/峰值命令速度、原始/过滤里程、
    逐层覆盖率、安全事件、正式识别数、离线定位误差和离线客观评分。
  - 换层段和超过可配置阈值的定位跳变不计入过滤里程，但同时保留原始里程、排除段数
    与排除距离，避免阈值隐藏数据。
- `src/danger_search/test/test_offline_timing_analyzer.py`
  - 覆盖成功终态、算法失败、控制器崩溃、测试夹具失败、零运行样本、末样本计数增长、
    缺失/损坏 sidecar、时间边界、逐层覆盖、安全事件、定位误差和评分读取。

## 3. 输入与缺失语义

| 输入 | 用途 | 缺失语义 |
|---|---|---|
| `mission.log` | 状态、阶段、事件、位姿、覆盖率和控制诊断 | 允许缺失；连续耗时和里程为 `null`，仍可用其他证据分类启动失败。 |
| `--auto-log` | 控制器段错误、站立失败、控制器未就绪 | 未提供时不猜测环境原因。 |
| `--test-summary` | 运行器结果、detail、seed 和历史证据路径 | 可自动读取其中绑定的 `auto_log` 和存在的 `drift_summary`，不自动读取可能陈旧的工作区结果文件。 |
| `--result-file` | 正式 `detected_danger.json` 识别数量 | 不存在、不可读、非法或未显式绑定时为 `null`。 |
| `--localization-report` | 任务结束后 `localization_drift.json` 的 RMSE、P95、最大/最终误差 | 不存在、不可读或无有限指标时为 `null`；只用于离线分析。 |
| `--evaluation-file` | 任务结束后 `evaluation_result.json` 的识别指标与客观得分 | 不存在、不可读或 schema 无效时为 `null`；只用于离线分析。 |

顶层跨楼层覆盖率仍只为兼容旧调用保留；正式 A/B 应使用 `coverage_by_floor`。有效移动和
命令速度是合法定位与已发送命令的采样代理，不是足端物理速度真值。

## 4. 独立历史日志复核

### 4.1 真实启动失败

证据目录：`/tmp/stage13_3_elevator_plane_seed6946`。该轮 `junior_ctrl` 在生成
`mission.log` 前段错误。修复后工具从 `TEST_SUMMARY.txt` 自动定位 `auto.log`，输出：

- `sample_count=0`；
- 连续耗时、速度、里程、覆盖率、识别、定位误差和评分均为 `null`；
- `run_outcome.category=控制器进程崩溃`；
- `terminal_outcome=environment_failed`；
- `reason=controller_segmentation_fault`。

证据：`/tmp/stage13_4_timing_startup_failure_fixed.json`。

### 4.2 第 13.3 层重试轮

- 采样跨度 `119.502 s`，活动态 `118.002 s`；
- 有效移动 `64.400 s`，占比 `0.546`；
- 已发送平移命令 `48.902 s`；
- 平均/峰值已发送平移速度 `0.193/0.597 m/s`；
- 过滤里程 `21.766 m`，原始里程 `21.769 m`，换层排除段 `1`；
- STUCK/恢复 `2/0`；
- 安全事件：`ground_support_gate_transition=3`、`unsafe_rotation_blocked=1`；
- 正式终态为“定位健康失败”，reason 为 `localization_stance_unhealthy`。

逐层覆盖率：

- 0 层：初始 `0.510`、最终 `0.335`、最大 `0.513`；
- 1 层：初始 `0.406`、最终 `0.295`、最大 `0.466`。

证据：`/tmp/stage13_4_timing_13_3_retry_fixed.json`。

### 4.3 第 13.1 层

- 过滤里程 `14.516 m`，原始里程 `14.520 m`；
- 有效移动占比 `0.472`；
- 平均/峰值已发送平移速度 `0.134/0.598 m/s`；
- STUCK/恢复 `2/1`；
- 安全事件：`unsafe_rotation_blocked=1`；
- 正式终态为“电梯或换层失败”，reason 为
  `floor_change_failed_return_home`。

证据：`/tmp/stage13_4_timing_13_1_fixed.json`。旧
`/tmp/stage13_4_timing_13_1.json` 是分类修复前生成的陈旧文件，不再作为正式证据。

上述两轮没有绑定的运行归档结果文件、定位真值 bag 或评分文件，因此识别数、定位误差
和评分为 `null`。代码路径由合成 sidecar 单测覆盖；后续正式 A/B 必须把同轮文件复制到
运行归档后显式传入，禁止读取工作区当前结果冒充历史结果。

## 5. 验证

```bash
python3 -m py_compile tools/offline/analyze_mission_timing.py \
  src/danger_search/test/test_offline_timing_analyzer.py
source devel/setup.bash && python3 -m pytest \
  src/danger_search/test/test_offline_timing_analyzer.py -q
source devel/setup.bash && python3 -m pytest src/danger_search/test -q
python3 tools/offline/check_production_boundary.py
git diff --check
sha256sum -c OFFICIAL_FILES.sha256
```

结果：

- 定向测试 `20 passed`；
- `danger_search` Python 全测试 `510 passed`，仅一条既有第三方弃用警告；
- 生产/离线边界审计通过；
- `git diff --check` 通过；
- 官方哈希仍恰有 4 个历史差异：`auto.sh`、`FSM.h`、`FSM.cpp`、
  `gazebo.xacro`，本层无新增；
- ROS master、Gazebo、控制器和任务进程检查为空。

本层只修改离线 Python 工具、测试和报告，不涉及 C++、消息、launch 或生产配置，因此
不重复 catkin 构建和 Gazebo。

## 6. 合规与下一层入口

- 生产代码、配置、官方文件和正式启动链均无新增差异。
- 没有真值、Gazebo state、world、layout metadata 或固定坐标进入生产。
- 没有固定 seed 分支或候选机制扫参；`1.0 m` 仅为可配置离线过滤阈值，原始与排除距离
  始终保留，后续低频日志必须按采样周期复核后再比较过滤里程。
- 下一层只允许使用同轮绑定且字段完整的账本做 5.2 等待重叠测量；若识别、定位误差、
  安全事件或评分缺失，不得据此选择性能候选。

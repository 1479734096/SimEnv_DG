# LOC-001：RGB-D 失效时 Livox-only 大平移跳变

## 现象

seed=8127 的上一轮二层闭环在 `TakeElevator(1->0)` 的
`ENTER_CABIN/CROSS` 阶段失败。定位日志在仿真时间 `851.346s` 报告
`localization_large_translation_increment`，位姿约从 `(0.81, 2.35)` 跳到
`(1.13, 2.36)`，随后机器人实际不再推进，控制器最终因
`route_progress_timeout` 失败。

## 根因

返程轿厢门附近 RGB-D 候选连续不可用，`adaptive_shared_map` 退化为
`livox_only`。原逻辑只使用 Livox 配准结果的通用运动学上限；在候选间隔较长时，
该上限允许 `0.1591m` 的单帧平移，缺少第二传感器交叉校验，导致错误配准直接进入
控制器。

## 修复

- 新增 `LivoxOnlyDeltaWithinLimit` 纯函数。
- RGB-D 候选无效时，对 Livox-only 候选增加独立的 `0.12m` 单帧平移上限；超限帧
  被拒绝并保持上一已接受位姿。
- 上限通过定位 launch 参数
  `shared_map_livox_only_max_translation` 配置，默认值保持保守。
- 两路候选均有效时，原有 RGB-D/Livox 分歧选择逻辑不变。

## 验证

- `./devel/lib/danger_search_localization/test_adaptive_translation_selector`
  ：7 项通过。
- `python3 -m pytest -q src/danger_search/test`：764 项通过，1 个既有弃用警告。
- `catkin_make -j2`、`git diff --check`、`bash -n tools/sim/run_elevator_gui_test.sh`：通过。
- seed=8127 Gazebo 复跑正在进行，最终闭环结果待记录。

## 回归复盘与修正

### retry3 真实失败链

`/tmp/seed8127_full_cycle_retry3` 在 1 层探索阶段以
`localization_unavailable` 失败。该 reason code 是 odom 停发后的下游结果，不是根因：

- RGB-D 配准连续无有效候选时，上游 Livox 配准仍为 `accepted`；
- 固定 `0.12m` Livox-only 后置上限否决了约 `0.159m`，随后约
  `0.288～0.290m` 的候选；
- 否决期间 `adaptive_base_translation_` 和
  `last_adaptive_output_stamp_` 均不推进，generation 冻结在 `4478`；
- 固定阈值不会随距最后正式输出的累计时间增长，因此形成不可逃逸的吸收态；
- pose age 最终达到约 `3.484s`，超过既有 stale/unavailable 窗口后任务失败。

失败窗口内 `adaptive_both_invalid_count_` 每仿真秒约增加 10，说明 Livox 回调仍约为
10Hz。低 RTF、frontier 重规划和任务进程退出均不是本次 odom 停发的根因。

### 修正后的行为

- 移除固定参数 `shared_map_livox_only_max_translation=0.12`。
- 新增 `shared_map_livox_only_maximum_speed=0.60`，与最后正式输出位姿使用同一区间的
  `frame_interval`，复用既有 `PlanarTranslationLimit`、`0.02m` margin 和
  `0.30m` absolute cap。
- 新增 `shared_map_livox_only_max_consecutive_vetoes=3`。前两次后置超限保持上一位姿，
  第三次本守卫弃权并交回已接受该候选的通用运动学守卫裁决。
- 弃权不会把上游 `kTranslationLimit` 候选改成有效，也不放宽定位 stale 判据。
- 保留 `adaptive_both_invalid_count_` 的累计诊断语义；单独记录连续 Livox-only 否决和
  累计弃权次数。

### 静态验证

- 测试先行：新增连续冻结基准测试后，首次构建因缺少
  `EvaluateLivoxOnlyGuard` 失败；实现后定向 gtest `11/11` 通过。
- 定位完整 gtest：全部通过。
- `python3 -m pytest src/danger_search/test`：`765 passed`，仅 1 个既有弃用告警。
- `catkin_make -j2`、launch 解析和 `git diff --check`：通过。

上述静态结果不能证明二层闭环成功。修正后的 seed=8127 正式 Gazebo 结果在下一节如实
登记。

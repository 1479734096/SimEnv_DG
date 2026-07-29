# 第 12 层 M5：Livox 主增量候选淘汰报告

## 1. 结论

全局 Livox 主增量候选已淘汰并从生产代码回退。候选在短程定位指标上优于基线，但同一 `seed=8072` 在进入建筑主入口时由 `OpenDoorAndPass(main_entrance)` 的 `REFRESH_OPENING` fail-closed，未进入电梯阶段。基线同 seed 能通过主入口并完成 0→1 换层，因此候选造成了明确的任务阶段回归。

按照预登记门槛，不以重跑消除算法失败，不继续执行成功 seed，也不围绕该候选扫参数。生产行为恢复为原 RGB-D 主增量 + RGB-D 失鲜时 Livox 回退。

## 2. 实验配置

- seed：`8072`。
- 两层、GUI、33 s/层、180 s 总预算、完整双向电梯与主入口返航目标。
- 漂移录制、逐帧定位诊断和独立离线真值开启。
- 与基线唯一预期差异：平移主增量从 `rgbd` 改为 `livox`。
- 证据目录：`logs/stage12_m5_livox/seed_8072/`。

## 3. 结果

- 任务在仿真约 9.15 s 进入 `WRITE_RESULT`，原因 `main_entrance_crossing_failed`。
- 直接失败点：`OpenDoorAndPass(main_entrance)` 的 `REFRESH_OPENING`；近场开放门洞重观测失败后按设计终止。
- 候选轮漂移：RMSE 0.019 m、P95 0.025 m、最大 0.025 m、最终 0.024 m，真值里程 1.45 m。
- 同窗样本仅 8 个，不满足不少于 20 个运动窗的离线候选门槛；Livox 同窗 RMSE 0.012 m，RGB-D 0.035 m。
- 失败时定位健康和安全门未触发异常，生产链按 fail-closed 写出失败结果并清理进程。

短程漂移更小不能抵消任务阶段回归。候选改变了入口接近轨迹；机器人到达开门等待点附近后，没有得到满足任务约束的近场门洞几何。即使误差只有厘米级，也说明任务链对轨迹与观测视角敏感，全局替换增量源缺乏足够鲁棒性。

## 4. 回退范围

以下候选改动全部撤销：

- 定位节点的 `planar_translation_source` 参数和 Livox 主增量分支。
- 两级 launch 的候选透传参数。
- 电梯测试 runner 的候选选择变量。
- 候选专用解析辅助与单元测试。

保留 M5 离线分析器、多 seed 基线报告、候选检查点报告、实验日志和 Git 标签，便于审计为何没有采用该方向。

## 5. 后续方向

不再尝试全局 Livox 主源。后续若继续优化，应先保持生产 RGB-D 主源不变，针对已确认的高转速/出电梯退化设计更保守的局部机制，并预先解决源切换锚定和主入口视角鲁棒性问题。任何新候选仍须默认关闭、跨 seed 验证，并以完整任务结果优先于局部里程计指标。

## 6. 合规性

候选与回退均只使用既有 `/scan`、RGB-D、`/trunk_imu` 和自身 `/cmd_vel`。真值只由独立录包进程记录并离线分析。未修改官方 SimEnv 文件，也未引入裁判里程计、Gazebo model state、布局元数据或其他禁用输入。

## 7. 回退验证

- 回退后的五个既有代码/launch/runner 文件与候选前提交 `16e6d7b` 逐文件一致。
- `catkin_make --pkg danger_search_localization -DCATKIN_ENABLE_TESTING=ON`：通过。
- `catkin_make run_tests_danger_search_localization`：通过，无当前测试目标失败。
- `build/test_results` 仍保留上一轮已删除候选测试的 XML，因此聚合工具显示 98 项；该数字包含生成物残留，不作为当前测试清单计数。
- `python3 tools/offline/check_production_boundary.py`：通过。
- 官方哈希仍恰有四项历史已知差异，无新增。
- 运行结束后无 ROS、Gazebo、控制器、定位、任务或录包进程残留。

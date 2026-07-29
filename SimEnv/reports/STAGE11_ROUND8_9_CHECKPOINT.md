# 第 11 层第 8～9 轮检查点：双向电梯恢复、主入口误关联与视觉扫描阻断

## 1. 结论

第 8 轮 `seed=42` 两层完整 GUI 仿真证明：普通探索 `STUCK` 不再把机器人起点写成永久 `BLACK`，机器人能够继续探索；返程 `TakeElevator(1->0)` 的精确候梯点由旧轮次的连续不可达变为一次规划可达，去程与返程电梯均完成。因此，第 7 轮确认的“探索卡滞自封 A* 起点”缺陷已完成仿真闭环验证。

第 8 轮最终仍失败于主入口退出。返航近场重观测把入口附近一组斜向结构误关联成门洞，几何从首次入场的 `(0.04, -0.09), yaw=1.53, width=1.15` 跳到 `(-0.20, 0.52), yaw=-2.13, width=2.20`，随后生成斜向穿墙路径。已增加“相对既有门洞轴线的模 `pi` 一致性”门控，并补充回归测试；该修复不固定旧中心，合法的当前传感器中心仍可随定位漂移更新。

第 9 轮原计划验证主入口修复，但在前置电梯视觉扫描阶段触发硬停止条件：机器人累计旋转 `8.292 rad`，扫描覆盖完成且 18 秒超时，168 帧中只有 2 帧通过门板宽度约束，连续确认最大值仅 1，未达到 3 帧注册要求。该轮因此没有进入电梯和主入口返航，不能作为主入口修复的 GUI 证据。按阶段约束停止继续运行和放宽视觉参数，第 11 层仍不可封口。

## 2. 本检查点累计修改

- `src/danger_search/danger_search/mission_manager.py`
  - 普通探索 `STUCK` 恢复统一通过旋转净空门控；净空不足时保持零速并输出结构化诊断。
  - 主入口返航近场候选新增门洞轴线一致性校验，拒绝与首次合法入口轴线不一致的斜向结构。
  - 保持既有设计：验证通过后采用当前传感器测得的中心和朝向，不平移定位、地图、轨迹或历史语义对象。
- `src/danger_search/danger_search/navigation_tasks.py`
  - 电梯导航 `STUCK` 分支复用任务管理器的旋转净空门控，不再无条件原地恢复。
- `src/danger_search/test/test_mission_manager_helpers.py`
  - 补充近 frontier 卡滞后机器人格仍可执行 A* 的端到端断言。
  - 补充净空不足时不调用 `recover()`、零速原因和结构化事件断言。
  - 补充返航入口斜向误候选被拒绝、合法当前观测仍能更新几何的回归测试。
- `src/danger_search/test/test_navigation_tasks.py`
  - 补充电梯任务 `STUCK` 委托净空门控的测试。
- `tools/sim/run_elevator_gui_test.sh`
  - 诊断 bag 增加 `/rosout`，用于把定位漂移曲线与生产节点自身诊断按时间对齐。
  - 真值仍仅由独立离线录包进程记录，不进入生产节点订阅链。

未修改官方 world、生成器、门/电梯服务、Unitree 控制器、传感器插件、既有说明文档或既有报告。

## 3. 第 8 轮完整 GUI 证据

证据目录：`/tmp/stage11_round8_fullcycle_20260722`

运行配置：

- `SEED=42`
- `FLOOR_COUNT=2`
- `GUI=true`
- `ELEVATOR_TEST_FLOOR_TIMEOUT=33.0`
- `ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5.0`
- `ELEVATOR_TEST_MAX_RUNTIME=180.0`
- `ELEVATOR_TEST_FULL_CYCLE=1`
- `ELEVATOR_TEST_RECORD_DRIFT=1`
- `ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1`

关键结果：

1. 仿真时间 `27.210 s`，RGB-D 连续 3 帧确认电梯门板并注册运行期几何：中心约 `(2.18, 2.21)`、`yaw=3.01`、宽 `0.79 m`；扫描提前成功结束，未超时。
2. `TakeElevator(0->1)` 完成；二层探索越过旧轮次发生自封的位置并继续选择其他可达前沿。
3. 返航时 `TakeElevator(1->0)` 的精确候梯点规划成功，任务在仿真时间 `152.916 s` 以 `DONE` 完成并切回 `RETURN_HOME`。旧轮次的“三次精确点不可达 + 三次历史轨迹点不可达”没有复现。
4. 返航入口近场重观测在 `159.016 s` 接受错误几何 `(-0.20, 0.52), yaw=-2.13, width=2.20`；机器人最终停在 `(-0.287, 0.383), yaw=-2.12`，以 `return_home_main_entrance_failed` 失败。该失败与电梯闭环和 STUCK 起点修复相互独立。

## 4. 主入口误关联根因与修复边界

原逻辑只要求近场候选连续两帧自身一致，没有验证它是否仍属于已注册主入口。斜向墙边/门框组合只要帧间稳定，就可能以最大允许宽度通过并替换入口几何。第 8 轮的错误候选相对首次入口轴线偏差约 `1.48 rad`（门平面轴线按模 `pi` 比较），明显不是同一门洞。

修复采用轴线关联而非固定坐标：

- 候选轴线与既有入口轴线的模 `pi` 偏差超过既有几何容差时，重置连续确认并继续等待；
- 允许法向正反翻转 `pi`，避免机器人从门另一侧观察时误拒；
- 候选通过后仍直接采用当前观测中心和朝向，使局部任务相对量继续抵消定位平移漂移；
- 没有修改门洞检测颜色、宽度、连续帧数或停车安全阈值。

该修复已通过单元回归，但因第 9 轮前置扫描失败，尚无 GUI 闭环证据。

## 5. 第 9 轮硬停止证据

证据目录：`/tmp/stage11_round9_fullcycle_20260722`

使用与第 8 轮相同的 seed、楼层预算和完整闭环配置。扫描结束诊断为：

```text
rotation=8.292
required_rotation=5.8
coverage_complete=true
timed_out=true
frames=168
accepted_panel=2
rejected_panel_width=116
eligible_candidate=2
max_confirmations=1
result=landmark_not_found
```

对照第 8 轮成功扫描：84 帧中 3 帧通过宽度约束，3 帧连续确认后在累计旋转 `4.514 rad` 时提前结束。第 9 轮并非扫描未执行、机器人未旋转或 RGB-D 不出帧，而是门板在该次运行中的有效可见帧不足且不连续。根据既定停止条件，本检查点不继续延长扫描、不降低宽度、不减少连续确认数，也不放宽平面、法向、距离或入口排除约束。

## 6. 定位漂移复核

第 8 轮离线统计：

- 行程：`30.257 m`
- 最终平移误差：`0.0556 m`
- 最大平移误差：`0.2306 m`
- P95 平移误差：`0.2191 m`
- RMS 平移误差：`0.1450 m`
- 最大 yaw 误差：`0.00289 rad`
- 最后 10 秒误差波动：`0.0298 m`

与此前结论一致，yaw 由 IMU 姿态稳定约束，主要问题是平面平移。第 8 轮在出厢后约 `69.8~70.0 s` 出现约 `0.15 m` 的 x 向阶跃，随后没有继续发散，最终误差回落。新录入的 `/rosout` 只能看到 2 秒节流后的 scan-to-map 日志：阶跃前后有一次 `(-0.053, 0.008)` 校正，但无法判断未打印帧中究竟是 RGB-D 帧间 GICP 还是 scan-to-map 校正贡献了其余阶跃。因此当前证据仍不足以支持直接调 GICP 或地图参数。

下一次定位最小诊断应只记录“大增量事件”，避免逐帧日志影响实时率：

1. RGB-D/Livox 帧间 GICP 接受增量超过阈值时，记录时间戳、来源、二维增量、fitness 和对应点数；
2. scan-to-map 注入校正超过阈值时，记录时间戳、二维校正、fitness 和是否更新局部地图；
3. 与现有离线漂移曲线按时间对齐后，只针对确认的支路设计迟滞、双稳态拒绝或跨层局部地图裁剪。

在完成上述判因前，不实施定位参数优化。

## 7. 测试与合规审计

本检查点已执行：

```bash
source devel/setup.bash
python3 -m pytest \
  src/danger_search/test/test_mission_manager_helpers.py \
  src/danger_search/test/test_navigation_tasks.py \
  src/danger_search/test/test_multi_floor_mapper.py \
  src/danger_search/test/test_support_shadow_online_map.py -q
# 205 passed

python3 -m pytest src/danger_search/test -q
# 455 passed

python3 tools/offline/check_production_boundary.py
git diff --check
```

结果：直接回归、完整 Python 测试、生产/离线边界审计和补丁格式检查均通过。

运行期订阅审计未发现 `/odometry_gazebo`、`/ground_truth/*`、裁判里程计、布局或 Gazebo model state 进入生产节点。官方哈希仍恰有 4 个历史已知差异：`auto.sh`、`FSM.h`、`FSM.cpp` 和用户修改的 `gazebo.xacro`；本检查点没有新增官方差异。两轮结束后 ROS/Gazebo/控制器进程已清理，11311 端口空闲。

## 8. 残余风险与阶段状态

- 主入口轴线关联修复尚缺 GUI 长程返航实证。
- 电梯视觉注册存在运行间可见帧波动；第 9 轮已按硬条件停止，需回到现有 bag/诊断数据分析候选宽度时序，而不是继续扫参。
- `_resume_after_negative_obstacle` 对确认负障碍写 `BLACK` 时仍可能在极近距离覆盖机器人格；本轮没有该事件，不是当前故障根因，但后续应评估排除当前 footprint 或至少记录写入前后的起点格状态。
- 定位漂移的阶跃来源尚未区分，不能把 scan-to-map 平滑或跨层地图裁剪写入生产默认值。
- 当前工作区保留用户和此前各层的未提交改动；本检查点未执行 commit、tag、push、reset 或 checkout。

阶段结论：第 11 层不可封口。下一最小工作不是再跑完整两层，而是先分析第 9 轮视觉候选为何只有 2 帧通过，并为定位后端补充低开销的大增量来源诊断；视觉注册恢复稳定后，再用同配置进行一轮主入口修复 GUI 验证。

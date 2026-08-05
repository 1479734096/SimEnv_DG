# bd3efdb 可用性回退基线报告

## 1. 回退结论

当前代码基线固定为 `bd3efdb0a64a0bd59752b7c698f17beb6bce28f6`。回退前的
staged、unstaged、untracked 和关键文档已完整归档至仓库外：

`/home/xbh/tzb26/codeDG_rebuild_archives/20260805-pre-rollback/`

归档经过独立临时克隆恢复演练，恢复后的 index、worktree 和 untracked 清单哈希均与
回退前一致。精确 iter5i 工作树没有 Git 对象或完整 patch，因此不能作为代码回退点；
它只作为后续重构的行为验收规格。

## 2. 已确认成功案例

### 2.1 当前提交可恢复证据

`STAGE17_REPORT.md` 记录 8/8 个短时两层完整闭环：seed `8127`、`1612`、
`8072`、`6946`、`4962`、`42`、`137`、`911`。配置为每层 33 s、最短探索
5 s、任务预算 180 s、`adaptive_shared_map` 和完整返程。该结果证明机械闭环可用，
不代表长时间高质量探索验收。

`STAGE17_1_REPORT.md` 记录 seed `6946` 完整完成 0→1、1→0、主入口退出和 home
返航，13 次 UNKNOWN HOLD 全部自动恢复，无 DROP、控制输出或定位终态。

### 2.2 当前提交之后的行为证据

| 运行 | 已完成行为 | 未通过原因 |
|---|---|---|
| seed 6896 | 完整机械闭环并写 success；0/1 层最高覆盖率 42.0%/46.6% | 前沿大量不可达导致提前收尾，不算高质量探索 |
| seed 8093 | 完整机械闭环并写 success；0/1 层最高覆盖率 63.6%/45.1% | 前沿大量不可达导致提前收尾，不算高质量探索 |
| seed 8127 iter5i | 两层覆盖率 63.2%/63.7%，双向电梯、出门并回到起点附近 | 0 层起点被派生 OCC 覆盖，终局写 connectivity blocked |
| seed 8127 iter5h | 0 层最高覆盖率 69.6%，双向电梯成功 | 返航入口替代目标未交接，直至 mission timeout |

iter3、iter5e 和 iter5k 至少完成 0→1；iter5g 验证失败断面锁存、veto 安装及恢复；
iter6 验证空间反馈账本未误触发。这些只作为局部合同证据。当前提交之后没有一次同时
满足“高质量探索、完整二层闭环、正常 success”。

## 3. 后续验收顺序

1. 在本提交上复现 seed `8127` 的 33 s/层、180 s 总预算短闭环。
2. 只做行为等价的模块分层，期间不修改地图语义、安全阈值或探索策略。
3. 分层后用相同配置再次通过短闭环。
4. 最后独立实现证据生命周期，并用 280 s/层长测验收。

最终安全出口保持当前已验证语义：只能原样通过请求命令或输出零速，不得生成新的
非零取证或恢复命令。所有生产速度继续经唯一 `/cmd_vel` publisher。

## 4. 回退后复测结果

回退后在无生产代码改动的 `bd3efdb` 上执行了一次固定 `seed=8127` 短时两层闭环：

- `GUI=false`；
- 每层预算 `33 s`，最短探索 `5 s`；
- 总预算 `180 s`；
- `adaptive_shared_map`；
- 完整返程开启；
- 生产链不使用真值，外置 `/scan` 仅作运行后证据。

运行完成了主入口进入、0 层探索、`0→1`、1 层探索、`1→0` 和返航入口等待点。
GroundSupport 的一次 UNKNOWN HOLD 正常完成停车、重观测、路径失效和恢复；全程 Gate
最终状态为 CLEAR，没有 DROP、定位终态或控制发布故障。

任务最终在仿真时间 `156.116 s` 失败，原因为
`return_home_main_entrance_failed`。机器人位姿约为
`(0.224, 0.588, -2.177)`，距入口等待点约 `0.294 m`。失败发生在
`OpenDoorAndPass(main_entrance)` 的 `REFRESH_OPENING` 阶段：三秒窗口内没有形成两帧
一致且满足净空的近场门洞观测。日志没有轴线不一致或中心偏移过大的拒绝记录，因此
失败范围只剩下检测器未形成候选、候选距离超限或候选宽度不足。

这不是本次回退引入的新缺陷。历史报告已经记录同一近场检测对视点、遮挡和局部几何
敏感，且当前提交在相同短时配置下既有成功证据，也有入口刷新失败证据。本轮只能证明：

1. `bd3efdb` 的双向电梯和基础安全反馈链仍可运行；
2. 当前提交的入口近场检测存在运行间波动，不能把历史 `8/8` 外推为确定性成功；
3. 本轮失败与建图语义重构无因果关系；
4. 在基线尚未复现成功前，不启动模块分层或地图证据生命周期重构。

运行证据位于：

`/tmp/rollback_baseline_seed8127_20260805_1305/`

外置 `/scan` 共 `1565` 帧，覆盖 `156.4 s`，频率约 `10 Hz`，SHA-256 为
`adc3d170c3511e0609fc6d8f1a5a9b360dee6e72ab334b59dd9bd6f680e65f32`。运行图审计
通过，收尾后无残留 ROS/Gazebo/rosbag 进程，端口 `11311` 已释放。

## 5. 第一轮后的暂停点

本轮没有修改生产代码，也没有开始后续重构。下一步应先把入口近场刷新作为独立的
基线稳定性问题处理：补足候选生成阶段的只读拒绝分类证据，再用同配置最多复测一次。
若仍失败，应停止并单独裁决门洞观测合同；若通过，才能以该次成功轨迹作为行为等价
重构的前后对照。不得把放宽门宽、扩大中心漂移容差或跳过近场确认作为回退手段。

## 6. 只读诊断与唯一复测

经用户确认后，只给 `refresh_open_door_geometry()` 增加了固定类别、只读的拒绝计数：

- `no_candidate`；
- `distance_exceeded`；
- `width_insufficient`；
- `axis_mismatch`；
- `center_shift_exceeded`；
- `inconsistent_candidate`。

诊断只在确认成功或超时时输出结构化事件，同时保存一个有界的最后候选摘要。它不改变
检测器、门宽、距离、中心偏移、连续帧数、超时、任务状态或控制命令。新增测试覆盖无
候选、距离超限、宽度不足以及超时事件内容；MissionManager 与导航任务定向套件
`225 passed`，`py_compile`、生产边界和两类 diff check 通过。

随后按第 4 节完全相同的配置只复测一次 `seed=8127`。本轮完整通过：

1. 主入口进入时近场候选为 `distance=0.725 m`、`width=1.35 m`、`106` 个证据点；
2. 完成 0 层探索、`0→1`、1 层探索和 `1→0`；
3. 返航主入口候选为 `distance=0.508 m`、`width=1.75 m`、`69` 个证据点；
4. 两次入口确认的拒绝计数均为空；
5. 正常出门、回到起点并以 `return_home_completed=true`、
   `terminal_outcome=success` 结束。

两层最高覆盖率约为 `0.486/0.462`。GroundSupport 共记录 5 次状态转换：一次陈旧证据
停车后自动恢复，另一次 UNKNOWN 观察停车最终使路径失效并由任务层恢复。DROP、定位
终态和失败终局均为零。运行图审计通过，任务退出后无残留进程，端口 `11311` 已释放。

证据目录：

`/tmp/rollback_baseline_seed8127_retry_20260805_132857/`

外置 `/scan` 共 `1735` 帧，覆盖 `173 s`，频率约 `10 Hz`，SHA-256 为
`c000f68a98fcc81c8c91e69ab7aed0b3626bd5afdd50fd91519c736a81d55a6c`；目录内
`evidence.sha256` 冻结了 preflight、运行图审计、bag 摘要、关键事件、任务日志、测试
摘要和 `/scan`。

## 7. 当前结论

`bd3efdb` 已重新获得一次同配置的 seed `8127` 完整短闭环证据，可以作为后续行为等价
分层的前后对照。第一轮失败与第二轮成功共同证明主入口近场检测存在运行间观测波动；
它不是确定性建图或 GroundSupport 缺陷，也不能因本轮成功而宣称已经稳定解决。

本轮到此暂停，不追加 seed、不修改门洞或地图语义。下一阶段若开始模块分层，必须先
冻结本轮状态序列和终局条件，只迁移职责、不改变行为；分层后先复现同一短闭环，再
独立处理长时间探索和地图证据生命周期。

## 8. 行为等价模块分层

以第 6 节成功轨迹为前后对照，本轮只迁移职责，不修改地图语义、安全阈值、探索策略、
门/电梯服务或控制命令：

1. `door_observation_runtime.py` 接管主入口初次门洞与返航近场刷新的跨帧确认、重复
   stamp 去重和固定拒绝分类；MissionManager 只采集点云、绑定公开 ID 和创建任务。
2. `return_home_planner.py` 接管地图快照复制、返航 A*、历史轨迹回退、失败首段否决、
   候选复验和后台线程；MissionManager 只冻结输入、轮询结果和分派任务状态。
3. `ground_support_runtime.py` 接管支撑线程到主线程的证据队列、Gate 状态镜像、重试
   身份和已登记 BLACK 重放状态；安全判定仍只有 `GroundSupportSafetyGate` 一份。
4. `progress_monitor.py` 接管前沿选择/位姿进度的诊断状态机，不再由 MissionManager
   内嵌实现。

旧字段、旧私有返航方法和 `_ProgressMonitor` 兼容接口均已删除，生产与测试零引用；
测试改为直接验证新模块，MissionManager 集成测试只验证编排边界。迁移中发现并修复
一处等价性偏差：`negative_obstacle` 局部重规划不可用时必须继续进入原有锁存终局，
不能因任务层分类而被静默视为已处理。

当前 MissionManager 仍保留顶层 tick、楼层切换、任务创建、公开 ID 绑定和感知输入
编排；电梯的非阻塞子状态机继续由 `navigation_tasks.TakeElevator` 持有。本轮不搬迁
电梯检测算法或修改其确认合同，以免在“先恢复可跑”阶段引入新的行为变量。

## 9. 分层后的行为等价验收

完成模块分层和旧接口清理后，先执行静态与离线测试：

- `python3 -m pytest test -q`：`605 passed`；
- 受影响生产文件与测试的 `py_compile`：通过；
- `tools/offline/check_production_boundary.py`：通过；
- 禁用输入静态审计：生产链未新增 truth、world 或 metadata 依赖；
- `/cmd_vel` publisher 静态审计：仍只有 `navigator.py` 的统一出口；
- `git diff --check` 与 `git diff --cached --check`：通过。

随后只运行一次与第 6 节同类的 seed `8127` 双层短闭环，配置为
`floor_count=2`、`floor_timeout=33`、`min_floor_explore_time=5`、
`max_runtime=180`、`full_cycle=1`，定位后端为 `adaptive_shared_map`。结果为：

1. 仿真时间 `33.270 s` 从 0 层探索进入 `TakeElevator(0->1)`；
2. `75.172 s` 完成 `0→1` 并进入 1 层探索；
3. `103.272 s` 因返航预算进入 `TakeElevator(1->0)`；
4. `146.374 s` 完成 `1→0`，随后完成主入口出门与回起点；
5. `164.970 s` 写出 `terminal_outcome=success`、
   `return_home_completed=true`，runner 的 `TEST_SUMMARY.txt` 判定通过。

两层运行期最高覆盖率约为 `0.514/0.456`。运行中没有 DROP 确认、
`ground_support_path_replan_exhausted`、定位终态或 Python traceback。运行图审计通过，
外置 `/scan` 共 `1705` 帧，覆盖 `170.378 s`，频率约 `10 Hz`，SHA-256 为
`877a9059f25317fe0f30f6cec86f50d5ea2b55ad9dc8493025d8f4de3ed68a48`。
证据目录为：

`/tmp/refactor_seed8127_s0drsmvM/`

任务自然退出后仅向本轮 `/scan` 录制进程组发送 `SIGINT`；ROS/Gazebo/rosbag 无残留，
端口 `11311` 已释放。HEAD 仍为
`bd3efdb0a64a0bd59752b7c698f17beb6bce28f6`，本轮未暂存、提交或打标签。

该结果证明本轮四项职责迁移在这条固定短闭环上保持了基线行为，但不等于长时间探索、
地图证据生命周期或随机 seed 稳定性已经验收。下一阶段应保持这些模块边界，分别处理
长期建图语义与探索质量，不再把功能实现重新堆回 MissionManager。

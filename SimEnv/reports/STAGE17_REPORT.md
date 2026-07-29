# STAGE17 自适应定位正式后端验收报告

## 1. 当前结论

本阶段定位漂移优化的工程验收状态为**通过**。用户在知晓冻结统计门槛未通过后，明确
接受当前证据和残余风险，并要求将 `adaptive_shared_map` 作为正式默认后端；
`rgbd_livox` 保留为显式回退。

统计矩阵状态与工程验收状态分开记录：当前完成八个有效 seed，seed `2027` 因
Gazebo/控制链未就绪而暂停，所以矩阵裁决仍为 `insufficient_evidence`，不是统计门槛
通过。该事实不再阻断本阶段工程验收和正式采用。

- 完整闭环：`8/8`；
- RMSE `<0.1 m`：`7/8`；
- RMSE `<0.05 m`：`0/8`；
- 定位诱发失败：`0`；
- 八轮全部通过 P95、最大误差、yaw、输出频率和最大间隔硬门；seed `1612` 的末端
  误差 `0.2529 m` 未通过 `0.20 m` 硬门。

seed `42` 的 RMSE 为 `0.1012 m`，已经使“20/20 RMSE `<0.1 m`”的明显提升门槛
不可能满足；seed `1612` 的末端硬门失败意味着原冻结矩阵即使补齐也不能晋级。本次切换
是用户明确接受残余风险后的工程验收决策，不应表述为矩阵通过。

## 2. 原因、方案、目的

| 原因 | 方案 | 目的 |
|---|---|---|
| Stage16 用相同时间、不同路程比较 seed `8072` | 改为同路程重算，并冻结完整任务矩阵门槛 | 避免运动进度混杂导致错误淘汰 |
| 三个 seed 不足以判断随机场景稳定性 | 固定 20 个 seed、统一两层完整闭环配置串行运行 | 用足够样本决定是否切正式默认 |
| 手工多轮容易混配置或重复 seed | runner 校验 `MATRIX_CONFIG.txt`，有效证据自动跳过 | 支持可审计断点续跑 |
| 任务成功不能代替定位精度 | 离线汇总同时检查 RMSE、P95、末端、最大误差、yaw 和输出连续性 | 防止只以“能跑完”宣称定位改善 |
| 两路同时无效可能触发停车 | 统计最大 odom 间隔和定位诱发失败 | 直接检验 fail-closed 是否频繁影响任务 |
| 暂停信号最初只终止子 runner | 状态码 `130/2` 立即停止父矩阵，且不消耗 seed 尝试额度 | 确保暂停和全局前置失败可恢复 |

## 3. 固定配置与门槛

矩阵固定为 `adaptive_shared_map`、20 个预登记 seed、GUI 关闭、两层、单层预算
`33 s`、最小探索 `5 s`、任务预算 `180 s`、墙钟超时 `1800 s`、完整返程、关闭原始
RGB-D 录包、开启隔离真值漂移和逐帧定位诊断。

可接受门槛：至少 18/20 RMSE `<0.1 m`，至少 18/20 完整闭环，定位诱发失败为 0。
明显提升门槛：在可接受基础上至少 16/20 RMSE `<0.05 m`，且 20/20 RMSE `<0.1 m`。
所有轮还必须满足 RMSE/P95/末端 `<0.20 m`、最大误差 `<0.35 m`、最大 yaw
`<0.005 rad`、平均 odom `>=8 Hz`、最大输出间隔 `<=0.5 s`。

## 4. 已完成数据

| seed | 闭环 | RMSE | P95 | 末端 | 最大 | yaw 最大 | Hz | 最大间隔 |
|---:|:---:|---:|---:|---:|---:|---:|---:|---:|
| 8127 | 是 | `0.0869` | `0.1379` | `0.0211` | `0.2063` | `0.00281` | `9.93` | `0.202 s` |
| 1612 | 是 | `0.0892` | `0.1222` | `0.2529` | `0.2723` | `0.00327` | `10.00` | `0.112 s` |
| 8072 | 是 | `0.0760` | `0.1345` | `0.1993` | `0.3315` | `0.00283` | `9.98` | `0.200 s` |
| 6946 | 是 | `0.0624` | `0.0932` | `0.1420` | `0.1751` | `0.00354` | `9.98` | `0.200 s` |
| 4962 | 是 | `0.0578` | `0.0874` | `0.0861` | `0.1158` | `0.00284` | `10.00` | `0.104 s` |
| 42 | 是 | `0.1012` | `0.1290` | `0.0305` | `0.1539` | `0.00353` | `9.98` | `0.202 s` |
| 137 | 是 | `0.0693` | `0.0974` | `0.0101` | `0.1309` | `0.00292` | `9.99` | `0.202 s` |
| 911 | 是 | `0.0888` | `0.1199` | `0.1036` | `0.1703` | `0.00260` | `9.99` | `0.202 s` |

八轮最大输出间隔均低于 `0.5 s`，没有定位陈旧、三秒不可用或定位姿态健康失败。
因此当前证据不支持“自适应选择会频繁停车”的担忧；但仅八轮仍不能推断 20-seed
总体发生率。

## 5. 实现与文档

- `tools/sim/run_localization_seed_matrix.sh`：固定配置、串行执行、有效证据跳过、最多两次
  有效尝试、用户中断/全局前置失败立即暂停；
- `tools/offline/summarize_localization_matrix.py`：验证单轮配置与证据，生成 JSON/CSV/
  Markdown，并按冻结门槛裁决；
- 两个定向测试覆盖门槛边界、证据选择、日志粘行解析、runner 固定配置和暂停语义；
- 修正 Stage16 及团队文档中 seed `8072` 的错误短窗结论；
- 新增 `docs/team/STAGE17_HANDOFF.md`，记录续跑、分析和结果回传方法；
- 将正式 launch 和电梯专项 runner 默认后端改为 `adaptive_shared_map`，同步默认值契约；
  旧 `rgbd_livox` 后端未删除，可用 launch 参数显式回退。

## 6. 验证与合规

已完成：

```text
catkin_make --pkg danger_search_localization -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_danger_search_localization
catkin_test_results build/test_results/danger_search_localization
python3 -m pytest（默认值、矩阵、runner 和接口文档定向测试）
python3 tools/offline/check_production_boundary.py
bash -n tools/sim/run_elevator_gui_test.sh tools/sim/run_localization_seed_matrix.sh
python3 -m py_compile tools/offline/summarize_localization_matrix.py
roslaunch danger_search danger_search.launch（短时启动并查询参数和订阅）
git diff --check（排除用户已有 plan.md）
sha256sum -c OFFICIAL_FILES.sha256
```

- 定位包 `130 tests, 0 errors, 0 failures`；本轮默认值、runner、矩阵与接口契约
  `25 passed`；
- 生产/离线边界审计通过；八轮生产节点订阅审计均未发现真值输入；
- 正式 launch 运行时参数为 `adaptive_shared_map`；定位节点仅订阅 `/scan`、
  `/trunk_imu`、`/real_sense/depth/points` 和 `/cmd_vel`，停止后 11311 与相关进程已清理；
- 官方哈希仍只有既有五项差异：`auto.sh`、`docs/quick-start.md`、`FSM.h`、`FSM.cpp`、
  `gazebo.xacro`；本阶段未修改赛方受保护文件；
- seed `2027` 失败后曾遗留 ROS/Gazebo 进程，正式启动检查前已按确认的进程组清理；
- 用户已有 `plan.md` 改动保持未覆盖、未纳入本阶段文件；其既有尾随空白导致全工作区
  `git diff --check` 非零，本阶段文件的排除检查通过。

## 7. 验收结论与风险

- 原暂停过程中产生了状态码 `130/2` 或无状态码的空 attempt 目录。它们没有任务或
  漂移证据，修正后的 runner 保留审计痕迹但计为 `used=0`，不会阻断续跑；
- 证据目录占用约 `2.4 GB`，剩余磁盘约 `153 GB`；
- 完整 20-seed 未完成，且 seed `1612` 已触发末端硬门，不得宣称冻结矩阵通过；
- Stage17 定位漂移优化按用户明确指示标记为工程验收通过，正式默认切换为
  `adaptive_shared_map`；
- 若后续运行出现定位诱发失败或末端误差不可接受，使用
  `localization_backend:=rgbd_livox` 显式回退，不按任务阶段或离线真值动态切换；
- 本报告随 Stage17 代码创建本地提交和 `migration-layer-17` 标签；未完成矩阵不再阻断
  阶段关闭，可按 `docs/team/STAGE17_HANDOFF.md` 继续用于残余风险量化。

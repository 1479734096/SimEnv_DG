# 第 12 层 M4 离线局部坐标诊断阻断报告

## 1. 结论

M4 离线诊断已完成，但预注册门槛未通过。本阶段不实现 `alpha` 降权，不运行 `seed=43` 或 Gazebo，不修改生产定位、启动参数和官方环境文件。

两轮 `seed=42` 数据中，没有同一预注册特征同时满足：各轮 AUC 分离度不低于 0.70、方向一致、必要校正与错误吸附各至少 8 个样本。低旋转窗口中的部分单轮高 AUC 来自少量样本，且跨轮方向或分离度不稳定，不能转成生产规则。

离线地图重放能以约 8 mm RMSE 复现生产校正，说明重放实现可用于诊断。换层后清空局部地图的反事实结果一轮变差、另一轮仅轻微改善，因此“跨楼层地图别名”仍是架构风险，但尚未证明是本次漂移的主要有害根因，也不支持无条件重置地图。

## 2. 范围与边界

本阶段只修改：

- `tools/offline/analyze_localization_quality_bag.py`
- `src/danger_search/test/test_localization_quality_analyzer.py`
- 本报告

未修改生产 C++、launch、融合参数、任务逻辑、官方 SimEnv 文件和既有报告；未启动 ROS、Gazebo 或控制器。真值仅由既有独立 rosbag 离线读取，不进入生产订阅链。

开始前已创建非侵入式 Git 快照，未移动 `main`、未改变真实暂存区：

- 标签：`backup-pre-m4-20260723`
- 快照提交：`c3440159b24d241da1e170dda81b2090073a3cfd`
- 原分支基线：`main@1cac842`

## 3. 输入证据

使用两轮已有诊断包，不重复运行仿真：

- `/tmp/stage12_m3_diagnostic_round1_20260722_222505/elevator_diagnostic.bag`
- `/tmp/stage12_m3_diagnostic_round2_20260722_224022/elevator_diagnostic.bag`

主要输入为定位 odom、`/scan`、`/trunk_imu`、定位结构化 `/rosout` 和独立录制的 `/ground_truth/base_w`。生成物：

- `/tmp/stage12_m4_round1.json`
- `/tmp/stage12_m4_round1_quality.csv`
- `/tmp/stage12_m4_round1_windows.csv`
- `/tmp/stage12_m4_round2.json`
- `/tmp/stage12_m4_round2_quality.csv`
- `/tmp/stage12_m4_round2_windows.csv`

## 4. 离线分析实现

分析器新增以下只读能力：

1. 按每个窗口的定位 yaw，把校正量、Livox 与 RGB-D 分歧、反事实误差转到机器人前向/侧向坐标。
2. 用同时间戳反事实计算校正收益，避免相邻 odom 差分把 RGB-D 误差错误归因给 scan-to-map。
3. 对机器人 4 m 内 `/scan` 做二维 PCA；少于 30 点或特征值比大于 0.5 时拒绝结构特征。
4. 按全量、低旋转、深度新鲜、低旋转且深度新鲜四层统计特征 AUC、方向、Spearman 相关和 5 连续窗口分块 bootstrap 95% 区间。
5. 离线重放生产 scan-to-map 的体素、半径、对应距离、残差裁剪、迭代和 0.10 m 校正上限，并记录匹配点来自换层前、轿厢阶段还是换层后。
6. 对换层结束时清空地图做单帧反事实。该结果不模拟重置后地图与位姿的闭环演化，只用于否定明显无效的候选。

## 5. 局部特征结果

所有窗口层的有效类别数为：

| 轮次 | 必要校正 | 错误吸附 | 合计 |
|---|---:|---:|---:|
| round1 | 22 | 8 | 30 |
| round2 | 22 | 12 | 34 |

低旋转层为：

| 轮次 | 必要校正 | 错误吸附 | 合计 |
|---|---:|---:|---:|
| round1 | 8 | 7 | 15 |
| round2 | 10 | 7 | 17 |

关键特征对照：

| 特征与分层 | round1 AUC 分离度 / 方向 | round2 AUC 分离度 / 方向 | 判定 |
|---|---|---|---|
| 全量：方向余弦 | 0.716 / 必要校正更低 | 0.606 / 必要校正更高 | 方向相反 |
| 全量：前向校正 | 0.631 / 更高 | 0.640 / 更高 | 分离度不足 |
| 低旋转：侧向分歧 | 0.661 / 更高 | 0.800 / 更高 | round1 不足，且两轮错误吸附均少于 8 |
| 低旋转：方向余弦 | 0.804 / 更低 | 0.629 / 更高 | 方向相反 |
| 低旋转：前向校正 | 0.893 / 更高 | 0.571 / 更低 | 方向相反 |

结构弱轴特征的单轮 AUC 偶有超过 0.70，但 round1 有效类别仅为 6/5，round2 为 9/6；低旋转层更少，不能通过样本门槛。bootstrap 区间也普遍很宽，说明当前数据不足以冻结判别规则。

结论：方向投影消除了世界坐标符号泄漏，但没有得到跨轮稳定、样本充分的“错误吸附”特征。

## 6. 跨楼层地图别名重放

重放自检：

| 指标 | round1 | round2 |
|---|---:|---:|
| 校正重放 RMSE | 0.008107 m | 0.007767 m |
| 校正重放 P95 | 0.011708 m | 0.010936 m |
| 换层后接受帧 | 84 | 283 |

两轮 RMSE 均低于预注册的 0.02 m 解释门槛，重放结果可用于诊断。

换层后清空地图反事实：

| 指标 | round1 | round2 |
|---|---:|---:|
| 原始轨迹 RMSE | 0.184569 m | 0.324377 m |
| 重置候选 RMSE | 0.189985 m | 0.323202 m |
| 平均误差改善 | -0.007403 m | +0.002621 m |
| 改善帧 / 恶化帧 | 16 / 68 | 142 / 141 |

round1 明显变差；round2 改善仅约 1.2 mm，且改善与恶化帧基本持平。无条件换层清图不具备跨轮收益。

匹配点来源也不稳定：round1 中错误吸附和必要校正使用换层前地图点的比例分别为 0.628 和 0.620，几乎相同；round2 分别为 0.469 和 0.366，虽有差异，但目标层匹配比例同时发生反向变化。该现象只能证明旧楼层点仍参与匹配，不能单独证明其造成有害校正。

## 7. 未验证假设

- `PlanarGICPOdometry` 内部优化完整刚体变换、只消费 XY 的隐含旋转风险仍成立，但现有 bag 未记录最终 GICP 旋转，M4 无法回溯验证。
- 地图别名是代码层面的真实架构风险，但现有反事实不支持直接修复。
- 离线重放不能模拟位姿变化后地图插入、对应点和后续校正的完整闭环，因此不作为生产收益证明。
- 两轮均为同一 `seed=42`，不能证明跨场景泛化；由于预注册门槛已在现有两轮失败，不再消耗资源运行 `seed=43`。

## 8. 阶段判定与下一步

M4 判定为阻断：不实现 `alpha`、不继续扫描阈值、不进入跨 seed 仿真。后续优先回到 `fix.md` 中独立的生产防跌闭环；定位方向若再次启动，应先新增可观测的 GICP 最终旋转和楼层分区地图实验设计，再独立立项，不能复用本轮未通过的特征作为生产门控。

## 9. 验证与合规

已完成验证：

- `python3 -m py_compile tools/offline/analyze_localization_quality_bag.py src/danger_search/test/test_localization_quality_analyzer.py`：通过；
- `python3 -m pytest -q src/danger_search/test/test_localization_quality_analyzer.py`：13 项通过；
- `python3 -m pytest -q src/danger_search/test`：479 项通过；
- `python3 tools/offline/check_production_boundary.py`：通过；
- 本阶段三个目标文件 `git diff --check`：通过；
- `sha256sum -c OFFICIAL_FILES.sha256`：仍只有 `auto.sh`、`FSM.h`、`FSM.cpp`、`gazebo.xacro` 四项既有差异，无新增官方差异。

测试只有系统 `Cryptodome` 对 Python `imp` 模块的既有弃用警告。本阶段未新增、安装或升级依赖；离线分析器使用环境中已有的 NumPy 与 SciPy，不影响生产节点依赖。

工作区继承大量第 11/12 层暂存和未暂存改动；既有 `reports/STAGE11_ROUND7_CHECKPOINT.md` 尾部空行会使全暂存区 `git diff --cached --check` 失败。本阶段不修改既有报告，也不创建普通分支提交；完成后只创建不改变真实索引的备份标签。

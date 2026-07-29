# 第 9 层：离线工具隔离报告

## 结论

第 9 层完成。正式 `danger_search` Python 包、安装清单和 launch 与真值评测、bag 回放、
Gazebo 状态分析工具形成单向隔离；离线评分只允许在正式任务退出后执行。该层不改变
控制、定位、感知、规划或官方环境行为，可以进入第 10 层最终集成验收。

## 审计事实与处理

- 旧工作区在 `danger_search` 包内包含 `evaluation_tools.py`，并安装
  `batch_evaluate.py`；它们还依赖 `run_seed.py`、真值评分、运行归档和调参工具。
- 新工作区在本层开始前已没有上述生产模块，CMake 只安装
  `scripts/danger_search_node`。本层没有把旧的大型混合工具重新搬入生产包。
- 新增工作区根目录 `tools/offline/`：它不属于 catkin 包，只提供边界说明和静态审计。
  环境自带的 `building_obstacles/evaluate_danger.py` 保持官方文件原样，只在任务退出后
  由人工离线调用。
- `check_production_boundary.py` 检查：
  1. `danger_search` Python 安装入口严格等于 `scripts/danger_search_node`；
  2. 生产 Python 模块没有离线评测导入或离线命名模块；
  3. 正式/诊断 launch 不引用 truth、metadata、world、bag 或评分入口；
  4. 任一 CMake 不得安装根目录 `tools/offline/`。
- 新增三条测试覆盖当前工作区闭包、额外安装入口检测和离线 import 检测。

## 变更文件

- `tools/offline/README.md`
- `tools/offline/check_production_boundary.py`
- `src/danger_search/test/test_offline_isolation.py`
- `src/danger_search/CMakeLists.txt`
- `src/danger_search/README.md`
- `TEAM_README.md`
- `docs/team/ARCHITECTURE.md`
- `docs/team/MIGRATION_INVENTORY.md`
- `docs/team/MIGRATION_LEDGER.md`
- `docs/team/RUNBOOK.md`
- `docs/team/BUG_LEDGER.md`

## 验证

| 项目 | 结果 |
|---|---|
| 离线边界命令 | 通过：`生产/离线边界审计通过。` |
| 新增定向测试 | `3 passed` |
| `danger_search` 完整 Python 测试 | `401 passed` |
| 受影响包构建 | `catkin_make --pkg danger_search -j2` 通过 |
| 隔离安装演练 | 通过；安装产物只有生产 Python 模块、`danger_search_node`、配置、launch 与包元数据，无离线工具 |
| 补丁格式 | `git diff --check` 通过 |
| 官方哈希 | 9 项预期不匹配；均为既有授权/用户改动，本层未修改官方文件 |

完整 pytest 首次直接从未 source 的 shell 运行时出现 14 个
`ModuleNotFoundError: danger_search`；加载 `/opt/ros/noetic/setup.bash` 和
`devel/setup.bash` 后 401 条全部通过，属于测试环境调用错误，不是代码失败。

隔离安装首次受当前 setuptools 影响，不识别 catkin 生成的
`--install-layout=deb`。只对安装命令设置 `SETUPTOOLS_USE_DISTUTILS=stdlib` 后成功；
登记为 `ENV-005`，不修改或升级依赖。

## 合规性

- 本层没有新增 ROS 话题、服务、TF、文件输入或运行时参数。
- 生产包没有读取 `/Odometry_gazebo`、`/ground_truth/*`、world、truth 或 metadata。
- 官方评分器仍由环境包安装是赛方环境事实；正式 `danger_search.launch` 不启动、导入
  或调用它。其输出不进入生产闭包。
- 用户保留的 `gazebo.xacro` 可视化改动未纳入本层修改或提交。

## 未迁移与下一层入口

- 旧 `evaluation_tools.py`、`batch_evaluate.py`、`run_seed.py` 和各类诊断脚本因混合
  启动、评测、归档和调参职责而不直接迁移。
- 第 10 层若需要批量 runner，只能在 `tools/offline/` 重新实现，并保持“正式任务退出
  后再读取真值”的单向顺序。
- 第 10 层按迁移台账依次进行最低可用防坠、单层闭环、两/三层电梯返航、坐标 z、
  性能、覆盖识别和预算验收；ENV-004 的已登记边界在最终全流程结束前统一收口。
- 本层纯静态隔离，不改变运行时行为，重复 Gazebo 不能增加证据，因此未启动仿真。

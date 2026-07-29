# 离线工具边界

本目录只保存运行结束后的审计、评测和分析工具，不属于任何 catkin 包，也不会被
`danger_search.launch`、`setup.py` 或 CMake 安装。这里的工具可以读取真值、bag、
Gazebo 状态或生成场景，但其输出不得回流到同一轮任务的感知、规划、控制、预算或结果。

## 静态 Python/离线边界

在工作区根目录执行：

```bash
python3 tools/offline/check_production_boundary.py
```

预登记定位矩阵由根目录 runner 串行执行，证据目录可断点续跑：

```bash
LOCALIZATION_MATRIX_OUTPUT_ROOT=/tmp/stage17_adaptive_shared_map_matrix \
  tools/sim/run_localization_seed_matrix.sh
```

任务退出并完成离线漂移分析后，可单独重建汇总：

```bash
python3 tools/offline/summarize_localization_matrix.py \
  /tmp/stage17_adaptive_shared_map_matrix
```

汇总器只读取 `TEST_SUMMARY.txt`、任务日志和漂移 sidecar，不进入生产启动链，也不向
运行中的定位、规划或控制反馈结果。

该命令只检查 `danger_search` 的 Python 安装清单、模块导入和 launch 入口，防止后续
修改把离线评测依赖带回生产包。它不能证明整机运行闭包合规。

## 完整运行图审计

正式节点和控制器全部启动后，在同一 ROS master 下执行：

```bash
python3 tools/offline/check_runtime_ros_graph.py
```

该检查覆盖所有节点的禁用真值订阅、`/cmd_vel` 与 `/joy` publisher，以及内部定位
话题的唯一生产者。当前延期保留的 `junior_ctrl` 真值订阅会使此检查明确失败；在
`COMPLIANCE-001` 关闭前不得把静态 Python 边界通过描述为完整生产闭包通过。

## 官方结果评测

必须先结束正式任务，再单独运行环境自带的评分器：

```bash
python3 src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file results/danger_truth.json \
  --detected-file results/detected_danger.json \
  --output-file results/evaluation_result.json
```

`danger_truth.json` 只能在这个离线步骤读取。`evaluation_result.json` 只用于人工分析和
后续轮次调参，不得由正式 launch 或生产模块读取。

旧工作区中的 `evaluation_tools.py`、`batch_evaluate.py`、`run_seed.py` 及各类 Gazebo
诊断脚本没有直接迁入：它们同时承担启动、真值评测、归档和调参，依赖关系过宽。第 10
层若需要批量运行器，应在本目录按“启动结束后再评测”的单向流程重新实现。

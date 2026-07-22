# 离线工具边界

本目录只保存运行结束后的审计、评测和分析工具，不属于任何 catkin 包，也不会被
`danger_search.launch`、`setup.py` 或 CMake 安装。这里的工具可以读取真值、bag、
Gazebo 状态或生成场景，但其输出不得回流到同一轮任务的感知、规划、控制、预算或结果。

## 生产闭包审计

在工作区根目录执行：

```bash
python3 tools/offline/check_production_boundary.py
```

该命令检查 `danger_search` 的 Python 安装清单、模块导入和 launch 入口，防止后续修改
把离线评测依赖带回生产包。

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

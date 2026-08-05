# 三层完整闭环 Seed 矩阵工具报告

## 1. 目标与结论

新增串行三层完整闭环矩阵工具。用户一次性预登记 2～20 个不同 seed 后，脚本逐轮复用
现有电梯 runner 和 `auto.sh` 启动链，自动归档同轮证据、离线评分并生成运行经过、整体
效果和问题摘要。

本轮只实现并静态验证工具。用户正在使用 Gazebo，因此没有启动、清理或检查当前仿真，
也没有产生新的三层运行结论。

## 2. 实现

`tools/sim/run_three_floor_seed_matrix.sh`：

- 冻结完整 40 位 HEAD、seed 顺序、3 层正式预算和诊断开关；
- 固定 `GUI=true`、`280/60/840 s` 仿真预算、`9000 s` 墙钟上限和
  `adaptive_shared_map`；
- 正式运行关闭真值话题、定位诊断和录包；
- 算法失败记账后继续，不能重跑；基础设施失败最多同配置两次，仍失败暂停；
- 同一 HEAD、seed 和证据目录支持断点恢复，已完成正式结果不会被覆盖；
- 每轮退出后按文件更新时间隔离归档检测结果和裁判真值，避免下一 seed 覆盖；
- 生成结果哈希、官方评分、时间账本和运行元数据。

`tools/offline/summarize_three_floor_matrix.py`：

- 汇总任务终态、失败分类、仿真/墙钟时长、里程、覆盖、STUCK、恢复和安全事件；
- 汇总检测数、真值数、正确数、漏报、虚警、召回率和客观得分；
- 同时输出 `matrix_summary.json` 和中文 `matrix_summary.md`；
- 缺少同轮结果、真值、评分、哈希或时间账本时不宣称矩阵证据完整。

矩阵根目录还会生成 `OPERATOR_NOTES.md`，用于人工记录结构化日志无法确认的卡门、物理
接触和姿态异常。人工观察不会替代算法终态和评分证据。

## 3. 验证

本轮执行：

- `bash -n tools/sim/run_three_floor_seed_matrix.sh`：通过；
- runner `--help`：通过，未启动 ROS；
- `python3 -m py_compile tools/offline/summarize_three_floor_matrix.py ...`：通过；
- runner 和汇总器定向测试：`8 passed`。
- 连同既有电梯 runner 和时间账本测试：`35 passed`；
- 参赛 Python/离线边界审计：通过；
- `git diff --check`：通过；
- 官方哈希仍为已记录的 5 项历史差异：`auto.sh`、`docs/quick-start.md`、`FSM.h`、
  `FSM.cpp`、`gazebo.xacro`，本轮没有新增受保护文件差异。

首次测试收集因 Python 3.8 下测试文件立即求值 `dict[str, int]` 失败；补充延迟类型注解后
同组测试通过。该问题只在新增测试文件中，不影响矩阵脚本设计。

## 4. 合规边界

- 没有修改 `auto.sh`、world、生成器、传感器、Unitree 控制器或生产算法；
- 真值只在单轮任务退出后由矩阵工具复制和评分，不进入当轮或后续生产决策；
- 正式运行继续由既有 runner 检查生产节点订阅，并保持唯一 `/cmd_vel` 出口；
- 当前 `DEFER-COMPLIANCE-001/002` 仍未关闭，后续矩阵只能称三层正式配置行为验证，不能
  据此宣称完整严格合规。

## 5. 使用与未验证项

提交本轮工具后，从工作区根目录运行：

```bash
THREE_FLOOR_MATRIX_OUTPUT_ROOT=/tmp/three_floor_20260729 \
  tools/sim/run_three_floor_seed_matrix.sh 8127 6946 1612
```

尚未验证：真实三层串行切换、长时退出清理、同轮结果时间戳归档和真实失败分类。原因是
用户明确要求当前不要启动 Gazebo。这些事实必须由后续实际矩阵报告补齐。

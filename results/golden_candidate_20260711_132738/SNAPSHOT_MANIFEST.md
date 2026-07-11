# Golden Candidate Snapshot: 20260711_132738_seed_42

## 创建时间

2026-07-11

## 当前 HEAD

5f68f60aa5ff71077587572d1d400e4299574401

## 快照内容

- `HEAD.txt`: 当前 HEAD SHA
- `git_status.txt`: `git status --short` 输出
- `tracked_diff.patch`: 所有 tracked 文件的 diff
- `untracked_files.txt`: 未跟踪文件列表（受 .gitignore 影响的未全部列出）
- `source_snapshot/`: 与本次修改相关的重要未跟踪源代码、配置和测试文件
- `effective_params.yaml`: 成功运行使用的有效参数
- `run_archive/`: 成功运行的完整归档（日志、summary、evaluation、detected_danger 等）
- `success_analysis.md`: 成功运行分析报告
- `unit_test_*.log`: 相关单元测试运行结果

## 成功运行关键指标

- seed: 42
- 配置文件: `config/medium_time_budget_test.yaml`
- 探索时间: 263.5s
- 最大离家距离: 23.09m
- 正确识别: 4/6
- 虚警: 0
- 返航成功: True
- 长停比例: 0.03

## 注意事项

本快照不包含 `build/`、`devel/`、`results/runs/` 中其他运行或无关大文件。

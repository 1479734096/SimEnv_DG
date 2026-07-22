# 旧工作区迁移分类

## 等价迁移

- `src/danger_search/danger_search/` 中的生产模块；
- `src/danger_search_localization/` 的参赛方节点、头文件、固定第三方源码和测试；
- 参赛方 package、launch、配置、脚本和直接测试。

## 重写迁移

- 正式 runner 和启动编排；
- 依赖禁用数据、Gazebo 状态服务或修改版官方文件的流程；
- 已确认存在状态机、安全或结果边界缺陷的模块。

## 仅作参考

- 根目录 `fix.md`、历史运行总结和裁剪后的性能指标；
- 既有 AI 分析及临时测试脚本；
- 旧工作区官方保护文件的差异。

## 离线隔离

读取 truth、world、layout、bag 或 Gazebo 状态的分析工具只能进入 `tools/offline/`，不得安装、导入或进入正式 launch。

第 9 层复核后，旧工作区的 `evaluation_tools.py`、`batch_evaluate.py`、`run_seed.py`
及诊断脚本不直接迁入：它们混合启动、真值评测、归档和调参，不满足单向依赖。当前只
保留根目录离线边界说明和自动审计；第 10 层如需批量运行器，应在 `tools/offline/`
重新实现“正式任务退出后才读取真值”的最小流程。

## 不迁移

`build/`、`devel/`、`logs/`、`results/`、生成场景、缓存、编辑器状态、临时文件以及对官方保护文件的修改。

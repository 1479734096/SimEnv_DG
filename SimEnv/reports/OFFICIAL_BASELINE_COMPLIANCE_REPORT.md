# 官方基线文件合规恢复报告

## 结论

本轮已将以下既有说明文件恢复到官方源码导入基线 `94ceb64`，后续不再直接修改：

- `README.md`
- `docs/algorithm-interfaces.md`
- `docs/quick-start.md`
- `docs/reference.md`
- `docs/troubleshooting.md`

后续操作说明、阶段结论和验收证据只通过新增 `reports/` 文件记录，不修改已有官方说明文档。

## 校验结果

- 上述 5 个文件与 Git 官方导入基线逐字节一致。
- `git diff --check` 通过。
- `OFFICIAL_FILES.sha256` 当前剩余 4 个不匹配项，均不属于本轮新增修改：
  - `auto.sh`：此前经用户明确授权修改，用于 Gazebo 安全启动同步。
  - `src/unitree_guide/unitree_guide/unitree_guide/include/FSM/FSM.h`：此前经用户确认的 reset 安全修复。
  - `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/FSM.cpp`：此前经用户确认的 reset 安全修复。
  - `src/unitree_guide/unitree_ros/robots/a1_description/xacro/gazebo.xacro`：用户本地改动，保持未提交且不覆盖。

## 后续合规门禁

每个阶段提交前执行以下检查：

1. 校验 `OFFICIAL_FILES.sha256`，新增不匹配必须停止并说明。
2. 审计外部话题、服务和文件输入，禁止参赛逻辑使用真值或禁用接口。
3. 优先在新增参赛代码、wrapper 或测试夹具中实现修复。
4. 如候选方案必须修改官方初始文件，实施前先报告依据、影响和替代方案，并等待用户明确授权。
5. 说明材料只新增，不编辑已有官方说明文档。

## 未关闭风险

`junior_ctrl` 官方控制链仍存在真值话题依赖的历史实现。由于 `unitree_guide` 属于官方基线，当前不直接修改；在赛方是否将该控制器视为豁免基础设施得到明确依据前，不宣称完整流程已通过“零真值依赖”合规验收。

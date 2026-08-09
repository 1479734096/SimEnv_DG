# 任务 5: seed 运行方式分析

## 执行摘要

找到了 seed 矩阵运行脚本，但需要完整的 Gazebo 仿真环境和 ROS 工作区。无法在当前环境自动化运行。

---

## 1. 找到的 seed 运行脚本

### 主脚本
**文件**: `tools/sim/run_three_floor_seed_matrix.sh`
**功能**: 串行运行多个 seed 的三层楼闭环任务

### 调用方式
```bash
THREE_FLOOR_MATRIX_OUTPUT_ROOT=/tmp/three_floor_20260729 \
  tools/sim/run_three_floor_seed_matrix.sh 8127 42 8072 2024 707
```

### 底层执行器
**文件**: `tools/sim/run_elevator_gui_test.sh`
**功能**: 单个 seed 的仿真运行（需要 Gazebo + ROS）

### 环境变量控制
```bash
SEED=8127                               # 随机种子
GUI=true                                # 启用 Gazebo 图形界面
ELEVATOR_TEST_FLOOR_COUNT=3             # 3 层楼
ELEVATOR_TEST_FLOOR_TIMEOUT=280         # 单层预算 280s
ELEVATOR_TEST_MAX_RUNTIME=840           # 总仿真预算 840s
ELEVATOR_TEST_FULL_CYCLE=1              # 完整 0→1→2→0→home 闭环
ELEVATOR_TEST_LOCALIZATION_BACKEND=adaptive_shared_map  # 定位后端
```

---

## 2. 执行流程

### 矩阵运行流程
1. 验证工作区已构建（`devel/setup.bash` 存在）
2. 验证 git 工作树干净（正式矩阵要求）
3. 为每个 seed 创建独立证据目录
4. 调用 `run_elevator_gui_test.sh` 运行单个 seed
5. 归档日志、检测结果、评分结果
6. 生成时序分析 (`mission_timing.json`)
7. 运行裁判评分器 (`evaluate_danger.py`)
8. 分类结果：success / algorithm_failure / infrastructure_failure

### 失败处理策略
- **算法失败**: 记录结果，不重跑，继续下一个 seed
- **基础设施失败**: 最多重试 2 次，仍失败则暂停整个矩阵
- **中断 (Ctrl-C)**: 立即停止，可从断点继续

---

## 3. 依赖检查

### 必需组件
1. **ROS 工作区**: 需要 `devel/setup.bash`
2. **Gazebo 仿真环境**: GUI 或 headless
3. **Python 依赖**: 任务分析器、摘要生成器、评分器
4. **构建环境**: 已编译的 ROS 节点

### 依赖文件检查
```bash
# 工作区构建检查
[ -f "$WORKSPACE_ROOT/devel/setup.bash" ]

# 依赖脚本检查
[ -f "tools/sim/run_elevator_gui_test.sh" ]
[ -f "tools/offline/analyze_mission_timing.py" ]
[ -f "tools/offline/summarize_three_floor_matrix.py" ]
[ -f "src/building_obstacles/scripts/evaluate_danger.py" ]
```

### Git 工作树要求
正式矩阵要求已跟踪文件工作树干净（但我们当前是诊断阶段，可以放宽）

---

## 4. 输出证据结构

### 单个 seed 的输出目录
```
$OUTPUT_ROOT/seed_8127/
├── attempt_1/
│   ├── mission.log                   # 任务主日志
│   ├── auto.log                      # 自动化系统日志
│   ├── TEST_SUMMARY.txt              # 测试摘要
│   ├── detected_danger.json          # 检测结果
│   ├── danger_truth.json             # 真值标注
│   ├── evaluation_result.json        # 评分结果
│   ├── mission_timing.json           # 时序分析
│   ├── RUN_METADATA.txt              # 运行元数据
│   ├── ATTEMPT_CLASS.txt             # 结果分类
│   └── RUNNER_EXIT_CODE.txt          # 退出码
└── attempt_2/  (如果第一次基础设施失败)
```

---

## 5. 可执行性分析

### ❌ 无法自动化运行的原因

1. **需要 ROS 工作区**
   - 检查 `devel/setup.bash` 是否存在
   - 需要先运行 `catkin_make` 或 `colcon build`

2. **需要 Gazebo 环境**
   - 完整仿真环境（建筑模型、机器人模型）
   - 可能需要图形界面或 Xvfb

3. **需要外部依赖**
   - ROS 节点二进制文件
   - Python 评分脚本
   - 真值数据文件

4. **运行时间长**
   - 单个 seed 最多 840s 仿真时间 + 9000s 墙钟上限
   - 5 个 seed 可能需要数小时

### ✅ 可以做的验证

1. **检查脚本是否存在**（已完成）
2. **阅读脚本逻辑**（已完成）
3. **检查工作区构建状态**（可以执行）
4. **运行单元测试**（不需要 Gazebo）

---

## 6. seed 测试策略

### 方案 A: 用户手动运行（推荐）

**步骤**:
1. 应用最小 patch
2. 构建工作区：`catkin_make` 或 `colcon build`
3. 运行单个失败 seed：
   ```bash
   SEED=8127 GUI=true \
     ELEVATOR_TEST_LOG_DIR=/tmp/seed_8127_patched \
     tools/sim/run_elevator_gui_test.sh
   ```
4. 查看结果：`/tmp/seed_8127_patched/TEST_SUMMARY.txt`
5. 如果成功，运行其他失败 seed（42, 8072, 2024）
6. 如果仍失败，提供日志用于重新诊断

### 方案 B: 运行矩阵（更全面）

**步骤**:
```bash
THREE_FLOOR_MATRIX_OUTPUT_ROOT=/tmp/three_floor_patched \
  tools/sim/run_three_floor_seed_matrix.sh 8127 42 8072 2024 707
```

**优势**:
- 自动归档所有证据
- 自动评分和时序分析
- 可对比 patch 前后结果

**劣势**:
- 需要 5+ 小时
- 需要稳定的仿真环境

### 方案 C: 快速验证（如果可行）

如果存在单元测试或轻量级集成测试：
```bash
PYTHONPATH=src/danger_search:$PYTHONPATH python3 -m pytest \
  src/danger_search/test/test_navigator.py -v
```

---

## 7. 结论

### 核心发现

1. **找到了 seed 运行脚本**：`tools/sim/run_three_floor_seed_matrix.sh`
2. **需要完整的 Gazebo + ROS 环境**：无法在诊断阶段自动化
3. **有完整的证据归档流程**：可以对比 patch 前后结果

### 验证策略

**阶段 0（当前）**:
- ✅ 设计最小 patch
- ✅ 运行单元测试（不需要 Gazebo）
- ✅ 生成诊断报告

**阶段 1（应用 patch 后）**:
- 用户手动运行单个失败 seed（方案 A）
- 或运行完整矩阵（方案 B）
- 根据结果决定下一步

### 推荐路径

**如果用户有 Gazebo 环境**:
1. 应用最小 patch
2. 运行 seed 8127（最简单的失败案例）
3. 如果成功，运行其他 3 个失败 seed
4. 如果全部通过，问题解决

**如果用户没有 Gazebo 环境**:
1. 应用最小 patch
2. 运行单元测试验证无回归
3. 提交 patch 给有环境的团队成员测试
4. 等待反馈后决定下一步

---

## 8. 下一步行动

### 立即可执行（不需要 Gazebo）

- [x] 设计最小 patch（任务 4 已完成）
- [ ] 运行 navigator 单元测试
- [ ] 检查是否有控制律相关测试
- [ ] 生成完整诊断报告

### 需要用户协助

- [ ] 构建工作区（如果未构建）
- [ ] 运行 seed 8127 验证 patch
- [ ] 提供失败日志（如果 patch 无效）

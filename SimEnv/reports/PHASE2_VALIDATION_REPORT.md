# Phase 2 Seed 8127 验证报告

## 执行状态

**启动时间**: 2026-08-08 10:59  
**测试 Seed**: 8127  
**Commit**: 26b176f (IDLE 模式死区补偿)  
**状态**: 运行中...

## 监控计划

由于测试需要 1-1.5 小时完成，采用以下策略：

1. **被动监控**: 测试在后台运行，完成后检查结果
2. **关键指标**: 
   - 返航阶段是否有原地摆动（核心目标）
   - rotate_in_place() 角速度补偿效果
   - 最终失败模式

3. **预期结果**:
   - 电梯导航失败（已知独立问题）
   - 返航应该成功（无 60+ 秒原地摆动）
   - 失败模式应该是 `floor_change_failed_elevator_navigation_unreachable`

## 验证步骤（测试完成后）

### 步骤 1: 检查返航阶段
```bash
# 提取返航阶段所有状态
grep -E '"state": "RETURN_HOME"' /tmp/phase2_quick_8127/mission.log | \
  jq -r '{time: .time, mode: .controller.controller_mode, pose: .controller.pose, cmd: .controller.active_motion_command}'

# 检查 rotate_in_place 调用时的角速度补偿
grep -E '"state": "RETURN_HOME"' /tmp/phase2_quick_8127/mission.log | \
  jq 'select(.controller.active_motion_command[2] != 0) | {time: .time, omega: .controller.active_motion_command[2]}'
```

### 步骤 2: 检查是否有原地摆动
```bash
# 查找位置几乎不变但时间持续的区间
grep -E '"state": "RETURN_HOME"' /tmp/phase2_quick_8127/mission.log | \
  jq -r '{time: .time, x: .controller.pose[0], y: .controller.pose[1]}' | \
  awk '{if (NR>1 && sqrt(($2-px)^2 + ($4-py)^2) < 0.05) stuck++; else stuck=0; if (stuck>10) print "原地摆动检测:", $0; px=$2; py=$4}'
```

### 步骤 3: 确认最终失败原因
```bash
# 提取终止原因
grep -E '"terminal_reason"' /tmp/phase2_quick_8127/mission.log | tail -1 | jq -r '.mission.terminal_reason'
```

## 当前状态

**测试启动成功** ✅  
**仿真时间**: ~32秒（11:04 检查）  
**任务状态**: EXPLORE_FLOOR（楼层探索阶段）  
**控制模式**: PATH  
**预计完成**: ~12:00-12:30（1-1.5小时后）

测试在后台正常运行中。返航阶段预计在仿真时间 300-400 秒后开始。

## 验证流程（测试完成后执行）

由于测试运行时间较长，建议采用以下验证流程：

### 1. 确认测试完成
```bash
# 检查是否有终止标记
grep -E '"terminal_reason"' /tmp/phase2_quick_8127/mission.log | tail -1
```

### 2. 快速验证关键指标

**返航阶段原地摆动检测**（核心指标）:
```bash
# 提取返航阶段所有状态，检查位置变化
grep '"state": "RETURN_HOME"' /tmp/phase2_quick_8127/mission.log | \
  grep -o '"pose": \[[^]]*\]' | \
  awk -F'[\\[\\],]' '{x=$2; y=$3; if (NR>1) {dx=x-px; dy=y-py; d=sqrt(dx*dx+dy*dy); if (d<0.05) stuck++; else stuck=0; if (stuck>20) print "检测到原地卡滞 >20 samples"} px=x; py=y}'
```

**rotate_in_place 死区补偿验证**:
```bash
# 检查返航时角速度命令是否都 ≥ 0.55 rad/s
grep '"state": "RETURN_HOME"' /tmp/phase2_quick_8127/mission.log | \
  grep -o '"angular_command": [0-9.-]*' | \
  awk -F': ' '{omega=$2; if (omega!=0 && omega>-0.55 && omega<0.55) print "发现小角速度命令:", omega}'
```

### 3. 生成完整报告

根据验证结果更新此报告，包含：
- 最终失败模式
- 返航阶段详细分析
- 死区补偿效果评估
- 下一步建议

---
**更新时间**: 2026-08-08 11:05  
**状态**: 测试运行中，等待完成

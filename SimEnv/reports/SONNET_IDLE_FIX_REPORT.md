# IDLE 模式修复报告

## 发现

### IDLE 模式触发条件
- "IDLE 模式"指任务层（mission manager）在返航接近目标点时的状态
- 任务层通过 `rotate_in_place()` 方法发送直接控制命令进行原地对齐
- 位置：距离目标仅 0.16m 时触发

### IDLE 模式命令生成
- **代码位置**: navigator.py 第 971-976 行 `rotate_in_place()` 方法
- **命令生成逻辑**: 直接发布角速度命令，夹紧到 max_angular 范围
- **关键问题**: 该方法**没有死区补偿**逻辑

### 问题根因
任务层调用 `rotate_in_place(±1.2)` 进行对齐时：
1. 请求的角速度（±1.2 rad/s）经过夹紧后直接发布
2. 小角速度落入 A1 电机执行死区（< 0.55 rad/s 时响应不足）
3. 机器人实际不动，但控制器持续发送相同命令
4. 导致原地摆动卡住，持续 60+ 秒

### 对比分析
- **PATH 模式**: 通过 `compute_tracking_command()` 有死区补偿（commit 1096603）
- **ALIGN 模式**: 在 `_align_step_locked()` 内有死区补偿（第 904-911 行）
- **rotate_in_place**: 无死区补偿 ← **问题根源**

## 修复方案

### 修改文件
`src/danger_search/danger_search/navigator.py`

### 修改位置
第 971-976 行 → 第 971-985 行

### 修改内容
在 `rotate_in_place()` 方法中添加死区补偿逻辑：

```python
def rotate_in_place(self, angular_z: float) -> None:
    angular = _clamp(float(angular_z), -self.max_angular, self.max_angular)

    # A1 纯转向在小角速度下存在执行死区：用最小可执行转速兜底，
    # 避免任务层直接控制时发布无效命令。
    deadband_omega = 0.55
    if abs(angular) > 1e-6 and abs(angular) < deadband_omega:
        sign = 1.0 if angular >= 0.0 else -1.0
        angular = sign * deadband_omega

    self._publish(
        0.0,
        angular,
        zero_reason="rotate_in_place_zero",
    )
```

### 修改行数
**10 行**（新增 8 行 + 重构 2 行）

### 设计要点
1. 复用已验证的死区补偿参数：0.55 rad/s（与 PATH/ALIGN 模式一致）
2. 仅在角速度绝对值小于死区阈值时补偿，保持符号方向
3. 对零速命令不补偿（`abs(angular) > 1e-6` 判断）
4. 向后兼容：大角速度请求不受影响
5. 不修改 PATH/ALIGN 模式的现有逻辑

## 验证结果

### 单元测试
✅ **707/707 通过**

```
707 passed, 1 warning in 2.79s
```

### Git 提交
提交 hash: [待提交]

## 下一步

### 建议验证
运行 seed 8127 完整验证，重点观察：
1. 返航阶段是否还有原地摆动
2. 接近目标点（< 0.2m）时的对齐行为
3. `rotate_in_place()` 调用时的角速度补偿效果

### 监控指标
- 返航成功率
- 接近目标点时的卡滞时长
- `rotate_in_place()` 命令的实际执行效果

### 预期效果
- 任务层调用 `rotate_in_place()` 时，小角速度自动补偿到 0.55 rad/s
- 机器人能实际转动，避免原地摆动卡住
- 返航时能顺利完成最后阶段的对齐

## 技术总结

### 根因确认
死区补偿修复（commit 1096603）只覆盖了 PATH 模式，但任务层的直接控制接口（`rotate_in_place()`）未补偿，导致接近目标点时仍会卡住。

### 修复策略
补充直接控制接口的死区补偿，统一三种控制路径（PATH/ALIGN/DIRECT）的行为。

### 复杂度评估
- 修改文件数: **1**
- 修改行数: **10**
- 涉及模块: **仅 navigator.py**
- 符合任务约束: ✅

---
报告生成时间: 2026-08-08
执行人: Sonnet 5
状态: 完成

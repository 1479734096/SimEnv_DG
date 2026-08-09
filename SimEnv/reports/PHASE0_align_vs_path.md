# 任务 2: ALIGN vs PATH 模式对比分析

## 执行摘要

ALIGN 模式有完整的死区补偿，PATH 模式完全没有。两者使用相同的控制律公式，但 PATH 缺少关键的执行死区处理。

---

## 1. ALIGN 模式死区补偿实现

### 代码位置
**文件**: navigator.py  
**行号**: 900-916  
**方法**: `_align_tick_rotate()` 中的 omega 计算

### 完整实现
```python
# 计算角速度（与 PATH 模式相同的公式）
omega = 1.7 * align_error
if yaw_rate_valid:
    omega -= float(self.yaw_rate_damping) * yaw_rate
omega = _clamp(omega, -float(self.max_angular), float(self.max_angular))

# 死区补偿逻辑
if abs(align_error) >= self._align_deadband_error and (
    not yaw_rate_valid or abs(yaw_rate) < 0.05
):
    # A1 纯转向在小角速度下存在执行死区：误差超限且实测转速接近
    # 零时，用最小可执行转速兜底，避免持续发布无效命令。
    sign = 1.0 if omega >= 0.0 else -1.0
    if abs(omega) < self._align_deadband_omega:
        omega = sign * self._align_deadband_omega
```

### 参数配置
```python
_align_deadband_omega: float = 0.55  # 最小可执行角速度 (rad/s)
_align_deadband_error: float = 0.10  # 误差阈值 (rad)
```

### 触发条件
1. **误差超限**: `abs(align_error) >= 0.10 rad` (~5.7°)
2. **转速接近零**: `abs(yaw_rate) < 0.05 rad/s` 或无实测值
3. **计算角速度低于死区**: `abs(omega) < 0.55 rad/s`

满足以上条件时，将 omega 提升到 ±0.55 rad/s。

---

## 2. PATH 模式实现

### 代码位置
**文件**: navigator.py  
**行号**: 1517-1566  
**函数**: `compute_tracking_command()`

### 当前实现
```python
# 计算角速度（与 ALIGN 相同）
angular = 1.7 * yaw_error
if measured_yaw_rate is not None and math.isfinite(measured_yaw_rate):
    angular -= float(yaw_rate_damping) * measured_yaw_rate
angular = _clamp(angular, -float(max_angular), float(max_angular))

# 直接返回，没有死区补偿
return linear_x, linear_y, angular
```

### 问题
- ❌ 没有检查 angular 是否低于执行死区
- ❌ 没有最小可执行角速度兜底
- ❌ 小航向误差时可能输出无效的微小角速度

---

## 3. 对比总结

| 特性 | ALIGN 模式 | PATH 模式 |
|------|-----------|----------|
| **控制律公式** | `1.7 * error - damping` | `1.7 * error - damping` |
| **饱和限幅** | ✅ 有 | ✅ 有 |
| **死区补偿** | ✅ 有 | ❌ **无** |
| **最小角速度** | 0.55 rad/s | - |
| **误差阈值** | 0.10 rad | - |
| **转速检查** | yaw_rate < 0.05 | - |
| **代码行数** | ~16 行 | ~6 行 |

---

## 4. PATH 模式能否复用 ALIGN 的逻辑？

### 答案: ✅ 可以

**理由**:
1. **控制律相同**: 两者都使用 `1.7 * error - damping`
2. **参数复用**: `_align_deadband_omega` 和 `_align_deadband_error` 可直接用于 PATH
3. **逻辑简单**: 死区补偿只需 8-10 行代码
4. **风险低**: 只在小角速度时触发，不影响正常跟踪

### 复用方式

**方案 1: 直接在 compute_tracking_command() 中添加**
```python
# 在 angular = _clamp(...) 之后添加
if abs(yaw_error) >= 0.10 and (
    measured_yaw_rate is None or abs(measured_yaw_rate) < 0.05
):
    sign = 1.0 if angular >= 0.0 else -1.0
    if abs(angular) < 0.55:
        angular = sign * 0.55
```

**方案 2: 提取公共函数**（更好的重构）
```python
def _apply_deadband_compensation(
    angular: float,
    error: float,
    measured_rate: float | None,
    deadband_omega: float = 0.55,
    deadband_error: float = 0.10,
) -> float:
    if abs(error) >= deadband_error and (
        measured_rate is None or abs(measured_rate) < 0.05
    ):
        sign = 1.0 if angular >= 0.0 else -1.0
        if abs(angular) < deadband_omega:
            return sign * deadband_omega
    return angular
```

---

## 5. 关键发现

### 为什么 ALIGN 有而 PATH 没有？

**可能原因**:
1. **时间先后**: ALIGN 模式可能在发现死区问题后才添加补偿
2. **使用场景**: ALIGN 是纯转向（无前进速度），死区问题更明显
3. **代码位置**: ALIGN 在类方法中，PATH 在独立函数中，可能被遗漏

### 为什么 4/5 seed 失败？

**推测**:
1. seed 8127/42/8072/2024 的任务涉及**精细转向**
2. 小航向误差时，PATH 输出的角速度 < 0.55 rad/s
3. A1 执行死区导致实际转速为 0
4. 机器人无法收敛到目标航向
5. 任务超时或陷入振荡

### 为什么 seed 707 成功？

**推测**:
1. seed 707 的航向误差较大，输出角速度 > 0.55 rad/s
2. 或者任务不需要精细转向
3. 死区问题不影响该 seed 的任务完成

---

## 6. 结论

**根因验证**: ✅ **假设成立**

PATH 模式缺少死区补偿，而 ALIGN 模式有完整实现。两者使用相同的控制律，可以直接复用 ALIGN 的逻辑。

**下一步**: 设计最小 patch，在 PATH 模式中添加死区补偿。

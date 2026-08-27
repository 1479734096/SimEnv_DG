# STAGE28：电梯目标门双通道放行与近距离几何放宽技术报告

## 一、 背景与根因

在测试（seed `8127`）中，电梯到达 1 层出梯时发生 8 秒超时。经日志回溯定位到两个关键根因：
1. **一刀切禁用 RGB-D**：STAGE27 将显式换层出梯的 RGB-D 确认硬编码封死（`not floor_transition_explicit`）。实测中 **RGB-D 深度图连续 71 帧高置信度确认门已大开**，但被系统全部强行无视；
2. **Livox 门洞立柱几何过于苛刻**：系统复用的 `detect_opening_ahead` 原为大厅远距离寻门设计，要求单侧门柱垂直连续跨度 $\ge 0.50\text{m}$ 且点数 $\ge 6$。机器狗在轿厢近距离（约 $1.2\text{m}$）朝外斜视时，受轿厢内壁遮挡，单侧门框垂直点云跨度只有 $0.3\text{m}$ 左右，导致 Livox 单通道未能连续拟合出两帧标准立柱。

---

## 二、 核心修改

### 1. 双通道并联互补放行（OR 逻辑）
在 [`elevator_observation_runtime.py`](../src/danger_search/danger_search/elevator_observation_runtime.py) 中：
- 消除任何阻断 RGB-D 确认的限制，实现真正的**双重冗余、互为兜底**：
  $$\text{confirmed} = (\text{visual\_confirmations} \ge 2) \;\;\mathbf{OR}\;\; (\text{livox\_confirmations} \ge 2)$$
- **0→1 上楼出梯**：RGB-D 视野开阔无盲区，**2 帧（约 0.2s）瞬间放行**；
- **1→0 下楼出梯**：若 0 楼深度更深比例不足，**Livox 点云无阻挡放行**。

### 2. 放宽近距离电梯门立柱几何约束
在 [`local_semantic_detector.py`](../src/danger_search/danger_search/local_semantic_detector.py) 中：
- 将 `detect_opening_ahead` 的单侧立柱垂直跨度 (`min_side_height_span`) 由 **`0.50m` 优化为 `0.25m`**（适配轿厢内斜视角）；
- 立柱最小点数 (`min_evidence_points`) 由 **`6` 点优化为 `3` 点**；
- 支持动态传参覆盖。

---

## 三、 测试与验证

1. **定向与全量单元测试**：
   ```bash
   PYTHONPATH=SimEnv/src/danger_search:$PYTHONPATH python3 -m pytest SimEnv/src/danger_search/test/ -q
   ```
   **结果：859 passed in 4.30s（100% 绿灯通过）**。
2. 覆盖场景：
   - 显式换层出梯时 RGB-D 2 帧瞬间放行；
   - 显式换层出梯时无 RGB-D、仅 Livox 2 帧稳定放行；
   - 两者均未满足时保持等待并超时兜底。

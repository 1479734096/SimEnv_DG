# 9.6 前沿远距离小碎片剪枝优化报告

## 一、问题背景与现象

在多房间与多楼层自主探索任务中，机器人多次出现**舍近求远、折返清扫已探索房间微小死角**的异常规划行为：

1. **现象案例（simtime 198.4s）**：
   - 机器人在右侧房间（`floor_0_room_1`）南端完成大部区域探索；
   - 此时中央走廊深处存在大量未探索区域（未知连通块多达数百格，是通往未探索房间的必经主通道）；
   - 但规划器计算出的最优目标并非走廊深处，而是掉头横跨大半个地图，穿过走廊返回左侧已访问房间（`floor_0_room_0`）去清扫下方残留的阴影；
2. **次生危害**：
   - 机器人因远距离（$> 3.5\text{m}$ 乃至十几米外）遗留的家具背阴阴影、桌腿遮挡夹缝或雷达噪点死角，在房间与走廊之间反复折返；
   - 高距离惩罚（`path_cost_weight: 3.2`）与开方收益压制了高价值远目标的优势，而零碎微小阴影又未在远距离被有效剪枝，极大地浪费了宝贵的 840 秒任务探索预算。

---

## 二、根因深入剖析

经对前沿提取逻辑 [`frontier.py`](../src/danger_search/danger_search/frontier.py)、正式配置文件 [`danger_search.yaml`](../src/danger_search/config/danger_search.yaml)、官方场景生成器 [`generate_competition_scene.py`](../src/building_obstacles/scripts/generate_competition_scene.py) 及比赛规则 [`competition-rules.md`](../docs/competition-rules.md) 的全链路静态审查，确认了如下核心根因：

### 1. 远距离剪枝门槛过低（旧参数 `min_far_expected_gain = 4.0`）

前沿规划器中具备远距离小孤岛剪枝逻辑 `_is_far_small_pocket`：
```python
def _is_far_small_pocket(self, target_xy, robot_xy, expected_gain):
    return (
        _world_distance(target_xy, robot_xy) > self.far_prune_distance # 3.5m
        and expected_gain < self.min_far_expected_gain                  # 旧值为 4.0
    )
```
当候选点距离 $> 3.5\text{m}$ 且相邻未知格数（`expected_unknown_gain`）低于阈值时直接剔除。
然而旧配置中的阈值设为 `4.0` 格，导致只有 $\le 3$ 格的极端噪点才会被剪枝，绝大部分面积在 4~15 格的细碎死角阴影依然会作为合法目标参与 A* 规划并竞选。

### 2. 旧认知假设与真实物理/生成规则的严重背离

旧配置中将阈值设为 `4.0` 的历史注释为：
> `“收益低于 4 格才直接剪枝。R=0.15m 红球的局部未知投影可接近这一量级，不能再以 8 格阈值漏掉小房间或壁龛。”`

这一假设忽略了**危险源的物理实体尺寸与场景生成硬性避让规则**：

1. **危险源本身的实体栅格占用**：
   - 地图分辨率为 $0.10\text{ m}$（每格面积 $0.01\text{ m}^2$）；
   - 危险源为红色实心球体，半径 $R = 0.15\text{ m}$（直径 $0.30\text{ m}$）；
   - 球体水平截面积 $S = \pi R^2 \approx 0.0707\text{ m}^2$；
   - 离散化至栅格地图上，**单个红球自身的物理实体就至少占据 5 ~ 8 个栅格**。
2. **官方生成规则的避让净空约束**（`generate_competition_scene.py`）：
   - **只在房间内生成**：走廊、过道绝无危险源；
   - **墙体避让（`ROOM_WALL_CLEARANCE = 0.45m`）**：球心距墙体 $\ge 0.60\text{m}$，球体表面距墙面 $\ge 0.45\text{m}$；
   - **家具避让（`FURNITURE_CLEARANCE = 0.35m`）**：球心距家具 $\ge 0.50\text{m}$，球体表面距家具 $\ge 0.35\text{m}$；
   - **门禁保留区（`DOOR_KEEP_OUT`）**：进门横向 2.4m、进深 1.7m 区域严禁放置任何源。
3. **物理与逻辑推论**：
   - 危险源绝对不可能孤立悬空存在于一个只有 4 格（$20\text{cm} \times 20\text{cm}$）的虚空死角里；
   - 任何能够合法生成危险源的位置，周围必须有至少直径 $1.0\text{ m}$ 的开阔区域（对应面积约 78 格）；
   - 机器人在房间内走过时，家具背阴或桌角由于视线遮挡残留的 $< 16$ 格（$< 0.16\text{ m}^2$）狭窄盲区，**在物理和规则上 100% 绝无可能容纳合法的危险源**；
   - 旧阈值 4.0 过于保守，使得机器人频繁把绝无藏球可能的死角噪点当成高价值目标折返。

---

## 三、修改方案与工程实施

基于上述定量几何与规则推导，对剪枝阈值及全链路配置进行了同步提升：

### 1. 将 `min_far_expected_gain` 提升至 `16.0`
- **文件**：[`SimEnv/src/danger_search/config/danger_search.yaml`](../src/danger_search/config/danger_search.yaml#L315-L319)
- **修改内容**：
  ```yaml
  # 远距离封闭孤岛前沿剪枝：收益低于 16 格才直接剪枝。结合危险源物理尺寸(D=0.3m,
  # 约 7 格)与墙体/家具避让安全间距(>=0.35m~0.45m)，远距离小阴影低于 16 格
  # 在物理与规则上不可能容纳危险源，直接剪枝避免无效折返。
  min_far_expected_gain: 16.0
  far_prune_distance: 3.5
  ```
- **物理保证**：
  - 距离 $> 3.5\text{m}$ 以外的目标，其连通未知空间若不足 16 格（约 $0.16\text{ m}^2$），在进入 A* 搜索前直接剪除；
  - 距离 $\le 3.5\text{m}$ 的近处目标不受此影响，机器人在房内探索顺手清扫依然被允许。

### 2. 同步更新运行时缺省回退参数
- **文件**：[`SimEnv/src/danger_search/danger_search/mission_config.py`](../src/danger_search/danger_search/mission_config.py#L1063-L1065)
- **修改内容**：将 `_param("frontier/min_far_expected_gain", 4.0)` 的回退缺省值同步更新为 `16.0`，保证脱离 YAML 参数服务器时的配置一致性。

### 3. 同步测试与接口契约断言
- **文件**：
  - [`SimEnv/src/danger_search/test/test_interface_documentation.py`](../src/danger_search/test/test_interface_documentation.py#L286)
  - [`SimEnv/src/danger_search/test/test_mission_components.py`](../src/danger_search/test/test_mission_components.py#L122)
- **修改内容**：更新对应单元测试与参数契约断言中的预期值（`16.0`），确保测试集与生产代码契约严格闭合。

---

## 四、静态审查结论与影响评估

1. **调用链一致性**：
   `danger_search.launch` $\to$ 加载 `danger_search.yaml` $\to$ `load_mission_config()` $\to$ `FrontierExplorer` 构造传入 `min_far_expected_gain=16.0` $\to$ `_prepare_frontier_targets` 及其预规划过滤。参数贯通链路完整无阻。
2. **预期收益**：
   - 彻底阻断距离 $> 3.5\text{m}$ 处小于 16 格（如桌角、沙发靠背、踢脚线微小缝隙）前沿参与 A* 竞争；
   - 极大降低机器人刚离开房间进入走廊后、又被旧房间微小残余阴影吸引掉头的概率；
   - 促使规划器在长距离下更倾向于选择走廊深处、新房间等数百格的高收益前沿。
3. **未实测说明**：
   按照系统规则，不主动执行物理仿真。修改对特定场景全图覆盖用时与轨迹连贯性的最终提升效果，将在后续 Gazebo 实际运行中检验。

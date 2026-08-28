# STAGE33 稀疏地标纠偏与电梯开口重试报告

## 1. 结论

本阶段在不读取任何真值或布局文件的前提下，完成了主入口与 0 层电梯双锚点稀疏平移纠偏、
定位位姿与 Adaptive Shared Map 原子平移、电梯开口确认一次有界重试，并将正式与电梯专项 runner
的 Livox 定位量程默认值统一为 `15.0m`。实现与静态验证已经完成，但因完整仿真尚未执行且
官方文件哈希检查存在历史不一致，本阶段尚不满足提交和打标签条件。

## 2. 行为变更

### 2.1 本阶段核心改动

- 首次传感器注册后冻结主入口和电梯周边静态 Livox 结构，排除动态门板条带。
- 复访只使用当前楼层、当前定位 epoch 的连续 3 帧，同时检查重叠率、P70 残差、唯一性、帧间离散和 `1.20m` 上界。
- Python 与 C++ 双层检查停车状态；C++ 在共享图临界区内同步平移已发布位姿、局部图和初始平移基准，成功后发布带序号回执。
- 电梯门首次开口确认超时为 `15.0s`；只允许一次 `10.0s` 重试，重试前必须等待至少 `0.5s` 后的新 Livox 帧。
- 源层重试执行锚点复核、重新对准、重发呼梯和开门；目标层不移动，只重新开门。两层都不再把呼梯成功当作开门成功。
- 门区地图仍只在局部传感器确认开口后清理，重试本身不提前清图。

### 2.2 同一提交候选中的前序未提交改动

本报告生成时，工作树还包含上次 Git 提交之后已经实现、但尚未形成独立提交的以下改动。
为避免把它们误记成 STAGE33 的稀疏纠偏算法，统一在此单列：

- 红球定位改为使用公开固定半径 `R=0.15m` 的高斯-牛顿几何拟合；Tracker 按距离、拟合
  RMSE 与观测质量加权融合，并在远距离证据不足时允许任务层主动靠近复核。
- 前沿选择在规划前永久跳过距离大于 `3.5m` 且收益小于 `30` 格的远距离小区域，A* 解析
  不得重新激活这些候选。
- 异步预规划基础设施仍保留，但正式配置固定
  `async_planning_enabled: false`、`opportunistic_replan_period: 0.0`，保持单线程同步规划。
- 正式安全基线固定为 `max_linear: 0.75m/s`、`boundary_hold_margin: 0.40m`、
  `observation_hold_timeout: 3.0s`；本提交候选没有放宽这些边界。
- 正式任务预算当前为单层/两层/三层 `500/720/900s`，非末层单层超时 `310s`，换层预留
  `50s`。这些是当前配置事实，尚未经过本轮完整仿真验证。

## 3. 配置与合规

- 正式 `danger_search.launch` 启用地标纠偏，底层独立 `scan_localization.launch` 仍默认关闭，避免诊断启动无上层质量门控时接受请求。
- 新增的 correction request/state 为参赛算法内部派生话题，不引入外部输入。
- 生产静态搜索未发现对 truth、world、metadata 或 Gazebo 真值话题的新直接读取；命中字符仅位于接口拒绝策略。
- 速度 `0.75m/s`、防坠边距 `0.40m`、同步规划与关闭机会重规划均未改变。
- 赛方基线 ZIP SHA-256：`168c61d25848bf4efe43f9d14083169cfa9b97e433a5f3699e880b499ac4c308`。

## 4. 验证

1. 定位包编译成功：
   `source /opt/ros/noetic/setup.bash && catkin_make --pkg danger_search_localization -j2`。
2. 定位包 gtest/rostest 全部通过；`catkin_test_results`
   汇总为 `158 tests, 0 errors, 0 failures, 0 skipped`。
3. 全仓 Python 测试：`893 passed, 0 failed`。
4. `python3 tools/offline/check_production_boundary.py`：参赛 Python/离线边界审计通过。
5. `git diff --check`：通过。
6. `sha256sum -c OFFICIAL_FILES.sha256`：未通过，共有 6 个不一致项。当前工作树相对 HEAD
   只有 `docs/doors-and-elevator.md` 仍存在本轮开始前已有的暂存修改；其余 5 项
   `auto.sh`、`docs/quick-start.md`、`FSM.h`、`FSM.cpp`、`gazebo.xacro` 已存在于 HEAD，
   不是 STAGE33 新增改动。本轮未擅自覆盖这些受保护文件。

## 5. 未验证与下一步

按用户要求，本阶段未启动 Gazebo 定向或完整闭环仿真。下一步应使用默认 `15.0m` 量程跑一次
0→1→0 闭环，重点检查锚点冻结/评估/注入回执事件、两侧显式开门调用、新帧屏障以及重试耗尽的显式失败原因。
在创建阶段提交和 `migration-layer-33` 标签前，还必须单独裁决上述 6 个官方文件哈希不一致项。

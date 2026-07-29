# 第 12 层 M6：GICP 隐藏旋转诊断检查点

## 1. 结论

已补齐相邻帧 GICP 隐藏旋转的只读诊断链。定位节点现在记录完整 GICP
候选变换中的平面 yaw 和三维轴角幅值，离线工具按既有 0.5 秒公共同窗聚合
RGB-D/Livox 的旋转均值、峰值及其与平移误差的相关性。

本阶段不新增融合门控，不改变 GICP 接受条件、平移累计、位姿输出、地图更新或
任务行为。隐藏旋转是否能够稳定识别高漂移窗口，仍需两轮 GUI 录包验证。

## 2. 根因假设与边界

输入点云已由 `/trunk_imu` 旋到统一方向，但 PCL GICP 仍优化完整刚体变换，
生产链只消费最终 XY 平移。未消费的旋转自由度可能在门口、转弯和电梯出口等
退化几何中与平移耦合。本阶段只暴露该候选旋转，不以假设替代运行证据。

诊断字段只进入既有逐帧 ROS 日志；该日志仍由
`diagnostic_frame_details=false` 默认关闭。没有新增话题、服务、文件输入、TF、
参数或生产订阅。

## 3. 改动

- `PlanarGicpResult` 追加 `candidate_yaw` 和
  `candidate_rotation_angle`；初始帧和提前失败路径保持 `NaN`。
- 逐帧日志和限频汇总追加候选旋转字段及区间峰值，融合分支不读取这些字段。
- 离线解析器兼容新旧日志；旧日志缺少字段时保持 `NaN`，JSON 输出转换为
  `null`，CSV 追加列而不改旧列。
- 同窗统计新增 RGB-D/Livox 旋转均值、峰值、yaw 绝对值，以及准确命名的
  `hidden_rotation_window_mean_p50`、
  `hidden_rotation_window_max_p95` 和
  `hidden_rotation_error_spearman`。

## 4. 测试与验证

- `/usr/bin/python3 -m pytest
  src/danger_search/test/test_localization_quality_analyzer.py -q`：22 项通过。
- `catkin_make --pkg danger_search_localization
  -DCATKIN_ENABLE_TESTING=ON`：通过。
- `catkin_make
  run_tests_danger_search_localization_gtest_test_planar_gicp_odometry`：6 项通过，
  XML 为 `tests=6, failures=0, errors=0`。
- 新增测试覆盖纯平移旋转接近零、带旋转点云仍保持原接受行为、新旧日志解析、
  非有限旋转、三窗口相关性以及少于三窗口时相关性为 `NaN`。
- 用既有 `seed=8072` M5 bag 重放：成功生成 184 个同窗；旧日志的 RGB-D、
  Livox 隐藏旋转均输出 `null`，证明兼容路径没有伪造零值。
- `git diff --check`、Python 语法检查和
  `tools/offline/check_production_boundary.py`：通过。
- `OFFICIAL_FILES.sha256` 仍仅有 4 项历史差异：`auto.sh`、`FSM.h`、
  `FSM.cpp`、用户修改的 `gazebo.xacro`；本阶段未新增。
- 验证后没有 ROS、Gazebo、控制器、任务或录包进程残留。

测试过程中曾把 `catkin_test_results` 的 XML 文件路径误当目录，聚合命令返回
非零；实际 gtest 已执行并生成有效 XML，不属于测试失败。

## 5. 预登记取证口径

- 先跑已知高漂移 `seed=8072`，再跑成功基线 `seed=6946`；两轮均为两层、
  GUI、33 秒每层、180 秒任务预算、逐帧诊断和独立离线真值。
- 主特征固定为每个 0.5 秒窗口的 `rgbd_rotation_max`；其他旋转字段只作解释。
- RGB-D 误差不超过 0.02 米记为低误差窗，不少于 0.05 米记为高误差窗，
  中间样本不计入 AUC。
- 每轮至少需要 8 个高误差窗和 8 个低误差窗；主特征方向一致且 AUC 均不低于
  0.70，才允许设计默认关闭的生产门控。
- 确定性算法失败不重跑；仅环境或录包失败允许同配置补跑一次。

## 6. 风险与下一步

隐藏旋转可能只是退化场景的伴随现象，而非平移错误的可用先兆。若任一 seed
样本不足、方向不一致或 AUC 未达门槛，本假设立即停止，不接入生产融合。离线
反事实也无法模拟门控后位姿与地图的闭环反馈；即使诊断通过，仍必须经过独立
生产候选和完整任务回归。

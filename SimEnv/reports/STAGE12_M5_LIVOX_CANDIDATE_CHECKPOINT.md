# 第 12 层 M5：Livox 主增量候选检查点

## 1. 目标与结论

根据三个预登记 seed 的同窗离线证据，实现一个默认关闭、可回退的 Livox 平移主增量候选。候选只改变 RGB-D/Livox 后端中“哪个已验收的帧间平移增量推进融合位置”，不改变 IMU 姿态、GICP 验收、scan-to-map 校正、定位健康门或任务状态机。

正式默认值仍为 `rgbd`。只有测试显式设置 `localization_planar_translation_source:=livox` 时才启用候选。

## 2. 行为

- `rgbd`：保持现有行为。RGB-D 新鲜时由 RGB-D 累计量推进融合位置；RGB-D 失鲜时继续使用已有的 Livox 尺度回退。
- `livox`：RGB-D 继续计算、验收并记录诊断量，但不推进融合位置；只有已验收且含有效增量的 Livox 帧推进融合位置。
- Livox 候选使用原始增量，不使用由 RGB-D 距离比反推的 `livox_scale`，避免把被比较支路的误差重新注入候选。
- scan-to-map 校正仍叠加到同一个 `fused_planar_position_`，发布位姿与局部地图提交保持同一坐标状态。
- 非法来源值启动时抛出异常；测试 runner 也在启动 Gazebo 前拒绝非法值。

## 3. 修改范围

- `src/danger_search_localization/include/danger_search_localization/planar_translation_source.hpp`
- `src/danger_search_localization/src/scan_localization_node.cpp`
- `src/danger_search_localization/launch/scan_localization.launch`
- `src/danger_search_localization/test/test_planar_translation_source.cpp`
- `src/danger_search_localization/CMakeLists.txt`
- `src/danger_search/launch/danger_search.launch`
- `tools/sim/run_elevator_gui_test.sh`

未修改官方 SimEnv 文件、正式 YAML、任务状态机、导航、建图、检测或既有报告。

## 4. 验证

- `catkin_make --pkg danger_search_localization -DCATKIN_ENABLE_TESTING=ON`：通过。
- `catkin_make run_tests_danger_search_localization`：98 项通过，0 失败。
- `roslaunch --files` 分别以 `rgbd` 和 `livox` 解析正式 launch：通过。
- `bash -n tools/sim/run_elevator_gui_test.sh`：通过。
- runner 非法来源值：启动前返回码 2 并明确报错。
- `python3 tools/offline/check_production_boundary.py`：通过。
- 官方哈希仍恰有四项历史已知差异，无新增。

完整 Python 回归本检查点未形成有效结果。第一次调用未加载参赛包路径；第二次显式覆盖 `PYTHONPATH` 后丢失 ROS Python 路径，461 项通过，5 项因 `rospy` 不可用失败，18 项因找不到 `rosbag` 报错。按照同一检查最多两次的限制停止重试。该缺口不影响已通过的定位包 C++/ROS 定向测试，但必须在后续统一验收环境中补跑。

## 5. A/B 计划

先用基线失败且漂移最大的 `seed=8072` 运行 Livox 候选，再用基线成功的 `seed=1612` 验证闭环不回归。两轮沿用 M5 完全相同的两层、33 s/层、180 s 总预算和离线真值协议。

淘汰条件：任一成功基线变为失败；或 RMSE、P95、最大/最终误差任一恶化超过 5%；或出现新的定位健康、安全、覆盖率或进程清理问题。淘汰后保持默认 `rgbd` 并回退候选实现，不继续扫参。

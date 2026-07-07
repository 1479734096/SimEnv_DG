# danger_search

`danger_search` 是比赛仿真环境中的自主搜索算法包，面向 `ROS1 Noetic + Gazebo Classic + Unitree A1`。

## 目标

- 使用公开 ROS 接口完成自主探索、危险源红球识别、返航和结果输出。
- 输出 `results/detected_danger.json`，格式遵循 `docs/evaluation.md`。
- 运行时不读取 `results/danger_truth.json`，不修改随机场景生成逻辑。

## 主要节点

```text
danger_search_node
```

该节点内部由 `rospy.Timer` 推进状态机，订阅里程计、点云和 RGB-D 图像，发布 `/cmd_vel`。
节点同时发布 `/danger_search/status`，用于查看当前状态、楼层、已确认目标数量和是否正在跟踪路径。

## 启动方式

在比赛环境启动、`junior_ctrl` 已进入 RL 模式后运行：

```bash
source ./devel/setup.bash
roslaunch danger_search danger_search.launch
```

如果只调试单层闭环，保持默认 `max_floor=1`。需要启用多楼层电梯流程时：

```bash
roslaunch danger_search danger_search.launch max_floor:=3
```

短超时自检可使用：

```bash
roslaunch danger_search danger_search_smoke.launch
```

自检会把结果写到 `/tmp/danger_search_smoke_detected.json`，用于确认节点能启动、超时退出并写出合法结果文件。

## 关键约束

- 检测结果使用多帧确认后才输出，优先降低虚警率。
- 默认使用 `/livox/Pointcloud2` 做 2.5D 栅格建图。
- 默认使用 `/real_sense/rgb/image_raw` 和 `/real_sense/depth/image_raw` 做红球识别。
- 输出坐标使用 Gazebo `world/odom` 数值坐标。
- RGB-D 反投影默认按 OpenCV 光学坐标计算，再转换到 ROS link 坐标后使用 TF。若实际相机 TF 已经是光学坐标，可把 `detection/depth_frame_is_optical` 设为 `false`。

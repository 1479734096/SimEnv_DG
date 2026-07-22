# 第 0 层官方底座验证报告

## 结论

第 0 层完成了官方源码清洁构建和一次固定 seed 冒烟。当时使用官方 `auto.sh` 已有的 `PAUSED=true` 和 6 秒自动恢复物理参数后，默认比赛传感器、底盘控制、站立和 RL `/cmd_vel` 模式同时运行成功。

官方文件没有改动。后续正式 runner 必须保留“暂停加载、延迟恢复物理”的启动约束，不能使用已确认会触发 ODE 段错误的 `PAUSED=false` 全传感器组合。

> 后续复测更正（第 2 层）：相同暂停启动策略再次运行时，四个 calf 全部出现 `+3.586 rad` 和 `-33.5 Nm` 饱和力矩，说明第 0 层的一次成功不足以证明 ros_control 启动具有确定性。该竞争已登记为 `ENV-002`，第 0 层报告以下内容只保留当次测试事实，不再作为“问题已解决”的结论。

## 基线来源

- 原始归档：`SimEnv初始.zip`
- 归档 SHA-256：`168c61d25848bf4efe43f9d14083169cfa9b97e433a5f3699e880b499ac4c308`
- 官方源码文件数：1109
- 排除内容：`build/`、`devel/`、`logs/`、`results/`、生成场景和 Python 缓存
- 官方文件校验：运行前、运行后执行 `sha256sum -c OFFICIAL_FILES.sha256`，全部通过

## 已执行验证

### 1. 清洁构建

命令：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

结果：28 个官方 Catkin 包构建成功。构建中存在官方工程原有的 VTK、PCL、Catkin 和 C++17 `register` 警告，但没有编译错误。

### 2. 非暂停全传感器运行

条件：单层、`SEED=42`、`GUI=false`、`PAUSED=false`，官方默认传感器全部启用；使用独立的 ROS/Gazebo 端口，避免默认 ROS master 的陈旧状态污染。

关键现象：

```text
load csv file name:.../scan_mode/mid360.csv
cannot convert str:
data size:800000
Segmentation fault (core dumped)
```

内核记录的实际崩溃点为：

```text
gzserver[166210]: segfault ... in libgazebo_ode.so.11.15.1
```

之后出现的 `SetModelConfiguration service failed` 是 `gzserver` 已退出后的下游错误，不是本次最早故障。

CSV 文件本身包含 800000 条数据和末尾换行。官方 `CsvReader` 使用 `while (!eof())`，因此会额外尝试解析一个空字符串并打印 `cannot convert str:`；该日志与崩溃相邻，但当前证据不足以认定它就是 ODE 段错误根因。

### 3. 关闭比赛传感器的对照运行

附加条件：`ENABLE_SENSOR_DATA=0`，其他底盘、Gazebo、控制器和官方源码保持不变。

结果：

- `/clock` 持续推进；
- `/a1_gazebo/joint_states` 稳定为 500 Hz；
- `/trunk_imu` 约为 410～500 Hz；
- 键盘 `2` 成功执行 `passive → fixed stand`；
- 站立 10 秒前后 IMU 四元数稳定，横滚和俯仰分量接近 0；
- 键盘 `6` 成功执行 `fixed stand → RL` 并进入 `/cmd_vel` 模式；
- RL 模式继续运行 10 秒，仿真时钟与姿态保持有效。

这说明干净官方底座的底盘、关节控制器和 `junior_ctrl` 可以在 `PAUSED=false` 下工作，非暂停运行本身不是充分触发条件；崩溃需要全传感器加载同时参与。

### 4. 暂停加载后恢复物理的全传感器运行

条件：保持官方源码和默认传感器不变，仅使用 `auto.sh` 已公开支持的参数：

```bash
PAUSED=true AUTO_UNPAUSE=1 AUTO_UNPAUSE_DELAY=6 ./auto.sh
```

结果：

- Mid-360 CSV 完成加载，`gzserver` 未崩溃；
- `spawn_model` 的延迟 `set_model_configuration` 在仿真仍暂停时成功完成；
- 恢复物理后 `/scan` 和 `/livox/Pointcloud2` 稳定为约 10 Hz；
- RealSense RGB 稳定为约 20 Hz，深度点云持续发布；
- `/trunk_imu` 持续发布，仿真时钟正常推进；
- 键盘 `2` 成功进入 FixedStand，四个 calf 收敛到约 `-1.83 rad`；
- 站立 10 秒后姿态稳定；
- 键盘 `6` 成功进入 RL `/cmd_vel` 模式，继续运行 10 秒后姿态和全部传感器仍有效。

该结果只说明暂停加载规避了当次 Gazebo/ODE 崩溃，并让该次底盘启动成功；第 2 层重复测试证明它没有消除独立的 ros_control 关节初始化竞争。

`fix.md` 第 280～320 行所述 Kimi 方案包含两个动作：暂停启动，以及在恢复物理前调用 Gazebo 服务重写 calf。干净 ZIP 的实测表明暂停启动确实必要，但额外重写 calf 并非当前底座站立成功的必要条件，因此不迁移修改版 `auto.sh`，也不从参赛方 runner 调用 Gazebo 模型配置服务。

## 环境隔离

默认 11311 端口存在一个旧 `rosmaster`，其中只剩 `/gazebo`、`/gazebo_gui` 等陈旧注册。本次测试未终止该外部进程，改用独立端口完成。测试结束后只清理本次运行创建的进程。

## 下一步边界

第 0 层已满足进入纯参赛代码分层迁移的条件，但不再宣称官方底盘启动确定性通过。后续测试仍固定使用官方参数 `PAUSED=true`，正式启动闭包须在第 8 层解决 `ENV-002`；不得迁移原工作区对 `auto.sh`、Unitree 控制器、Livox 插件或其他官方保护文件的修改。

如果未来在相同参数下重新出现 ODE 崩溃，应先作为环境回归处理并停止参赛算法迁移，不得通过修改官方插件掩盖失败。

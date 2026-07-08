传感器数据
↓
SLAM：我在哪？地图长什么样？
↓
任务决策：我要去哪？
├─ Frontier-Based Exploration：探索未知地图
└─ Coverage Path Planning：覆盖整个区域
↓
全局路径规划：从当前位置到目标点大概要怎么走？
├─ Dijkstra
├─ A\*
├─ D\*
├─ Theta\*
├─ RRT / RRT\*
└─ PRM
↓
局部轨迹规划 / 避障 / 控制：眼前怎么安全运动？
├─ DWA
├─ TEB
├─ MPC
└─ Artificial Potential Field
↓
输出速度 v, w 或轨迹


建图定位：SLAM

地图表示：2.5D elevation map

地形判断：traversability map

探索决策：Frontier-Based Exploration

全局路径：D\* Lite + A\*\\Theta\*，代价应该改成：

cost =

距离代价

\+ 坡度代价

\+ 台阶高度代价

\+ 粗糙度代价

\+ 落脚风险

\+ 动态障碍风险



局部运动：TEB / MPC

落脚：Footstep Planning

---

Plan 0：建立可复现基线
目标：固定 seed 和场景规模，拿到一份可信的失败证据。
范围：只运行和记录，不改代码。
执行：
SEED=42 FLOOR_COUNT=1 DANGER_COUNT=1 DISTRACTOR_COUNT=0 ./auto.sh
roslaunch danger_search danger_search.launch max_floor:=1
验收：
有 mission_runtime_summary.json
有 /cmd_vel、/danger_search/status、/Odometry_gazebo 记录
明确失败类型：启动超时、探索超时、返航失败、检测为空中的哪一种
Plan 1：启动稳定性
目标：定位 /Odometry_gazebo 偶发超时和仿真启动超时。
范围：只查启动链路，不碰导航和识别。
优先检查：
rostopic echo -n 1 /Odometry_gazebo
rostopic hz /Odometry_gazebo
rosnode list | grep state_from_gazebo
同时看：
logs/competition_gazebo.log
logs/junior_ctrl.log
auto.sh 中等待 Gazebo、控制器、odom 的逻辑
验收：
连续 3 次单层启动都能拿到 /Odometry_gazebo
/Odometry_gazebo 频率稳定，不再随机卡在等待阶段
若仍偶发失败，能明确失败在哪个节点或哪个等待条件
Plan 2：打开最小导航控制能力
目标：让机器人真的能转向、横移，并跟随路径离开起点。
范围：只调 controller 参数，先不调红球识别。
候选修改：
controller:
  max_linear: 0.30
  max_lateral: 0.15
  max_angular: 0.35
  min_linear_fraction: 0.35
验证：
rostopic echo /cmd_vel
rostopic echo /danger_search/status
验收：
/cmd_vel.linear.x 有非零值
/cmd_vel.linear.y 不再一直是 0
/cmd_vel.angular.z 不再一直是 0
odom 显示机器人实际移动，max_home_distance 至少超过 6m，理想超过 10m
Plan 3：单层探索闭环
目标：固定单层场景，确认机器人能进走廊、进第一个房间、绕障、返航。
范围：只处理探索、建图、前沿选择、返航，不处理检测准确率。
运行：
SEED=42 FLOOR_COUNT=1 DANGER_COUNT=1 DISTRACTOR_COUNT=0 ./auto.sh
roslaunch danger_search danger_search.launch max_floor:=1
验收：
不再只在起点附近抖动
max_home_distance 达标
return_home_success=true
没有探索超时
planner_debug.json 能说明选点、路径、失败恢复是正常推进的
Plan 4：传感器链路确认
目标：确认红球识别依赖的数据都正常。
范围：只查话题、频率、相机标定和 TF，不改检测算法。
检查：
rostopic hz /Odometry_gazebo
rostopic hz /livox/Pointcloud2
rostopic hz /real_sense/rgb/image_raw
rostopic hz /real_sense/depth/image_raw
rostopic echo -n 1 /real_sense/rgb/camera_info
验收：
RGB、Depth、CameraInfo 都有数据
Livox 点云正常
odom 正常变化
如果检测使用 TF，确认相机帧到世界/odom 的转换可用
Plan 5：危险源识别最小闭环
目标：在机器人确实进入房间后，至少检测到 1 个红球。
范围：只调 detection 和 tracker，不再同时改导航。
观察：
rostopic echo /danger_search/status
cat results/detected_danger.json
验收：
track_count > 0
confirmed_count > 0
detected_danger_sources 至少 1 个
没有明显把绿色球或红色方块误判为危险源
Plan 6：从单层扩展到三层
目标：在单层闭环稳定后，验证 3 层、电梯、返航完整流程。
范围：只处理多楼层状态机和电梯流程，不再大幅改控制参数。
运行：
SEED=42 FLOOR_COUNT=3 DANGER_COUNT=6 DISTRACTOR_COUNT=0 ./auto.sh
roslaunch danger_search danger_search.launch max_floor:=3
验收：
max_floor=3 生效
elevator_success=true
每层都有有效探索距离
最终能写出 detected_danger.json
不因第一层超时导致后续楼层完全没跑
Plan 7：参数固化和批量评测
目标：把能稳定工作的参数固化，再用多个 seed 验证不是偶然成功。
范围：只做小步参数归档、评测和文档同步。
执行：
python3 src/danger_search/tools/run_seed.py --seed 42 --timeout 620 --max-floor 3
python3 src/danger_search/scripts/batch_evaluate.py --seeds 10 42 100 --timeout 620 --max-floor 3
验收：
多 seed 不再长期停留在 15 分
漏检率下降
返航失败率下降
若修改了启动方式、参数含义或调试流程，同步更新 README 或 docs/quick-start.md
建议顺序是：Plan 0 -> Plan 1 -> Plan 2 -> Plan 3 -> Plan 4 -> Plan 5 -> Plan 6 -> Plan 7。当前最关键的是 Plan 2 和 Plan 3，红球识别等机器人能进房间后再调。

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


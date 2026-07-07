# 四足机器人危险源自主搜索与识别：论文学习笔记与实现路线

> 目标：把参考论文整理成“我应该做什么、为什么做、怎么做”的工程学习笔记。  
> 适用场景：ROS1 Noetic + Gazebo + Unitree A1 四足机器人仿真，任务为未知楼栋自主探索、危险源识别定位、返航。

---

## 0. 赛题目标先拆开

比赛不是单纯“机器人从 A 点走到 B 点”，而是一个完整自主系统：

```text
未知楼栋环境
  ↓
机器人依靠激光雷达 / 相机 / 深度点云感知
  ↓
自主探索全部可通行区域
  ↓
同步识别并定位危险源：红色球体
  ↓
记录危险源相对三维坐标
  ↓
探索完成后返回出发点
```

因此，你要做的不是一个单独算法，而是一条系统链路：

```text
SLAM / 建图定位
  ↓
Frontier 前沿探索
  ↓
全局路径规划 A* / Dijkstra
  ↓
局部避障 DWA / TEB
  ↓
危险源候选点观察与定位
  ↓
探索结束后返航
```

---

## 1. 总体参考论文地图

| 顺序 | 论文 / 方法 | 主要学习内容 | 对本题作用 |
|---|---|---|---|
| 1 | Yamauchi 1997 Frontier-Based Exploration | 什么是 frontier，如何自主探索未知环境 | 核心 |
| 2 | Topiwala 2018 WFD | ROS/Gazebo 中如何高效检测 frontier | 核心 |
| 3 | Cartographer 2016 | 2D LiDAR SLAM，建图与定位 | 核心 |
| 4 | RTAB-Map 2013 | RGB-D / 视觉回环与长期建图 | 可选 |
| 5 | LOAM 2014 | 3D LiDAR 里程计和建图 | 可选 / 进阶 |
| 6 | A* 1968 | 栅格地图全局路径规划 | 核心 |
| 7 | DWA 1997 | 局部避障与速度输出 | 核心 |
| 8 | RRT-Connect 2000 | 复杂区域备用路径搜索 | 可选 |
| 9 | RRT* 2011 | 采样式路径优化理论 | 背景 |
| 10 | Informed RRT* 2014 | 目标偏置采样与优化区域收缩 | 背景 / 创新启发 |
| 11 | Fast Frontier-based Information-driven Exploration 2020 | 信息增益 + 路径代价的探索目标选择 | 重要创新点 |
| 12 | Semantic Exploration 2025 | 探索与语义目标识别结合 | 高分创新点 |

你的优先级应为：

```text
必须先做：Frontier / WFD + SLAM + A* + DWA
之后增强：信息增益 frontier + 语义观察点
暂时背景：RRT* / Informed-RRT* / CGT / CBS / 多机器人协同
```

---

# 第一部分：自主探索

---

## 2. Yamauchi 1997：Frontier-Based Exploration

### 2.1 这篇论文解决什么问题？

它解决的是：

```text
地图还没全知道，机器人下一步应该去哪？
```

核心概念是 **frontier，前沿点**：

> frontier 是已知自由区域和未知区域之间的边界。

在占据栅格地图里可以理解成：

```text
? ? ? ? ? ? ?
? . . F ? ? ?
? . R . F ? ?
? . . . ? ? ?
? ? ? ? ? ? ?

? = 未知区域 unknown
. = 已知可通行区域 free
R = 机器人
F = frontier 前沿点
```

机器人不断去 frontier，就能把地图从已知区域向未知区域扩展，直到没有可达 frontier。

---

### 2.2 为什么适合你的比赛？

比赛要求“未知环境自主探索”，不是“已知目标点导航”。

所以核心问题不是：

```text
从当前位置到指定终点怎么走？
```

而是：

```text
还没探索完时，下一个目标点应该选哪里？
```

这正是 Frontier-Based Exploration 的作用。

---

### 2.3 你应该做什么？

你应该实现一个 `frontier_explorer_node`，循环执行：

```text
读取 /map
读取机器人当前位置 /tf 或 /odom
找出 frontier cells
对 frontier cells 聚类
给每个 frontier cluster 计算评分
选择最优 frontier
发送 goal 给 move_base
```

最小版本评分函数：

```text
score = frontier_cluster_size - λ * distance_to_robot
```

含义：

- `frontier_cluster_size` 越大，可能探索到的新区域越多；
- `distance_to_robot` 越大，过去越费时间；
- `λ` 是距离惩罚系数。

---

### 2.4 怎么判断一个格子是不是 frontier？

OccupancyGrid 常见编码：

```text
-1  = unknown 未知
0   = free 可通行
100 = occupied 障碍
```

判断逻辑：

```text
如果一个 cell 是 free，且它周围 8 邻域里存在 unknown cell，
那么它就是 frontier cell。
```

伪代码：

```python
for cell in map:
    if cell.value == FREE:
        if any(neighbor.value == UNKNOWN for neighbor in neighbors8(cell)):
            mark_as_frontier(cell)
```

---

## 3. Topiwala 2018：WFD / Wavefront Frontier Detector

### 3.1 它解决什么问题？

Yamauchi 告诉你 frontier 是什么。Topiwala 这类工作重点在于：

```text
如何在 ROS/Gazebo 中高效找到 frontier？
```

其中常用方法是 **WFD，Wavefront Frontier Detector**。

---

### 3.2 WFD 和暴力扫描有什么区别？

暴力扫描：

```text
遍历整张地图 → 找所有 frontier
```

缺点：地图变大后慢，而且可能把不可达区域也算进去。

WFD：

```text
从机器人当前位置开始 BFS
只搜索机器人当前可达的 free 区域
在可达区域边界寻找 frontier
```

---

### 3.3 WFD 的基本流程

```text
1. 从机器人所在 grid 开始 BFS
2. 只扩展 free cell
3. 如果发现 frontier cell：
      对该 frontier 做一次 BFS 聚类
      得到 frontier cluster
4. 对每个 cluster 计算中心点、面积、距离
5. 选择最优 cluster 作为导航目标
```

需要两个 BFS：

### BFS 1：搜索可达自由空间

```text
queue = [robot_cell]

while queue not empty:
    cell = pop(queue)
    if cell is frontier:
        frontier_cluster = BFS_frontier(cell)
        save cluster
    else:
        push free neighbors
```

### BFS 2：frontier 聚类

```text
从一个 frontier cell 开始
把相邻 frontier cell 全部找出来
形成 frontier cluster
```

---

### 3.4 你应该怎么落地？

第一版：可以全图扫描，先让系统跑起来。  
第二版：升级成 WFD，提升效率和可解释性。

ROS 输入输出：

```text
输入：
/map                nav_msgs/OccupancyGrid
/tf 或 /odom         机器人位姿

输出：
/move_base_simple/goal
或 actionlib goal
```

---

# 第二部分：SLAM / 建图定位

---

## 4. Cartographer 2016：Real-Time Loop Closure in 2D LIDAR SLAM

### 4.1 它解决什么问题？

SLAM 同时解决两个问题：

```text
我在哪？
地图长什么样？
```

Cartographer 是经典的 2D LiDAR SLAM 系统，核心包括：

```text
scan-to-submap matching
实时回环检测
图优化
```

---

### 4.2 为什么探索必须先有 SLAM？

你的系统依赖链是：

```text
SLAM 不稳
  ↓
地图不准
  ↓
frontier 错
  ↓
路径规划错
  ↓
危险源坐标错
  ↓
返航失败
```

所以 SLAM 是整个导航系统地基。

---

### 4.3 你应该做什么？

新手阶段不要自己写 SLAM。应该使用现有包。

你的仿真环境有点云和图像，因此有两条路线：

### 路线 A：点云降维成 2D 导航地图

```text
3D 点云
  ↓
过滤高度范围，例如 z ∈ [0.1, 1.2]
  ↓
投影到 2D
  ↓
生成 /scan 或 /costmap
  ↓
Cartographer 2D / gmapping / hector
  ↓
move_base
```

优点：适合新手，和 ROS1 `move_base` 配合简单。

### 路线 B：直接做 3D SLAM

```text
Livox 点云
  ↓
3D LiDAR SLAM
  ↓
3D map / occupancy map
  ↓
再切片成 2D 导航地图
```

优点：适合复杂三维环境。  
缺点：工程难度更高。

第一版建议选路线 A。

---

### 4.4 怎么判断 SLAM 成功？

检查话题：

```text
/map
/tf
/odom
/scan 或点云
```

成功标准：

```text
1. RViz 中机器人位置和仿真运动一致
2. 墙体轮廓稳定
3. 走一圈后地图没有明显错位
4. /map 能被 move_base 使用
```

---

## 5. RTAB-Map 2013：Appearance-Based Loop Closure

### 5.1 它解决什么问题？

RTAB-Map 主要面向视觉 / RGB-D SLAM。

核心问题是：

```text
机器人现在看到的地方，是不是以前来过？
```

这叫 **回环检测**。

---

### 5.2 和你的比赛有什么关系？

你的比赛有 RGB 图像和深度点云，危险源又是视觉目标。RTAB-Map 可以作为：

```text
RGB-D 建图
视觉辅助回环
语义地图
危险源语义标注
```

但它不是第一版必须做的东西。

---

### 5.3 你应该怎么处理？

第一版不建议押宝 RTAB-Map。先做：

```text
LiDAR / 点云建图 + 2D 导航
```

识别部分独立做：

```text
RGB 图像找红色球体
  ↓
深度图 / 点云取三维坐标
  ↓
tf 转换到 map 坐标系
```

后期如果有时间，再考虑 RTAB-Map 做视觉语义地图。

---

## 6. LOAM 2014：Lidar Odometry and Mapping in Real-time

### 6.1 它解决什么问题？

LOAM 是经典 3D LiDAR SLAM。

它处理的是：

```text
用 3D 激光雷达点云估计机器人 6 自由度运动
并同时构建 3D 点云地图
```

---

### 6.2 和 Cartographer 的区别

| 项目 | Cartographer 2D | LOAM |
|---|---|---|
| 输入 | 2D LiDAR / 2D scan | 3D LiDAR 点云 |
| 地图 | 2D occupancy grid | 3D 点云地图 |
| 适合 | 室内平面导航 | 复杂三维空间 |
| 新手难度 | 较低 | 较高 |
| 和 move_base 配合 | 容易 | 需要转 2D costmap |

---

### 6.3 你应该怎么用？

建议：

```text
第一阶段：不要自己写 LOAM
第二阶段：如果 2D SLAM 不稳定，再考虑 3D LiDAR SLAM
第三阶段：把 3D 地图切片生成 2D 导航层
```

把 LOAM 当作备选方案和背景知识，不要一开始就实现。

---

# 第三部分：全局路径规划

---

## 7. Hart et al. 1968：A* 算法

### 7.1 它解决什么问题？

A* 回答：

```text
已知地图
已知当前位置
已知目标点
怎么找到一条代价尽量小的路径？
```

在你的系统里：

```text
frontier_explorer 选目标点
  ↓
A* 规划去目标点的路径
  ↓
局部规划器执行路径
```

---

### 7.2 A* 的核心公式

```text
f(n) = g(n) + h(n)
```

含义：

```text
g(n) = 从起点到 n 已经走了多少代价
h(n) = 从 n 到目标点估计还要多少代价
f(n) = 总估计代价
```

如果 `h(n) = 0`，A* 就退化成 Dijkstra。

---

### 7.3 你应该做什么？

新手不需要自己从零写 A*。ROS1 里可以使用：

```text
move_base + navfn
move_base + global_planner
```

报告中可以写：

```text
全局规划采用基于占据栅格地图的 A*/Dijkstra 搜索，
根据障碍膨胀后的代价地图生成从机器人当前位置到目标 frontier 的全局路径。
```

---

### 7.4 真正要调的是 costmap

你要重点调这些参数：

```text
robot_radius / footprint
inflation_radius
cost_scaling_factor
map_resolution
obstacle_range
raytrace_range
```

四足机器人不要让路径贴墙。建议 `inflation_radius` 稍微保守一些。

---

## 8. RRT-Connect 2000

### 8.1 它解决什么问题？

RRT-Connect 是采样式路径规划算法。

它从起点和终点各长一棵树：

```text
起点树  →        ←  目标树
```

两棵树如果成功连接，就得到一条路径。

---

### 8.2 和 A* 的区别

| 项目 | A* | RRT-Connect |
|---|---|---|
| 地图形式 | 栅格图 | 连续空间 / 配置空间 |
| 搜索方式 | 系统性搜索 | 随机采样 |
| 优点 | 稳定、可解释 | 复杂空间找路快 |
| 缺点 | 依赖栅格分辨率 | 路径不平滑、随机性强 |
| 是否适合作主线 | 适合 | 不适合 |

---

### 8.3 为什么不建议你主用？

RRT-Connect 只解决：

```text
给定 A 点、B 点、障碍物，找一条路
```

它不解决：

```text
地图怎么建？
哪里还没探索？
危险源怎么识别？
什么时候返航？
```

因此它不适合作为完整自主探索主方案。

---

### 8.4 你可以怎么用？

作为备用规划器：

```text
A* 找不到去某个 frontier 的路径
  ↓
调用 RRT-Connect 在局部地图上尝试连接
  ↓
如果成功，把路径转成 waypoints
  ↓
交给局部规划器跟踪
```

报告中可以写：

```text
针对窄通道、门口附近代价地图膨胀导致的全局规划失败问题，
引入采样式备用规划器 RRT-Connect，以提高复杂区域可达性。
```

第一版先不做，等主线跑通再加。

---

## 9. Karaman & Frazzoli 2011：RRT* / PRM*

### 9.1 它解决什么问题？

RRT* 和 PRM* 主要解决采样式路径规划的路径质量问题。

核心概念：

```text
概率完备：如果路径存在，采样次数足够多，找到路径的概率趋近于 1。
渐近最优：采样次数越来越多，路径代价逐渐趋近最优路径。
```

RRT 通常偏“找到可行路径”。  
RRT* 追求“路径越来越优”。

---

### 9.2 和你的比赛有什么关系？

它适合写相关工作，不适合第一版实现。

你的第一优先级是：

```text
稳定完成探索
不漏危险源
能返航
```

不是追求单条路径渐近最优。

---

### 9.3 可以怎么用？

后期可以用于：

```text
探索完成后优化返航路径
复杂局部区域对 RRT-Connect 路径做重连优化
```

但不建议在基础版本中实现。

---

## 10. Gammell et al. 2014：Informed RRT*

### 10.1 它解决什么问题？

Informed RRT* 是 RRT* 的加速版本。

普通 RRT*：

```text
全地图继续随机采样优化路径
```

Informed RRT*：

```text
已有一条路径后，只在可能改进路径的椭圆区域内采样
```

---

### 10.2 对你的启发是什么？

不要把“采样”理解成完全乱试。它给你的启发是：

```text
搜索应该有目标偏置。
```

迁移到本题就是：

```text
frontier 不应该随便选
应该优先选信息量大、路径短、危险源可能性高、风险低的位置
```

---

### 10.3 可以形成你的探索评分函数

```text
score(frontier) =
    α * 信息增益
  - β * 路径长度
  + γ * 危险源疑似收益
  - δ * 风险代价
```

其中：

```text
信息增益：frontier 附近 unknown cell 数量
路径长度：A* 到 frontier 的路径长度
危险源疑似收益：相机检测到红色区域的方向优先级
风险代价：离障碍太近扣分
```

这比直接实现 Informed RRT* 更适合你的题目。

---

# 第四部分：局部避障与速度控制

---

## 11. Fox, Burgard, Thrun 1997：DWA

### 11.1 它解决什么问题？

DWA 解决的是：

```text
机器人未来 1~3 秒应该发什么速度，才能不撞障碍并朝目标前进？
```

它不是全局路径规划器。它是局部规划 / 局部避障器。

---

### 11.2 DWA 和 A* 的区别

| 项目 | A* | DWA |
|---|---|---|
| 层级 | 全局规划 | 局部规划 |
| 输入 | 地图、起点、目标点 | 局部障碍、当前速度、全局路径 |
| 输出 | 一串路径点 | `/cmd_vel` 速度 |
| 时间尺度 | 整条路线 | 未来几秒 |
| 作用 | 走哪条路 | 眼前怎么动 |

---

### 11.3 DWA 的核心思想

DWA 在速度空间采样：

```text
v = 线速度
w = 角速度
```

例如：

```text
(v=0.1, w=0.0)
(v=0.2, w=0.2)
(v=0.2, w=-0.2)
```

对每组速度模拟未来轨迹，再评分：

```text
会不会撞障碍？
离全局路径近不近？
离目标近不近？
速度是否够快？
```

最后选择评分最高的一组速度发布到 `/cmd_vel`。

---

### 11.4 你应该做什么？

第一版直接使用 ROS1 的：

```text
dwa_local_planner
```

重点参数：

```text
max_vel_x
min_vel_x
max_vel_theta
acc_lim_x
acc_lim_theta
sim_time
path_distance_bias
goal_distance_bias
occdist_scale
xy_goal_tolerance
yaw_goal_tolerance
```

四足机器人建议初期保守：

```text
max_vel_x: 0.2 ~ 0.4
max_vel_theta: 0.5 ~ 0.8
sim_time: 1.5 ~ 2.5
inflation_radius: 偏大一点
```

原因：比赛更看稳定完成任务，不是竞速。

---

## 12. TEB：Timed Elastic Band

### 12.1 它解决什么问题？

TEB 也是局部规划器，但比 DWA 更偏优化。

它把局部轨迹看成一条带时间信息的“弹性带”：

```text
路径点 + 时间间隔
```

优化目标包括：

```text
离障碍远
接近全局路径
满足速度 / 加速度约束
轨迹平滑
尽量快到目标
```

---

### 12.2 DWA 和 TEB 怎么选？

| 项目 | DWA | TEB |
|---|---|---|
| 难度 | 较低 | 较高 |
| 参数调试 | 相对简单 | 参数多 |
| 路径平滑 | 一般 | 较好 |
| 速度约束 | 基础 | 更强 |
| 新手第一版 | 推荐 | 可后期尝试 |

第一版建议 DWA，后期再试 TEB。

---

# 第五部分：信息增益探索与语义探索

---

## 13. Dai et al. 2020：Fast Frontier-based Information-driven Exploration

### 13.1 它解决什么问题？

普通 frontier 方法可能只选最近 frontier。

问题是：

```text
最近的 frontier 不一定最有价值。
```

信息驱动探索会考虑：

```text
能看到多少未知区域？
过去要花多少时间？
能否减少地图不确定性？
```

---

### 13.2 和你比赛有什么关系？

比赛既看探索时间，也看危险源识别定位。

所以你不应该只追求：

```text
去最近的 frontier
```

而应该追求：

```text
用尽量短时间探索更多未知区域，同时提高发现危险源的机会。
```

---

### 13.3 推荐评分函数

```text
score = α * information_gain
      - β * path_cost
      + γ * detection_gain
      - δ * risk_cost
```

解释：

### information_gain

frontier 附近未知格子数量：

```text
information_gain = count_unknown_cells_around(frontier, radius)
```

### path_cost

从当前位置到 frontier 的 A* 路径长度：

```text
path_cost = length(A_star_path)
```

### detection_gain

如果识别模块发现某方向有红色目标候选，但还没定位准，则提高该方向 frontier 优先级：

```text
detection_gain = 1 / distance_to_suspected_red_object
```

### risk_cost

离墙、门框、障碍物太近扣分：

```text
risk_cost = 1 / distance_to_nearest_obstacle
```

---

### 13.4 实现阶段

第一版：

```text
score = frontier_size - 0.5 * distance
```

第二版：

```text
score = unknown_count_near_frontier - λ * Astar_path_length
```

第三版：

```text
score = α * unknown_count
      - β * path_length
      + γ * red_candidate_score
      - δ * obstacle_risk
```

第三版就是你报告里的主要创新点之一。

---

## 14. Semantic Exploration 2025：语义探索与稠密建图

### 14.1 它解决什么问题？

普通探索只关心：

```text
地图有没有探索完？
```

语义探索还关心：

```text
目标有没有被看见？
目标有没有被看清？
目标位置是否定位准确？
是否需要换角度观察？
```

---

### 14.2 和你比赛有什么关系？

你的危险源是红色球体，但环境里可能有干扰物，例如：

```text
红色方块
绿色球体
其他类似颜色物体
```

如果系统只看到红色就报警，会造成虚警率高。

因此你需要：

```text
发现疑似目标
  ↓
靠近或换角度观察
  ↓
确认是红色球体而不是红色方块
  ↓
再记录坐标
```

---

### 14.3 你的状态机可以这样设计

```text
EXPLORE:
    找 frontier，继续探索

DETECT_CANDIDATE:
    相机发现红色区域，生成候选目标

INSPECT:
    移动到更好观察点，确认目标类别和三维位置

RECORD:
    如果确认是红色球体，记录 map 坐标

RETURN:
    没有 frontier 后返回起点
```

---

### 14.4 语义确认怎么做？

针对仿真红色球体，第一版可以不用 YOLO，先做传统视觉：

```text
RGB 图像中提取红色区域
  ↓
判断形状：圆形 / 球体投影 vs 方形
  ↓
从深度点云取该区域 3D 点
  ↓
计算中心位置
  ↓
tf 转到 map 坐标系
  ↓
多帧融合，取平均 / 聚类
```

不要一开始就上复杂深度学习。因为任务物体定义简单，传统颜色 + 几何方法更稳定、更容易解释。

---

# 第六部分：最终系统方案

---

## 15. 推荐系统架构

```text
传感器层：
  Livox 点云 / 深度点云 / RGB 图像 / 里程计

感知建图层：
  点云滤波 → 2D scan / costmap → SLAM → /map + /tf

探索决策层：
  WFD frontier 检测
  information gain + path cost + detection gain 评分

路径规划层：
  A* / Dijkstra 全局路径
  RRT-Connect 作为可选备用规划器

局部避障层：
  DWA / TEB 输出 /cmd_vel

危险源识别定位层：
  红色区域检测
  球体 / 方块区分
  深度点云三维定位
  map 坐标系融合记录

任务状态机：
  EXPLORE → INSPECT → RECORD → RETURN
```

---

## 16. 推荐 ROS 节点划分

```text
/slam_node
  输入：/scan 或 /points, /odom, /tf
  输出：/map, map->odom tf

/frontier_explorer_node
  输入：/map, /tf, /danger_candidates
  输出：/move_base goal

/move_base
  输入：/map, costmap, goal
  输出：/cmd_vel

/danger_detector_node
  输入：/camera/rgb, /camera/depth 或 /points, /tf
  输出：/danger_candidates, /danger_results

/task_manager_node
  输入：frontier 状态、move_base 状态、危险源候选状态
  输出：任务状态切换、返航命令、最终结果文件
```

---

## 17. 任务状态机

```text
INIT
  ↓
EXPLORE
  ↓                ↘
DETECT_CANDIDATE   NO_FRONTIER
  ↓                 ↓
INSPECT             RETURN_HOME
  ↓                 ↓
RECORD              FINISH
  ↓
EXPLORE
```

每个状态的职责：

| 状态 | 作用 |
|---|---|
| INIT | 记录起点、初始化地图和参数 |
| EXPLORE | 选择 frontier，执行探索 |
| DETECT_CANDIDATE | 发现疑似危险源候选 |
| INSPECT | 生成观察点并靠近确认 |
| RECORD | 记录危险源坐标，去重融合 |
| RETURN_HOME | 回到出发点 |
| FINISH | 输出结果，结束实验 |

---

## 18. 探索结束判断

不要只用“没有 frontier”一个条件。推荐组合：

```text
连续 N 次没有可达 frontier
或
frontier 数量低于阈值
或
地图覆盖率增长低于阈值持续 T 秒
```

然后进入返航：

```text
home_pose = 初始 map 坐标
send_goal(home_pose)
```

---

## 19. 危险源定位流程

第一版建议：

```text
1. RGB 图像转 HSV
2. 提取红色 mask
3. 形态学开闭运算去噪
4. 找轮廓
5. 用圆度 / 面积 / 长宽比筛选球体候选
6. 在候选区域内取深度点云
7. 计算候选中心 3D 坐标
8. 通过 tf 转到 map 坐标系
9. 多帧聚类去重
10. 输出危险源坐标
```

圆度指标：

```text
circularity = 4π * area / perimeter^2
```

圆形接近 1，方形更低。

---

## 20. 第一版最小可行目标

不要一开始追求完整创新。第一版目标：

```text
1. Gazebo 中 A1 能接收 /cmd_vel
2. RViz 中能看到地图 /map
3. 手动点目标，move_base 能到达
4. frontier_explorer 能自动选点
5. 机器人能自主探索一层
6. 没有 frontier 后能返回起点
```

这一步完成后，再加危险源识别和语义观察。

---

## 21. 第二版增强目标

```text
1. frontier 评分加入 information_gain
2. 路径代价使用 A* 路径长度，而不是直线距离
3. 加入红色候选目标 detection_gain
4. 加入 obstacle risk，避免贴墙目标
5. 加入 INSPECT 语义确认状态
6. 多帧融合危险源坐标，降低虚警率
```

---

## 22. 第三版创新包装

可以在报告中写成：

```text
本文提出一种面向危险源搜索任务的语义引导前沿探索策略。
在传统 WFD 前沿点检测基础上，构建综合评价函数，
同时考虑未知区域信息增益、路径执行代价、障碍风险和危险源候选观测收益。
当视觉模块发现疑似危险源时，导航系统主动生成语义观察点，
引导机器人靠近并从更优视角确认目标类别和三维位置，
从而在保证环境覆盖效率的同时提高危险源识别定位精度并降低虚警率。
```

---

# 第七部分：按周学习与实现计划

---

## 第 1 周：ROS/Gazebo 基础接口

目标：

```text
跑通仿真
确认 /cmd_vel 控制有效
确认传感器话题
确认 /odom /tf
```

要做：

```bash
rostopic list
rostopic echo /cmd_vel
rostopic echo /odom
rviz
```

---

## 第 2 周：SLAM 与地图

目标：

```text
机器人运动时，RViz 中地图能正常增长
```

要做：

```text
点云转 2D scan
跑 Cartographer / gmapping / hector
检查 /map 和 /tf
```

---

## 第 3 周：move_base

目标：

```text
手动点目标，机器人能自动走过去
```

要做：

```text
配置 global_costmap
配置 local_costmap
配置 DWA
调 inflation_radius 和 velocity limits
```

---

## 第 4 周：Frontier Exploration

目标：

```text
机器人不用人点目标，能自动探索
```

要做：

```text
实现 frontier 检测
实现聚类
实现目标评分
发送 move_base goal
```

---

## 第 5 周：危险源检测定位

目标：

```text
看到红色球体后能输出 map 坐标
```

要做：

```text
HSV 红色分割
圆度筛选
深度点云定位
tf 坐标转换
多帧融合
```

---

## 第 6 周：语义观察和返航

目标：

```text
发现疑似危险源后主动靠近确认
探索结束后自动返航
```

要做：

```text
状态机 EXPLORE / INSPECT / RECORD / RETURN
观察点生成
结果输出格式
```

---

# 第八部分：参考文献初稿

```text
[1] B. Yamauchi, “A Frontier-Based Approach for Autonomous Exploration,” Proceedings of the IEEE International Symposium on Computational Intelligence in Robotics and Automation, 1997.

[2] A. Topiwala, P. Inani, and A. Kathpal, “Frontier Based Exploration for Autonomous Robot,” arXiv:1806.03581, 2018.

[3] W. Hess, D. Kohler, H. Rapp, and D. Andor, “Real-Time Loop Closure in 2D LIDAR SLAM,” IEEE International Conference on Robotics and Automation, 2016.

[4] M. Labbé and F. Michaud, “Appearance-Based Loop Closure Detection for Online Large-Scale and Long-Term Operation,” IEEE Transactions on Robotics, vol. 29, no. 3, pp. 734–745, 2013.

[5] J. Zhang and S. Singh, “LOAM: Lidar Odometry and Mapping in Real-time,” Robotics: Science and Systems, 2014.

[6] P. E. Hart, N. J. Nilsson, and B. Raphael, “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on Systems Science and Cybernetics, 1968.

[7] D. Fox, W. Burgard, and S. Thrun, “The Dynamic Window Approach to Collision Avoidance,” IEEE Robotics & Automation Magazine, 1997.

[8] J. J. Kuffner and S. M. LaValle, “RRT-Connect: An Efficient Approach to Single-Query Path Planning,” IEEE International Conference on Robotics and Automation, 2000.

[9] S. Karaman and E. Frazzoli, “Sampling-based Algorithms for Optimal Motion Planning,” The International Journal of Robotics Research, 2011.

[10] J. D. Gammell, S. S. Srinivasa, and T. D. Barfoot, “Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic,” IEEE/RSJ International Conference on Intelligent Robots and Systems, 2014.

[11] A. Dai, S. Papatheodorou, N. Funk, D. Tzoumanikas, and S. Leutenegger, “Fast Frontier-based Information-driven Autonomous Exploration with an MAV,” arXiv:2002.04440, 2020.

[12] X. Zhan et al., “Semantic Exploration and Dense Mapping of Complex Environments using Ground Robots Equipped with LiDAR and Panoramic Camera,” arXiv:2505.22880, 2025.
```

---

# 第九部分：一句话总结

你这题目的最稳路线是：

```text
SLAM 建图定位
+ WFD Frontier 自主探索
+ A*/Dijkstra 全局规划
+ DWA/TEB 局部避障
+ 颜色/几何/深度点云危险源定位
+ 语义观察点确认
+ 自动返航
```

不要一开始做复杂多机器人 CGT，也不要一开始写 RRT*。  
先把“能稳定自主探索并返航”跑通，再把危险源识别和语义探索接进去。

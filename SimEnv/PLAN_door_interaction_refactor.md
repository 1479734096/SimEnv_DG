0. 核心原则

本次重构目标不是“让机器人知道所有门在哪里”，而是：

只允许机器人基于局部传感器发现可交互对象；
配置文件只用于把已发现对象匹配到仿真服务 ID。

比赛要求机器人依靠自身传感器实时感知环境，并自主完成探索、危险源识别定位和返航，不应使用全局真值做导航目标。

1. 问题修正
1.1 Mapper 的能力边界

当前 Mapper 只负责导航建图：

free / occupied / unknown

也就是：

空地 / 障碍物 / 未知区域

它不能表达：

门
电梯
危险源
房间入口

注意：

不是传感器只能区分空地和障碍物，
而是当前 Mapper 把传感器信息压缩成了占据栅格。
1.2 禁止全局门透视

禁止逻辑：

读取 layout_metadata
-> 获得所有门坐标
-> 无 Frontier 时选最近门
-> 跨地图导航过去

这个必须删除。

2. 对象分类
2.1 动态服务门

包括：

main_entrance
elevator_floor_0
elevator_floor_1
elevator_floor_2

这类对象可以调用：

/set_door_state

但不能作为全局导航目标。

2.2 房间入口

房间入口不是动态门。

处理方式：

由 Frontier 探索自然进入
不调用 /set_door_state
不创建 OpenDoorAndPass
2.3 电梯

电梯不能当普通门处理。

电梯必须走独立任务：

TakeElevator

只有在需要换楼时才允许触发。

普通探索状态下，即使靠近电梯门，也不要直接执行电梯流程。

3. 模块职责
3.1 Mapper

职责：

输入：雷达 / 深度点云
输出：2D occupancy grid
用途：路径规划、Frontier 探索

不负责语义识别。

3.2 LocalSemanticDetector

新增或整理一个局部语义检测层。

职责：

1. 检测附近是否存在门状结构
2. 检测附近是否存在电梯入口
3. 检测危险源红球

第一版可以保持简单：

RGB / 深度 / 点云规则检测
不引入复杂模型
3.3 BuildingControlClient

职责降级为服务客户端：

1. 调用 /set_door_state
2. 调用电梯相关服务
3. 根据已发现对象匹配 door_id / elevator_id

禁止它做：

1. 提供全局门导航目标
2. 主动生成未访问门列表
3. 替代传感器发现门
4. 门检测策略
4.1 正式策略

使用：

Sensor-Gated Local Door Detector

触发条件：

1. 距离小于 3.0 m
2. 在传感器视野内
3. 局部图像 / 深度 / 点云中存在门状结构
4. 当前状态允许开门
5. 能匹配到可控动态门 ID

流程：

局部传感器检测到门候选
-> 根据候选位置匹配最近 dynamic door ID
-> 创建 OpenDoorAndPass(door_id)
4.2 Debug 策略

可以保留真值近距检测，但必须默认关闭：

USE_TRUTH_PROXIMITY_DETECTOR = False

仅调试时使用：

机器人距离 metadata 中某动态门 < 3.0 m
-> 临时认为检测到门

该逻辑不能作为正式方案主路径。

5. MissionManager 修改
5.1 删除旧逻辑

删除或禁用：

_nearest_unvisited_door
unvisited_doors 全局任务池
无 Frontier 时去最近门
启动时硬编码 OpenDoorAndPass("main_entrance")

同时删除：

if door_id == "main_entrance":
    ...

除非只是日志。

5.2 探索流程

新的 _tick_explore()：

1. 如果当前有任务，继续任务
2. 检查局部门候选
3. 如果发现可控动态门，执行 OpenDoorAndPass
4. 否则继续 Frontier 探索
5. 如果 Frontier 失败，进入探索失败 / 换楼 / 返航逻辑
6. 不允许用门坐标兜底导航


伪代码：

def _tick_explore(self):
    if self.current_task is not None:
        return

    door_id = self.local_detector.detect_controllable_door()

    if door_id is not None:
        self.current_task = OpenDoorAndPass(door_id)
        return

    self._run_frontier_exploration()

## 门检测与开门决策闭环

门不是全局已知目标，而是在探索过程中被局部感知发现。

LocalSemanticDetector.detect_controllable_door() 只在以下条件满足时返回 DoorCandidate：

1. 机器人处于局部传感器可观测范围内；
2. RGB / 深度 / 点云中存在门状候选；
3. 候选距离机器人小于 3.0 m；
4. 候选在传感器视野内，且无遮挡；
5. 候选能匹配到 BuildingControlClient 中的可控动态门 ID；
6. 该候选不是房间入口；
7. 普通探索状态下不返回 elevator_floor_i。

MissionManager 接收到 DoorCandidate 后，不立即无脑开门，而是调用 _should_open_door(candidate) 判断是否执行 OpenDoorAndPass。

_should_open_door(candidate) 的第一版规则：

1. 当前状态必须是 EXPLORE；
2. 当前没有 current_task；
3. candidate.kind 必须是 dynamic_door；
4. candidate.door_id 不能属于 opened_doors；
5. candidate.door_id 不能属于 failed_doors；
6. candidate.door_id 不能是 active_door；
7. candidate.door_id 不能是 elevator_floor_i；
8. candidate.distance < 3.0；
9. candidate.visible == True。

如果判断通过，则 MissionManager 取消当前 Frontier 目标，创建 OpenDoorAndPass(candidate.door_id)，开门通过后标记 opened_doors，并恢复 Frontier 探索。

电梯门不走该逻辑。电梯只在 CHANGE_FLOOR 状态下由 TakeElevator 独立状态机处理。

6. 大门处理
6.1 不再特殊硬编码

禁止：

OpenDoorAndPass("main_entrance")

大门只能通过以下方式触发：

局部传感器检测到入口门
-> 匹配到 main_entrance
-> 执行 OpenDoorAndPass
6.2 初始状态问题单独处理

如果仿真配置写了 initial_open=True，但 Gazebo 里门板物理上仍关闭，这是环境初始化一致性问题。

可以单独做：

ensure_initial_dynamic_doors_applied()

但不要把它混进自主探索逻辑。

7. 电梯处理

电梯只在换楼状态触发：

state == CHANGE_FLOOR

流程：

TakeElevator(target_floor)
-> 到达电梯口
-> 打开当前楼层电梯门
-> 进入轿厢
-> 切换楼层
-> 打开目标楼层电梯门
-> 离开轿厢
-> 更新 current_floor

普通探索中禁止：

靠近 elevator_floor_i
-> OpenDoorAndPass(elevator_floor_i)
8. Codex 修改任务
Task 1：搜索旧门逻辑
grep -RIn "unvisited_doors\|nearest_unvisited_door\|main_entrance\|layout_metadata\|OpenDoorAndPass\|TakeElevator" src/

目标：

找出所有门、电梯、metadata 相关逻辑。
Task 2：删除全局门导航

删除：

1. 所有门加入 unvisited_doors
2. 无 Frontier 时选最近门
3. 根据 metadata 门坐标远距离规划
4. main_entrance 启动硬编码
Task 3：整理 BuildingControlClient

保留：

/set_door_state 调用
door_id 查询
elevator_id 查询

禁止：

主动向 MissionManager 提供全局导航目标
Task 4：新增局部门检测接口

建议接口：

def detect_controllable_door(self) -> Optional[str]:
    ...

要求：

1. 只返回近距离、传感器可见的动态门
2. 默认不使用 truth proximity
3. 不处理房间入口
4. 不处理电梯换楼
Task 5：独立电梯流程

保持或重构：

TakeElevator

要求：

1. 只在 CHANGE_FLOOR 状态触发
2. 不走 OpenDoorAndPass
3. 不被普通门检测器触发
9. 验收标准
静态验收

必须满足：

1. 没有 _nearest_unvisited_door 兜底逻辑
2. 没有 main_entrance 启动硬编码
3. MissionManager 不用全局门坐标做导航目标
4. 房间入口不调用 /set_door_state
5. 电梯只由 TakeElevator 处理
6. truth proximity detector 默认关闭
运行验收

期望行为：

1. 机器人启动后正常 Frontier 探索
2. 看到/接近动态门后才开门
3. 房间入口自然探索进入，不调用开门服务
4. 无 Frontier 时不去全局最近门
5. 需要换楼时才执行电梯流程
6. 完成探索后返航

最终原则
导航靠 Frontier；
门靠局部传感器发现；
服务 ID 靠配置匹配；
房间入口靠自然探索；
电梯靠独立状态机；
禁止全局门坐标兜底导航。
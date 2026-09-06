# STAGE35 技术报告：转角墙皮擦边导致前向避障误判死锁修复

## 1. 问题现象
在 0 层走廊巡航与转角探索过程中，机器狗在 (X ≈ -5.05m, Y ≈ 6.92m, yaw ≈ 9.5°) 处停滞。
正前方完全开阔，A* 规划器与 navigator 下发正常路径，但避障链路强制线速度降为 0.0 m/s；
因角速度偏差极小（-0.027 rad/s）落入 junior_ctrl 步态死区，导致机器狗原地永久死锁。

## 2. 根本原因
1. 前向避障算法在 STAGE23 后仍保留了刚性矩形检测盒 (0.20m <= forward <= 1.5m 且 lateral <= 0.22m)；
2. 拐角凸出墙皮在点云中落在 (forward=0.2001m, lateral=0.2078m)，欧氏距离 0.2885m < obstacle_stop_distance (0.40m)；
3. A1 机器狗机身半宽 0.097m、肩宽极限 0.147m。在 forward=0.20m 处侧偏 0.2078m 的墙角实际距肩部外侧尚有 >6cm 净空，直行绝不碰撞，急停属刚性几何假阳性。

## 3. 修改方案与落地
改用贴合四足机器人本体轮廓的梯形收腰走廊 (Tapered Lateral Corridor)：
- 近身段 (forward <= 0.35m)：半宽收腰至 0.16m，彻底排除侧向擦边回波（0.2078m > 0.16m）；
- 过渡段 (0.35m < forward < 0.60m)：平滑线性过渡至 0.22m；
- 远端段 (forward >= 0.60m)：保持 0.22m 原有观察半宽，确保提前降速减震。

## 4. 修改文件
- SimEnv/src/danger_search/danger_search/map_update_runtime.py
- SimEnv/src/danger_search/danger_search/mission_manager.py

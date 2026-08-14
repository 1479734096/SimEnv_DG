# 定位漂移与地图假障碍暂停交接（2026-08-14）

## 暂停状态

本问题自 2026-08-14 起暂停，当前没有实施新的定位或地图修复。下一项工作是独立分析
“返航时无法出大门、无法回到出生点”，不得在没有新证据时把该返航故障归因于本问题，
也不得借返航修复改变生产定位、规划、控制或换层语义。

代码基线为：

```text
分支：main
提交：88ebb41515313b298a2a1185b6ed220abb55f2e0
说明：轨迹保护、电梯局部、RViz 假错位、广度优先
```

暂停时仓库仅有文档改动：`docs/team/BUG_LEDGER.md`、`docs/team/RUNBOOK.md` 和本文件；
另有仓库外记录 `/home/xbh/tzb26/co/jilu.md`。没有为本次根因分析修改代码或配置。

## 冻结证据

主证据目录：

```text
/home/xbh/tzb26/evidence/simenv/seed8127_g3_old_crossing_full_max8_20260814_r1
```

关键文件 SHA-256：

```text
505918be1150cd9243c14f58d465a2ef58810ad876db786e67fd1f8ac7c64d98  TEST_SUMMARY.txt
d10a71c1b85468a206df8a3b6cf75f33e88dbb6bc5d293e0690cbeef1985c28b  localization_drift.json
fcce6c2ca44b110adb526e5d93c491c6689dcc3cbc11396e1b5e2413d748de8d  localization_drift.csv
a78458915cc566d240d8c2b44a55adca4bd70d879a18acb54d8625997f31f2cb  mission.log
741202a7b767921acb1fa9203c1423526fd36a1c9dc5f2d984602992e7c0e805  elevator_diagnostic.bag
```

该轮为 seed=8127、G3 `adaptive_shared_map`、Livox-only、`max_range=8m`、GUI=true、
`floor_timeout=280s`、总预算 `620s` 的完整配置启动，但在 elapsed `153.5s`、真值行程
`38.24m` 时人工结束，尚未进入电梯。它不是完整二层闭环，也不记作算法失败。

定位指标为 RMSE `0.12189m`、P95 `0.16683m`、最大 `0.20130m`、末端
`0.17311m`、最大 yaw `0.00463rad`；odom 平均 `9.947Hz`，最大间隔 `0.204s`。

## 已确认结论

1. 误差以走廊方向平移为主，yaw 正常；命令积分没有少于真值，扫描匹配却把估计向旧地图
   拉回，因此不能用 `command_velocity_scale` 补偿。
2. `max_range=8m` 只裁剪当前输入和交给 Mapper 的注册点云，不改变
   `maximum_map_distance=12m`、最近邻范围、残差截断、迭代次数和每帧 Estimate/Commit。
   8m 内仍有重复长墙，无法单独消除退化方向及共享地图正反馈。
3. 同机 15m 轮按相同 `38.24m` 行程截取后的 RMSE/P95/最大/末端为
   `0.1271/0.1827/0.1973/0.1499m`。8m 的 RMSE/P95 略好，最大/末端结论混合；两轮
   路径不同且各一次，不能据此判定最优参数。
4. 本轮输出连续，没有形成“机器过慢导致定位掉帧”的证据链。其他电脑的 8m 最优结论尚无
   同一原始 bag、Git SHA、可执行文件哈希和等里程统计支撑，不能直接移植结论。
5. Mapper 会把高度 `0.30～1.40m` 的回波首帧写为 `HIGH_HARD`；普通反证射线不能撤销
   保护带外的 HIGH_HARD。定位漂移使同一墙面落入新栅格后，旧核心保留、新核心继续增长，
   再经膨胀形成越来越厚的墙影和门口假障碍。
6. 本轮 `hard_core_count` 从 elapsed `89.92s` 的 `3304` 增至结束时 `4347`，末尾
   `occupied_to_free_count=0`，与上述生命周期一致。轨迹保护只覆盖实际路径的 `0.25m`
   核心带，不负责清除整个大厅。

## 尚未证实

- 当前 bag 没有 `/nav/map_debug` 和
  `/danger_search/localization/cloud_registered`，不能逐格证明截图中每个黑格的来源。
- 15m 旧轮早于当前最终提交，8m/15m 证据都没有保存 Git SHA 和定位可执行文件哈希，
  不能称为严格冻结代码的 A/B。
- 历史 Stage17 的 `RMSE=0.0869m/P95=0.1379m` 来自 GUI=false、单层预算 33s 的短闭环，
  且 handoff 未冻结 `max_range`；不能作为当前长轮的直接基准。
- 尚未重新完成 8m GUI 二层完整闭环，亦未验证 HIGH_HARD 反证生命周期候选修复。

## 恢复顺序

1. 先补诊断证据：长轮低频记录地图；仅在短问题窗口记录 `/scan`、注册点云、odom 和离线
   真值。每轮保存 Git SHA、可执行文件哈希、有效参数、seed、GUI/诊断开关及公开场景哈希。
2. 两台电脑重放同一份 `/scan + /trunk_imu + /cmd_vel` 原始 bag。输出不同则检查构建和
   数值环境；输出一致而在线结果不同，则检查 Gazebo 物理轨迹与调度。
3. 地图候选方案保持 HIGH_HARD 首帧阻挡，但让当前障碍命中之前的自由射线，在多代次、
   多位姿严格反证后撤销旧核心。同帧命中格、BLACK、NEGATIVE 和当前近障停车语义不放宽。
4. 定位参数按单变量顺序离线重放：`max_range=8m + maximum_map_distance=8m`；随后分别测试
   `minimum_residual_cutoff=0.06`、`maximum_iterations=8`。不得一次改多个参数。
5. 同 bag 胜出后才运行 GUI=true 的完整二层闭环，并做同 seed 重复或跨 seed 验证。

## 明确禁止

- 不扩大历史轨迹永久 FREE 范围，不降低障碍高度阈值或全局停车距离。
- 不用命令速度尺度掩盖扫描匹配误差。
- 不恢复此前回退的共同模式门、冻结子图、定位 state/epoch、逃逸证书或换层改造。
- 诊断真值只供运行结束后分析，不反馈当轮定位、规划或控制。
- 不把启动成功写成二层闭环成功，也不把人工中止写成算法失败。

## 运行态断点

暂停检查时 ROS 端口 `11311` 空闲，Gazebo、RViz、roscore 和任务节点均未运行。仅发现上一轮
runner 遗留的 FIFO 保活 `sleep` 外壳；它不占 ROS 端口，未在本次文档备份中擅自结束。


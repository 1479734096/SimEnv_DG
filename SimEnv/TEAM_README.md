# 参赛系统隔离重建入口

本仓库以官方 `SimEnv初始.zip` 为只读底座，按照依赖关系逐层迁移参赛系统。官方文件由 `OFFICIAL_FILES.sha256` 保护；参赛代码只允许进入 `src/danger_search/`、`src/danger_search_localization/` 及团队文档、测试和工具目录。

当前状态与每层证据见 `reports/`，总体迁移状态见 `docs/team/MIGRATION_LEDGER.md`。

基础检查：

```bash
sha256sum -c OFFICIAL_FILES.sha256
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

官方全传感器底座默认使用已验证的暂停加载和条件恢复物理流程：

```bash
GUI=false ./auto.sh
```

对官方保护文件的授权修改和哈希差异必须记录在阶段报告，不得静默扩大改动范围。

正式参赛节点：

```bash
source devel/setup.bash
roslaunch danger_search danger_search.launch controller_ready_ack:=true
```

只有在 `junior_ctrl` 已输出 `Entered RL /cmd_vel mode.` 后才能传入该确认。
正式节点等待定位 ready 后才开始任务预算，并严格使用
`generated_building/team_scene_info.json` 公布的结果路径。默认启动竞争、标准 RL
交接和跌倒健康闭环已完成定向验收；第 10 层仍需完成单层、多层、安全与性能总验收。

生产/离线边界检查：

```bash
python3 tools/offline/check_production_boundary.py
```

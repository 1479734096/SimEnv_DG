"""危险源自主搜索算法包。

包内代码按“数据接入—环境理解—任务决策—运动执行”分层：

* ``pose_provider`` 与 ``sensor_buffer`` 接收定位、点云和 RGB-D 数据；
* ``mapper``、各类 detector 与 ``detection_tracker`` 生成地图和稳定目标；
* ``frontier``、``planner`` 与 ``mission_manager`` 选择探索/返航行为；
* ``navigator`` 和 ``navigation_tasks`` 把行为转换为唯一的 ``/cmd_vel`` 输出；
* ``interface_policy`` 与 ``building_feature_provider`` 在最外层限制正式运行
  能访问的话题、服务和文件，防止业务模块绕过比赛接口边界。

主入口是 ``scripts/danger_search_node``。除入口脚本外，导入本包不会主动创建
ROS 节点、订阅话题或写结果文件。
"""

__all__ = [
    "detection_tracker",
    "frontier_explorer",
    "local_controller",
    "mission_manager",
    "multi_floor_mapper",
    "red_sphere_detector",
    "result_writer",
    "sensor_buffer",
]

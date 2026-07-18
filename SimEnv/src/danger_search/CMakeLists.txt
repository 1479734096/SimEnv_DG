cmake_minimum_required(VERSION 3.0.2)
project(danger_search)

# Python 包由 catkin 安装为 ROS 节点；运行依赖覆盖传感器消息、同步、图像桥接、
# 建筑服务和控制消息。定位 C++ 节点通过 launch 组合，不形成 Python 构建依赖。
find_package(catkin REQUIRED COMPONENTS
  building_generator_interfaces
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  building_generator_interfaces
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
  # danger_search_node 是正式入口，其余脚本只用于离线评测和参数工具。
  scripts/batch_evaluate.py
  scripts/danger_search_node
  tools/generate_optimization_report.py
  tools/run_seed.py
  tools/summarize_results.py
  tools/tune_params.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

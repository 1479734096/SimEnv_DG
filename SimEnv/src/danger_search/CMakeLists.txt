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

# 只安装正式节点入口；离线审计与评测工具固定放在工作区根目录 tools/offline，
# 不属于 catkin 包，也不得进入安装清单。
catkin_install_python(PROGRAMS
  scripts/danger_search_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

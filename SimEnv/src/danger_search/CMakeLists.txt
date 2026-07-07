cmake_minimum_required(VERSION 3.0.2)
project(danger_search)

find_package(catkin REQUIRED COMPONENTS
  building_generator_interfaces
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
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
  tf
)

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

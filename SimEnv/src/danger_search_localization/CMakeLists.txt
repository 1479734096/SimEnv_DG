cmake_minimum_required(VERSION 3.16)
project(danger_search_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

# KISS-ICP v0.3.0 的核心库；不编译其 ROS wrapper，避免任何 TF 订阅。
set(USE_SYSTEM_EIGEN3 OFF CACHE BOOL "" FORCE)
set(USE_SYSTEM_SOPHUS OFF CACHE BOOL "" FORCE)
set(USE_SYSTEM_TBB ON CACHE BOOL "" FORCE)
set(USE_SYSTEM_TSL-ROBIN-MAP OFF CACHE BOOL "" FORCE)
set(USE_CCACHE OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/kiss_icp/cpp/kiss_icp ${CMAKE_CURRENT_BINARY_DIR}/kiss_icp)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs)

add_executable(scan_localization_node src/scan_localization_node.cpp)
target_compile_features(scan_localization_node PRIVATE cxx_std_17)
target_include_directories(scan_localization_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(scan_localization_node kiss_icp::pipeline ${catkin_LIBRARIES})

install(TARGETS scan_localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

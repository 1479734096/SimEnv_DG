cmake_minimum_required(VERSION 3.16)
project(danger_search_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roslib
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)

# KISS-ICP v0.3.0 的核心库；不编译其 ROS wrapper，避免任何 TF 订阅。
set(USE_SYSTEM_EIGEN3 OFF CACHE BOOL "" FORCE)
set(USE_SYSTEM_SOPHUS OFF CACHE BOOL "" FORCE)
set(USE_SYSTEM_TBB ON CACHE BOOL "" FORCE)
set(USE_SYSTEM_TSL-ROBIN-MAP OFF CACHE BOOL "" FORCE)
set(USE_CCACHE OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/kiss_icp/cpp/kiss_icp ${CMAKE_CURRENT_BINARY_DIR}/kiss_icp)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roslib roscpp sensor_msgs std_msgs
  INCLUDE_DIRS include
)

add_executable(scan_localization_node src/scan_localization_node.cpp)
target_compile_features(scan_localization_node PRIVATE cxx_std_17)
target_include_directories(scan_localization_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(scan_localization_node kiss_icp::pipeline ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_velocity_transform test/test_velocity_transform.cpp)
  target_compile_features(test_velocity_transform PRIVATE cxx_std_17)
  target_include_directories(test_velocity_transform PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_velocity_transform ${catkin_LIBRARIES} gtest_main)
endif()

install(TARGETS scan_localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

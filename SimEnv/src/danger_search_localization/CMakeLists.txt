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
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration)

# KISS-ICP v0.3.0 的核心库；不编译其 ROS wrapper，避免任何 TF 订阅。
set(USE_SYSTEM_EIGEN3 OFF CACHE BOOL "" FORCE)
set(USE_SYSTEM_SOPHUS OFF CACHE BOOL "" FORCE)
set(USE_SYSTEM_TBB ON CACHE BOOL "" FORCE)
set(USE_SYSTEM_TSL-ROBIN-MAP OFF CACHE BOOL "" FORCE)
set(USE_CCACHE OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/kiss_icp/cpp/kiss_icp ${CMAKE_CURRENT_BINARY_DIR}/kiss_icp)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roslib roscpp sensor_msgs std_msgs
  DEPENDS PCL
  INCLUDE_DIRS include
)

add_executable(scan_localization_node src/scan_localization_node.cpp)
target_compile_features(scan_localization_node PRIVATE cxx_std_17)
# catkin 默认构建未设置优化级别；PCL GICP 在 -O0 下会阻塞 10 Hz 定位回调。
# 仅优化参赛方定位节点，不改变赛方包或全工作区构建语义。
target_compile_options(scan_localization_node PRIVATE -O2)
target_include_directories(scan_localization_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan_localization_node
  kiss_icp::pipeline
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_velocity_transform test/test_velocity_transform.cpp)
  target_compile_features(test_velocity_transform PRIVATE cxx_std_17)
  target_include_directories(test_velocity_transform PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_velocity_transform ${catkin_LIBRARIES} gtest_main)

  catkin_add_gtest(test_point_range_filter test/test_point_range_filter.cpp)
  target_compile_features(test_point_range_filter PRIVATE cxx_std_17)
  target_include_directories(test_point_range_filter PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_point_range_filter gtest_main)

  catkin_add_gtest(test_ground_plane_initializer test/test_ground_plane_initializer.cpp)
  target_compile_features(test_ground_plane_initializer PRIVATE cxx_std_17)
  target_include_directories(test_ground_plane_initializer PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_ground_plane_initializer gtest_main)

  catkin_add_gtest(test_imu_constrained_odometry test/test_imu_constrained_odometry.cpp)
  target_compile_features(test_imu_constrained_odometry PRIVATE cxx_std_17)
  target_include_directories(test_imu_constrained_odometry PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_imu_constrained_odometry
    kiss_icp::pipeline
    ${PCL_LIBRARIES}
    gtest_main
  )

  catkin_add_gtest(test_imu_motion_predictor test/test_imu_motion_predictor.cpp)
  target_compile_features(test_imu_motion_predictor PRIVATE cxx_std_17)
  target_include_directories(test_imu_motion_predictor PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_imu_motion_predictor gtest_main)

  catkin_add_gtest(test_planar_gicp_odometry test/test_planar_gicp_odometry.cpp)
  target_compile_features(test_planar_gicp_odometry PRIVATE cxx_std_17)
  target_include_directories(test_planar_gicp_odometry PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_planar_gicp_odometry
    ${PCL_LIBRARIES}
    gtest_main
  )

endif()

install(TARGETS scan_localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

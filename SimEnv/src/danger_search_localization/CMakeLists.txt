cmake_minimum_required(VERSION 3.16)
project(danger_search_localization)

# 构建边界：只编译参赛方 scan_localization_node 和定向单测；第三方 KISS-ICP
# 作为固定源码子目录链接，不启用其 ROS wrapper，避免额外节点进入正式启动链。
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
# Ubuntu 20.04 的 FindEigen3 模块导出 EIGEN3_INCLUDE_DIRS，而 catkin_package
# 按依赖名检查 Eigen3_INCLUDE_DIRS；显式桥接后才能正确导出公共头文件依赖。
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})

# KISS-ICP 固定提交 460c6f2（v0.2.9-42-g460c6f2）；不编译其 ROS wrapper。
set(USE_SYSTEM_EIGEN3 OFF CACHE BOOL "" FORCE)
set(USE_SYSTEM_SOPHUS OFF CACHE BOOL "" FORCE)
set(USE_SYSTEM_TBB ON CACHE BOOL "" FORCE)
set(USE_SYSTEM_TSL-ROBIN-MAP OFF CACHE BOOL "" FORCE)
set(USE_CCACHE OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/kiss_icp/cpp/kiss_icp ${CMAKE_CURRENT_BINARY_DIR}/kiss_icp)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roslib roscpp sensor_msgs std_msgs
  DEPENDS Eigen3 PCL
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
  # 每个数学/时间/配准组件独立成 gtest，便于定位注释或实现改动影响的具体层次。
  catkin_add_gtest(test_velocity_transform test/test_velocity_transform.cpp)
  target_compile_features(test_velocity_transform PRIVATE cxx_std_17)
  target_include_directories(test_velocity_transform PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_velocity_transform ${catkin_LIBRARIES} gtest_main)

  catkin_add_gtest(test_ros_time_window test/test_ros_time_window.cpp)
  target_compile_features(test_ros_time_window PRIVATE cxx_std_17)
  target_include_directories(test_ros_time_window PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_ros_time_window ${catkin_LIBRARIES} gtest_main)

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

cmake_minimum_required(VERSION 3.14)
project(unitree_guide)
# 添加 torch 依赖。优先使用 -DLIBTORCH_ROOT 或环境变量 LIBTORCH_ROOT，
# 否则按常见本机路径和原始工程路径自动查找。
set(LIBTORCH_ROOT "" CACHE PATH "libtorch 根目录")
if(NOT LIBTORCH_ROOT AND DEFINED ENV{LIBTORCH_ROOT})
    set(LIBTORCH_ROOT "$ENV{LIBTORCH_ROOT}")
endif()
if(NOT LIBTORCH_ROOT)
    foreach(_candidate_libtorch_root
            "/home/xbh/third_party/libtorch"
            "/home/ros/Guoyulun/Download/libtorch")
        if(EXISTS "${_candidate_libtorch_root}/share/cmake/Torch/TorchConfig.cmake")
            set(LIBTORCH_ROOT "${_candidate_libtorch_root}")
            break()
        endif()
    endforeach()
endif()
if(NOT LIBTORCH_ROOT)
    message(FATAL_ERROR "未找到 libtorch，请设置 -DLIBTORCH_ROOT=/path/to/libtorch 或导出环境变量 LIBTORCH_ROOT")
endif()
message(STATUS "Using libtorch: ${LIBTORCH_ROOT}")
set(_LIBTORCH_RUNTIME_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../scripts/prepare_libtorch_runtime.sh")
if(EXISTS "${_LIBTORCH_RUNTIME_SCRIPT}")
    execute_process(
        COMMAND bash "${_LIBTORCH_RUNTIME_SCRIPT}" "${LIBTORCH_ROOT}"
        RESULT_VARIABLE _LIBTORCH_RUNTIME_RESULT
        OUTPUT_VARIABLE _LIBTORCH_RUNTIME_OUTPUT
        ERROR_VARIABLE _LIBTORCH_RUNTIME_ERROR
    )
    if(NOT _LIBTORCH_RUNTIME_RESULT EQUAL 0)
        message(FATAL_ERROR
            "准备 libtorch 运行库失败：${_LIBTORCH_RUNTIME_ERROR}"
            "请确认 LIBTORCH_ROOT 可写，或手动检查 libtorch/lib 下的 SONAME 软链接。")
    endif()
    if(_LIBTORCH_RUNTIME_OUTPUT)
        string(STRIP "${_LIBTORCH_RUNTIME_OUTPUT}" _LIBTORCH_RUNTIME_OUTPUT)
        message(STATUS "${_LIBTORCH_RUNTIME_OUTPUT}")
    endif()
else()
    message(WARNING "未找到 libtorch 运行库准备脚本：${_LIBTORCH_RUNTIME_SCRIPT}")
endif()
list(APPEND CMAKE_MODULE_PATH "${LIBTORCH_ROOT}/share/cmake/Caffe2/public")
list(APPEND CMAKE_MODULE_PATH "${LIBTORCH_ROOT}/share/cmake")

find_path(CUDNN_INCLUDE_PATH cudnn.h
    HINTS
        "$ENV{CUDNN_INCLUDE_DIR}"
        "$ENV{CUDNN_ROOT}/include"
        "${LIBTORCH_ROOT}/include"
        "/usr/local/cuda/include"
        "/usr/include"
)
find_library(CUDNN_LIBRARY_PATH
    NAMES cudnn libcudnn.so.8
    HINTS
        "$ENV{CUDNN_LIBRARY}"
        "$ENV{CUDNN_ROOT}/lib"
        "$ENV{CUDNN_ROOT}/lib64"
        "${LIBTORCH_ROOT}/lib"
        "/usr/local/cuda/lib64"
        "/usr/lib/x86_64-linux-gnu"
)
if(NOT CUDNN_INCLUDE_PATH OR NOT CUDNN_LIBRARY_PATH)
    message(FATAL_ERROR
        "未找到完整 cuDNN 开发环境。当前 libtorch 需要 cuDNN，"
        "请安装 cuDNN 开发包以提供 cudnn.h，或设置 CUDNN_ROOT/CUDNN_INCLUDE_DIR/CUDNN_LIBRARY。"
        " 已检测到 CUDNN_INCLUDE_PATH=${CUDNN_INCLUDE_PATH}, CUDNN_LIBRARY_PATH=${CUDNN_LIBRARY_PATH}")
endif()
set(CUDNN_INCLUDE_DIR "${CUDNN_INCLUDE_PATH}" CACHE PATH "cuDNN 头文件目录" FORCE)
set(CUDNN_LIBRARY "${CUDNN_LIBRARY_PATH}" CACHE FILEPATH "cuDNN 库文件" FORCE)
find_package(Torch REQUIRED PATHS "${LIBTORCH_ROOT}" NO_DEFAULT_PATH)

set(ROBOT_TYPE A1)         # The type of robot, support Go1 and A1 currently
set(PLATFORM amd64)         # The platform to compile, support amd64 and arm64

set(CATKIN_MAKE ON)        # Use CATKIN_MAKE or not, ON or OFF
set(SIMULATION ON)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT OFF)          # Link real robot or not, ON or OFF
set(DEBUG OFF)              # Use debug functions or not, ON or OFF
set(MOVE_BASE OFF)               # Need move_base or not, ON or OFF 

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(SIMULATION OR MOVE_BASE)
    add_definitions(-DRUN_ROS)
    set(CATKIN_MAKE ON)
endif()

set(CMAKE_CXX_STANDARD 11)
find_package(Boost)

if(CATKIN_MAKE)
    add_definitions(-DCOMPILE_WITH_ROS)
    if(MOVE_BASE)
        add_definitions(-DCOMPILE_WITH_MOVE_BASE)
    endif()
endif()

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()




if(CATKIN_MAKE)
    if(SIMULATION)
        add_definitions(-DCOMPILE_WITH_SIMULATION)
        find_package(catkin REQUIRED COMPONENTS
            controller_manager
            joint_state_controller
            gazebo_ros
            # gazebo
        )
    endif()

    find_package(catkin REQUIRED COMPONENTS
        genmsg
        robot_state_publisher
        message_generation
        roscpp
        rospy
        std_msgs
        tf
        geometry_msgs
        unitree_legged_msgs
    )
    ##添加livox的msg包
    ## Generate messages in the 'msg' folder
    add_message_files(FILES
        CustomPoint.msg
        CustomMsg.msg
        #   Message2.msg
    )

    ## Generate added messages and services with any dependencies listed here
    generate_messages(DEPENDENCIES
        std_msgs
    )

    catkin_package(
        CATKIN_DEPENDS 
        unitree_legged_msgs 
    )
endif()

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        include_directories(
            library/unitree_legged_sdk_3.2/include
        )
        link_directories(
            library/unitree_legged_sdk_3.2/lib
        )
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        include_directories(
            library/unitree_legged_sdk-3.8.0/include
        )
        if(${PLATFORM} STREQUAL "amd64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/amd64
            )
        elseif(${PLATFORM} STREQUAL "arm64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/arm64
            )
        endif()
    endif()
endif()

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)  # torch 编译加速
set(CMAKE_CXX_STANDARD 17) #torch编译加速


add_executable(state_from_gazebo src/state_from_gazebo.cpp)
target_link_libraries(state_from_gazebo ${catkin_LIBRARIES})


add_executable(junior_ctrl src/main.cpp ${SRC_LIST})
target_link_libraries(junior_ctrl ${TORCH_LIBRARIES})
if(CATKIN_MAKE)
    target_link_libraries(junior_ctrl ${catkin_LIBRARIES})
    add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python2::Python Python2::NumPy)
endif()
if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
            target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    endif()
endif()
target_link_libraries(junior_ctrl -pthread lcm)
if(NOT CATKIN_MAKE)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
endif()


catkin_install_python(PROGRAMS
  scripts/virtual_joy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/pointcloud2livox.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



# generate custom msg headers
# set(LIVOX_INTERFACES livox_interfaces2)
# rosidl_generate_interfaces(${LIVOX_INTERFACES}
# "msg/CustomPoint.msg"
# "msg/CustomMsg.msg"
# DEPENDENCIES builtin_interfaces std_msgs
# LIBRARY_NAME ${PROJECT_NAME}
# )

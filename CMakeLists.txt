cmake_minimum_required(VERSION 3.16.3)

project(sp_vision CXX)

#add_definitions(-DSPDLOG_FMT_EXTERNAL) # 强制 spdlog 用系统安装的 fmt

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# 编译模式选项：控制是否使用CUDA/TensorRT和OpenVINO
option(USE_CUDA "Enable CUDA/TensorRT support" ON)
option(USE_OPENVINO "Enable OpenVINO support" ON)

if(NOT USE_CUDA AND NOT USE_OPENVINO)
    message(FATAL_ERROR "At least one of USE_CUDA or USE_OPENVINO must be enabled.")
endif()

# 其他编译选项,默认不编译多余组件，可以改OFF为ON来启用或禁用，或者在cmake ..中添加-DBUILD_XXX=ON临时启用
option(BUILD_SENTRY "Build sentry tools" OFF)
option(BUILD_TEST "Build test tools" ON)
option(BUILD_CALIBRATION "Build calibration tools" OFF)
option(BUILD_ALL "Build all components" OFF)

# 全局宏控制，注释上即禁用，不注释即启用，亦可直接在cmake命令中添加 -DBUILD_GIMAL_ADD_VEL=ON 即可
add_compile_definitions(SR_VEL) # 带前馈角速度的串口收发模式(哨兵模式暂时不可用)
add_compile_definitions(LIM_CODE) #简化代码模式，启用后会去掉一些暂时不必要的代码，用于提升编译速度和开销
add_compile_definitions(NOVA_Q) #基于peek2的四元数取数方式,待多几个版本测试以验证可靠性
add_compile_definitions(FIRE_CONSTRAINT) # 开火约束
add_compile_definitions(NOVA_OUTPOST_V2) # 新版前哨站识别方案2
add_compile_definitions(NOVA_AIM_CENTER) # 新版锁中心模式

if(BUILD_SENTRY)
add_compile_definitions(SENTRY_SR) # 串口通信编译为哨兵导航联合通信版本
message(STATUS "Sentry mode enabled, union serial will be built.")
else()
message(STATUS "Sentry mode disabled, standard serial will be built.")
endif()

message(STATUS "--------------------BUILD_CALIBRATION: ${BUILD_CALIBRATION}--------------------")
message(STATUS "--------------------BUILD_TEST: ${BUILD_TEST}--------------------")
message(STATUS "--------------------BUILD_SENTRY: ${BUILD_SENTRY}--------------------")
message(STATUS "--------------------BUILD_ALL: ${BUILD_ALL}--------------------")

# 添加查找模块路径（用于TensorRT）
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# ========== 条件启用CUDA/TensorRT ==========
if(USE_CUDA)
    enable_language(CUDA)
    set(CMAKE_CUDA_ARCHITECTURES 86) # 设置CUDA架构:xavier为72,orin为87，16/20系显卡为75，30系显卡为80/86，40系显卡为89
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")
    
    find_package(CUDA REQUIRED)
    find_package(TensorRT REQUIRED)
    
    add_compile_definitions(USE_CUDA)
    message(STATUS "CUDA/TensorRT support is enabled.")
else()
    message(STATUS "CUDA/TensorRT support is disabled.")
endif()

# ========== 条件启用OpenVINO ==========
if(USE_OPENVINO)
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake" CACHE PATH "OpenVINO CMake directory")
    find_package(OpenVINO REQUIRED COMPONENTS Runtime)
    add_compile_definitions(USE_OPENVINO)
    message(STATUS "OpenVINO support is enabled.")
else()
    message(STATUS "OpenVINO support is disabled.")
endif()

# 公共依赖查找
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# 添加子目录
add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

#################### src 可执行文件 ###################
if(NOT BUILD_SENTRY)
    add_executable(standard src/standard.cpp)
    #add_executable(mt_standard src/mt_standard.cpp)
    #add_executable(standard_mpc src/standard_mpc.cpp)
    add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
    #add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
    add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
    add_executable(standard_serial src/standard_serial.cpp)
    add_executable(auto_buff_debug_serial src/auto_buff_debug_serial.cpp)
    #add_executable(uav src/uav.cpp)
    #add_executable(uav_debug src/uav_debug.cpp)

    target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    #target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    #target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    #target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff auto_aim tools io ${CERES_LIBRARIES})
    target_link_libraries(standard_serial ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    target_link_libraries(auto_buff_debug_serial ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff auto_aim tools io ${CERES_LIBRARIES})
    #target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    #target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
else()
    message(STATUS "Sentry mode enabled, skipping standard executables.")
endif()

################## calibration ##################
# 只在 BUILD_CALIBRATION 或 BUILD_ALL 为 ON 时编译 calibration 工具
if(BUILD_CALIBRATION OR BUILD_ALL)
    add_executable(capture calibration/capture.cpp)
    add_executable(calibrate_camera calibration/calibrate_camera.cpp)
    add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
    add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
    add_executable(split_video calibration/split_video.cpp)

    target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
    target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
    target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
    target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)
    message(STATUS "Calibration tools are enabled.")
else()
    message(STATUS "Calibration tools are disabled. Use -DBUILD_CALIBRATION=ON or -DBUILD_ALL=ON to enable.")
endif()

################## demo ##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

################## tests ##################
if(BUILD_TEST OR BUILD_ALL AND NOT BUILD_SENTRY)
    add_executable(auto_buff_test tests/auto_buff_test.cpp)
    add_executable(camera_detect_test tests/camera_detect_test.cpp)
    add_executable(camera_test tests/camera_test.cpp)
    add_executable(camera_thread_test tests/camera_thread_test.cpp)
    add_executable(cboard_test tests/cboard_test.cpp)
    add_executable(fire_test tests/fire_test.cpp)
    add_executable(detector_video_test tests/detector_video_test.cpp)
    add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
    add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
    add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
    add_executable(usbcamera_test tests/usbcamera_test.cpp)
    add_executable(handeye_test tests/handeye_test.cpp)
    #add_executable(dm_test tests/dm_test.cpp)
    #add_executable(minimum_vision_system tests/minimum_vision_system.cpp)

    target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff auto_aim tools io)
    target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
    target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
    target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
    target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
    target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
    target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
    target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
    #target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
    #target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(gimbal_test tests/gimbal_test.cpp)
    target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

    add_executable(planner_test tests/planner_test.cpp)
    target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

    add_executable(planner_test_offline tests/planner_test_offline.cpp)
    target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
else()
    message(STATUS "Test tools are disabled. Use -DBUILD_TEST=ON or -DBUILD_ALL=ON to enable.")
endif()

# 检查ROS2 相关环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(cv_bridge QUIET)

# 只有在找到 ROS 环境且BUILD_SENTRY 或 BUILD_ALL 为 ON 时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND AND geometry_msgs_FOUND AND sensor_msgs_FOUND AND cv_bridge_FOUND AND BUILD_SENTRY)
    #add_compile_definitions(SENTRY_SR) # 串口通信编译为哨兵导航联合通信版本
    # 编译 sentry 和 publish_test
    add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
    add_executable(standard_serial src/standard_serial.cpp)
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(sentry_odin src/sentry_odin.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)
    add_executable(odincamera_test tests/odincamera_test.cpp)

    target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    target_link_libraries(standard_serial ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
    target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_odin ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(odincamera_test ${OpenCV_LIBS} fmt::fmt tools io)
else()
    message(STATUS "ROS2 environment not found or sentry disabled, skipping ROS2-related code.")
endif()

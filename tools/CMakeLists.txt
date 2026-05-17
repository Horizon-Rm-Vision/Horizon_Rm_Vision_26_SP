cmake_minimum_required(VERSION 3.16)
add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
    ui_manager.cpp
    ui_stream_recorder.cpp
    ui_web_stream.cpp
)

# 链接yaml-cpp库，因为ui_manager.cpp使用了YAML功能
target_link_libraries(tools PUBLIC yaml-cpp)
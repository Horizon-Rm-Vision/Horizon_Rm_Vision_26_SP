cmake_minimum_required(VERSION 3.16)

find_package(Ceres REQUIRED)

set(AUTO_BUFF_SRCS
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    buff_tracker.cpp
)

if(USE_OPENVINO)
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
    find_package(OpenVINO REQUIRED)
    list(APPEND AUTO_BUFF_SRCS
        yolo11_buff.cpp
        yolox_buff.cpp
    )
endif()

if(USE_CUDA)
    list(APPEND AUTO_BUFF_SRCS yolox_buff_trt.cpp)
endif()

add_library(auto_buff OBJECT ${AUTO_BUFF_SRCS})

target_link_libraries(auto_buff auto_aim ${CERES_LIBRARIES})

if(USE_OPENVINO)
    target_link_libraries(auto_buff openvino::runtime)
endif()

# CUDA/TensorRT 库 (用于 yolox_buff_trt)
if(USE_CUDA)
    target_link_libraries(auto_buff ${CUDA_LIBRARIES} ${TensorRT_LIBRARIES})
endif()
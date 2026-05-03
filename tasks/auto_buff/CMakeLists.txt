cmake_minimum_required(VERSION 3.16)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

add_library(auto_buff OBJECT
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
    yolox_buff.cpp
)

if(USE_CUDA)
    target_sources(auto_buff PRIVATE yolox_buff_trt.cpp)
endif()

target_link_libraries(auto_buff auto_aim openvino::runtime ${CERES_LIBRARIES})

# CUDA/TensorRT 库 (用于 yolox_buff_trt)
if(USE_CUDA)
    target_link_libraries(auto_buff ${CUDA_LIBRARIES} ${TensorRT_LIBRARIES})
endif()
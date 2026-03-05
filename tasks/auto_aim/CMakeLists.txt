cmake_minimum_required(VERSION 3.16)

# 添加 tinympc 子目录（该目录应位于 auto_aim/planner/tinympc）
add_subdirectory(planner/tinympc)

# 基础源文件（不依赖特定推理后端）
set(AUTO_AIM_SRCS
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
    # int8_calibrator.cpp
)

# 如果启用 CUDA，添加 TensorRT 相关源文件
if(USE_CUDA)
    list(APPEND AUTO_AIM_SRCS
        yolos/yolov5_trt.cpp
        yolos/yolov5_trt_gpu_kernel.cu
        trt_engine.cpp
    )
endif()

# 如果启用 OpenVINO，添加 OpenVINO 相关源文件
if(USE_OPENVINO)
    list(APPEND AUTO_AIM_SRCS
        yolos/yolov5_ov.cpp
        yolos/yolov8.cpp
        yolos/yolo11.cpp
    )
endif()

# 创建对象库
add_library(auto_aim OBJECT ${AUTO_AIM_SRCS})

# 设置公共包含目录
target_include_directories(auto_aim PUBLIC include)

# 如果启用 CUDA，添加 CUDA 和 TensorRT 包含目录
if(USE_CUDA)
    target_include_directories(auto_aim PUBLIC ${CUDA_INCLUDE_DIRS} ${TensorRT_INCLUDE_DIRS})
endif()

# 链接依赖（对象库的依赖会传播给链接它的目标）
# 公共依赖
target_link_libraries(auto_aim PUBLIC io tinympcstatic)

# CUDA/TensorRT 库
if(USE_CUDA)
    target_link_libraries(auto_aim PUBLIC ${CUDA_LIBRARIES} ${TensorRT_LIBRARIES})
endif()

# OpenVINO 库
if(USE_OPENVINO)
    target_link_libraries(auto_aim PUBLIC openvino::runtime)
endif()
cmake_minimum_required(VERSION 3.16)

if(USE_OPENVINO)
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
    find_package(OpenVINO REQUIRED COMPONENTS Runtime)
endif()

add_library(omniperception OBJECT
    decider.cpp
    perceptron.cpp
)

if(USE_OPENVINO)
    target_link_libraries(omniperception openvino::runtime)
endif()
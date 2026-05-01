cmake_minimum_required(VERSION 3.8)
project(udp_visualization_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(udp_marker_bridge
  src/udp_marker_bridge.cpp
)

target_include_directories(udp_marker_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(udp_marker_bridge
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(udp_marker_bridge
  rclcpp
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  udp_marker_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  include/udp_visualization_bridge/udp_marker_bridge.hpp
  DESTINATION include/udp_visualization_bridge
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

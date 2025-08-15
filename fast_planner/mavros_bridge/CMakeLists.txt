cmake_minimum_required(VERSION 3.8)
project(mavros_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(mavros_bridge src/mavros_bridge.cpp)
ament_target_dependencies(mavros_bridge
    rclcpp
    geometry_msgs
    mavros_msgs
    quadrotor_msgs
    tf2
    tf2_geometry_msgs
)

install(TARGETS mavros_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

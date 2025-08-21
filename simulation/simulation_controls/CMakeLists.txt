cmake_minimum_required(VERSION 3.8)
project(simulation_controls LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp geometry_msgs std_srvs mavros_msgs)

install(
  TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

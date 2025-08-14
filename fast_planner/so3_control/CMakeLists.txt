cmake_minimum_required(VERSION 3.8)
project(so3_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(SO3Control SHARED src/SO3Control.cpp)

add_executable(so3_control_nodelet src/so3_control_nodelet.cpp)
ament_target_dependencies(SO3Control
  rclcpp
  std_msgs
  nav_msgs
  quadrotor_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)
target_link_libraries(so3_control_nodelet SO3Control)

install(TARGETS SO3Control so3_control_nodelet
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()

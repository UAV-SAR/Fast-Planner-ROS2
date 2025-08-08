
cmake_minimum_required(VERSION 3.8)
project(odom_visualization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(odom_visualization src/odom_visualization.cpp)
ament_target_dependencies(odom_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  quadrotor_msgs
  tf2_ros
  pose_utils
  std_msgs
)
target_link_libraries(odom_visualization ${ARMADILLO_LIBRARIES})

install(TARGETS odom_visualization
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(so3_disturbance_generator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(pose_utils REQUIRED)

include_directories(
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(so3_disturbance_generator src/so3_disturbance_generator.cpp)
ament_target_dependencies(so3_disturbance_generator
  rclcpp
  std_msgs
  nav_msgs
  quadrotor_msgs
  tf2
  visualization_msgs
  sensor_msgs
  geometry_msgs
  pose_utils
)
target_link_libraries(so3_disturbance_generator 
   ${ARMADILLO_LIBRARIES}
)

install(TARGETS so3_disturbance_generator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()


cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(quadrotor_dynamics SHARED src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
ament_target_dependencies(quadrotor_simulator_so3
  rclcpp
  quadrotor_msgs
  uav_utils
  sensor_msgs
  nav_msgs
)
target_link_libraries(quadrotor_simulator_so3 
   quadrotor_dynamics
   ${ARMADILLO_LIBRARIES}
)

install(TARGETS quadrotor_simulator_so3 quadrotor_dynamics
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(FILES 
  launch/simulator_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(quadrotor_dynamics)
ament_package()


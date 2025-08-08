cmake_minimum_required(VERSION 3.8)
project(uav_utils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cmake_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_add_gtest(uav_utils-test src/uav_utils_test.cpp)
ament_target_dependencies(uav_utils-test
  rclcpp
  nav_msgs
  geometry_msgs
  cmake_utils
)

ament_package()
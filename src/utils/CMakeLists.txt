cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(link_pose_publisher src/link_pose_publisher.cpp)
ament_target_dependencies(link_pose_publisher rclcpp geometry_msgs tf2_msgs)

install(
  TARGETS link_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(plan_env LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env
  src/sdf_map.cpp
  src/obj_predictor.cpp
  src/edt_environment.cpp
  src/raycast.cpp
)
ament_target_dependencies(plan_env
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  OpenCV
  message_filters
)

add_executable(obj_generator
  src/obj_generator.cpp
)
ament_target_dependencies(obj_generator
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  OpenCV
  message_filters
)
target_link_libraries(obj_generator plan_env)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS plan_env obj_generator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(plan_env)
ament_package()

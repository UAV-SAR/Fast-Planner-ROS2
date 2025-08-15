cmake_minimum_required(VERSION 3.8)
project(multi_map_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)
find_package(std_srvs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(multi_map_msgs REQUIRED)


include_directories(
   include
   ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(multi_map_visualization src/multi_map_visualization.cc)
ament_target_dependencies(multi_map_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  quadrotor_msgs
  tf2
  tf2_geometry_msgs
  pose_utils
  std_srvs
  laser_geometry
  multi_map_msgs
)
target_link_libraries(multi_map_visualization ${ARMADILLO_LIBRARIES})

install(TARGETS multi_map_visualization
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(multi_map_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiOccupancyGrid.msg"
  "msg/MultiSparseMap3D.msg"
  "msg/SparseMap3D.msg"
  "msg/VerticalOccupancyGridList.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

ament_package()

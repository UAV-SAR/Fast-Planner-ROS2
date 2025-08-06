cmake_minimum_required(VERSION 3.8)
project(plan_manage LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(bspline_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(fast_planner_node
  src/fast_planner_node.cpp 
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  src/planner_manager.cpp
)
ament_target_dependencies(fast_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  cv_bridge
  nav_msgs
  visualization_msgs
  plan_env 
  path_searching 
  bspline 
  bspline_opt 
  traj_utils
  bspline_msgs
  pcl_conversions
  poly_traj
)

add_executable(traj_server 
  src/traj_server.cpp
)
ament_target_dependencies(traj_server
  rclcpp
  std_msgs
  geometry_msgs
  cv_bridge
  nav_msgs
  visualization_msgs
  plan_env 
  path_searching 
  bspline 
  bspline_opt 
  traj_utils
  bspline_msgs
  pcl_conversions
  poly_traj
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS fast_planner_node traj_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_package()
cmake_minimum_required(VERSION 3.8)
project(traj_utils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(poly_traj REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories( 
    include
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(traj_utils SHARED
    src/planning_visualization.cpp 
)
ament_target_dependencies(traj_utils
  rclcpp
  std_msgs
  cv_bridge
  bspline
  bspline_opt
  poly_traj
  path_searching
  plan_env
  visualization_msgs
  nav_msgs
  pcl_conversions
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS traj_utils
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(traj_utils)
ament_package()
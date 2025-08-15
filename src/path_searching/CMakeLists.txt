cmake_minimum_required(VERSION 3.8)
project(path_searching LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(path_searching SHARED
    src/kinodynamic_astar.cpp 
    src/astar.cpp
    src/topo_prm.cpp
)
ament_target_dependencies(path_searching
  rclcpp
  std_msgs
  visualization_msgs
  nav_msgs
  cv_bridge
  plan_env
  pcl_conversions
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS path_searching
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(path_searching)
ament_package()

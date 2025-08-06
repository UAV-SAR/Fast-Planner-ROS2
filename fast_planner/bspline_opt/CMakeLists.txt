cmake_minimum_required(VERSION 3.8)
project(bspline_opt LANGUAGES CXX)

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

set(NLOPT_INCLUDE_DIR "/usr/local/include")  
set(NLOPT_LIBRARY "/usr/local/lib/libnlopt.so")

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}  
)

link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${NLOPT_LIBRARY})

add_library(bspline_opt SHARED
    src/bspline_optimizer.cpp 
)
ament_target_dependencies(bspline_opt
    rclcpp
    std_msgs
    visualization_msgs
    cv_bridge
    plan_env
    nav_msgs
    pcl_conversions
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS bspline_opt
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(bspline_opt)
ament_package()

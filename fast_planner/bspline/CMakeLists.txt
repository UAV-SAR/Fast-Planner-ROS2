cmake_minimum_required(VERSION 3.8)
project(bspline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(bspline 
    src/non_uniform_bspline.cpp
)
ament_target_dependencies(bspline
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS bspline
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(bspline)
ament_package()

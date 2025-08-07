cmake_minimum_required(VERSION 3.8)
project(local_sensing LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_CUDA OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(
  pcl_render_node
  src/pcl_render_node.cpp
)

set_target_properties(pcl_render_node PROPERTIES
  CUDA_ARCHITECTURES OFF
)

ament_target_dependencies(pcl_render_node
  rclcpp
  cv_bridge
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  visualization_msgs
  quadrotor_msgs
  image_transport
  message_filters
  OpenCV
)

target_link_libraries(pcl_render_node
  ${Boost_LIBRARIES}
)

if(ENABLE_CUDA)
  set(CMAKE_CUDA_ARCHITECTURES 75)

  add_library(depth_render_cuda STATIC
    src/depth_render.cu
  )

  set_target_properties(depth_render_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
  )

  target_include_directories(depth_render_cuda PUBLIC
    ${CUDA_INCLUDE_DIRS}
    include
  )

  target_compile_options(depth_render_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>
  )

  target_link_libraries(pcl_render_node 
    depth_render_cuda
  )

  install(TARGETS depth_render_cuda
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )
endif()

install(TARGETS pcl_render_node
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
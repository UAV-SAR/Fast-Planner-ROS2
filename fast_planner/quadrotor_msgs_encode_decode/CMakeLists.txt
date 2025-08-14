cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs_encode_decode LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(decode_msgs SHARED src/decode_msgs.cpp)
ament_target_dependencies(decode_msgs rclcpp quadrotor_msgs)
add_library(encode_msgs SHARED src/encode_msgs.cpp)
ament_target_dependencies(encode_msgs rclcpp quadrotor_msgs)

install(TARGETS decode_msgs encode_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(decode_msgs encode_msgs)
ament_package()

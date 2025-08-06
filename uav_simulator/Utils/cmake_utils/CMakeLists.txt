cmake_minimum_required(VERSION 3.8)
project(cmake_utils)

find_package(ament_cmake REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

file(GLOB CMAKE_UTILS_FILES cmake/*.cmake)

install(
  FILES ${CMAKE_UTILS_FILES}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
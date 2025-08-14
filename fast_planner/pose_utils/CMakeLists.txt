cmake_minimum_required(VERSION 3.8)
project(pose_utils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
    include
    ${ARMADILLO_INCLUDE_DIRS}
)

add_library(pose_utils SHARED src/pose_utils.cpp)
target_link_libraries(pose_utils ${ARMADILLO_LIBRARIES})

install(TARGETS pose_utils
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(pose_utils)
ament_package()
cmake_minimum_required(VERSION 3.8)
project(rviz_plugins LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(multi_map_server REQUIRED)
find_package(multi_map_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(nav_msgs REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_definitions(-DQT_NO_KEYWORDS)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/goal_tool.cpp
  src/probmap_display.cpp
  src/aerialmap_display.cpp   
  src/multi_probmap_display.cpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rviz_common
  multi_map_server
  multi_map_msgs
  nav_msgs
  rviz_ogre_vendor
  tf2
  tf2_ros
  rviz_default_plugins
  rviz_rendering
  pluginlib
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  rviz_config.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
cmake_minimum_required(VERSION 3.8)
project(astar_nav2_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(astar_planner_core SHARED
  src/astar_planner.cpp)
target_include_directories(astar_planner_core PRIVATE
  include)
ament_target_dependencies(astar_planner_core
  nav2_core
  nav2_costmap_2d
  nav2_util
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros)

# 安装插件库
install(TARGETS astar_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# 导出插件描述
pluginlib_export_plugin_description_file(nav2_core astar_plugin.xml)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(nav2_core)
ament_export_dependencies(nav2_costmap_2d)
ament_export_dependencies(rclcpp)
ament_package()

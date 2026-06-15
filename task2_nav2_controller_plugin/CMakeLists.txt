cmake_minimum_required(VERSION 3.8)
project(nav2_simple_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(pluginlib REQUIRED)

find_package(nav2_core REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_library(
  nav2_simple_controller
  SHARED
  src/simple_controller.cpp
)

target_include_directories(
  nav2_simple_controller
  PUBLIC
  include
)

ament_target_dependencies(
  nav2_simple_controller

  rclcpp
  rclcpp_lifecycle

  pluginlib

  nav2_core

  nav_msgs

  geometry_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(
  nav2_core
  plugin.xml
)

install(
  TARGETS nav2_simple_controller
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY Config/
  DESTINATION share/${PROJECT_NAME}/Config
)

ament_package()

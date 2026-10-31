# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Example package, NOT part of the ros2_pulse build: colcon and catkin_pkg stop crawling at the
# repository's own package.xml, so this directory is only built when named explicitly:
#   colcon build --base-paths src/ros2_pulse/examples/flood_compare/controller

cmake_minimum_required(VERSION 3.22)
project(flood_compare_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Modern target form (ament_target_dependencies() is gone on rolling), same as the main package.
add_executable(flood_controller src/flood_controller.cpp)
target_link_libraries(flood_controller rclcpp::rclcpp ${geometry_msgs_TARGETS})

add_executable(flood_publisher src/flood_publisher.cpp)
target_link_libraries(flood_publisher rclcpp::rclcpp ${geometry_msgs_TARGETS})

install(TARGETS flood_controller flood_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()

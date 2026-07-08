# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").

cmake_minimum_required(VERSION 3.22)
project(ros2_pulse)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)

# --- pure-C++ core (no ROS / tracetools dependency) ---
add_library(${PROJECT_NAME}_core STATIC
  src/core/topic_registry.cpp
  src/core/timer.cpp
  src/core/window_format.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
set_property(TARGET ${PROJECT_NAME}_core PROPERTY POSITION_INDEPENDENT_CODE ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Threads::Threads)

# --- the LD_PRELOAD probe shared library ---
# Interposes libtracetools.so symbols by name; forwards via dlsym(RTLD_NEXT). No link-time
# dependency on rclcpp/tracetools is required (symbols resolved at runtime).
add_library(${PROJECT_NAME} SHARED
  src/probe/interposers.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_core ${CMAKE_DL_LIBS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_topic_registry test/unit/test_topic_registry.cpp)
  target_link_libraries(test_topic_registry ${PROJECT_NAME}_core)

  ament_add_gtest(test_window_format test/unit/test_window_format.cpp)
  target_link_libraries(test_window_format ${PROJECT_NAME}_core)

  # Integration test nodes (talker / listener / intra) — only if rclcpp is available.
  find_package(rclcpp QUIET)
  find_package(std_msgs QUIET)
  if(rclcpp_FOUND AND std_msgs_FOUND)
    add_executable(stats_probe_test_nodes test/integration/test_nodes.cpp)
    ament_target_dependencies(stats_probe_test_nodes rclcpp std_msgs)
    install(TARGETS stats_probe_test_nodes DESTINATION lib/${PROJECT_NAME})

    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(integration_probe test/integration/test_probe_integration.py
      TIMEOUT 120)
  endif()
endif()

ament_package()

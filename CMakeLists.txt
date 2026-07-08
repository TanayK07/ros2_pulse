# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").

cmake_minimum_required(VERSION 3.22)
project(ros2_pulse)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Standalone (no-ROS) build path: builds the pure-C++ core + the gtest unit suite via
# find_package(GTest)/find_package(Threads), with NO ament/colcon. Lets the fast core lane and
# the sanitizer lanes run on a plain ubuntu image without a ROS container. The ament build below
# is unaffected (this option defaults OFF).
option(ROS2_PULSE_STANDALONE
  "Build the pure-C++ core and gtest unit suite standalone (no ament/ROS)" OFF)

# Optional sanitizer for the standalone lanes: "thread" or "address" (address => address,undefined).
# Applied here, BEFORE any target is created, so the core library is instrumented too.
set(ROS2_PULSE_SANITIZER "" CACHE STRING "Sanitizer for standalone build: thread | address | (empty)")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

if(ROS2_PULSE_SANITIZER STREQUAL "thread")
  add_compile_options(-fsanitize=thread -g -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
elseif(ROS2_PULSE_SANITIZER STREQUAL "address")
  add_compile_options(-fsanitize=address,undefined -g -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
elseif(NOT ROS2_PULSE_SANITIZER STREQUAL "")
  message(FATAL_ERROR "ROS2_PULSE_SANITIZER must be 'thread', 'address' or empty (got '${ROS2_PULSE_SANITIZER}')")
endif()

# --- pure-C++ core (no ROS / tracetools dependency) — common to both build paths ---
add_library(${PROJECT_NAME}_core STATIC
  src/core/topic_registry.cpp
  src/core/timer.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
set_property(TARGET ${PROJECT_NAME}_core PROPERTY POSITION_INDEPENDENT_CODE ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Threads::Threads)

# The core gtest suite — one binary compiled identically under both build paths. The existing
# test file provides main(); the fix-independent additions (lifecycle/UAF, init-order property,
# concurrency stress) are header-free TEST()s that register into the same binary.
set(ROS2_PULSE_CORE_TEST_SOURCES
  test/unit/test_topic_registry.cpp
  test/unit/test_registry_lifecycle.cpp
  test/property/test_init_order.cpp
  test/stress/test_concurrency.cpp
)

if(ROS2_PULSE_STANDALONE)
  # ---- standalone lane: core + gtest, no ament ----
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(test_topic_registry ${ROS2_PULSE_CORE_TEST_SOURCES})
  # GTest::gtest only (main comes from the test file, so gtest_main is intentionally not linked).
  target_link_libraries(test_topic_registry PRIVATE
    ${PROJECT_NAME}_core GTest::gtest Threads::Threads)
  add_test(NAME test_topic_registry COMMAND test_topic_registry)
else()
  # ---- ament / colcon lane (default) ----
  find_package(ament_cmake REQUIRED)

  # The LD_PRELOAD probe shared library. Interposes libtracetools.so symbols by name; forwards
  # via dlsym(RTLD_NEXT). No link-time dependency on rclcpp/tracetools (symbols resolved at run).
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
    ament_add_gtest(test_topic_registry ${ROS2_PULSE_CORE_TEST_SOURCES})
    target_link_libraries(test_topic_registry ${PROJECT_NAME}_core)

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
      # Accuracy harness: known-rate publisher -> reported Hz within tolerance (inter + intra).
      ament_add_pytest_test(integration_accuracy test/integration/test_accuracy.py
        TIMEOUT 180)
    endif()
  endif()

  ament_package()
endif()

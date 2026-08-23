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

# --- pure-C++ core (no ROS / tracetools dependency), common to both build paths ---
# Globbed so a new core source (e.g. from a fix PR) joins the library in BOTH lanes automatically.
file(GLOB ROS2_PULSE_CORE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cpp")
add_library(${PROJECT_NAME}_core STATIC ${ROS2_PULSE_CORE_SOURCES})
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
set_property(TARGET ${PROJECT_NAME}_core PROPERTY POSITION_INDEPENDENT_CODE ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Threads::Threads)

# The core gtest suite, one binary compiled identically under both build paths.
# test_topic_registry.cpp provides main() (gtest_main intentionally not linked); every other
# unit/property/stress file is a header-free set of TEST()s that registers into the same binary.
# Globbed so a fix PR's new test file joins BOTH lanes (fast + sanitizers) with no CMake edit.
# Invariant: exactly ONE globbed file may define main() (test_topic_registry.cpp).
file(GLOB ROS2_PULSE_CORE_TEST_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/test/property/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/test/stress/*.cpp")

# pulse-check CLI (ROADMAP R1): offline expected-rate verdict over probe logs. Pure core, no
# ROS dependency, built in BOTH lanes so the sanitizer lanes cover it too.
add_executable(pulse-check tools/pulse_check.cpp)
target_link_libraries(pulse-check PRIVATE ${PROJECT_NAME}_core)

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
  # Preloaded into every process on the robot: export ONLY the ros_trace_* interposer contract
  # (explicit visibility("default") in interposers.cpp), nothing else may leak into the host's
  # dynamic symbol namespace (KNOWN_ISSUES #14; pinned by test/integration/test_symbols.py).
  # The core objects are linked in statically, so they get the same treatment.
  target_compile_options(${PROJECT_NAME} PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
  target_compile_options(${PROJECT_NAME}_core PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
  # Version script localizes the vague-linkage std:: instantiations that header pragmas force
  # back to default visibility, the export list is the script's global clause, nothing else.
  target_link_options(${PROJECT_NAME} PRIVATE
    "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/src/probe/exports.map")
  set_target_properties(${PROJECT_NAME} PROPERTIES
    LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/probe/exports.map")

  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  # `ros2 run ros2_pulse pulse-check`, or invoked by path from watchdogs/CI (no ROS needed).
  install(TARGETS pulse-check RUNTIME DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_topic_registry ${ROS2_PULSE_CORE_TEST_SOURCES})
    target_link_libraries(test_topic_registry ${PROJECT_NAME}_core)

    # Integration test nodes (talker / listener / intra), only if rclcpp is available.
    find_package(rclcpp QUIET)
    find_package(std_msgs QUIET)
    if(rclcpp_FOUND AND std_msgs_FOUND)
      add_executable(stats_probe_test_nodes test/integration/test_nodes.cpp)
      # Modern target form, ament_target_dependencies() was removed on rolling (deprecated
      # since jazzy); rclcpp::rclcpp and ${std_msgs_TARGETS} exist on every supported distro.
      target_link_libraries(stats_probe_test_nodes rclcpp::rclcpp ${std_msgs_TARGETS})
      install(TARGETS stats_probe_test_nodes DESTINATION lib/${PROJECT_NAME})

      find_package(ament_cmake_pytest REQUIRED)
      ament_add_pytest_test(integration_probe test/integration/test_probe_integration.py
        TIMEOUT 120)
      # Accuracy harness: known-rate publisher -> reported Hz within tolerance (inter + intra).
      ament_add_pytest_test(integration_accuracy test/integration/test_accuracy.py
        TIMEOUT 180)
      # Shutdown path: final partial window written at exit; straggler tracepoints can't crash
      # the host during static destruction (KNOWN_ISSUES #9).
      ament_add_pytest_test(integration_shutdown test/integration/test_shutdown.py
        TIMEOUT 120)
      # fork() without exec: the child re-arms its own flush timer (KNOWN_ISSUES #10).
      ament_add_pytest_test(integration_fork test/integration/test_fork.py
        TIMEOUT 120)
      # Stall visibility: dead upstream must read RECV 0.0, not vanish (KNOWN_ISSUES #12).
      ament_add_pytest_test(integration_stall test/integration/test_stall.py
        TIMEOUT 120)
      # Output rotation: ROS_TOPIC_STATS_MAX_BYTES bounds the file (KNOWN_ISSUES #11).
      ament_add_pytest_test(integration_rotation test/integration/test_rotation.py
        TIMEOUT 120)
      # Symbol surface: only the ros_trace_* interposers may be exported (KNOWN_ISSUES #14).
      ament_add_pytest_test(integration_symbols test/integration/test_symbols.py
        TIMEOUT 60)
      # Publish-side intra counts via rclcpp_intra_publish (jazzy+; humble asserts absence).
      ament_add_pytest_test(integration_intra_publish test/integration/test_intra_publish.py
        TIMEOUT 120)
      # Expected-rate alerting: WARN lines + first-window grace + pulse-check CLI (ROADMAP R1).
      ament_add_pytest_test(integration_rate_spec test/integration/test_rate_spec.py
        TIMEOUT 180)
      # Gap visibility (ROADMAP R5). Longer timeout: the R5-thesis case needs a 16 s run to fit
      # a deliberate 800 ms stall inside a 5 s window and still leave clean windows either side.
      ament_add_pytest_test(integration_jitter test/integration/test_jitter.py
        TIMEOUT 240)
      # QUIET banner suppression (ROADMAP R6): ROS_TOPIC_STATS_QUIET=1 silences the
      # informational banner ONLY, error diagnostics must survive it (see test_quiet.py).
      ament_add_pytest_test(integration_quiet test/integration/test_quiet.py
        TIMEOUT 120)
      # LTTng coexistence (ROADMAP R3): probe forwarding via dlsym(RTLD_NEXT) must feed a LIVE
      # LTTng session too. Runs on Iron+ (lttng-ust-linked tracetools + lttng-tools/babeltrace
      # from the test_depends); self-skips on stock humble, where the tracepoint bodies are
      # no-ops, see the skip-vs-fail rationale at the top of the test file.
      ament_add_pytest_test(integration_lttng_coexist test/integration/test_lttng_coexist.py
        TIMEOUT 180)
      # Output formats (ROADMAP R6): jsonl windows parse with an independent JSON parser,
      # unknown formats warn once + fall back to text, pulse-check gates jsonl logs.
      ament_add_pytest_test(integration_format test/integration/test_format.py
        TIMEOUT 120)
    endif()
  endif()

  ament_package()
endif()

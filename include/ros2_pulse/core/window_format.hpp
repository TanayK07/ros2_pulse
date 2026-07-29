// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ROS2_PULSE__CORE__WINDOW_FORMAT_HPP_
#define ROS2_PULSE__CORE__WINDOW_FORMAT_HPP_

#include <string>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"

namespace ros2_pulse::core {

/// @brief Render one flush window as a single text block, byte-compatible with the on-disk format.
///
/// Layout (one trailing blank line closes the block):
/// @code
/// # ts_ns=<ts_ns> window_s=<window_s to 3dp>
/// TOPIC <topic> <pub_inter_hz to 6dp>          // when pub_inter_count>0 || no recv traffic
/// RECV <topic> inter=<recv_inter_hz to 6dp> intra=<recv_intra_hz to 6dp>  // when any recv count>0
/// NODE <node>
/// <blank line>
/// @endcode
///
/// Pure (no ROS / tracetools / I/O dependency) so the probe can build the whole window once and
/// write it with a single call, and so the exact format is unit-testable.
auto formatWindow(const std::vector<sTopicStat>& stats, const std::vector<std::string>& nodes,
                  long long ts_ns, double window_s) -> std::string;

/// @brief Per-process default output path: `/root/ssd2tb/logs/topic_freq.<pid>.log`.
///
/// Embedding the pid stops every LD_PRELOADed process from appending to one shared file. Kept as a
/// pure function of @p pid so the derivation is testable without spawning a process.
auto defaultOutputPath(long pid) -> std::string;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__WINDOW_FORMAT_HPP_

// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef ROS2_PULSE__CORE__LOG_READER_HPP_
#define ROS2_PULSE__CORE__LOG_READER_HPP_

#include <string>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"

namespace ros2_pulse::core {

/// One flush window reconstructed from an on-disk probe log (the inverse of formatWindow).
/// Only the rate fields of sTopicStat are recoverable from the log; counts stay zero, and
/// recv_endpoint_seen is set when a RECV line was present.
struct sLogWindow {
    long long ts_ns{0};
    double window_s{0.0};
    std::vector<sTopicStat> stats;
    std::vector<std::string> nodes;
};

/// @brief Parse a probe log (any number of windows) back into structured form.
///
/// Consumes the exact grammar formatWindow() emits — `# ts_ns=... window_s=...` headers,
/// TOPIC / PUB / RECV / NODE lines — and merges the per-line rates into one sTopicStat per
/// topic per window. Unknown lines (e.g. WARN) are ignored, so pulse-check re-evaluates from
/// the raw rates rather than trusting probe-side warnings. Pure; used by the pulse-check CLI
/// and unit-tested round-trip against formatWindow.
auto parseLog(const std::string& text) -> std::vector<sLogWindow>;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__LOG_READER_HPP_

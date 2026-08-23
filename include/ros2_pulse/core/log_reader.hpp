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
/// Consumes BOTH on-disk formats, sniffed per line so mixed files work:
///  - text: the exact grammar formatWindow() emits, `# ts_ns=... window_s=...` headers,
///    TOPIC / PUB / RECV / JITTER / NODE lines, merged into one sTopicStat per topic per
///    window;
///  - jsonl (ROADMAP R6): a '{' first byte is one self-contained formatWindowJsonl() record,
///    no text line can start with '{' (it is not a valid ROS name character), so the sniff
///    cannot misfire. pulse-check therefore gates jsonl logs identically: picking the
///    exporter-friendly format never costs CI/watchdog gating.
///
/// Unknown text lines (e.g. WARN) and unknown jsonl keys (e.g. "warns") are ignored, so
/// pulse-check re-evaluates from the raw rates rather than trusting probe-side warnings, and
/// a newer probe's additive fields don't break an older reader. Malformed jsonl lines (a
/// truncated tail after a crash) are skipped, one torn record costs one window, like a torn
/// text block. Pure; used by the pulse-check CLI and unit-tested round-trip against both
/// emitters.
auto parseLog(const std::string& text) -> std::vector<sLogWindow>;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__LOG_READER_HPP_

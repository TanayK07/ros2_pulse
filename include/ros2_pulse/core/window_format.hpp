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
/// TOPIC <topic> <pub_inter_hz to 6dp>          // when TopicRegistry::shouldEmitTopic(stat, emit_idle)
/// RECV <topic> inter=<recv_inter_hz to 6dp> intra=<recv_intra_hz to 6dp>  // when any recv count>0
/// NODE <node>
/// WARN ...                                     // one line per entry of @p warnings, verbatim
/// <blank line>
/// @endcode
///
/// @p emit_idle is the ROS_PULSE_EMIT_IDLE opt-in: fully-idle topics are suppressed unless true
/// (see KNOWN_ISSUES.md #7 / docs/issues/issue-7-idle-topic-line.md).
///
/// @p warnings are pre-rendered alert lines (see evaluateRateSpec in rate_spec.hpp, ROADMAP R1);
/// they are emitted verbatim after the NODE lines so existing TOPIC/PUB/RECV/NODE parsers are
/// unaffected. Empty by default — the output is then byte-identical to the pre-R1 format.
///
/// Pure (no ROS / tracetools / I/O dependency) so the probe can build the whole window once and
/// write it with a single call, and so the exact format is unit-testable.
auto formatWindow(const std::vector<sTopicStat>& stats, const std::vector<std::string>& nodes,
                  long long ts_ns, double window_s, bool emit_idle,
                  const std::vector<std::string>& warnings = {}) -> std::string;

/// @brief Per-process default output path: `$TMPDIR/topic_freq.<pid>.log`, `/tmp` fallback.
///
/// Embedding the pid stops every LD_PRELOADed process from appending to one shared file. Kept as
/// a pure function of its arguments so the derivation is testable without spawning a process —
/// the probe passes `getenv("TMPDIR")` as @p tmpdir. Only an absolute @p tmpdir is honoured
/// (trailing slashes normalized); anything else falls back to `/tmp`, because the probe runs
/// inside arbitrary processes whose cwd is unknown.
auto defaultOutputPath(long pid, const char* tmpdir = nullptr) -> std::string;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__WINDOW_FORMAT_HPP_

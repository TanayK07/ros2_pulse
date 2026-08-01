// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "ros2_pulse/core/window_format.hpp"

#include <cstdio>
#include <string>

namespace ros2_pulse::core {

namespace {

// Format a single value with a printf format into a std::string. Keeps the on-disk output
// byte-identical to the previous fprintf-based emitter (same conversions, same precision).
template <typename T>
auto sprintfStr(const char* fmt, T value) -> std::string {
    char buf[64];
    const int n = std::snprintf(buf, sizeof(buf), fmt, value);
    if (n <= 0) {
        return {};
    }
    return std::string(buf, static_cast<size_t>(n));
}

}  // namespace

auto formatWindow(const std::vector<sTopicStat>& stats, const std::vector<std::string>& nodes,
                  long long ts_ns, double window_s, bool emit_idle,
                  const std::vector<std::string>& warnings) -> std::string {
    (void)warnings;
    std::string out;
    // Reserve a rough upper bound so the common window is built without reallocating.
    out.reserve(64 + stats.size() * 96 + nodes.size() * 32);

    out += "# ts_ns=";
    out += sprintfStr("%lld", ts_ns);
    out += " window_s=";
    out += sprintfStr("%.3f", window_s);
    out += '\n';

    for (const auto& s : stats) {
        // Publish-side line, reading the genuine publish counter (split buckets, issue #1). The
        // emit decision — incl. the idle-topic suppression gated by emit_idle (issue #7) — lives
        // in the core registry so it has one home and stays unit-testable.
        if (TopicRegistry::shouldEmitTopic(s, emit_idle)) {
            out += "TOPIC ";
            out += s.topic;
            out += ' ';
            out += sprintfStr("%.6f", s.pub_inter_hz);
            out += '\n';
        }
        // Publish-side intra line (jazzy+: the rclcpp_intra_publish tracepoint) — ADDITIVE,
        // like RECV, so legacy TOPIC parsers stay valid. Emitted only when intra publishes
        // happened this window (impossible on humble: the tracepoint doesn't exist there).
        if (s.pub_intra_count > 0) {
            out += "PUB ";
            out += s.topic;
            out += " inter=";
            out += sprintfStr("%.6f", s.pub_inter_hz);
            out += " intra=";
            out += sprintfStr("%.6f", s.pub_intra_hz);
            out += '\n';
        }
        // Additive receive-side line incl. intra-process, independent of the publish counter so
        // a same-process pub+sub is not double-counted. Proven receive endpoints emit an
        // explicit zero line when idle — stall visibility (KNOWN_ISSUES #12).
        if (TopicRegistry::shouldEmitRecv(s)) {
            out += "RECV ";
            out += s.topic;
            out += " inter=";
            out += sprintfStr("%.6f", s.recv_inter_hz);
            out += " intra=";
            out += sprintfStr("%.6f", s.recv_intra_hz);
            out += '\n';
        }
    }

    for (const auto& n : nodes) {
        out += "NODE ";
        out += n;
        out += '\n';
    }

    out += '\n';  // blank line closes the window block
    return out;
}

auto defaultOutputPath(long pid) -> std::string {
    return "/root/ssd2tb/logs/topic_freq." + std::to_string(pid) + ".log";
}

}  // namespace ros2_pulse::core

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
                  long long ts_ns, double window_s) -> std::string {
    std::string out;
    // Reserve a rough upper bound so the common window is built without reallocating.
    out.reserve(64 + stats.size() * 96 + nodes.size() * 32);

    out += "# ts_ns=";
    out += sprintfStr("%lld", ts_ns);
    out += " window_s=";
    out += sprintfStr("%.3f", window_s);
    out += '\n';

    for (const auto& s : stats) {
        // Publish-side line, reading the genuine publish counter (split buckets, issue #1).
        // Idle known topics still emit a zero line (see KNOWN_ISSUES.md #7).
        if (s.pub_inter_count > 0 || (s.recv_inter_count == 0 && s.recv_intra_count == 0)) {
            out += "TOPIC ";
            out += s.topic;
            out += ' ';
            out += sprintfStr("%.6f", s.pub_inter_hz);
            out += '\n';
        }
        // Additive receive-side line incl. intra-process, independent of the publish counter so
        // a same-process pub+sub is not double-counted.
        if (s.recv_inter_count > 0 || s.recv_intra_count > 0) {
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

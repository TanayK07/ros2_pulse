// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#include "ros2_pulse/core/log_reader.hpp"

#include <cstdio>

namespace ros2_pulse::core {

namespace {

// The topic named on a TOPIC/PUB/RECV line; sTopicStat entries merge per topic per window
// because one topic can carry up to three lines (TOPIC + PUB + RECV).
auto statFor(sLogWindow& win, const std::string& topic) -> sTopicStat& {
    for (auto& s : win.stats) {
        if (s.topic == topic) {
            return s;
        }
    }
    win.stats.emplace_back();
    win.stats.back().topic = topic;
    return win.stats.back();
}

}  // namespace

auto parseLog(const std::string& text) -> std::vector<sLogWindow> {
    std::vector<sLogWindow> windows;
    sLogWindow* cur = nullptr;
    char name[512];
    size_t pos = 0;
    while (pos <= text.size()) {
        const auto nl = text.find('\n', pos);
        const std::string line =
            text.substr(pos, (nl == std::string::npos ? text.size() : nl) - pos);
        pos = nl == std::string::npos ? text.size() + 1 : nl + 1;
        if (line.empty()) {
            continue;
        }

        long long ts = 0;
        double a = 0.0;
        double b = 0.0;
        if (std::sscanf(line.c_str(), "# ts_ns=%lld window_s=%lf", &ts, &a) == 2) {
            windows.emplace_back();
            cur = &windows.back();
            cur->ts_ns = ts;
            cur->window_s = a;
            continue;
        }
        if (cur == nullptr) {
            continue;  // pre-header noise: not a probe log line
        }
        if (std::sscanf(line.c_str(), "TOPIC %511s %lf", name, &a) == 2) {
            statFor(*cur, name).pub_inter_hz = a;
        } else if (std::sscanf(line.c_str(), "PUB %511s inter=%lf intra=%lf", name, &a, &b) == 3) {
            auto& s = statFor(*cur, name);
            s.pub_inter_hz = a;
            s.pub_intra_hz = b;
        } else if (std::sscanf(line.c_str(), "RECV %511s inter=%lf intra=%lf", name, &a, &b) ==
                   3) {
            auto& s = statFor(*cur, name);
            s.recv_inter_hz = a;
            s.recv_intra_hz = b;
            s.recv_endpoint_seen = true;
        } else if (std::sscanf(line.c_str(), "NODE %511s", name) == 1) {
            cur->nodes.emplace_back(name);
        }
        // anything else (WARN lines, future additions) is deliberately skipped: pulse-check
        // re-derives warnings from the raw rates instead of trusting probe-side output.
    }
    return windows;
}

}  // namespace ros2_pulse::core

// Copyright 2026 ros2_pulse contributors
//
// See topic_statistic_mapper.hpp for the mapping contract.

#include "ros2_pulse/core/topic_statistic_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace ros2_pulse::core {

namespace {

auto toCount(double hz, double window_s) -> std::int32_t {
    const double n = std::round(hz * window_s);
    if (n <= 0.0) return 0;
    if (n >= static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
        return std::numeric_limits<std::int32_t>::max();
    }
    return static_cast<std::int32_t>(n);
}

auto sample(eTopicStatisticType type, const std::string& node, const std::string& topic,
            double hz, double window_s, bool has_max_dt, double max_dt_ms)
    -> sTopicStatisticSample {
    sTopicStatisticSample s;
    s.type = type;
    s.node_name = node;
    s.topic_name = topic;
    s.window_count = toCount(hz, window_s);
    s.mean_ns = std::llround(1e9 / hz);
    s.has_max = has_max_dt;
    s.max_ns = has_max_dt ? std::llround(max_dt_ms * 1e6) : 0;
    return s;
}

}  // namespace

auto mapWindowTopicStatistics(const sLogWindow& win) -> sTopicStatisticsBatch {
    sTopicStatisticsBatch out;
    out.timestamp_ns = win.ts_ns;

    const bool attributable = win.nodes.size() == 1;
    for (const auto& s : win.stats) {
        const double pub_hz = std::max(s.pub_inter_hz, s.pub_intra_hz);
        const double recv_hz = std::max(s.recv_inter_hz, s.recv_intra_hz);
        if (pub_hz <= 0.0 && recv_hz <= 0.0) continue;
        if (!attributable) {
            out.ambiguous_node = true;
            continue;
        }
        const std::string& node = win.nodes.front();
        if (pub_hz > 0.0) {
            out.statistics.push_back(sample(eTopicStatisticType::PublishedPeriod, node, s.topic,
                                            pub_hz, win.window_s, s.has_pub_max_dt,
                                            s.pub_max_dt_ms));
        }
        if (recv_hz > 0.0) {
            out.statistics.push_back(sample(eTopicStatisticType::ReceivedPeriod, node, s.topic,
                                            recv_hz, win.window_s, s.has_recv_max_dt,
                                            s.recv_max_dt_ms));
        }
    }
    return out;
}

}  // namespace ros2_pulse::core

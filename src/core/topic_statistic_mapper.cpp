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

auto msToNs(double ms) -> long long { return std::llround(ms * 1e6); }

// A subscription's own interval distribution, when the probe measured one: the mean of the
// observed intervals (what graph-monitor's shim averages) and their minimum. max stays the
// gap, which also folds the interval still open at the window's end.
void fillReceived(const sEndpointStat& e, sTopicStatisticSample& s) {
    if (!e.has_recv_dt || e.recv_dt.count == 0) return;
    s.mean_ns = msToNs(e.recv_dt.mean_ms);
    s.has_min = true;
    s.min_ns = msToNs(e.recv_dt.min_ms);
}

auto takeAge(const sEndpointStat& e) -> sTopicStatisticSample {
    sTopicStatisticSample s;
    s.type = eTopicStatisticType::TakeAge;
    s.node_name = e.node;
    s.topic_name = e.topic;
    const auto n = e.recv_age.count;
    s.window_count = n >= static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())
                         ? std::numeric_limits<std::int32_t>::max()
                         : static_cast<std::int32_t>(n);
    s.mean_ns = msToNs(e.recv_age.mean_ms);
    s.has_min = true;
    s.min_ns = msToNs(e.recv_age.min_ms);
    s.has_max = true;
    s.max_ns = msToNs(e.recv_age.max_ms);
    return s;
}

// One (node, topic, type) key while the window's endpoints are folded into it. Several handles
// can share a key (one node subscribing twice, or publishing twice); graph-monitor keeps one
// statistic per key, so the key carries its SLOWEST handle, and a stalled handle (0 Hz, a proven
// subscription that went quiet) makes the whole key dead.
struct sKeyed {
    const sEndpointStat* slowest{nullptr};
    double hz{0.0};
    bool dead{false};
};

void mapEndpoints(const sLogWindow& win, sTopicStatisticsBatch& out) {
    std::vector<sKeyed> keys;
    keys.reserve(win.endpoints.size());
    auto sameKey = [](const sEndpointStat& a, const sEndpointStat& b) {
        return a.side == b.side && a.node == b.node && a.topic == b.topic;
    };
    for (const auto& e : win.endpoints) {
        // An endpoint of a node the probe never saw initialize has no name graph-monitor could
        // match; the probe's own infrastructure filter (/rosout, /parameter_events, ...) is
        // re-applied for logs written by a probe with a different filter.
        if (e.node.empty() || TopicRegistry::shouldFilter(e.topic)) continue;
        const double hz = std::max(e.inter_hz, e.intra_hz);
        auto it = std::find_if(keys.begin(), keys.end(),
                               [&](const sKeyed& k) { return sameKey(*k.slowest, e); });
        if (it == keys.end()) {
            keys.push_back(sKeyed{&e, hz, hz <= 0.0});
            continue;
        }
        if (hz <= 0.0) {
            it->dead = true;
        } else if (hz < it->hz) {
            it->slowest = &e;
            it->hz = hz;
        }
    }
    for (const auto& k : keys) {
        if (k.dead) continue;
        const sEndpointStat& e = *k.slowest;
        const bool measured_dt = e.has_recv_dt && e.recv_dt.count > 0;
        // A partial window's count / window_s is diluted by the lead-in before traffic started
        // (or cut short at exit), so 1 / rate would read as a slow period. Only a measured
        // interval mean is sent then; the age does not depend on the window length.
        if (e.side == eEndpointSide::kPub) {
            if (win.partial) continue;
            out.statistics.push_back(sample(eTopicStatisticType::PublishedPeriod, e.node, e.topic,
                                            k.hz, win.window_s, e.has_max_dt, e.max_dt_ms));
            continue;
        }
        if (!win.partial || measured_dt) {
            out.statistics.push_back(sample(eTopicStatisticType::ReceivedPeriod, e.node, e.topic,
                                            k.hz, win.window_s, e.has_max_dt, e.max_dt_ms));
            fillReceived(e, out.statistics.back());
        }
        if (e.has_recv_age && e.recv_age.count > 0) out.statistics.push_back(takeAge(e));
    }
}

// Older probe, no per-endpoint split: the per-process topic sums can only be attributed when
// the process has exactly one active node. Such a log never marks a window partial, but a
// partial one is skipped whole here: it has no interval mean to fall back on.
void mapTopicsSingleNode(const sLogWindow& win, sTopicStatisticsBatch& out) {
    if (win.partial) return;
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
}

}  // namespace

auto mapWindowTopicStatistics(const sLogWindow& win) -> sTopicStatisticsBatch {
    sTopicStatisticsBatch out;
    out.timestamp_ns = win.ts_ns;
    if (!win.endpoints.empty()) {
        mapEndpoints(win, out);
    } else {
        mapTopicsSingleNode(win, out);
    }
    return out;
}

}  // namespace ros2_pulse::core

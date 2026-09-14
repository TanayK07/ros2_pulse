// Copyright 2026 ros2_pulse contributors
//
// See metrics_mapper.hpp for the mapping contract.

#include "ros2_pulse/core/metrics_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace ros2_pulse::core {

namespace {

auto sourceName(const sLogWindow& win, const sMapperOptions& opts) -> std::string {
    if (!opts.source_name.empty()) return opts.source_name;
    if (win.nodes.size() == 1) return win.nodes.front();
    return "ros2_pulse";
}

}  // namespace

auto mapWindow(const sLogWindow& win, const sMapperOptions& opts) -> std::vector<sMetricSample> {
    std::vector<sMetricSample> out;
    out.reserve(win.stats.size());
    const std::string source = sourceName(win, opts);
    const long long window_ns = std::llround(win.window_s * 1e9);

    for (const auto& s : win.stats) {
        sMetricSample m;
        m.measurement_source_name = source;
        m.metrics_source = s.topic;
        m.window_start_ns = win.ts_ns - window_ns;
        m.window_stop_ns = win.ts_ns;

        // Measure where the built-in statistics measure (the subscription) when this process has
        // one; a pure publisher process reports its publish rate instead of vanishing.
        const bool recv_side =
            s.recv_endpoint_seen || s.recv_inter_hz > 0.0 || s.recv_intra_hz > 0.0;
        const double hz = recv_side ? std::max(s.recv_inter_hz, s.recv_intra_hz)
                                    : std::max(s.pub_inter_hz, s.pub_intra_hz);
        m.sample_count = std::round(hz * win.window_s);

        if (opts.unit == eRateUnit::Period_ms) {
            m.unit = "ms";
            m.has_average = hz > 0.0;
            m.average = m.has_average ? 1000.0 / hz : 0.0;
            m.has_maximum = recv_side && s.has_recv_max_dt;
            m.maximum = m.has_maximum ? s.recv_max_dt_ms : 0.0;
        } else {
            m.unit = "Hz";
            m.has_average = true;
            m.average = hz;
        }
        out.push_back(std::move(m));
    }
    return out;
}

}  // namespace ros2_pulse::core

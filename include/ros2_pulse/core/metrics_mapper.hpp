// Copyright 2026 ros2_pulse contributors
//
// Maps one probe window onto statistics_msgs-shaped samples for pulse_bridge, the optional
// sidecar that republishes the jsonl log on /statistics. Pure core: no ROS types, so the mapping
// is unit-tested in the standalone lane and the node is a thin struct-to-message copy.
//
// Semantics follow rclcpp's built-in topic statistics so existing /statistics consumers read
// pulse windows unchanged: one sample per topic per window, `unit` "ms", AVERAGE = message
// period, SAMPLE_COUNT = messages in the window, MAXIMUM = largest inter-arrival gap when the
// probe measured one. Rate is taken at the subscription when this process has one (what the
// built-in statistics measure) and at the publisher otherwise. Inter and intra process paths
// report the busier of the two, never the sum: one publish can hit both tracepoints.

#ifndef ROS2_PULSE__CORE__METRICS_MAPPER_HPP_
#define ROS2_PULSE__CORE__METRICS_MAPPER_HPP_

#include <string>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"

namespace ros2_pulse::core {

enum class eRateUnit {
    Period_ms,  // AVERAGE = 1000 / Hz, the built-in topic statistics convention
    Rate_hz,    // AVERAGE = Hz directly
};

struct sMapperOptions {
    eRateUnit unit{eRateUnit::Period_ms};
    // measurement_source_name override. Empty: the window's single node name, else "ros2_pulse".
    std::string source_name;
};

/// One /statistics sample, field for field what MetricsMessage carries.
struct sMetricSample {
    std::string measurement_source_name;
    std::string metrics_source;  // topic name
    std::string unit;            // "ms" or "Hz"
    long long window_start_ns{0};
    long long window_stop_ns{0};
    bool has_average{false};
    double average{0.0};
    bool has_maximum{false};
    double maximum{0.0};
    double sample_count{0.0};
};

auto mapWindow(const sLogWindow& win, const sMapperOptions& opts) -> std::vector<sMetricSample>;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__METRICS_MAPPER_HPP_

// Copyright 2026 ros2_pulse contributors
//
// Maps one probe window onto statistics_msgs-shaped samples for pulse_bridge, the optional
// sidecar that republishes the jsonl log on /statistics. Pure core: no ROS types, so the mapping
// is unit-tested in the standalone lane and the node is a thin struct-to-message copy.
//
// Semantics follow rclcpp's built-in topic statistics (removed in rolling by rclcpp#3203) so
// existing /statistics consumers read pulse windows unchanged: `unit` "ms", AVERAGE = message
// period, SAMPLE_COUNT = messages in the window, and, when the probe tracked gaps on the
// receive side, MAXIMUM = largest inter-arrival gap, MINIMUM = smallest one, STDDEV = standard
// deviation of the intervals. Rate is taken at the subscription when this process has one (what
// the built-in statistics measure) and at the publisher otherwise. Inter and intra process paths
// report the busier of the two, never the sum: one publish can hit both tracepoints.
//
// Naming: by default (eMetricsSource::Topic) one message_period sample per topic, with the topic
// in metrics_source. eMetricsSource::Metric reproduces rclcpp's layout exactly: metrics_source is
// the metric name ("message_period", "message_age") and the topic is not carried, as upstream;
// that layout also has room for the message_age sample (publish-to-take latency at rmw_take).

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

enum class eMetricsSource {
    Topic,   // metrics_source = topic name, message_period only (pulse_bridge default)
    Metric,  // metrics_source = "message_period" / "message_age", rclcpp's own layout
};

struct sMapperOptions {
    eRateUnit unit{eRateUnit::Period_ms};
    eMetricsSource metrics_source{eMetricsSource::Topic};
    // measurement_source_name override. Empty: the window's single node name, else "ros2_pulse".
    std::string source_name;
};

/// One /statistics sample, field for field what MetricsMessage carries.
struct sMetricSample {
    std::string measurement_source_name;
    std::string metrics_source;  // topic name, or the metric name in eMetricsSource::Metric
    std::string unit;            // "ms" or "Hz"
    long long window_start_ns{0};
    long long window_stop_ns{0};
    bool has_average{false};
    double average{0.0};
    bool has_maximum{false};
    double maximum{0.0};
    bool has_minimum{false};
    double minimum{0.0};
    bool has_stddev{false};
    double stddev{0.0};
    double sample_count{0.0};
};

auto mapWindow(const sLogWindow& win, const sMapperOptions& opts) -> std::vector<sMetricSample>;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__METRICS_MAPPER_HPP_

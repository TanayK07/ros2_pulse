// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// Maps one probe window onto rosgraph_monitor_msgs/TopicStatistics-shaped data for pulse_bridge,
// so ros-tooling/graph-monitor can use the probe as its topic-rate source
// (ros-tooling/graph-monitor#14). Pure core: no ROS types, unit-tested in the standalone lane.
//
// graph-monitor matches every TopicStatistic to one endpoint by (node_name, topic_name):
// PUBLISHED_PERIOD to a publisher, RECEIVED_PERIOD to a subscription, and checks `mean` against
// the endpoint's QoS deadline. The probe counts per topic per PROCESS, not per endpoint, so a
// window is attributed only when the process has exactly one active node. Several active nodes
// would hand every node the process-wide rate; the window is dropped instead (`ambiguous_node`).
//
// What is filled, and what is not:
//   statistic_type  PUBLISHED_PERIOD when the topic was published in the window, RECEIVED_PERIOD
//                   when it was received; both when the process does both. TAKE_AGE is never
//                   emitted: the probe does not read rmw_take's source_timestamp yet.
//   node_name       the window's single active node, fully qualified (/ns/name), which is the
//                   form graph-monitor builds from the graph.
//   topic_name      the topic as the probe recorded it (resolved, fully qualified).
//   window_count    messages in the window: rate * window length, rounded.
//   mean            1 / rate. Inter and intra paths report the busier one, never the sum.
//   max             the largest inter-arrival gap on that side, only when the probe's gap
//                   tracking measured one (has_max); otherwise left at zero.
//   min             never filled: the probe does not track the smallest gap.
// A side with zero rate is left out rather than reported as an infinite period: graph-monitor
// treats a missing update as stale, which is the right verdict for a dead topic.

#ifndef ROS2_PULSE__CORE__TOPIC_STATISTIC_MAPPER_HPP_
#define ROS2_PULSE__CORE__TOPIC_STATISTIC_MAPPER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"

namespace ros2_pulse::core {

enum class eTopicStatisticType {
    PublishedPeriod,  // rosgraph_monitor_msgs TopicStatistic::PUBLISHED_PERIOD
    ReceivedPeriod,   // rosgraph_monitor_msgs TopicStatistic::RECEIVED_PERIOD
};

/// One TopicStatistic, field for field. Durations in nanoseconds.
struct sTopicStatisticSample {
    eTopicStatisticType type{eTopicStatisticType::PublishedPeriod};
    std::string node_name;
    std::string topic_name;
    std::int32_t window_count{0};
    long long mean_ns{0};
    bool has_max{false};
    long long max_ns{0};
};

/// One TopicStatistics message: every endpoint of the window's process, stamped with the
/// window's end (system time, the clock graph-monitor's own shim stamps with).
struct sTopicStatisticsBatch {
    long long timestamp_ns{0};
    std::vector<sTopicStatisticSample> statistics;
    // True when the window carried traffic but the process had no single active node to
    // attribute it to; statistics is then empty.
    bool ambiguous_node{false};
};

auto mapWindowTopicStatistics(const sLogWindow& win) -> sTopicStatisticsBatch;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__TOPIC_STATISTIC_MAPPER_HPP_

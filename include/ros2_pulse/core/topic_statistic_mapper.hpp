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
// the endpoint's QoS deadline. The probe's per-endpoint split (sLogWindow::endpoints, one entry
// per publisher / subscription handle with its owning node) maps onto that directly, so a
// component container or any multi-node process reports every node. Several handles of one
// node on one topic and side (a node subscribing twice) share one key, and graph-monitor holds
// one statistic per key: the key reports its SLOWEST handle, the one a deadline would catch
// first, and is left out when any of them stalled to 0 Hz. Endpoints whose node the probe never
// saw initialize, and the probe's infrastructure topics (TopicRegistry::shouldFilter: /rosout,
// /parameter_events, ...), are not sent.
//
// A log from an older probe has no per-endpoint split. Its per-process topic sums are then
// attributed only when the process has exactly one active node; several active nodes would hand
// every node the process-wide rate, so the window is dropped instead (`ambiguous_node`).
//
// What is filled, and what is not:
//   statistic_type  PUBLISHED_PERIOD for a publisher, RECEIVED_PERIOD for a subscription. TAKE_AGE
//                   is never emitted: the probe does not read rmw_take's source_timestamp yet.
//   node_name       the endpoint's owning node, fully qualified (/ns/name), which is the form
//                   graph-monitor builds from the graph.
//   topic_name      the topic as the probe recorded it (resolved, fully qualified).
//   window_count    messages in the window: rate * window length, rounded.
//   mean            1 / rate. Inter and intra paths report the busier one, never the sum.
//   max             the endpoint's largest inter-arrival gap, only when the probe's gap tracking
//                   measured one (has_max); otherwise left at zero.
//   min             never filled: the probe does not track the smallest gap.
// A zero-rate endpoint is left out rather than reported as an infinite period: graph-monitor
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

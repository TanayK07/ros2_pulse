// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// The QoS both flood_compare nodes build from their `reliability` / `depth` parameters, so the
// controller and the reference publisher can never disagree on what a combo means.

#ifndef FLOOD_COMPARE_CONTROLLER__QOS_PARAMS_HPP_
#define FLOOD_COMPARE_CONTROLLER__QOS_PARAMS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace flood_compare {

/// KEEP_LAST(depth), volatile, and `reliable` or `best_effort`. Anything else throws, so a typo
/// fails the run instead of silently measuring the default.
inline auto qosFromParams(rclcpp::Node& node) -> rclcpp::QoS {
    const auto reliability = node.declare_parameter<std::string>("reliability", "reliable");
    const auto depth = node.declare_parameter<int64_t>("depth", 10);
    if (depth < 1) {
        throw std::invalid_argument("depth must be >= 1, got " + std::to_string(depth));
    }
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(depth))};
    qos.durability_volatile();
    if (reliability == "reliable") {
        qos.reliable();
    } else if (reliability == "best_effort") {
        qos.best_effort();
    } else {
        throw std::invalid_argument("reliability must be 'reliable' or 'best_effort', got '" +
                                    reliability + "'");
    }
    RCLCPP_INFO(node.get_logger(), "QoS: %s, KEEP_LAST depth %lld, volatile", reliability.c_str(),
                static_cast<long long>(depth));
    return qos;
}

}  // namespace flood_compare

#endif  // FLOOD_COMPARE_CONTROLLER__QOS_PARAMS_HPP_

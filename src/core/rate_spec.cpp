// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#include "ros2_pulse/core/rate_spec.hpp"

namespace ros2_pulse::core {

auto parseRateSpec(const std::string& text, std::string& error) -> std::optional<sRateSpec> {
    (void)text;
    error = "not implemented";
    return std::nullopt;
}

auto evaluateRateSpec(const sRateSpec& spec, const std::vector<sTopicStat>& stats,
                      const std::vector<std::string>& active_nodes,
                      const std::vector<std::string>& known_nodes,
                      bool missing_as_zero) -> std::vector<std::string> {
    (void)spec;
    (void)stats;
    (void)active_nodes;
    (void)known_nodes;
    (void)missing_as_zero;
    return {};
}

}  // namespace ros2_pulse::core

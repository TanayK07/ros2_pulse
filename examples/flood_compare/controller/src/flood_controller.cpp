// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// flood_controller: the subscriber side of the flood test (../README.md). One rclcpp node, one
// /cmd_vel subscription with the QoS from the `reliability` / `depth` parameters, and a trivial
// controller step per callback (integrate a unicycle pose). `work_us` adds a busy wait per
// callback, to make the executor the bottleneck on purpose.
//
// Run it with the probe preloaded (../run.sh does): every callback is counted by ros2_pulse at
// rclcpp's callback_start tracepoint, so the probe's callback rate for /cmd_vel is exactly the
// number of times onTwist() ran. The total printed at exit is the same number, for a cross-check.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "qos_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

class FloodController : public rclcpp::Node {
public:
    FloodController() : rclcpp::Node("flood_controller") {
        const auto topic = declare_parameter<std::string>("topic", "/cmd_vel");
        m_work_us = declare_parameter<int64_t>("work_us", 0);
        m_sub = create_subscription<geometry_msgs::msg::Twist>(
            topic, flood_compare::qosFromParams(*this),
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { onTwist(*msg); });
        RCLCPP_INFO(get_logger(), "subscribed to %s, work_us=%lld", topic.c_str(),
                    static_cast<long long>(m_work_us));
    }

    auto callbacks() const -> uint64_t { return m_callbacks; }
    auto pose() const -> std::string {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "x=%.3f y=%.3f yaw=%.3f", m_x, m_y, m_yaw);
        return buf;
    }

private:
    void onTwist(const geometry_msgs::msg::Twist& cmd) {
        // Nominal 10 ms control period: the numbers are irrelevant, the per-message work is not
        // optimized away because the pose is printed at exit.
        constexpr double kDt = 0.01;
        m_yaw += cmd.angular.z * kDt;
        m_x += cmd.linear.x * std::cos(m_yaw) * kDt;
        m_y += cmd.linear.x * std::sin(m_yaw) * kDt;
        ++m_callbacks;
        if (m_work_us > 0) {
            const auto until =
                std::chrono::steady_clock::now() + std::chrono::microseconds(m_work_us);
            while (std::chrono::steady_clock::now() < until) {
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub;
    int64_t m_work_us = 0;
    uint64_t m_callbacks = 0;
    double m_x = 0.0;
    double m_y = 0.0;
    double m_yaw = 0.0;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FloodController>();
    rclcpp::spin(node);
    // stdout, not the logger: rclcpp is shut down by now (SIGINT from run.sh's timeout).
    std::printf("flood_controller: %llu callbacks, %s\n",
                static_cast<unsigned long long>(node->callbacks()), node->pose().c_str());
    std::fflush(stdout);
    node.reset();
    rclcpp::shutdown();
    return 0;
}

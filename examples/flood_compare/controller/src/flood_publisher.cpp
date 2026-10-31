// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// flood_publisher: a reference /cmd_vel flood for dry runs and CI (../README.md). The real test
// uses the wire-capture side's own generator; this one exists so the controller, the probe and
// compare.py can be checked end to end without it.
//
// Open loop: message n is due at n / rate_hz seconds after start, and each tick of a timer of at
// most 1 kHz publishes whatever is due by then. Nothing feeds back from the subscriber, so the
// target never adapts. publish() itself may take longer than the schedule allows (a reliable
// writer with depth 1 can push back on a slow reader), which is why the number actually sent is
// a recorded result, printed at exit, not an assumption. `duration_s` > 0 stops the node by
// itself after that long; 0 runs until SIGINT. Same `reliability` / `depth` parameters as the
// controller.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "qos_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

// `rate_hz:=200` arrives as an integer, `rate_hz:=200.0` as a double: accept both.
auto declareNumber(rclcpp::Node& node, const std::string& name, double fallback) -> double {
    rcl_interfaces::msg::ParameterDescriptor any_number;
    any_number.dynamic_typing = true;
    const auto& value = node.declare_parameter(name, rclcpp::ParameterValue(fallback), any_number);
    return value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
               ? static_cast<double>(value.get<int64_t>())
               : value.get<double>();
}

class FloodPublisher : public rclcpp::Node {
public:
    FloodPublisher() : rclcpp::Node("flood_publisher") {
        const auto topic = declare_parameter<std::string>("topic", "/cmd_vel");
        m_rate_hz = declareNumber(*this, "rate_hz", 100.0);
        m_duration_s = declareNumber(*this, "duration_s", 0.0);
        if (!(m_rate_hz > 0.0) || m_duration_s < 0.0) {
            throw std::invalid_argument("rate_hz must be > 0 and duration_s >= 0");
        }
        m_pub = create_publisher<geometry_msgs::msg::Twist>(topic,
                                                            flood_compare::qosFromParams(*this));
        const double tick_hz = std::min(m_rate_hz, 1000.0);
        const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / tick_hz));
        m_start = std::chrono::steady_clock::now();
        m_last = m_start;
        m_timer = create_wall_timer(period, [this]() { onTick(); });
        const std::string how_long =
            m_duration_s > 0.0 ? std::to_string(m_duration_s) + " s" : "until stopped";
        RCLCPP_INFO(get_logger(), "publishing %s at %.1f Hz for %s (%.0f Hz timer)",
                    topic.c_str(), m_rate_hz, how_long.c_str(), tick_hz);
    }

    /// The exit line run.sh and the README refer to: target vs actually sent.
    void report() const {
        const double elapsed = std::chrono::duration<double>(m_last - m_start).count();
        const auto target = dueBy(m_last);
        std::printf("flood_publisher: sent %llu of %llu due in %.3f s (target %.1f Hz, "
                    "achieved %.1f Hz)\n",
                    static_cast<unsigned long long>(m_sent),
                    static_cast<unsigned long long>(target), elapsed, m_rate_hz,
                    elapsed > 0.0 ? static_cast<double>(m_sent) / elapsed : 0.0);
        std::fflush(stdout);
    }

private:
    auto dueBy(std::chrono::steady_clock::time_point t) const -> uint64_t {
        double s = std::chrono::duration<double>(t - m_start).count();
        if (m_duration_s > 0.0) {
            s = std::min(s, m_duration_s);
        }
        return static_cast<uint64_t>(std::floor(s * m_rate_hz));
    }

    void onTick() {
        const auto now = std::chrono::steady_clock::now();
        const uint64_t due = dueBy(now);
        while (m_sent < due) {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.5;
            msg.angular.z = 0.1 * static_cast<double>(m_sent % 10);
            m_pub->publish(msg);
            ++m_sent;
        }
        m_last = std::chrono::steady_clock::now();
        if (m_duration_s > 0.0 &&
            std::chrono::duration<double>(m_last - m_start).count() >= m_duration_s) {
            m_timer->cancel();
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    double m_rate_hz = 100.0;
    double m_duration_s = 0.0;
    std::chrono::steady_clock::time_point m_start;
    std::chrono::steady_clock::time_point m_last;
    uint64_t m_sent = 0;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FloodPublisher>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    node->report();
    executor.remove_node(node);
    node.reset();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}

// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// flood_publisher: a reference /cmd_vel flood for dry runs and CI (../README.md). The real test
// uses the wire-capture side's own generator; this one exists so the controller, the probe and
// compare.py can be checked end to end without it, and so a preloaded rclcpp publisher shows what
// the probe's publish column looks like.
//
// `rate_hz` messages per second, sent from a timer of at most 1 kHz (several publishes per tick
// above that, spread by a fractional accumulator so the average rate is exact). Same
// `reliability` / `depth` parameters as the controller.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "qos_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

class FloodPublisher : public rclcpp::Node {
public:
    FloodPublisher() : rclcpp::Node("flood_publisher") {
        const auto topic = declare_parameter<std::string>("topic", "/cmd_vel");
        // `rate_hz:=200` arrives as an integer, `rate_hz:=200.0` as a double: accept both.
        rcl_interfaces::msg::ParameterDescriptor any_number;
        any_number.dynamic_typing = true;
        const auto& rate = declare_parameter("rate_hz", rclcpp::ParameterValue(100.0), any_number);
        const double rate_hz = rate.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                                   ? static_cast<double>(rate.get<int64_t>())
                                   : rate.get<double>();
        if (!(rate_hz > 0.0)) {
            throw std::invalid_argument("rate_hz must be > 0");
        }
        m_pub = create_publisher<geometry_msgs::msg::Twist>(topic,
                                                            flood_compare::qosFromParams(*this));
        const double tick_hz = std::min(rate_hz, 1000.0);
        m_per_tick = rate_hz / tick_hz;
        const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / tick_hz));
        m_timer = create_wall_timer(period, [this]() { onTick(); });
        RCLCPP_INFO(get_logger(), "publishing %s at %.1f Hz (%.0f Hz timer)", topic.c_str(),
                    rate_hz, tick_hz);
    }

private:
    void onTick() {
        m_owed += m_per_tick;
        while (m_owed >= 1.0) {
            m_owed -= 1.0;
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.5;
            msg.angular.z = 0.1 * static_cast<double>(m_seq++ % 10);
            m_pub->publish(msg);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    double m_per_tick = 1.0;
    double m_owed = 0.0;
    uint64_t m_seq = 0;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloodPublisher>());
    rclcpp::shutdown();
    return 0;
}

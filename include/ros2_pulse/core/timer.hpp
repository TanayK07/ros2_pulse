// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef ROS2_PULSE__CORE__TIMER_HPP_
#define ROS2_PULSE__CORE__TIMER_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace ros2_pulse::core {

/// @brief Simple periodic timer that calls a function at a fixed interval on a background thread.
class Timer {
public:
    Timer() = delete;
    Timer(std::function<void(void)> func, std::chrono::milliseconds interval);
    ~Timer();

    void start();  ///< spawn background thread (no-op if running)
    void stop();   ///< interrupt and join (no-op if stopped)
    auto isRunning() const -> bool;

private:
    void runThread();

    std::mutex m_mu;
    std::condition_variable m_cv;
    std::function<void(void)> m_func;
    std::chrono::milliseconds m_interval;
    std::thread m_thread;
    bool m_running = false;
};

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__TIMER_HPP_

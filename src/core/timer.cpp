// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#include "ros2_pulse/core/timer.hpp"

namespace ros2_pulse::core {

Timer::Timer(std::function<void(void)> func, std::chrono::milliseconds interval)
    : m_func(std::move(func)), m_interval(interval) {}

Timer::~Timer() { stop(); }

void Timer::start() {
    if (m_running) {
        return;
    }
    m_running = true;
    m_thread = std::thread(std::bind(&Timer::runThread, this));
}

void Timer::runThread() {
    std::unique_lock<std::mutex> lock(m_mu);
    auto end_time = std::chrono::steady_clock::now() + m_interval;
    while (m_running) {
        end_time += m_interval;
        while (m_running && m_cv.wait_until(lock, end_time) != std::cv_status::timeout) {
        }
        if (m_running) {
            m_func();
        }
    }
}

void Timer::stop() {
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (!m_running) {
            return;
        }
        m_running = false;
    }
    m_cv.notify_all();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

auto Timer::isRunning() const -> bool { return m_running; }

}  // namespace ros2_pulse::core

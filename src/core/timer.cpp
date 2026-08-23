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
    // Absolute-deadline cadence: the first fire lands at exactly ONE interval, and the deadline
    // advances AFTER each callback so cadence never drifts by the callback's own duration.
    // (Advancing it before the first wait made the first fire land at 2x the interval and the
    // first stats window report ~2x Hz, KNOWN_ISSUES #8a.)
    auto deadline = std::chrono::steady_clock::now() + m_interval;
    while (m_running) {
        // The deadline lives on steady_clock (immune to wall-clock jumps), but each wait slice
        // is issued against system_clock: libstdc++ maps a steady_clock wait_until to
        // pthread_cond_clockwait, which gcc-11's libtsan does not intercept, TSan then loses
        // the unlock-during-wait and reports false double-locks/races on m_mu (the sanitizer CI
        // lane runs exactly that toolchain). A system_clock wait uses the intercepted
        // pthread_cond_timedwait. Correctness is unaffected: every wakeup, timeout, notify or
        // wall-clock jump, re-derives the remaining time from the steady deadline, so a jump
        // costs at most an extra loop iteration, never a wrong fire time.
        const auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining > std::chrono::steady_clock::duration::zero()) {
            m_cv.wait_until(lock, std::chrono::system_clock::now() + remaining);
            continue;  // re-check m_running and the steady deadline after any wakeup
        }
        if (m_running) {
            m_func();
        }
        deadline += m_interval;
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

void Timer::forkPrepare() { m_mu.lock(); }

void Timer::forkRelease() { m_mu.unlock(); }

void Timer::forkChildReset() {
    // fork() child: the flush thread does not survive, but the inherited std::thread handle
    // still reports joinable. join() would hang forever (the red state of the issue-10
    // regression test) and destroying/assigning a joinable std::thread calls std::terminate,
    // detaching the stale descriptor is the one safe way to make the handle droppable. Then
    // mark the timer stopped so the next start() arms a fresh thread. Called from the atfork
    // child handler with the timer mutex held (taken in forkPrepare).
    if (m_thread.joinable()) {
        m_thread.detach();
    }
    m_running = false;
}

}  // namespace ros2_pulse::core

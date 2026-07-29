// Copyright 2026 ros2_pulse contributors
//
// Unit tests for core::Timer cadence (KNOWN_ISSUES #8a).
//
// The first window's Hz depends on the timer firing at exactly ONE interval: firing at 2x
// (the off-by-one this guards against) doubles the first window's accumulation time, and firing
// immediately would make it ~zero-length. Registers into the shared test binary (no main()).

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "ros2_pulse/core/timer.hpp"

using ros2_pulse::core::Timer;
using namespace std::chrono;

// Issue #8a: the first callback must arrive after ~one interval — not two (the off-by-one on
// current main), and not immediately (a ~0-length first window would break Hz just as badly).
// Bounds are wide (0.5x..1.6x) so sanitizer-lane scheduling noise cannot flake this.
TEST(Timer, FirstCallbackArrivesWithinOneInterval) {
    constexpr auto kInterval = milliseconds(400);
    std::mutex mu;
    std::condition_variable cv;
    bool fired = false;
    steady_clock::time_point first_fire;

    const auto start = steady_clock::now();
    Timer t(
        [&] {
            std::lock_guard<std::mutex> lk(mu);
            if (!fired) {
                fired = true;
                first_fire = steady_clock::now();
                cv.notify_all();
            }
        },
        kInterval);
    t.start();
    {
        std::unique_lock<std::mutex> lk(mu);
        ASSERT_TRUE(cv.wait_for(lk, seconds(3), [&] { return fired; })) << "timer never fired";
    }
    t.stop();

    const auto elapsed_ms = duration_cast<milliseconds>(first_fire - start).count();
    EXPECT_GE(elapsed_ms, 200) << "first fire too early — a ~0-length first window breaks Hz";
    EXPECT_LE(elapsed_ms, 640) << "first fire late by ~one interval (first-tick off-by-one)";
}

// After the first fire the cadence must stay one-per-interval. A 300 ms timer observed for
// ~1.05 s fires at ~300/600/900 -> exactly 3 times. The off-by-one yields 2 (600/900); an
// immediate-fire pathology yields 4 (0/300/600/900) — both bounds assert.
TEST(Timer, SteadyCadenceAfterFirstFire) {
    constexpr auto kInterval = milliseconds(300);
    std::atomic<int> fires{0};
    Timer t([&] { fires.fetch_add(1, std::memory_order_relaxed); }, kInterval);
    t.start();
    std::this_thread::sleep_for(milliseconds(1050));
    t.stop();

    const int n = fires.load();
    EXPECT_GE(n, 3) << "cadence lost a tick (first-tick off-by-one)";
    EXPECT_LE(n, 4) << "timer fired more often than one-per-interval";
}

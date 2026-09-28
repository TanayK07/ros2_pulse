// Copyright 2026 ros2_pulse contributors
//
// Unit tests for core::Timer cadence (KNOWN_ISSUES #8a).
//
// The first window's Hz depends on the timer firing at exactly ONE interval: firing at 2x
// (the off-by-one this guards against) doubles the first window's accumulation time, and firing
// immediately would make it ~zero-length. Registers into the shared test binary (no main()).
//
// Synchronization here is atomics + sleep-polling on purpose: a condition_variable timed wait
// compiles to pthread_cond_clockwait on this toolchain, which gcc-11's libtsan does not
// intercept, the sanitizer lane would report false double-locks/races inside the test itself.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "ros2_pulse/core/timer.hpp"

using ros2_pulse::core::Timer;
using namespace std::chrono;

// Issue #8a: the first callback must arrive after ~one interval, not two (the off-by-one on
// current main), and not immediately (a ~0-length first window would break Hz just as badly).
// Bounds are wide (0.5x..1.6x) so sanitizer-lane scheduling noise cannot flake this.
TEST(Timer, FirstCallbackArrivesWithinOneInterval) {
    constexpr auto kInterval = milliseconds(400);
    const auto start = steady_clock::now();
    std::atomic<int64_t> first_fire_ms{-1};

    Timer t(
        [&] {
            const auto e = duration_cast<milliseconds>(steady_clock::now() - start).count();
            int64_t expected = -1;
            first_fire_ms.compare_exchange_strong(expected, e);  // record the FIRST fire only
        },
        kInterval);
    t.start();
    for (int i = 0; i < 300 && first_fire_ms.load() < 0; ++i) {  // <= 3 s
        std::this_thread::sleep_for(milliseconds(10));
    }
    t.stop();

    const int64_t elapsed_ms = first_fire_ms.load();
    ASSERT_GE(elapsed_ms, 0) << "timer never fired";
    EXPECT_GE(elapsed_ms, 200) << "first fire too early, a ~0-length first window breaks Hz";
    EXPECT_LE(elapsed_ms, 640) << "first fire late by ~one interval (first-tick off-by-one)";
}

// After the first fire the cadence must stay one-per-interval. A 300 ms timer observed for
// ~1.05 s fires at ~300/600/900 -> exactly 3 times. The off-by-one yields 2 (600/900); an
// immediate-fire pathology yields 4 (0/300/600/900), both bounds assert.
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

// A frozen flush thread (SIGSTOP then SIGCONT, a debugger pause, a VM stall, a flush blocked on
// I/O) must NOT be followed by a back-to-back burst of catch-up fires. The old cadence advanced
// the deadline by exactly one interval per fire, so after a stall of N intervals the timer fired
// N times in a row within microseconds. Each of those flushes is a ~0-length window: the first
// carries the stall's gap, the rest report a near-0 gap and nonsense Hz, and a reader that keeps
// the latest window (pulse-top, pulse-export) sees a ~0 ms gap for a 5 s publisher freeze.
// Here the first callback blocks for 4.5 intervals to put the deadline behind real time, the
// same state a SIGSTOP leaves it in. Every later window must be at least half an interval long.
TEST(Timer, NoCatchUpBurstAfterStall) {
    constexpr auto kInterval = milliseconds(100);
    constexpr int kMaxFires = 16;
    std::atomic<int> n{0};
    std::atomic<int64_t> fire_us[kMaxFires];
    for (auto& f : fire_us) {
        f.store(0);
    }
    const auto start = steady_clock::now();
    Timer t(
        [&] {
            const int i = n.fetch_add(1, std::memory_order_relaxed);
            if (i < kMaxFires) {
                fire_us[i].store(duration_cast<microseconds>(steady_clock::now() - start).count());
            }
            if (i == 0) {
                std::this_thread::sleep_for(milliseconds(450));  // the flush thread is frozen
            }
        },
        kInterval);
    t.start();
    std::this_thread::sleep_for(milliseconds(1000));
    t.stop();

    const int fires = std::min(n.load(), kMaxFires);
    ASSERT_GE(fires, 3) << "timer stopped firing after the stall";
    for (int i = 1; i < fires; ++i) {
        const int64_t spacing_us = fire_us[i].load() - fire_us[i - 1].load();
        EXPECT_GE(spacing_us, 45000) << "catch-up burst: fire " << i << " landed " << spacing_us
                                     << " us after fire " << (i - 1);
    }
    // A burst replays every missed tick: ~4 extra fires on top of the ~5 real-time ones.
    EXPECT_LE(n.load(), 7) << "timer replayed missed ticks";
}

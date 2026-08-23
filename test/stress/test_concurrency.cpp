// Copyright 2026 ros2_pulse contributors
//
// Concurrency stress / soak for the hot path (atomics + shared_mutex + thread-local cache).
// Functional single-thread tests can't surface data races on the resolution maps or on the
// m_id / thread-local interaction, so this drives many topics from many threads with a mix of
// publishes and intra/inter callbacks and asserts EXACT aggregate totals plus a bounded topic
// map. Kept CI-fast by default; every dimension is env-overridable for a longer local soak.
// Designed to run clean under ThreadSanitizer.
//
// Publish-side and receive-side use DISTINCT topic names, so the totals are exact and do NOT
// depend on issue #1 (single-process double counting), this stays green before and after the
// fix PRs.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"

using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::TopicRegistry;

namespace {

const void* H(uintptr_t v) { return reinterpret_cast<const void*>(v); }

auto findTopic(const std::vector<sTopicStat>& v, const std::string& t) -> const sTopicStat* {
    for (const auto& s : v) {
        if (s.topic == t) return &s;
    }
    return nullptr;
}

int envInt(const char* key, int def) {
    const char* v = std::getenv(key);
    if (v == nullptr || *v == '\0') return def;
    try {
        return std::max(1, std::stoi(v));
    } catch (...) {
        return def;
    }
}

// Encode a per-topic handle triple from a base so distinct topics never collide.
const void* pubHandle(int i) { return H(0x100000u + static_cast<uintptr_t>(i)); }
const void* subHandle(int i) { return H(0x200000u + static_cast<uintptr_t>(i)); }
const void* rclSub(int i) { return H(0x300000u + static_cast<uintptr_t>(i)); }
const void* cbHandle(int i) { return H(0x400000u + static_cast<uintptr_t>(i)); }

}  // namespace

TEST(ConcurrencyStress, ManyThreadsManyTopicsExactTotals) {
    const int kThreads = envInt("ROS2_PULSE_STRESS_THREADS", 8);
    const int kTopics = envInt("ROS2_PULSE_STRESS_TOPICS", 6);
    const int kRounds = envInt("ROS2_PULSE_STRESS_ROUNDS", 1500);

    TopicRegistry reg;
    // Pre-declare the whole graph single-threaded (init is the low-frequency path).
    for (int i = 0; i < kTopics; ++i) {
        const std::string pub = "/pub_" + std::to_string(i);
        const std::string recv = "/recv_" + std::to_string(i);
        reg.onPublisherInit(pubHandle(i), nullptr, pub.c_str());
        reg.onSubscriptionInit(subHandle(i), nullptr, recv.c_str());
        reg.onRclcppSubscriptionInit(rclSub(i), subHandle(i));
        reg.onCallbackAdded(cbHandle(i), rclSub(i));
    }

    // Each thread walks every topic every round, doing exactly one publish, one intra receive
    // and one inter receive. So each topic index sees the same, computable total.
    std::vector<std::thread> ts;
    ts.reserve(kThreads);
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([&] {
            for (int r = 0; r < kRounds; ++r) {
                for (int i = 0; i < kTopics; ++i) {
                    reg.onPublish(pubHandle(i));
                    reg.onCallbackStart(cbHandle(i), /*intra=*/true);
                    reg.onCallbackStart(cbHandle(i), /*intra=*/false);
                }
            }
        });
    }
    for (auto& th : ts) th.join();

    const uint64_t expected = static_cast<uint64_t>(kThreads) * static_cast<uint64_t>(kRounds);
    auto snap = reg.snapshot(1.0);

    for (int i = 0; i < kTopics; ++i) {
        const auto* p = findTopic(snap, "/pub_" + std::to_string(i));
        ASSERT_NE(p, nullptr) << "missing /pub_" << i;
        EXPECT_EQ(p->pub_inter_count, expected);

        const auto* rcv = findTopic(snap, "/recv_" + std::to_string(i));
        ASSERT_NE(rcv, nullptr) << "missing /recv_" << i;
        EXPECT_EQ(rcv->recv_inter_count, expected);
        EXPECT_EQ(rcv->recv_intra_count, expected);
    }
}

// ROADMAP R5 (JitterMaxIsRaceFree): many threads hammer
// ONE endpoint per side, so noteArrival's exchange partition and its now > prev guard run under
// real contention, and under TSan in the sanitizer lanes. The guard is what this regresses:
// `now` is sampled before the exchange, so two threads can exchange out of order, and without
// the guard the unsigned subtraction underflows to ~1.8e19 ns and poisons max_dt permanently.
// Hence the two-sided assertion:
//   - a deliberate mid-run stall must be visible (max_dt_ms >= the stall, sleep_for guarantees
//     at least the requested duration, and no other arrival can land inside it), and
//   - max_dt_ms can never exceed the measured wall time of the whole test, the underflow value
//     is ~570 years, so one unguarded reordered pair fails this bound.
TEST(ConcurrencyStress, JitterMaxIsRaceFree) {
    const int kThreads = envInt("ROS2_PULSE_STRESS_THREADS", 8);
    const int kRounds = envInt("ROS2_PULSE_STRESS_ROUNDS", 1500);
    const int kStallMs = envInt("ROS2_PULSE_STRESS_STALL_MS", 25);

    TopicRegistry reg;
    reg.setGapTracking(true);
    reg.onPublisherInit(pubHandle(0), nullptr, "/pub_0");
    reg.onSubscriptionInit(subHandle(0), nullptr, "/recv_0");
    reg.onRclcppSubscriptionInit(rclSub(0), subHandle(0));
    reg.onCallbackAdded(cbHandle(0), rclSub(0));

    // Tight contended bursts on a single publisher handle and a single callback (both intra and
    // inter deliveries, the recv accumulator is transport-merged, so both must feed one series).
    const auto hammer = [&reg, kThreads, kRounds] {
        std::vector<std::thread> ts;
        ts.reserve(kThreads);
        for (int t = 0; t < kThreads; ++t) {
            ts.emplace_back([&reg, kRounds] {
                for (int r = 0; r < kRounds; ++r) {
                    reg.onPublish(pubHandle(0));
                    reg.onCallbackStart(cbHandle(0), /*intra=*/true);
                    reg.onCallbackStart(cbHandle(0), /*intra=*/false);
                }
            });
        }
        for (auto& th : ts) th.join();
    };

    const auto t0 = std::chrono::steady_clock::now();
    hammer();
    std::this_thread::sleep_for(std::chrono::milliseconds(kStallMs));  // the known gap
    hammer();

    // fold_open_gap=false so only intervals measured by noteArrival itself are reported, the
    // snapshot-time fold is a separate mechanism with its own single-threaded tests.
    auto snap = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    const auto* p = findTopic(snap, "/pub_0");
    ASSERT_NE(p, nullptr);
    ASSERT_TRUE(p->has_pub_max_dt);
    EXPECT_GE(p->pub_max_dt_ms, static_cast<double>(kStallMs));
    EXPECT_LE(p->pub_max_dt_ms, elapsed_ms) << "max_dt exceeds test wall time, underflow?";

    const auto* rcv = findTopic(snap, "/recv_0");
    ASSERT_NE(rcv, nullptr);
    ASSERT_TRUE(rcv->has_recv_max_dt);
    EXPECT_GE(rcv->recv_max_dt_ms, static_cast<double>(kStallMs));
    EXPECT_LE(rcv->recv_max_dt_ms, elapsed_ms) << "max_dt exceeds test wall time, underflow?";

    // Exact counts still hold with gap tracking enabled: the two mechanisms share a hot path
    // and must not perturb each other.
    const uint64_t expected = 2ULL * static_cast<uint64_t>(kThreads) * static_cast<uint64_t>(kRounds);
    EXPECT_EQ(p->pub_inter_count, expected);
    EXPECT_EQ(rcv->recv_intra_count, expected);
    EXPECT_EQ(rcv->recv_inter_count, expected);
}

// No unbounded map growth: the set of tracked topics is fixed by the graph, and repeated
// snapshots (which reset counts but must not create/leak topics) keep the topic set constant.
TEST(ConcurrencyStress, TopicSetStaysBounded) {
    const int kTopics = envInt("ROS2_PULSE_STRESS_TOPICS", 6);
    const int kSnapshots = envInt("ROS2_PULSE_STRESS_SNAPSHOTS", 200);

    TopicRegistry reg;
    for (int i = 0; i < kTopics; ++i) {
        const std::string pub = "/pub_" + std::to_string(i);
        const std::string recv = "/recv_" + std::to_string(i);
        reg.onPublisherInit(pubHandle(i), nullptr, pub.c_str());
        reg.onSubscriptionInit(subHandle(i), nullptr, recv.c_str());
        reg.onRclcppSubscriptionInit(rclSub(i), subHandle(i));
        reg.onCallbackAdded(cbHandle(i), rclSub(i));
    }
    // pub_i and recv_i are distinct names -> 2 * kTopics tracked counters.
    const size_t expected_topics = static_cast<size_t>(2 * kTopics);

    // Hammer a lot of traffic through, then take many snapshots. Also feed a stream of
    // never-resolvable callbacks (timers/services), these must not add topics.
    for (int i = 0; i < kTopics; ++i) {
        for (int k = 0; k < 1000; ++k) {
            reg.onPublish(pubHandle(i));
            reg.onCallbackStart(cbHandle(i), true);
        }
    }
    for (int k = 0; k < 1000; ++k) {
        reg.onCallbackStart(H(0xDEAD0000u + static_cast<uintptr_t>(k % 32)), false);  // unresolved
    }

    for (int s = 0; s < kSnapshots; ++s) {
        auto snap = reg.snapshot(1.0);
        EXPECT_EQ(snap.size(), expected_topics) << "topic set changed at snapshot " << s;
    }
}

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
// depend on issue #1 (single-process double counting) — this stays green before and after the
// fix PRs.

#include <gtest/gtest.h>

#include <algorithm>
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
        reg.onPublisherInit(pubHandle(i), pub.c_str());
        reg.onSubscriptionInit(subHandle(i), recv.c_str());
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
        EXPECT_EQ(p->inter_count, expected);

        const auto* rcv = findTopic(snap, "/recv_" + std::to_string(i));
        ASSERT_NE(rcv, nullptr) << "missing /recv_" << i;
        EXPECT_EQ(rcv->inter_count, expected);
        EXPECT_EQ(rcv->intra_count, expected);
    }
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
        reg.onPublisherInit(pubHandle(i), pub.c_str());
        reg.onSubscriptionInit(subHandle(i), recv.c_str());
        reg.onRclcppSubscriptionInit(rclSub(i), subHandle(i));
        reg.onCallbackAdded(cbHandle(i), rclSub(i));
    }
    // pub_i and recv_i are distinct names -> 2 * kTopics tracked counters.
    const size_t expected_topics = static_cast<size_t>(2 * kTopics);

    // Hammer a lot of traffic through, then take many snapshots. Also feed a stream of
    // never-resolvable callbacks (timers/services) — these must not add topics.
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

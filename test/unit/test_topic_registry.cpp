// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the pure-C++ probe core. No ROS / tracetools dependency.

#include <gtest/gtest.h>

#include <thread>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"

using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::TopicRegistry;

namespace {

// fake opaque handles
const void* H(uintptr_t v) { return reinterpret_cast<const void*>(v); }

auto findTopic(const std::vector<sTopicStat>& v, const std::string& t) -> const sTopicStat* {
    for (const auto& s : v) {
        if (s.topic == t) return &s;
    }
    return nullptr;
}

}  // namespace

TEST(TopicRegistry, PublishCountsAndHz) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), "/scan");
    for (int i = 0; i < 100; i++) reg.onPublish(H(0x10));

    auto snap = reg.snapshot(2.0);  // 100 msgs over 2 s -> 50 Hz
    const auto* s = findTopic(snap, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_inter_count, 100u);
    EXPECT_EQ(s->recv_inter_count, 0u);
    EXPECT_EQ(s->recv_intra_count, 0u);
    EXPECT_DOUBLE_EQ(s->pub_inter_hz, 50.0);
}

TEST(TopicRegistry, SnapshotResetsCounts) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), "/scan");
    reg.onPublish(H(0x10));
    reg.snapshot(1.0);
    auto snap2 = reg.snapshot(1.0);
    const auto* s = findTopic(snap2, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_inter_count, 0u);
}

TEST(TopicRegistry, UnknownPublisherIgnored) {
    TopicRegistry reg;
    reg.onPublish(H(0xdead));  // never init'd -> must not crash, no topic
    auto snap = reg.snapshot(1.0);
    EXPECT_TRUE(snap.empty());
}

TEST(TopicRegistry, InterProcessReceiveViaCallback) {
    TopicRegistry reg;
    // chain: sub_handle -> topic ; rclcpp_sub -> sub_handle ; callback -> rclcpp_sub
    reg.onSubscriptionInit(H(0x20), "/img");
    reg.onRclcppSubscriptionInit(H(0x21), H(0x20));
    reg.onCallbackAdded(H(0x22), H(0x21));
    for (int i = 0; i < 30; i++) reg.onCallbackStart(H(0x22), /*intra=*/false);

    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/img");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 30u);
    EXPECT_EQ(s->recv_intra_count, 0u);
    EXPECT_EQ(s->pub_inter_count, 0u);  // pure receiver: no publish-side count
}

TEST(TopicRegistry, IntraProcessReceiveBucketedSeparately) {
    TopicRegistry reg;
    reg.onSubscriptionInit(H(0x20), "/cloud");
    reg.onRclcppSubscriptionInit(H(0x21), H(0x20));
    reg.onCallbackAdded(H(0x22), H(0x21));
    for (int i = 0; i < 10; i++) reg.onCallbackStart(H(0x22), /*intra=*/true);
    for (int i = 0; i < 5; i++) reg.onCallbackStart(H(0x22), /*intra=*/false);

    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/cloud");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_intra_count, 10u);
    EXPECT_EQ(s->recv_inter_count, 5u);
    EXPECT_EQ(s->pub_inter_count, 0u);
}

// Regression for KNOWN_ISSUES.md #1: a publisher and an inter-process subscriber for the SAME topic
// in the SAME process must not share one counter. The publish side is an independent measurement
// from the receive side; N publishes + N inter-receives is a publish rate of N and a receive rate of
// N, NOT a single inter rate of 2N. On the pre-fix code both writers fetch_add the one `inter` field,
// so this collapses to inter_count == 2N.
TEST(TopicRegistry, SameProcessPubAndSubDoNotDoubleCount) {
    TopicRegistry reg;
    // publisher + resolvable subscriber, same topic, one registry (a same-process pub+sub).
    reg.onPublisherInit(H(0x10), "/odom");
    reg.onSubscriptionInit(H(0x20), "/odom");
    reg.onRclcppSubscriptionInit(H(0x21), H(0x20));
    reg.onCallbackAdded(H(0x22), H(0x21));

    constexpr int N = 40;
    for (int i = 0; i < N; i++) reg.onPublish(H(0x10));
    for (int i = 0; i < N; i++) reg.onCallbackStart(H(0x22), /*intra=*/false);

    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/odom");
    ASSERT_NE(s, nullptr);
    // Publish and inter-receive are independent measurements — each exactly N, NOT one bucket of 2N.
    EXPECT_EQ(s->pub_inter_count, static_cast<uint64_t>(N));
    EXPECT_EQ(s->recv_inter_count, static_cast<uint64_t>(N));
    EXPECT_EQ(s->recv_intra_count, 0u);
}

// Publish side and receive side must be fully decoupled: driving only one leaves the other at zero.
TEST(TopicRegistry, PublishSideIndependentOfReceive) {
    {
        TopicRegistry reg;
        reg.onPublisherInit(H(0x10), "/pub_only");
        for (int i = 0; i < 12; i++) reg.onPublish(H(0x10));
        auto snap = reg.snapshot(1.0);
        const auto* s = findTopic(snap, "/pub_only");
        ASSERT_NE(s, nullptr);
        EXPECT_EQ(s->pub_inter_count, 12u);
        EXPECT_EQ(s->recv_inter_count, 0u);  // no callbacks -> nothing received
        EXPECT_EQ(s->recv_intra_count, 0u);
    }
    {
        TopicRegistry reg;
        reg.onSubscriptionInit(H(0x20), "/recv_only");
        reg.onRclcppSubscriptionInit(H(0x21), H(0x20));
        reg.onCallbackAdded(H(0x22), H(0x21));
        for (int i = 0; i < 9; i++) reg.onCallbackStart(H(0x22), /*intra=*/false);
        auto snap = reg.snapshot(1.0);
        const auto* s = findTopic(snap, "/recv_only");
        ASSERT_NE(s, nullptr);
        EXPECT_EQ(s->recv_inter_count, 9u);
        EXPECT_EQ(s->pub_inter_count, 0u);  // no publisher -> no publish-side count
    }
}

// The intra-process ordering bug we hit in the PoC: callback_added fires for the intra
// waitable BEFORE its sub_handle->topic chain is populated. Eager resolution would lose it;
// lazy resolution at callback_start must still bind it.
TEST(TopicRegistry, LazyResolutionWhenInitOutOfOrder) {
    TopicRegistry reg;
    // callback added first, with the chain only partially known
    reg.onCallbackAdded(H(0x22), H(0x21));
    reg.onRclcppSubscriptionInit(H(0x21), H(0x20));
    // a message arrives BEFORE sub_handle->topic is known -> not yet resolvable, must not crash
    reg.onCallbackStart(H(0x22), true);
    // now the final link arrives
    reg.onSubscriptionInit(H(0x20), "/late");
    // subsequent messages must resolve and count
    for (int i = 0; i < 7; i++) reg.onCallbackStart(H(0x22), true);

    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/late");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_intra_count, 7u);  // the 1 early msg before resolution is allowed to be lost
}

// Regression for KNOWN_ISSUES #3. A callback that is not a subscription (timer/service) is never
// linked via onCallbackAdded, so it can never resolve to a topic. It must take the EXCLUSIVE write
// lock AT MOST ONCE no matter how many times it fires. Before the negative cache, every invocation
// fell through to the unique_lock branch -> a write-lock storm on a path sold as lock-light.
TEST(TopicRegistry, UnresolvableCallbackTakesWriteLockAtMostOnce) {
    TopicRegistry reg;
    const void* timer_cb = H(0x99);  // never onCallbackAdded'd -> provably not a subscription
    for (int i = 0; i < 1000; i++) {
        reg.onCallbackStart(timer_cb, /*intra=*/false);
    }
    EXPECT_LE(reg.writeLockResolutions(), 1u);
    // and it must never fabricate a topic
    auto snap = reg.snapshot(1.0);
    EXPECT_TRUE(snap.empty());
}

// Same guarantee under a multi-threaded executor: an unresolvable callback hammered from many
// threads escalates to the write lock at most once PER THREAD (bounded by thread count), never once
// per message. This is the concurrency shape the "no global lock" design promises.
TEST(TopicRegistry, UnresolvableCallbackNoWriteLockStormConcurrent) {
    TopicRegistry reg;
    const void* timer_cb = H(0xabcd);  // not a subscription
    constexpr int kThreads = 8;
    constexpr int kPer = 100000;
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; t++) {
        ts.emplace_back([&] {
            for (int i = 0; i < kPer; i++) reg.onCallbackStart(timer_cb, /*intra=*/false);
        });
    }
    for (auto& th : ts) th.join();

    EXPECT_LE(reg.writeLockResolutions(), static_cast<uint64_t>(kThreads));
    auto snap = reg.snapshot(1.0);
    EXPECT_TRUE(snap.empty());
}

TEST(TopicRegistry, FilteredTopicsExcluded) {
    TopicRegistry reg;
    EXPECT_TRUE(TopicRegistry::shouldFilter("/rosout"));
    EXPECT_TRUE(TopicRegistry::shouldFilter("/parameter_events"));
    EXPECT_TRUE(TopicRegistry::shouldFilter("/diagnostics"));
    EXPECT_FALSE(TopicRegistry::shouldFilter("/scan"));

    reg.onPublisherInit(H(0x10), "/rosout");
    reg.onPublish(H(0x10));
    auto snap = reg.snapshot(1.0);
    EXPECT_EQ(findTopic(snap, "/rosout"), nullptr);
}

TEST(TopicRegistry, NodeTracking) {
    TopicRegistry reg;
    reg.onNodeInit("talker", "");
    reg.onNodeInit("planner", "/nav");
    auto nodes = reg.activeNodes();
    ASSERT_EQ(nodes.size(), 2u);
    EXPECT_EQ(nodes[0], "/talker");
    EXPECT_EQ(nodes[1], "/nav/planner");
}

TEST(TopicRegistry, ConcurrentPublishExactTotal) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), "/hot");
    constexpr int kThreads = 8;
    constexpr int kPer = 100000;
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; t++) {
        ts.emplace_back([&] {
            for (int i = 0; i < kPer; i++) reg.onPublish(H(0x10));
        });
    }
    for (auto& th : ts) th.join();

    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/hot");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_inter_count, static_cast<uint64_t>(kThreads) * kPer);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

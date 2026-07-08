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
    EXPECT_EQ(s->inter_count, 100u);
    EXPECT_EQ(s->intra_count, 0u);
    EXPECT_DOUBLE_EQ(s->inter_hz, 50.0);
}

TEST(TopicRegistry, SnapshotResetsCounts) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), "/scan");
    reg.onPublish(H(0x10));
    reg.snapshot(1.0);
    auto snap2 = reg.snapshot(1.0);
    const auto* s = findTopic(snap2, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->inter_count, 0u);
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
    EXPECT_EQ(s->inter_count, 30u);
    EXPECT_EQ(s->intra_count, 0u);
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
    EXPECT_EQ(s->intra_count, 10u);
    EXPECT_EQ(s->inter_count, 5u);
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
    EXPECT_EQ(s->intra_count, 7u);  // the 1 early msg before resolution is allowed to be lost
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
    EXPECT_EQ(s->inter_count, static_cast<uint64_t>(kThreads) * kPer);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

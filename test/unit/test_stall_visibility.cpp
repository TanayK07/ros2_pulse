// Copyright 2026 ros2_pulse contributors
//
// Unit tests for stalled-subscription visibility (KNOWN_ISSUES #12): a topic whose subscription
// has delivered at least once must keep being reported when traffic stops, "active then
// stopped" is signal, unlike "declared but never active" (issue #7 noise). Registers into the
// shared test binary (no main()).

#include <gtest/gtest.h>

#include <string>
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

auto recvStat(uint64_t recv_inter, uint64_t recv_intra, bool endpoint_seen) -> sTopicStat {
    sTopicStat s;
    s.topic = "/x";
    s.recv_inter_count = recv_inter;
    s.recv_intra_count = recv_intra;
    s.recv_endpoint_seen = endpoint_seen;
    return s;
}

}  // namespace

// A subscription that delivered once and then went quiet: the zero-traffic window must still
// carry the topic, flagged as a known receive endpoint, so the emitter can print a 0.0 line.
TEST(StallVisibility, StalledSubscriptionStillReported) {
    TopicRegistry reg;
    reg.onSubscriptionInit(H(0x20), nullptr, "/scan");
    reg.onRclcppSubscriptionInit(H(0x21), H(0x20));
    reg.onCallbackAdded(H(0x22), H(0x21));
    for (int i = 0; i < 5; i++) reg.onCallbackStart(H(0x22), /*intra=*/false);
    reg.snapshot(1.0);  // window with traffic

    auto stalled = reg.snapshot(1.0);  // upstream died: zero traffic
    const auto* s = findTopic(stalled, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 0u);
    EXPECT_TRUE(s->recv_endpoint_seen) << "resolved subscription lost across windows";
}

// A subscription that never delivered a single message is NOT a proven endpoint, reporting it
// at 0.0 would conflate a mis-wired/never-active topic with a genuine stall (issue #7 rationale).
TEST(StallVisibility, NeverDeliveredSubscriptionNotMarked) {
    TopicRegistry reg;
    reg.onSubscriptionInit(H(0x20), nullptr, "/never");
    reg.onRclcppSubscriptionInit(H(0x21), H(0x20));
    reg.onCallbackAdded(H(0x22), H(0x21));

    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/never");
    ASSERT_NE(s, nullptr);
    EXPECT_FALSE(s->recv_endpoint_seen);
}

// The emit policy, pinned as a pure predicate (one home, like shouldEmitTopic).
TEST(ShouldEmitRecv, TrafficAlwaysEmits) {
    EXPECT_TRUE(TopicRegistry::shouldEmitRecv(recvStat(3, 0, false)));
    EXPECT_TRUE(TopicRegistry::shouldEmitRecv(recvStat(0, 7, true)));
}

TEST(ShouldEmitRecv, ProvenEndpointEmitsZeroLineWhenIdle) {
    EXPECT_TRUE(TopicRegistry::shouldEmitRecv(recvStat(0, 0, true)));
}

TEST(ShouldEmitRecv, NeverActiveStaysSuppressed) {
    EXPECT_FALSE(TopicRegistry::shouldEmitRecv(recvStat(0, 0, false)));
}

// Copyright 2026 ros2_pulse contributors
//
// Unit tests for publish-side intra-process counting (ROADMAP R3: the rclcpp_intra_publish
// tracepoint, jazzy+). Registers into the shared test binary (no main()).

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

}  // namespace

// Intra publishes land in their own bucket, never in pub_inter (rate inflation) and never in
// the receive buckets (that's the subscription side's signal).
TEST(IntraPublish, CountedInOwnBucket) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/points");
    for (int i = 0; i < 60; ++i) reg.onIntraPublish(H(0x10));
    for (int i = 0; i < 40; ++i) reg.onPublish(H(0x10));

    auto snap = reg.snapshot(2.0);
    const auto* s = findTopic(snap, "/points");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_intra_count, 60u);
    EXPECT_EQ(s->pub_inter_count, 40u);
    EXPECT_DOUBLE_EQ(s->pub_intra_hz, 30.0);
    EXPECT_EQ(s->recv_inter_count, 0u);
    EXPECT_EQ(s->recv_intra_count, 0u);
}

// Same handle alternating inter/intra publishes through the TLS cache: totals stay exact.
TEST(IntraPublish, ExactTotalsAlternatingTransports) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/mixed");
    for (int i = 0; i < 500; ++i) {
        reg.onPublish(H(0x10));
        reg.onIntraPublish(H(0x10));
    }
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/mixed");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_inter_count, 500u);
    EXPECT_EQ(s->pub_intra_count, 500u);
}

// An intra-only publisher is ACTIVE, it must not be treated as fully idle by the TOPIC-line
// policy (its rate is carried on the additive PUB line; the opt-in idle zero-line must not
// fire for it either).
TEST(IntraPublish, IntraOnlyPublisherIsNotIdle) {
    sTopicStat s;
    s.topic = "/points";
    s.pub_intra_count = 60;
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(s, /*emit_idle=*/false));
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(s, /*emit_idle=*/true));
}

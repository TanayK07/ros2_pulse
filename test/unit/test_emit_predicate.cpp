// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the publish-side TOPIC emit predicate (KNOWN_ISSUES #7).
// Pure-C++ core, no ROS / tracetools dependency. Shares the gtest main() defined in
// test_topic_registry.cpp when the two files are linked together.

#include <gtest/gtest.h>

#include "ros2_pulse/core/topic_registry.hpp"

using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::TopicRegistry;

namespace {

auto makeStat(uint64_t pub_inter, uint64_t recv_inter, uint64_t recv_intra) -> sTopicStat {
    sTopicStat s;
    s.topic = "/x";
    s.pub_inter_count = pub_inter;
    s.recv_inter_count = recv_inter;
    s.recv_intra_count = recv_intra;
    return s;
}

}  // namespace

// A fully-idle topic (no inter- and no intra-process traffic) is the zero-line bug: by default it
// must NOT be emitted, but it IS restored when the operator opts in via ROS_PULSE_EMIT_IDLE=1.
TEST(ShouldEmitTopic, FullyIdleSuppressedByDefault) {
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(makeStat(0, 0, 0), /*emit_idle=*/false));
}

TEST(ShouldEmitTopic, FullyIdleEmittedWhenOptedIn) {
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(0, 0, 0), /*emit_idle=*/true));
}

// A locally-published topic is always emitted, regardless of the idle flag.
TEST(ShouldEmitTopic, ActivePublisherAlwaysEmitted) {
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(10, 0, 0), /*emit_idle=*/false));
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(10, 0, 0), /*emit_idle=*/true));
}

// Publish + receive in the same process (both buckets active): the publish-side line is still
// meaningful and must be emitted.
TEST(ShouldEmitTopic, ActivePubAndRecvEmitted) {
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(10, 10, 5), /*emit_idle=*/false));
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(10, 10, 5), /*emit_idle=*/true));
}

// An intra-receive-only topic must NOT emit the publish-side line (its signal is on the RECV
// line). Independent of the idle flag.
TEST(ShouldEmitTopic, IntraReceiveOnlyPublishLineSuppressed) {
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(makeStat(0, 0, 7), /*emit_idle=*/false));
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(makeStat(0, 0, 7), /*emit_idle=*/true));
}

// Same for an inter-receive-only topic (pure subscriber in this process): the split buckets
// (issue #1) mean receive traffic no longer fabricates a publish-side rate.
TEST(ShouldEmitTopic, InterReceiveOnlyPublishLineSuppressed) {
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(makeStat(0, 9, 0), /*emit_idle=*/false));
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(makeStat(0, 9, 0), /*emit_idle=*/true));
}

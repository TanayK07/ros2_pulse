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

auto makeStat(uint64_t inter, uint64_t intra) -> sTopicStat {
    sTopicStat s;
    s.topic = "/x";
    s.inter_count = inter;
    s.intra_count = intra;
    s.inter_hz = 0.0;
    s.intra_hz = 0.0;
    return s;
}

}  // namespace

// A fully-idle topic (no inter- and no intra-process traffic) is the zero-line bug: by default it
// must NOT be emitted, but it IS restored when the operator opts in via ROS_PULSE_EMIT_IDLE=1.
TEST(ShouldEmitTopic, FullyIdleSuppressedByDefault) {
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(makeStat(0, 0), /*emit_idle=*/false));
}

TEST(ShouldEmitTopic, FullyIdleEmittedWhenOptedIn) {
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(0, 0), /*emit_idle=*/true));
}

// An active inter-process topic is always emitted, regardless of the idle flag.
TEST(ShouldEmitTopic, ActiveInterAlwaysEmitted) {
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(10, 0), /*emit_idle=*/false));
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(10, 0), /*emit_idle=*/true));
}

// A topic with both transports active is always emitted (publish-side line still meaningful).
TEST(ShouldEmitTopic, ActiveBothTransportsEmitted) {
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(10, 5), /*emit_idle=*/false));
    EXPECT_TRUE(TopicRegistry::shouldEmitTopic(makeStat(10, 5), /*emit_idle=*/true));
}

// An intra-only topic must NOT emit the publish-side line (its signal is on the RECV line). This
// preserves the pre-fix behaviour and must be independent of the idle flag.
TEST(ShouldEmitTopic, IntraOnlyPublishLineSuppressed) {
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(makeStat(0, 7), /*emit_idle=*/false));
    EXPECT_FALSE(TopicRegistry::shouldEmitTopic(makeStat(0, 7), /*emit_idle=*/true));
}

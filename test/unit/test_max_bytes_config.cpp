// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the ROS_TOPIC_STATS_MAX_BYTES parser (KNOWN_ISSUES #11). Same noexcept
// contract and style as parsePeriodSeconds (issue #5). Registers into the shared test binary.

#include <gtest/gtest.h>

#include "ros2_pulse/core/env_config.hpp"

using ros2_pulse::core::parseMaxBytes;

namespace {
constexpr unsigned long long kDef = 10ULL * 1024 * 1024;  // the probe's 10 MiB default
}  // namespace

TEST(ParseMaxBytes, ParsesPlainIntegers) {
    EXPECT_EQ(parseMaxBytes("512", kDef), 512ULL);
    EXPECT_EQ(parseMaxBytes("1048576", kDef), 1048576ULL);
}

TEST(ParseMaxBytes, ZeroDisablesRotation) {
    EXPECT_EQ(parseMaxBytes("0", kDef), 0ULL);  // explicit opt-out, NOT the default
}

TEST(ParseMaxBytes, ToleratesSurroundingWhitespace) {
    EXPECT_EQ(parseMaxBytes("  4096 ", kDef), 4096ULL);
}

TEST(ParseMaxBytes, RejectsEmptyOrMissing) {
    EXPECT_EQ(parseMaxBytes(nullptr, kDef), kDef);  // env var unset
    EXPECT_EQ(parseMaxBytes("", kDef), kDef);
    EXPECT_EQ(parseMaxBytes("   ", kDef), kDef);
}

TEST(ParseMaxBytes, RejectsGarbage) {
    EXPECT_EQ(parseMaxBytes("abc", kDef), kDef);
    EXPECT_EQ(parseMaxBytes("512abc", kDef), kDef);  // trailing junk, not a prefix parse
    EXPECT_EQ(parseMaxBytes("1.5", kDef), kDef);     // bytes are integral
    EXPECT_EQ(parseMaxBytes("-1", kDef), kDef);      // negative
}

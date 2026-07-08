// Copyright 2026 ros2_pulse contributors
//
// Unit tests for env-var config parsing. No ROS / tracetools dependency.
//
// Guards KNOWN_ISSUES.md #5: parsePeriodSeconds() must be noexcept and must never let a bad
// ROS_TOPIC_STATISTICS_PUBLISH_PERIOD value escape as an exception (which, from an extern "C"
// tracepoint, would terminate the host process). Invalid input falls back to the default.

#include <gtest/gtest.h>

#include "ros2_pulse/core/env_config.hpp"

using ros2_pulse::core::parsePeriodSeconds;

namespace {
// Sentinel default, deliberately distinct from every "accept" expectation below so a bug that
// wrongly returns the default can never masquerade as a pass.
constexpr double kDef = 99.0;
}  // namespace

// The contract that actually protects the process: no exception may ever leave this function.
static_assert(noexcept(parsePeriodSeconds("", 0.0)), "parsePeriodSeconds must be noexcept");

TEST(ParsePeriodSeconds, AcceptsValidValues) {
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("5.0", kDef), 5.0);
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("1e3", kDef), 1000.0);  // scientific notation
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("0.1", kDef), 0.1);
}

TEST(ParsePeriodSeconds, ToleratesSurroundingWhitespace) {
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("  2.5 ", kDef), 2.5);
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("\t10\n", kDef), 10.0);
}

TEST(ParsePeriodSeconds, RejectsEmptyOrMissing) {
    EXPECT_DOUBLE_EQ(parsePeriodSeconds(nullptr, kDef), kDef);  // env var unset
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("", kDef), kDef);
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("   ", kDef), kDef);  // whitespace only
}

TEST(ParsePeriodSeconds, RejectsNonNumeric) {
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("abc", kDef), kDef);
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("5abc", kDef), kDef);  // trailing garbage, not a prefix parse
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("1,5", kDef), kDef);   // locale-style comma rejected
}

TEST(ParsePeriodSeconds, RejectsNonPositive) {
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("-1", kDef), kDef);
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("0", kDef), kDef);
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("-2.5", kDef), kDef);
}

TEST(ParsePeriodSeconds, RejectsNonFinite) {
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("nan", kDef), kDef);
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("inf", kDef), kDef);
    EXPECT_DOUBLE_EQ(parsePeriodSeconds("-inf", kDef), kDef);
}
// main() intentionally omitted: this file is linked into the shared core gtest binary whose
// main() lives in test_topic_registry.cpp (see CMakeLists ROS2_PULSE_CORE_TEST_SOURCES).

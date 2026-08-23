// Copyright 2026 ros2_pulse contributors
//
// Golden-file unit tests for the JSON Lines window emitter (ROS_TOPIC_STATS_FORMAT=jsonl,
// ROADMAP R6). Exact-bytes style, same as test_window_format.cpp for the text format: the
// record is what sidecar exporters and log shippers parse, so its bytes are the contract.
//
// Conventions pinned here (see window_format.hpp for the rationale of each):
//  - one JSON object per window, ONE line, '\n'-terminated (jsonlines.org);
//  - ts_ns as a decimal STRING (int64 > 2^53 loses digits as a JSON number, the OTLP/JSON
//    timeUnixNano precedent);
//  - same emit gates as the text format; absent key = not measured, never 0;
//  - topics/nodes/warns always present, warns structured;
//  - user-controlled names escaped per RFC 8259.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "ros2_pulse/core/rate_spec.hpp"
#include "ros2_pulse/core/topic_registry.hpp"
#include "ros2_pulse/core/window_format.hpp"

using ros2_pulse::core::eWarnKind;
using ros2_pulse::core::formatWindowJsonl;
using ros2_pulse::core::sRateWarning;
using ros2_pulse::core::sTopicStat;

namespace {

auto stat(const std::string& topic, uint64_t pub_count, uint64_t recv_inter_count,
          uint64_t recv_intra_count, double pub_hz, double recv_inter_hz, double recv_intra_hz)
    -> sTopicStat {
    sTopicStat s;
    s.topic = topic;
    s.pub_inter_count = pub_count;
    s.recv_inter_count = recv_inter_count;
    s.recv_intra_count = recv_intra_count;
    s.pub_inter_hz = pub_hz;
    s.recv_inter_hz = recv_inter_hz;
    s.recv_intra_hz = recv_intra_hz;
    return s;
}

}  // namespace

// The text-format mixed window (publish-only, intra-receive-only, same-process pub+sub),
// re-encoded as one jsonl record. Locks: ts_ns as a decimal string (1782887153899445923 >
// 2^53, as a JSON number the low digits would be gone in every double-based consumer), the
// text precisions (%.3f window, %.6f Hz), the per-side key gates mirroring TOPIC/RECV, and
// the single trailing '\n'.
TEST(WindowFormatJsonl, MixedWindowExactBytes) {
    std::vector<sTopicStat> stats = {
        stat("/scan", 100, 0, 0, 20.0, 0.0, 0.0),
        stat("/points", 0, 0, 150, 0.0, 0.0, 30.0),
        stat("/both", 50, 50, 0, 10.0, 10.0, 0.0),
    };
    std::vector<std::string> nodes = {"/perception", "/planner"};

    const std::string got =
        formatWindowJsonl(stats, nodes, 1782887153899445923LL, 5.0, /*emit_idle=*/false);

    const std::string want =
        "{\"ts_ns\":\"1782887153899445923\",\"window_s\":5.000,\"topics\":["
        "{\"topic\":\"/scan\",\"pub_inter_hz\":20.000000,\"pub_intra_hz\":0.000000},"
        "{\"topic\":\"/points\",\"recv_inter_hz\":0.000000,\"recv_intra_hz\":30.000000,"
        "\"recv_endpoint_seen\":true},"
        "{\"topic\":\"/both\",\"pub_inter_hz\":10.000000,\"pub_intra_hz\":0.000000,"
        "\"recv_inter_hz\":10.000000,\"recv_intra_hz\":0.000000,\"recv_endpoint_seen\":true}"
        "],\"nodes\":[\"/perception\",\"/planner\"],\"warns\":[]}\n";
    EXPECT_EQ(got, want);
}

// Idle suppression is the same policy as text (issue #7): a declared-but-silent topic is
// omitted from the topics array by default, and the arrays are still PRESENT (empty), so a
// sidecar's .topics[] / .warns[] never needs null guards.
TEST(WindowFormatJsonl, IdleTopicSuppressedAndArraysAlwaysPresent) {
    std::vector<sTopicStat> stats = {stat("/idle", 0, 0, 0, 0.0, 0.0, 0.0)};
    std::vector<std::string> nodes = {};

    const std::string got = formatWindowJsonl(stats, nodes, 42LL, 1.5, /*emit_idle=*/false);

    EXPECT_EQ(got, "{\"ts_ns\":\"42\",\"window_s\":1.500,\"topics\":[],\"nodes\":[],"
                   "\"warns\":[]}\n");
}

// ROS_PULSE_EMIT_IDLE=1 restores the zero publish-side entry, exactly like the text TOPIC line.
TEST(WindowFormatJsonl, IdleTopicRestoredByOptIn) {
    std::vector<sTopicStat> stats = {stat("/idle", 0, 0, 0, 0.0, 0.0, 0.0)};
    std::vector<std::string> nodes = {};

    const std::string got = formatWindowJsonl(stats, nodes, 42LL, 1.5, /*emit_idle=*/true);

    EXPECT_EQ(got,
              "{\"ts_ns\":\"42\",\"window_s\":1.500,\"topics\":[{\"topic\":\"/idle\","
              "\"pub_inter_hz\":0.000000,\"pub_intra_hz\":0.000000}],\"nodes\":[],"
              "\"warns\":[]}\n");
}

// An intra-only publisher (iron+ tracepoint) carries the pub pair, the analogue of the
// additive PUB line; no recv keys appear because no recv gate fired (absence, not zeros).
TEST(WindowFormatJsonl, IntraPublisherCarriesPubPairOnly) {
    sTopicStat s;
    s.topic = "/points";
    s.pub_intra_count = 150;
    s.pub_intra_hz = 30.0;
    std::vector<sTopicStat> stats = {s};

    const std::string got = formatWindowJsonl(stats, {}, 42LL, 5.0, /*emit_idle=*/false);

    EXPECT_EQ(got,
              "{\"ts_ns\":\"42\",\"window_s\":5.000,\"topics\":[{\"topic\":\"/points\","
              "\"pub_inter_hz\":0.000000,\"pub_intra_hz\":30.000000}],\"nodes\":[],"
              "\"warns\":[]}\n");
}

// A stalled proven receive endpoint reads an explicit 0.0 pair (KNOWN_ISSUES #12), same as the
// RECV zero line, a dead upstream must not vanish just because the encoding changed.
TEST(WindowFormatJsonl, StalledRecvEndpointEmitsZeroPair) {
    sTopicStat s;
    s.topic = "/scan";
    s.recv_endpoint_seen = true;
    std::vector<sTopicStat> stats = {s};

    const std::string got = formatWindowJsonl(stats, {}, 42LL, 1.5, /*emit_idle=*/false);

    EXPECT_EQ(got,
              "{\"ts_ns\":\"42\",\"window_s\":1.500,\"topics\":[{\"topic\":\"/scan\","
              "\"recv_inter_hz\":0.000000,\"recv_intra_hz\":0.000000,"
              "\"recv_endpoint_seen\":true}],\"nodes\":[],\"warns\":[]}\n");
}

// Gap fields appear ONLY when measured (mirror of has_pub_max_dt / has_recv_max_dt): a probe
// without gap tracking must not report a gap of zero, absence is "not measured", 0 would be
// "perfectly smooth", and conflating them is how a stall hides.
TEST(WindowFormatJsonl, MaxDtFieldsOnlyWhenMeasured) {
    sTopicStat with = stat("/scan", 100, 100, 0, 20.0, 20.0, 0.0);
    with.recv_endpoint_seen = true;
    with.pub_max_dt_ms = 1.5;
    with.has_pub_max_dt = true;
    with.recv_max_dt_ms = 21.284;
    with.has_recv_max_dt = true;
    sTopicStat without = stat("/cam", 30, 0, 0, 6.0, 0.0, 0.0);  // tracking off: no gap keys

    const std::string got =
        formatWindowJsonl({with, without}, {}, 42LL, 5.0, /*emit_idle=*/false);

    const std::string want =
        "{\"ts_ns\":\"42\",\"window_s\":5.000,\"topics\":["
        "{\"topic\":\"/scan\",\"pub_inter_hz\":20.000000,\"pub_intra_hz\":0.000000,"
        "\"recv_inter_hz\":20.000000,\"recv_intra_hz\":0.000000,\"recv_endpoint_seen\":true,"
        "\"pub_max_dt_ms\":1.500,\"recv_max_dt_ms\":21.284},"
        "{\"topic\":\"/cam\",\"pub_inter_hz\":6.000000,\"pub_intra_hz\":0.000000}"
        "],\"nodes\":[],\"warns\":[]}\n";
    EXPECT_EQ(got, want);
}

// Structured warns: JSON numbers a sidecar can threshold on directly, never a preformatted
// sentence to regex. An unbounded max_hz OMITS the key, RFC 8259 has no Infinity literal, and
// absence-means-unbounded matches the gap-field absence semantics.
TEST(WindowFormatJsonl, WarnsAreStructuredObjects) {
    std::vector<sTopicStat> stats = {stat("/scan", 6, 0, 0, 1.2, 0.0, 0.0)};
    std::vector<std::string> nodes = {"/perception"};

    sRateWarning rate;
    rate.kind = eWarnKind::kTopicRate;
    rate.name = "/scan";
    rate.hz = 1.2;
    rate.min_hz = 18.0;
    rate.max_hz = 22.0;
    sRateWarning unbounded;
    unbounded.kind = eWarnKind::kTopicRate;
    unbounded.name = "/cam";
    unbounded.hz = 0.0;
    unbounded.min_hz = 15.0;  // max_hz stays infinity -> key omitted
    sRateWarning gap;
    gap.kind = eWarnKind::kTopicGap;
    gap.name = "/scan";
    gap.max_dt_ms = 812.437;
    gap.max_gap_ms = 100.0;
    sRateWarning node;
    node.kind = eWarnKind::kNodeMissing;
    node.name = "/planner";

    const std::string got = formatWindowJsonl(stats, nodes, 42LL, 5.0, /*emit_idle=*/false,
                                              {rate, unbounded, gap, node});

    const std::string want =
        "{\"ts_ns\":\"42\",\"window_s\":5.000,\"topics\":["
        "{\"topic\":\"/scan\",\"pub_inter_hz\":1.200000,\"pub_intra_hz\":0.000000}"
        "],\"nodes\":[\"/perception\"],\"warns\":["
        "{\"kind\":\"topic_rate\",\"topic\":\"/scan\",\"hz\":1.200000,\"min_hz\":18,"
        "\"max_hz\":22},"
        "{\"kind\":\"topic_rate\",\"topic\":\"/cam\",\"hz\":0.000000,\"min_hz\":15},"
        "{\"kind\":\"topic_gap\",\"topic\":\"/scan\",\"max_dt_ms\":812.437,"
        "\"max_gap_ms\":100},"
        "{\"kind\":\"node_missing\",\"node\":\"/planner\"}"
        "]}\n";
    EXPECT_EQ(got, want);
}

// Topic and node names come from user code, a hostile name (quote, backslash, newline, tab,
// raw control byte) must be escaped per RFC 8259 and must NEVER break the one-object-per-line
// framing: exactly one '\n' in the record, at its end.
TEST(WindowFormatJsonl, HostileNamesEscapedAndFramingHolds) {
    std::string hostile = "/e\"q\\b\nn\tt";
    hostile.push_back('\x01');
    hostile += "x";
    sTopicStat s;
    s.topic = hostile;
    s.recv_endpoint_seen = true;
    std::vector<std::string> nodes = {"/n\"ode"};

    const std::string got = formatWindowJsonl({s}, nodes, 42LL, 1.5, /*emit_idle=*/false);

    const std::string want =
        "{\"ts_ns\":\"42\",\"window_s\":1.500,\"topics\":["
        "{\"topic\":\"/e\\\"q\\\\b\\nn\\tt\\u0001x\","
        "\"recv_inter_hz\":0.000000,\"recv_intra_hz\":0.000000,"
        "\"recv_endpoint_seen\":true}],\"nodes\":[\"/n\\\"ode\"],\"warns\":[]}\n";
    EXPECT_EQ(got, want);

    // Framing property, independent of the exact bytes: one line, terminated.
    ASSERT_FALSE(got.empty());
    EXPECT_EQ(got.back(), '\n');
    EXPECT_EQ(std::count(got.begin(), got.end(), '\n'), 1);
}

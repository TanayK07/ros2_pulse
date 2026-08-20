// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the offline log reader used by the pulse-check CLI (ROADMAP R1): parseLog
// must invert formatWindow for every line kind. Registers into the shared test binary.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/window_format.hpp"

using ros2_pulse::core::formatWindow;
using ros2_pulse::core::parseLog;
using ros2_pulse::core::sTopicStat;

namespace {

auto findTopic(const std::vector<sTopicStat>& v, const std::string& t) -> const sTopicStat* {
    for (const auto& s : v) {
        if (s.topic == t) return &s;
    }
    return nullptr;
}

}  // namespace

// Round-trip through the real formatter: TOPIC + PUB + RECV lines for one topic must merge
// back into a single per-topic stat with all rate fields populated.
TEST(LogReader, RoundTripsMergedTopicLines) {
    sTopicStat s;
    s.topic = "/points";
    s.pub_inter_count = 100;
    s.pub_intra_count = 150;
    s.recv_inter_count = 100;
    s.recv_intra_count = 150;
    s.pub_inter_hz = 20.0;
    s.pub_intra_hz = 30.0;
    s.recv_inter_hz = 20.0;
    s.recv_intra_hz = 30.0;
    const std::string text =
        formatWindow({s}, {"/perception"}, 1782887153899445923LL, 5.0, /*emit_idle=*/false);

    auto windows = parseLog(text);
    ASSERT_EQ(windows.size(), 1u);
    EXPECT_EQ(windows[0].ts_ns, 1782887153899445923LL);
    EXPECT_DOUBLE_EQ(windows[0].window_s, 5.0);
    ASSERT_EQ(windows[0].nodes.size(), 1u);
    EXPECT_EQ(windows[0].nodes[0], "/perception");
    const auto* p = findTopic(windows[0].stats, "/points");
    ASSERT_NE(p, nullptr);
    EXPECT_DOUBLE_EQ(p->pub_inter_hz, 20.0);
    EXPECT_DOUBLE_EQ(p->pub_intra_hz, 30.0);
    EXPECT_DOUBLE_EQ(p->recv_inter_hz, 20.0);
    EXPECT_DOUBLE_EQ(p->recv_intra_hz, 30.0);
    EXPECT_TRUE(p->recv_endpoint_seen);
}

// Multiple windows split correctly and keep file order; a WARN line (or any future addition)
// must be skipped, not treated as a parse failure.
TEST(LogReader, MultipleWindowsAndUnknownLines) {
    sTopicStat a;
    a.topic = "/scan";
    a.pub_inter_count = 100;
    a.pub_inter_hz = 20.0;
    std::string text = formatWindow({a}, {}, 100LL, 1.0, false);
    a.pub_inter_hz = 10.0;
    text += formatWindow({a}, {}, 200LL, 1.0, false,
                         {"WARN TOPIC /scan hz=10.000000 expected=[15,inf]"});

    auto windows = parseLog(text);
    ASSERT_EQ(windows.size(), 2u);
    EXPECT_EQ(windows[0].ts_ns, 100LL);
    EXPECT_EQ(windows[1].ts_ns, 200LL);
    const auto* s0 = findTopic(windows[0].stats, "/scan");
    const auto* s1 = findTopic(windows[1].stats, "/scan");
    ASSERT_NE(s0, nullptr);
    ASSERT_NE(s1, nullptr);
    EXPECT_DOUBLE_EQ(s0->pub_inter_hz, 20.0);
    EXPECT_DOUBLE_EQ(s1->pub_inter_hz, 10.0);
    EXPECT_FALSE(s0->recv_endpoint_seen);
}

// Empty / garbage input parses to zero windows, never throws.
TEST(LogReader, EmptyAndGarbageInput) {
    EXPECT_TRUE(parseLog("").empty());
    EXPECT_TRUE(parseLog("not a probe log\nat all\n").empty());
}

// R5 JITTER lines round-trip through the real formatter: the max_dt values AND the has_* flags
// must both survive, because absence is a semantic ("never measured"), not a zero — pulse-check's
// exit-2 path keys on the flag, so a reader that dropped it would turn "cannot answer" into
// "healthy". Also pins that a stat formatted WITHOUT the flags parses back without them.
TEST(LogReader, RoundTripsJitterLines) {
    sTopicStat s;
    s.topic = "/scan";
    s.pub_inter_count = 100;
    s.pub_inter_hz = 20.0;
    s.recv_inter_count = 100;
    s.recv_inter_hz = 20.0;
    s.pub_max_dt_ms = 51.284;
    s.has_pub_max_dt = true;
    s.recv_max_dt_ms = 812.4;
    s.has_recv_max_dt = true;

    sTopicStat bare;  // same topic shape, no measured gap — flags must stay false through parse
    bare.topic = "/imu";
    bare.pub_inter_count = 50;
    bare.pub_inter_hz = 100.0;

    const std::string text = formatWindow({s, bare}, {}, 100LL, 5.0, /*emit_idle=*/false);

    auto windows = parseLog(text);
    ASSERT_EQ(windows.size(), 1u);
    const auto* p = findTopic(windows[0].stats, "/scan");
    ASSERT_NE(p, nullptr);
    EXPECT_TRUE(p->has_pub_max_dt);
    EXPECT_DOUBLE_EQ(p->pub_max_dt_ms, 51.284);
    EXPECT_TRUE(p->has_recv_max_dt);
    EXPECT_DOUBLE_EQ(p->recv_max_dt_ms, 812.4);
    // The rate fields on the same topic must still merge in from their own lines.
    EXPECT_DOUBLE_EQ(p->pub_inter_hz, 20.0);
    EXPECT_DOUBLE_EQ(p->recv_inter_hz, 20.0);

    const auto* q = findTopic(windows[0].stats, "/imu");
    ASSERT_NE(q, nullptr);
    EXPECT_FALSE(q->has_pub_max_dt);
    EXPECT_FALSE(q->has_recv_max_dt);
}

// A log written by a pre-R5 probe has no JITTER lines at all; it must parse exactly as before
// with both has_* flags false on every stat — the additive-format promise, read direction.
TEST(LogReader, PreR5LogParsesWithoutJitterFields) {
    const std::string text =
        "# ts_ns=100 window_s=5.000\n"
        "TOPIC /scan 20.000000\n"
        "RECV /scan inter=20.000000 intra=0.000000\n"
        "NODE /perception\n"
        "\n";
    auto windows = parseLog(text);
    ASSERT_EQ(windows.size(), 1u);
    const auto* s = findTopic(windows[0].stats, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_DOUBLE_EQ(s->pub_inter_hz, 20.0);
    EXPECT_TRUE(s->recv_endpoint_seen);
    EXPECT_FALSE(s->has_pub_max_dt);
    EXPECT_FALSE(s->has_recv_max_dt);
}

// A JITTER line whose side is neither `pub` nor `recv` (a future side, or corruption) is skipped
// like any unknown line — and must NOT conjure a topic entry as a side effect: statFor only runs
// after a full pattern match. Same for a JITTER line with a non-numeric value.
TEST(LogReader, UnknownJitterSideIsIgnored) {
    const std::string text =
        "# ts_ns=100 window_s=5.000\n"
        "JITTER /scan both max_dt_ms=5.000\n"
        "JITTER /scan pubx max_dt_ms=2.000\n"
        "JITTER /scan pub max_dt_ms=oops\n"
        "\n";
    auto windows = parseLog(text);
    ASSERT_EQ(windows.size(), 1u);
    EXPECT_TRUE(windows[0].stats.empty());
}

// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the expected-rate spec: the restricted-YAML parser and the flush-time
// evaluator that turns one window's stats into WARN lines (ROADMAP R1). Pure — no ROS, no
// I/O. Registers into the shared test binary (no main()).

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "ros2_pulse/core/rate_spec.hpp"

using ros2_pulse::core::eRateSide;
using ros2_pulse::core::eRateTransport;
using ros2_pulse::core::evaluateRateSpec;
using ros2_pulse::core::parseRateSpec;
using ros2_pulse::core::sRateSpec;
using ros2_pulse::core::sTopicStat;

namespace {

auto recvStat(const std::string& topic, double inter_hz, double intra_hz) -> sTopicStat {
    sTopicStat s;
    s.topic = topic;
    s.recv_inter_hz = inter_hz;
    s.recv_intra_hz = intra_hz;
    s.recv_endpoint_seen = true;
    return s;
}

auto pubStat(const std::string& topic, double inter_hz, double intra_hz) -> sTopicStat {
    sTopicStat s;
    s.topic = topic;
    s.pub_inter_hz = inter_hz;
    s.pub_intra_hz = intra_hz;
    return s;
}

}  // namespace

// ---- parser ----

// The exact ROADMAP R1 example must parse: two topic rules with mixed keys plus a flow-style
// node list, with documented defaults (side=recv, transport=any, max_hz=inf) filled in.
TEST(RateSpecParse, RoadmapExample) {
    const std::string text =
        "topics:\n"
        "  /scan:   {min_hz: 18, max_hz: 22, side: recv}\n"
        "  /points: {min_hz: 25, transport: intra}\n"
        "nodes: [/perception, /planner]\n";
    std::string err;
    auto spec = parseRateSpec(text, err);
    ASSERT_TRUE(spec.has_value()) << err;
    ASSERT_EQ(spec->topics.size(), 2u);
    EXPECT_EQ(spec->topics[0].first, "/scan");
    EXPECT_DOUBLE_EQ(spec->topics[0].second.min_hz, 18.0);
    EXPECT_DOUBLE_EQ(spec->topics[0].second.max_hz, 22.0);
    EXPECT_EQ(spec->topics[0].second.side, eRateSide::kRecv);
    EXPECT_EQ(spec->topics[0].second.transport, eRateTransport::kAny);
    EXPECT_EQ(spec->topics[1].first, "/points");
    EXPECT_DOUBLE_EQ(spec->topics[1].second.min_hz, 25.0);
    EXPECT_TRUE(std::isinf(spec->topics[1].second.max_hz));
    EXPECT_EQ(spec->topics[1].second.transport, eRateTransport::kIntra);
    ASSERT_EQ(spec->nodes.size(), 2u);
    EXPECT_EQ(spec->nodes[0], "/perception");
    EXPECT_EQ(spec->nodes[1], "/planner");
}

// Block-style node lists, comments, blank lines and side: pub are all part of the subset.
TEST(RateSpecParse, BlockNodesCommentsAndPubSide) {
    const std::string text =
        "# watchdog spec for the perception stack\n"
        "topics:\n"
        "\n"
        "  /image: {min_hz: 28, side: pub, transport: inter}  # camera driver\n"
        "nodes:\n"
        "  - /camera\n"
        "  - /rectifier\n";
    std::string err;
    auto spec = parseRateSpec(text, err);
    ASSERT_TRUE(spec.has_value()) << err;
    ASSERT_EQ(spec->topics.size(), 1u);
    EXPECT_EQ(spec->topics[0].second.side, eRateSide::kPub);
    EXPECT_EQ(spec->topics[0].second.transport, eRateTransport::kInter);
    ASSERT_EQ(spec->nodes.size(), 2u);
    EXPECT_EQ(spec->nodes[0], "/camera");
    EXPECT_EQ(spec->nodes[1], "/rectifier");
}

// Windows-authored specs must parse. A leading UTF-8 BOM is a signature, not content — and its
// bytes render invisibly, so without the strip the error reads "unknown top-level entry
// 'topics:'", indistinguishable from the correct spelling, and alerting silently turns off.
TEST(RateSpecParse, LeadingUtf8BomIgnored) {
    std::string err;
    auto spec = parseRateSpec("\xEF\xBB\xBF"
                              "topics:\n  /x: {min_hz: 1}\n",
                              err);
    ASSERT_TRUE(spec.has_value()) << err;
    ASSERT_EQ(spec->topics.size(), 1u);
    EXPECT_EQ(spec->topics[0].first, "/x");

    // ...on the nodes: key too, and line numbers must survive the strip.
    auto nodes = parseRateSpec("\xEF\xBB\xBF"
                               "nodes: [/a]\n",
                               err);
    ASSERT_TRUE(nodes.has_value()) << err;
    ASSERT_EQ(nodes->nodes.size(), 1u);
    EXPECT_FALSE(parseRateSpec("\xEF\xBB\xBF"
                               "topics:\n  /x: {min_hz: bad}\n",
                               err)
                     .has_value());
    EXPECT_NE(err.find("line 2"), std::string::npos) << err;
}

// CRLF is already handled by trim(), but pin it in every position so the Windows path stays
// green as the parser evolves: top-level keys, flow-map bodies, flow lists and block items.
TEST(RateSpecParse, CrlfLineEndings) {
    std::string err;
    auto spec = parseRateSpec(
        "topics:\r\n"
        "  /scan: {min_hz: 18, max_hz: 22, side: pub}\r\n"
        "nodes: [/a, /b]\r\n",
        err);
    ASSERT_TRUE(spec.has_value()) << err;
    ASSERT_EQ(spec->topics.size(), 1u);
    EXPECT_DOUBLE_EQ(spec->topics[0].second.max_hz, 22.0);
    EXPECT_EQ(spec->topics[0].second.side, eRateSide::kPub);
    ASSERT_EQ(spec->nodes.size(), 2u);
    EXPECT_EQ(spec->nodes[1], "/b");

    auto block = parseRateSpec("nodes:\r\n  - /camera\r\n", err);
    ASSERT_TRUE(block.has_value()) << err;
    ASSERT_EQ(block->nodes.size(), 1u);
    EXPECT_EQ(block->nodes[0], "/camera");
}

// A rule must constrain something: at least one of min_hz / max_hz.
TEST(RateSpecParse, RejectsRuleWithoutBounds) {
    std::string err;
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {side: pub}\n", err).has_value());
    EXPECT_NE(err.find("line 2"), std::string::npos) << err;
}

// Errors must be hard and carry the line number: unknown key, bad number, inverted bounds,
// duplicate topic, unknown enum value, and junk outside any section.
TEST(RateSpecParse, RejectsMalformedInput) {
    std::string err;
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {min_hz: 5, rate: 9}\n", err).has_value());
    EXPECT_NE(err.find("line 2"), std::string::npos) << err;
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {min_hz: fast}\n", err).has_value());
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {min_hz: 9, max_hz: 3}\n", err).has_value());
    EXPECT_FALSE(
        parseRateSpec("topics:\n  /x: {min_hz: 1}\n  /x: {min_hz: 2}\n", err).has_value());
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {min_hz: 1, side: down}\n", err).has_value());
    EXPECT_FALSE(parseRateSpec("bogus: 1\n", err).has_value());
    EXPECT_FALSE(parseRateSpec("  /x: {min_hz: 1}\n", err).has_value());  // rule outside topics:
}

// ---- evaluator ----

// Below-min and above-max both produce the pinned WARN TOPIC line; in-range produces nothing.
TEST(RateSpecEval, TopicBoundsExactLines) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /scan: {min_hz: 18, max_hz: 22}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    auto low = evaluateRateSpec(*spec, {recvStat("/scan", 1.2, 0.0)}, {}, {});
    ASSERT_EQ(low.size(), 1u);
    EXPECT_EQ(low[0], "WARN TOPIC /scan hz=1.200000 expected=[18,22]");

    auto high = evaluateRateSpec(*spec, {recvStat("/scan", 31.5, 0.0)}, {}, {});
    ASSERT_EQ(high.size(), 1u);
    EXPECT_EQ(high[0], "WARN TOPIC /scan hz=31.500000 expected=[18,22]");

    EXPECT_TRUE(evaluateRateSpec(*spec, {recvStat("/scan", 20.0, 0.0)}, {}, {}).empty());
}

// An unbounded max renders as 'inf' and side/transport select the right rate field:
// recv/any sums inter+intra; pub/inter reads only the inter publish rate.
TEST(RateSpecEval, SideAndTransportSelection) {
    std::string err;
    auto spec = parseRateSpec(
        "topics:\n"
        "  /points: {min_hz: 25}\n"
        "  /image:  {min_hz: 28, side: pub, transport: inter}\n",
        err);
    ASSERT_TRUE(spec.has_value()) << err;

    // recv any: 10 inter + 20 intra = 30 >= 25 -> no warning
    EXPECT_TRUE(evaluateRateSpec(*spec, {recvStat("/points", 10.0, 20.0)}, {}, {}).empty());

    // pub inter: intra rate must NOT rescue an under-rate inter publisher
    auto w = evaluateRateSpec(*spec, {pubStat("/image", 5.0, 100.0)}, {}, {});
    ASSERT_EQ(w.size(), 1u);
    EXPECT_EQ(w[0], "WARN TOPIC /image hz=5.000000 expected=[28,inf]");
}

// Probe mode: a spec topic this process doesn't host is another process's business — no
// warning. pulse-check mode (missing_as_zero) owns the whole picture: it warns at 0 Hz.
TEST(RateSpecEval, MissingTopicSkippedUnlessMissingAsZero) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /gps: {min_hz: 5}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    EXPECT_TRUE(evaluateRateSpec(*spec, {}, {}, {}).empty());

    auto w = evaluateRateSpec(*spec, {}, {}, {}, /*missing_as_zero=*/true);
    ASSERT_EQ(w.size(), 1u);
    EXPECT_EQ(w[0], "WARN TOPIC /gps hz=0.000000 expected=[5,inf]");
}

// Node liveness: known-but-inactive fires the pinned WARN NODE line; active stays silent;
// never-seen is skipped in probe mode and warned in pulse-check mode.
TEST(RateSpecEval, NodePresence) {
    std::string err;
    auto spec = parseRateSpec("nodes: [/perception]\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    auto gone = evaluateRateSpec(*spec, {}, /*active=*/{}, /*known=*/{"/perception"});
    ASSERT_EQ(gone.size(), 1u);
    EXPECT_EQ(gone[0], "WARN NODE /perception missing");

    EXPECT_TRUE(evaluateRateSpec(*spec, {}, {"/perception"}, {"/perception"}).empty());
    EXPECT_TRUE(evaluateRateSpec(*spec, {}, {}, {}).empty());

    auto never = evaluateRateSpec(*spec, {}, {}, {}, /*missing_as_zero=*/true);
    ASSERT_EQ(never.size(), 1u);
    EXPECT_EQ(never[0], "WARN NODE /perception missing");
}

// Warnings come out in spec order (topics first, then nodes) so log diffs are stable.
TEST(RateSpecEval, DeterministicOrder) {
    std::string err;
    auto spec = parseRateSpec(
        "topics:\n"
        "  /b: {min_hz: 10}\n"
        "  /a: {min_hz: 10}\n"
        "nodes: [/n]\n",
        err);
    ASSERT_TRUE(spec.has_value()) << err;
    auto w = evaluateRateSpec(*spec, {recvStat("/a", 0.0, 0.0), recvStat("/b", 0.0, 0.0)}, {},
                              {"/n"});
    ASSERT_EQ(w.size(), 3u);
    EXPECT_EQ(w[0], "WARN TOPIC /b hz=0.000000 expected=[10,inf]");
    EXPECT_EQ(w[1], "WARN TOPIC /a hz=0.000000 expected=[10,inf]");
    EXPECT_EQ(w[2], "WARN NODE /n missing");
// pub + any takes the LARGER bucket, not the sum: on iron+ one publish() fires
// rclcpp_intra_publish AND rcl_publish for the same message whenever a non-intra subscriber is
// matched (or the QoS is TransientLocal on jazzy+), so both pub buckets carry the same produce
// rate. Summing would report 2x and invert max_hz. recv is unaffected — its buckets are
// disjoint deliveries — and that must stay true.
TEST(RateSpecEval, PubAnyTakesMaxNotSumAcrossTransports) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /image: {min_hz: 45, max_hz: 55, side: pub}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    // Both tracepoints fired for one 50 Hz stream (IPC on, mixed subscribers): 50, not 100.
    EXPECT_TRUE(evaluateRateSpec(*spec, {pubStat("/image", 50.0, 50.0)}, {}, {}).empty());
    // Only the intra path fired (all subscribers in-process — rcl_publish never called).
    EXPECT_TRUE(evaluateRateSpec(*spec, {pubStat("/image", 0.0, 50.0)}, {}, {}).empty());
    // Only the RMW path fired (intra-process comms off).
    EXPECT_TRUE(evaluateRateSpec(*spec, {pubStat("/image", 50.0, 0.0)}, {}, {}).empty());

    // max() must not rescue a genuinely slow publisher.
    auto slow = evaluateRateSpec(*spec, {pubStat("/image", 10.0, 10.0)}, {}, {});
    ASSERT_EQ(slow.size(), 1u);
    EXPECT_EQ(slow[0], "WARN TOPIC /image hz=10.000000 expected=[45,55]");

    // recv/any still SUMS: 10 + 20 = 30 is over max, and must stay that way.
    auto recv_spec = parseRateSpec("topics:\n  /points: {min_hz: 5, max_hz: 25}\n", err);
    ASSERT_TRUE(recv_spec.has_value()) << err;
    auto summed = evaluateRateSpec(*recv_spec, {recvStat("/points", 10.0, 20.0)}, {}, {});
    ASSERT_EQ(summed.size(), 1u);
    EXPECT_EQ(summed[0], "WARN TOPIC /points hz=30.000000 expected=[5,25]");
}

}

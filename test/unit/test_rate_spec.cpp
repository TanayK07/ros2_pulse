// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the expected-rate spec: the restricted-YAML parser and the flush-time
// evaluator that turns one window's stats into WARN lines (ROADMAP R1). Pure, no ROS, no
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

// Windows-authored specs must parse. A leading UTF-8 BOM is a signature, not content, and its
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

// One topic may carry several rules as long as they measure different things, constraining both
// ends of a topic ("the driver publishes ~20 Hz AND we receive ~20 Hz") is a first-class spec.
// Rules that measure the SAME thing are still a copy-paste error.
TEST(RateSpecParse, TwoSidedRulesOnOneTopic) {
    std::string err;
    auto spec = parseRateSpec(
        "topics:\n"
        "  /scan: {min_hz: 18, side: pub}\n"
        "  /scan: {min_hz: 18, side: recv}\n",
        err);
    ASSERT_TRUE(spec.has_value()) << err;
    ASSERT_EQ(spec->topics.size(), 2u);
    EXPECT_EQ(spec->topics[0].first, "/scan");
    EXPECT_EQ(spec->topics[0].second.side, eRateSide::kPub);
    EXPECT_EQ(spec->topics[1].first, "/scan");
    EXPECT_EQ(spec->topics[1].second.side, eRateSide::kRecv);

    // differing only in transport is also a distinct measurement
    auto by_transport = parseRateSpec(
        "topics:\n"
        "  /points: {min_hz: 25, transport: inter}\n"
        "  /points: {min_hz: 25, transport: intra}\n",
        err);
    ASSERT_TRUE(by_transport.has_value()) << err;
    EXPECT_EQ(by_transport->topics.size(), 2u);

    // same side AND same transport (both defaulted) is still rejected
    EXPECT_FALSE(
        parseRateSpec("topics:\n  /x: {min_hz: 1}\n  /x: {min_hz: 2}\n", err).has_value());
    EXPECT_NE(err.find("same side and transport"), std::string::npos) << err;
    // ...including when the duplicated side is spelled out on only one of them
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {min_hz: 1}\n  /x: {max_hz: 9, side: recv}\n", err)
                     .has_value());
}

// A key repeated inside one flow map used to silently last-win, the one hole in the parser's
// "duplicates are hard errors" contract.
TEST(RateSpecParse, RejectsRepeatedKeyInOneRule) {
    std::string err;
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {min_hz: 1, min_hz: 2}\n", err).has_value());
    EXPECT_NE(err.find("duplicate key 'min_hz'"), std::string::npos) << err;
    EXPECT_FALSE(
        parseRateSpec("topics:\n  /x: {min_hz: 1, side: pub, side: recv}\n", err).has_value());
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {max_hz: 5, max_hz: 5}\n", err).has_value());
    EXPECT_FALSE(
        parseRateSpec("topics:\n  /x: {min_hz: 1, transport: any, transport: intra}\n", err)
            .has_value());
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

// pub + any takes the LARGER bucket, not the sum: on iron+ one publish() fires
// rclcpp_intra_publish AND rcl_publish for the same message whenever a non-intra subscriber is
// matched (or the QoS is TransientLocal on jazzy+), so both pub buckets carry the same produce
// rate. Summing would report 2x and invert max_hz. recv is unaffected, its buckets are
// disjoint deliveries, and that must stay true.
TEST(RateSpecEval, PubAnyTakesMaxNotSumAcrossTransports) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /image: {min_hz: 45, max_hz: 55, side: pub}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    // Both tracepoints fired for one 50 Hz stream (IPC on, mixed subscribers): 50, not 100.
    EXPECT_TRUE(evaluateRateSpec(*spec, {pubStat("/image", 50.0, 50.0)}, {}, {}).empty());
    // Only the intra path fired (all subscribers in-process, rcl_publish never called).
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

// Probe mode: a spec topic this process doesn't host is another process's business, no
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
}

// ---- gap rules (ROADMAP R5) ----

// max_gap_ms is a first-class constraint: a rule may carry it ALONE. The whole point of R5 is
// that min_hz is the weak detector, so forcing a redundant rate bound on every gap rule would
// have the tool contradicting its own documentation.
TEST(RateSpecParse, MaxGapMsAloneIsAValidRule) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /cmd_vel: {max_gap_ms: 30, side: pub}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;
    ASSERT_EQ(spec->topics.size(), 1u);
    EXPECT_DOUBLE_EQ(spec->topics[0].second.max_gap_ms, 30.0);
    EXPECT_TRUE(std::isinf(spec->topics[0].second.max_hz)) << "rate bounds stay unconstrained";
    EXPECT_EQ(spec->topics[0].second.side, eRateSide::kPub);

    // ...and combines with rate bounds in one rule
    auto both = parseRateSpec("topics:\n  /scan: {min_hz: 45, max_gap_ms: 60}\n", err);
    ASSERT_TRUE(both.has_value()) << err;
    EXPECT_DOUBLE_EQ(both->topics[0].second.min_hz, 45.0);
    EXPECT_DOUBLE_EQ(both->topics[0].second.max_gap_ms, 60.0);
}

// A rule still has to constrain SOMETHING, and an unsatisfiable gap bound is rejected up front
// rather than warning on every window forever.
TEST(RateSpecParse, RejectsBadGapRules) {
    std::string err;
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {side: pub}\n", err).has_value());
    EXPECT_NE(err.find("max_gap_ms"), std::string::npos) << "error should name all three keys";

    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {max_gap_ms: 0}\n", err).has_value());
    EXPECT_NE(err.find("must be > 0"), std::string::npos) << err;
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {max_gap_ms: -5}\n", err).has_value());
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {max_gap_ms: soon}\n", err).has_value());
    EXPECT_FALSE(parseRateSpec("topics:\n  /x: {max_gap_ms: 5, max_gap_ms: 9}\n", err).has_value());
    EXPECT_NE(err.find("duplicate key 'max_gap_ms'"), std::string::npos) << err;

    // transport picks a rate bucket; the gap series is transport-merged, so pairing them alone
    // constrains nothing.
    EXPECT_FALSE(
        parseRateSpec("topics:\n  /x: {max_gap_ms: 5, transport: intra}\n", err).has_value());
    EXPECT_NE(err.find("no effect"), std::string::npos) << err;
}

// The pinned WARN line, and the fact that a rule can violate rate AND gap independently.
TEST(RateSpecEval, GapWarnLineAndDualViolation) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /scan: {min_hz: 45, max_gap_ms: 60, side: recv}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    auto in_range = recvStat("/scan", 50.0, 0.0);
    in_range.recv_max_dt_ms = 21.0;
    in_range.has_recv_max_dt = true;
    EXPECT_TRUE(evaluateRateSpec(*spec, {in_range}, {}, {}).empty());

    // healthy mean rate, but a freeze, exactly the case a windowed mean cannot see
    auto frozen = recvStat("/scan", 50.0, 0.0);
    frozen.recv_max_dt_ms = 812.4;
    frozen.has_recv_max_dt = true;
    auto w = evaluateRateSpec(*spec, {frozen}, {}, {});
    ASSERT_EQ(w.size(), 1u);
    EXPECT_EQ(w[0], "WARN TOPIC /scan max_dt_ms=812.400 expected_max_gap_ms=60");

    // slow AND frozen: two distinct faults, two lines, rate first
    auto both = recvStat("/scan", 10.0, 0.0);
    both.recv_max_dt_ms = 812.4;
    both.has_recv_max_dt = true;
    auto w2 = evaluateRateSpec(*spec, {both}, {}, {});
    ASSERT_EQ(w2.size(), 2u);
    EXPECT_EQ(w2[0], "WARN TOPIC /scan hz=10.000000 expected=[45,inf]");
    EXPECT_EQ(w2[1], "WARN TOPIC /scan max_dt_ms=812.400 expected_max_gap_ms=60");
}

// A gap rule reads its own side's accumulator.
TEST(RateSpecEval, GapRuleRespectsSide) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /image: {max_gap_ms: 50, side: pub}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    auto s = pubStat("/image", 20.0, 0.0);
    s.pub_max_dt_ms = 900.0;
    s.has_pub_max_dt = true;
    s.recv_max_dt_ms = 1.0;  // a healthy recv side must not rescue a frozen pub side
    s.has_recv_max_dt = true;
    auto w = evaluateRateSpec(*spec, {s}, {}, {});
    ASSERT_EQ(w.size(), 1u);
    EXPECT_NE(w[0].find("max_dt_ms=900.000"), std::string::npos) << w[0];
}

// Measurement absence is not a health verdict. In probe mode an unmeasured gap is silently
// skipped (another process's endpoint); offline it is collected so pulse-check can exit 2
// rather than claim the check passed.
TEST(RateSpecEval, UnmeasuredGapIsReportedNotPassed) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /scan: {max_gap_ms: 60}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    auto no_jitter = recvStat("/scan", 50.0, 0.0);  // has_recv_max_dt stays false
    EXPECT_TRUE(evaluateRateSpec(*spec, {no_jitter}, {}, {}).empty()) << "probe mode: skip";

    std::vector<std::string> unmeasured;
    auto w = evaluateRateSpec(*spec, {no_jitter}, {}, {}, /*missing_as_zero=*/true, &unmeasured);
    EXPECT_TRUE(w.empty()) << "must not fabricate a violation it could not measure";
    ASSERT_EQ(unmeasured.size(), 1u);
    EXPECT_EQ(unmeasured[0], "/scan");
}

// ---- structured warnings (ROADMAP R6: the jsonl emitter needs data, not sentences) ----

using ros2_pulse::core::evaluateRateSpecWarnings;
using ros2_pulse::core::eWarnKind;
using ros2_pulse::core::renderWarnLine;
using ros2_pulse::core::sRateWarning;

// The structured evaluator carries every number the text line renders, a sidecar thresholds
// on warn.hz directly instead of regexing "hz=1.200000" back out of our own sentence.
TEST(RateSpecEval, StructuredWarningsCarryTheNumbers) {
    std::string err;
    auto spec = parseRateSpec(
        "topics:\n"
        "  /scan: {min_hz: 18, max_hz: 22, max_gap_ms: 60}\n"
        "nodes: [/planner]\n",
        err);
    ASSERT_TRUE(spec.has_value()) << err;

    auto slow_and_frozen = recvStat("/scan", 1.2, 0.0);
    slow_and_frozen.recv_max_dt_ms = 812.4;
    slow_and_frozen.has_recv_max_dt = true;

    const auto warns = evaluateRateSpecWarnings(*spec, {slow_and_frozen}, {}, {},
                                                /*missing_as_zero=*/true);
    ASSERT_EQ(warns.size(), 3u);

    EXPECT_EQ(warns[0].kind, eWarnKind::kTopicRate);
    EXPECT_EQ(warns[0].name, "/scan");
    EXPECT_DOUBLE_EQ(warns[0].hz, 1.2);
    EXPECT_DOUBLE_EQ(warns[0].min_hz, 18.0);
    EXPECT_DOUBLE_EQ(warns[0].max_hz, 22.0);

    EXPECT_EQ(warns[1].kind, eWarnKind::kTopicGap);
    EXPECT_EQ(warns[1].name, "/scan");
    EXPECT_DOUBLE_EQ(warns[1].max_dt_ms, 812.4);
    EXPECT_DOUBLE_EQ(warns[1].max_gap_ms, 60.0);

    EXPECT_EQ(warns[2].kind, eWarnKind::kNodeMissing);
    EXPECT_EQ(warns[2].name, "/planner");
}

// An unbounded max_hz stays infinity in the struct, the jsonl emitter turns that into an
// omitted key, and renderWarnLine into the literal 'inf'.
TEST(RateSpecEval, StructuredUnboundedMaxStaysInfinity) {
    std::string err;
    auto spec = parseRateSpec("topics:\n  /scan: {min_hz: 45}\n", err);
    ASSERT_TRUE(spec.has_value()) << err;

    const auto warns = evaluateRateSpecWarnings(*spec, {recvStat("/scan", 10.0, 0.0)}, {}, {});
    ASSERT_EQ(warns.size(), 1u);
    EXPECT_TRUE(std::isinf(warns[0].max_hz));
}

// The string API is a pure projection of the structured one: same window, element-for-element
// renderWarnLine equality. This is the invariant that keeps text and jsonl from ever drifting.
TEST(RateSpecEval, StringApiIsRenderedStructuredApi) {
    std::string err;
    auto spec = parseRateSpec(
        "topics:\n"
        "  /scan:  {min_hz: 45, max_gap_ms: 60}\n"
        "  /image: {min_hz: 28, max_hz: 32, side: pub, transport: inter}\n"
        "nodes: [/planner]\n",
        err);
    ASSERT_TRUE(spec.has_value()) << err;

    auto frozen = recvStat("/scan", 10.0, 0.0);
    frozen.recv_max_dt_ms = 812.4;
    frozen.has_recv_max_dt = true;
    const std::vector<sTopicStat> stats = {frozen, pubStat("/image", 5.0, 0.0)};

    const auto lines = evaluateRateSpec(*spec, stats, {}, {}, /*missing_as_zero=*/true);
    const auto warns = evaluateRateSpecWarnings(*spec, stats, {}, {}, /*missing_as_zero=*/true);
    ASSERT_EQ(lines.size(), warns.size());
    ASSERT_EQ(lines.size(), 4u);  // rate+gap for /scan, rate for /image, node missing
    for (size_t i = 0; i < lines.size(); ++i) {
        EXPECT_EQ(renderWarnLine(warns[i]), lines[i]) << "index " << i;
    }
    // Spot-pin one rendered line so a joint drift of BOTH APIs cannot pass the equality above.
    EXPECT_EQ(lines[0], "WARN TOPIC /scan hz=10.000000 expected=[45,inf]");
}

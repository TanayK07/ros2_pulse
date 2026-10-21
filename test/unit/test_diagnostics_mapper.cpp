// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the /diagnostics mapper behind pulse_bridge's diagnostics output: the probe's
// warns become levels per spec rule and node, a log that stops producing windows goes ERROR.
// Pure core, no ROS types. Registers into the shared test binary.

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "ros2_pulse/core/diagnostics_mapper.hpp"
#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/rate_spec.hpp"

using ros2_pulse::core::eDiagLevel;
using ros2_pulse::core::eEndpointSide;
using ros2_pulse::core::eWarnKind;
using ros2_pulse::core::mapDiagnostics;
using ros2_pulse::core::parseRateSpec;
using ros2_pulse::core::sDiagLog;
using ros2_pulse::core::sDiagOptions;
using ros2_pulse::core::sDiagStatus;
using ros2_pulse::core::sEndpointStat;
using ros2_pulse::core::sRateSpec;
using ros2_pulse::core::sRateWarning;
using ros2_pulse::core::sTopicStat;

namespace {

auto spec(const std::string& text) -> sRateSpec {
    std::string error;
    auto s = parseRateSpec(text, error);
    EXPECT_TRUE(s.has_value()) << error;
    return s.value_or(sRateSpec{});
}

// One process receiving /scan at @p hz with a 21.284 ms worst gap, node /perception alive.
auto scanLog(double hz, const std::string& path = "/tmp/topic_freq.1.log") -> sDiagLog {
    sDiagLog log;
    log.path = path;
    log.period_s = 5.0;
    log.age_s = 1.0;
    log.window.window_s = 5.0;
    sTopicStat s;
    s.topic = "/scan";
    s.recv_inter_hz = hz;
    s.recv_endpoint_seen = true;
    s.recv_max_dt_ms = 21.284;
    s.has_recv_max_dt = true;
    log.window.stats.push_back(s);
    log.window.nodes = {"/perception"};
    return log;
}

auto rateWarn(const std::string& topic, double hz, double min_hz, double max_hz) -> sRateWarning {
    sRateWarning w;
    w.kind = eWarnKind::kTopicRate;
    w.name = topic;
    w.hz = hz;
    w.min_hz = min_hz;
    w.max_hz = max_hz;
    return w;
}

auto find(const std::vector<sDiagStatus>& v, const std::string& name) -> const sDiagStatus* {
    for (const auto& s : v) {
        if (s.name == name) return &s;
    }
    return nullptr;
}

auto value(const sDiagStatus& s, const std::string& key) -> std::string {
    for (const auto& kv : s.values) {
        if (kv.key == key) return kv.value;
    }
    return "<absent>";
}

auto inf() -> double { return std::numeric_limits<double>::infinity(); }

const char* kScanSpec = "topics:\n  /scan: {min_hz: 18, max_hz: 22}\nnodes: [/perception]\n";

}  // namespace

TEST(DiagnosticsMapper, RuleInRangeIsOkWithItsNumbers) {
    const auto sp = spec(kScanSpec);
    sDiagOptions opts;
    opts.spec = &sp;
    const auto out = mapDiagnostics({scanLog(20.0)}, opts);
    ASSERT_EQ(out.size(), 3u);  // log, node, rule
    const auto* log = find(out, "ros2_pulse: log topic_freq.1.log");
    ASSERT_NE(log, nullptr);
    EXPECT_EQ(log->level, eDiagLevel::Ok);
    const auto* node = find(out, "ros2_pulse: node /perception");
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->level, eDiagLevel::Ok);
    const auto* scan = find(out, "ros2_pulse: /scan (recv)");
    ASSERT_NE(scan, nullptr);
    EXPECT_EQ(scan->level, eDiagLevel::Ok);
    EXPECT_EQ(scan->message, "ok");
    EXPECT_EQ(value(*scan, "hz"), "20.000");
    EXPECT_EQ(value(*scan, "min_hz"), "18");
    EXPECT_EQ(value(*scan, "max_hz"), "22");
    EXPECT_EQ(value(*scan, "max_dt_ms"), "21.284");
    EXPECT_EQ(value(*scan, "max_gap_ms"), "<absent>");
}

TEST(DiagnosticsMapper, RateWarnBecomesWarn) {
    const auto sp = spec(kScanSpec);
    sDiagOptions opts;
    opts.spec = &sp;
    auto log = scanLog(1.2);
    log.window.warns.push_back(rateWarn("/scan", 1.2, 18.0, 22.0));
    const auto out = mapDiagnostics({log}, opts);
    const auto* scan = find(out, "ros2_pulse: /scan (recv)");
    ASSERT_NE(scan, nullptr);
    EXPECT_EQ(scan->level, eDiagLevel::Warn);
    EXPECT_EQ(scan->message, "rate 1.200 Hz outside [18, 22]");
    EXPECT_EQ(value(*scan, "hz"), "1.200");
}

TEST(DiagnosticsMapper, GapWarnAndRateWarnShareTheRuleStatus) {
    const auto sp = spec("topics:\n  /scan: {min_hz: 18, max_gap_ms: 60}\n");
    sDiagOptions opts;
    opts.spec = &sp;
    auto log = scanLog(1.2);
    log.window.stats[0].recv_max_dt_ms = 812.4;
    log.window.warns.push_back(rateWarn("/scan", 1.2, 18.0, inf()));
    sRateWarning gap;
    gap.kind = eWarnKind::kTopicGap;
    gap.name = "/scan";
    gap.max_dt_ms = 812.4;
    gap.max_gap_ms = 60.0;
    log.window.warns.push_back(gap);
    const auto out = mapDiagnostics({log}, opts);
    const auto* scan = find(out, "ros2_pulse: /scan (recv)");
    ASSERT_NE(scan, nullptr);
    EXPECT_EQ(scan->level, eDiagLevel::Warn);
    EXPECT_EQ(scan->message, "rate 1.200 Hz outside [18, inf]; gap 812.400 ms over 60 ms");
    EXPECT_EQ(value(*scan, "max_hz"), "<absent>");
    EXPECT_EQ(value(*scan, "max_gap_ms"), "60");
    EXPECT_EQ(value(*scan, "max_dt_ms"), "812.400");
    EXPECT_EQ(find(out, "ros2_pulse: /scan"), nullptr);  // matched, so no loose status
}

// Two rules of one topic with the same bounds: the warn's hz says which side it is about.
TEST(DiagnosticsMapper, WarnMatchedToTheSideWhoseRateItCarries) {
    const auto sp = spec("topics:\n  /scan: {min_hz: 18, side: pub}\n  /scan: {min_hz: 18}\n");
    sDiagOptions opts;
    opts.spec = &sp;
    auto log = scanLog(5.0);
    log.window.stats[0].pub_inter_hz = 20.0;
    log.window.warns.push_back(rateWarn("/scan", 5.0, 18.0, inf()));
    const auto out = mapDiagnostics({log}, opts);
    const auto* pub = find(out, "ros2_pulse: /scan (pub)");
    const auto* recv = find(out, "ros2_pulse: /scan (recv)");
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(recv, nullptr);
    EXPECT_EQ(pub->level, eDiagLevel::Ok);
    EXPECT_EQ(recv->level, eDiagLevel::Warn);
}

TEST(DiagnosticsMapper, NodeMissingIsError) {
    const auto sp = spec("nodes: [/perception, /planner]\n");
    sDiagOptions opts;
    opts.spec = &sp;
    auto log = scanLog(20.0);
    sRateWarning w;
    w.kind = eWarnKind::kNodeMissing;
    w.name = "/planner";
    log.window.warns.push_back(w);
    const auto out = mapDiagnostics({log}, opts);
    ASSERT_EQ(out.size(), 3u);
    EXPECT_EQ(out[1].name, "ros2_pulse: node /perception");  // spec order
    EXPECT_EQ(out[1].level, eDiagLevel::Ok);
    EXPECT_EQ(out[2].name, "ros2_pulse: node /planner");
    EXPECT_EQ(out[2].level, eDiagLevel::Error);
    EXPECT_EQ(out[2].message, "missing");
}

// A spec node or rule no log hosts belongs to another machine's bridge: no status at all.
TEST(DiagnosticsMapper, UnhostedRulesAndNodesAreLeftOut) {
    const auto sp = spec("topics:\n  /odom: {min_hz: 50}\n  /scan: {min_hz: 18, side: pub}\n"
                         "nodes: [/base]\n");
    sDiagOptions opts;
    opts.spec = &sp;
    const auto out = mapDiagnostics({scanLog(20.0)}, opts);  // receives /scan, publishes nothing
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].name, "ros2_pulse: log topic_freq.1.log");
}

TEST(DiagnosticsMapper, StaleLogIsErrorAndItsWindowIsNotUsed) {
    const auto sp = spec(kScanSpec);
    sDiagOptions opts;
    opts.spec = &sp;
    opts.poll_period_s = 1.0;
    auto log = scanLog(20.0);
    log.age_s = 11.5;  // over 2 * 5 s + 1 s
    const auto out = mapDiagnostics({log}, opts);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].level, eDiagLevel::Error);
    EXPECT_EQ(out[0].message, "stale: no window for 11.5 s");
    log.age_s = 10.5;
    EXPECT_EQ(mapDiagnostics({log}, opts)[0].level, eDiagLevel::Ok);
    log.age_s = 301.0;
    EXPECT_TRUE(mapDiagnostics({log}, opts).empty());
}

// Only a partial (attach) window so far: the period is not known, the probe's default is used.
TEST(DiagnosticsMapper, UnknownPeriodUsesTheProbeDefault) {
    sDiagOptions opts;
    auto log = scanLog(20.0);
    log.period_s = 0.0;
    log.window.window_s = 0.3;
    log.window.partial = true;
    log.age_s = 4.0;
    EXPECT_EQ(mapDiagnostics({log}, opts)[0].level, eDiagLevel::Ok);
}

// The exit window (partial, after a full one, so the period is known): a clean exit drops the
// log and everything it reported, fresh or not, instead of going stale.
TEST(DiagnosticsMapper, ExitWindowDropsTheLog) {
    const auto sp = spec(kScanSpec);
    sDiagOptions opts;
    opts.spec = &sp;
    auto log = scanLog(20.0);
    log.window.partial = true;
    log.window.window_s = 0.4;
    EXPECT_TRUE(mapDiagnostics({log}, opts).empty());
    log.age_s = 60.0;
    EXPECT_TRUE(mapDiagnostics({log}, opts).empty());
    // Another process still running keeps its statuses.
    const auto out = mapDiagnostics({log, scanLog(20.0, "/tmp/topic_freq.2.log")}, opts);
    ASSERT_EQ(out.size(), 3u);
    EXPECT_EQ(out[0].name, "ros2_pulse: log topic_freq.2.log");
}

// No spec: a warn still surfaces, under the topic alone since its side is not known.
TEST(DiagnosticsMapper, WarnWithoutSpecIsLooseWarn) {
    sDiagOptions opts;
    auto log = scanLog(1.2);
    log.window.warns.push_back(rateWarn("/scan", 1.2, 18.0, 22.0));
    const auto out = mapDiagnostics({log}, opts);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_EQ(out[1].name, "ros2_pulse: /scan");
    EXPECT_EQ(out[1].level, eDiagLevel::Warn);
    EXPECT_EQ(value(out[1], "min_hz"), "18");
}

TEST(DiagnosticsMapper, AllTopicsAddsUnruledTopicsAsOk) {
    const auto sp = spec(kScanSpec);
    sDiagOptions opts;
    opts.spec = &sp;
    opts.all_topics = true;
    auto log = scanLog(20.0);
    sTopicStat odom;
    odom.topic = "/odom";
    odom.pub_inter_hz = 50.0;
    log.window.stats.push_back(odom);
    sTopicStat rosout;
    rosout.topic = "/rosout";
    rosout.pub_inter_hz = 1.0;
    log.window.stats.push_back(rosout);
    const auto out = mapDiagnostics({log}, opts);
    ASSERT_EQ(out.size(), 4u);  // log, node, /scan rule, /odom
    EXPECT_EQ(out[3].name, "ros2_pulse: /odom (pub)");
    EXPECT_EQ(out[3].level, eDiagLevel::Ok);
    EXPECT_EQ(value(out[3], "hz"), "50.000");
    opts.all_topics = false;
    EXPECT_EQ(mapDiagnostics({log}, opts).size(), 3u);
}

// With the per-endpoint split, sides come from the endpoints: a recv rule is not reported by a
// process that only publishes the topic, even though its topic line exists.
TEST(DiagnosticsMapper, EndpointsDecideWhichSideALogHosts) {
    const auto sp = spec("topics:\n  /scan: {min_hz: 18}\n");
    sDiagOptions opts;
    opts.spec = &sp;
    auto log = scanLog(0.0);
    log.window.stats[0].pub_inter_hz = 20.0;
    sEndpointStat e;
    e.topic = "/scan";
    e.side = eEndpointSide::kPub;
    e.node = "/perception";
    e.inter_hz = 20.0;
    log.window.endpoints.push_back(e);
    log.window.warns.push_back(rateWarn("/scan", 0.0, 18.0, inf()));
    const auto out = mapDiagnostics({log}, opts);
    EXPECT_EQ(find(out, "ros2_pulse: /scan (recv)"), nullptr);
    EXPECT_EQ(find(out, "ros2_pulse: /scan"), nullptr);
}

// Two processes receiving /scan: one status, the worst level and that process's rate.
TEST(DiagnosticsMapper, LogsFoldIntoOneStatusWorstFirst) {
    const auto sp = spec(kScanSpec);
    sDiagOptions opts;
    opts.spec = &sp;
    auto slow = scanLog(3.0, "/tmp/topic_freq.2.log");
    slow.window.warns.push_back(rateWarn("/scan", 3.0, 18.0, 22.0));
    const auto out = mapDiagnostics({scanLog(20.0), slow}, opts);
    const auto* scan = find(out, "ros2_pulse: /scan (recv)");
    ASSERT_NE(scan, nullptr);
    EXPECT_EQ(scan->level, eDiagLevel::Warn);
    EXPECT_EQ(value(*scan, "hz"), "3.000");
    const auto ok = mapDiagnostics({scanLog(20.0), scanLog(19.0, "/tmp/topic_freq.2.log")}, opts);
    EXPECT_EQ(value(*find(ok, "ros2_pulse: /scan (recv)"), "hz"), "19.000");
}

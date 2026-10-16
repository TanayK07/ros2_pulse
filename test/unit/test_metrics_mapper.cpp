// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the /statistics mapper behind pulse_bridge: one probe window becomes one
// statistics_msgs-shaped sample per topic, in the units the built-in rclcpp topic statistics
// use (message period in ms), so existing /statistics consumers read pulse windows unchanged.
// Pure core, no ROS types. Registers into the shared test binary.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/metrics_mapper.hpp"

using ros2_pulse::core::eMetricsSource;
using ros2_pulse::core::eRateUnit;
using ros2_pulse::core::mapWindow;
using ros2_pulse::core::sLogWindow;
using ros2_pulse::core::sMapperOptions;
using ros2_pulse::core::sMetricSample;
using ros2_pulse::core::sTopicStat;

namespace {

constexpr long long kTs = 1782887153899445923LL;

auto scanWindow() -> sLogWindow {
    sLogWindow w;
    w.ts_ns = kTs;
    w.window_s = 5.0;
    sTopicStat s;
    s.topic = "/scan";
    s.pub_inter_hz = 20.0;
    s.recv_inter_hz = 20.0;
    s.recv_endpoint_seen = true;
    s.recv_max_dt_ms = 21.284;
    s.has_recv_max_dt = true;
    w.stats.push_back(s);
    w.nodes = {"/perception"};
    return w;
}

auto only(std::vector<sMetricSample> v) -> sMetricSample {
    EXPECT_EQ(v.size(), 1u);
    return v.empty() ? sMetricSample{} : v.front();
}

}  // namespace

TEST(MetricsMapper, RecvSideBecomesMessagePeriodInMs) {
    const auto m = only(mapWindow(scanWindow(), sMapperOptions{}));
    EXPECT_EQ(m.metrics_source, "/scan");
    EXPECT_EQ(m.measurement_source_name, "/perception");
    EXPECT_EQ(m.unit, "ms");
    EXPECT_TRUE(m.has_average);
    EXPECT_DOUBLE_EQ(m.average, 50.0);  // 20 Hz -> 50 ms period
    EXPECT_DOUBLE_EQ(m.sample_count, 100.0);  // 20 Hz * 5 s
    EXPECT_TRUE(m.has_maximum);
    EXPECT_DOUBLE_EQ(m.maximum, 21.284);
}

TEST(MetricsMapper, WindowBoundsAreStopMinusLengthAndStop) {
    const auto m = only(mapWindow(scanWindow(), sMapperOptions{}));
    EXPECT_EQ(m.window_stop_ns, kTs);
    EXPECT_EQ(m.window_start_ns, kTs - 5000000000LL);
}

TEST(MetricsMapper, PubSideWhenNoSubscriptionSeen) {
    auto w = scanWindow();
    w.stats[0].recv_inter_hz = 0.0;
    w.stats[0].recv_endpoint_seen = false;
    w.stats[0].has_recv_max_dt = false;
    w.stats[0].pub_inter_hz = 40.0;
    const auto m = only(mapWindow(w, sMapperOptions{}));
    EXPECT_DOUBLE_EQ(m.average, 25.0);
    EXPECT_DOUBLE_EQ(m.sample_count, 200.0);
    EXPECT_FALSE(m.has_maximum);  // pub-side gap is not reported
}

TEST(MetricsMapper, InterAndIntraReportTheBusierPathNotTheSum) {
    // One publish can hit BOTH rclcpp_intra_publish and rcl_publish; summing would double it.
    auto w = scanWindow();
    w.stats[0].recv_inter_hz = 20.0;
    w.stats[0].recv_intra_hz = 30.0;
    const auto m = only(mapWindow(w, sMapperOptions{}));
    EXPECT_DOUBLE_EQ(m.average, 1000.0 / 30.0);
    EXPECT_DOUBLE_EQ(m.sample_count, 150.0);
}

TEST(MetricsMapper, ZeroRateKeepsSampleCountButNoPeriod) {
    // A dead upstream reads as 0 messages, never as an infinite or zero period.
    auto w = scanWindow();
    w.stats[0].recv_inter_hz = 0.0;
    w.stats[0].pub_inter_hz = 0.0;
    const auto m = only(mapWindow(w, sMapperOptions{}));
    EXPECT_FALSE(m.has_average);
    EXPECT_DOUBLE_EQ(m.sample_count, 0.0);
}

TEST(MetricsMapper, RateUnitPublishesHzAndNoGap) {
    sMapperOptions o;
    o.unit = eRateUnit::Rate_hz;
    const auto m = only(mapWindow(scanWindow(), o));
    EXPECT_EQ(m.unit, "Hz");
    EXPECT_TRUE(m.has_average);
    EXPECT_DOUBLE_EQ(m.average, 20.0);
    EXPECT_FALSE(m.has_maximum);  // a gap in ms would mix units
}

TEST(MetricsMapper, RateUnitReportsZeroHzAsAValue) {
    auto w = scanWindow();
    w.stats[0].recv_inter_hz = 0.0;
    w.stats[0].pub_inter_hz = 0.0;
    sMapperOptions o;
    o.unit = eRateUnit::Rate_hz;
    const auto m = only(mapWindow(w, o));
    EXPECT_TRUE(m.has_average);
    EXPECT_DOUBLE_EQ(m.average, 0.0);
}

TEST(MetricsMapper, SourceNameFallsBackWhenNodesAreAmbiguous) {
    auto w = scanWindow();
    w.nodes = {"/a", "/b"};
    EXPECT_EQ(only(mapWindow(w, sMapperOptions{})).measurement_source_name, "ros2_pulse");
    w.nodes.clear();
    EXPECT_EQ(only(mapWindow(w, sMapperOptions{})).measurement_source_name, "ros2_pulse");
}

TEST(MetricsMapper, SourceNameOverrideWins) {
    sMapperOptions o;
    o.source_name = "orin-planner";
    EXPECT_EQ(only(mapWindow(scanWindow(), o)).measurement_source_name, "orin-planner");
}

TEST(MetricsMapper, OneSamplePerTopic) {
    auto w = scanWindow();
    sTopicStat t;
    t.topic = "/tf";
    t.pub_inter_hz = 100.0;
    w.stats.push_back(t);
    const auto v = mapWindow(w, sMapperOptions{});
    ASSERT_EQ(v.size(), 2u);
    EXPECT_EQ(v[0].metrics_source, "/scan");
    EXPECT_EQ(v[1].metrics_source, "/tf");
    EXPECT_DOUBLE_EQ(v[1].average, 10.0);
}

// rclcpp's layout (metrics_source:=metric) mirrors upstream's per-subscription collectors: a
// publisher-only topic has none there, and without the topic in the message its publish rate
// would be indistinguishable from a receive period, so it is skipped. The topic layout keeps it.
TEST(MetricsMapper, MetricLayoutSkipsPublisherOnlyTopics) {
    auto w = scanWindow();
    sTopicStat t;
    t.topic = "/tf";
    t.pub_inter_hz = 100.0;
    w.stats.push_back(t);
    sMapperOptions o;
    o.metrics_source = eMetricsSource::Metric;
    const auto v = mapWindow(w, o);
    ASSERT_EQ(v.size(), 2u) << "/scan only: message_age + message_period";
    EXPECT_EQ(v[0].metrics_source, "message_age");
    EXPECT_EQ(v[1].metrics_source, "message_period");
    EXPECT_DOUBLE_EQ(v[1].average, 50.0);  // /scan's receive period, not /tf's 10 ms
    EXPECT_EQ(mapWindow(w, sMapperOptions{}).size(), 2u) << "topic layout keeps /tf";
}

// Every receive-side topic gets a message_age, before its message_period (upstream's order),
// even with no age sample this window: empty, SAMPLE_COUNT 0, no measured point.
TEST(MetricsMapper, MetricLayoutAlwaysHasMessageAgeFirst) {
    sMapperOptions o;
    o.metrics_source = eMetricsSource::Metric;
    const auto v = mapWindow(scanWindow(), o);  // no recv_age in this window
    ASSERT_EQ(v.size(), 2u);
    const auto& a = v[0];
    EXPECT_EQ(a.metrics_source, "message_age");
    EXPECT_EQ(a.unit, "ms");
    EXPECT_EQ(a.measurement_source_name, "/perception");
    EXPECT_EQ(a.window_start_ns, v[1].window_start_ns);
    EXPECT_EQ(a.window_stop_ns, v[1].window_stop_ns);
    EXPECT_FALSE(a.has_average || a.has_minimum || a.has_maximum || a.has_stddev);
    EXPECT_DOUBLE_EQ(a.sample_count, 0.0);
    EXPECT_EQ(v[1].metrics_source, "message_period");
    EXPECT_DOUBLE_EQ(v[1].sample_count, 100.0);

    // A proven subscription at 0 Hz is still receive side: both messages, both empty-ish.
    auto idle = scanWindow();
    idle.stats[0].recv_inter_hz = 0.0;
    idle.stats[0].pub_inter_hz = 0.0;
    idle.stats[0].has_recv_max_dt = false;
    const auto w = mapWindow(idle, o);
    ASSERT_EQ(w.size(), 2u);
    EXPECT_DOUBLE_EQ(w[0].sample_count, 0.0);
    EXPECT_FALSE(w[1].has_average);
}

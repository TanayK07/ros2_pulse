// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the rosgraph_monitor_msgs/TopicStatistics mapper behind pulse_bridge's
// graph-monitor output (ros-tooling/graph-monitor#14): one probe window becomes one batch of
// per-endpoint period statistics keyed by (node, topic), the shape graph-monitor matches against
// its graph. Pure core, no ROS types. Registers into the shared test binary.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/topic_statistic_mapper.hpp"

using ros2_pulse::core::eEndpointSide;
using ros2_pulse::core::eTopicStatisticType;
using ros2_pulse::core::mapWindowTopicStatistics;
using ros2_pulse::core::sEndpointStat;
using ros2_pulse::core::sLogWindow;
using ros2_pulse::core::sTopicStat;

namespace {

constexpr long long kTs = 1782887153899445923LL;

// One process, one node, publishing /scan at 20 Hz and receiving it back at 20 Hz.
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

auto endpoint(const std::string& node, const std::string& topic, eEndpointSide side, double hz)
    -> sEndpointStat {
    sEndpointStat e;
    e.node = node;
    e.topic = topic;
    e.side = side;
    e.inter_hz = hz;
    e.endpoint_seen = side == eEndpointSide::kRecv;
    return e;
}

// The same window as written by a probe with the per-endpoint split: the topic sums stay, and
// the endpoints say which node owns what.
auto scanWindowWithEndpoints() -> sLogWindow {
    auto w = scanWindow();
    w.endpoints.push_back(endpoint("/perception", "/scan", eEndpointSide::kPub, 20.0));
    auto r = endpoint("/perception", "/scan", eEndpointSide::kRecv, 20.0);
    r.max_dt_ms = 21.284;
    r.has_max_dt = true;
    w.endpoints.push_back(r);
    return w;
}

}  // namespace

TEST(TopicStatisticMapper, PublishAndReceiveBecomeOneStatisticEach) {
    const auto b = mapWindowTopicStatistics(scanWindow());
    EXPECT_EQ(b.timestamp_ns, kTs);
    EXPECT_FALSE(b.ambiguous_node);
    ASSERT_EQ(b.statistics.size(), 2u);

    const auto& pub = b.statistics[0];
    EXPECT_EQ(pub.type, eTopicStatisticType::PublishedPeriod);
    EXPECT_EQ(pub.node_name, "/perception");
    EXPECT_EQ(pub.topic_name, "/scan");
    EXPECT_EQ(pub.window_count, 100);       // 20 Hz * 5 s
    EXPECT_EQ(pub.mean_ns, 50000000LL);     // 50 ms period
    EXPECT_FALSE(pub.has_max);              // no pub-side gap measured
    EXPECT_EQ(pub.max_ns, 0);

    const auto& recv = b.statistics[1];
    EXPECT_EQ(recv.type, eTopicStatisticType::ReceivedPeriod);
    EXPECT_EQ(recv.node_name, "/perception");
    EXPECT_EQ(recv.window_count, 100);
    EXPECT_EQ(recv.mean_ns, 50000000LL);
    EXPECT_TRUE(recv.has_max);
    EXPECT_EQ(recv.max_ns, 21284000LL);
}

TEST(TopicStatisticMapper, PubSideGapFillsMax) {
    auto w = scanWindow();
    w.stats[0].pub_max_dt_ms = 60.5;
    w.stats[0].has_pub_max_dt = true;
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_TRUE(b.statistics[0].has_max);
    EXPECT_EQ(b.statistics[0].max_ns, 60500000LL);
}

TEST(TopicStatisticMapper, PurePublisherReportsOnlyPublishedPeriod) {
    auto w = scanWindow();
    w.stats[0].recv_inter_hz = 0.0;
    w.stats[0].recv_endpoint_seen = false;
    w.stats[0].has_recv_max_dt = false;
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 1u);
    EXPECT_EQ(b.statistics[0].type, eTopicStatisticType::PublishedPeriod);
}

TEST(TopicStatisticMapper, ZeroRateSideIsLeftOutNotInfinite) {
    // A dead upstream: RECV 0.0 is in the log, but a period cannot express it. Leaving it out
    // lets graph-monitor's stale timeout flag the topic.
    auto w = scanWindow();
    w.stats[0].recv_inter_hz = 0.0;
    w.stats[0].pub_inter_hz = 0.0;
    const auto b = mapWindowTopicStatistics(w);
    EXPECT_TRUE(b.statistics.empty());
    EXPECT_FALSE(b.ambiguous_node);
}

TEST(TopicStatisticMapper, InterAndIntraReportTheBusierPathNotTheSum) {
    auto w = scanWindow();
    w.stats[0].recv_inter_hz = 20.0;
    w.stats[0].recv_intra_hz = 40.0;
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_EQ(b.statistics[1].mean_ns, 25000000LL);
    EXPECT_EQ(b.statistics[1].window_count, 200);
}

TEST(TopicStatisticMapper, OlderProbeSeveralNodesInOneProcessAreNotGuessed) {
    // Without the per-endpoint split the probe's rates are per topic per process; handing the
    // process total to every node would report a wrong period for each, so nothing is emitted
    // and the caller is told why.
    auto w = scanWindow();
    w.nodes = {"/a", "/b"};
    const auto b = mapWindowTopicStatistics(w);
    EXPECT_TRUE(b.statistics.empty());
    EXPECT_TRUE(b.ambiguous_node);
}

TEST(TopicStatisticMapper, NoActiveNodeIsAmbiguousToo) {
    auto w = scanWindow();
    w.nodes.clear();
    const auto b = mapWindowTopicStatistics(w);
    EXPECT_TRUE(b.statistics.empty());
    EXPECT_TRUE(b.ambiguous_node);
}

TEST(TopicStatisticMapper, IdleWindowWithSeveralNodesIsNotAmbiguous) {
    auto w = scanWindow();
    w.nodes = {"/a", "/b"};
    w.stats[0].recv_inter_hz = 0.0;
    w.stats[0].pub_inter_hz = 0.0;
    EXPECT_FALSE(mapWindowTopicStatistics(w).ambiguous_node);
}

TEST(TopicStatisticMapper, EveryActiveTopicIsInOneBatch) {
    auto w = scanWindow();
    sTopicStat t;
    t.topic = "/tf";
    t.pub_inter_hz = 100.0;
    w.stats.push_back(t);
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 3u);
    EXPECT_EQ(b.statistics[2].topic_name, "/tf");
    EXPECT_EQ(b.statistics[2].type, eTopicStatisticType::PublishedPeriod);
    EXPECT_EQ(b.statistics[2].mean_ns, 10000000LL);
    EXPECT_EQ(b.statistics[2].window_count, 500);
}

TEST(TopicStatisticMapper, ParsedJsonlWindowMapsEndToEnd) {
    const std::string line =
        "{\"ts_ns\":\"1782887153899445923\",\"window_s\":5.000,\"topics\":[{\"topic\":\"/scan\","
        "\"pub_inter_hz\":0.000000,\"pub_intra_hz\":0.000000,\"recv_inter_hz\":10.000000,"
        "\"recv_intra_hz\":0.000000,\"recv_endpoint_seen\":true}],"
        "\"nodes\":[\"/ns/lidar_filter\"],\"warns\":[]}\n";
    const auto wins = ros2_pulse::core::parseLog(line);
    ASSERT_EQ(wins.size(), 1u);
    const auto b = mapWindowTopicStatistics(wins[0]);
    ASSERT_EQ(b.statistics.size(), 1u);
    EXPECT_EQ(b.statistics[0].type, eTopicStatisticType::ReceivedPeriod);
    EXPECT_EQ(b.statistics[0].node_name, "/ns/lidar_filter");
    EXPECT_EQ(b.statistics[0].mean_ns, 100000000LL);
    EXPECT_EQ(b.statistics[0].window_count, 50);
    EXPECT_EQ(b.timestamp_ns, kTs);
}

// ---- per-endpoint split (probe 0.7+) ----

TEST(TopicStatisticMapper, EndpointsMapLikeTheSingleNodeCase) {
    const auto b = mapWindowTopicStatistics(scanWindowWithEndpoints());
    EXPECT_FALSE(b.ambiguous_node);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_EQ(b.statistics[0].type, eTopicStatisticType::PublishedPeriod);
    EXPECT_EQ(b.statistics[0].node_name, "/perception");
    EXPECT_EQ(b.statistics[0].mean_ns, 50000000LL);
    EXPECT_EQ(b.statistics[0].window_count, 100);
    EXPECT_FALSE(b.statistics[0].has_max);
    EXPECT_EQ(b.statistics[1].type, eTopicStatisticType::ReceivedPeriod);
    EXPECT_TRUE(b.statistics[1].has_max);
    EXPECT_EQ(b.statistics[1].max_ns, 21284000LL);
}

TEST(TopicStatisticMapper, ComposedProcessReportsEveryNode) {
    // A component container: /driver publishes /scan at 20 Hz, /filter receives it and
    // publishes /scan_filtered at 10 Hz. Before the per-endpoint split this window was dropped.
    sLogWindow w;
    w.ts_ns = kTs;
    w.window_s = 5.0;
    w.nodes = {"/driver", "/filter"};
    w.endpoints.push_back(endpoint("/driver", "/scan", eEndpointSide::kPub, 20.0));
    w.endpoints.push_back(endpoint("/filter", "/scan", eEndpointSide::kRecv, 20.0));
    w.endpoints.push_back(endpoint("/filter", "/scan_filtered", eEndpointSide::kPub, 10.0));
    const auto b = mapWindowTopicStatistics(w);
    EXPECT_FALSE(b.ambiguous_node);
    ASSERT_EQ(b.statistics.size(), 3u);
    EXPECT_EQ(b.statistics[0].node_name, "/driver");
    EXPECT_EQ(b.statistics[0].type, eTopicStatisticType::PublishedPeriod);
    EXPECT_EQ(b.statistics[1].node_name, "/filter");
    EXPECT_EQ(b.statistics[1].type, eTopicStatisticType::ReceivedPeriod);
    EXPECT_EQ(b.statistics[1].mean_ns, 50000000LL);
    EXPECT_EQ(b.statistics[2].node_name, "/filter");
    EXPECT_EQ(b.statistics[2].topic_name, "/scan_filtered");
    EXPECT_EQ(b.statistics[2].mean_ns, 100000000LL);
}

TEST(TopicStatisticMapper, TwoSubscriptionsOfOneNodeReportOnePeriodNotHalfOfIt) {
    // One node subscribing twice to a 5 Hz topic: the topic sum reads 10 Hz (a 100 ms period
    // that would pass a 150 ms deadline), each subscription 5 Hz. One statistic, 200 ms.
    sLogWindow w;
    w.ts_ns = kTs;
    w.window_s = 5.0;
    w.nodes = {"/n"};
    sTopicStat s;
    s.topic = "/x";
    s.recv_inter_hz = 10.0;
    s.recv_endpoint_seen = true;
    w.stats.push_back(s);
    w.endpoints.push_back(endpoint("/n", "/x", eEndpointSide::kRecv, 5.0));
    w.endpoints.push_back(endpoint("/n", "/x", eEndpointSide::kRecv, 5.0));
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 1u);
    EXPECT_EQ(b.statistics[0].mean_ns, 200000000LL);
    EXPECT_EQ(b.statistics[0].window_count, 25);
}

TEST(TopicStatisticMapper, SharedKeyReportsTheSlowestHandle) {
    sLogWindow w;
    w.window_s = 5.0;
    w.endpoints.push_back(endpoint("/n", "/x", eEndpointSide::kRecv, 10.0));
    auto slow = endpoint("/n", "/x", eEndpointSide::kRecv, 4.0);
    slow.max_dt_ms = 400.0;
    slow.has_max_dt = true;
    w.endpoints.push_back(slow);
    w.endpoints.push_back(endpoint("/n", "/x", eEndpointSide::kRecv, 8.0));
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 1u);
    EXPECT_EQ(b.statistics[0].mean_ns, 250000000LL);
    EXPECT_TRUE(b.statistics[0].has_max);
    EXPECT_EQ(b.statistics[0].max_ns, 400000000LL);
}

TEST(TopicStatisticMapper, StalledHandleMakesItsKeyDead) {
    sLogWindow w;
    w.window_s = 5.0;
    w.endpoints.push_back(endpoint("/n", "/x", eEndpointSide::kRecv, 10.0));
    w.endpoints.push_back(endpoint("/n", "/x", eEndpointSide::kRecv, 0.0));
    w.endpoints.push_back(endpoint("/n", "/y", eEndpointSide::kRecv, 2.0));
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 1u);
    EXPECT_EQ(b.statistics[0].topic_name, "/y");
}

TEST(TopicStatisticMapper, PublisherAndSubscriptionOfOneNodeAreSeparateKeys) {
    sLogWindow w;
    w.window_s = 5.0;
    w.endpoints.push_back(endpoint("/n", "/x", eEndpointSide::kPub, 20.0));
    w.endpoints.push_back(endpoint("/n", "/x", eEndpointSide::kRecv, 10.0));
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_EQ(b.statistics[0].mean_ns, 50000000LL);
    EXPECT_EQ(b.statistics[1].mean_ns, 100000000LL);
}

TEST(TopicStatisticMapper, LogOnlyActivityOfASecondNodeNoLongerSuppressesOutput) {
    // /b only logged this window: the probe lists it as active, which made the single-node gate
    // drop the whole window. Its /rosout endpoint is filtered, /a still reports.
    auto w = scanWindowWithEndpoints();
    w.nodes = {"/perception", "/b"};
    w.endpoints.push_back(endpoint("/b", "/rosout", eEndpointSide::kPub, 1.0));
    w.endpoints.push_back(endpoint("/b", "/parameter_events", eEndpointSide::kPub, 1.0));
    const auto b = mapWindowTopicStatistics(w);
    EXPECT_FALSE(b.ambiguous_node);
    ASSERT_EQ(b.statistics.size(), 2u);
    for (const auto& st : b.statistics) EXPECT_EQ(st.node_name, "/perception");
}

TEST(TopicStatisticMapper, EndpointOfAnUnknownNodeIsLeftOut) {
    auto w = scanWindowWithEndpoints();
    w.endpoints.push_back(endpoint("", "/odom", eEndpointSide::kPub, 50.0));
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_EQ(b.statistics[0].topic_name, "/scan");
    EXPECT_EQ(b.statistics[1].topic_name, "/scan");
}

TEST(TopicStatisticMapper, IntraEndpointReportsTheBusierPath) {
    sLogWindow w;
    w.window_s = 5.0;
    auto e = endpoint("/n", "/x", eEndpointSide::kRecv, 2.0);
    e.intra_hz = 20.0;
    w.endpoints.push_back(e);
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 1u);
    EXPECT_EQ(b.statistics[0].mean_ns, 50000000LL);
}

TEST(TopicStatisticMapper, ParsedJsonlEndpointsMapEndToEnd) {
    const std::string line =
        "{\"ts_ns\":\"1782887153899445923\",\"window_s\":5.000,\"topics\":[{\"topic\":\"/scan\","
        "\"recv_inter_hz\":10.000000,\"recv_intra_hz\":0.000000,\"recv_endpoint_seen\":true}],"
        "\"nodes\":[\"/a\",\"/b\"],\"warns\":[],\"endpoints\":["
        "{\"topic\":\"/scan\",\"side\":\"recv\",\"id\":4,\"node\":\"/a\",\"inter_hz\":5.000000,"
        "\"intra_hz\":0.000000},"
        "{\"topic\":\"/scan\",\"side\":\"recv\",\"id\":5,\"node\":\"/b\",\"inter_hz\":5.000000,"
        "\"intra_hz\":0.000000,\"max_dt_ms\":201.500}]}\n";
    const auto wins = ros2_pulse::core::parseLog(line);
    ASSERT_EQ(wins.size(), 1u);
    const auto b = mapWindowTopicStatistics(wins[0]);
    EXPECT_FALSE(b.ambiguous_node);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_EQ(b.statistics[0].node_name, "/a");
    EXPECT_EQ(b.statistics[0].mean_ns, 200000000LL);
    EXPECT_FALSE(b.statistics[0].has_max);
    EXPECT_EQ(b.statistics[1].node_name, "/b");
    EXPECT_EQ(b.statistics[1].max_ns, 201500000LL);
}

// ---- receive distributions and TAKE_AGE (per-subscription, gap tracking on) ----

namespace {

auto measuredRecv() -> sEndpointStat {
    auto r = endpoint("/n", "/scan", eEndpointSide::kRecv, 20.0);
    r.max_dt_ms = 61.0;
    r.has_max_dt = true;
    r.has_recv_dt = true;
    r.recv_dt.count = 99;
    r.recv_dt.mean_ms = 49.5;
    r.recv_dt.min_ms = 45.25;
    r.has_recv_age = true;
    r.recv_age.count = 100;
    r.recv_age.mean_ms = 1.5;
    r.recv_age.min_ms = 0.75;
    r.recv_age.max_ms = 4.0;
    return r;
}

}  // namespace

TEST(TopicStatisticMapper, ReceivedPeriodUsesTheIntervalDistribution) {
    sLogWindow w;
    w.window_s = 5.0;
    w.endpoints.push_back(measuredRecv());
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 2u);
    const auto& p = b.statistics[0];
    EXPECT_EQ(p.type, eTopicStatisticType::ReceivedPeriod);
    EXPECT_EQ(p.window_count, 100);       // still messages in the window
    EXPECT_EQ(p.mean_ns, 49500000LL);     // interval mean, not 1 / 20 Hz
    EXPECT_TRUE(p.has_min);
    EXPECT_EQ(p.min_ns, 45250000LL);
    EXPECT_TRUE(p.has_max);
    EXPECT_EQ(p.max_ns, 61000000LL);      // the gap, open interval included
}

TEST(TopicStatisticMapper, TakeAgeFollowsItsSubscription) {
    sLogWindow w;
    w.window_s = 5.0;
    w.endpoints.push_back(measuredRecv());
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 2u);
    const auto& a = b.statistics[1];
    EXPECT_EQ(a.type, eTopicStatisticType::TakeAge);
    EXPECT_EQ(a.node_name, "/n");
    EXPECT_EQ(a.topic_name, "/scan");
    EXPECT_EQ(a.window_count, 100);
    EXPECT_EQ(a.mean_ns, 1500000LL);
    EXPECT_TRUE(a.has_min);
    EXPECT_EQ(a.min_ns, 750000LL);
    EXPECT_TRUE(a.has_max);
    EXPECT_EQ(a.max_ns, 4000000LL);
}

TEST(TopicStatisticMapper, NegativeAgeStaysSignedInTheCore) {
    sLogWindow w;
    w.window_s = 5.0;
    auto r = measuredRecv();
    r.recv_age.min_ms = -2.0;
    w.endpoints.push_back(r);
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_EQ(b.statistics[1].min_ns, -2000000LL);  // the bridge clamps, the core does not
}

TEST(TopicStatisticMapper, PublishedPeriodHasNoMinimumOrAge) {
    sLogWindow w;
    w.window_s = 5.0;
    auto p = endpoint("/n", "/scan", eEndpointSide::kPub, 20.0);
    p.has_recv_dt = true;  // never set by the probe on a publisher; ignored if it were
    p.recv_dt.count = 5;
    p.recv_dt.mean_ms = 1.0;
    p.has_recv_age = true;
    p.recv_age.count = 5;
    w.endpoints.push_back(p);
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 1u);
    EXPECT_EQ(b.statistics[0].type, eTopicStatisticType::PublishedPeriod);
    EXPECT_EQ(b.statistics[0].mean_ns, 50000000LL);
    EXPECT_FALSE(b.statistics[0].has_min);
}

TEST(TopicStatisticMapper, NoAgeMeasuredNoTakeAge) {
    sLogWindow w;
    w.window_s = 5.0;
    auto r = measuredRecv();
    r.has_recv_age = false;
    r.has_recv_dt = false;
    w.endpoints.push_back(r);
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 1u);
    EXPECT_EQ(b.statistics[0].mean_ns, 50000000LL);  // back to 1 / rate
    EXPECT_FALSE(b.statistics[0].has_min);
}

TEST(TopicStatisticMapper, ParsedJsonlEndpointDistributionsMapEndToEnd) {
    const std::string line =
        "{\"ts_ns\":\"1\",\"window_s\":5.000,\"topics\":[],\"nodes\":[\"/n\"],\"warns\":[],"
        "\"endpoints\":[{\"topic\":\"/scan\",\"side\":\"recv\",\"id\":4,\"node\":\"/n\","
        "\"inter_hz\":20.000000,\"intra_hz\":0.000000,\"max_dt_ms\":61.000,"
        "\"avg_dt_ms\":49.500,\"min_dt_ms\":45.250,\"stddev_dt_ms\":1.000,\"dt_count\":99,"
        "\"age_avg_ms\":1.500,\"age_min_ms\":0.750,\"age_max_ms\":4.000,\"age_count\":100}]}\n";
    const auto wins = ros2_pulse::core::parseLog(line);
    ASSERT_EQ(wins.size(), 1u);
    const auto b = mapWindowTopicStatistics(wins[0]);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_EQ(b.statistics[0].mean_ns, 49500000LL);
    EXPECT_EQ(b.statistics[0].min_ns, 45250000LL);
    EXPECT_EQ(b.statistics[1].type, eTopicStatisticType::TakeAge);
    EXPECT_EQ(b.statistics[1].mean_ns, 1500000LL);
}

// ---- partial windows (the probe's attach / exit window) ----

TEST(TopicStatisticMapper, PartialWindowSendsNoPublishedPeriod) {
    auto w = scanWindowWithEndpoints();
    w.partial = true;
    const auto b = mapWindowTopicStatistics(w);
    // No interval distribution on the recv side either: 1 / rate would be diluted, so nothing.
    EXPECT_TRUE(b.statistics.empty());
    EXPECT_FALSE(b.ambiguous_node);
}

TEST(TopicStatisticMapper, PartialWindowKeepsAMeasuredReceivePeriodAndTheAge) {
    sLogWindow w;
    w.window_s = 5.0;
    w.partial = true;
    w.endpoints.push_back(endpoint("/n", "/scan", eEndpointSide::kPub, 4.0));
    auto r = measuredRecv();
    r.inter_hz = 4.0;  // 20 messages in a 5 s window that started 4 s before traffic
    w.endpoints.push_back(r);
    const auto b = mapWindowTopicStatistics(w);
    ASSERT_EQ(b.statistics.size(), 2u);
    EXPECT_EQ(b.statistics[0].type, eTopicStatisticType::ReceivedPeriod);
    EXPECT_EQ(b.statistics[0].mean_ns, 49500000LL);  // the intervals, not 1 / 4 Hz
    EXPECT_EQ(b.statistics[1].type, eTopicStatisticType::TakeAge);
}

TEST(TopicStatisticMapper, PartialWindowOfAnOlderLogIsSkippedWhole) {
    auto w = scanWindow();
    w.partial = true;
    EXPECT_TRUE(mapWindowTopicStatistics(w).statistics.empty());
}

TEST(TopicStatisticMapper, ParsedPartialJsonlWindowMapsEndToEnd) {
    const std::string line =
        "{\"ts_ns\":\"1\",\"window_s\":5.000,\"partial\":true,\"topics\":[],"
        "\"nodes\":[\"/n\"],\"warns\":[],\"endpoints\":[{\"topic\":\"/scan\",\"side\":\"pub\","
        "\"id\":1,\"node\":\"/n\",\"inter_hz\":4.000000,\"intra_hz\":0.000000}]}\n";
    const auto wins = ros2_pulse::core::parseLog(line);
    ASSERT_EQ(wins.size(), 1u);
    EXPECT_TRUE(wins[0].partial);
    EXPECT_TRUE(mapWindowTopicStatistics(wins[0]).statistics.empty());
}

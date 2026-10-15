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

using ros2_pulse::core::eTopicStatisticType;
using ros2_pulse::core::mapWindowTopicStatistics;
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

TEST(TopicStatisticMapper, SeveralNodesInOneProcessAreNotGuessed) {
    // The probe counts per topic per process; handing the process total to every node would
    // report a wrong period for each, so nothing is emitted and the caller is told why.
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

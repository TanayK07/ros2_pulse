// Copyright 2026 ros2_pulse contributors
//
// Per-endpoint counting and node attribution. Every rcl publisher handle and every rcl
// subscription handle has its own counter, attributed to the node whose handle its init carried;
// the per-topic numbers are summed from those at flush time. Pins the three shapes the PR #68 /
// #70 reviews found broken by a shared per-topic counter:
//  - two nodes in one process, each with its own subscription on the same topic (composition);
//  - one node with two subscriptions on the same topic;
//  - /rosout shared by every node of the process, attributed per node for liveness.
// Plus the format and reader round trip of the additive output. Registers into the shared test
// binary (no main()).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/topic_registry.hpp"
#include "ros2_pulse/core/window_format.hpp"

using ros2_pulse::core::eEndpointSide;
using ros2_pulse::core::formatWindow;
using ros2_pulse::core::formatWindowJsonl;
using ros2_pulse::core::parseLog;
using ros2_pulse::core::sEndpointStat;
using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::sWindowSnapshot;
using ros2_pulse::core::TopicRegistry;

namespace {

const void* H(uintptr_t v) { return reinterpret_cast<const void*>(v); }

auto findTopic(const std::vector<sTopicStat>& v, const std::string& t) -> const sTopicStat* {
    for (const auto& s : v) {
        if (s.topic == t) return &s;
    }
    return nullptr;
}

auto endpointsOf(const sWindowSnapshot& snap, const std::string& topic, eEndpointSide side)
    -> std::vector<sEndpointStat> {
    std::vector<sEndpointStat> out;
    for (const auto& e : snap.endpoints) {
        if (e.topic == topic && e.side == side) out.push_back(e);
    }
    return out;
}

auto findByNode(const std::vector<sEndpointStat>& v, const std::string& node)
    -> const sEndpointStat* {
    for (const auto& e : v) {
        if (e.node == node) return &e;
    }
    return nullptr;
}

bool hasNode(const std::vector<std::string>& v, const std::string& n) {
    return std::find(v.begin(), v.end(), n) != v.end();
}

// An rclcpp subscription of node @p node: rcl init, rclcpp_subscription_init, callback_added.
void initRclcppSub(TopicRegistry& reg, uintptr_t node, uintptr_t rcl, uintptr_t sub,
                   uintptr_t cb, const char* topic) {
    reg.onSubscriptionInit(H(rcl), H(node), topic, H(rcl + 1));
    reg.onRclcppSubscriptionInit(H(sub), H(rcl));
    reg.onCallbackAdded(H(cb), H(sub));
}

}  // namespace

// Composition: two nodes in one container, each with its own subscription on /odom. Each
// endpoint is attributed to its own node with its own count; the topic RECV is still the sum.
TEST(PerEndpoint, TwoNodesInOneProcessEachWithOwnSubOnSameTopic) {
    TopicRegistry reg;
    reg.onNodeInit(H(0x100), "controller_server", "/");
    reg.onNodeInit(H(0x200), "bt_navigator", "/");
    initRclcppSub(reg, 0x100, 0x1000, 0x1100, 0x1200, "/odom");
    initRclcppSub(reg, 0x200, 0x2000, 0x2100, 0x2200, "/odom");

    for (int i = 0; i < 20; ++i) reg.onCallbackStart(H(0x1200), /*intra=*/false);
    for (int i = 0; i < 8; ++i) reg.onCallbackStart(H(0x2200), /*intra=*/true);

    const auto snap = reg.snapshotWindow(2.0);
    const auto* t = findTopic(snap.topics, "/odom");
    ASSERT_NE(t, nullptr);
    EXPECT_EQ(t->recv_inter_count, 20u);  // per-topic view unchanged: the sum
    EXPECT_EQ(t->recv_intra_count, 8u);

    const auto eps = endpointsOf(snap, "/odom", eEndpointSide::kRecv);
    ASSERT_EQ(eps.size(), 2u);
    const auto* a = findByNode(eps, "/controller_server");
    const auto* b = findByNode(eps, "/bt_navigator");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(a->inter_count, 20u);
    EXPECT_EQ(a->intra_count, 0u);
    EXPECT_DOUBLE_EQ(a->inter_hz, 10.0);
    EXPECT_EQ(b->inter_count, 0u);
    EXPECT_EQ(b->intra_count, 8u);
    EXPECT_DOUBLE_EQ(b->intra_hz, 4.0);
    EXPECT_NE(a->id, b->id);
    EXPECT_TRUE(a->endpoint_seen);
    EXPECT_TRUE(b->endpoint_seen);
}

// One node, two subscriptions on the same topic (two costmap layers on /scan): two endpoints of
// the same node, never summed into one rate. The per-topic sum stays what it always was.
TEST(PerEndpoint, OneNodeTwoSubscriptionsOnSameTopicStaySeparate) {
    TopicRegistry reg;
    reg.onNodeInit(H(0x100), "local_costmap", "/local_costmap");
    initRclcppSub(reg, 0x100, 0x1000, 0x1100, 0x1200, "/scan");
    initRclcppSub(reg, 0x100, 0x2000, 0x2100, 0x2200, "/scan");

    for (int i = 0; i < 5; ++i) {
        reg.onCallbackStart(H(0x1200), false);
        reg.onCallbackStart(H(0x2200), false);
    }

    const auto snap = reg.snapshotWindow(1.0);
    const auto eps = endpointsOf(snap, "/scan", eEndpointSide::kRecv);
    ASSERT_EQ(eps.size(), 2u);
    for (const auto& e : eps) {
        EXPECT_EQ(e.node, "/local_costmap/local_costmap");
        EXPECT_EQ(e.inter_count, 5u) << "each subscription counts its own deliveries only";
        EXPECT_DOUBLE_EQ(e.inter_hz, 5.0);
    }
    EXPECT_NE(eps[0].id, eps[1].id);
    const auto* t = findTopic(snap.topics, "/scan");
    ASSERT_NE(t, nullptr);
    EXPECT_EQ(t->recv_inter_count, 10u);
}

// The interleaving hazard of the shared counter (PR #70 review): two subscriptions of one topic
// in one process fed one inter-arrival stream. Per endpoint, each handle measures its OWN
// intervals: a second subscription that stopped receiving shows its stall, while the busy one
// keeps reporting its real period. The merged topic-level gap keeps its old meaning (gap
// between any two deliveries on the topic).
TEST(PerEndpoint, GapIsMeasuredPerSubscriptionNotInterleaved) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    reg.onNodeInit(H(0x100), "fusion", "/");
    initRclcppSub(reg, 0x100, 0x1000, 0x1100, 0x1200, "/imu");
    initRclcppSub(reg, 0x100, 0x2000, 0x2100, 0x2200, "/imu");

    reg.onCallbackStart(H(0x2200), false);  // B delivers once, then stalls
    for (int i = 0; i < 6; ++i) {
        reg.onCallbackStart(H(0x1200), false);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    reg.onCallbackStart(H(0x1200), false);

    const auto snap = reg.snapshotWindow(1.0, /*fold_open_gap=*/true);
    const auto eps = endpointsOf(snap, "/imu", eEndpointSide::kRecv);
    ASSERT_EQ(eps.size(), 2u);
    const sEndpointStat* a = eps[0].inter_count == 7u ? &eps[0] : &eps[1];
    const sEndpointStat* b = a == &eps[0] ? &eps[1] : &eps[0];
    ASSERT_EQ(a->inter_count, 7u);
    ASSERT_EQ(b->inter_count, 1u);
    ASSERT_TRUE(a->has_max_dt);
    ASSERT_TRUE(b->has_max_dt);
    // B's open gap spans the whole run (>= 60 ms); A's own intervals are ~10 ms each.
    EXPECT_GE(b->max_dt_ms, 55.0);
    EXPECT_LT(a->max_dt_ms, b->max_dt_ms);

    const auto* t = findTopic(snap.topics, "/imu");
    ASSERT_NE(t, nullptr);
    ASSERT_TRUE(t->has_recv_max_dt);
    EXPECT_LT(t->recv_max_dt_ms, b->max_dt_ms) << "merged stream keeps its pre-split meaning";
}

// Every rclcpp node has its own /rosout publisher, and rcl creates it INSIDE rcl_node_init, so
// its rcl_publisher_init arrives BEFORE the node's tracepoint. Liveness must still be per node:
// one node logging keeps that node alive, not every node in the process (PR #68 review).
TEST(PerEndpoint, RosoutSharedByManyNodesAttributedPerNode) {
    TopicRegistry reg(/*quiet_windows=*/1);
    const uintptr_t nodes[] = {0x100, 0x200, 0x300};
    const char* names[] = {"planner", "controller", "smoother"};
    for (int i = 0; i < 3; ++i) {
        // rcl order: the node's /rosout publisher first, then rcl_node_init's tracepoint.
        reg.onPublisherInit(H(nodes[i] + 0x10), H(nodes[i]), "/rosout");
        reg.onNodeInit(H(nodes[i]), names[i], "/");
    }
    reg.onPublisherInit(H(0x210), H(0x200), "/cmd_vel");

    for (int i = 0; i < 3; ++i) reg.onPublish(H(0x110));  // /planner logs
    auto snap = reg.snapshotWindow(1.0);
    auto active = reg.activeNodes();
    EXPECT_TRUE(hasNode(active, "/planner"));
    EXPECT_FALSE(hasNode(active, "/controller")) << "another node's log line is not liveness";
    EXPECT_FALSE(hasNode(active, "/smoother"));
    // /rosout is filtered from output, per topic and per endpoint alike.
    EXPECT_EQ(findTopic(snap.topics, "/rosout"), nullptr);
    EXPECT_TRUE(endpointsOf(snap, "/rosout", eEndpointSide::kPub).empty());

    reg.onPublish(H(0x210));  // /controller publishes a real topic
    snap = reg.snapshotWindow(1.0);
    active = reg.activeNodes();
    EXPECT_FALSE(hasNode(active, "/planner"));
    EXPECT_TRUE(hasNode(active, "/controller"));
    const auto eps = endpointsOf(snap, "/cmd_vel", eEndpointSide::kPub);
    ASSERT_EQ(eps.size(), 1u);
    EXPECT_EQ(eps[0].node, "/controller");
}

// rmw_take path (rclpy and other rcl clients): two subscriptions on one topic in one process
// are two endpoints, attributed to their nodes; loaned takes land on the right one too.
TEST(PerEndpoint, TakePathAndLoanedTakeCountPerSubscription) {
    TopicRegistry reg;
    reg.onNodeInit(H(0x100), "py_a", "/");
    reg.onNodeInit(H(0x200), "py_b", "/");
    reg.onSubscriptionInit(H(0x1000), H(0x100), "/points", H(0x1001));
    reg.onSubscriptionInit(H(0x2000), H(0x200), "/points", H(0x2001));
    for (int i = 0; i < 6; ++i) reg.onTake(H(0x1001), true);
    for (int i = 0; i < 3; ++i) reg.onTake(H(0x2001), true);
    reg.onTake(H(0x2001), false);  // a failed take is never a receive
    reg.onLoanedTake(H(0x2000));

    const auto snap = reg.snapshotWindow(1.0);
    const auto eps = endpointsOf(snap, "/points", eEndpointSide::kRecv);
    ASSERT_EQ(eps.size(), 2u);
    const auto* a = findByNode(eps, "/py_a");
    const auto* b = findByNode(eps, "/py_b");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(a->inter_count, 6u);
    EXPECT_EQ(a->loaned_count, 0u);
    EXPECT_EQ(b->inter_count, 3u);
    EXPECT_EQ(b->loaned_count, 1u);
    const auto* t = findTopic(snap.topics, "/points");
    ASSERT_NE(t, nullptr);
    EXPECT_EQ(t->recv_inter_count, 9u);
    EXPECT_EQ(t->recv_loaned_count, 1u);
}

// Publish side: inter, intra (iron+) and loaned publishes land on the publishing handle, and
// two publishers of one topic in one process stay two endpoints.
TEST(PerEndpoint, PublishersCountPerHandle) {
    TopicRegistry reg;
    reg.onNodeInit(H(0x100), "a", "/");
    reg.onNodeInit(H(0x200), "b", "/");
    reg.onPublisherInit(H(0x110), H(0x100), "/tf");
    reg.onPublisherInit(H(0x210), H(0x200), "/tf");
    for (int i = 0; i < 4; ++i) reg.onPublish(H(0x110));
    for (int i = 0; i < 2; ++i) reg.onIntraPublish(H(0x210));
    reg.onLoanedPublish(H(0x210), /*counted_by_tracepoint=*/false);

    const auto snap = reg.snapshotWindow(1.0);
    const auto eps = endpointsOf(snap, "/tf", eEndpointSide::kPub);
    ASSERT_EQ(eps.size(), 2u);
    const auto* a = findByNode(eps, "/a");
    const auto* b = findByNode(eps, "/b");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(a->inter_count, 4u);
    EXPECT_EQ(b->inter_count, 1u);  // the Humble loaned publish is also a publish
    EXPECT_EQ(b->intra_count, 2u);
    EXPECT_EQ(b->loaned_count, 1u);
    const auto* t = findTopic(snap.topics, "/tf");
    ASSERT_NE(t, nullptr);
    EXPECT_EQ(t->pub_inter_count, 5u);
    EXPECT_EQ(t->pub_intra_count, 2u);
    EXPECT_EQ(t->pub_loaned_count, 1u);
}

// No destroy tracepoint exists, so a destroyed publisher's address can come back as a new
// publisher on another topic of another node. The warm thread-local cache entry must follow the
// new identity (the endpoint is re-homed in place), with a fresh id.
TEST(PerEndpoint, RecycledPublisherHandleRehomesEndpoint) {
    TopicRegistry reg;
    reg.onNodeInit(H(0x100), "old_node", "/");
    reg.onNodeInit(H(0x200), "new_node", "/");
    reg.onPublisherInit(H(0x500), H(0x100), "/old");
    for (int i = 0; i < 3; ++i) reg.onPublish(H(0x500));  // warms this thread's cache slot
    auto first = reg.snapshotWindow(1.0);
    const auto old_eps = endpointsOf(first, "/old", eEndpointSide::kPub);
    ASSERT_EQ(old_eps.size(), 1u);

    reg.onPublisherInit(H(0x500), H(0x200), "/new");  // same address, new publisher
    for (int i = 0; i < 2; ++i) reg.onPublish(H(0x500));
    const auto snap = reg.snapshotWindow(1.0);

    const auto* n = findTopic(snap.topics, "/new");
    ASSERT_NE(n, nullptr);
    EXPECT_EQ(n->pub_inter_count, 2u);
    const auto* o = findTopic(snap.topics, "/old");
    if (o != nullptr) {
        EXPECT_EQ(o->pub_inter_count, 0u) << "stale cache served the destroyed publisher's topic";
    }
    const auto eps = endpointsOf(snap, "/new", eEndpointSide::kPub);
    ASSERT_EQ(eps.size(), 1u);
    EXPECT_EQ(eps[0].node, "/new_node");
    EXPECT_NE(eps[0].id, old_eps[0].id);
    EXPECT_TRUE(endpointsOf(snap, "/old", eEndpointSide::kPub).empty());
}

// A node address can be recycled too. The new node's /rosout (or any) publisher init arrives
// before its rcl_node_init and is provisionally given to the destroyed node at that address;
// the node init then claims it.
TEST(PerEndpoint, RecycledNodeHandleIsReattributedOnNodeInit) {
    TopicRegistry reg;
    reg.onNodeInit(H(0x100), "first", "/");
    reg.onPublisherInit(H(0x600), H(0x100), "/status");
    reg.onPublish(H(0x600));
    auto snap = reg.snapshotWindow(1.0);
    ASSERT_EQ(endpointsOf(snap, "/status", eEndpointSide::kPub).at(0).node, "/first");

    reg.onPublisherInit(H(0x700), H(0x100), "/status");  // second node's publisher, before init
    reg.onNodeInit(H(0x100), "second", "/");
    reg.onPublish(H(0x700));
    snap = reg.snapshotWindow(1.0);
    const auto eps = endpointsOf(snap, "/status", eEndpointSide::kPub);
    const sEndpointStat* live = nullptr;
    for (const auto& e : eps) {
        if (e.inter_count == 1u) live = &e;
    }
    ASSERT_NE(live, nullptr);
    EXPECT_EQ(live->node, "/second");
    EXPECT_TRUE(hasNode(reg.activeNodes(), "/second"));
}

// An endpoint whose node was never initialized (no rcl_node_init seen) is still counted, with
// an empty node, and is reported without a node in both formats.
TEST(PerEndpoint, UnknownNodeLeavesNodeEmpty) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), H(0x999), "/orphan");
    reg.onPublisherInit(H(0x20), nullptr, "/orphan");
    reg.onPublish(H(0x10));
    reg.onPublish(H(0x20));
    const auto snap = reg.snapshotWindow(1.0);
    const auto eps = endpointsOf(snap, "/orphan", eEndpointSide::kPub);
    ASSERT_EQ(eps.size(), 2u);
    EXPECT_TRUE(eps[0].node.empty());
    EXPECT_TRUE(eps[1].node.empty());
}

// Endpoint emit gate mirrors the topic gates.
TEST(PerEndpoint, EmitGateMirrorsTopicGates) {
    sEndpointStat e;
    e.side = eEndpointSide::kPub;
    EXPECT_FALSE(TopicRegistry::shouldEmitEndpoint(e, false));  // declared, silent
    EXPECT_TRUE(TopicRegistry::shouldEmitEndpoint(e, true));    // ROS_PULSE_EMIT_IDLE=1
    e.inter_count = 1;
    EXPECT_TRUE(TopicRegistry::shouldEmitEndpoint(e, false));
    e.inter_count = 0;
    e.has_max_dt = true;  // a publisher that stopped, gap tracked: stays visible
    EXPECT_TRUE(TopicRegistry::shouldEmitEndpoint(e, false));

    sEndpointStat r;
    r.side = eEndpointSide::kRecv;
    EXPECT_FALSE(TopicRegistry::shouldEmitEndpoint(r, false));  // never delivered
    r.endpoint_seen = true;  // proven, now stalled: explicit 0.0
    EXPECT_TRUE(TopicRegistry::shouldEmitEndpoint(r, false));
}

namespace {

auto twoEndpoints() -> std::vector<sEndpointStat> {
    sEndpointStat p;
    p.topic = "/scan";
    p.node = "/lidar";
    p.side = eEndpointSide::kPub;
    p.id = 3;
    p.inter_count = 100;
    p.inter_hz = 20.0;
    p.has_max_dt = true;
    p.max_dt_ms = 52.5;
    sEndpointStat r;
    r.topic = "/scan";
    r.side = eEndpointSide::kRecv;  // node unknown
    r.id = 7;
    r.intra_count = 50;
    r.intra_hz = 10.0;
    r.loaned_count = 5;
    r.loaned_hz = 1.0;
    r.endpoint_seen = true;
    sEndpointStat idle;
    idle.topic = "/idle";
    idle.node = "/lidar";
    idle.side = eEndpointSide::kPub;
    idle.id = 9;  // declared, silent: gated out
    return {p, r, idle};
}

}  // namespace

TEST(PerEndpoint, JsonlEndpointsExactBytes) {
    const auto eps = twoEndpoints();
    const std::string got =
        formatWindowJsonl({}, {"/lidar"}, 1782887153899445923LL, 5.0, false, {}, &eps);
    const std::string want =
        "{\"ts_ns\":\"1782887153899445923\",\"window_s\":5.000,\"topics\":[],"
        "\"nodes\":[\"/lidar\"],\"warns\":[],\"endpoints\":["
        "{\"topic\":\"/scan\",\"side\":\"pub\",\"id\":3,\"node\":\"/lidar\","
        "\"inter_hz\":20.000000,\"intra_hz\":0.000000,\"max_dt_ms\":52.500},"
        "{\"topic\":\"/scan\",\"side\":\"recv\",\"id\":7,"
        "\"inter_hz\":0.000000,\"intra_hz\":10.000000,\"loaned_hz\":1.000000}"
        "]}\n";
    EXPECT_EQ(got, want);

    // Passed but empty: the key is present, "measured, nothing to report".
    const std::vector<sEndpointStat> none;
    const std::string empty = formatWindowJsonl({}, {}, 1, 1.0, false, {}, &none);
    EXPECT_NE(empty.find(",\"endpoints\":[]}\n"), std::string::npos) << empty;
    // Not passed: byte-identical to the pre-endpoint record.
    EXPECT_EQ(formatWindowJsonl({}, {}, 1, 1.0, false),
              "{\"ts_ns\":\"1\",\"window_s\":1.000,\"topics\":[],\"nodes\":[],\"warns\":[]}\n");
}

TEST(PerEndpoint, TextEpLinesExactBytes) {
    const auto eps = twoEndpoints();
    const std::string got = formatWindow({}, {"/lidar"}, 42, 5.0, false, {}, &eps);
    const std::string want =
        "# ts_ns=42 window_s=5.000\n"
        "NODE /lidar\n"
        "EP /scan pub id=3 node=/lidar inter=20.000000 intra=0.000000 max_dt_ms=52.500\n"
        "EP /scan recv id=7 inter=0.000000 intra=10.000000 loaned_hz=1.000000\n"
        "\n";
    EXPECT_EQ(got, want);
    // Default (opt-in not set): byte-identical to the pre-endpoint text block.
    EXPECT_EQ(formatWindow({}, {"/lidar"}, 42, 5.0, false),
              "# ts_ns=42 window_s=5.000\nNODE /lidar\n\n");
}

// The reader (pulse-check, pulse_bridge) recovers the split from both formats, and an old-style
// record without it still parses with an empty list.
TEST(PerEndpoint, ReaderRoundTripsBothFormats) {
    const auto eps = twoEndpoints();
    for (const bool jsonl : {true, false}) {
        SCOPED_TRACE(jsonl ? "jsonl" : "text");
        const std::string log = jsonl ? formatWindowJsonl({}, {"/lidar"}, 7, 5.0, false, {}, &eps)
                                      : formatWindow({}, {"/lidar"}, 7, 5.0, false, {}, &eps);
        const auto wins = parseLog(log);
        ASSERT_EQ(wins.size(), 1u);
        const auto& got = wins[0].endpoints;
        ASSERT_EQ(got.size(), 2u);
        EXPECT_EQ(got[0].topic, "/scan");
        EXPECT_EQ(got[0].side, eEndpointSide::kPub);
        EXPECT_EQ(got[0].id, 3u);
        EXPECT_EQ(got[0].node, "/lidar");
        EXPECT_DOUBLE_EQ(got[0].inter_hz, 20.0);
        EXPECT_TRUE(got[0].has_max_dt);
        EXPECT_DOUBLE_EQ(got[0].max_dt_ms, 52.5);
        EXPECT_EQ(got[1].side, eEndpointSide::kRecv);
        EXPECT_EQ(got[1].id, 7u);
        EXPECT_TRUE(got[1].node.empty());
        EXPECT_DOUBLE_EQ(got[1].intra_hz, 10.0);
        EXPECT_DOUBLE_EQ(got[1].loaned_hz, 1.0);
        EXPECT_FALSE(got[1].has_max_dt);
        EXPECT_TRUE(got[1].endpoint_seen);
    }
    const auto old = parseLog(formatWindowJsonl({}, {"/a"}, 1, 1.0, false));
    ASSERT_EQ(old.size(), 1u);
    EXPECT_TRUE(old[0].endpoints.empty());

    // An endpoint with a side this reader does not know is dropped, not fatal to the window.
    const auto future = parseLog(
        "{\"ts_ns\":\"1\",\"window_s\":1.0,\"topics\":[],\"nodes\":[],\"warns\":[],"
        "\"endpoints\":[{\"topic\":\"/x\",\"side\":\"service\",\"id\":1,\"inter_hz\":1.0}]}\n");
    ASSERT_EQ(future.size(), 1u);
    EXPECT_TRUE(future[0].endpoints.empty());
}

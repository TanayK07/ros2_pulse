// Copyright 2026 ros2_pulse contributors
//
// Unit tests for loaned-message counting (rclcpp#3153 follow-up). The probe wraps
// rcl_publish_loaned_message / rcl_take_loaned_message directly, because at the tracepoint
// level a loan is indistinguishable from a copy (Jazzy+ fires the plain rcl_publish
// tracepoint for a loaned publish, Humble traces neither loaned call). Registers into the
// shared test binary (no main()).
//
// Contract pinned here:
//  - pub_inter stays the TOTAL publish count (loaned + copied); pub_loaned is a SUBSET.
//  - On Humble (no tracepoint inside rcl_publish_loaned_message) the wrapper adds the publish
//    to the total itself; on Jazzy+ the tracepoint already did, so it must not be added twice.
//  - A publish loan counts only on RCL_RET_OK; a take loan only on RCL_RET_OK with a non-null
//    message.
//  - Loaned rates are ABSENT from the output when zero (text and jsonl), so every existing
//    log stays byte-identical.

#include <gtest/gtest.h>

#include <string>
#include <thread>
#include <vector>

#include "ros2_pulse/core/loan.hpp"
#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/topic_registry.hpp"
#include "ros2_pulse/core/window_format.hpp"

using ros2_pulse::core::countLoanedPublish;
using ros2_pulse::core::countLoanedTake;
using ros2_pulse::core::eLoanTracepoint;
using ros2_pulse::core::formatWindow;
using ros2_pulse::core::formatWindowJsonl;
using ros2_pulse::core::forwardLoanedPublish;
using ros2_pulse::core::LoanedPublishScope;
using ros2_pulse::core::loanTracepointMode;
using ros2_pulse::core::noteRclPublishTracepoint;
using ros2_pulse::core::parseLog;
using ros2_pulse::core::resetLoanTracepointModeForTest;
using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::TopicRegistry;

namespace {

const void* H(uintptr_t v) { return reinterpret_cast<const void*>(v); }

auto findTopic(const std::vector<sTopicStat>& v, const std::string& t) -> const sTopicStat* {
    for (const auto& s : v) {
        if (s.topic == t) return &s;
    }
    return nullptr;
}

// rcl return codes the wrappers see (rcl/types.h, identical on humble..rolling).
constexpr int kRclRetOk = 0;
constexpr int kRclRetError = 1;
constexpr int kRclRetPublisherInvalid = 300;
constexpr int kRclRetSubscriptionTakeFailed = 401;

}  // namespace

// ---- when does a wrapper call count ----

TEST(LoanedPredicate, PublishCountsOnlyOnOk) {
    EXPECT_TRUE(countLoanedPublish(kRclRetOk));
    EXPECT_FALSE(countLoanedPublish(kRclRetError));
    EXPECT_FALSE(countLoanedPublish(kRclRetPublisherInvalid));
}

TEST(LoanedPredicate, TakeCountsOnlyOnOkWithMessage) {
    int dummy = 0;
    EXPECT_TRUE(countLoanedTake(kRclRetOk, &dummy));
    EXPECT_FALSE(countLoanedTake(kRclRetOk, nullptr));
    EXPECT_FALSE(countLoanedTake(kRclRetSubscriptionTakeFailed, &dummy));
    EXPECT_FALSE(countLoanedTake(kRclRetError, &dummy));
}

// ---- tracepoint latch: did the plain rcl_publish tracepoint fire inside the loaned call ----

TEST(LoanedLatch, HumbleNoTracepointInsideCall) {
    resetLoanTracepointModeForTest();
    LoanedPublishScope scope;
    EXPECT_FALSE(scope.tracepointFired());
}

TEST(LoanedLatch, JazzyTracepointInsideCallIsSeen) {
    resetLoanTracepointModeForTest();
    LoanedPublishScope scope;
    noteRclPublishTracepoint();
    EXPECT_TRUE(scope.tracepointFired());
}

// An ordinary (copied) publish outside any loaned call must not leak into the next scope.
TEST(LoanedLatch, TracepointOutsideScopeDoesNotLeak) {
    resetLoanTracepointModeForTest();
    noteRclPublishTracepoint();
    {
        LoanedPublishScope scope;
        EXPECT_FALSE(scope.tracepointFired());
    }
    noteRclPublishTracepoint();
    LoanedPublishScope scope;
    EXPECT_FALSE(scope.tracepointFired());
}

// Thread-local: another thread's copied publish never marks this thread's loaned call.
TEST(LoanedLatch, OtherThreadsTracepointDoesNotMarkScope) {
    resetLoanTracepointModeForTest();
    LoanedPublishScope scope;
    std::thread t([] {
        LoanedPublishScope other;
        noteRclPublishTracepoint();
        EXPECT_TRUE(other.tracepointFired());
    });
    t.join();
    EXPECT_FALSE(scope.tracepointFired());
}

// ---- learn-once: the mode is a property of the process's librcl ----

// Humble librcl: the first OK call teaches "silent", the total is the wrapper's to count, and
// later calls skip the latch entirely.
TEST(LoanedMode, HumbleLearnsSilentOnFirstOk) {
    resetLoanTracepointModeForTest();
    bool counted = true;
    EXPECT_EQ(forwardLoanedPublish([] { return int32_t{0}; }, counted), 0);
    EXPECT_FALSE(counted);
    EXPECT_EQ(loanTracepointMode(), eLoanTracepoint::kSilent);
    counted = true;
    forwardLoanedPublish([] { return int32_t{0}; }, counted);
    EXPECT_FALSE(counted);
}

// Jazzy+ librcl: the tracepoint fires inside the call; once learned, it is trusted even though
// the interposer no longer marks latches (the steady-state fast path).
TEST(LoanedMode, JazzyLearnsFiresAndStopsLatching) {
    resetLoanTracepointModeForTest();
    bool counted = false;
    forwardLoanedPublish([] { noteRclPublishTracepoint(); return int32_t{0}; }, counted);
    EXPECT_TRUE(counted);
    EXPECT_EQ(loanTracepointMode(), eLoanTracepoint::kFires);
    counted = false;
    forwardLoanedPublish([] { noteRclPublishTracepoint(); return int32_t{0}; }, counted);
    EXPECT_TRUE(counted);
}

// A failed call returns before rcl's tracepoint (invalid publisher, null message), so it must
// not teach "silent" to a librcl that fires.
TEST(LoanedMode, FailedCallTeachesNothing) {
    resetLoanTracepointModeForTest();
    bool counted = true;
    EXPECT_EQ(forwardLoanedPublish([] { return int32_t{300}; }, counted), 300);
    EXPECT_EQ(loanTracepointMode(), eLoanTracepoint::kUnknown);
    forwardLoanedPublish([] { noteRclPublishTracepoint(); return int32_t{0}; }, counted);
    EXPECT_EQ(loanTracepointMode(), eLoanTracepoint::kFires);
}

// Race: another thread learns the mode while this call is in flight. The first learner wins
// (CAS from unknown) and this call defers to it, so the cached mode never flaps.
TEST(LoanedMode, InFlightCallDefersToConcurrentLearner) {
    resetLoanTracepointModeForTest();
    bool counted = true;
    forwardLoanedPublish(
        [] {
            std::thread([] {
                bool other = true;
                forwardLoanedPublish([] { return int32_t{0}; }, other);  // learns "silent"
                EXPECT_FALSE(other);
            }).join();
            return int32_t{0};
        },
        counted);
    EXPECT_FALSE(counted);
    EXPECT_EQ(loanTracepointMode(), eLoanTracepoint::kSilent);
}

// Steady state: a process that never loans, or one that already learned, never opens a latch,
// so ordinary publishes never reach the thread-local check.
TEST(LoanedMode, NoLatchOutsideLearning) {
    resetLoanTracepointModeForTest();
    bool counted = false;
    forwardLoanedPublish([] { return int32_t{0}; }, counted);  // learns
    EXPECT_EQ(ros2_pulse::core::detail::g_learning.load(), 0);
    forwardLoanedPublish([] { return int32_t{0}; }, counted);
    EXPECT_EQ(ros2_pulse::core::detail::g_learning.load(), 0);
}

// ---- registry ----

// Humble shape: the tracepoint does not fire inside rcl_publish_loaned_message, so the wrapper
// adds each loan to the total. Mixed with ordinary publishes, the total is loaned + copied.
TEST(LoanedRegistry, HumbleLoanedPublishAddsToTotalAndSubset) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/cloud");
    for (int i = 0; i < 30; ++i) reg.onLoanedPublish(H(0x10), /*counted_by_tracepoint=*/false);
    for (int i = 0; i < 20; ++i) reg.onPublish(H(0x10));
    auto snap = reg.snapshot(2.0);
    const auto* s = findTopic(snap, "/cloud");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_inter_count, 50u);
    EXPECT_EQ(s->pub_loaned_count, 30u);
    EXPECT_DOUBLE_EQ(s->pub_loaned_hz, 15.0);
    EXPECT_DOUBLE_EQ(s->pub_inter_hz, 25.0);
    EXPECT_EQ(s->recv_loaned_count, 0u);
}

// Jazzy+ shape: the plain tracepoint fired first and already counted the publish. The wrapper
// must count the loan and nothing else, no double count of the total.
TEST(LoanedRegistry, JazzyLoanedPublishDoesNotDoubleCountTotal) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/cloud");
    for (int i = 0; i < 30; ++i) {
        reg.onPublish(H(0x10));  // ros_trace_rcl_publish inside rcl_publish_loaned_message
        reg.onLoanedPublish(H(0x10), /*counted_by_tracepoint=*/true);
    }
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/cloud");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_inter_count, 30u);
    EXPECT_EQ(s->pub_loaned_count, 30u);
}

// Take side keys on the rcl subscription handle (the one rcl_subscription_init carries), the
// executor passes exactly that handle to rcl_take_loaned_message. The receive TOTAL stays the
// callback_start count, so a loaned take alone never moves recv_inter.
TEST(LoanedRegistry, LoanedTakeCountsSubsetByRclHandle) {
    TopicRegistry reg;
    reg.onSubscriptionInit(H(0x20), nullptr, "/cloud");
    for (int i = 0; i < 25; ++i) reg.onLoanedTake(H(0x20));
    auto snap = reg.snapshot(5.0);
    const auto* s = findTopic(snap, "/cloud");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_loaned_count, 25u);
    EXPECT_DOUBLE_EQ(s->recv_loaned_hz, 5.0);
    EXPECT_EQ(s->recv_inter_count, 0u);
    EXPECT_EQ(s->pub_loaned_count, 0u);
}

// Handles the init hooks never saw (probe attached late, or a non-ROS caller) and null
// handles are dropped, never invent a topic.
TEST(LoanedRegistry, UnknownAndNullHandlesIgnored) {
    TopicRegistry reg;
    reg.onLoanedPublish(H(0x99), false);
    reg.onLoanedTake(H(0x98));
    reg.onLoanedPublish(nullptr, false);
    reg.onLoanedTake(nullptr);
    EXPECT_TRUE(reg.snapshot(1.0).empty());
}

TEST(LoanedRegistry, SnapshotResetsLoanedCounts) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/cloud");
    reg.onSubscriptionInit(H(0x20), nullptr, "/cloud");
    reg.onLoanedPublish(H(0x10), false);
    reg.onLoanedTake(H(0x20));
    (void)reg.snapshot(1.0);
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/cloud");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_loaned_count, 0u);
    EXPECT_EQ(s->recv_loaned_count, 0u);
    EXPECT_EQ(s->pub_inter_count, 0u);
}

// A node whose only traffic is loaned is alive (on Humble that is the WHOLE publish signal).
TEST(LoanedRegistry, LoanedOnlyTrafficKeepsNodeAlive) {
    TopicRegistry reg(/*quiet_windows=*/1);
    reg.onNodeInit(H(0x1), "talker", "/");
    reg.onNodeInit(H(0x2), "listener", "/");
    reg.onPublisherInit(H(0x10), H(0x1), "/cloud");
    reg.onSubscriptionInit(H(0x20), H(0x2), "/cloud");
    reg.onLoanedPublish(H(0x10), false);
    reg.onLoanedTake(H(0x20));
    (void)reg.snapshot(1.0);
    auto nodes = reg.activeNodes();
    EXPECT_EQ(nodes.size(), 2u);
}

TEST(LoanedRegistry, FilteredTopicNotReported) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/rosout");
    reg.onLoanedPublish(H(0x10), false);
    EXPECT_TRUE(reg.snapshot(1.0).empty());
}

// With gap tracking on, a Humble loaned publish IS a publish arrival (it is the only signal
// for it), so it feeds the pub gap accumulator; a Jazzy loan already did via the tracepoint.
TEST(LoanedRegistry, HumbleLoanedPublishFeedsGapTracking) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    reg.onPublisherInit(H(0x10), nullptr, "/cloud");
    reg.onLoanedPublish(H(0x10), false);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    reg.onLoanedPublish(H(0x10), false);
    auto snap = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = findTopic(snap, "/cloud");
    ASSERT_NE(s, nullptr);
    EXPECT_TRUE(s->has_pub_max_dt);
    EXPECT_GE(s->pub_max_dt_ms, 1.0);
}

// Concurrent loaned publishers on one topic: relaxed atomics, totals exact.
TEST(LoanedRegistry, ConcurrentLoanedPublishExact) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/cloud");
    reg.onPublisherInit(H(0x11), nullptr, "/cloud");
    std::thread a([&] { for (int i = 0; i < 10000; ++i) reg.onLoanedPublish(H(0x10), false); });
    std::thread b([&] { for (int i = 0; i < 10000; ++i) reg.onLoanedPublish(H(0x11), false); });
    a.join();
    b.join();
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/cloud");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_loaned_count, 20000u);
    EXPECT_EQ(s->pub_inter_count, 20000u);
}

// ---- output formats ----

namespace {

auto loanedStat() -> sTopicStat {
    sTopicStat s;
    s.topic = "/cloud";
    s.pub_inter_count = 100;
    s.pub_inter_hz = 50.0;
    s.pub_loaned_count = 100;
    s.pub_loaned_hz = 50.0;
    s.recv_inter_count = 98;
    s.recv_inter_hz = 49.0;
    s.recv_endpoint_seen = true;
    s.recv_loaned_count = 98;
    s.recv_loaned_hz = 49.0;
    return s;
}

}  // namespace

// Additive line kind, one per side, after that side's lines. Never appended to TOPIC/PUB/RECV:
// in-tree parsers anchor those with '$'.
TEST(LoanedFormat, TextLoanLinesExactBytes) {
    const std::string out = formatWindow({loanedStat()}, {"/talker"}, 7LL, 2.0, false);
    EXPECT_EQ(out,
              "# ts_ns=7 window_s=2.000\n"
              "TOPIC /cloud 50.000000\n"
              "LOAN /cloud pub hz=50.000000\n"
              "RECV /cloud inter=49.000000 intra=0.000000\n"
              "LOAN /cloud recv hz=49.000000\n"
              "NODE /talker\n"
              "\n");
}

// Absent when zero: a window without loans is byte-identical to the pre-loan format.
TEST(LoanedFormat, TextNoLoanLinesWhenZero) {
    sTopicStat s = loanedStat();
    s.pub_loaned_count = 0;
    s.pub_loaned_hz = 0.0;
    s.recv_loaned_count = 0;
    s.recv_loaned_hz = 0.0;
    const std::string out = formatWindow({s}, {}, 7LL, 2.0, false);
    EXPECT_EQ(out.find("LOAN"), std::string::npos) << out;
}

TEST(LoanedFormat, JsonlKeysExactBytes) {
    const std::string out = formatWindowJsonl({loanedStat()}, {}, 7LL, 2.0, false);
    EXPECT_EQ(out,
              "{\"ts_ns\":\"7\",\"window_s\":2.000,\"topics\":[{\"topic\":\"/cloud\","
              "\"pub_inter_hz\":50.000000,\"pub_intra_hz\":0.000000,"
              "\"recv_inter_hz\":49.000000,\"recv_intra_hz\":0.000000,"
              "\"recv_endpoint_seen\":true,"
              "\"pub_loaned_hz\":50.000000,\"recv_loaned_hz\":49.000000}],"
              "\"nodes\":[],\"warns\":[]}\n");
}

TEST(LoanedFormat, JsonlKeysAbsentWhenZero) {
    sTopicStat s = loanedStat();
    s.recv_loaned_count = 0;
    s.recv_loaned_hz = 0.0;
    const std::string out = formatWindowJsonl({s}, {}, 7LL, 2.0, false);
    EXPECT_NE(out.find("\"pub_loaned_hz\":50.000000"), std::string::npos) << out;
    EXPECT_EQ(out.find("recv_loaned_hz"), std::string::npos) << out;
}

// The offline reader (pulse-check, pulse_bridge) inverts both encodings of the loan fields.
TEST(LoanedFormat, LogReaderRoundTripsBothFormats) {
    for (const std::string& text :
         {formatWindow({loanedStat()}, {}, 7LL, 2.0, false),
          formatWindowJsonl({loanedStat()}, {}, 7LL, 2.0, false)}) {
        auto windows = parseLog(text);
        ASSERT_EQ(windows.size(), 1u) << text;
        const auto* p = findTopic(windows[0].stats, "/cloud");
        ASSERT_NE(p, nullptr) << text;
        EXPECT_DOUBLE_EQ(p->pub_loaned_hz, 50.0) << text;
        EXPECT_DOUBLE_EQ(p->recv_loaned_hz, 49.0) << text;
        EXPECT_DOUBLE_EQ(p->pub_inter_hz, 50.0) << text;
    }
}

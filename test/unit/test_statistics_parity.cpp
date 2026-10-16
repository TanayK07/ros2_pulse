// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the /statistics parity fields: what rclcpp's built-in topic statistics
// published (removed in rolling by rclcpp#3203) beyond the rate and the largest gap, i.e. the
// MINIMUM and STDDEV of the received message period, and message_age. Covers the registry
// accumulators, the jsonl keys that carry them, the reader, and the bridge mapper. Pure, no
// ROS. Registers into the shared test binary.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/metrics_mapper.hpp"
#include "ros2_pulse/core/topic_registry.hpp"
#include "ros2_pulse/core/window_format.hpp"

using ros2_pulse::core::eMetricsSource;
using ros2_pulse::core::eRateUnit;
using ros2_pulse::core::formatWindowJsonl;
using ros2_pulse::core::mapWindow;
using ros2_pulse::core::parseLog;
using ros2_pulse::core::sLogWindow;
using ros2_pulse::core::sMapperOptions;
using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::TopicRegistry;

namespace {

const void* H(uintptr_t v) { return reinterpret_cast<const void*>(v); }

auto statOf(const std::vector<sTopicStat>& v, const std::string& t) -> const sTopicStat* {
    for (const auto& s : v) {
        if (s.topic == t) return &s;
    }
    return nullptr;
}

auto wallNowNs() -> int64_t {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// rclpy-style: counted at the take. rclcpp-style: counted at callback_start, take is age only.
void initRclpySub(TopicRegistry& reg, uintptr_t rcl, uintptr_t rmw, const char* topic) {
    reg.onSubscriptionInit(H(rcl), nullptr, topic, H(rmw));
}
void initRclcppSub(TopicRegistry& reg, uintptr_t rcl, uintptr_t rmw, uintptr_t sub,
                   uintptr_t cb, const char* topic) {
    reg.onSubscriptionInit(H(rcl), nullptr, topic, H(rmw));
    reg.onRclcppSubscriptionInit(H(sub), H(rcl));
    reg.onCallbackAdded(H(cb), H(sub));
}

}  // namespace

// Off by default: no distribution, no age, even with source timestamps on every take.
TEST(StatisticsParity, NothingWithoutGapTracking) {
    TopicRegistry reg;
    initRclpySub(reg, 0x40, 0x41, "/scan");
    for (int i = 0; i < 5; ++i) reg.onTake(H(0x41), true, wallNowNs() - 1000000);
    const auto snap = reg.snapshot(1.0);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 5u);
    EXPECT_FALSE(s->has_recv_dt);
    EXPECT_FALSE(s->has_recv_age);
}

// Intervals of ~20 ms and ~60 ms: minimum is the short one, population stddev is half the
// spread (~20 ms). sleep_for only guarantees a lower bound, so the upper bounds are generous.
TEST(StatisticsParity, ReceiveIntervalMinimumAndStddev) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    reg.onTake(H(0x41), true);
    sleepMs(20);
    reg.onTake(H(0x41), true);
    sleepMs(60);
    reg.onTake(H(0x41), true);

    const auto snap = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_dt);
    EXPECT_EQ(s->recv_dt.count, 2u);
    EXPECT_GE(s->recv_dt.min_ms, 20.0);
    EXPECT_LT(s->recv_dt.min_ms, 55.0);
    ASSERT_TRUE(s->has_recv_max_dt);
    EXPECT_GE(s->recv_max_dt_ms, 60.0);
    EXPECT_LE(s->recv_dt.min_ms, s->recv_max_dt_ms);
    // stddev of two samples is |a - b| / 2; a and b both carry scheduling slack.
    EXPECT_GT(s->recv_dt.stddev_ms, 5.0);
    EXPECT_LT(s->recv_dt.stddev_ms, 60.0);
}

// The rclcpp callback path feeds the same distribution (it is where rclcpp receives count).
TEST(StatisticsParity, CallbackPathFeedsTheDistribution) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclcppSub(reg, 0x50, 0x51, 0x52, 0x53, "/odom");
    reg.onCallbackStart(H(0x53), false);
    sleepMs(10);
    reg.onCallbackStart(H(0x53), false);
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/odom");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_dt);
    EXPECT_EQ(s->recv_dt.count, 1u);
    EXPECT_DOUBLE_EQ(s->recv_dt.stddev_ms, 0.0);  // one interval has no spread
    EXPECT_GE(s->recv_dt.min_ms, 10.0);
}

// Two subscriptions on one topic (a component container, or one node subscribing twice) each
// receive every message at period T. Intervals are per subscription, never across the merged
// stream: interleaved, those would alternate ~0 and ~T, MINIMUM ~0 and STDDEV ~T/2 (PR #70
// review). Pooled per topic, MINIMUM stays ~T and the deviation is scheduling slack only.
TEST(StatisticsParity, TwoSubscriptionsOneTopicKeepTheirOwnIntervals) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    initRclcppSub(reg, 0x50, 0x51, 0x52, 0x53, "/scan");
    constexpr int kPeriodMs = 20;
    constexpr int kMessages = 6;
    for (int i = 0; i < kMessages; ++i) {
        if (i > 0) sleepMs(kPeriodMs);
        reg.onTake(H(0x41), true);
        reg.onCallbackStart(H(0x53), false);
    }
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 2u * kMessages);
    ASSERT_TRUE(s->has_recv_dt);
    EXPECT_EQ(s->recv_dt.count, 2u * (kMessages - 1)) << "intervals of both subscriptions";
    EXPECT_GE(s->recv_dt.min_ms, kPeriodMs);
    EXPECT_GE(s->recv_dt.mean_ms, kPeriodMs);
    EXPECT_LT(s->recv_dt.stddev_ms, kPeriodMs / 4.0) << "no ~0 cross-subscription intervals";
    // The merged-stream JITTER gap is unchanged: still the gap between ANY two messages.
    ASSERT_TRUE(s->has_recv_max_dt);
    EXPECT_GE(s->recv_max_dt_ms, kPeriodMs);
}

// One arrival is a timestamp, not an interval: no distribution, like no max_dt.
TEST(StatisticsParity, SingleArrivalHasNoDistribution) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    reg.onTake(H(0x41), true);
    const auto snap = reg.snapshot(1.0, false);
    EXPECT_FALSE(statOf(snap, "/scan")->has_recv_dt);
}

// Per window: the next window without traffic reports no stale distribution or age.
TEST(StatisticsParity, DistributionsResetPerWindow) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    reg.onTake(H(0x41), true, wallNowNs() - 2000000);
    reg.onTake(H(0x41), true, wallNowNs() - 2000000);
    auto first = reg.snapshot(1.0, false);
    ASSERT_TRUE(statOf(first, "/scan")->has_recv_dt);
    ASSERT_TRUE(statOf(first, "/scan")->has_recv_age);
    auto second = reg.snapshot(1.0, false);
    const auto* s = statOf(second, "/scan");
    ASSERT_NE(s, nullptr);  // proven endpoint, still reported at 0 Hz
    EXPECT_FALSE(s->has_recv_dt);
    EXPECT_FALSE(s->has_recv_age);
}

// Age = local wall clock at the take minus the DDS source timestamp.
TEST(StatisticsParity, MessageAgeFromSourceTimestamp) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    reg.onTake(H(0x41), true, wallNowNs() - 10000000);  // published 10 ms ago
    reg.onTake(H(0x41), true, wallNowNs() - 30000000);  // published 30 ms ago

    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_age);
    EXPECT_EQ(s->recv_age.count, 2u);
    EXPECT_GE(s->recv_age.min_ms, 10.0);
    EXPECT_LT(s->recv_age.min_ms, 25.0);
    EXPECT_GE(s->recv_age.max_ms, 30.0);
    EXPECT_LT(s->recv_age.max_ms, 45.0);
    EXPECT_GE(s->recv_age.mean_ms, 20.0);
    EXPECT_LT(s->recv_age.mean_ms, 35.0);
    EXPECT_NEAR(s->recv_age.stddev_ms, 10.0, 3.0);  // |30 - 10| / 2, same take-time slack
}

// rclcpp subscriptions: the take must not count the receive (callback_start does), but it is
// the only place the source timestamp exists, so it still feeds the age.
TEST(StatisticsParity, RclcppTakeFeedsAgeButNotCount) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclcppSub(reg, 0x50, 0x51, 0x52, 0x53, "/odom");
    for (int i = 0; i < 3; ++i) {
        reg.onTake(H(0x51), true, wallNowNs() - 5000000);
        reg.onCallbackStart(H(0x53), false);
    }
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/odom");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 3u) << "take and callback both counted";
    ASSERT_TRUE(s->has_recv_age);
    EXPECT_EQ(s->recv_age.count, 3u);
    EXPECT_GE(s->recv_age.min_ms, 5.0);
}

// Gap tracking off keeps the rclcpp take path a cache hit and a return: no age, no count.
TEST(StatisticsParity, RclcppTakeWithoutGapTrackingIsANoOp) {
    TopicRegistry reg;
    initRclcppSub(reg, 0x50, 0x51, 0x52, 0x53, "/odom");
    reg.onTake(H(0x51), true, wallNowNs() - 5000000);
    reg.onTake(H(0x51), true, wallNowNs() - 5000000);
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/odom");
    if (s != nullptr) {
        EXPECT_EQ(s->recv_inter_count, 0u);
        EXPECT_FALSE(s->has_recv_age);
    }
}

// No source timestamp (0) is not an age sample.
TEST(StatisticsParity, MissingSourceTimestampIgnored) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    reg.onTake(H(0x41), true, 0);
    reg.onTake(H(0x41), true, 0);
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 2u);
    EXPECT_FALSE(s->has_recv_age);
}

// A source timestamp ahead of the local clock (the publisher's host runs fast) is a NEGATIVE
// age, kept like rclcpp Jazzy+ keeps it: min and the mean go negative, nothing wraps. Dropping
// it would bias the mean up and hide the skew.
TEST(StatisticsParity, FutureSourceTimestampIsANegativeAge) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    reg.onTake(H(0x41), true, wallNowNs() + 60LL * 1000000000LL);  // 60 s ahead
    reg.onTake(H(0x41), true, wallNowNs() - 10000000LL);            // 10 ms ago
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_age);
    EXPECT_EQ(s->recv_age.count, 2u);
    EXPECT_LE(s->recv_age.min_ms, -59990.0);
    EXPECT_GT(s->recv_age.min_ms, -60000.1);
    EXPECT_GE(s->recv_age.max_ms, 10.0);
    EXPECT_LT(s->recv_age.max_ms, 30.0);
    EXPECT_NEAR(s->recv_age.mean_ms, (-60000.0 + 10.0) / 2.0, 10.0);
    ASSERT_TRUE(s->recv_age.has_stddev);
    EXPECT_NEAR(s->recv_age.stddev_ms, 30005.0, 10.0);

    // Signed through jsonl and the reader.
    const std::string line = formatWindowJsonl(snap, {}, 42LL, 1.0, false);
    EXPECT_NE(line.find("\"recv_age_min_ms\":-"), std::string::npos) << line;
    const auto wins = parseLog(line);
    ASSERT_EQ(wins.size(), 1u);
    ASSERT_EQ(wins[0].stats.size(), 1u);
    EXPECT_LT(wins[0].stats[0].recv_age.min_ms, -59990.0);
}

// A constant clock skew between hosts (10 min here) lands in the age, and must not cost the
// deviation: unshifted, 100 squares of 6e8 us wrap a uint64 (PR #70 review). Ages alternate
// skew + 0 and skew + 10 ms, so the population stddev is 5 ms plus take-time slack.
TEST(StatisticsParity, AgeStddevSurvivesLargeConstantSkew) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    constexpr int64_t kSkewNs = 600LL * 1000000000LL;
    for (int i = 0; i < 100; ++i) {
        reg.onTake(H(0x41), true, wallNowNs() - kSkewNs - (i % 2) * 10000000LL);
    }
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_age);
    EXPECT_EQ(s->recv_age.count, 100u);
    ASSERT_TRUE(s->recv_age.has_stddev);
    EXPECT_NEAR(s->recv_age.stddev_ms, 5.0, 0.5);
    EXPECT_GE(s->recv_age.mean_ms, 600000.0);
    EXPECT_LT(s->recv_age.mean_ms, 600050.0);
}

// 1 h of skew: past the old 2^31 us per-sample clamp, which pinned the mean BELOW the minimum.
TEST(StatisticsParity, AgeMeanNotClampedUnderHourOfSkew) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    constexpr int64_t kSkewNs = 3600LL * 1000000000LL;
    for (int i = 0; i < 10; ++i) reg.onTake(H(0x41), true, wallNowNs() - kSkewNs);
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_age);
    EXPECT_GE(s->recv_age.mean_ms, s->recv_age.min_ms);
    EXPECT_LE(s->recv_age.mean_ms, s->recv_age.max_ms);
    EXPECT_GE(s->recv_age.min_ms, 3600000.0);
    ASSERT_TRUE(s->recv_age.has_stddev);
    EXPECT_LT(s->recv_age.stddev_ms, 1.0);
}

// Samples hours apart in one window overflow the squares even shifted: the deviation is then
// unknown and dropped (no key, no STDDEV point), never reported as a wrapped or zero value.
// The mean, min and max stay exact.
TEST(StatisticsParity, AgeSumOfSquaresOverflowDropsStddev) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    reg.onTake(H(0x41), true, wallNowNs() - 1000000LL);                  // 1 ms
    reg.onTake(H(0x41), true, wallNowNs() - 2LL * 3600LL * 1000000000LL);  // 2 h
    auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_age);
    EXPECT_EQ(s->recv_age.count, 2u);
    EXPECT_FALSE(s->recv_age.has_stddev);
    EXPECT_NEAR(s->recv_age.mean_ms, 3600000.5, 1.0);
    EXPECT_GE(s->recv_age.max_ms, 7200000.0);

    const std::string line = formatWindowJsonl(snap, {}, 42LL, 1.0, false);
    EXPECT_NE(line.find("\"recv_age_avg_ms\""), std::string::npos) << line;
    EXPECT_EQ(line.find("recv_age_stddev_ms"), std::string::npos) << line;
    const auto wins = parseLog(line);
    ASSERT_EQ(wins.size(), 1u);
    sMapperOptions o;
    o.metrics_source = eMetricsSource::Metric;
    bool saw_age = false;
    for (const auto& m : mapWindow(wins[0], o)) {
        if (m.metrics_source != "message_age") continue;
        saw_age = true;
        EXPECT_FALSE(m.has_stddev);
        EXPECT_DOUBLE_EQ(m.sample_count, 2.0);
    }
    EXPECT_TRUE(saw_age);

    // The flag is per window: the next window's samples get their deviation back.
    reg.onTake(H(0x41), true, wallNowNs() - 1000000LL);
    reg.onTake(H(0x41), true, wallNowNs() - 1000000LL);
    snap = reg.snapshot(1.0, false);
    ASSERT_TRUE(statOf(snap, "/scan")->has_recv_age);
    EXPECT_TRUE(statOf(snap, "/scan")->recv_age.has_stddev);
}

// Two subscriptions with their own shifts (different skews) pool exactly: ages ~10 min on one,
// ~10 min + 20 ms on the other, so the topic's stddev is ~10 ms and its mean sits between.
TEST(StatisticsParity, AgePoolsAcrossSubscriptionsWithDifferentShifts) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x40, 0x41, "/scan");
    initRclpySub(reg, 0x60, 0x61, "/scan");
    constexpr int64_t kSkewNs = 600LL * 1000000000LL;
    for (int i = 0; i < 20; ++i) {
        reg.onTake(H(0x41), true, wallNowNs() - kSkewNs);
        reg.onTake(H(0x61), true, wallNowNs() - kSkewNs - 20000000LL);
    }
    const auto snap = reg.snapshot(1.0, false);
    const auto* s = statOf(snap, "/scan");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_age);
    EXPECT_EQ(s->recv_age.count, 40u);
    ASSERT_TRUE(s->recv_age.has_stddev);
    EXPECT_NEAR(s->recv_age.stddev_ms, 10.0, 0.5);
    EXPECT_NEAR(s->recv_age.mean_ms, 600010.0, 1.0);
    EXPECT_GE(s->recv_age.min_ms, 600000.0);
    EXPECT_GE(s->recv_age.max_ms, 600020.0);
}

// jsonl: keys only when measured, %.3f like every other ms field, and the reader inverts them.
TEST(StatisticsParity, JsonlKeysExactBytesAndRoundTrip) {
    sTopicStat s;
    s.topic = "/scan";
    s.recv_inter_hz = 20.0;
    s.recv_inter_count = 100;
    s.recv_endpoint_seen = true;
    s.recv_max_dt_ms = 21.284;
    s.has_recv_max_dt = true;
    s.has_recv_dt = true;
    s.recv_dt.min_ms = 48.7514;
    s.recv_dt.stddev_ms = 0.8123;
    s.has_recv_age = true;
    s.recv_age.mean_ms = 1.25;
    s.recv_age.min_ms = 0.8;
    s.recv_age.max_ms = 3.1;
    s.recv_age.stddev_ms = 0.4;
    s.recv_age.count = 100;

    const std::string got = formatWindowJsonl({s}, {"/perception"}, 42LL, 5.0, false);
    const std::string want =
        "{\"ts_ns\":\"42\",\"window_s\":5.000,\"topics\":[{\"topic\":\"/scan\","
        "\"recv_inter_hz\":20.000000,\"recv_intra_hz\":0.000000,\"recv_endpoint_seen\":true,"
        "\"recv_max_dt_ms\":21.284,\"recv_min_dt_ms\":48.751,\"recv_stddev_dt_ms\":0.812,"
        "\"recv_age_avg_ms\":1.250,\"recv_age_min_ms\":0.800,\"recv_age_max_ms\":3.100,"
        "\"recv_age_stddev_ms\":0.400,\"recv_age_count\":100}],"
        "\"nodes\":[\"/perception\"],\"warns\":[]}\n";
    EXPECT_EQ(got, want);

    const auto wins = parseLog(got);
    ASSERT_EQ(wins.size(), 1u);
    ASSERT_EQ(wins[0].stats.size(), 1u);
    const auto& r = wins[0].stats[0];
    EXPECT_TRUE(r.has_recv_dt);
    EXPECT_DOUBLE_EQ(r.recv_dt.min_ms, 48.751);
    EXPECT_DOUBLE_EQ(r.recv_dt.stddev_ms, 0.812);
    EXPECT_TRUE(r.has_recv_age);
    EXPECT_DOUBLE_EQ(r.recv_age.mean_ms, 1.25);
    EXPECT_DOUBLE_EQ(r.recv_age.min_ms, 0.8);
    EXPECT_DOUBLE_EQ(r.recv_age.max_ms, 3.1);
    EXPECT_DOUBLE_EQ(r.recv_age.stddev_ms, 0.4);
    EXPECT_EQ(r.recv_age.count, 100u);
}

TEST(StatisticsParity, JsonlUnchangedWhenNotMeasured) {
    sTopicStat s;
    s.topic = "/scan";
    s.recv_inter_hz = 20.0;
    s.recv_inter_count = 100;
    s.recv_endpoint_seen = true;
    const std::string got = formatWindowJsonl({s}, {}, 42LL, 5.0, false);
    EXPECT_EQ(got.find("min_dt"), std::string::npos) << got;
    EXPECT_EQ(got.find("age"), std::string::npos) << got;
}

namespace {

auto parityWindow() -> sLogWindow {
    sLogWindow w;
    w.ts_ns = 1782887153899445923LL;
    w.window_s = 5.0;
    sTopicStat s;
    s.topic = "/scan";
    s.recv_inter_hz = 20.0;
    s.recv_endpoint_seen = true;
    s.has_recv_max_dt = true;
    s.recv_max_dt_ms = 52.0;
    s.has_recv_dt = true;
    s.recv_dt.min_ms = 48.0;
    s.recv_dt.stddev_ms = 1.5;
    s.has_recv_age = true;
    s.recv_age.mean_ms = 2.0;
    s.recv_age.min_ms = 1.0;
    s.recv_age.max_ms = 4.0;
    s.recv_age.stddev_ms = 0.5;
    s.recv_age.count = 99;
    w.stats.push_back(s);
    w.nodes = {"/perception"};
    return w;
}

}  // namespace

// Default topic layout: one message_period sample per topic, now with MINIMUM and STDDEV.
TEST(StatisticsParity, MapperTopicLayoutAddsMinimumAndStddev) {
    const auto v = mapWindow(parityWindow(), sMapperOptions{});
    ASSERT_EQ(v.size(), 1u) << "message_age has no slot in the topic layout";
    const auto& m = v[0];
    EXPECT_EQ(m.metrics_source, "/scan");
    EXPECT_DOUBLE_EQ(m.average, 50.0);
    ASSERT_TRUE(m.has_minimum);
    EXPECT_DOUBLE_EQ(m.minimum, 48.0);
    ASSERT_TRUE(m.has_maximum);
    EXPECT_DOUBLE_EQ(m.maximum, 52.0);
    ASSERT_TRUE(m.has_stddev);
    EXPECT_DOUBLE_EQ(m.stddev, 1.5);
}

// rclcpp's own layout: metric name in metrics_source, node in measurement_source_name, and a
// message_age message before message_period, as rclcpp's subscription published them.
TEST(StatisticsParity, MapperMetricLayoutMatchesRclcpp) {
    sMapperOptions o;
    o.metrics_source = eMetricsSource::Metric;
    const auto v = mapWindow(parityWindow(), o);
    ASSERT_EQ(v.size(), 2u);
    EXPECT_EQ(v[1].metrics_source, "message_period");
    EXPECT_EQ(v[1].measurement_source_name, "/perception");
    EXPECT_TRUE(v[1].has_minimum);
    EXPECT_TRUE(v[1].has_stddev);
    const auto& a = v[0];
    EXPECT_EQ(a.metrics_source, "message_age");
    EXPECT_EQ(a.measurement_source_name, "/perception");
    EXPECT_EQ(a.unit, "ms");
    EXPECT_EQ(a.window_start_ns, v[1].window_start_ns);
    EXPECT_EQ(a.window_stop_ns, v[1].window_stop_ns);
    EXPECT_DOUBLE_EQ(a.average, 2.0);
    EXPECT_DOUBLE_EQ(a.minimum, 1.0);
    EXPECT_DOUBLE_EQ(a.maximum, 4.0);
    EXPECT_DOUBLE_EQ(a.stddev, 0.5);
    EXPECT_DOUBLE_EQ(a.sample_count, 99.0);
}

// No age measured (gap tracking off, intra-process only): message_age is still published, like
// upstream's always-present collector, as an empty window (no points measured, count 0).
TEST(StatisticsParity, MapperMetricLayoutWithoutAge) {
    auto w = parityWindow();
    w.stats[0].has_recv_age = false;
    sMapperOptions o;
    o.metrics_source = eMetricsSource::Metric;
    const auto v = mapWindow(w, o);
    ASSERT_EQ(v.size(), 2u);
    EXPECT_EQ(v[0].metrics_source, "message_age");
    EXPECT_FALSE(v[0].has_average);
    EXPECT_FALSE(v[0].has_minimum);
    EXPECT_FALSE(v[0].has_maximum);
    EXPECT_FALSE(v[0].has_stddev);
    EXPECT_DOUBLE_EQ(v[0].sample_count, 0.0);
    EXPECT_EQ(v[1].metrics_source, "message_period");
}

// Same side rule as MAXIMUM: a publisher-only process carries no distribution, and Hz mode
// never mixes in ms values.
TEST(StatisticsParity, MapperNoMinimumOnPubSideOrInHz) {
    auto w = parityWindow();
    w.stats[0].recv_inter_hz = 0.0;
    w.stats[0].recv_endpoint_seen = false;
    w.stats[0].pub_inter_hz = 20.0;
    auto m = mapWindow(w, sMapperOptions{});
    ASSERT_EQ(m.size(), 1u);
    EXPECT_FALSE(m[0].has_minimum);
    EXPECT_FALSE(m[0].has_stddev);

    sMapperOptions hz;
    hz.unit = eRateUnit::Rate_hz;
    m = mapWindow(parityWindow(), hz);
    ASSERT_EQ(m.size(), 1u);
    EXPECT_FALSE(m[0].has_minimum);
    EXPECT_FALSE(m[0].has_stddev);
}

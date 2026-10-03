// Copyright 2026 ros2_pulse contributors
//
// Unit tests for take-layer receive counting (the rmw_take tracepoint). rclcpp receives are
// counted at callback_start; every OTHER rcl client (rclpy, rclc, ...) never reaches
// callback_start, so its receives are counted when the rmw layer reports a successful take.
// rmw_take carries the rmw subscription handle, which rcl_subscription_init links to the rcl
// handle and topic. Registers into the shared test binary (no main()).

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"

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

// An rclpy-style subscription: rcl_subscription_init only, no rclcpp tracepoints ever.
void initRclpySub(TopicRegistry& reg, uintptr_t rcl, uintptr_t rmw, const char* topic) {
    reg.onSubscriptionInit(H(rcl), nullptr, topic, H(rmw));
}

// An rclcpp-style subscription: rcl init, then rclcpp_subscription_init and callback_added.
void initRclcppSub(TopicRegistry& reg, uintptr_t rcl, uintptr_t rmw, uintptr_t sub,
                   uintptr_t cb, const char* topic) {
    reg.onSubscriptionInit(H(rcl), nullptr, topic, H(rmw));
    reg.onRclcppSubscriptionInit(H(sub), H(rcl));
    reg.onCallbackAdded(H(cb), H(sub));
}

}  // namespace

// The core of the spike: a successful take on a non-rclcpp subscription is a receive.
TEST(TakeReceive, RclpyTakeCountsAsInterReceive) {
    TopicRegistry reg;
    initRclpySub(reg, 0x20, 0x21, "/chatter");
    for (int i = 0; i < 50; ++i) reg.onTake(H(0x21), /*taken=*/true);

    auto snap = reg.snapshot(5.0);
    const auto* s = findTopic(snap, "/chatter");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 50u);
    EXPECT_DOUBLE_EQ(s->recv_inter_hz, 10.0);
    EXPECT_EQ(s->recv_intra_count, 0u);
    EXPECT_EQ(s->pub_inter_count, 0u);
    EXPECT_TRUE(TopicRegistry::shouldEmitRecv(*s));
}

// rmw_take fires on EVERY take attempt, including the empty ones (a spurious wakeup, or Fast
// DDS skipping an ignore_local_publications sample). Only taken=true is a delivered message.
TEST(TakeReceive, FailedTakeIsNotAReceive) {
    TopicRegistry reg;
    initRclpySub(reg, 0x20, 0x21, "/chatter");
    for (int i = 0; i < 30; ++i) reg.onTake(H(0x21), /*taken=*/false);

    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/chatter");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 0u);
    // Never delivered, so not a proven receive endpoint: stays suppressed (issue #7).
    EXPECT_FALSE(s->recv_endpoint_seen);
    EXPECT_FALSE(TopicRegistry::shouldEmitRecv(*s));
}

// A subscription with no publisher never takes, so it reports nothing at all.
TEST(TakeReceive, SubscriptionWithoutPublisherReportsZero) {
    TopicRegistry reg;
    initRclpySub(reg, 0x20, 0x21, "/nobody");
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/nobody");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 0u);
    EXPECT_FALSE(TopicRegistry::shouldEmitRecv(*s));
}

// The first successful take proves the endpoint, so a dead upstream later reads an explicit
// RECV 0.0 instead of vanishing (KNOWN_ISSUES #12), same contract as the callback path.
TEST(TakeReceive, FirstTakeProvesEndpointForStallVisibility) {
    TopicRegistry reg;
    initRclpySub(reg, 0x20, 0x21, "/scan");
    reg.onTake(H(0x21), true);
    (void)reg.snapshot(1.0);

    auto idle = reg.snapshot(1.0);
    const auto* s = findTopic(idle, "/scan");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 0u);
    EXPECT_TRUE(s->recv_endpoint_seen);
    EXPECT_TRUE(TopicRegistry::shouldEmitRecv(*s));
}

// Dedupe: an rclcpp subscription's inter-process message fires BOTH rmw_take and
// callback_start(intra=false). callback_start stays the rclcpp source of truth (it also sees the
// serialized and loaned takes that emit no rmw_take on Humble), so the take must not count.
TEST(TakeReceive, RclcppSubscriptionIsNotDoubleCounted) {
    TopicRegistry reg;
    initRclcppSub(reg, 0x20, 0x21, 0x22, 0x23, "/chatter");
    for (int i = 0; i < 40; ++i) {
        reg.onTake(H(0x21), true);
        reg.onCallbackStart(H(0x23), /*is_intra_process=*/false);
    }
    auto snap = reg.snapshot(4.0);
    const auto* s = findTopic(snap, "/chatter");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 40u);
    EXPECT_DOUBLE_EQ(s->recv_inter_hz, 10.0);
}

// rclcpp intra-process deliveries never touch rmw (callback_start(intra=true) only), and the
// rclcpp-owned take is still ignored when an inter-process peer shares the topic.
TEST(TakeReceive, RclcppIntraStillCountedViaCallbackStart) {
    TopicRegistry reg;
    initRclcppSub(reg, 0x20, 0x21, 0x22, 0x23, "/points");
    for (int i = 0; i < 30; ++i) reg.onCallbackStart(H(0x23), /*is_intra_process=*/true);
    for (int i = 0; i < 10; ++i) {
        reg.onTake(H(0x21), true);
        reg.onCallbackStart(H(0x23), /*is_intra_process=*/false);
    }
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/points");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_intra_count, 30u);
    EXPECT_EQ(s->recv_inter_count, 10u);
}

// Ownership is decided per subscription, not per topic: in one process an rclcpp sub and a
// non-rclcpp sub on the same topic each contribute exactly their own deliveries (the same
// per-subscription semantics two rclcpp subs on one topic already have).
TEST(TakeReceive, OwnershipIsPerSubscription) {
    TopicRegistry reg;
    initRclcppSub(reg, 0x20, 0x21, 0x22, 0x23, "/mixed");
    initRclpySub(reg, 0x30, 0x31, "/mixed");
    for (int i = 0; i < 7; ++i) {
        reg.onTake(H(0x21), true);
        reg.onCallbackStart(H(0x23), false);
        reg.onTake(H(0x31), true);
    }
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/mixed");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 14u);
}

// Takes on a handle the registry never saw initialized, or a null handle, are dropped quietly.
// The rmw's internal discovery reader (ros_discovery_info) takes through the same tracepoint
// with a handle no rcl_subscription_init names; its steady traffic must stay off the lock too.
TEST(TakeReceive, UnknownOrNullHandleIgnored) {
    TopicRegistry reg;
    initRclpySub(reg, 0x20, 0x21, "/known");
    reg.onTake(H(0x99), true);
    reg.onTake(nullptr, true);
    const auto before = reg.sharedLockLookups();
    for (int i = 0; i < 100; ++i) reg.onTake(H(0x99), true);
    EXPECT_EQ(reg.sharedLockLookups(), before);
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/known");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 0u);
}

// The cached "unknown handle" verdict must not outlive a later rcl_subscription_init of that
// address: once it is a real subscription, its takes count.
TEST(TakeReceive, UnknownVerdictDroppedOnLaterInit) {
    TopicRegistry reg;
    for (int i = 0; i < 3; ++i) reg.onTake(H(0x51), true);  // cached as unknown
    initRclpySub(reg, 0x50, 0x51, "/later");
    for (int i = 0; i < 6; ++i) reg.onTake(H(0x51), true);
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/later");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 6u);
}

// A subscription initialized WITHOUT an rmw handle (the legacy 3-argument form) is simply not
// reachable from the take layer; nothing crashes and the callback path is unaffected.
TEST(TakeReceive, LegacyInitWithoutRmwHandle) {
    TopicRegistry reg;
    reg.onSubscriptionInit(H(0x20), nullptr, "/legacy");
    reg.onTake(nullptr, true);
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/legacy");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 0u);
}

// The take path's thread-local cache must not serve a stale answer after the rmw handle is
// recycled by a NEW subscription (rclpy destroy + create is routine): the first sub's topic must
// stop receiving and the new topic must get every take, even on a thread that warmed the cache.
TEST(TakeReceive, RecycledRmwHandleFollowsTheNewSubscription) {
    TopicRegistry reg;
    initRclpySub(reg, 0x20, 0x21, "/old");
    for (int i = 0; i < 5; ++i) reg.onTake(H(0x21), true);  // warm the TLS slot
    initRclpySub(reg, 0x40, 0x21, "/new");                   // same rmw address, new sub
    for (int i = 0; i < 9; ++i) reg.onTake(H(0x21), true);

    auto snap = reg.snapshot(1.0);
    const auto* o = findTopic(snap, "/old");
    const auto* n = findTopic(snap, "/new");
    ASSERT_NE(o, nullptr);
    ASSERT_NE(n, nullptr);
    EXPECT_EQ(o->recv_inter_count, 5u);
    EXPECT_EQ(n->recv_inter_count, 9u);
}

// A recycled rcl handle that was rclcpp-owned and is now a non-rclcpp subscription must start
// counting at the take layer again (ownership is reset by each rcl_subscription_init).
TEST(TakeReceive, RecycledRclHandleResetsOwnership) {
    TopicRegistry reg;
    initRclcppSub(reg, 0x20, 0x21, 0x22, 0x23, "/a");
    reg.onTake(H(0x21), true);  // ignored, and caches the "rclcpp-owned" verdict
    initRclpySub(reg, 0x20, 0x21, "/a");
    for (int i = 0; i < 4; ++i) reg.onTake(H(0x21), true);
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/a");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 4u);
}

// The rclcpp-owned verdict must also win over a TLS slot warmed BEFORE rclcpp_subscription_init
// (cannot happen in rclcpp's real ordering, but the cache must never be the reason it would).
TEST(TakeReceive, OwnershipMarkInvalidatesWarmCache) {
    TopicRegistry reg;
    initRclpySub(reg, 0x20, 0x21, "/late_owner");
    reg.onTake(H(0x21), true);  // counted, slot warm
    reg.onRclcppSubscriptionInit(H(0x22), H(0x20));
    reg.onTake(H(0x21), true);  // now rclcpp-owned, must be ignored
    auto snap = reg.snapshot(1.0);
    const auto* s = findTopic(snap, "/late_owner");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->recv_inter_count, 1u);
}

// Steady state stays off the lock: after the first take per thread, every further take on the
// same handle is a thread-local hit (same KNOWN_ISSUES #13 contract as publish/callback).
TEST(TakeReceive, SteadyStateIsLockFree) {
    TopicRegistry reg;
    initRclpySub(reg, 0x20, 0x21, "/a");
    initRclpySub(reg, 0x30, 0x31, "/b");
    reg.onTake(H(0x21), true);
    reg.onTake(H(0x31), true);
    const auto before = reg.sharedLockLookups();
    for (int i = 0; i < 1000; ++i) {
        reg.onTake(H(0x21), true);
        reg.onTake(H(0x31), true);
        reg.onTake(H(0x21), false);  // failed takes never even reach the cache
    }
    EXPECT_EQ(reg.sharedLockLookups(), before);
    auto snap = reg.snapshot(1.0);
    EXPECT_EQ(findTopic(snap, "/a")->recv_inter_count, 1001u);
    EXPECT_EQ(findTopic(snap, "/b")->recv_inter_count, 1001u);
}

// Gap tracking (R5) applies to take-layer receives too, so a stalled rclpy subscriber is
// visible to a max_gap_ms rule exactly like an rclcpp one.
TEST(TakeReceive, GapTrackingCoversTakePath) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    initRclpySub(reg, 0x20, 0x21, "/gappy");
    reg.onTake(H(0x21), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    reg.onTake(H(0x21), true);
    auto snap = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = findTopic(snap, "/gappy");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_recv_max_dt);
    EXPECT_GE(s->recv_max_dt_ms, 15.0);
}

// Concurrency (exercised under the tsan lane): several executor threads taking on several
// handles while graph init churns ownership on an unrelated subscription. Totals stay exact.
TEST(TakeReceive, ConcurrentTakesExactTotals) {
    TopicRegistry reg;
    initRclpySub(reg, 0x100, 0x101, "/c0");
    initRclpySub(reg, 0x200, 0x201, "/c1");
    constexpr int kThreads = 4;
    constexpr int kPer = 20000;
    std::atomic<bool> go{false};
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([&] {
            while (!go.load()) {
            }
            for (int i = 0; i < kPer; ++i) {
                reg.onTake(H(0x101), true);
                reg.onTake(H(0x201), (i & 1) == 0);
            }
        });
    }
    std::thread churn([&] {
        while (!go.load()) {
        }
        for (int i = 0; i < 200; ++i) {
            reg.onSubscriptionInit(H(0x900), nullptr, "/churn", H(0x901));
            reg.onRclcppSubscriptionInit(H(0x902), H(0x900));
        }
    });
    go.store(true);
    for (auto& t : ts) t.join();
    churn.join();
    auto snap = reg.snapshot(1.0);
    EXPECT_EQ(findTopic(snap, "/c0")->recv_inter_count, uint64_t{kThreads} * kPer);
    EXPECT_EQ(findTopic(snap, "/c1")->recv_inter_count, uint64_t{kThreads} * kPer / 2);
}

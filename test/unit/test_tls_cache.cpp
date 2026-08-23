// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the thread-local hot-path cache under ALTERNATING endpoints
// (KNOWN_ISSUES #13). The single-entry cache thrashed on the realistic pattern (one thread
// publishing several topics per cycle) and pushed every operation onto the shared rw-lock;
// the mini-map must keep the steady state lock-free. Registers into the shared test binary.

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
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

}  // namespace

// The camera-pipeline pattern: one thread alternating between endpoints every call. After a
// warm-up, the steady state must be served from the thread-local cache, the shared-lock
// lookup counter stays flat. Pre-fix, the single-entry cache missed on EVERY call (~2000).
TEST(TlsMiniMap, AlternatingPublishersStayLockFree) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/img");
    reg.onPublisherInit(H(0x20), nullptr, "/info");
    reg.onPublish(H(0x10));  // warm both slots
    reg.onPublish(H(0x20));

    const uint64_t before = reg.sharedLockLookups();
    for (int i = 0; i < 1000; ++i) {
        reg.onPublish(H(0x10));
        reg.onPublish(H(0x20));
    }
    EXPECT_LE(reg.sharedLockLookups() - before, 4u)
        << "alternating publishers fell off the thread-local cache onto the shared lock";
}

TEST(TlsMiniMap, AlternatingCallbacksStayLockFree) {
    TopicRegistry reg;
    reg.onSubscriptionInit(H(0x30), nullptr, "/x");
    reg.onRclcppSubscriptionInit(H(0x31), H(0x30));
    reg.onCallbackAdded(H(0x40), H(0x31));
    reg.onSubscriptionInit(H(0x50), nullptr, "/y");
    reg.onRclcppSubscriptionInit(H(0x51), H(0x50));
    reg.onCallbackAdded(H(0x60), H(0x51));
    reg.onCallbackStart(H(0x40), false);  // resolve + warm
    reg.onCallbackStart(H(0x60), true);

    const uint64_t before = reg.sharedLockLookups();
    for (int i = 0; i < 1000; ++i) {
        reg.onCallbackStart(H(0x40), false);
        reg.onCallbackStart(H(0x60), true);
    }
    EXPECT_LE(reg.sharedLockLookups() - before, 4u)
        << "alternating callbacks fell off the thread-local cache onto the shared lock";
}

// A realistic publisher farm (KNOWN_ISSUES #15): many same-type handles sit at a uniform
// allocator stride. The 16-slot cache's (ptr>>4) index aliases uniform strides, at stride 64
// the index advances by 4 (mod 16), so 38 handles shared 4 slots and evicted each other on
// ~every publish. Each miss is a contended shared_mutex read-lock + a shared-counter
// fetch_add, measured at +2-4% workload CPU under a MultiThreadedExecutor (see the issue
// note). Steady state must stay on the thread-local cache: fallbacks under 10% of accesses.
TEST(TlsMiniMap, StridedPublisherFarmStaysLockFree) {
    constexpr int kPubs = 38;
    constexpr int kStride = 64;
    constexpr int kRounds = 100;
    alignas(4096) static std::byte arena[kPubs * kStride];

    TopicRegistry reg;
    for (int i = 0; i < kPubs; ++i) {
        reg.onPublisherInit(&arena[i * kStride], nullptr, ("/farm" + std::to_string(i)).c_str());
    }
    for (int i = 0; i < kPubs; ++i) {
        reg.onPublish(&arena[i * kStride]);  // warm-up pass
    }

    const uint64_t before = reg.sharedLockLookups();
    for (int round = 0; round < kRounds; ++round) {
        for (int i = 0; i < kPubs; ++i) {
            reg.onPublish(&arena[i * kStride]);
        }
    }
    const uint64_t fallbacks = reg.sharedLockLookups() - before;
    EXPECT_LE(fallbacks, kPubs * kRounds / 10)
        << "allocator-stride publisher farm fell off the thread-local cache onto the shared "
           "lock (" << fallbacks << " of " << kPubs * kRounds << " accesses)";

    auto snap = reg.snapshot(1.0);
    for (int i = 0; i < kPubs; ++i) {
        const auto* s = findTopic(snap, "/farm" + std::to_string(i));
        ASSERT_NE(s, nullptr);
        EXPECT_EQ(s->pub_inter_count, static_cast<uint64_t>(kRounds) + 1)
            << "count lost or doubled for /farm" << i;
    }
}

// Correctness under alternation: replace-on-collision must never lose or double a count.
TEST(TlsMiniMap, ExactTotalsUnderAlternation) {
    TopicRegistry reg;
    reg.onPublisherInit(H(0x10), nullptr, "/img");
    reg.onPublisherInit(H(0x20), nullptr, "/info");
    for (int i = 0; i < 500; ++i) {
        reg.onPublish(H(0x10));
        reg.onPublish(H(0x20));
    }
    auto snap = reg.snapshot(1.0);
    const auto* a = findTopic(snap, "/img");
    const auto* b = findTopic(snap, "/info");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(a->pub_inter_count, 500u);
    EXPECT_EQ(b->pub_inter_count, 500u);
}

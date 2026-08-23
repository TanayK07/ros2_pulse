// Copyright 2026 ros2_pulse contributors
//
// Property / fuzz test for init-order robustness. The probe layer feeds the registry five
// graph-init calls whose relative order is NOT guaranteed (see the intra-process ordering bug
// captured by TopicRegistry.LazyResolutionWhenInitOutOfOrder): callback_added can fire before
// the sub_handle->topic chain is populated. Here we randomize the order of all five init calls
// and sprinkle hot-path calls in between, across many deterministic trials, and assert:
//
//   (a) it never crashes / never mis-resolves, regardless of interleaving, and
//   (b) once every chain is complete, counting CONVERGES to exact totals.
//
// Deterministic: a fixed-seed std::mt19937 drives every choice, so a failure reproduces from
// the printed trial index + seed. Fix-independent, passes on main today.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <random>
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

int envInt(const char* key, int def) {
    const char* v = std::getenv(key);
    if (v == nullptr || *v == '\0') return def;
    try {
        return std::max(1, std::stoi(v));
    } catch (...) {
        return def;
    }
}

// The five low-frequency graph-init operations, as closures over one registry + trial handles.
enum InitStep { kPubInit, kSubInit, kRclcppSub, kCbAdded, kNodeInit };

}  // namespace

TEST(InitOrderProperty, RandomOrderNeverCrashesAndConverges) {
    // Fixed seed -> fully reproducible. Override trial count for a longer local soak.
    constexpr uint32_t kSeed = 0xC0FFEEu;
    const int kTrials = envInt("ROS2_PULSE_PROP_TRIALS", 400);
    std::mt19937 rng(kSeed);

    // Stable fake handles reused every trial; a fresh registry (with a fresh m_id) per trial
    // means the thread-local cache from the previous trial can never bleed in.
    const void* kPub = H(0xA100);
    const void* kSubH = H(0xA200);
    const void* kRclSub = H(0xA300);
    const void* kCb = H(0xA400);

    for (int trial = 0; trial < kTrials; ++trial) {
        SCOPED_TRACE("trial=" + std::to_string(trial) + " seed=" + std::to_string(kSeed));
        TopicRegistry reg;

        std::array<InitStep, 5> order{kPubInit, kSubInit, kRclcppSub, kCbAdded, kNodeInit};
        std::shuffle(order.begin(), order.end(), rng);

        // Apply the inits in the shuffled order, firing a random number of hot-path calls
        // BEFORE each init step. These early calls may legitimately be dropped (chain not yet
        // resolvable), the only requirement here is that nothing crashes.
        std::uniform_int_distribution<int> noise(0, 4);
        std::bernoulli_distribution intra(0.5);
        // In real rclcpp a subscription callback is registered (callback_added) before the executor
        // can ever invoke it, so callback_start never precedes callback_added. Respect that here:
        // only fire callback noise once kCbAdded has run (the negative cache relies on this real
        // invariant). This still exercises the genuine hazard, callback_start firing while the
        // sub_handle->topic chain is only partially populated. Publishes have no such constraint.
        bool cb_added = false;
        for (InitStep step : order) {
            int pre = noise(rng);
            for (int i = 0; i < pre; ++i) {
                reg.onPublish(kPub);
                if (cb_added) reg.onCallbackStart(kCb, intra(rng));
            }
            switch (step) {
                case kPubInit:
                    reg.onPublisherInit(kPub, nullptr, "/p");
                    break;
                case kSubInit:
                    reg.onSubscriptionInit(kSubH, nullptr, "/s");
                    break;
                case kRclcppSub:
                    reg.onRclcppSubscriptionInit(kRclSub, kSubH);
                    break;
                case kCbAdded:
                    reg.onCallbackAdded(kCb, kRclSub);
                    cb_added = true;
                    break;
                case kNodeInit:
                    reg.onNodeInit(nullptr, "n", "/ns");
                    break;
            }
        }

        // Every chain is now complete. Reset whatever partial counts the noise produced so the
        // convergence assertion is exact, then drive fixed, known batches.
        reg.snapshot(1.0);

        std::uniform_int_distribution<int> batch(1, 50);
        const int kp = batch(rng);
        const int ki = batch(rng);
        const int kx = batch(rng);
        for (int i = 0; i < kp; ++i) reg.onPublish(kPub);
        for (int i = 0; i < ki; ++i) reg.onCallbackStart(kCb, /*intra=*/false);
        for (int i = 0; i < kx; ++i) reg.onCallbackStart(kCb, /*intra=*/true);

        auto snap = reg.snapshot(1.0);

        // Publish side ("/p") and receive side ("/s") are distinct topic names -> distinct
        // counters, so the totals are unambiguous and independent of the double-count issue #1.
        const auto* p = findTopic(snap, "/p");
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(p->pub_inter_count, static_cast<uint64_t>(kp));

        const auto* s = findTopic(snap, "/s");
        ASSERT_NE(s, nullptr);
        EXPECT_EQ(s->recv_inter_count, static_cast<uint64_t>(ki));
        EXPECT_EQ(s->recv_intra_count, static_cast<uint64_t>(kx));

        auto nodes = reg.activeNodes();
        ASSERT_EQ(nodes.size(), 1u);
        EXPECT_EQ(nodes[0], "/ns/n");
    }
}

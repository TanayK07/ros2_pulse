// Copyright 2026 ros2_pulse contributors
//
// Fix-independent regression guard: the thread-local hot-path cache in onPublish /
// onCallbackStart is scoped by a per-registry id (m_id). When a registry is destroyed and a
// new one is constructed, possibly at the SAME heap address, the new instance must NEVER
// serve a counter that belonged to the destroyed one. This is the subtlest invariant in the
// core (a classic use-after-free footgun) and it is already correct on main; this test pins
// it down under both the plain and ASan build lanes.

#include <gtest/gtest.h>

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

}  // namespace

// Warm the publish-side thread-local cache on reg1, free it, then reuse the same handle on a
// fresh reg2 that maps it to a DIFFERENT topic. reg2 must resolve to its OWN counter.
TEST(RegistryLifecycle, PublishCacheNeverServesDestroyedInstance) {
    auto* reg1 = new TopicRegistry();
    reg1->onPublisherInit(H(0x10), nullptr, "/old");
    reg1->onPublish(H(0x10));  // primes thread-local {id=reg1.id, key=0x10, ctr=&/old}
    reg1->onPublish(H(0x10));  // hits the primed fast path
    {
        auto s1 = reg1->snapshot(1.0);
        const auto* o = findTopic(s1, "/old");
        ASSERT_NE(o, nullptr);
        EXPECT_EQ(o->pub_inter_count, 2u);
    }
    delete reg1;  // thread-local last_ctr is now dangling, must never be dereferenced again

    // Fresh instance; may be recycled at reg1's address. Same handle, different topic.
    auto* reg2 = new TopicRegistry();
    reg2->onPublisherInit(H(0x10), nullptr, "/new");
    reg2->onPublish(H(0x10));  // id mismatch forces a real lookup -> reg2's own counter
    reg2->onPublish(H(0x10));
    reg2->onPublish(H(0x10));

    auto s2 = reg2->snapshot(1.0);
    const auto* n = findTopic(s2, "/new");
    ASSERT_NE(n, nullptr);
    EXPECT_EQ(n->pub_inter_count, 3u);
    EXPECT_EQ(findTopic(s2, "/old"), nullptr);  // reg2 never heard of /old
    delete reg2;
}

// Same guard, but reg2 never learns the recycled handle at all. A stale cache hit would leak
// the destroyed counter (UAF); the correct behaviour is: nothing is counted.
TEST(RegistryLifecycle, PublishCacheDropsUnknownHandleOnFreshInstance) {
    auto* reg1 = new TopicRegistry();
    reg1->onPublisherInit(H(0x2000), nullptr, "/gone");
    for (int i = 0; i < 5; i++) reg1->onPublish(H(0x2000));
    delete reg1;

    auto* reg2 = new TopicRegistry();  // knows nothing about 0x2000
    reg2->onPublish(H(0x2000));
    auto s2 = reg2->snapshot(1.0);
    EXPECT_TRUE(s2.empty());  // no stale counter served
    delete reg2;
}

// The callback path has an independent thread-local cache; assert the same id-scoping.
TEST(RegistryLifecycle, CallbackCacheNeverServesDestroyedInstance) {
    auto* reg1 = new TopicRegistry();
    reg1->onSubscriptionInit(H(0x20), nullptr, "/old_recv");
    reg1->onRclcppSubscriptionInit(H(0x21), H(0x20));
    reg1->onCallbackAdded(H(0x22), H(0x21));
    for (int i = 0; i < 4; i++) reg1->onCallbackStart(H(0x22), /*intra=*/true);  // prime cache
    delete reg1;

    auto* reg2 = new TopicRegistry();
    reg2->onSubscriptionInit(H(0x20), nullptr, "/new_recv");  // same handles, new topic
    reg2->onRclcppSubscriptionInit(H(0x21), H(0x20));
    reg2->onCallbackAdded(H(0x22), H(0x21));
    for (int i = 0; i < 6; i++) reg2->onCallbackStart(H(0x22), /*intra=*/true);

    auto s2 = reg2->snapshot(1.0);
    const auto* n = findTopic(s2, "/new_recv");
    ASSERT_NE(n, nullptr);
    EXPECT_EQ(n->recv_intra_count, 6u);
    EXPECT_EQ(findTopic(s2, "/old_recv"), nullptr);
    delete reg2;
}

// The cache is thread-local, so a worker thread that primed its cache against reg1 must also
// not serve reg1's counter after reg1 dies and reg2 is created. Run the priming and the
// verification on the SAME worker thread so the thread-local actually carries over.
TEST(RegistryLifecycle, CacheScopingHoldsOnWorkerThread) {
    uint64_t observed = ~0ull;
    std::thread worker([&] {
        auto* reg1 = new TopicRegistry();
        reg1->onPublisherInit(H(0x30), nullptr, "/t_old");
        reg1->onPublish(H(0x30));
        reg1->onPublish(H(0x30));
        delete reg1;

        auto* reg2 = new TopicRegistry();
        reg2->onPublisherInit(H(0x30), nullptr, "/t_new");
        reg2->onPublish(H(0x30));
        auto s = reg2->snapshot(1.0);
        const auto* n = findTopic(s, "/t_new");
        observed = (n != nullptr) ? n->pub_inter_count : ~0ull;
        EXPECT_EQ(findTopic(s, "/t_old"), nullptr);
        delete reg2;
    });
    worker.join();
    EXPECT_EQ(observed, 1u);
}

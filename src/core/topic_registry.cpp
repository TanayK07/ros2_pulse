// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "ros2_pulse/core/topic_registry.hpp"

#include <chrono>
#include <mutex>  // for std::unique_lock with shared_mutex
#include <new>    // placement-new (forkChildReset)
#include <unordered_set>

namespace ros2_pulse::core {

namespace {
std::atomic<uint64_t> g_next_registry_id{1};

// Negative-cache sentinel (KNOWN_ISSUES #3). A callback proven NEVER to be a subscription, a timer
// or service callback, for which callback_added never fired, resolves to this marker instead of
// nullptr, so repeat sightings are served from the shared-lock / thread-local fast paths rather than
// re-taking the exclusive lock on every call. It is a unique, valid address that is never
// dereferenced: every read site compares it by identity and skips (see onCallbackStart).
sTopicCounter g_not_a_subscription;
sTopicCounter* const kNotASubscription = &g_not_a_subscription;

// Direct-mapped thread-local hot-path cache, shared by onPublish and onCallbackStart (publisher
// handles and callback objects are distinct live allocations, so the key domains cannot
// collide). A single-entry cache thrashed on the REALISTIC pattern, one thread alternating
// between a few endpoints per cycle, pushing every operation onto the shared rw-lock
// (KNOWN_ISSUES #13). A same-slot collision merely degrades that key to the shared-lock path
// (it is a cache, not a map). Entries are scoped by registry id so a recycled instance address
// can never be served a destroyed registry's counter (same guard the single-entry cache had).
//
// Sizing/hash (KNOWN_ISSUES #15): a bare (ptr>>4) index aliases on uniform allocator strides,
// same-type handles at a 64-byte stride advanced the index by 4, so a 38-publisher farm shared
// 4 of 16 slots and missed on ~every event; under a MultiThreadedExecutor each miss is a
// CONTENDED rw-lock RMW + shared-counter bounce, measured at +2-4% workload CPU. 256 slots
// (6 KB zero-init TLS per thread) cover a realistic per-thread working set, and xor-folding
// higher address bits breaks stride aliasing.
struct sTlsSlot {
    uint64_t id{0};
    const void* key{nullptr};
    sTopicCounter* ctr{nullptr};
};
constexpr uintptr_t kTlsSlotMask = 255;
thread_local sTlsSlot t_tls_cache[kTlsSlotMask + 1];

inline auto tlsSlotFor(const void* key) -> sTlsSlot& {
    // Live-object addresses have zeroed alignment bits; >>4 drops them, and folding in >>10
    // spreads uniform allocation strides across the slot space.
    const auto p = reinterpret_cast<uintptr_t>(key);
    return t_tls_cache[((p >> 4) ^ (p >> 10)) & kTlsSlotMask];
}

// ROADMAP R5: record one arrival into a side's gap accumulator.
//
// `exchange`, not load+store: it guarantees exactly ONE caller consumes each predecessor
// timestamp, so two threads delivering to the same endpoint can never derive two dt's from the
// same `prev` (which would double-count one interval and lose another).
//
// The `now > prev` guard is MANDATORY, not defensive. `now` is sampled before the exchange, so
// two threads can exchange out of order; without the guard the unsigned subtraction underflows
// to ~1.8e19 ns and poisons max_dt permanently. Discarding those samples is the correct trade:
// measured under 8-way contention on one endpoint, 0.6-9.7% of samples are dropped and the
// reported max is over-stated by at most 5%, never garbage, never negative, and erring high is
// the safe direction for a stall detector. Single-threaded delivery is exact.
inline void noteArrival(std::atomic<uint64_t>& last_ns, std::atomic<uint64_t>& max_dt_ns) {
    const auto now = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const uint64_t prev = last_ns.exchange(now, std::memory_order_relaxed);
    if (prev == 0 || now <= prev) {
        return;  // first arrival ever, or a reordered pair, no usable interval
    }
    const uint64_t dt = now - prev;
    // std::atomic::fetch_max is C++26; this is the portable form. It costs nothing in steady
    // state because new maxima follow the harmonic number, ~14 CAS per 500 messages.
    uint64_t cur = max_dt_ns.load(std::memory_order_relaxed);
    while (dt > cur && !max_dt_ns.compare_exchange_weak(cur, dt, std::memory_order_relaxed)) {
    }
}

// Drain one side's gap accumulator for the window just ending. Returns false when the endpoint
// has never seen an arrival: no interval exists, which is NOT the same as a gap of zero, so the
// caller must not report a field. Resets max_dt; deliberately leaves last_ns intact.
inline auto takeGap(std::atomic<uint64_t>& last_ns, std::atomic<uint64_t>& max_dt_ns,
                    uint64_t now_ns, bool fold_open_gap, double& out_ms) -> bool {
    const uint64_t last = last_ns.load(std::memory_order_relaxed);
    if (last == 0) {
        return false;  // never active
    }
    uint64_t dt = max_dt_ns.exchange(0, std::memory_order_relaxed);
    if (fold_open_gap && now_ns > last) {
        const uint64_t open = now_ns - last;
        if (open > dt) {
            dt = open;
        }
    }
    if (dt == 0) {
        // A timestamp but no measured interval, the endpoint has been seen exactly once (and
        // the open gap is not being folded). Reporting 0.000 would read as "perfectly regular",
        // which is the best possible value, inferred from a single data point. The only real
        // sample this discards is two arrivals inside one clock tick (~21 ns), which can never
        // be a maximum and is already dropped by noteArrival's now > prev guard.
        return false;
    }
    out_ms = static_cast<double>(dt) / 1e6;
    return true;
}

}  // namespace

TopicRegistry::TopicRegistry(uint32_t quiet_windows)
    : m_id(g_next_registry_id.fetch_add(1, std::memory_order_relaxed)),
      m_quiet_windows(quiet_windows) {}

auto TopicRegistry::shouldFilter(const std::string& topic) -> bool {
    return topic == "/parameter_events" || topic == "/rosout" || topic == "/diagnostics";
}

auto TopicRegistry::shouldEmitTopic(const sTopicStat& stat, bool emit_idle) -> bool {
    // Fully-idle topic (declared but silent this window): keep it out of the file by default so a
    // large graph isn't padded with `TOPIC /x 0.000000` lines every window; only the explicit
    // opt-in restores it. See KNOWN_ISSUES.md #7.
    const bool fully_idle = stat.pub_inter_count == 0 && stat.pub_intra_count == 0 &&
                            stat.recv_inter_count == 0 && stat.recv_intra_count == 0;
    if (fully_idle) {
        return emit_idle;
    }
    // Non-idle: TOPIC is the publish-side line (split buckets, issue #1), emit only when this
    // process actually published. A receive-only topic's rate lives on the RECV line instead.
    return stat.pub_inter_count > 0;
}

auto TopicRegistry::shouldEmitRecv(const sTopicStat& stat) -> bool {
    // Receive traffic always emits; a proven (once-delivered) endpoint emits a zero line when
    // idle so a dead upstream reads 0.0 instead of vanishing (KNOWN_ISSUES #12). Never-active
    // subscriptions stay suppressed (issue #7: declared-but-silent is noise).
    return stat.recv_inter_count > 0 || stat.recv_intra_count > 0 || stat.recv_endpoint_seen;
}

auto TopicRegistry::counterForTopic(const std::string& topic) -> sTopicCounter* {
    auto it = m_by_topic.find(topic);
    if (it != m_by_topic.end()) {
        return it->second.get();
    }
    auto ctr = std::make_unique<sTopicCounter>();
    ctr->topic = topic;
    auto* raw = ctr.get();
    m_by_topic.emplace(topic, std::move(ctr));
    return raw;
}

void TopicRegistry::linkNodeCounter(const void* node_handle, sTopicCounter* counter) {
    if (node_handle == nullptr || counter == nullptr) {
        return;
    }
    auto it = m_nodehandle_to_node.find(node_handle);
    if (it == m_nodehandle_to_node.end()) {
        return;
    }
    auto& counters = it->second->counters;
    for (auto* c : counters) {
        if (c == counter) {
            return;  // already listed for this node
        }
    }
    counters.push_back(counter);
}

void TopicRegistry::onPublisherInit(const void* pub_handle, const void* node_handle,
                                    const char* topic) {
    if (pub_handle == nullptr || topic == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    auto* counter = counterForTopic(topic);
    m_pub_to_counter[pub_handle] = counter;
    linkNodeCounter(node_handle, counter);
}

void TopicRegistry::onSubscriptionInit(const void* sub_handle, const void* node_handle,
                                       const char* topic) {
    if (sub_handle == nullptr || topic == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    auto* counter = counterForTopic(topic);
    m_subhandle_to_counter[sub_handle] = counter;
    linkNodeCounter(node_handle, counter);
}

void TopicRegistry::onRclcppSubscriptionInit(const void* subscription, const void* sub_handle) {
    if (subscription == nullptr || sub_handle == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    m_sub_to_subhandle[subscription] = sub_handle;
}

void TopicRegistry::onCallbackAdded(const void* callback, const void* subscription) {
    if (callback == nullptr || subscription == nullptr) {
        return;
    }
    // Record the link only. Resolution to a topic is deferred to onCallbackStart, because for
    // intra-process subscriptions this fires before the intra waitable's full chain is populated.
    std::unique_lock<std::shared_mutex> lock(m_mu);
    m_cb_to_sub[callback] = subscription;
}

void TopicRegistry::onNodeInit(const void* node_handle, const char* node_name,
                               const char* node_namespace) {
    if (node_name == nullptr) {
        return;
    }
    std::string ns = node_namespace ? node_namespace : "";
    std::string full = (ns.empty() || ns == "/") ? ("/" + std::string(node_name))
                                                  : (ns + "/" + std::string(node_name));
    std::unique_lock<std::shared_mutex> lock(m_mu);
    sNode* node = nullptr;
    auto existing = m_node_by_name.find(full);
    if (existing != m_node_by_name.end()) {
        // Dedup: same name -> reuse the record. A re-init is treated as a fresh start.
        node = existing->second;
        node->idle_windows = 0;
    } else {
        auto owned = std::make_unique<sNode>();
        owned->name = full;
        node = owned.get();
        m_nodes.push_back(std::move(owned));
        m_node_by_name.emplace(full, node);
    }
    if (node_handle != nullptr) {
        m_nodehandle_to_node[node_handle] = node;
    }
}

auto TopicRegistry::resolveCallback(const void* callback) -> sTopicCounter* {
    auto cached = m_cb_to_counter.find(callback);
    if (cached != m_cb_to_counter.end()) {
        return cached->second;  // a resolved counter, or the kNotASubscription sentinel
    }
    auto s = m_cb_to_sub.find(callback);
    if (s == m_cb_to_sub.end()) {
        // No callback_added ever recorded a subscription for this callback. Since callback_added
        // (a graph-init event) always precedes the first callback_start of a real subscription,
        // absence here PROVES this is a timer/service callback that can never resolve. Cache the
        // negative result so later sightings hit the shared-lock / thread-local fast paths.
        m_cb_to_counter[callback] = kNotASubscription;
        return kNotASubscription;
    }
    auto h = m_sub_to_subhandle.find(s->second);
    if (h == m_sub_to_subhandle.end()) {
        return nullptr;  // subscription, but chain not populated yet, stay lazy, do NOT cache
    }
    auto c = m_subhandle_to_counter.find(h->second);
    if (c == m_subhandle_to_counter.end()) {
        return nullptr;  // ditto: retry on a later delivery once subscription_init lands
    }
    m_cb_to_counter[callback] = c->second;  // cache once fully resolved
    // First delivered message for this subscription: the topic is now a PROVEN receive
    // endpoint, so idle windows keep emitting an explicit RECV 0.0 line for it
    // (KNOWN_ISSUES #12). Written under the exclusive lock resolution already holds.
    c->second->recv_endpoint_seen = true;
    return c->second;
}

void TopicRegistry::setGapTracking(bool on) {
    m_gap_tracking.store(on, std::memory_order_relaxed);
}

auto TopicRegistry::gapTracking() const -> bool {
    return m_gap_tracking.load(std::memory_order_relaxed);
}

void TopicRegistry::onPublish(const void* pub_handle) { publishCount(pub_handle, false); }

void TopicRegistry::onIntraPublish(const void* pub_handle) { publishCount(pub_handle, true); }

void TopicRegistry::publishCount(const void* pub_handle, bool is_intra_process) {
    if (pub_handle == nullptr) {
        return;
    }
    const bool gap = m_gap_tracking.load(std::memory_order_relaxed);
    sTlsSlot& slot = tlsSlotFor(pub_handle);
    if (slot.id == m_id && slot.key == pub_handle) {
        auto& bucket = is_intra_process ? slot.ctr->pub_intra : slot.ctr->pub_inter;
        bucket.fetch_add(1, std::memory_order_relaxed);
        if (gap) {
            noteArrival(slot.ctr->pub_last_ns, slot.ctr->pub_max_dt_ns);
        }
        return;
    }
    std::shared_lock<std::shared_mutex> lock(m_mu);
    m_shared_lock_lookups.fetch_add(1, std::memory_order_relaxed);
    auto it = m_pub_to_counter.find(pub_handle);
    if (it != m_pub_to_counter.end()) {
        auto& bucket = is_intra_process ? it->second->pub_intra : it->second->pub_inter;
        bucket.fetch_add(1, std::memory_order_relaxed);
        if (gap) {
            noteArrival(it->second->pub_last_ns, it->second->pub_max_dt_ns);
        }
        slot = {m_id, pub_handle, it->second};
    }
}

void TopicRegistry::onCallbackStart(const void* callback, bool is_intra_process) {
    if (callback == nullptr) {
        return;
    }
    sTlsSlot& slot = tlsSlotFor(callback);
    if (slot.id == m_id && slot.key == callback) {
        // slot.ctr is either a resolved counter or the kNotASubscription sentinel. The sentinel
        // means "known timer/service callback", skip it without touching the lock or a counter.
        if (slot.ctr != kNotASubscription) {
            if (is_intra_process) {
                slot.ctr->recv_intra.fetch_add(1, std::memory_order_relaxed);
            } else {
                slot.ctr->recv_inter.fetch_add(1, std::memory_order_relaxed);
            }
            if (m_gap_tracking.load(std::memory_order_relaxed)) {
                noteArrival(slot.ctr->recv_last_ns, slot.ctr->recv_max_dt_ns);
            }
        }
        return;
    }
    // Steady state: the callback is already resolved+cached (as a real counter OR the sentinel),
    // so a SHARED (concurrent) lock suffices. Only the FIRST time we see a callback do we take the
    // exclusive lock to run the full resolution chain and insert into the cache. This keeps
    // multi-threaded executors from serializing every received message on a single writer lock.
    sTopicCounter* ctr = nullptr;
    {
        std::shared_lock<std::shared_mutex> lock(m_mu);
        m_shared_lock_lookups.fetch_add(1, std::memory_order_relaxed);
        auto it = m_cb_to_counter.find(callback);
        if (it != m_cb_to_counter.end()) {
            ctr = it->second;
        }
    }
    if (ctr == nullptr) {
        std::unique_lock<std::shared_mutex> lock(m_mu);
        m_write_lock_resolutions.fetch_add(1, std::memory_order_relaxed);
        ctr = resolveCallback(callback);
    }
    // A null result here means "subscription, not resolvable yet" (chain still being populated):
    // leave the thread-local slot untouched so the next delivery retries resolution. A non-null
    // result, a real counter or the sentinel, is decisive, so cache it thread-locally; that lets
    // even a not-a-subscription callback drop the shared lock on subsequent same-thread calls.
    if (ctr == nullptr) {
        return;
    }
    slot = {m_id, callback, ctr};
    if (ctr == kNotASubscription) {
        return;  // proven timer/service callback, nothing to count
    }
    if (is_intra_process) {
        ctr->recv_intra.fetch_add(1, std::memory_order_relaxed);
    } else {
        ctr->recv_inter.fetch_add(1, std::memory_order_relaxed);
    }
    if (m_gap_tracking.load(std::memory_order_relaxed)) {
        noteArrival(ctr->recv_last_ns, ctr->recv_max_dt_ns);
    }
}

auto TopicRegistry::snapshot(double window_s, bool fold_open_gap)
    -> std::vector<sTopicStat> {
    std::vector<sTopicStat> out;
    std::unique_lock<std::shared_mutex> lock(m_mu);
    const double w = window_s > 0.0 ? window_s : 1.0;
    const bool gap = m_gap_tracking.load(std::memory_order_relaxed);
    const auto now_ns = gap ? static_cast<uint64_t>(
                                  std::chrono::steady_clock::now()
                                      .time_since_epoch().count())
                            : 0ULL;
    // Counters that carried traffic this window, used below to age per-node liveness. Filtered
    // topics are excluded from the stats output but still count as node activity (a node emitting
    // only /rosout is alive), so we record activity before the filter check.
    std::unordered_set<const sTopicCounter*> active;
    for (auto& kv : m_by_topic) {
        sTopicCounter* c = kv.second.get();
        // Exchange all three split buckets first (this also resets them), so filtered topics still
        // count toward node activity even though they are excluded from the stats output.
        uint64_t p = c->pub_inter.exchange(0, std::memory_order_relaxed);
        uint64_t px = c->pub_intra.exchange(0, std::memory_order_relaxed);
        uint64_t ri = c->recv_inter.exchange(0, std::memory_order_relaxed);
        uint64_t rx = c->recv_intra.exchange(0, std::memory_order_relaxed);
        // Gap accumulators drain here too, for the same reason the counts do: a filtered topic
        // is not REPORTED, but its per-window state must still be reset or it ratchets upward
        // for the process lifetime and the first window after any filter change would report a
        // gap accumulated since startup.
        double pub_gap_ms = 0.0;
        double recv_gap_ms = 0.0;
        bool has_pub_gap = false;
        bool has_recv_gap = false;
        if (gap) {
            has_pub_gap = takeGap(c->pub_last_ns, c->pub_max_dt_ns, now_ns, fold_open_gap,
                                  pub_gap_ms);
            has_recv_gap = takeGap(c->recv_last_ns, c->recv_max_dt_ns, now_ns, fold_open_gap,
                                   recv_gap_ms);
        }
        if (p > 0 || px > 0 || ri > 0 || rx > 0) {
            active.insert(c);  // this topic's endpoints saw traffic -> owning node(s) are live
        }
        if (shouldFilter(c->topic)) {
            continue;  // counts and gaps reset above, but the topic itself is never reported
        }
        sTopicStat s;
        s.topic = c->topic;
        s.pub_inter_count = p;
        s.pub_intra_count = px;
        s.recv_inter_count = ri;
        s.recv_intra_count = rx;
        s.pub_inter_hz = static_cast<double>(p) / w;
        s.pub_intra_hz = static_cast<double>(px) / w;
        s.recv_inter_hz = static_cast<double>(ri) / w;
        s.recv_intra_hz = static_cast<double>(rx) / w;
        s.recv_endpoint_seen = c->recv_endpoint_seen;
        s.has_pub_max_dt = has_pub_gap;
        s.pub_max_dt_ms = pub_gap_ms;
        s.has_recv_max_dt = has_recv_gap;
        s.recv_max_dt_ms = recv_gap_ms;
        out.push_back(std::move(s));
    }
    // snapshot() is the once-per-window boundary, so it also ages node liveness: a node whose owned
    // counters all sat idle this window advances its quiet-window count; any traffic resets it.
    for (auto& node : m_nodes) {
        bool node_active = false;
        for (auto* c : node->counters) {
            if (active.count(c) != 0) {
                node_active = true;
                break;
            }
        }
        node->idle_windows = node_active ? 0u : (node->idle_windows + 1u);
    }
    return out;
}

auto TopicRegistry::activeNodes() const -> std::vector<std::string> {
    std::shared_lock<std::shared_mutex> lock(m_mu);
    std::vector<std::string> out;
    out.reserve(m_nodes.size());
    for (const auto& n : m_nodes) {
        if (n->idle_windows < m_quiet_windows) {
            out.push_back(n->name);
        }
    }
    return out;
}

auto TopicRegistry::knownNodes() const -> std::vector<std::string> {
    std::shared_lock<std::shared_mutex> lock(m_mu);
    std::vector<std::string> out;
    out.reserve(m_nodes.size());
    for (const auto& n : m_nodes) {
        out.push_back(n->name);
    }
    return out;
}

auto TopicRegistry::writeLockResolutions() const -> uint64_t {
    return m_write_lock_resolutions.load(std::memory_order_relaxed);
}

auto TopicRegistry::sharedLockLookups() const -> uint64_t {
    return m_shared_lock_lookups.load(std::memory_order_relaxed);
}

void TopicRegistry::forkPrepare() { m_mu.lock(); }

void TopicRegistry::forkRelease() { m_mu.unlock(); }

void TopicRegistry::forkChildReset() {
    // The write lock taken in forkPrepare() CANNOT be released in the fork child: glibc's
    // rwlock records the writer's TID, and the forking thread's TID differs in the child, so
    // pthread_rwlock_unlock is a silent no-op there, the registry would stay write-locked
    // forever and the child's first shared_lock would hang (exactly the futex wait the
    // issue-10 regression test caught). The child is single-threaded and is the lock's owner
    // by inheritance, so re-initialize the mutex in place instead of unlocking it.
    new (&m_mu) std::shared_mutex();
    // Gap state (R5): last_ns is CLOCK_MONOTONIC, which is per-boot and process-independent, so
    // an interval the child measures against it is real, keep it. The per-window maxima are the
    // PARENT's observations and would be attributed to the child's first window, where a stale
    // large value can fire a max_gap_ms rule on a perfectly healthy child. That is why this
    // diverges from the inherited-counts smear documented in issue #10: a stale count cannot
    // raise an alert, a stale gap can. The child is single-threaded here, so a plain loop is safe.
    for (auto& kv : m_by_topic) {
        kv.second->pub_max_dt_ns.store(0, std::memory_order_relaxed);
        kv.second->recv_max_dt_ns.store(0, std::memory_order_relaxed);
    }
}

}  // namespace ros2_pulse::core

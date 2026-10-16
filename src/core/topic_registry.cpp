// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "ros2_pulse/core/topic_registry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>  // for std::unique_lock with shared_mutex
#include <new>    // placement-new (forkChildReset)

namespace ros2_pulse::core {

namespace {
std::atomic<uint64_t> g_next_registry_id{1};

// Negative-cache sentinel (KNOWN_ISSUES #3). A callback proven NEVER to be a subscription, a timer
// or service callback, for which callback_added never fired, resolves to this marker instead of
// nullptr, so repeat sightings are served from the shared-lock / thread-local fast paths rather than
// re-taking the exclusive lock on every call. It is a unique, valid address that is never
// dereferenced: every read site compares it by identity and skips (see onCallbackStart).
sEndpointCounter g_not_a_subscription;
sEndpointCounter* const kNotASubscription = &g_not_a_subscription;

// Take-path sentinel: the rmw handle belongs to an rclcpp subscription, whose receives are
// counted at callback_start, so the take is deliberately ignored (no double count). Same
// identity-only contract as kNotASubscription, never dereferenced.
sEndpointCounter g_take_owned_by_rclcpp;
sEndpointCounter* const kTakeOwnedByRclcpp = &g_take_owned_by_rclcpp;
// A take-path verdict for an rclcpp subscription that DOES have an endpoint is that endpoint
// with its low bit set: the take still must not count the receive (callback_start does), but it
// is the only place the message age can be measured, so the endpoint has to be reachable from
// the same thread-local slot. Endpoints are heap objects aligned well past 2, so bit 0 is free.
static_assert(alignof(sEndpointCounter) >= 2, "take-path ownership tag needs a free low bit");
inline auto tagRclcppOwned(sEndpointCounter* c) -> sEndpointCounter* {
    return reinterpret_cast<sEndpointCounter*>(reinterpret_cast<uintptr_t>(c) | uintptr_t{1});
}
inline auto isRclcppOwnedTag(const sEndpointCounter* c) -> bool {
    return (reinterpret_cast<uintptr_t>(c) & uintptr_t{1}) != 0;
}
inline auto untagCounter(sEndpointCounter* c) -> sEndpointCounter* {
    return reinterpret_cast<sEndpointCounter*>(reinterpret_cast<uintptr_t>(c) & ~uintptr_t{1});
}
// Take-path sentinel: an rmw handle no rcl_subscription_init ever named (the rmw's internal
// discovery reader). Cached so repeat takes skip the lock; never dereferenced.
sEndpointCounter g_take_unknown_handle;
sEndpointCounter* const kTakeUnknownHandle = &g_take_unknown_handle;

// Direct-mapped thread-local hot-path cache, shared by every hot path. Keys of different kinds
// (publisher handle, rcl subscription handle, callback object, rmw handle) can be the same
// address over time, a destroyed object's memory reused for another kind, so the tag carries the
// kind (TopicRegistry::tlsTag) and a slot is only ever served to a lookup of its own kind. A
// single-entry cache thrashed on the REALISTIC pattern, one thread alternating
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
    sEndpointCounter* ctr{nullptr};
};
constexpr uintptr_t kTlsSlotMask = 255;
thread_local sTlsSlot t_tls_cache[kTlsSlotMask + 1];

inline auto tlsSlotFor(const void* key) -> sTlsSlot& {
    // Live-object addresses have zeroed alignment bits; >>4 drops them, and folding in >>10
    // spreads uniform allocation strides across the slot space.
    const auto p = reinterpret_cast<uintptr_t>(key);
    return t_tls_cache[((p >> 4) ^ (p >> 10)) & kTlsSlotMask];
}

// Lock-free max, the portable form of C++26 std::atomic::fetch_max.
inline void atomicMax(std::atomic<uint64_t>& m, uint64_t v) {
    uint64_t cur = m.load(std::memory_order_relaxed);
    while (v > cur && !m.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
    }
}

// Feed one duration sample into a window accumulator: three relaxed adds and a min CAS that,
// like the max, only loops when a new minimum lands (rare in steady state).
inline void addDuration(sDurationAcc& a, uint64_t v_ns) {
    uint64_t us = v_ns / 1000;
    if (us > sDurationAcc::kMaxSampleUs) {
        us = sDurationAcc::kMaxSampleUs;
    }
    a.n.fetch_add(1, std::memory_order_relaxed);
    a.sum_us.fetch_add(us, std::memory_order_relaxed);
    a.sumsq_us2.fetch_add(us * us, std::memory_order_relaxed);
    uint64_t cur = a.min_ns.load(std::memory_order_relaxed);
    while (v_ns < cur && !a.min_ns.compare_exchange_weak(cur, v_ns, std::memory_order_relaxed)) {
    }
}

// One window of one accumulator, drained: plain integers, so a topic's subscriptions pool by
// adding counts and sums and taking the min of the mins (the variance of the union follows from
// the pooled sums exactly, no per-endpoint deviation is ever averaged).
struct sDurationSums {
    uint64_t n{0};
    uint64_t sum_us{0};
    uint64_t sumsq_us2{0};
    uint64_t min_ns{UINT64_MAX};
    void add(const sDurationSums& o) {
        n += o.n;
        sum_us += o.sum_us;
        sumsq_us2 += o.sumsq_us2;
        min_ns = o.min_ns < min_ns ? o.min_ns : min_ns;
    }
};

// Drain a duration accumulator for the window just ending. The four fields are exchanged one by
// one, so a sample racing the drain can split across two windows, the same smear the counts
// accept; finishDuration() clamps the variance at zero against exactly that.
inline auto drainDuration(sDurationAcc& a) -> sDurationSums {
    sDurationSums d;
    d.n = a.n.exchange(0, std::memory_order_relaxed);
    d.sum_us = a.sum_us.exchange(0, std::memory_order_relaxed);
    d.sumsq_us2 = a.sumsq_us2.exchange(0, std::memory_order_relaxed);
    d.min_ns = a.min_ns.exchange(UINT64_MAX, std::memory_order_relaxed);
    return d;
}

// The window's statistics from (pooled) sums. False when there was no sample: no distribution
// exists, which must not be reported as a minimum or a deviation of zero.
inline auto finishDuration(const sDurationSums& d, sDurationStats& out) -> bool {
    if (d.n == 0 || d.min_ns == UINT64_MAX) {
        return false;
    }
    const long double mean_us = static_cast<long double>(d.sum_us) / static_cast<long double>(d.n);
    long double var_us2 =
        static_cast<long double>(d.sumsq_us2) / static_cast<long double>(d.n) - mean_us * mean_us;
    if (var_us2 < 0.0L) {
        var_us2 = 0.0L;
    }
    out.count = d.n;
    out.mean_ms = static_cast<double>(mean_us / 1000.0L);
    out.min_ms = static_cast<double>(d.min_ns) / 1e6;
    out.stddev_ms = static_cast<double>(std::sqrt(var_us2) / 1000.0L);
    return true;
}

// Floor division: the age is quantized to the microsecond BELOW it on both sides of zero, so a
// skewed (negative) age is not biased toward zero by truncation.
inline auto floorDivUs(int64_t ns) -> int64_t {
    int64_t q = ns / 1000;
    if (ns % 1000 < 0) {
        --q;
    }
    return q;
}

// Feed one (signed) age sample. The shift is the first sample this endpoint ever saw: one
// relaxed load per sample, a CAS only on that first one (a racing first sample reads the
// winner's). Overflow of the squares is detected from fetch_add's return, never wrapped into a
// plausible-looking deviation.
inline void addAge(sAgeAcc& a, int64_t v_ns) {
    const int64_t us = floorDivUs(v_ns);
    int64_t shift = a.shift_us.load(std::memory_order_relaxed);
    if (shift == sAgeAcc::kNoShift &&
        a.shift_us.compare_exchange_strong(shift, us, std::memory_order_relaxed)) {
        shift = us;
    }
    const int64_t d = us - shift;  // both within int64 ns / 1000, cannot overflow
    a.n.fetch_add(1, std::memory_order_relaxed);
    a.sum_us.fetch_add(d, std::memory_order_relaxed);
    const uint64_t ad = d < 0 ? static_cast<uint64_t>(-d) : static_cast<uint64_t>(d);
    if (ad >= (uint64_t{1} << 32)) {
        a.sumsq_overflow.store(true, std::memory_order_relaxed);  // ad * ad would wrap
    } else {
        const uint64_t sq = ad * ad;
        const uint64_t old = a.sumsq_us2.fetch_add(sq, std::memory_order_relaxed);
        if (old + sq < old) {
            a.sumsq_overflow.store(true, std::memory_order_relaxed);
        }
    }
    int64_t cur = a.min_ns.load(std::memory_order_relaxed);
    while (v_ns < cur && !a.min_ns.compare_exchange_weak(cur, v_ns, std::memory_order_relaxed)) {
    }
    cur = a.max_ns.load(std::memory_order_relaxed);
    while (v_ns > cur && !a.max_ns.compare_exchange_weak(cur, v_ns, std::memory_order_relaxed)) {
    }
}

// One window of age samples as moments (count, mean, sum of squared deviations from the mean),
// so endpoints with different shifts pool exactly (Chan et al.'s parallel update) without
// re-centering their integer sums.
struct sAgeMoments {
    uint64_t n{0};
    long double mean_us{0.0L};
    long double m2_us2{0.0L};
    bool overflow{false};
    int64_t min_ns{INT64_MAX};
    int64_t max_ns{INT64_MIN};
    void add(const sAgeMoments& o) {
        if (o.n == 0) {
            return;
        }
        const uint64_t total = n + o.n;
        const long double delta = o.mean_us - mean_us;
        const long double nb = static_cast<long double>(o.n);
        const long double na = static_cast<long double>(n);
        mean_us += delta * nb / static_cast<long double>(total);
        m2_us2 += o.m2_us2 + delta * delta * na * nb / static_cast<long double>(total);
        n = total;
        overflow = overflow || o.overflow;
        min_ns = o.min_ns < min_ns ? o.min_ns : min_ns;
        max_ns = o.max_ns > max_ns ? o.max_ns : max_ns;
    }
};

// Drain an age accumulator for the window just ending (same field-by-field smear as
// drainDuration; m2 is clamped at zero against it). The shift stays: it only centers the sums.
inline auto drainAge(sAgeAcc& a) -> sAgeMoments {
    const uint64_t n = a.n.exchange(0, std::memory_order_relaxed);
    const int64_t sum = a.sum_us.exchange(0, std::memory_order_relaxed);
    const uint64_t sumsq = a.sumsq_us2.exchange(0, std::memory_order_relaxed);
    const bool overflow = a.sumsq_overflow.exchange(false, std::memory_order_relaxed);
    const int64_t mn = a.min_ns.exchange(INT64_MAX, std::memory_order_relaxed);
    const int64_t mx = a.max_ns.exchange(INT64_MIN, std::memory_order_relaxed);
    sAgeMoments m;
    if (n == 0) {
        return m;  // no sample (any field a racing one left behind goes with it)
    }
    m.overflow = overflow;
    m.min_ns = mn;
    m.max_ns = mx;
    const long double ln = static_cast<long double>(n);
    const long double lsum = static_cast<long double>(sum);
    const int64_t shift = a.shift_us.load(std::memory_order_relaxed);
    m.n = n;
    m.mean_us = static_cast<long double>(shift) + lsum / ln;
    m.m2_us2 = static_cast<long double>(sumsq) - lsum * lsum / ln;
    if (m.m2_us2 < 0.0L) {
        m.m2_us2 = 0.0L;
    }
    return m;
}

inline auto finishAge(const sAgeMoments& m, sDurationStats& out) -> bool {
    if (m.n == 0 || m.min_ns == INT64_MAX) {
        return false;
    }
    out.count = m.n;
    out.min_ms = static_cast<double>(m.min_ns) / 1e6;
    out.max_ms = static_cast<double>(m.max_ns) / 1e6;
    // The mean comes from microsecond sums, min and max are exact nanoseconds: keep the sub-us
    // quantization from ever reporting a mean outside them (a constant age would read mean < min).
    out.mean_ms = std::min(std::max(static_cast<double>(m.mean_us / 1000.0L), out.min_ms),
                           out.max_ms);
    out.has_stddev = !m.overflow;
    out.stddev_ms = out.has_stddev
                        ? static_cast<double>(
                              std::sqrt(m.m2_us2 / static_cast<long double>(m.n)) / 1000.0L)
                        : 0.0;
    return true;
}

// Discard whatever an endpoint's distributions hold (re-home, fork child).
inline void resetDistributions(sEndpointCounter* ep) {
    drainDuration(ep->recv_dt);
    drainAge(ep->recv_age);
}

// Message age at rmw_take: local wall clock minus the DDS source timestamp, which is what
// rclcpp Jazzy+ computes (now - message_info.source_timestamp); Humble / Iron rclcpp used
// header.stamp instead. Both clocks are CLOCK_REALTIME-based nanoseconds since the epoch, on two
// machines when the publisher is remote, so clock skew lands in the value either way. A source
// timestamp AHEAD of the local clock gives a negative age, which is kept, as upstream keeps it:
// dropping it would bias the mean up and hide the skew. Both operands are positive int64, so the
// difference cannot overflow.
inline void noteAge(sEndpointCounter& c, int64_t source_ns) {
    const auto now = static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    addAge(c.recv_age, now - source_ns);
}

// ROADMAP R5: record one arrival at @p now into a gap accumulator.
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
//
// @p dist, when given (receive side), also gets the interval for the window's MINIMUM / STDDEV.
inline void noteArrivalAt(uint64_t now, std::atomic<uint64_t>& last_ns,
                          std::atomic<uint64_t>& max_dt_ns, sDurationAcc* dist = nullptr) {
    const uint64_t prev = last_ns.exchange(now, std::memory_order_relaxed);
    if (prev == 0 || now <= prev) {
        return;  // first arrival ever, or a reordered pair, no usable interval
    }
    const uint64_t dt = now - prev;
    // Costs nothing in steady state: new maxima follow the harmonic number, ~14 CAS per 500
    // messages.
    atomicMax(max_dt_ns, dt);
    if (dist != nullptr) {
        addDuration(*dist, dt);
    }
}

// One arrival on @p ep, gap tracking on: the handle's own accumulator and its topic's merged
// per-side accumulator share ONE clock read, so per-endpoint gaps cost an extra exchange, not
// an extra ~21 ns clock read. The receive-side distribution is fed from the handle's OWN
// interval only: the merged stream's intervals interleave subscriptions.
inline void noteArrival(sEndpointCounter* ep) {
    const auto now = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    noteArrivalAt(now, ep->last_ns, ep->max_dt_ns,
                  ep->side == eEndpointSide::kRecv ? &ep->recv_dt : nullptr);
    sTopicCounter* t = ep->topic.load(std::memory_order_relaxed);
    if (t == nullptr) {
        return;
    }
    if (ep->side == eEndpointSide::kPub) {
        noteArrivalAt(now, t->pub_last_ns, t->pub_max_dt_ns);
    } else {
        noteArrivalAt(now, t->recv_last_ns, t->recv_max_dt_ns);
    }
}

// The hot-path count itself: one relaxed add on the handle's own bucket, plus the opt-in gap.
inline void countArrival(sEndpointCounter* ep, bool is_intra_process, bool gap) {
    (is_intra_process ? ep->intra : ep->inter).fetch_add(1, std::memory_order_relaxed);
    if (gap) {
        noteArrival(ep);
    }
}

// Drain one gap accumulator for the window just ending. Returns false when the endpoint has
// never seen an arrival: no interval exists, which is NOT the same as a gap of zero, so the
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
        // be a maximum and is already dropped by noteArrivalAt's now > prev guard.
        return false;
    }
    out_ms = static_cast<double>(dt) / 1e6;
    return true;
}

template <typename T>
void eraseValue(std::vector<T*>& v, const T* value) {
    v.erase(std::remove(v.begin(), v.end(), value), v.end());
}

}  // namespace

TopicRegistry::TopicRegistry(uint32_t quiet_windows)
    : m_id(g_next_registry_id.fetch_add(1, std::memory_order_relaxed)),
      m_pub_tag(tlsTag(kTlsPub, 0)),
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

auto TopicRegistry::shouldEmitEndpoint(const sEndpointStat& stat, bool emit_idle) -> bool {
    // The topic-level gates, applied per handle: traffic always; a proven receive endpoint at
    // 0.0 (KNOWN_ISSUES #12); a tracked gap, so a publisher that stopped stays visible exactly as
    // its JITTER line does; and the declared-but-silent rest only on the issue #7 opt-in.
    if (stat.inter_count > 0 || stat.intra_count > 0 || stat.loaned_count > 0) {
        return true;
    }
    if (stat.side == eEndpointSide::kRecv && stat.endpoint_seen) {
        return true;
    }
    // A destroyed endpoint reaches here only for its final window, with neither flag set and
    // with traffic, so it is never kept alive by this gate (see snapshotWindow()).
    return stat.has_max_dt || emit_idle;
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

void TopicRegistry::unlinkEndpointNode(sEndpointCounter* ep) {
    if (ep->node != nullptr) {
        eraseValue(ep->node->endpoints, ep);
        ep->node = nullptr;
    }
    unindexNodeHandle(ep);
}

void TopicRegistry::unindexNodeHandle(sEndpointCounter* ep) {
    if (ep->node_handle != nullptr) {
        auto it = m_eps_by_nodehandle.find(ep->node_handle);
        if (it != m_eps_by_nodehandle.end()) {
            eraseValue(it->second, ep);
            if (it->second.empty()) {
                m_eps_by_nodehandle.erase(it);
            }
        }
        ep->node_handle = nullptr;
    }
}

auto TopicRegistry::mapFor(eEndpointSide side)
    -> std::unordered_map<const void*, sEndpointCounter*>& {
    return side == eEndpointSide::kPub ? m_pub_to_counter : m_subhandle_to_counter;
}

void TopicRegistry::retireEndpoint(sEndpointCounter* ep) {
    if (ep->retired) {
        return;
    }
    ep->retired = true;
    // Out of the node-handle index NOW, so a recycled node address cannot claim it. The node
    // link itself stays until the final drain, which still reports the last window's traffic
    // under the node that really owned it.
    unindexNodeHandle(ep);
}

auto TopicRegistry::bindEndpoint(const void* handle, const char* topic, const void* node_handle,
                                 eEndpointSide side) -> sEndpointCounter* {
    sTopicCounter* t = counterForTopic(topic);
    // The same address still mapped on the OTHER side is a dead object of the other kind whose
    // fini we did not see. Retire it so it stops being reported; the TLS tag carries the kind,
    // so no cached slot of that kind can be served to this one.
    auto& other = mapFor(side == eEndpointSide::kPub ? eEndpointSide::kRecv : eEndpointSide::kPub);
    auto o = other.find(handle);
    if (o != other.end()) {
        retireEndpoint(o->second);
    }
    auto& map = mapFor(side);
    sEndpointCounter* ep = nullptr;
    auto it = map.find(handle);
    if (it == map.end()) {
        auto owned = std::make_unique<sEndpointCounter>();
        owned->side = side;
        ep = owned.get();
        m_endpoints.push_back(std::move(owned));
        map.emplace(handle, ep);
    } else {
        // A recycled handle: the previous publisher / subscription at this address was destroyed
        // (no tracepoint says so) and a new one was initialized in its place. Re-home the SAME
        // endpoint object instead of allocating a second one: every thread-local cache slot and
        // resolved callback that still points at it now counts into the new identity, where a
        // fresh object would leave them feeding the dead one (or, before the per-endpoint split,
        // the wrong topic). Gap state restarts: an interval spanning two endpoints is not one.
        // Counts still pending from the previous life are reported under the new identity in
        // the next window; the hot path cannot tell them apart without a lock.
        ep = it->second;
        if (sTopicCounter* old = ep->topic.load(std::memory_order_relaxed)) {
            eraseValue(old->endpoints, ep);  // a no-op once a retired endpoint was dropped
        }
        unlinkEndpointNode(ep);
        ep->retired = false;
        ep->seen = false;
        ep->last_ns.store(0, std::memory_order_relaxed);
        ep->max_dt_ns.store(0, std::memory_order_relaxed);
        resetDistributions(ep);
    }
    ep->id = m_next_endpoint_id++;
    ep->topic.store(t, std::memory_order_relaxed);
    t->endpoints.push_back(ep);
    if (node_handle != nullptr) {
        ep->node_handle = node_handle;
        m_eps_by_nodehandle[node_handle].push_back(ep);
        auto n = m_nodehandle_to_node.find(node_handle);
        if (n != m_nodehandle_to_node.end()) {
            // Provisional when the node address was recycled: the new node's rcl_node_init
            // fires later and re-attributes this endpoint (see onNodeInit).
            ep->node = n->second;
            n->second->endpoints.push_back(ep);
        }
    }
    return ep;
}

void TopicRegistry::markSeen(sEndpointCounter* ep) {
    ep->seen = true;
    if (sTopicCounter* t = ep->topic.load(std::memory_order_relaxed)) {
        t->recv_endpoint_seen = true;
    }
}

void TopicRegistry::onPublisherInit(const void* pub_handle, const void* node_handle,
                                    const char* topic) {
    if (pub_handle == nullptr || topic == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    bindEndpoint(pub_handle, topic, node_handle, eEndpointSide::kPub);
}

void TopicRegistry::onSubscriptionInit(const void* sub_handle, const void* node_handle,
                                       const char* topic, const void* rmw_sub_handle) {
    if (sub_handle == nullptr || topic == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    bindEndpoint(sub_handle, topic, node_handle, eEndpointSide::kRecv);
    // A fresh rcl subscription: ownership is unknown until rclcpp_subscription_init (rclcpp
    // fires it right after rcl init, in the Subscription ctor, before any executor can take).
    // Resetting here also covers a recycled rcl handle that used to be rclcpp's.
    m_subhandle_rclcpp[sub_handle] = false;
    if (rmw_sub_handle != nullptr) {
        m_rmwsub_to_subhandle[rmw_sub_handle] = sub_handle;
    }
    m_sub_epoch.fetch_add(1, std::memory_order_relaxed);
}

void TopicRegistry::onRclcppSubscriptionInit(const void* subscription, const void* sub_handle) {
    if (subscription == nullptr || sub_handle == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    m_sub_to_subhandle[subscription] = sub_handle;
    m_subhandle_rclcpp[sub_handle] = true;
    m_sub_epoch.fetch_add(1, std::memory_order_relaxed);
}

void TopicRegistry::onCallbackAdded(const void* callback, const void* subscription) {
    if (callback == nullptr || subscription == nullptr) {
        return;
    }
    // Record the link only. Resolution to an endpoint is deferred to onCallbackStart, because for
    // intra-process subscriptions this fires before the intra waitable's full chain is populated.
    // A callback address seen before belongs to a destroyed subscription (or timer) whose memory
    // was reused: drop its resolved verdict and bump the epoch so every thread's cached slot for
    // it misses, and the first delivery re-resolves (and re-proves the endpoint, KNOWN_ISSUES
    // #12) instead of counting into the dead one.
    std::unique_lock<std::shared_mutex> lock(m_mu);
    m_cb_to_sub[callback] = subscription;
    m_cb_to_counter.erase(callback);
    m_sub_epoch.fetch_add(1, std::memory_order_relaxed);
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
        // Claim the endpoints rcl_node_init created before its own tracepoint (/rosout): live
        // and still unowned. Only live endpoints are indexed (retired ones leave at fini), and a
        // destroyed node's handle stops naming it at rcl_node_fini, so on a recycled node
        // address the new /rosout is unowned here while the old node's endpoints are not in the
        // index at all. One already owned by another node is never taken: without its fini
        // there is no telling it apart from the old node's own, and attribution must not lie.
        auto eps = m_eps_by_nodehandle.find(node_handle);
        if (eps != m_eps_by_nodehandle.end()) {
            for (auto* ep : eps->second) {
                if (ep->node != nullptr || ep->retired) {
                    continue;
                }
                ep->node = node;
                node->endpoints.push_back(ep);
            }
        }
    }
}

void TopicRegistry::onEndpointFini(const void* handle, eEndpointSide side) {
    if (handle == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    auto& map = mapFor(side);
    auto it = map.find(handle);
    if (it != map.end()) {
        retireEndpoint(it->second);
    }
}

void TopicRegistry::onPublisherFini(const void* pub_handle) {
    onEndpointFini(pub_handle, eEndpointSide::kPub);
}

void TopicRegistry::onSubscriptionFini(const void* sub_handle) {
    onEndpointFini(sub_handle, eEndpointSide::kRecv);
}

void TopicRegistry::onNodeFini(const void* node_handle) {
    if (node_handle == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    m_nodehandle_to_node.erase(node_handle);
    auto eps = m_eps_by_nodehandle.find(node_handle);
    if (eps == m_eps_by_nodehandle.end()) {
        return;
    }
    // Anything still bound to the handle is dead with it: rcl finalizes the node's /rosout
    // publisher inside rcl_node_fini (a call within librcl that need not go through the PLT, so
    // its own fini wrapper may not see it), and rcl endpoints cannot outlive their node.
    const std::vector<sEndpointCounter*> bound = std::move(eps->second);
    m_eps_by_nodehandle.erase(eps);
    for (auto* ep : bound) {
        ep->node_handle = nullptr;  // already out of the index
        retireEndpoint(ep);
    }
}

auto TopicRegistry::resolveCallback(const void* callback) -> sEndpointCounter* {
    auto cached = m_cb_to_counter.find(callback);
    if (cached != m_cb_to_counter.end()) {
        return cached->second;  // a resolved endpoint, or the kNotASubscription sentinel
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
    // First delivered message for this subscription: it is now a PROVEN receive endpoint, so
    // idle windows keep emitting an explicit RECV 0.0 line for it (KNOWN_ISSUES #12). Written
    // under the exclusive lock resolution already holds.
    markSeen(c->second);
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
    if (slot.id == m_pub_tag && slot.key == pub_handle) {
        countArrival(slot.ctr, is_intra_process, gap);
        return;
    }
    std::shared_lock<std::shared_mutex> lock(m_mu);
    m_shared_lock_lookups.fetch_add(1, std::memory_order_relaxed);
    auto it = m_pub_to_counter.find(pub_handle);
    if (it != m_pub_to_counter.end()) {
        countArrival(it->second, is_intra_process, gap);
        slot = {m_pub_tag, pub_handle, it->second};
    }
}

auto TopicRegistry::lookupHandle(const void* handle, eEndpointSide side) -> sEndpointCounter* {
    // Same tag as publishCount for publisher handles (one map, so one cache), a different kind
    // for rcl subscription handles. Neither needs an epoch: both maps re-home in place.
    const uint64_t tag = side == eEndpointSide::kPub ? m_pub_tag : tlsTag(kTlsSubHandle, 0);
    sTlsSlot& slot = tlsSlotFor(handle);
    if (slot.id == tag && slot.key == handle) {
        return slot.ctr;
    }
    std::shared_lock<std::shared_mutex> lock(m_mu);
    m_shared_lock_lookups.fetch_add(1, std::memory_order_relaxed);
    const auto& map = side == eEndpointSide::kPub ? m_pub_to_counter : m_subhandle_to_counter;
    auto it = map.find(handle);
    if (it == map.end()) {
        return nullptr;
    }
    slot = {tag, handle, it->second};
    return it->second;
}

void TopicRegistry::onLoanedPublish(const void* pub_handle, bool counted_by_tracepoint) {
    if (pub_handle == nullptr) {
        return;
    }
    sEndpointCounter* c = lookupHandle(pub_handle, eEndpointSide::kPub);
    if (c == nullptr) {
        return;
    }
    c->loaned.fetch_add(1, std::memory_order_relaxed);
    if (!counted_by_tracepoint) {
        // Humble: no tracepoint fired inside rcl_publish_loaned_message, so this IS the only
        // signal for the publish. Count it in the total (and the gap accumulator) here, exactly
        // as onPublish would have.
        countArrival(c, /*is_intra_process=*/false, m_gap_tracking.load(std::memory_order_relaxed));
    }
}

void TopicRegistry::onLoanedTake(const void* sub_handle) {
    if (sub_handle == nullptr) {
        return;
    }
    sEndpointCounter* c = lookupHandle(sub_handle, eEndpointSide::kRecv);
    if (c != nullptr) {
        c->loaned.fetch_add(1, std::memory_order_relaxed);
    }
}

void TopicRegistry::onCallbackStart(const void* callback, bool is_intra_process) {
    if (callback == nullptr) {
        return;
    }
    // Epoch in the tag: a callback_added (re-registered callback address) or subscription init
    // since this slot was filled makes it miss, see onCallbackAdded. Tag read BEFORE the lookup,
    // like onTake, so a racing graph event leaves what we cache below already stale.
    const uint64_t tag = epochTag(kTlsCallback);
    sTlsSlot& slot = tlsSlotFor(callback);
    if (slot.id == tag && slot.key == callback) {
        // slot.ctr is either a resolved endpoint or the kNotASubscription sentinel. The sentinel
        // means "known timer/service callback", skip it without touching the lock or a counter.
        if (slot.ctr != kNotASubscription) {
            countArrival(slot.ctr, is_intra_process,
                         m_gap_tracking.load(std::memory_order_relaxed));
        }
        return;
    }
    // Steady state: the callback is already resolved+cached (as a real endpoint OR the sentinel),
    // so a SHARED (concurrent) lock suffices. Only the FIRST time we see a callback do we take the
    // exclusive lock to run the full resolution chain and insert into the cache. This keeps
    // multi-threaded executors from serializing every received message on a single writer lock.
    sEndpointCounter* ctr = nullptr;
    bool prove_endpoint = false;
    {
        std::shared_lock<std::shared_mutex> lock(m_mu);
        m_shared_lock_lookups.fetch_add(1, std::memory_order_relaxed);
        auto it = m_cb_to_counter.find(callback);
        if (it != m_cb_to_counter.end()) {
            ctr = it->second;
            // Cached, but its rcl handle was recycled since (bindEndpoint re-homed the endpoint
            // and reset `seen`): this delivery re-proves it, as onTake does.
            prove_endpoint = ctr != kNotASubscription && !ctr->seen;
        }
    }
    if (ctr == nullptr) {
        std::unique_lock<std::shared_mutex> lock(m_mu);
        m_write_lock_resolutions.fetch_add(1, std::memory_order_relaxed);
        ctr = resolveCallback(callback);
    } else if (prove_endpoint) {
        std::unique_lock<std::shared_mutex> lock(m_mu);
        markSeen(ctr);
    }
    // A null result here means "subscription, not resolvable yet" (chain still being populated):
    // leave the thread-local slot untouched so the next delivery retries resolution. A non-null
    // result, a real endpoint or the sentinel, is decisive, so cache it thread-locally; that lets
    // even a not-a-subscription callback drop the shared lock on subsequent same-thread calls.
    if (ctr == nullptr) {
        return;
    }
    slot = {tag, callback, ctr};
    if (ctr == kNotASubscription) {
        return;  // proven timer/service callback, nothing to count
    }
    countArrival(ctr, is_intra_process, m_gap_tracking.load(std::memory_order_relaxed));
}

auto TopicRegistry::resolveTake(const void* rmw_sub_handle) const -> sEndpointCounter* {
    auto r = m_rmwsub_to_subhandle.find(rmw_sub_handle);
    if (r == m_rmwsub_to_subhandle.end()) {
        return nullptr;  // never initialized through rcl, nothing to attribute it to
    }
    auto c = m_subhandle_to_counter.find(r->second);
    sEndpointCounter* ctr = c == m_subhandle_to_counter.end() ? nullptr : c->second;
    auto own = m_subhandle_rclcpp.find(r->second);
    if (own != m_subhandle_rclcpp.end() && own->second) {
        // Not counted here, but the endpoint stays reachable (tagged) for the message age.
        return ctr == nullptr ? kTakeOwnedByRclcpp : tagRclcppOwned(ctr);
    }
    return ctr;
}

void TopicRegistry::onTake(const void* rmw_sub_handle, bool taken,
                           int64_t source_timestamp_ns) {
    // rmw_take fires for every take ATTEMPT (both Humble DDS rmws emit it before the caller
    // looks at the result); only a taken message is a receive. Cheapest possible reject.
    if (!taken || rmw_sub_handle == nullptr) {
        return;
    }
    // Tag read BEFORE the lookup: an init racing with this take bumps the epoch, so whatever we
    // cache below carries the old tag and is re-resolved on the next take (conservative).
    const uint64_t tag = epochTag(kTlsRmw);
    sTlsSlot& slot = tlsSlotFor(rmw_sub_handle);
    sEndpointCounter* ctr = nullptr;
    if (slot.id == tag && slot.key == rmw_sub_handle) {
        ctr = slot.ctr;
    } else {
        bool prove_endpoint = false;
        {
            std::shared_lock<std::shared_mutex> lock(m_mu);
            m_shared_lock_lookups.fetch_add(1, std::memory_order_relaxed);
            ctr = resolveTake(rmw_sub_handle);
            prove_endpoint = ctr != nullptr && ctr != kTakeOwnedByRclcpp &&
                             !isRclcppOwnedTag(ctr) && !ctr->seen;
        }
        if (ctr == nullptr) {
            // Unknown handle: the rmw's own internal readers (ros_discovery_info) take through
            // the same tracepoint but never pass rcl_subscription_init. Cache the miss so that
            // steady discovery traffic stays off the lock too; the epoch in the tag re-resolves
            // it if this address is later initialized as a real subscription.
            ctr = kTakeUnknownHandle;
        }
        if (prove_endpoint) {
            // First delivered take for this subscription: mark it a proven receive endpoint so a
            // later stall reads RECV 0.0 (KNOWN_ISSUES #12). The flags are plain bools written
            // only under the exclusive lock; once per subscription, so this escalation is cold.
            std::unique_lock<std::shared_mutex> lock(m_mu);
            markSeen(ctr);
        }
        slot = {tag, rmw_sub_handle, ctr};
    }
    if (ctr == kTakeOwnedByRclcpp || ctr == kTakeUnknownHandle) {
        return;  // rclcpp subscription without an endpoint, or rmw-internal reader
    }
    const bool gap = m_gap_tracking.load(std::memory_order_relaxed);
    if (isRclcppOwnedTag(ctr)) {
        // rclcpp subscription: callback_start counts the receive (and its interval), the take
        // only contributes the age. Gap tracking off keeps this a cache hit and a return.
        if (gap && source_timestamp_ns > 0) {
            noteAge(*untagCounter(ctr), source_timestamp_ns);
        }
        return;
    }
    countArrival(ctr, /*is_intra_process=*/false, gap);
    if (gap && source_timestamp_ns > 0) {
        noteAge(*ctr, source_timestamp_ns);
    }
}

auto TopicRegistry::snapshot(double window_s, bool fold_open_gap)
    -> std::vector<sTopicStat> {
    return snapshotWindow(window_s, fold_open_gap).topics;
}

auto TopicRegistry::snapshotWindow(double window_s, bool fold_open_gap) -> sWindowSnapshot {
    sWindowSnapshot out;
    std::unique_lock<std::shared_mutex> lock(m_mu);
    const double w = window_s > 0.0 ? window_s : 1.0;
    const bool gap = m_gap_tracking.load(std::memory_order_relaxed);
    const auto now_ns = gap ? static_cast<uint64_t>(
                                  std::chrono::steady_clock::now()
                                      .time_since_epoch().count())
                            : 0ULL;
    out.endpoints.reserve(m_endpoints.size());
    // Destroyed endpoints drained for the last time this window, dropped after the liveness pass.
    std::vector<sEndpointCounter*> retired;
    for (auto& kv : m_by_topic) {
        sTopicCounter* t = kv.second.get();
        // Filtered topics are excluded from the stats output but still drained (so nothing
        // ratchets) and still count as node activity: a node emitting only /rosout is alive.
        const bool filtered = shouldFilter(t->topic);
        sTopicStat s;
        s.topic = t->topic;
        // The per-topic numbers are SUMS of the per-endpoint ones, computed here at the window
        // boundary: the hot path only ever touches the handle's own counter. The distributions
        // pool the same way, from integer sums, and drain unconditionally (cheap, and a stale one
        // must never leak into the first window after gap tracking or a filter changes).
        sDurationSums recv_dt_sums;
        sAgeMoments recv_age_moments;
        for (auto* ep : t->endpoints) {
            const sDurationSums ep_dt = drainDuration(ep->recv_dt);
            const sAgeMoments ep_age = drainAge(ep->recv_age);
            recv_dt_sums.add(ep_dt);
            recv_age_moments.add(ep_age);
            const uint64_t in = ep->inter.exchange(0, std::memory_order_relaxed);
            const uint64_t ix = ep->intra.exchange(0, std::memory_order_relaxed);
            const uint64_t lo = ep->loaned.exchange(0, std::memory_order_relaxed);
            // Gap accumulators drain here too, for the same reason the counts do: a filtered
            // topic is not REPORTED, but its per-window state must still be reset or it ratchets
            // upward for the process lifetime and the first window after any filter change would
            // report a gap accumulated since startup.
            double dt_ms = 0.0;
            const bool has_dt =
                gap && takeGap(ep->last_ns, ep->max_dt_ns, now_ns, fold_open_gap, dt_ms);
            ep->active_this_window = in > 0 || ix > 0 || lo > 0;
            if (ep->side == eEndpointSide::kPub) {
                s.pub_inter_count += in;
                s.pub_intra_count += ix;
                s.pub_loaned_count += lo;
            } else {
                s.recv_inter_count += in;
                s.recv_intra_count += ix;
                s.recv_loaned_count += lo;
            }
            if (ep->retired) {
                retired.push_back(ep);
            }
            if (filtered) {
                continue;
            }
            // A destroyed endpoint is reported only for traffic it carried before its fini, so
            // the topic sum still equals its endpoints; never as a proven-idle 0.0 or by its
            // ever-growing open gap, which would read as a stall of a live node.
            if (ep->retired && !ep->active_this_window) {
                continue;
            }
            sEndpointStat e;
            e.topic = t->topic;
            e.node = ep->node != nullptr ? ep->node->name : std::string();
            e.side = ep->side;
            e.id = ep->id;
            e.inter_count = in;
            e.intra_count = ix;
            e.loaned_count = lo;
            e.inter_hz = static_cast<double>(in) / w;
            e.intra_hz = static_cast<double>(ix) / w;
            e.loaned_hz = static_cast<double>(lo) / w;
            e.endpoint_seen = ep->seen && !ep->retired;
            e.has_max_dt = has_dt && !ep->retired;
            e.max_dt_ms = e.has_max_dt ? dt_ms : 0.0;
            // Samples taken before a fini are real ones, unlike its open gap: kept.
            e.has_recv_dt = finishDuration(ep_dt, e.recv_dt);
            e.has_recv_age = finishAge(ep_age, e.recv_age);
            out.endpoints.push_back(std::move(e));
        }
        double pub_gap_ms = 0.0;
        double recv_gap_ms = 0.0;
        bool has_pub_gap = false;
        bool has_recv_gap = false;
        if (gap) {
            has_pub_gap = takeGap(t->pub_last_ns, t->pub_max_dt_ns, now_ns, fold_open_gap,
                                  pub_gap_ms);
            has_recv_gap = takeGap(t->recv_last_ns, t->recv_max_dt_ns, now_ns, fold_open_gap,
                                   recv_gap_ms);
        }
        sDurationStats recv_dt;
        sDurationStats recv_age;
        const bool has_recv_dt = finishDuration(recv_dt_sums, recv_dt);
        const bool has_recv_age = finishAge(recv_age_moments, recv_age);
        if (filtered) {
            continue;  // counts and gaps reset above, but the topic itself is never reported
        }
        s.pub_inter_hz = static_cast<double>(s.pub_inter_count) / w;
        s.pub_intra_hz = static_cast<double>(s.pub_intra_count) / w;
        s.recv_inter_hz = static_cast<double>(s.recv_inter_count) / w;
        s.recv_intra_hz = static_cast<double>(s.recv_intra_count) / w;
        s.pub_loaned_hz = static_cast<double>(s.pub_loaned_count) / w;
        s.recv_loaned_hz = static_cast<double>(s.recv_loaned_count) / w;
        s.recv_endpoint_seen = t->recv_endpoint_seen;
        s.has_pub_max_dt = has_pub_gap;
        s.pub_max_dt_ms = pub_gap_ms;
        s.has_recv_max_dt = has_recv_gap;
        s.recv_max_dt_ms = recv_gap_ms;
        s.has_recv_dt = has_recv_dt;
        s.recv_dt = recv_dt;
        s.has_recv_age = has_recv_age;
        s.recv_age = recv_age;
        out.topics.push_back(std::move(s));
    }
    // snapshot() is the once-per-window boundary, so it also ages node liveness: a node whose OWN
    // endpoints all sat idle this window advances its quiet-window count; any traffic resets it.
    // Per-endpoint, not per-topic: every rclcpp node has a /rosout and a /parameter_events
    // endpoint, and a shared per-topic counter let one node's log line keep every node in the
    // process alive (PR #68 review).
    for (auto& node : m_nodes) {
        bool node_active = false;
        for (auto* ep : node->endpoints) {
            if (ep->active_this_window) {
                node_active = true;
                break;
            }
        }
        node->idle_windows = node_active ? 0u : (node->idle_windows + 1u);
    }
    // Final drain done: a destroyed endpoint leaves its topic and node for good. It stays in its
    // handle map, so a recycled address re-homes (and un-retires) this same object.
    for (auto* ep : retired) {
        if (sTopicCounter* t = ep->topic.load(std::memory_order_relaxed)) {
            eraseValue(t->endpoints, ep);
        }
        unlinkEndpointNode(ep);
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
    for (auto& ep : m_endpoints) {
        ep->max_dt_ns.store(0, std::memory_order_relaxed);
        resetDistributions(ep.get());
    }
}

}  // namespace ros2_pulse::core

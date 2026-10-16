// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ROS2_PULSE__CORE__TOPIC_REGISTRY_HPP_
#define ROS2_PULSE__CORE__TOPIC_REGISTRY_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_pulse::core {

struct sNode;
struct sTopicCounter;

/// Which side of a topic an endpoint sits on: a publisher handle or a subscription handle.
enum class eEndpointSide : uint8_t { kPub, kRecv };

/// @brief One publisher or subscription handle's running message counts (the hot-path target).
///
/// Every rcl publisher handle and every rcl subscription handle gets its own counter, so two
/// subscriptions on one topic in one process (a component container, or one node subscribing
/// twice) never share an increment or an inter-arrival stream (PR #68 / #70 reviews). The
/// per-topic numbers the TOPIC / RECV lines carry are summed from these at flush time, never on
/// the hot path, which stays one relaxed fetch_add on the handle's own counter.
///
/// Separate publish and receive endpoints also keep a same-process publisher and subscriber of
/// one topic from ever fetch_add'ing the same field (KNOWN_ISSUES.md #1).
struct sEndpointCounter {
    // Owning topic. Atomic only because a recycled handle re-homes its endpoint (under the
    // registry's write lock) while a hot-path gap update may be reading it; topics are never
    // freed, so a racing reader at worst updates the previous topic's merged gap once.
    std::atomic<sTopicCounter*> topic{nullptr};
    eEndpointSide side{eEndpointSide::kPub};  // fixed at creation
    // The fields below up to the atomics are written and read only under the registry's lock.
    uint32_t id{0};                    // process-unique, a fresh one on every (re)init
    const void* node_handle{nullptr};  // rcl node handle the init tracepoint carried
    sNode* node{nullptr};              // resolved owner; nullptr until its rcl_node_init is seen
    // Receive side: a message has been delivered here at least once, the per-endpoint twin of
    // sTopicCounter::recv_endpoint_seen (KNOWN_ISSUES #12).
    bool seen{false};
    bool active_this_window{false};  // scratch for snapshot()'s node-liveness pass

    // pub:  rcl_publish | rclcpp_intra_publish (iron+) | loaned subset of inter (rcl wrapper).
    // recv: callback_start(intra=false) for rclcpp subscriptions, a successful rmw_take for
    //       every other rcl client (rclpy, rclc, ...), never both for one subscription |
    //       callback_start(intra=true) | loaned subset of the total (rcl_take_loaned_message).
    std::atomic<uint64_t> inter{0};
    std::atomic<uint64_t> intra{0};
    std::atomic<uint64_t> loaned{0};

    // This handle's own gap accumulator (ROADMAP R5), same contract as sTopicCounter's pairs.
    std::atomic<uint64_t> last_ns{0};
    std::atomic<uint64_t> max_dt_ns{0};  // reset per window by snapshot()
};

/// @brief One topic's process-wide record: its endpoints plus the merged-stream gap accumulators.
struct sTopicCounter {
    std::string topic;
    // Every endpoint (either side, any node) currently bound to this topic. Write-lock only.
    std::vector<sEndpointCounter*> endpoints;
    // True once a subscription for this topic has delivered at least once. Sticky, written only
    // under the registry's write lock; lets a later zero-traffic window emit an explicit RECV 0.0
    // line instead of dropping the topic (KNOWN_ISSUES #12).
    bool recv_endpoint_seen{false};

    // --- gap tracking (ROADMAP R5), only written when the registry has it enabled ---
    // Transport-merged, one accumulator per SIDE over every endpoint of the topic in this
    // process, so the topic-level JITTER line keeps measuring the gap between ANY two messages
    // on the topic exactly as it did before the per-endpoint split (a second, slower publisher
    // on a topic must not read as a stall). The per-handle view lives on sEndpointCounter.
    // A windowed mean cannot see a stall (a 400 ms freeze on a 50 Hz topic still averages
    // 46 Hz over 5 s), so the max inter-arrival gap is the window-length-independent detector.
    // Nanoseconds, integer: lock-free everywhere and exact.
    //
    // last_* is the timestamp of the previous arrival and is deliberately NOT reset by snapshot():
    // the first message of window N+1 must measure its gap back into window N, or a stall that
    // straddles a flush would be silently discarded, precisely the case this exists to catch.
    // 0 means "no arrival ever", which is how a never-active endpoint stays unreported.
    std::atomic<uint64_t> pub_last_ns{0};
    std::atomic<uint64_t> recv_last_ns{0};
    std::atomic<uint64_t> pub_max_dt_ns{0};   // reset per window by snapshot()
    std::atomic<uint64_t> recv_max_dt_ns{0};  // reset per window by snapshot()
};

/// @brief Aggregated, windowed view of one topic (returned by snapshot()).
struct sTopicStat {
    std::string topic;
    uint64_t pub_inter_count{0};
    uint64_t pub_intra_count{0};
    uint64_t recv_inter_count{0};
    uint64_t recv_intra_count{0};
    double pub_inter_hz{0.0};
    double pub_intra_hz{0.0};
    double recv_inter_hz{0.0};
    double recv_intra_hz{0.0};
    // Loaned subsets of the publish / receive totals (zero when no loan happened, and then
    // absent from both output formats).
    uint64_t pub_loaned_count{0};
    uint64_t recv_loaned_count{0};
    double pub_loaned_hz{0.0};
    double recv_loaned_hz{0.0};
    // A subscription for this topic has delivered at least once, emit RECV even at zero so a
    // dead upstream reads 0.0 instead of vanishing (KNOWN_ISSUES #12).
    bool recv_endpoint_seen{false};
    // Largest inter-arrival gap seen this window, per side (ROADMAP R5). Valid only when the
    // matching has_* flag is set: gap tracking is opt-in, and an endpoint that has never seen a
    // message has no gap to report (which is NOT the same as a gap of zero).
    double pub_max_dt_ms{0.0};
    double recv_max_dt_ms{0.0};
    bool has_pub_max_dt{false};
    bool has_recv_max_dt{false};
};

/// @brief Windowed view of ONE publisher or subscription handle (returned by snapshotWindow()).
///
/// The per-node, per-endpoint breakdown behind the sTopicStat sums: a topic's counts are exactly
/// the sum of these over its endpoints. @c node is the owning node's fully qualified name
/// ("/ns/name"), empty when that node's rcl_node_init was never seen.
struct sEndpointStat {
    std::string topic;
    std::string node;
    eEndpointSide side{eEndpointSide::kPub};
    uint32_t id{0};
    uint64_t inter_count{0};
    uint64_t intra_count{0};
    uint64_t loaned_count{0};
    double inter_hz{0.0};
    double intra_hz{0.0};
    double loaned_hz{0.0};
    // Receive side: delivered at least once, so a stalled proven endpoint keeps reporting 0.0.
    bool endpoint_seen{false};
    // This handle's own largest inter-arrival gap this window (R5), valid only when has_max_dt.
    double max_dt_ms{0.0};
    bool has_max_dt{false};
};

/// @brief Everything one flush window measured: per-topic sums plus the per-endpoint split.
struct sWindowSnapshot {
    std::vector<sTopicStat> topics;
    std::vector<sEndpointStat> endpoints;
};

/// @brief One initialized node plus its recent-activity bookkeeping.
///
/// Liveness is inferred from topic traffic (there is no node-teardown tracepoint on Humble, see
/// KNOWN_ISSUES.md #2). @c endpoints are the publisher / subscription handles this node owns;
/// @c idle_windows counts consecutive windows in which none of them saw traffic. A node is
/// reported active while @c idle_windows is below the registry's K. Only the node's OWN handles
/// count: another node's /rosout publish in the same process says nothing about this one.
struct sNode {
    std::string name;
    uint32_t idle_windows{0};
    std::vector<sEndpointCounter*> endpoints;
};

/// @brief Pure C++ core of the probe. Holds the ROS-graph handle→endpoint maps and per-endpoint
/// atomic counters, resolves callback→endpoint lazily, attributes every endpoint to its owning
/// node, and aggregates windowed frequencies per endpoint and per topic.
///
/// NO ROS / tracetools dependency, testable in isolation. The probe layer feeds it raw handles
/// captured from the interposed tracetools functions.
///
/// Thread-safety: graph-init methods (rare) take a write lock; hot-path methods (onPublish,
/// onCallbackStart) take a read lock and increment a relaxed atomic, with a thread-local cache
/// that elides the lock for the common repeated-endpoint case.
class TopicRegistry {
public:
    /// Consecutive quiet (zero-traffic) windows after which a node is dropped from activeNodes().
    static constexpr uint32_t kDefaultQuietWindows = 3;

    explicit TopicRegistry(uint32_t quiet_windows = kDefaultQuietWindows);
    TopicRegistry(TopicRegistry const&) = delete;
    auto operator=(TopicRegistry const&) -> TopicRegistry& = delete;

    // --- graph init (low frequency) ---
    void onPublisherInit(const void* pub_handle, const void* node_handle, const char* topic);
    /// @p rmw_sub_handle links the rcl subscription to the rmw handle rmw_take reports, which is
    /// how a take-layer receive is attributed to a topic. nullptr keeps the subscription
    /// invisible to onTake (the callback path is unaffected).
    void onSubscriptionInit(const void* sub_handle, const void* node_handle, const char* topic,
                            const void* rmw_sub_handle = nullptr);
    /// Also marks @p sub_handle as rclcpp-owned: its receives are counted at callback_start
    /// (which sees intra-process, serialized and loaned deliveries too), so onTake skips it.
    void onRclcppSubscriptionInit(const void* subscription, const void* sub_handle);
    void onCallbackAdded(const void* callback, const void* subscription);
    /// Also attributes every endpoint initialized with @p node_handle to this node, including
    /// ones that arrived first (rcl creates the node's /rosout publisher inside rcl_node_init,
    /// before this tracepoint fires).
    void onNodeInit(const void* node_handle, const char* node_name, const char* node_namespace);

    // --- hot path ---
    void onPublish(const void* pub_handle);       // inter-process publish (rcl_publish)
    void onIntraPublish(const void* pub_handle);  // intra-process publish (iron+ tracepoint)
    void onCallbackStart(const void* callback, bool is_intra_process);  // rclcpp receive
    /// Take-layer receive (rmw_take tracepoint): counts a successful take (@p taken) into
    /// recv_inter for every subscription NOT owned by rclcpp. This is what makes rclpy and any
    /// other rcl-based client visible; rclcpp subscriptions stay on callback_start so a message
    /// is never counted twice. Failed takes (taken=false) return before any lookup.
    void onTake(const void* rmw_sub_handle, bool taken);

    /// A successful rcl_publish_loaned_message (RCL_RET_OK) on @p pub_handle, the rcl publisher
    /// handle rcl_publisher_init carried. Counts the loaned subset, and ALSO the publish total
    /// unless @p counted_by_tracepoint: on Jazzy+ the plain rcl_publish tracepoint fires inside
    /// the loaned call and onPublish already counted it, on Humble nothing did.
    void onLoanedPublish(const void* pub_handle, bool counted_by_tracepoint);
    /// A successful rcl_take_loaned_message (RCL_RET_OK, non-null message) on the rcl
    /// subscription handle rcl_subscription_init carried. Counts the loaned subset only; the
    /// receive total stays the callback_start count, which the loaned callback also fires.
    void onLoanedTake(const void* sub_handle);

    /// Enable per-endpoint inter-arrival gap tracking (ROADMAP R5). Off by default: it costs one
    /// clock read per message (~21 ns, vs ~0.3 ns for the counting path alone), so it is opt-in
    /// via ROS_TOPIC_STATS_JITTER=1 or implied by a max_gap_ms spec rule. Set once at startup,
    /// before any tracepoint can fire; atomic so the read in the hot path stays race-free.
    void setGapTracking(bool on);
    auto gapTracking() const -> bool;

    // --- aggregation ---
    /// Compute Hz over the given window and RESET all counts. Filtered topics excluded. Returns
    /// the per-topic sums only; snapshotWindow() also returns the per-endpoint split.
    ///
    /// @p fold_open_gap closes the currently-open interval into each side's reported gap, i.e.
    /// reports max(observed gaps, now - last arrival). Without it an endpoint that has gone
    /// completely silent produces no inter-arrival pair at all and would vanish from the gap
    /// report, missing the TOTAL stall, the worst case R5 exists to catch. Pass false for the
    /// atexit window: rclcpp teardown stops traffic before the process exits, so the open gap
    /// there measures the shutdown sequence and would fire every gap rule on a healthy stop.
    auto snapshot(double window_s, bool fold_open_gap = true) -> std::vector<sTopicStat>;
    /// snapshot() plus one sEndpointStat per publisher / subscription handle of every reported
    /// topic, in topic order then creation order. One drain: the topic numbers are summed from
    /// exactly the endpoint numbers returned alongside them.
    auto snapshotWindow(double window_s, bool fold_open_gap = true) -> sWindowSnapshot;
    auto activeNodes() const -> std::vector<std::string>;
    /// Every node ever initialized in this process, active or quiet. Lets the expected-rate
    /// evaluator (ROADMAP R1) tell "aged out, warn" from "never ours, skip".
    auto knownNodes() const -> std::vector<std::string>;

    /// System/util topics excluded from output.
    static auto shouldFilter(const std::string& topic) -> bool;

    /// Decide whether the publish-side `TOPIC` line should be written for this window's stat.
    /// A fully-idle topic (no publish AND no receive traffic in any bucket) is emitted only when
    /// @p emit_idle is true (operator opt-in via ROS_PULSE_EMIT_IDLE=1). Otherwise TOPIC is the
    /// publish-side line: emitted only when this process published (pub_inter_count > 0); a
    /// receive-only topic's signal is carried on the additive RECV line instead.
    static auto shouldEmitTopic(const sTopicStat& stat, bool emit_idle) -> bool;

    /// Decide whether the receive-side `RECV` line should be written for this window's stat:
    /// when the window saw receive traffic, or when the topic is a proven receive endpoint
    /// (delivered at least once), so a stalled upstream reads an explicit 0.0 every window
    /// instead of vanishing (KNOWN_ISSUES #12). Never-active subscriptions stay suppressed
    /// (issue #7 rationale: declared-but-silent is noise, active-then-stopped is signal).
    static auto shouldEmitRecv(const sTopicStat& stat) -> bool;

    /// Decide whether one endpoint's record (jsonl "endpoints", text EP) is written: traffic this
    /// window, a proven receive endpoint (explicit 0.0 when stalled, like RECV), a reported gap
    /// (a stalled publisher stays visible, like JITTER), or the @p emit_idle opt-in.
    static auto shouldEmitEndpoint(const sEndpointStat& stat, bool emit_idle) -> bool;

    /// Observability hook: number of times onCallbackStart has escalated to the EXCLUSIVE
    /// (write) lock to run the full resolution chain. In steady state this must stay flat,
    /// resolved callbacks and proven non-subscriptions are served from the shared-lock and
    /// thread-local fast paths. Used by the KNOWN_ISSUES #3 write-lock-storm regression tests.
    auto writeLockResolutions() const -> uint64_t;

    /// Observability hook: number of times a hot-path call missed the thread-local cache and
    /// took the SHARED lock. Steady state for a warm per-thread working set must stay flat,
    /// the KNOWN_ISSUES #13 regression asserts alternating endpoints don't thrash the cache.
    /// Incremented only on the miss path (cold after warm-up), so the counter itself costs
    /// nothing where it matters.
    auto sharedLockLookups() const -> uint64_t;

    // pthread_atfork support (used by the probe layer, KNOWN_ISSUES #10): quiesce the registry
    // across fork() so the child never inherits m_mu locked by a thread it doesn't have.
    void forkPrepare();     ///< before fork: take the write lock
    void forkRelease();     ///< after fork, PARENT only: release it
    void forkChildReset();  ///< after fork, CHILD only: re-init the lock (unlock is a no-op
                            ///< there, the rwlock's stored writer TID no longer matches)

private:
    // Caller must hold the EXCLUSIVE lock: this may insert into m_cb_to_counter. Resolves
    // callback→endpoint through the chain (callback → rclcpp-sub → rcl-handle → endpoint) and
    // caches the result. Returns nullptr while the chain of a real subscription is not yet
    // populated (kept uncached, so lazy resolution retries), or the kNotASubscription sentinel,
    // cached, for a callback with no m_cb_to_sub entry at all (a timer/service callback that can
    // never resolve).
    auto resolveCallback(const void* callback) -> sEndpointCounter*;
    auto counterForTopic(const std::string& topic) -> sTopicCounter*;

    // Caller must hold the write lock. Binds @p handle (a publisher or rcl subscription handle)
    // to an endpoint on @p topic owned by @p node_handle. A handle seen for the first time gets
    // a new endpoint; a RECYCLED handle (destroyed, address reused: there is no destroy
    // tracepoint) re-homes its existing endpoint in place under a fresh id, so every
    // thread-local cache entry and resolved callback pointing at it follows the new identity.
    auto bindEndpoint(std::unordered_map<const void*, sEndpointCounter*>& map, const void* handle,
                      const char* topic, const void* node_handle, eEndpointSide side)
        -> sEndpointCounter*;
    // Caller must hold the write lock. Detach @p ep from its owning node (if resolved) and from
    // the by-node-handle index.
    void unlinkEndpointNode(sEndpointCounter* ep);
    // Caller must hold the write lock. Mark a receive endpoint (and its topic) proven.
    static void markSeen(sEndpointCounter* ep);

    // Caller must hold the lock (shared suffices). rmw handle -> endpoint for the take path:
    // nullptr when the handle is unknown, kTakeOwnedByRclcpp's sentinel for an rclcpp-owned
    // subscription, otherwise the subscription's endpoint.
    auto resolveTake(const void* rmw_sub_handle) const -> sEndpointCounter*;
    // Thread-local cache tag for take-path entries: registry id plus the subscription-graph
    // epoch, so any (re)init of a subscription invalidates every thread's cached take verdicts.
    auto takeTag() const -> uint64_t;

    // Shared hot path for both publish transports: TLS-cache hit or shared-lock lookup on
    // m_pub_to_counter, then bump the inter or intra publish bucket.
    void publishCount(const void* pub_handle, bool is_intra_process);

    // TLS-cache hit or shared-lock lookup of a handle in @p map (the loaned paths). nullptr when
    // the handle is unknown. Publisher handles, rcl subscription handles and callback objects
    // are distinct live allocations, so they share the TLS cache without key collisions.
    auto lookupHandle(const void* handle,
                      const std::unordered_map<const void*, sEndpointCounter*>& map)
        -> sEndpointCounter*;

    // Unique per-instance id (from a process-global atomic). Used to scope the thread-local
    // hot-path cache so a recycled stack/heap address never serves a destroyed instance's counter.
    uint64_t m_id;

    // Consecutive quiet windows tolerated before a node drops out of activeNodes().
    uint32_t m_quiet_windows;

    mutable std::shared_mutex m_mu;

    // owns topic records, keyed by topic name (never erased: endpoints point at them)
    std::unordered_map<std::string, std::unique_ptr<sTopicCounter>> m_by_topic;
    // owns endpoint counters, one per distinct publisher / rcl subscription handle address
    // (never erased: a recycled handle re-homes its endpoint, so memory stays bounded by the
    // number of distinct addresses ever used, not by churn)
    std::vector<std::unique_ptr<sEndpointCounter>> m_endpoints;
    uint32_t m_next_endpoint_id{1};

    // publish side: publisher_handle → endpoint
    std::unordered_map<const void*, sEndpointCounter*> m_pub_to_counter;

    // receive side resolution chain
    std::unordered_map<const void*, sEndpointCounter*> m_subhandle_to_counter;  // rcl sub → ep
    std::unordered_map<const void*, const void*> m_sub_to_subhandle;  // rclcpp sub → rcl sub
    std::unordered_map<const void*, const void*> m_cb_to_sub;         // callback → rclcpp sub
    std::unordered_map<const void*, sEndpointCounter*> m_cb_to_counter;  // resolved cache

    // take-layer receive resolution (rmw_take carries only the rmw handle)
    std::unordered_map<const void*, const void*> m_rmwsub_to_subhandle;  // rmw sub -> rcl sub
    std::unordered_map<const void*, bool> m_subhandle_rclcpp;            // rcl sub -> rclcpp-owned
    // Bumped (under the write lock) by every subscription init / ownership change; part of the
    // take-path TLS tag, so a recycled rmw handle or a late ownership mark is never served a
    // stale cached verdict. Read relaxed once per take.
    std::atomic<uint32_t> m_sub_epoch{0};

    // Count of hot-path escalations to the exclusive lock (see writeLockResolutions()).
    std::atomic<uint64_t> m_write_lock_resolutions{0};
    std::atomic<uint64_t> m_shared_lock_lookups{0};
    // Gap tracking gate (R5). Relaxed-loaded once per message; the branch is perfectly predicted
    // and measured at under 0.02 ns/op when off, so there is no second code path.
    std::atomic<bool> m_gap_tracking{false};

    // Node liveness: owned records in insertion order, plus dedup-by-name and handle->node indexes.
    std::vector<std::unique_ptr<sNode>> m_nodes;
    std::unordered_map<std::string, sNode*> m_node_by_name;
    std::unordered_map<const void*, sNode*> m_nodehandle_to_node;
    // Every endpoint by the node handle its init carried, resolved or not. rcl creates a node's
    // /rosout publisher INSIDE rcl_node_init, before that function's tracepoint fires, so that
    // publisher's init always arrives first, and on a recycled node address m_nodehandle_to_node
    // still names the destroyed node at that moment. onNodeInit therefore (re)attributes every
    // endpoint carrying its handle to the node it just initialized.
    std::unordered_map<const void*, std::vector<sEndpointCounter*>> m_eps_by_nodehandle;
};

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__TOPIC_REGISTRY_HPP_

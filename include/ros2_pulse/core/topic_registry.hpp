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

/// @brief One topic's running message counts, split by role + transport.
///
/// Separate publish and receive buckets so a same-process publisher and subscriber of one topic
/// never fetch_add the same field (KNOWN_ISSUES.md #1).
struct sTopicCounter {
    std::string topic;
    // True once a subscription callback for this topic has RESOLVED (first delivered message).
    // Written only under the registry's write lock; lets a later zero-traffic window emit an
    // explicit RECV 0.0 line instead of dropping the topic (KNOWN_ISSUES #12).
    bool recv_endpoint_seen{false};
    std::atomic<uint64_t> pub_inter{0};   // inter-process publishes (rcl_publish)
    std::atomic<uint64_t> pub_intra{0};   // intra-process publishes (rclcpp_intra_publish, iron+)
    std::atomic<uint64_t> recv_inter{0};  // inter-process receives (callback_start, intra=false)
    std::atomic<uint64_t> recv_intra{0};  // intra-process receives (callback_start, intra=true)

    // --- gap tracking (ROADMAP R5), only written when the registry has it enabled ---
    // Transport-merged, one accumulator per SIDE: a windowed mean cannot see a stall (a 400 ms
    // freeze on a 50 Hz topic still averages 46 Hz over 5 s), so the max inter-arrival gap is the
    // window-length-independent detector. Nanoseconds, integer: lock-free everywhere and exact.
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

/// @brief One initialized node plus its recent-activity bookkeeping.
///
/// Liveness is inferred from topic traffic (there is no node-teardown tracepoint on Humble, see
/// KNOWN_ISSUES.md #2). @c counters are the per-topic counters this node owns as a
/// publisher or subscriber; @c idle_windows counts consecutive windows in which none of them saw
/// traffic. A node is reported active while @c idle_windows is below the registry's K.
struct sNode {
    std::string name;
    uint32_t idle_windows{0};
    std::vector<sTopicCounter*> counters;
};

/// @brief Pure C++ core of the probe. Holds the ROS-graph handle→topic maps and per-endpoint
/// atomic counters, resolves callback→topic lazily, and aggregates windowed frequencies.
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
    void onSubscriptionInit(const void* sub_handle, const void* node_handle, const char* topic);
    void onRclcppSubscriptionInit(const void* subscription, const void* sub_handle);
    void onCallbackAdded(const void* callback, const void* subscription);
    void onNodeInit(const void* node_handle, const char* node_name, const char* node_namespace);

    // --- hot path ---
    void onPublish(const void* pub_handle);       // inter-process publish (rcl_publish)
    void onIntraPublish(const void* pub_handle);  // intra-process publish (iron+ tracepoint)
    void onCallbackStart(const void* callback, bool is_intra_process);  // any-transport receive

    /// Enable per-endpoint inter-arrival gap tracking (ROADMAP R5). Off by default: it costs one
    /// clock read per message (~21 ns, vs ~0.3 ns for the counting path alone), so it is opt-in
    /// via ROS_TOPIC_STATS_JITTER=1 or implied by a max_gap_ms spec rule. Set once at startup,
    /// before any tracepoint can fire; atomic so the read in the hot path stays race-free.
    void setGapTracking(bool on);
    auto gapTracking() const -> bool;

    // --- aggregation ---
    /// Compute Hz over the given window and RESET all counts. Filtered topics excluded.
    ///
    /// @p fold_open_gap closes the currently-open interval into each side's reported gap, i.e.
    /// reports max(observed gaps, now - last arrival). Without it an endpoint that has gone
    /// completely silent produces no inter-arrival pair at all and would vanish from the gap
    /// report, missing the TOTAL stall, the worst case R5 exists to catch. Pass false for the
    /// atexit window: rclcpp teardown stops traffic before the process exits, so the open gap
    /// there measures the shutdown sequence and would fire every gap rule on a healthy stop.
    auto snapshot(double window_s, bool fold_open_gap = true) -> std::vector<sTopicStat>;
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
    // callback→counter through the chain (callback → rclcpp-sub → rcl-handle → counter) and caches
    // the result. Returns nullptr while the chain of a real subscription is not yet populated (kept
    // uncached, so lazy resolution retries), or the kNotASubscription sentinel, cached, for a
    // callback with no m_cb_to_sub entry at all (a timer/service callback that can never resolve).
    auto resolveCallback(const void* callback) -> sTopicCounter*;
    auto counterForTopic(const std::string& topic) -> sTopicCounter*;

    // Shared hot path for both publish transports: TLS-cache hit or shared-lock lookup on
    // m_pub_to_counter, then bump the inter or intra publish bucket.
    void publishCount(const void* pub_handle, bool is_intra_process);

    // Caller must hold the write lock. Records that the node identified by node_handle owns counter
    // (so window-boundary liveness can tell whether that node saw any traffic). No-op if the handle
    // is unknown or the counter is already listed for that node.
    void linkNodeCounter(const void* node_handle, sTopicCounter* counter);

    // Unique per-instance id (from a process-global atomic). Used to scope the thread-local
    // hot-path cache so a recycled stack/heap address never serves a destroyed instance's counter.
    uint64_t m_id;

    // Consecutive quiet windows tolerated before a node drops out of activeNodes().
    uint32_t m_quiet_windows;

    mutable std::shared_mutex m_mu;

    // owns counters, keyed by topic name
    std::unordered_map<std::string, std::unique_ptr<sTopicCounter>> m_by_topic;

    // publish side: publisher_handle → counter
    std::unordered_map<const void*, sTopicCounter*> m_pub_to_counter;

    // receive side resolution chain
    std::unordered_map<const void*, sTopicCounter*> m_subhandle_to_counter;  // rcl sub_handle → counter
    std::unordered_map<const void*, const void*> m_sub_to_subhandle;          // rclcpp sub → rcl sub_handle
    std::unordered_map<const void*, const void*> m_cb_to_sub;                 // callback → rclcpp sub
    std::unordered_map<const void*, sTopicCounter*> m_cb_to_counter;          // resolved cache

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
};

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__TOPIC_REGISTRY_HPP_

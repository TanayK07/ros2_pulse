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
/// never fetch_add the same field (KNOWN_ISSUES.md #1 — see docs/issues/issue-1-double-count.md).
struct sTopicCounter {
    std::string topic;
    std::atomic<uint64_t> pub_inter{0};   // inter-process publishes (rcl_publish)
    std::atomic<uint64_t> recv_inter{0};  // inter-process receives (callback_start, intra=false)
    std::atomic<uint64_t> recv_intra{0};  // intra-process receives (callback_start, intra=true)
};

/// @brief Aggregated, windowed view of one topic (returned by snapshot()).
struct sTopicStat {
    std::string topic;
    uint64_t pub_inter_count{0};
    uint64_t recv_inter_count{0};
    uint64_t recv_intra_count{0};
    double pub_inter_hz{0.0};
    double recv_inter_hz{0.0};
    double recv_intra_hz{0.0};
};

/// @brief Pure C++ core of the probe. Holds the ROS-graph handle→topic maps and per-endpoint
/// atomic counters, resolves callback→topic lazily, and aggregates windowed frequencies.
///
/// NO ROS / tracetools dependency — testable in isolation. The probe layer feeds it raw handles
/// captured from the interposed tracetools functions.
///
/// Thread-safety: graph-init methods (rare) take a write lock; hot-path methods (onPublish,
/// onCallbackStart) take a read lock and increment a relaxed atomic, with a thread-local cache
/// that elides the lock for the common repeated-endpoint case.
class TopicRegistry {
public:
    TopicRegistry();
    TopicRegistry(TopicRegistry const&) = delete;
    auto operator=(TopicRegistry const&) -> TopicRegistry& = delete;

    // --- graph init (low frequency) ---
    void onPublisherInit(const void* pub_handle, const char* topic);
    void onSubscriptionInit(const void* sub_handle, const char* topic);
    void onRclcppSubscriptionInit(const void* subscription, const void* sub_handle);
    void onCallbackAdded(const void* callback, const void* subscription);
    void onNodeInit(const char* node_name, const char* node_namespace);

    // --- hot path ---
    void onPublish(const void* pub_handle);                  // inter-process publish
    void onCallbackStart(const void* callback, bool is_intra_process);  // any-transport receive

    // --- aggregation ---
    /// Compute Hz over the given window and RESET all counts. Filtered topics excluded.
    auto snapshot(double window_s) -> std::vector<sTopicStat>;
    auto activeNodes() const -> std::vector<std::string>;

    /// System/util topics excluded from output.
    static auto shouldFilter(const std::string& topic) -> bool;

    /// Observability hook: number of times onCallbackStart has escalated to the EXCLUSIVE
    /// (write) lock to run the full resolution chain. In steady state this must stay flat —
    /// resolved callbacks and proven non-subscriptions are served from the shared-lock and
    /// thread-local fast paths. Used by the KNOWN_ISSUES #3 write-lock-storm regression tests.
    auto writeLockResolutions() const -> uint64_t;

private:
    // Caller must hold the EXCLUSIVE lock: this may insert into m_cb_to_counter. Resolves
    // callback→counter through the chain (callback → rclcpp-sub → rcl-handle → counter) and caches
    // the result. Returns nullptr while the chain of a real subscription is not yet populated (kept
    // uncached, so lazy resolution retries), or the kNotASubscription sentinel — cached — for a
    // callback with no m_cb_to_sub entry at all (a timer/service callback that can never resolve).
    auto resolveCallback(const void* callback) -> sTopicCounter*;
    auto counterForTopic(const std::string& topic) -> sTopicCounter*;

    // Unique per-instance id (from a process-global atomic). Used to scope the thread-local
    // hot-path cache so a recycled stack/heap address never serves a destroyed instance's counter.
    uint64_t m_id;

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

    std::vector<std::string> m_nodes;
};

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__TOPIC_REGISTRY_HPP_

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

/// @brief One topic's running message counts, split by transport.
struct sTopicCounter {
    std::string topic;
    std::atomic<uint64_t> inter{0};  // inter-process (DDS) messages
    std::atomic<uint64_t> intra{0};  // intra-process (rclcpp IPC) messages
};

/// @brief Aggregated, windowed view of one topic (returned by snapshot()).
struct sTopicStat {
    std::string topic;
    uint64_t inter_count{0};
    uint64_t intra_count{0};
    double inter_hz{0.0};
    double intra_hz{0.0};
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

    /// Decide whether the publish-side `TOPIC` line should be written for this window's stat.
    /// A fully-idle topic (no inter- AND no intra-process traffic) is emitted only when
    /// @p emit_idle is true (operator opt-in via ROS_PULSE_EMIT_IDLE=1); otherwise the historical
    /// rule stands — emit when inter-process traffic is present, and suppress the publish-side line
    /// for intra-only topics (whose signal is carried on the additive RECV line instead).
    static auto shouldEmitTopic(const sTopicStat& stat, bool emit_idle) -> bool;

private:
    // Caller must hold at least a read lock. Resolves callback→counter through the chain
    // (callback → rclcpp-sub → rcl-handle → counter) and caches the result; returns nullptr
    // until the chain is fully populated.
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

    std::vector<std::string> m_nodes;
};

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__TOPIC_REGISTRY_HPP_

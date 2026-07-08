// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// LD_PRELOAD interposers for the ROS2 tracetools instrumentation layer.
//
// We export the same symbols as libtracetools.so's tracepoint API. The dynamic linker binds
// rclcpp's calls to ours first (preload interposition); each interposer records a statistic and
// forwards to the real function obtained once via dlsym(RTLD_NEXT, ...). Because rclcpp calls
// these functions unconditionally (the LTTng enable-check is *inside* them), the probe works
// with no LTTng session running and adds no DDS traffic.
//
// Hooking the tracetools layer (instead of rmw_*) is what lets us see INTRA-process traffic:
// callback_start() fires for every subscription callback regardless of transport, carrying an
// is_intra_process flag.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <dlfcn.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>

#include "ros2_pulse/core/env_config.hpp"
#include "ros2_pulse/core/timer.hpp"
#include "ros2_pulse/core/topic_registry.hpp"

namespace {

using ros2_pulse::core::parsePeriodSeconds;
using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::Timer;
using ros2_pulse::core::TopicRegistry;

auto getEnv(const char* key, const char* def) -> std::string {
    const char* v = std::getenv(key);
    return (v && *v) ? std::string(v) : std::string(def);
}

/// Process-wide probe runtime: the registry, the flush timer and output config.
class ProbeRuntime {
public:
    static auto instance() -> ProbeRuntime& {
        static ProbeRuntime s_instance;
        return s_instance;
    }

    auto registry() -> TopicRegistry& { return m_registry; }

    void ensureStarted() {
        bool expected = false;
        if (m_started.compare_exchange_strong(expected, true)) {
            std::fprintf(stderr, "[ros2_pulse] active — interposing tracetools layer "
                                 "(out=%s, period=%.1fs)\n",
                         m_out_path.c_str(), m_period_s);
            m_timer.emplace([this]() { flush(); },
                            std::chrono::milliseconds(static_cast<long>(m_period_s * 1000.0)));
            m_timer->start();
        }
    }

private:
    ProbeRuntime()
        : m_out_path(getEnv("ROS_TOPIC_STATS_OUTPUT_FILE", "/root/ssd2tb/logs/topic_freq.log")),
          // noexcept parse: a bad ROS_TOPIC_STATISTICS_PUBLISH_PERIOD must fall back to the default,
          // never throw out of this tracepoint-reached ctor into rclcpp (KNOWN_ISSUES.md #5).
          m_period_s(parsePeriodSeconds(std::getenv("ROS_TOPIC_STATISTICS_PUBLISH_PERIOD"), 5.0)) {}

    void flush() {
        auto stats = m_registry.snapshot(m_period_s);
        auto nodes = m_registry.activeNodes();
        if (stats.empty() && nodes.empty()) {
            return;
        }
        std::FILE* f = std::fopen(m_out_path.c_str(), "a");
        if (!f) {
            return;
        }
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        std::fprintf(f, "# ts_ns=%lld window_s=%.3f\n", static_cast<long long>(ns), m_period_s);
        for (const auto& s : stats) {
            // publish-side line; output format kept compatible with the earlier global-mutex +
            // per-message-string-hash stats prototype this design replaced. It now reads the
            // genuine publish counter, so a pure in-process subscriber no longer fabricates a
            // publish rate. Idle known topics still emit a zero line (see KNOWN_ISSUES.md #7).
            if (s.pub_inter_count > 0 || (s.recv_inter_count == 0 && s.recv_intra_count == 0)) {
                std::fprintf(f, "TOPIC %s %.6f\n", s.topic.c_str(), s.pub_inter_hz);
            }
            // additive receive-side line incl. intra-process (the new capability), independent of
            // the publish counter so a same-process pub+sub is no longer double-counted.
            if (s.recv_inter_count > 0 || s.recv_intra_count > 0) {
                std::fprintf(f, "RECV %s inter=%.6f intra=%.6f\n", s.topic.c_str(),
                             s.recv_inter_hz, s.recv_intra_hz);
            }
        }
        for (const auto& n : nodes) {
            std::fprintf(f, "NODE %s\n", n.c_str());
        }
        std::fprintf(f, "\n");
        std::fclose(f);
    }

    TopicRegistry m_registry;
    std::optional<Timer> m_timer;
    std::atomic<bool> m_started{false};
    std::string m_out_path;
    double m_period_s;
};

template <typename Fn>
auto realFn(const char* name) -> Fn {
    return reinterpret_cast<Fn>(dlsym(RTLD_NEXT, name));
}

}  // namespace

extern "C" {

// ---- graph init (low frequency) ----

void ros_trace_rcl_node_init(const void* node_handle, const void* rmw_handle, const char* name,
                             const char* ns) {
    ProbeRuntime::instance().ensureStarted();
    ProbeRuntime::instance().registry().onNodeInit(node_handle, name, ns);
    static auto fn = realFn<void (*)(const void*, const void*, const char*, const char*)>(
        "ros_trace_rcl_node_init");
    if (fn) fn(node_handle, rmw_handle, name, ns);
}

void ros_trace_rcl_publisher_init(const void* pub_handle, const void* node_handle,
                                  const void* rmw_pub, const char* topic, size_t depth) {
    ProbeRuntime::instance().ensureStarted();
    ProbeRuntime::instance().registry().onPublisherInit(pub_handle, node_handle, topic);
    static auto fn = realFn<void (*)(const void*, const void*, const void*, const char*, size_t)>(
        "ros_trace_rcl_publisher_init");
    if (fn) fn(pub_handle, node_handle, rmw_pub, topic, depth);
}

void ros_trace_rcl_subscription_init(const void* sub_handle, const void* node_handle,
                                     const void* rmw_sub, const char* topic, size_t depth) {
    ProbeRuntime::instance().ensureStarted();
    ProbeRuntime::instance().registry().onSubscriptionInit(sub_handle, node_handle, topic);
    static auto fn = realFn<void (*)(const void*, const void*, const void*, const char*, size_t)>(
        "ros_trace_rcl_subscription_init");
    if (fn) fn(sub_handle, node_handle, rmw_sub, topic, depth);
}

void ros_trace_rclcpp_subscription_init(const void* sub_handle, const void* subscription) {
    ProbeRuntime::instance().registry().onRclcppSubscriptionInit(subscription, sub_handle);
    static auto fn = realFn<void (*)(const void*, const void*)>("ros_trace_rclcpp_subscription_init");
    if (fn) fn(sub_handle, subscription);
}

void ros_trace_rclcpp_subscription_callback_added(const void* subscription, const void* callback) {
    ProbeRuntime::instance().registry().onCallbackAdded(callback, subscription);
    auto fn =
        realFn<void (*)(const void*, const void*)>("ros_trace_rclcpp_subscription_callback_added");
    if (fn) fn(subscription, callback);
}

// ---- hot path ----

void ros_trace_rcl_publish(const void* pub_handle, const void* message) {
    ProbeRuntime::instance().registry().onPublish(pub_handle);
    static auto fn = realFn<void (*)(const void*, const void*)>("ros_trace_rcl_publish");
    if (fn) fn(pub_handle, message);
}

void ros_trace_callback_start(const void* callback, bool is_intra_process) {
    ProbeRuntime::instance().registry().onCallbackStart(callback, is_intra_process);
    static auto fn = realFn<void (*)(const void*, bool)>("ros_trace_callback_start");
    if (fn) fn(callback, is_intra_process);
}

}  // extern "C"

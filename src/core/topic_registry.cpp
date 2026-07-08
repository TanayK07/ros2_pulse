// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "ros2_pulse/core/topic_registry.hpp"

#include <mutex>  // for std::unique_lock with shared_mutex

namespace ros2_pulse::core {

namespace {
std::atomic<uint64_t> g_next_registry_id{1};
}  // namespace

TopicRegistry::TopicRegistry() : m_id(g_next_registry_id.fetch_add(1, std::memory_order_relaxed)) {}

auto TopicRegistry::shouldFilter(const std::string& topic) -> bool {
    return topic == "/parameter_events" || topic == "/rosout" || topic == "/diagnostics";
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

void TopicRegistry::onPublisherInit(const void* pub_handle, const char* topic) {
    if (pub_handle == nullptr || topic == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    m_pub_to_counter[pub_handle] = counterForTopic(topic);
}

void TopicRegistry::onSubscriptionInit(const void* sub_handle, const char* topic) {
    if (sub_handle == nullptr || topic == nullptr) {
        return;
    }
    std::unique_lock<std::shared_mutex> lock(m_mu);
    m_subhandle_to_counter[sub_handle] = counterForTopic(topic);
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

void TopicRegistry::onNodeInit(const char* node_name, const char* node_namespace) {
    if (node_name == nullptr) {
        return;
    }
    std::string ns = node_namespace ? node_namespace : "";
    std::string full = (ns.empty() || ns == "/") ? ("/" + std::string(node_name))
                                                  : (ns + "/" + std::string(node_name));
    std::unique_lock<std::shared_mutex> lock(m_mu);
    m_nodes.push_back(full);
}

auto TopicRegistry::resolveCallback(const void* callback) -> sTopicCounter* {
    auto cached = m_cb_to_counter.find(callback);
    if (cached != m_cb_to_counter.end()) {
        return cached->second;
    }
    auto s = m_cb_to_sub.find(callback);
    if (s == m_cb_to_sub.end()) {
        return nullptr;
    }
    auto h = m_sub_to_subhandle.find(s->second);
    if (h == m_sub_to_subhandle.end()) {
        return nullptr;
    }
    auto c = m_subhandle_to_counter.find(h->second);
    if (c == m_subhandle_to_counter.end()) {
        return nullptr;
    }
    m_cb_to_counter[callback] = c->second;  // cache once fully resolved
    return c->second;
}

void TopicRegistry::onPublish(const void* pub_handle) {
    // Thread-local cache is scoped to (registry id, handle) so it can never serve a counter
    // owned by a different/destroyed registry instance, even at a recycled address.
    thread_local uint64_t last_id = 0;
    thread_local const void* last_key = nullptr;
    thread_local sTopicCounter* last_ctr = nullptr;
    if (pub_handle != nullptr && last_id == m_id && pub_handle == last_key) {
        last_ctr->pub_inter.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    std::shared_lock<std::shared_mutex> lock(m_mu);
    auto it = m_pub_to_counter.find(pub_handle);
    if (it != m_pub_to_counter.end()) {
        it->second->pub_inter.fetch_add(1, std::memory_order_relaxed);
        last_id = m_id;
        last_key = pub_handle;
        last_ctr = it->second;
    }
}

void TopicRegistry::onCallbackStart(const void* callback, bool is_intra_process) {
    // Fast path: thread-local cache of the last resolved callback for this thread, scoped to
    // (registry id, callback) to avoid serving a counter from a destroyed registry instance.
    thread_local uint64_t last_id = 0;
    thread_local const void* last_key = nullptr;
    thread_local sTopicCounter* last_ctr = nullptr;
    if (callback != nullptr && last_id == m_id && callback == last_key) {
        if (is_intra_process) {
            last_ctr->recv_intra.fetch_add(1, std::memory_order_relaxed);
        } else {
            last_ctr->recv_inter.fetch_add(1, std::memory_order_relaxed);
        }
        return;
    }
    // Steady state: the callback is already resolved+cached, so a SHARED (concurrent) lock
    // suffices. Only the first time we see a callback do we take the exclusive lock to run the
    // full resolution chain and insert into the cache. This keeps multi-threaded executors from
    // serializing every received message on a single writer lock.
    sTopicCounter* ctr = nullptr;
    {
        std::shared_lock<std::shared_mutex> lock(m_mu);
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
    if (ctr == nullptr) {
        return;
    }
    if (is_intra_process) {
        ctr->recv_intra.fetch_add(1, std::memory_order_relaxed);
    } else {
        ctr->recv_inter.fetch_add(1, std::memory_order_relaxed);
    }
    last_id = m_id;
    last_key = callback;
    last_ctr = ctr;
}

auto TopicRegistry::snapshot(double window_s) -> std::vector<sTopicStat> {
    std::vector<sTopicStat> out;
    std::unique_lock<std::shared_mutex> lock(m_mu);
    const double w = window_s > 0.0 ? window_s : 1.0;
    for (auto& kv : m_by_topic) {
        sTopicCounter* c = kv.second.get();
        if (shouldFilter(c->topic)) {
            // still reset so counts don't accumulate unbounded
            c->pub_inter.exchange(0, std::memory_order_relaxed);
            c->recv_inter.exchange(0, std::memory_order_relaxed);
            c->recv_intra.exchange(0, std::memory_order_relaxed);
            continue;
        }
        uint64_t p = c->pub_inter.exchange(0, std::memory_order_relaxed);
        uint64_t ri = c->recv_inter.exchange(0, std::memory_order_relaxed);
        uint64_t rx = c->recv_intra.exchange(0, std::memory_order_relaxed);
        sTopicStat s;
        s.topic = c->topic;
        s.pub_inter_count = p;
        s.recv_inter_count = ri;
        s.recv_intra_count = rx;
        s.pub_inter_hz = static_cast<double>(p) / w;
        s.recv_inter_hz = static_cast<double>(ri) / w;
        s.recv_intra_hz = static_cast<double>(rx) / w;
        out.push_back(std::move(s));
    }
    return out;
}

auto TopicRegistry::activeNodes() const -> std::vector<std::string> {
    std::shared_lock<std::shared_mutex> lock(m_mu);
    return m_nodes;
}

auto TopicRegistry::writeLockResolutions() const -> uint64_t {
    return m_write_lock_resolutions.load(std::memory_order_relaxed);
}

}  // namespace ros2_pulse::core

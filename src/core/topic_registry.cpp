// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "ros2_pulse/core/topic_registry.hpp"

#include <mutex>  // for std::unique_lock with shared_mutex
#include <unordered_set>

namespace ros2_pulse::core {

namespace {
std::atomic<uint64_t> g_next_registry_id{1};

// Negative-cache sentinel (KNOWN_ISSUES #3). A callback proven NEVER to be a subscription — a timer
// or service callback, for which callback_added never fired — resolves to this marker instead of
// nullptr, so repeat sightings are served from the shared-lock / thread-local fast paths rather than
// re-taking the exclusive lock on every call. It is a unique, valid address that is never
// dereferenced: every read site compares it by identity and skips (see onCallbackStart).
sTopicCounter g_not_a_subscription;
sTopicCounter* const kNotASubscription = &g_not_a_subscription;
}  // namespace

TopicRegistry::TopicRegistry(uint32_t quiet_windows)
    : m_id(g_next_registry_id.fetch_add(1, std::memory_order_relaxed)),
      m_quiet_windows(quiet_windows) {}

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
        return nullptr;  // subscription, but chain not populated yet — stay lazy, do NOT cache
    }
    auto c = m_subhandle_to_counter.find(h->second);
    if (c == m_subhandle_to_counter.end()) {
        return nullptr;  // ditto: retry on a later delivery once subscription_init lands
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
        // last_ctr is either a resolved counter or the kNotASubscription sentinel. The sentinel
        // means "known timer/service callback" — skip it without touching the lock or a counter.
        if (last_ctr != kNotASubscription) {
            if (is_intra_process) {
                last_ctr->recv_intra.fetch_add(1, std::memory_order_relaxed);
            } else {
                last_ctr->recv_inter.fetch_add(1, std::memory_order_relaxed);
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
    // result — a real counter or the sentinel — is decisive, so cache it thread-locally; that lets
    // even a not-a-subscription callback drop the shared lock on subsequent same-thread calls.
    if (ctr == nullptr) {
        return;
    }
    last_id = m_id;
    last_key = callback;
    last_ctr = ctr;
    if (ctr == kNotASubscription) {
        return;  // proven timer/service callback — nothing to count
    }
    if (is_intra_process) {
        ctr->recv_intra.fetch_add(1, std::memory_order_relaxed);
    } else {
        ctr->recv_inter.fetch_add(1, std::memory_order_relaxed);
    }
}

auto TopicRegistry::snapshot(double window_s) -> std::vector<sTopicStat> {
    std::vector<sTopicStat> out;
    std::unique_lock<std::shared_mutex> lock(m_mu);
    const double w = window_s > 0.0 ? window_s : 1.0;
    // Counters that carried traffic this window, used below to age per-node liveness. Filtered
    // topics are excluded from the stats output but still count as node activity (a node emitting
    // only /rosout is alive), so we record activity before the filter check.
    std::unordered_set<const sTopicCounter*> active;
    for (auto& kv : m_by_topic) {
        sTopicCounter* c = kv.second.get();
        // Exchange all three split buckets first (this also resets them), so filtered topics still
        // count toward node activity even though they are excluded from the stats output.
        uint64_t p = c->pub_inter.exchange(0, std::memory_order_relaxed);
        uint64_t ri = c->recv_inter.exchange(0, std::memory_order_relaxed);
        uint64_t rx = c->recv_intra.exchange(0, std::memory_order_relaxed);
        if (p > 0 || ri > 0 || rx > 0) {
            active.insert(c);  // this topic's endpoints saw traffic -> owning node(s) are live
        }
        if (shouldFilter(c->topic)) {
            continue;  // counts reset above, but the topic itself is never reported
        }
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

auto TopicRegistry::writeLockResolutions() const -> uint64_t {
    return m_write_lock_resolutions.load(std::memory_order_relaxed);
}

}  // namespace ros2_pulse::core

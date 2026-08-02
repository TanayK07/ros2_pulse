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
#include <pthread.h>   // pthread_atfork
#include <sys/stat.h>  // stat (size rotation)
#include <unistd.h>    // getpid

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "ros2_pulse/core/env_config.hpp"
#include "ros2_pulse/core/rate_spec.hpp"
#include "ros2_pulse/core/timer.hpp"
#include "ros2_pulse/core/topic_registry.hpp"
#include "ros2_pulse/core/window_format.hpp"

namespace {

using ros2_pulse::core::defaultOutputPath;
using ros2_pulse::core::evaluateRateSpec;
using ros2_pulse::core::formatWindow;
using ros2_pulse::core::parseMaxBytes;
using ros2_pulse::core::parsePeriodSeconds;
using ros2_pulse::core::parseRateSpec;
using ros2_pulse::core::sRateSpec;
using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::Timer;
using ros2_pulse::core::TopicRegistry;

// True iff the env var is set to exactly "1" (the documented opt-in form).
auto envFlag(const char* key) -> bool {
    const char* v = std::getenv(key);
    return v != nullptr && v[0] == '1' && v[1] == '\0';
}

// Resolve where this process writes. An explicit ROS_TOPIC_STATS_OUTPUT_FILE is honoured verbatim
// (operators can still deliberately share a path); otherwise default to a PER-PROCESS path with the
// pid embedded, so a normal multi-process ROS launch no longer has every LD_PRELOADed process
// appending to one shared file with no locking.
auto resolveOutputPath() -> std::string {
    const char* v = std::getenv("ROS_TOPIC_STATS_OUTPUT_FILE");
    if (v && *v) {
        return std::string(v);
    }
    return defaultOutputPath(static_cast<long>(::getpid()));
}

// Load the optional expected-rate spec (ROADMAP R1) named by ROS_TOPIC_STATS_EXPECTED. Runs in
// the tracepoint-reached singleton constructor, so it must never throw or take the host down: an
// unreadable or malformed spec warns ONCE on stderr and disables alerting, nothing more.
auto loadRateSpec() -> std::optional<sRateSpec> {
    const char* path = std::getenv("ROS_TOPIC_STATS_EXPECTED");
    if (path == nullptr || *path == '\0') {
        return std::nullopt;
    }
    std::FILE* f = std::fopen(path, "r");
    if (f == nullptr) {
        std::fprintf(stderr,
                     "[ros2_pulse] cannot read ROS_TOPIC_STATS_EXPECTED '%s' (%s) — "
                     "expected-rate alerting disabled\n",
                     path, std::strerror(errno));
        return std::nullopt;
    }
    std::string text;
    char buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
        text.append(buf, n);
    }
    std::fclose(f);
    std::string err;
    auto spec = parseRateSpec(text, err);
    if (!spec.has_value()) {
        std::fprintf(stderr,
                     "[ros2_pulse] invalid ROS_TOPIC_STATS_EXPECTED '%s' (%s) — "
                     "expected-rate alerting disabled\n",
                     path, err.c_str());
    }
    return spec;
}

/// Process-wide probe runtime: the registry, the flush timer and output config.
class ProbeRuntime {
public:
    static auto instance() -> ProbeRuntime& {
        // LEAKY singleton — intentionally never destroyed (KNOWN_ISSUES #9). DDS transport and
        // executor threads keep firing tracepoints while static destructors run in undefined
        // cross-library order; a Meyers singleton's destroyed registry/mutex made those
        // stragglers a use-after-free (reproducibly SIGSEGV under test/integration/
        // test_shutdown.py's exit_storm). Leaked, the runtime stays valid for any straggler at
        // any point of teardown — late events just count into buckets that are never flushed.
        // The flush thread is stopped (and the tail window written) by the atexit hook below;
        // the OS reclaims the rest at process exit.
        static auto* s_instance = new ProbeRuntime();
        return *s_instance;
    }

    auto registry() -> TopicRegistry& { return m_registry; }

    void ensureStarted() {
        bool expected = false;
        if (m_started.compare_exchange_strong(expected, true)) {
            std::fprintf(stderr, "[ros2_pulse] active — interposing tracetools layer "
                                 "(out=%s, period=%.1fs)\n",
                         m_out_path.c_str(), m_period_s);
            // Counting effectively begins here (first tracepoint) — stamp the window start
            // before the flush thread exists so the first window's denominator is measured
            // from the same origin the counts accumulate from (KNOWN_ISSUES #8b).
            m_window_start = std::chrono::steady_clock::now();
            // Heap-allocate the timer and LEAK any previous one (a fork()ed child re-arming
            // here still holds the parent's timer object, whose condition variable may carry
            // waiter refs from the dead flush thread — destroying such a cv can block forever
            // in pthread_cond_destroy). One small leak per fork generation, same philosophy as
            // the leaky runtime itself (KNOWN_ISSUES #9/#10).
            m_timer = new Timer([this]() { flush(); },
                                std::chrono::milliseconds(static_cast<long>(m_period_s * 1000.0)));
            m_timer->start();
            // Process-lifecycle hooks, registered exactly once per PROCESS IMAGE (guarded by a
            // flag the fork-child handler does NOT reset — atexit/atfork registrations are
            // inherited across fork(), so re-registering per generation would stack duplicates
            // in grandchildren):
            //  - atexit (KNOWN_ISSUES #9): with the runtime leaked, nothing stops the flush
            //    thread implicitly anymore — join it at exit and write the FINAL PARTIAL
            //    window. Runs on the exiting thread, touches only leaked objects + libc.
            //  - pthread_atfork (KNOWN_ISSUES #10): quiesce our locks across fork() and let
            //    the child re-arm its own flush timer.
            bool hooks_expected = false;
            if (m_hooks_registered.compare_exchange_strong(hooks_expected, true)) {
                std::atexit([] { ProbeRuntime::instance().shutdownAtExit(); });
                pthread_atfork([] { ProbeRuntime::instance().forkPrepare(); },
                               [] { ProbeRuntime::instance().forkParent(); },
                               [] { ProbeRuntime::instance().forkChild(); });
            }
        }
    }

private:
    // --- pthread_atfork handlers (KNOWN_ISSUES #10) ---
    // Lock order matches the flush thread (Timer::runThread holds the timer mutex while
    // flush() -> snapshot() takes the registry write lock), so prepare can never deadlock
    // against a concurrent flush; fork then only lands at a quiescent point and the child
    // inherits both locks HELD BY THE FORKING THREAD, which its handler may legally release.
    void forkPrepare() {
        if (m_timer) {
            m_timer->forkPrepare();
        }
        m_registry.forkPrepare();
    }
    void forkParent() {
        m_registry.forkRelease();
        if (m_timer) {
            m_timer->forkRelease();
        }
    }
    void forkChild() {
        // Registry: RE-INIT, not unlock — pthread rwlock unlock is a silent no-op in the child
        // (stored writer TID no longer matches), which left the registry locked forever.
        m_registry.forkChildReset();
        if (m_timer) {
            m_timer->forkChildReset();  // drop the stale (dead) flush-thread handle
            m_timer->forkRelease();     // plain mutex: no owner check, unlock works in child
        }
        // The child is a new process: give it its own default output path (an explicit
        // ROS_TOPIC_STATS_OUTPUT_FILE stays honoured verbatim inside resolveOutputPath), a
        // fresh window origin, and let the NEXT tracepoint lazily re-arm the flush timer via
        // ensureStarted(). Until then the inherited atexit hook still guarantees a final flush
        // of whatever the child counts. Inherited pre-fork counts may smear into the child's
        // first window — documented in docs/issues/issue-10-fork-without-exec.md.
        m_out_path = resolveOutputPath();
        m_window_start = std::chrono::steady_clock::now();
        m_started.store(false);
    }

    // Single-generation size rotation (KNOWN_ISSUES #11): at/over the cap, atomically rename
    // <path> -> <path>.1 (replacing any previous generation) and let the append below start a
    // fresh file. One stat() per window, on the flush thread — not the hot path. Reopen-per-
    // window is preserved, so external logrotate keeps working for operators who prefer it.
    void rotateIfNeeded() {
        if (m_max_bytes == 0) {
            return;  // rotation disabled
        }
        struct stat st {};
        if (::stat(m_out_path.c_str(), &st) != 0) {
            return;  // nothing written yet (or path inaccessible — the fopen below will warn)
        }
        if (static_cast<unsigned long long>(st.st_size) < m_max_bytes) {
            return;
        }
        const std::string rotated = m_out_path + ".1";
        ::rename(m_out_path.c_str(), rotated.c_str());
    }

    void shutdownAtExit() {
        if (m_timer) {
            m_timer->stop();  // join the flush thread; periodic flushing ends here
        }
        flush();  // final partial window (measured window_s keeps its Hz honest, issue #8)
    }

    ProbeRuntime()
        : m_out_path(resolveOutputPath()),
          // noexcept parse: a bad ROS_TOPIC_STATISTICS_PUBLISH_PERIOD must fall back to the default,
          // never throw out of this tracepoint-reached ctor into rclcpp (KNOWN_ISSUES.md #5).
          m_period_s(parsePeriodSeconds(std::getenv("ROS_TOPIC_STATISTICS_PUBLISH_PERIOD"), 5.0)),
          // Size-rotation cap (KNOWN_ISSUES #11): rotate <path> -> <path>.1 at this size;
          // 0 disables (pure append). Default 10 MiB bounds worst-case disk at 2x cap.
          m_max_bytes(parseMaxBytes(std::getenv("ROS_TOPIC_STATS_MAX_BYTES"),
                                    10ULL * 1024 * 1024)),
          // Declared-but-silent topics (no traffic in a window) are suppressed by default so large
          // graphs don't accrue a `TOPIC /x 0.000000` line every window. Set ROS_PULSE_EMIT_IDLE=1
          // to restore the legacy behaviour of printing them. See KNOWN_ISSUES.md #7.
          m_emit_idle(envFlag("ROS_PULSE_EMIT_IDLE")),
          m_spec(loadRateSpec()) {}

    void flush() {
        // Hz must divide by the MEASURED window, not the configured period: the first window is
        // longer than the period (probe attaches before the timer's first fire) and any window
        // can be stretched by flush latency or scheduler jitter (KNOWN_ISSUES #8b). steady_clock
        // for the length (monotonic); ts_ns below stays wall-clock for log correlation.
        const auto now_mono = std::chrono::steady_clock::now();
        const double window_s = std::chrono::duration<double>(now_mono - m_window_start).count();
        m_window_start = now_mono;
        auto stats = m_registry.snapshot(window_s);
        auto nodes = m_registry.activeNodes();
        if (stats.empty() && nodes.empty()) {
            return;
        }
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        // Build the whole window block up front, then emit it with ONE fwrite. A window under
        // BUFSIZ is a single write() at fclose, so its lines stay contiguous instead of
        // interleaving mid-block with another process's per-line writes (see docs/issues/
        // issue-4-per-process-output.md). The block carries the MEASURED window_s (issue #8b),
        // and the TOPIC emit decision (incl. the idle-topic policy gated by ROS_PULSE_EMIT_IDLE)
        // lives in the pure core so it stays unit-testable (issue #7).
        // Expected-rate alerting (ROADMAP R1): evaluated here at flush time only — the hot path
        // never sees the spec. The FIRST non-empty window is grace-skipped: the probe attaches
        // mid-flight, so that window's rates are ramp-up partials that would cry wolf on start.
        const uint64_t window_index = m_windows_flushed.fetch_add(1, std::memory_order_relaxed);
        std::vector<std::string> warnings;
        if (m_spec.has_value() && window_index > 0) {
            warnings = evaluateRateSpec(*m_spec, stats, nodes, m_registry.knownNodes());
        }
        const std::string block = formatWindow(stats, nodes, static_cast<long long>(ns),
                                               window_s, m_emit_idle, warnings);

        rotateIfNeeded();
        std::FILE* f = std::fopen(m_out_path.c_str(), "a");
        if (!f) {
            // Don't silently drop every window (e.g. the output directory doesn't exist). Warn
            // ONCE — this runs on the timer thread every window, so a per-window log would spam.
            bool expected = false;
            if (m_warned_open_fail.compare_exchange_strong(expected, true)) {
                std::fprintf(stderr,
                             "[ros2_pulse] cannot open output file '%s' (%s) — dropping windows\n",
                             m_out_path.c_str(), std::strerror(errno));
            }
            return;
        }
        std::fwrite(block.data(), 1, block.size(), f);
        std::fclose(f);
    }

    TopicRegistry m_registry;
    Timer* m_timer{nullptr};  // heap-allocated, intentionally leaked (see ensureStarted)
    std::atomic<bool> m_started{false};
    // Once-per-process-image guard for atexit/pthread_atfork registration. Deliberately NOT
    // reset by forkChild(): the child inherits the parent's registrations.
    std::atomic<bool> m_hooks_registered{false};
    std::atomic<bool> m_warned_open_fail{false};
    std::string m_out_path;
    double m_period_s;
    unsigned long long m_max_bytes;
    bool m_emit_idle;
    // Expected-rate spec (ROADMAP R1); nullopt when unset/unreadable/invalid (warned once).
    std::optional<sRateSpec> m_spec;
    // Windows flushed so far — the first non-empty window is grace-skipped for alerting.
    std::atomic<uint64_t> m_windows_flushed{0};
    // Start of the current stats window. Written in ensureStarted() (before the flush thread is
    // created — the thread creation orders it) and thereafter only by flush() on the timer thread.
    std::chrono::steady_clock::time_point m_window_start{};
};

template <typename Fn>
auto realFn(const char* name) -> Fn {
    return reinterpret_cast<Fn>(dlsym(RTLD_NEXT, name));
}

}  // namespace

// The library is compiled with -fvisibility=hidden (KNOWN_ISSUES #14) so nothing leaks into
// the dynamic symbol table of every preloaded process; the seven interposers below are the
// ONLY contract and are re-exported explicitly. test/integration/test_symbols.py pins this.
#define ROS2_PULSE_EXPORT __attribute__((visibility("default")))

extern "C" {

// ---- graph init (low frequency) ----

ROS2_PULSE_EXPORT void ros_trace_rcl_node_init(const void* node_handle, const void* rmw_handle, const char* name,
                             const char* ns) {
    ProbeRuntime::instance().ensureStarted();
    ProbeRuntime::instance().registry().onNodeInit(node_handle, name, ns);
    static auto fn = realFn<void (*)(const void*, const void*, const char*, const char*)>(
        "ros_trace_rcl_node_init");
    if (fn) fn(node_handle, rmw_handle, name, ns);
}

ROS2_PULSE_EXPORT void ros_trace_rcl_publisher_init(const void* pub_handle, const void* node_handle,
                                  const void* rmw_pub, const char* topic, size_t depth) {
    ProbeRuntime::instance().ensureStarted();
    ProbeRuntime::instance().registry().onPublisherInit(pub_handle, node_handle, topic);
    static auto fn = realFn<void (*)(const void*, const void*, const void*, const char*, size_t)>(
        "ros_trace_rcl_publisher_init");
    if (fn) fn(pub_handle, node_handle, rmw_pub, topic, depth);
}

ROS2_PULSE_EXPORT void ros_trace_rcl_subscription_init(const void* sub_handle, const void* node_handle,
                                     const void* rmw_sub, const char* topic, size_t depth) {
    ProbeRuntime::instance().ensureStarted();
    ProbeRuntime::instance().registry().onSubscriptionInit(sub_handle, node_handle, topic);
    static auto fn = realFn<void (*)(const void*, const void*, const void*, const char*, size_t)>(
        "ros_trace_rcl_subscription_init");
    if (fn) fn(sub_handle, node_handle, rmw_sub, topic, depth);
}

ROS2_PULSE_EXPORT void ros_trace_rclcpp_subscription_init(const void* sub_handle, const void* subscription) {
    ProbeRuntime::instance().registry().onRclcppSubscriptionInit(subscription, sub_handle);
    static auto fn = realFn<void (*)(const void*, const void*)>("ros_trace_rclcpp_subscription_init");
    if (fn) fn(sub_handle, subscription);
}

ROS2_PULSE_EXPORT void ros_trace_rclcpp_subscription_callback_added(const void* subscription, const void* callback) {
    ProbeRuntime::instance().registry().onCallbackAdded(callback, subscription);
    auto fn =
        realFn<void (*)(const void*, const void*)>("ros_trace_rclcpp_subscription_callback_added");
    if (fn) fn(subscription, callback);
}

// ---- hot path ----

ROS2_PULSE_EXPORT void ros_trace_rcl_publish(const void* pub_handle, const void* message) {
    ProbeRuntime::instance().registry().onPublish(pub_handle);
    static auto fn = realFn<void (*)(const void*, const void*)>("ros_trace_rcl_publish");
    if (fn) fn(pub_handle, message);
}

// jazzy+ only: rclcpp publishes an intra-process message through the IntraProcessManager. On
// humble this symbol is exported but never called (the tracepoint doesn't exist there) — the
// probe stays a single binary across distros.
ROS2_PULSE_EXPORT void ros_trace_rclcpp_intra_publish(const void* publisher_handle,
                                                      const void* message) {
    ProbeRuntime::instance().registry().onIntraPublish(publisher_handle);
    static auto fn =
        realFn<void (*)(const void*, const void*)>("ros_trace_rclcpp_intra_publish");
    if (fn) fn(publisher_handle, message);
}

ROS2_PULSE_EXPORT void ros_trace_callback_start(const void* callback, bool is_intra_process) {
    ProbeRuntime::instance().registry().onCallbackStart(callback, is_intra_process);
    static auto fn = realFn<void (*)(const void*, bool)>("ros_trace_callback_start");
    if (fn) fn(callback, is_intra_process);
}

}  // extern "C"

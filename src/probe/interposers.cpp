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
#include <fcntl.h>     // open flags (O_NOFOLLOW hardening on the default output path)
#include <pthread.h>   // pthread_atfork
#include <sys/stat.h>  // stat (size rotation)
#include <unistd.h>    // getpid, close

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
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
using ros2_pulse::core::eStatsFormat;
using ros2_pulse::core::evaluateRateSpecWarnings;
using ros2_pulse::core::formatWindow;
using ros2_pulse::core::formatWindowJsonl;
using ros2_pulse::core::parseMaxBytes;
using ros2_pulse::core::parsePeriodSeconds;
using ros2_pulse::core::parseRateSpec;
using ros2_pulse::core::parseStatsFormat;
using ros2_pulse::core::readSpecFile;
using ros2_pulse::core::renderWarnLine;
using ros2_pulse::core::sRateSpec;
using ros2_pulse::core::sRateWarning;
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
// appending to one shared file with no locking. The default respects TMPDIR and falls back to
// /tmp, a directory that exists everywhere, unlike the field-test mount it replaced.
// The two cases also differ in trust (see the open in flush()): the default lands in a
// world-writable sticky directory with a predictable name, so its open refuses symlinks; an
// explicit path is the operator's own and may legitimately be one (e.g. a link to another disk).
auto outputPathIsDefault() -> bool {
    const char* v = std::getenv("ROS_TOPIC_STATS_OUTPUT_FILE");
    return v == nullptr || *v == '\0';
}

auto resolveOutputPath() -> std::string {
    const char* v = std::getenv("ROS_TOPIC_STATS_OUTPUT_FILE");
    if (v && *v) {
        return std::string(v);
    }
    return defaultOutputPath(static_cast<long>(::getpid()), std::getenv("TMPDIR"));
}

// Load the optional expected-rate spec (ROADMAP R1) named by ROS_TOPIC_STATS_EXPECTED. Runs in
// the tracepoint-reached singleton constructor, so it must never throw or take the host down: an
// unreadable or malformed spec warns ONCE on stderr and disables alerting, nothing more.
auto loadRateSpec() -> std::optional<sRateSpec> {
    const char* path = std::getenv("ROS_TOPIC_STATS_EXPECTED");
    if (path == nullptr || *path == '\0') {
        return std::nullopt;
    }
    // Bounded, regular-files-only read (core helper): an operator typo pointing at a rosbag, a
    // directory, a FIFO or /dev/urandom must not stall or balloon the host inside rcl_node_init.
    std::string text;
    std::string err;
    if (!readSpecFile(path, text, err)) {
        std::fprintf(stderr,
                     "[ros2_pulse] cannot read ROS_TOPIC_STATS_EXPECTED '%s' (%s), "
                     "expected-rate alerting disabled\n",
                     path, err.c_str());
        return std::nullopt;
    }
    auto spec = parseRateSpec(text, err);
    if (!spec.has_value()) {
        std::fprintf(stderr,
                     "[ros2_pulse] invalid ROS_TOPIC_STATS_EXPECTED '%s' (%s), "
                     "expected-rate alerting disabled\n",
                     path, err.c_str());
        return std::nullopt;
    }
    // An empty (or all-comment) spec parses cleanly into zero rules, which would arm alerting as
    // a permanent no-op, the silent absence of monitoring this feature exists to prevent. Say so.
    if (spec->topics.empty() && spec->nodes.empty()) {
        std::fprintf(stderr,
                     "[ros2_pulse] ROS_TOPIC_STATS_EXPECTED '%s' declares no topics and no "
                     "nodes, expected-rate alerting disabled\n",
                     path);
        return std::nullopt;
    }
    return spec;
}

/// Process-wide probe runtime: the registry, the flush timer and output config.
class ProbeRuntime {
public:
    static auto instance() -> ProbeRuntime& {
        // LEAKY singleton, intentionally never destroyed (KNOWN_ISSUES #9). DDS transport and
        // executor threads keep firing tracepoints while static destructors run in undefined
        // cross-library order; a Meyers singleton's destroyed registry/mutex made those
        // stragglers a use-after-free (reproducibly SIGSEGV under test/integration/
        // test_shutdown.py's exit_storm). Leaked, the runtime stays valid for any straggler at
        // any point of teardown, late events just count into buckets that are never flushed.
        // The flush thread is stopped (and the tail window written) by the atexit hook below;
        // the OS reclaims the rest at process exit.
        static auto* s_instance = new ProbeRuntime();
        return *s_instance;
    }

    auto registry() -> TopicRegistry& { return m_registry; }

    void ensureStarted() {
        bool expected = false;
        if (m_started.compare_exchange_strong(expected, true)) {
            // Report whether a spec armed: "why am I getting no WARNs?" is otherwise invisible,
            // and every disabling path (unset, unreadable, malformed, zero-rule) lands on
            // spec=none. snprintf into a fixed buffer keeps the banner allocation-free.
            //
            // ROS_TOPIC_STATS_QUIET=1 (ROADMAP R6) suppresses THIS banner and nothing else,
            // it exists for deployments that parse the wrapped process's stderr and can't
            // tolerate a foreign line. Scope decided deliberately narrow: the one-shot error
            // diagnostics (unreadable/malformed/zero-rule spec in loadRateSpec, unwritable
            // output file in flush) are NOT gated on it, because each of those paths disables
            // a feature the operator explicitly configured, and silently disabled alerting is
            // a bug class 0.2.0 fixed (see CHANGELOG: a directory read as a spec parsed to a
            // zero-rule no-op, a UTF-8 BOM failed a valid spec invisibly). A QUIET that also
            // swallowed diagnostics would reintroduce that class as a supported flag.
            // Pinned by test/integration/test_quiet.py.
            if (!m_quiet) {
                char spec_desc[64];
                if (m_spec.has_value()) {
                    std::snprintf(spec_desc, sizeof(spec_desc), "%zu topics, %zu nodes",
                                  m_spec->topics.size(), m_spec->nodes.size());
                } else {
                    std::snprintf(spec_desc, sizeof(spec_desc), "none");
                }
                std::fprintf(stderr, "[ros2_pulse] active, interposing tracetools layer "
                                     "(out=%s, format=%s, period=%.1fs, spec=%s, jitter=%s)\n",
                             m_out_path.c_str(),
                             m_format == eStatsFormat::kJsonl ? "jsonl" : "text", m_period_s,
                             spec_desc, m_registry.gapTracking() ? "on" : "off");
            }
            // Counting effectively begins here (first tracepoint), stamp the window start
            // before the flush thread exists so the first window's denominator is measured
            // from the same origin the counts accumulate from (KNOWN_ISSUES #8b).
            m_window_start = std::chrono::steady_clock::now();
            // Heap-allocate the timer and LEAK any previous one (a fork()ed child re-arming
            // here still holds the parent's timer object, whose condition variable may carry
            // waiter refs from the dead flush thread, destroying such a cv can block forever
            // in pthread_cond_destroy). One small leak per fork generation, same philosophy as
            // the leaky runtime itself (KNOWN_ISSUES #9/#10).
            m_timer = new Timer([this]() { flush(); },
                                std::chrono::milliseconds(static_cast<long>(m_period_s * 1000.0)));
            m_timer->start();
            // Process-lifecycle hooks, registered exactly once per PROCESS IMAGE (guarded by a
            // flag the fork-child handler does NOT reset, atexit/atfork registrations are
            // inherited across fork(), so re-registering per generation would stack duplicates
            // in grandchildren):
            //  - atexit (KNOWN_ISSUES #9): with the runtime leaked, nothing stops the flush
            //    thread implicitly anymore, join it at exit and write the FINAL PARTIAL
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
        // Registry: RE-INIT, not unlock, pthread rwlock unlock is a silent no-op in the child
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
        // first window (KNOWN_ISSUES #10).
        m_out_path = resolveOutputPath();
        m_window_start = std::chrono::steady_clock::now();
        m_started.store(false);
    }

    // Single-generation size rotation (KNOWN_ISSUES #11): at/over the cap, atomically rename
    // <path> -> <path>.1 (replacing any previous generation) and let the append below start a
    // fresh file. One stat() per window, on the flush thread, not the hot path. Reopen-per-
    // window is preserved, so external logrotate keeps working for operators who prefer it.
    void rotateIfNeeded() {
        if (m_max_bytes == 0) {
            return;  // rotation disabled
        }
        struct stat st {};
        if (::stat(m_out_path.c_str(), &st) != 0) {
            return;  // nothing written yet (or path inaccessible, the fopen below will warn)
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
        // Final partial window: still MEASURED and logged (window_s keeps its Hz correct,
        // issue #8), but not alert-judged, see the exiting guard in flush().
        flush(/*exiting=*/true);
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
          // Banner suppression for stderr-parsing deployments (ROADMAP R6). Same exact-"1"
          // opt-in as every other flag: QUIET=true/yes/0 keep the banner, so a half-set flag
          // fails loud (banner still there) instead of half-silencing.
          m_quiet(envFlag("ROS_TOPIC_STATS_QUIET")),
          m_spec(loadRateSpec()) {
        // Output format (ROADMAP R6). Parsed in the ctor BODY so the unknown-value warning can
        // show the offending text. The ctor runs exactly once (tracepoint-reached singleton),
        // so this warning is once-per-process by construction, same shape as the spec-file
        // errors: say it out loud, fall back safely, never take the host down. Unset/empty is
        // the silent text default (no warning): byte-identical output for everyone who never
        // touches the variable.
        const char* fmt_raw = std::getenv("ROS_TOPIC_STATS_FORMAT");
        const auto fmt = parseStatsFormat(fmt_raw);
        if (!fmt.has_value()) {
            std::fprintf(stderr,
                         "[ros2_pulse] unknown ROS_TOPIC_STATS_FORMAT '%s' (expected 'text' or "
                         "'jsonl'), falling back to text\n",
                         fmt_raw);
        }
        m_format = fmt.value_or(eStatsFormat::kText);
        // Per-endpoint inter-arrival gap tracking (ROADMAP R5). Off by default: it costs one
        // clock read per message (~21 ns against ~0.3 ns for counting alone), which is 0.012% of
        // a core at 4900 msg/s but still 25x the counting path, so it stays opt-in. Set in the
        // ctor BODY, not an initializer: m_registry is declared before m_spec, and a later
        // revision will also enable this when the spec carries a max_gap_ms rule.
        // A declared max_gap_ms rule is a stronger statement of intent than the absence of an
        // env var, and silently not checking a declared rule is the failure mode this project
        // fights everywhere else (zero-rule spec, malformed spec -> warn and disable).
        bool spec_wants_gap = false;
        if (m_spec.has_value()) {
            for (const auto& [name, rule] : m_spec->topics) {
                (void)name;
                if (!std::isinf(rule.max_gap_ms)) {
                    spec_wants_gap = true;
                    break;
                }
            }
        }
        m_registry.setGapTracking(envFlag("ROS_TOPIC_STATS_JITTER") || spec_wants_gap);
    }

    void flush(bool exiting = false) {
        // Hz must divide by the MEASURED window, not the configured period: the first window is
        // longer than the period (probe attaches before the timer's first fire) and any window
        // can be stretched by flush latency or scheduler jitter (KNOWN_ISSUES #8b). steady_clock
        // for the length (monotonic); ts_ns below stays wall-clock for log correlation.
        const auto now_mono = std::chrono::steady_clock::now();
        const double window_s = std::chrono::duration<double>(now_mono - m_window_start).count();
        m_window_start = now_mono;
        // Gap tracking (R5): fold the still-open interval into each endpoint's reported
        // max, EXCEPT on the exit window, rclcpp teardown stops traffic before the
        // process exits, so the open gap there measures the shutdown sequence and would
        // fire every max_gap_ms rule on a perfectly healthy stop.
        auto stats = m_registry.snapshot(window_s, /*fold_open_gap=*/!exiting);
        auto nodes = m_registry.activeNodes();
        if (stats.empty() && nodes.empty()) {
            return;
        }
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        // Build the whole window block up front, then emit it with ONE fwrite. A window under
        // BUFSIZ is a single write() at fclose, so its lines stay contiguous instead of
        // interleaving mid-block with another process's per-line writes (KNOWN_ISSUES #4). The block carries the MEASURED window_s (issue #8b),
        // and the TOPIC emit decision (incl. the idle-topic policy gated by ROS_PULSE_EMIT_IDLE)
        // lives in the pure core so it stays unit-testable (issue #7).
        // Expected-rate alerting (ROADMAP R1): evaluated here at flush time only, the hot path
        // never sees the spec. BOTH lifecycle transients are grace-skipped, because a window
        // shorter than the period makes count/window_s a 1-2 sample estimate (+-1/window_s Hz,
        // in either direction, and exactly 0 Hz when the sliver caught no message):
        //  - ATTACH: the FIRST non-empty window. The probe attaches mid-flight, so its rates are
        //    ramp-up partials that would cry wolf on start.
        //  - DETACH: the atexit window. rclcpp teardown ends traffic before the process does, and
        //    a run whose duration lands near a multiple of the period leaves a few-millisecond
        //    tail, reproducibly `WARN TOPIC ... hz=0.000000` on a perfectly healthy shutdown.
        // The rates themselves are still logged for both; only the alert judgement is suppressed.
        // (A `for: N consecutive windows` debounce would cover these generally, ROADMAP R1.1.)
        const uint64_t window_index = m_windows_flushed.fetch_add(1, std::memory_order_relaxed);
        // Evaluated STRUCTURED (R6): jsonl gets warnings as data; text renders the exact
        // pre-R6 lines from the same result, so the two formats can never judge differently.
        std::vector<sRateWarning> warnings;
        if (m_spec.has_value() && window_index > 0 && !exiting) {
            warnings = evaluateRateSpecWarnings(*m_spec, stats, nodes, m_registry.knownNodes());
        }
        std::string block;
        if (m_format == eStatsFormat::kJsonl) {
            block = formatWindowJsonl(stats, nodes, static_cast<long long>(ns), window_s,
                                      m_emit_idle, warnings);
        } else {
            std::vector<std::string> warn_lines;
            warn_lines.reserve(warnings.size());
            for (const auto& w : warnings) {
                warn_lines.push_back(renderWarnLine(w));
            }
            block = formatWindow(stats, nodes, static_cast<long long>(ns), window_s,
                                 m_emit_idle, warn_lines);
        }

        rotateIfNeeded();
        // The DEFAULT path lives in a world-writable sticky directory (/tmp) under a name
        // predictable from the pid, so a hostile local user could pre-create it as a symlink
        // and have a root-owned probed process append to the target (CWE-379). O_NOFOLLOW
        // turns that into ELOOP -> the warn-once path below. An EXPLICIT operator-set path
        // keeps full symlink freedom (linking the log onto another disk is legitimate).
        // O_CLOEXEC on both: a probed host that fork+execs must not leak the fd.
        const int flags = O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC |
                          (outputPathIsDefault() ? O_NOFOLLOW : 0);
        std::FILE* f = nullptr;
        const int fd = ::open(m_out_path.c_str(), flags, 0644);
        if (fd >= 0) {
            f = ::fdopen(fd, "a");
            if (!f) {
                ::close(fd);
            }
        }
        if (!f) {
            // Don't silently drop every window (e.g. the output directory doesn't exist). Warn
            // ONCE, this runs on the timer thread every window, so a per-window log would spam.
            bool expected = false;
            if (m_warned_open_fail.compare_exchange_strong(expected, true)) {
                std::fprintf(stderr,
                             "[ros2_pulse] cannot open output file '%s' (%s), dropping windows\n",
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
    // ROS_TOPIC_STATS_QUIET=1: suppress the informational banner (never diagnostics, see
    // the rationale block in ensureStarted()).
    bool m_quiet;
    // Output format (R6): text (default, byte-identical to pre-R6) or jsonl. Set once in the
    // ctor; an unrecognized value warned about there and fell back to text.
    eStatsFormat m_format{eStatsFormat::kText};
    // Expected-rate spec (ROADMAP R1); nullopt when unset/unreadable/invalid (warned once).
    std::optional<sRateSpec> m_spec;
    // Non-empty windows flushed so far (incremented past the empty-window early-out above), so
    // the first one is grace-skipped for alerting; the exit window is skipped via flush(exiting).
    std::atomic<uint64_t> m_windows_flushed{0};
    // Start of the current stats window. Written in ensureStarted() (before the flush thread is
    // created, the thread creation orders it) and thereafter only by flush() on the timer thread.
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

// iron+ only: rclcpp publishes an intra-process message through the IntraProcessManager. On
// humble this symbol is exported but never called (the tracepoint doesn't exist there), the
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

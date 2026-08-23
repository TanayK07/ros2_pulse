// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// pulse-check: offline expected-rate verdict for probe logs (ROADMAP R1). No ROS dependency,
// links only the pure ros2_pulse core, so watchdogs, CI jobs and systemd units can gate on it
// anywhere the log files land.
//
//   pulse-check --spec <spec.yaml> [--skip-last] <log> [<log> ...]
//
// Reads BOTH on-disk formats, text and jsonl (ROS_TOPIC_STATS_FORMAT, ROADMAP R6), via the
// shared parseLog, which sniffs the kind per line. Choosing the exporter-friendly format must
// not cost an operator their CI/watchdog gating.
//
// Each log is one probed process (the per-PID default output). The verdict is computed from
// the LAST window of every log, "the system's current state", with per-topic rates summed
// across logs (a publisher's process and a subscriber's process both report the same topic).
// Node knowledge is the union over all windows, so a node that died mid-log is reported
// missing rather than forgotten. Unlike the in-probe alerting, a spec topic that appears in
// NO log is a violation here (0 Hz): the log set is the whole picture.
//
// Summing across logs is the publish-side truth (two publisher processes at 10 Hz do produce
// 20 Hz) and, on the receive side, the DELIVERY rate, issue #1's per-topic semantic, so K
// subscriber processes on one topic report K x the publish rate. Bound `side: recv` with
// min_hz for liveness; use `side: pub` when you mean the topic's message rate under a max_hz cap.
//
// --skip-last judges the second-to-last window instead. Use it on logs from an EXITED stack:
// the probe's final window is the atexit flush, a sub-period sliver whose rate is a one-sample
// estimate, so a post-run CI gate on the true last window can go red on a healthy shutdown.
// A live watchdog tailing a running stack wants the default (the last window is current state).
//
// Exit codes: 0 = all checks pass, 1 = violations (printed to stdout, one per line),
//             2 = usage / unreadable file / invalid spec / no windows / --skip-last with <2.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/rate_spec.hpp"

namespace {

using ros2_pulse::core::evaluateRateSpec;
using ros2_pulse::core::parseLog;
using ros2_pulse::core::parseRateSpec;
using ros2_pulse::core::readSpecFile;
using ros2_pulse::core::sTopicStat;

// Logs are written by the probe itself and can legitimately be large (rotation caps them at
// ROS_TOPIC_STATS_MAX_BYTES, default 10 MiB), so they keep the plain slurp, unlike the spec
// path, which is operator-supplied and goes through the bounded core helper.
auto readFile(const char* path, std::string& out) -> bool {
    std::FILE* f = std::fopen(path, "r");
    if (f == nullptr) {
        return false;
    }
    char buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
        out.append(buf, n);
    }
    std::fclose(f);
    return true;
}

void addUnique(std::vector<std::string>& v, const std::string& s) {
    for (const auto& x : v) {
        if (x == s) {
            return;
        }
    }
    v.push_back(s);
}

// Sum rate fields per topic across logs: each log is one process, and the system-wide rate of
// a topic is what all its endpoints saw combined. On the recv side that is a DELIVERY rate which
// scales with subscriber count (see the file header), deliberate, and consistent with issue #1.
void mergeStat(std::vector<sTopicStat>& stats, const sTopicStat& add) {
    for (auto& s : stats) {
        if (s.topic == add.topic) {
            s.pub_inter_hz += add.pub_inter_hz;
            s.pub_intra_hz += add.pub_intra_hz;
            s.recv_inter_hz += add.recv_inter_hz;
            s.recv_intra_hz += add.recv_intra_hz;
            s.recv_endpoint_seen = s.recv_endpoint_seen || add.recv_endpoint_seen;
            // Gap fields are MAX, not sum. "Some endpoint of this topic saw a gap this big" is
            // the alertable statement; adding two logs' gaps would be meaningless. This also
            // behaves better than the rate fields across logs, which sum to K x the publish rate
            // for K subscriber processes.
            if (add.has_pub_max_dt) {
                s.pub_max_dt_ms = s.has_pub_max_dt
                                      ? (add.pub_max_dt_ms > s.pub_max_dt_ms ? add.pub_max_dt_ms
                                                                             : s.pub_max_dt_ms)
                                      : add.pub_max_dt_ms;
                s.has_pub_max_dt = true;
            }
            if (add.has_recv_max_dt) {
                s.recv_max_dt_ms = s.has_recv_max_dt
                                       ? (add.recv_max_dt_ms > s.recv_max_dt_ms
                                              ? add.recv_max_dt_ms
                                              : s.recv_max_dt_ms)
                                       : add.recv_max_dt_ms;
                s.has_recv_max_dt = true;
            }
            return;
        }
    }
    stats.push_back(add);
}

void usage(std::FILE* to) {
    std::fprintf(to,
                 "usage: pulse-check --spec <spec.yaml> [--skip-last] <log> [<log> ...]\n"
                 "  Checks the LAST window of each ros2_pulse log against an expected-rate\n"
                 "  spec. Prints one WARN line per violation.\n"
                 "  --skip-last  judge the second-to-last window instead. For logs from an\n"
                 "               EXITED stack: the final window is the atexit flush, a\n"
                 "               sub-period sliver whose rate is a one-sample estimate.\n"
                 "               Needs >=2 windows per log (>=3 to clear the ramp-up one).\n"
                 "  exit 0: all checks pass   1: violations   2: bad input\n");
}

}  // namespace

int main(int argc, char** argv) {
    const char* spec_path = nullptr;
    bool skip_last = false;
    std::vector<const char*> log_paths;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--spec") == 0 && i + 1 < argc) {
            spec_path = argv[++i];
        } else if (std::strcmp(argv[i], "--skip-last") == 0) {
            skip_last = true;
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            usage(stdout);
            return 0;
        } else {
            log_paths.push_back(argv[i]);
        }
    }
    if (spec_path == nullptr || log_paths.empty()) {
        usage(stderr);
        return 2;
    }

    std::string spec_text;
    std::string err;
    // Bounded, regular-files-only (shared with the probe): --spec /dev/zero must fail fast, not
    // hang the CI runner it was meant to gate.
    if (!readSpecFile(spec_path, spec_text, err)) {
        std::fprintf(stderr, "pulse-check: cannot read spec '%s': %s\n", spec_path, err.c_str());
        return 2;
    }
    auto spec = parseRateSpec(spec_text, err);
    if (!spec.has_value()) {
        std::fprintf(stderr, "pulse-check: invalid spec '%s': %s\n", spec_path, err.c_str());
        return 2;
    }

    std::vector<sTopicStat> stats;
    std::vector<std::string> active_nodes;
    std::vector<std::string> known_nodes;
    for (const char* path : log_paths) {
        std::string text;
        if (!readFile(path, text)) {
            std::fprintf(stderr, "pulse-check: cannot read log '%s'\n", path);
            return 2;
        }
        const auto windows = parseLog(text);
        if (windows.empty()) {
            std::fprintf(stderr, "pulse-check: no probe windows in '%s'\n", path);
            return 2;
        }
        for (const auto& w : windows) {
            for (const auto& n : w.nodes) {
                addUnique(known_nodes, n);
            }
        }
        // Fail loudly rather than silently falling back to the ramp-up window, which the probe
        // grace-skips for the same reason --skip-last exists.
        if (skip_last && windows.size() < 2) {
            std::fprintf(stderr,
                         "pulse-check: --skip-last needs >=2 windows in '%s' (found %zu)\n",
                         path, windows.size());
            return 2;
        }
        const auto& last = skip_last ? windows[windows.size() - 2] : windows.back();
        for (const auto& n : last.nodes) {
            addUnique(active_nodes, n);
        }
        for (const auto& s : last.stats) {
            mergeStat(stats, s);
        }
    }

    std::vector<std::string> unmeasured;
    const auto warnings = evaluateRateSpec(*spec, stats, active_nodes, known_nodes,
                                           /*missing_as_zero=*/true, &unmeasured);
    // A gap rule the logs cannot answer is bad input, not a verdict: exit 0 would claim it was
    // checked and healthy, exit 1 would send someone chasing a stall that was never measured.
    // Print every violation we DID measure first. An unmeasurable gap rule on one topic says
    // nothing about a measured failure on another, and swallowing those would report a broken
    // stack as "bad input", someone fixes the env var, reruns, and only then finds the fault.
    for (const auto& w : warnings) {
        std::printf("%s\n", w.c_str());
    }
    if (!unmeasured.empty()) {
        for (const auto& t : unmeasured) {
            std::fprintf(stderr,
                         "pulse-check: spec sets max_gap_ms for '%s' but the logs carry no "
                         "JITTER line for it, was the probe run with "
                         "ROS_TOPIC_STATS_JITTER=1?\n",
                         t.c_str());
        }
        return 2;  // the verdict is incomplete either way, so bad-input dominates
    }
    return warnings.empty() ? 0 : 1;
}

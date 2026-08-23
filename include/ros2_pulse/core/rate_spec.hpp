// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef ROS2_PULSE__CORE__RATE_SPEC_HPP_
#define ROS2_PULSE__CORE__RATE_SPEC_HPP_

#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"

namespace ros2_pulse::core {

/// Which endpoint of a topic a rate rule constrains.
enum class eRateSide { kRecv, kPub };

/// Which transport's rate a rule constrains. kAny is the topic's rate however it travels: recv
/// sums the two (disjoint) buckets, pub takes the larger, one publish() can fire both the
/// intra-process and the RMW tracepoint for the same message (iron+), so summing would double it.
enum class eRateTransport { kAny, kInter, kIntra };

/// One expected-rate rule for a topic (ROADMAP R1).
struct sRateRule {
    double min_hz{0.0};
    double max_hz{std::numeric_limits<double>::infinity()};
    eRateSide side{eRateSide::kRecv};
    eRateTransport transport{eRateTransport::kAny};
    /// Largest tolerated inter-arrival gap, milliseconds; infinity means unconstrained. This is
    /// the detector a windowed mean cannot be: at 50 Hz a `min_hz: 45` rule needs >0.5 s of dead
    /// time to fire, so a 400 ms freeze reports 46 Hz and passes. Requires gap tracking
    /// (ROADMAP R5), a spec carrying this key turns it on.
    double max_gap_ms{std::numeric_limits<double>::infinity()};
};

/// A parsed expected-rate spec: per-topic rules plus a list of nodes expected alive.
/// Topics keep spec order so WARN output is deterministic.
struct sRateSpec {
    std::vector<std::pair<std::string, sRateRule>> topics;
    std::vector<std::string> nodes;
};

/// @brief Parse the restricted-YAML expected-rate spec (ROS_TOPIC_STATS_EXPECTED).
///
/// Accepted grammar, a deliberate, documented YAML subset so the LD_PRELOAD probe needs no
/// YAML library (see README "Expected-rate alerting"):
/// @code
/// # comments and blank lines anywhere
/// topics:
///   /scan:   {min_hz: 18, max_hz: 22, side: recv}
///   /points: {min_hz: 25, transport: intra}
/// nodes: [/perception, /planner]     # flow list...
/// nodes:                             # ...or block list
///   - /perception
/// @endcode
///
/// A leading UTF-8 BOM is ignored (Unicode 23.8.1) and CRLF line endings are accepted, so a spec
/// authored on Windows parses unchanged.
///
/// Topic entries must use the inline flow-map form with at least one of min_hz / max_hz.
/// Keys: min_hz, max_hz, max_gap_ms (non-negative finite numbers; max_gap_ms must be > 0),
/// side (pub|recv, default recv), transport (inter|intra|any, default any). A rule must carry at
/// least one of min_hz / max_hz / max_gap_ms. Unknown keys, malformed numbers, min_hz > max_hz,
/// a key repeated within one rule, or two rules measuring the same thing (same topic + side +
/// transport) are hard errors. A topic MAY appear more than once when the rules constrain
/// different endpoints, "publishes at 20 Hz and we receive it at 20 Hz" is one spec.
///
/// @param text  the spec file contents.
/// @param error on failure, receives "line N: <reason>".
/// @return the parsed spec, or std::nullopt on any error (the probe then runs unspecced,
///         a bad spec must never take the host process down).
auto parseRateSpec(const std::string& text, std::string& error) -> std::optional<sRateSpec>;

/// Upper bound on a spec file. A spec with hundreds of topics is a few tens of KB; anything past
/// this is not a spec, so it is refused rather than read.
constexpr size_t kMaxSpecBytes = 1u << 20;  // 1 MiB

/// @brief Slurp a spec file into @p out, bounded. Returns false and sets @p error on refusal.
///
/// Refuses, before reading a byte, anything that is not a regular file under @ref kMaxSpecBytes.
/// The probe loads specs inside a tracepoint-reached constructor, so an operator typo in
/// ROS_TOPIC_STATS_EXPECTED must not be able to stall or balloon the host process:
///   - a FIFO would block forever in open(), with O_NONBLOCK it is rejected instead;
///   - a directory opens fine but reads EISDIR, yielding empty text that parses as a valid
///     zero-rule spec (alerting silently armed as a permanent no-op);
///   - /dev/urandom and /dev/zero never signal EOF, so an unbounded read loop never terminates;
///   - a mistyped rosbag path (run1.bag vs run1.yaml) costs seconds of startup and GBs of RSS
///     in EVERY preloaded process before the parse rejects it.
/// This bounds RSS and startup latency. It is deliberately NOT a no-throw guarantee, it fills a
/// std::string, and parseRateSpec allocates too; the surrounding ctor already can throw.
///
/// Shared verbatim by the probe and the pulse-check CLI, so `pulse-check --spec /dev/zero` in a
/// CI job fails fast instead of hanging the runner.
auto readSpecFile(const char* path, std::string& out, std::string& error) -> bool;

/// @brief Evaluate a spec against one window's stats; returns WARN lines (no trailing '\n').
///
/// Line grammar (additive to the window format, pinned by unit tests):
/// @code
/// WARN TOPIC <name> hz=<observed to 6dp> expected=[<min>,<max>]   // max renders 'inf' when unbounded
/// WARN NODE <name> missing
/// @endcode
///
/// A spec topic with no entry in @p stats, or a spec node absent from @p known_nodes, belongs
/// to some OTHER process and is skipped, unless @p missing_as_zero is set (pulse-check mode,
/// where the log set IS the whole picture): then a missing topic is evaluated at 0 Hz and a
/// missing node is warned about. Node warnings fire for known-but-inactive nodes (aged out by
/// the liveness window).
///
/// @p unmeasured_gaps (offline mode only) collects topics whose rule carries max_gap_ms but
/// whose logs have no JITTER line for the required side. That is a measurement gap, not a health
/// verdict, so pulse-check reports it as bad input rather than passing or failing the check.
///
/// Pure and I/O-free: the probe calls it at flush time only (zero hot-path cost) and the
/// pulse-check CLI reuses it verbatim on offline logs.
auto evaluateRateSpec(const sRateSpec& spec, const std::vector<sTopicStat>& stats,
                      const std::vector<std::string>& active_nodes,
                      const std::vector<std::string>& known_nodes,
                      bool missing_as_zero = false,
                      std::vector<std::string>* unmeasured_gaps = nullptr)
    -> std::vector<std::string>;

/// What a single warning is about (ROADMAP R6: the jsonl emitter needs warnings as DATA, not
/// preformatted strings, a sidecar exporter should never have to regex our own WARN lines).
enum class eWarnKind {
    kTopicRate,    ///< observed hz outside [min_hz, max_hz]
    kTopicGap,     ///< observed max inter-arrival gap above max_gap_ms
    kNodeMissing,  ///< expected node not in the active set
};

/// One structured warning. Carries every number that the text WARN line renders, so the text
/// form is a pure projection of this struct (renderWarnLine below) and the jsonl form can emit
/// real JSON numbers instead of quoting a preformatted sentence. Only the fields meaningful for
/// @c kind are populated; the others keep their zero/infinity defaults:
///   kTopicRate:   name (topic), hz, min_hz, max_hz (infinity = unbounded above)
///   kTopicGap:    name (topic), max_dt_ms, max_gap_ms
///   kNodeMissing: name (node)
struct sRateWarning {
    eWarnKind kind{eWarnKind::kTopicRate};
    std::string name;
    double hz{0.0};
    double min_hz{0.0};
    double max_hz{std::numeric_limits<double>::infinity()};
    double max_dt_ms{0.0};
    double max_gap_ms{std::numeric_limits<double>::infinity()};
};

/// @brief Structured twin of evaluateRateSpec: same evaluation, same order, same semantics,
/// but returns data instead of rendered lines. evaluateRateSpec is a thin wrapper that maps
/// renderWarnLine over this result, so the two can never disagree.
auto evaluateRateSpecWarnings(const sRateSpec& spec, const std::vector<sTopicStat>& stats,
                              const std::vector<std::string>& active_nodes,
                              const std::vector<std::string>& known_nodes,
                              bool missing_as_zero = false,
                              std::vector<std::string>* unmeasured_gaps = nullptr)
    -> std::vector<sRateWarning>;

/// @brief Render one structured warning as the exact text WARN line (no trailing '\n') the
/// probe has always emitted, grammar pinned by the R1 unit tests:
/// @code
/// WARN TOPIC <name> hz=<%.6f> expected=[<min>,<max>]      // bounds via %g, 'inf' when unbounded
/// WARN TOPIC <name> max_dt_ms=<%.3f> expected_max_gap_ms=<%g>
/// WARN NODE <name> missing
/// @endcode
auto renderWarnLine(const sRateWarning& warn) -> std::string;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__RATE_SPEC_HPP_

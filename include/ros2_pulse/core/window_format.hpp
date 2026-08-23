// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ROS2_PULSE__CORE__WINDOW_FORMAT_HPP_
#define ROS2_PULSE__CORE__WINDOW_FORMAT_HPP_

#include <string>
#include <vector>

#include "ros2_pulse/core/rate_spec.hpp"
#include "ros2_pulse/core/topic_registry.hpp"

namespace ros2_pulse::core {

/// @brief Render one flush window as a single text block, byte-compatible with the on-disk format.
///
/// Layout (one trailing blank line closes the block):
/// @code
/// # ts_ns=<ts_ns> window_s=<window_s to 3dp>
/// TOPIC <topic> <pub_inter_hz to 6dp>          // when TopicRegistry::shouldEmitTopic(stat, emit_idle)
/// RECV <topic> inter=<recv_inter_hz to 6dp> intra=<recv_intra_hz to 6dp>  // when any recv count>0
/// NODE <node>
/// WARN ...                                     // one line per entry of @p warnings, verbatim
/// <blank line>
/// @endcode
///
/// @p emit_idle is the ROS_PULSE_EMIT_IDLE opt-in: fully-idle topics are suppressed unless true
/// (see KNOWN_ISSUES.md #7).
///
/// @p warnings are pre-rendered alert lines (see evaluateRateSpec in rate_spec.hpp, ROADMAP R1);
/// they are emitted verbatim after the NODE lines so existing TOPIC/PUB/RECV/NODE parsers are
/// unaffected. Empty by default, the output is then byte-identical to the pre-R1 format.
///
/// Pure (no ROS / tracetools / I/O dependency) so the probe can build the whole window once and
/// write it with a single call, and so the exact format is unit-testable.
auto formatWindow(const std::vector<sTopicStat>& stats, const std::vector<std::string>& nodes,
                  long long ts_ns, double window_s, bool emit_idle,
                  const std::vector<std::string>& warnings = {}) -> std::string;

/// @brief Render one flush window as ONE JSON Lines record (ROS_TOPIC_STATS_FORMAT=jsonl,
/// ROADMAP R6): a single JSON object on a single line, '\n'-terminated, UTF-8, per
/// jsonlines.org, so `tail -f | jq` and every log shipper's jsonl input work unmodified.
///
/// Shape (exact bytes pinned by test_window_format_jsonl.cpp golden tests):
/// @code
/// {"ts_ns":"<int64>","window_s":W.WWW,
///  "topics":[{"topic":"/x","pub_inter_hz":N,"pub_intra_hz":N,
///             "recv_inter_hz":N,"recv_intra_hz":N,"recv_endpoint_seen":true,
///             "pub_max_dt_ms":N,"recv_max_dt_ms":N}, ...],
///  "nodes":["/a", ...],
///  "warns":[{"kind":"topic_rate","topic":"/x","hz":N,"min_hz":N,"max_hz":N},
///           {"kind":"topic_gap","topic":"/x","max_dt_ms":N,"max_gap_ms":N},
///           {"kind":"node_missing","node":"/a"}, ...]}
/// @endcode
///
/// Design decisions, each mirrored from the text format or from exporter prior art:
///  - `ts_ns` is a decimal STRING, not a JSON number: epoch nanoseconds (~1.8e18) exceed
///    2^53-1, so a number would silently lose the low digits in every IEEE-754-double consumer
///    (JavaScript, jq). OTLP/JSON encodes its (u)int64 fields, timeUnixNano included, as
///    decimal strings for exactly this reason (RFC 8259 §6 interop note); we follow it.
///  - Same emit gates as the text format (shouldEmitTopic / pub_intra_count / shouldEmitRecv /
///    has_*_max_dt): jsonl is a re-encoding of the same measurement, never a different one. A
///    topic none of the gates would print is omitted here too.
///  - ABSENCE means "not measured", never 0: pub_* / recv_* / *_max_dt_ms keys appear only when
///    the matching text line would, a Humble probe never claims `pub_intra_hz` it cannot see
///    stalls behind, and a gap that was not tracked is not a gap of zero.
///  - `topics` / `nodes` / `warns` are ALWAYS present (empty arrays when empty): a sidecar's
///    `.warns[]` must not need null guards, and "checked, none" is a different statement from
///    "not carried by this record". Absence is reserved for optional per-topic measurements.
///  - Rates keep the text precisions (%.6f Hz, %.3f ms/s): golden-byte testable, and a
///    text↔jsonl diff of one window differs only in structure, never in value.
///  - `warns` carries STRUCTURED warnings (numbers as JSON numbers); the text WARN line is
///    reconstructible via renderWarnLine, and an unbounded max_hz is expressed by omitting the
///    key (JSON has no Infinity literal, RFC 8259 forbids it).
///  - Topic/node names come from USER code: escaped per RFC 8259 (quote, backslash, control
///    chars) so a hostile name can never break the one-object-per-line framing.
///
/// Pure and I/O-free, exactly like formatWindow: the probe builds the record once and writes it
/// with a single call, and the exact bytes stay unit-testable.
auto formatWindowJsonl(const std::vector<sTopicStat>& stats, const std::vector<std::string>& nodes,
                       long long ts_ns, double window_s, bool emit_idle,
                       const std::vector<sRateWarning>& warnings = {}) -> std::string;

/// @brief Per-process default output path: `$TMPDIR/topic_freq.<pid>.log`, `/tmp` fallback.
///
/// Embedding the pid stops every LD_PRELOADed process from appending to one shared file. Kept as
/// a pure function of its arguments so the derivation is testable without spawning a process,
/// the probe passes `getenv("TMPDIR")` as @p tmpdir. Only an absolute @p tmpdir is honoured
/// (trailing slashes normalized); anything else falls back to `/tmp`, because the probe runs
/// inside arbitrary processes whose cwd is unknown.
auto defaultOutputPath(long pid, const char* tmpdir = nullptr) -> std::string;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__WINDOW_FORMAT_HPP_

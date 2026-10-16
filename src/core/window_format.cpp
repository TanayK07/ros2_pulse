// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "ros2_pulse/core/window_format.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace ros2_pulse::core {

namespace {

// Format a single value with a printf format into a std::string. Keeps the on-disk output
// byte-identical to the previous fprintf-based emitter (same conversions, same precision).
template <typename T>
auto sprintfStr(const char* fmt, T value) -> std::string {
    char buf[64];
    const int n = std::snprintf(buf, sizeof(buf), fmt, value);
    if (n <= 0) {
        return {};
    }
    return std::string(buf, static_cast<size_t>(n));
}

}  // namespace

auto formatWindow(const std::vector<sTopicStat>& stats, const std::vector<std::string>& nodes,
                  long long ts_ns, double window_s, bool emit_idle,
                  const std::vector<std::string>& warnings,
                  const std::vector<sEndpointStat>* endpoints) -> std::string {
    std::string out;
    // Reserve a rough upper bound so the common window is built without reallocating.
    out.reserve(64 + stats.size() * 96 + nodes.size() * 32 + warnings.size() * 64);

    out += "# ts_ns=";
    out += sprintfStr("%lld", ts_ns);
    out += " window_s=";
    out += sprintfStr("%.3f", window_s);
    out += '\n';

    for (const auto& s : stats) {
        // Publish-side line, reading the genuine publish counter (split buckets, issue #1). The
        // emit decision, incl. the idle-topic suppression gated by emit_idle (issue #7), lives
        // in the core registry so it has one home and stays unit-testable.
        if (TopicRegistry::shouldEmitTopic(s, emit_idle)) {
            out += "TOPIC ";
            out += s.topic;
            out += ' ';
            out += sprintfStr("%.6f", s.pub_inter_hz);
            out += '\n';
        }
        // Publish-side intra line (iron+: the rclcpp_intra_publish tracepoint), ADDITIVE,
        // like RECV, so legacy TOPIC parsers stay valid. Emitted only when intra publishes
        // happened this window (impossible on humble: the tracepoint doesn't exist there).
        if (s.pub_intra_count > 0) {
            out += "PUB ";
            out += s.topic;
            out += " inter=";
            out += sprintfStr("%.6f", s.pub_inter_hz);
            out += " intra=";
            out += sprintfStr("%.6f", s.pub_intra_hz);
            out += '\n';
        }
        // Publish-side gap line (ROADMAP R5), only when gap tracking is on and this endpoint has
        // ever published. A NEW line kind rather than fields appended to TOPIC/PUB/RECV: several
        // in-tree parsers anchor those with '$', so appending would break them silently.
        // Milliseconds at 3dp, not the %.6f Hz convention, 1 us is the physical floor here
        // (the clock read itself is ~21 ns and scheduler noise is microseconds), and %.6f would
        // print three guaranteed-zero bytes on every stalled line.
        if (s.has_pub_max_dt) {
            out += "JITTER ";
            out += s.topic;
            out += " pub max_dt_ms=";
            out += sprintfStr("%.3f", s.pub_max_dt_ms);
            out += '\n';
        }
        // Loaned subset of the publish total (rclcpp#3153 follow-up), a NEW additive line kind
        // for the same '$'-anchor reason as JITTER, absent when no loan happened this window so
        // every loan-free log stays byte-identical.
        if (s.pub_loaned_count > 0) {
            out += "LOAN ";
            out += s.topic;
            out += " pub hz=";
            out += sprintfStr("%.6f", s.pub_loaned_hz);
            out += '\n';
        }
        // Additive receive-side line incl. intra-process, independent of the publish counter so
        // a same-process pub+sub is not double-counted. Proven receive endpoints emit an
        // explicit zero line when idle, stall visibility (KNOWN_ISSUES #12).
        if (TopicRegistry::shouldEmitRecv(s)) {
            out += "RECV ";
            out += s.topic;
            out += " inter=";
            out += sprintfStr("%.6f", s.recv_inter_hz);
            out += " intra=";
            out += sprintfStr("%.6f", s.recv_intra_hz);
            out += '\n';
        }
        if (s.has_recv_max_dt) {
            out += "JITTER ";
            out += s.topic;
            out += " recv max_dt_ms=";
            out += sprintfStr("%.3f", s.recv_max_dt_ms);
            out += '\n';
        }
        if (s.recv_loaned_count > 0) {
            out += "LOAN ";
            out += s.topic;
            out += " recv hz=";
            out += sprintfStr("%.6f", s.recv_loaned_hz);
            out += '\n';
        }
    }

    for (const auto& n : nodes) {
        out += "NODE ";
        out += n;
        out += '\n';
    }

    // Expected-rate warnings (ROADMAP R1), pre-rendered by evaluateRateSpec. After the NODE
    // lines so every pre-R1 line kind keeps its position in the block.
    for (const auto& w : warnings) {
        out += w;
        out += '\n';
    }

    // Per-endpoint split (opt-in in text). LAST in the block so every older line kind keeps its
    // position, and a NEW kind whose first token prefixes no older one: the in-tree parsers
    // sscanf "NODE %s" etc., where the space directive matches zero whitespace, so a kind like
    // "NODEX" would have parsed as a NODE line. Fields after the side are key=value, so the
    // optional node / max_dt_ms / loaned_hz never shift a positional reader.
    if (endpoints != nullptr) {
        for (const auto& e : *endpoints) {
            if (!TopicRegistry::shouldEmitEndpoint(e, emit_idle)) {
                continue;
            }
            out += "EP ";
            out += e.topic;
            out += ' ';
            out += endpointSideName(e.side);
            out += " id=";
            out += std::to_string(e.id);
            if (!e.node.empty()) {
                out += " node=";
                out += e.node;
            }
            out += " inter=";
            out += sprintfStr("%.6f", e.inter_hz);
            out += " intra=";
            out += sprintfStr("%.6f", e.intra_hz);
            if (e.has_max_dt) {
                out += " max_dt_ms=";
                out += sprintfStr("%.3f", e.max_dt_ms);
            }
            if (e.loaned_count > 0) {
                out += " loaned_hz=";
                out += sprintfStr("%.6f", e.loaned_hz);
            }
            out += '\n';
        }
    }

    out += '\n';  // blank line closes the window block
    return out;
}

namespace {

// RFC 8259 §7 string escaping for USER-CONTROLLED names (topics and nodes come from host
// code): the two mandatory escapes (quote, backslash), the conventional two-char forms for the
// common control characters, and \u00XX for the rest of C0. Everything >= 0x20 passes through
// verbatim, jsonl is UTF-8 and multi-byte sequences need no escaping. Escaping '\n' is what
// makes the one-object-per-LINE framing unbreakable by a hostile topic name.
void appendJsonEscaped(std::string& out, const std::string& s) {
    for (const char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x",
                                  static_cast<unsigned>(static_cast<unsigned char>(c)));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
}

// Spec bounds render with %g, the same conversion formatBound() uses for the text WARN line
// ("expected=[18,22]"), so a bound reads identically in both formats.
void appendJsonBound(std::string& out, double v) {
    out += sprintfStr("%g", v);
}

}  // namespace

auto formatWindowJsonl(const std::vector<sTopicStat>& stats, const std::vector<std::string>& nodes,
                       long long ts_ns, double window_s, bool emit_idle,
                       const std::vector<sRateWarning>& warnings,
                       const std::vector<sEndpointStat>* endpoints, bool partial) -> std::string {
    std::string out;
    // Rough upper bound (fields are wider than their text twins), same single-write goal.
    out.reserve(96 + stats.size() * 192 + nodes.size() * 40 + warnings.size() * 112);

    // ts_ns as a decimal STRING: ~1.8e18 exceeds 2^53-1, so as a JSON number every IEEE-754-
    // double consumer (JavaScript, jq) silently corrupts the low digits. OTLP/JSON encodes
    // timeUnixNano, and every (u)int64, as a decimal string for exactly this reason
    // (RFC 8259 §6 interop note); we follow that precedent rather than invent our own.
    out += "{\"ts_ns\":\"";
    out += sprintfStr("%lld", ts_ns);
    // window_s / Hz / ms keep the text precisions (%.3f / %.6f / %.3f): the record stays
    // golden-byte testable and a text↔jsonl diff differs in structure only, never in value.
    out += "\",\"window_s\":";
    out += sprintfStr("%.3f", window_s);
    // Attach / exit window (see the header): absent, not false, on a full one, so every
    // existing record stays byte-identical.
    if (partial) {
        out += ",\"partial\":true";
    }

    out += ",\"topics\":[";
    bool first = true;
    for (const auto& s : stats) {
        // Identical emit gates to the text format (one home: TopicRegistry): jsonl is a
        // re-encoding of the same measurement, so a topic no text line would carry is omitted
        // here too, and the idle-topic policy (issue #7, ROS_PULSE_EMIT_IDLE) is inherited.
        const bool pub_side =
            TopicRegistry::shouldEmitTopic(s, emit_idle) || s.pub_intra_count > 0;
        const bool recv_side = TopicRegistry::shouldEmitRecv(s);
        const bool loaned = s.pub_loaned_count > 0 || s.recv_loaned_count > 0;
        if (!pub_side && !recv_side && !s.has_pub_max_dt && !s.has_recv_max_dt && !loaned &&
            !s.has_recv_dt && !s.has_recv_age) {
            continue;
        }
        if (!first) {
            out += ',';
        }
        first = false;
        out += "{\"topic\":\"";
        appendJsonEscaped(out, s.topic);
        out += '"';
        // Key ABSENCE means "not measured", never 0, the same statement the text format makes
        // by not printing a line. The pub pair covers both TOPIC and the additive PUB line
        // (unlike text, intra is included even at 0.0: "no intra publishes observed" is a
        // genuine count, and a fixed pair keeps the pub schema one shape).
        if (pub_side) {
            out += ",\"pub_inter_hz\":";
            out += sprintfStr("%.6f", s.pub_inter_hz);
            out += ",\"pub_intra_hz\":";
            out += sprintfStr("%.6f", s.pub_intra_hz);
        }
        if (recv_side) {
            out += ",\"recv_inter_hz\":";
            out += sprintfStr("%.6f", s.recv_inter_hz);
            out += ",\"recv_intra_hz\":";
            out += sprintfStr("%.6f", s.recv_intra_hz);
            // Constant true under today's gate (traffic implies a proven endpoint), but kept
            // as an explicit key so the recv group stays self-describing if the gate ever
            // loosens (an EMIT_IDLE-style opt-in for never-active subscriptions would carry
            // false here), additive evolution instead of a schema break.
            out += ",\"recv_endpoint_seen\":true";
        }
        // Mirrors has_pub_max_dt / has_recv_max_dt exactly (R5): a probe without gap tracking
        // must not report a gap of 0.0 ms, that claims "perfectly smooth", which is how a
        // stall would hide from a max_gap_ms sidecar rule.
        if (s.has_pub_max_dt) {
            out += ",\"pub_max_dt_ms\":";
            out += sprintfStr("%.3f", s.pub_max_dt_ms);
        }
        if (s.has_recv_max_dt) {
            out += ",\"recv_max_dt_ms\":";
            out += sprintfStr("%.3f", s.recv_max_dt_ms);
        }
        // Receive-side distributions (jsonl only, the text JITTER line keeps its '$'-anchored
        // shape): what pulse_bridge needs for the MINIMUM / STDDEV of message_period and for
        // message_age. Same absence rule: no interval or no source timestamp, no key.
        if (s.has_recv_dt) {
            out += ",\"recv_min_dt_ms\":";
            out += sprintfStr("%.3f", s.recv_dt.min_ms);
            out += ",\"recv_stddev_dt_ms\":";
            out += sprintfStr("%.3f", s.recv_dt.stddev_ms);
        }
        if (s.has_recv_age) {
            out += ",\"recv_age_avg_ms\":";
            out += sprintfStr("%.3f", s.recv_age.mean_ms);
            out += ",\"recv_age_min_ms\":";
            out += sprintfStr("%.3f", s.recv_age.min_ms);
            out += ",\"recv_age_max_ms\":";
            out += sprintfStr("%.3f", s.recv_age.max_ms);
            if (s.recv_age.has_stddev) {  // absent, not 0, when the window's squares overflowed
                out += ",\"recv_age_stddev_ms\":";
                out += sprintfStr("%.3f", s.recv_age.stddev_ms);
            }
            out += ",\"recv_age_count\":";
            out += sprintfStr("%llu", static_cast<unsigned long long>(s.recv_age.count));
        }
        // Loaned subsets (rclcpp#3153 follow-up): absent when zero, like the text LOAN lines.
        // A subset of pub_inter_hz / the recv total, never added to them.
        if (s.pub_loaned_count > 0) {
            out += ",\"pub_loaned_hz\":";
            out += sprintfStr("%.6f", s.pub_loaned_hz);
        }
        if (s.recv_loaned_count > 0) {
            out += ",\"recv_loaned_hz\":";
            out += sprintfStr("%.6f", s.recv_loaned_hz);
        }
        out += '}';
    }

    // topics/nodes/warns are ALWAYS present (empty arrays when empty): ".warns[]" in a sidecar
    // must not need null guards, and "checked, none" is a different statement from "this record
    // doesn't carry warns". Absence stays reserved for unmeasured per-topic fields.
    out += "],\"nodes\":[";
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i > 0) {
            out += ',';
        }
        out += '"';
        appendJsonEscaped(out, nodes[i]);
        out += '"';
    }

    out += "],\"warns\":[";
    for (size_t i = 0; i < warnings.size(); ++i) {
        const auto& w = warnings[i];
        if (i > 0) {
            out += ',';
        }
        // Structured, not our preformatted sentence: a sidecar thresholds on .hz directly and
        // the text line stays reconstructible via renderWarnLine. Numbers use the same
        // conversions as that line (%.6f hz, %.3f gap, %g bounds), so the two never disagree.
        switch (w.kind) {
            case eWarnKind::kTopicRate:
                out += "{\"kind\":\"topic_rate\",\"topic\":\"";
                appendJsonEscaped(out, w.name);
                out += "\",\"hz\":";
                out += sprintfStr("%.6f", w.hz);
                out += ",\"min_hz\":";
                appendJsonBound(out, w.min_hz);
                // RFC 8259 has no Infinity literal; an unbounded max OMITS the key,
                // absence-means-unbounded, consistent with the gap-field absence semantics.
                if (!std::isinf(w.max_hz)) {
                    out += ",\"max_hz\":";
                    appendJsonBound(out, w.max_hz);
                }
                out += '}';
                break;
            case eWarnKind::kTopicGap:
                out += "{\"kind\":\"topic_gap\",\"topic\":\"";
                appendJsonEscaped(out, w.name);
                out += "\",\"max_dt_ms\":";
                out += sprintfStr("%.3f", w.max_dt_ms);
                out += ",\"max_gap_ms\":";
                appendJsonBound(out, w.max_gap_ms);
                out += '}';
                break;
            case eWarnKind::kNodeMissing:
                out += "{\"kind\":\"node_missing\",\"node\":\"";
                appendJsonEscaped(out, w.name);
                out += "\"}";
                break;
        }
    }

    out += ']';

    // Per-endpoint split of the topic sums. Present whenever the caller passes it (the probe
    // always does), so "endpoints":[] means "measured, nothing to report" and an absent key
    // means an older probe. Same gates and absence rules as the topic entries.
    if (endpoints != nullptr) {
        out += ",\"endpoints\":[";
        bool first_ep = true;
        for (const auto& e : *endpoints) {
            if (!TopicRegistry::shouldEmitEndpoint(e, emit_idle)) {
                continue;
            }
            if (!first_ep) {
                out += ',';
            }
            first_ep = false;
            out += "{\"topic\":\"";
            appendJsonEscaped(out, e.topic);
            out += "\",\"side\":\"";
            out += endpointSideName(e.side);
            out += "\",\"id\":";
            out += std::to_string(e.id);
            // Absent when the owning node's rcl_node_init was never seen: "" would read as a
            // node with an empty name.
            if (!e.node.empty()) {
                out += ",\"node\":\"";
                appendJsonEscaped(out, e.node);
                out += '"';
            }
            out += ",\"inter_hz\":";
            out += sprintfStr("%.6f", e.inter_hz);
            out += ",\"intra_hz\":";
            out += sprintfStr("%.6f", e.intra_hz);
            if (e.has_max_dt) {
                out += ",\"max_dt_ms\":";
                out += sprintfStr("%.3f", e.max_dt_ms);
            }
            // This subscription's own distributions, the per-handle twins of the topic's
            // recv_*_dt / recv_age_* keys (jsonl only, like those). Same absence rule.
            if (e.has_recv_dt) {
                out += ",\"avg_dt_ms\":";
                out += sprintfStr("%.3f", e.recv_dt.mean_ms);
                out += ",\"min_dt_ms\":";
                out += sprintfStr("%.3f", e.recv_dt.min_ms);
                out += ",\"stddev_dt_ms\":";
                out += sprintfStr("%.3f", e.recv_dt.stddev_ms);
                out += ",\"dt_count\":";
                out += sprintfStr("%llu", static_cast<unsigned long long>(e.recv_dt.count));
            }
            if (e.has_recv_age) {
                out += ",\"age_avg_ms\":";
                out += sprintfStr("%.3f", e.recv_age.mean_ms);
                out += ",\"age_min_ms\":";
                out += sprintfStr("%.3f", e.recv_age.min_ms);
                out += ",\"age_max_ms\":";
                out += sprintfStr("%.3f", e.recv_age.max_ms);
                if (e.recv_age.has_stddev) {
                    out += ",\"age_stddev_ms\":";
                    out += sprintfStr("%.3f", e.recv_age.stddev_ms);
                }
                out += ",\"age_count\":";
                out += sprintfStr("%llu", static_cast<unsigned long long>(e.recv_age.count));
            }
            if (e.loaned_count > 0) {
                out += ",\"loaned_hz\":";
                out += sprintfStr("%.6f", e.loaned_hz);
            }
            out += '}';
        }
        out += ']';
    }

    out += "}\n";  // '\n' closes the record: one object per line (jsonlines.org)
    return out;
}

auto endpointSideName(eEndpointSide side) -> const char* {
    return side == eEndpointSide::kPub ? "pub" : "recv";
}

auto defaultOutputPath(long pid, const char* tmpdir) -> std::string {
    // POSIX default: honour TMPDIR, fall back to /tmp. The probe runs inside arbitrary
    // processes, so the only defensible default is a directory that exists and is writable on
    // an unconfigured machine. A relative TMPDIR is ignored rather than obeyed: cwd is unknown
    // (and often unwritable) inside a preloaded host, so a relative path would scatter logs,
    // or open-failures, across whatever directory each process happens to sit in.
    std::string dir = "/tmp";
    if (tmpdir != nullptr && tmpdir[0] == '/') {
        dir = tmpdir;
        while (dir.size() > 1 && dir.back() == '/') {
            dir.pop_back();  // "/scratch/" and "/scratch" must name the same file
        }
        if (dir == "/") {
            dir.clear();  // root: the '/' below provides the separator
        }
    }
    return dir + "/topic_freq." + std::to_string(pid) + ".log";
}

}  // namespace ros2_pulse::core

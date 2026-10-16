// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#include "ros2_pulse/core/log_reader.hpp"

#include <charconv>
#include <cstdio>
#include <string_view>
#include <system_error>

namespace ros2_pulse::core {

namespace {

// The topic named on a TOPIC/PUB/RECV line; sTopicStat entries merge per topic per window
// because one topic can carry up to three lines (TOPIC + PUB + RECV).
auto statFor(sLogWindow& win, const std::string& topic) -> sTopicStat& {
    for (auto& s : win.stats) {
        if (s.topic == topic) {
            return s;
        }
    }
    win.stats.emplace_back();
    win.stats.back().topic = topic;
    return win.stats.back();
}

// ---- restricted JSON parser for jsonl windows (ROADMAP R6) ----
//
// Hand-rolled for the same reason the spec parser is a hand-rolled YAML subset: no JSON
// library enters this codebase (pulse-check links only the pure core, and the core is also
// linked into an LD_PRELOAD probe). It is NOT a general JSON parser: it consumes the grammar
// formatWindowJsonl emits, tolerates unknown KEYS by skipping their (arbitrarily nested)
// values, so an older pulse-check keeps reading a newer probe's additive fields, and
// rejects the line on any malformation. Rejection is per-LINE: jsonl's framing guarantees a
// record never spans lines, so one truncated tail (crash mid-write) costs one window, exactly
// like a torn text block.

struct sCursor {
    const char* p;
    const char* end;
};

void skipWs(sCursor& c) {
    while (c.p != c.end && (*c.p == ' ' || *c.p == '\t' || *c.p == '\r' || *c.p == '\n')) {
        ++c.p;
    }
}

auto consume(sCursor& c, char ch) -> bool {
    skipWs(c);
    if (c.p != c.end && *c.p == ch) {
        ++c.p;
        return true;
    }
    return false;
}

auto parseHex4(sCursor& c, unsigned& v) -> bool {
    if (c.end - c.p < 4) {
        return false;
    }
    v = 0;
    for (int i = 0; i < 4; ++i) {
        const char h = *c.p++;
        v <<= 4u;
        if (h >= '0' && h <= '9') {
            v |= static_cast<unsigned>(h - '0');
        } else if (h >= 'a' && h <= 'f') {
            v |= static_cast<unsigned>(h - 'a' + 10);
        } else if (h >= 'A' && h <= 'F') {
            v |= static_cast<unsigned>(h - 'A' + 10);
        } else {
            return false;
        }
    }
    return true;
}

void appendUtf8(std::string& out, unsigned cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

// RFC 8259 §7 string, escapes included (\uXXXX with surrogate pairs, the emitter only writes
// \u00xx for C0 controls, but a foreign writer's record should still round-trip).
auto parseJsonString(sCursor& c, std::string& out) -> bool {
    if (!consume(c, '"')) {
        return false;
    }
    out.clear();
    while (c.p != c.end) {
        const char ch = *c.p++;
        if (ch == '"') {
            return true;
        }
        if (static_cast<unsigned char>(ch) < 0x20) {
            return false;  // raw control byte: invalid JSON, and our emitter escapes them
        }
        if (ch != '\\') {
            out += ch;
            continue;
        }
        if (c.p == c.end) {
            return false;
        }
        const char esc = *c.p++;
        switch (esc) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                unsigned cp = 0;
                if (!parseHex4(c, cp)) {
                    return false;
                }
                if (cp >= 0xD800 && cp <= 0xDBFF) {  // high surrogate: need a low one
                    if (c.end - c.p < 6 || c.p[0] != '\\' || c.p[1] != 'u') {
                        return false;
                    }
                    c.p += 2;
                    unsigned lo = 0;
                    if (!parseHex4(c, lo) || lo < 0xDC00 || lo > 0xDFFF) {
                        return false;
                    }
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
                    return false;  // lone low surrogate
                }
                appendUtf8(out, cp);
                break;
            }
            default:
                return false;
        }
    }
    return false;  // unterminated
}

auto parseJsonNumber(sCursor& c, double& out) -> bool {
    skipWs(c);
    const std::from_chars_result res = std::from_chars(c.p, c.end, out);
    if (res.ec != std::errc{}) {
        return false;
    }
    c.p = res.ptr;
    return true;
}

// Skip any well-formed JSON value: how unknown keys stay tolerated without being understood.
auto skipJsonValue(sCursor& c) -> bool {  // NOLINT(misc-no-recursion), nesting is user-bounded
    skipWs(c);
    if (c.p == c.end) {
        return false;
    }
    const char ch = *c.p;
    if (ch == '"') {
        std::string sink;
        return parseJsonString(c, sink);
    }
    if (ch == '{' || ch == '[') {
        const char close = ch == '{' ? '}' : ']';
        ++c.p;
        if (consume(c, close)) {
            return true;  // empty container
        }
        while (true) {
            if (ch == '{') {  // object: "key": value
                std::string key;
                if (!parseJsonString(c, key) || !consume(c, ':')) {
                    return false;
                }
            }
            if (!skipJsonValue(c)) {
                return false;
            }
            if (consume(c, ',')) {
                continue;
            }
            return consume(c, close);
        }
    }
    if (ch == 't') {
        if (c.end - c.p >= 4 && std::string_view(c.p, 4) == "true") {
            c.p += 4;
            return true;
        }
        return false;
    }
    if (ch == 'f') {
        if (c.end - c.p >= 5 && std::string_view(c.p, 5) == "false") {
            c.p += 5;
            return true;
        }
        return false;
    }
    if (ch == 'n') {
        if (c.end - c.p >= 4 && std::string_view(c.p, 4) == "null") {
            c.p += 4;
            return true;
        }
        return false;
    }
    double sink = 0.0;
    return parseJsonNumber(c, sink);
}

auto parseJsonBool(sCursor& c, bool& out) -> bool {
    skipWs(c);
    if (c.end - c.p >= 4 && std::string_view(c.p, 4) == "true") {
        c.p += 4;
        out = true;
        return true;
    }
    if (c.end - c.p >= 5 && std::string_view(c.p, 5) == "false") {
        c.p += 5;
        out = false;
        return true;
    }
    return false;
}

// One element of "topics": known keys land in the stat (gap keys also set the has_* flag,
// PRESENCE is the measurement marker, mirroring the emitter's absence-means-unmeasured rule),
// unknown keys are skipped.
auto parseTopicObject(sCursor& c, sLogWindow& win) -> bool {
    if (!consume(c, '{')) {
        return false;
    }
    sTopicStat tmp;
    bool have_name = false;
    bool have_age_stddev = false;
    if (!consume(c, '}')) {
        while (true) {
            std::string key;
            if (!parseJsonString(c, key) || !consume(c, ':')) {
                return false;
            }
            if (key == "topic") {
                if (!parseJsonString(c, tmp.topic)) {
                    return false;
                }
                have_name = true;
            } else if (key == "pub_inter_hz") {
                if (!parseJsonNumber(c, tmp.pub_inter_hz)) return false;
            } else if (key == "pub_intra_hz") {
                if (!parseJsonNumber(c, tmp.pub_intra_hz)) return false;
            } else if (key == "recv_inter_hz") {
                if (!parseJsonNumber(c, tmp.recv_inter_hz)) return false;
            } else if (key == "recv_intra_hz") {
                if (!parseJsonNumber(c, tmp.recv_intra_hz)) return false;
            } else if (key == "recv_endpoint_seen") {
                if (!parseJsonBool(c, tmp.recv_endpoint_seen)) return false;
            } else if (key == "pub_max_dt_ms") {
                if (!parseJsonNumber(c, tmp.pub_max_dt_ms)) return false;
                tmp.has_pub_max_dt = true;
            } else if (key == "recv_max_dt_ms") {
                if (!parseJsonNumber(c, tmp.recv_max_dt_ms)) return false;
                tmp.has_recv_max_dt = true;
            } else if (key == "recv_min_dt_ms") {
                if (!parseJsonNumber(c, tmp.recv_dt.min_ms)) return false;
                tmp.has_recv_dt = true;
            } else if (key == "recv_stddev_dt_ms") {
                if (!parseJsonNumber(c, tmp.recv_dt.stddev_ms)) return false;
                tmp.has_recv_dt = true;
            } else if (key == "recv_age_avg_ms") {
                if (!parseJsonNumber(c, tmp.recv_age.mean_ms)) return false;
                tmp.has_recv_age = true;
            } else if (key == "recv_age_min_ms") {
                if (!parseJsonNumber(c, tmp.recv_age.min_ms)) return false;
            } else if (key == "recv_age_max_ms") {
                if (!parseJsonNumber(c, tmp.recv_age.max_ms)) return false;
            } else if (key == "recv_age_stddev_ms") {
                if (!parseJsonNumber(c, tmp.recv_age.stddev_ms)) return false;
                have_age_stddev = true;
            } else if (key == "recv_age_count") {
                double n = 0.0;
                if (!parseJsonNumber(c, n)) return false;
                tmp.recv_age.count = n > 0.0 ? static_cast<uint64_t>(n) : 0;
            } else if (key == "pub_loaned_hz") {
                if (!parseJsonNumber(c, tmp.pub_loaned_hz)) return false;
            } else if (key == "recv_loaned_hz") {
                if (!parseJsonNumber(c, tmp.recv_loaned_hz)) return false;
            } else if (!skipJsonValue(c)) {
                return false;
            }
            if (consume(c, ',')) {
                continue;
            }
            if (!consume(c, '}')) {
                return false;
            }
            break;
        }
    }
    // An age group without its deviation is one whose squares overflowed: unknown, not 0.
    tmp.recv_age.has_stddev = have_age_stddev;
    if (!have_name) {
        return false;  // a rates entry with no topic is not attributable to anything
    }
    // Merge via statFor like the text lines do, the emitter writes one entry per topic, but a
    // foreign/duplicated record should still coalesce rather than shadow.
    auto& s = statFor(win, tmp.topic);
    const std::string name = s.topic;
    s = tmp;
    s.topic = name;
    return true;
}

auto parseSide(const std::string& token, eEndpointSide& out) -> bool {
    if (token == "pub") {
        out = eEndpointSide::kPub;
        return true;
    }
    if (token == "recv") {
        out = eEndpointSide::kRecv;
        return true;
    }
    return false;
}

// One element of "endpoints". Known keys land in the stat (max_dt_ms / loaned_hz presence is the
// measurement marker, as for topics), unknown keys are skipped. An element with no topic or an
// unknown side is DROPPED, not fatal: a newer probe may grow a side this reader cannot name, and
// that must not cost the window its topic rates.
auto parseEndpointObject(sCursor& c, sLogWindow& win) -> bool {
    if (!consume(c, '{')) {
        return false;
    }
    sEndpointStat e;
    bool have_topic = false;
    bool have_side = false;
    bool have_age_stddev = false;
    if (!consume(c, '}')) {
        while (true) {
            std::string key;
            if (!parseJsonString(c, key) || !consume(c, ':')) {
                return false;
            }
            if (key == "topic") {
                if (!parseJsonString(c, e.topic)) return false;
                have_topic = true;
            } else if (key == "side") {
                std::string side;
                if (!parseJsonString(c, side)) return false;
                have_side = parseSide(side, e.side);
            } else if (key == "node") {
                if (!parseJsonString(c, e.node)) return false;
            } else if (key == "id") {
                double id = 0.0;
                if (!parseJsonNumber(c, id)) return false;
                e.id = id >= 0.0 && id <= 4294967295.0 ? static_cast<uint32_t>(id) : 0u;
            } else if (key == "inter_hz") {
                if (!parseJsonNumber(c, e.inter_hz)) return false;
            } else if (key == "intra_hz") {
                if (!parseJsonNumber(c, e.intra_hz)) return false;
            } else if (key == "max_dt_ms") {
                if (!parseJsonNumber(c, e.max_dt_ms)) return false;
                e.has_max_dt = true;
            } else if (key == "loaned_hz") {
                if (!parseJsonNumber(c, e.loaned_hz)) return false;
            } else if (key == "avg_dt_ms") {
                if (!parseJsonNumber(c, e.recv_dt.mean_ms)) return false;
                e.has_recv_dt = true;
            } else if (key == "min_dt_ms") {
                if (!parseJsonNumber(c, e.recv_dt.min_ms)) return false;
                e.has_recv_dt = true;
            } else if (key == "stddev_dt_ms") {
                if (!parseJsonNumber(c, e.recv_dt.stddev_ms)) return false;
            } else if (key == "dt_count") {
                double n = 0.0;
                if (!parseJsonNumber(c, n)) return false;
                e.recv_dt.count = n > 0.0 ? static_cast<uint64_t>(n) : 0;
            } else if (key == "age_avg_ms") {
                if (!parseJsonNumber(c, e.recv_age.mean_ms)) return false;
                e.has_recv_age = true;
            } else if (key == "age_min_ms") {
                if (!parseJsonNumber(c, e.recv_age.min_ms)) return false;
            } else if (key == "age_max_ms") {
                if (!parseJsonNumber(c, e.recv_age.max_ms)) return false;
            } else if (key == "age_stddev_ms") {
                if (!parseJsonNumber(c, e.recv_age.stddev_ms)) return false;
                have_age_stddev = true;
            } else if (key == "age_count") {
                double n = 0.0;
                if (!parseJsonNumber(c, n)) return false;
                e.recv_age.count = n > 0.0 ? static_cast<uint64_t>(n) : 0;
            } else if (!skipJsonValue(c)) {
                return false;
            }
            if (consume(c, ',')) {
                continue;
            }
            if (!consume(c, '}')) {
                return false;
            }
            break;
        }
    }
    e.recv_age.has_stddev = have_age_stddev;
    if (have_topic && have_side) {
        // A receive endpoint is only ever written once proven or active (shouldEmitEndpoint).
        e.endpoint_seen = e.side == eEndpointSide::kRecv;
        win.endpoints.push_back(std::move(e));
    }
    return true;
}

// Text EP line: "EP <topic> <side> id=<n> [node=<node>] inter=<hz> intra=<hz> [max_dt_ms=<ms>]
// [loaned_hz=<hz>]". key=value after the side, unknown keys ignored, so later additions parse.
// Returns false (line ignored) when the fixed part or a known value is malformed.
auto parseEpLine(const std::string& line, sEndpointStat& e) -> bool {
    std::vector<std::string> tok;
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && line[i] == ' ') ++i;
        const size_t j = line.find(' ', i);
        const size_t stop = j == std::string::npos ? line.size() : j;
        if (stop > i) {
            tok.emplace_back(line.substr(i, stop - i));
        }
        i = stop;
    }
    if (tok.size() < 3 || tok[0] != "EP" || !parseSide(tok[2], e.side)) {
        return false;
    }
    e.topic = tok[1];
    bool have_inter = false;
    for (size_t k = 3; k < tok.size(); ++k) {
        const auto eq = tok[k].find('=');
        if (eq == std::string::npos) {
            continue;
        }
        const std::string key = tok[k].substr(0, eq);
        const std::string val = tok[k].substr(eq + 1);
        const char* b = val.data();
        const char* end = val.data() + val.size();
        double d = 0.0;
        if (key == "node") {
            e.node = val;
            continue;
        }
        if (key != "id" && key != "inter" && key != "intra" && key != "max_dt_ms" &&
            key != "loaned_hz") {
            continue;
        }
        const auto res = std::from_chars(b, end, d);
        if (res.ec != std::errc{} || res.ptr != end) {
            return false;
        }
        if (key == "id") {
            e.id = d >= 0.0 && d <= 4294967295.0 ? static_cast<uint32_t>(d) : 0u;
        } else if (key == "inter") {
            e.inter_hz = d;
            have_inter = true;
        } else if (key == "intra") {
            e.intra_hz = d;
        } else if (key == "max_dt_ms") {
            e.max_dt_ms = d;
            e.has_max_dt = true;
        } else {
            e.loaned_hz = d;
        }
    }
    e.endpoint_seen = e.side == eEndpointSide::kRecv;
    return have_inter;
}

// One whole jsonl window record. Returns false on ANY malformation (including trailing bytes
// after the closing brace, the framing is one object per line, nothing else on it).
auto parseJsonlWindow(const std::string& line, sLogWindow& win) -> bool {
    sCursor c{line.data(), line.data() + line.size()};
    if (!consume(c, '{')) {
        return false;
    }
    // A probe window MUST carry its header fields, like the text `# ts_ns=... window_s=...`
    // line must scan completely: some other tool's jsonl piped into pulse-check should read
    // as "no probe windows" (exit 2), not as a stack of zero-rate windows judged at 0 Hz.
    bool have_ts = false;
    bool have_window = false;
    if (!consume(c, '}')) {
        while (true) {
            std::string key;
            if (!parseJsonString(c, key) || !consume(c, ':')) {
                return false;
            }
            if (key == "ts_ns") {
                // Emitted as a decimal string (int64 > 2^53, see formatWindowJsonl); a plain
                // JSON number is accepted too for foreign writers, at double precision.
                skipWs(c);
                if (c.p != c.end && *c.p == '"') {
                    std::string digits;
                    if (!parseJsonString(c, digits)) {
                        return false;
                    }
                    long long ts = 0;
                    const auto res =
                        std::from_chars(digits.data(), digits.data() + digits.size(), ts);
                    if (res.ec != std::errc{} || res.ptr != digits.data() + digits.size()) {
                        return false;
                    }
                    win.ts_ns = ts;
                } else {
                    double d = 0.0;
                    if (!parseJsonNumber(c, d)) {
                        return false;
                    }
                    win.ts_ns = static_cast<long long>(d);
                }
                have_ts = true;
            } else if (key == "window_s") {
                if (!parseJsonNumber(c, win.window_s)) {
                    return false;
                }
                have_window = true;
            } else if (key == "topics") {
                if (!consume(c, '[')) {
                    return false;
                }
                if (!consume(c, ']')) {
                    while (true) {
                        if (!parseTopicObject(c, win)) {
                            return false;
                        }
                        if (consume(c, ',')) {
                            continue;
                        }
                        if (!consume(c, ']')) {
                            return false;
                        }
                        break;
                    }
                }
            } else if (key == "endpoints") {
                if (!consume(c, '[')) {
                    return false;
                }
                if (!consume(c, ']')) {
                    while (true) {
                        if (!parseEndpointObject(c, win)) {
                            return false;
                        }
                        if (consume(c, ',')) {
                            continue;
                        }
                        if (!consume(c, ']')) {
                            return false;
                        }
                        break;
                    }
                }
            } else if (key == "nodes") {
                if (!consume(c, '[')) {
                    return false;
                }
                if (!consume(c, ']')) {
                    while (true) {
                        std::string node;
                        if (!parseJsonString(c, node)) {
                            return false;
                        }
                        win.nodes.push_back(std::move(node));
                        if (consume(c, ',')) {
                            continue;
                        }
                        if (!consume(c, ']')) {
                            return false;
                        }
                        break;
                    }
                }
            } else {
                // Every other top-level key, "warns" included, BY DESIGN, is skipped whole:
                // pulse-check re-derives verdicts from the raw rates, it never trusts
                // probe-side warnings (same stance as the text parser skipping WARN lines).
                if (!skipJsonValue(c)) {
                    return false;
                }
            }
            if (consume(c, ',')) {
                continue;
            }
            if (!consume(c, '}')) {
                return false;
            }
            break;
        }
    }
    skipWs(c);
    return c.p == c.end && have_ts && have_window;
}

}  // namespace

auto parseLog(const std::string& text) -> std::vector<sLogWindow> {
    std::vector<sLogWindow> windows;
    sLogWindow* cur = nullptr;
    char name[512];
    size_t pos = 0;
    while (pos <= text.size()) {
        const auto nl = text.find('\n', pos);
        const std::string line =
            text.substr(pos, (nl == std::string::npos ? text.size() : nl) - pos);
        pos = nl == std::string::npos ? text.size() + 1 : nl + 1;
        if (line.empty()) {
            continue;
        }

        // jsonl sniff (ROADMAP R6): a '{' first byte can only be a jsonl window, no text-format
        // line starts with it (and ROS names cannot: '{' is not a valid name character). Each
        // record is self-contained, so it parses without a current-window context; a malformed
        // '{' line (truncated tail after a crash) is skipped like any other noise. Sniffing per
        // line, rather than per file, keeps a mixed file (probe restarted with the other
        // format, same ROS_TOPIC_STATS_OUTPUT_FILE) fully readable.
        if (line[0] == '{') {
            sLogWindow win;
            if (parseJsonlWindow(line, win)) {
                windows.push_back(std::move(win));
                // push_back may reallocate: any dangling text-window pointer must die with it.
                // Orphaned text lines after a jsonl record are noise until their own header.
                cur = nullptr;
            }
            continue;
        }

        long long ts = 0;
        double a = 0.0;
        double b = 0.0;
        if (std::sscanf(line.c_str(), "# ts_ns=%lld window_s=%lf", &ts, &a) == 2) {
            windows.emplace_back();
            cur = &windows.back();
            cur->ts_ns = ts;
            cur->window_s = a;
            continue;
        }
        if (cur == nullptr) {
            continue;  // pre-header noise: not a probe log line
        }
        if (std::sscanf(line.c_str(), "TOPIC %511s %lf", name, &a) == 2) {
            statFor(*cur, name).pub_inter_hz = a;
        } else if (std::sscanf(line.c_str(), "PUB %511s inter=%lf intra=%lf", name, &a, &b) == 3) {
            auto& s = statFor(*cur, name);
            s.pub_inter_hz = a;
            s.pub_intra_hz = b;
        } else if (std::sscanf(line.c_str(), "RECV %511s inter=%lf intra=%lf", name, &a, &b) ==
                   3) {
            auto& s = statFor(*cur, name);
            s.recv_inter_hz = a;
            s.recv_intra_hz = b;
            s.recv_endpoint_seen = true;
        } else if (std::sscanf(line.c_str(), "JITTER %511s pub max_dt_ms=%lf", name, &a) == 2) {
            auto& s = statFor(*cur, name);
            s.pub_max_dt_ms = a;
            s.has_pub_max_dt = true;
        } else if (std::sscanf(line.c_str(), "JITTER %511s recv max_dt_ms=%lf", name, &a) == 2) {
            auto& s = statFor(*cur, name);
            s.recv_max_dt_ms = a;
            s.has_recv_max_dt = true;
        } else if (std::sscanf(line.c_str(), "LOAN %511s pub hz=%lf", name, &a) == 2) {
            statFor(*cur, name).pub_loaned_hz = a;
        } else if (std::sscanf(line.c_str(), "LOAN %511s recv hz=%lf", name, &a) == 2) {
            statFor(*cur, name).recv_loaned_hz = a;
        } else if (std::sscanf(line.c_str(), "NODE %511s", name) == 1) {
            cur->nodes.emplace_back(name);
        } else if (line.compare(0, 3, "EP ") == 0) {
            sEndpointStat e;
            if (parseEpLine(line, e)) {
                cur->endpoints.push_back(std::move(e));
            }
        }
        // anything else (WARN lines, future additions) is deliberately skipped: pulse-check
        // re-derives warnings from the raw rates instead of trusting probe-side output.
    }
    return windows;
}

}  // namespace ros2_pulse::core

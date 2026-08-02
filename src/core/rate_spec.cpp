// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#include "ros2_pulse/core/rate_spec.hpp"

#include <fcntl.h>     // open
#include <sys/stat.h>  // fstat, S_ISREG
#include <unistd.h>    // read, close

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_set>

namespace ros2_pulse::core {

namespace {

auto trim(const std::string& s) -> std::string {
    const auto b = s.find_first_not_of(" \t\r");
    if (b == std::string::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r");
    return s.substr(b, e - b + 1);
}

auto splitCommas(const std::string& s) -> std::vector<std::string> {
    std::vector<std::string> out;
    size_t pos = 0;
    while (pos <= s.size()) {
        const auto comma = s.find(',', pos);
        const auto end = comma == std::string::npos ? s.size() : comma;
        out.push_back(trim(s.substr(pos, end - pos)));
        pos = end + 1;
        if (comma == std::string::npos) {
            break;
        }
    }
    return out;
}

// Whole-token, non-negative, finite. Same defensive stance as env_config (KNOWN_ISSUES #5):
// the probe loads specs inside a tracepoint-reached constructor, so no exceptions.
auto parseHz(const std::string& tok, double& out) -> bool {
    if (tok.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const double v = std::strtod(tok.c_str(), &end);
    if (end != tok.c_str() + tok.size() || errno != 0 || !std::isfinite(v) || v < 0.0) {
        return false;
    }
    out = v;
    return true;
}

auto fail(std::string& error, int line_no, const std::string& msg) -> std::optional<sRateSpec> {
    error = "line " + std::to_string(line_no) + ": " + msg;
    return std::nullopt;
}

enum class eSection { kNone, kTopics, kNodes };

// "min_hz: 18" / "side: pub" — one flow-map item into the rule. Returns an error message, or
// empty on success. has_min/has_max let the caller enforce "a rule must constrain something",
// and double as the repeat guard for those two keys.
auto applyRuleItem(const std::string& item, sRateRule& rule, bool& has_min, bool& has_max,
                   bool& has_side, bool& has_transport) -> std::string {
    const auto colon = item.find(':');
    if (colon == std::string::npos) {
        return "expected 'key: value', got '" + item + "'";
    }
    const std::string key = trim(item.substr(0, colon));
    const std::string val = trim(item.substr(colon + 1));
    // A key given twice in one flow map used to silently last-win ({min_hz: 1, min_hz: 2} -> 2),
    // which is the one place the "duplicates are hard errors" contract leaked.
    const bool repeated = (key == "min_hz" && has_min) || (key == "max_hz" && has_max) ||
                          (key == "side" && has_side) || (key == "transport" && has_transport);
    if (repeated) {
        return "duplicate key '" + key + "' in one rule";
    }
    if (key == "min_hz") {
        if (!parseHz(val, rule.min_hz)) {
            return "bad min_hz '" + val + "' (need a non-negative number)";
        }
        has_min = true;
    } else if (key == "max_hz") {
        if (!parseHz(val, rule.max_hz)) {
            return "bad max_hz '" + val + "' (need a non-negative number)";
        }
        has_max = true;
    } else if (key == "side") {
        if (val == "pub") {
            rule.side = eRateSide::kPub;
        } else if (val == "recv") {
            rule.side = eRateSide::kRecv;
        } else {
            return "side must be 'pub' or 'recv', got '" + val + "'";
        }
        has_side = true;
    } else if (key == "transport") {
        if (val == "inter") {
            rule.transport = eRateTransport::kInter;
        } else if (val == "intra") {
            rule.transport = eRateTransport::kIntra;
        } else if (val == "any") {
            rule.transport = eRateTransport::kAny;
        } else {
            return "transport must be 'inter', 'intra' or 'any', got '" + val + "'";
        }
        has_transport = true;
    } else {
        return "unknown key '" + key + "' (expected min_hz, max_hz, side, transport)";
    }
    return {};
}

// The rate field a rule constrains. kAny means "how fast is this topic, however it travels" —
// the split is a transport detail the operator usually doesn't spec. How the two buckets combine
// differs by side, because only one of them counts disjoint events:
//
//   recv: DISJOINT. callback_start fires once per delivery and its is_intra_process flag selects
//         exactly one bucket, so inter + intra IS the delivery rate.
//
//   pub:  NOT disjoint on iron+. One publish() on an intra-process-enabled publisher fires
//         rclcpp_intra_publish AND, whenever a non-intra subscriber is matched (or the QoS is
//         TransientLocal, jazzy+), rcl_publish for the SAME message — rclcpp publisher.hpp
//         computes `inter_process_publish_needed = get_subscription_count() >
//         get_intra_process_subscription_count() || buffer_` and calls BOTH helpers on the true
//         branch. Both tracepoints carry the same rcl_publisher_t*, so both land on one counter
//         and a sum would report 2x the produce rate. max() is exact instead: it equals the one
//         live bucket when only one path fires, and the single produce rate when both do.
//         "Just read pub_inter" does not work — the all-in-process branch never calls rcl at all,
//         so pub_inter is 0 there. (No-op on humble, which has no intra-publish tracepoint.)
auto observedHz(const sRateRule& rule, const sTopicStat& s) -> double {
    const double inter = rule.side == eRateSide::kPub ? s.pub_inter_hz : s.recv_inter_hz;
    const double intra = rule.side == eRateSide::kPub ? s.pub_intra_hz : s.recv_intra_hz;
    switch (rule.transport) {
        case eRateTransport::kInter:
            return inter;
        case eRateTransport::kIntra:
            return intra;
        default:  // kAny
            if (rule.side == eRateSide::kPub) {
                return inter > intra ? inter : intra;
            }
            return inter + intra;
    }
}

// Bounds render compactly ("18", "22.5", "inf") — they echo the spec, unlike observed rates
// which keep the window format's 6dp.
auto formatBound(double v) -> std::string {
    if (std::isinf(v)) {
        return "inf";
    }
    char buf[32];
    const int n = std::snprintf(buf, sizeof(buf), "%g", v);
    return n > 0 ? std::string(buf, static_cast<size_t>(n)) : std::string();
}

auto contains(const std::vector<std::string>& v, const std::string& s) -> bool {
    for (const auto& x : v) {
        if (x == s) {
            return true;
        }
    }
    return false;
}

}  // namespace

auto readSpecFile(const char* path, std::string& out, std::string& error) -> bool {
    if (path == nullptr || *path == '\0') {
        error = "empty path";
        return false;
    }
    // O_NONBLOCK so a FIFO cannot park the host process inside open() forever.
    const int fd = ::open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        error = std::strerror(errno);
        return false;
    }
    struct stat st {};
    if (::fstat(fd, &st) != 0) {
        error = std::strerror(errno);
        ::close(fd);
        return false;
    }
    // Everything that is not a plain file is refused here: directories (which read EISDIR into
    // an empty-but-"valid" spec), FIFOs, and character devices that never reach EOF.
    if (!S_ISREG(st.st_mode)) {
        error = "not a regular file";
        ::close(fd);
        return false;
    }
    if (static_cast<unsigned long long>(st.st_size) > kMaxSpecBytes) {
        error = "larger than " + std::to_string(kMaxSpecBytes) + " bytes — not a spec?";
        ::close(fd);
        return false;
    }
    out.reserve(static_cast<size_t>(st.st_size));
    char buf[4096];
    ssize_t n = 0;
    while ((n = ::read(fd, buf, sizeof(buf))) > 0) {
        if (out.size() + static_cast<size_t>(n) > kMaxSpecBytes) {
            error = "grew past the size cap while being read";  // raced with a writer
            ::close(fd);
            return false;
        }
        out.append(buf, static_cast<size_t>(n));
    }
    const int read_errno = errno;
    ::close(fd);
    if (n < 0) {
        error = std::strerror(read_errno);
        return false;
    }
    return true;
}

auto parseRateSpec(const std::string& text, std::string& error) -> std::optional<sRateSpec> {
    sRateSpec spec;
    std::unordered_set<std::string> seen_rules;
    eSection section = eSection::kNone;
    int line_no = 0;
    size_t pos = 0;
    // A leading UTF-8 BOM is a signature, not content (Unicode 23.8.1; YAML 1.2 §5.2 consumes
    // c-byte-order-mark as a document prefix, and libyaml/PyYAML/SnakeYAML all strip it). Without
    // this, a spec saved by Windows Notepad or PowerShell fails as
    // "line 1: unknown top-level entry 'topics:'" — bytes the terminal renders invisibly, so the
    // message looks identical to the correct spelling and alerting silently turns off.
    // line_no still starts at 1, so error line numbers are unaffected. CRLF is handled by trim().
    if (text.compare(0, 3, "\xEF\xBB\xBF") == 0) {
        pos = 3;
    }
    while (pos <= text.size()) {
        const auto nl = text.find('\n', pos);
        std::string raw =
            text.substr(pos, (nl == std::string::npos ? text.size() : nl) - pos);
        pos = nl == std::string::npos ? text.size() + 1 : nl + 1;
        ++line_no;

        const auto hash = raw.find('#');
        if (hash != std::string::npos) {
            raw.resize(hash);
        }
        const bool indented = !raw.empty() && (raw[0] == ' ' || raw[0] == '\t');
        const std::string line = trim(raw);
        if (line.empty()) {
            continue;
        }

        if (!indented) {
            if (line == "topics:") {
                section = eSection::kTopics;
                continue;
            }
            if (line == "nodes:") {
                section = eSection::kNodes;  // block list follows
                continue;
            }
            if (line.rfind("nodes:", 0) == 0) {
                const std::string rest = trim(line.substr(6));
                if (rest.size() >= 2 && rest.front() == '[' && rest.back() == ']') {
                    for (const auto& n : splitCommas(rest.substr(1, rest.size() - 2))) {
                        if (!n.empty()) {
                            spec.nodes.push_back(n);
                        }
                    }
                    section = eSection::kNone;
                    continue;
                }
                return fail(error, line_no, "nodes: needs a [flow list] or an indented '- /name' block");
            }
            return fail(error, line_no,
                        "unknown top-level entry '" + line + "' (expected 'topics:' or 'nodes:')");
        }

        switch (section) {
            case eSection::kNone:
                return fail(error, line_no, "indented entry outside a 'topics:'/'nodes:' section");
            case eSection::kNodes: {
                if (line.rfind("-", 0) != 0) {
                    return fail(error, line_no, "expected '- /node_name' inside 'nodes:'");
                }
                const std::string name = trim(line.substr(1));
                if (name.empty()) {
                    return fail(error, line_no, "empty node entry");
                }
                spec.nodes.push_back(name);
                break;
            }
            case eSection::kTopics: {
                const auto colon = line.find(':');
                const auto brace = line.find('{');
                if (colon == std::string::npos || brace == std::string::npos ||
                    line.back() != '}' || brace < colon) {
                    return fail(error, line_no,
                                "expected '<topic>: {min_hz: ..., ...}' inside 'topics:'");
                }
                const std::string name = trim(line.substr(0, colon));
                if (name.empty()) {
                    return fail(error, line_no, "empty topic name");
                }
                sRateRule rule;
                bool has_min = false;
                bool has_max = false;
                bool has_side = false;
                bool has_transport = false;
                const std::string body = trim(line.substr(brace + 1, line.size() - brace - 2));
                if (!body.empty()) {
                    for (const auto& item : splitCommas(body)) {
                        const std::string item_err =
                            applyRuleItem(item, rule, has_min, has_max, has_side, has_transport);
                        if (!item_err.empty()) {
                            return fail(error, line_no, item_err);
                        }
                    }
                }
                if (!has_min && !has_max) {
                    return fail(error, line_no,
                                "rule for '" + name + "' needs min_hz and/or max_hz");
                }
                if (rule.min_hz > rule.max_hz) {
                    return fail(error, line_no, "min_hz > max_hz for '" + name + "'");
                }
                // Dedup on the measurement domain, not the name: constraining both ends of one
                // topic ("the driver publishes ~20 Hz AND we receive ~20 Hz") is a first-class
                // use case, and evaluateRateSpec already walks the vector rule by rule. Two
                // entries that measure the SAME thing are still a copy-paste error.
                const std::string rule_key = name + '\x01' +
                                             std::to_string(static_cast<int>(rule.side)) + '\x01' +
                                             std::to_string(static_cast<int>(rule.transport));
                if (!seen_rules.insert(rule_key).second) {
                    return fail(error, line_no,
                                "duplicate rule for '" + name + "' (same side and transport)");
                }
                spec.topics.emplace_back(name, rule);
                break;
            }
        }
    }
    return spec;
}

auto evaluateRateSpec(const sRateSpec& spec, const std::vector<sTopicStat>& stats,
                      const std::vector<std::string>& active_nodes,
                      const std::vector<std::string>& known_nodes,
                      bool missing_as_zero) -> std::vector<std::string> {
    std::vector<std::string> out;
    for (const auto& [name, rule] : spec.topics) {
        const sTopicStat* found = nullptr;
        for (const auto& s : stats) {
            if (s.topic == name) {
                found = &s;
                break;
            }
        }
        double hz = 0.0;
        if (found != nullptr) {
            hz = observedHz(rule, *found);
        } else if (!missing_as_zero) {
            continue;  // some other process's endpoint — not this probe's business
        }
        if (hz < rule.min_hz || hz > rule.max_hz) {
            char hz_buf[32];
            std::snprintf(hz_buf, sizeof(hz_buf), "%.6f", hz);
            out.push_back("WARN TOPIC " + name + " hz=" + hz_buf + " expected=[" +
                          formatBound(rule.min_hz) + "," + formatBound(rule.max_hz) + "]");
        }
    }
    for (const auto& name : spec.nodes) {
        if (contains(active_nodes, name)) {
            continue;
        }
        if (!contains(known_nodes, name) && !missing_as_zero) {
            continue;  // never initialized in this process — skip (probe mode)
        }
        out.push_back("WARN NODE " + name + " missing");
    }
    return out;
}

}  // namespace ros2_pulse::core

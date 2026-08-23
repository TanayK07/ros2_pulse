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
// the probe loads specs inside a tracepoint-reached constructor, so no exceptions. Not named
// parseHz: it also parses max_gap_ms, and a function named for one unit parsing another rots.
auto parseNonNegative(const std::string& tok, double& out) -> bool {
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

// Which keys a rule has supplied. Lets the caller enforce "a rule must constrain something"
// and doubles as the repeat guard. A struct rather than N bool& out-params: max_gap_ms would
// have made it seven parameters.
struct sSeenKeys {
    bool min{false};
    bool max{false};
    bool gap{false};
    bool side{false};
    bool transport{false};
};

// "min_hz: 18" / "side: pub", one flow-map item into the rule. Returns an error message, or
// empty on success.
auto applyRuleItem(const std::string& item, sRateRule& rule, sSeenKeys& seen) -> std::string {
    const auto colon = item.find(':');
    if (colon == std::string::npos) {
        return "expected 'key: value', got '" + item + "'";
    }
    const std::string key = trim(item.substr(0, colon));
    const std::string val = trim(item.substr(colon + 1));
    // A key given twice in one flow map used to silently last-win ({min_hz: 1, min_hz: 2} -> 2),
    // which is the one place the "duplicates are hard errors" contract leaked.
    const bool repeated = (key == "min_hz" && seen.min) || (key == "max_hz" && seen.max) ||
                          (key == "max_gap_ms" && seen.gap) || (key == "side" && seen.side) ||
                          (key == "transport" && seen.transport);
    if (repeated) {
        return "duplicate key '" + key + "' in one rule";
    }
    if (key == "min_hz") {
        if (!parseNonNegative(val, rule.min_hz)) {
            return "bad min_hz '" + val + "' (need a non-negative number)";
        }
        seen.min = true;
    } else if (key == "max_hz") {
        if (!parseNonNegative(val, rule.max_hz)) {
            return "bad max_hz '" + val + "' (need a non-negative number)";
        }
        seen.max = true;
    } else if (key == "max_gap_ms") {
        if (!parseNonNegative(val, rule.max_gap_ms)) {
            return "bad max_gap_ms '" + val + "' (need a non-negative number)";
        }
        // A gap bound of zero can never be satisfied, same class of unsatisfiable rule as
        // min_hz > max_hz, so reject it rather than emit a WARN on every window forever.
        if (rule.max_gap_ms <= 0.0) {
            return "max_gap_ms must be > 0";
        }
        seen.gap = true;
    } else if (key == "side") {
        if (val == "pub") {
            rule.side = eRateSide::kPub;
        } else if (val == "recv") {
            rule.side = eRateSide::kRecv;
        } else {
            return "side must be 'pub' or 'recv', got '" + val + "'";
        }
        seen.side = true;
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
        seen.transport = true;
    } else {
        return "unknown key '" + key + "' (expected min_hz, max_hz, max_gap_ms, side, transport)";
    }
    return {};
}

// The rate field a rule constrains. kAny means "how fast is this topic, however it travels",
// the split is a transport detail the operator usually doesn't spec. How the two buckets combine
// differs by side, because only one of them counts disjoint events:
//
//   recv: DISJOINT. callback_start fires once per delivery and its is_intra_process flag selects
//         exactly one bucket, so inter + intra IS the delivery rate.
//
//   pub:  NOT disjoint on iron+. One publish() on an intra-process-enabled publisher fires
//         rclcpp_intra_publish AND, whenever a non-intra subscriber is matched (or the QoS is
//         TransientLocal, jazzy+), rcl_publish for the SAME message, rclcpp publisher.hpp
//         computes `inter_process_publish_needed = get_subscription_count() >
//         get_intra_process_subscription_count() || buffer_` and calls BOTH helpers on the true
//         branch. Both tracepoints carry the same rcl_publisher_t*, so both land on one counter
//         and a sum would report 2x the produce rate. max() is exact instead: it equals the one
//         live bucket when only one path fires, and the single produce rate when both do.
//         "Just read pub_inter" does not work, the all-in-process branch never calls rcl at all,
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

// Bounds render compactly ("18", "22.5", "inf"), they echo the spec, unlike observed rates
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
        error = "larger than " + std::to_string(kMaxSpecBytes) + " bytes, not a spec?";
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
    // "line 1: unknown top-level entry 'topics:'", bytes the terminal renders invisibly, so the
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
                sSeenKeys seen;
                const std::string body = trim(line.substr(brace + 1, line.size() - brace - 2));
                if (!body.empty()) {
                    for (const auto& item : splitCommas(body)) {
                        const std::string item_err = applyRuleItem(item, rule, seen);
                        if (!item_err.empty()) {
                            return fail(error, line_no, item_err);
                        }
                    }
                }
                if (!seen.min && !seen.max && !seen.gap) {
                    return fail(error, line_no,
                                "rule for '" + name +
                                    "' needs at least one of min_hz, max_hz, max_gap_ms");
                }
                // transport selects which rate bucket a rate bound reads; the gap series is
                // transport-merged, so pairing it ONLY with transport constrains nothing.
                if (seen.gap && !seen.min && !seen.max && seen.transport) {
                    return fail(error, line_no,
                                "transport has no effect on max_gap_ms for '" + name +
                                    "' (the gap series is transport-merged)");
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

// The single evaluation. Since R6 it produces STRUCTURED warnings; the historical string API
// below is a pure projection of this result (renderWarnLine), so the two can never disagree,
// the same one-home principle as the emit gates living in TopicRegistry.
auto evaluateRateSpecWarnings(const sRateSpec& spec, const std::vector<sTopicStat>& stats,
                              const std::vector<std::string>& active_nodes,
                              const std::vector<std::string>& known_nodes,
                              bool missing_as_zero,
                              std::vector<std::string>* unmeasured_gaps)
    -> std::vector<sRateWarning> {
    std::vector<sRateWarning> out;
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
            continue;  // some other process's endpoint, not this probe's business
        }
        if (hz < rule.min_hz || hz > rule.max_hz) {
            sRateWarning w;
            w.kind = eWarnKind::kTopicRate;
            w.name = name;
            w.hz = hz;
            w.min_hz = rule.min_hz;
            w.max_hz = rule.max_hz;
            out.push_back(std::move(w));
        }
        // Gap bound, evaluated independently: one rule can violate BOTH (a topic that is slow
        // AND freezes), and the two are different faults, so both warnings are emitted. Rate
        // first, preserving spec order.
        if (std::isinf(rule.max_gap_ms)) {
            continue;  // no gap bound on this rule
        }
        const bool measured = rule.side == eRateSide::kPub
                                  ? (found != nullptr && found->has_pub_max_dt)
                                  : (found != nullptr && found->has_recv_max_dt);
        if (!measured) {
            if (missing_as_zero) {
                // Offline the log set is the whole picture: a spec topic nobody measured is a
                // violation at an unbounded gap, not a silent pass.
                if (unmeasured_gaps != nullptr) {
                    unmeasured_gaps->push_back(name);
                }
            }
            continue;  // probe mode: some other process's endpoint, or tracking is off
        }
        const double gap = rule.side == eRateSide::kPub ? found->pub_max_dt_ms
                                                        : found->recv_max_dt_ms;
        if (gap > rule.max_gap_ms) {
            sRateWarning w;
            w.kind = eWarnKind::kTopicGap;
            w.name = name;
            w.max_dt_ms = gap;
            w.max_gap_ms = rule.max_gap_ms;
            out.push_back(std::move(w));
        }
    }
    for (const auto& name : spec.nodes) {
        if (contains(active_nodes, name)) {
            continue;
        }
        if (!contains(known_nodes, name) && !missing_as_zero) {
            continue;  // never initialized in this process, skip (probe mode)
        }
        sRateWarning w;
        w.kind = eWarnKind::kNodeMissing;
        w.name = name;
        out.push_back(std::move(w));
    }
    return out;
}

auto renderWarnLine(const sRateWarning& warn) -> std::string {
    // The exact pre-R6 line bytes, pinned by the R1/R5 unit tests: %.6f for hz, %.3f for the
    // gap, %g (via formatBound) for spec bounds with 'inf' for an unbounded max.
    switch (warn.kind) {
        case eWarnKind::kTopicRate: {
            char hz_buf[32];
            std::snprintf(hz_buf, sizeof(hz_buf), "%.6f", warn.hz);
            return "WARN TOPIC " + warn.name + " hz=" + hz_buf + " expected=[" +
                   formatBound(warn.min_hz) + "," + formatBound(warn.max_hz) + "]";
        }
        case eWarnKind::kTopicGap: {
            char gap_buf[32];
            std::snprintf(gap_buf, sizeof(gap_buf), "%.3f", warn.max_dt_ms);
            return "WARN TOPIC " + warn.name + " max_dt_ms=" + gap_buf +
                   " expected_max_gap_ms=" + formatBound(warn.max_gap_ms);
        }
        case eWarnKind::kNodeMissing:
            return "WARN NODE " + warn.name + " missing";
    }
    return {};  // unreachable; keeps -Wreturn-type quiet without a default: that hides new kinds
}

auto evaluateRateSpec(const sRateSpec& spec, const std::vector<sTopicStat>& stats,
                      const std::vector<std::string>& active_nodes,
                      const std::vector<std::string>& known_nodes,
                      bool missing_as_zero,
                      std::vector<std::string>* unmeasured_gaps) -> std::vector<std::string> {
    const auto warns = evaluateRateSpecWarnings(spec, stats, active_nodes, known_nodes,
                                                missing_as_zero, unmeasured_gaps);
    std::vector<std::string> out;
    out.reserve(warns.size());
    for (const auto& w : warns) {
        out.push_back(renderWarnLine(w));
    }
    return out;
}

}  // namespace ros2_pulse::core

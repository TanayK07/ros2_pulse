// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// See diagnostics_mapper.hpp for the mapping contract.

#include "ros2_pulse/core/diagnostics_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"

namespace ros2_pulse::core {

namespace {

auto fmt(const char* f, double v) -> std::string {
    char buf[64];
    const int n = std::snprintf(buf, sizeof(buf), f, v);
    return n > 0 ? std::string(buf, static_cast<size_t>(n)) : std::string();
}

// Spec bounds as the spec wrote them ("18", "22.5", "inf").
auto bound(double v) -> std::string { return std::isinf(v) ? "inf" : fmt("%g", v); }

// The jsonl warns render bounds with %g and observed values with %.6f / %.3f, so they are
// compared at that precision, not bit for bit.
auto sameBound(double a, double b) -> bool {
    if (std::isinf(a) || std::isinf(b)) return std::isinf(a) && std::isinf(b);
    return std::fabs(a - b) <= 1e-5 * std::max(1.0, std::fabs(b));
}
auto sameHz(double a, double b) -> bool { return std::fabs(a - b) <= 1e-5; }
auto sameMs(double a, double b) -> bool { return std::fabs(a - b) <= 1e-3; }

auto basename(const std::string& path) -> std::string {
    const auto slash = path.rfind('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

auto findStat(const sLogWindow& win, const std::string& topic) -> const sTopicStat* {
    for (const auto& s : win.stats) {
        if (s.topic == topic) return &s;
    }
    return nullptr;
}

// Whether this window has an endpoint on @p side of the topic. The per-endpoint split says so
// directly; an older probe's topic line only marks a proven subscription (recv_endpoint_seen),
// so there anything else on the line is the publisher.
auto hostsSide(const sLogWindow& win, const sTopicStat& s, eRateSide side) -> bool {
    if (!win.endpoints.empty()) {
        const auto want = side == eRateSide::kPub ? eEndpointSide::kPub : eEndpointSide::kRecv;
        return std::any_of(win.endpoints.begin(), win.endpoints.end(), [&](const auto& e) {
            return e.topic == s.topic && e.side == want;
        });
    }
    if (side == eRateSide::kRecv) return s.recv_endpoint_seen;
    return s.pub_inter_hz > 0.0 || s.pub_intra_hz > 0.0 || s.has_pub_max_dt ||
           !s.recv_endpoint_seen;
}

auto hasGap(const sTopicStat& s, eRateSide side) -> bool {
    return side == eRateSide::kPub ? s.has_pub_max_dt : s.has_recv_max_dt;
}
auto gapMs(const sTopicStat& s, eRateSide side) -> double {
    return side == eRateSide::kPub ? s.pub_max_dt_ms : s.recv_max_dt_ms;
}

auto matchesRule(const sRateWarning& w, const std::string& topic, const sRateRule& rule,
                 const sTopicStat& s) -> bool {
    if (w.name != topic) return false;
    if (w.kind == eWarnKind::kTopicRate) {
        return sameBound(w.min_hz, rule.min_hz) && sameBound(w.max_hz, rule.max_hz) &&
               sameHz(w.hz, observedHz(rule, s));
    }
    if (w.kind == eWarnKind::kTopicGap) {
        return sameBound(w.max_gap_ms, rule.max_gap_ms) && hasGap(s, rule.side) &&
               sameMs(w.max_dt_ms, gapMs(s, rule.side));
    }
    return false;
}

auto sideName(eRateSide side) -> const char* { return side == eRateSide::kPub ? "pub" : "recv"; }

auto ruleName(const std::string& topic, const sRateRule& rule) -> std::string {
    std::string name = "ros2_pulse: " + topic + " (" + sideName(rule.side);
    if (rule.transport == eRateTransport::kInter) name += ", inter";
    if (rule.transport == eRateTransport::kIntra) name += ", intra";
    return name + ")";
}

auto rateMessage(double hz, double min_hz, double max_hz) -> std::string {
    return "rate " + fmt("%.3f", hz) + " Hz outside [" + bound(min_hz) + ", " + bound(max_hz) +
           "]";
}
auto gapMessage(double max_dt_ms, double max_gap_ms) -> std::string {
    return "gap " + fmt("%.3f", max_dt_ms) + " ms over " + bound(max_gap_ms) + " ms";
}

// One log's view of one status, folded across logs: the worst level wins, the slowest on a tie.
struct sCandidate {
    sDiagStatus status;
    double hz{0.0};
};

void fold(std::vector<sCandidate>& out, std::map<std::string, size_t>& index, sCandidate c) {
    const auto it = index.find(c.status.name);
    if (it == index.end()) {
        index.emplace(c.status.name, out.size());
        out.push_back(std::move(c));
        return;
    }
    sCandidate& cur = out[it->second];
    const bool worse = c.status.level > cur.status.level;
    if (worse || (c.status.level == cur.status.level && c.hz < cur.hz)) cur = std::move(c);
}

void flush(std::vector<sCandidate>& from, std::vector<sDiagStatus>& to) {
    for (auto& c : from) to.push_back(std::move(c.status));
}

}  // namespace

auto mapDiagnostics(const std::vector<sDiagLog>& logs, const sDiagOptions& opts)
    -> std::vector<sDiagStatus> {
    std::vector<sDiagStatus> out;
    std::vector<const sDiagLog*> fresh;

    for (const auto& log : logs) {
        if (log.age_s > kDiagForgetAfterS) continue;
        const double period = log.period_s > 0.0 ? log.period_s : kDiagDefaultPeriodS;
        const double limit = 2.0 * period + opts.poll_period_s;
        sDiagStatus st;
        st.name = "ros2_pulse: log " + basename(log.path);
        if (log.age_s > limit) {
            st.level = eDiagLevel::Error;
            st.message = "stale: no window for " + fmt("%.1f", log.age_s) + " s";
        } else {
            st.message = "ok";
            fresh.push_back(&log);
        }
        std::string nodes;
        for (const auto& n : log.window.nodes) nodes += (nodes.empty() ? "" : ",") + n;
        st.values = {{"path", log.path},
                     {"age_s", fmt("%.1f", log.age_s)},
                     {"window_s", fmt("%.3f", period)},
                     {"nodes", nodes}};
        out.push_back(std::move(st));
    }

    const sRateSpec empty;
    const sRateSpec& spec = opts.spec != nullptr ? *opts.spec : empty;

    // Nodes: spec order, then nodes only a node_missing warn names.
    std::vector<sCandidate> nodes;
    std::map<std::string, size_t> node_index;
    auto nodeStatus = [](const std::string& name, bool missing) {
        sCandidate c;
        c.status.name = "ros2_pulse: node " + name;
        c.status.level = missing ? eDiagLevel::Error : eDiagLevel::Ok;
        c.status.message = missing ? "missing" : "alive";
        return c;
    };
    for (const auto& name : spec.nodes) {
        for (const auto* log : fresh) {
            const auto& n = log->window.nodes;
            if (std::find(n.begin(), n.end(), name) != n.end()) {
                fold(nodes, node_index, nodeStatus(name, false));
            }
        }
    }
    for (const auto* log : fresh) {
        for (const auto& w : log->window.warns) {
            if (w.kind == eWarnKind::kNodeMissing) {
                fold(nodes, node_index, nodeStatus(w.name, true));
            }
        }
    }
    // Spec nodes first, in spec order, whatever order the folds created them in.
    std::stable_sort(nodes.begin(), nodes.end(), [&](const sCandidate& a, const sCandidate& b) {
        auto rank = [&](const sCandidate& c) {
            for (size_t i = 0; i < spec.nodes.size(); ++i) {
                if (c.status.name == "ros2_pulse: node " + spec.nodes[i]) return i;
            }
            return spec.nodes.size();
        };
        return rank(a) < rank(b);
    });
    flush(nodes, out);

    // Spec rules, in spec order.
    std::vector<sCandidate> rules;
    std::map<std::string, size_t> rule_index;
    for (const auto& [topic, rule] : spec.topics) {
        for (const auto* log : fresh) {
            const sTopicStat* s = findStat(log->window, topic);
            if (s == nullptr || !hostsSide(log->window, *s, rule.side)) continue;
            sCandidate c;
            c.hz = observedHz(rule, *s);
            c.status.name = ruleName(topic, rule);
            std::string msg;
            for (const auto& w : log->window.warns) {
                if (!matchesRule(w, topic, rule, *s)) continue;
                c.status.level = eDiagLevel::Warn;
                if (!msg.empty()) msg += "; ";
                msg += w.kind == eWarnKind::kTopicRate ? rateMessage(w.hz, w.min_hz, w.max_hz)
                                                       : gapMessage(w.max_dt_ms, w.max_gap_ms);
            }
            c.status.message = msg.empty() ? "ok" : msg;
            auto& v = c.status.values;
            v.push_back({"hz", fmt("%.3f", c.hz)});
            if (rule.min_hz > 0.0) v.push_back({"min_hz", bound(rule.min_hz)});
            if (!std::isinf(rule.max_hz)) v.push_back({"max_hz", bound(rule.max_hz)});
            if (!std::isinf(rule.max_gap_ms)) v.push_back({"max_gap_ms", bound(rule.max_gap_ms)});
            if (hasGap(*s, rule.side)) {
                v.push_back({"max_dt_ms", fmt("%.3f", gapMs(*s, rule.side))});
            }
            fold(rules, rule_index, std::move(c));
        }
    }
    flush(rules, out);

    // Topic warns no spec rule accounts for.
    std::vector<sCandidate> loose;
    std::map<std::string, size_t> loose_index;
    for (const auto* log : fresh) {
        for (const auto& w : log->window.warns) {
            if (w.kind == eWarnKind::kNodeMissing) continue;
            const sTopicStat* s = findStat(log->window, w.name);
            const bool matched =
                s != nullptr &&
                std::any_of(spec.topics.begin(), spec.topics.end(), [&](const auto& r) {
                    return matchesRule(w, r.first, r.second, *s);
                });
            if (matched) continue;
            sCandidate c;
            c.status.name = "ros2_pulse: " + w.name;
            c.status.level = eDiagLevel::Warn;
            auto& v = c.status.values;
            if (w.kind == eWarnKind::kTopicRate) {
                c.hz = w.hz;
                c.status.message = rateMessage(w.hz, w.min_hz, w.max_hz);
                v.push_back({"hz", fmt("%.3f", w.hz)});
                if (w.min_hz > 0.0) v.push_back({"min_hz", bound(w.min_hz)});
                if (!std::isinf(w.max_hz)) v.push_back({"max_hz", bound(w.max_hz)});
            } else {
                c.status.message = gapMessage(w.max_dt_ms, w.max_gap_ms);
                v.push_back({"max_gap_ms", bound(w.max_gap_ms)});
                v.push_back({"max_dt_ms", fmt("%.3f", w.max_dt_ms)});
            }
            const auto it = loose_index.find(c.status.name);
            if (it != loose_index.end()) {
                // Rate and gap warns of one topic share the status, like a rule's.
                auto& cur = loose[it->second].status;
                if (cur.message.find(c.status.message) == std::string::npos) {
                    cur.message += "; " + c.status.message;
                    for (auto& kv : v) {
                        const bool have =
                            std::any_of(cur.values.begin(), cur.values.end(),
                                        [&](const auto& x) { return x.key == kv.key; });
                        if (!have) cur.values.push_back(std::move(kv));
                    }
                }
                continue;
            }
            loose_index.emplace(c.status.name, loose.size());
            loose.push_back(std::move(c));
        }
    }
    flush(loose, out);

    if (!opts.all_topics) return out;

    // Every other topic and side seen, OK with its rate. Sorted by name.
    std::vector<sCandidate> rest;
    std::map<std::string, size_t> rest_index;
    for (const auto* log : fresh) {
        for (const auto& s : log->window.stats) {
            if (TopicRegistry::shouldFilter(s.topic)) continue;
            for (const auto side : {eRateSide::kPub, eRateSide::kRecv}) {
                if (!hostsSide(log->window, s, side)) continue;
                const bool ruled =
                    std::any_of(spec.topics.begin(), spec.topics.end(), [&](const auto& r) {
                        return r.first == s.topic && r.second.side == side;
                    });
                if (ruled) continue;
                sRateRule any;
                any.side = side;
                sCandidate c;
                c.hz = observedHz(any, s);
                c.status.name = ruleName(s.topic, any);
                c.status.message = "ok";
                c.status.values.push_back({"hz", fmt("%.3f", c.hz)});
                if (hasGap(s, side)) {
                    c.status.values.push_back({"max_dt_ms", fmt("%.3f", gapMs(s, side))});
                }
                fold(rest, rest_index, std::move(c));
            }
        }
    }
    std::sort(rest.begin(), rest.end(), [](const sCandidate& a, const sCandidate& b) {
        return a.status.name < b.status.name;
    });
    flush(rest, out);
    return out;
}

}  // namespace ros2_pulse::core

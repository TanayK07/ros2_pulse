// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").
//
// Maps the latest window of every followed probe log onto diagnostic_msgs/DiagnosticStatus-shaped
// data for pulse_bridge's /diagnostics output, so diagnostic_aggregator can alert on topic rates
// without a DiagnosedPublisher in every driver. Pure core: no ROS types, unit-tested in the
// standalone lane.
//
// The verdict is the probe's: a topic is WARN when its process's window carries a topic_rate or
// topic_gap warn for it, a node is ERROR when one carries node_missing. The bridge does not
// re-judge the rates; it only maps those warns onto statuses and adds staleness, which the probe
// cannot report about itself.
//
// Statuses, all named "ros2_pulse: ...":
//   log <file>          one per log with a window: OK, or ERROR "stale" once no window has
//                       arrived for two window periods plus one poll (pulse-export's rule). A
//                       log silent for kDiagForgetAfterS is dropped. A stale log's windows are
//                       not used for the statuses below. A log whose latest window is the
//                       probe's exit window (partial, after a full one) is dropped at once:
//                       the process exited cleanly, which is not a fault. A partial window
//                       with no full one before it is the attach window and reports as usual.
//   node <name>         one per spec node: OK when a fresh window lists it, ERROR on a
//                       node_missing warn, absent when no log knows it (another machine's).
//   <topic> (<side>)    one per spec rule, ", intra" / ", inter" appended for a transport rule:
//                       OK, or WARN on a matching warn. Only logs that host that side of the topic
//                       count, so a publisher-only process does not report a receive rule. Absent
//                       when no fresh log hosts it.
//   <topic>             a topic warn that matches no spec rule (no spec given, or the probe runs
//                       a different one): WARN, side unknown.
//   <topic> (<side>)    with all_topics, every other topic and side a fresh log hosts: OK.
// Several logs hosting one rule fold into one status: the worst level, and the numbers of the log
// that set it (the slowest on a tie).
//
// The probe's warns carry no side, so a warn is matched to a rule by topic, bounds and observed
// value: a topic_rate warn's hz must equal observedHz(rule) of that log's window, a topic_gap
// warn's max_dt_ms that side's gap. Two rules of one topic with the same bounds still separate.

#ifndef ROS2_PULSE__CORE__DIAGNOSTICS_MAPPER_HPP_
#define ROS2_PULSE__CORE__DIAGNOSTICS_MAPPER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/rate_spec.hpp"

namespace ros2_pulse::core {

/// diagnostic_msgs/DiagnosticStatus levels, same values.
enum class eDiagLevel : std::uint8_t { Ok = 0, Warn = 1, Error = 2 };

struct sDiagKeyValue {
    std::string key;
    std::string value;
};

/// One DiagnosticStatus minus hardware_id, which the bridge sets on every status.
struct sDiagStatus {
    eDiagLevel level{eDiagLevel::Ok};
    std::string name;
    std::string message;
    std::vector<sDiagKeyValue> values;
};

/// One followed log: its latest window and how long ago it was written.
struct sDiagLog {
    std::string path;
    sLogWindow window;
    double age_s{0.0};     // since the latest window was written
    double period_s{0.0};  // the probe's window period (latest full window_s), 0 = not known yet
};

struct sDiagOptions {
    const sRateSpec* spec{nullptr};  // nullptr: no spec, only warns, nodes and logs
    bool all_topics{false};
    double poll_period_s{1.0};
};

/// Period assumed while a log has shown only a partial window: the probe's default
/// ROS_TOPIC_STATISTICS_PUBLISH_PERIOD.
constexpr double kDiagDefaultPeriodS = 5.0;
/// A log silent this long is dropped from the output.
constexpr double kDiagForgetAfterS = 300.0;

auto mapDiagnostics(const std::vector<sDiagLog>& logs, const sDiagOptions& opts)
    -> std::vector<sDiagStatus>;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__DIAGNOSTICS_MAPPER_HPP_

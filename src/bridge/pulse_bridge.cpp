// Copyright 2026 ros2_pulse contributors
//
// pulse_bridge: optional sidecar that tails the probe's jsonl log(s) and republishes every window
// on /statistics as statistics_msgs/MetricsMessage, in the shape rclcpp's built-in topic
// statistics use, so dashboards and exporters that already read /statistics get the probe's
// numbers (intra-process included) with no integration work.
//
// It runs as its own process on purpose. The probe stays out of the DDS graph and free of
// rclcpp; this is the only ros2_pulse binary that links it. Where the probe writes is the
// probe's business (ROS_TOPIC_STATS_OUTPUT_FILE / ROS_TOPIC_STATS_FORMAT=jsonl); the bridge only
// reads.
//
// Parameters
//   files          string[]  explicit log paths to follow                       (default: none)
//   glob           string    pattern re-expanded on every poll, so a probed process that starts
//                            later is picked up               (default: $TMPDIR/topic_freq.*.log)
//   topic          string    where to publish                             (default: /statistics)
//   unit           string    "ms": AVERAGE is the message period, the built-in convention;
//                            "Hz": AVERAGE is the rate                            (default: ms)
//   poll_period_s  double    how often the logs are read                        (default: 1.0)
//   source_name    string    measurement_source_name override; empty means the window's single
//                            node name, or "ros2_pulse" when that is ambiguous   (default: "")
//   metrics_source string    "topic": metrics_source is the topic, message_period only;
//                            "metric": rclcpp's layout, metrics_source is "message_period" /
//                            "message_age" and the topic is not carried     (default: topic)
//   graph_monitor  bool      also publish rosgraph_monitor_msgs/TopicStatistics, the input of
//                            ros-tooling/graph-monitor (ros-tooling/graph-monitor#14). Only in
//                            builds that found rosgraph_monitor_msgs, otherwise it warns
//                                                                               (default: false)
//   graph_monitor_topic  string  where to publish those          (default: /topic_statistics)
//
// Text-format logs are refused per file with one warning: their windows span several lines and
// the bridge reads line by line. Set ROS_TOPIC_STATS_FORMAT=jsonl on the probed process.

#include <glob.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

#include "ros2_pulse/core/line_follower.hpp"
#include "ros2_pulse/core/log_reader.hpp"
#include "ros2_pulse/core/metrics_mapper.hpp"
#include "ros2_pulse/core/topic_statistic_mapper.hpp"

// Set by CMake when rosgraph_monitor_msgs was found at build time. The dependency is optional so
// a source build without it still gets the /statistics bridge.
#ifdef ROS2_PULSE_HAVE_ROSGRAPH_MONITOR_MSGS
#include "rosgraph_monitor_msgs/msg/topic_statistic.hpp"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"
#endif

namespace {

using ros2_pulse::core::eMetricsSource;
using ros2_pulse::core::eRateUnit;
using ros2_pulse::core::LineFollower;
using ros2_pulse::core::mapWindow;
using ros2_pulse::core::parseLog;
using ros2_pulse::core::sMapperOptions;
using ros2_pulse::core::sMetricSample;
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;

auto defaultGlob() -> std::string {
    const char* tmp = std::getenv("TMPDIR");
    const std::string dir = (tmp != nullptr && *tmp != '\0') ? tmp : "/tmp";
    return dir + "/topic_freq.*.log";
}

auto expandGlob(const std::string& pattern) -> std::vector<std::string> {
    std::vector<std::string> out;
    if (pattern.empty()) return out;
    glob_t g{};
    if (::glob(pattern.c_str(), 0, nullptr, &g) == 0) {
        for (std::size_t i = 0; i < g.gl_pathc; ++i) out.emplace_back(g.gl_pathv[i]);
    }
    globfree(&g);
    return out;
}

auto toTime(long long ns) -> builtin_interfaces::msg::Time {
    builtin_interfaces::msg::Time t;
    if (ns < 0) ns = 0;
    t.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return t;
}

auto point(std::uint8_t type, double value) -> StatisticDataPoint {
    StatisticDataPoint p;
    p.data_type = type;
    p.data = value;
    return p;
}

// @p all_points: rclcpp's layout (metrics_source:=metric) always carries all five points, an
// unmeasured one as NaN, which is what libstatistics_collector publishes for an empty window.
// The topic layout keeps its optional points (absent = not measured), in the same relative order.
auto toMessage(const sMetricSample& s, bool all_points) -> MetricsMessage {
    MetricsMessage m;
    m.measurement_source_name = s.measurement_source_name;
    m.metrics_source = s.metrics_source;
    m.unit = s.unit;
    m.window_start = toTime(s.window_start_ns);
    m.window_stop = toTime(s.window_stop_ns);
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    auto add = [&](std::uint8_t type, bool has, double value) {
        if (has) {
            m.statistics.push_back(point(type, value));
        } else if (all_points) {
            m.statistics.push_back(point(type, kNaN));
        }
    };
    // libstatistics_collector's GenerateStatisticMessage order: AVERAGE, MAXIMUM, MINIMUM,
    // SAMPLE_COUNT, STDDEV. Some consumers index the points by position.
    add(StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE, s.has_average, s.average);
    add(StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM, s.has_maximum, s.maximum);
    add(StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM, s.has_minimum, s.minimum);
    add(StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT, true, s.sample_count);
    add(StatisticDataType::STATISTICS_DATA_TYPE_STDDEV, s.has_stddev, s.stddev);
    return m;
}

#ifdef ROS2_PULSE_HAVE_ROSGRAPH_MONITOR_MSGS
using rosgraph_monitor_msgs::msg::TopicStatistic;
using rosgraph_monitor_msgs::msg::TopicStatistics;

auto toDuration(long long ns) -> builtin_interfaces::msg::Duration {
    builtin_interfaces::msg::Duration d;
    if (ns < 0) ns = 0;
    d.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    d.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return d;
}

// Only fields the probe measures are set, see topic_statistic_mapper.hpp: `min` stays zero on
// PUBLISHED_PERIOD (not tracked) and whenever the receive interval distribution was not measured,
// `max` stays zero when gap tracking is off, and TAKE_AGE comes only with jitter tracking on. A
// negative age (the publisher's clock ahead of ours) is clamped to zero: Duration here is the
// non-negative span graph-monitor's shim fills.
auto toMessage(const ros2_pulse::core::sTopicStatisticsBatch& b) -> TopicStatistics {
    using ros2_pulse::core::eTopicStatisticType;
    TopicStatistics m;
    m.timestamp = toTime(b.timestamp_ns);
    m.statistics.reserve(b.statistics.size());
    for (const auto& s : b.statistics) {
        TopicStatistic t;
        switch (s.type) {
            case eTopicStatisticType::PublishedPeriod:
                t.statistic_type = TopicStatistic::PUBLISHED_PERIOD;
                break;
            case eTopicStatisticType::ReceivedPeriod:
                t.statistic_type = TopicStatistic::RECEIVED_PERIOD;
                break;
            case eTopicStatisticType::TakeAge:
                t.statistic_type = TopicStatistic::TAKE_AGE;
                break;
        }
        t.node_name = s.node_name;
        t.topic_name = s.topic_name;
        t.window_count = s.window_count;
        t.mean = toDuration(s.mean_ns);
        if (s.has_min) t.min = toDuration(s.min_ns);
        if (s.has_max) t.max = toDuration(s.max_ns);
        m.statistics.push_back(std::move(t));
    }
    return m;
}
#endif

struct sFollowed {
    LineFollower follower;
    bool sniffed{false};  // first line seen, format decided
    bool jsonl{false};
    bool warned_ambiguous{false};  // graph_monitor: several nodes in this process, said once
};

class PulseBridge : public rclcpp::Node {
public:
    PulseBridge() : Node("pulse_bridge") {
        m_files = declare_parameter<std::vector<std::string>>("files", std::vector<std::string>{});
        m_glob = declare_parameter<std::string>("glob", defaultGlob());
        const auto topic = declare_parameter<std::string>("topic", "/statistics");
        const auto unit = declare_parameter<std::string>("unit", "ms");
        double period = declare_parameter<double>("poll_period_s", 1.0);
        m_opts.source_name = declare_parameter<std::string>("source_name", "");
        const auto source = declare_parameter<std::string>("metrics_source", "topic");

        if (unit == "Hz") {
            m_opts.unit = eRateUnit::Rate_hz;
        } else if (unit != "ms") {
            RCLCPP_WARN(get_logger(), "unit '%s' is neither ms nor Hz, using ms", unit.c_str());
        }
        if (source == "metric") {
            m_opts.metrics_source = eMetricsSource::Metric;
        } else if (source != "topic") {
            RCLCPP_WARN(get_logger(), "metrics_source '%s' is neither topic nor metric, using topic",
                        source.c_str());
        }
        if (period <= 0.0) {
            RCLCPP_WARN(get_logger(), "poll_period_s %.3f is not positive, using 1.0", period);
            period = 1.0;
        }

        const bool graph_monitor = declare_parameter<bool>("graph_monitor", false);
        const auto gm_topic =
            declare_parameter<std::string>("graph_monitor_topic", "/topic_statistics");

        m_pub = create_publisher<MetricsMessage>(topic, rclcpp::QoS(10));
        if (graph_monitor) {
#ifdef ROS2_PULSE_HAVE_ROSGRAPH_MONITOR_MSGS
            // QoS matches graph-monitor's subscription (reliable, keep last 10).
            m_gm_pub = create_publisher<TopicStatistics>(gm_topic, rclcpp::QoS(10));
            RCLCPP_INFO(get_logger(), "also publishing rosgraph_monitor_msgs on %s",
                        gm_topic.c_str());
#else
            (void)gm_topic;
            RCLCPP_WARN(get_logger(),
                        "graph_monitor is set, but this pulse_bridge was built without "
                        "rosgraph_monitor_msgs; only %s is published",
                        topic.c_str());
#endif
        }
        m_timer = create_wall_timer(std::chrono::duration<double>(period), [this]() { tick(); });
        RCLCPP_INFO(get_logger(), "publishing %s from %zu file(s) + glob '%s', every %.2f s",
                    topic.c_str(), m_files.size(), m_glob.c_str(), period);
    }

private:
    void discover() {
        auto add = [this](const std::string& path) {
            if (m_followed.find(path) == m_followed.end()) {
                m_followed.emplace(path, sFollowed{LineFollower(path)});
                RCLCPP_INFO(get_logger(), "following %s", path.c_str());
            }
        };
        for (const auto& f : m_files) add(f);
        for (const auto& f : expandGlob(m_glob)) add(f);
    }

    void tick() {
        discover();
        for (auto& [path, f] : m_followed) {
            for (const auto& line : f.follower.poll()) {
                if (line.empty()) continue;
                if (!f.sniffed) {
                    f.sniffed = true;
                    f.jsonl = line[0] == '{';
                    if (!f.jsonl) {
                        RCLCPP_WARN(get_logger(),
                                    "%s is not a jsonl log (first line starts with '%c'); set "
                                    "ROS_TOPIC_STATS_FORMAT=jsonl on the probed process. Ignoring "
                                    "this file.",
                                    path.c_str(), line[0]);
                    }
                }
                if (!f.jsonl) break;
                for (const auto& win : parseLog(line + "\n")) {
                    const bool all = m_opts.metrics_source == eMetricsSource::Metric;
                    for (const auto& s : mapWindow(win, m_opts)) {
                        m_pub->publish(toMessage(s, all));
                    }
#ifdef ROS2_PULSE_HAVE_ROSGRAPH_MONITOR_MSGS
                    if (m_gm_pub) publishGraphMonitor(path, f, win);
#endif
                }
            }
        }
    }

#ifdef ROS2_PULSE_HAVE_ROSGRAPH_MONITOR_MSGS
    void publishGraphMonitor(const std::string& path, sFollowed& f,
                             const ros2_pulse::core::sLogWindow& win) {
        const auto batch = ros2_pulse::core::mapWindowTopicStatistics(win);
        if (batch.ambiguous_node && !f.warned_ambiguous) {
            f.warned_ambiguous = true;
            RCLCPP_WARN(get_logger(),
                        "%s: %zu active nodes in this process. The probe counts per topic per "
                        "process, so its windows cannot be attributed to one node's endpoints "
                        "and are not sent to graph-monitor",
                        path.c_str(), win.nodes.size());
        }
        if (!batch.statistics.empty()) m_gm_pub->publish(toMessage(batch));
    }

    rclcpp::Publisher<TopicStatistics>::SharedPtr m_gm_pub;
#endif

    std::vector<std::string> m_files;
    std::string m_glob;
    sMapperOptions m_opts;
    std::map<std::string, sFollowed> m_followed;
    rclcpp::Publisher<MetricsMessage>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
};

}  // namespace

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PulseBridge>());
    rclcpp::shutdown();
    return 0;
}

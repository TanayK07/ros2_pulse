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
//
// Text-format logs are refused per file with one warning: their windows span several lines and
// the bridge reads line by line. Set ROS_TOPIC_STATS_FORMAT=jsonl on the probed process.

#include <glob.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
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

namespace {

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

auto toMessage(const sMetricSample& s) -> MetricsMessage {
    MetricsMessage m;
    m.measurement_source_name = s.measurement_source_name;
    m.metrics_source = s.metrics_source;
    m.unit = s.unit;
    m.window_start = toTime(s.window_start_ns);
    m.window_stop = toTime(s.window_stop_ns);
    if (s.has_average) {
        m.statistics.push_back(point(StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE, s.average));
    }
    if (s.has_maximum) {
        m.statistics.push_back(point(StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM, s.maximum));
    }
    m.statistics.push_back(
        point(StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT, s.sample_count));
    return m;
}

struct sFollowed {
    LineFollower follower;
    bool sniffed{false};  // first line seen, format decided
    bool jsonl{false};
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

        if (unit == "Hz") {
            m_opts.unit = eRateUnit::Rate_hz;
        } else if (unit != "ms") {
            RCLCPP_WARN(get_logger(), "unit '%s' is neither ms nor Hz, using ms", unit.c_str());
        }
        if (period <= 0.0) {
            RCLCPP_WARN(get_logger(), "poll_period_s %.3f is not positive, using 1.0", period);
            period = 1.0;
        }

        m_pub = create_publisher<MetricsMessage>(topic, rclcpp::QoS(10));
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
                    for (const auto& s : mapWindow(win, m_opts)) m_pub->publish(toMessage(s));
                }
            }
        }
    }

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

// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the pure-C++ window formatter. No ROS / tracetools dependency.
//
// These pin the exact on-disk byte format so the probe can build a whole window in one buffer and
// emit it with a single write (issue #4) without changing what consumers parse.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"
#include "ros2_pulse/core/window_format.hpp"

using ros2_pulse::core::defaultOutputPath;
using ros2_pulse::core::formatWindow;
using ros2_pulse::core::sTopicStat;

namespace {

auto stat(const std::string& topic, uint64_t pub_count, uint64_t recv_inter_count,
          uint64_t recv_intra_count, double pub_hz, double recv_inter_hz, double recv_intra_hz)
    -> sTopicStat {
    sTopicStat s;
    s.topic = topic;
    s.pub_inter_count = pub_count;
    s.recv_inter_count = recv_inter_count;
    s.recv_intra_count = recv_intra_count;
    s.pub_inter_hz = pub_hz;
    s.recv_inter_hz = recv_inter_hz;
    s.recv_intra_hz = recv_intra_hz;
    return s;
}

}  // namespace

// A mixed window over the split buckets (issue #1): a publish-only topic (TOPIC line only), an
// intra-receive-only topic (RECV line only), and a same-process pub+sub topic (both lines,
// independent values). Locks the header (ts_ns / window_s to 3dp), the TOPIC / RECV precision
// (6dp), the emit gates, and the single trailing blank line.
TEST(WindowFormat, MixedWindowExactBytes) {
    std::vector<sTopicStat> stats = {
        stat("/scan", 100, 0, 0, 20.0, 0.0, 0.0),
        stat("/points", 0, 0, 150, 0.0, 0.0, 30.0),
        stat("/both", 50, 50, 0, 10.0, 10.0, 0.0),
    };
    std::vector<std::string> nodes = {"/perception", "/planner"};

    const std::string got = formatWindow(stats, nodes, 1782887153899445923LL, 5.0);

    const std::string want =
        "# ts_ns=1782887153899445923 window_s=5.000\n"
        "TOPIC /scan 20.000000\n"
        "RECV /points inter=0.000000 intra=30.000000\n"
        "TOPIC /both 10.000000\n"
        "RECV /both inter=10.000000 intra=0.000000\n"
        "NODE /perception\n"
        "NODE /planner\n"
        "\n";
    EXPECT_EQ(got, want);
}

// A declared-but-silent topic (no traffic) emits the zero TOPIC line and NO RECV line
// (issue #7 behaviour — locked here so the atomic-write refactor does not change it).
TEST(WindowFormat, IdleTopicEmitsZeroTopicLineOnly) {
    std::vector<sTopicStat> stats = {stat("/idle", 0, 0, 0, 0.0, 0.0, 0.0)};
    std::vector<std::string> nodes = {};

    const std::string got = formatWindow(stats, nodes, 42LL, 1.5);

    const std::string want =
        "# ts_ns=42 window_s=1.500\n"
        "TOPIC /idle 0.000000\n"
        "\n";
    EXPECT_EQ(got, want);
}

// Nodes-only window (no topics yet) still emits a valid, closed block.
TEST(WindowFormat, NodesOnlyWindow) {
    std::vector<sTopicStat> stats = {};
    std::vector<std::string> nodes = {"/talker"};

    const std::string got = formatWindow(stats, nodes, 7LL, 5.0);

    const std::string want =
        "# ts_ns=7 window_s=5.000\n"
        "NODE /talker\n"
        "\n";
    EXPECT_EQ(got, want);
}

// The per-process default path embeds the pid so preloaded processes stop sharing one file.
TEST(WindowFormat, DefaultOutputPathEmbedsPid) {
    EXPECT_EQ(defaultOutputPath(4242), "/root/ssd2tb/logs/topic_freq.4242.log");
    EXPECT_EQ(defaultOutputPath(1), "/root/ssd2tb/logs/topic_freq.1.log");
    // distinct pids must map to distinct files
    EXPECT_NE(defaultOutputPath(100), defaultOutputPath(101));
}

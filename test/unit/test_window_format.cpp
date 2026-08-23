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

    const std::string got =
        formatWindow(stats, nodes, 1782887153899445923LL, 5.0, /*emit_idle=*/false);

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

// A declared-but-silent topic (no traffic) is suppressed by default (issue #7): the window is
// header + blank line only, no TOPIC zero line and no RECV line.
TEST(WindowFormat, IdleTopicSuppressedByDefault) {
    std::vector<sTopicStat> stats = {stat("/idle", 0, 0, 0, 0.0, 0.0, 0.0)};
    std::vector<std::string> nodes = {};

    const std::string got = formatWindow(stats, nodes, 42LL, 1.5, /*emit_idle=*/false);

    const std::string want =
        "# ts_ns=42 window_s=1.500\n"
        "\n";
    EXPECT_EQ(got, want);
}

// With the ROS_PULSE_EMIT_IDLE opt-in the zero TOPIC line comes back (and still no RECV line).
TEST(WindowFormat, IdleTopicZeroLineRestoredByOptIn) {
    std::vector<sTopicStat> stats = {stat("/idle", 0, 0, 0, 0.0, 0.0, 0.0)};
    std::vector<std::string> nodes = {};

    const std::string got = formatWindow(stats, nodes, 42LL, 1.5, /*emit_idle=*/true);

    const std::string want =
        "# ts_ns=42 window_s=1.500\n"
        "TOPIC /idle 0.000000\n"
        "\n";
    EXPECT_EQ(got, want);
}

// Publish-side intra traffic (jazzy+, rclcpp_intra_publish) renders the ADDITIVE `PUB` line,
// the legacy TOPIC line format stays untouched so existing parsers keep working. An intra-only
// publisher gets a PUB line and no TOPIC line.
TEST(WindowFormat, IntraPublisherEmitsAdditivePubLine) {
    sTopicStat s;
    s.topic = "/points";
    s.pub_intra_count = 150;
    s.pub_intra_hz = 30.0;
    std::vector<sTopicStat> stats = {s};
    std::vector<std::string> nodes = {};

    const std::string got = formatWindow(stats, nodes, 42LL, 5.0, /*emit_idle=*/false);

    const std::string want =
        "# ts_ns=42 window_s=5.000\n"
        "PUB /points inter=0.000000 intra=30.000000\n"
        "\n";
    EXPECT_EQ(got, want);
}

// A stalled subscription (delivered once, then upstream died) must render an explicit RECV
// zero line every window (KNOWN_ISSUES #12), while its TOPIC line stays idle-suppressed.
TEST(WindowFormat, StalledTopicEmitsRecvZeroLine) {
    sTopicStat s;
    s.topic = "/scan";
    s.recv_endpoint_seen = true;  // proven receive endpoint, all counts zero this window
    std::vector<sTopicStat> stats = {s};
    std::vector<std::string> nodes = {};

    const std::string got = formatWindow(stats, nodes, 42LL, 1.5, /*emit_idle=*/false);

    const std::string want =
        "# ts_ns=42 window_s=1.500\n"
        "RECV /scan inter=0.000000 intra=0.000000\n"
        "\n";
    EXPECT_EQ(got, want);
}

// Expected-rate warnings (ROADMAP R1) render verbatim after the NODE lines, before the blank
// line, additive, so TOPIC/PUB/RECV/NODE parsers never see a changed prefix.
TEST(WindowFormat, WarningsRenderAfterNodes) {
    std::vector<sTopicStat> stats = {stat("/scan", 6, 0, 0, 1.2, 0.0, 0.0)};
    std::vector<std::string> nodes = {"/perception"};
    const std::vector<std::string> warnings = {
        "WARN TOPIC /scan hz=1.200000 expected=[18,22]",
        "WARN NODE /planner missing",
    };

    const std::string got = formatWindow(stats, nodes, 42LL, 5.0, /*emit_idle=*/false, warnings);

    const std::string want =
        "# ts_ns=42 window_s=5.000\n"
        "TOPIC /scan 1.200000\n"
        "NODE /perception\n"
        "WARN TOPIC /scan hz=1.200000 expected=[18,22]\n"
        "WARN NODE /planner missing\n"
        "\n";
    EXPECT_EQ(got, want);
}

// Nodes-only window (no topics yet) still emits a valid, closed block.
TEST(WindowFormat, NodesOnlyWindow) {
    std::vector<sTopicStat> stats = {};
    std::vector<std::string> nodes = {"/talker"};

    const std::string got = formatWindow(stats, nodes, 7LL, 5.0, /*emit_idle=*/false);

    const std::string want =
        "# ts_ns=7 window_s=5.000\n"
        "NODE /talker\n"
        "\n";
    EXPECT_EQ(got, want);
}

// The per-process default path embeds the pid so preloaded processes stop sharing one file.
// The default must be a path that exists and is writable on an arbitrary machine. The previous
// default, /root/ssd2tb/logs/, was the original field-test box's SSD mount: on every other
// system the directory doesn't exist, so a fresh install silently dropped every window (the
// probe warns once on stderr and keeps running, correct fail-safe behaviour, wrong default).
// Even test/orin/run_orin_probe_test.sh had to special-case it. POSIX's answer is TMPDIR with a
// /tmp fallback; the caller passes TMPDIR in so this stays a pure function of its arguments.
TEST(WindowFormat, DefaultOutputPathEmbedsPid) {
    EXPECT_EQ(defaultOutputPath(4242), "/tmp/topic_freq.4242.log");
    EXPECT_EQ(defaultOutputPath(1), "/tmp/topic_freq.1.log");
    // distinct pids must map to distinct files
    EXPECT_NE(defaultOutputPath(100), defaultOutputPath(101));
}

TEST(WindowFormat, DefaultOutputPathHonoursTmpdir) {
    EXPECT_EQ(defaultOutputPath(7, "/run/user/1000"), "/run/user/1000/topic_freq.7.log");
    // Trailing slashes are the most common TMPDIR spelling mistake; never emit '//'.
    EXPECT_EQ(defaultOutputPath(7, "/scratch/"), "/scratch/topic_freq.7.log");
    // Unset or empty TMPDIR falls back to /tmp; a relative TMPDIR is rejected too, the probe
    // runs inside arbitrary processes whose cwd is unknown and possibly unwritable, so a
    // relative path would scatter logs (or open-failures) across random directories.
    EXPECT_EQ(defaultOutputPath(7, nullptr), "/tmp/topic_freq.7.log");
    EXPECT_EQ(defaultOutputPath(7, ""), "/tmp/topic_freq.7.log");
    EXPECT_EQ(defaultOutputPath(7, "relative/dir"), "/tmp/topic_freq.7.log");
    // Root itself: legal, and must not double the slash.
    EXPECT_EQ(defaultOutputPath(7, "/"), "/topic_freq.7.log");
}

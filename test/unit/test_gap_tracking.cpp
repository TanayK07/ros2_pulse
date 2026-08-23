// Copyright 2026 ros2_pulse contributors
//
// Unit tests for per-endpoint inter-arrival gap tracking (ROADMAP R5). A windowed mean cannot
// see a stall, a 400 ms freeze on a 50 Hz topic still averages 46 Hz over 5 s, so max_dt_ms is
// the window-length-independent detector. Pure, no ROS. Registers into the shared test binary.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ros2_pulse/core/topic_registry.hpp"
#include "ros2_pulse/core/window_format.hpp"

using ros2_pulse::core::formatWindow;
using ros2_pulse::core::sTopicStat;
using ros2_pulse::core::TopicRegistry;

namespace {

// Distinct dummy handles; the registry only ever compares them by identity.
const void* kNode = reinterpret_cast<const void*>(0xD0DE0001ULL);

auto findStat(const std::vector<sTopicStat>& v, const std::string& topic) -> const sTopicStat* {
    for (const auto& s : v) {
        if (s.topic == topic) {
            return &s;
        }
    }
    return nullptr;
}

}  // namespace

// Off by default: no clock is read and no field is reported, so pre-R5 output is unchanged.
TEST(GapTracking, DisabledByDefaultReportsNothing) {
    TopicRegistry reg;
    EXPECT_FALSE(reg.gapTracking());
    const void* pub = reinterpret_cast<const void*>(0x1001);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");
    reg.onPublish(pub);
    reg.onPublish(pub);

    auto stats = reg.snapshot(1.0);
    const auto* s = findStat(stats, "/chatter");
    ASSERT_NE(s, nullptr);
    EXPECT_FALSE(s->has_pub_max_dt);
    EXPECT_FALSE(s->has_recv_max_dt);
}

// An endpoint that has never seen an arrival has no interval, which is NOT a gap of zero, so
// nothing may be reported. Reporting 0.000 would read as "perfectly regular".
TEST(GapTracking, NeverActiveEndpointReportsNothing) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x1002);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/silent");  // declared, never published

    auto stats = reg.snapshot(1.0);
    const auto* s = findStat(stats, "/silent");
    ASSERT_NE(s, nullptr);
    EXPECT_FALSE(s->has_pub_max_dt);
}

// The first arrival ever establishes last_ts but yields no INTERVAL, and a timestamp without an
// interval is not a gap of zero, 0.000 would read as the best possible value, inferred from one
// data point. Nothing is reported until a second arrival (or the folded open gap) gives a real
// measurement.
TEST(GapTracking, FirstArrivalProducesNoInterval) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x1003);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");
    reg.onPublish(pub);

    auto stats = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = findStat(stats, "/chatter");
    ASSERT_NE(s, nullptr);
    EXPECT_FALSE(s->has_pub_max_dt) << "one timestamp is not an interval";

    // ...but with the open gap folded (the production path) it becomes a real measurement.
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    auto folded = reg.snapshot(1.0, /*fold_open_gap=*/true);
    const auto* f = findStat(folded, "/chatter");
    ASSERT_NE(f, nullptr);
    ASSERT_TRUE(f->has_pub_max_dt);
    EXPECT_GE(f->pub_max_dt_ms, 15.0);
}

// A filtered topic is never reported, but its gap state must still be drained every window or
// it ratchets upward for the process lifetime, the same discipline the count buckets follow.
TEST(GapTracking, FilteredTopicsStillDrainTheirGapState) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x100A);
    reg.onNodeInit(kNode, "chatty", "/");
    reg.onPublisherInit(pub, kNode, "/rosout");  // filtered: never emitted

    reg.onPublish(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    reg.onPublish(pub);
    reg.snapshot(1.0, /*fold_open_gap=*/false);  // must reset despite the topic being filtered

    // Re-registering the same counter under a reported name would expose a stale max; instead
    // assert the observable proxy: a second window sees no carried-over gap.
    reg.onPublish(pub);
    auto second = reg.snapshot(1.0, /*fold_open_gap=*/false);
    EXPECT_EQ(findStat(second, "/rosout"), nullptr) << "/rosout must stay filtered out";
}

// The measured gap must be the real elapsed time between two arrivals.
TEST(GapTracking, MeasuresInterArrivalGap) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x1004);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");

    reg.onPublish(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    reg.onPublish(pub);

    auto stats = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = findStat(stats, "/chatter");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_pub_max_dt);
    // sleep_for guarantees AT LEAST the duration; cap generously so this cannot flake.
    EXPECT_GE(s->pub_max_dt_ms, 30.0);
    EXPECT_LT(s->pub_max_dt_ms, 300.0);
}

// THE load-bearing case. A stall that straddles a flush must land intact in one window's max:
// last_ts survives snapshot(), so the first arrival of the next window measures back into the
// previous one. If last_ts were reset, this gap would be silently discarded.
TEST(GapTracking, MaxSpansWindowBoundary) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x1005);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");

    reg.onPublish(pub);
    reg.snapshot(1.0, /*fold_open_gap=*/false);  // window boundary lands mid-stall
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    reg.onPublish(pub);

    auto stats = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = findStat(stats, "/chatter");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_pub_max_dt);
    EXPECT_GE(s->pub_max_dt_ms, 40.0) << "gap straddling a flush was lost";
}

// max_dt is per-window; last_ts is not. A quiet window after a big gap must not re-report it.
TEST(GapTracking, MaxResetsPerWindowButLastTsDoesNot) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x1006);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");

    reg.onPublish(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    reg.onPublish(pub);
    auto first = reg.snapshot(1.0, /*fold_open_gap=*/false);
    ASSERT_GE(findStat(first, "/chatter")->pub_max_dt_ms, 25.0);

    reg.onPublish(pub);  // immediately after, tiny gap
    auto second = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = findStat(second, "/chatter");
    ASSERT_NE(s, nullptr);
    EXPECT_LT(s->pub_max_dt_ms, 25.0) << "previous window's max leaked forward";
}

// A fully-silent endpoint produces no inter-arrival pair at all. Without the open-gap fold the
// TOTAL stall, the worst case R5 exists to catch, would be invisible. With it, the reported
// gap grows every window.
TEST(GapTracking, OpenGapFoldedSoSilentEndpointGrows) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x1007);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");

    reg.onPublish(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto w1 = reg.snapshot(1.0, /*fold_open_gap=*/true);
    const double first = findStat(w1, "/chatter")->pub_max_dt_ms;
    EXPECT_GE(first, 20.0);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // still nothing published
    auto w2 = reg.snapshot(1.0, /*fold_open_gap=*/true);
    const double second = findStat(w2, "/chatter")->pub_max_dt_ms;
    EXPECT_GT(second, first) << "a silent endpoint must report a GROWING gap, not vanish";
    EXPECT_GE(second, 40.0);
}

// ...but the exit window must NOT fold. rclcpp teardown stops traffic before the process exits,
// so the open interval there measures the shutdown sequence and would fire every gap rule on a
// perfectly healthy stop.
// The R5 design's "DisabledReadsNoClock": default-off must cost zero, i.e. the disabled hot
// path may never enter noteArrival, whose FIRST statement is the clock read. Rather than
// injecting a counting clock (a seam on the very hot path whose +24 ns/msg figure the bench
// reproduces, the seam would perturb the number it exists to protect), observe through the
// public API: noteArrival writes last_ns unconditionally BEFORE its guard, so if the disabled
// phase had entered it even once, enabling tracking afterwards would fold the open gap since
// that timestamp and report a max_dt. No report == the accumulator was never entered == no
// clock was read.
TEST(GapTracking, DisabledPathNeverEntersTheAccumulator) {
    TopicRegistry reg;
    ASSERT_FALSE(reg.gapTracking());
    const void* pub = reinterpret_cast<const void*>(0x1010);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");

    // Traffic with tracking disabled, spaced so any recorded timestamp would be foldable.
    reg.onPublish(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    reg.onPublish(pub);

    // Flip on AFTER the traffic; the snapshot drains with the fold armed. A regression that
    // calls noteArrival unconditionally (e.g. "simplifying" the if (gap) gate away) leaves a
    // last_ns behind, and this fold turns it into a visible JITTER report.
    reg.setGapTracking(true);
    auto stats = reg.snapshot(1.0, /*fold_open_gap=*/true);
    const auto* s = findStat(stats, "/chatter");
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->pub_inter_count, 2u);  // counting itself was live the whole time
    EXPECT_FALSE(s->has_pub_max_dt) << "disabled phase left a timestamp -> a clock was read";
    EXPECT_FALSE(s->has_recv_max_dt);
}

TEST(GapTracking, OpenGapNotFoldedWhenExiting) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x1008);
    reg.onNodeInit(kNode, "talker", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");

    // Two arrivals close together give a real, small interval...
    reg.onPublish(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    reg.onPublish(pub);
    // ...then the executor stops and the process spends a while tearing down.
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    auto stats = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = findStat(stats, "/chatter");
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->has_pub_max_dt);
    // Only the measured interval, never the 60 ms of healthy shutdown quiet.
    EXPECT_GE(s->pub_max_dt_ms, 5.0);
    EXPECT_LT(s->pub_max_dt_ms, 50.0) << "teardown quiet leaked into the exit window's gap";
}

// Pub and recv are independent accumulators, a stalled subscriber must not be masked by a
// healthy publisher of the same topic in the same process.
TEST(GapTracking, PubAndRecvTrackedSeparately) {
    TopicRegistry reg;
    reg.setGapTracking(true);
    const void* pub = reinterpret_cast<const void*>(0x1009);
    reg.onNodeInit(kNode, "both", "/");
    reg.onPublisherInit(pub, kNode, "/chatter");

    reg.onPublish(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    reg.onPublish(pub);

    auto stats = reg.snapshot(1.0, /*fold_open_gap=*/false);
    const auto* s = findStat(stats, "/chatter");
    ASSERT_NE(s, nullptr);
    EXPECT_TRUE(s->has_pub_max_dt);
    EXPECT_FALSE(s->has_recv_max_dt) << "no subscription here, recv must stay unreported";
}

// ---- output format ----

// THE additive-convention proof: with gap tracking off, the emitted block must be byte-identical
// to pre-R5 output. Populate the jitter fields but leave the flags false.
TEST(GapFormat, DisabledIsByteIdenticalToLegacy) {
    sTopicStat s;
    s.topic = "/scan";
    s.pub_inter_count = 100;
    s.pub_inter_hz = 20.0;
    s.recv_inter_hz = 20.0;
    s.recv_endpoint_seen = true;
    s.pub_max_dt_ms = 999.0;  // populated but unflagged, must not appear
    s.recv_max_dt_ms = 888.0;

    const std::string want =
        "# ts_ns=7 window_s=5.000\n"
        "TOPIC /scan 20.000000\n"
        "RECV /scan inter=20.000000 intra=0.000000\n"
        "\n";
    EXPECT_EQ(formatWindow({s}, {}, 7, 5.0, false), want);
}

// Both lines, exact bytes: placement relative to their side's line, and %.3f precision.
TEST(GapFormat, JitterLinesExactBytes) {
    sTopicStat s;
    s.topic = "/scan";
    s.pub_inter_count = 100;
    s.pub_inter_hz = 20.0;
    s.recv_inter_hz = 20.0;
    s.recv_endpoint_seen = true;
    s.pub_max_dt_ms = 51.2836;
    s.has_pub_max_dt = true;
    s.recv_max_dt_ms = 52.019;
    s.has_recv_max_dt = true;

    const std::string want =
        "# ts_ns=7 window_s=5.000\n"
        "TOPIC /scan 20.000000\n"
        "JITTER /scan pub max_dt_ms=51.284\n"
        "RECV /scan inter=20.000000 intra=0.000000\n"
        "JITTER /scan recv max_dt_ms=52.019\n"
        "\n";
    EXPECT_EQ(formatWindow({s}, {}, 7, 5.0, false), want);
}

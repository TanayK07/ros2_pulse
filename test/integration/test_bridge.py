# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# The graph-monitor cases (rosgraph_monitor_msgs/TopicStatistics on /topic_statistics) skip when
# rosgraph_monitor_msgs is not installed; the flag-never-breaks-/statistics case runs either way.
#
# pulse_bridge integration:a jsonl probe log is republished on /statistics as
# statistics_msgs/MetricsMessage in the shape rclcpp's built-in topic statistics use (unit "ms",
# AVERAGE = message period, SAMPLE_COUNT, MAXIMUM / MINIMUM / STDDEV of the intervals, and
# message_age in the metrics_source:=metric layout), lines appended AFTER the bridge
# started are picked up (tail-follow), and a text-format log is refused with a warning instead
# of a crash. Distro-independent: no probe, no LD_PRELOAD, the log is written by hand.
import math
import os
import subprocess
import sys
import tempfile
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]
from ament_index_python.packages import get_package_prefix  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from statistics_msgs.msg import MetricsMessage, StatisticDataType  # noqa: E402

# The README's example window, verbatim: /scan at 20 Hz for 5 s with a 21.284 ms worst gap.
WIN1 = ('{"ts_ns":"1782887153899445923","window_s":5.000,"topics":[{"topic":"/scan",'
        '"pub_inter_hz":20.000000,"pub_intra_hz":0.000000,"recv_inter_hz":20.000000,'
        '"recv_intra_hz":0.000000,"recv_endpoint_seen":true,"recv_max_dt_ms":21.284}],'
        '"nodes":["/perception"],"warns":[]}\n')
# Five seconds later, rate halved, no gap measured this window.
WIN2 = ('{"ts_ns":"1782887158899445923","window_s":5.000,"topics":[{"topic":"/scan",'
        '"pub_inter_hz":10.000000,"pub_intra_hz":0.000000,"recv_inter_hz":10.000000,'
        '"recv_intra_hz":0.000000,"recv_endpoint_seen":true}],'
        '"nodes":["/perception"],"warns":[]}\n')
# Same window from a probe with gap tracking on and a source-timestamped transport: the receive
# interval distribution and the message age (publish-to-take latency at rmw_take).
WIN_PARITY = ('{"ts_ns":"1782887153899445923","window_s":5.000,"topics":[{"topic":"/scan",'
              '"recv_inter_hz":20.000000,"recv_intra_hz":0.000000,"recv_endpoint_seen":true,'
              '"recv_max_dt_ms":52.019,"recv_min_dt_ms":48.120,"recv_stddev_dt_ms":0.912,'
              '"recv_age_avg_ms":1.250,"recv_age_min_ms":0.800,"recv_age_max_ms":3.100,'
              '"recv_age_stddev_ms":0.400,"recv_age_count":99}],'
              '"nodes":["/perception"],"warns":[]}\n')
# libstatistics_collector's GenerateStatisticMessage order.
ALL_FIVE = [StatisticDataType.STATISTICS_DATA_TYPE_AVERAGE,
            StatisticDataType.STATISTICS_DATA_TYPE_MAXIMUM,
            StatisticDataType.STATISTICS_DATA_TYPE_MINIMUM,
            StatisticDataType.STATISTICS_DATA_TYPE_SAMPLE_COUNT,
            StatisticDataType.STATISTICS_DATA_TYPE_STDDEV]
TEXT_WINDOW = ("# ts_ns=1782887153899445923 window_s=5.000\n"
               "TOPIC /scan PUB 20.000000 RECV 20.000000\n")


def bridge_path():
    p = os.path.join(get_package_prefix(ph.PKG), "lib", ph.PKG, "pulse_bridge")
    assert os.path.exists(p), f"pulse_bridge not installed: {p}"
    return p


class Collector(Node):
    def __init__(self, topic):
        super().__init__("pulse_bridge_test_collector")
        self.msgs = []
        self.create_subscription(MetricsMessage, topic, self.msgs.append, 10)

    def wait_for(self, n, timeout_s):
        deadline = time.monotonic() + timeout_s
        while len(self.msgs) < n and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return len(self.msgs) >= n


def stats_by_type(msg):
    return {s.data_type: s.data for s in msg.statistics}


@pytest.fixture
def ros():
    # Private domain so a host node with built-in topic statistics on cannot leak into the test.
    os.environ["ROS_DOMAIN_ID"] = str(1 + os.getpid() % 100)
    rclpy.init()
    yield
    rclpy.shutdown()


def start_bridge(log_path, extra_params=()):
    return subprocess.Popen(
        [bridge_path(), "--ros-args", "-p", f'files:=["{log_path}"]',
         "-p", "poll_period_s:=0.2", *extra_params],
        env=dict(os.environ), stderr=subprocess.PIPE, text=True)


def stop(proc):
    proc.terminate()
    try:
        return proc.communicate(timeout=5)[1]
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.communicate()[1]


def test_jsonl_window_becomes_builtin_shaped_statistics(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(WIN1)
        proc = start_bridge(path)
        col = Collector("/statistics")
        try:
            assert col.wait_for(1, 10.0), "no MetricsMessage on /statistics within 10 s"
            m = col.msgs[0]
            assert m.metrics_source == "/scan"
            assert m.measurement_source_name == "/perception"
            assert m.unit == "ms"
            assert (m.window_stop.sec, m.window_stop.nanosec) == (1782887153, 899445923)
            assert (m.window_start.sec, m.window_start.nanosec) == (1782887148, 899445923)
            st = stats_by_type(m)
            assert st[StatisticDataType.STATISTICS_DATA_TYPE_AVERAGE] == pytest.approx(50.0)
            assert st[StatisticDataType.STATISTICS_DATA_TYPE_SAMPLE_COUNT] == 100.0
            assert st[StatisticDataType.STATISTICS_DATA_TYPE_MAXIMUM] == pytest.approx(21.284)

            # tail-follow: a window appended after startup arrives without a restart
            with open(path, "a") as f:
                f.write(WIN2)
            assert col.wait_for(2, 10.0), "appended window was not republished"
            m2 = col.msgs[1]
            st2 = stats_by_type(m2)
            assert m2.window_stop.sec == 1782887158
            assert st2[StatisticDataType.STATISTICS_DATA_TYPE_AVERAGE] == pytest.approx(100.0)
            assert st2[StatisticDataType.STATISTICS_DATA_TYPE_SAMPLE_COUNT] == 50.0
            assert StatisticDataType.STATISTICS_DATA_TYPE_MAXIMUM not in st2
        finally:
            col.destroy_node()
            stop(proc)


def test_minimum_and_stddev_published_when_measured(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(WIN_PARITY)
        proc = start_bridge(path)
        col = Collector("/statistics")
        try:
            assert col.wait_for(1, 10.0), "no MetricsMessage on /statistics within 10 s"
            # The topic layout has one message per topic: no message_age beside it.
            assert not col.wait_for(2, 1.0), [m.metrics_source for m in col.msgs]
            m = col.msgs[0]
            assert m.metrics_source == "/scan"
            T = StatisticDataType
            # libstatistics_collector's GenerateStatisticMessage order, which some consumers
            # index by position.
            assert [p.data_type for p in m.statistics] == ALL_FIVE
            st = stats_by_type(m)
            assert st[T.STATISTICS_DATA_TYPE_AVERAGE] == pytest.approx(50.0)
            assert st[T.STATISTICS_DATA_TYPE_MINIMUM] == pytest.approx(48.12)
            assert st[T.STATISTICS_DATA_TYPE_MAXIMUM] == pytest.approx(52.019)
            assert st[T.STATISTICS_DATA_TYPE_STDDEV] == pytest.approx(0.912)
            assert st[T.STATISTICS_DATA_TYPE_SAMPLE_COUNT] == 100.0
        finally:
            col.destroy_node()
            stop(proc)


def test_metric_layout_matches_rclcpp_and_adds_message_age(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(WIN_PARITY)
        proc = start_bridge(path, ("-p", "metrics_source:=metric"))
        col = Collector("/statistics")
        try:
            assert col.wait_for(2, 10.0), "expected message_period and message_age"
            by_name = {m.metrics_source: m for m in col.msgs}
            assert set(by_name) == {"message_period", "message_age"}
            T = StatisticDataType
            period = by_name["message_period"]
            assert [p.data_type for p in period.statistics] == ALL_FIVE
            assert [p.data_type for p in by_name["message_age"].statistics] == ALL_FIVE
            assert period.measurement_source_name == "/perception"
            assert period.unit == "ms"
            assert stats_by_type(period)[T.STATISTICS_DATA_TYPE_MINIMUM] == pytest.approx(48.12)
            age = by_name["message_age"]
            assert age.measurement_source_name == "/perception"
            assert age.unit == "ms"
            assert (age.window_start, age.window_stop) == (period.window_start,
                                                           period.window_stop)
            st = stats_by_type(age)
            assert st[T.STATISTICS_DATA_TYPE_AVERAGE] == pytest.approx(1.25)
            assert st[T.STATISTICS_DATA_TYPE_MINIMUM] == pytest.approx(0.8)
            assert st[T.STATISTICS_DATA_TYPE_MAXIMUM] == pytest.approx(3.1)
            assert st[T.STATISTICS_DATA_TYPE_STDDEV] == pytest.approx(0.4)
            assert st[T.STATISTICS_DATA_TYPE_SAMPLE_COUNT] == 99.0
        finally:
            col.destroy_node()
            stop(proc)


def test_metric_layout_always_carries_all_five_points(ros):
    # rclcpp's layout publishes every point, an unmeasured one as NaN (an empty
    # libstatistics_collector window), so positional consumers never see a short message.
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(WIN1)  # MAXIMUM measured, no MINIMUM / STDDEV
        proc = start_bridge(path, ("-p", "metrics_source:=metric"))
        col = Collector("/statistics")
        try:
            # message_age comes first, so wait for the pair (no age in this window: it is
            # still published, empty, like upstream's collector).
            assert col.wait_for(2, 10.0), "expected message_age and message_period"
            assert [m.metrics_source for m in col.msgs[:2]] == ["message_age", "message_period"]
            period = col.msgs[1]
            assert [p.data_type for p in period.statistics] == ALL_FIVE
            st = stats_by_type(period)
            T = StatisticDataType
            assert st[T.STATISTICS_DATA_TYPE_AVERAGE] == pytest.approx(50.0)
            assert st[T.STATISTICS_DATA_TYPE_MAXIMUM] == pytest.approx(21.284)
            assert math.isnan(st[T.STATISTICS_DATA_TYPE_MINIMUM])
            assert math.isnan(st[T.STATISTICS_DATA_TYPE_STDDEV])
            assert st[T.STATISTICS_DATA_TYPE_SAMPLE_COUNT] == 100.0
            age = col.msgs[0]
            assert [p.data_type for p in age.statistics] == ALL_FIVE
            st = stats_by_type(age)
            assert st[T.STATISTICS_DATA_TYPE_SAMPLE_COUNT] == 0.0
            for t in (T.STATISTICS_DATA_TYPE_AVERAGE, T.STATISTICS_DATA_TYPE_MAXIMUM,
                      T.STATISTICS_DATA_TYPE_MINIMUM, T.STATISTICS_DATA_TYPE_STDDEV):
                assert math.isnan(st[t]), (t, st[t])
        finally:
            col.destroy_node()
            stop(proc)


def test_unit_hz_publishes_rate_directly(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(WIN1)
        proc = start_bridge(path, ("-p", "unit:=Hz", "-p", "topic:=/pulse_rates"))
        col = Collector("/pulse_rates")
        try:
            assert col.wait_for(1, 10.0)
            m = col.msgs[0]
            assert m.unit == "Hz"
            st = stats_by_type(m)
            assert st[StatisticDataType.STATISTICS_DATA_TYPE_AVERAGE] == pytest.approx(20.0)
            assert StatisticDataType.STATISTICS_DATA_TYPE_MAXIMUM not in st
        finally:
            col.destroy_node()
            stop(proc)


def test_text_format_log_is_refused_with_a_warning_not_a_crash(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.log")
        with open(path, "w") as f:
            f.write(TEXT_WINDOW)
        proc = start_bridge(path)
        col = Collector("/statistics")
        try:
            assert not col.wait_for(1, 2.0), "text-format log must not be republished"
            assert proc.poll() is None, "bridge exited on a text-format log"
        finally:
            col.destroy_node()
            err = stop(proc)
        assert "jsonl" in err, f"expected a jsonl hint on stderr, got: {err!r}"


# --- graph-monitor output (ros-tooling/graph-monitor#14) -------------------------------------
# One process, one node, publishing /scan (pub-side gap measured) and receiving /scan.
GM_WIN = ('{"ts_ns":"1782887153899445923","window_s":5.000,"topics":[{"topic":"/scan",'
          '"pub_inter_hz":20.000000,"pub_intra_hz":0.000000,"recv_inter_hz":10.000000,'
          '"recv_intra_hz":0.000000,"recv_endpoint_seen":true,"pub_max_dt_ms":60.000}],'
          '"nodes":["/perception"],"warns":[]}\n')
# Same traffic, two nodes in the process, from a probe without the per-endpoint split: not
# attributable to one endpoint.
GM_WIN_TWO_NODES = GM_WIN.replace('["/perception"]', '["/perception","/planner"]')
# A component container with the per-endpoint split: /driver publishes /scan, /filter receives
# it (gap tracking on: its own interval distribution and message age), /planner only logged.
GM_WIN_ENDPOINTS = (
    '{"ts_ns":"1782887153899445923","window_s":5.000,"topics":[{"topic":"/scan",'
    '"pub_inter_hz":20.000000,"pub_intra_hz":0.000000,"recv_inter_hz":20.000000,'
    '"recv_intra_hz":0.000000,"recv_endpoint_seen":true}],'
    '"nodes":["/driver","/filter","/planner"],"warns":[],"endpoints":['
    '{"topic":"/scan","side":"pub","id":1,"node":"/driver","inter_hz":20.000000,'
    '"intra_hz":0.000000},'
    '{"topic":"/scan","side":"recv","id":2,"node":"/filter","inter_hz":20.000000,'
    '"intra_hz":0.000000,"max_dt_ms":61.000,"avg_dt_ms":49.500,"min_dt_ms":45.250,'
    '"stddev_dt_ms":1.000,"dt_count":99,"age_avg_ms":1.500,"age_min_ms":-0.250,'
    '"age_max_ms":4.000,"age_count":100},'
    '{"topic":"/rosout","side":"pub","id":3,"node":"/planner","inter_hz":0.200000,'
    '"intra_hz":0.000000}]}\n')

try:
    from rosgraph_monitor_msgs.msg import TopicStatistic, TopicStatistics
    HAVE_GM = True
except ImportError:
    HAVE_GM = False


class GmCollector(Node):
    def __init__(self, topic):
        super().__init__("pulse_bridge_test_gm_collector")
        self.msgs = []
        self.create_subscription(TopicStatistics, topic, self.msgs.append, 10)

    wait_for = Collector.wait_for


@pytest.mark.skipif(not HAVE_GM, reason="rosgraph_monitor_msgs not installed")
def test_graph_monitor_topic_statistics(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(GM_WIN)
        proc = start_bridge(path, ("-p", "graph_monitor:=true"))
        col = GmCollector("/topic_statistics")
        try:
            assert col.wait_for(1, 10.0), "no TopicStatistics on /topic_statistics within 10 s"
            m = col.msgs[0]
            assert (m.timestamp.sec, m.timestamp.nanosec) == (1782887153, 899445923)
            by_type = {s.statistic_type: s for s in m.statistics}
            assert len(m.statistics) == 2
            pub = by_type[TopicStatistic.PUBLISHED_PERIOD]
            assert (pub.node_name, pub.topic_name) == ("/perception", "/scan")
            assert pub.window_count == 100
            assert (pub.mean.sec, pub.mean.nanosec) == (0, 50_000_000)
            assert (pub.max.sec, pub.max.nanosec) == (0, 60_000_000)
            assert (pub.min.sec, pub.min.nanosec) == (0, 0)  # not tracked by the probe
            recv = by_type[TopicStatistic.RECEIVED_PERIOD]
            assert recv.window_count == 50
            assert (recv.mean.sec, recv.mean.nanosec) == (0, 100_000_000)
            assert (recv.max.sec, recv.max.nanosec) == (0, 0)  # no recv gap in this window
        finally:
            col.destroy_node()
            stop(proc)


@pytest.mark.skipif(not HAVE_GM, reason="rosgraph_monitor_msgs not installed")
def test_graph_monitor_reports_every_node_of_a_container(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(GM_WIN_ENDPOINTS)
        proc = start_bridge(path, ("-p", "graph_monitor:=true"))
        col = GmCollector("/topic_statistics")
        try:
            assert col.wait_for(1, 10.0), "no TopicStatistics on /topic_statistics within 10 s"
            m = col.msgs[0]
            got = {(s.node_name, s.statistic_type): s for s in m.statistics}
            assert sorted(got) == sorted([
                ("/driver", TopicStatistic.PUBLISHED_PERIOD),
                ("/filter", TopicStatistic.RECEIVED_PERIOD),
                ("/filter", TopicStatistic.TAKE_AGE)])
            pub = got[("/driver", TopicStatistic.PUBLISHED_PERIOD)]
            assert (pub.mean.sec, pub.mean.nanosec) == (0, 50_000_000)
            assert (pub.min.sec, pub.min.nanosec) == (0, 0)
            recv = got[("/filter", TopicStatistic.RECEIVED_PERIOD)]
            assert recv.window_count == 100
            assert (recv.mean.sec, recv.mean.nanosec) == (0, 49_500_000)
            assert (recv.min.sec, recv.min.nanosec) == (0, 45_250_000)
            assert (recv.max.sec, recv.max.nanosec) == (0, 61_000_000)
            age = got[("/filter", TopicStatistic.TAKE_AGE)]
            assert age.window_count == 100
            assert (age.mean.sec, age.mean.nanosec) == (0, 1_500_000)
            assert (age.min.sec, age.min.nanosec) == (0, 0)  # -0.25 ms clamped
            assert (age.max.sec, age.max.nanosec) == (0, 4_000_000)
        finally:
            col.destroy_node()
            err = stop(proc)
        assert "active nodes in this process" not in err, err


@pytest.mark.skipif(not HAVE_GM, reason="rosgraph_monitor_msgs not installed")
def test_graph_monitor_skips_multi_node_process_with_one_warning(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(GM_WIN_TWO_NODES * 2)
        proc = start_bridge(path, ("-p", "graph_monitor:=true"))
        gm = GmCollector("/topic_statistics")
        col = Collector("/statistics")
        try:
            # /statistics still flows; graph-monitor gets nothing it would misattribute.
            assert col.wait_for(2, 10.0)
            assert not gm.wait_for(1, 1.0)
        finally:
            gm.destroy_node()
            col.destroy_node()
            err = stop(proc)
        assert err.count("active nodes in this process") == 1, err


def test_graph_monitor_is_off_by_default(ros):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(GM_WIN)
        proc = start_bridge(path)
        col = Collector("/statistics")
        try:
            assert col.wait_for(1, 10.0)
            assert col.count_publishers("/topic_statistics") == 0
        finally:
            col.destroy_node()
            stop(proc)


def test_graph_monitor_flag_never_breaks_statistics(ros):
    # Runs in both builds. Without rosgraph_monitor_msgs the parameter warns instead of failing.
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pulse.jsonl")
        with open(path, "w") as f:
            f.write(GM_WIN)
        proc = start_bridge(path, ("-p", "graph_monitor:=true"))
        col = Collector("/statistics")
        try:
            assert col.wait_for(1, 10.0), "/statistics must flow with graph_monitor set"
        finally:
            col.destroy_node()
            err = stop(proc)
        if not HAVE_GM:
            assert "built without rosgraph_monitor_msgs" in err, err

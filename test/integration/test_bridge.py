# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# pulse_bridge integration: a jsonl probe log is republished on /statistics as
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
            assert col.wait_for(1, 10.0), "no MetricsMessage on /statistics within 10 s"
            period = next(m for m in col.msgs if m.metrics_source == "message_period")
            assert [p.data_type for p in period.statistics] == ALL_FIVE
            st = stats_by_type(period)
            T = StatisticDataType
            assert st[T.STATISTICS_DATA_TYPE_AVERAGE] == pytest.approx(50.0)
            assert st[T.STATISTICS_DATA_TYPE_MAXIMUM] == pytest.approx(21.284)
            assert math.isnan(st[T.STATISTICS_DATA_TYPE_MINIMUM])
            assert math.isnan(st[T.STATISTICS_DATA_TYPE_STDDEV])
            assert st[T.STATISTICS_DATA_TYPE_SAMPLE_COUNT] == 100.0
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

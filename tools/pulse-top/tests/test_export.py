"""pulse-export tests: probe jsonl windows -> Prometheus text and OTLP/HTTP JSON.

The exporter must keep the probe's contract (README "JSON Lines output"): an absent
key is "not measured", so it becomes an ABSENT series, never a 0 sample. A topic the
probe stopped reporting, or a process that stopped flushing, must drop out of the
scrape instead of repeating its last rate as if current.
"""

from __future__ import annotations

import io
import json
import os
import threading
import time
import urllib.request
from contextlib import redirect_stdout
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from pulse_top import export
from pulse_top.export import (
    Exporter,
    otlp_payload,
    otlp_url,
    pid_label,
    push_otlp,
    render_prometheus,
    start_server,
)

PUB_LOG = "/tmp/topic_freq.4242.log"
SUB_LOG = "/tmp/topic_freq.5151.log"
TS0 = 1782887153000000000


def rec(ts_ns, topics, nodes=(), warns=(), window_s=1.0):
    return json.dumps({"ts_ns": str(ts_ns), "window_s": window_s, "topics": list(topics),
                       "nodes": list(nodes), "warns": list(warns)})


PUB_LINE = rec(TS0, [{"topic": "/scan", "pub_inter_hz": 20.0, "pub_intra_hz": 0.0,
                      "pub_max_dt_ms": 812.4}], nodes=["/lidar"])
SUB_LINE = rec(TS0 + 400_000_000,
               [{"topic": "/scan", "recv_inter_hz": 19.5, "recv_intra_hz": 0.0,
                 "recv_endpoint_seen": True}],
               nodes=["/perception"],
               warns=[{"kind": "node_missing", "node": "/localization"},
                      {"kind": "topic_rate", "topic": "/scan", "hz": 19.5, "min_hz": 19.8}])


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def two_process_exporter():
    clock = Clock()
    ex = Exporter(poll_s=1.0, clock=clock)
    ex.ingest(PUB_LOG, PUB_LINE)
    clock.t += 0.5
    ex.ingest(SUB_LOG, SUB_LINE)
    ex.ingest(SUB_LOG, "[INFO] not a probe line")  # foreign line: skipped
    clock.t += 0.25
    return ex, clock


EXPECTED = """\
# HELP ros2_pulse_topic_publish_rate_hertz Publish rate in the process's latest window, per path (inter or intra process).
# TYPE ros2_pulse_topic_publish_rate_hertz gauge
ros2_pulse_topic_publish_rate_hertz{pid="4242",topic="/scan",path="inter"} 20
ros2_pulse_topic_publish_rate_hertz{pid="4242",topic="/scan",path="intra"} 0
# HELP ros2_pulse_topic_receive_rate_hertz Subscription callback rate in the process's latest window, per path (inter or intra process).
# TYPE ros2_pulse_topic_receive_rate_hertz gauge
ros2_pulse_topic_receive_rate_hertz{pid="5151",topic="/scan",path="inter"} 19.5
ros2_pulse_topic_receive_rate_hertz{pid="5151",topic="/scan",path="intra"} 0
# HELP ros2_pulse_topic_max_gap_seconds Largest inter-arrival gap in the latest window (only with ROS_TOPIC_STATS_JITTER).
# TYPE ros2_pulse_topic_max_gap_seconds gauge
ros2_pulse_topic_max_gap_seconds{pid="4242",topic="/scan",side="pub"} 0.8124
# HELP ros2_pulse_topic_loaned_rate_hertz Middleware-loaned share of the publish or receive rate in the latest window (a subset of the total; absent when no loan happened).
# TYPE ros2_pulse_topic_loaned_rate_hertz gauge
# HELP ros2_pulse_topic_recv_lag_deficit_ratio Active recv_lag: (pub - recv) / pub while callbacks sit under the publish rate.
# TYPE ros2_pulse_topic_recv_lag_deficit_ratio gauge
# HELP ros2_pulse_warn_active 1 for each kind and topic or node warned in the process's latest window (topic_rate, topic_gap, node_missing, recv_lag).
# TYPE ros2_pulse_warn_active gauge
ros2_pulse_warn_active{pid="5151",kind="node_missing",node="/localization"} 1
ros2_pulse_warn_active{pid="5151",kind="topic_rate",topic="/scan"} 1
# HELP ros2_pulse_node_up 1 if the node is in the process's latest window, 0 if the probe reports it missing.
# TYPE ros2_pulse_node_up gauge
ros2_pulse_node_up{pid="4242",node="/lidar"} 1
ros2_pulse_node_up{pid="5151",node="/localization"} 0
ros2_pulse_node_up{pid="5151",node="/perception"} 1
# HELP ros2_pulse_node_last_seen_age_seconds Seconds since the exporter last read a window listing the node.
# TYPE ros2_pulse_node_last_seen_age_seconds gauge
ros2_pulse_node_last_seen_age_seconds{node="/lidar"} 0.75
ros2_pulse_node_last_seen_age_seconds{node="/perception"} 0.25
# HELP ros2_pulse_process_window_seconds Window period of the process's latest window (ROS_TOPIC_STATISTICS_PUBLISH_PERIOD).
# TYPE ros2_pulse_process_window_seconds gauge
ros2_pulse_process_window_seconds{pid="4242"} 1
ros2_pulse_process_window_seconds{pid="5151"} 1
# HELP ros2_pulse_process_last_window_age_seconds Seconds since the exporter last read a window from the process's log.
# TYPE ros2_pulse_process_last_window_age_seconds gauge
ros2_pulse_process_last_window_age_seconds{pid="4242"} 0.75
ros2_pulse_process_last_window_age_seconds{pid="5151"} 0.25
# HELP ros2_pulse_process_last_window_timestamp_seconds Probe timestamp of the process's latest window, Unix seconds.
# TYPE ros2_pulse_process_last_window_timestamp_seconds gauge
ros2_pulse_process_last_window_timestamp_seconds{pid="4242"} 1782887153
ros2_pulse_process_last_window_timestamp_seconds{pid="5151"} 1782887153.4
# HELP ros2_pulse_windows_total Probe windows read from the process's log.
# TYPE ros2_pulse_windows_total counter
ros2_pulse_windows_total{pid="4242"} 1
ros2_pulse_windows_total{pid="5151"} 1
# HELP ros2_pulse_warns_total Probe warns read, per kind.
# TYPE ros2_pulse_warns_total counter
ros2_pulse_warns_total{pid="5151",kind="node_missing"} 1
ros2_pulse_warns_total{pid="5151",kind="topic_rate"} 1
# HELP ros2_pulse_lines_skipped_total Log lines that were not probe output in either format (truncated, foreign).
# TYPE ros2_pulse_lines_skipped_total counter
ros2_pulse_lines_skipped_total 1
"""


class TestPrometheus:
    def test_fixture_renders_exact_exposition(self):
        ex, _ = two_process_exporter()
        assert render_prometheus(ex.collect()) == EXPECTED

    def test_absent_key_is_absent_series_not_zero(self):
        ex, _ = two_process_exporter()
        text = render_prometheus(ex.collect())
        # the publisher's process has no receive side: no receive series for pid 4242
        assert 'receive_rate_hertz{pid="4242"' not in text
        # recv gap not measured: no recv max_gap series at all
        assert 'side="recv"' not in text

    def test_stale_process_drops_topic_series_keeps_age(self):
        ex, clock = two_process_exporter()
        clock.t += 5.0  # well past 2 periods + poll for both processes
        text = render_prometheus(ex.collect())
        assert 'ros2_pulse_topic_publish_rate_hertz{' not in text
        assert 'ros2_pulse_node_up{' not in text
        assert 'ros2_pulse_warn_active{' not in text
        assert 'ros2_pulse_process_last_window_age_seconds{pid="4242"} 5.75' in text
        assert 'ros2_pulse_windows_total{pid="4242"} 1' in text

    def test_forgotten_after_long_silence(self):
        ex, clock = two_process_exporter()
        clock.t += export.FORGET_S + 1
        text = render_prometheus(ex.collect())
        assert 'pid="4242"' not in text
        assert 'node="/lidar"' not in text

    def test_topic_absent_from_latest_window_disappears(self):
        ex, clock = two_process_exporter()
        ex.ingest(PUB_LOG, rec(TS0 + 1_000_000_000, [], nodes=["/lidar"]))
        text = render_prometheus(ex.collect())
        assert 'publish_rate_hertz{pid="4242"' not in text
        assert 'ros2_pulse_windows_total{pid="4242"} 2' in text

    def test_recv_lag_exported_per_subscriber_process(self):
        clock = Clock()
        ex = Exporter(poll_s=1.0, clock=clock)
        for i in range(3):
            ts = TS0 + i * 1_000_000_000
            ex.ingest(PUB_LOG, rec(ts, [{"topic": "/scan", "pub_inter_hz": 20.0,
                                         "pub_intra_hz": 0.0}]))
            ex.ingest(SUB_LOG, rec(ts + 100_000_000,
                                   [{"topic": "/scan", "recv_inter_hz": 12.0,
                                     "recv_intra_hz": 0.0, "recv_endpoint_seen": True}]))
            clock.t += 1.0
        text = render_prometheus(ex.collect())
        assert 'ros2_pulse_topic_recv_lag_deficit_ratio{pid="5151",topic="/scan"} 0.4' in text
        assert 'ros2_pulse_warn_active{pid="5151",kind="recv_lag",topic="/scan"} 1' in text

    # A spec with a pub and a recv rule on one topic: both sides of one process under min_hz,
    # and both gaps over max_gap_ms. The probe's warns carry no side, so each pair reads alike.
    BOTH_SIDES_JSONL = rec(TS0, [{"topic": "/scan", "pub_inter_hz": 5.0, "pub_intra_hz": 0.0,
                                  "recv_inter_hz": 3.0, "recv_intra_hz": 0.0,
                                  "recv_endpoint_seen": True}],
                           warns=[{"kind": "topic_rate", "topic": "/scan", "hz": 5.0, "min_hz": 10},
                                  {"kind": "topic_rate", "topic": "/scan", "hz": 3.0, "min_hz": 10},
                                  {"kind": "topic_gap", "topic": "/scan", "max_dt_ms": 900.0,
                                   "max_gap_ms": 200},
                                  {"kind": "topic_gap", "topic": "/scan", "max_dt_ms": 950.0,
                                   "max_gap_ms": 200}])
    BOTH_SIDES_TEXT = ("# ts_ns=1782887153000000000 window_s=1.000\n"
                       "PUB /scan inter=5.000000 intra=0.000000\n"
                       "RECV /scan inter=3.000000 intra=0.000000\n"
                       "WARN TOPIC /scan hz=5.000000 expected=[10,inf]\n"
                       "WARN TOPIC /scan hz=3.000000 expected=[10,inf]\n"
                       "WARN TOPIC /scan max_dt_ms=900.000 expected_max_gap_ms=200\n"
                       "WARN TOPIC /scan max_dt_ms=950.000 expected_max_gap_ms=200\n"
                       "\n")

    @pytest.mark.parametrize("fmt", ["jsonl", "text"])
    def test_pub_and_recv_warn_of_one_kind_is_one_series(self, fmt):
        ex = Exporter(poll_s=1.0, clock=Clock())
        log = self.BOTH_SIDES_JSONL if fmt == "jsonl" else self.BOTH_SIDES_TEXT
        for line in log.split("\n"):
            ex.ingest(SUB_LOG, line)
        metrics = ex.collect()
        text = render_prometheus(metrics)
        series = [line.rpartition(" ")[0] for line in text.splitlines()
                  if line and not line.startswith("#")]
        assert len(series) == len(set(series)), [s for s in series if series.count(s) > 1]
        assert text.count('ros2_pulse_warn_active{pid="5151",kind="topic_rate",topic="/scan"} 1') == 1
        assert text.count('ros2_pulse_warn_active{pid="5151",kind="topic_gap",topic="/scan"} 1') == 1
        # the counter still counts every warn read
        assert 'ros2_pulse_warns_total{pid="5151",kind="topic_rate"} 2' in text
        # OTLP reads the same samples, so it has one point per attribute set too
        scope = otlp_payload(metrics, 1, 0)["resourceMetrics"][0]["scopeMetrics"][0]
        for entry in scope["metrics"]:
            points = (entry.get("gauge") or entry["sum"])["dataPoints"]
            attrs = [json.dumps(p["attributes"]) for p in points]
            assert len(attrs) == len(set(attrs)), entry["name"]

    def test_label_escaping(self):
        clock = Clock()
        ex = Exporter(poll_s=1.0, clock=clock)
        ex.ingest(PUB_LOG, rec(TS0, [{"topic": '/we"ird\\name\n', "pub_inter_hz": 1.0}]))
        text = render_prometheus(ex.collect())
        assert 'topic="/we\\"ird\\\\name\\n"' in text

    @pytest.mark.parametrize("path,pid", [
        ("/tmp/topic_freq.4242.log", "4242"),
        ("/var/log/pulse/topic_freq.17.log", "17"),
        ("/tmp/shared.log", "shared.log"),  # no pid in the name: the file is the identity
    ])
    def test_pid_label(self, path, pid):
        assert pid_label(path) == pid


def partial(line):
    r = json.loads(line)
    r["partial"] = True
    return json.dumps(r)


class TestProcessLifecycle:
    # ROS2PulseProcessStale reads ros2_pulse_process_last_window_age_seconds as "this process
    # crashed or hung", so a clean exit and an earlier run's log must not look like one.

    def test_attach_window_starts_the_process(self):
        ex = Exporter(poll_s=1.0, clock=Clock())
        ex.ingest(PUB_LOG, partial(PUB_LINE))
        assert 'ros2_pulse_windows_total{pid="4242"} 1' in render_prometheus(ex.collect())

    def test_exit_window_forgets_the_process(self):
        ex, clock = two_process_exporter()
        ex.ingest(PUB_LOG, partial(rec(TS0 + 1_000_000_000, [], nodes=["/lidar"],
                                       window_s=0.2)))
        text = render_prometheus(ex.collect())
        assert 'pid="4242"' not in text
        assert 'ros2_pulse_process_last_window_age_seconds{pid="5151"}' in text
        ex.ingest(PUB_LOG, partial(PUB_LINE))  # pid reused: a new process attaches
        assert 'ros2_pulse_windows_total{pid="4242"} 1' in render_prometheus(ex.collect())

    def test_slow_window_outlives_three_periods(self):
        # ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=120: the alert needs the age past 360 s, so
        # the series has to outlive a fixed 300 s.
        clock = Clock()
        ex = Exporter(poll_s=1.0, clock=clock)
        ex.ingest(PUB_LOG, rec(TS0, [], nodes=["/lidar"], window_s=120.0))
        clock.t += 3 * 120 + 30
        assert 'ros2_pulse_process_last_window_age_seconds{pid="4242"} 390' in \
            render_prometheus(ex.collect())
        clock.t += 120
        assert 'pid="4242"' not in render_prometheus(ex.collect())

    def test_log_idle_past_the_forget_horizon_is_skipped(self, tmp_path):
        old, fresh = tmp_path / "topic_freq.1.log", tmp_path / "topic_freq.2.log"
        for f in (old, fresh):
            f.write_text(PUB_LINE + "\n")
        hour_ago = time.time() - 3600
        os.utime(old, (hour_ago, hour_ago))
        ex = Exporter(poll_s=1.0, clock=Clock(), skip_idle=True)
        for f in (old, fresh):
            ex.ingest(str(f), PUB_LINE)
        text = render_prometheus(ex.collect())
        assert 'pid="1"' not in text
        assert 'ros2_pulse_windows_total{pid="2"} 1' in text
        with old.open("a") as f:  # the pid is reused and writes again
            f.write(PUB_LINE + "\n")
        ex.ingest(str(old), PUB_LINE)
        assert 'ros2_pulse_windows_total{pid="1"} 1' in render_prometheus(ex.collect())


class TestOtlp:
    def test_payload_shape(self):
        ex, _ = two_process_exporter()
        p = otlp_payload(ex.collect(), time_ns=TS0 + 10, start_ns=TS0)
        json.dumps(p)  # must serialize as is
        rm = p["resourceMetrics"][0]
        attrs = {a["key"]: a["value"] for a in rm["resource"]["attributes"]}
        assert attrs["service.name"] == {"stringValue": "ros2_pulse"}
        sm = rm["scopeMetrics"][0]
        assert sm["scope"]["name"] == "pulse_export"
        by_name = {m["name"]: m for m in sm["metrics"]}

        pub = by_name["ros2_pulse_topic_publish_rate_hertz"]
        assert pub["unit"] == "Hz"
        dp = pub["gauge"]["dataPoints"][0]
        assert dp["asDouble"] == 20.0
        assert dp["timeUnixNano"] == str(TS0 + 10)  # int64 as a decimal string, like ts_ns
        assert {"key": "topic", "value": {"stringValue": "/scan"}} in dp["attributes"]
        assert {"key": "pid", "value": {"stringValue": "4242"}} in dp["attributes"]

        # counters are cumulative monotonic sums, named without the Prometheus _total
        win = by_name["ros2_pulse_windows"]
        assert "ros2_pulse_windows_total" not in by_name
        assert win["sum"]["isMonotonic"] is True
        assert win["sum"]["aggregationTemporality"] == 2
        assert win["sum"]["dataPoints"][0]["startTimeUnixNano"] == str(TS0)

    def test_metric_with_no_samples_is_omitted(self):
        ex, _ = two_process_exporter()
        names = [m["name"] for m in otlp_payload(ex.collect(), 1, 0)
                 ["resourceMetrics"][0]["scopeMetrics"][0]["metrics"]]
        assert "ros2_pulse_topic_recv_lag_deficit_ratio" not in names

    @pytest.mark.parametrize("given,want", [
        ("http://collector:4318", "http://collector:4318/v1/metrics"),
        ("http://collector:4318/", "http://collector:4318/v1/metrics"),
        ("https://otlp.example/otlp/v1/metrics", "https://otlp.example/otlp/v1/metrics"),
    ])
    def test_endpoint(self, given, want):
        assert otlp_url(given) == want

    def test_push_posts_json_with_headers(self):
        got = {}

        class H(BaseHTTPRequestHandler):
            def do_POST(self):
                n = int(self.headers["Content-Length"])
                got["body"] = json.loads(self.rfile.read(n))
                got["ctype"] = self.headers["Content-Type"]
                got["auth"] = self.headers["Authorization"]
                got["path"] = self.path
                self.send_response(200)
                self.end_headers()

            def log_message(self, *a):
                pass

        srv = HTTPServer(("127.0.0.1", 0), H)
        t = threading.Thread(target=srv.handle_request, daemon=True)
        t.start()
        url = f"http://127.0.0.1:{srv.server_address[1]}/v1/metrics"
        status = push_otlp(url, {"resourceMetrics": []}, {"Authorization": "Bearer x"})
        t.join(5)
        srv.server_close()
        assert status == 200
        assert got == {"body": {"resourceMetrics": []}, "ctype": "application/json",
                       "auth": "Bearer x", "path": "/v1/metrics"}


class TestServer:
    def test_metrics_endpoint(self):
        ex, _ = two_process_exporter()
        lock = threading.Lock()
        srv = start_server(ex, lock, "127.0.0.1", 0)
        try:
            base = f"http://127.0.0.1:{srv.server_address[1]}"
            with urllib.request.urlopen(base + "/metrics", timeout=5) as r:
                assert r.status == 200
                assert r.headers["Content-Type"].startswith("text/plain; version=0.0.4")
                body = r.read().decode()
            assert body.startswith("# HELP ros2_pulse_topic_publish_rate_hertz")
            assert 'ros2_pulse_windows_total{pid="5151"} 1' in body
            with pytest.raises(urllib.error.HTTPError) as e:
                urllib.request.urlopen(base + "/nope", timeout=5)
            assert e.value.code == 404
        finally:
            srv.shutdown()
            srv.server_close()


class TestCli:
    def test_once_prints_metrics_for_a_log(self, tmp_path):
        log = tmp_path / "topic_freq.99.log"
        log.write_text(PUB_LINE + "\n")
        out = io.StringIO()
        with redirect_stdout(out):
            assert export.main(["--once", str(log)]) == 0
        assert 'ros2_pulse_topic_publish_rate_hertz{pid="99",topic="/scan",path="inter"} 20' \
            in out.getvalue()

    def test_no_log_found_exits_2(self, tmp_path, monkeypatch):
        monkeypatch.setattr(export, "default_log_path", lambda: None)
        assert export.main(["--once"]) == 2

    # The Prometheus default-port list gives 9464 to the OpenTelemetry Node.js exporter.
    # Scrape configs depend on the default, so changing it again is a minor release.
    @pytest.mark.parametrize("argv, port", [([], 10060), (["--port", "9464"], 9464)])
    def test_server_port(self, tmp_path, monkeypatch, argv, port):
        log = tmp_path / "topic_freq.99.log"
        log.write_text(PUB_LINE + "\n")
        bound = []
        monkeypatch.setattr(export, "start_server",
                            lambda ex, lock, host, p: bound.append((host, p)))

        def stop(_s):
            raise KeyboardInterrupt

        monkeypatch.setattr(export.time, "sleep", stop)
        assert export.main([str(log), *argv]) == 0
        assert bound == [("0.0.0.0", port)]

    def test_once_prints_a_log_however_old(self, tmp_path):
        log = tmp_path / "topic_freq.99.log"
        log.write_text(PUB_LINE + "\n")
        os.utime(log, (0, 0))
        out = io.StringIO()
        with redirect_stdout(out):
            assert export.main(["--once", str(log)]) == 0
        assert 'publish_rate_hertz{pid="99",topic="/scan",path="inter"} 20' in out.getvalue()

    def test_server_skips_logs_left_by_an_earlier_run(self, tmp_path, monkeypatch):
        # The probe never deletes its log; the follower replays each one on attach.
        (tmp_path / "topic_freq.1.log").write_text(PUB_LINE + "\n")
        (tmp_path / "topic_freq.2.log").write_text(PUB_LINE + "\n")
        os.utime(tmp_path / "topic_freq.1.log", (0, 0))
        exporters = []
        monkeypatch.setattr(export, "start_server",
                            lambda ex, lock, host, p: exporters.append(ex))

        def stop(_s):
            raise KeyboardInterrupt

        monkeypatch.setattr(export.time, "sleep", stop)
        assert export.main([str(tmp_path / "topic_freq.*.log")]) == 0
        text = render_prometheus(exporters[0].collect())
        assert 'pid="1"' not in text
        assert 'ros2_pulse_windows_total{pid="2"} 1' in text


class TestTextFormat:
    # Issue #58: the probe's default format is text. pulse-export used to print a
    # hint and export nothing; it now reads text windows like jsonl ones.
    TEXT = ("# ts_ns=1782887153899445923 window_s=1.000\n"
            "TOPIC /scan 20.000000\n"
            "NODE /lidar\n"
            "\n")

    def test_text_log_is_exported(self, tmp_path):
        log = tmp_path / "topic_freq.77.log"
        log.write_text(self.TEXT)
        out = io.StringIO()
        with redirect_stdout(out):
            assert export.main(["--once", str(log)]) == 0
        text = out.getvalue()
        assert 'ros2_pulse_topic_publish_rate_hertz{pid="77",topic="/scan",path="inter"} 20' in text
        assert 'ros2_pulse_node_up{pid="77",node="/lidar"} 1' in text
        assert "ros2_pulse_lines_skipped_total 0" in text

    def test_text_and_jsonl_files_side_by_side(self):
        ex = Exporter(clock=Clock())
        for line in self.TEXT.split("\n"):
            ex.ingest(SUB_LOG, line)
        ex.ingest(PUB_LOG, PUB_LINE)
        text = render_prometheus(ex.collect())
        assert 'ros2_pulse_windows_total{pid="5151"} 1' in text
        assert 'ros2_pulse_windows_total{pid="4242"} 1' in text

    def test_foreign_lines_are_still_counted(self):
        ex = Exporter(clock=Clock())
        ex.ingest(SUB_LOG, "not a probe line")
        ex.ingest(SUB_LOG, "")  # a blank line is framing, not a skipped record
        assert "ros2_pulse_lines_skipped_total 1" in render_prometheus(ex.collect())

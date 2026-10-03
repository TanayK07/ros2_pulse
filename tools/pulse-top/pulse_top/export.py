"""pulse-export: the probe's windows as Prometheus metrics and OTLP/HTTP JSON.

A pure log consumer, like pulse-top: it tails the files the probe already writes
(default text format or ROS_TOPIC_STATS_FORMAT=jsonl) with the same follower and
parser, and serves
/metrics in the Prometheus text exposition format from a stdlib HTTP server.
Optionally it pushes the same series to an OTLP/HTTP endpoint as JSON. Stdlib only:
no prometheus_client, no opentelemetry-sdk, nothing new to install on a robot.

What a scrape shows is each process's LATEST window. The probe omits a silent topic
from a window and an absent key means "not measured", so both become an absent
series here, never a 0 sample. A process that stops flushing keeps only its
process-level series (age, counters) until it is forgotten, so a dead publisher's
last rate is never scraped as if it were current.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import re
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import Counter
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .model import LAG_TOL_DEFAULT, LAG_WINDOWS_DEFAULT, LogParser, StatsState, Window
from .reader import MultiFollower, default_log_path

DEFAULT_PORT = 9464
DEFAULT_OTLP_INTERVAL_S = 15.0
# A process whose last window is older than this (exporter clock) is forgotten
# entirely, so a stack restarted every few minutes does not grow the scrape forever.
FORGET_S = 300.0
PROM_CONTENT_TYPE = "text/plain; version=0.0.4; charset=utf-8"

_PID = re.compile(r"topic_freq\.(\d+)\.log")


def pid_label(path: str) -> str:
    """The probe writes $TMPDIR/topic_freq.<pid>.log by default; jsonl records carry
    no pid, so the file name is the only provenance. Any other name (a shared
    ROS_TOPIC_STATS_OUTPUT_FILE) is labelled by its basename, which is unique per file."""
    base = os.path.basename(path)
    m = _PID.fullmatch(base)
    return m.group(1) if m else base


@dataclass
class Metric:
    name: str
    kind: str  # "gauge" | "counter"
    help: str
    unit: str  # UCUM, for OTLP
    labels: tuple[str, ...] = ()
    samples: list[tuple[tuple[str | None, ...], float]] = field(default_factory=list)

    def add(self, value: float, *label_values: str | None) -> None:
        self.samples.append((tuple(label_values), float(value)))


def _metrics() -> dict[str, Metric]:
    """The metric catalogue, in exposition order. One definition feeds both outputs."""
    ms = [
        Metric("ros2_pulse_topic_publish_rate_hertz", "gauge",
               "Publish rate in the process's latest window, per path (inter or intra process).",
               "Hz", ("pid", "topic", "path")),
        Metric("ros2_pulse_topic_receive_rate_hertz", "gauge",
               "Subscription callback rate in the process's latest window, per path "
               "(inter or intra process).", "Hz", ("pid", "topic", "path")),
        Metric("ros2_pulse_topic_max_gap_seconds", "gauge",
               "Largest inter-arrival gap in the latest window (only with ROS_TOPIC_STATS_JITTER).",
               "s", ("pid", "topic", "side")),
        Metric("ros2_pulse_topic_loaned_rate_hertz", "gauge",
               "Middleware-loaned share of the publish or receive rate in the latest window "
               "(a subset of the total; absent when no loan happened).",
               "Hz", ("pid", "topic", "side")),
        Metric("ros2_pulse_topic_recv_lag_deficit_ratio", "gauge",
               "Active recv_lag: (pub - recv) / pub while callbacks sit under the publish rate.",
               "1", ("pid", "topic")),
        Metric("ros2_pulse_warn_active", "gauge",
               "1 for each warn in the process's latest window "
               "(topic_rate, topic_gap, node_missing, recv_lag).",
               "1", ("pid", "kind", "topic", "node")),
        Metric("ros2_pulse_node_up", "gauge",
               "1 if the node is in the process's latest window, 0 if the probe reports it missing.",
               "1", ("pid", "node")),
        Metric("ros2_pulse_node_last_seen_age_seconds", "gauge",
               "Seconds since the exporter last read a window listing the node.", "s", ("node",)),
        Metric("ros2_pulse_process_window_seconds", "gauge",
               "Window period of the process's latest window (ROS_TOPIC_STATISTICS_PUBLISH_PERIOD).",
               "s", ("pid",)),
        Metric("ros2_pulse_process_last_window_age_seconds", "gauge",
               "Seconds since the exporter last read a window from the process's log.",
               "s", ("pid",)),
        Metric("ros2_pulse_process_last_window_timestamp_seconds", "gauge",
               "Probe timestamp of the process's latest window, Unix seconds.", "s", ("pid",)),
        Metric("ros2_pulse_windows_total", "counter",
               "Probe windows read from the process's log.", "1", ("pid",)),
        Metric("ros2_pulse_warns_total", "counter",
               "Probe warns read, per kind.", "1", ("pid", "kind")),
        Metric("ros2_pulse_lines_skipped_total", "counter",
               "Log lines that were not probe output in either format (truncated, foreign).",
               "1"),
    ]
    return {m.name: m for m in ms}


@dataclass
class _Proc:
    pid: str
    window: Window
    read_at: float
    windows_total: int = 0
    warns_total: Counter = field(default_factory=Counter)


class Exporter:
    """Folds probe windows per source log and renders them as a metric set.

    `clock` is monotonic seconds (injectable for tests). Ages are measured on the
    exporter's clock, when a window was READ, not from the probe's ts_ns: the two
    may sit on different machines, and a replayed log must not read as hours stale.
    """

    def __init__(self, poll_s: float = 1.0, lag_tol: float = LAG_TOL_DEFAULT,
                 lag_windows: int = LAG_WINDOWS_DEFAULT, clock=time.monotonic):
        self.poll_s = poll_s
        self._clock = clock
        self._procs: dict[str, _Proc] = {}
        self._node_seen: dict[str, float] = {}
        self._skipped = 0
        self._parsers: dict[str, LogParser] = {}
        # pulse-top's model, reused for the one derived signal (recv_lag), which
        # needs the publisher's and the subscriber's windows side by side.
        self._state = StatsState(history=1, lag_tol=lag_tol, lag_windows=lag_windows)

    def ingest(self, path: str, line: str) -> None:
        """One log line from `path`. Text and jsonl are both read (issue #58); a text
        window completes on its closing blank line, so the parser is per file."""
        parser = self._parsers.get(path)
        if parser is None:
            parser = self._parsers[path] = LogParser()
        before = parser.unrecognised
        windows = parser.feed(line)
        self._skipped += parser.unrecognised - before
        for window in windows:
            self._ingest_window(path, window)

    def _ingest_window(self, path: str, window: Window) -> None:
        now = self._clock()
        proc = self._procs.get(path)
        if proc is None:
            proc = self._procs[path] = _Proc(pid_label(path), window, now)
        proc.window, proc.read_at = window, now
        proc.windows_total += 1
        proc.warns_total.update(w.kind for w in window.warns)
        for n in window.nodes:
            self._node_seen[n] = now
        # A copy: the model merges later windows' sides INTO the TopicWindow it first
        # retained, which would leak one process's receive side into another's series.
        self._state.apply(copy.deepcopy(window), source=path)

    def _live(self, proc: _Proc, now: float) -> bool:
        """Latest window still current: two of its periods plus one poll of slack,
        so a scrape that lands just before the next flush does not blink the series."""
        return now - proc.read_at <= 2.0 * proc.window.window_s + self.poll_s

    def collect(self) -> list[Metric]:
        now = self._clock()
        for path in [p for p, pr in self._procs.items() if now - pr.read_at > FORGET_S]:
            del self._procs[path]
            self._parsers.pop(path, None)
        for n in [n for n, t in self._node_seen.items() if now - t > FORGET_S]:
            del self._node_seen[n]

        m = _metrics()
        pub, recv = m["ros2_pulse_topic_publish_rate_hertz"], m["ros2_pulse_topic_receive_rate_hertz"]
        gap, warn = m["ros2_pulse_topic_max_gap_seconds"], m["ros2_pulse_warn_active"]
        loaned = m["ros2_pulse_topic_loaned_rate_hertz"]
        node_up = m["ros2_pulse_node_up"]
        live: dict[str, _Proc] = {}
        for path, pr in self._procs.items():
            pid, w = pr.pid, pr.window
            m["ros2_pulse_process_window_seconds"].add(w.window_s, pid)
            m["ros2_pulse_process_last_window_age_seconds"].add(now - pr.read_at, pid)
            m["ros2_pulse_process_last_window_timestamp_seconds"].add(w.ts_ns / 1e9, pid)
            m["ros2_pulse_windows_total"].add(pr.windows_total, pid)
            for kind, n in pr.warns_total.items():
                m["ros2_pulse_warns_total"].add(n, pid, kind)
            if not self._live(pr, now):
                continue
            live[path] = pr
            for t in w.topics:
                for metric, attr, lab in ((pub, "pub_inter_hz", "inter"), (pub, "pub_intra_hz", "intra"),
                                          (recv, "recv_inter_hz", "inter"),
                                          (recv, "recv_intra_hz", "intra")):
                    v = getattr(t, attr)
                    if v is not None:
                        metric.add(v, pid, t.topic, lab)
                for attr, side in (("pub_max_dt_ms", "pub"), ("recv_max_dt_ms", "recv")):
                    v = getattr(t, attr)
                    if v is not None:
                        gap.add(v / 1000.0, pid, t.topic, side)
                for attr, side in (("pub_loaned_hz", "pub"), ("recv_loaned_hz", "recv")):
                    v = getattr(t, attr)
                    if v is not None:
                        loaned.add(v, pid, t.topic, side)
            for x in w.warns:
                warn.add(1, pid, x.kind, x.topic, x.node)
            for n in w.nodes:
                node_up.add(1, pid, n)
            for x in w.warns:
                if x.kind == "node_missing" and x.node and x.node not in w.nodes:
                    node_up.add(0, pid, x.node)
        for x in self._state.warns:
            if x.kind == "recv_lag" and x.source in live and x.deficit is not None:
                pid = live[x.source].pid
                m["ros2_pulse_topic_recv_lag_deficit_ratio"].add(x.deficit, pid, x.topic)
                warn.add(1, pid, x.kind, x.topic, None)
        for n, t in self._node_seen.items():
            m["ros2_pulse_node_last_seen_age_seconds"].add(now - t, n)
        m["ros2_pulse_lines_skipped_total"].add(self._skipped)
        return list(m.values())


# -- Prometheus text exposition (format 0.0.4) --------------------------------------------

def _fmt(v: float) -> str:
    if math.isnan(v):
        return "NaN"
    if math.isinf(v):
        return "+Inf" if v > 0 else "-Inf"
    if v.is_integer() and abs(v) < 1e15:
        return str(int(v))
    return repr(v)


def _esc_label(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def _esc_help(s: str) -> str:
    return s.replace("\\", "\\\\").replace("\n", "\\n")


def _sorted(metric: Metric):
    return sorted(metric.samples, key=lambda s: tuple("" if v is None else v for v in s[0]))


def render_prometheus(metrics: list[Metric]) -> str:
    out: list[str] = []
    for m in metrics:
        out.append(f"# HELP {m.name} {_esc_help(m.help)}")
        out.append(f"# TYPE {m.name} {m.kind}")
        for values, v in _sorted(m):
            pairs = [f'{k}="{_esc_label(val)}"' for k, val in zip(m.labels, values) if val is not None]
            lbl = "{" + ",".join(pairs) + "}" if pairs else ""
            out.append(f"{m.name}{lbl} {_fmt(v)}")
    return "\n".join(out) + "\n"


# -- OTLP/HTTP JSON (opentelemetry-proto metrics v1, JSON encoding) ------------------------

_CUMULATIVE = 2  # AGGREGATION_TEMPORALITY_CUMULATIVE


def _version() -> str:
    try:
        from importlib.metadata import version

        return version("ros2-pulse-top")
    except Exception:  # running from a source tree without an install
        return "0"


def _resource_attrs() -> list[dict]:
    return [{"key": "service.name", "value": {"stringValue": "ros2_pulse"}},
            {"key": "host.name", "value": {"stringValue": socket.gethostname()}}]


def otlp_payload(metrics: list[Metric], time_ns: int, start_ns: int) -> dict:
    """ExportMetricsServiceRequest as JSON. Names match the Prometheus ones so one
    dashboard reads either path; counters drop `_total`, which an OTLP-to-Prometheus
    translation adds back. (u)int64 fields are decimal strings, as OTLP/JSON requires."""
    out = []
    for m in metrics:
        if not m.samples:
            continue
        points = []
        for values, v in _sorted(m):
            dp = {"attributes": [{"key": k, "value": {"stringValue": val}}
                                 for k, val in zip(m.labels, values) if val is not None],
                  "timeUnixNano": str(time_ns), "asDouble": v}
            if m.kind == "counter":
                dp["startTimeUnixNano"] = str(start_ns)
            points.append(dp)
        entry = {"name": m.name, "description": m.help, "unit": m.unit}
        if m.kind == "counter":
            entry["name"] = m.name.removesuffix("_total")
            entry["sum"] = {"dataPoints": points, "aggregationTemporality": _CUMULATIVE,
                            "isMonotonic": True}
        else:
            entry["gauge"] = {"dataPoints": points}
        out.append(entry)
    return {"resourceMetrics": [{
        "resource": {"attributes": _resource_attrs()},
        "scopeMetrics": [{"scope": {"name": "pulse_export", "version": _version()},
                          "metrics": out}],
    }]}


def otlp_url(endpoint: str) -> str:
    """A bare collector address gets the standard metrics path (OTLP/HTTP spec);
    an explicit path is kept as given."""
    scheme, sep, rest = endpoint.partition("://")
    if "/" not in rest.rstrip("/"):
        return f"{scheme}{sep}{rest.rstrip('/')}/v1/metrics"
    return endpoint


def push_otlp(url: str, payload: dict, headers: dict[str, str] | None = None,
              timeout: float = 5.0) -> int:
    req = urllib.request.Request(
        url, data=json.dumps(payload).encode(), method="POST",
        headers={"Content-Type": "application/json", **(headers or {})})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status


# -- HTTP server ---------------------------------------------------------------------------

def start_server(exporter: Exporter, lock: threading.Lock, host: str, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.split("?", 1)[0] == "/metrics":
                with lock:
                    body = render_prometheus(exporter.collect()).encode()
                ctype = PROM_CONTENT_TYPE
                self.send_response(200)
            elif self.path == "/":
                body = b"pulse-export: metrics at /metrics\n"
                ctype = "text/plain; charset=utf-8"
                self.send_response(200)
            else:
                body = b"not found\n"
                ctype = "text/plain; charset=utf-8"
                self.send_response(404)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):  # a scrape every 15 s is not news
            pass

    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, name="pulse-export-http", daemon=True).start()
    return srv


# -- CLI -------------------------------------------------------------------------------------

def _header(s: str) -> tuple[str, str]:
    k, sep, v = s.partition("=")
    if not sep or not k.strip():
        raise argparse.ArgumentTypeError(f"expected KEY=VALUE, got {s!r}")
    return k.strip(), v.strip()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="pulse-export",
        description="Serve ros2_pulse probe logs (text or jsonl) as Prometheus /metrics, "
                    "optionally push OTLP/HTTP JSON.",
    )
    ap.add_argument("file", nargs="?",
                    help="probe log, or a quoted glob like '/tmp/topic_freq.*.log' "
                         "(default: every $TMPDIR/topic_freq.<pid>.log)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help=f"Prometheus port (default {DEFAULT_PORT}; 0 disables the server)")
    ap.add_argument("--bind", default="0.0.0.0", help="listen address (default 0.0.0.0)")
    ap.add_argument("--poll", type=float, default=1.0, help="log poll interval seconds (default 1.0)")
    ap.add_argument("--otlp", metavar="URL",
                    help="also push OTLP/HTTP JSON here, e.g. http://localhost:4318 "
                         "(/v1/metrics is appended to a bare address)")
    ap.add_argument("--otlp-interval", type=float, default=DEFAULT_OTLP_INTERVAL_S, metavar="S",
                    help=f"OTLP push interval seconds (default {DEFAULT_OTLP_INTERVAL_S:g})")
    ap.add_argument("--otlp-header", type=_header, action="append", default=[], metavar="K=V",
                    help="extra OTLP request header, repeatable (e.g. Authorization=Bearer ...)")
    ap.add_argument("--lag-tol", type=float, default=LAG_TOL_DEFAULT, metavar="FRACTION",
                    help=f"recv_lag tolerance, as in pulse-top (default {LAG_TOL_DEFAULT})")
    ap.add_argument("--lag-windows", type=int, default=LAG_WINDOWS_DEFAULT, metavar="N",
                    help=f"recv_lag windows, 0 disables (default {LAG_WINDOWS_DEFAULT})")
    ap.add_argument("--demo", action="store_true", help="export pulse-top's self-generated demo log")
    ap.add_argument("--once", action="store_true",
                    help="read the log once, print the /metrics text to stdout and exit")
    args = ap.parse_args(argv)

    if args.demo:
        from .demo import start_demo_writer

        path = start_demo_writer()
    else:
        path = args.file or default_log_path()
        if path is None:
            print("pulse-export: no probe log ($TMPDIR/topic_freq.<pid>.log) found. Start the "
                  "probe, pass a path, or try --demo.", file=sys.stderr)
            return 2

    follower = MultiFollower(path)
    exporter = Exporter(poll_s=args.poll, lag_tol=args.lag_tol, lag_windows=args.lag_windows)
    lock = threading.Lock()

    def poll() -> None:
        tagged = follower.poll_tagged()
        with lock:
            for src, line in tagged:
                exporter.ingest(src, line)

    if args.once:
        poll()
        sys.stdout.write(render_prometheus(exporter.collect()))
        return 0

    if args.port:
        start_server(exporter, lock, args.bind, args.port)
        print(f"pulse-export: following {path}, metrics on http://{args.bind}:{args.port}/metrics",
              file=sys.stderr)
    otlp = otlp_url(args.otlp) if args.otlp else None
    headers = dict(args.otlp_header)
    start_ns = time.time_ns()
    next_push = time.monotonic() + args.otlp_interval
    failing = False
    try:
        while True:
            poll()
            if otlp and time.monotonic() >= next_push:
                next_push += args.otlp_interval
                with lock:
                    payload = otlp_payload(exporter.collect(), time.time_ns(), start_ns)
                try:
                    push_otlp(otlp, payload, headers)
                    if failing:
                        print(f"pulse-export: OTLP push to {otlp} recovered", file=sys.stderr)
                    failing = False
                except (urllib.error.URLError, OSError, ValueError) as e:
                    if not failing:  # once per outage, not once per interval
                        print(f"pulse-export: OTLP push to {otlp} failed: {e}", file=sys.stderr)
                    failing = True
            time.sleep(args.poll)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())

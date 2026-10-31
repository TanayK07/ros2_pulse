#!/usr/bin/env python3
# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Line up a wire-rate capture with ros2_pulse's publish and callback rates, window by window.

Inputs (positional, any order):
  DIR          a run directory written by run.sh (pulse*.jsonl [+ wire*.csv] [+ meta.json]), or
               a directory of such run directories (one per QoS combo)
  FILE.csv     wire rates (format below). Given next to a DIR, it is used for every run that has
               no wire*.csv of its own: one capture over the whole session is fine, rows are
               matched to runs by time.
  FILE         any other file is a probe log in jsonl format (ROS_TOPIC_STATS_FORMAT=jsonl), one
               file per process (ROS_TOPIC_STATS_OUTPUT_FILE)

Wire CSV: a header row, then one row per sample. Column names are case-insensitive.
  timestamp    unix time, seconds (float). ms / us / ns integers and ISO 8601 also accepted.
               Marks the END of the row's window (--wire-ts-at start if it marks the start).
  topic        optional; "/cmd_vel", "cmd_vel" and the DDS name "rt/cmd_vel" all match /cmd_vel.
               Without it every row is the topic under test.
  msgs_per_s   the wire rate over the row's window, or instead
  count        messages seen in the row's window
  window_s     optional, the row's window length; otherwise the gap to the previous row
               (the median gap after a pause, so a row never stretches over an idle period)
  With neither msgs_per_s nor count, each row is ONE message seen on the wire, and the rate of a
  window is the number of rows inside it. Rows starting with '#' are comments. Rows with the same
  timestamp and topic add up (one row per writer or per reader is fine).

The probe's windows are the timeline: each jsonl record covers (ts_ns - window_s, ts_ns], wall
clock. Wire rows are spread over their own windows and re-binned into the probe's by overlap, so
the two sides need not tick in phase, but their clocks must agree (same host, or NTP). Use
--wire-offset when they do not.

Stdlib only: runs on any python3 >= 3.8, no ROS environment needed.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import glob
import json
import math
import os
import statistics
import sys
from dataclasses import dataclass
from datetime import datetime, timezone

# A window whose source covers less than this fraction of it gets no value for that source
# (shown as "-"), rather than a rate diluted by the uncovered part.
COVERAGE_MIN = 0.95

TS_KEYS = ("timestamp", "unix_s", "unix_time", "ts", "time", "t")
TOPIC_KEYS = ("topic", "topic_name", "name")
RATE_KEYS = ("msgs_per_s", "msg_per_s", "rate", "rate_hz", "hz")
COUNT_KEYS = ("count", "msgs", "messages", "n")
WINDOW_KEYS = ("window_s", "interval_s", "period_s")

CSV_FIELDS = (
    "run", "reliability", "depth", "topic", "window_end_unix", "window_s", "wire_hz", "pub_hz",
    "cb_hz", "cb_minus_wire_hz", "pub_minus_wire_hz", "cb_over_wire", "flags",
)


def normalize_topic(name: str) -> str:
    """'/cmd_vel', 'cmd_vel' and the DDS topic name 'rt/cmd_vel' are all '/cmd_vel'."""
    name = name.strip()
    if name.startswith("rt/"):
        name = name[2:]
    if not name.startswith("/"):
        name = "/" + name
    return name


def parse_unix_seconds(text: str) -> float:
    """Unix seconds from a float, an ms/us/ns integer (by magnitude) or an ISO 8601 string."""
    text = text.strip()
    try:
        value = float(text)
    except ValueError:
        stamp = datetime.fromisoformat(text.replace("Z", "+00:00"))
        if stamp.tzinfo is None:
            stamp = stamp.replace(tzinfo=timezone.utc)
        return stamp.timestamp()
    if value > 1e17:
        return value / 1e9
    if value > 1e14:
        return value / 1e6
    if value > 1e11:
        return value / 1e3
    return value


# --------------------------------------------------------------------------- rate series


@dataclass
class Interval:
    start: float
    end: float
    rate: float


def _overlap(a0: float, a1: float, b0: float, b1: float) -> float:
    return max(0.0, min(a1, b1) - max(a0, b0))


def _union_length(spans: list) -> float:
    total = 0.0
    cur_lo = cur_hi = None
    for lo, hi in sorted(spans):
        if cur_hi is None or lo > cur_hi:
            if cur_hi is not None:
                total += cur_hi - cur_lo
            cur_lo, cur_hi = lo, hi
        else:
            cur_hi = max(cur_hi, hi)
    if cur_hi is not None:
        total += cur_hi - cur_lo
    return total


class IntervalSeries:
    """Piecewise-constant rates. rate(a, b) is the count the intervals put inside (a, b], over
    b - a; coverage is the fraction of (a, b] the source was observing.

    With a span, the source observed the whole span and a time no interval covers was a rate of
    0 (a capture that omits silent windows). Without one, only the intervals were observed.
    """

    def __init__(self, intervals: list, span: tuple | None = None):
        self.intervals = sorted(intervals, key=lambda iv: iv.start)
        self.starts = [iv.start for iv in self.intervals]
        self.max_len = max((iv.end - iv.start for iv in self.intervals), default=0.0)
        self.span = span

    def rate(self, a: float, b: float) -> tuple:
        count = 0.0
        covered = []
        i = bisect.bisect_left(self.starts, a - self.max_len)
        while i < len(self.intervals) and self.intervals[i].start < b:
            iv = self.intervals[i]
            ov = _overlap(a, b, iv.start, iv.end)
            if ov > 0.0:
                count += iv.rate * ov
                covered.append((max(a, iv.start), min(b, iv.end)))
            i += 1
        if self.span is not None:
            seen = _overlap(a, b, self.span[0], self.span[1])
        else:
            seen = _union_length(covered)
        return count / (b - a), seen / (b - a)


class EventSeries:
    """One timestamp per message; observed over span."""

    def __init__(self, times: list, span: tuple):
        self.times = sorted(times)
        self.span = span

    def rate(self, a: float, b: float) -> tuple:
        n = bisect.bisect_right(self.times, b) - bisect.bisect_right(self.times, a)
        return n / (b - a), _overlap(a, b, self.span[0], self.span[1]) / (b - a)


def sum_rate(series: list, a: float, b: float) -> tuple:
    """Several processes' series add up; the window is only as covered as the least covered."""
    if not series:
        return None, 0.0
    total, coverage = 0.0, 1.0
    for s in series:
        r, c = s.rate(a, b)
        total += r
        coverage = min(coverage, c)
    return total, coverage


# --------------------------------------------------------------------------- probe jsonl


@dataclass
class PulseWindow:
    start: float
    end: float
    window_s: float
    partial: bool
    pub_hz: float | None  # None: this process did not publish the topic in this window
    cb_hz: float | None  # None: this process had no receive line for the topic


def read_pulse(path: str, topic: str, warn) -> list:
    """The probe's windows in one jsonl file, with the topic's publish and callback rates.

    Publish = pub_inter_hz + pub_intra_hz (rcl_publish / intra-process publishes in this
    process). Callback = recv_inter_hz + recv_intra_hz (rclcpp callbacks, or successful takes for
    rclpy). Absent keys are "not measured in this process", per the probe's schema.
    """
    windows = []
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if line.startswith("# ts_ns="):
                raise ValueError(
                    f"{path}: text-format probe log; rerun with ROS_TOPIC_STATS_FORMAT=jsonl")
            if not line.startswith("{"):
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                warn(f"{path}:{lineno}: not valid JSON, skipped (a line cut short at exit?)")
                continue
            if "ts_ns" not in rec or "window_s" not in rec:
                continue
            end = int(rec["ts_ns"]) / 1e9
            win = float(rec["window_s"])
            if win <= 0.0:
                continue
            pub = cb = None
            for t in rec.get("topics", []):
                if t.get("topic") != topic:
                    continue
                if "pub_inter_hz" in t or "pub_intra_hz" in t:
                    pub = float(t.get("pub_inter_hz", 0.0)) + float(t.get("pub_intra_hz", 0.0))
                if "recv_inter_hz" in t or "recv_intra_hz" in t:
                    cb = float(t.get("recv_inter_hz", 0.0)) + float(t.get("recv_intra_hz", 0.0))
            windows.append(PulseWindow(end - win, end, win, bool(rec.get("partial")), pub, cb))
    for prev, cur in zip(windows, windows[1:]):
        if cur.end <= prev.end or cur.start < prev.end - 0.5 * cur.window_s:
            warn(f"{path}: windows overlap; looks like several processes share this file. "
                 "Give each process its own ROS_TOPIC_STATS_OUTPUT_FILE.")
            break
    return windows


@dataclass
class PulseSource:
    path: str
    windows: list
    pub: IntervalSeries | None
    cb: IntervalSeries | None


def pulse_source(path: str, topic: str, warn) -> PulseSource:
    windows = read_pulse(path, topic, warn)
    pub = cb = None
    # A process that published (or received) the topic in any window measured it in all of
    # them: a window without the key is 0 Hz there, not unknown. These series are what another
    # process's windows are re-binned from, so partial windows stay out: their rate is spread
    # over time before traffic started (or cut at exit), and spreading it again onto someone
    # else's window would put the wrong share in it. That stretch reads as a gap instead.
    full = [w for w in windows if not w.partial]
    if any(w.pub_hz is not None for w in windows):
        pub = IntervalSeries([Interval(w.start, w.end, w.pub_hz or 0.0) for w in full])
    if any(w.cb_hz is not None for w in windows):
        cb = IntervalSeries([Interval(w.start, w.end, w.cb_hz or 0.0) for w in full])
    return PulseSource(path, windows, pub, cb)


# --------------------------------------------------------------------------- wire csv


def _column(header: list, keys: tuple):
    for key in keys:
        if key in header:
            return header.index(key)
    return None


def read_wire(path: str, topic: str, ts_at: str = "end", window_s: float | None = None,
              offset: float = 0.0):
    """The wire capture for one topic, as an IntervalSeries (rate/count rows) or an EventSeries
    (one row per message)."""
    with open(path, newline="", encoding="utf-8-sig") as f:
        rows = [r for r in csv.reader(f) if r and any(c.strip() for c in r)
                and not r[0].lstrip().startswith("#")]
    if not rows:
        raise ValueError(f"{path}: empty wire CSV")
    header = [h.strip().lower() for h in rows[0]]
    c_ts = _column(header, TS_KEYS)
    if c_ts is None:
        raise ValueError(f"{path}: no timestamp column (one of {', '.join(TS_KEYS)})")
    c_topic = _column(header, TOPIC_KEYS)
    c_rate = _column(header, RATE_KEYS)
    c_count = _column(header, COUNT_KEYS)
    c_win = _column(header, WINDOW_KEYS)
    mode = "rate" if c_rate is not None else "count" if c_count is not None else "events"

    all_ts = []
    mine = {}  # ts -> [summed value, window_s or None]
    seen_topics = set()
    for lineno, r in enumerate(rows[1:], 2):
        try:
            ts = parse_unix_seconds(r[c_ts]) + offset
        except (ValueError, IndexError) as exc:
            raise ValueError(f"{path}:{lineno}: bad timestamp ({exc})") from None
        all_ts.append(ts)
        name = normalize_topic(r[c_topic]) if c_topic is not None else topic
        seen_topics.add(name)
        if name != topic:
            continue
        value = 1.0
        if mode != "events":
            cell = r[c_rate if mode == "rate" else c_count].strip()
            value = float(cell) if cell else 0.0
        win = float(r[c_win]) if c_win is not None and r[c_win].strip() else None
        entry = mine.setdefault(ts, [0.0, win])
        entry[0] += value
    if not mine:
        raise ValueError(f"{path}: no rows for {topic} (topics in file: "
                         f"{', '.join(sorted(seen_topics)) or 'none'})")

    times = sorted(mine)
    if mode == "events":
        return EventSeries([t for t in times for _ in range(int(mine[t][0]))],
                           (min(all_ts), max(all_ts)))

    gaps = [b - a for a, b in zip(times, times[1:]) if b > a]
    typical = window_s or (statistics.median(gaps) if gaps else 1.0)
    intervals = []
    for i, t in enumerate(times):
        value, win = mine[t]
        if win is None and window_s:
            win = window_s
        if win is None:
            if ts_at == "end":
                d = t - times[i - 1] if i > 0 else None
            else:
                d = times[i + 1] - t if i + 1 < len(times) else None
            # after a pause the gap is not this row's window
            win = d if d is not None and 0.0 < d <= 1.5 * typical else typical
        start, end = (t - win, t) if ts_at == "end" else (t, t + win)
        intervals.append(Interval(start, end, value / win if mode == "count" else value))
    lo, hi = min(all_ts), max(all_ts)
    span = (lo - typical, hi) if ts_at == "end" else (lo, hi + typical)
    span = (min(span[0], intervals[0].start), max(span[1], intervals[-1].end))
    return IntervalSeries(intervals, span)


# --------------------------------------------------------------------------- one run


@dataclass
class Row:
    end: float
    window_s: float
    wire_hz: float | None
    pub_hz: float | None
    cb_hz: float | None
    flags: list

    @property
    def full(self) -> bool:
        return not self.flags


@dataclass
class Run:
    label: str
    pulse_paths: list
    wire_path: str | None
    meta: dict


def compare_run(run: Run, topic: str, wire_opts: dict, warn) -> list:
    sources = [pulse_source(p, topic, warn) for p in run.pulse_paths]
    # Timeline: the subscriber's windows (the controller), else a publisher's, else any.
    timeline_src = next((s for s in sources if s.cb is not None), None) \
        or next((s for s in sources if s.pub is not None), None) \
        or next((s for s in sources if s.windows), None)
    if timeline_src is None:
        raise ValueError(f"{run.label}: no probe windows in {', '.join(run.pulse_paths)}")
    if not any(s.cb is not None for s in sources):
        warn(f"{run.label}: no process received {topic} (QoS mismatch, wrong topic, or no "
             "flood?); callback column empty")
    wire = read_wire(run.wire_path, topic, **wire_opts) if run.wire_path else None

    # The timeline process's own rates are taken from its record as written; only the other
    # processes' windows (and the wire rows) are re-binned onto it.
    own_cb = timeline_src.cb is not None
    own_pub = timeline_src.pub is not None
    other_cb = [s.cb for s in sources if s is not timeline_src and s.cb is not None]
    other_pub = [s.pub for s in sources if s is not timeline_src and s.pub is not None]

    rows = []
    for w in timeline_src.windows:
        a, b = w.start, w.end
        flags = ["partial"] if w.partial else []
        values = {}
        for key, own, others, flag in (
                ("cb", (w.cb_hz or 0.0) if own_cb else None, other_cb, "cb-gap"),
                ("pub", (w.pub_hz or 0.0) if own_pub else None, other_pub, "pub-gap"),
                ("wire", None, [wire] if wire else [], "no-wire")):
            if own is None and not others:
                values[key] = None
                continue
            rate, coverage = sum_rate(others, a, b) if others else (0.0, 1.0)
            if coverage < COVERAGE_MIN:
                values[key] = None
                flags.append(flag)
            else:
                values[key] = rate + (own or 0.0)
        rows.append(Row(b, w.window_s, values["wire"], values["pub"], values["cb"], flags))
    return rows


# --------------------------------------------------------------------------- output


def _fmt(v, nd=1):
    return "-" if v is None else f"{v:.{nd}f}"


def _diff(x, y):
    return None if x is None or y is None else x - y


def _ratio(x, y):
    return None if x is None or y is None or y <= 0.0 else x / y


def _clock(t: float) -> str:
    return datetime.fromtimestamp(t, timezone.utc).strftime("%H:%M:%S.%f")[:-3]


def print_table(label: str, meta: dict, rows: list, out, show_windows: bool = True) -> None:
    qos = ""
    if meta.get("reliability"):
        qos = f"  ({meta['reliability']}, depth {meta.get('depth', '?')})"
    print(f"== {label}{qos}", file=out)
    if show_windows:
        head = ("end (UTC)", "win_s", "wire_hz", "pub_hz", "cb_hz", "cb-wire", "cb/wire",
                "pub-wire", "flags")
        widths = (12, 6, 10, 10, 10, 9, 8, 9, 0)
        print("  ".join(h.rjust(n) if n else h for h, n in zip(head, widths)), file=out)
        for r in rows:
            ratio = _ratio(r.cb_hz, r.wire_hz)
            cells = (_clock(r.end), f"{r.window_s:.3f}", _fmt(r.wire_hz), _fmt(r.pub_hz),
                     _fmt(r.cb_hz), _fmt(_diff(r.cb_hz, r.wire_hz)),
                     "-" if ratio is None else f"{100.0 * ratio:.1f}%",
                     _fmt(_diff(r.pub_hz, r.wire_hz)), ",".join(r.flags))
            line = "  ".join(c.rjust(n) if n else c for c, n in zip(cells, widths))
            print(line.rstrip(), file=out)
    for line in summarize(rows):
        print(f"   {line}", file=out)
    print(file=out)


def totals(rows: list) -> dict:
    """Messages per column over the full windows (each column where it has a value)."""
    out = {}
    for key in ("wire", "pub", "cb"):
        vals = [(getattr(r, f"{key}_hz"), r.window_s) for r in rows if r.full]
        vals = [(v, w) for v, w in vals if v is not None]
        out[key] = sum(v * w for v, w in vals) if vals else None
    return out


def summarize(rows: list) -> list:
    full = [r for r in rows if r.full]
    lines = [f"{len(full)} full windows of {len(rows)} "
             "(partial / gap windows are listed but not counted)"]
    if not full:
        return lines
    t = totals(rows)
    parts = [f"{name} {t[key]:.0f} msgs" for key, name in
             (("wire", "wire"), ("pub", "publishes"), ("cb", "callbacks")) if t[key] is not None]
    lines.append(", ".join(parts))
    ratios = []
    if t["cb"] is not None and t["wire"]:
        ratios.append(f"callbacks/wire {100.0 * t['cb'] / t['wire']:.2f}%")
    if t["pub"] is not None and t["wire"]:
        ratios.append(f"publishes/wire {100.0 * t['pub'] / t['wire']:.2f}%")
    if t["cb"] is not None and t["pub"]:
        ratios.append(f"callbacks/publishes {100.0 * t['cb'] / t['pub']:.2f}%")
    if ratios:
        lines.append(", ".join(ratios))
    worst = [(abs(d), d, r) for r in full
             for d in (_diff(r.cb_hz, r.wire_hz),) if d is not None]
    if worst:
        _, d, r = max(worst, key=lambda x: x[0])
        lines.append(f"largest callback-wire gap {d:+.1f} Hz in the window ending {_clock(r.end)}")
    return lines


def csv_rows(label: str, meta: dict, topic: str, rows: list):
    for r in rows:
        ratio = _ratio(r.cb_hz, r.wire_hz)
        yield {
            "run": label, "reliability": meta.get("reliability", ""),
            "depth": meta.get("depth", ""), "topic": topic,
            "window_end_unix": f"{r.end:.6f}", "window_s": f"{r.window_s:.3f}",
            "wire_hz": _fmt(r.wire_hz, 3), "pub_hz": _fmt(r.pub_hz, 3),
            "cb_hz": _fmt(r.cb_hz, 3), "cb_minus_wire_hz": _fmt(_diff(r.cb_hz, r.wire_hz), 3),
            "pub_minus_wire_hz": _fmt(_diff(r.pub_hz, r.wire_hz), 3),
            "cb_over_wire": "-" if ratio is None else f"{ratio:.4f}",
            "flags": ",".join(r.flags),
        }


def worst_pct(rows: list):
    """Largest |callbacks - reference| / reference over full windows, reference = wire when
    there is a wire column, else publishes. None when no full window has both."""
    worst = None
    for r in rows:
        if not r.full or r.cb_hz is None:
            continue
        ref = r.wire_hz if r.wire_hz is not None else r.pub_hz
        if ref is None or ref <= 0.0:
            continue
        pct = 100.0 * abs(r.cb_hz - ref) / ref
        worst = pct if worst is None else max(worst, pct)
    return worst


# --------------------------------------------------------------------------- discovery


_REL_ORDER = {"reliable": 0, "best_effort": 1}


def _load_meta(d: str) -> dict:
    try:
        with open(os.path.join(d, "meta.json"), encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _run_from_dir(d: str, wire_default: str | None) -> Run | None:
    pulses = sorted(glob.glob(os.path.join(d, "pulse*.jsonl")))
    if not pulses:
        return None
    wires = sorted(glob.glob(os.path.join(d, "wire*.csv")))
    return Run(os.path.basename(os.path.normpath(d)), pulses, wires[0] if wires else wire_default,
               _load_meta(d))


def discover_runs(inputs: list, label: str | None) -> list:
    dirs = [p for p in inputs if os.path.isdir(p)]
    wires = [p for p in inputs if not os.path.isdir(p) and p.lower().endswith(".csv")]
    pulses = [p for p in inputs if not os.path.isdir(p) and not p.lower().endswith(".csv")]
    if len(wires) > 1:
        raise ValueError("give one wire CSV (it may cover all runs)")
    wire = wires[0] if wires else None
    runs = []
    for d in dirs:
        run = _run_from_dir(d, wire)
        if run is not None:
            runs.append(run)
            continue
        found = [r for r in (_run_from_dir(os.path.join(d, s), wire)
                             for s in sorted(os.listdir(d))
                             if os.path.isdir(os.path.join(d, s))) if r is not None]
        if not found:
            raise ValueError(f"{d}: no pulse*.jsonl here or in its subdirectories")
        runs.extend(found)
    if pulses:
        runs.append(Run(label or os.path.splitext(os.path.basename(pulses[0]))[0], pulses, wire,
                        {}))
    if not runs:
        raise ValueError("no probe logs given")
    if label and len(runs) == 1:
        runs[0].label = label

    def order(run: Run):
        rel = _REL_ORDER.get(run.meta.get("reliability"), 2)
        depth = run.meta.get("depth")
        return (rel, depth if isinstance(depth, int) else math.inf, run.label)

    return sorted(runs, key=order)


# --------------------------------------------------------------------------- main


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Per-window table: wire rate vs ros2_pulse publish and callback rates.",
        epilog="Wire CSV format and alignment rules: see the module docstring or README.md.",
    )
    ap.add_argument("inputs", nargs="+",
                    help="run dir(s) from run.sh, probe jsonl file(s), and at most one wire CSV")
    ap.add_argument("--topic", default="/cmd_vel", help="topic to compare (default /cmd_vel)")
    ap.add_argument("--label", help="run name when comparing loose files")
    ap.add_argument("--wire-ts-at", choices=("end", "start"), default="end",
                    help="whether a wire row's timestamp marks the end (default) or the start "
                         "of its window")
    ap.add_argument("--wire-window", type=float, metavar="S",
                    help="window length of every wire row, when the CSV has no window_s column "
                         "(default: inferred from the row spacing)")
    ap.add_argument("--wire-offset", type=float, default=0.0, metavar="S",
                    help="seconds added to every wire timestamp (clock offset between hosts)")
    ap.add_argument("--csv", metavar="PATH", help="also write every window of every run here")
    ap.add_argument("--summary", action="store_true", help="print only the per-run summaries")
    ap.add_argument("--max-diff-pct", type=float, metavar="P",
                    help="exit 1 if a full window's callback rate differs from the wire rate "
                         "(the publish rate without a wire CSV) by more than P%%, or if no full "
                         "window can be checked")
    args = ap.parse_args(argv)
    topic = normalize_topic(args.topic)

    def warn(msg):
        print(f"compare.py: {msg}", file=sys.stderr)

    wire_opts = {"ts_at": args.wire_ts_at, "window_s": args.wire_window,
                 "offset": args.wire_offset}
    try:
        runs = discover_runs(args.inputs, args.label)
        results = [(run, compare_run(run, topic, wire_opts, warn)) for run in runs]
    except (OSError, ValueError) as exc:
        print(f"compare.py: {exc}", file=sys.stderr)
        return 2

    for run, rows in results:
        print_table(run.label, run.meta, rows, sys.stdout, show_windows=not args.summary)

    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for run, rows in results:
                writer.writerows(csv_rows(run.label, run.meta, topic, rows))

    if args.max_diff_pct is not None:
        failed = False
        for run, rows in results:
            pct = worst_pct(rows)
            if pct is None:
                warn(f"{run.label}: no full window with both a callback rate and a reference")
                failed = True
            elif pct > args.max_diff_pct:
                warn(f"{run.label}: callbacks differ from the reference by up to {pct:.1f}% "
                     f"(limit {args.max_diff_pct:g}%)")
                failed = True
        if failed:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

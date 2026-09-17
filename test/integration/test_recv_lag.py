# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# recv_lag (issue #50): a publisher's process and a subscriber's process are probed separately,
# and the probe cannot compare them (each judges only the endpoints it hosts), so the comparison
# is pulse-top's. Two claims, in this order so the first stands without pulse-top: (1) the two
# logs carry the gap, a slow subscriber's callback rate reads well under the talker's publish
# rate; (2) pulse-top's model derives a recv_lag warn from exactly those two files. Plus a
# control: a healthy pair never warns, so the subscriber's partial first window (discovery
# latency) is not a false positive. Distro-independent (inter-process rates only).

import json
import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

PULSE_TOP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools", "pulse-top")
JSONL = {"ROS_TOPIC_STATS_FORMAT": "jsonl"}
# A 100 ms sleep in the callback caps the single-threaded executor at ~10 callbacks/s against a
# 50 Hz talker; KeepLast(10) RELIABLE overflows the reader history and the DDS reader drops the
# oldest samples. This is the "executor behind" case the design assumes the probe reports as-is.
SLOW_CALLBACK_MS = 100


def _records(text):
    return [json.loads(ln) for ln in text.splitlines() if ln.strip()]


def _rates(recs, topic, key):
    return [t[key] for r in recs for t in r["topics"] if t["topic"] == topic and key in t]


def _model():
    """pulse-top's model straight from the source tree: stdlib only, textual is not needed.
    Skip with the reason, never pass silently, if the tree layout changed."""
    if PULSE_TOP not in sys.path:
        sys.path.insert(0, PULSE_TOP)
    try:
        from pulse_top.model import StatsState, parse_jsonl_line
    except ImportError as e:  # pragma: no cover
        pytest.skip(f"pulse-top model not importable from {PULSE_TOP}: {e}")
    return StatsState, parse_jsonl_line


def _derive_recv_lag(paths):
    """Feed every window of the given logs to the model in ts_ns order, tagged by file (what the
    app does per poll), and return every recv_lag warn that fired, cleared or not."""
    StatsState, parse_jsonl_line = _model()
    windows = []
    for p in paths:
        with open(p) as f:
            for ln in f:
                w = parse_jsonl_line(ln)
                if w is not None:
                    windows.append((w.ts_ns, p, w))
    s = StatsState()
    for _, p, w in sorted(windows, key=lambda x: x[0]):
        s.apply(w, source=p)
    return [w for _, w in s.recent_warns if w.kind == "recv_lag"]


def test_slow_subscriber_logs_carry_the_gap_and_pulse_top_derives_recv_lag():
    with tempfile.TemporaryDirectory() as d:
        out_t, out_l = os.path.join(d, "topic_freq.1.log"), os.path.join(d, "topic_freq.2.log")
        tt, lt = ph.run_pair(out_t, out_l, run_s=8.0, period="1.0", extra_env=JSONL,
                             listener_mode="slow_listener", listener_args=(SLOW_CALLBACK_MS,))
        talker, listener = _records(tt), _records(lt)
        assert talker and listener, f"a probed process wrote no windows\n--- talker ---\n{tt}\n--- listener ---\n{lt}"

        # (1) The logs carry the gap. Interior windows only: the first is ramp-up, the last is
        # truncated by SIGTERM (probe_harness.measured_rate applies the same rule).
        pub = _rates(talker[1:-1], "/chatter", "pub_inter_hz")
        lo, hi = ph.KNOWN_RATE_HZ * 0.75, ph.KNOWN_RATE_HZ * 1.25
        assert pub and lo <= max(pub) <= hi, f"talker pub_inter_hz {pub} outside [{lo},{hi}]\n{tt}"
        recv = _rates(listener[1:-1], "/chatter", "recv_inter_hz")
        assert recv and max(recv) <= 15.0, (
            f"slow listener recv_inter_hz {recv} not under 15 Hz; the executor is not behind\n{lt}")

        # (2) pulse-top's model derives the warn from those two files.
        fired = _derive_recv_lag([out_t, out_l])
        assert fired, f"no recv_lag derived\n--- talker ---\n{tt}\n--- listener ---\n{lt}"
        w = fired[0]
        assert w.topic == "/chatter" and w.source == out_l, w
        assert w.pub_hz > w.recv_hz and w.windows >= 3, w


def test_healthy_pair_never_warns():
    """Control: talker + plain listener. The subscriber's first window is partial (probe attaches
    mid-flight, discovery adds latency) and reads under the publish rate; N consecutive windows
    must absorb it."""
    with tempfile.TemporaryDirectory() as d:
        out_t, out_l = os.path.join(d, "topic_freq.1.log"), os.path.join(d, "topic_freq.2.log")
        tt, lt = ph.run_pair(out_t, out_l, run_s=8.0, period="1.0", extra_env=JSONL)
        assert _records(tt) and _records(lt), f"a probed process wrote no windows\n{tt}\n{lt}"
        recv = _rates(_records(lt)[1:-1], "/chatter", "recv_inter_hz")
        assert recv and max(recv) >= ph.KNOWN_RATE_HZ * 0.75, f"listener never kept up\n{lt}"
        assert _derive_recv_lag([out_t, out_l]) == [], f"false positive\n--- talker ---\n{tt}\n--- listener ---\n{lt}"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

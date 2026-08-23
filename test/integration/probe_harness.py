# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Reusable integration harness for driving the real probe (LD_PRELOAD) over known-rate ROS
# nodes and reading back per-topic frequencies. Kept dependency-free (no launch_testing) so the
# accuracy tests here AND the per-issue regression tests in the fix PRs can share one code path.
#
# Output format produced by src/probe/interposers.cpp, one block per window:
#     # ts_ns=<int> window_s=<float>
#     TOPIC <name> <inter_hz>
#     PUB <name> inter=<inter_hz> intra=<intra_hz>     (jazzy+: publish-side intra)
#     RECV <name> inter=<inter_hz> intra=<intra_hz>
#     NODE <name>
#     WARN TOPIC <name> hz=<hz> expected=[lo,hi]       (only with ROS_TOPIC_STATS_EXPECTED)
#     WARN NODE <name> missing
#     <blank line>

import os
import re
import subprocess
import time

from ament_index_python.packages import get_package_prefix

PKG = "ros2_pulse"

# test/integration/test_nodes.cpp publishes on a 20 ms wall timer -> 50 Hz nominal.
KNOWN_RATE_HZ = 50.0

_HEADER_RE = re.compile(r"^# ts_ns=(\d+)\s+window_s=([\d.]+)", re.M)
_TOPIC_RE = re.compile(r"^TOPIC (\S+) ([\d.]+)$")
_PUB_RE = re.compile(r"^PUB (\S+) inter=([\d.]+) intra=([\d.]+)$")
_RECV_RE = re.compile(r"^RECV (\S+) inter=([\d.]+) intra=([\d.]+)$")
# Deliberately NOT $-anchored, unlike the three above: R5 may grow this line (min_dt_ms was cut
# but is re-addable), and an anchored regex here would silently stop matching every JITTER line
# the day a field is appended, which is exactly how appending to RECV would have broken this
# harness. Keep it prefix-matching.
_JITTER_RE = re.compile(r"^JITTER (\S+) (pub|recv) max_dt_ms=([\d.]+)")


def probe_paths():
    """Return (probe.so, test-node binary), asserting both are installed."""
    prefix = get_package_prefix(PKG)
    so = os.path.join(prefix, "lib", "libros2_pulse.so")
    node = os.path.join(prefix, "lib", PKG, "stats_probe_test_nodes")
    assert os.path.exists(so), f"probe .so not found: {so}"
    assert os.path.exists(node), f"test nodes not found: {node}"
    return so, node


def make_env(so, out_path, period="1.0", extra=None):
    """LD_PRELOAD env for one probed process writing to out_path with the given flush period."""
    env = dict(os.environ)
    env["LD_PRELOAD"] = so
    env["ROS_TOPIC_STATS_OUTPUT_FILE"] = out_path
    env["ROS_TOPIC_STATISTICS_PUBLISH_PERIOD"] = str(period)
    if extra:
        env.update(extra)
    return env


def parse_windows(text):
    """Split the log into per-window dicts.

    Returns a list (in file order); each entry is
        {"window_s": float, "topic": {name: hz}, "recv": {name: (inter_hz, intra_hz)}}.
    """
    windows = []
    matches = list(_HEADER_RE.finditer(text))
    for idx, m in enumerate(matches):
        start = m.end()
        end = matches[idx + 1].start() if idx + 1 < len(matches) else len(text)
        block = text[start:end]
        win = {"window_s": float(m.group(2)), "topic": {}, "pub": {}, "recv": {}}
        for line in block.splitlines():
            tm = _TOPIC_RE.match(line)
            if tm:
                win["topic"][tm.group(1)] = float(tm.group(2))
                continue
            pm = _PUB_RE.match(line)
            if pm:
                win["pub"][pm.group(1)] = (float(pm.group(2)), float(pm.group(3)))
                continue
            rm = _RECV_RE.match(line)
            if rm:
                win["recv"][rm.group(1)] = (float(rm.group(2)), float(rm.group(3)))
        windows.append(win)
    return windows


def _value(win, topic, field):
    if field == "topic":
        return win["topic"].get(topic)
    pair = win["recv"].get(topic)
    if pair is None:
        return None
    return pair[0] if field == "inter" else pair[1]


def measured_rate(windows, topic, field):
    """Best steady-state rate for a topic/field.

    The first window (ramp-up) and last window (truncated by SIGTERM) under-count, so when there
    are >= 3 windows we drop both ends and take the max of the interior; otherwise we use whatever
    is available. Returns None if the topic never appeared.
    """
    interior = windows[1:-1] if len(windows) >= 3 else windows
    vals = [v for v in (_value(w, topic, field) for w in interior) if v is not None]
    if not vals:
        vals = [v for v in (_value(w, topic, field) for w in windows) if v is not None]
    return max(vals) if vals else None


def run_single(mode, out_path, run_s=6.0, period="1.0", extra_env=None, mode_args=()):
    """Run one probed node for run_s seconds; return its log text.

    mode = talker|listener|intra|stall_pub|... ; mode_args are extra argv for the node
    (stall_pub takes <stall_ms> <stall_at_s>).
    """
    so, node = probe_paths()
    p = subprocess.Popen([node, mode, *(str(a) for a in mode_args)],
                         env=make_env(so, out_path, period, extra_env))
    try:
        time.sleep(run_s)
    finally:
        p.terminate()
        p.wait(timeout=5)
    assert os.path.exists(out_path), f"probe produced no output file for mode={mode}"
    with open(out_path) as f:
        return f.read()


def run_pair(out_talker, out_listener, run_s=6.0, period="1.0", extra_env=None):
    """Run talker + listener as SEPARATE processes (true inter-process). Return (talker, listener)."""
    so, node = probe_paths()
    pt = subprocess.Popen([node, "talker"], env=make_env(so, out_talker, period, extra_env))
    pl = subprocess.Popen([node, "listener"], env=make_env(so, out_listener, period, extra_env))
    try:
        time.sleep(run_s)
    finally:
        for p in (pt, pl):
            p.terminate()
            p.wait(timeout=5)
    tt = open(out_talker).read() if os.path.exists(out_talker) else ""
    lt = open(out_listener).read() if os.path.exists(out_listener) else ""
    return tt, lt


def assert_rate_within(text, topic, expected, field, rel_tol, note=""):
    """Assert the measured rate for topic/field is within rel_tol (fraction) of expected."""
    windows = parse_windows(text)
    got = measured_rate(windows, topic, field)
    assert got is not None, (
        f"topic {topic} ({field}) never reported {note}\n--- log ---\n{text}"
    )
    lo, hi = expected * (1.0 - rel_tol), expected * (1.0 + rel_tol)
    assert lo <= got <= hi, (
        f"{topic} {field}={got:.3f}Hz outside [{lo:.3f},{hi:.3f}] "
        f"(expected {expected}Hz +/-{rel_tol * 100:.0f}%) {note}\n--- log ---\n{text}"
    )
    return got

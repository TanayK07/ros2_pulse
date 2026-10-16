# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Output format integration (ROADMAP R6): ROS_TOPIC_STATS_FORMAT=jsonl makes a REAL probed
# process write one JSON object per window, every non-blank line must satisfy a standard
# JSON parser (python's json module, deliberately not our own reader: an independent
# implementation is what proves the format claim). An unknown value warns once on stderr and
# falls back to text, and pulse-check gates jsonl logs with the same verdicts as text ones.
# Distro-independent (uses only inter-process publish rates).

import json
import os
import subprocess
import sys
import tempfile
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

from ament_index_python.packages import get_package_prefix  # noqa: E402


def pulse_check_path():
    p = os.path.join(get_package_prefix(ph.PKG), "lib", ph.PKG, "pulse-check")
    assert os.path.exists(p), f"pulse-check not installed: {p}"
    return p


def _write(d, name, text):
    path = os.path.join(d, name)
    with open(path, "w") as f:
        f.write(text)
    return path


def _jsonl_windows(text):
    """Every non-blank line MUST parse as one JSON object, json.loads raising fails the test,
    which is the point: jsonlines.org compliance checked by an independent parser."""
    wins = []
    for ln in text.splitlines():
        if not ln.strip():
            continue
        obj = json.loads(ln)
        assert isinstance(obj, dict), f"jsonl line is not an object: {ln!r}"
        wins.append(obj)
    return wins


def test_jsonl_windows_are_valid_json_with_real_rates():
    """A real talker in jsonl mode: schema keys present, ts_ns a digit string (int64 > 2^53
    must not be a JSON number), and the known 50 Hz rate readable straight out of the record."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "out.jsonl")
        text = ph.run_single("talker", out, run_s=6.0, period="1.0",
                             extra_env={"ROS_TOPIC_STATS_FORMAT": "jsonl"})
        wins = _jsonl_windows(text)
        assert wins, f"no jsonl windows\n--- log ---\n{text}"

        for w in wins:
            assert isinstance(w["ts_ns"], str) and w["ts_ns"].isdigit(), w
            assert isinstance(w["window_s"], float), w
            # always-present arrays: exporters must not need null guards
            assert isinstance(w["topics"], list), w
            assert isinstance(w["nodes"], list), w
            assert isinstance(w["warns"], list), w

        interior = wins[1:-1] if len(wins) >= 3 else wins
        rates = [t["pub_inter_hz"] for w in interior for t in w["topics"]
                 if t["topic"] == "/chatter" and "pub_inter_hz" in t]
        assert rates, f"/chatter never reported\n--- log ---\n{text}"
        best = max(rates)
        lo, hi = ph.KNOWN_RATE_HZ * 0.75, ph.KNOWN_RATE_HZ * 1.25
        assert lo <= best <= hi, f"pub_inter_hz={best} outside [{lo},{hi}]\n{text}"
        assert any("/poc_talker" in w["nodes"] for w in wins), text

        # Per-endpoint split: always present in probe jsonl, and the /chatter publisher is
        # attributed to the node that created it (real rcl_publisher_init -> rcl_node_init).
        for w in wins:
            assert isinstance(w["endpoints"], list), w
        eps = [e for w in interior for e in w["endpoints"]
               if e["topic"] == "/chatter" and e["side"] == "pub"]
        assert eps, f"/chatter publisher endpoint never reported\n{text}"
        assert all(e.get("node") == "/poc_talker" for e in eps), eps
        assert lo <= max(e["inter_hz"] for e in eps) <= hi, eps


def test_text_endpoint_lines_are_opt_in():
    """ROS_TOPIC_STATS_ENDPOINTS=1 appends EP lines to text windows; without it the text
    format is unchanged (no EP line at all)."""
    with tempfile.TemporaryDirectory() as d:
        on = ph.run_single("talker", os.path.join(d, "on.log"), run_s=4.0, period="1.0",
                           extra_env={"ROS_TOPIC_STATS_ENDPOINTS": "1"})
        off = ph.run_single("talker", os.path.join(d, "off.log"), run_s=4.0, period="1.0")
        ep = [ln for ln in on.splitlines() if ln.startswith("EP /chatter pub ")]
        assert ep, f"no EP line for /chatter\n{on}"
        assert all(" node=/poc_talker " in ln for ln in ep), ep
        assert not [ln for ln in off.splitlines() if ln.startswith("EP ")], off


def test_destroyed_endpoints_stop_being_reported():
    """A real rclcpp publisher and subscription destroyed mid-run (node still alive) leave the
    endpoints list after one final window, through the rcl_*_fini wrappers. Before them, the
    once-delivered subscription stayed forever as a `recv inter_hz=0.0` endpoint of the live
    node, and with gap tracking the publisher reported an ever-growing max_dt_ms."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "out.jsonl")
        text = ph.run_single("churn", out, run_s=8.0, period="1.0",
                             extra_env={"ROS_TOPIC_STATS_FORMAT": "jsonl",
                                        "ROS_TOPIC_STATS_JITTER": "1"})
        wins = _jsonl_windows(text)
        assert len(wins) >= 6, f"too few windows\n{text}"

        def churn_eps(w):
            return [e for e in w["endpoints"] if e["topic"] == "/churn"]

        assert any(e["side"] == "recv" and e["inter_hz"] > 0 and e.get("node") == "/poc_churn"
                   for w in wins for e in churn_eps(w)), f"/churn never delivered\n{text}"
        last = max(i for i, w in enumerate(wins) if churn_eps(w))
        later = wins[last + 1:]
        assert len(later) >= 2, (
            f"/churn endpoints still reported {len(wins) - 1 - last} window(s) from the end, "
            f"long after both were destroyed\n{text}")
        # The process kept running and writing endpoints the whole time.
        assert all(any(e["topic"] == "/churn_keepalive" for e in w["endpoints"])
                   for w in later[:-1]), text


def test_unknown_format_warns_once_and_falls_back_to_text():
    """A typo'd format must neither crash the host nor be silently ignored: exactly one
    stderr warning, and the log is plain text (byte-compatible headers, no '{' lines)."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "out.log")
        so, node = ph.probe_paths()
        env = ph.make_env(so, out, "1.0", {"ROS_TOPIC_STATS_FORMAT": "json"})  # likely typo
        p = subprocess.Popen([node, "talker"], env=env, stderr=subprocess.PIPE)
        try:
            time.sleep(4.0)
        finally:
            p.terminate()
            p.wait(timeout=5)
        stderr = p.stderr.read().decode()

        assert stderr.count("unknown ROS_TOPIC_STATS_FORMAT 'json'") == 1, stderr
        assert "format=text" in stderr, stderr  # the banner states the effective format

        with open(out) as f:
            text = f.read()
        assert "# ts_ns=" in text, f"fallback did not produce text windows\n{text}"
        assert not [ln for ln in text.splitlines() if ln.startswith("{")], text


def test_pulse_check_gates_jsonl_logs():
    """The R6 decision under test: pulse-check reads jsonl logs (per-line sniff in parseLog),
    so switching the probe to the exporter-friendly format keeps CI/watchdog gating, same
    verdicts, same exit codes, and NEVER a silent exit 0 on bad input."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "run.jsonl")
        ph.run_single("talker", out, run_s=6.0, period="1.0",
                      extra_env={"ROS_TOPIC_STATS_FORMAT": "jsonl"})
        cli = pulse_check_path()
        passing = _write(d, "pass.yaml",
                         "topics:\n  /chatter: {min_hz: 35, max_hz: 65, side: pub}\n"
                         "nodes: [/poc_talker]\n")
        failing = _write(d, "fail.yaml",
                         "topics:\n  /chatter: {min_hz: 80, side: pub}\n")

        ok = subprocess.run([cli, "--spec", passing, out], capture_output=True, text=True)
        assert ok.returncode == 0, f"expected pass, got {ok.returncode}\n{ok.stdout}{ok.stderr}"
        assert ok.stdout == ""

        bad = subprocess.run([cli, "--spec", failing, out], capture_output=True, text=True)
        assert bad.returncode == 1, f"expected exit 1\n{bad.stdout}{bad.stderr}"
        assert "WARN TOPIC /chatter" in bad.stdout

        # A file with no probe window in EITHER format stays bad input (exit 2), the
        # "must not silently exit 0" guarantee. First line is well-formed JSON but not a
        # probe window (no header fields); second is a truncated record.
        junk = _write(d, "junk.jsonl", '{"not": "a probe window"}\n{truncated\n')
        r = subprocess.run([cli, "--spec", passing, junk], capture_output=True, text=True)
        assert r.returncode == 2, f"expected exit 2\n{r.stdout}{r.stderr}"
        assert "no probe windows" in r.stderr, r.stderr


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Gap visibility integration (ROADMAP R5): a real probed publisher against a real stall. These
# tests exist because a windowed mean CANNOT detect a stall, at 50 Hz a `min_hz: 45` rule needs
# more than half a second of dead time to fire, so the whole feature rests on max_dt_ms being
# the window-length-independent detector. Distro-independent (inter-process publish rates only).

import os
import re
import subprocess
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

from ament_index_python.packages import get_package_prefix  # noqa: E402


def _write(d, name, text):
    path = os.path.join(d, name)
    with open(path, "w") as f:
        f.write(text)
    return path


def _warn_lines(text):
    return [ln for ln in text.splitlines() if ln.startswith("WARN ")]


def _jitter(text, topic, side):
    """Every max_dt_ms reported for one topic/side, in window order."""
    out = []
    for ln in text.splitlines():
        m = ph._JITTER_RE.match(ln)
        if m and m.group(1) == topic and m.group(2) == side:
            out.append(float(m.group(3)))
    return out


def _windows_blocks(text):
    blocks, cur = [], None
    for ln in text.splitlines():
        if ln.startswith("# ts_ns="):
            cur = []
            blocks.append(cur)
        elif cur is not None:
            cur.append(ln)
    return blocks


def pulse_check_path():
    p = os.path.join(get_package_prefix(ph.PKG), "lib", ph.PKG, "pulse-check")
    assert os.path.exists(p), f"pulse-check not installed: {p}"
    return p


def test_gap_detected_where_mean_rate_passes():
    """THE R5 thesis, executable: the mean rate stays green while the gap rule fires.

    50 Hz publisher, 5 s windows, one 800 ms freeze. The stalled window loses ~40 of its ~250
    messages, so it still measures ~42 Hz and a `min_hz: 40` rule passes, while max_dt_ms is
    ~800 and a `max_gap_ms: 100` rule catches it. If this test ever fails because the rate rule
    fired, the premise changed and the whole feature needs rethinking.
    """
    with tempfile.TemporaryDirectory() as d:
        spec = _write(d, "spec.yaml",
                      "topics:\n  /chatter: {min_hz: 40, max_gap_ms: 100, side: pub}\n")
        out = os.path.join(d, "stall.log")
        text = ph.run_single("stall_pub", out, run_s=16.0, period="5.0",
                             extra_env={"ROS_TOPIC_STATS_EXPECTED": spec},
                             mode_args=(800, 7.0))

        warns = _warn_lines(text)
        rate_warns = [w for w in warns if " hz=" in w]
        gap_warns = [w for w in warns if "max_dt_ms=" in w]

        # The setup guard: if the mean rate DID drop below 40 the test proves nothing, because
        # min_hz alone would have caught the stall and R5 would be unnecessary here.
        assert not rate_warns, (
            "mean rate fired, so this stall was NOT invisible to min_hz, the test no longer "
            f"demonstrates the gap detector\n{chr(10).join(rate_warns)}\n--- log ---\n{text}")

        assert gap_warns, f"gap rule did not fire on an 800 ms stall\n--- log ---\n{text}"
        found = [re.search(r"max_dt_ms=([\d.]+)", w) for w in gap_warns]
        biggest = max(float(m.group(1)) for m in found if m is not None)
        assert biggest >= 800.0, f"expected a gap >= 800 ms, got {biggest}\n{text}"


def test_jitter_off_by_default():
    """No env var and no gap rule: not one JITTER line, and the banner says so."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "plain.log")
        text = ph.run_single("talker", out, run_s=4.0, period="1.0")
        assert not [ln for ln in text.splitlines() if ln.startswith("JITTER ")], text


def test_gap_rule_auto_enables_measurement():
    """A max_gap_ms rule turns tracking on by itself, a declared rule is never silently
    unchecked, the same stance as the zero-rule-spec warning."""
    with tempfile.TemporaryDirectory() as d:
        spec = _write(d, "spec.yaml", "topics:\n  /chatter: {max_gap_ms: 500, side: pub}\n")
        out = os.path.join(d, "auto.log")
        text = ph.run_single("talker", out, run_s=4.0, period="1.0",
                             extra_env={"ROS_TOPIC_STATS_EXPECTED": spec})  # no JITTER env var
        gaps = _jitter(text, "/chatter", "pub")
        assert gaps, f"a max_gap_ms rule must enable measurement on its own\n{text}"
        # 50 Hz publisher: the largest gap in a healthy window sits near the 20 ms period.
        assert all(5.0 < g < 500.0 for g in gaps), f"implausible gaps {gaps}\n{text}"
        assert not _warn_lines(text), f"healthy publisher warned\n{text}"


def test_exit_window_gap_is_not_folded_or_judged():
    """rclcpp teardown stops traffic before the process exits, so the final window's open
    interval measures the shutdown, folding it would fire every gap rule on a healthy stop.
    This exercises the flush(exiting=true) plumbing, which the unit tests cannot reach."""
    with tempfile.TemporaryDirectory() as d:
        spec = _write(d, "spec.yaml", "topics:\n  /chatter: {max_gap_ms: 60, side: pub}\n")
        out = os.path.join(d, "exit.log")
        text = ph.run_single("talker", out, run_s=4.0, period="1.0",
                             extra_env={"ROS_TOPIC_STATS_EXPECTED": spec})
        blocks = _windows_blocks(text)
        assert len(blocks) >= 2, f"need >=2 windows\n{text}"
        assert not any(ln.startswith("WARN ") for ln in blocks[-1]), (
            "exit window must not be alert-judged\n" + "\n".join(blocks[-1]))
        tail_gaps = [float(ph._JITTER_RE.match(ln).group(3))
                     for ln in blocks[-1] if ln.startswith("JITTER ")]
        assert all(g < 500.0 for g in tail_gaps), (
            f"teardown quiet leaked into the exit window's gap: {tail_gaps}")


def test_pulse_check_gates_on_max_gap():
    """Offline verdicts: satisfied gap rule exits 0, violated exits 1 with the pinned line."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "run.log")
        ph.run_single("talker", out, run_s=6.0, period="1.0",
                      extra_env={"ROS_TOPIC_STATS_JITTER": "1"})
        cli = pulse_check_path()

        lenient = _write(d, "ok.yaml", "topics:\n  /chatter: {max_gap_ms: 500, side: pub}\n")
        r = subprocess.run([cli, "--spec", lenient, "--skip-last", out],
                           capture_output=True, text=True)
        assert r.returncode == 0, f"expected pass\n{r.stdout}{r.stderr}"

        strict = _write(d, "bad.yaml", "topics:\n  /chatter: {max_gap_ms: 1, side: pub}\n")
        r = subprocess.run([cli, "--spec", strict, "--skip-last", out],
                           capture_output=True, text=True)
        assert r.returncode == 1, f"expected violation\n{r.stdout}{r.stderr}"
        assert "expected_max_gap_ms=1" in r.stdout, r.stdout


def test_pulse_check_reports_measured_violations_alongside_exit_2():
    """An unmeasurable gap rule must not swallow violations that WERE measured: someone
    debugging a red CI job needs to see the real fault, not just a config complaint."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "nojitter.log")
        ph.run_single("talker", out, run_s=5.0, period="1.0")  # no jitter measured
        cli = pulse_check_path()
        spec = _write(d, "mixed.yaml",
                      "topics:\n"
                      "  /chatter: {min_hz: 500, side: pub}\n"      # measured, and violated
                      "  /chatter: {max_gap_ms: 50, side: recv}\n")  # unmeasurable
        r = subprocess.run([cli, "--spec", spec, "--skip-last", out],
                           capture_output=True, text=True)
        assert r.returncode == 2, f"unmeasurable rule must dominate the exit code\n{r.stderr}"
        assert "WARN TOPIC /chatter hz=" in r.stdout, (
            f"the measured violation was swallowed by the exit-2 path\n{r.stdout}{r.stderr}")
        assert "no JITTER line" in r.stderr, r.stderr


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

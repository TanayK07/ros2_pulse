# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Expected-rate alerting integration (ROADMAP R1): a real probed talker against a spec file,
# WARN lines appear when the spec is violated, stay absent when satisfied, the first window is
# grace-skipped, and the pulse-check CLI reproduces the verdict offline with the right exit
# codes. Distro-independent (uses only inter-process publish rates).

import os
import subprocess
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

from ament_index_python.packages import get_package_prefix  # noqa: E402

# test_nodes talker: node /poc_talker publishing /chatter at 50 Hz nominal.
PASSING_SPEC = (
    "topics:\n"
    "  /chatter: {min_hz: 35, max_hz: 65, side: pub}\n"
    "nodes: [/poc_talker]\n"
)
FAILING_SPEC = (
    "topics:\n"
    "  /chatter: {min_hz: 80, side: pub}\n"
    "  /ghost:   {min_hz: 5}\n"          # not hosted by the talker: probe must SKIP it
    "nodes: [/some_other_process_node]\n"  # never inited here: probe must SKIP it
)


def _write(d, name, text):
    path = os.path.join(d, name)
    with open(path, "w") as f:
        f.write(text)
    return path


def _warn_lines(text):
    return [ln for ln in text.splitlines() if ln.startswith("WARN ")]


def _windows_blocks(text):
    """Window blocks in file order (split on the '# ts_ns=' headers)."""
    blocks = []
    cur = None
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


def test_underrate_topic_warns_with_first_window_grace():
    """50 Hz talker vs min_hz 80: WARN TOPIC lines appear, but never in the first window."""
    with tempfile.TemporaryDirectory() as d:
        spec = _write(d, "spec.yaml", FAILING_SPEC)
        out = os.path.join(d, "warn.log")
        text = ph.run_single("talker", out, run_s=6.0, period="1.0",
                             extra_env={"ROS_TOPIC_STATS_EXPECTED": spec})
        warns = _warn_lines(text)
        assert any(w.startswith("WARN TOPIC /chatter hz=") and "expected=[80,inf]" in w
                   for w in warns), f"no under-rate WARN for /chatter\n--- log ---\n{text}"
        # Foreign endpoints/nodes are another process's business, the probe must not warn.
        assert not any("/ghost" in w for w in warns), f"probe warned about a foreign topic\n{text}"
        assert not any("/some_other_process_node" in w for w in warns), (
            f"probe warned about a foreign node\n{text}")
        blocks = _windows_blocks(text)
        assert len(blocks) >= 2, f"need >=2 windows to see the grace skip\n{text}"
        assert not any(ln.startswith("WARN ") for ln in blocks[0]), (
            f"first (ramp-up) window must be grace-skipped\n--- first block ---\n"
            + "\n".join(blocks[0]))


def test_exit_flush_window_is_not_alert_judged():
    """The final atexit window is logged but never judged, a healthy stop must not alert.

    The tail window is truncated by process exit, so count/window_s over a sliver is a one- or
    two-sample estimate that swings in both directions (and reads 0 Hz when the sliver caught no
    message). The spec here is violated in EVERY window regardless of length, so an unguarded
    exit flush would always warn: the assertion is the invariant, not a timing race.
    """
    with tempfile.TemporaryDirectory() as d:
        spec = _write(d, "spec.yaml", "topics:\n  /chatter: {min_hz: 500, side: pub}\n")
        out = os.path.join(d, "exit.log")
        text = ph.run_single("talker", out, run_s=4.0, period="1.0",
                             extra_env={"ROS_TOPIC_STATS_EXPECTED": spec})
        blocks = _windows_blocks(text)
        assert len(blocks) >= 3, f"need >=3 windows (first, interior, exit)\n{text}"
        # an interior window must warn, otherwise the test proves nothing
        assert any(ln.startswith("WARN ") for ln in blocks[1]), (
            f"interior window should have warned\n--- block 1 ---\n" + "\n".join(blocks[1]))
        assert not any(ln.startswith("WARN ") for ln in blocks[-1]), (
            f"exit window must be grace-skipped\n--- last block ---\n" + "\n".join(blocks[-1]))
        # the tail window is still MEASURED and logged, just not judged
        assert blocks[-1], f"exit window should still be emitted\n{text}"


def test_satisfied_spec_stays_silent():
    """50 Hz talker vs [35,65] + its own node expected: zero WARN lines."""
    with tempfile.TemporaryDirectory() as d:
        spec = _write(d, "spec.yaml", PASSING_SPEC)
        out = os.path.join(d, "ok.log")
        text = ph.run_single("talker", out, run_s=6.0, period="1.0",
                             extra_env={"ROS_TOPIC_STATS_EXPECTED": spec})
        assert not _warn_lines(text), f"unexpected WARN on a satisfied spec\n--- log ---\n{text}"


def test_invalid_spec_disables_alerting_without_crashing():
    """A malformed spec must warn-and-disable, not take the host process down."""
    with tempfile.TemporaryDirectory() as d:
        spec = _write(d, "spec.yaml", "topics:\n  /chatter: {rate: 9}\n")
        out = os.path.join(d, "bad.log")
        text = ph.run_single("talker", out, run_s=4.0, period="1.0",
                             extra_env={"ROS_TOPIC_STATS_EXPECTED": spec})
        assert not _warn_lines(text)
        # the probe still measured normally
        ph.assert_rate_within(text, "/chatter", ph.KNOWN_RATE_HZ, field="topic", rel_tol=0.30,
                              note="(alerting disabled, measurement intact)")


def test_hostile_spec_paths_disable_alerting_without_stalling():
    """A spec path that isn't a spec must degrade instantly, not stall or balloon the host.

    All three used to be read unbounded inside the singleton constructor reached from
    rcl_node_init: /dev/zero never reaches EOF, a directory yields empty text that parses as a
    valid zero-rule spec (alerting silently armed as a no-op), and an oversize file costs
    seconds of startup and GBs of RSS in every preloaded process.
    """
    with tempfile.TemporaryDirectory() as d:
        oversize = os.path.join(d, "big.yaml")
        with open(oversize, "w") as f:
            f.write("#" * (1024 * 1024 + 1))  # one byte past the 1 MiB cap

        for label, spec in (("oversize", oversize), ("chardev", "/dev/zero"), ("directory", d)):
            out = os.path.join(d, f"{label}.log")
            text = ph.run_single("talker", out, run_s=4.0, period="1.0",
                                 extra_env={"ROS_TOPIC_STATS_EXPECTED": spec})
            assert not _warn_lines(text), f"{label} spec should disable alerting\n{text}"
            # the probe must still have measured normally throughout
            ph.assert_rate_within(text, "/chatter", ph.KNOWN_RATE_HZ, field="topic", rel_tol=0.30,
                                  note=f"({label} spec: alerting disabled, measurement intact)")


def test_pulse_check_cli_verdicts():
    """pulse-check: exit 1 + WARN on stdout for violations, 0 for a pass, 2 for bad input.
    Offline mode owns the whole picture: a topic in NO log IS a violation."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "run.log")
        # produce a log WITHOUT in-probe alerting: pulse-check must re-derive from raw rates
        ph.run_single("talker", out, run_s=6.0, period="1.0")
        cli = pulse_check_path()
        passing = _write(d, "pass.yaml", PASSING_SPEC)
        failing = _write(d, "fail.yaml", FAILING_SPEC)
        broken = _write(d, "broken.yaml", "what: ever\n")

        ok = subprocess.run([cli, "--spec", passing, out], capture_output=True, text=True)
        assert ok.returncode == 0, f"expected pass, got {ok.returncode}\n{ok.stdout}{ok.stderr}"
        assert ok.stdout == ""

        bad = subprocess.run([cli, "--spec", failing, out], capture_output=True, text=True)
        assert bad.returncode == 1, f"expected exit 1\n{bad.stdout}{bad.stderr}"
        assert "WARN TOPIC /chatter" in bad.stdout
        assert "WARN TOPIC /ghost hz=0.000000" in bad.stdout  # offline: absent topic = 0 Hz
        assert "WARN NODE /some_other_process_node missing" in bad.stdout

        err = subprocess.run([cli, "--spec", broken, out], capture_output=True, text=True)
        assert err.returncode == 2, f"expected exit 2\n{err.stdout}{err.stderr}"

        nolog = subprocess.run([cli, "--spec", passing, os.path.join(d, "absent.log")],
                               capture_output=True, text=True)
        assert nolog.returncode == 2


def test_pulse_check_skip_last_ignores_exit_window():
    """--skip-last judges the second-to-last window, and refuses a log too short to have one."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "run.log")
        ph.run_single("talker", out, run_s=6.0, period="1.0")
        cli = pulse_check_path()
        passing = _write(d, "pass.yaml", PASSING_SPEC)

        ok = subprocess.run([cli, "--spec", passing, "--skip-last", out],
                            capture_output=True, text=True)
        assert ok.returncode == 0, f"expected pass, got {ok.returncode}\n{ok.stdout}{ok.stderr}"
        assert ok.stdout == ""

        # a single-window log must error, not silently fall back to the ramp-up window
        short = _write(d, "short.log", "# ts_ns=1 window_s=5.000000\n"
                                       "TOPIC /chatter 50.000000\n"
                                       "PUB /chatter inter=50.000000 intra=0.000000\n")
        few = subprocess.run([cli, "--spec", passing, "--skip-last", short],
                             capture_output=True, text=True)
        assert few.returncode == 2, f"expected exit 2\n{few.stdout}{few.stderr}"
        assert "--skip-last" in few.stderr


def test_pulse_check_rejects_non_spec_paths_fast():
    """`--spec /dev/zero` must fail the CI gate it guards, not hang the runner reading it."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "run.log")
        ph.run_single("talker", out, run_s=4.0, period="1.0")
        cli = pulse_check_path()

        for spec in ("/dev/zero", d):
            r = subprocess.run([cli, "--spec", spec, out], capture_output=True, text=True,
                               timeout=30)
            assert r.returncode == 2, f"expected exit 2 for {spec}\n{r.stdout}{r.stderr}"
            assert "not a regular file" in r.stderr, r.stderr


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

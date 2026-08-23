# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# QUIET banner suppression (ROADMAP R6): ROS_TOPIC_STATS_QUIET=1 silences the informational
# `[ros2_pulse] active` stderr banner for deployments that parse the wrapped process's stderr.
#
# The load-bearing invariant here is the THIRD test: QUIET must silence the banner ONLY, never
# the one-shot error diagnostics. 0.2.0 fixed a whole class of bugs where alerting was silently
# disabled (a directory read as a spec parsed to a zero-rule no-op, a UTF-8 BOM made a valid
# spec fail invisibly, see CHANGELOG 0.2.0 "Fixed"); a QUIET flag that also ate the "invalid
# spec" warning would hand operators a supported way to recreate exactly that failure mode.
#
# The banner is emitted by the probe layer (src/probe/interposers.cpp ensureStarted()), not the
# pure core, so this is an integration test by necessity: a real LD_PRELOADed node with stderr
# captured. probe_harness.run_single() discards stderr, so we launch the node ourselves.

import os
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

BANNER = "[ros2_pulse] active"

# Rejected by parseRateSpec ("unknown section", pinned by test/unit/test_rate_spec.cpp
# RejectsMalformedInput), so loadRateSpec() must warn "invalid ROS_TOPIC_STATS_EXPECTED".
MALFORMED_SPEC = "bogus: 1\n"


def _run_capture(extra_env=None, run_s=3.0):
    """Run one probed talker with stderr CAPTURED; return the decoded stderr text.

    Short run: the banner and the spec diagnostics are both emitted at attach time (first
    tracepoint / runtime construction), so we only need the process to start, not to flush
    multiple windows.
    """
    so, node = ph.probe_paths()
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "quiet.log")
        env = ph.make_env(so, out, period="1.0", extra=extra_env)
        p = subprocess.Popen([node, "talker"], env=env,
                             stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        try:
            time.sleep(run_s)
        finally:
            p.terminate()
            try:
                _, err = p.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                _, err = p.communicate()
    return err


def test_banner_present_by_default():
    """Without QUIET the banner must keep printing, it is load-bearing 'is the probe even
    attached?' feedback, and 0.2.0 deliberately extended it with the spec=... field."""
    err = _run_capture()
    assert BANNER in err, f"default run lost the active banner\n--- stderr ---\n{err}"


def test_quiet_suppresses_banner():
    """ROS_TOPIC_STATS_QUIET=1: no banner on stderr (the stats file itself is unaffected)."""
    err = _run_capture(extra_env={"ROS_TOPIC_STATS_QUIET": "1"})
    assert BANNER not in err, f"QUIET=1 did not silence the banner\n--- stderr ---\n{err}"


def test_quiet_is_exact_opt_in():
    """envFlag semantics: only the documented '1' silences; QUIET=true/0 keep the banner.

    Same exact-match contract as every other flag in the probe (ROS_PULSE_EMIT_IDLE,
    ROS_TOPIC_STATS_JITTER), a half-set flag must fail LOUD (banner still there), not
    half-silence.
    """
    for v in ("0", "true", "yes"):
        err = _run_capture(extra_env={"ROS_TOPIC_STATS_QUIET": v})
        assert BANNER in err, (
            f"QUIET={v!r} must NOT silence the banner (only '1' does)\n--- stderr ---\n{err}")


def test_quiet_never_silences_error_diagnostics():
    """QUIET=1 + malformed ROS_TOPIC_STATS_EXPECTED: the 'invalid spec' warning MUST still
    print. Silently disabled alerting was a fixed 0.2.0 bug class; QUIET is an informational-
    banner switch, not a diagnostics switch."""
    with tempfile.TemporaryDirectory() as d:
        spec = os.path.join(d, "bad.yaml")
        with open(spec, "w") as f:
            f.write(MALFORMED_SPEC)
        err = _run_capture(extra_env={"ROS_TOPIC_STATS_QUIET": "1",
                                      "ROS_TOPIC_STATS_EXPECTED": spec})
    assert "invalid ROS_TOPIC_STATS_EXPECTED" in err, (
        f"QUIET=1 swallowed the malformed-spec diagnostic, that silently disables alerting, "
        f"the exact 0.2.0 bug class\n--- stderr ---\n{err}")
    assert BANNER not in err, (
        f"banner leaked through QUIET=1 in the diagnostics run\n--- stderr ---\n{err}")

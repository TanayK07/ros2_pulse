# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# LTTng coexistence integration test (ROADMAP R3, last open item).
#
# The probe's whole compatibility claim with ros2_tracing rests on one line of design: every
# interposer forwards to the real libtracetools function via dlsym(RTLD_NEXT) (see
# src/probe/interposers.cpp). If that forwarding works, a LIVE LTTng session and the probe are
# not rivals, the same tracepoint call feeds BOTH consumers: the probe counts it, then the real
# ros_trace_* body hands it to lttng-ust. Nothing had ever asserted this; a silently dropped
# forward (e.g. a typo'd dlsym name returning nullptr) would pass every other test in this suite
# and only be noticed by the first user who runs `ros2 trace` alongside us.
#
# So: start an LTTng session enabling ros2:* userspace events, run the 50 Hz talker WITH the
# probe preloaded, then assert BOTH sides ate:
#   (a) the probe log reports a plausible /chatter rate (probe still works under a live session);
#   (b) babeltrace2 (or babeltrace) shows >0 ros2:* events in the trace (forwarding delivered).
#
# Skip-vs-fail design (this is where the assertion gets its teeth):
#   - lttng CLI absent, sessiond won't start, no trace viewer  -> SKIP: environment can't test.
#   - libtracetools.so NOT linked against lttng-ust            -> SKIP: the real tracepoint body
#     is a no-op, so 0 events proves nothing about our forwarding. This is stock ros:humble,
#     the bench/run_bakeoff.sh bake-off measured exactly 0 events there, hence this test skips
#     on the humble CI lane and RUNS on jazzy/kilted (Iron+ ships the lttng-ust backend).
#     The ldd check is deliberately preferred over a canary trace run: ldd distinguishes
#     "environment can't test this" from "probe broke forwarding", a canary can't tell a broken
#     sessiond from a no-op tracepoint.
#   - Environment fine but 0 events with the probe preloaded   -> FAIL: forwarding is broken.
# test_unprobed_talker_baseline documents the sanity leg: the SAME session machinery on an
# UN-probed talker also captures events, so a coexistence failure can never be waved off as
# "LTTng just doesn't work here".

import os
import shutil
import subprocess
import sys
import tempfile
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

# The talker mode of test_nodes.cpp publishes /chatter at 50 Hz nominal.
TOPIC = "/chatter"


# ---------------------------------------------------------------------------
# Environment detection (each returns a skip reason string, or None if usable)
# ---------------------------------------------------------------------------

def _find_libtracetools():
    """Locate the distro's libtracetools.so via the ament index (same lookup order the dynamic
    linker effectively sees through the sourced environment)."""
    prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    for prefix in prefixes:
        cand = os.path.join(prefix, "lib", "libtracetools.so")
        if os.path.exists(cand):
            return cand
    return None


def _tracetools_skip_reason():
    lib = _find_libtracetools()
    if lib is None:
        return "libtracetools.so not found on AMENT_PREFIX_PATH"
    # The load-bearing check: is the REAL tracepoint body wired to lttng-ust? On stock humble
    # tracetools is compiled with TRACETOOLS_DISABLED-style no-op bodies (not linked against
    # lttng-ust), so an LTTng session captures nothing no matter what we forward, skipping
    # there is correct and expected. Iron+ links the backend in the stock binaries.
    out = subprocess.run(["ldd", lib], capture_output=True, text=True, check=False)
    if "lttng-ust" not in out.stdout:
        return (f"{lib} is not linked against lttng-ust (stock humble ships no-op "
                "tracepoints; Iron+ required), coexistence untestable here")
    return None


def _viewer():
    """Trace reader: prefer babeltrace2, fall back to babeltrace (v1 reads CTF 1.8 fine)."""
    return shutil.which("babeltrace2") or shutil.which("babeltrace")


def _lttng_skip_reason():
    if shutil.which("lttng") is None:
        return "lttng CLI not installed"
    if _viewer() is None:
        return "no trace viewer installed (need babeltrace2 or babeltrace)"
    return None


def _ensure_sessiond():
    """Make sure a session daemon is reachable; return a skip reason or None.

    `lttng list` auto-spawns a per-user sessiond in most setups, but minimal containers can
    lack that path, try an explicit daemonized spawn (--no-kernel: we only need the userspace
    domain, and skipping the kernel domain avoids module-load noise as root) before giving up.
    """
    if subprocess.run(["lttng", "list"], capture_output=True, check=False).returncode == 0:
        return None
    subprocess.run(["lttng-sessiond", "--daemonize", "--no-kernel"],
                   capture_output=True, check=False)
    for _ in range(10):
        if subprocess.run(["lttng", "list"], capture_output=True,
                          check=False).returncode == 0:
            return None
        time.sleep(0.5)
    return "lttng-sessiond could not be started/reached"


def _skip_unless_testable():
    for reason in (_lttng_skip_reason(), _tracetools_skip_reason()):
        if reason:
            pytest.skip(reason)
    reason = _ensure_sessiond()
    if reason:
        pytest.skip(reason)


# ---------------------------------------------------------------------------
# Session driving (mirrors the leg-D sequence in bench/run_bakeoff.sh)
# ---------------------------------------------------------------------------

class LttngSession:
    """Context manager: create+start an LTTng session capturing ros2:* userspace events."""

    def __init__(self, name, out_dir):
        self.name = name
        self.out_dir = out_dir

    def _run(self, *argv):
        r = subprocess.run(argv, capture_output=True, text=True, check=False)
        assert r.returncode == 0, (
            f"{' '.join(argv)} failed (rc={r.returncode})\n"
            f"stdout: {r.stdout}\nstderr: {r.stderr}")

    def __enter__(self):
        # Session creation was reachability-checked in _ensure_sessiond, so failures from here
        # on are asserted, not skipped: an environment that can create sessions but not enable
        # ros2:* events is broken in a way worth surfacing loudly.
        self._run("lttng", "create", self.name, "--output", self.out_dir)
        self._run("lttng", "enable-event", "-u", "ros2:*")
        self._run("lttng", "start")
        return self

    def __exit__(self, *exc):
        # stop before destroy so buffers are flushed to the CTF output; destroy even on test
        # failure or sessions leak across test runs (lttng session names are daemon-global).
        subprocess.run(["lttng", "stop", self.name], capture_output=True, check=False)
        subprocess.run(["lttng", "destroy", self.name], capture_output=True, check=False)
        return False

    def ros2_event_count(self):
        """Number of ros2:* events babeltrace can read back from the (stopped) session."""
        r = subprocess.run([_viewer(), self.out_dir],
                           capture_output=True, text=True, check=False)
        # babeltrace exits non-zero on an empty/absent stream; that is a legitimate
        # "0 events" outcome for the assertion, not a harness error.
        return sum(1 for line in r.stdout.splitlines() if "ros2:" in line)


def _run_unprobed_talker(run_s):
    """Run the talker WITHOUT the probe (plain env) for run_s seconds, baseline leg."""
    _, node = ph.probe_paths()
    p = subprocess.Popen([node, "talker"], env=dict(os.environ))
    try:
        time.sleep(run_s)
    finally:
        p.terminate()
        p.wait(timeout=5)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_probe_and_live_lttng_session_coexist():
    """BOTH consumers must capture the same run: probe log rates AND LTTng ros2:* events."""
    _skip_unless_testable()
    with tempfile.TemporaryDirectory() as d:
        trace_dir = os.path.join(d, "trace")
        log = os.path.join(d, "probe.log")
        with LttngSession(f"pulse_coexist_{os.getpid()}", trace_dir) as sess:
            text = ph.run_single("talker", log, run_s=6.0, period="1.0")
        # (a) The probe still measured a plausible rate under a live session. 30% tolerance
        # matches the other integration tests (containers/CI schedulers are jittery).
        ph.assert_rate_within(text, TOPIC, ph.KNOWN_RATE_HZ, field="topic",
                              rel_tol=0.30, note="(with live LTTng session)")
        events = sess.ros2_event_count()
        # (b) Forwarding delivered events to lttng-ust. >0 is the claim; a 6 s 50 Hz talker
        # actually produces hundreds of rcl_publish events alone, so 0 here, in an environment
        # the skip-guards proved capable of tracing, means dlsym(RTLD_NEXT) forwarding is
        # broken, which is precisely the regression this test exists to catch.
        assert events > 0, (
            "live LTTng session captured 0 ros2:* events while the probe was preloaded, "
            "interposer forwarding (dlsym RTLD_NEXT) is broken; probe log was:\n" + text)


def test_unprobed_talker_baseline():
    """Sanity leg: the SAME session machinery captures events from an UN-probed talker.

    This is what gives the coexistence assertion teeth: if this passes and the probed run
    captured 0 events, the fault is provably in the probe's forwarding, not in LTTng, the
    sessiond, or the tracetools backend. (And if the environment can't trace at all, both
    tests skip together via the ldd/sessiond guards, skip means untestable, fail means us.)
    """
    _skip_unless_testable()
    with tempfile.TemporaryDirectory() as d:
        trace_dir = os.path.join(d, "trace")
        with LttngSession(f"pulse_baseline_{os.getpid()}", trace_dir) as sess:
            _run_unprobed_talker(run_s=4.0)
        events = sess.ros2_event_count()
        assert events > 0, (
            "un-probed talker produced 0 ros2:* events despite lttng-ust-linked tracetools, "
            "LTTng environment is broken in a way the skip-guards did not catch")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

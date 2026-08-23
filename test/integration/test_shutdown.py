# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Shutdown-path integration tests (KNOWN_ISSUES #9): the probe must (a) write the final partial
# window at process exit instead of silently dropping the tail counts, and (b) never crash the
# host process when straggler tracepoints fire during static destruction.

import os
import subprocess
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]


def test_final_partial_window_flushed_at_exit():
    """KNOWN_ISSUES #9 (deterministic regression): the tail window survives exit.

    With period=1.0 and a ~2.5 s run, the periodic flushes cover the first N whole seconds and
    the remainder is counted but, on the buggy code, never written: the Meyers-singleton
    destructor chain joins the flush thread without a final flush. Fixed code appends one last
    partial window at exit, whose measured window_s is well under the period (issue #8's
    measured denominator keeps its Hz correct).
    """
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "final.log")
        text = ph.run_single("talker", out, run_s=2.5, period="1.0")
        windows = ph.parse_windows(text)
        assert len(windows) >= 2, (
            f"expected periodic window(s) plus a final partial window at exit; tail was "
            f"dropped (KNOWN_ISSUES #9)\n--- log ---\n{text}")
        final = windows[-1]["window_s"]
        assert final < 0.9, (
            f"last window looks periodic (window_s={final:.3f}); the final PARTIAL window was "
            f"dropped at exit (KNOWN_ISSUES #9)\n--- log ---\n{text}")


def test_exit_with_straggler_tracepoints_is_clean():
    """KNOWN_ISSUES #9 (hazard guard): stragglers through teardown must not crash the host.

    exit_storm leaves detached threads hammering the interposers while main() returns; on the
    buggy code that dereferences a destroyed registry (probabilistic crash), after the leaky-
    singleton fix it is safe by construction. A signal death shows up as a negative returncode.
    """
    so, node = ph.probe_paths()
    with tempfile.TemporaryDirectory() as d:
        env = ph.make_env(so, os.path.join(d, "storm.log"), period="1.0")
        for i in range(10):
            p = subprocess.Popen([node, "exit_storm"], env=env,
                                 stderr=subprocess.DEVNULL)
            rc = p.wait(timeout=15)
            assert rc == 0, (
                f"iteration {i}: exit_storm exited rc={rc}, shutdown crash in the probe "
                f"(KNOWN_ISSUES #9)")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

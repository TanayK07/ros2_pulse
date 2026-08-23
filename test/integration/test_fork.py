# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# fork()-without-exec integration test (KNOWN_ISSUES #10): a child forked without exec inherits
# m_started=true but no flush thread, on the buggy code it counts forever and never writes.

import os
import subprocess
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]


def test_forked_child_flushes_its_own_counts():
    """The child's /forked publishes (~200 Hz for ~2.5 s) must reach the output file.

    The fork_pub mode registers /forked, forks, and only the CHILD publishes. On the buggy code
    no window ever carries /forked traffic (the parent's windows show it idle at best, and idle
    topics are suppressed by default per issue #7). The parent propagates the child's exit
    status, so a deadlocked/crashed child fails by rc/timeout rather than hanging silently.
    """
    so, node = ph.probe_paths()
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "fork.log")
        env = ph.make_env(so, out, period="1.0")
        p = subprocess.Popen([node, "fork_pub"], env=env, stderr=subprocess.DEVNULL)
        rc = p.wait(timeout=30)
        assert rc == 0, f"fork_pub rc={rc}, child crashed or deadlocked (KNOWN_ISSUES #10)"

        assert os.path.exists(out), "no output file written at all"
        text = open(out).read()
        windows = ph.parse_windows(text)
        rates = [w["topic"].get("/forked") for w in windows]
        rates = [r for r in rates if r is not None and r > 0.0]
        assert rates, (
            f"child's /forked publishes never flushed (KNOWN_ISSUES #10)\n--- log ---\n{text}")
        # ~200 Hz nominal; generous band, the point is "the child's counts survived", not
        # cadence precision (covered by the accuracy suite).
        assert max(rates) > 50.0, (
            f"/forked rate implausibly low ({max(rates):.1f} Hz)\n--- log ---\n{text}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

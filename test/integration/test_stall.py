# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Stall-visibility integration test (KNOWN_ISSUES #12): when an upstream publisher dies, the
# subscribed topic must read RECV 0.000000 instead of vanishing from the listener's output.

import os
import subprocess
import sys
import tempfile
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]


def test_dead_upstream_reads_zero():
    """Kill the talker mid-run; a later listener window must carry an explicit zero rate.

    On the buggy code /chatter simply disappears from the RECV lines once traffic stops, so a
    rate-checking consumer cannot tell "stalled" from "never existed".
    """
    so, node = ph.probe_paths()
    with tempfile.TemporaryDirectory() as d:
        out_t = os.path.join(d, "talker.log")
        out_l = os.path.join(d, "listener.log")
        pt = subprocess.Popen([node, "talker"], env=ph.make_env(so, out_t, period="1.0"))
        pl = subprocess.Popen([node, "listener"], env=ph.make_env(so, out_l, period="1.0"))
        try:
            time.sleep(3.0)          # healthy traffic
            pt.terminate()           # upstream dies
            pt.wait(timeout=5)
            time.sleep(3.0)          # stall windows accumulate
        finally:
            for p in (pt, pl):
                if p.poll() is None:
                    p.terminate()
                    p.wait(timeout=5)

        text = open(out_l).read() if os.path.exists(out_l) else ""
        windows = ph.parse_windows(text)
        assert windows, f"listener produced no windows\n--- log ---\n{text}"

        # Sanity: the topic was alive at some point.
        alive = [w for w in windows if w["recv"].get("/chatter", (0, 0))[0] > 0]
        assert alive, f"/chatter never delivered, test setup broken\n--- log ---\n{text}"

        # The regression: after the talker dies, /chatter must STILL be present, at zero.
        stalled = [w for w in windows if w["recv"].get("/chatter") == (0.0, 0.0)]
        assert stalled, (
            "no window reports RECV /chatter inter=0.000000 after the upstream died, "
            f"stall is invisible (KNOWN_ISSUES #12)\n--- log ---\n{text}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

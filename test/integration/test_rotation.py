# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Output-rotation integration tests (KNOWN_ISSUES #11): the output file must stay bounded by
# ROS_TOPIC_STATS_MAX_BYTES via single-generation rotation (<path> -> <path>.1); 0 disables.

import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

CAP = 512  # bytes, tiny so a ~6 s run rotates several times
SLACK = 400  # one window block of headroom above the cap


def test_size_cap_rotates():
    """At/over the cap the file rotates to <path>.1 and restarts; disk stays bounded."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "rot.log")
        ph.run_single("intra", out, run_s=6.0, period="1.0",
                      extra_env={"ROS_TOPIC_STATS_MAX_BYTES": str(CAP)})
        assert os.path.exists(out + ".1"), (
            f"no rotated generation produced; file grew unbounded (KNOWN_ISSUES #11); "
            f"size={os.path.getsize(out)}")
        assert os.path.getsize(out) <= CAP + SLACK, (
            f"live file {os.path.getsize(out)}B exceeds cap {CAP}B + slack")
        # Rotated generation holds complete windows: it must parse.
        assert ph.parse_windows(open(out + ".1").read()), "rotated file unparsable"


def test_zero_disables_rotation():
    """MAX_BYTES=0 is the documented escape hatch: pure append, no .1 generation."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "norot.log")
        ph.run_single("intra", out, run_s=4.0, period="1.0",
                      extra_env={"ROS_TOPIC_STATS_MAX_BYTES": "0"})
        assert not os.path.exists(out + ".1")
        assert os.path.getsize(out) > 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

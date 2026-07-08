# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Accuracy tests built on probe_harness. These cover ONLY the cases that are correct on main:
#   * separate-process inter-process receive  (talker + listener, distinct processes)
#   * intra-process receive                    (single process, use_intra_process_comms ON)
# Both drive a known 50 Hz publisher and assert the reported Hz is within tolerance.
#
# The single-process inter-process DOUBLE-COUNT accuracy assertion is intentionally NOT here:
# it fails on main by design (KNOWN_ISSUES #1) and its regression test lives in the fix PR. The
# harness is written so that test drops in as one more assert_rate_within() call.

import os
import sys
import tempfile

import pytest

# probe_harness lives beside this file. Put its dir on the path explicitly so the import works
# regardless of pytest's import mode (and when the file is run directly), not only when pytest
# happens to prepend the test dir.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

# Wall-timer jitter + window quantisation + CI scheduling noise: a 30% band around the nominal
# 50 Hz is tight enough to catch a ~2x double-count or a dropped-transport regression while
# staying reliable on shared runners.
TOL = 0.30


def test_accuracy_intra_process_known_rate():
    """Single process, IPC on: /intra_topic intra rate ~= 50 Hz."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "intra.log")
        text = ph.run_single("intra", out, run_s=6.0, period="1.0")
        got = ph.assert_rate_within(
            text, "/intra_topic", ph.KNOWN_RATE_HZ, field="intra", rel_tol=TOL,
            note="(intra-process delivery)")
        print(f"intra /intra_topic measured {got:.2f} Hz")


def test_accuracy_inter_process_separate_known_rate():
    """Talker + listener in SEPARATE processes: /chatter inter rate ~= 50 Hz on the listener."""
    with tempfile.TemporaryDirectory() as d:
        out_t = os.path.join(d, "talker.log")
        out_l = os.path.join(d, "listener.log")
        _, listener = ph.run_pair(out_t, out_l, run_s=6.0, period="1.0")
        got = ph.assert_rate_within(
            listener, "/chatter", ph.KNOWN_RATE_HZ, field="inter", rel_tol=TOL,
            note="(separate-process inter-process delivery)")
        print(f"inter /chatter measured {got:.2f} Hz")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

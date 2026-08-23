# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Publish-side intra-process integration test (ROADMAP R3). The rclcpp_intra_publish tracepoint
# exists on jazzy and newer only, on humble the probe simply exports an extra never-called
# symbol and intra rates stay receive-side, so this test skips there.

import os
import re
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

DISTRO = os.environ.get("ROS_DISTRO", "")
HAS_INTRA_PUBLISH_TP = DISTRO not in ("", "humble", "iron")

_PUB_RE = re.compile(r"^PUB (\S+) inter=([\d.]+) intra=([\d.]+)", re.M)


@pytest.mark.skipif(not HAS_INTRA_PUBLISH_TP,
                    reason=f"rclcpp_intra_publish tracepoint absent on '{DISTRO}'")
def test_publish_side_intra_rate_reported():
    """The intra node (50 Hz, IPC on) must produce a PUB line with intra ~= 50 Hz."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "pubintra.log")
        text = ph.run_single("intra", out, run_s=6.0, period="1.0")
        rates = [float(m.group(3)) for m in _PUB_RE.finditer(text)
                 if m.group(1) == "/intra_topic"]
        assert rates, (
            "no PUB line for /intra_topic, publish-side intra not counted (ROADMAP R3)"
            f"\n--- log ---\n{text}")
        got = max(rates)
        assert 50.0 * 0.7 <= got <= 50.0 * 1.3, (
            f"PUB intra rate {got:.2f}Hz outside 50Hz ±30%\n--- log ---\n{text}")


@pytest.mark.skipif(HAS_INTRA_PUBLISH_TP, reason="humble-only guard")
def test_no_pub_line_on_humble():
    """On humble the tracepoint doesn't exist: no PUB lines, receive-side intra still works."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "nopub.log")
        text = ph.run_single("intra", out, run_s=4.0, period="1.0")
        assert not _PUB_RE.search(text), f"unexpected PUB line on {DISTRO}\n{text}"
        ph.assert_rate_within(text, "/intra_topic", ph.KNOWN_RATE_HZ, field="intra",
                              rel_tol=0.30, note="(receive-side intra, humble)")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

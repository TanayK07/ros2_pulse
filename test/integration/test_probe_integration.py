# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Integration test: run real ROS nodes under LD_PRELOAD=libros2_pulse.so and assert the
# probe wrote correct per-topic frequencies, including the INTRA-process case that rmw-level
# monitors (and built-in topic statistics) cannot see.

import os
import re
import subprocess
import tempfile
import time

import pytest
from ament_index_python.packages import get_package_prefix

PKG = "ros2_pulse"


def _paths():
    prefix = get_package_prefix(PKG)
    so = os.path.join(prefix, "lib", "libros2_pulse.so")
    node = os.path.join(prefix, "lib", PKG, "stats_probe_test_nodes")
    assert os.path.exists(so), f"probe .so not found: {so}"
    assert os.path.exists(node), f"test nodes not found: {node}"
    return so, node


def _env(so, out_path, period="1.0"):
    env = dict(os.environ)
    env["LD_PRELOAD"] = so
    env["ROS_TOPIC_STATS_OUTPUT_FILE"] = out_path
    env["ROS_TOPIC_STATISTICS_PUBLISH_PERIOD"] = period
    return env


def _parse_recv(text):
    """Return {topic: (peak inter_hz, peak intra_hz)} across all windows.

    Peak, not last: the final partial window at exit (issue #9) legitimately reads ~0, and
    proven receive endpoints now emit explicit zero lines when idle (issue #12), "did the
    topic ever flow at rate" is the health signal these tests assert.
    """
    out = {}
    for m in re.finditer(r"^RECV (\S+) inter=([\d.]+) intra=([\d.]+)", text, re.M):
        topic, inter, intra = m.group(1), float(m.group(2)), float(m.group(3))
        prev_inter, prev_intra = out.get(topic, (0.0, 0.0))
        out[topic] = (max(prev_inter, inter), max(prev_intra, intra))
    return out


def test_intra_process_counted():
    so, node = _paths()
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "stats.log")
        p = subprocess.Popen([node, "intra"], env=_env(so, out))
        try:
            time.sleep(4)
        finally:
            p.terminate()
            p.wait(timeout=5)
        assert os.path.exists(out), "probe produced no output file"
        text = open(out).read()
        recv = _parse_recv(text)
        assert "/intra_topic" in recv, f"/intra_topic missing; got: {text}"
        intra_hz = recv["/intra_topic"][1]
        # 50 Hz publisher, intra-process delivery -> intra rate dominant, inter ~0
        assert intra_hz > 20.0, f"expected intra>20Hz, got {intra_hz}; file:\n{text}"


def test_inter_process_counted():
    so, node = _paths()
    with tempfile.TemporaryDirectory() as d:
        out_t = os.path.join(d, "talker.log")
        out_l = os.path.join(d, "listener.log")
        pt = subprocess.Popen([node, "talker"], env=_env(so, out_t))
        pl = subprocess.Popen([node, "listener"], env=_env(so, out_l))
        try:
            time.sleep(4)
        finally:
            for p in (pt, pl):
                p.terminate()
                p.wait(timeout=5)
        ltext = open(out_l).read() if os.path.exists(out_l) else ""
        recv = _parse_recv(ltext)
        assert "/chatter" in recv, f"/chatter missing on listener side; got: {ltext}"
        inter_hz = recv["/chatter"][0]
        assert inter_hz > 20.0, f"expected inter>20Hz, got {inter_hz}; file:\n{ltext}"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))

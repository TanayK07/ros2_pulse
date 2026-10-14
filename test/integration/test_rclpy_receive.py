# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Take-layer receive (rmw_take): an rclpy subscriber never reaches callback_start, so before
# this hook its receives were invisible. Now every rcl client's successful take is counted, and
# rclcpp subscriptions stay on callback_start so nothing is counted twice.
#  - rclpy listener: RECV total matches the listener's own callback count exactly.
#  - rclcpp listener: RECV rate stays at the talker's rate (not doubled).
#  - rclpy listener with no publisher: no RECV line at all (zero takes, never a proven endpoint).

import os
import re
import subprocess
import sys
import tempfile
import time

import pytest
from ament_index_python.packages import get_package_prefix

PKG = "ros2_pulse"

# A minimal rclpy listener that records how many callbacks it ran, the ground truth the probe's
# take-layer count is compared against.
PY_LISTENER = r"""
import sys, time
import rclpy
from std_msgs.msg import String
topic, dur, count_file = sys.argv[1], float(sys.argv[2]), sys.argv[3]
rclpy.init()
node = rclpy.create_node("py_take_listener")
got = [0]
def cb(_msg):
    got[0] += 1
node.create_subscription(String, topic, cb, 10)
end = time.time() + dur
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
with open(count_file, "w") as f:
    f.write(str(got[0]))
node.destroy_node()
rclpy.shutdown()
"""


def _paths():
    prefix = get_package_prefix(PKG)
    so = os.path.join(prefix, "lib", "libros2_pulse.so")
    node = os.path.join(prefix, "lib", PKG, "stats_probe_test_nodes")
    assert os.path.exists(so), f"probe .so not found: {so}"
    assert os.path.exists(node), f"test nodes not found: {node}"
    return so, node


def _env(so, out_path, domain):
    env = dict(os.environ)
    env["LD_PRELOAD"] = so
    env["ROS_TOPIC_STATS_OUTPUT_FILE"] = out_path
    env["ROS_TOPIC_STATISTICS_PUBLISH_PERIOD"] = "1.0"
    env["ROS_DOMAIN_ID"] = str(domain)
    return env


def _recv_totals(text, topic):
    """Sum count = hz * window_s over every window for topic: (inter, intra, peak_inter_hz)."""
    inter = intra = peak = 0.0
    window_s = None
    for line in text.splitlines():
        m = re.match(r"# ts_ns=\d+ window_s=([\d.]+)", line)
        if m:
            window_s = float(m.group(1))
            continue
        m = re.match(r"RECV (\S+) inter=([\d.]+) intra=([\d.]+)", line)
        if m and m.group(1) == topic and window_s is not None:
            inter += float(m.group(2)) * window_s
            intra += float(m.group(3)) * window_s
            peak = max(peak, float(m.group(2)))
    return inter, intra, peak


def _run_py_listener(so, d, domain, dur, with_talker, node):
    out = os.path.join(d, "py_listener.log")
    count_file = os.path.join(d, "count.txt")
    script = os.path.join(d, "py_listener.py")
    with open(script, "w") as f:
        f.write(PY_LISTENER)
    talker = None
    if with_talker:
        talker = subprocess.Popen([node, "talker"], env=_env(so, os.path.join(d, "t.log"), domain))
    try:
        subprocess.run([sys.executable, script, "/chatter", str(dur), count_file],
                       env=_env(so, out, domain), timeout=dur + 30, check=True)
    finally:
        if talker is not None:
            talker.terminate()
            talker.wait(timeout=5)
    text = open(out).read() if os.path.exists(out) else ""
    got = int(open(count_file).read())
    return text, got


def test_rclpy_receive_counted_exactly():
    so, node = _paths()
    with tempfile.TemporaryDirectory() as d:
        text, got = _run_py_listener(so, d, 71, 5.0, True, node)
        inter, intra, peak = _recv_totals(text, "/chatter")
        assert got > 50, f"rclpy listener received too little to judge ({got}); log:\n{text}"
        # Every window is flushed (the last one at exit), so the take-layer total is the
        # listener's own callback count. Hz is printed with 6 decimals, allow rounding of 1.
        assert abs(inter - got) <= 1.0, f"probe {inter:.1f} vs callbacks {got}; log:\n{text}"
        assert intra == 0.0
        assert peak > 20.0, f"50 Hz talker, peak inter {peak}; log:\n{text}"


def test_rclcpp_receive_not_doubled():
    so, node = _paths()
    with tempfile.TemporaryDirectory() as d:
        out_l = os.path.join(d, "listener.log")
        pt = subprocess.Popen([node, "talker"], env=_env(so, os.path.join(d, "t.log"), 72))
        pl = subprocess.Popen([node, "listener"], env=_env(so, out_l, 72))
        try:
            time.sleep(5)
        finally:
            for p in (pt, pl):
                p.terminate()
                p.wait(timeout=5)
        text = open(out_l).read() if os.path.exists(out_l) else ""
        _, _, peak = _recv_totals(text, "/chatter")
        # 50 Hz talker: a doubled count (take + callback_start) would read ~100 Hz.
        assert 20.0 < peak < 65.0, f"rclcpp listener peak inter {peak}; log:\n{text}"


def test_rclpy_subscription_without_publisher_reports_nothing():
    so, node = _paths()
    with tempfile.TemporaryDirectory() as d:
        text, got = _run_py_listener(so, d, 73, 3.0, False, node)
        assert got == 0
        assert "RECV /chatter" not in text, f"unexpected RECV line; log:\n{text}"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))

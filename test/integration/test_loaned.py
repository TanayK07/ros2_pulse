# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Loaned-message counting, end to end (rclcpp#3153 follow-up). A loan is invisible at the
# tracepoint level, so the probe wraps rcl_publish_loaned_message / rcl_take_loaned_message.
#
#  - LOANS ACTIVE: Fast DDS with data sharing switched on through an XML profile
#    (rmw_fastrtps forces data_sharing OFF unless RMW_FASTRTPS_USE_QOS_FROM_XML=1) and a
#    fixed-size type. The talker's log must carry the publish total AND a LOAN pub line at the
#    same rate; the listener's a LOAN recv line. On Humble the total is the point: before the
#    wrappers, a loaned publisher had no TOPIC line at all (no tracepoint fires).
#  - FALLBACK: the same binaries and the same borrow/publish code without the profile. rclcpp
#    allocates locally and publishes through plain rcl_publish, so not one LOAN line may appear.
#  - RTLD_LOCAL: a plugin linking librcl, dlopen()ed RTLD_LOCAL by a host that does not. The
#    plugin's call binds to the preloaded wrapper, but dlsym(RTLD_NEXT) cannot see librcl; the
#    wrapper must still reach the real function instead of failing the call.

import os
import re
import subprocess
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

TOPIC = "/chatter_loan"
_LOAN_RE = re.compile(r"^LOAN (\S+) (pub|recv) hz=([\d.]+)$", re.M)

XML = """<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <data_writer profile_name="default_datawriter" is_default_profile="true">
    <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    <qos><data_sharing><kind>AUTOMATIC</kind></data_sharing></qos>
  </data_writer>
  <data_reader profile_name="default_datareader" is_default_profile="true">
    <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    <qos><data_sharing><kind>AUTOMATIC</kind></data_sharing></qos>
  </data_reader>
</profiles>
"""


def _have_fastdds():
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix("rmw_fastrtps_cpp")
        return True
    except Exception:
        return False


needs_fastdds = pytest.mark.skipif(not _have_fastdds(), reason="rmw_fastrtps_cpp not installed")


def _loan_rates(text, side):
    return [float(m.group(3)) for m in _LOAN_RE.finditer(text)
            if m.group(1) == TOPIC and m.group(2) == side]


def _run(d, extra):
    env = {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS_DISABLE_LOANED_MESSAGES": "0"}
    # The fallback leg must not inherit a profile from the caller's environment.
    for k in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
              "RMW_FASTRTPS_USE_QOS_FROM_XML"):
        os.environ.pop(k, None)
    env.update(extra)
    return ph.run_pair(os.path.join(d, "talker.log"), os.path.join(d, "listener.log"),
                       run_s=6.0, period="1.0", extra_env=env,
                       talker_mode="loan_talker", listener_mode="loan_listener")


@needs_fastdds
def test_loans_counted_with_data_sharing():
    with tempfile.TemporaryDirectory() as d:
        xml = os.path.join(d, "datasharing.xml")
        with open(xml, "w") as f:
            f.write(XML)
        # FASTRTPS_ for Fast DDS 2.x (humble, jazzy), FASTDDS_ for 3.x (kilted, rolling).
        talker, listener = _run(d, {"FASTRTPS_DEFAULT_PROFILES_FILE": xml,
                                    "FASTDDS_DEFAULT_PROFILES_FILE": xml,
                                    "RMW_FASTRTPS_USE_QOS_FROM_XML": "1"})
        # Total: on Humble this line only exists because the wrapper counts the loaned publish.
        ph.assert_rate_within(talker, TOPIC, ph.KNOWN_RATE_HZ, field="topic", rel_tol=0.30,
                              note="(loaned publish total)")
        pub = _loan_rates(talker, "pub")
        assert pub and max(pub) >= ph.KNOWN_RATE_HZ * 0.7, (
            f"no LOAN pub line near {ph.KNOWN_RATE_HZ}Hz\n--- talker ---\n{talker}")
        # Subset never exceeds the total within a window (no double count on any distro).
        for w_text in talker.split("\n\n"):
            tot = re.search(rf"^TOPIC {re.escape(TOPIC)} ([\d.]+)$", w_text, re.M)
            sub = re.search(rf"^LOAN {re.escape(TOPIC)} pub hz=([\d.]+)$", w_text, re.M)
            if sub:
                assert tot and float(sub.group(1)) <= float(tot.group(1)) + 1e-6, w_text
        recv = _loan_rates(listener, "recv")
        assert recv and max(recv) >= ph.KNOWN_RATE_HZ * 0.7, (
            f"no LOAN recv line near {ph.KNOWN_RATE_HZ}Hz\n--- listener ---\n{listener}")


@needs_fastdds
def test_fallback_counts_zero_loans():
    # No data-sharing XML is not enough on Jazzy+: Fast DDS loans any plain type by default
    # there (can_loan_messages = is_plain()). ROS_DISABLE_LOANED_MESSAGES=1 makes rcl refuse the
    # loan on every distro, so borrow_loaned_message() takes rclcpp's local-allocation fallback.
    with tempfile.TemporaryDirectory() as d:
        talker, listener = _run(d, {"ROS_DISABLE_LOANED_MESSAGES": "1"})
        ph.assert_rate_within(talker, TOPIC, ph.KNOWN_RATE_HZ, field="topic", rel_tol=0.30,
                              note="(fallback publish through plain rcl_publish)")
        assert not _LOAN_RE.search(talker), f"LOAN line without a loan\n{talker}"
        assert not _LOAN_RE.search(listener), f"LOAN line without a loan\n{listener}"


def test_wrapper_forwards_when_librcl_is_rtld_local():
    so, node = ph.probe_paths()
    libdir = os.path.dirname(node)
    host = os.path.join(libdir, "stats_probe_rtld_local_host")
    plugin = os.path.join(libdir, "libstats_probe_rtld_local_plugin.so")
    with tempfile.TemporaryDirectory() as d:
        env = ph.make_env(so, os.path.join(d, "probe.log"))
        env["ROS_TOPIC_STATS_QUIET"] = "1"
        r = subprocess.run([host, plugin], env=env, capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    # 300 = RCL_RET_PUBLISHER_INVALID, the real rcl's answer for a zero-initialized publisher.
    # 1 (RCL_RET_ERROR) would mean the wrapper failed to resolve librcl and broke the call.
    assert r.stdout.strip() == "ret=300", f"stdout={r.stdout!r} stderr={r.stderr!r}"
    assert "cannot resolve" not in r.stderr, r.stderr


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v", "-s"]))

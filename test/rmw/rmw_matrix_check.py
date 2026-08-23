# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# One leg of the RMW support matrix (ROADMAP R6): under the CURRENT environment's
# RMW_IMPLEMENTATION, run the known-rate integration nodes with the probe preloaded and assert
# the probe reports the nominal 50 Hz on every path the leg exercises:
#
#   inter : talker + listener as separate processes
#           -> talker log   TOPIC /chatter ~= 50 Hz   (publish-side, via rcl_publish)
#           -> listener log RECV  /chatter inter ~= 50 Hz (receive-side, via callback_start)
#   intra : one process, use_intra_process_comms(true)
#           -> RECV /intra_topic intra ~= 50 Hz (transport never touches the RMW at all, but
#              the process still brings the RMW up, so a hostile RMW could break the probe)
#   --fixed adds the same inter pair on /chatter_fixed with std_msgs/UInt64, the fixed-size
#   (self-contained) type the CycloneDDS+iceoryx leg needs: String is not SHM-eligible, so
#   without this the SHM leg would quietly measure loopback UDP and prove nothing.
#
# Parsing/thresholds deliberately reuse test/integration/probe_harness.py, the same code path
# CI trusts on rmw_fastrtps, so a red here is an RMW finding, not a second parser's bug.
# Runs INSIDE the container that test/rmw/run_rmw_matrix.sh prepares; needs a sourced overlay.
#
# Exit: 0 all asserted rates in band, 1 assertion failure, 2 setup error.

import argparse
import os
import subprocess
import sys
import time

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "integration"))
import probe_harness as ph  # noqa: E402

# Same band as test/integration/test_accuracy.py: wide enough for wall-timer jitter + container
# scheduling noise, tight enough to catch a double-count (~2x) or a dropped transport (0).
TOL = 0.30


def _spawn(mode, out_log, capture_dir, tag):
    """Start one probed node; stdout+stderr go to <capture_dir>/<tag>.stderr.txt as evidence
    (the CycloneDDS/iceoryx and zenoh session banners are part of the proof of which transport
    ran)."""
    so, node = ph.probe_paths()
    err = open(os.path.join(capture_dir, f"{tag}.stderr.txt"), "w")
    # PULSE_ROLE is only consumed by the SHM leg's CycloneDDS config (${PULSE_ROLE} expansion
    # routes each process's cyclone trace to its own file); inert everywhere else.
    env = ph.make_env(so, out_log, period="1.0", extra={"PULSE_ROLE": tag})
    return subprocess.Popen([node, mode], env=env, stdout=err, stderr=err), err


def run_pair_modes(mode_t, mode_l, capture_dir, tag_prefix, run_s):
    out_t = os.path.join(capture_dir, f"{tag_prefix}talker.pulse.log")
    out_l = os.path.join(capture_dir, f"{tag_prefix}listener.pulse.log")
    pt, ft = _spawn(mode_t, out_t, capture_dir, f"{tag_prefix}talker")
    pl, fl = _spawn(mode_l, out_l, capture_dir, f"{tag_prefix}listener")
    try:
        time.sleep(run_s)
    finally:
        for p in (pt, pl):
            p.terminate()
        for p in (pt, pl):
            p.wait(timeout=10)
        ft.close()
        fl.close()
    tt = open(out_t).read() if os.path.exists(out_t) else ""
    lt = open(out_l).read() if os.path.exists(out_l) else ""
    return tt, lt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--leg", required=True, help="label for output lines (e.g. zenoh)")
    ap.add_argument("--out", required=True, help="dir for pulse logs + captured stderr")
    ap.add_argument("--fixed", action="store_true",
                    help="also run the fixed-size UInt64 pair (SHM-eligible type)")
    ap.add_argument("--run-s", type=float, default=8.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rmw = os.environ.get("RMW_IMPLEMENTATION", "(default)")
    print(f"[{args.leg}] RMW_IMPLEMENTATION={rmw}", flush=True)
    failures = []

    def check(text, topic, field, what):
        try:
            got = ph.assert_rate_within(text, topic, ph.KNOWN_RATE_HZ, field=field,
                                        rel_tol=TOL, note=f"({args.leg}: {what})")
            print(f"[{args.leg}] PASS {what}: {topic} {field}={got:.3f} Hz "
                  f"(expect {ph.KNOWN_RATE_HZ} +/-{TOL * 100:.0f}%)", flush=True)
        except AssertionError as e:
            failures.append(f"{what}: {e}")
            print(f"[{args.leg}] FAIL {what}: {e}", flush=True)

    # --- inter-process (String pair, the standard integration workload) ---
    tt, lt = run_pair_modes("talker", "listener", args.out, "", args.run_s)
    check(tt, "/chatter", "topic", "inter publish-side (talker TOPIC)")
    check(lt, "/chatter", "inter", "inter receive-side (listener RECV)")

    # --- fixed-size inter pair (SHM-eligible) ---
    if args.fixed:
        tt, lt = run_pair_modes("talker_fixed", "listener_fixed", args.out, "fixed_",
                                args.run_s)
        check(tt, "/chatter_fixed", "topic", "fixed-size inter publish-side")
        check(lt, "/chatter_fixed", "inter", "fixed-size inter receive-side")

    # --- intra-process ---
    out_i = os.path.join(args.out, "intra.pulse.log")
    pi, fi = _spawn("intra", out_i, args.out, "intra")
    time.sleep(args.run_s)
    pi.terminate()
    pi.wait(timeout=10)
    fi.close()
    it = open(out_i).read() if os.path.exists(out_i) else ""
    check(it, "/intra_topic", "intra", "intra-process receive-side (RECV intra)")

    if failures:
        print(f"[{args.leg}] {len(failures)} failure(s)", flush=True)
        return 1
    print(f"[{args.leg}] all rates in band", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

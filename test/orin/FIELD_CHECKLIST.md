# Orin Measurement Run

Field checklist for validating `ros2_pulse` v0.3.0 on the robot. Everything here feeds the public docs site verbatim — capture **files, not screenshots**. Total hands-on time **~1.5–2 h**, most of it waiting on timers. The one result that gates going public is **Phase 1.5** (~10 min).

Full rationale lives in [`ORIN_RUNBOOK.md`](ORIN_RUNBOOK.md) (same directory, pinned to the `v0.3.0` tag on the robot); this is the do-it version.

---

## Before anything — pin the power state (host, not container)

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks
sudo nvpmodel -q          # save this output — paste into platform.txt later
```

Unpinned Tegra clocks swing benchmarks 10–30%. Skip this and every number is noise.

---

## Phase 0 — get the code there, then build (15 min)

The repo is PRIVATE and the robot should never hold credentials for it. Ship a **git bundle** from your machine instead — self-contained, no token, no remote, and one file to delete afterwards:

```bash
# on your machine (repo checkout):
git bundle create /tmp/ros2_pulse.bundle v0.3.0
scp /tmp/ros2_pulse.bundle orin:/tmp/

# on the Orin host:
docker cp /tmp/ros2_pulse.bundle ros2_dev:/tmp/
docker exec -it ros2_dev bash
# inside the container:
git clone /tmp/ros2_pulse.bundle /root/ros2_ws/src/ros2_pulse
cd /root/ros2_ws/src/ros2_pulse && git checkout v0.3.0
```

Build:

```bash
cd /root/ros2_ws
colcon build --packages-select ros2_pulse
source install/setup.bash
```

Also store the clock state so teardown can restore it exactly:

```bash
# on the Orin host, BEFORE nvpmodel -m 0:
sudo nvpmodel -q | tee /tmp/nvpmodel_before.txt     # note the original mode number
sudo jetson_clocks --store /tmp/jetson_clocks_before.txt
```

---

## Phase 1 — does it work here (15 min)

```bash
# 1a. Image instrumented?  MUST be > 0 — if 0, STOP and report (blocker finding)
TT=$(find /opt/ros -name 'libtracetools.so*' | head -1)
nm -D "$TT" | grep -c ros_trace

# 1b. LTTng backend linked?  Either answer is DATA, note it
ldd "$TT" | grep -i lttng || echo "not linked -> LTTng bake-off leg will read 0 events (expected)"

# 1c. Quick controlled graph under the probe
export LD_PRELOAD=$(find /root/ros2_ws/install -name libros2_pulse.so | head -1)
export ROS_TOPIC_STATS_OUTPUT_FILE=/tmp/orin_probe.log
export ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=2.0
ros2 run demo_nodes_cpp talker & ros2 run demo_nodes_cpp listener &
sleep 8; kill %1 %2
cat /tmp/orin_probe.log      # expect TOPIC /chatter ~2.0 + RECV + NODE lines

# 1d. v0.3.0 feature spot-checks (~30 s)
export ROS_TOPIC_STATS_OUTPUT_FILE=/tmp/orin_v030.log
ROS_TOPIC_STATS_JITTER=1  timeout 10 ros2 run demo_nodes_cpp talker & timeout 10 ros2 run demo_nodes_cpp listener; wait
grep JITTER /tmp/orin_v030.log     # expect: JITTER /chatter ... max_dt_ms=~1000-ish
ROS_TOPIC_STATS_FORMAT=jsonl timeout 10 ros2 run demo_nodes_cpp talker & timeout 10 ros2 run demo_nodes_cpp listener; wait
tail -1 /tmp/orin_v030.log | python3 -m json.tool >/dev/null && echo "jsonl parses"
ROS_TOPIC_STATS_QUIET=1 timeout 5 ros2 run demo_nodes_cpp talker 2>&1 | grep -c ros2_pulse   # expect: 0
```

> **Path change since the deploy env was written:** the _default_ output is now `$TMPDIR/topic_freq.<pid>.log` (one file per process). `/root/ssd2tb/logs/topic_freq.log` only fills if `ros2_env_common.conf` still sets `ROS_TOPIC_STATS_OUTPUT_FILE` explicitly — check it; if it relied on the old default, the log moved to `/tmp`.

**Expected:** 1a > 0 · 1c shows TOPIC/RECV/NODE · 1d all three pass. **Watch for on the real stack:** `intra=<nonzero>` on composable / point-cloud topics — that is the data no graph-side tool can produce.

---

## Phase 1.5 — the headline measurement (~10 min) — GATES GOING PUBLIC

```bash
test/orin/run_hotpath_orin.sh        # 10 trials x 8 threads, self-verdicting
```

| Verdict printed | Meaning | What to do |
|---|---|---|
| clock ~20–40 ns → "vDSO path works" | +24 ns claim transfers to Tegra | Publish Orin row as measured |
| clock ≥150 ns → "SYSCALL FALLBACK" | **Also a valid result** — the finding IS that vDSO falls back | Measured R5 number becomes the Orin-specific row, with the vDSO explanation |
| clock 60–150 ns, or trial swings >10% | Governor/thermal problem, not silicon | Re-pin (nvpmodel + jetson_clocks), rerun |

Artifacts land in `test/orin/out/hotpath/` — **commit the whole directory**.

---

## Phase 2 — real-stack CPU overhead (2 × 60 s + one restart)

```bash
# probe ON (LD_PRELOAD wired in deploy env):
test/orin/run_orin_probe_test.sh 60 perception
mv /tmp/orin_probe_report test/orin/out/report_on

# comment LD_PRELOAD out of the env conf, restart stack, rerun identical:
test/orin/run_orin_probe_test.sh 60 perception
mv /tmp/orin_probe_report test/orin/out/report_off
```

**Expected:** per-node CPU delta ≈ 0% (matches x86) · socket-count delta **exactly 0**.

---

## Phase 3 — iceoryx SHM on/off (2 restarts)

```bash
export CYCLONEDDS_URI=<path>/cyclonedds_default_shm.xml     # restart stack
# capture a settled window:
tail -40 <topic_freq log> > test/orin/out/stack_window_shm.log
# repeat with cyclonedds_default_no_shm.xml:
tail -40 <topic_freq log> > test/orin/out/stack_window.log
```

**Expected:** rates unchanged, SHM topics still counted.

---

## Phase 4 — eBPF / LTTng bake-off (20 min)

```bash
docker run --rm --privileged \
  -v /root/ros2_ws/src/ros2_pulse:/pkg \
  -v /tmp/bench:/work \
  <your-ros-image> bash /pkg/bench/run_bakeoff.sh | tee test/orin/out/bakeoff.txt
```

**Expected on Jetson:** eBPF leg may print `n/a` (kernel lacks uprobe/BTF) — **that IS the result**, it's the portability argument. LTTng = 0 events unless 1b said linked. Ours ≈ 0%.

---

## Hand-back — 6 artifacts, commit on a branch, open a PR

| # | File(s) under `test/orin/out/` | From | Proves |
|---|---|---|---|
| 1 | `instrumentation.txt` (paste 1a/1b output) | Phase 1 | probe hooks exist; lttng link status |
| 2 | `hotpath/` (platform, summary.csv/txt, trial_*.txt) | Phase 1.5 | +24 ns on Tegra, vDSO verdict |
| 3 | `stack_window.log`, `stack_window_shm.log` | Phases 1/3 | real topics incl. intra; SHM behavior |
| 4 | `report_on/`, `report_off/` | Phase 2 | CPU delta, zero sockets |
| 5 | `bakeoff.txt` | Phase 4 | eBPF/LTTng portability vs ours |
| 6 | `v030_features.txt` (paste 1d output) | Phase 1 | JITTER / jsonl / QUIET on target |

Redact topic names if the stack's graph is sensitive — rates and counts are what the docs need.

**Get the results OFF the robot first** — the robot has no git credentials, so commit from your machine, not from there:

```bash
# on your machine:
ssh orin 'docker exec ros2_dev tar -C /root/ros2_ws/src/ros2_pulse/test/orin -cf - out' > /tmp/orin_out.tar
tar -tf /tmp/orin_out.tar | head          # sanity: lists out/hotpath/... before you delete anything on the robot
# (or: docker cp out of the container on the host, then scp)
tar -xf /tmp/orin_out.tar -C ~/repos/ros2_pulse/test/orin/
cd ~/repos/ros2_pulse && git checkout -b orin/results && git add test/orin/out && git commit && git push
```

Once the PR is up, the numbers get folded into README + the docs-site benchmarks page, each traced to its committed raw file.

---

## Teardown — remove all trace (10 min, AFTER results are confirmed off the robot)

Order matters: verify the tarball landed on your machine and extracts cleanly **before** deleting anything.

```bash
# inside the container — remove source, build, install artifacts:
rm -rf /root/ros2_ws/src/ros2_pulse
rm -rf /root/ros2_ws/build/ros2_pulse /root/ros2_ws/install/ros2_pulse
rm -rf /root/ros2_ws/log/*ros2_pulse* 2>/dev/null

# probe output + bench leftovers:
rm -f /tmp/orin_probe.log /tmp/orin_v030.log /tmp/topic_freq.*.log
rm -f /tmp/hotpath_bench /tmp/ros2_pulse.bundle
rm -rf /tmp/orin_probe_report /tmp/bench

# on the Orin host:
rm -f /tmp/ros2_pulse.bundle
```

Restore the stack to exactly its pre-run state:

```bash
# 1. env conf: re-check the LD_PRELOAD line in ros2_env_common.conf matches what it was
#    before Phase 2's A/B toggle, and CYCLONEDDS_URI matches pre-Phase-3.
# 2. restart the stack once so nothing is running with a preload path that no longer exists.
# 3. clocks back to original:
sudo jetson_clocks --restore /tmp/jetson_clocks_before.txt
sudo nvpmodel -m <original mode from /tmp/nvpmodel_before.txt>
rm -f /tmp/nvpmodel_before.txt /tmp/jetson_clocks_before.txt
```

Verify clean:

```bash
docker exec ros2_dev bash -lc 'find / -name "*ros2_pulse*" 2>/dev/null; ls /tmp/topic_freq.* 2>/dev/null'
# expect: no output
```

Step 2 of the restore matters most: a stack env still pointing `LD_PRELOAD` at a deleted `.so` makes every ROS process print a loader warning on start — that IS a trace, and it's noisy. Confirm one node starts silently after the restart.

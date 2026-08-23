# On-Orin test runbook for ros2_pulse

Goal: on the real robot, reproduce the same data we measured locally: per-topic Hz (inter +
intra), CPU overhead (probe ON vs OFF), network cost, SHM on/off, and the bake-off vs eBPF /
LTTng. Run everything **inside the ROS container on the Orin**. Adjust `ros2_dev` /
`/root/ros2_ws` to your actual container name / workspace path.

Nothing here modifies the running stack except toggling one env var + restarting (Phase 2/3).

**Results of the 2026-08-23 run on a production AGX Orin: [`RESULTS.md`](RESULTS.md)**, raw
files under [`out/`](out/).

---

## Phase 0: build the package on the Orin

```bash
# The repo is private and the robot must not end up holding credentials for it. Two ways in,
# neither leaves a .git/ or a remote on the robot:

# (a) no token at all: archive from your checkout, copy over ssh:
git archive --format=tar.gz --prefix=ros2_pulse/ -o /tmp/ros2_pulse-v0.3.0.tar.gz v0.3.0
scp /tmp/ros2_pulse-v0.3.0.tar.gz <orin>:/tmp/
#   on the Orin host:  docker cp /tmp/ros2_pulse-v0.3.0.tar.gz ros2_dev:/tmp/
#   in the container:  tar -xzf /tmp/ros2_pulse-v0.3.0.tar.gz -C /root/ros2_ws/src/

# (b) robot has egress: short-lived read-only token, pasted without echo or history:
#   in the container:
#   set +o history; read -rs PAT
#   mkdir -p /root/ros2_ws/src/ros2_pulse
#   curl -fsSL -H "Authorization: Bearer $PAT" \
#     https://api.github.com/repos/TanayK07/ros2_pulse/tarball/v0.3.0 \
#     | tar -xz --strip-components=1 -C /root/ros2_ws/src/ros2_pulse
#   unset PAT; set -o history        # and revoke the token on GitHub when the run is over

# then, inside the container:
cd /root/ros2_ws
colcon build --packages-select ros2_pulse
source install/setup.bash
```

Teardown afterwards: remove `src/ros2_pulse`, `build/ros2_pulse`, `install/ros2_pulse` (on the
host side if the workspace is a bind mount), the probe's output files, and any env-file lines
added for Phase 2, then restart the stack once so no process still references the deleted
`.so`. Revoke the token if (b) was used.

## Phase 1: does it work here, and what data do we get? (the core ask)

```bash
# 1a. Confirm the Orin image is instrumented (must be non-zero):
TT=$(find /opt/ros -name 'libtracetools.so*' | head -1)
nm -D "$TT" | grep -c ros_trace          # expect >0 (rclcpp calls these; our probe hooks them)

# 1b. Check the LTTng-backend question that captured 0 events locally:
ldd "$TT" | grep -i lttng || echo "NOT linked to lttng-ust -> ros2_tracing needs a rebuild here too"

# 1c. Run a quick controlled graph under the probe (no need to touch the real stack yet):
export LD_PRELOAD=$(find /root/ros2_ws/install -name libros2_pulse.so | head -1)
export ROS_TOPIC_STATS_OUTPUT_FILE=/tmp/orin_probe.log
export ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=2.0
# inter-process (2 procs):
ros2 run demo_nodes_cpp talker & ros2 run demo_nodes_cpp listener &
sleep 8; kill %1 %2
cat /tmp/orin_probe.log        # expect TOPIC /chatter <hz> + RECV ... + NODE lines
```

To see **the real stack's** topics, just launch the stack normally; `LD_PRELOAD` is already wired
into the deploy env (`docker_v2/config/ros2_env_common.conf`), so `topic_freq.log`
(`/root/ssd2tb/logs/topic_freq.log`) fills with every real topic's inter/intra Hz + active nodes.
Watch it live:

```bash
tail -f /root/ssd2tb/logs/topic_freq.log
```

> **v0.3.0 path change:** the *default* output path is now `$TMPDIR/topic_freq.<pid>.log`
> (`/tmp` fallback), `/root/ssd2tb/logs/...` only fills if the deploy env still sets
> `ROS_TOPIC_STATS_OUTPUT_FILE` explicitly. Check the env conf; if it relies on the old
> default, the log moved to `/tmp/topic_freq.<pid>.log` (one file per process).

Look specifically for **`intra=<nonzero>`** on composable/point-cloud topics, that's the data no
rmw-level tool or built-in stat can give you.

```bash
# 1d. v0.3.0 feature spot-checks against the demo pair from 1c (30 s total):
export ROS_TOPIC_STATS_OUTPUT_FILE=/tmp/orin_v030.log
ROS_TOPIC_STATS_JITTER=1  timeout 10 ros2 run demo_nodes_cpp talker & timeout 10 ros2 run demo_nodes_cpp listener; wait
grep JITTER /tmp/orin_v030.log            # expect: JITTER /chatter pub|recv max_dt_ms=~1000±jitter
ROS_TOPIC_STATS_FORMAT=jsonl timeout 10 ros2 run demo_nodes_cpp talker & timeout 10 ros2 run demo_nodes_cpp listener; wait
tail -1 /tmp/orin_v030.log | python3 -m json.tool >/dev/null && echo "jsonl parses"
ROS_TOPIC_STATS_QUIET=1 timeout 5 ros2 run demo_nodes_cpp talker 2>&1 | grep -c ros2_pulse   # expect: 0
```

## Phase 1.5: hot-path microbench: the +24 ns / Tegra clock question (REQUIRED before public)

Every published perf number is x86-64. The one most at risk on Tegra: `ROS_TOPIC_STATS_JITTER=1`
costs **+24 ns/msg**, ~96% of which is one `CLOCK_MONOTONIC` read through the vDSO. Some Tegra
kernels route that read through a syscall (~200 ns) instead. One script answers it:

```bash
# on the HOST first: pin the power state or the numbers are noise:
sudo nvpmodel -m 0 && sudo jetson_clocks
# then inside the container, from the repo:
test/orin/run_hotpath_orin.sh          # 10 trials x 8 threads, ~5-8 min on Orin
```

**Expected** (vDSO works): `raw_clock_1thread_ns` ~20-40, `R5_fixed_CPU_ns/msg` ~+20-40, script
prints `vDSO path works`. The x86 story holds; the Orin row goes in the docs as measured.

**Also a valid result** (syscall fallback): `raw_clock_1thread_ns` >=150, R5 cost ~+150-250.
That is not a failure; it is *the* finding: the README R5 cost table gains an Orin-specific
row with the measured number and the vDSO-fallback explanation. Do not average it with x86.

**Not acceptable**: in-between clock numbers (~60-150 ns) or trial-to-trial swings > ~10%,
that is an unpinned governor / thermal throttle, not silicon. Re-pin (nvpmodel + jetson_clocks),
re-run. Verdict logic is in the script; artifacts land in `test/orin/out/hotpath/`, **commit
that directory**, the docs site consumes it.

## Phase 2: CPU + network overhead (probe ON vs OFF)

```bash
# with the real stack running under the probe (LD_PRELOAD set):
graph-monitor/ros2_pulse/test/orin/run_orin_probe_test.sh 60 perception
#   -> per-node CPU over 60s, socket-count delta (probe opens none), captured topic data

# then A/B: comment the LD_PRELOAD line in the env conf, restart the stack, rerun the same:
graph-monitor/ros2_pulse/test/orin/run_orin_probe_test.sh 60 perception
# diff the per-node cpu_s between the two runs = the probe's real overhead on Orin.
```

## Phase 3: iceoryx SHM on vs off

```bash
# point CYCLONEDDS_URI at the SHM profile, restart stack, confirm loaned/zero-copy receives count:
export CYCLONEDDS_URI=/root/ros2_ws/src/10xCode/setup/deployment/docker_v2/planner_computer/config/cyclonedds_default_shm.xml
# ... restart stack, tail topic_freq.log, confirm intra/inter counts on SHM topics ...
# then repeat with cyclonedds_default_no_shm.xml and compare.
```

## Phase 4: bake-off vs eBPF + LTTng, on Orin hardware

This is the important portability test: **does eBPF even work on the Jetson kernel?**

```bash
# needs a privileged container on the Orin (eBPF: debugfs + CAP_SYS_ADMIN + BTF):
docker run --rm --privileged \
  -v /root/ros2_ws/src/10xCode/graph-monitor/ros2_pulse:/pkg \
  -v /tmp/bench:/work \
  <your-ros-image> bash /pkg/bench/run_bakeoff.sh
```

Expected Orin-specific outcomes to capture:
- **eBPF leg** may report `n/a` if the Jetson kernel lacks uprobe/BTF, that itself is the result
  (eBPF not portable to the fleet).
- **LTTng leg** = 0 events unless the Orin image links lttng-ust (Phase 1b tells you).
- **ours** should match the local ~0% overhead.

## What to send back

Everything below feeds the public docs site verbatim, so capture files, not screenshots:

| # | Artifact | From | Proves |
|---|----------|------|--------|
| 1 | Phase 1a/1b terminal output (paste into `test/orin/out/instrumentation.txt`) | Phase 1 | probe hooks exist on the Orin image; lttng-ust link status |
| 2 | `test/orin/out/hotpath/` (platform.txt, summary.csv, summary.txt, trial_*.txt) | Phase 1.5 | the +24 ns claim on Tegra, vDSO verdict |
| 3 | Two `topic_freq` windows: real stack + SHM-on (`test/orin/out/stack_window.log`, `stack_window_shm.log`) | Phases 1/3 | real topics incl. `intra=` nonzero; SHM behavior |
| 4 | Both `run_orin_probe_test.sh` reports, probe ON and OFF (`test/orin/out/report_on/`, `report_off/`) | Phase 2 | per-node CPU delta, zero sockets opened |
| 5 | Phase 4 bake-off table (`test/orin/out/bakeoff.txt`) | Phase 4 | eBPF/LTTng portability vs ours on Jetson kernel |
| 6 | Spot-check outputs from 1d (`test/orin/out/v030_features.txt`) | Phase 1 | JITTER/jsonl/QUIET work on the target |

Commit `test/orin/out/**` on a branch and open a PR, or paste raw; either way the numbers get
turned into the on-Orin RESULTS section + docs-site page. Redact topic names if the stack's
graph is sensitive; rates and node counts are what the docs need.

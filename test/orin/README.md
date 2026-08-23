# On-Orin field test for `ros2_pulse`

Non-destructive test to run on the real robot. Confirms the probe loads against the Orin's
tracetools build, counts real topics (including intra-process), and measures CPU + network cost.

## Prerequisites

```bash
# In the ROS container on the Orin, build the package:
colcon build --packages-select ros2_pulse
source install/setup.bash
```

`LD_PRELOAD=libros2_pulse.so` is already wired into the deploy env configs
(`docker_v2/config/ros2_env_common.conf`, `services/ros2_env.conf`, `docker_up/docker-compose.yml`).

## Run

```bash
# 60s window; optionally filter CPU sampling to a node-name substring (e.g. perception)
./run_orin_probe_test.sh 60 perception
```

## What it reports

1. **Environment**: RMW, CYCLONEDDS_URI, whether SHM is enabled, LD_PRELOAD.
2. **Instrumentation present**: `ros_trace_*` symbol count in the Orin's `libtracetools.so`
   (must be non-zero; if zero, the image was built without tracing and the probe can't hook).
3. **CPU**: per-PID `utime+stime` delta over the window → cpu_s and %-of-core, top consumers.
4. **Network**: unix/udp socket counts before/after. The probe opens **no** sockets and **no**
   DDS endpoints, so any delta is the live stack, not the probe.
5. **Captured data**: last window of `topic_freq.log`, plus counts of topics with inter- and
   **intra**-process traffic (nonzero intra proves the intra-process capability on real nodes).

## Comparisons to capture

- **Probe overhead:** run once with `LD_PRELOAD` unset (comment it in the env conf, restart the
  stack) and once with it set. Diff the per-node cpu_s. Expected: negligible (< ~1% of a core even
  under heavy point-cloud load; the hot path is one relaxed atomic + a thread-local cache).
- **SHM on/off:** point `CYCLONEDDS_URI` at `cyclonedds_default_shm.xml` vs `cyclonedds_default_no_shm.xml`,
  restart, rerun. Confirms loaned/zero-copy receives still land on `callback_start` and get counted.

## Interpreting output

The output file (`/root/ssd2tb/logs/topic_freq.log`, override with `ROS_TOPIC_STATS_OUTPUT_FILE`)
appends one block per window:

```
# ts_ns=<...> window_s=5.000
TOPIC <topic> <hz>                      # publish-side, inter-process
RECV  <topic> inter=<hz> intra=<hz>     # receive-side, both transports
NODE  <node>
```

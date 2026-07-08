# On-Orin test runbook — ros2_pulse

Goal: on the real robot, reproduce the same data we measured locally — per-topic Hz (inter +
intra), CPU overhead (probe ON vs OFF), network cost, SHM on/off — and the bake-off vs eBPF /
LTTng. Run everything **inside the ROS container on the Orin**. Adjust `ros2_dev` /
`/root/ros2_ws` to your actual container name / workspace path.

Nothing here modifies the running stack except toggling one env var + restarting (Phase 2/3).

---

## Phase 0 — build the package on the Orin

```bash
# on the Orin host
docker exec -it ros2_dev bash
# inside the container:
cd /root/ros2_ws/src/ros2_pulse
git fetch origin && git checkout main   # has probe + bench + orin kit
cd /root/ros2_ws
colcon build --packages-select ros2_pulse
source install/setup.bash
```

## Phase 1 — does it work here, and what data do we get? (the core ask)

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

To see **the real stack's** topics, just launch the stack normally — `LD_PRELOAD` is already wired
into the deploy env (`docker_v2/config/ros2_env_common.conf`), so `topic_freq.log`
(`/root/ssd2tb/logs/topic_freq.log`) fills with every real topic's inter/intra Hz + active nodes.
Watch it live:

```bash
tail -f /root/ssd2tb/logs/topic_freq.log
```

Look specifically for **`intra=<nonzero>`** on composable/point-cloud topics — that's the data no
rmw-level tool or built-in stat can give you.

## Phase 2 — CPU + network overhead (probe ON vs OFF)

```bash
# with the real stack running under the probe (LD_PRELOAD set):
graph-monitor/ros2_pulse/test/orin/run_orin_probe_test.sh 60 perception
#   -> per-node CPU over 60s, socket-count delta (probe opens none), captured topic data

# then A/B: comment the LD_PRELOAD line in the env conf, restart the stack, rerun the same:
graph-monitor/ros2_pulse/test/orin/run_orin_probe_test.sh 60 perception
# diff the per-node cpu_s between the two runs = the probe's real overhead on Orin.
```

## Phase 3 — iceoryx SHM on vs off

```bash
# point CYCLONEDDS_URI at the SHM profile, restart stack, confirm loaned/zero-copy receives count:
export CYCLONEDDS_URI=/root/ros2_ws/src/10xCode/setup/deployment/docker_v2/planner_computer/config/cyclonedds_default_shm.xml
# ... restart stack, tail topic_freq.log, confirm intra/inter counts on SHM topics ...
# then repeat with cyclonedds_default_no_shm.xml and compare.
```

## Phase 4 — bake-off vs eBPF + LTTng, on Orin hardware

This is the important portability test: **does eBPF even work on the Jetson kernel?**

```bash
# needs a privileged container on the Orin (eBPF: debugfs + CAP_SYS_ADMIN + BTF):
docker run --rm --privileged \
  -v /root/ros2_ws/src/10xCode/graph-monitor/ros2_pulse:/pkg \
  -v /tmp/bench:/work \
  <your-ros-image> bash /pkg/bench/run_bakeoff.sh
```

Expected Orin-specific outcomes to capture:
- **eBPF leg** may report `n/a` if the Jetson kernel lacks uprobe/BTF — that itself is the result
  (eBPF not portable to the fleet).
- **LTTng leg** = 0 events unless the Orin image links lttng-ust (Phase 1b tells you).
- **ours** should match the local ~0% overhead.

## What to send back

Paste: Phase 1a/1b output, the two `topic_freq.log` windows (real stack, and SHM-on), both
`run_orin_probe_test.sh` reports (probe ON and OFF), and the Phase 4 results table. I'll turn it
into the on-Orin RESULTS section.

# ros2_pulse

**The heartbeat of your ROS 2 graph.** A low-overhead probe (sub-ns counting hot path, ≈2 %
workload CPU on a deliberately hostile 4,900 msg/s stress — less on real graphs) that measures
per-topic message frequency and active-node liveness — for **both inter-process and
intra-process** traffic — on **stock ROS 2 binaries**, with **no rebuild, no privileges, and
zero network cost**.

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-Apache%202.0-green)](LICENSE)

---

## Why

You want to answer a simple question in production: **"is every topic flowing at the rate it
should, and which nodes are alive?"** The existing options each fall short:

- `ros2 topic hz` — subscribes to each topic (adds DDS traffic + CPU), one topic at a time, and is
  **blind to intra-process messages**.
- **Built-in topic statistics** — on Humble-class binaries,
  [bypassed entirely by intra-process comms](https://github.com/ros2/rclcpp/issues/2911), so composable
  nodes carrying point clouds lose all introspection. (Fixed upstream for intra-process on `rolling`/newer
  by [rclcpp#3130](https://github.com/ros2/rclcpp/pull/3130), merged Apr 2026; no public Humble backport
  as of this writing.)
- **`ros2_tracing` / LTTng** — powerful, but built for offline analysis: a running session daemon
  and post-processing a CTF trace just to get a rate (and on Humble it additionally needs ROS
  rebuilt with the lttng-ust backend; Jazzy+ binaries trace out-of-the-box).
- **CARET (Tier IV)** — same hook layer as this probe (LD_PRELOAD over tracetools — independent
  validation of the mechanism), built for deep offline latency/chain analysis: needs LTTng, a
  forked rclcpp, and Jupyter post-processing. Complementary, not always-on.
- **eBPF/uprobe probes** — need `CAP_SYS_ADMIN` + debugfs + a kernel with BTF/uprobes (often a
  non-starter on Jetson/embedded), and pay a kernel-trap per message.

`ros2_pulse` fills the gap: it hooks the ROS 2 **tracetools instrumentation layer** that rclcpp
already calls on every publish and every callback, counts in-process with a lock-free hot path, and
writes ready-to-read Hz to a small rolling file.

## What you get

```
# ts_ns=1782887153899445923 window_s=5.000
TOPIC /scan 20.000000                      # publish-side, inter-process
PUB   /points inter=0.000000 intra=30.000000   # publish-side incl. intra (Jazzy+)
RECV  /scan inter=20.000000 intra=0.000000 # receive-side, BOTH transports
RECV  /points inter=0.000000 intra=30.000000   # <- intra-process, invisible to other tools on Humble
NODE  /perception
NODE  /planner
WARN  TOPIC /scan hz=1.200000 expected=[18,22]  # only with an expected-rate spec (see below)
```

## How it works

`libros2_pulse.so` is injected via `LD_PRELOAD`. It exports the same symbols as
`libtracetools.so`'s tracepoint API (`ros_trace_rcl_publish`, `ros_trace_callback_start`, the
init tracepoints, …); the dynamic linker binds rclcpp's calls to ours first, and each interposer
records a stat then forwards to the real function via `dlsym(RTLD_NEXT, …)`. Because rclcpp calls
these functions unconditionally (the LTTng enable-check is *inside* them), the probe works with **no
tracing session** and adds no DDS traffic.

- **Intra-process visibility** comes from `callback_start(callback, is_intra_process)`, which fires
  for every subscription callback regardless of transport.
- **Hot path** is a per-endpoint relaxed atomic increment behind a 256-slot thread-local cache
  with a stride-breaking hash — no global lock, no per-message string hashing. Counting costs
  ~0.3 ns/op fixed-endpoint and ~0.6–1.2 ns alternating across a working set (reference box) —
  two orders of magnitude under a single LTTng-UST tracepoint (~158 ns).
- A background timer snapshots + resets counts every `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` seconds
  and appends Hz to `ROS_TOPIC_STATS_OUTPUT_FILE`.

The pure-C++ core (`core/`) has no ROS dependency and is unit-tested in isolation; the probe layer
(`probe/`) is a thin `LD_PRELOAD` shim.

## Install

```bash
cd ~/ros2_ws/src && git clone https://github.com/TanayK07/ros2_pulse.git
cd ~/ros2_ws && colcon build --packages-select ros2_pulse && source install/setup.bash
```

Requires: ROS 2 Humble, a `libtracetools.so` with instrumentation compiled in (the default on
Humble/Isaac binaries — verify with `nm -D $(ros2 pkg prefix tracetools)/lib/libtracetools.so* | grep -c ros_trace`).

## Usage

```bash
export LD_PRELOAD=libros2_pulse.so                      # resolved from the sourced workspace
export ROS_TOPIC_STATS_OUTPUT_FILE=/tmp/pulse.log       # default: /root/ssd2tb/logs/topic_freq.log
export ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=5.0          # seconds
export ROS_PULSE_EMIT_IDLE=1                            # optional; default 0 — see below
ros2 launch your_stack your.launch.py
tail -f /tmp/pulse.log
```
A missing preload lib is non-fatal (`ld.so` warns and ignores), so it is safe to set fleet-wide.

### Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `ROS_TOPIC_STATS_OUTPUT_FILE` | `/root/ssd2tb/logs/topic_freq.<pid>.log` | Where the stats file is appended (per-process by default; set an explicit path to share one file deliberately). |
| `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` | `5.0` | Flush/snapshot window, in seconds. |
| `ROS_TOPIC_STATS_MAX_BYTES` | `10485760` (10 MiB) | Size cap: at/over it the file rotates to `<path>.1` (single generation, worst-case disk = 2× cap per process). `0` disables rotation (pure append). Reopen-per-window is kept, so external logrotate also works. |
| `ROS_PULSE_EMIT_IDLE` | `0` | When `1`, also emit a `TOPIC /x 0.000000` line for a **declared-but-silent** topic (one with no traffic in the window). By default (`0`) such topics are omitted, so a large graph isn't padded with a zero line per silent topic every window. Only the publish-side `TOPIC` line is affected; `RECV` emits an explicit `0.000000` line for topics that have delivered at least once (so a **stalled** upstream stays visible) and omits never-active topics. |
| `ROS_TOPIC_STATS_EXPECTED` | unset | Path to an expected-rate spec (below). When set, each window is checked at flush time and violations are appended as `WARN` lines. Unreadable or malformed specs warn once on stderr and disable alerting — never crash the host. |

### Expected-rate alerting

Declare what "healthy" means and let the probe say when reality disagrees — no per-node
`diagnostic_updater` code, whole process tree at once:

```yaml
# /etc/pulse/expected.yaml — a documented YAML subset (flow-map topic rules; no YAML lib in the probe)
topics:
  /scan:   {min_hz: 18, max_hz: 22, side: recv}   # side: pub|recv (default recv)
  /points: {min_hz: 25, transport: intra}         # transport: inter|intra|any (default any = sum)
nodes: [/perception, /planner]                    # expected alive
```

```bash
`transport: any` is the topic's rate however it travels. On the receive side the two buckets are
disjoint deliveries, so they are summed. On the publish side one `publish()` can fire both the
intra-process and the RMW tracepoint for the same message (Iron+, and always under TransientLocal
QoS on Jazzy+), so the larger bucket is used rather than the sum.

export ROS_TOPIC_STATS_EXPECTED=/etc/pulse/expected.yaml
```

Violations render inside the normal window block (additive — existing parsers unaffected):

```
WARN TOPIC /scan hz=1.200000 expected=[18,22]
WARN NODE /planner missing
```

Evaluation happens only at flush time (the hot path never sees the spec), the first window is
grace-skipped (attach ramp-up), and each probed process only judges endpoints it hosts.

**`pulse-check`** turns any log (or set of per-process logs) into an exit code for watchdogs,
systemd or CI — it re-derives the verdict from the raw rates of the LAST window, no ROS needed:

```bash
pulse-check --spec /etc/pulse/expected.yaml /tmp/pulse.*.log && echo healthy
# exit 0: pass   1: violations (printed)   2: bad input
# offline it owns the whole picture: a spec topic in NO log is reported at 0 Hz
```

## Benchmarks

Measured in `ros:humble` / `ros:jazzy` / `ros:kilted` containers, workload ≈ 4900 msg/s across
53 mixed topics (30 light @100 Hz + 8 heavy ~100 KB @50 Hz inter-process + 15 intra @100 Hz) —
a deliberately hostile stress. Paired, order-alternated trials (N=10 per distro) with error
bars; full harness + methodology in [`bench/`](bench/).

| Method | CPU overhead | Monitor's own cost | Disk | Intra-proc | Stock binaries | Privileges |
|---|---|---|---|---|---|---|
| **ros2_pulse** | **≈ +2 % at worst-case stress** (pooled +1.9 % ± 0.7 % across humble/jazzy/kilted; ~1 µs/msg all-in) | in-process | ~22 KB rolling | ✅ | ✅ | none |
| eBPF uprobe | ~noise at this rate¹ | bpftrace proc | 0 | ✅ | ✅ | CAP_SYS_ADMIN + BTF |
| LTTng / ros2_tracing | — captured **0 events** on stock binaries² | daemons | CTF (large) | ✅ | ❌ needs rebuild | sessiond |

Counting hot path microbench: **~0.3 ns/op** warm-cache, **~0.6–1.2 ns/op** alternating
endpoints — two orders of magnitude under one LTTng-UST tracepoint (~158 ns). The end-to-end
≈2 % is the diffuse footprint of observing at all (chained tracepoints, cache/TLB residency),
attributed by controlled experiments in [`bench/RESULTS.md`](bench/RESULTS.md); real graphs
with lower aggregate rates see proportionally less.

¹ uprobe = per-event kernel trap (~µs), grows with message rate. ² stock `libtracetools.so` isn't
always linked to lttng-ust; then the tracepoints are no-ops. See [`bench/RESULTS.md`](bench/RESULTS.md).

## Limitations

- On ROS 2 Humble there is no intra-process *publish* tracepoint, so intra rate is measured
  **receive-side** (per subscription) — the signal you usually want. On Jazzy+ the probe also
  hooks `rclcpp_intra_publish` and emits an additive `PUB` line with publish-side intra rates.
- **Node liveness is traffic-derived.** There is no node-teardown tracepoint on stock Humble, so a
  node appears in the `NODE` lines only while a topic it publishes or subscribes to has carried
  traffic within the last few windows; a node silent for several windows is treated as *quiet* and
  drops out. A genuinely-alive but idle node (e.g. a pure timer/service node with no topic traffic)
  therefore reads as quiet. Re-`init` of a node name does not duplicate its entry.
- Requires tracing instrumentation compiled into the ROS build (default on Humble/Isaac debs;
  runtime-checkable via `ros_trace_compile_status()`).
- **Python (rclpy) nodes: publish-side only.** `rcl_publish` fires in the C layer, so Python
  publishers are counted; but `callback_start` is rclcpp-only and rclpy was never instrumented
  ([ros2_tracing#15](https://github.com/ros2/ros2_tracing/issues/15)), so a Python subscriber's
  deliveries do not appear in `RECV` lines.
- File output only; no live network export (by design — zero network cost).

## Compatibility

| Distro | Status | Notes |
|---|---|---|
| Humble (LTS, EOL 2027) | ✅ CI-tested | tracetools ships without the LTTng backend — the case where this probe is the *only* zero-rebuild option |
| Jazzy (LTS, EOL 2029) | ✅ CI-tested | full suite green on stock `ros:jazzy`; tracetools is LTTng-enabled, the probe forwards so a live tracing session coexists |
| Kilted | ✅ CI-tested | full suite green on stock `ros:kilted` |
| Rolling | 🟡 non-blocking CI lane | observational — watches upstream churn |
| Iron | ❌ not targeted | EOL December 2024 |

Middleware-agnostic (hooks sit above the DDS vendor): validated with FastRTPS and CycloneDDS on
Humble. On Jazzy+ the probe additionally hooks the dedicated `rclcpp_intra_publish` tracepoint:
**publish-side** intra rates appear on an additive `PUB` line; on Humble intra stays
receive-side (the tracepoint does not exist there).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Issues and PRs welcome.

## License

Apache-2.0. See [LICENSE](LICENSE).

# ros2_pulse

**The heartbeat of your ROS 2 graph.** A near-zero-overhead probe that measures per-topic message
frequency and active-node liveness — for **both inter-process and intra-process** traffic — on
**stock ROS 2 binaries**, with **no rebuild, no privileges, and zero network cost**.

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
- **`ros2_tracing` / LTTng** — powerful, but built for offline analysis: needs ROS built with the
  lttng-ust backend, a running session daemon, and post-processing a CTF trace just to get a rate.
- **eBPF/uprobe probes** — need `CAP_SYS_ADMIN` + debugfs + a kernel with BTF/uprobes (often a
  non-starter on Jetson/embedded), and pay a kernel-trap per message.

`ros2_pulse` fills the gap: it hooks the ROS 2 **tracetools instrumentation layer** that rclcpp
already calls on every publish and every callback, counts in-process with a lock-free hot path, and
writes ready-to-read Hz to a small rolling file.

## What you get

```
# ts_ns=1782887153899445923 window_s=5.000
TOPIC /scan 20.000000                      # publish-side, inter-process
RECV  /scan inter=20.000000 intra=0.000000 # receive-side, BOTH transports
RECV  /points inter=0.000000 intra=30.000000   # <- intra-process, invisible to other tools on Humble
NODE  /perception
NODE  /planner
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
- **Hot path** is a per-endpoint relaxed atomic increment behind a 16-slot thread-local cache —
  no global lock, no per-message string hashing. Counting costs a few ns/op with a warm cache
  (~3 ns fixed-endpoint, ~12 ns alternating across a 4-topic working set on the reference box) —
  one to two orders of magnitude under a single LTTng-UST tracepoint (~158 ns).
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

## Benchmarks

Measured on a `ros:humble` container, workload ≈ 4900 msg/s across 53 mixed topics
(30 light @100 Hz + 8 heavy ~100 KB @50 Hz inter-process + 15 intra @100 Hz). Full harness +
methodology in [`bench/`](bench/).

| Method | CPU overhead | Monitor's own cost | Disk | Intra-proc | Stock binaries | Privileges |
|---|---|---|---|---|---|---|
| **ros2_pulse** | **within noise** (interleaved N=6: −0.2%) | in-process | ~22 KB rolling | ✅ | ✅ | none |
| eBPF uprobe | ~noise at this rate¹ | bpftrace proc | 0 | ✅ | ✅ | CAP_SYS_ADMIN + BTF |
| LTTng / ros2_tracing | — captured **0 events** on stock binaries² | daemons | CTF (large) | ✅ | ❌ needs rebuild | sessiond |

Counting hot path microbench: **~3 ns/op** warm-cache, **~12 ns/op** alternating endpoints
(hundreds of times cheaper than a naive global-mutex + per-message
string-hash design).

¹ uprobe = per-event kernel trap (~µs), grows with message rate. ² stock `libtracetools.so` isn't
always linked to lttng-ust; then the tracepoints are no-ops. See [`bench/RESULTS.md`](bench/RESULTS.md).

## Limitations

- On ROS 2 Humble there is no intra-process *publish* tracepoint, so intra rate is measured
  **receive-side** (per subscription) — the signal you usually want.
- **Node liveness is traffic-derived.** There is no node-teardown tracepoint on stock Humble, so a
  node appears in the `NODE` lines only while a topic it publishes or subscribes to has carried
  traffic within the last few windows; a node silent for several windows is treated as *quiet* and
  drops out. A genuinely-alive but idle node (e.g. a pure timer/service node with no topic traffic)
  therefore reads as quiet. Re-`init` of a node name does not duplicate its entry.
- Requires tracing instrumentation compiled into the ROS build (default on Humble/Isaac debs;
  runtime-checkable via `ros_trace_compile_status()`).
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
Humble. Jazzy+ adds a dedicated `rclcpp_intra_publish` tracepoint enabling **publish-side** intra
counts — hooking it is on the roadmap; intra rates are receive-side on all distros today.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Issues and PRs welcome.

## License

Apache-2.0. See [LICENSE](LICENSE).

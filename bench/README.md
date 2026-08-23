# ros2_pulse bake-off

Empirical comparison of `ros2_pulse` against the real alternatives for per-topic frequency
monitoring, on an identical workload. Answers "is ours actually better?" with measured numbers
instead of reasoning.

## Methods compared

| Method | Hook | Aggregation |
|---|---|---|
| **baseline** | none | — |
| **ours** | `LD_PRELOAD` interposes `ros_trace_*` (tracetools), in-process | per-endpoint atomic, local file |
| **eBPF (uprobe)** | `bpftrace` uprobe on the SAME `ros_trace_*` symbols | in-kernel BPF map |
| **LTTng / ros2_tracing** | lttng-ust `ros2:*` tracepoints (built into rclcpp) | CTF trace to disk, offline analysis |

All hook the same instrumentation points, so it's a fair per-message comparison.

## Workload

`stress_nodes` builds a mixed graph: 30 light topics @100 Hz + 8 heavy (~100 KB) @50 Hz
inter-process + 15 intra-process topics @100 Hz (~4900 msg/s aggregate). Each process reports its
own CPU via `getrusage` so we can attribute cost precisely.

## What is measured, per leg

- **workload_cpu_s**: added CPU to the *monitored* processes (uprobe traps land here as kernel
  time; lttng-ust tracepoint cost lands here as user time; ours is fully in-process).
- **monitor_cpu_s**: the monitor's *own* process cost (bpftrace userspace, lttng daemons). Ours
  has none (in-process) beyond the workload figure.
- **disk**: bytes written (ours: small rolling file; eBPF: 0, in-kernel map; LTTng: full CTF).
- **network**: all three: 0 (none publish to DDS).

## Run

Requires a **privileged** container (eBPF needs debugfs + CAP_SYS_ADMIN):

```bash
docker run --rm --privileged \
  -v <path>/graph-monitor/ros2_pulse:/pkg \
  -v /tmp/bench:/work \
  ros:humble bash /pkg/bench/run_bakeoff.sh
```

On an Orin, run `run_bakeoff.sh` inside the ROS container (privileged); note that Jetson kernels may
lack uprobe/BTF support, in which case the eBPF leg reports `n/a` (which is itself a result: eBPF
is not portable to the fleet).

## Files

- `stress_nodes.cpp` / `CMakeLists.txt`: the shared workload.
- `bpftrace_probe.bt`: the eBPF leg (uprobe on `ros_trace_rcl_publish` + `ros_trace_callback_start`).
- `run_bakeoff.sh`: orchestrator (baseline / ours / eBPF / LTTng) + results table.
- `RESULTS.md`: recorded measurements (committed after a run).

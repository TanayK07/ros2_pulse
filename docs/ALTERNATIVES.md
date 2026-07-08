# Alternatives & benchmarking

Two parts: **(A)** other ways to get "per-topic Hz + node liveness, incl. intra-process", built-in
or third-party, and where each falls short; **(B)** how to benchmark ros2_pulse rigorously.
Companion: [KNOWN_ISSUES.md](KNOWN_ISSUES.md), [TESTING_PLAN.md](TESTING_PLAN.md).

---

## Part A — ways to get similar output

Constraint set ros2_pulse targets: **intra-process visibility + zero added DDS traffic + zero
privilege + stock (unrebuilt) binaries + drop-in ready-to-read output**. No single off-the-shelf
tool meets all five. Rundown:

### `ros2 topic hz` (CLI)
- **What.** Subscribes to a topic and prints receive rate.
- **Gap.** Adds a real DDS subscriber (extra traffic + CPU), one topic per invocation, and is
  **blind to intra-process** delivery (it receives over the middleware). No node liveness.
- **Verdict.** Fine for a spot check on one inter-process topic; not a always-on graph probe.

### Built-in topic statistics
- **What.** rclcpp can publish per-subscription stats (`message_age`, `message_period`) on
  `/statistics` when enabled via `SubscriptionOptions::topic_stats_options` (opt-in per
  subscription, `state = Enable`).
- **Gap (Humble).** Bypassed by intra-process comms —
  [ros2/rclcpp#2911](https://github.com/ros2/rclcpp/issues/2911). Also: opt-in per subscription (not
  automatic on stock binaries), **publishes over DDS** (adds traffic), receive-side only (no
  publish-side rate, no liveness), and gives period/age not a plain Hz.
- **Upstream status.** #2911 was fixed by
  [PR #3130](https://github.com/ros2/rclcpp/pull/3130) (merged to `rolling`, Apr 2026) — a
  type-erased stats handler wired into `SubscriptionIntraProcess`. **On `rolling`/newer the
  intra-process gap is closed.** No public Humble backport as of this writing, so ros2_pulse's
  intra claim holds for stock Humble but should be scoped to that. Even post-fix, built-in stats
  remain opt-in, DDS-published, and receive-only — ros2_pulse still differs on zero-config +
  publish-side + zero-network + file output.

### `ros2_tracing` / LTTng + tracetools
- **What.** The canonical instrumentation path. Hooks the same `ros_trace_*` tracepoints, records a
  CTF trace via an LTTng session, analyse offline with `tracetools_analysis` (pandas/Jupyter) to
  derive rates.
- **Gap.** Needs `libtracetools.so` **built against `lttng-ust`** (not the case on some stock
  images — the bake-off measured **0 events** on `ros:humble`), a running `lttng-sessiond`, and
  post-processing. Built for offline analysis, not a cheap always-on "current Hz" readout.
- **Relationship.** ros2_pulse hooks the *same layer* but replaces "record everything → analyse
  offline" with "count in-process → emit Hz now", and works even when the tracepoints are no-ops
  (it interposes the function symbols themselves).

### eBPF / uprobe (bpftrace on the tracetools functions)
- **What.** Attach uprobes to `ros_trace_rcl_publish` / `ros_trace_callback_start` and aggregate in
  a kernel map. `bench/bpftrace_probe.bt` demonstrates it.
- **Gap.** Needs **`CAP_SYS_ADMIN` + debugfs + a BTF/uprobe-capable kernel** (a privileged container
  was required just to attach in the bake-off), and pays a **kernel trap per message** (~1–2 µs)
  that scales with rate — fine at a few kHz, visible at point-cloud rates. A real portability risk
  on Jetson/Orin.
- **Verdict.** Closest technical cousin (same hooks) but loses on privilege + kernel portability.

### DDS-vendor tooling (Fast DDS Monitor, Cyclone tools)
- **What.** Wire-level DDS discovery/traffic monitors.
- **Gap.** See only what hits the middleware → **no intra-process**, and report DDS entities not ROS
  topics/nodes 1:1. Middleware-specific.

### Custom node / graph API
- **What.** A monitoring node using the rclcpp graph API + generic subscriptions, or a Prometheus
  exporter node.
- **Gap.** Generic subscriptions still ride the middleware (no intra), and any subscriber adds
  traffic/CPU. Requires writing + deploying a node into the graph.

### Interpose at `rmw_*` instead of tracetools
- **What.** The original `rmw_stats_shim` approach.
- **Gap.** `RMW_IMPLEMENTATION_WRAPPER` does not exist in stock Humble `rmw_implementation`, so it
  silently never ran (per `docs/DESIGN.md`); and even if it did, the rmw layer is **below** the
  intra-process path → no intra visibility. This is *why* ros2_pulse moved up to the tracetools
  layer.

### Positioning summary

| Method | Intra-proc | Added DDS traffic | Privilege | Stock binaries | Ready-to-read Hz | Node liveness |
|---|---|---|---|---|---|---|
| ros2_pulse | ✅ | none | none | ✅ | ✅ (file) | ✅ (see issue #2) |
| `ros2 topic hz` | ❌ | yes (subscriber) | none | ✅ | ✅ (stdout, 1 topic) | ❌ |
| Built-in topic stats (Humble) | ❌ (#2911) | yes (/statistics) | none | opt-in | period/age not Hz | ❌ |
| Built-in topic stats (rolling, #3130) | ✅ | yes (/statistics) | none | opt-in | period/age not Hz | ❌ |
| ros2_tracing / LTTng | ✅ | none (CTF disk) | sessiond | ❌ (needs lttng-ust build) | ❌ (offline) | ✅ (offline) |
| eBPF uprobe | ✅ | none (kernel map) | CAP_SYS_ADMIN | ✅ | needs script | possible |
| DDS monitor | ❌ | monitor traffic | none | ✅ | ❌ | partial |

**The honest one-liner:** ros2_pulse is not "cheaper CPU than eBPF" (the bake-off shows both ≈ noise
at moderate rates). It wins on **deployability**: intra-process + zero-network + zero-privilege +
stock-binary + drop-in file, simultaneously. Keep the marketing there.

---

## Part B — benchmarking methodology

The existing `bench/` is already sound (interleaved trials, microbench, honest verdicts). This
codifies it and fills gaps. See `bench/RESULTS.md` for current numbers.

### Metrics that matter (and how to measure)

| Metric | Why | Tool |
|---|---|---|
| CPU overhead (probe ON vs OFF) | headline claim | `perf stat -r N`, `pidstat -u`, wall-clock of a fixed workload |
| Per-op hot-path cost | isolates the atomic increment | microbench (`bench/hotpath_bench.cpp`) |
| Added network traffic | "zero DDS cost" claim | `ros2 topic bw`, `ifstat`/`sar -n DEV` on `lo`, DDS vendor stats |
| Memory (RSS) | always-on footprint | `/proc/<pid>/status`, `pidstat -r` |
| Disk write rate | rolling-file cost | file growth / window, `iostat` |
| **Accuracy** | is the reported Hz correct? | drive a known-rate publisher, compare (see below) |

### Beating variance (the important part)

Single short samples carry ~±10% run-to-run variance (documented in `bench/RESULTS.md`), which
swamps a <2% overhead signal. Rules:

1. **Interleave** baseline and probe trials (A,B,A,B,…), don't run all-A then all-B — drift and
   thermal state bias blocked runs. `bench/run_overhead_repeated.sh` does this; keep N ≥ 6.
2. **Pin** the workload (`taskset`/cpuset) and disable turbo/frequency scaling where possible.
3. Report **mean ± stddev**, not a single delta. Treat anything inside ±1σ as "within noise" —
   don't claim a win there (the current verdict does this correctly).
4. Use `hyperfine` for wall-clock A/B when the workload is a fixed-duration run.

### Accuracy benchmark (missing today — add it)

Overhead ≠ correctness. Add a harness that drives publishers at **known** rates and asserts the
probe's reported Hz is within tolerance:

- Publisher at exactly R Hz for T seconds → expect `|measured − R| / R < 5%` per window (after the
  first partial window).
- **Double-count regression** (issue #1): one process, one inter-process publisher **and**
  subscriber on the same topic → today reports ~2R; after the fix must report R on both
  `TOPIC` and `RECV inter`. This is the single most important accuracy test.
- Multi-subscriber: N in-process subscribers on one topic → receive-side must not multiply the
  topic rate by N (or must clearly define "N deliveries").
- Intra vs inter attribution: `use_intra_process_comms` on → traffic lands in `intra`, `inter ≈ 0`;
  off → the reverse.

### Bake-off reproducibility

- Keep the workload spec in `bench/RESULTS.md` (msg counts, sizes, rates) so numbers are
  comparable across runs/machines.
- Record host (cores, CPU, kernel), container image + tag, and ROS distro alongside every result.
- For the eBPF/LTTng legs, record the *enabling requirements* met (privileged? lttng-ust present?) —
  a "0 events" result is a finding, not a failure, and must be labelled as such.

### On-hardware (Orin/Jetson)

`test/orin/` already scripts this. Additions: capture the same accuracy assertions on-target (kernel
and DDS SHM behaviour differ), and record whether eBPF could attach at all (the portability claim is
the whole point).

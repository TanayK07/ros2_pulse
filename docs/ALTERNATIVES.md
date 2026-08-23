# Alternatives & benchmarking

Two parts: **(A)** other ways to get "per-topic Hz + node liveness, incl. intra-process", built-in
or third-party, and where each falls short; **(B)** how to benchmark ros2_pulse rigorously.
Companion: [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

---

## Part A: ways to get similar output

Constraint set ros2_pulse targets: **intra-process visibility + zero added DDS traffic + zero
privilege + stock (unrebuilt) binaries + drop-in ready-to-read output**. No single off-the-shelf
tool meets all five. Rundown:

### `ros2 topic hz` (CLI)
- **What.** Subscribes to a topic and prints receive rate.
- **Gap.** Adds a real DDS subscriber, one topic per invocation, and is **blind to
  intra-process** delivery (it receives over the middleware). No node liveness. Measured
  ([bench/RESULTS.md, observer effect](../bench/RESULTS.md)): `hz` costs 7 % of a core per
  watched 100 KB topic and `echo` 31 %; the rate they print is accurate, but pointing `hz` at an
  intra-process topic makes the publisher serialize every message: +52 % CPU on that process,
  for a rate it could not see before you looked.
- **Verdict.** Fine for a spot check on one inter-process topic; not an always-on graph probe,
  and not a neutral observer of intra-process traffic.

### Built-in topic statistics
- **What.** rclcpp can publish per-subscription stats (`message_age`, `message_period`) on
  `/statistics` when enabled via `SubscriptionOptions::topic_stats_options` (opt-in per
  subscription, `state = Enable`).
- **Gap (Humble).** Bypassed by intra-process comms,
  [ros2/rclcpp#2911](https://github.com/ros2/rclcpp/issues/2911). Also: opt-in per subscription (not
  automatic on stock binaries), **publishes over DDS** (adds traffic), receive-side only (no
  publish-side rate, no liveness), and gives period/age not a plain Hz.
- **Upstream status.** #2911 was fixed by
  [PR #3130](https://github.com/ros2/rclcpp/pull/3130) (merged to `rolling`, Apr 2026), a
  type-erased stats handler wired into `SubscriptionIntraProcess`. **On `rolling`/newer the
  intra-process gap is closed.** No public Humble backport as of this writing, so ros2_pulse's
  intra claim holds for stock Humble but should be scoped to that. Even post-fix, built-in stats
  remain opt-in, DDS-published, and receive-only, ros2_pulse still differs on zero-config +
  publish-side + zero-network + file output.

### `ros2_tracing` / LTTng + tracetools
- **What.** The canonical instrumentation path. Hooks the same `ros_trace_*` tracepoints, records a
  CTF trace via an LTTng session, analyse offline with `tracetools_analysis` (pandas/Jupyter) to
  derive rates.
- **Gap (distro-dependent, be precise).** On **Humble**, `libtracetools.so` is **not built
  against `lttng-ust`** (the bake-off measured **0 events** on stock `ros:humble`), so tracing
  needs a ROS rebuild. Since **Iron**, the LTTng tracer is a ROS dependency and **stock binaries
  trace out-of-the-box**; on Jazzy+ the "needs a rebuild" argument is gone, and the
  differentiators are: no `lttng-sessiond`, no CTF post-processing, an *online* ready-to-read Hz
  file, and zero setup. Built for offline analysis either way, not a cheap always-on readout.
- **Relationship.** ros2_pulse hooks the *same layer* but replaces "record everything → analyse
  offline" with "count in-process → emit Hz now", and works even when the tracepoints are no-ops
  (it interposes the function symbols themselves). On LTTng-enabled distros the probe *forwards*
  to the real tracepoints, so it coexists with a live tracing session.

### CARET (Tier IV): the mechanism cousin
- **What.** [CARET](https://tier4.github.io/caret_doc/) is Tier IV's performance-analysis tool
  for ROS 2 (built for Autoware): callback/communication/**chain** latency, frequency, period and
  jitter. Crucially, it uses the **same core mechanism as ros2_pulse**, `LD_PRELOAD` function
  hooking over the tracetools layer to add/observe tracepoints without rebuilding ROS core.
  Independent, production-scale validation that preload-over-tracetools is sound.
- **Gap (for this use case).** CARET is a *deep offline analysis* suite: it requires **LTTng**
  recording sessions, a **forked/patched rclcpp** for its extra tracepoints, a from-source build
  (no apt package), and Jupyter/CLI post-processing. Superb for "why is my pipeline slow";
  heavyweight for "is every topic flowing right now".
- **Relationship.** Same hook layer, opposite trade: CARET maximizes analytical depth at
  deployment cost; ros2_pulse maximizes deployability (zero deps, always-on, online Hz file) at
  analytical depth. They compose: pulse for 24/7 fleet monitoring, CARET for the deep dive when
  pulse flags something.

### `diagnostic_updater` topic diagnostics: the in-code incumbent
- **What.** `diagnostic_updater::TopicDiagnostic` / `HeaderlessTopicDiagnostic`: the standard ROS
  way to monitor a topic's rate against expected bounds, publishing to `/diagnostics`.
- **Gap.** Requires **source changes in every node** (wrap each publisher / add an updater),
  publishes over DDS, covers only what authors remembered to instrument, and nothing intra-only.
- **Relationship.** ros2_pulse is the zero-touch equivalent for a whole process tree; the
  expected-rate spec (ROADMAP R1) will close the "against bounds" half without per-node code.

### rclpy (Python nodes): a real limitation
- Publish side **works** for Python nodes: `rcl_publish` fires in the C `rcl` layer under rclpy.
- Receive side is **invisible**: `callback_start` is emitted by rclcpp only, and rclpy was never
  instrumented at the client-library level
  ([ros2_tracing#15](https://github.com/ros2/ros2_tracing/issues/15)). A Python subscriber's
  delivery rate does not appear in `RECV` lines. Document this wherever receive-side claims are
  made.

### eBPF / uprobe (bpftrace on the tracetools functions)
- **What.** Attach uprobes to `ros_trace_rcl_publish` / `ros_trace_callback_start` and aggregate in
  a kernel map. `bench/bpftrace_probe.bt` demonstrates it.
- **Gap.** Needs **`CAP_SYS_ADMIN` + debugfs + a BTF/uprobe-capable kernel** (a privileged container
  was required just to attach in the bake-off), and pays a **kernel trap per message** (~1-2 µs)
  that scales with rate, fine at a few kHz, visible at point-cloud rates. A real portability risk
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
| ros2_tracing / LTTng | ✅ | none (CTF disk) | sessiond | ❌ Humble / ✅ Jazzy+ | ❌ (offline) | ✅ (offline) |
| CARET (Tier IV) | ✅ | none (CTF disk) | sessiond | ❌ (forked rclcpp, source build) | ❌ (offline) | possible |
| `diagnostic_updater` | ❌ | yes (/diagnostics) | none | ❌ (per-node source changes) | vs bounds | ❌ |
| eBPF uprobe | ✅ | none (kernel map) | CAP_SYS_ADMIN | ✅ | needs script | possible |
| DDS monitor | ❌ | monitor traffic | none | ✅ | ❌ | partial |

In one line: ros2_pulse is not "cheaper CPU than eBPF" (the bake-off shows both ≈ noise
at moderate rates). It wins on **deployability**: intra-process + zero-network + zero-privilege +
stock-binary + drop-in file, simultaneously. Keep the marketing there.

---

## Part B: benchmarking methodology

The existing `bench/` is sound (interleaved trials, microbench). This
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

1. **Interleave** baseline and probe trials (A,B,A,B,...); don't run all-A then all-B, because drift and
   thermal state bias blocked runs. `bench/run_overhead_repeated.sh` does this; keep N ≥ 6.
2. **Pin** the workload (`taskset`/cpuset) and disable turbo/frequency scaling where possible.
3. Report **mean ± stddev**, not a single delta. Treat anything inside ±1σ as "within noise",
   don't claim a win there (the current verdict does this correctly).
4. Use `hyperfine` for wall-clock A/B when the workload is a fixed-duration run.

### Accuracy benchmark (missing today; add it)

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
- For the eBPF/LTTng legs, record the *enabling requirements* met (privileged? lttng-ust present?),
  a "0 events" result is a finding, not a failure, and must be labelled as such.

### On-hardware (Orin/Jetson)

`test/orin/` already scripts this. Additions: capture the same accuracy assertions on-target (kernel
and DDS SHM behaviour differ), and record whether eBPF could attach at all (the portability claim is
the whole point).

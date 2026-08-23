# Benchmark results

Measured 2026-08-01 in stock `ros:humble` / `ros:jazzy` / `ros:kilted` containers on a 16-core
x86 host. Workload: 30 light publishers @100 Hz + 8 heavy (~100 KB) @50 Hz inter-process + 15
intra-process topics @100 Hz, ≈4,900 msg/s aggregate across 91 endpoints and 3 processes, all
on `MultiThreadedExecutor`s. Metric: summed workload CPU seconds (`getrusage(RUSAGE_SELF)`).
Harnesses: `run_overhead_repeated.sh` (paired end-to-end overhead + microbench) and
`run_bakeoff.sh` (single-shot comparison vs eBPF/LTTng).

## Headline

| Method | CPU overhead @4900 msg/s | Monitor's own cost | Disk | Intra-proc | Works on stock ROS binaries | Privileges / kernel |
|---|---|---|---|---|---|---|
| **ours (LD_PRELOAD tracetools)** | **≈ +2 % worst-case stress** (pooled +1.9 % ± 0.7 % SEM)¹ | in-process, none | ~22 KB rolling file | ✅ | ✅ **yes, as-is** | **none** |
| eBPF uprobe (bpftrace) | ~+0.2 % (noise) at this rate² | bpftrace proc ~0.02 s | 0 (in-kernel map) | ✅ (same hooks) | ✅ | **CAP_SYS_ADMIN + debugfs + BTF + uprobe kernel** |
| LTTng / ros2_tracing | **n/a, captured 0 events³** | sessiond + consumerd | CTF (large when working) | ✅ | ❌ **needs ROS rebuilt with lttng-ust** | sessiond |

¹ Paired, order-alternated trials (N=10 per distro, methodology below). This workload is a
a harsh upper bound, 91 endpoints churning at 4,900 msg/s across every core.
² uprobe cost is a per-event kernel trap (~1-2 µs). At 4900 msg/s that's ~0.1-0.2 % of a core:
noise here, but it grows with message rate, whereas our in-line count is ~0.3-1.2 ns/op.
³ On this stock `ros:humble` image `libtracetools.so` is **not linked against lttng-ust**, so
the `ros2:*` tracepoints are no-ops and `lttng` + `babeltrace2` recorded **0 events**.
ros2_tracing requires ROS rebuilt with instrumentation to capture anything.

## End-to-end overhead (paired, order-alternated, N=10 per distro)

Single samples of this workload swing ±4 % run-to-run, enough to fake (or hide) a ~2 %
effect. So each trial runs baseline and probe back-to-back with the **arm order alternated
every trial**, and the statistic is the mean of the per-trial differences with its standard
error. A delta is only reported as real if it clears ~2× SEM. (This protocol exists because a
fixed arm order and N=6 means *did* mislead us once; see the KNOWN_ISSUES #15 note.)

Probe config: `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=2.0` (normal flushing).

| Distro | baseline mean | probe mean | paired diff ± SEM | as % of baseline |
|---|---|---|---|---|
| humble | 2.449 s | 2.510 s | +0.061 s ± 0.036 | +2.5 % ± 1.5 % |
| jazzy  | 2.515 s | 2.579 s | +0.064 s ± 0.041 | +2.5 % ± 1.6 % |
| kilted | 2.570 s | 2.609 s | +0.039 s ± 0.023 | +1.5 % ± 0.9 % |
| **pooled (inverse-variance)** | | | **+0.049 s ± 0.018** | **+1.9 % ± 0.7 %** |

Per message that is ≈1 µs of added system cost against ≈51 µs the stack already spends
delivering it. On realistic graphs (fewer endpoints, lower aggregate rate per process) the
share is proportionally smaller.

### Where the cost is (and is not): controlled attributions, ros:jazzy

| Experiment | Result | Conclusion |
|---|---|---|
| Null shim (same 8 exported symbols, empty bodies) vs baseline | −0.3 % | LD_PRELOAD interposition itself is free |
| Probe with flushing disabled (`PERIOD=60`) vs with flushing (`PERIOD=2.0`), paired N=10 | +0.027 s ± 0.033 | flushing is not the cost |
| Instrumented flush time (6 windows, 3 processes) | 0.7-7 ms wall total | flush work is µs to ms scale |
| TLS-cache miss counters, pre-fix → post-fix | 31,926 → 6,509 fallbacks (pubfarm) | KNOWN_ISSUES #15 stride-aliasing fixed |
| Paired bench pre-fix → post-fix | +2.6 % → +1.9 % (pooled) | cache fix reclaimed part of the delta |

The residual ≈2 % does not localize to the hot path (sub-ns/op, below), the flush path, or
symbol interposition; it is the diffuse footprint of observing at all: the chained call into
the real tracepoint, extra code/data resident in cache and TLB across all 16 executor
threads, and one parked flush thread. We report it rather than subtract it.

## Per-operation microbench (isolated hot path)

Two access patterns: *fixed* = each thread hammers one endpoint (best case); *alt-4* = each
thread alternates across 4 endpoints (the realistic camera-pipeline pattern). 8 threads,
2026-08-01 run of `hotpath_bench.cpp` inside each distro container:

| Leg | humble | jazzy | kilted |
|---|---|---|---|
| OLD design (global mutex + per-msg string hash), fixed | 140.9 ns/op | 167.4 ns/op | 137.0 ns/op |
| single-entry TLS cache (pre-#13), fixed | 0.3 ns/op | 0.3 ns/op | 0.4 ns/op |
| single-entry TLS cache (pre-#13), alt-4 | 46.8 ns/op | 52.3 ns/op | 46.6 ns/op |
| current TLS cache (post-#15), fixed | 0.3 ns/op | 0.3 ns/op | 0.3 ns/op |
| current TLS cache (post-#15), alt-4 | **0.6 ns/op** | **1.2 ns/op** | **1.1 ns/op** |

The alternating-pattern number is the one that matters in practice (KNOWN_ISSUES #13/#15):
the single-entry cache thrashed to ~50 ns/op, and the 16-slot cache's stride-aliasing pushed
realistic multi-topic farms onto a contended lock entirely. The current 256-slot
stride-breaking cache holds alternation at ~1 ns/op, two orders of magnitude under one
LTTng-UST tracepoint (~158 ns, Bédard et al. 2022). Numbers move with host/thread count,
treat ratios, not absolutes, as the signal.

### aarch64: Jetson AGX Orin (2026-08-23, production robot, clocks pinned)

Same `hotpath_bench.cpp`, 10 trials × 8 threads, mean ± SEM. Full write-up and raw trials in
[`test/orin/RESULTS.md`](../test/orin/RESULTS.md).

| Leg | Orin (Cortex-A78AE @ 2.2 GHz) | x86-64 reference |
|---|---|---|
| OLD design (mutex + string hash), fixed | 351 ns/op | 141-167 ns/op |
| single-entry TLS cache, alt-4 | 225 ns/op | 47-52 ns/op |
| current TLS cache, fixed | **0.9 ns/op** | 0.3 ns/op |
| current TLS cache, alt-4 | **2.1-2.2 ns/op** | 0.6-1.2 ns/op |
| raw `steady_clock::now()`, 1 thread | 38.7 ns ± 0.08 | ~21 ns |
| R5 jitter clock read, fixed (`JITTER=1`) | **+51.0 ns/msg ± 0.6** | +24.4 ns/msg ± 0.02 |
| R5 jitter clock read, alt-4 | +55.6 ns/msg ± 0.9 | — |

The ratios carry over: the cache fix holds alternation two orders of magnitude under the old
design on aarch64 too, and the R5 cost is the clock read (89 % of the delta); the vDSO path
works on the 5.15 Tegra kernel, so the syscall-fallback risk flagged in the R5 design did not
materialize.

## Observer effect: what watching a topic with the stock CLI costs (2026-08-23)

`bench/run_observer_effect.sh`, ros:humble, CycloneDDS default, AMD Ryzen 7 7435HS (16 cores).
Workload: the stress farm above (8 × ~100 KB @ 50 Hz + 30 light @ 100 Hz inter-process, 15
intra-process @ 100 Hz, all with subscribers). The probe is the ruler in every arm: it counts
`rcl_publish` in-process, so its publish-side rate is the true rate whatever else subscribes.
Four arms rotated each trial, N = 10, 10 s runs with the watcher attached for 8 s; mean ± SEM.
Raw: [`out/observer_effect/`](out/observer_effect/) (CSV, summary, per-arm probe logs, watcher output).

| Arm | true `/heavy_0` Hz | rate the watcher printed | watcher's own CPU | publisher farm CPU | intra farm CPU | `TOPIC /intra_0` windows |
|---|---|---|---|---|---|---|
| nothing watching | 50.000 | — | 0 | 1.152 s ± 0.016 | 0.303 s ± 0.010 | 0 / 5 |
| `ros2 topic hz /heavy_0` | 50.000 | 49.845 ± 0.053 | **0.566 s ± 0.005 = 7.1 % of a core** | 1.171 ± 0.020 | 0.329 ± 0.009 | 0 |
| `ros2 topic echo /heavy_0 > /dev/null` | 50.000 | — | **2.504 s ± 0.007 = 31.3 % of a core** | 1.133 ± 0.015 | 0.318 ± 0.007 | 0 |
| `ros2 topic hz /intra_0` (intra-process topic) | 50.000 | 99.997 ± 0.001 | 0.654 s ± 0.004 = 8.2 % | 1.116 ± 0.014 | **0.460 s ± 0.004 (+52 %)** | **4 / 5, in 10 / 10 trials** |

What it says, in order of how sure we are:

1. **The stock tools are accurate here.** The publisher held 50.000 Hz in every arm and
   `hz` reported it within 0.3 %. We expected a slow reliable reader to back-pressure a
   100 KB writer; on this box at this load it did not. Published as measured.
2. **Watching is not free.** One `ros2 topic hz` on one 100 KB topic is 7 % of a core for
   as long as you look; `ros2 topic echo` of the same topic is 31 % of a core with its output
   thrown away. Per topic. The probe adds no subscriber and pulse-top reads a file.
3. **On an intra-process topic the watcher is the perturbation.** A pure intra-process
   publisher never reaches `rcl_publish`; the first out-of-process subscriber, the `hz`
   you just started, makes it serialize and send every message. The watched process's CPU
   went up 52 % and the publish-side path lit up in 4 of 5 windows, 10 trials out of 10.
   The number `hz` prints is right; the system it describes is no longer the one that was
   running before you looked.
4. **Publisher and subscriber CPU did not move** with an extra inter-process reader
   (differences are within ~2 SEM in both directions, reliable / KeepLast(10)).

Caveats: one box, one distro, one RMW, 100 KB messages at 50 Hz. A slower CPU (the Orin),
bigger messages or several watchers scale the watcher's cost; they do not change finding 1.

## Verdict

- **CPU cost is ≈2 % on a worst-case synthetic stress and proportionally less on real
  graphs.** The paired numbers are published with error bars rather than a "zero overhead" claim;
  at moderate rates ours ≈ eBPF ≈ small, and the differentiator is elsewhere.
- **Ours wins on deployability, and it's measured:**
  - eBPF **required a privileged container** (CAP_SYS_ADMIN + debugfs + BTF) to attach at all,
    the Orin/Jetson kernel-portability risk is real, not hypothetical. Ours needs zero privileges.
  - LTTng/ros2_tracing **captured nothing on the stock binaries** and needs a ROS rebuild with
    lttng-ust, then offline CTF analysis to derive Hz. Ours runs on the exact deployed binaries
    and emits ready-to-read Hz to a tiny file.
- **No single off-the-shelf option meets all constraints** (intra-process + zero-network +
  zero-priv + stock-binary + drop-in file). Ours does; that is the empirically supported
  "better."

## Reproduce

```bash
# per distro: paired overhead trials + microbench
docker run --rm -e ROS_DISTRO=humble -v <pkg>:/pkg:ro -v /tmp/bench-humble:/work ros:humble \
  bash /pkg/bench/run_overhead_repeated.sh
docker run --rm -e ROS_DISTRO=jazzy  -v <pkg>:/pkg:ro -v /tmp/bench-jazzy:/work  ros:jazzy \
  bash /pkg/bench/run_overhead_repeated.sh
docker run --rm -e ROS_DISTRO=kilted -v <pkg>:/pkg:ro -v /tmp/bench-kilted:/work ros:kilted \
  bash /pkg/bench/run_overhead_repeated.sh

# observer effect: what `ros2 topic hz` / `echo` cost, and what they do to intra-process topics
docker run --rm -v "$PWD":/pkg -v /tmp/oe:/work ros:humble-ros-base bash /pkg/bench/run_observer_effect.sh

# bake-off vs eBPF / LTTng (needs --privileged for the eBPF leg)
docker run --rm --privileged -v <pkg>:/pkg -v /tmp/bench:/work ros:humble bash /pkg/bench/run_bakeoff.sh
```

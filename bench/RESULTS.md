# Bake-off results

Measured in a `ros:humble` container (16-core x86 host). Workload: 30 light @100 Hz + 8 heavy
(~100 KB) @50 Hz inter-process + 15 intra-process @100 Hz (~4900 msg/s aggregate). See
`run_bakeoff.sh` (single-shot 4-leg) and `run_overhead_repeated.sh` (rigorous interleaved trials).

## Headline

| Method | CPU overhead @4900 msg/s | Monitor's own cost | Disk | Intra-proc | Works on stock ROS binaries | Privileges / kernel |
|---|---|---|---|---|---|---|
| **ours (LD_PRELOAD tracetools)** | **−0.2 % (within noise)** | in-process, none | ~22 KB rolling file | ✅ | ✅ **yes, as-is** | **none** |
| eBPF uprobe (bpftrace) | ~+0.2 % (noise) at this rate¹ | bpftrace proc ~0.02 s | 0 (in-kernel map) | ✅ (same hooks) | ✅ | **CAP_SYS_ADMIN + debugfs + BTF + uprobe kernel** |
| LTTng / ros2_tracing | **n/a — captured 0 events²** | sessiond + consumerd | CTF (large when working) | ✅ | ❌ **needs ROS rebuilt with lttng-ust** | sessiond |

¹ uprobe cost is a per-event kernel trap (~1–2 µs). At 4900 msg/s that's ~0.1–0.2 % of a core —
noise here — but it grows with message rate (point clouds at kHz would make it visible), whereas
ours is an in-process relaxed atomic (~0.2 ns/op).
² On this stock `ros:humble` image `libtracetools.so` is **not linked against lttng-ust**, so the
`ros2:*` tracepoints are no-ops and `lttng` + `babeltrace2` recorded **0 events**. ros2_tracing
requires ROS rebuilt with instrumentation to capture anything. (Verify separately on the Isaac image.)

## Rigorous overhead (interleaved, N=6)

Single 10 s samples have ~±10 % run-to-run variance (the LTTng no-op leg swung −9 % vs baseline),
which swamps a <2 % signal. Interleaving baseline/ours across 6 trials and averaging:

```
trial  baseline   ours
1      6.306      6.288
2      6.141      6.133
3      6.169      6.113
4      6.114      6.178
5      6.162      6.173
6      6.263      6.180
mean baseline=6.192s  mean ours=6.177s  delta=-0.2%
```

**Probe overhead is statistically indistinguishable from zero at this workload.**

## Per-operation microbench (isolated hot path)

Two access patterns per design: *fixed* = each thread hammers one endpoint (best case);
*alt-4* = each thread alternates across 4 endpoints (the realistic camera-pipeline pattern —
image + camera_info + compressed from one timer callback). 8 threads, 16-core x86 host,
2026-07-31 run of `hotpath_bench.cpp`:

```
OLD (global mutex + per-msg string hash), fixed  1717.0 ns/op     0.6 M ops/s
single-entry TLS cache (pre-#13), fixed             3.8 ns/op   259.9 M ops/s
single-entry TLS cache (pre-#13), alt-4           496.2 ns/op     2.0 M ops/s   <- the thrash
mini-map TLS cache (current), fixed                 2.9 ns/op   341.7 M ops/s
mini-map TLS cache (current), alt-4                11.6 ns/op    86.1 M ops/s   -> 43x vs pre-#13
```

The single-entry cache's alternating number is the KNOWN_ISSUES #13 finding: every call missed
the cache and hit the shared rw-lock, whose cross-core contention dominated. The 16-slot
direct-mapped mini-map keeps a typical per-thread working set cached; its alt-4 residue
(11.6 vs 2.9 ns) is occasional same-slot collisions, still an order of magnitude under one
LTTng-UST tracepoint (~158 ns). Numbers move with host/thread count — treat ratios, not
absolutes, as the signal. This compares ours to the design it replaces (the earlier
global-mutex + per-message string-hash stats prototype), not to eBPF/LTTng.

## Verdict

- **CPU is not the differentiator.** At moderate rates ours ≈ eBPF ≈ measurement noise. Claiming
  ours is dramatically cheaper on CPU than eBPF would be wrong at these rates.
- **Ours wins on deployability, and it's measured:**
  - eBPF **required a privileged container** (CAP_SYS_ADMIN + debugfs + BTF) to attach at all — the
    Orin/Jetson kernel-portability risk is real, not hypothetical. Ours needs zero privileges.
  - LTTng/ros2_tracing **captured nothing on the stock binaries** — needs a ROS rebuild with
    lttng-ust, then offline CTF analysis to derive Hz. Ours runs on the exact deployed binaries and
    emits ready-to-read Hz to a tiny file.
- **No single off-the-shelf option meets all constraints** (intra-process + zero-network + zero-priv
  + stock-binary + drop-in file). Ours does — that is the empirically supported "better."

## Reproduce

```bash
docker run --rm --privileged -v <pkg>:/pkg -v /tmp/bench:/work ros:humble bash /pkg/bench/run_bakeoff.sh
docker run --rm            -v <pkg>:/pkg -v /tmp/bench:/work ros:humble bash /pkg/bench/run_overhead_repeated.sh
```

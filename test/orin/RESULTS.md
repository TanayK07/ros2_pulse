# On-Orin results for ros2_pulse v0.3.0

Measured 2026-08-23 on a production Jetson AGX Orin running a 77-node ROS 2 Humble /
CycloneDDS navigation stack. Every number below traces to a raw file under
[`out/`](out/); nothing here is transcribed from a screenshot.

| | |
|---|---|
| Platform | Jetson AGX Orin, 12× Cortex-A78AE @ 2.2 GHz, 61 GiB, L4T R36.4.4, kernel 5.15.148-tegra, `nvpmodel` MAXN (mode 0) + `jetson_clocks` pinned |
| ROS | Humble (distro debs), `rmw_cyclonedds_cpp`, inside the deployed container |
| Probe | `ros2_pulse` at tag `v0.3.0`, built on-device with `colcon`, LD_PRELOAD |
| Raw | [`out/platform_host.txt`](out/platform_host.txt), [`out/hotpath/platform.txt`](out/hotpath/platform.txt) |

## Headline: the R5 clock-read cost on Tegra

The one number that gated publishing. `ROS_TOPIC_STATS_JITTER=1` adds one `steady_clock`
read per message; on some Tegra kernels `CLOCK_MONOTONIC` falls out of the vDSO into a
syscall (hundreds of ns), which would have changed the cost story on exactly the platform
this probe targets. It does not.

| | x86-64 reference box | **Jetson AGX Orin** | source |
|---|---|---|---|
| raw `steady_clock::now()`, 1 thread | ~21 ns | **38.7 ns ± 0.08** | `out/hotpath/summary.txt` |
| R5 gap cost, fixed endpoint, 8 threads | +24.4 ns/msg ± 0.02 | **+51.0 ns/msg ± 0.6** | `out/hotpath/summary.csv` |
| R5 gap cost, alt-4 endpoints, 8 threads | — | **+55.6 ns/msg ± 0.9** | `out/hotpath/summary.csv` |
| verdict printed by `run_hotpath_orin.sh` | | **vDSO path works** | `out/hotpath/summary.txt` |

10 trials × 8 threads × 40 M ops, mean ± SEM; per-trial R5 deltas ranged 47.7-54.5 ns with
no drift across trials (clocks pinned; [`out/hotpath/trial_*.txt`](out/hotpath/)). The clock
read alone is 89 % of the delta; the remaining ~6 ns is the gap bookkeeping, same as on x86.
So the Orin cost is ~2.1× x86, explained by the ~1.8× slower vDSO clock on Cortex-A78AE, not by
a syscall fallback. At a 4900 msg/s worst-case stress that is 0.025 % of one core.

The counting hot path itself (no jitter) is unchanged from x86: fixed-endpoint
**0.9 ns/op**, alt-4 **2.1-2.2 ns/op**, vs the pre-#13 single-entry cache thrashing to
225 ns/op under alternation; the KNOWN_ISSUES #13/#15 fix holds on aarch64
(`out/hotpath/trial_1.txt`).

## It runs on the deployed binaries

| Check | Result | source |
|---|---|---|
| `libtracetools.so` instrumented | 29 `ros_trace_*` symbols | `out/instrumentation.txt` |
| lttng-ust linked into tracetools | **not linked** (stock Humble debs); ros2_tracing would capture 0 events here, the probe does not care | `out/instrumentation.txt` |
| controlled graph (demo talker/listener, 1 Hz) | `TOPIC /chatter 1.000`, `RECV inter=1.000 intra=0.000`, `NODE` lines, first window correctly partial (0.5 Hz over a half window) | `out/demo_graph.log` |
| `ROS_TOPIC_STATS_JITTER=1` | `JITTER /chatter pub max_dt_ms=999.96` / `recv 999.99` at 1 Hz | `out/v030_features.txt` |
| `ROS_TOPIC_STATS_FORMAT=jsonl` | last window parses with `json.tool` | `out/v030_features.txt` |
| `ROS_TOPIC_STATS_QUIET=1` | 0 banner lines (1 without) | `out/v030_features.txt` |

## On the production stack (77 nodes, 249 topics)

`LD_PRELOAD` + `ROS_TOPIC_STATS_FORMAT=jsonl` were added to the stack's env file and the
service restarted; the probe loaded into every process the supervisor launched, including
`ros2` CLI invocations, and wrote 1580 windows (267 KB) to tmpfs in the ~8 minutes it was
live. Env reverted and stack restarted clean afterwards (78 nodes back, no preload in any
process environment).

- [`out/stack_topic_freq.jsonl`](out/stack_topic_freq.jsonl): 60 nodes and 39 topic
  endpoints seen. **Names of the operator's own nodes/topics are redacted to `/node_NN` /
  `/topic_NN`; open-source components (nav2, tf, EKF, nvblox, ...) keep their names; every
  rate is as measured.**
- The robot was **parked**: nav2 lifecycle nodes unconfigured, highest rate on the graph
  5 Hz (`/mode_state`-class status topics), `tf_static` at 2 Hz, no sensor streams. So this is
  a liveness/coverage result, not a load result.
- **Intra-process: 0 endpoints.** Expected: this stack runs one process per node under a
  supervisor, so there is no intra-process traffic to count. The intra capability is
  demonstrated on the demo graph and in the x86 bench, not here.
- Sockets opened by the probe: **0 → 0** (unix and udp), both runs
  (`out/report_{off,on}/`).

### CPU A/B: captured, not reported as an overhead number

`run_orin_probe_test.sh 60` was run with the probe off (baseline, stack at 35 h uptime) and
on (~5 min after the restart that enabled it). The two samples are not comparable: the
baseline contains a transient 62 s-CPU process that died in the restart, and the "on" sample
sits in post-restart startup churn (36 processes above 0.5 s CPU vs 8 at steady state). With
the probe on, every ROS node sat at 1-2 % of a core. The raw per-pid jiffies are committed
(`out/report_*/cpu_{start,end}.txt`) so anyone can redo the arithmetic; the overhead claim
itself rests on the x86 paired trials in [`bench/RESULTS.md`](../../bench/RESULTS.md) and the
Orin hot-path bench above. A clean Orin A/B needs both samples at steady state and the same
load, the script now records process names so a future run can match nodes across restarts.

## Not run

- **iceoryx SHM on/off**: the deployed CycloneDDS config has no SHM transport; nothing to
  toggle.
- **eBPF / LTTng bake-off on Orin**: needs a privileged throwaway container on a production
  host; skipped. lttng-ust is not linked into this image's tracetools (above), so the LTTng
  leg would read 0 events, as on x86.

## Files

```
out/
  platform_host.txt         host uname, nvpmodel, L4T, CPU, memory, stack-state note
  instrumentation.txt       ros_trace symbol count, lttng link status
  demo_graph.log            controlled talker/listener under the probe (text format)
  v030_features.txt         JITTER / jsonl / QUIET spot checks
  hotpath/                  run_hotpath_orin.sh: platform, summary.csv/txt, trial_1..10
  stack_topic_freq.jsonl    production stack windows, names redacted (see above)
  report_off/, report_on/   per-pid CPU jiffies before/after each 60 s window
```

Reproduce: [`ORIN_RUNBOOK.md`](ORIN_RUNBOOK.md) (rationale) and the scripts next to it.

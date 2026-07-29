# Roadmap

Defects live in [KNOWN_ISSUES.md](KNOWN_ISSUES.md); this file tracks **features and positioning
work** — things the probe doesn't do yet, ordered by leverage. Research grounding: ros2_tracing
design docs + paper (Bédard et al., RA-L 2022), CARET (Tier IV), ROS 2 distro release notes.

## R1. Expected-rate spec + alerting — turns the logger into a monitor

The headline question is "is every topic flowing at the rate it *should*" — today the operator
must eyeball the log. Add an optional spec (`ROS_TOPIC_STATS_EXPECTED=/path/spec.yaml`):

```yaml
topics:
  /scan:   {min_hz: 18, max_hz: 22, side: recv}
  /points: {min_hz: 25, transport: intra}
nodes: [/perception, /planner]
```

- Probe emits `WARN <topic> hz=<x> expected=[lo,hi]` at flush time (zero hot-path cost).
- Separate `pulse-check` CLI (no ROS dep): log + spec → exit code, for watchdogs/CI/systemd.

Prior art: `diagnostic_updater::TopicDiagnostic` does this **in-code per node** — pulse does it
zero-touch for a whole process tree.

## R2. Timer & service rate monitoring

Hook `rcl_timer_init`, `rclcpp_timer_callback_added`, `rclcpp_timer_link_node` (all exist on
Humble) → `TIMER /node period_ms=20 actual_hz=49.8` lines. Control-loop liveness catches a
wedged node whose subscriptions still drain — arguably a stronger health signal than topic Hz.
Service rates via `rcl_service_init` + `rclcpp_service_callback_added`. Bonus: timer callbacks
become *resolvable*, retiring the negative-cache sentinel path for them (KNOWN_ISSUES #3).

## R3. Distro matrix: Jazzy / Kilted / rolling

Humble EOL is May 2027; Jazzy is the current LTS; Iron is already EOL. Work items:

- CI: extend the industrial_ci matrix beyond `humble` (rolling lane exists, non-blocking).
- Interposer signatures must be re-verified per distro — LD_PRELOAD interposition is
  ABI-sensitive to `tracetools.h` changes.
- Hook `rclcpp_intra_publish` (landed via rclcpp#2091 / ros2_tracing#30) → **publish-side
  intra** counts on Jazzy+, retiring the Humble receive-side-only limitation there. Exporting
  the extra symbol on Humble is harmless (never called).
- Iron+ binaries ship LTTng-enabled tracetools: add an integration assertion that the probe and
  a live LTTng session coexist (we forward via `dlsym(RTLD_NEXT)`, so both should fire).

## R4. Positioning docs (ALTERNATIVES.md gaps)

- **CARET** (Tier IV) is prior art for the exact mechanism — LD_PRELOAD function hooking over
  the tracetools layer — currently uncited. Differences to state plainly: CARET targets deep
  latency/chain analysis and needs LTTng + a forked rclcpp + offline analysis; pulse is a
  permanently-on Hz/liveness probe with zero deps. Independent validation of the approach.
- **Per-distro honesty:** since Iron, stock binaries trace out-of-the-box; the "needs a ROS
  rebuild" claim is Humble-only. On Jazzy the differentiators are: no sessiond, no CTF
  post-processing, online ready-to-read Hz, tiny file.
- **rclpy:** publish side works (rcl-layer tracepoint); receive side is invisible
  (`callback_start` is rclcpp-only; rclpy was never instrumented — ros2_tracing#15). Document.
- Comparison row for `diagnostic_updater::TopicDiagnostic`.

## R5. Jitter / burst visibility (opt-in)

Windowed mean hides a topic alternating 0/40 Hz. `ROS_TOPIC_STATS_JITTER=1`: per-endpoint
min/max inter-arrival per window (one `steady_clock` read + relaxed stores per message,
~20 ns — default OFF, cost documented). Output: `... min_dt_ms=.. max_dt_ms=..` on RECV lines.
CARET reports frequency + period + jitter for exactly this reason.

## R6. Output & ecosystem (small, independent)

- `ROS_TOPIC_STATS_FORMAT=jsonl` — one object per window; golden-file tests for both formats.
- Callback names via `rclcpp_callback_register` for human-readable labels.
- Opt-in exporter (Prometheus/OTel) as a **sidecar reading the log** — keeps the probe itself
  network-zero.
- Validate on `rmw_zenoh` (no DDS at all) and CycloneDDS+iceoryx SHM; add support-matrix rows.
  Hooks sit above rmw, so both should work unmodified — worth proving.
- `ROS_TOPIC_STATS_QUIET=1` to silence the stderr banner for stderr-parsing deployments.

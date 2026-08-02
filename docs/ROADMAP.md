# Roadmap

Defects live in [KNOWN_ISSUES.md](KNOWN_ISSUES.md); this file tracks **features and positioning
work** — things the probe doesn't do yet. Research grounding: ros2_tracing design docs + paper
(Bédard et al., RA-L 2022), CARET (Tier IV), ROS 2 distro release notes.

Section numbers are stable (referenced from commits and CHANGELOG), so they are **not** priority
order. Current order of work: **R5** (next — it closes a soundness hole in the R1 alerting we
just shipped), then R6, then R2 if R5 hasn't already covered the need.

## R1. Expected-rate spec + alerting — turns the logger into a monitor — **DONE**

> Shipped: `ROS_TOPIC_STATS_EXPECTED` spec (documented YAML subset, no YAML lib in the probe),
> flush-time `WARN TOPIC` / `WARN NODE` lines with first-window grace, and the no-ROS
> `pulse-check` CLI (exit 0/1/2) re-deriving verdicts from raw log rates. See README
> "Expected-rate alerting".

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

## R2. Timer liveness — **rescoped after research (2026-08-02)**

> Was: "timer & service rate monitoring", `TIMER /node period_ms=20 actual_hz=49.8`, pitched as
> control-loop monitoring. Research killed that framing. Ships as **liveness**, after R5, and
> only if R5's gap detection doesn't already cover the need.

**Why the original pitch doesn't hold.** The canonical ROS 2 control loops are not `rclcpp`
timers, so these tracepoints are structurally blind to them:

| Stack | Loop driver | Emits timer tracepoints? |
|---|---|---|
| `ros2_control` `ros2_control_node` | raw `std::thread` + `sleep_until` | no |
| Nav2 `controller_server` | `nav2::Rate` in the action-server thread | no |
| MoveIt Servo | `rclcpp::WallRate` in a `while` loop | no |

"Catches a wedged node whose subscriptions still drain" also needs a `MultiThreadedExecutor`
with separate callback groups — under a single-threaded executor a wedged timer blocks the
subscriptions too, so they do *not* still drain. And `ros2_control` on Jazzy+ already ships
periodicity avg/min/max/stddev with `/diagnostics` thresholds and an overrun count; competing
there on a mean Hz is a losing comparison.

**What survives.** The tracepoints are real and cheap, and they fix a defect we already
document in the README: *"a genuinely-alive but idle node (e.g. a pure timer/service node with
no topic traffic) therefore reads as quiet."* Timer activity makes `NODE` liveness honest and
distinguishes "topic silent because the node is idle" from "topic silent because it is wedged".

Scope if built: emit `TIMER` only for timers that resolve to **no publisher** (the
non-redundant case — motor drivers writing CAN/serial on a timer, watchdogs, service callers);
a timer that publishes already reports its rate as the topic's rate. Carry the declared period
from `rcl_timer_init` as *metadata only*, never as an auto-derived expected rate.

Corrections to the original entry, all verified against upstream source:

- **The listed tracepoints cannot produce `actual_hz`.** `rcl_timer_init`,
  `rclcpp_timer_callback_added` and `rclcpp_timer_link_node` fire once each and give period,
  handle and node. There is **no per-fire timer tracepoint anywhere in ROS 2** — `rcl_timer_call`
  does not exist and `rcl/src/rcl/timer.c` has none. The per-fire signal is `callback_start`,
  fired from `GenericTimer::execute_callback()`, **which the probe already interposes**. Timer
  fires reach `onCallbackStart` today and are dropped by the `kNotASubscription` sentinel. That
  makes R2 three init-time interposers, not a new counting path — cheaper than written.
- **~~Bonus: retires the negative-cache sentinel~~ — struck; it is a net cost.** The sentinel
  path early-returns; counting replaces that with a `fetch_add`. The machinery cannot be retired
  regardless, because lazy subscription resolution still needs the "not yet populated, retry"
  case. Its present cost is ~0.0001% of a core, unmeasurable against the ±0.7% SEM of our own
  benchmark. `docs/issues/issue-3-callback-lock-storm.md` already considered and rejected
  eager timer resolution on these grounds.
- **`TIMER /node period_ms=20 actual_hz=49.8` has no unique key** — two 20 ms timers on one node
  are indistinguishable. Needs the `rclcpp_callback_register` symbol (see R6) or an index.
- **The declared period is not a reliable expected rate.** Sim time (`create_timer` with a
  `RCL_ROS_TIME` clock), `rcl_timer_exchange_period` and `rcl_timer_cancel` are all untraced, so
  a cancelled or one-shot timer would read `actual_hz=0.000` forever.
- **rclpy timers produce orphans.** `rcl_timer_init` fires, but `rclcpp_timer_link_node` and
  `callback_start` never do (ros2_tracing#15) — period, no node, no fires. Must be dropped, not
  reported at 0 Hz.
- **Services: cut.** `rclcpp::ParameterService` creates six services on every node, so an
  unfiltered 10-node process emits 60 near-always-zero lines per window. Request rates are
  bursty and usually zero; `min_hz` on them is mostly noise. Revisit only on user demand.

Tracepoint availability, verified per branch — all six exist with identical signatures on
humble / iron / jazzy / kilted / rolling, so the original "all exist on Humble" was right.

## R3. Distro matrix: Jazzy / Kilted / rolling — **mostly DONE**

- ✅ CI: blocking matrix humble+jazzy+kilted on official `ros:<distro>` images; rolling
  observational lane fixed (PR #16 — it immediately caught the `ament_target_dependencies`
  removal on rolling).
- ✅ Interposer signatures verified per distro: all hooked events plain-called by rcl/rclcpp on
  jazzy/kilted; full suite green on both.
- ✅ `rclcpp_intra_publish` hooked → **publish-side intra** counts on Iron+ (additive `PUB`
  line; symbol exported-but-never-called on Humble).
- ⬜ Remaining: integration assertion that the probe and a **live LTTng session** coexist
  (we forward via `dlsym(RTLD_NEXT)`, so both should fire — assert it). Testable on any distro
  whose binaries carry the lttng-ust backend: Iron+ out of the box, Humble only if ROS was
  rebuilt for it.

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

## R5. Gap / jitter visibility — **NEXT** (promoted 2026-08-02)

**Promoted above R2 because it fixes a soundness hole in R1, the feature we just shipped.**
A windowed mean cannot detect a stall, so `min_hz` alerting silently passes real faults:

> 50 Hz topic, 5 s window, rule `min_hz: 45`. The rule fires only below 225 messages, i.e. after
> **>0.5 s of dead time**. A 400 ms freeze — 20 lost cycles, catastrophic for a 50 Hz control
> loop — reports 46 Hz and stays green. Widening the window makes it worse: the same 2 s stall
> at a 20 s window averages to exactly 45.0 Hz and never fires. One 500 ms freeze and 500
> spread-out 1 ms hiccups are indistinguishable — both 45.0 Hz.

Max inter-arrival gap is the window-length-independent detector, and it covers the loops R2
structurally cannot: a `Rate`-driven `ros2_control` or Nav2 loop is invisible as a timer but
plainly visible in the gap statistics of the topics it publishes.

This is also what the field converges on — **nobody uses mean rate as the primary loop-health
signal**: cyclictest reports max latency, Prometheus exposes scrape-interval quantiles plus
`rule_group_iterations_missed_total`, `ros2_control` reports periodicity avg/min/max/stddev with
an overrun count, Nav2 logs per-miss events, CARET plots period and frequency histograms.
`diagnostic_updater::FrequencyStatus` is the one mean-only design in the survey, and it is the
weakest. Percentiles don't rescue a mean either: with 250 samples a single 2 s gap sits at the
99.6th percentile, so you need max, not p99.

Shape: `ROS_TOPIC_STATS_JITTER=1` (default OFF, cost documented), per-endpoint min/max
inter-arrival per window, and a `max_gap_ms:` rule in the expected-rate spec so `pulse-check`
can gate on it. Exact line grammar, spec extension, cross-log merge semantics and hot-path
design are being specified separately — the open questions are the clock cost, lock-free
min/max accumulation across callback threads, whether the first message of a window measures
its gap across the window boundary (it must — a stall that straddles a flush is exactly the
interesting case), and how a gap field merges in `pulse-check` (max-of-max, never a sum).

## R6. Output & ecosystem (small, independent)

- `ROS_TOPIC_STATS_FORMAT=jsonl` — one object per window; golden-file tests for both formats.
- Callback names via `rclcpp_callback_register` for human-readable labels.
- Opt-in exporter (Prometheus/OTel) as a **sidecar reading the log** — keeps the probe itself
  network-zero.
- Validate on `rmw_zenoh` (no DDS at all) and CycloneDDS+iceoryx SHM; add support-matrix rows.
  Hooks sit above rmw, so both should work unmodified — worth proving.
- `ROS_TOPIC_STATS_QUIET=1` to silence the stderr banner for stderr-parsing deployments.

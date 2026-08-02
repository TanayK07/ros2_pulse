# R5 design — gap visibility (`max_dt_ms`)

Implementation spec for ROADMAP R5. Written from research done 2026-08-02; every number below
was measured on this tree, and every claim about existing behaviour cites `file:line`.

## Problem

R1 shipped rate alerting. A windowed mean cannot detect a stall, so `min_hz` silently passes
real faults:

> 50 Hz topic, 5 s window, rule `min_hz: 45`. The rule fires only below 225 messages — i.e.
> after **>0.5 s of dead time**. A 400 ms freeze (20 lost cycles, catastrophic for a 50 Hz
> control loop) reports 46 Hz and stays green. Widening the window makes it worse, not better:
> the same 2 s stall at a 20 s window averages to exactly 45.0 Hz and never fires. One 500 ms
> freeze and 500 spread-out 1 ms hiccups are indistinguishable — both 45.0 Hz.

Max inter-arrival gap is the window-length-independent detector. It also covers the loops R2
structurally cannot see: a `Rate`-driven `ros2_control` or Nav2 loop emits no timer tracepoint,
but its stall is plain in the gap statistics of the topics it publishes.

Percentiles do not substitute for max. With 250 samples a single 2 s gap sits at the 99.6th
percentile — p99 discards precisely the fault. Max is not an approximation of what we want; it
is what we want.

## What ships

One statistic, per side, opt-in:

```
JITTER /scan pub max_dt_ms=51.284
JITTER /scan recv max_dt_ms=52.019
```

plus a `max_gap_ms:` spec rule and its WARN line:

```
WARN TOPIC /scan max_dt_ms=812.400 expected_max_gap_ms=60
```

Enabled by `ROS_TOPIC_STATS_JITTER=1`, **or** implicitly by any `max_gap_ms` rule in the spec.

## Decisions, and why

### Both sides, not RECV-only

The ROADMAP proposed RECV-only. Wrong, for three reasons:

- **rclpy is publish-side only.** `callback_start` is rclcpp-only (README "Limitations";
  ros2_tracing#15), so a Python node's stall is visible *only* on the pub side. RECV-only makes
  R5 structurally blind to every Python publisher.
- **Attribution.** pub gap large + recv gap large ⇒ producer wedged. pub gap small + recv gap
  large ⇒ transport or executor problem in the consumer. One side cannot distinguish these, and
  it is the first question an operator asks.
- **Humble has no intra publish tracepoint**, so an all-intra pipeline has no pub-side signal
  there. Recv is needed too.

### `max_dt_ms` only — `min_dt_ms` is cut

The ROADMAP proposed min *and* max. Cut min:

1. **No alert rule uses it.** It would be the only field on the line with no spec key.
2. **It is structurally corrupted on the pub side from Iron on.** One `publish()` fires both
   `rclcpp_intra_publish` and `rcl_publish` for the same message — the double-fire documented at
   `src/core/rate_spec.cpp` in `observedHz`. Both feed one dt series, so the sequence is
   `~1 µs, period, ~1 µs, period, …`. `max_dt_ms` is unaffected (still the period);
   `min_dt_ms` reads ~0.001 ms on a perfectly healthy topic, forever. Shipping a field that is
   always garbage on Jazzy-with-IPC is worse than not shipping it.
3. **Largely derivable.** 250 messages with `max_dt_ms=200` already tells you 249 were clumped.

Re-addable later at zero cost: the line grammar below is a new line kind, so appending a field
to it breaks nothing (§ Compatibility).

### No overrun / missed-cycle count

An overrun count needs an expected period. We have no ground truth for one, and every
derivation is unsound:

- **From the spec's `min_hz`** — breaks "measurable with no spec loaded", and `min_hz` is
  deliberately slack (18 for a 20 Hz topic), so the derived period is not the loop period.
- **From the window's own mean rate** — self-defeating. At 50 Hz with a 2 s stall in a 5 s
  window, 150 messages ⇒ derived period 33 ms against a true 20 ms. The threshold moves *away*
  from the fault in proportion to the fault's severity. At a total stall the count is 0 and the
  period is undefined. Event-driven topics (`/tf_static`) have no period at all, and this
  manufactures overruns from nothing.
- **From a global env threshold** — one threshold for per-topic periods; making it per-topic
  reinvents the spec, and the spec must not enter the hot path.

The alert never needed a count. `pulse-check` derives the boolean it actually wants
(`max_dt_ms > max_gap_ms`) offline.

### The open-gap fold — a hole in R5 as specified

An endpoint with **zero** messages in a window produces no inter-arrival pair, so `max_dt_ms` is
undefined and no line is emitted: the gap detector silently misses the *total* stall, the worst
case it exists to catch.

Fix: at flush, report `max(observed_max_dt, now - last_ts)`.

- Healthy 50 Hz topic: `now - last_ts` is uniform in [0, 20 ms], always ≤ the observed max. No
  change.
- Dead 3 s: reports 3000 → 8000 → 13000 ms across successive windows, monotonically growing —
  exactly what a watchdog wants.
- Never had a message: `last_ts == 0`, no line. Matches `shouldEmitRecv`'s never-active
  suppression and issue #7's "declared-but-silent is noise".

This is the direct analogue of KNOWN_ISSUES #12: a dead endpoint must read a growing number,
not vanish.

### …and it must be suppressed in the exit window

`interposers.cpp` already notes that rclcpp teardown ends traffic before the process exits. If
the executor stops spinning 300 ms before `atexit` runs, the folded open gap is 300 ms of
*healthy shutdown*, and `max_gap_ms: 100` fires on every clean stop — deterministically.

Thread the existing exit flag through: `snapshot(window_s, /*fold_open_gap=*/!exiting)`. The
exit window then reports observed inter-message dts only, or emits no `JITTER` line.

Note this is a *different* problem from R1's exit-flush bug, and R1's fix does not cover it.
R1's bug was that `count/window_s` over a sliver is a one-sample estimate. That does not apply
to max, which is window-length-independent by construction — a short window yields a max over
fewer samples, conservative-low, never spuriously high. The artifact here comes from the fold,
not the window length. R1's `!exiting` alert guard needs no change and generalizes for free.

### A `max_gap_ms` rule auto-enables measurement

A declared rule that is silently not checked is the failure mode this project fights (zero-rule
spec → warn+disable; malformed spec → warn+disable; the banner reports `spec=` precisely
because "why no WARNs?" is otherwise invisible — all added in PR #23).

Setting `ROS_TOPIC_STATS_EXPECTED` with a gap rule is a *stronger, more specific* declaration of
intent than the absence of an env var:

```cpp
m_track_gap = envFlag("ROS_TOPIC_STATS_JITTER") || specHasGapRule(m_spec);
```

The env var keeps its meaning: "measure everything, I'm debugging." Banner reports it:

```
[ros2_pulse] active — interposing tracetools layer
  (out=/tmp/pulse.log, period=5.0s, spec=2 topics 1 nodes, jitter=on (1 max_gap_ms rule))
```

Wiring note: `m_registry` is declared before `m_spec` in `ProbeRuntime`, so a ctor *param*
cannot depend on the spec. Call `m_registry.setGapTracking(bool)` in the ctor **body** — safe,
because the singleton is not published until the ctor returns. Make the member
`std::atomic<bool>` with relaxed loads to keep the TSan lane clean.

In `pulse-check`, a spec that requires a gap the logs do not carry is **exit 2**, not 0 or 1.
Exit 0 would be a lie ("checked, healthy"); exit 1 would send someone debugging a stall that was
never measured. Exit 2 already means "cannot judge".

```
pulse-check: spec requires max_gap_ms for '/scan' (side recv) but the logs carry no
             JITTER line for it — was the probe run with ROS_TOPIC_STATS_JITTER=1?
```

A topic absent from *all* logs stays a violation at `max_dt_ms=inf` (consistent with
`missing_as_zero`; `formatBound` already renders `inf`).

## Hot path

### Cost — measured, not estimated

Paired order-alternated, N=15, 8 threads, on an AMD Ryzen 7 7435HS (clocksource `tsc`,
`constant_tsc nonstop_tsc`):

```
today            :   2.435 ns CPU/op
today + R5 jitter:  26.801 ns CPU/op
PAIRED DELTA     : +24.366 +/- 0.023 ns CPU/op (SEM)
```

The ROADMAP's "~20 ns" estimate was right. **~96% of it is the clock read alone** — clock+store
only is 27.27 ns vs 26.72 for full accumulation, so the accumulation design has essentially no
performance content. Pick it on correctness.

In context:

```
recv-only @4900 msg/s : 119.4 us CPU/s = 0.0119% of one core
project pooled SEM    : +/-0.7%
effect / SEM          : 0.066   (1/15th of one SEM)
paired trials to resolve at 2xSEM: ~6400  (~2 days per distro)
vs the ~51 us/msg the stack already spends: 0.048%
```

**Do not claim an end-to-end measurement.** Publish the microbench delta with its SEM and the
arithmetic, and state that the effect is below the resolution of `run_overhead_repeated.sh`.
That is stronger and more honest than a fabricated end-to-end number.

Disabled is genuinely free: **−0.011 ± 0.002 ns/op** for a plain `bool` member branch
(negative sign is code-layout noise, not a speedup). No second code path, no function pointer.
Put `bool m_jitter` adjacent to `m_id`, which the hot path already loads on every call, so it
costs zero extra cache traffic.

### Clock choice: `steady_clock`, and the alternatives are rejected

Measured, min of 5 × 50M iterations, pinned:

| Clock | ns/call | `clock_getres` | Verdict |
|---|---|---|---|
| `steady_clock::now()` | 21.6 | 1 ns | **chosen** |
| `clock_gettime(CLOCK_MONOTONIC)` | 21.3 | 1 ns | same thing, no gain |
| `CLOCK_MONOTONIC_COARSE` | 5.3 | **1 ms** | rejected |
| `__rdtsc()` | 8.2 | — | rejected |
| forced `syscall()` | 203.3 | — | (proves vDSO) |

vDSO confirmed three ways: `strace -c` over 2000 calls shows **zero** `clock_gettime` syscalls,
`linux-vdso.so.1` is mapped, and the forced-syscall leg is 9.6× more expensive. A syscall-backed
clock would put R5 at ~205 ns/msg and change the verdict — not the case here, but worth
re-checking on aarch64 (see Orin note below).

`CLOCK_MONOTONIC_COARSE` is 4× cheaper and disqualified: its 1 ms granularity destroys the
signal. Driving the bursty stream the ROADMAP names:

```
burst 0.5ms x4 then 18ms stall (~100Hz):
  CLOCK_MONOTONIC        min_dt=0.500 ms  max_dt=18.000 ms   dt==0:  0/199
  CLOCK_MONOTONIC_COARSE min_dt=0.000 ms  max_dt=18.001 ms   dt==0: 80/199
```

40–60% of samples collapse to `dt == 0`. Save 16 ns, lose the feature.

`__rdtsc()` is 3× cheaper but needs ns calibration, got *worse* under contention (255.7 vs
152.1 ns CPU/op on a shared endpoint — the faster read just lets threads pound the shared line
harder), and does not exist on aarch64, which `test/orin/` targets.

`steady_clock` is `CLOCK_MONOTONIC`, confirmed from the libstdc++ disassembly
(`mov $0x1,%edi` before `clock_gettime@plt`) — not `_RAW`. So: immune to NTP steps, and NTP
slewing only rate-adjusts it.

### Accumulation

```cpp
// sTopicCounter — integer ns, not double: lock-free everywhere and exact
std::atomic<uint64_t> last_ts{0};   // 0 = no message ever. NEVER reset at flush.
std::atomic<uint64_t> max_dt{0};    // reset per window by snapshot().
```

```cpp
if (m_jitter) {
    const uint64_t now = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    // exchange, not load+store: guarantees exactly ONE thread consumes each prev value,
    // so two threads can never derive two dt's from the same predecessor.
    const uint64_t prev = ctr->last_ts.exchange(now, std::memory_order_relaxed);
    if (prev != 0 && now > prev) {          // reordered arrival -> DISCARD
        const uint64_t dt = now - prev;
        uint64_t cur = ctr->max_dt.load(std::memory_order_relaxed);
        while (dt > cur && !ctr->max_dt.compare_exchange_weak(
                   cur, dt, std::memory_order_relaxed)) {}
    }
}
```

`std::atomic::fetch_max` is C++26; verified absent under `-std=c++17/20/23` on GCC 11.4, and the
project is C++17. The CAS loop is the only portable form, and it costs nothing in steady state
because it almost never fires — new extrema follow the harmonic number, so at a 5 s window and
100 Hz you pay ~14 CAS out of 500 messages (~2.8%). Even at a pathological 405 ns/CAS with
8-way contention that is ~11 ns/msg amortized; uncontended it is ~0.2 ns/msg.

**The `now > prev` guard is mandatory.** `now` is sampled before the exchange, so two threads
can exchange out of order; without the guard the unsigned subtraction underflows to ~1.8e19 ns
and poisons `max_dt` permanently.

**Measured failure mode** (ground truth from the true global order of the same timestamps):

```
8 threads, tight   : 0.574% samples discarded, max over-reported +5.0%
8 threads, ~1us    : 7.477% discarded,          max exact
2 threads          : 9.744% discarded,          max over-reported +1.5%
1 thread           : 0.000% discarded,          exact
```

So: samples are **dropped, never garbage and never negative**, and the residual error is
`max_dt` over-reported by ≤5% (a losing thread's exchange regresses `last_ts`, so the next
message measures from further back). Erring high on a stall detector is the safe direction.

Mitigating: rclcpp's default callback group is MutuallyExclusive, so a `MultiThreadedExecutor`
does not run one subscription's callback concurrently with itself — the recv-side race needs an
explicitly Reentrant group. The pub side is genuinely multi-threaded.

Rejected: **per-thread accumulation in the TLS cache**. Not faster (27.37 vs 26.98 ns), and it
breaks the cache's contract — `t_tls_cache` is direct-mapped and documents that "a same-slot
collision merely degrades that key to the shared-lock path (it is a cache, not a map)". Put
*state* there and a collision becomes a **correctness** event: two endpoints sharing slot
`((p>>4)^(p>>10))&255` would silently interleave their dt streams. Also, thread death before
flush loses the window, and `snapshot()` has no path to another thread's `thread_local`.

### `last_ts` must survive the window boundary

`snapshot()` resets `max_dt` to 0 but must **not** reset `last_ts`. If it did, the first message
of every window would find `prev == 0` and skip its dt — silently discarding exactly the
cross-boundary stall R5 exists to catch.

Consequence: `max_dt` is retrospective without the open-gap fold — a topic that stalls at T and
resumes two windows later reports the big gap in the *resumption* window. The fold (above)
fixes that, which is why it is not optional.

### Emit iff a sample exists

Emit the field iff `last_ts != 0`, never based on a message count.

- 0 messages, never active: no line.
- 0 messages, previously active: the fold produces a growing number. Line emitted.
- 1 message: exactly one dt, from the previous window's last message. Correct and useful.
- Do **not** test `max_dt != 0` — two arrivals inside one 20 ns clock tick yield a legitimate
  `dt == 0`.

## ~~Land first, separately: `alignas(64)`~~ — measured, rejected

**This section is kept as a negative result so nobody redoes the analysis. Do not do this.**

The claim below (a pre-existing false-sharing bug worth 4.5×) came out of the design research and
**does not reproduce**. Three measurements killed it:

```
8 threads, own endpoint      packed 1.967 / aligned 1.720 ns/op   1.14x  (SEM 0.012 / 0.023)
8 threads, ADJACENT endpoints packed 1.981 / aligned 2.015 ns/op  0.98x  (SEM 0.113 / 0.014)
8 threads, full R5 hot path  packed 29.07 / aligned 28.75 ns/op   1.011x (SEM 0.37 / 0.35)
```

Why the premise was wrong:

1. **The hot path touches ONE atomic per message, not four.** `onPublish` does
   `bucket.fetch_add(...)` on *either* `pub_intra` or `pub_inter`; `onCallbackStart` likewise on
   one recv bucket. "The four count-atomics straddle a line" is therefore irrelevant to the
   per-message path — it would only matter to code touching several at once, which is `snapshot()`,
   once per window.
2. **The allocator already separates them.** `sizeof` is 72 and the observed `make_unique` stride
   is a uniform **80 bytes**, which exceeds the 64-byte line, so consecutive counters' hot fields
   land on distinct lines anyway (measured: 8 adjacent endpoints → 8 distinct lines).
3. **With R5 the clock read dominates completely.** At ~21 ns for `steady_clock::now()`, a layout
   difference of ~0.3 ns is inside the SEM.

So the 63/128 "straddling" figure is real but inert, and `alignas(64)` buys nothing for 56 extra
bytes per topic. R5 adds its fields to `sTopicCounter` as plain members with no layout change.

The original reasoning, retained for the record:

`sizeof(sTopicCounter)` is 72 with alignof 8, and there is no alignment discipline anywhere
(`grep -c alignas` over `include/ros2_pulse/core/` and `src/core/` returns 0). Measured over 128
counters allocated exactly as `counterForTopic` does:

```
current (72 B)   : count-atomics straddling a cache line 63/128 | lines shared by >1 endpoint 95/161
R5 inline (96 B) : straddling 64/128 | jitter fields on a different line from counts 64/128
R5 alignas(64)   : straddling  0/128 | lines shared by >1 endpoint 0/256
```

**63 of 128 counters currently have their four count-atomics split across a cache line.** That
is a pre-existing latent repeat of the KNOWN_ISSUES #15 class, not something R5 creates — but
R5 makes it worse by putting the jitter fields on a *second* line for half of all endpoints.

Measured cost, 8 threads on neighbouring endpoints: baseline 1.94 → 8.79 ns CPU/op (**4.5×**);
with jitter 26.98 → 50.16 ns (1.9×).

```cpp
struct alignas(64) sTopicCounter {
    // hot line: everything the per-message path touches, one 64 B line, never straddling
    std::atomic<uint64_t> pub_inter{0}, pub_intra{0}, recv_inter{0}, recv_intra{0};
    std::atomic<uint64_t> last_ts{0}, max_dt{0};
    // cold: flush-path only
    std::string topic;
    bool recv_endpoint_seen{false};
};
```

128 B/topic vs 72 — 12.8 KB vs 7.2 KB at 100 topics. Irrelevant. `make_unique` honours
over-aligned `new` in C++17.

Do **not** side-allocate the jitter fields: it saves a few KB when jitter is off and buys a
dependent pointer load on the hot path before the clock read.

~~This is independently justified and independently testable, so it lands as its own `perf:`
commit ahead of R5.~~ **Retracted — see the measurements at the top of this section.**

## Compatibility

**New line kind, not appended fields.** Appending to `RECV` is safe for `sscanf` (it stops after
its conversions), but `test/integration/probe_harness.py:32-34` anchors with `$`:

```python
_TOPIC_RE = re.compile(r"^TOPIC (\S+) ([\d.]+)$")
_PUB_RE   = re.compile(r"^PUB (\S+) inter=([\d.]+) intra=([\d.]+)$")
_RECV_RE  = re.compile(r"^RECV (\S+) inter=([\d.]+) intra=([\d.]+)$")
```

Verified: legacy line matches, appended line does not. That harness backs nine integration
tests; under `JITTER=1` every RECV assertion in the suite would report "topic never reported".
Meanwhile `test/integration/test_probe_integration.py:46` uses the same pattern *without* `$`
and survives. Two parsers in one repo, divergent anchoring, one broken by appending — a fair
sample of what downstream parsers look like.

The `JITTER` line is invisible to every pre-R5 parser:

| Parser | Behaviour |
|---|---|
| `parseLog` sscanf chain (`log_reader.cpp`) | no branch matches; falls to the documented "anything else … is deliberately skipped" |
| `probe_harness.parse_windows` | loop with no `else`; unmatched lines dropped |
| `test_probe_integration._parse_recv` | `^RECV` prefix |
| `test_rate_spec._warn_lines` | `startswith("WARN ")` |

Verified no sscanf collision: `sscanf("JITTER /scan recv max_dt_ms=21.3", "NODE %511s")` → 0.

**Precision `%.3f`, diverging from the `%.6f` Hz convention.** Meaningful resolution is bounded
by the clock read (~21 ns) and scheduler noise, so 3dp = 1 µs sits at the physical floor.
`%.6f` on a stall prints `5000.000000` — three guaranteed-zero bytes per line. No parser assumes
field width. Document the divergence in `window_format.hpp` so it reads as a decision.

**No `formatWindow` signature change.** Gate emission on the stat's own flag, exactly as `PUB`
is gated on `pub_intra_count > 0`. "Jitter disabled" is then *expressed as* the flag being
false, which makes byte-identical legacy output true by construction rather than by a parallel
code path.

## Spec grammar

```yaml
topics:
  /scan:    {min_hz: 45, max_gap_ms: 60, side: recv}   # slowdown AND freeze
  /cmd_vel: {max_gap_ms: 30, side: pub}                # gap-only — a valid rule
  /points:  {min_hz: 25, transport: intra}             # unchanged
```

- `sRateRule` gains `double max_gap_ms{infinity()}`, mirroring `max_hz`.
- The `has_min && has_max` check **relaxes** to "at least one of min_hz, max_hz, max_gap_ms".
  Forcing every gap rule to carry a redundant `min_hz` would have the tool contradicting its own
  documentation. `RejectsRuleWithoutBounds` (`{side: pub}`) still fails.
- `applyRuleItem` already carries four `bool&` out-params; a fifth makes seven. Replace with a
  small `struct sSeenKeys` — mechanical, keeps the duplicate-key contract intact.
- Rename `parseHz` → `parseNonNegative`. It is really "whole-token, non-negative, finite
  double"; leaving a function named `parseHz` parsing milliseconds is how things rot.
- **Reject `max_gap_ms <= 0`** — same class as `min_hz > max_hz`.
- Dedup key stays `(name, side, transport)`. `/scan: {min_hz: 18}` + `/scan: {max_gap_ms: 60}`
  remains a duplicate; merging them into one rule is better UX and the existing error already
  says so.
- `transport` has no effect on a gap check (one accumulator per side, transport-merged), so a
  rule where `transport` is the only companion of `max_gap_ms` is a hard error — consistent with
  the parser's stance on rules that constrain nothing.
- A rule with both `min_hz` and `max_gap_ms` can emit **two** WARN lines in one window. Rate
  first, then gap, preserving spec order.

## `pulse-check`

`mergeStat` stops being a uniform sum:

| Field | Merge | Why |
|---|---|---|
| `pub_inter_hz`, `pub_intra_hz`, `recv_inter_hz`, `recv_intra_hz` | sum | unchanged |
| `recv_endpoint_seen` | OR | unchanged |
| `pub_max_dt_ms`, `recv_max_dt_ms` | **max** | worst gap any endpoint saw |
| `has_pub_max_dt`, `has_recv_max_dt` | **OR** | drives the exit-2 path |

Max-of-max is right: log A's subscriber sees 20 ms, log B's sees 800 ms ⇒ the publisher was fine
and B's executor wedged. That is a real fault, in B. The gap field is also *better behaved*
across logs than the rates — summing recv across K subscriber logs gives K× the publish rate (a
documented wart that forces `max_hz` onto `side: pub`), whereas max is invariant to log count.

`--skip-last` needs no change; `windows[size-2]` is a normal full window with the fold applied.
First-window grace is unnecessary for gaps (the probe attaching mid-stream still measures
genuine dts) but keep the guard for uniformity.

## Limitation to document, not hide

Counters are keyed by **topic name**, so two publishers of one topic in one process interleave
into one dt series — which can mask one of them stalling. Per-topic-per-side is what the spec
keys on anyway, but this must be documented as a limitation rather than quietly delivered as
"per-endpoint".

## Implementation order

1. ~~`perf: align sTopicCounter to a cache line`~~ — **dropped**, measured at 1.01x (inside the
   SEM). See the rejected-prerequisite section.
2. `feat(jitter): per-endpoint max inter-arrival gap` — `last_ts`/`max_dt`, the `m_jitter` gate,
   the open-gap fold with exit suppression, `JITTER` lines, log-reader branches.
3. `feat(rate-spec): max_gap_ms rules` — spec grammar, WARN line, auto-enable, `pulse-check`
   max-of-max merge and the exit-2 unmeasured path.

## Test plan

Unit files land in both lanes with no CMake edit (`test/unit` is globbed); integration files need
an `ament_add_pytest_test` entry.

`test/unit/test_window_format.cpp`
- `JitterDisabledIsByteIdenticalToLegacy` — populate the fields, leave the flag false, assert the
  existing golden bytes verbatim. **The additive-convention proof.**
- `JitterLinesFollowTheirSideLines` — golden bytes, both sides, pinning placement and `%.3f`.
- `StalledEndpointEmitsGrowingMaxDt` — proven endpoint, zero counts, fold on. The KNOWN_ISSUES
  #12 analogue.

`test/unit/test_log_reader.cpp`
- `RoundTripsJitterLines`, `PreR5LogParsesWithoutJitterFields`, `UnknownJitterSideIsIgnored`.

`test/unit/test_jitter_accumulator.cpp` (new)
- `MaxSpansWindowBoundary` — the load-bearing one.
- `OpenGapFoldedAtFlush`, `OpenGapNotFoldedWhenExiting`.
- `FirstEverMessageProducesNoSample`, `MaxResetsPerWindowButLastTsDoesNot`.
- `DisabledReadsNoClock` — inject a counting clock so "default OFF costs nothing" is *tested*.

`test/unit/test_rate_spec.cpp`
- `MaxGapMsAlone`, `MaxGapMsWithRateBounds`, `RejectsNonPositiveMaxGap`,
  `RejectsRepeatedMaxGapKey`, `RejectsTransportOnGapOnlyRule`, `StillRejectsRuleWithNoConstraint`.
- `MaxGapExactWarnLine`, `RateAndGapViolationsBothFire`, `UnmeasuredGapReportedSeparately`,
  `MissingTopicGapIsInfinite`, `GapRuleRespectsSide`.

`test/stress/test_concurrency.cpp`
- `JitterMaxIsRaceFree` — N threads, known dts; covers the exchange partition and the
  `now > prev` guard under TSan.

`test/integration/test_jitter.py` (new) + a `stall_pub <stall_ms> <at_s>` mode in
`test_nodes.cpp` — a 20 ms wall-timer talker that calls `sleep_for(stall_ms)` **inside the
callback**. Under the default single-threaded spin this blocks the executor, so publishing stops
for exactly that long. Deterministic, unlike killing a process mid-run: `sleep_for` guarantees
*at least* the duration, so `max_dt_ms >= stall_ms` always, and because `last_ts` survives the
boundary the first message after the stall carries the full dt however it straddles a flush.

- **`test_gap_detected_where_mean_rate_passes`** — the R5 thesis as an executable test, and the
  most valuable one here. 50 Hz, period 5 s, run 16 s, 800 ms stall at t=7 s. The stalled window
  measures ~42 Hz. Spec `{min_hz: 40, max_gap_ms: 100, side: pub}`. Assert **zero**
  `WARN TOPIC /chatter hz=` anywhere *and* ≥1 `max_dt_ms` WARN ≥ 800. Guard the setup by also
  asserting the stalled window's rate is ≥ 40, or the test proves nothing.
- `test_jitter_off_by_default_no_lines`, `test_jitter_lines_have_plausible_values`,
  `test_gap_rule_auto_enables_jitter`, `test_exit_window_gap_not_judged_and_not_folded`.
- `test_pulse_check_gates_on_max_gap`, `test_pulse_check_exit_2_when_spec_needs_jitter_but_log_lacks_it`.
- `test_pulse_check_merges_max_of_max` — two hand-written logs, A=600 ms, B=700 ms, spec 1000 ms.
  Max-of-max = 700 ⇒ exit 0; a sum would be 1300 ⇒ exit 1. A clean discriminator.

`bench/hotpath_bench.cpp` gains a jitter leg, measured with the paired order-alternated harness.

**Measure on Orin before publishing a number.** On some Tegra parts the `CLOCK_MONOTONIC` vDSO
falls back to a syscall (~hundreds of ns), which would change the cost story on exactly the
platform this probe targets. `test/orin/` exists for this.

Also update: `probe_harness.py` gains a `_JITTER_RE` **deliberately not `$`-anchored**, with a
comment saying why — that is what makes a later `min_dt_ms` append free.

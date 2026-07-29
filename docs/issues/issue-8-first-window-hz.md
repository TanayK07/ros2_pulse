# Issue #8 — First window reports up to ~2× Hz

Research note for the fix on branch `fix/first-window-hz`.
Addresses [`KNOWN_ISSUES.md` #8](../KNOWN_ISSUES.md) (High, correctness).

## Problem

The first stats window a user sees can report roughly **twice** the true rate. With a 50 Hz
publisher and `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=1.0`, the first window prints ~100 Hz. For a
tool whose entire output is "Hz per topic", the first number being 2× wrong reads as "topic
overrunning" and poisons any consumer that alerts on rate bounds. The accuracy harness has been
*dodging* the bug rather than documenting it: `probe_harness.measured_rate()` silently drops the
first window as "ramp-up".

Two independent defects compound:

### (a) Timer first-fire off-by-one

`Timer::runThread()` (`src/core/timer.cpp:22-33`):

```cpp
auto end_time = std::chrono::steady_clock::now() + m_interval;   // t = 1×interval
while (m_running) {
    end_time += m_interval;                                      // t = 2×interval  <-- BUG
    while (m_running && m_cv.wait_until(lock, end_time) != std::cv_status::timeout) {}
    if (m_running) m_func();
}
```

`end_time` is initialized to one interval ahead **and then advanced again** before the first
wait, so the first callback fires at `t ≈ 2 × period`. Every later period is correct (the
`+=` keeps an absolute, drift-free cadence — that part is right; only its placement is wrong).

### (b) Nominal window denominator

`ProbeRuntime::flush()` passes the **configured** period to the registry —
`m_registry.snapshot(m_period_s)` (`src/probe/interposers.cpp:75`) — and prints the same
constant in the header (`window_s=%.3f`, `:86`). Elapsed time is never measured. So:

- the doubled first window (defect a) is divided by *one* period → ~2× Hz;
- any later window stretched by flush latency or scheduler jitter is silently skewed;
- the `window_s=` field in the output is an assertion, not a measurement.

Counts start accumulating at the **first tracepoint** (probe construction), which precedes the
timer start — so even with (a) fixed, dividing by the nominal period misattributes the
startup-lag portion of the first window.

## Why both fixes, not just one

Either fix alone happens to make the first-window number roughly right: (a) alone shrinks the
first window to ~1 period; (b) alone divides the doubled window by its true ~2 s length. Landing
only one would leave a latent defect that the other masks — (a) without (b) still mis-reports
any window whose real length deviates from the configured period; (b) without (a) still delays
the first data point to 2× the configured period (operator-visible startup latency).

## Fix

1. **Timer cadence** (`src/core/timer.cpp`): advance `end_time` *after* running the callback:

   ```cpp
   auto end_time = std::chrono::steady_clock::now() + m_interval;
   while (m_running) {
       while (m_running && m_cv.wait_until(lock, end_time) != std::cv_status::timeout) {}
       if (m_running) m_func();
       end_time += m_interval;
   }
   ```

   First fire at 1×interval; absolute-deadline cadence (no drift) preserved.

2. **Measured window** (`src/probe/interposers.cpp`): `ProbeRuntime` stamps
   `steady_clock::now()` when the probe starts (`ensureStarted`) and at every flush; each flush
   computes `elapsed = now - m_window_start`, advances the stamp, passes `elapsed` to
   `snapshot()` and prints it as `window_s=`. The header field becomes truthful; the output
   format is unchanged. `steady_clock` (monotonic) is used for the window length — the
   wall-clock `ts_ns` field keeps `system_clock` for log correlation, as before.

No registry change: `snapshot(double window_s)` already takes the denominator as a parameter
(and keeps its `w <= 0 → 1.0` guard). Single responsibility stays intact — the registry counts,
the runtime owns time.

## Regression tests (red on current `main`)

- `unit(Timer): FirstCallbackArrivesWithinOneInterval` — 400 ms timer must first fire in
  `[0.5×, 1.6×]` interval. Current code fires at 2× → red. The lower bound also rejects a
  degenerate "fire immediately" implementation (a ~0-length first window would break Hz).
- `unit(Timer): SteadyCadenceAfterFirstFire` — ≥3 fires of a 300 ms timer within ~1.05 s
  (current code: 2). Also bounds above (≤4) so immediate-fire cannot pass.
- `integration: test_accuracy_first_window_not_inflated` — the **first** window of the intra
  node must report ≤ 50 Hz × (1+TOL). Current code: ~100 Hz → red.
- `integration: test_window_header_reports_measured_elapsed` — `window_s` in each header must
  agree with the `ts_ns` delta between consecutive headers (guards the measured-window plumbing;
  green before and after, prevents regression to a constant).

## Interactions

- **PR #6 (per-process output)** rewrites `flush()` into `window_format`. The measured-window
  stamp lives in `ProbeRuntime` either way; merge order is flexible, conflict is textual only.
- `probe_harness.measured_rate()` keeps dropping first/last windows (last is genuinely truncated
  by SIGTERM); after this fix the first window is accurate, but dropping it stays harmless.

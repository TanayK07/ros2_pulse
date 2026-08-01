# Issue 15 — TLS hot-path cache aliases on allocator strides → ~90% miss rate → +2–4% workload CPU

**Severity.** Medium (efficiency / headline claim)
**Area.** `src/core/topic_registry.cpp` (thread-local cache), benchmark honesty
**Found.** 2026-08-01, per-distro end-to-end overhead runs for the launch benchmarks
(`bench/run_overhead_repeated.sh` in ros:humble / ros:jazzy / ros:kilted containers).

## Symptom

The end-to-end stress benchmark (30 light publishers @100 Hz + 8 heavy @50 Hz, a 38-topic
subscriber farm on a `MultiThreadedExecutor`, and a 15-topic intra-process farm; workload CPU
seconds via `getrusage(RUSAGE_SELF)`) showed the probed run consistently **+2–4% CPU over
baseline** across distros (jazzy +2.5% with ours-higher in 5/6 paired trials; kilted +3.8%,
6/6) — orders of magnitude more than the ~3–12 ns/op the isolated hot-path microbench
predicts (~10 k events/s × 12 ns ≈ 0.005% of the workload's CPU).

## Isolation (all on ros:jazzy, same stress workload)

Four controlled experiments, run interleaved-per-trial to cancel machine drift:

1. **Arm ladder** (N=3): baseline / null shim (same 8 exported symbols, empty bodies) /
   probe with `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=60` (flush timer never fires during the
   run) / probe with `PERIOD=2.0`. Null shim ≈ baseline (−0.3%) — LD_PRELOAD interposition
   itself is free. No-flush probe already +3.3% — **the cost is in the counting hot path,
   not the flush side.** (The flush arm added ~+1.7pp, inside noise.)
2. **Paired high-power test** (N=10, DUR=10 s, arm order alternated per trial): shipped code
   vs baseline = **+0.074 s ± 0.039 (SEM)** on a ~2.87 s baseline → **+2.6%**, positive in
   8/10 trials. The delta is real, not run-to-run noise.
3. **Counter evidence** (debug build printing `sharedLockLookups()` at exit): under the bench
   workload the shipped 16-slot cache took the shared-lock fallback on **31,926 of ~35,500
   events** in the publisher farm (~90% miss), 21,128 in the subscriber farm, 21,839 in the
   intra farm. ~75 k misses over the run; at a contended-miss cost of ~1 µs (rw-lock atomic
   RMW + shared counter cacheline bouncing across 16 executor threads) that is ~75 ms —
   matching the measured +74 ms.
4. **Candidate fix, same paired test** (256 slots + higher-bit-mixing hash): misses collapse
   to 6,509 / 3,080 / 2,028 (~85–90% reduction) and the paired delta becomes
   **−0.014 s ± 0.051 (SEM)** — statistically zero.

## Root cause

Two compounding design points in the direct-mapped thread-local cache
(`src/core/topic_registry.cpp`):

- **Hash aliases on uniform allocator strides.** Slot = `(ptr >> 4) & 15`. Same-type
  publisher handles come out of the allocator at a near-uniform stride; at the common
  64-byte stride the slot index advances by 4 (mod 16), so **38 live keys land on only 4
  distinct slots** and evict each other cyclically — the steady-state miss rate approaches
  100% regardless of working-set size vs capacity.
- **16 slots < realistic per-thread working set.** A `MultiThreadedExecutor` dispatches every
  callback/publish across all workers, so each worker thread's cache must eventually hold
  the whole process working set (38 publishers + 38 callbacks in the bench; a real
  perception/planning process is comparable). Even a perfect hash cannot fit 76 keys in 16
  slots.

Each miss is not the microbench's ~12 ns: it is `m_mu.lock_shared()` (an atomic RMW on a
cacheline all workers contend on), an `unordered_map` find, **plus a `fetch_add` on the
globally shared `m_shared_lock_lookups` observability counter** — a second guaranteed
cross-core cacheline bounce per miss. Under burst arrival (30 messages delivered per 10 ms
timer tick, fanned across 16 threads) these serialize at ~µs each.

The microbench missed it because its legs are single-threaded over ≤4 keys — no aliasing, no
capacity pressure, no cross-thread cacheline traffic. (Issue #13 fixed alternation thrash for
the *few-key* case; this is the same failure mode one level up — realistic key counts at
realistic allocator strides.)

## Fix

- Grow the cache to **256 slots** (`kTlsSlotMask = 255`): 256 × 24 B = 6 KB TLS per thread,
  zero-initialized (no dynamic TLS init), covering realistic per-thread working sets.
- Mix higher address bits into the slot index — `((ptr >> 4) ^ (ptr >> 10)) & 255` — so
  uniform strides no longer alias (stride-64 keys now map to distinct slots instead of 4).

Not chosen (YAGNI): set-associativity / LRU (miss rate after the fix is already ~0 in the
regression pattern and ~10–18% under the full bench, at which point the residual cost is
below measurement noise), per-thread miss counters (the shared counter only bounces when
misses are frequent — fixing the miss rate fixes the counter too).

## Regression test

`test/unit/test_tls_cache.cpp` gains a publisher-farm leg: 38 handles at a 64-byte stride
(the allocator pattern that aliased), round-robin published for 100 rounds after warm-up;
asserts the shared-lock fallback stays under 10% of accesses. Red on the 16-slot cache
(~3,800 misses = 100%), green with the fix (0 misses — the mixed hash maps the stride-64
farm to fully distinct slots).

## Verification

- Unit red→green above; existing exact-total and alternation tests unchanged.
- Fast / TSan / ASan+UBSan standalone lanes.
- Paired end-to-end bench (N=10, order-alternated) on ros:jazzy: delta −0.5% ± 1.8% (SEM),
  statistically zero; per-distro numbers re-run for `bench/RESULTS.md` on the fixed probe.

## Outcome addendum (post-merge, PR #19)

The aliasing itself is fixed — regression red→green (3800/3800 fallbacks → 0), bench miss
counters down 85–91%, microbench alt-4 from ~12 ns/op to 0.6–1.2 ns/op across distros.

The final per-distro paired benchmarks (N=10 each) still measure a **residual ≈+1.9% ± 0.7%
(pooled)** at the worst-case stress workload. Two further controls attribute it to neither
the hot path nor the flush side: flushing-vs-not is +0.027 s ± 0.033 (zero), and instrumented
flush time is 0.7–7 ms per process for the whole run. The single-experiment "statistically
zero" reading above was underpowered (its CI, −0.116..+0.088 s, never excluded a +0.05 s
effect) — pooling three independent distro runs resolves it. The residual is the diffuse
footprint of observation (chained real tracepoints, extra cache/TLB residency across all
executor threads) and is published as-is in `bench/RESULTS.md` rather than claimed away.

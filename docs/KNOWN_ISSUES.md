# Known issues & open concerns

Review of the current implementation (commit at time of writing). Each item lists severity,
evidence (`file:line`), impact, and a fix direction. Companion docs:
[ALTERNATIVES.md](ALTERNATIVES.md) (prior art + benchmarking) and
[TESTING_PLAN.md](TESTING_PLAN.md) (how each issue gets a regression test).

> **Status.** Round 1 (issues 1–7): all addressed with research-backed, test-first PRs
> ([docs/issues/](issues/) carries the per-issue research note; each PR ships the fix + its
> regression test, red→green). #1, #2, #4, #5, #7, #8 merged; #3 and #6 open (in review).
> **Round 2 (issues 8–14): found in a fresh audit of current `main` (2026-07-29)** — verified
> against the post-round-1 code; each gets the same one-issue-one-PR treatment, fix order by
> severity. Features/positioning work is tracked separately in [ROADMAP.md](ROADMAP.md).

## Summary

| # | Severity | Area | One-line | PR |
|---|----------|------|----------|----|
| 1 | **High** | correctness | Inter-process counter double-counts when publisher + subscriber share a process | [#4](https://github.com/TanayK07/ros2_pulse/pull/4) |
| 2 | Medium | correctness / claim | "Node liveness" is really "ever-initialized" — nodes never removed | [#7](https://github.com/TanayK07/ros2_pulse/pull/7) |
| 3 | Medium | efficiency | Unresolvable callbacks (timers/services) take the write lock on every call | [#2](https://github.com/TanayK07/ros2_pulse/pull/2) |
| 4 | Medium | robustness | Default output file is shared across all processes → interleaved writes | [#6](https://github.com/TanayK07/ros2_pulse/pull/6) |
| 5 | Low | robustness | `std::stod` on a bad env var throws out of a tracepoint → `std::terminate` | [#5](https://github.com/TanayK07/ros2_pulse/pull/5) |
| 6 | Low | docs | Dangling internal issue ref `#1204`; `#2911` premise needs a distro caveat | [#1](https://github.com/TanayK07/ros2_pulse/pull/1) |
| 7 | Low | behaviour | Idle topics print `TOPIC /x 0.000000` every window (undocumented) | [#3](https://github.com/TanayK07/ros2_pulse/pull/3) |
| 8 | **High** | correctness | ~~First window reports up to ~2× Hz: timer first-fire off-by-one + nominal (not measured) window denominator~~ | [#9](https://github.com/TanayK07/ros2_pulse/pull/9) |
| 9 | Medium | robustness | Exit-time UAF: tracepoints can run during/after `ProbeRuntime` static destruction | — |
| 10 | Medium | robustness | `fork()` without `exec`: child inherits `m_started` but no flush thread → counts never written | — |
| 11 | Low | docs / behaviour | Output file is append-only and unbounded; README claims a "rolling" file | — |
| 12 | Medium | behaviour | Stalled subscription emits no `RECV` line — stall indistinguishable from absence | — |
| 13 | Low | efficiency / claim | Thread-local cache is single-entry; "0.2 ns/op" is the 100%-cache-hit best case only | — |
| 14 | Low | build | Probe .so exports every symbol (not just `ros_trace_*`); lint test-deps declared but never wired | — |

---

## 1. Inter-process counter double-counts in a single process — **High**

**Evidence.** `counterForTopic()` deduplicates by topic *name*
(`src/core/topic_registry.cpp:22-32`), so a publisher and a subscriber of the same topic in the
same process resolve to **one** `sTopicCounter`. Both paths write its `inter` field:

- publish side: `onPublish` → `inter.fetch_add` (`src/core/topic_registry.cpp:113`)
- inter-process receive: `onCallbackStart(intra=false)` → `inter.fetch_add`
  (`src/core/topic_registry.cpp:130`, `:156`)

**Impact.** For an inter-process topic whose publisher and subscriber live in the **same process**
— e.g. a component container with `use_intra_process_comms` **off** (the default), or a node that
subscribes to a topic it also publishes — `inter` is incremented once per publish *and* once per
delivery, so the reported rate is **~2× the true rate**. N in-process subscribers → (1+N)×.

The bug is masked today because the integration test deliberately splits talker and listener into
**separate processes** with separate output files
(`test/integration/test_probe_integration.py:70-71`), and the README example reads like two
single-endpoint processes concatenated into one file.

**Secondary effect.** `TOPIC` and `RECV inter` are printed from the *same* field `s.inter_hz`
(`src/probe/interposers.cpp:86` and `:90`). So `RECV inter` is never an independent receive-side
measurement — it re-echoes the (possibly polluted) publish counter. Only the `intra` bucket is a
genuinely separate signal.

**Fix direction.** Split `sTopicCounter` into independent buckets:
`{pub_inter, recv_inter, recv_intra}`. `TOPIC` = `pub_inter`, `RECV inter` = `recv_inter`,
`RECV intra` = `recv_intra`. Publish and receive stop colliding, and the two output lines carry
distinct meaning.

## 2. "Node liveness" is monotonic, never decremented — **Medium**

**Evidence.** `onNodeInit` only appends (`src/core/topic_registry.cpp:76`); `activeNodes()` returns
the full list (`:188`). There is no node-shutdown hook and no dedup.

**Impact.** A crashed or shut-down node keeps printing `NODE /x` forever. The README markets
"active-node **liveness**", but the data is "nodes that were ever initialized in this process".
Re-init (rare) would also duplicate an entry.

**Fix direction.** Hook `ros_trace_rcl_node_init` teardown is not available cheaply; better to treat
NODE as "seen" and rename the claim, **or** derive liveness from recent per-node publish/callback
activity (a node with zero events for K windows is "quiet"). Dedup on insert regardless.

## 3. Unresolvable callbacks take the exclusive lock every call — **Medium**

**Evidence.** `ros_trace_callback_start` fires for **all** callbacks — subscription, timer, service
— not just subscriptions. Timer/service callbacks are never in `m_cb_to_sub`, so `resolveCallback`
returns `nullptr` (`src/core/topic_registry.cpp:85`) and is **never cached** (only non-null results
are inserted at `:96`). The steady-state shared-lock cache lookup also misses
(`:141`), so control flow falls to the `unique_lock` branch (`:147`) **on every invocation** of
every unresolved callback.

**Impact.** Directly undercuts the headline "no global lock" property. A node with a high-rate timer
(or many service calls) under a multi-threaded executor serializes every one of those callbacks on
the writer lock. The atomic-increment hot path is only lock-light for callbacks that *do* resolve.

**Fix direction.** Negative-result cache: insert a sentinel (e.g. `reinterpret_cast<sTopicCounter*>(1)`
or a dedicated "not-a-subscription" marker) into `m_cb_to_counter` so the second sighting of a timer
callback hits the shared-lock fast path and returns immediately. Guard the thread-local cache to skip
sentinels.

## 4. Default output file shared across every process — **Medium**

**Evidence.** Default `ROS_TOPIC_STATS_OUTPUT_FILE = /root/ssd2tb/logs/topic_freq.log`
(`src/probe/interposers.cpp:67`); each flush does `fopen(path,"a")` + multiple `fprintf`
(`:76-98`). Every `LD_PRELOAD`ed process writes the same path unless the operator overrides it.

**Impact.** In a multi-process launch (the normal ROS layout), all processes append to one file with
no locking. `fprintf` calls from different processes interleave *within* a window block, so a
consumer cannot reliably attribute a line to a process, and windows overlap. Also silently drops
data if the directory doesn't exist (`fopen` fails, `flush` returns — `:77`).

**Fix direction.** Default to a per-process path (embed pid, e.g. `.../topic_freq.<pid>.log`), or
write one line atomically (build the window block in a buffer, single `fwrite`), or `flock` the file.
Log once to stderr when `fopen` fails instead of silently dropping.

## 5. `std::stod` on a bad env var can call `std::terminate` — **Low**

**Evidence.** `m_period_s(std::stod(getEnv("ROS_TOPIC_STATISTICS_PUBLISH_PERIOD","5.0")))`
(`src/probe/interposers.cpp:68`) runs in the `ProbeRuntime` Meyers-singleton constructor, reached
from an interposed tracepoint called by rclcpp.

**Impact.** If the operator sets the period to a non-numeric value, `std::stod` throws; the exception
propagates out of `ros_trace_*` back into rclcpp, which is not expecting it → `std::terminate`. A
monitoring probe should never be able to crash the monitored process.

**Fix direction.** Parse in a `noexcept` free function `parsePeriodSeconds(const char*, double def)`
that catches/validates (also reject `<= 0`) and returns the default. Bonus: the function becomes
unit-testable (see [TESTING_PLAN.md](TESTING_PLAN.md)).

## 6. Dangling issue references & distro-specific premise — **Low**

- **`#1204` is not an upstream reference.** The flush comment "`#1204-compatible publish-side line`"
  (`src/probe/interposers.cpp:84`) and `bench/RESULTS.md` ("the design it replaces, #1204") point at
  an **internal** tracker from the original monorepo, not `ros2/rclcpp`. To a public reader it is a
  dead link. (`ros2/rclcpp#1204` is unrelated — a lifecycle-service-client test race.) Either inline
  what "#1204" was (global-mutex + per-message string-hash prototype) or drop the number.
- **`#2911` needs a distro caveat.** [ros2/rclcpp#2911](https://github.com/ros2/rclcpp/issues/2911)
  ("use_intra_process_comms bypasses topic statistics computation") is **closed** — fixed by
  [PR #3130](https://github.com/ros2/rclcpp/pull/3130) ("Fix topic statistics for IPC subscriptions"),
  merged to `rolling` in April 2026. So the gap ros2_pulse fills is **real on stock Humble** (no
  public backport as of this writing) but is closing on `rolling`/newer. The README should scope the
  "invisible to other tools" claim to Humble-class binaries and note built-in stats now cover intra
  upstream. See [ALTERNATIVES.md](ALTERNATIVES.md#built-in-topic-statistics).

## 7. Idle topics emit a zero line — **Low**

**Evidence.** `if (s.inter_count > 0 || s.intra_count == 0)` (`src/probe/interposers.cpp:85`) is true
when a known topic saw no traffic at all → prints `TOPIC /x 0.000000` every window.

**Impact.** Probably intentional (shows a declared-but-silent topic), but undocumented and it inflates
the file for large graphs. Decide + document, or gate behind an env flag.

---

# Round 2 — audit of post-round-1 `main` (2026-07-29)

## 8. First window reports up to ~2× Hz — **High**

Two compounding defects; either alone skews the very first number a user sees.

**Evidence (a) — timer first-fire off-by-one.** `Timer::runThread()`
(`src/core/timer.cpp:24-26`) sets `end_time = now + m_interval`, then adds `m_interval`
**again** at the top of the loop before the first wait. The first callback fires at
`t ≈ 2 × period`; all later periods are correct. Counts have been accumulating since the
first tracepoint (process start), so the first window holds ~2 periods of traffic.

**Evidence (b) — nominal denominator.** `flush()` calls `snapshot(m_period_s)`
(`src/probe/interposers.cpp:75`) and prints `window_s=%.3f` from the same configured constant
(`:86`). Hz is *never* divided by measured elapsed time — not for the doubled first window, and
not for later windows stretched by flush latency or scheduler jitter.

**Impact.** With a 50 Hz publisher and `period=1.0`, the first window reports ~100 Hz. For a
tool whose single output is Hz, the first sample being ~2× wrong reads as "topic overrunning"
and poisons any alerting consumer that doesn't skip window one. The accuracy harness currently
*documents* dodging this (`probe_harness.py: measured_rate()` drops the first window as
"ramp-up").

**Fix direction.** (a) Fire at `1 × interval`: advance `end_time` **after** the callback, not
before the first wait. (b) `ProbeRuntime` timestamps the previous flush (`steady_clock`) and
passes measured elapsed seconds into `snapshot()`; the header prints the measured value. Format
unchanged (`window_s=` simply becomes truthful). Regression tests: unit — first Timer callback
arrives within ~1.6×interval (red today at 2×); integration — first window's reported rate is
within tolerance of the known 50 Hz (red today at ~100 Hz); header `window_s` ≈ Δ`ts_ns`
between consecutive windows.

## 9. Exit-time UAF: tracepoints during static destruction — **Medium**

**Evidence.** `ProbeRuntime::instance()` is a Meyers singleton (`src/probe/interposers.cpp:48-51`)
— a function-local static destroyed during `atexit`/static teardown, in undefined order relative
to other shared libraries. Every interposer dereferences it (`:129-179`). DDS transport threads
and executor threads routinely deliver messages (→ `ros_trace_callback_start`,
`ros_trace_rcl_publish`) while `main` has returned and static destructors run.

**Impact.** A tracepoint that fires after `ProbeRuntime`'s destructor runs calls methods on a
destroyed object — registry maps and the `shared_mutex` are gone → UAF/segfault **in the host
process at shutdown**, blamed on the stack being monitored. Low probability per exit, near-certain
at fleet scale. (The registry-id-scoped TLS cache guards against *recycled registry instances*;
it does not guard the shared-lock path dereferencing the destroyed singleton's maps.)

**Fix direction.** Leaky singleton: `static auto* s = new ProbeRuntime(); return *s;` — never
destroyed, OS reclaims at exit. Join/stop the flush thread via `std::atexit` (final best-effort
flush), leaving the registry alive for straggler tracepoints. Regression: shutdown-storm
integration test (multithreaded executor + max-rate publishers, SIGINT loop) run under ASan in
the existing sanitizer lane.

## 10. `fork()` without `exec` leaves the child counting but never flushing — **Medium**

**Evidence.** Threads do not survive `fork()`. The child inherits `m_started == true`
(`src/probe/interposers.cpp:55-65`) but no timer thread; nothing re-arms it (no
`pthread_atfork` anywhere in the probe). Additionally, if the fork lands mid-flush the child
inherits `m_mu`/stdio locks in a locked state → first graph event in the child deadlocks.

**Impact.** Any node forking workers without exec (Python `multiprocessing` default start
method on Linux, daemonized helpers) silently loses all stats in the children — or hangs them.
`ros2 launch` (fork+exec) is unaffected, which is why this hasn't been seen.

**Fix direction.** `pthread_atfork(prepare=lock m_mu, parent=unlock, child=unlock +
m_started=false)` so the child lazily restarts its own timer on the next tracepoint (and, once
issue #4's per-PID path lands, re-derives its output path). Regression: integration test that
forks a preloaded publisher and asserts the child's windows appear.

## 11. Output file is append-only and unbounded; README says "rolling" — **Low**

**Evidence.** `flush()` always `fopen(path, "a")` (`src/probe/interposers.cpp:80`); no
truncation, rotation, or size cap exists anywhere. README markets "a small rolling file"
(`README.md:31`) and "~22 KB rolling" (`:92`).

**Impact.** A robot running for weeks with a 5 s period and ~50 topics grows the file without
bound on what is often small eMMC. Current reopen-per-window behaviour *is* logrotate-friendly,
but nothing ships that config, and the claim is false as written.

**Fix direction.** Either fix the wording, or (preferred) `ROS_TOPIC_STATS_MAX_BYTES`
(default ~10 MiB): one `fstat` per window; over the cap → rename to `<path>.1` (single
generation) and start fresh. Keep reopen-per-window so external logrotate still works.
Unit-test the rotation decision + rename; integration-test with a tiny cap.

## 12. Stalled subscription emits no RECV line — **Medium**

**Evidence.** The RECV line prints only when `recv_inter_count > 0 || recv_intra_count > 0`
(`src/probe/interposers.cpp:97`).

**Impact.** The tool's headline question is "is every topic flowing at the rate it should" —
but when an upstream publisher dies, the subscribed topic **vanishes** from output instead of
reading `0.000000`. A consumer cannot distinguish "stalled" from "never existed / probe not
attached". Publish-side idle currently *does* print a zero line (issue #7 debates gating it),
making the asymmetry worse: the side that most needs a zero is the only side that lacks one.

**Fix direction.** Track "topic has ≥1 resolved subscription" in the registry (known at
`resolveCallback` success); emit `RECV <t> inter=0.000000 intra=0.000000` for those topics every
window. Coordinate with issue #7 / PR #3's emit-predicate: idle-suppression (if adopted) must
not suppress the stalled-subscription zero — "declared but never active" (noise) and "active
then stopped" (signal) are different states. Regression: integration test — talker stops after
3 s, listener keeps running; a later window must contain `RECV /chatter inter=0.000000`.

## 13. Single-entry thread-local cache; "0.2 ns/op" is best-case-only — **Low**

**Evidence.** The TLS fast path caches exactly one `(id, key, counter)` triple per thread per
hook (`src/core/topic_registry.cpp:158-160`, `:178-180`). The microbench pins each thread to a
*single fixed key* (`bench/hotpath_bench.cpp`: `t % kTopics` constant per thread) → 100% cache
hit → the advertised 0.2 ns/op (`README.md:56`, `:96`).

**Impact.** Real nodes publish several topics from one thread (image + camera_info +
compressed in one timer callback) and multithreaded executors round-robin callbacks across
workers — alternating endpoints thrash the one-entry cache, so most operations take the
`shared_lock` path: rw-lock atomic RMWs with cross-core contention on `m_mu`'s cacheline,
realistically ~20–50 ns/op. Still far cheaper than an LTTng-UST tracepoint (~158 ns, Bédard et
al. 2022), but the README figure describes only the ideal case, and `m_mu` becomes a shared
contention point at high aggregate rates.

**Fix direction.** Replace the single entry with a small fixed-size open-addressed TLS map
(~16 slots, keyed by handle, scoped by registry id — preserving the recycled-instance guard).
Add an alternating-key leg to `hotpath_bench.cpp` and publish both numbers in README/RESULTS.
Regression: existing exact-total tests must stay green; new bench leg documents before/after.

## 14. Probe .so exports everything; lint deps declared but never wired — **Low**

**Evidence.** (a) No visibility control in `CMakeLists.txt`: the shared library exports the
core's symbols (`TopicRegistry`, `Timer`, …) into **every process on the robot**, not just the
`ros_trace_*` interposers. (b) `package.xml:24-25` declares `ament_lint_auto` /
`ament_lint_common` as test deps, but `CMakeLists.txt` never calls
`ament_lint_auto_find_test_dependencies()` — the lint suite never runs.

**Impact.** (a) Symbol-namespace pollution with generic class names risks odd interposition/ODR
collisions in unrelated code; also bloats the dynamic symbol table on the hot lookup path.
(b) Dead manifest weight and a false sense of lint coverage.

**Fix direction.** (a) `-fvisibility=hidden` for the probe target +
`__attribute__((visibility("default")))` on the `extern "C"` interposers (or a version script);
verify with an `nm -D` check in CI (only `ros_trace_*` exported). (b) Wire
`ament_lint_auto_find_test_dependencies()` under `BUILD_TESTING` and fix what it flags, or drop
the two deps.

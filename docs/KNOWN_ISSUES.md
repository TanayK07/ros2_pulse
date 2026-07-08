# Known issues & open concerns

Review of the current implementation (commit at time of writing). Each item lists severity,
evidence (`file:line`), impact, and a fix direction. Companion docs:
[ALTERNATIVES.md](ALTERNATIVES.md) (prior art + benchmarking) and
[TESTING_PLAN.md](TESTING_PLAN.md) (how each issue gets a regression test).

> **Status (all 7 addressed).** Each issue has a research-backed, test-first PR open against `main`
> ([docs/issues/](issues/) carries the per-issue research note; each PR ships the fix + its
> regression test, red→green). Cross-cutting test infrastructure + CI matrix is in a separate
> `ci/test-suite` PR. #6 is merged; the rest are open (in review).

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

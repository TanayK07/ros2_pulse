# Issue #1 — inter-process counter double-counts in a single process

Internal research/design note for the fix on branch `fix/double-count-inter`.
Companion: [KNOWN_ISSUES.md](../KNOWN_ISSUES.md) item #1 (severity **High**),
[TESTING_PLAN.md](../TESTING_PLAN.md) (issue #1 test matrix), [DESIGN.md](../DESIGN.md).

## Problem

For an inter-process topic whose publisher **and** subscriber live in the *same* process — a
component container with `use_intra_process_comms` off (the default), or a node that subscribes to a
topic it also publishes — `ros2_pulse` reports roughly **2x** the true rate. With N in-process
subscribers it reports (1+N)x. The reported publish rate (`TOPIC`) and the reported inter-process
receive rate (`RECV ... inter=`) are also numerically identical because they read the same field, so
`RECV inter` is never an independent receive-side measurement.

## Root cause (file:line evidence)

`counterForTopic()` deduplicates by topic **name**, returning one shared `sTopicCounter` per topic:

- `src/core/topic_registry.cpp:22-32` — `counterForTopic()` looks up `m_by_topic[topic]` and creates
  at most one counter per topic name. Both the publisher-init and subscription-init paths funnel
  through it (`:39` `m_pub_to_counter[pub_handle] = counterForTopic(topic)` and `:47`
  `m_subhandle_to_counter[sub_handle] = counterForTopic(topic)`), so a same-process pub+sub of one
  topic share a single counter object.

That single counter has one `inter` field, and **two independent hot paths both `fetch_add` it**:

- publish: `onPublish()` -> `it->second->inter.fetch_add(1, relaxed)` (`src/core/topic_registry.cpp:113`;
  thread-local fast path `:107`).
- inter-process receive: `onCallbackStart(is_intra_process=false)` ->
  `ctr->inter.fetch_add(1, relaxed)` (`src/core/topic_registry.cpp:156`; thread-local fast path `:130`).

So each in-process delivery of an inter-process message increments `inter` a second time on top of the
publish increment. Over a window with a publisher at R Hz and one same-process subscriber, `inter`
accumulates ~2R.

Secondary effect in the flush: `TOPIC` and `RECV ... inter=` both format `s.inter_hz`
(`src/probe/interposers.cpp:86` and `:90`), i.e. the same (polluted) field. Only the `intra` bucket
(`callback_start(is_intra_process=true)`, `:154`) is a genuinely separate signal today.

The bug is masked by the current tests: the integration test splits talker and listener into
**separate processes** with separate output files (`test/integration/test_probe_integration.py:70-71`),
and every unit test drives only one side (publish-only or receive-only) of a given topic, so the two
writers never collide on one counter.

## Why the current shape counts this way

The registry is intentionally topic-centric: one `sTopicCounter` per topic name, pointer-keyed maps
from `pub_handle` and the `callback -> ... -> sub_handle` chain both resolving to that one counter
(`topic_registry.hpp:83-92`). That is the right ownership model — the fix should keep it — but a
single counter cannot represent three physically distinct events (a local publish, an inter-process
delivery, an intra-process delivery) that a topic can experience simultaneously in one process.

## Options considered

### Option A — per-endpoint counters (one counter per publisher handle / per subscription)
Give every publisher handle and every subscription its own counter; aggregate by topic name only at
`snapshot()`.

- Pro: most physically faithful; naturally supports "which endpoint is slow" if ever needed.
- Con: larger change to ownership/maps; `snapshot()` must group-by topic; more allocations and map
  entries on the hot init path; multiple subscribers of one topic need a merge policy. Overkill for
  the stated output contract (per-topic `TOPIC`/`RECV` lines). More surface area, more risk for a
  HIGH-severity correctness fix that wants to stay small and reviewable.

### Option B — per-topic split buckets `{pub_inter, recv_inter, recv_intra}` (chosen)
Keep one `sTopicCounter` per topic (unchanged ownership and maps), but split its single `inter` field
into three role-specific atomics: publishes, inter-process receives, intra-process receives.

- Pro: minimal, local change; ownership/maps/resolution untouched; directly maps to the three output
  numbers the flush needs (`TOPIC`=publish, `RECV inter=`=recv_inter, `RECV intra=`=recv_intra);
  matches the fix direction already written into KNOWN_ISSUES.md #1 and TESTING_PLAN.md.
- Pro (contention): today publish and receive contend on the *same* atomic in the same-process case
  (true sharing). Splitting them into separate atomics removes that RMW contention on the common
  counter — the two hot paths now touch different words.
- Con: N same-topic subscribers in one process still sum into one `recv_inter` (that is the intended
  per-topic semantic — total deliveries — and matches TESTING_PLAN.md `MultipleSubscribersSameTopic`).

### Option C — subtract-publishes-from-receives heuristic
Keep one field and try to correct at read time (e.g. report `inter - publishes`).

- Rejected: fragile, assumes a fixed topology, breaks for pure-subscriber or pure-publisher
  processes, and still can't distinguish the two signals for output.

## Chosen approach + rationale

**Option B.** Split `sTopicCounter.{inter,intra}` into `{pub_inter, recv_inter, recv_intra}`:

- `onPublish` -> `pub_inter.fetch_add` (was `inter`).
- `onCallbackStart(false)` -> `recv_inter.fetch_add` (was `inter`).
- `onCallbackStart(true)` -> `recv_intra.fetch_add` (was `intra`).

`sTopicStat` and `snapshot()` expose all three as counts + Hz (`pub_inter_*`, `recv_inter_*`,
`recv_intra_*`). Flush maps `TOPIC` -> `pub_inter_hz`, `RECV ... inter=` -> `recv_inter_hz`,
`intra=` -> `recv_intra_hz`. This is the smallest change that makes publish and receive stop
colliding and gives the two output lines distinct meaning, while leaving the (correct) topic-keyed
ownership, the resolution chain, and the thread-local fast-path caching untouched.

The integration test stays green by construction: on the listener process `/chatter` now lands in
`recv_inter` and still surfaces as `RECV /chatter inter=...>20Hz` (`test_probe_integration.py:81-82`),
and `/intra_topic` lands in `recv_intra` -> `RECV ... intra=...` (`:60-62`).

## Concurrency notes (memory ordering + layout)

- **Memory ordering — keep `memory_order_relaxed`.** Each bucket is an independent monotonically
  increasing counter with no data dependency on other memory; relaxed gives atomicity without
  ordering, which is exactly the canonical counter use case in the standard
  ([cppreference std::memory_order, "Relaxed ordering"](https://en.cppreference.com/w/cpp/atomic/memory_order) —
  the documented example is N threads doing `cnt.fetch_add(1, memory_order_relaxed)` and joining).
  Splitting one relaxed counter into three relaxed counters preserves that property per bucket.
  `snapshot()` runs on the flush thread and does a relaxed `exchange(0)` to read-and-reset. Exact
  window boundaries are not synchronized (a handful of increments may land in the adjacent window),
  which is acceptable and already true for a statistical rate monitor — the running total is
  eventually consistent because `exchange` never drops counts. The `ConcurrentPublishExactTotal`
  unit test observes an exact total precisely because `std::thread::join()` establishes
  happens-before with the snapshot, per the same cppreference example.
- **Struct layout / false sharing — packed, not per-bucket cache-line padded (deliberate).** The
  three atomics can be written by different threads in the same-process pub+sub case (publisher
  thread hits `pub_inter`; executor thread(s) hit `recv_inter`/`recv_intra`), so they may sit on one
  64-byte line and ping-pong. We deliberately do **not** `alignas(hardware_destructive_interference_size)`
  each bucket, because: (1) topic cardinality is low (tens, not millions of objects), so per-topic
  bloat from ~64B to ~192B is irrelevant, whereas the design's stated priority is per-*message*
  hot-path cost, which is unchanged — one relaxed increment via the thread-local cache regardless of
  layout; (2) the fix already *removes* the worse problem, true sharing on a single contended atomic,
  which dominates false sharing; (3) `hardware_destructive_interference_size` is a known ABI/portability
  wart (GCC emits `-Winterference-size` warnings and its value differs across `-mtune`), so adding it
  buys little here and costs simplicity. If profiling on a busy multi-topic multi-core executor ever
  shows this line as hot, padding each bucket (or grouping publish vs receive onto separate lines) is
  the follow-up — noted, not done. See
  [cppreference std::hardware_destructive_interference_size](https://en.cppreference.com/w/cpp/thread/hardware_destructive_interference_size).

## Test plan for this fix (red -> green)

Unit (`test/unit/test_topic_registry.cpp`), names per TESTING_PLAN.md issue #1:

- `SameProcessPubAndSubDoNotDoubleCount` (regression, RED today): one publisher + one resolvable
  subscriber on the **same** topic in one registry; N publishes + N inter-receives. Assert
  `pub_inter_count == N` **and** `recv_inter_count == N` as separate values. On today's code both
  collapse into `inter == 2N`, so the separate-field assertion cannot be satisfied.
- `PublishSideIndependentOfReceive`: publish-only leaves `recv_inter_count == 0`; receive-only leaves
  `pub_inter_count == 0`.
- Update existing tests (`PublishCountsAndHz`, `SnapshotResetsCounts`, `InterProcessReceiveViaCallback`,
  `IntraProcessReceiveBucketedSeparately`, `LazyResolutionWhenInitOutOfOrder`, `FilteredTopicsExcluded`,
  `ConcurrentPublishExactTotal`) to the new field names/semantics.

Sanitizer: rerun the suite (incl. `ConcurrentPublishExactTotal`) under `-fsanitize=thread` since the
hot path is concurrent (atomics + `shared_mutex` + thread-local cache).

Because `src/probe/interposers.cpp` changes, run the full `colcon build --packages-select ros2_pulse`
compile-check.

## References

- cppreference, *std::memory_order* — "Relaxed ordering" section and the relaxed counter example
  (`cnt.fetch_add(1, std::memory_order_relaxed)` across joined threads):
  https://en.cppreference.com/w/cpp/atomic/memory_order (C++17).
- cppreference, *std::hardware_destructive_interference_size / _constructive_interference_size*
  (C++17; use with `alignas` to place objects on distinct cache lines to avoid false sharing):
  https://en.cppreference.com/w/cpp/thread/hardware_destructive_interference_size
- H. Boehm, *A Relaxed Guide to memory_order_relaxed*, WG21 P2055/P2135R1 — relaxed is appropriate for
  independent counters; avoid it where inter-variable ordering is load-bearing:
  https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2024/p2135r1.pdf
- ros2_pulse `docs/KNOWN_ISSUES.md` #1 and `docs/TESTING_PLAN.md` (issue #1 test matrix).
- Upstream context: rclcpp topic statistics do not observe intra-process traffic on stock Humble
  ([ros2/rclcpp#2911](https://github.com/ros2/rclcpp/issues/2911), fixed on rolling by
  [PR #3130](https://github.com/ros2/rclcpp/pull/3130)); ros2_pulse fills that gap, which is why the
  receive-side split (inter vs intra) must be independent of the publish count.

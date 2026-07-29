# Issue #3 — Unresolvable callbacks take the exclusive lock on every call

Research note for the fix on branch `fix/callback-lock-storm`.
Addresses [`KNOWN_ISSUES.md` #3](../KNOWN_ISSUES.md) (Medium, efficiency).

## Problem

`ros_trace_callback_start` fires for **every** callback the executor runs — subscription, timer,
*and* service — not just subscription deliveries (see `docs/DESIGN.md`, "Why hook the tracetools
layer"). Only subscription callbacks can ever resolve to a topic counter. Timer and service
callbacks never can: nothing ever calls `onCallbackAdded` for them, so they never get an entry in
`m_cb_to_sub`.

Under a multi-threaded executor with high-rate timers, those unresolvable callbacks are hammered on
the hot path, and today each hit takes the **exclusive** (`unique_lock`) write lock. That directly
defeats the "no global lock" property the design sells (`docs/DESIGN.md`, "Hot path").

## Root cause (file:line)

Trace one timer callback through `onCallbackStart` (`src/core/topic_registry.cpp:120-161`):

1. Thread-local fast path miss — `last_ctr` was never set for this callback, because we only set it
   when `ctr != nullptr` (`:158-160`), and a timer callback always resolves to `nullptr`.
2. Shared-lock cache lookup **misses** — `m_cb_to_counter.find(callback)` is empty
   (`:141-144`), so `ctr` stays `nullptr`.
3. Fall through to the **exclusive** lock and run `resolveCallback` (`:146-149`).
4. `resolveCallback` returns `nullptr` because `m_cb_to_sub.find(callback) == end()`
   (`src/core/topic_registry.cpp:85`), and — critically — that `nullptr` is **never cached**: only
   fully-resolved, non-null results are inserted into `m_cb_to_counter` (`:96`).

So there is no negative memory. Steps 1-4 repeat *identically on every single invocation*. For a
50 Hz timer that is 50 write-lock acquisitions per second per timer, each serializing all readers on
a multi-threaded executor — a write-lock storm on a path advertised as lock-light.

## Correctness hazard — do NOT break lazy resolution

The naive fix ("cache `nullptr` too") is wrong. `resolveCallback` returns `nullptr` for **two
different reasons** that must not be conflated:

- **(A) Proven not-a-subscription** — no entry in `m_cb_to_sub` at all (`:85`). Timer/service
  callbacks. This can never become resolvable.
- **(B) Not-yet-resolved subscription** — there *is* a `m_cb_to_sub` entry, but the rest of the
  chain (`m_sub_to_subhandle` `:89`, or `m_subhandle_to_counter` `:93`) is not populated yet.

Case (B) is real and load-bearing: for an intra-process subscription, rclcpp fires
`callback_added` *before* the intra waitable's `sub_handle -> topic` chain is populated (see
`docs/DESIGN.md`, "Lazy callback resolution", and test `LazyResolutionWhenInitOutOfOrder`). A message
can arrive while the chain is still incomplete; resolution must be retried on later deliveries and
must succeed once the final `subscription_init` lands. If we poisoned (B) with a negative-cache
entry, that subscription's traffic would be silently dropped forever.

**Therefore the negative cache may only be populated for case (A).**

## Options considered

| Option | Idea | Verdict |
|--------|------|---------|
| 1. Cache every `nullptr` | Insert a sentinel whenever `resolveCallback` returns null | **Rejected** — poisons case (B), breaks `LazyResolutionWhenInitOutOfOrder` |
| 2. Resolve timers eagerly / maintain a timer set | Track timer handles from a `timer_init` hook and skip them | **Rejected** — needs a new interposer + graph state; there is no cheap timer/subscription discriminator at `callback_start` beyond "is it in `m_cb_to_sub`" |
| 3. Downgrade the miss path to a shared lock | Skip the write lock for unresolved callbacks | **Rejected** — the write lock is genuinely needed to *insert* the resolved result; the real problem is the repeated escalation, not the lock kind |
| 4. Negative-result cache, sentinel, only for case (A) | Insert a "not-a-subscription" sentinel into `m_cb_to_counter`, keyed by callback, only when `m_cb_to_sub` has no entry | **Chosen** |

## Chosen approach

Add a single sentinel pointer `kNotASubscription` (the address of a file-local static
`sTopicCounter`; a unique, valid address that is **never dereferenced** — every read site compares by
identity). Then:

- `resolveCallback`: when `m_cb_to_sub.find(callback) == end()` (case A), insert
  `m_cb_to_counter[callback] = kNotASubscription` and return it. For case (B) — chain present but
  incomplete — keep returning `nullptr` **without** caching, exactly as today, so lazy resolution
  still works.
- `onCallbackStart`: treat the sentinel as "skip, do nothing" in both fast paths — the thread-local
  cache and the shared-lock cache. Cache the sentinel into the thread-local slot too, so repeat
  sightings on the same thread take *no* lock at all.

### Steady-state cost after the fix

- 1st sighting of a timer callback (process-wide): one write lock, inserts the sentinel.
- Later sightings on a *new* thread: one shared lock, finds the sentinel, caches it thread-locally.
- All further sightings on that thread: thread-local hit, zero locks.

So an unresolvable callback escalates to the exclusive lock **at most once per process**, and to a
shared lock at most once per thread — down from once per invocation.

## Invariant

> A `nullptr` from `resolveCallback` means "subscription callback, not resolvable *yet*" and is never
> cached. `kNotASubscription` means "proven never a subscription" and is cached. The negative cache
> is populated **only** when `callback` has no entry in `m_cb_to_sub` at all. This is sound because
> `callback_added` (which populates `m_cb_to_sub`) is a graph-init event that, by construction,
> always precedes the first `callback_start` for a real subscription callback — so "absent from
> `m_cb_to_sub` at delivery time" proves the callback is a timer/service, not a subscription whose
> link merely has not arrived. The sentinel is a valid, unique address that is never dereferenced;
> every consumer compares it by identity and skips.

## Test strategy (red -> green)

- Add an always-compiled observability counter `writeLockResolutions()` — an atomic incremented each
  time `onCallbackStart` escalates to the exclusive lock to run the resolution chain.
- `UnresolvableCallbackTakesWriteLockAtMostOnce`: fire an unlinked callback 1000× on one thread;
  assert `writeLockResolutions() <= 1`. **Red** before the fix (counter == 1000).
- `UnresolvableCallbackNoWriteLockStormConcurrent`: fire it from N threads × many iterations; assert
  `writeLockResolutions() <= N` and `<<` total calls — proves the storm is gone under a
  multi-threaded executor.
- `LazyResolutionWhenInitOutOfOrder` (existing) must stay green — the not-yet-resolved subscription
  callback must still bind once its chain completes.
- Run the suite under **ThreadSanitizer** (concurrent hot-path code).

## References

- std::shared_mutex, reader/writer semantics — https://en.cppreference.com/w/cpp/thread/shared_mutex
- Understanding std::shared_mutex from C++17 (reader-writer contention, read-mostly workloads) —
  https://www.cppstories.com/2026/shared_mutex/
- Reader-Writer Locks, MC++ BLOG — https://www.modernescpp.com/index.php/reader-writer-locks/
- Benchmarking reader-writer lock performance (why the writer lock hurts scalability) —
  https://turingcompl33t.github.io/RWLock-Benchmark/

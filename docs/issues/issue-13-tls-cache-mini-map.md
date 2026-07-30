# Issue #13 — Single-entry thread-local cache; "0.2 ns/op" is best-case-only

Research note for the fix on branch `fix/tls-cache-mini-map`.
Addresses [`KNOWN_ISSUES.md` #13](../KNOWN_ISSUES.md) (Low, efficiency/claim).

## Problem

Each hot-path hook caches exactly one `(registry-id, key, counter)` triple per thread. The
microbench pins each thread to a single fixed key → 100% hit → the advertised 0.2 ns/op. Real
nodes publish several topics from one thread (image + camera_info + compressed in one timer
callback) and multithreaded executors round-robin callbacks across workers — alternating
endpoints **thrash the one-entry cache**, so nearly every operation takes the `shared_lock`
path: rw-lock atomic RMWs with cross-core contention on `m_mu`'s cacheline (~20–50 ns/op, and a
shared contention point at high aggregate rates). Still far cheaper than an LTTng-UST
tracepoint (~158 ns, Bédard et al. 2022), but the README number describes only the ideal case.

## Fix

Replace both single-entry caches with one small **direct-mapped thread-local cache**
(16 slots, shared by the publish and callback hooks — handle and callback addresses are
distinct live objects, so the key domains cannot collide):

- slot = `hash(key) & 15`; hit requires `entry.id == registry-id && entry.key == key`
  (preserving the recycled-instance UAF guard);
- miss falls through to the existing shared-lock lookup and **replaces** the slot (it is a
  cache, not a map — a same-slot collision degrades to today's behaviour, never breaks
  correctness);
- the callback path may cache the `kNotASubscription` sentinel in a slot, same identity-compare
  skip as before.

A typical node's per-thread working set (a handful of publishers + subscriptions + a timer
callback) now fits cached → lock-free steady state for the *realistic* pattern, not just the
single-endpoint one.

## Observability + honest numbers

- `TopicRegistry::sharedLockLookups()` — relaxed counter incremented **only on the TLS-miss
  path** (which this fix makes cold; in the pre-fix worst case it counted every operation, so
  the counter itself is the regression signal).
- `bench/hotpath_bench.cpp` gains an **alternating-key leg** (each thread cycles 4 endpoints —
  the camera-pipeline pattern). README/RESULTS publish both numbers: single-key best case and
  alternating-key realistic case, before/after.

## Regression tests (red on current `main`)

- `unit: AlternatingEndpointsStayLockFree` — resolve two topics, then alternate
  `onPublish(A)/onPublish(B)` ×1000 on one thread; `sharedLockLookups()` must stay ≤ a small
  constant (pre-fix: ~2000 — every call misses the single-entry cache). Same shape for
  `onCallbackStart` with two callbacks.
- `unit: TlsCacheExactTotalsUnderAlternation` — totals stay exact while alternating (guards the
  replace-on-collision logic).
- Existing suites (exact totals, UAF-guard lifecycle, lazy resolution, negative cache) must
  stay green — they pin every correctness property the cache touches.

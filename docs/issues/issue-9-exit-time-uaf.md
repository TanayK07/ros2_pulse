# Issue #9 — Exit-time UAF: tracepoints during static destruction

Research note for the fix on branch `fix/exit-time-uaf`.
Addresses [`KNOWN_ISSUES.md` #9](../KNOWN_ISSUES.md) (Medium, robustness).

## Problem

`ProbeRuntime::instance()` is a Meyers singleton (`src/probe/interposers.cpp`): a function-local
static, destroyed during `atexit`/static teardown in undefined order relative to every other
shared library in the process. Every interposer dereferences it.

DDS transport threads and executor threads routinely deliver messages — firing
`ros_trace_callback_start` / `ros_trace_rcl_publish` — while `main` has already returned and
static destructors run. A tracepoint that lands after `ProbeRuntime`'s destructor:

- takes a `shared_lock` on a **destroyed** `std::shared_mutex` (UB), and
- runs `find()` on **freed** `unordered_map` heap nodes (use-after-free).

A monitoring probe must never be able to crash the host at shutdown; a fleet-scale deployment
turns "low probability per exit" into "regular mystery segfaults blamed on the stack being
monitored". Note the registry-id-scoped thread-local cache does *not* guard this: it protects
against a *recycled* registry instance, while the shared-lock path dereferences the destroyed
singleton's members directly.

## Secondary defect (this is the observable one)

The Meyers destructor chain (`~ProbeRuntime` → `~Timer` → `stop()`) joins the flush thread but
**never writes a final window**: everything counted since the last periodic flush is silently
dropped at exit. For a probe whose whole job is "tell me the rate until the process died", the
tail window is exactly the data an operator wants after a crash-adjacent shutdown.

This defect gives the issue a **deterministic** regression test (below) — the UAF itself is a
race and only fails probabilistically (deterministically only under an ASan-instrumented preload,
which the integration lane doesn't run).

## Fix

Leaky singleton + explicit `atexit` shutdown:

```cpp
static auto instance() -> ProbeRuntime& {
    static auto* s_instance = new ProbeRuntime();   // intentionally leaked
    return *s_instance;
}
```

- The runtime (registry, maps, mutex) is **never destroyed**; the OS reclaims at exit. A
  straggler tracepoint from any thread, at any point of teardown, operates on live objects and
  merely counts into buckets that will never be flushed — safe by construction.
- `ensureStarted()` registers one `std::atexit` handler: stop (join) the flush thread, then run
  one final `flush()`. It runs on the exiting thread, touches only our leaked objects and libc,
  and makes the tail window durable. `atexit` after `main` returns is exactly the window where
  the old code destroyed the runtime.

Rejected alternatives:

- **Suppress counting after a shutdown flag** — still needs the objects alive to *check* the
  flag; solves nothing the leak doesn't already solve, adds a hot-path branch.
- **`__attribute__((destructor))` ordering games** — priority values only order within one
  image; DDS threads live in other images. Not a guarantee.
- **Keep Meyers, wrap everything in shared_ptr keep-alives per interposer** — hot-path refcount
  traffic to solve a shutdown-only problem.

## Regression tests

- `integration: test_final_partial_window_flushed_at_exit` — **red on main**. Run the talker
  ~1.6 s with `period=1.0`: main writes only the ~1.0 s periodic window and drops the tail;
  fixed code appends a final partial window at exit (measured `window_s` well under the
  period — the issue-8 measured denominator makes the partial window honest).
- `integration: test_exit_with_straggler_tracepoints_is_clean` — guard (probabilistic on main,
  safe by construction after). A test-node mode (`exit_storm`) spawns detached threads that
  hammer the `ros_trace_*` symbols directly — no rcl machinery, so nothing else can crash — and
  returns from `main` while they run. Asserts clean exit (rc 0, no SIGSEGV/SIGABRT) across
  repeated runs.

## Interactions

- Relies on issue #8's measured window: the final flush divides the tail counts by the actual
  partial elapsed time, so the last window's Hz is accurate, not inflated by a nominal period.
- Issue #10 (`fork()` without exec) will touch `ensureStarted()` too (`pthread_atfork`); kept
  strictly separate.

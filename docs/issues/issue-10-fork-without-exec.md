# Issue #10 — `fork()` without `exec` leaves the child counting but never flushing

Research note for the fix on branch `fix/fork-without-exec`.
Addresses [`KNOWN_ISSUES.md` #10](../KNOWN_ISSUES.md) (Medium, robustness).

## Problem

Threads do not survive `fork()`. A child forked **without** `exec` inherits the probe's full
state — `m_started == true`, the registry, the armed `std::optional<Timer>` — but **no flush
thread**. Nothing re-arms it, so the child counts forever and never writes a window. Users hit
this with Python `multiprocessing` (default `fork` start method on Linux) and daemonized worker
patterns. `ros2 launch` (fork **+ exec**) is unaffected: exec wipes the process image and the
preload re-initializes from scratch.

Two secondary hazards in the same scenario:

1. **Inherited locked mutexes.** If the fork lands mid-flush, the child's copy of the registry's
   `shared_mutex` (and the Timer's mutex) is locked by a thread that doesn't exist in the child →
   the child's first graph event deadlocks.
2. **Stale `std::thread` handle.** The child's inherited `Timer::m_thread` claims `joinable()`,
   but the thread is gone. Re-arming naively (`m_timer.emplace(...)`) destroys the old Timer,
   whose `stop()` would `join()` a nonexistent thread — `std::terminate` / undefined behaviour.
3. **Wrong output attribution.** With issue #4's per-PID default path, the child would keep
   writing to the *parent's* `topic_freq.<parent-pid>.log`.

## Fix

Standard `pthread_atfork` quiescence, registered once alongside the issue-9 `atexit` hook:

- **prepare** (parent, before fork): acquire the Timer mutex, then the registry's write lock —
  the same order the flush thread uses (`Timer::runThread` holds its mutex while `flush()` →
  `snapshot()` takes the registry lock), so no lock-order inversion. Fork can then only land at
  a quiescent point: the child inherits both locks **held by the forking thread itself**.
- **parent** (after fork): release both in reverse order; nothing else changes.
- **child** (after fork): release both (legal: this thread holds them), then
  - `Timer::forkChildReset()` — detach the stale thread handle (glibc: flips a flag in the
    inherited descriptor copy; the sole safe way to make a joinable-but-dead handle droppable)
    and mark the timer stopped, so the next `start()` arms a fresh thread;
  - re-resolve the output path (a default per-PID path now embeds the **child's** pid; an
    explicit `ROS_TOPIC_STATS_OUTPUT_FILE` stays honoured verbatim);
  - restamp the window start and clear `m_started`, so the child's **next tracepoint lazily
    restarts** the flush timer via the existing `ensureStarted()` CAS.

Hook registration (`atexit` + `pthread_atfork`) is guarded by a dedicated flag that the child
does **not** reset — handlers are inherited across `fork()`, so re-registering per generation
would stack duplicates in grandchildren.

Inherited counts: the child's registry is a snapshot of the parent's (same addresses, copied
memory) — its first window may include a partial parent-side count. Documented, not corrected:
resetting counters in the child would equally misattribute in the other direction, and the
steady state is correct either way.

## Regression test (red on main)

`integration: test_fork.py::test_forked_child_flushes_its_own_counts` — a synthetic test-node
mode (`fork_pub`) arms the probe and registers a publisher via direct interposer calls (no rcl
machinery — DDS state is not fork-safe and is irrelevant to the probe path), forks without exec,
and the **child** publishes ~500 messages over ~2.5 s and exits normally:

- on `main`: the child never flushes (and its atexit final-flush inherits a timer whose thread
  is gone) → no window ever contains `/forked` traffic → **fail**;
- fixed: the child's first tracepoint re-arms the flush timer; periodic + final windows carry
  `TOPIC /forked` at a nonzero rate; the parent (which publishes nothing) stays clean.

The parent `waitpid`s the child and propagates its exit status, so a deadlocked or crashed child
fails the test by timeout/rc instead of hanging silently.

## Debugging findings (what the red test actually caught)

Two things beyond the original analysis, both found by tracing the hung child
(`strace -k` stack capture; gdb attach was blocked by yama ptrace_scope):

1. **`std::shared_mutex` unlock is a silent no-op in a fork child.** glibc's rwlock stores the
   writer's TID; the forking thread's TID differs in the child, so `pthread_rwlock_unlock`
   fails the owner check and leaves the registry write-locked forever — the child's first
   `shared_lock` blocked on the rwlock futex with no possible waker. The child handler must
   **re-initialize** the mutex in place (`TopicRegistry::forkChildReset()`, placement-new) —
   legal because the child is single-threaded and owns the lock by inheritance. A plain
   `std::mutex` (the Timer's) has no owner check, so its unlock works in the child as-is.
2. **`fork()` after `rclcpp::init` wedges the child with no probe involved** (verified with a
   no-preload control run): rclcpp/DDS fork handlers leave the child stuck on a condvar. The
   regression test therefore drives the interposers directly and forks *before* any rclcpp
   init — which also matches the real-world pattern (Python `multiprocessing` forks before the
   child touches ROS state).

Also fixed along the way: a fork child re-arming the probe must not destroy the inherited
Timer — its condition variable can carry waiter refs from the dead flush thread, and
`pthread_cond_destroy` on such a cv can block forever. The runtime now heap-allocates the timer
and leaks the previous instance on re-arm (one small leak per fork generation, same philosophy
as the leaky runtime).

## Interactions

- Builds on issue #9 (leaky singleton + atexit): the child inherits the parent's atexit
  registration, so a child that exits still writes its final partial window.
- Uses issue #4's `resolveOutputPath()` for child re-resolution and issue #8's window stamp.

# Issue #4 — default output file shared across every process

Internal research + design note for [KNOWN_ISSUES.md](../KNOWN_ISSUES.md) #4 (Medium, robustness).

## Problem

Every `LD_PRELOAD`ed process writes windowed stats to the **same** default path, with no locking,
using many separate `fprintf` calls per window. In a normal multi-process ROS launch this means:

- lines from different processes interleave *within* a single window block, so a consumer cannot
  attribute a `TOPIC` / `RECV` / `NODE` line to the process that produced it, and window headers
  overlap;
- if the default directory does not exist, `fopen` fails and every window is dropped **silently**.

## Root cause (file:line)

- Default path is a single shared constant:
  `ProbeRuntime()` initialises `m_out_path` to `/root/ssd2tb/logs/topic_freq.log`
  — `src/probe/interposers.cpp:67`. It is process-independent, so N preloaded processes all target
  one file.
- The flush emits a window as **many** un-synchronised `fprintf` calls to a freshly `fopen`ed
  handle — `src/probe/interposers.cpp:76-98` (header `:82`, `TOPIC`/`RECV` loop `:83-93`,
  `NODE` loop `:94-96`, trailing newline `:97`). There is no `flock`, and each `fprintf` can be an
  independent `write()`, so two processes' lines can be interleaved by the kernel.
- On `fopen` failure the flush simply `return`s — `src/probe/interposers.cpp:77` — with no
  diagnostic, so a missing directory looks identical to "no traffic".

## Background research (POSIX append semantics)

- `O_APPEND` (which `fopen(path, "a")` sets on the underlying fd) guarantees the offset is moved to
  EOF *and* the write happens with no intervening modification — i.e. appends never clobber each
  other's bytes. It does **not** guarantee that two concurrent writes to a **regular file** are
  non-interleaved. The `PIPE_BUF` "atomic up to N bytes" rule is specified for **pipes/FIFOs**, not
  regular files. ([POSIX `write`](https://pubs.opengroup.org/onlinepubs/9699919799/functions/write.html),
  [nullprogram: appending from multiple processes](https://nullprogram.com/blog/2016/08/03/))
- stdio buffering defeats even the per-`write()` atomicity you would get from raw `write()`: a
  sequence of `fprintf`s is batched in a user-space buffer and flushed in whatever chunks stdio
  chooses, so partial windows can hit the file between another process's writes.
  ([Jim Fisher: concurrent fwrites are not atomic](https://jameshfisher.com/2017/07/29/concurrent-fwrites/),
  [linuxvox: multi-process fopen append](https://linuxvox.com/blog/can-multiple-processes-append-to-a-file-using-fopen-without-any-concurrency-problems/))
- Practical consequence: building the whole window into **one** buffer and writing it with a single
  `fwrite` collapses a window to one stdio flush. For a small window block (well under `BUFSIZ`,
  typically 8 KiB) that flush is a single `write()` at `fclose`, which O_APPEND makes as atomic as a
  regular-file append gets on Linux. It narrows — but does not by spec eliminate — cross-process
  interleaving. `getpid()` is POSIX and cheap; deriving a per-process path removes the sharing
  entirely.

## Options considered

| Option | What | Pro | Con |
|---|---|---|---|
| (a) per-process default path | default becomes `.../topic_freq.<pid>.log` | eliminates sharing by construction; each window block is attributable; no cross-process races at all | one file per process (consumers must glob/merge); explicit env override still lets an operator share a path |
| (b) single-buffer write | format the window into one `std::string`, one `fwrite` | atomic-ish per window even on a shared file; also makes formatting a **pure, unit-testable** function | not a hard POSIX guarantee for regular files; doesn't help attribution |
| (c) `flock`/`O_APPEND` locking | advisory-lock the file per flush | true mutual exclusion on a shared file | adds a syscall pair per window; NFS/overlay lock semantics vary; still doesn't attribute lines to a pid |

## Chosen approach + rationale

Do **(a) + (b) + a diagnostic**, skip (c):

1. **Per-process default path** — new pure helper `defaultOutputPath(long pid)` in the core returns
   `/root/ssd2tb/logs/topic_freq.<pid>.log`. The probe uses `defaultOutputPath(getpid())` only when
   `ROS_TOPIC_STATS_OUTPUT_FILE` is unset; an explicit override is still honoured verbatim. This is
   the real fix: it makes interleaving impossible in the default multi-process layout.
2. **Single-buffer write** — extract window formatting into a pure `formatWindow(stats, nodes,
   ts_ns, window_s) -> std::string` in the core (no ROS dep, unit-testable), byte-compatible with
   today's format. `flush()` builds the block once and emits it with a single `fwrite`. This keeps a
   window atomic-per-flush even if an operator points several processes at one path, and it is what
   makes the format testable.
3. **Diagnostic on open failure** — log once to `stderr` (guarded by an atomic flag, with
   `strerror(errno)`) instead of silently dropping every window.

`flock` (c) is not adopted: with (a) there is no shared file to lock in the default case, and the
per-window syscall cost + variable lock semantics across filesystems are not worth it for a
monitoring probe whose contract is "adds no cost, opens nothing special".

The on-disk byte format is unchanged (`# ts_ns=.. window_s=..` header, `TOPIC`/`RECV`/`NODE` lines,
trailing blank line) — the goal is attributable, atomic writes, not a format change.

## References

- POSIX `write` (O_APPEND semantics): https://pubs.opengroup.org/onlinepubs/9699919799/functions/write.html
- "Appending to a File from Multiple Processes" (nullprogram): https://nullprogram.com/blog/2016/08/03/
- "Are concurrent `fwrite`s atomic? No!" (Jim Fisher): https://jameshfisher.com/2017/07/29/concurrent-fwrites/
- "Can multiple processes append to a file using fopen…" (linuxvox): https://linuxvox.com/blog/can-multiple-processes-append-to-a-file-using-fopen-without-any-concurrency-problems/

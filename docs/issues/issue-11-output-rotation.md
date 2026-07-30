# Issue #11 — Output file is append-only and unbounded; README says "rolling"

Research note for the fix on branch `fix/output-rotation`.
Addresses [`KNOWN_ISSUES.md` #11](../KNOWN_ISSUES.md) (Low, docs/behaviour).

## Problem

`flush()` always opens with `"a"` and appends; no truncation, rotation, or size cap exists.
README markets "a small rolling file" and "~22 KB rolling" — false as written. A robot running
for weeks with a 5 s period and ~50 topics grows the file without bound on what is often small
eMMC. The stall zero-lines from issue #12 add a line per proven endpoint per window, making
growth strictly worse.

Current behaviour is logrotate-*friendly* (reopen per window → an external rename+create works),
but nothing ships that config and the claim shouldn't depend on an operator setting up logrotate.

## Fix

Built-in single-generation size rotation:

- `ROS_TOPIC_STATS_MAX_BYTES` — cap in bytes. Default **10 MiB**; `0` disables rotation
  (pure append, old behaviour). Parsed by a `noexcept` core function
  (`parseMaxBytes`, same contract and rationale as issue #5's `parsePeriodSeconds`: this runs
  in the tracepoint-reached singleton constructor and must never throw).
- Before each append, `flush()` checks the file size (`stat`, one syscall per window); at or
  over the cap it renames `<path>` → `<path>.1` (replacing any previous `.1`) and starts fresh.
  Worst-case disk = 2 × cap per process. `rename(2)` is atomic on POSIX filesystems.
- Reopen-per-window is kept, so external logrotate still works for operators who prefer it.
- README wording updated to describe the actual mechanism.

Costs: one `stat` per window (~µs, on the flush thread, not the hot path).

Rejected: multi-generation rotation (`.2`, `.3`, …) and time-based rotation — nothing needs
them yet (YAGNI); the single `.1` generation bounds disk while keeping the last full window of
history readable.

## Regression tests (red on current `main`)

- `unit: ParseMaxBytes*` — unset/empty/garbage/trailing-junk/negative → default; `"0"` →
  disabled; plain integers parse; whitespace tolerated. (Red: function absent → compile.)
- `integration: test_rotation.py::test_size_cap_rotates` — tiny cap (512 B), ~6 s run with a
  1 s period: `<path>.1` must exist and `<path>` must stay under cap + one window's slack. On
  main the file just grows past the cap and no `.1` appears → red.
- `integration: test_rotation.py::test_zero_disables_rotation` — `MAX_BYTES=0`, same run:
  no `.1`, file grows freely (pins the escape hatch).

# Issue 6 — Dangling issue references & distro-specific premise (docs only)

Tracks `docs/KNOWN_ISSUES.md` #6 (**Low**, docs). No logic changes: the only edit to a source
file is a code comment in `src/probe/interposers.cpp`. This note records the two problems, the
evidence, what was verified upstream, and the exact wording changes.

## Problem 1 — `#1204` is not an upstream reference

`#1204` reads like a `ros2/rclcpp` issue/PR to a public reader, but it is an **internal** tracker
number carried over from the original monorepo. It points at nothing in this repo and, worse,
the public `ros2/rclcpp#1204` is an **unrelated** change.

Evidence (file:line):
- `src/probe/interposers.cpp:84` — comment `// #1204-compatible publish-side line`.
- `bench/RESULTS.md:43` — microbench row `OLD (#1204 global mutex + per-msg string hash)`.
- `bench/RESULTS.md:46` — prose `This compares ours to the design it replaces (#1204), ...`.

What `#1204` actually was (inlined so the meaning survives without the number): the earlier
topic-statistics prototype this design replaced — a **global mutex + per-message string-hash**
counter. The 0.2 ns/op vs 109.7 ns/op (478x) microbench compares the current per-endpoint atomic
+ thread-local cache against that prototype.

Verified that the public number is a dead/misleading link:
- `ros2/rclcpp#1204` = PR **"fix race in test_lifecycle_service_client"** (closed) —
  <https://github.com/ros2/rclcpp/pull/1204>. Nothing to do with topic statistics.

Fix: inline the description in the `interposers.cpp` comment; drop the number in `bench/RESULTS.md`
and replace the parenthetical `(#1204)` with the inlined description. README's benchmark line
already inlines this ("naive global-mutex + per-message string-hash design") with no number, so it
needs no change.

## Problem 2 — `#2911` premise needs a distro caveat

The README's "Why" section and its sample-output comment claim built-in topic statistics are
"bypassed entirely by intra-process comms" / intra-process traffic is "invisible to other tools".
That is **true on stock Humble**, but the upstream gap is **closing on rolling/newer**, so the
claim must be scoped to Humble-class binaries.

Evidence (file:line):
- `README.md:19-20` — "Why" bullet linking `#2911` with an unscoped claim.
- `README.md:36` — code-block comment `# <- intra-process, invisible to other tools`.

Verified upstream (via GitHub API, 2026-07-08):
- **ros2/rclcpp#2911** "use_intra_process_comms bypasses topic statistics computation" —
  **CLOSED (completed)**, opened 2025-07-18, closed **2026-04-20**.
  <https://github.com/ros2/rclcpp/issues/2911>. Symptom: with `use_intra_process_comms=True`,
  topic statistics reported only `.nan`; disabled, they reported valid `message_age`/`message_period`.
- **ros2/rclcpp#3130** "Fix topic statistics for IPC subscriptions" — **MERGED** into base
  branch **`rolling`**, opened 2026-04-15, merged **2026-04-20**.
  <https://github.com/ros2/rclcpp/pull/3130>. PR description notes it was first validated as a
  backport onto ROS 2 Kilted (rclcpp == 29.5.6).
- **Backport status:** PR #3130 carries no backport labels, and a repo PR search for a Humble
  backport of #3130 / IPC topic-statistics returned only #3130 itself. So there is **no public
  Humble backport as of this writing (2026-07-08)**.

Net: the gap ros2_pulse fills is real on stock Humble, but is closed upstream for intra-process on
`rolling`/newer. Scope the claim to Humble-class binaries and point at #3130 for the upstream fix.
`docs/ALTERNATIVES.md` already carries this nuance (Built-in topic statistics → Upstream status),
so the README is now consistent with it.

## Exact wording changes

- `src/probe/interposers.cpp:84` (comment only, no logic change):
  - before: `// #1204-compatible publish-side line`
  - after: `// publish-side line; output format kept compatible with the earlier global-mutex +
    per-message-string-hash stats prototype this design replaced`
- `bench/RESULTS.md`:
  - `OLD (#1204 global mutex + per-msg string hash)` -> `OLD (global mutex + per-msg string hash)`
  - `This compares ours to the design it replaces (#1204), not to eBPF/LTTng.` ->
    `This compares ours to the design it replaces (the earlier global-mutex + per-message
    string-hash stats prototype), not to eBPF/LTTng.`
- `README.md`:
  - "Why" bullet: scope the built-in-topic-statistics claim to Humble-class binaries and add the
    upstream-fix note (rclcpp#3130, merged to rolling Apr 2026; no Humble backport yet).
  - code-block comment: `# <- intra-process, invisible to other tools` ->
    `# <- intra-process, invisible to other tools on Humble`

## Not touched

- `docs/KNOWN_ISSUES.md` (owned elsewhere).
- Any code logic — the `interposers.cpp` change is a comment; core behaviour is unchanged.
</content>
</invoke>

# Issue #12 — Stalled subscription emits no RECV line

Research note for the fix on branch `fix/stall-visibility`.
Addresses [`KNOWN_ISSUES.md` #12](../KNOWN_ISSUES.md) (Medium, behaviour).

## Problem

The RECV line is emitted only when the window saw receive traffic
(`recv_inter_count > 0 || recv_intra_count > 0`). The tool's headline question is *"is every
topic flowing at the rate it should"* — but when an upstream publisher dies, the subscribed
topic **vanishes** from the output instead of reading `0.000000`. A consumer tailing the file
cannot distinguish three very different states:

- topic stalled (upstream dead — the alarm case),
- topic never existed / no subscription,
- probe not attached.

Publish-side idle is handled (issue #7: zero line, opt-in gated). The receive side — the side
that actually detects a dead *upstream*, which no publish-side counter in this process can see —
is the only side with no zero line.

## Semantics: "active then stopped" vs "declared but never active"

Issue #7 established that *declared-but-never-active* topics are noise (suppressed by default).
A stalled subscription is the opposite — it is **signal**, and the two must not be conflated:

- A subscription whose topic **never delivered a single message** stays unreported (nothing to
  distinguish it from a mis-wired remapping; and its callback chain has never even resolved).
- A subscription that **has delivered at least once** and then goes quiet must be affirmatively
  reported at `0.000000` every window, so rate-checking consumers see the stall.

The registry learns "this topic has a live, resolved subscription" at exactly the right moment
for that rule: `resolveCallback()` succeeds on the **first delivered message** (resolution is
lazy, driven by `callback_start`). Setting a flag there costs nothing on the hot path — it runs
under the exclusive lock that first resolution already takes.

## Fix

- `sTopicCounter` gains `bool recv_endpoint_seen` — set once in `resolveCallback()` when a real
  counter is cached (already under the write lock).
- `sTopicStat` carries it out of `snapshot()` as `recv_endpoint_seen`.
- New pure predicate `TopicRegistry::shouldEmitRecv(stat)` next to `shouldEmitTopic()` (one
  home for emit policy, unit-testable): emit when the window saw receive traffic **or** a
  resolved subscription exists for the topic.
- `formatWindow()` RECV gate switches to the predicate. Output format unchanged; stalled topics
  now render `RECV <t> inter=0.000000 intra=0.000000`.

Interaction with #7: unchanged. The TOPIC-line idle suppression keys off *fully idle and gated
by `ROS_PULSE_EMIT_IDLE`*; the RECV zero line keys off *proven-once receive endpoint* — a
stalled subscribed topic emits the RECV zero line while its TOPIC line stays suppressed.

## Regression tests (red on current `main`)

- `unit: StalledSubscriptionStillReported` — resolve a chain, deliver, snapshot; a second
  zero-traffic snapshot must still report the topic with `recv_endpoint_seen == true`.
- `unit: ShouldEmitRecv*` — never-resolved+idle → suppressed; resolved+idle → emitted;
  traffic → emitted.
- `unit(WindowFormat): StalledTopicEmitsRecvZeroLine` — golden bytes: RECV zero line present,
  TOPIC line absent (default `emit_idle=false`).
- `integration: test_stall.py::test_dead_upstream_reads_zero` — talker + listener in separate
  processes; kill the talker after ~3 s while the listener runs on; a post-stall window in the
  listener's log must contain `RECV /chatter inter=0.000000`. On main the topic simply
  disappears → red.

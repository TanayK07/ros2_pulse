<!--
Copyright 2026 ros2_pulse contributors
Licensed under the Apache License, Version 2.0 (the "License").
-->

# Issue #7 — Idle topics emit a zero line

Internal research note for [KNOWN_ISSUES.md](../KNOWN_ISSUES.md) #7 (severity: **Low**, area:
behaviour). Written before the fix; drives the TDD change on `fix/idle-topic-line`.

## Problem

A topic that is known to the probe (a publisher/subscriber was declared for it) but saw **no
traffic at all** in a flush window still gets a publish-side line written every window:

```
TOPIC /x 0.000000
```

For a large graph with many declared-but-silent topics this is one zero line per topic **per
window**, forever — it inflates the rolling output file and drowns the topics that are actually
flowing. Whether a declared-but-silent topic should appear at all is a judgement call, but today
it is neither documented nor configurable.

## Root cause

`src/probe/interposers.cpp:85`, inside `ProbeRuntime::flush()`:

```cpp
// #1204-compatible publish-side line
if (s.inter_count > 0 || s.intra_count == 0) {
    std::fprintf(f, "TOPIC %s %.6f\n", s.topic.c_str(), s.inter_hz);
}
```

The `TOPIC` line is the **publish-side inter-process** rate. The intent of the two clauses is:

- `inter_count > 0` — the topic saw inter-process publishes this window, so print its rate;
- `|| s.intra_count == 0` — suppress the publish-side line for an **intra-only** topic
  (`inter == 0, intra > 0`), because a `0.000000` publish rate there is misleading (the real
  signal is on the additive `RECV` line).

The trap: `s.intra_count == 0` is **also** true for a fully-idle topic (`inter == 0, intra == 0`),
so the guard lets the zero line through for exactly the case we don't want.

`snapshot()` (`src/core/topic_registry.cpp:163-186`) only ever returns topics present in
`m_by_topic` — i.e. topics for which an endpoint was declared — so a fully-idle entry here is a
real declared-but-silent topic, not noise.

## Current condition — exact truth table

`emit = (inter_count > 0) || (intra_count == 0)`

| inter_count | intra_count | `inter>0` | `intra==0` | emits `TOPIC`? | meaning |
|---|---|---|---|---|---|
| 0 | 0 | F | T | **YES** | **fully idle — the bug: `TOPIC /x 0.000000`** |
| >0 | 0 | T | T | YES | active inter-process (correct) |
| 0 | >0 | F | F | NO | intra-only — publish-side line suppressed (correct) |
| >0 | >0 | T | F | YES | active, both transports (correct) |

Only the first row is wrong. The `RECV` line guard just below
(`s.intra_count > 0 || s.inter_count > 0`, `interposers.cpp:89`) already excludes the fully-idle
row, so `RECV` is unaffected — this issue is scoped to the `TOPIC` line only.

## Chosen behaviour

Default to **NOT** emitting a fully-idle topic, with an opt-in env flag to restore the old
behaviour for operators who want to see declared-but-silent topics.

- Env flag: **`ROS_PULSE_EMIT_IDLE`**. `1` = emit idle topics (legacy behaviour); anything else
  (unset / `0`) = suppress. Default **off**.
- Extract the decision into a pure, unit-testable predicate in the core, alongside the existing
  `TopicRegistry::shouldFilter`:

```cpp
// Emit the publish-side TOPIC line for this window?
// Fully-idle topic (no inter- AND no intra-process traffic) -> only when emit_idle is set.
// Otherwise the historical rule stands: emit when inter traffic is present; suppress the
// publish-side line for intra-only topics.
static auto shouldEmitTopic(const sTopicStat& stat, bool emit_idle) -> bool {
    if (stat.inter_count == 0 && stat.intra_count == 0) {
        return emit_idle;
    }
    return stat.inter_count > 0 || stat.intra_count == 0;
}
```

New truth table:

| inter_count | intra_count | `emit_idle=false` (default) | `emit_idle=true` |
|---|---|---|---|
| 0 | 0 | **NO** (fixed) | YES (legacy restored) |
| >0 | 0 | YES | YES |
| 0 | >0 | NO | NO |
| >0 | >0 | YES | YES |

Every non-idle row is **identical** to the current behaviour, so the change is strictly limited to
the fully-idle case and is fully reversible via the flag. The output line format
(`TOPIC %s %.6f`) is untouched.

### Rationale

- **Default-off is the least-surprising default at scale.** The common complaint is file bloat on
  big graphs; the zero line carries no rate information, and a declared-but-silent topic is still
  discoverable via `ros2 topic list`. Making the noisy behaviour opt-in matches the "small rolling
  file" promise in the README.
- **Opt-in preserves the (possibly intentional) signal.** Some operators do want to see that a
  topic was declared but is silent (e.g. a sensor that stopped publishing). `ROS_PULSE_EMIT_IDLE=1`
  gives them the exact old output back.
- **Predicate in the core** keeps the decision unit-testable with no ROS/process spin-up, matching
  the project's core/probe split and the existing `shouldFilter` pattern.

### Env-flag convention

The probe's existing knobs are full-word env vars (`ROS_TOPIC_STATS_OUTPUT_FILE`,
`ROS_TOPIC_STATISTICS_PUBLISH_PERIOD`). A boolean `1`/`0` toggle is the common Unix/ROS convention
for feature flags (e.g. `RCUTILS_COLORIZED_OUTPUT`, `ROS_DISABLE_LOANED_MESSAGES`), so
`ROS_PULSE_EMIT_IDLE=1` is idiomatic. Parsing treats the string `"1"` as true and everything else
(including unset) as false, so a malformed value fails safe to the quiet default.

## References

- `src/probe/interposers.cpp:83-92` — the flush emit conditions.
- `src/core/topic_registry.cpp:163-186` — `snapshot()`; only declared topics are returned.
- `include/ros2_pulse/core/topic_registry.hpp:27-33` — `sTopicStat`.
- `include/ros2_pulse/core/topic_registry.hpp:67` — `shouldFilter` (existing static predicate, the
  pattern followed here).
- [KNOWN_ISSUES.md](../KNOWN_ISSUES.md) #7.

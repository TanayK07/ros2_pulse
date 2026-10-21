# Expected-rate alerting

Write down what healthy looks like once, for the whole process tree, and the probe flags every
window that disagrees. No `diagnostic_updater` code in each node.

## The spec

```yaml
# /etc/pulse/expected.yaml
topics:
  /scan:    {min_hz: 18, max_hz: 22, side: recv}
  /scan:    {min_hz: 18, side: pub}             # same topic, other end
  /points:  {min_hz: 25, transport: intra}
  /cmd_vel: {max_gap_ms: 30, side: pub}
nodes: [/perception, /planner]
```

```bash
export ROS_TOPIC_STATS_EXPECTED=/etc/pulse/expected.yaml
```

The file is a small documented subset of YAML (one flow map per topic rule), because the probe
carries no YAML library. A spec that cannot be read or parsed is reported once on stderr and
alerting stays off; it never crashes the host.

| Key | Values | Meaning |
|---|---|---|
| `min_hz` | number | Lowest acceptable rate. |
| `max_hz` | number | Highest acceptable rate. Put it on `side: pub`: several subscriber processes on one topic each report their own receive rate. |
| `max_gap_ms` | number | Longest acceptable gap between two messages. Turns gap tracking on by itself. |
| `side` | `recv` (default), `pub` | Which end of the topic the rule is about. |
| `transport` | `any` (default), `inter`, `intra` | Which path to measure. `any` sums the two receive paths and takes the busier publish path, since one publish can fire both. |
| `nodes` | list | Nodes that must be alive. |

## What a violation looks like

```text
WARN TOPIC /scan hz=1.200000 expected=[18,22]
WARN TOPIC /cmd_vel max_dt_ms=812.400 expected_max_gap_ms=30
WARN NODE /planner missing
```

In jsonl the same data is in `warns`, with `kind` set to `topic_rate`, `topic_gap` or
`node_missing`. A rule can break its rate bound and its gap bound in the same window, which
gives two warns.

Rules are checked when a window is written, never on the message path. Each process judges
only the endpoints it hosts. The first window after attach and the last one at exit are not
judged, so a normal start or stop never alerts.

(max-gap)=

## Frozen topics (max_gap_ms)

A rate bound cannot see a short freeze. At 50 Hz with `min_hz: 45` on a 5 s window, a 400 ms
freeze (20 lost cycles) still averages 46 Hz and passes. A longer window makes it worse.
`max_gap_ms` bounds the largest gap directly, independent of the window length:

```yaml
topics:
  /scan: {min_hz: 45, max_gap_ms: 60, side: recv}   # too slow and frozen are different faults
```

A gap is reported in every window it is open in, so a topic that went silent keeps firing until
it comes back. If the whole process is frozen (SIGSTOP, a debugger) it writes nothing during the
stall; watch window age from outside with pulse-top or pulse-export for that case.

(pulse-check)=

## pulse-check: exit codes for CI and watchdogs

`pulse-check` reads one or more logs and turns the last window into an exit code. It needs no
ROS environment at run time.

```bash
# live watchdog: the last window is the current state
ros2 run ros2_pulse pulse-check --spec /etc/pulse/expected.yaml /tmp/topic_freq.*.log

# after a test run: skip the exit window of a stack that has stopped
ros2 run ros2_pulse pulse-check --spec expected.yaml --skip-last /tmp/topic_freq.*.log
```

| Exit code | Meaning |
|---|---|
| `0` | Every rule holds. |
| `1` | Violations, printed one per line. |
| `2` | Bad input: no probe windows, an unreadable spec, or a `max_gap_ms` rule with no gap data to check it against. |

Offline, `pulse-check` sees every log at once, so a spec topic that appears in no log is reported
at 0 Hz. Receive rates add up across logs.

# Configuration

The probe is configured with environment variables, read once when it attaches. A bad value
never stops the host process: it is reported once on stderr and the default is used.

```bash
export LD_PRELOAD=libros2_pulse.so
export ROS_TOPIC_STATS_OUTPUT_FILE=/tmp/pulse.log
export ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=5.0
export ROS_TOPIC_STATS_FORMAT=jsonl
```

## Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `ROS_TOPIC_STATS_OUTPUT_FILE` | `$TMPDIR/topic_freq.<pid>.log` | Where windows are appended. One file per process by default (`/tmp` when `TMPDIR` is unset). Set a path to share one file on purpose. |
| `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` | `5.0` | Window length in seconds. |
| `ROS_TOPIC_STATS_FORMAT` | `text` | `text` or `jsonl`. See [output formats](output.md). |
| `ROS_TOPIC_STATS_EXPECTED` | unset | Path to an [expected-rate spec](alerting.md). Violations are written as warns. |
| `ROS_TOPIC_STATS_JITTER` | `0` | `1` measures inter-arrival gaps (`JITTER` lines) and the receive period spread and message age that pulse_bridge publishes. Any `max_gap_ms` rule turns it on by itself. |
| `ROS_TOPIC_STATS_ENDPOINTS` | `0` | `1` adds one `EP` line per publisher and subscription to text output. jsonl always has them. |
| `ROS_PULSE_EMIT_IDLE` | `0` | `1` also writes a `TOPIC /x 0.000000` line for a publisher that sent nothing this window. |
| `ROS_TOPIC_STATS_MAX_BYTES` | `10485760` | Rotate the file to `<path>.1` at this size (one old generation is kept). `0` never rotates. |
| `ROS_TOPIC_STATS_QUIET` | `0` | `1` drops the `[ros2_pulse] active` banner. Errors are still printed. |

## Notes

**Per process files.** Each probed process owns its file, so writes never interleave and a
reader can tell processes apart by pid. The default path refuses to follow symlinks because it
lives in a world-writable directory; an explicit path may be a symlink.

**Window length.** Rates are averaged over the window, so a longer window is smoother and slower
to react. A short stall hides inside a long window; use a `max_gap_ms` rule for that (see
{ref}`alerting <max-gap>`). For graph-monitor keep the window under its
stale timeout (see {ref}`pulse_bridge <bridge-graph-monitor>`).

**Gap tracking cost.** `ROS_TOPIC_STATS_JITTER=1` reads the clock once per message: about 24 ns
on x86-64 and 51 ns on a Jetson AGX Orin. That is why it is off by default.

**Silent topics.** A receive side that has delivered at least once keeps reporting an explicit
`0.000000` when it stops, so a stall stays visible. A publisher that never sent anything is left
out unless `ROS_PULSE_EMIT_IDLE=1`.

**Disk.** Worst case per process is twice `ROS_TOPIC_STATS_MAX_BYTES`. The file is reopened
every window, so an external `logrotate` works as well.

# pulse-top

![pulse-top demo](../../docs/assets/pulse-top-demo.gif)

Live terminal dashboard over a `ros2_pulse` probe log. A pure log **consumer**:
no ROS dependency, no node, no subscriptions; it tails the jsonl file the probe
already writes, so watching costs the robot nothing and works over plain ssh or on
a dead log after the incident.

That is the difference from graph-joining monitors (`ros2top`, `ornis`,
`ros2_topic_monitor`, ...): they join the graph to measure it; `pulse-top` reads
what the in-process probe measured, including the intra-process traffic no
graph-side tool can see.

## Run

```bash
pip install ./tools/pulse-top          # or: pipx install ros2-pulse-top (once published)

# probe side: emit jsonl
export ROS_TOPIC_STATS_FORMAT=jsonl
LD_PRELOAD=libros2_pulse.so ros2 launch my_robot bringup.launch.py

# dashboard side (any shell, any machine with the file):
pulse-top                               # every $TMPDIR/topic_freq.<pid>.log (one per probed process)
pulse-top /path/to/shared.log           # one explicit file (e.g. a fleet-wide ROS_TOPIC_STATS_OUTPUT_FILE)
pulse-top '/var/log/topic_freq.*.log'   # a quoted glob; new files are picked up as nodes start
pulse-top --demo                        # self-generated demo graph with a scripted incident
```

## Keys

| Key | Action |
|-----|--------|
| `↑`/`↓` | select topic (detail sidebar follows) |
| `tab` | switch view: Topics · Tree · Nodes · Warns |
| `s` | cycle sort: topic / rate / gap |
| `w` | toggle warns-only filter |
| `q` | quit |

## What it shows

- **Topics**: per-topic publish/intra/receive Hz, max inter-arrival gap, 60-window
  sparkline. Absence renders as `—`; the probe's "not measured" is never shown as 0.
- **Tree**: topic namespace hierarchy with live rates.
- **Nodes**: liveness from `NODE` records; missing nodes flagged from structured warns.
- **Warns**: the probe's structured `warns[]` (`topic_rate`, `topic_gap`,
  `node_missing`), parsed as JSON rather than regex.

## Development

```bash
cd tools/pulse-top
uv venv .venv && uv pip install -p .venv/bin/python -e ".[dev]"
env -u PYTHONPATH .venv/bin/python -m pytest tests/   # clear ROS's pytest plugins
```

## Reading a live stack (many processes, one view)

Every probed process flushes its own window; a 77-node stack is ~15 windows/s interleaved
across files (or within one shared file). The view is built for that:

- **Stale is measured in time**, from the window timestamps: a topic is `stale 12s` once more
  than 1.5× its own window period has passed since its last window, regardless of how many
  other processes' windows landed in between. A 5 Hz topic on a 77-node stack used to read
  `stale 3w` for that reason (Orin, 2026-08-23).
- **Sides merge.** One process publishes `/tf_static`, twenty receive it. A receive-only window
  refreshes the `RECV` fields and leaves `PUB` as learned from the publisher's window.
- **One sparkline sample per period**, whichever process's window lands first in it.
- **Only changed cells repaint.** Textual's `DataTable.update_cell` invalidates and refreshes
  unconditionally; pulse-top diffs against what it last rendered, so an idle screen costs
  nothing over ssh.

`transition_event`, NITROS `_supported_types` and other one-shot topics going stale minutes
after startup is correct: they fired once.

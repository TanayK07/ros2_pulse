# pulse-top

Live terminal dashboard over a `ros2_pulse` probe log. A pure log **consumer**:
no ROS dependency, no node, no subscriptions — it tails the jsonl file the probe
already writes, so watching costs the robot nothing and works over plain ssh or on
a dead log after the incident.

That is the difference from graph-joining monitors (`ros2top`, `ornis`,
`ros2_topic_monitor`, ...): they join the graph to measure it; `pulse-top` reads
what the in-process probe measured, including the intra-process traffic no
graph-side tool can see.

## Run

```bash
pip install ./tools/pulse-top          # or: pipx install ros2-pulse-top (once published)

# probe side — emit jsonl:
export ROS_TOPIC_STATS_FORMAT=jsonl
LD_PRELOAD=libros2_pulse.so ros2 launch my_robot bringup.launch.py

# dashboard side (any shell, any machine with the file):
pulse-top                               # newest $TMPDIR/topic_freq.<pid>.log
pulse-top /path/to/topic_freq.1234.log  # explicit file
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
  sparkline. Absence renders as `—` — the probe's "not measured" is never shown as 0.
- **Tree**: topic namespace hierarchy with live rates.
- **Nodes**: liveness from `NODE` records; missing nodes flagged from structured warns.
- **Warns**: the probe's structured `warns[]` (`topic_rate`, `topic_gap`,
  `node_missing`) — parsed as JSON, no regex.

## Development

```bash
cd tools/pulse-top
uv venv .venv && uv pip install -p .venv/bin/python -e ".[dev]"
env -u PYTHONPATH .venv/bin/python -m pytest tests/   # clear ROS's pytest plugins
```

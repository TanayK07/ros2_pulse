# pulse-top

A live terminal dashboard over the probe's log files. It reads files and nothing else: no ROS
dependency, no node, no subscriptions. Watching costs the robot nothing, works over plain ssh,
and works just as well on logs copied off a robot after an incident.

![pulse-top demo: per-topic rates and sparklines, a /scan stall and a /cmd_vel rate sag](images/pulse-top-demo.gif)

## Run

```bash
pip3 install ros2-pulse-top

pulse-top                               # every $TMPDIR/topic_freq.<pid>.log
pulse-top /path/to/shared.log           # one file
pulse-top '/var/log/topic_freq.*.log'   # a quoted glob, re-read as processes start
pulse-top --demo                        # a scripted demo graph, no robot needed
pulse-top --theme light                 # also: dark (default), terminal
```

Text and jsonl logs can be mixed. `PULSE_TOP_THEME` sets the default theme.

## Views

| Key | Action |
|---|---|
| `↑` / `↓` | Select a topic; the detail panel follows. |
| `tab` | Switch view: Topics, Tree, Nodes, Warns. |
| `s` | Sort by topic, rate or gap. |
| `w` | Show only topics with warns. |
| `q` | Quit. |

- **Topics**: publish, intra-process and receive rates, largest gap and a 60-window sparkline
  per topic. Not measured shows as `—`, never as 0.
- **Tree**: the topic namespace with live rates.
- **Nodes**: liveness, with missing nodes from the spec flagged.
- **Warns**: the probe's warns, plus `recv_lag`.

## Many processes, one view

Each probed process writes its own file. pulse-top merges them: one process publishes
`/tf_static` and twenty receive it, and the row shows both sides. A topic is marked stale once
more than 1.5 of its own window periods pass without a window, measured in time, so a quiet
topic on a busy stack is judged on its own clock.

(recv-lag)=

## recv_lag

The one warn pulse-top works out itself. The probe sees one process at a time, so the publish
rate is in the publisher's log and the callback rate in the subscriber's. When callbacks keep
falling behind the publish rate (default: more than 10 % and more than two messages per window,
for three windows in a row), pulse-top raises `recv_lag` for that topic and subscriber. A
publisher that stopped is treated as a dead upstream, not as lag. `--lag-tol` and
`--lag-windows` tune it.

The full rules are in the
[pulse-top README](https://github.com/TanayK07/ros2_pulse/tree/main/tools/pulse-top#recv_lag-callbacks-under-the-publish-rate).

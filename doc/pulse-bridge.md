# pulse_bridge

`pulse_bridge` is a small sidecar node that reads the probe's jsonl logs and publishes them as
ROS messages:

| Output | Message | For |
|---|---|---|
| `/statistics` | `statistics_msgs/MetricsMessage` | Anything that read rclcpp's built-in topic statistics. |
| `/topic_statistics` | `rosgraph_monitor_msgs/TopicStatistics` | [graph-monitor](https://github.com/ros-tooling/graph-monitor) deadline checks. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostic_aggregator`, `rqt_robot_monitor`. Since 0.7.0. |

It is a separate process on purpose. The probe never joins the DDS graph, so measuring cannot
change what is measured; the only graph traffic is one small message per topic per window, from
a process that is not the one under test.

```bash
# probed processes write jsonl
export ROS_TOPIC_STATS_FORMAT=jsonl
LD_PRELOAD=libros2_pulse.so ros2 launch my_robot bringup.launch.py

# one bridge per machine; it picks up processes that start later
ros2 run ros2_pulse pulse_bridge
ros2 topic echo /statistics
```

## Common parameters

| Parameter | Default | Meaning |
|---|---|---|
| `files` | `[]` | Explicit log paths to follow. |
| `glob` | `$TMPDIR/topic_freq.*.log` | Expanded again on every poll. |
| `poll_period_s` | `1.0` | How often the logs are read. |

A text-format log is skipped with one warning per file.

## /statistics

rclcpp's built-in topic statistics publisher was removed from Rolling
([rclcpp#3203](https://github.com/ros2/rclcpp/pull/3203)), and on Humble it never saw
intra-process traffic ([rclcpp#2911](https://github.com/ros2/rclcpp/issues/2911)). The bridge
publishes the same `statistics_msgs/MetricsMessage`, one per topic per window, without any
change to the nodes being measured.

- `AVERAGE` is the message period in ms and `SAMPLE_COUNT` the messages in the window.
- With `ROS_TOPIC_STATS_JITTER=1` the receive side adds `MAXIMUM`, `MINIMUM` and `STDDEV` of
  the period.
- The rate comes from the subscription when the process has one, else from the publisher.
  Inter- and intra-process report the busier of the two, not the sum.

| Parameter | Default | Meaning |
|---|---|---|
| `topic` | `/statistics` | Where to publish. |
| `unit` | `ms` | `ms`: `AVERAGE` is the period. `Hz`: `AVERAGE` is the rate, no min, max or stddev. |
| `source_name` | `""` | `measurement_source_name`. Empty: the window's node, or `ros2_pulse` for a process with several. |
| `metrics_source` | `topic` | `topic`: `metrics_source` is the topic name. `metric`: rclcpp's own layout, see below. |

`metrics_source:=metric` reproduces what rclcpp wrote: `metrics_source` is `message_age` or
`message_period`, receive side only, all five statistics present (NaN when not measured). Age is
measured at `rmw_take` as local wall clock minus the DDS source timestamp, the same definition
rclcpp uses from Jazzy on. Age needs gap tracking and an inter-process delivery through Cyclone
DDS or Fast DDS.

(bridge-graph-monitor)=

## graph-monitor

With `graph_monitor:=true` the bridge also publishes `rosgraph_monitor_msgs/TopicStatistics`,
the input [graph-monitor](https://github.com/ros-tooling/graph-monitor) checks topic deadlines
against. Each publisher and subscription is reported under its own node, so a component
container reports every node it hosts.

```bash
export ROS_TOPIC_STATS_FORMAT=jsonl ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=1 ROS_TOPIC_STATS_JITTER=1
LD_PRELOAD=libros2_pulse.so ros2 launch my_robot bringup.launch.py

ros2 run ros2_pulse pulse_bridge --ros-args -p graph_monitor:=true
```

graph-monitor marks an endpoint stale after `topic_statistics.stale_timeout_ms` (3000 by
default) without a statistic. One statistic arrives per probe window, up to `poll_period_s`
late, so keep `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` plus `poll_period_s` under that timeout.
The 5 s default window does not fit; the example above uses 1 s. The bridge warns once per log
when a window is too long.

| Parameter | Default | Meaning |
|---|---|---|
| `graph_monitor` | `false` | Publish `TopicStatistics`. |
| `graph_monitor_topic` | `/topic_statistics` | Where to publish. |
| `graph_monitor_stale_timeout_ms` | `3000` | graph-monitor's timeout, only used for the warning. |

| Field | Filled with |
|---|---|
| `statistic_type` | `PUBLISHED_PERIOD` per publisher, `RECEIVED_PERIOD` per subscription, `TAKE_AGE` when message age was measured. |
| `mean` | Mean interval with gap tracking on, else 1 / rate. Mean age for `TAKE_AGE`. |
| `max`, `min` | Largest and smallest interval (or age) with gap tracking on, else 0. `min` is 0 for publishers. |
| `window_count` | Messages in the window, or age samples. |

An endpoint with no traffic is left out, so graph-monitor's stale timeout flags it.

(bridge-diagnostics)=

## Diagnostics for diagnostic_aggregator

*Since 0.7.0.*

With `diagnostics:=true` the bridge publishes a `DiagnosticArray` on `/diagnostics` every poll,
so expected-rate violations reach `diagnostic_aggregator` and `rqt_robot_monitor` without a
`DiagnosedPublisher` in each driver. The verdict is the probe's: the bridge maps each process's
latest `warns` onto levels and does not judge rates itself.

```bash
export ROS_TOPIC_STATS_FORMAT=jsonl ROS_TOPIC_STATS_EXPECTED=/etc/pulse/expected.yaml
LD_PRELOAD=libros2_pulse.so ros2 launch my_robot bringup.launch.py

ros2 run ros2_pulse pulse_bridge --ros-args -p diagnostics:=true -p hardware_id:=robot1 \
  -p expected_file:=/etc/pulse/expected.yaml
```

| Status name | Level |
|---|---|
| `ros2_pulse: <topic> (<side>)` | One per spec rule. `OK`, or `WARN` while the probe reports a rate or gap warn for it. Values: `hz`, the rule's bounds, `max_dt_ms` when measured. |
| `ros2_pulse: node <name>` | One per spec node. `OK` while it is seen, `ERROR` when missing. |
| `ros2_pulse: log <file>` | One per log. `ERROR` once no window has arrived for two window periods plus `poll_period_s`. |
| `ros2_pulse: <topic>` | A warn no rule accounts for. `WARN`. |

| Parameter | Default | Meaning |
|---|---|---|
| `diagnostics` | `false` | Publish `DiagnosticArray`. |
| `diagnostics_topic` | `/diagnostics` | Where to publish. |
| `hardware_id` | `""` | `hardware_id` of every status. |
| `diagnostics_all_topics` | `false` | Also report topics without a rule, as `OK` with their rate. |
| `expected_file` | `""` | The spec. Empty: `$ROS_TOPIC_STATS_EXPECTED` when set. |

Statuses are only published for rules and nodes some local process hosts, so a rule for another
machine's topic does not show up as missing here. Several processes on one rule fold into one
status with the worst level. A process that exits cleanly drops out at once; one that dies
reads as a stale log until it has been silent for five minutes.

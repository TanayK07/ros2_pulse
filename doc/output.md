# Output formats

Every window is written as one block of text lines (the default) or as one JSON object per line
(`ROS_TOPIC_STATS_FORMAT=jsonl`). Both carry the same values under the same rules; only the
encoding differs. pulse-top, pulse-export and `pulse-check` read both. pulse_bridge reads jsonl.

## Text

```text
# ts_ns=1782887153899445923 window_s=5.000
TOPIC /scan 20.000000
PUB /points inter=0.000000 intra=30.000000
JITTER /scan recv max_dt_ms=21.284
LOAN /cloud pub hz=30.000000
RECV /scan inter=20.000000 intra=0.000000
RECV /points inter=0.000000 intra=30.000000
NODE /perception
NODE /planner
WARN TOPIC /scan hz=1.200000 expected=[18,22]
```

| Line | Meaning |
|---|---|
| `# ts_ns=... window_s=...` | Window header: end time in epoch nanoseconds and the measured window length. |
| `TOPIC <topic> <hz>` | Publish rate, inter-process. |
| `PUB <topic> inter= intra=` | Publish rate split by path. Iron and newer, where rclcpp has an intra-process publish tracepoint. |
| `RECV <topic> inter= intra=` | Callback rate in this process, split by transport. This is where intra-process traffic shows up on every distro. |
| `JITTER <topic> pub\|recv max_dt_ms=` | Largest gap between two messages in the window. With `ROS_TOPIC_STATS_JITTER=1` or a `max_gap_ms` rule. |
| `LOAN <topic> pub\|recv hz=` | The part of the rate that used middleware loaned messages. A subset of the total, absent when zero. |
| `EP <topic> <side> id= node= inter= intra=` | One line per publisher or subscription, with `ROS_TOPIC_STATS_ENDPOINTS=1`. |
| `NODE <name>` | A node whose own publishers or subscriptions carried traffic recently. |
| `WARN ...` | An [expected-rate](alerting.md) violation. |

Only lines with something to say are written. A topic with no traffic and no subscriber is
absent rather than `0`, and `/rosout`, `/parameter_events` and `/diagnostics` are left out.

## JSON Lines

One object per window, `\n` terminated, so `tail -f pulse.log | jq .` works.

```json
{"ts_ns":"1782887153899445923","window_s":5.000,
 "topics":[{"topic":"/scan","pub_inter_hz":20.000000,"pub_intra_hz":0.000000,
            "recv_inter_hz":20.000000,"recv_intra_hz":0.000000,
            "recv_endpoint_seen":true,"recv_max_dt_ms":21.284}],
 "endpoints":[{"topic":"/scan","side":"pub","id":3,"node":"/lidar",
               "inter_hz":20.000000,"intra_hz":0.000000}],
 "nodes":["/perception"],
 "warns":[{"kind":"topic_rate","topic":"/scan","hz":1.200000,"min_hz":18,"max_hz":22}]}
```

(Shown across lines here; the probe writes each record on one line.)

Rules a consumer can rely on:

- **Absent means not measured, never 0.** A `pub_*` or `recv_*` pair appears only when that side
  exists in the process. `*_max_dt_ms` appears only when gap tracking measured it.
- **`ts_ns` is a string.** Epoch nanoseconds do not fit in a double, so a JSON number would lose
  digits in JavaScript or `jq`.
- **`topics`, `endpoints`, `nodes` and `warns` are always present**, empty when there is
  nothing to report.
- **`"partial": true`** marks the first window after attach and the last one written at exit.
  Their rates are logged but never judged against the spec.
- **`warns[].kind`** is `topic_rate`, `topic_gap` or `node_missing`, with the numbers as JSON
  numbers. An unbounded `max_hz` is left out, since JSON has no infinity.
- With gap tracking on, receive sides also carry `recv_min_dt_ms`, `recv_stddev_dt_ms` and the
  message age (`recv_age_avg_ms`, `recv_age_min_ms`, `recv_age_max_ms`, `recv_age_stddev_ms`,
  `recv_age_count`), and each subscription in `endpoints` carries its own copies.
- Names are escaped per RFC 8259, so no topic name can break the one-record-per-line framing.

The full field list is in the
[README](https://github.com/TanayK07/ros2_pulse#json-lines-output-ros_topic_stats_formatjsonl).

## Per node, per endpoint

Every publisher and subscription has its own counter, attributed to the node that created it.
The per-topic rates are the sums. The split is in `endpoints` (jsonl) or `EP` lines (text):

```text
EP /scan pub id=3 node=/lidar inter=20.000000 intra=0.000000 max_dt_ms=52.500
EP /scan recv id=7 node=/local_costmap/local_costmap inter=20.000000 intra=0.000000
EP /scan recv id=8 node=/local_costmap/local_costmap inter=20.000000 intra=0.000000
```

Two subscriptions of one node on one topic stay two entries, so one of them stalling is visible
on its own. In a component container each node reports its own endpoints.

## Where the numbers come from

- **Publishes** are counted at `rcl_publish`, plus rclcpp's intra-process publish on Iron and
  newer.
- **Receives** are counted at rclcpp's callback start (both transports), or at a successful
  `rmw_take` for rclpy and other rcl clients.
- **Rates** divide by the measured window length, not the configured one.
- **Message age** is taken at `rmw_take`: local wall clock minus the DDS source timestamp.

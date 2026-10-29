# Prometheus, OTLP and Grafana (`pulse-export`)

`pulse-export` puts a probed ROS 2 stack on a Grafana dashboard with no rebuild and no ROS
dependency. Like `pulse-top` it is a pure log consumer: it tails the log files the probe
already writes (the default text format or jsonl), serves them as Prometheus metrics on `/metrics`, and can push the same series
to an OpenTelemetry collector over OTLP/HTTP. Standard library only; it ships in the
`ros2-pulse-top` package.

## One command: Prometheus + Grafana

```bash
git clone https://github.com/TanayK07/ros2_pulse && cd ros2_pulse
docker compose -f examples/grafana/docker-compose.yml up --build
# http://localhost:3000 opens on the ros2_pulse dashboard, no login needed to view
```

The compose file ([`examples/grafana/`](../examples/grafana/)) runs three containers:
`pulse-export` reading the host's `/tmp/topic_freq.*.log` (the probe's default output, one file
per probed process), Prometheus scraping it every second, and Grafana with the datasource and
dashboard provisioned. Start the probe on the robot as usual (either output format works):

```bash
LD_PRELOAD=libros2_pulse.so ros2 launch my_robot bringup.launch.py
```

Set `PULSE_LOG_DIR` if the probe writes somewhere other than `/tmp` (a non-default `TMPDIR`).
No robot at hand: `PULSE_EXPORT_ARGS=--demo docker compose -f examples/grafana/docker-compose.yml up --build`
exports pulse-top's scripted demo graph (a `/scan` stall, a `/cmd_vel` rate sag, a `recv_lag`
on `/camera/image_raw`, `/localization` going missing).

The example is for a local look: anonymous viewing is on and the Grafana admin password is the
default. For an existing Prometheus, skip compose and point a scrape job at the exporter.

## Running the exporter directly

```bash
pipx install ros2-pulse-top    # other ways: tools/pulse-top/README.md#install
pulse-export                                   # every $TMPDIR/topic_freq.<pid>.log, port 10060
pulse-export '/var/log/topic_freq.*.log'       # a quoted glob; new files are picked up as nodes start
pulse-export --otlp http://localhost:4318      # also push OTLP/HTTP JSON every 15 s
pulse-export --once /tmp/topic_freq.4242.log   # print the /metrics text once and exit
```

| Flag | Default | Meaning |
|---|---|---|
| `file` | every `$TMPDIR/topic_freq.*.log` | Log path or quoted glob, re-expanded on every poll. |
| `--port` | `10060` | Prometheus port; `0` disables the HTTP server (OTLP only). 9464 before pulse-top 0.7.0. |
| `--bind` | `0.0.0.0` | Listen address. Use `127.0.0.1` to keep topic names off the network. |
| `--poll` | `1.0` | Seconds between log reads. |
| `--otlp URL` | off | Push OTLP/HTTP JSON here. A bare `http://host:4318` gets `/v1/metrics` appended. |
| `--otlp-interval` | `15` | Seconds between pushes. |
| `--otlp-header K=V` | none | Extra request header, repeatable (for example `Authorization=Basic ...`). |
| `--lag-tol`, `--lag-windows` | `0.10`, `3` | The `recv_lag` detector, same rules as [pulse-top](../tools/pulse-top/README.md). |
| `--demo` | off | Export pulse-top's self-generated demo log. |
| `--once` | off | Read the log once, print the exposition, exit. |

## Metrics

Every series is labelled `pid`, taken from the probe's file name `topic_freq.<pid>.log` (jsonl
records carry no pid). A log with any other name, such as a shared
`ROS_TOPIC_STATS_OUTPUT_FILE`, is labelled with its file name instead.

| Metric | Type | Labels | Meaning |
|---|---|---|---|
| `ros2_pulse_topic_publish_rate_hertz` | gauge | `pid`, `topic`, `path` | Publish rate, `path` is `inter` or `intra` process. |
| `ros2_pulse_topic_receive_rate_hertz` | gauge | `pid`, `topic`, `path` | Subscription callback rate. |
| `ros2_pulse_topic_max_gap_seconds` | gauge | `pid`, `topic`, `side` | Largest inter-arrival gap, `side` is `pub` or `recv`. Only with `ROS_TOPIC_STATS_JITTER`. |
| `ros2_pulse_topic_loaned_rate_hertz` | gauge | `pid`, `topic`, `side` | Middleware-loaned share of the publish (`pub`) or receive (`recv`) rate, a subset of the total. Absent when the window saw no loan. |
| `ros2_pulse_topic_recv_lag_deficit_ratio` | gauge | `pid`, `topic` | While a `recv_lag` warn is active: `(pub - recv) / pub`. `pid` is the subscriber's. |
| `ros2_pulse_warn_active` | gauge | `pid`, `kind`, `topic`, `node` | `1` per warn in the latest window: `topic_rate`, `topic_gap`, `node_missing`, `recv_lag`. The probe's warns name no side or transport, so a `pub` and a `recv` rule warning on one topic in one window are one series. `warns_total` counts both. |
| `ros2_pulse_node_up` | gauge | `pid`, `node` | `1` if listed in the latest window, `0` if the probe reports it missing. |
| `ros2_pulse_node_last_seen_age_seconds` | gauge | `node` | Seconds since a window last listed the node. |
| `ros2_pulse_process_window_seconds` | gauge | `pid` | The process's window period. |
| `ros2_pulse_process_last_window_age_seconds` | gauge | `pid` | Seconds since the process's last window was read. Grows when a process stops flushing. |
| `ros2_pulse_process_last_window_timestamp_seconds` | gauge | `pid` | Probe timestamp of that window, Unix seconds. |
| `ros2_pulse_windows_total` | counter | `pid` | Windows read. |
| `ros2_pulse_warns_total` | counter | `pid`, `kind` | Probe warns read. Catches a warn that came and went between two scrapes. |
| `ros2_pulse_lines_skipped_total` | counter | none | Lines that were not probe output in either format (truncated, foreign). |

Useful queries: `max by (topic) (ros2_pulse_topic_publish_rate_hertz)` is the probe's own
headline rate (one publish can fire both paths, so the busier one, not the sum);
`sum by (topic, pid) (ros2_pulse_topic_receive_rate_hertz)` is callbacks per subscribing
process; `increase(ros2_pulse_warns_total[5m]) > 0` alerts on any probe warn. Ready-made
rules are under [Alerting](#alerting).

## What a scrape means

A scrape shows each process's **latest** window, and keeps the probe's rule that absent is not
zero:

- A key the probe did not write (no receive side in this process, no gap measured) is an
  absent series, never a `0` sample. A topic missing from a process's latest window drops out
  of that process's series.
- A process whose last window is older than two of its periods plus one poll (a crash, a hang,
  a clean exit) loses its topic, node and warn series, so a dead publisher's last rate is not
  scraped as if current. Its `process_last_window_age_seconds` keeps growing, which is the
  series to alert on. After five minutes of silence, or four window periods if longer, it is
  forgotten entirely. A jsonl log's exit window (`"partial": true` after another window)
  forgets it at once.
- Ages use the exporter's clock at read time, not the probe's `ts_ns`, so a log copied from
  another machine or replayed after the fact does not read as hours stale. On start,
  pulse-export skips a log whose mtime is older than the forget horizon: the probe never deletes
  its log, so `$TMPDIR` keeps one from every earlier run. `--once` reads a log however old.
- Scrape at or under the window period (`ROS_TOPIC_STATISTICS_PUBLISH_PERIOD`, 5 s by default).
  A slower scrape skips windows; the counters still count them.

## Alerting

[`examples/prometheus/ros2_pulse_alerts.yml`](../examples/prometheus/ros2_pulse_alerts.yml)
holds Prometheus alerting rules over these metrics. Add it to `rule_files:` in your
`prometheus.yml`. CI runs `promtool check rules` on it and `promtool test rules` on
[`ros2_pulse_alerts_test.yml`](../examples/prometheus/ros2_pulse_alerts_test.yml).

| Alert | Fires when | `for` |
|---|---|---|
| `ROS2PulseTopicRate` | The probe reports a `topic_rate` warn. | 15s |
| `ROS2PulseTopicGap` | The probe reports a `topic_gap` warn. | 15s |
| `ROS2PulseNodeMissing` | The probe reports a `node_missing` warn. | 15s |
| `ROS2PulseWarnRecurring` | Six of the last 5 minutes' windows carried a warn of one kind (severity `info`). | none |
| `ROS2PulseRecvLag` | A `recv_lag` deficit is exported for a topic. | 15s |
| `ROS2PulseTopicSilent` | A topic that published in the last 15 minutes has no publish series, or only `0`, while its process still writes windows. | 1m |
| `ROS2PulseProcessStale` | A process has written no window for three of its window periods. | 15s |
| `ROS2PulseExporterDown` | `up{job="ros2_pulse"} == 0`. Change the job name to yours. | 1m |

- The four warn alerts need an [expected-rate spec](../README.md#expected-rate-alerting).
- The `for` durations assume the default 5 s window and a scrape and rule evaluation interval
  at or under it. A warn shorter than `for` does not fire its own alert; one that keeps coming
  back fires `ROS2PulseWarnRecurring`, which names the kind but not the topic. A warn that lasts
  fires both.
- A stopped publisher has no series to compare with `0`, so `ROS2PulseTopicSilent` compares with
  the last 15 minutes. It resolves 15 minutes after the topic stops, and it fires for a topic
  that publishes less than once a minute. Exclude such topics with a `topic!~` matcher. To keep
  alerting until the topic is back, give it a `min_hz` or `max_gap_ms` rule in the spec: the
  probe then warns every window while it is silent. With gap tracking on
  (`ROS_TOPIC_STATS_JITTER=1`), `ros2_pulse_topic_max_gap_seconds{side="pub"}` is the time since
  the topic's last publish.
- `ROS2PulseProcessStale` is a `warning`: it cannot tell a crash from a clean exit of a
  text-format log (only jsonl marks its exit window), or from a process with no reported topics
  (only `/rosout`, `/parameter_events`, `/diagnostics`, or services) whose nodes have been idle
  for three windows, since the probe writes no window then. It resolves when pulse-export
  forgets the process.
- No rule fires while pulse-export follows no log at all (a glob or `PULSE_LOG_DIR` that matches
  nothing, the probe not preloaded, the stack stopped). To alert on that, use
  `ros2_pulse_lines_skipped_total unless on (job, instance) ros2_pulse_process_window_seconds`
  with `for: 5m`. pulse-export always exports the first series, so this needs no job name.

Two other ways to get alerted without watching a terminal:

- `pulse_bridge` with `diagnostics:=true` publishes the probe's warns on
  [`/diagnostics`](../README.md#diagnostic_aggregator-diagnostics), for `diagnostic_aggregator`
  and `rqt_robot_monitor`.
- [`pulse-check`](../README.md#expected-rate-alerting) turns the last window into an exit code
  for a watchdog, systemd or CI.

## OTLP

`--otlp` posts an `ExportMetricsServiceRequest` in the OTLP/HTTP JSON encoding: gauges as
gauges, counters as cumulative monotonic sums with the `_total` suffix dropped (an
OTLP-to-Prometheus translation adds it back, so the dashboard queries work on either path),
UCUM units (`Hz`, `s`, `1`), resource `service.name=ros2_pulse` and `host.name`. It uses
`urllib`, so there is no `opentelemetry-sdk` to install; a failed push is reported once per
outage on stderr and retried on the next interval. Checked against OpenTelemetry Collector
0.128.0 (`partialSuccess` empty, no rejected points).

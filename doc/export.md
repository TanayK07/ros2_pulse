# Prometheus, OTLP and Grafana

`pulse-export` serves the probe's logs as Prometheus metrics and can push the same series to an
OpenTelemetry collector. Like pulse-top it only reads files: no ROS dependency, no graph
presence, Python standard library only. It ships in the same pip package.

```bash
pipx install ros2-pulse-top   # see the pulse-top page for other ways

pulse-export                                   # every $TMPDIR/topic_freq.<pid>.log on :10060/metrics
pulse-export '/var/log/topic_freq.*.log'       # a quoted glob, re-read as processes start
pulse-export --otlp http://localhost:4318      # also push OTLP/HTTP JSON every 15 s
pulse-export --once /tmp/topic_freq.4242.log   # print one scrape and exit
```

## Grafana in one command

```bash
git clone https://github.com/TanayK07/ros2_pulse && cd ros2_pulse
docker compose -f examples/grafana/docker-compose.yml up --build
# open http://localhost:3000
```

This runs pulse-export over the host's `/tmp/topic_freq.*.log`, Prometheus scraping it every
second, and Grafana with the dashboard already provisioned. No robot at hand:
`PULSE_EXPORT_ARGS=--demo` exports pulse-top's demo graph instead. The example enables anonymous
viewing and keeps Grafana's default admin password, so use it for a local look only.

## Flags

| Flag | Default | Meaning |
|---|---|---|
| `file` | every `$TMPDIR/topic_freq.*.log` | Log path or quoted glob. |
| `--port` | `10060` | Prometheus port. `0` turns the HTTP server off. 9464 before pulse-top 0.7.0. |
| `--bind` | `0.0.0.0` | Listen address. `127.0.0.1` keeps topic names off the network. |
| `--poll` | `1.0` | Seconds between log reads. |
| `--otlp URL` | off | Push OTLP/HTTP JSON. `/v1/metrics` is appended to a bare host. |
| `--otlp-interval` | `15` | Seconds between pushes. |
| `--otlp-header K=V` | none | Extra request header, repeatable. |
| `--lag-tol`, `--lag-windows` | `0.10`, `3` | The `recv_lag` detector, as in {ref}`pulse-top <recv-lag>`. |
| `--demo` | off | Export the demo graph. |
| `--once` | off | Print one exposition and exit. |

## Metrics

Series are labelled `pid`, taken from the log's file name (`topic_freq.<pid>.log`; any other
name is used as is), except `ros2_pulse_node_last_seen_age_seconds`, which is per node across
processes.

| Metric | Labels | Meaning |
|---|---|---|
| `ros2_pulse_topic_publish_rate_hertz` | `topic`, `path` | Publish rate, `path` is `inter` or `intra`. |
| `ros2_pulse_topic_receive_rate_hertz` | `topic`, `path` | Callback rate. |
| `ros2_pulse_topic_max_gap_seconds` | `topic`, `side` | Largest gap, with gap tracking on. |
| `ros2_pulse_topic_loaned_rate_hertz` | `topic`, `side` | Loaned share of the rate. |
| `ros2_pulse_topic_recv_lag_deficit_ratio` | `topic` | `(pub - recv) / pub` while `recv_lag` is active. |
| `ros2_pulse_warn_active` | `kind`, `topic`, `node` | `1` per warn in the latest window. |
| `ros2_pulse_warns_total` | `kind` | Warns read, so a warn between two scrapes still counts. |
| `ros2_pulse_node_up` | `node` | `1` when listed, `0` when reported missing. |
| `ros2_pulse_node_last_seen_age_seconds` | `node` | Seconds since a window listed the node. |
| `ros2_pulse_process_last_window_age_seconds` | | Seconds since the process last wrote. Alert on this for a hung process. |
| `ros2_pulse_process_window_seconds` | | The process's window length. |
| `ros2_pulse_windows_total` | | Windows read. |

Useful queries:

```text
max by (topic) (ros2_pulse_topic_publish_rate_hertz)     # headline publish rate per topic
sum by (topic, pid) (ros2_pulse_topic_receive_rate_hertz) # callbacks per subscribing process
increase(ros2_pulse_warns_total[5m]) > 0                  # any probe warn
```

## What a scrape means

- Each scrape shows every process's latest window. A value the probe did not measure is a
  missing series, never `0`.
- A process that has not written for two window periods plus one poll loses its topic, node and
  warn series, so a dead publisher's last rate is not reported as current. Its window age keeps
  growing. After five minutes it is forgotten.
- Scrape at or faster than the window period (5 s by default). Slower scrapes skip windows; the
  counters still count them.

## Alerting

[`examples/prometheus/ros2_pulse_alerts.yml`](https://github.com/TanayK07/ros2_pulse/blob/main/examples/prometheus/ros2_pulse_alerts.yml)
holds Prometheus alerting rules, with promtool unit tests: probe warns, `recv_lag`, a topic that
stopped publishing, a process that stopped writing, and the exporter itself down. The list and
its caveats are in
[docs/EXPORT.md](https://github.com/TanayK07/ros2_pulse/blob/main/docs/EXPORT.md#alerting).
Without Prometheus, {ref}`/diagnostics <bridge-diagnostics>` and
{ref}`pulse-check <pulse-check>` alert on the same expected-rate spec.

OTLP pushes use the OTLP/HTTP JSON encoding with `service.name=ros2_pulse`, checked against
OpenTelemetry Collector 0.128.0. The full metric reference is in
[docs/EXPORT.md](https://github.com/TanayK07/ros2_pulse/blob/main/docs/EXPORT.md).

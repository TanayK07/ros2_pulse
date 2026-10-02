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
pip3 install ros2-pulse-top
pulse-export                                   # every $TMPDIR/topic_freq.<pid>.log, port 9464
pulse-export '/var/log/topic_freq.*.log'       # a quoted glob; new files are picked up as nodes start
pulse-export --otlp http://localhost:4318      # also push OTLP/HTTP JSON every 15 s
pulse-export --once /tmp/topic_freq.4242.log   # print the /metrics text once and exit
```

| Flag | Default | Meaning |
|---|---|---|
| `file` | every `$TMPDIR/topic_freq.*.log` | Log path or quoted glob, re-expanded on every poll. |
| `--port` | `9464` | Prometheus port; `0` disables the HTTP server (OTLP only). |
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
| `ros2_pulse_topic_recv_lag_deficit_ratio` | gauge | `pid`, `topic` | While a `recv_lag` warn is active: `(pub - recv) / pub`. `pid` is the subscriber's. |
| `ros2_pulse_warn_active` | gauge | `pid`, `kind`, `topic`, `node` | `1` per warn in the latest window: `topic_rate`, `topic_gap`, `node_missing`, `recv_lag`. |
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
process; `increase(ros2_pulse_warns_total[5m]) > 0` alerts on any probe warn.

## What a scrape means

A scrape shows each process's **latest** window, and keeps the probe's rule that absent is not
zero:

- A key the probe did not write (no receive side in this process, no gap measured) is an
  absent series, never a `0` sample. A topic missing from a process's latest window drops out
  of that process's series.
- A process whose last window is older than two of its periods plus one poll (a crash, a hang,
  a clean exit) loses its topic, node and warn series, so a dead publisher's last rate is not
  scraped as if current. Its `process_last_window_age_seconds` keeps growing, which is the
  series to alert on. After five minutes of silence it is forgotten entirely.
- Ages use the exporter's clock at read time, not the probe's `ts_ns`, so a log copied from
  another machine or replayed after the fact does not read as hours stale.
- Scrape at or under the window period (`ROS_TOPIC_STATISTICS_PUBLISH_PERIOD`, 5 s by default).
  A slower scrape skips windows; the counters still count them.

## OTLP

`--otlp` posts an `ExportMetricsServiceRequest` in the OTLP/HTTP JSON encoding: gauges as
gauges, counters as cumulative monotonic sums with the `_total` suffix dropped (an
OTLP-to-Prometheus translation adds it back, so the dashboard queries work on either path),
UCUM units (`Hz`, `s`, `1`), resource `service.name=ros2_pulse` and `host.name`. It uses
`urllib`, so there is no `opentelemetry-sdk` to install; a failed push is reported once per
outage on stderr and retried on the next interval. Checked against OpenTelemetry Collector
0.128.0 (`partialSuccess` empty, no rejected points).

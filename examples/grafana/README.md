# ros2_pulse on Grafana

```bash
docker compose -f examples/grafana/docker-compose.yml up --build      # from the repo root
# http://localhost:3000 -> dashboard "ros2_pulse"
```

`pulse-export` reads the host's `/tmp/topic_freq.*.log` (run the probe with
`ROS_TOPIC_STATS_FORMAT=jsonl`), Prometheus scrapes it, Grafana shows it. `PULSE_LOG_DIR`
changes the log directory; `PULSE_EXPORT_ARGS=--demo` exports a scripted demo graph instead.

| File | What |
|---|---|
| `docker-compose.yml` | pulse-export (built from `tools/pulse-top`), Prometheus, Grafana |
| `prometheus.yml` | one scrape job, 1 s interval |
| `provisioning/` | Prometheus datasource (uid `prometheus`) and the dashboard provider |
| `dashboards/ros2_pulse.json` | the dashboard; import it into any Grafana with a Prometheus datasource |

Metric reference and semantics: [docs/EXPORT.md](../../docs/EXPORT.md).

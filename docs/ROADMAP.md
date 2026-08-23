# Roadmap

Feature work, numbered so commits, tests and the CHANGELOG can refer to an item (`ROADMAP R5`).
The numbers are not priority order. Defects are tracked in [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

## R1. Expected-rate spec and alerting (done, 0.2.0)

`ROS_TOPIC_STATS_EXPECTED` names a spec file (a documented YAML subset, no YAML library in the
probe). Each flush window is checked against it and violations are written as `WARN TOPIC` /
`WARN NODE` lines, with the first window and the atexit window skipped. `pulse-check` re-derives
the verdict from a log with no ROS installed and exits 0/1/2. See README, "Expected-rate
alerting".

Open: a `for: N` debounce so a rule fires only after N consecutive bad windows (R1.1).

## R2. Timer liveness (rescoped, not started)

The original idea was per-timer rate monitoring pitched at control loops. The common ROS 2
control loops (`ros2_control`, Nav2, MoveIt Servo) do not run on `rclcpp` timers, so the timer
tracepoints cannot see them; R5 covers their stalls through the topics they publish. What
remains of R2 is liveness for nodes that have timers but no topic traffic, which today read as
quiet. It ships only if R5 turns out not to cover the need.

## R3. Distro matrix (done, 0.2.0 and 0.3.0)

Blocking CI on `ros:humble`, `ros:jazzy` and `ros:kilted`, a non-blocking rolling lane, the
`rclcpp_intra_publish` hook for publish-side intra-process rates on Iron and newer, and the
LTTng coexistence test on distros whose tracetools link lttng-ust.

## R4. Positioning docs (done)

[ALTERNATIVES.md](ALTERNATIVES.md) covers CARET (same hook mechanism, different goal),
`diagnostic_updater`, rclpy limits and the per-distro tracing situation.

## R5. Gap and jitter visibility (done, 0.3.0)

A windowed mean cannot see a stall: at 50 Hz with `min_hz: 45` and a 5 s window the rule only
fires after more than half a second of dead time. `ROS_TOPIC_STATS_JITTER=1` records the largest
inter-arrival gap per endpoint and side, `max_gap_ms` rules bound it, and `pulse-check` gates on
it. Measured cost: +24 ns per message on x86-64, +51 ns on a Jetson AGX Orin
([test/orin/RESULTS.md](../test/orin/RESULTS.md)).

## R6. Output and ecosystem

- `ROS_TOPIC_STATS_FORMAT=jsonl`, one object per window: done (0.3.0).
- `ROS_TOPIC_STATS_QUIET=1` banner suppression: done (0.3.0).
- RMW support matrix (FastDDS, CycloneDDS with and without iceoryx, Zenoh): done (0.3.0).
- Callback names via `rclcpp_callback_register`, for readable labels: open.
- An opt-in Prometheus/OpenTelemetry exporter that reads the log as a sidecar, so the probe
  itself stays network-free: open.
- apt packages through bloom/rosdistro for humble, jazzy and kilted: open.

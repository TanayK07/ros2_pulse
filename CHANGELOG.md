# Changelog

All notable changes to this project are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/); versions follow [SemVer](https://semver.org/).

## [Unreleased]

## [0.1.0] - 2026-07-01

Initial release.

### Added
- `libros2_pulse.so` — `LD_PRELOAD` probe over the ROS 2 tracetools instrumentation layer.
- Per-topic frequency (Hz) for inter-process (via `rcl_publish`) and intra-process (via
  `callback_start`'s `is_intra_process` flag) traffic, plus active-node listing.
- Pure-C++ core (`TopicRegistry`, `Timer`) with no ROS dependency; lock-free hot path
  (per-endpoint relaxed atomics + thread-local cache); lazy callback→topic resolution.
- Rolling file output in a `TOPIC` / `RECV` / `NODE` format; configurable via
  `ROS_TOPIC_STATS_OUTPUT_FILE` and `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD`.
- Unit tests (GTest) and integration tests (launch/pytest) — 13 tests.
- Benchmark harness (`bench/`) comparing against eBPF-uprobe and LTTng/ros2_tracing, with an
  interleaved-trials overhead measurement and a hot-path microbench.
- On-Orin field-test kit (`test/orin/`).

### Validated
- ROS 2 Humble, FastRTPS and CycloneDDS (middleware-agnostic; hooks above the DDS vendor).
- CPU overhead within measurement noise at ~4900 msg/s across 53 mixed topics.

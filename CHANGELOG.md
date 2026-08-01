# Changelog

All notable changes to this project are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/); versions follow [SemVer](https://semver.org/).

## [Unreleased]

## [0.2.0] - 2026-08-01

Hardening + honesty release: two full test-first audit rounds (15 issues found, fixed and
regression-tested), the Jazzy+ publish-side intra tracepoint, expected-rate alerting, a
three-distro CI matrix, and benchmarks with error bars.

### Added
- **Expected-rate alerting (ROADMAP R1):** `ROS_TOPIC_STATS_EXPECTED` spec file (documented
  YAML subset, zero-dependency parser), flush-time `WARN TOPIC` / `WARN NODE` lines with
  first-window grace, and the no-ROS `pulse-check` CLI (exit 0/1/2) for watchdogs/CI.
- **Publish-side intra-process rates on Jazzy+ (ROADMAP R3):** the probe hooks
  `rclcpp_intra_publish` and emits an additive `PUB <topic> inter=… intra=…` line (on Humble
  the tracepoint doesn't exist; receive-side intra still covers it).
- **CI matrix:** blocking `industrial_ci` lanes on ros:humble / ros:jazzy / ros:kilted plus a
  non-blocking rolling lane, and three standalone core lanes (fast / TSan / ASan+UBSan).
- Size-based output rotation (`ROS_TOPIC_STATS_MAX_BYTES`, default 10 MiB, `<path>.1`).
- Per-process default output path (`topic_freq.<pid>.log`).
- Stall visibility: proven receive endpoints emit an explicit `RECV … 0.000000` line.
- Idle-topic suppression by default with `ROS_PULSE_EMIT_IDLE=1` opt-in.
- `docs/`: KNOWN_ISSUES tracker (15 issues, all fixed), per-issue research notes, ROADMAP,
  ALTERNATIVES (CARET / diagnostic_updater / rclpy positioning), DESIGN.

### Fixed
- First window reported up to ~2× Hz (timer first-fire off-by-one + nominal instead of
  measured window denominator).
- Exit-time use-after-free: tracepoints firing during static destruction (leaky-singleton
  runtime + atexit final flush).
- `fork()` without exec: child inherited a dead flush thread and never wrote counts
  (pthread_atfork quiescence + rwlock reinit + child re-arm).
- Same-process pub+sub double-count: counters split into pub/recv × inter/intra buckets.
- Env parsing could `std::terminate` the host on malformed values (noexcept whole-token
  parsers with fallbacks).
- Probe exported 74 symbols into every preloaded process; now exactly the 8 `ros_trace_*`
  interposers (visibility flags + linker version script, pinned by CI).
- Thread-local hot-path cache: single-entry thrash (round 2), then stride-aliasing at
  realistic allocator layouts (round 3) — now 256 slots with a stride-breaking hash
  (~0.3–1.2 ns/op; end-to-end cost bounded honestly in `bench/RESULTS.md`).

### Changed
- Benchmarks rewritten with paired, order-alternated trials and SEM error bars across all
  three distros: ≈ +2 % workload CPU at a deliberately hostile 4,900 msg/s stress (pooled
  +1.9 % ± 0.7 %), with controlled attribution — replaces the earlier "within noise" claim.
- README: compatibility matrix, honest overhead figures, positioning vs CARET/eBPF/LTTng.

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

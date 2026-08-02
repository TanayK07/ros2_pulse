# Changelog

All notable changes to this project are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/); versions follow [SemVer](https://semver.org/).

## [Unreleased]

## [0.2.0] - 2026-08-02

Hardening + honesty release: two full test-first audit rounds (15 issues found, fixed and
regression-tested) plus a review round on R1 (7 more), the Iron+ publish-side intra tracepoint,
expected-rate alerting, a three-distro CI matrix, and benchmarks with error bars.

### Added
- **Expected-rate alerting (ROADMAP R1):** `ROS_TOPIC_STATS_EXPECTED` spec file (documented
  YAML subset, zero-dependency parser), flush-time `WARN TOPIC` / `WARN NODE` lines with both
  lifecycle transients grace-skipped (attach ramp-up and the atexit window), and the no-ROS
  `pulse-check` CLI (exit 0/1/2) for watchdogs/CI, with `--skip-last` for post-run gates.
- One topic may carry several rules when they constrain different endpoints, so "the driver
  publishes ~20 Hz **and** we receive ~20 Hz" is one spec; rules are deduplicated on
  (topic, `side`, `transport`) rather than topic name.
- **Publish-side intra-process rates on Iron+ (ROADMAP R3):** the probe hooks
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
- `transport: any` on `side: pub` summed the two publish buckets, but one `publish()` fires
  both `rclcpp_intra_publish` and `rcl_publish` for the same message on Iron+ whenever a
  non-intra subscriber is matched (or the QoS is TransientLocal, Jazzy+) — a healthy 50 Hz
  intra-process publisher read as 100 Hz and tripped `max_hz`. Now the larger bucket; receive
  side keeps the sum, where the buckets are disjoint deliveries.
- Expected-rate alerting judged the atexit window, so a graceful stop could log
  `WARN TOPIC … hz=0.000000` on a perfectly healthy stack — a sub-period tail makes
  `count/window_s` a one-sample estimate. Both transients are now skipped; rates still logged.
  `pulse-check` judged that same window, which broke the post-run CI gate the README
  advertises; use `--skip-last` there.
- A bad `ROS_TOPIC_STATS_EXPECTED` path could stall or balloon the host inside `rcl_node_init`:
  character devices never reached EOF, a FIFO blocked forever in `fopen`, an oversize file was
  read whole (2.76 s and 2.05 GiB RSS for 2 GiB, per process), and a directory yielded empty
  text that parsed as a valid zero-rule spec — alerting silently armed as a permanent no-op.
  Spec files are now `S_ISREG`-gated, opened non-blocking and capped at 1 MiB, and a spec with
  no rules warns and disables. Shared with `pulse-check`, so `--spec /dev/zero` fails fast.
- A leading UTF-8 BOM made a Windows-authored spec fail with an error visually identical to a
  misspelled key (the bytes render invisibly), silently disabling alerting.
- A key repeated inside one rule (`{min_hz: 1, min_hz: 2}`) silently last-won instead of erroring.

### Changed
- Benchmarks rewritten with paired, order-alternated trials and SEM error bars across all
  three distros: ≈ +2 % workload CPU at a deliberately hostile 4,900 msg/s stress (pooled
  +1.9 % ± 0.7 %), with controlled attribution — replaces the earlier "within noise" claim.
- README: compatibility matrix, honest overhead figures, positioning vs CARET/eBPF/LTTng.
- The `[ros2_pulse] active` banner reports `spec=N topics, M nodes`, or `spec=none` for every
  path that disables alerting — unset, unreadable, malformed, or no rules.
- `pulse-check` documents that it sums receive rates across logs, so K subscriber processes on
  one topic report K× the publish rate: bound `side: recv` with `min_hz` for liveness and put
  `max_hz` on `side: pub`.

### Upgrade note
A `side: pub` rule with the default `transport: any` now reports the true produce rate instead
of the inter+intra sum. On Iron+ with intra-process comms enabled, specs whose bounds were
tuned against the doubled figure will need their `max_hz` (and any `min_hz` above the real rate)
revisited. Humble is unaffected — it has no publish-side intra tracepoint.

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

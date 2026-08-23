# Changelog

All notable changes to this project are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/); versions follow [SemVer](https://semver.org/).

## [Unreleased]

## [0.4.1] - 2026-08-23

Packaging fix for the apt release; no behaviour change.

### Fixed
- `babeltrace` is no longer a declared `test_depend`: it has no rosdep mapping for RHEL, and
  humble, jazzy and kilted all release on RHEL, so the declaration blocked bloom's RPM
  generation and would have failed the build farm. The LTTng coexistence test already skips
  when no trace viewer is installed; CI installs `babeltrace2` explicitly so the jazzy and
  kilted lanes keep running it.

### Added
- **Observer-effect bench (`bench/run_observer_effect.sh`, raw under `bench/out/observer_effect/`):**
  what `ros2 topic hz` / `echo` cost and what they do to the topic. N=10 rotated arms on the
  stress farm, probe as the in-process ruler. Findings: the stock tools read the rate right
  (publisher held 50.000 Hz, `hz` within 0.3 %); they cost 7 % (`hz`) / 31 % (`echo`) of a
  core per watched 100 KB topic; and on an intra-process topic the watcher switches
  serialization on, +52 % CPU on the watched process, `rcl_publish` path lit in 10/10 trials.
  README "Why", ALTERNATIVES and bench/RESULTS.md carry the numbers.
- **Launch video source (`video/`):** a 40 s Remotion composition: hook, the measured cost of
  `ros2 topic hz` / `echo`, the one-line probe, real `pulse-top` frames through a `/scan` stall,
  the Orin/x86 numbers, CTA. Every on-screen number lives in `video/src/data.ts` with the file
  it cites; `capture_frames.py` labels stall / Warns-tab frames by pixel colour so the cut does
  not depend on capture timing. Rendered MP4 ships as a GitHub Release asset, not in git.
- README demo GIF of `pulse-top --demo` (`docs/assets/pulse-top-demo.gif`, 30 frames through one
  scripted-incident loop, rendered headlessly from Textual screenshots).

### Fixed
- pulse-top: a `topic_rate` warn for an in-range rate could read `20.04Hz > max 22.0Hz`; the
  detail picked "> max" whenever the rate was not below min. It now names the bound actually
  crossed, or the bounds when neither is, and rounds the rate to one decimal. The demo's
  `/cmd_vel` sag (18.4 ± 1.5 Hz against a 19 Hz min) overlapped the bound and fired such
  warns; it now sags to 14.5-17.5 Hz. Window title is `pulse-top`, not the class name.

## [0.4.0] - 2026-08-23

Launch release: the live dashboard lands, and every performance claim now has an aarch64
row behind it: v0.3.0 was run on a production Jetson AGX Orin and the numbers, raw trials
and a redacted production log are committed. pulse-top was then fixed against that very log.

### Added
- **pulse-top (`tools/pulse-top/`):** live terminal dashboard over the probe's jsonl log
  (`pip install ./tools/pulse-top`, command `pulse-top`). A pure log consumer (no ROS
  dependency, no node, no subscriptions), so watching costs the probed system nothing and
  works over ssh or on a dead log post-mortem, unlike graph-joining monitors (ros2top,
  ornis, ...), and it sees the intra-process rates only the in-process probe can measure.
  Topics table with 60-window sparklines and warn-colored rows, namespace tree, node
  liveness, and a retained structured-warns view with ages (a one-window stall stays
  readable instead of blinking for one window period). Two rules carried into the
  UI: an absent jsonl key renders as `—` (never 0), and a topic absent from the current
  window renders as `stale <age>`, never as its old rate. Hardened by review (#32/#33):
  byte-exact tail-following (rotation/truncation-safe, partial multi-byte holds),
  tail-seek attach with a per-poll read cap, and log-derived text rendered without any
  markup parsing so a hostile or corrupt log cannot crash the viewer. `pulse-top --demo`
  runs a scripted-incident graph; 36 tests (model, byte-exact reader, headless pilot) plus a
  hostile-log smoke run in a dedicated no-ROS CI lane.
- **On-Orin validation (`test/orin/RESULTS.md`, raw under `test/orin/out/`):** v0.3.0 run on a
  production Jetson AGX Orin (L4T R36.4, Humble/CycloneDDS, 77 nodes). Hot-path bench on
  aarch64: 0.9 ns/op fixed, 2.1 ns/op alternating; the opt-in jitter clock read costs
  **+51 ns/msg ± 0.6** (x86: +24) and the `CLOCK_MONOTONIC` vDSO works; the syscall-fallback
  risk from the R5 design is closed. Probe loaded into every process of the deployed stack on
  stock binaries, 0 sockets, all v0.3.0 features (`JITTER`, `jsonl`, `QUIET`) verified on
  target. Production log committed with operator node/topic names redacted. README, bench
  results and the R5 design note carry the aarch64 numbers.
- `test/orin/run_orin_probe_test.sh` honors `ROS_TOPIC_STATS_OUTPUT_FILE` and records process
  names with the CPU samples so ON/OFF runs can be matched across a stack restart.
  `ORIN_RUNBOOK.md` Phase 0 no longer needs git credentials on the robot.

### Fixed
- **pulse-top on a multi-process stack.** Staleness was counted in windows, but windows from
  every probed process interleave (77 nodes ≈ 15 windows/s), so a healthy 5 Hz topic read
  `stale 3w` and every row re-rendered every tick; `/tf_static` lost its `PUB` rate whenever a
  subscriber's window landed; the sparkline took one sample per process-window instead of per
  period; and the default path followed only the newest `topic_freq.<pid>.log`, one node of
  the graph. Now: staleness is wall time from the window timestamps (`stale 12s` past
  1.5× the topic's window), publish and receive sides merge per topic, one history sample per
  period, every matching per-pid file is followed (new ones picked up live), and only cells
  whose text or style changed are pushed to the table; Textual's `DataTable.update_cell`
  invalidates and refreshes unconditionally, so the idle repaint cost over ssh is now zero.
  Found on the Orin run (2026-08-23).

## [0.3.0] - 2026-08-20

Observability-formats + soundness release: gap visibility closes the stall blind spot in 0.2.0's
rate alerting, JSON Lines output lands for sidecar exporters, and the RMW matrix / LTTng
coexistence results make the compatibility claims measured rather than assumed.

### Added
- **Gap visibility (ROADMAP R5):** `ROS_TOPIC_STATS_JITTER=1` measures the largest inter-arrival
  gap per endpoint per side and emits `JITTER <topic> <side> max_dt_ms=…`, plus a `max_gap_ms:`
  spec rule, `WARN TOPIC <name> max_dt_ms=… expected_max_gap_ms=…`, and `pulse-check` gating.
  This closes a soundness hole in the 0.2.0 rate alerting: a windowed mean cannot see a stall, so
  at 50 Hz a `min_hz: 45` rule needs >0.5 s of dead time to fire and a 400 ms freeze passes at
  46 Hz. Max gap is window-length-independent, and it also covers `Rate`-driven control loops
  (ros2_control, Nav2, MoveIt Servo) that emit no timer tracepoint, via the topics they publish.
  Costs +24 ns per message when on (~0.012% of a core at 4900 msg/s); default off, and a
  `max_gap_ms` rule enables it implicitly so a declared rule is never silently unchecked.
- `pulse-check` exits 2 when a spec requires a gap the logs cannot answer: measurement absence
  is not a health verdict. Violations it *did* measure are still printed.
- **Quiet banner (ROADMAP R6):** `ROS_TOPIC_STATS_QUIET=1` suppresses the `[ros2_pulse] active`
  stderr banner for stderr-parsing deployments. Narrow on purpose: it silences the
  informational banner only; the one-shot error diagnostics (unreadable/malformed/zero-rule
  spec, unwritable output file) always print, because silently disabled alerting is the 0.2.0
  bug class this flag must not reintroduce.
- **LTTng coexistence test (closes ROADMAP R3):** `test/integration/test_lttng_coexist.py`
  asserts the probe and a live LTTng session capture the same run: probe log rates and >0
  `ros2:*` events via babeltrace, proving the `dlsym(RTLD_NEXT)` forwarding feeds both
  consumers. Runs on the jazzy/kilted CI lanes (new `lttng-tools`/`babeltrace` test_depends);
  skips on stock humble, whose tracetools ships no lttng-ust backend (detected via `ldd`).
- **RMW support matrix (ROADMAP R6):** `test/rmw/run_rmw_matrix.sh` validates the probe
  per-middleware in stock `ros:<distro>` containers: rmw_fastrtps (control leg),
  rmw_cyclonedds plain **and** with iceoryx shared memory (iox-roudi + `<SharedMemory>` config
  + a new fixed-size UInt64 talker/listener pair, because String is not SHM-eligible), and
  rmw_zenoh (no DDS at all; needs the `rmw_zenohd` router for discovery), asserting
  publish-side, receive-side and intra-process rates at 50 Hz ±30% with the same parser the CI
  accuracy suite trusts. All four configurations green on ros:humble / ros:jazzy / ros:kilted
  (2026-08-08). README gained a per-RMW matrix with measured rates, exact package versions and
  the SHM-attribution caveat; evidence lands under `test/rmw/out/`. Containers get a
  per-distro `ROS_DOMAIN_ID` because DDS multicast discovery crosses the docker bridge,
  without it a concurrent run's foreign talker doubles the listener's measured rate.
- **JSON Lines output (ROADMAP R6):** `ROS_TOPIC_STATS_FORMAT=jsonl` re-encodes each flush
  window as one JSON object per line (jsonlines.org) for sidecar exporters and log shippers,
  same emit gates, values and precisions as the text format, which stays the byte-identical
  default. `ts_ns` is a decimal string (int64 > 2^53 would lose digits as a JSON number; the
  OTLP/JSON `timeUnixNano` convention), absent keys mean "not measured" (mirroring the
  `JITTER` has-measured semantics), `warns` are structured objects (`topic_rate` / `topic_gap`
  / `node_missing`) rather than preformatted strings, and user-controlled names are escaped
  per RFC 8259 so a hostile topic name cannot break the one-object-per-line framing. Unknown
  format values warn once on stderr and fall back to text. `pulse-check`/`parseLog` sniff the
  format per line, so jsonl logs keep full CI/watchdog gating (a non-probe file still exits 2,
  never a silent pass); golden-byte unit tests pin both formats.

### Fixed
- The default output path was `/root/ssd2tb/logs/topic_freq.<pid>.log`, the original field-test
  machine's SSD mount. On any machine without that directory a fresh install silently dropped
  every window after a single stderr warning. The default is now `$TMPDIR/topic_freq.<pid>.log`
  (`/tmp` when `TMPDIR` is unset, empty, or relative). Because the new default lives in a
  world-writable sticky directory under a pid-predictable name, its open now refuses symlinks
  (`O_NOFOLLOW`, CWE-379) and both open paths set `O_CLOEXEC`; an explicit
  `ROS_TOPIC_STATS_OUTPUT_FILE` keeps full symlink freedom.

### Upgrade note
Deployments that read the old default location must either set
`ROS_TOPIC_STATS_OUTPUT_FILE=/root/ssd2tb/logs/topic_freq.$$.log` explicitly or start reading
from `/tmp`. Anyone setting the path explicitly (the documented practice) is unaffected.

## [0.2.0] - 2026-08-02

Hardening release: two full test-first audit rounds (15 issues found, fixed and
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
- `docs/`: KNOWN_ISSUES tracker (15 issues, all fixed), ROADMAP,
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
  realistic allocator layouts (round 3), now 256 slots with a stride-breaking hash
  (~0.3-1.2 ns/op; end-to-end cost bounded in `bench/RESULTS.md`).
- `transport: any` on `side: pub` summed the two publish buckets, but one `publish()` fires
  both `rclcpp_intra_publish` and `rcl_publish` for the same message on Iron+ whenever a
  non-intra subscriber is matched (or the QoS is TransientLocal, Jazzy+), so a healthy 50 Hz
  intra-process publisher read as 100 Hz and tripped `max_hz`. Now the larger bucket; receive
  side keeps the sum, where the buckets are disjoint deliveries.
- Expected-rate alerting judged the atexit window, so a graceful stop could log
  `WARN TOPIC … hz=0.000000` on a perfectly healthy stack: a sub-period tail makes
  `count/window_s` a one-sample estimate. Both transients are now skipped; rates still logged.
  `pulse-check` judged that same window, which broke the post-run CI gate the README
  advertises; use `--skip-last` there.
- A bad `ROS_TOPIC_STATS_EXPECTED` path could stall or balloon the host inside `rcl_node_init`:
  character devices never reached EOF, a FIFO blocked forever in `fopen`, an oversize file was
  read whole (2.76 s and 2.05 GiB RSS for 2 GiB, per process), and a directory yielded empty
  text that parsed as a valid zero-rule spec, so alerting silently armed as a permanent no-op.
  Spec files are now `S_ISREG`-gated, opened non-blocking and capped at 1 MiB, and a spec with
  no rules warns and disables. Shared with `pulse-check`, so `--spec /dev/zero` fails fast.
- A leading UTF-8 BOM made a Windows-authored spec fail with an error visually identical to a
  misspelled key (the bytes render invisibly), silently disabling alerting.
- A key repeated inside one rule (`{min_hz: 1, min_hz: 2}`) silently last-won instead of erroring.

### Changed
- Benchmarks rewritten with paired, order-alternated trials and SEM error bars across all
  three distros: ≈ +2 % workload CPU at a harsh 4,900 msg/s stress (pooled
  +1.9 % ± 0.7 %), with controlled attribution; this replaces the earlier "within noise" claim.
- README: compatibility matrix, measured overhead figures, positioning vs CARET/eBPF/LTTng.
- The `[ros2_pulse] active` banner reports `spec=N topics, M nodes`, or `spec=none` for every
  path that disables alerting: unset, unreadable, malformed, or no rules.
- `pulse-check` documents that it sums receive rates across logs, so K subscriber processes on
  one topic report K× the publish rate: bound `side: recv` with `min_hz` for liveness and put
  `max_hz` on `side: pub`.

### Upgrade note
A `side: pub` rule with the default `transport: any` now reports the true produce rate instead
of the inter+intra sum. On Iron+ with intra-process comms enabled, specs whose bounds were
tuned against the doubled figure will need their `max_hz` (and any `min_hz` above the real rate)
revisited. Humble is unaffected; it has no publish-side intra tracepoint.

## [0.1.0] - 2026-07-01

Initial release.

### Added
- `libros2_pulse.so`: an `LD_PRELOAD` probe over the ROS 2 tracetools instrumentation layer.
- Per-topic frequency (Hz) for inter-process (via `rcl_publish`) and intra-process (via
  `callback_start`'s `is_intra_process` flag) traffic, plus active-node listing.
- Pure-C++ core (`TopicRegistry`, `Timer`) with no ROS dependency; lock-free hot path
  (per-endpoint relaxed atomics + thread-local cache); lazy callback→topic resolution.
- Rolling file output in a `TOPIC` / `RECV` / `NODE` format; configurable via
  `ROS_TOPIC_STATS_OUTPUT_FILE` and `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD`.
- Unit tests (GTest) and integration tests (launch/pytest): 13 tests.
- Benchmark harness (`bench/`) comparing against eBPF-uprobe and LTTng/ros2_tracing, with an
  interleaved-trials overhead measurement and a hot-path microbench.
- On-Orin field-test kit (`test/orin/`).

### Validated
- ROS 2 Humble, FastRTPS and CycloneDDS (middleware-agnostic; hooks above the DDS vendor).
- CPU overhead within measurement noise at ~4900 msg/s across 53 mixed topics.

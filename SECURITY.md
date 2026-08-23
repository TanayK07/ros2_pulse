# Security policy

## Supported versions

| Version | Supported |
|---|---|
| 0.2.x | ✅ |
| < 0.2 | ❌ |

## Reporting a vulnerability

Please **do not open a public issue** for security-sensitive reports. Use GitHub's private
[security advisory form](https://github.com/TanayK07/ros2_pulse/security/advisories/new)
instead. You can expect an acknowledgement within a week.

## Threat model notes

`libros2_pulse.so` is injected via `LD_PRELOAD` into every process of a launch, so its own
attack surface matters more than most tools':

- The probe **reads** two kinds of input: environment variables and (optionally) the
  expected-rate spec file named by `ROS_TOPIC_STATS_EXPECTED`. All parsers are `noexcept`,
  whole-token, and fall back to safe defaults: malformed input warns on stderr and degrades,
  never crashes the host process.
- The probe **writes** exactly one artifact: the stats file (`ROS_TOPIC_STATS_OUTPUT_FILE`,
  per-PID by default, size-capped by `ROS_TOPIC_STATS_MAX_BYTES`). Point it only at paths the
  robot user may write; the probe never creates directories, changes permissions, or executes
  anything.
- The probe opens **no sockets** and adds **no DDS traffic**; it requires **no privileges**.
- The dynamic symbol surface is pinned to the 8 `ros_trace_*` interposers by a linker version
  script and enforced in CI (`test/integration/test_symbols.py`), so nothing else is exported
  into the host's namespace.

Reports about crashing, hanging or corrupting a *host* process through any of these inputs
are in scope and very welcome.

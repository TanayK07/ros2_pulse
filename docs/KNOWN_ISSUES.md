# Known issues

Defects found in review and benchmarking, with the PR that fixed each. All fifteen are fixed
and have regression tests; the numbers are referenced from code comments and tests
(`KNOWN_ISSUES #n`). Current overhead figures are in [bench/RESULTS.md](../bench/RESULTS.md);
planned work is in [ROADMAP.md](ROADMAP.md).

| # | Severity | Area | Problem | Fix |
|---|---|---|---|---|
| 1 | High | correctness | Inter-process counter double-counted when publisher and subscriber shared a process | [#4](https://github.com/TanayK07/ros2_pulse/pull/4) |
| 2 | Medium | correctness | "Node liveness" was "ever initialized": nodes were never removed | [#7](https://github.com/TanayK07/ros2_pulse/pull/7) |
| 3 | Medium | efficiency | Unresolvable callbacks (timers, services) took the write lock on every call | [#2](https://github.com/TanayK07/ros2_pulse/pull/2) |
| 4 | Medium | robustness | Default output file was shared by all processes, so writes interleaved | [#6](https://github.com/TanayK07/ros2_pulse/pull/6) |
| 5 | Low | robustness | `std::stod` on a bad env var threw out of a tracepoint and terminated the host | [#5](https://github.com/TanayK07/ros2_pulse/pull/5) |
| 6 | Low | docs | Dangling issue reference; the rclcpp#2911 premise needed a distro caveat | [#1](https://github.com/TanayK07/ros2_pulse/pull/1) |
| 7 | Low | behaviour | Idle topics printed `TOPIC /x 0.000000` every window | [#3](https://github.com/TanayK07/ros2_pulse/pull/3) |
| 8 | High | correctness | First window reported up to 2x the rate (timer first-fire off by one, nominal instead of measured window length) | [#9](https://github.com/TanayK07/ros2_pulse/pull/9) |
| 9 | Medium | robustness | Exit-time use after free: tracepoints could run during static destruction | [#10](https://github.com/TanayK07/ros2_pulse/pull/10) |
| 10 | Medium | robustness | `fork()` without `exec`: the child kept counting but never flushed | [#11](https://github.com/TanayK07/ros2_pulse/pull/11) |
| 11 | Low | behaviour | Output file was append-only and unbounded while the README said "rolling" | [#13](https://github.com/TanayK07/ros2_pulse/pull/13) |
| 12 | Medium | behaviour | A stalled subscription emitted no `RECV` line, so a stall looked like absence | [#12](https://github.com/TanayK07/ros2_pulse/pull/12) |
| 13 | Low | efficiency | Thread-local cache was single-entry; the "0.2 ns/op" figure was the all-hits best case | [#14](https://github.com/TanayK07/ros2_pulse/pull/14) |
| 14 | Low | build | The probe exported every symbol, not just `ros_trace_*`; lint deps were declared but unused | [#15](https://github.com/TanayK07/ros2_pulse/pull/15) |
| 15 | Medium | efficiency | TLS cache hash aliased on allocator strides (about 90 % misses under realistic farms, +2-4 % workload CPU) | [#19](https://github.com/TanayK07/ros2_pulse/pull/19) |

Sub-items referenced from code: 8a is the timer first-fire off-by-one, 8b is the nominal
window denominator (both fixed in #9).

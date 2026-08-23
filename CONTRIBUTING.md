# Contributing to ros2_pulse

Thanks for your interest. This is a small, focused package; contributions that keep it small and
focused are the most welcome.

## Ground rules

- **The core stays pure.** `include/ros2_pulse/core/` and `src/core/` must not depend on ROS,
  rclcpp, or tracetools. All ROS/tracetools coupling lives in `src/probe/`. This is what keeps the
  logic unit-testable in isolation.
- **The hot path stays cheap.** `onPublish` / `onCallbackStart` run per message. No allocations, no
  string hashing, no global locks, per-endpoint atomics + the thread-local cache only.
- **No DDS traffic, no privileges.** The probe writes a local file and opens no sockets. Keep it
  that way.

## Dev workflow

```bash
# unit tests (pure core, no ROS): the fastest loop
GT=/opt/ros/humble/src/gtest_vendor
g++ -O2 -std=c++17 -pthread -Iinclude -I$GT/include -I$GT \
  test/unit/test_topic_registry.cpp src/core/topic_registry.cpp $GT/src/gtest-all.cc -o /tmp/ut && /tmp/ut

# full suite (unit + integration)
colcon build --packages-select ros2_pulse
colcon test --packages-select ros2_pulse && colcon test-result --verbose

# benchmark / overhead (see bench/README.md)
```

## Pull requests

- Add or update tests for any behavior change (`test/unit` for core, `test/integration` for the
  probe end-to-end).
- Run `colcon test` and include the result in the PR.
- For performance-affecting changes, run `bench/run_overhead_repeated.sh` and paste the numbers.
- Keep commits conventional (`feat:`, `fix:`, `perf:`, `docs:`, `test:`).

## Reporting bugs

Open an issue with: ROS distro, RMW (FastRTPS/CycloneDDS), whether intra-process comms are on, the
relevant `pulse.log` window, and `nm -D libtracetools.so | grep -c ros_trace` output.

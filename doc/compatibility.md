# Compatibility and limitations

## Distros

| Distro | Status | Notes |
|---|---|---|
| Humble | CI-tested, apt | Stock tracetools has no LTTng backend, so this is the zero-rebuild option there. |
| Jazzy | CI-tested, apt | tracetools is LTTng-enabled; the probe forwards, so a tracing session runs alongside. |
| Kilted | CI-tested, apt | |
| Rolling | non-blocking CI lane | Watches upstream changes. |

On Iron and newer the probe also hooks rclcpp's intra-process publish tracepoint, which gives
the `PUB` line. Humble has no such tracepoint, so intra-process rates there are measured at the
subscriptions.

## Middleware

The hooks sit above the rmw layer, and each configuration below was measured with
`test/rmw/run_rmw_matrix.sh` on Humble, Jazzy and Kilted: separate talker and listener processes
plus an intra-process node, every rate checked at 50 Hz.

| RMW | Status |
|---|---|
| `rmw_fastrtps_cpp` | CI-tested on every lane |
| `rmw_cyclonedds_cpp` | validated |
| Cyclone DDS with iceoryx shared memory | validated |
| `rmw_zenoh_cpp` | validated |
| `rmw_zenoh_cpp` with Zenoh shared memory | validated |

## Limitations

- **tracetools must be instrumented.** This is the default for the official binaries and Isaac
  ROS. A build with instrumentation compiled out has nothing to hook. The startup banner names
  the tracetools version and warns when it is not one CI covers.
- **Node liveness comes from traffic.** A node is listed while its own publishers or
  subscriptions carried traffic in the last few windows. A node that only runs timers or
  services, with no topic traffic, reads as quiet.
- **rclpy and other non-rclcpp clients** are counted at `rcl_publish` and at the rmw's
  `rmw_take` tracepoint. Cyclone DDS and Fast DDS emit it on every distro, Connext from Jazzy.
  On Humble, serialized and loaned takes (rosbag2 recording, for one) emit none, so those
  receives are not counted.
- **Loan counting wraps two rcl functions by name**, not a tracepoint. Their signatures are the
  same from Humble to Rolling today, but rcl does not promise that across distros the way
  tracetools does.
- **A frozen process writes nothing.** Its flush thread is frozen too. pulse-top and
  pulse-export show the growing window age; the probe reports the full gap once it resumes.
- **Files only.** The probe itself never talks to the network. Use pulse_bridge or pulse-export
  for that.

All defects found so far, with their fixes, are listed in
[KNOWN_ISSUES.md](https://github.com/TanayK07/ros2_pulse/blob/main/docs/KNOWN_ISSUES.md).
Questions and bug reports go to the
[issue tracker](https://github.com/TanayK07/ros2_pulse/issues).

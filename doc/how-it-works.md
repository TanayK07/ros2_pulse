# How it works

```{image} images/how-it-works.svg
:alt: The probe sits between rclcpp and libtracetools inside each process and writes a log file that pulse-top, pulse-export, pulse_bridge and pulse-check read.
:class: diagram
```

rclcpp and rcl call the tracetools functions (`ros_trace_rcl_publish`,
`ros_trace_callback_start`, the init tracepoints and so on) on every publish and every callback,
whether or not a tracing session is running; the LTTng check is inside those functions.
`libros2_pulse.so` exports the same symbols. Loaded with `LD_PRELOAD`, it is found first by the
dynamic linker, so each call lands in the probe, which counts it and forwards to the real
function with `dlsym(RTLD_NEXT, ...)`. Nothing is rebuilt, and an LTTng session still sees every
event.

## Counting

- **Intra-process traffic** is visible because `callback_start` fires for every subscription
  callback and says whether the message came intra-process.
- **Every publisher and subscription has its own counter**, attributed to its node through the
  init tracepoints, which carry the node handle. Per-topic rates are summed when a window is
  written.
- **The hot path** is one relaxed atomic increment behind a small thread-local cache: no lock,
  no string hashing per message. That is about 0.3 to 1.2 ns per message on x86-64, against
  about 158 ns for one LTTng-UST tracepoint.
- **Loaned messages** are the one case a tracepoint cannot tell apart from a copy, so the probe
  also wraps `rcl_publish_loaned_message` and `rcl_take_loaned_message`.
- **Destroyed endpoints** have no tracepoint either; `rcl_publisher_fini`,
  `rcl_subscription_fini` and `rcl_node_fini` are wrapped so a removed subscription does not
  linger as a stalled one.

## Writing

A background thread snapshots and resets the counters every
`ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` seconds, divides by the measured window length, checks the
expected-rate spec if there is one, and appends one window to the log in a single write. It
starts with the first tracepoint, so a process that never touches ROS never writes anything.

The probe opens no socket and joins no DDS graph. Everything that publishes or serves the
numbers (pulse_bridge, pulse-export) is a separate process that reads the file.

## Compared with other tools

| | Intra-process | Stock binaries | Privileges | Cost per watched topic |
|---|---|---|---|---|
| ros2_pulse | yes | yes | none | none extra; about 2 % CPU for the whole process at 4,900 msg/s |
| `ros2 topic hz` / `echo` | no | yes | none | a subscriber per topic, 7 % / 31 % of a core on 100 KB at 50 Hz |
| rclcpp topic statistics | not on Humble | yes | none | removed in Rolling |
| ros2_tracing / LTTng | yes | needs lttng-ust tracetools (not Humble's binaries) | session daemon | offline trace analysis |
| eBPF uprobes | yes | yes | `CAP_SYS_ADMIN`, BTF | a kernel trap per message |

CARET uses the same `LD_PRELOAD` over tracetools mechanism for deep offline latency analysis; it
is complementary rather than always-on. A longer comparison is in
[docs/ALTERNATIVES.md](https://github.com/TanayK07/ros2_pulse/blob/main/docs/ALTERNATIVES.md),
and the design notes are in
[docs/DESIGN.md](https://github.com/TanayK07/ros2_pulse/blob/main/docs/DESIGN.md).

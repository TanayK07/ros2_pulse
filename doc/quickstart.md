# Quickstart

Ten minutes from install to a dashboard and a first alert.

## 1. Install

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-ros2-pulse
```

Packages are built for Humble, Jazzy and Kilted. If apt cannot find the package yet, it is
still in the `ros2-testing` repository for your distro; add it and retry:

```bash
echo "deb http://packages.ros.org/ros2-testing/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2-testing.list
sudo apt update
```

From source instead:

```bash
cd ~/ros2_ws/src && git clone https://github.com/TanayK07/ros2_pulse.git
cd ~/ros2_ws && colcon build --packages-select ros2_pulse && source install/setup.bash
```

The probe needs a `libtracetools.so` with its instrumentation compiled in, which is the default
for the official binaries and for Isaac ROS. To check:

```bash
nm -D $(ros2 pkg prefix tracetools)/lib/libtracetools.so* | grep -c ros_trace   # > 0 is good
```

## 2. Probe your stack

Preload the library into whatever you normally start. Every process the launch spawns inherits
it.

```bash
export LD_PRELOAD=libros2_pulse.so
ros2 launch my_robot bringup.launch.py
```

Each probed process prints one line on stderr, `[ros2_pulse] active ...`, and appends a block
to `$TMPDIR/topic_freq.<pid>.log` (`/tmp` when `TMPDIR` is unset) every 5 seconds. A missing
library is not fatal: `ld.so` prints a warning and starts the process without it, so the
variable is safe to set fleet-wide.

## 3. Watch it

```bash
pip3 install ros2-pulse-top
pulse-top
```

pulse-top reads the log files; it does not join the ROS graph, so it can run over ssh, inside a
container that shares `/tmp`, or on a copy of the logs after an incident. See
[pulse-top](pulse-top.md).

## 4. Say what healthy looks like

Write the rates you expect:

```yaml
# expected.yaml
topics:
  /scan:    {min_hz: 18, max_hz: 22, side: recv}
  /cmd_vel: {max_gap_ms: 100, side: pub}
nodes: [/lidar_driver, /controller_server]
```

and point the probe at it:

```bash
export ROS_TOPIC_STATS_EXPECTED=$PWD/expected.yaml
ros2 launch my_robot bringup.launch.py
```

A topic out of range, a gap longer than allowed or a missing node now adds a `WARN` line to that
window, which pulse-top, pulse-export and pulse_bridge all pick up:

```text
WARN TOPIC /scan hz=1.200000 expected=[18,22]
WARN NODE /lidar_driver missing
```

See [expected-rate alerting](alerting.md) for the rule syntax and `pulse-check`.

## 5. Send it somewhere

- `/statistics`, `/diagnostics` or graph-monitor's `/topic_statistics`:
  `ros2 run ros2_pulse pulse_bridge`, see [pulse_bridge](pulse-bridge.md).
- Prometheus or an OpenTelemetry collector: `pulse-export`, see [export](export.md).

Both read the probe's jsonl format, so add `export ROS_TOPIC_STATS_FORMAT=jsonl` next to
`LD_PRELOAD` when you use pulse_bridge. pulse-top and pulse-export read either format.

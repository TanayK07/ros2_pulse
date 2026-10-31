# Flood test: wire rate vs ros2_pulse rates

One rclcpp controller subscribes to `/cmd_vel` with the probe preloaded. A flood generator on the
other side publishes into it, and a wire capture counts what crosses the network. Six QoS combos
(reliable and best effort, KEEP_LAST depth 1, 10 and 100), 1 s windows, jsonl output.
`compare.py` then lines up the wire rate with the probe's publish and callback rates, window by
window.

| File | What |
|---|---|
| `controller/` | ament package `flood_compare_controller`: `flood_controller` (the subscriber) and `flood_publisher` (a reference flood for dry runs). Not built with ros2_pulse, see Build. |
| `run.sh` | starts the controller under the probe for one combo or all six, writes `flood_out/<combo>/` |
| `compare.py` | the per-window table (stdlib python, no ROS needed) |
| `test_compare.py` | its tests: `python3 -m pytest examples/flood_compare` |

## Build

colcon stops crawling at ros2_pulse's own `package.xml`, so the example package is only built
when you name its directory. In a workspace with ros2_pulse cloned into `src/`:

```bash
cd ~/ros2_ws
colcon build --base-paths src/ros2_pulse src/ros2_pulse/examples/flood_compare/controller
source install/setup.bash
```

With ros2_pulse from apt (`ros-$ROS_DISTRO-ros2-pulse`), pass only the second path.

## Run

```bash
cd ~/ros2_ws/src/ros2_pulse/examples/flood_compare
./run.sh all 30                 # six combos, 30 s each, pauses before each so the generator can switch QoS
./run.sh best_effort 10 30      # or one combo
```

For each combo `run.sh` starts the controller like this:

```bash
LD_PRELOAD=$(ros2 pkg prefix ros2_pulse)/lib/libros2_pulse.so \
ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=1 \
ROS_TOPIC_STATS_FORMAT=jsonl \
ROS_TOPIC_STATS_OUTPUT_FILE=flood_out/best_effort_d10/pulse.jsonl \
  $(ros2 pkg prefix flood_compare_controller)/lib/flood_compare_controller/flood_controller \
  --ros-args -p reliability:=best_effort -p depth:=10
```

Parameters: `topic` (`/cmd_vel`), `reliability` (`reliable` | `best_effort`), `depth` (KEEP_LAST
depth, durability volatile), `work_us` (busy wait per callback, `WORK_US=` in `run.sh`, to make
the controller the bottleneck on purpose). Each callback integrates a pose from the Twist, and
the controller prints its callback total at exit, which should equal the sum of the probe's
callback counts.

The generator has to match: topic `/cmd_vel` (DDS topic `rt/cmd_vel`, type
`geometry_msgs::msg::dds_::Twist_`), the same reliability and depth, durability volatile. A best
effort writer never matches a reliable reader, and `compare.py` warns when the controller
received nothing.

Dry run without the generator: `PUB_HZ=1000 ./run.sh all 10` also starts `flood_publisher` with
the probe preloaded (`pulse_pub.jsonl`), which fills the publish column.

**Keep the traffic on the wire.** Fast DDS delivers between processes on one host over shared
memory by default, and then no packet crosses the network. Run the generator and the controller
on two hosts, or turn shared memory off on both sides (Jazzy and newer:
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`; Humble: an XML profile with a UDPv4 transport and
`useBuiltinTransports=false`). Cyclone DDS uses UDP unless iceoryx is configured.

## What to send back

- `flood_out/` as is (per combo: `pulse.jsonl`, `meta.json`, `controller.log`).
- The wire capture as CSV, 1 s windows, one file for the whole session or `wire.csv` inside
  each combo's directory.
- The generator's rate, payload, DDS vendor, and whether the capture host and the controller host
  share a clock (same host, or NTP and the measured offset).

### Wire CSV

A header row, then one row per window (or per message). Column names are case-insensitive.

| Column | Meaning |
|---|---|
| `timestamp` | unix time in seconds (float) at the END of the row's window. ms / us / ns integers and ISO 8601 also work. `--wire-ts-at start` if it marks the start. |
| `topic` | optional. `/cmd_vel`, `cmd_vel` and `rt/cmd_vel` all match. |
| `msgs_per_s` or `count` | the rate over the row's window, or the messages in it. With neither, every row is one message. |
| `window_s` | optional. Without it a row's window is the gap to the previous row. |

```csv
timestamp,topic,msgs_per_s
1782887153.000,rt/cmd_vel,10012
1782887154.000,rt/cmd_vel,9987
```

Count DATA samples of `rt/cmd_vel` only, not heartbeats or ACKNACKs. Rows with the same timestamp
and topic add up, so one row per writer or per reader is fine; say which one it is. A window with
no row inside the capture reads as 0. A window the capture does not cover gets no value.

## Compare

```bash
python3 compare.py flood_out wire.csv --csv table.csv
```

```
== best_effort_d1  (best_effort, depth 1)
   end (UTC)   win_s     wire_hz      pub_hz       cb_hz    cb-wire   cb/wire   pub-wire  flags
08:00:01.000   1.000      1000.0           -       400.0     -600.0     40.0%          -  partial
08:00:02.000   1.000      1000.0           -       400.0     -600.0     40.0%          -
...
   4 full windows of 6 (partial / gap windows are listed but not counted)
   wire 4000 msgs, callbacks 1600 msgs
   callbacks/wire 40.00%
   largest callback-wire gap -600.0 Hz in the window ending 08:00:02.000
```

The probe's windows are the timeline: a record covers `(ts_ns - window_s, ts_ns]`, wall clock.
Wire rows (and windows of other probed processes) are re-binned onto it by overlap, so the two
sides need not tick in phase, but their clocks must agree; `--wire-offset S` shifts the wire side.
Flags: `partial` is the probe's first window after attach or its exit window, `no-wire` /
`pub-gap` / `cb-gap` mean that source does not cover at least 95 % of the window. Only unflagged
windows count in the totals. `--max-diff-pct P` exits 1 when a full window's callback rate is
more than P % off the wire rate. A difference of a message or two per window is phase between the
two clocks; compare the totals.

## What a disagreement means

ros2_pulse counts inside each process, in rclcpp and rcl, never on the wire:

- **publish** = `rcl_publish` calls in a preloaded publishing process: one per `publish()`,
  however many readers or packets it turns into. Empty unless the publisher runs under the probe.
- **callback** = rclcpp subscription callbacks that ran in the controller (the `callback_start`
  tracepoint). This is the number of messages the controller acted on.

The wire rate is what crossed the network. So:

| Seen | Likely cause |
|---|---|
| callback ≈ wire | every sample on the wire reached the callback |
| callback < wire, best effort | samples arrived, but the reader's KEEP_LAST history overwrote them before the executor took them (more depth helps only with bursts), or the socket buffer dropped them |
| callback < wire, reliable | retransmissions show on the wire but deliver once; a KEEP_LAST reader still drops its oldest samples when the callback falls behind |
| wire > publish | retransmissions, or one copy per matched reader or locator |
| wire < publish | writer-side drops (send buffer, best effort; history overwritten, reliable KEEP_LAST), shared-memory delivery, or capture loss (check the capture's drop counter) |
| callback > wire | delivery that bypassed the capture (shared memory, loopback not captured) or capture loss; rclcpp does not run a callback twice for one sample |

# Flood test: wire rate vs ros2_pulse rates

One rclcpp controller subscribes to `/cmd_vel` with the probe preloaded. The generator side runs
a flood publisher (also under the probe) and a wire capture that counts what crosses the network.
The scenario is fixed: Jazzy, `rmw_fastrtps_cpp` on UDPv4, six QoS combos (reliable and best
effort, KEEP_LAST depth 1, 10 and 100), 60 s per run, 3 repeats each, 1 s windows, jsonl output.
`compare.py` then lines up the wire rate with the probe's publish and callback rates, window by
window, and totals each run.

| File | What |
|---|---|
| `controller/` | ament package `flood_compare_controller`: `flood_controller` (the subscriber) and `flood_publisher` (a reference flood for dry runs). Not built with ros2_pulse, see Build. |
| `run.sh` | starts the controller under the probe for one combo or all six, `REPEATS` times each, and writes `flood_out/<combo>_r<n>/` |
| `compare.py` | the per-window table, per-run totals and the per-combo summary (stdlib python, no ROS needed) |
| `test_compare.py` | its tests: `python3 -m pytest examples/flood_compare` |

## Build

colcon stops crawling at ros2_pulse's own `package.xml`, so the example package is only built
when you name its directory. In a Jazzy workspace with ros2_pulse cloned into `src/`:

```bash
cd ~/ros2_ws
colcon build --base-paths src/ros2_pulse src/ros2_pulse/examples/flood_compare/controller
source install/setup.bash
```

With ros2_pulse from apt (`ros-jazzy-ros2-pulse`), pass only the second path.

## Run

```bash
cd ~/ros2_ws/src/ros2_pulse/examples/flood_compare
./run.sh all              # 6 combos x 3 repeats x 60 s, about 20 min; pauses before each run
./run.sh best_effort 10   # one combo, 3 repeats
```

Repeat 1 of every combo runs before repeat 2 of any, so a slow drift over the session spreads
across combos instead of landing on one. For each run `run.sh` starts the controller like this:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
LD_PRELOAD=$(ros2 pkg prefix ros2_pulse)/lib/libros2_pulse.so \
ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=1 ROS_TOPIC_STATS_FORMAT=jsonl \
ROS_TOPIC_STATS_OUTPUT_FILE=flood_out/best_effort_d10_r1/pulse.jsonl \
  $(ros2 pkg prefix flood_compare_controller)/lib/flood_compare_controller/flood_controller \
  --ros-args -p reliability:=best_effort -p depth:=10 -p work_us:=1000
```

`run.sh` settings, all optional: `OUT` (`./flood_out`), `REPEATS` (3), `TOPIC` (`/cmd_vel`),
`WORK_US` (1000), `RMW_IMPLEMENTATION` (`rmw_fastrtps_cpp`), `TRANSPORT` (`UDPv4`), `PUB_HZ`,
`JITTER`, `NO_PROMPT`, `PULSE_SO`. The header of `run.sh` explains each.

### The controller

One node on a single-threaded executor. Parameters: `topic`, `reliability` (`reliable` |
`best_effort`), `depth` (KEEP_LAST, durability volatile) and `work_us`, a busy wait in every
callback. **`work_us` of about 1000** caps the callback near 1 kHz, so a faster flood outruns it
and the reader's history depth decides what is kept. That is the setting `run.sh` uses; with
`WORK_US=0` the callback is nearly free and the depth rarely matters. Each callback also
integrates a pose from the Twist. At exit the controller prints its callback total, which equals
the sum of the probe's callback counts over the run.

### The generator side

Same topic (`/cmd_vel`, DDS topic `rt/cmd_vel`, type `geometry_msgs::msg::dds_::Twist_`), same
reliability and depth, durability volatile, same `RMW_IMPLEMENTATION` and
`FASTDDS_BUILTIN_TRANSPORTS`. A best effort writer never matches a reliable reader, and
`compare.py` warns when the controller received nothing.

Run the publisher under the probe too, with the same settings and its own output file:

```bash
LD_PRELOAD=/path/to/libros2_pulse.so ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=1 \
ROS_TOPIC_STATS_FORMAT=jsonl ROS_TOPIC_STATS_OUTPUT_FILE=pulse_gen.jsonl  <generator>
```

The probe counts `rcl_publish`, so an rcl-based publisher shows up as well as an rclcpp one.
Publish open loop at a fixed target rate for the run's duration. A reliable writer at depth 1
can push back on a slow reader, so the publish rate is a result to record, not a setting to
assume. `flood_publisher` (for dry runs) works this way: `PUB_HZ=2000 ./run.sh all 10` starts it
under the probe for each run, writes `pulse_pub.jsonl`, and logs `sent N of M due` at exit.

### Transport

Fast DDS delivers between processes on one host over shared memory by default, and then no
packet crosses the network for a capture to see. So both sides pin `rmw_fastrtps_cpp` with
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (Fast DDS 2.13 and newer, which Jazzy has). This holds
whether the generator and the controller share a host or not.

Shared memory is an optional, separate run: `TRANSPORT=SHM OUT=./flood_out_shm ./run.sh all`,
with the generator on the same host and the same setting. A capture sees no traffic there by
design; the run shows the probe's publish and callback counts when the network is out of the
path.

## What to send back

- `flood_out/` as is: per run, `pulse.jsonl`, `meta.json`, `controller.log`. Put the
  generator's probe log for that run into the same directory (any `pulse*.jsonl` name, e.g.
  `pulse_gen.jsonl`); `compare.py` reads every one there.
- The wire capture as CSV, 1 s windows, one file for the whole session or `wire.csv` inside
  each run's directory.
- The generator's target rate, payload, how many it actually sent per run, and whether the
  capture host and the controller host share a clock (same host, or NTP and the measured offset).

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
python3 compare.py flood_out wire.csv --csv windows.csv --totals-csv totals.csv
```

Per run, a table of windows and its totals; then one line per combo over its repeats:

```
== best_effort_d1_r1  (best_effort, depth 1)
   end (UTC)   win_s     wire_hz      pub_hz       cb_hz    cb-wire   cb/wire   pub-wire  flags
08:05:01.000   1.000      1000.0           -       400.0     -600.0     40.0%          -  partial
08:05:02.000   1.000      1000.0           -       400.0     -600.0     40.0%          -
...
   4 of 6 windows compared (partial and gap windows are listed, not compared)
   totals, partial windows included: wire 6000, callbacks 2400
   callbacks/wire 40.00%
   largest callback-wire gap -600.0 Hz in the window ending 08:05:02.000

== by combo: mean per run of the totals (partial windows included); ratios are the mean of the runs' ratios [min-max]
combo             runs        sent        wire   callbacks            callbacks/wire  ...
reliable_d1          3           -        6000        5900     98.33% [95.00-100.00]  ...
best_effort_d1       1           -        6000        2400                    40.00%  ...
```

(Synthetic numbers from the test fixtures; `sent` is `-` there because no publisher log was
given.) `--summary` leaves out the per-window tables. `--max-diff-pct P` exits 1 when a compared
window's callback rate is more than P % off the wire rate (off the publish rate without a wire
CSV).

**Per window.** `pub_hz` is the sum over the probed publishing processes, `cb_hz` over the probed
subscribing ones. Flags: `partial` is the probe's first window after attach or its exit window;
`no-wire` / `pub-gap` / `cb-gap` mean that source covers less than 95 % of the window. Flagged
windows are listed but not compared, because their rates do not line up window by window. A
difference of a message or two per window is phase between the two sides; the totals settle it.

**Totals.** sent, wire and callbacks per run are each `sum(rate x window_s)` over every window of
the processes that measured them, **partial windows included**: their messages happened, they
only are not comparable window by window. The wire total is the capture's count over the run's
span (first probe window start to last window end, over all of the run's probe logs). It is
left out, with a warning, when the capture covers less than 95 % of that span. `window_s` is
written to the millisecond, so each window's count is good to about 0.05 %.

### Clocks

Each probe record's `ts_ns` is the probe's `std::chrono::system_clock` (`CLOCK_REALTIME` on
Linux) in epoch nanoseconds, written as a decimal string so it survives readers that parse
numbers as doubles. It is read when the window is flushed, right after the counts are
snapshotted and just before the record is written, so it marks the END of the window.
`window_s` is the window's measured length on `steady_clock`, not the configured period: the
first window runs from the probe's first tracepoint, the exit window is cut short. A record
therefore covers `[ts_ns - window_s, ts_ns]`, and its rates are counts divided by that measured
length.

`compare.py` puts the wire rows and other processes' windows onto the controller's windows by
time overlap, so the sides need not tick in phase, but their wall clocks must agree. On two hosts,
run NTP and report the offset; `--wire-offset S` adds S seconds to every wire timestamp.

## Reading a difference

ros2_pulse counts inside each process, in rclcpp and rcl, never on the wire:

- **publish (sent)** = `rcl_publish` calls in a probed publishing process: one per publish,
  however many readers or packets it turns into.
- **callback** = rclcpp subscription callbacks that ran in the controller (the `callback_start`
  tracepoint): the messages the controller acted on.

The wire rate is what the capture counted on the network. A difference between wire and
callbacks is a **gap**, not loss: the numbers alone do not say where the messages went, and
calling it loss needs a run that isolates the cause (for example, the same combo with
`WORK_US=0`, or the capture's own drop counter at zero). Candidate causes:

| Seen | Candidate causes |
|---|---|
| callbacks ≈ wire | every sample on the wire reached the callback |
| callbacks < wire, best effort | the reader's KEEP_LAST history overwrote samples before the executor took them (more depth helps only with bursts); the socket buffer dropped them |
| callbacks < wire, reliable | retransmissions counted on the wire but delivered once; a KEEP_LAST reader still replaces its oldest samples when the callback falls behind |
| wire > sent | retransmissions, or one copy per matched reader or locator |
| wire < sent | the writer's history overwrote samples before they went out, the send buffer was full (best effort), traffic took shared memory, or the capture missed packets (check its drop counter) |
| callbacks > wire | delivery that bypassed the capture (shared memory, loopback not captured) or packets the capture missed; rclcpp does not run a callback twice for one sample |

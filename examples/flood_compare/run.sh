#!/bin/bash
# flood_compare runner (README.md): starts flood_controller with the ros2_pulse probe preloaded,
# 1 s windows, jsonl output, on Jazzy with Fast DDS pinned to UDPv4 so the traffic is on the wire.
#
#   run.sh all [seconds]                                the six combos (reliable, best_effort x
#                                                       depth 1, 10, 100), REPEATS times each
#   run.sh <reliable|best_effort> <depth> [seconds]    one combo, REPEATS times
#                                                       (default 60 s per run)
#
# Environment (all optional):
#   OUT=./flood_out    results: $OUT/<reliability>_d<depth>_r<repeat>/{pulse.jsonl,meta.json,...}
#   REPEATS=3          runs per combo; repeat 1 of every combo runs before repeat 2 of any
#   TOPIC=/cmd_vel     topic for both nodes
#   WORK_US=1000       busy wait per callback (a ~1 kHz ceiling), so the flood outruns the
#                      callback and the depth matters; 0 for a callback that does nothing extra
#   RMW_IMPLEMENTATION=rmw_fastrtps_cpp   pinned; the generator side must use the same
#   TRANSPORT=UDPv4    FASTDDS_BUILTIN_TRANSPORTS for the nodes started here. UDPv4 keeps
#                      same-host traffic off shared memory so a capture sees it. TRANSPORT=SHM
#                      (with OUT=./flood_out_shm) is the optional shared-memory run, same host
#                      only, invisible to a network capture by design.
#   PUB_HZ=            when set, also start the reference flood_publisher (open loop, this target
#                      rate, for the run's length), probe preloaded (pulse_pub.jsonl). Leave unset
#                      when the flood comes from the generator side.
#   JITTER=0           1 sets ROS_TOPIC_STATS_JITTER=1 (adds recv_max_dt_ms and friends)
#   NO_PROMPT=0        1 skips the "press Enter" pause before each run when PUB_HZ is unset
#   PULSE_SO=          path to libros2_pulse.so (default: from `ros2 pkg prefix ros2_pulse`)
#
# Needs a sourced workspace with ros2_pulse and flood_compare_controller (README.md, Build).
set -u

usage() { sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
case "${1:-}" in all|reliable|best_effort) ;; *) usage ;; esac

OUT="${OUT:-./flood_out}"
REPEATS="${REPEATS:-3}"
TOPIC="${TOPIC:-/cmd_vel}"
WORK_US="${WORK_US:-1000}"
TRANSPORT="${TRANSPORT:-UDPv4}"
PUB_HZ="${PUB_HZ:-}"
JITTER="${JITTER:-0}"
NO_PROMPT="${NO_PROMPT:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

case "$REPEATS" in ''|*[!0-9]*|0) echo "run.sh: REPEATS must be a positive integer" >&2; exit 2;; esac
if [ "${ROS_DISTRO:-}" != jazzy ]; then
  echo "run.sh: the scenario is defined on Jazzy (ROS_DISTRO=${ROS_DISTRO:-unset});" \
       "FASTDDS_BUILTIN_TRANSPORTS needs Fast DDS 2.13 or newer" >&2
fi

prefix_of() { ros2 pkg prefix "$1" 2>/dev/null || { echo "run.sh: package $1 not found, source the workspace first" >&2; exit 2; }; }
PULSE_SO="${PULSE_SO:-$(prefix_of ros2_pulse)/lib/libros2_pulse.so}"
CTL_PREFIX="$(prefix_of flood_compare_controller)/lib/flood_compare_controller"
[ -f "$PULSE_SO" ] || { echo "run.sh: probe not found at $PULSE_SO" >&2; exit 2; }
[ -x "$CTL_PREFIX/flood_controller" ] || { echo "run.sh: flood_controller not built (README.md, Build)" >&2; exit 2; }

# The probe settings this test is defined by: 1 s windows, jsonl, one file per process, plus the
# pinned transport. exec: run in the background, this function is a subshell, and $! must be the
# node itself or the signals in stop_pid would stop the subshell and leave the node running.
probed() { # <output file> <command...>
  local out="$1"; shift
  exec env LD_PRELOAD="$PULSE_SO" \
      ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=1 \
      ROS_TOPIC_STATS_FORMAT=jsonl \
      ROS_TOPIC_STATS_OUTPUT_FILE="$out" \
      ROS_TOPIC_STATS_JITTER="$JITTER" \
      FASTDDS_BUILTIN_TRANSPORTS="$TRANSPORT" \
      "$@"
}

CTL_PID=""
PUB_PID=""
# SIGINT is rclcpp's clean shutdown (the probe then writes its exit window); SIGTERM after 5 s in
# case the node did not stop.
stop_pid() {
  local pid="$1" _
  [ -n "$pid" ] || return 0
  kill -INT "$pid" 2>/dev/null || return 0
  for _ in $(seq 50); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null
  wait "$pid" 2>/dev/null
  return 0
}
stop_all() {
  stop_pid "$PUB_PID"   # the flood stops first, so the controller's last windows drain it
  stop_pid "$CTL_PID"
  PUB_PID=""; CTL_PID=""
}
trap 'stop_all; exit 130' INT TERM

run_one() { # <reliability> <depth> <seconds> <repeat>
  local rel="$1" depth="$2" secs="$3" rep="$4"
  case "$rel" in reliable|best_effort) ;; *) echo "run.sh: reliability must be reliable or best_effort" >&2; exit 2;; esac
  case "$depth" in ''|*[!0-9]*|0) echo "run.sh: depth must be a positive integer" >&2; exit 2;; esac
  local combo="${rel}_d${depth}"
  local label="${combo}_r${rep}"
  local dir="$OUT/$label"
  local qos=(--ros-args -p "topic:=$TOPIC" -p "reliability:=$rel" -p "depth:=$depth")

  if [ -z "$PUB_HZ" ] && [ "$NO_PROMPT" != 1 ]; then
    read -r -p "next: $label ($rel, depth $depth). Set the generator to this QoS, then press Enter " _
  fi
  rm -rf "$dir" && mkdir -p "$dir" || exit 2
  echo "== $label: $TOPIC, $rel, KEEP_LAST $depth, ${secs} s, $RMW_IMPLEMENTATION/$TRANSPORT -> $dir"
  local start
  start="$(date +%s.%N)"
  probed "$dir/pulse.jsonl" "$CTL_PREFIX/flood_controller" "${qos[@]}" -p "work_us:=$WORK_US" \
    > "$dir/controller.log" 2>&1 &
  CTL_PID=$!
  if [ -n "$PUB_HZ" ]; then
    sleep 1   # let discovery match the endpoints before the flood starts
    probed "$dir/pulse_pub.jsonl" "$CTL_PREFIX/flood_publisher" "${qos[@]}" \
      -p "rate_hz:=$PUB_HZ" -p "duration_s:=$secs" > "$dir/publisher.log" 2>&1 &
    PUB_PID=$!
    sleep "$secs"
    # it stops itself after duration_s; give it 10 s past that before stopping it
    for _ in $(seq 100); do
      kill -0 "$PUB_PID" 2>/dev/null || break
      sleep 0.1
    done
    stop_pid "$PUB_PID"
    PUB_PID=""
    sleep 1   # let the controller take what is still queued
  else
    echo "   start the flood now: $TOPIC geometry_msgs/msg/Twist, $rel, depth $depth," \
         "$RMW_IMPLEMENTATION, FASTDDS_BUILTIN_TRANSPORTS=$TRANSPORT"
    sleep "$secs"
  fi
  stop_all
  local end
  end="$(date +%s.%N)"
  printf '{"label":"%s","combo":"%s","repeat":%s,"topic":"%s","reliability":"%s","depth":%s,"seconds":%s,"work_us":%s,"pub_hz":"%s","rmw":"%s","transport":"%s","ros_distro":"%s","host":"%s","start_unix":%s,"end_unix":%s}\n' \
    "$label" "$combo" "$rep" "$TOPIC" "$rel" "$depth" "$secs" "$WORK_US" "$PUB_HZ" \
    "$RMW_IMPLEMENTATION" "$TRANSPORT" "${ROS_DISTRO:-}" "$(uname -n)" "$start" "$end" \
    > "$dir/meta.json"
  local windows
  windows="$(grep -c '^{' "$dir/pulse.jsonl" 2>/dev/null)"
  echo "   ${windows:-0} windows written;" \
       "$(grep -h 'callbacks' "$dir/controller.log" || echo 'controller printed no total, see controller.log')"
  [ -n "$PUB_HZ" ] && echo "   $(grep -h 'sent' "$dir/publisher.log" || echo 'publisher printed no total, see publisher.log')"
  return 0
}

if [ "$1" = all ]; then
  secs="${2:-60}"
  for rep in $(seq "$REPEATS"); do
    for rel in reliable best_effort; do
      for depth in 1 10 100; do
        run_one "$rel" "$depth" "$secs" "$rep"
      done
    done
  done
else
  [ $# -ge 2 ] || usage
  for rep in $(seq "$REPEATS"); do
    run_one "$1" "$2" "${3:-60}" "$rep"
  done
fi

echo "done. Send back: $OUT/ (tar czf flood_out.tgz -C \"$(dirname "$OUT")\" \"$(basename "$OUT")\") plus the wire CSV."

#!/bin/bash
# flood_compare runner (README.md): starts flood_controller with the ros2_pulse probe preloaded,
# 1 s windows, jsonl output, for one QoS combo or all six.
#
#   run.sh <reliable|best_effort> <depth> [seconds]    one combo (default 30 s)
#   run.sh all [seconds]                                the six combos, reliable then best_effort,
#                                                       depth 1, 10, 100
#
# Environment (all optional):
#   OUT=./flood_out    where results go: $OUT/<reliability>_d<depth>/{pulse.jsonl,meta.json,...}
#   TOPIC=/cmd_vel     topic for both nodes
#   WORK_US=0          busy wait per callback in the controller, to make it the bottleneck
#   PUB_HZ=            when set, also start the reference flood_publisher at this rate, probe
#                      preloaded too (pulse_pub.jsonl). Leave unset when the flood comes from
#                      the wire-capture side's own generator.
#   JITTER=0           1 sets ROS_TOPIC_STATS_JITTER=1 (adds recv_max_dt_ms and friends)
#   NO_PROMPT=0        1 skips the "press Enter" pause between combos in `all` mode
#   PULSE_SO=          path to libros2_pulse.so (default: from `ros2 pkg prefix ros2_pulse`)
#
# Needs a sourced workspace with ros2_pulse and flood_compare_controller (README.md, Build).
set -u

usage() { sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
case "${1:-}" in all|reliable|best_effort) ;; *) usage ;; esac

OUT="${OUT:-./flood_out}"
TOPIC="${TOPIC:-/cmd_vel}"
WORK_US="${WORK_US:-0}"
PUB_HZ="${PUB_HZ:-}"
JITTER="${JITTER:-0}"
NO_PROMPT="${NO_PROMPT:-0}"

prefix_of() { ros2 pkg prefix "$1" 2>/dev/null || { echo "run.sh: package $1 not found, source the workspace first" >&2; exit 2; }; }
PULSE_SO="${PULSE_SO:-$(prefix_of ros2_pulse)/lib/libros2_pulse.so}"
CTL_PREFIX="$(prefix_of flood_compare_controller)/lib/flood_compare_controller"
[ -f "$PULSE_SO" ] || { echo "run.sh: probe not found at $PULSE_SO" >&2; exit 2; }
[ -x "$CTL_PREFIX/flood_controller" ] || { echo "run.sh: flood_controller not built (README.md, Build)" >&2; exit 2; }

# The probe settings this test is defined by: 1 s windows, jsonl, one file per process.
probed() { # <output file> <command...>
  local out="$1"; shift
  env LD_PRELOAD="$PULSE_SO" \
      ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=1 \
      ROS_TOPIC_STATS_FORMAT=jsonl \
      ROS_TOPIC_STATS_OUTPUT_FILE="$out" \
      ROS_TOPIC_STATS_JITTER="$JITTER" \
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

run_combo() { # <reliability> <depth> <seconds>
  local rel="$1" depth="$2" secs="$3"
  case "$rel" in reliable|best_effort) ;; *) echo "run.sh: reliability must be reliable or best_effort" >&2; exit 2;; esac
  case "$depth" in ''|*[!0-9]*|0) echo "run.sh: depth must be a positive integer" >&2; exit 2;; esac
  local label="${rel}_d${depth}"
  local dir="$OUT/$label"
  rm -rf "$dir" && mkdir -p "$dir" || exit 2
  local qos=(--ros-args -p "topic:=$TOPIC" -p "reliability:=$rel" -p "depth:=$depth")

  echo "== $label: $TOPIC, $rel, KEEP_LAST $depth, ${secs} s -> $dir"
  local start
  start="$(date +%s.%N)"
  probed "$dir/pulse.jsonl" "$CTL_PREFIX/flood_controller" "${qos[@]}" -p "work_us:=$WORK_US" \
    > "$dir/controller.log" 2>&1 &
  CTL_PID=$!
  if [ -n "$PUB_HZ" ]; then
    sleep 1   # let discovery match the endpoints before the flood starts
    probed "$dir/pulse_pub.jsonl" "$CTL_PREFIX/flood_publisher" "${qos[@]}" -p "rate_hz:=$PUB_HZ" \
      > "$dir/publisher.log" 2>&1 &
    PUB_PID=$!
  else
    echo "   start the flood now: $TOPIC geometry_msgs/msg/Twist, reliability=$rel, depth=$depth"
  fi
  sleep "$secs"
  stop_all
  local end
  end="$(date +%s.%N)"
  printf '{"label":"%s","topic":"%s","reliability":"%s","depth":%s,"seconds":%s,"work_us":%s,"pub_hz":"%s","rmw":"%s","ros_distro":"%s","host":"%s","start_unix":%s,"end_unix":%s}\n' \
    "$label" "$TOPIC" "$rel" "$depth" "$secs" "$WORK_US" "$PUB_HZ" "${RMW_IMPLEMENTATION:-default}" \
    "${ROS_DISTRO:-}" "$(uname -n)" "$start" "$end" > "$dir/meta.json"
  local windows
  windows="$(grep -c '^{' "$dir/pulse.jsonl" 2>/dev/null)"
  echo "   ${windows:-0} windows written;" \
       "$(grep -h 'callbacks' "$dir/controller.log" || echo 'controller printed no total, see controller.log')"
}

case "${1:-}" in
  all)
    secs="${2:-30}"
    for rel in reliable best_effort; do
      for depth in 1 10 100; do
        if [ -z "$PUB_HZ" ] && [ "$NO_PROMPT" != 1 ]; then
          read -r -p "next: ${rel}, depth ${depth}. Set the generator to this QoS, then press Enter " _
        fi
        run_combo "$rel" "$depth" "$secs"
      done
    done
    ;;
  reliable|best_effort)
    [ $# -ge 2 ] || usage
    run_combo "$1" "$2" "${3:-30}"
    ;;
  *) usage ;;
esac

echo "done. Send back: $OUT/ (tar czf flood_out.tgz -C \"$(dirname "$OUT")\" \"$(basename "$OUT")\") plus the wire CSV."

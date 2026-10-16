#!/usr/bin/env bash
# Runs Nav2's loopback sim bringup under the probe, drives a few goals, then shuts down.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
COMPOSED=${1:-True}
OUT=$PWD/out; mkdir -p "$OUT"
LAUNCH=$(ls "$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch" | grep -i loopback | head -1)
echo "launch file: $LAUNCH  use_composition:=$COMPOSED"
export TMPDIR=$OUT ROS_TOPIC_STATS_FORMAT=jsonl ROS_TOPIC_STATS_JITTER=1 \
       ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=2 ROS_TOPIC_STATS_QUIET=1
PROBE=$(ros2 pkg prefix ros2_pulse)/lib/libros2_pulse.so
LD_PRELOAD=$PROBE setsid ros2 launch nav2_bringup "$LAUNCH" use_rviz:=False \
  use_composition:="$COMPOSED" > "$OUT/launch.log" 2>&1 &
PGID=$!
python3 ci-runs/nav2/goals.py 2>&1 | tee "$OUT/goals.log" || true
kill -INT -- -$PGID || true
for _ in $(seq 30); do kill -0 $PGID 2>/dev/null || break; sleep 1; done
kill -KILL -- -$PGID 2>/dev/null || true
ls -la "$OUT"

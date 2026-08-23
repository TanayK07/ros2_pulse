#!/bin/bash
# ros2_pulse, on-Orin field test.
#
# Captures, on the real robot: (1) that the probe loads and counts real topics incl. intra-process,
# (2) CPU overhead (probe ON vs OFF) for the heaviest nodes, (3) network cost (should be ZERO, the
# probe publishes nothing), (4) behavior with iceoryx SHM enabled vs disabled.
#
# Run INSIDE the ROS container on the Orin, after sourcing the workspace. Non-destructive: it only
# reads /proc, ss, and the probe's output file. Does NOT modify the running stack.
#
# Usage:
#   ./run_orin_probe_test.sh <duration_s> [node_name_substr]
# Example:
#   ./run_orin_probe_test.sh 60 perception
set -u
DUR="${1:-60}"
NODE_MATCH="${2:-}"
OUT=/tmp/orin_probe_report
mkdir -p "$OUT"
# The probe's output file: honor the env var the stack was launched with, else the legacy
# deploy path, else the v0.3.0 default location.
STATS="${ROS_TOPIC_STATS_OUTPUT_FILE:-/root/ssd2tb/logs/topic_freq.log}"
[ -f "$STATS" ] || STATS=$(ls -t "${TMPDIR:-/tmp}"/topic_freq.*.log 2>/dev/null | head -1)
STATS="${STATS:-/tmp/topic_freq.log}"

echo "=== 0. environment ==="
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
echo "LD_PRELOAD=${LD_PRELOAD:-unset}"
grep -q "<SharedMemory>" "${CYCLONEDDS_URI:-/nonexistent}" 2>/dev/null && \
  echo "SHM: config present ($(grep -o '<Enable>[a-z]*</Enable>' ${CYCLONEDDS_URI} 2>/dev/null | head -1))" || \
  echo "SHM: no CYCLONEDDS_URI SharedMemory block"

echo
echo "=== 1. instrumentation present on this image? (must be non-zero) ==="
TT=$(find /opt/ros -name 'libtracetools.so*' 2>/dev/null | head -1)
echo "libtracetools: $TT"
echo "ros_trace_* symbols: $(nm -D "$TT" 2>/dev/null | grep -c ros_trace)"
PROBE=$(find / -name 'libros2_pulse.so' 2>/dev/null | head -1)
echo "probe .so: ${PROBE:-NOT FOUND, build graph-monitor first}"

echo
echo "=== 2. sample CPU of matching nodes (probe is ${LD_PRELOAD:+ON}${LD_PRELOAD:-OFF}) ==="
# Sum utime+stime deltas over the window for processes whose cmdline matches NODE_MATCH (or all ros).
sample_cpu () {
  local tag="$1"
  ps -eo pid,comm,args | grep -E "ros|component_container|${NODE_MATCH}" | grep -v grep \
    | awk '{print $1}' | while read -r pid; do
      [ -r "/proc/$pid/stat" ] || continue
      # pid, utime+stime jiffies, comm, comm lets an ON sample be matched to an OFF sample
      # across a stack restart, where every pid changes (Orin run 2026-08-23).
      comm=$(tr -d '\n' < "/proc/$pid/comm" 2>/dev/null | tr ' ' '_')
      awk -v pid="$pid" -v comm="${comm:-?}" '{print pid, $14+$15, comm}' "/proc/$pid/stat" 2>/dev/null
    done > "$OUT/cpu_$tag.txt"
}
sample_cpu start
SOCK0=$(ss -x -H 2>/dev/null | wc -l)
UDP0=$(ss -u -H 2>/dev/null | wc -l)
echo "waiting ${DUR}s while the live stack runs..."
sleep "$DUR"
sample_cpu end
SOCK1=$(ss -x -H 2>/dev/null | wc -l)
UDP1=$(ss -u -H 2>/dev/null | wc -l)

CLK=$(getconf CLK_TCK)
echo "--- top CPU consumers over ${DUR}s (jiffies -> seconds, CLK_TCK=$CLK) ---"
join -j1 <(sort "$OUT/cpu_start.txt") <(sort "$OUT/cpu_end.txt") 2>/dev/null \
  | awk -v clk="$CLK" '{d=($4-$2)/clk; if(d>0) printf "pid=%s comm=%s cpu_s=%.2f (%.1f%% of 1 core)\n", $1, $3, d, 100*d/'"$DUR"'}' \
  | sort -t= -k3 -nr | head -15

echo
echo "=== 3. network cost of the probe (expect ~0 delta; probe publishes nothing) ==="
echo "unix sockets: $SOCK0 -> $SOCK1   udp sockets: $UDP0 -> $UDP1"
echo "(the probe opens no DDS endpoints and no sockets; any delta is the live stack, not the probe)"

echo
echo "=== 4. captured topic data (last window of $STATS) ==="
if [ -f "$STATS" ]; then
  awk '/^# ts_ns/{buf=""} {buf=buf $0 ORS} END{printf "%s", buf}' "$STATS" | tail -60
  echo "--- counts ---"
  echo "topics with inter-process traffic: $(grep -c 'inter=[1-9]' "$STATS")"
  echo "topics with intra-process traffic: $(grep -c 'intra=[1-9]' "$STATS")   <-- key: nonzero proves intra visibility"
  echo "output file size: $(ls -la "$STATS" | awk '{print $5}') bytes"
else
  echo "NO OUTPUT FILE at $STATS, is LD_PRELOAD set and the .so on LD_LIBRARY_PATH?"
fi

echo
echo "=== report dir: $OUT ==="
echo "To compare probe ON vs OFF: run once WITHOUT LD_PRELOAD (unset it, restart stack), once WITH."
echo "To test SHM on/off: point CYCLONEDDS_URI at cyclonedds_default_shm.xml vs _no_shm.xml, restart, rerun."

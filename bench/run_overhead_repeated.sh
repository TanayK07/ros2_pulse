#!/bin/bash
# Rigorous end-to-end overhead: interleave baseline vs ours over N trials to cancel CPU drift and
# beat the ~±10% single-sample variance. Reports every sample + means. Also runs the isolated
# hot-path microbench (the clean per-operation signal).
set +e
cd /work
source /opt/ros/humble/setup.bash

g++ -O2 -std=c++17 -fPIC -shared -I/pkg/include \
  /pkg/src/probe/interposers.cpp /pkg/src/core/topic_registry.cpp /pkg/src/core/timer.cpp \
  -ldl -pthread -o /work/libprod.so 2>/dev/null || { echo so_fail; exit 1; }
rm -rf /work/bbuild
cmake -S /pkg/bench -B /work/bbuild -DCMAKE_PREFIX_PATH=/opt/ros/humble -DCMAKE_BUILD_TYPE=Release >/dev/null 2>&1
cmake --build /work/bbuild -j4 >/dev/null 2>&1
NODE=/work/bbuild/stress_nodes
NL=30; NH=8; NI=15; DUR=8; N=6

run_workload () {  # $1=env  -> echoes summed workload cpu_s
  env $1 $NODE subfarm   $DUR $NL $NH 2>/work/s.res &
  local a=$!; env $1 $NODE intrafarm $DUR $NI 2>/work/i.res & local b=$!
  sleep 1; env $1 $NODE pubfarm $DUR $NL $NH 2>/work/p.res & local c=$!
  wait $a $b $c 2>/dev/null
  awk '/RESULT/{split($3,x,"=");s+=x[2]} END{printf "%.3f", s}' /work/p.res /work/s.res /work/i.res
}

echo "=== interleaved trials (N=$N), workload CPU seconds ==="
printf "%-6s %-12s %-12s\n" "trial" "baseline" "ours"
BSUM=0; OSUM=0
for i in $(seq 1 $N); do
  b=$(run_workload "")
  o=$(run_workload "LD_PRELOAD=/work/libprod.so ROS_TOPIC_STATS_OUTPUT_FILE=/work/o.log ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=2.0")
  printf "%-6s %-12s %-12s\n" "$i" "$b" "$o"
  BSUM=$(awk "BEGIN{print $BSUM+$b}"); OSUM=$(awk "BEGIN{print $OSUM+$o}")
done
awk -v b="$BSUM" -v o="$OSUM" -v n="$N" 'BEGIN{
  bm=b/n; om=o/n; printf "\nmean baseline=%.3fs  mean ours=%.3fs  delta=%+.1f%%\n", bm, om, 100*(om-bm)/bm }'

echo
echo "=== isolated hot-path microbench (clean per-operation signal) ==="
# Committed under bench/ and referenced from the mounted package path (same convention as
# run_bakeoff.sh's /pkg/bench/...), so the headline microbench number is reproducible on a fresh
# checkout, not dependent on a scratch file mounted at /work.
if [ -f /pkg/bench/hotpath_bench.cpp ]; then
  g++ -O2 -std=c++17 -pthread /pkg/bench/hotpath_bench.cpp -o /work/hb && /work/hb 8
else
  echo "ERROR: /pkg/bench/hotpath_bench.cpp not found — mount the package at /pkg (-v <pkg>:/pkg)"
fi
echo "=== DONE ==="

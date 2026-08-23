#!/bin/bash
# Rigorous end-to-end overhead: PAIRED trials (baseline vs ours back-to-back) with the arm order
# ALTERNATED each trial, so slow machine drift and any warm-up/order effect cancel in the
# per-trial difference. Reports every sample plus mean/SD/SEM of the paired diffs, a delta is
# only believable if it clears ~2x the SEM (the KNOWN_ISSUES #15 hunt showed single runs swing
# +/-4% and a fixed arm order can masquerade as overhead). Also runs the isolated hot-path
# microbench (the clean per-operation signal).
set +e
cd /work
ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

g++ -O2 -std=c++17 -fPIC -shared -I/pkg/include \
  /pkg/src/probe/interposers.cpp /pkg/src/core/*.cpp \
  -ldl -pthread -o /work/libprod.so 2>/dev/null || { echo so_fail; exit 1; }
rm -rf /work/bbuild
cmake -S /pkg/bench -B /work/bbuild -DCMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} -DCMAKE_BUILD_TYPE=Release >/dev/null 2>&1
cmake --build /work/bbuild -j4 >/dev/null 2>&1
NODE=/work/bbuild/stress_nodes
NL=30; NH=8; NI=15; DUR=10; N=10

run_workload () {  # $1=env  -> echoes summed workload cpu_s
  env $1 $NODE subfarm   $DUR $NL $NH 2>/work/s.res &
  local a=$!; env $1 $NODE intrafarm $DUR $NI 2>/work/i.res & local b=$!
  sleep 1; env $1 $NODE pubfarm $DUR $NL $NH 2>/work/p.res & local c=$!
  wait $a $b $c 2>/dev/null
  awk '/RESULT/{split($3,x,"=");s+=x[2]} END{printf "%.3f", s}' /work/p.res /work/s.res /work/i.res
}

ENVP="LD_PRELOAD=/work/libprod.so ROS_TOPIC_STATS_OUTPUT_FILE=/work/o.log ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=2.0"
echo "=== paired trials (N=$N, DUR=${DUR}s, order alternated), workload CPU seconds ==="
printf "%-6s %-8s %-12s %-12s %-10s\n" "trial" "order" "baseline" "ours" "diff"
BSUM=0; OSUM=0; DIFFS=""
for i in $(seq 1 $N); do
  if [ $((i % 2)) -eq 1 ]; then
    b=$(run_workload ""); o=$(run_workload "$ENVP"); ord="b,o"
  else
    o=$(run_workload "$ENVP"); b=$(run_workload ""); ord="o,b"
  fi
  d=$(awk "BEGIN{printf \"%.3f\", $o-$b}")
  printf "%-6s %-8s %-12s %-12s %-10s\n" "$i" "$ord" "$b" "$o" "$d"
  BSUM=$(awk "BEGIN{print $BSUM+$b}"); OSUM=$(awk "BEGIN{print $OSUM+$o}")
  DIFFS="$DIFFS $d"
done
awk -v b="$BSUM" -v o="$OSUM" -v n="$N" 'BEGIN{
  bm=b/n; om=o/n; printf "\nmean baseline=%.3fs  mean ours=%.3fs  delta=%+.1f%%\n", bm, om, 100*(om-bm)/bm }'
echo "$DIFFS" | awk -v bsum="$BSUM" '{
  n=NF; s=0; for(i=1;i<=n;i++) s+=$i; m=s/n;
  v=0; for(i=1;i<=n;i++) v+=($i-m)^2; sd=sqrt(v/(n-1)); sem=sd/sqrt(n);
  printf "paired diff mean=%+.3fs sd=%.3fs sem=%.3fs (%+.1f%% +/- %.1f%% sem of baseline)\n",
         m, sd, sem, 100*m/(bsum/n), 100*sem/(bsum/n) }'

echo
echo "=== isolated hot-path microbench (clean per-operation signal) ==="
# Committed under bench/ and referenced from the mounted package path (same convention as
# run_bakeoff.sh's /pkg/bench/...), so the headline microbench number is reproducible on a fresh
# checkout, not dependent on a scratch file mounted at /work.
if [ -f /pkg/bench/hotpath_bench.cpp ]; then
  g++ -O2 -std=c++17 -pthread /pkg/bench/hotpath_bench.cpp -o /work/hb && /work/hb 8
else
  echo "ERROR: /pkg/bench/hotpath_bench.cpp not found, mount the package at /pkg (-v <pkg>:/pkg)"
fi
echo "=== DONE ==="

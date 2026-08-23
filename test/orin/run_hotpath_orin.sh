#!/bin/bash
# ros2_pulse, Orin hot-path microbench: the +24 ns question, answered on Tegra silicon.
#
# Every published perf number is x86-64. Two of them ride on this box's clock:
#   1. ROS_TOPIC_STATS_JITTER=1 costs +24 ns/msg (bench/hotpath_bench.cpp, fixed leg).
#   2. ~96% of that is ONE CLOCK_MONOTONIC read through the vDSO. Some Tegra kernels route
#      that read through a syscall instead, the "raw steady_clock (1 thread)" leg answers
#      this directly: ~20-40 ns means the vDSO path works and the x86 story stands;
#      >=150 ns means syscall fallback, and the README's R5 cost table needs an
#      Orin-specific row before the repo goes public.
#
# Non-destructive: compiles one binary to /tmp, writes only under test/orin/out/hotpath/.
# Run inside the ROS container (any shell with g++ >= 9 works) from anywhere:
#   test/orin/run_hotpath_orin.sh [trials] [threads]
# Defaults: 10 trials, 8 threads. Use the PHYSICAL core count (nproc on Orin counts all).
# Before running, pin the power state on the HOST or the numbers are noise:
#   sudo nvpmodel -m 0 && sudo jetson_clocks
set -u
TRIALS="${1:-10}"
TH="${2:-8}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$ROOT/test/orin/out/hotpath"
mkdir -p "$OUT"

echo "=== platform ==="
{ uname -a
  echo "clocksource: $(cat /sys/devices/system/clocksource/clocksource0/current_clocksource 2>/dev/null || echo unreadable)"
  grep -m1 -E "model name|CPU part" /proc/cpuinfo
  echo "cores: $(nproc)"
  echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unreadable)"
  # Tegra power state lives on the HOST; record it if visible, demand it if not.
  command -v nvpmodel >/dev/null 2>&1 && nvpmodel -q 2>/dev/null | head -4 \
    || echo "nvpmodel: not visible here, run 'sudo nvpmodel -q' on the host and paste into platform.txt"
} | tee "$OUT/platform.txt"

echo
echo "=== build ==="
g++ -O2 -std=c++17 -pthread "$ROOT/bench/hotpath_bench.cpp" -o /tmp/hotpath_bench || exit 1
echo "ok"

echo
echo "=== $TRIALS trials x $TH threads (each ~30 s on Orin) ==="
echo "trial,r5_fixed_cpu_ns,r5_alt4_cpu_ns,raw_clock_1t_ns,clock_pct_of_delta" > "$OUT/summary.csv"
for i in $(seq 1 "$TRIALS"); do
  /tmp/hotpath_bench "$TH" > "$OUT/trial_$i.txt" 2>&1
  fixed=$(awk '/^  fixed :/{print $3}' "$OUT/trial_$i.txt")
  alt=$(awk '/^  alt-4 :/{print $3}' "$OUT/trial_$i.txt")
  clk=$(awk '/^raw steady_clock \(1 thread\)/{print $5}' "$OUT/trial_$i.txt")
  pct=$(awk -F'-> ' '/clock read alone/{print $2+0}' "$OUT/trial_$i.txt")
  echo "$i,$fixed,$alt,$clk,$pct" | tee -a "$OUT/summary.csv"
done

echo
echo "=== mean ± SEM over $TRIALS trials ==="
awk -F, 'NR>1{n++; for(i=2;i<=4;i++){s[i]+=$i; q[i]+=$i*$i}}
  END{ if(n<2){print "need >=2 trials"; exit 1}
       split("R5_fixed_CPU_ns/msg R5_alt4_CPU_ns/msg raw_clock_1thread_ns",h," ");
       for(i=2;i<=4;i++){m=s[i]/n; v=(q[i]/n-m*m)/(n-1); if(v<0)v=0;
         printf "%-22s %8.2f ± %.2f\n", h[i-1], m, sqrt(v)} }' \
  "$OUT/summary.csv" | tee "$OUT/summary.txt"

clk_mean=$(awk -F, 'NR>1{s+=$4;n++} END{printf "%.1f", s/n}' "$OUT/summary.csv")
echo
if awk "BEGIN{exit !($clk_mean < 60)}"; then
  echo "VERDICT: clock read ${clk_mean} ns -> vDSO path works on this kernel." | tee -a "$OUT/summary.txt"
  echo "         The published x86-64 R5 cost story holds; publish the Orin row as measured." | tee -a "$OUT/summary.txt"
elif awk "BEGIN{exit !($clk_mean >= 150)}"; then
  echo "VERDICT: clock read ${clk_mean} ns -> SYSCALL FALLBACK on this kernel." | tee -a "$OUT/summary.txt"
  echo "         The +24 ns claim does NOT transfer. Use the measured R5_fixed number above as" | tee -a "$OUT/summary.txt"
  echo "         the Orin-specific cost in README + docs site, and say why (vDSO fallback)." | tee -a "$OUT/summary.txt"
else
  echo "VERDICT: clock read ${clk_mean} ns -> between vDSO (~25) and syscall (~200); unusual." | tee -a "$OUT/summary.txt"
  echo "         Check governor/nvpmodel pinning and rerun before drawing conclusions." | tee -a "$OUT/summary.txt"
fi
echo
echo "Artifacts in $OUT, commit the whole directory (platform.txt, summary.*, trial_*.txt)."

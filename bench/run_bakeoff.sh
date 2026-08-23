#!/bin/bash
# ros2_pulse bake-off: measure OUR probe vs eBPF-uprobe vs LTTng/ros2_tracing on the SAME
# workload, plus a no-monitor baseline. Fair comparison: identical stress graph each leg; we record
# (a) added CPU to the monitored workload, (b) the monitor's own process CPU, (c) disk, (d) network.
#
# Must run in a --privileged container (eBPF needs debugfs + CAP_SYS_ADMIN). apt is used to install
# bpftrace + lttng tooling.  Mount: -v <pkg>:/pkg  (the ros2_pulse package for the probe .so).
set +e
cd /work
source /opt/ros/humble/setup.bash

echo "==================== SETUP ===================="
mount -t debugfs none /sys/kernel/debug 2>/dev/null
apt-get update -qq >/dev/null 2>&1
echo "installing bpftrace + lttng + ros2trace (may take a few min)..."
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
  bpftrace lttng-tools liblttng-ust-dev ros-humble-ros2trace ros-humble-tracetools-launch \
  babeltrace2 >/work/apt.log 2>&1
echo "bpftrace: $(command -v bpftrace || echo MISSING)   lttng: $(command -v lttng || echo MISSING)"

TT=$(find /opt/ros -name 'libtracetools.so*' | head -1)
echo "libtracetools: $TT"

echo "=== build probe .so + stress_nodes ==="
g++ -O2 -std=c++17 -fPIC -shared -I/pkg/include \
  /pkg/src/probe/interposers.cpp /pkg/src/core/topic_registry.cpp /pkg/src/core/timer.cpp \
  -ldl -pthread -o /work/libprod.so 2>/dev/null || { echo "so build fail"; exit 1; }
rm -rf /work/bbuild
cmake -S /pkg/bench -B /work/bbuild -DCMAKE_PREFIX_PATH=/opt/ros/humble -DCMAKE_BUILD_TYPE=Release >/work/cmake.log 2>&1
cmake --build /work/bbuild -j4 >>/work/cmake.log 2>&1
NODE=/work/bbuild/stress_nodes
[ -x "$NODE" ] || { echo "node build fail"; tail -20 /work/cmake.log; exit 1; }

NL=30; NH=8; NI=15; DUR=10
echo "workload: $NL light@100Hz + $NH heavy(~100KB)@50Hz inter + $NI intra@100Hz, ${DUR}s/leg"

# Runs the 3-farm workload; env passed as $1; echoes summed workload CPU seconds.
run_workload () {
  local envp="$1"
  env $envp $NODE subfarm   $DUR $NL $NH 2>/work/s.res &
  local a=$!
  env $envp $NODE intrafarm $DUR $NI     2>/work/i.res &
  local b=$!
  sleep 1
  env $envp $NODE pubfarm   $DUR $NL $NH 2>/work/p.res &
  local c=$!
  wait $a $b $c 2>/dev/null
  awk '/RESULT/{split($3,x,"=");s+=x[2]} END{printf "%.3f", s}' /work/p.res /work/s.res /work/i.res
}

# CPU seconds of a pid over its lifetime (utime+stime jiffies / CLK_TCK)
proc_cpu () { awk -v c="$(getconf CLK_TCK)" '{print ($14+$15)/c}' "/proc/$1/stat" 2>/dev/null || echo 0; }

echo; echo "==================== LEG A: BASELINE (no monitor) ===================="
CPU_BASE=$(run_workload "")
echo "workload CPU: ${CPU_BASE}s"

echo; echo "==================== LEG B: OURS (LD_PRELOAD tracetools) ===================="
rm -f /work/ours.log
CPU_OURS=$(run_workload "LD_PRELOAD=/work/libprod.so ROS_TOPIC_STATS_OUTPUT_FILE=/work/ours.log ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=2.0")
OURS_DISK=$(stat -c%s /work/ours.log 2>/dev/null || echo 0)
echo "workload CPU: ${CPU_OURS}s   monitor proc: in-process (0 extra)   disk: ${OURS_DISK}B   net: 0"

echo; echo "==================== LEG C: eBPF uprobe (bpftrace) ===================="
if command -v bpftrace >/dev/null; then
  bpftrace /pkg/bench/bpftrace_probe.bt "$TT" >/work/bpf.out 2>/work/bpf.err &
  BPF_PID=$!
  # Gate on ACTUAL uprobe attachment, not mere liveness: bpftrace stays alive while still
  # compiling / partially attached, and a fixed sleep races attach on slow hosts (Orin). Wait for
  # its "Attaching N probes..." line, then confirm the uprobes are live in the kernel.
  for _ in $(seq 1 20); do grep -q "Attaching .* probe" /work/bpf.err && break; sleep 0.5; done
  UPROBES=$(wc -l < /sys/kernel/debug/tracing/uprobe_events 2>/dev/null || echo 0)
  if kill -0 $BPF_PID 2>/dev/null && grep -q "Attaching .* probe" /work/bpf.err && [ "$UPROBES" -gt 0 ]; then
    BPF_C0=$(proc_cpu $BPF_PID)
    CPU_EBPF=$(run_workload "")
    BPF_C1=$(proc_cpu $BPF_PID)
    kill $BPF_PID 2>/dev/null; wait $BPF_PID 2>/dev/null
    BPF_CPU=$(awk "BEGIN{printf \"%.3f\", $BPF_C1-$BPF_C0}")
    echo "workload CPU: ${CPU_EBPF}s   bpftrace proc CPU: ${BPF_CPU}s   disk: 0 (in-kernel map)   net: 0"
    echo "attach OK (${UPROBES} uprobe(s) live); sample map output:"; grep -A3 "@" /work/bpf.out | head -6
  else
    echo "bpftrace FAILED to attach (alive=$(kill -0 $BPF_PID 2>/dev/null && echo yes || echo no), uprobes=${UPROBES}):"
    tail -5 /work/bpf.err; kill $BPF_PID 2>/dev/null; wait $BPF_PID 2>/dev/null; CPU_EBPF="n/a"; BPF_CPU="n/a"
  fi
else
  echo "bpftrace not installed"; CPU_EBPF="n/a"; BPF_CPU="n/a"
fi

echo; echo "==================== LEG D: LTTng / ros2_tracing ===================="
if command -v lttng >/dev/null; then
  rm -rf /work/trace
  lttng-sessiond --daemonize 2>/dev/null; sleep 1
  lttng create bench --output=/work/trace >/dev/null 2>&1
  # fair parity: enable only the two events our probe + eBPF hook
  lttng enable-event -u 'ros2:rcl_publish','ros2:callback_start' >/dev/null 2>&1
  lttng start >/dev/null 2>&1
  CONS_PID=$(pgrep -f lttng-consumerd | head -1)
  C0=$(proc_cpu ${CONS_PID:-0}); S0=$(proc_cpu $(pgrep -f lttng-sessiond|head -1))
  CPU_LTTNG=$(run_workload "")
  lttng stop >/dev/null 2>&1
  CONS_PID=$(pgrep -f lttng-consumerd | head -1)
  C1=$(proc_cpu ${CONS_PID:-0}); S1=$(proc_cpu $(pgrep -f lttng-sessiond|head -1))
  lttng destroy >/dev/null 2>&1
  TRACE_DISK=$(du -sb /work/trace 2>/dev/null | awk '{print $1}')
  LTTNG_CPU=$(awk "BEGIN{printf \"%.3f\", ($C1-$C0)+($S1-$S0)}")
  # Verify events were actually captured (babeltrace2). Stock ROS binaries may ship tracetools
  # WITHOUT the lttng-ust backend, in which case the tracepoints are no-ops and NOTHING is traced.
  EVENTS=$(babeltrace2 /work/trace 2>/dev/null | wc -l)
  echo "workload CPU: ${CPU_LTTNG}s   lttng daemons CPU: ${LTTNG_CPU}s   disk(CTF): ${TRACE_DISK}B   net: 0"
  echo "events captured (babeltrace2): $EVENTS"
  if ! ldd "$TT" | grep -qiE "lttng|ust"; then
    echo "NOTE: libtracetools is NOT linked against lttng-ust on this image -> ros2_tracing captures"
    echo "      NOTHING without rebuilding ROS with instrumentation. (ours works on stock binaries.)"
    LTTNG_CPU="${LTTNG_CPU} (no-capture)"
  fi
else
  echo "lttng not installed"; CPU_LTTNG="n/a"; LTTNG_CPU="n/a"; TRACE_DISK="n/a"
fi

echo; echo "==================== RESULTS ===================="
printf "%-22s %-14s %-16s %-12s %s\n" "method" "workload_cpu_s" "monitor_cpu_s" "disk_B" "vs_baseline"
printf "%-22s %-14s %-16s %-12s %s\n" "baseline"        "$CPU_BASE"  "0"          "0"             "-"
# Pass every field via -v: values like LTTNG_CPU carry a " (no-capture)" suffix (spaces/parens)
# that would break a single-quoted awk program when interpolated directly.
awk -v b="$CPU_BASE" -v w="$CPU_OURS"  -v d="$OURS_DISK" 'BEGIN{printf "%-22s %-14s %-16s %-12s +%.1f%%\n","ours (LD_PRELOAD)",w,"in-proc",d,100*(w-b)/b}'
awk -v b="$CPU_BASE" -v w="$CPU_EBPF"  -v c="$BPF_CPU"   'BEGIN{if(w=="n/a"){print "ebpf: n/a"}else printf "%-22s %-14s %-16s %-12s +%.1f%%\n","ebpf (uprobe)",w,c,"0",100*(w-b)/b}'
awk -v b="$CPU_BASE" -v w="$CPU_LTTNG" -v c="$LTTNG_CPU" -v d="$TRACE_DISK" 'BEGIN{if(w=="n/a"){print "lttng: n/a"}else printf "%-22s %-14s %-16s %-12s +%.1f%%\n","lttng (ros2_tracing)",w,c,d,100*(w-b)/b}'
echo "(workload_cpu = added CPU to monitored processes; monitor_cpu = the monitor's own process cost)"
echo "NOTE: each leg above is a SINGLE 10s sample, run-to-run variance is ~±10%, which swamps a"
echo "      <2% signal (the LTTng no-op leg has measured -9% vs baseline here). Treat any |delta|<10%"
echo "      as noise; for the real overhead figure use run_overhead_repeated.sh (interleaved, N=6)."
echo "==================== DONE ===================="

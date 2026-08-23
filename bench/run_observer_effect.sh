#!/bin/bash
# Observer effect: what does WATCHING a topic with the stock CLI do to the topic?
#
# The probe is the ruler in every arm: it counts the publisher's calls in-process, so its
# publish-side TOPIC rate is the true rate whatever subscribers exist. Arms, rotated per trial
# so drift and warm-up cancel across arms:
#   none   nothing watches
#   hz     ros2 topic hz   /heavy_0     (~100 KB ByteMultiArray @ 50 Hz)
#   echo   ros2 topic echo /heavy_0 > /dev/null
#   intra  ros2 topic hz   /intra_0     (an intra-process topic: the watcher is the first
#                                        out-of-process subscriber, so the publisher starts
#                                        serializing messages it never had to before)
# Per arm, per trial: true /heavy_0 publish Hz, the rate `hz` REPORTED, publisher / subscriber /
# intra-farm CPU seconds, and the watcher's own CPU seconds. Then mean +/- SEM per arm.
#
# Run inside a ros:<distro> container with the package at /pkg and a scratch dir at /work:
#   docker run --rm -v "$PWD":/pkg -v /tmp/oe:/work ros:humble-ros-base bash /pkg/bench/run_observer_effect.sh
set +e
cd /work || exit 1
ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
N="${N:-10}"; DUR="${DUR:-10}"; NL="${NL:-30}"; NH="${NH:-8}"; NI="${NI:-15}"
WATCH=$((DUR - 2))   # watcher attaches after 1 s of settling, detaches 1 s before the farms stop

g++ -O2 -std=c++17 -fPIC -shared -I/pkg/include \
  /pkg/src/probe/interposers.cpp /pkg/src/core/*.cpp -ldl -pthread -o /work/libprod.so 2>/dev/null \
  || { echo so_fail; exit 1; }
rm -rf /work/bbuild
cmake -S /pkg/bench -B /work/bbuild -DCMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} -DCMAKE_BUILD_TYPE=Release >/dev/null 2>&1
cmake --build /work/bbuild -j4 >/dev/null 2>&1 || { echo build_fail; exit 1; }
NODE=/work/bbuild/stress_nodes
PROBE="LD_PRELOAD=/work/libprod.so ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=2.0 ROS_TOPIC_STATS_QUIET=1"

# Mean of the probe's publish-side rate for a topic, skipping the first and last (partial) windows.
true_hz () {  # $1=log $2=topic
  awk -v t="$2" '$1=="TOPIC" && $2==t {v[n++]=$3} END{ if(n<=2){print "nan"; exit} s=0; for(i=1;i<n-1;i++) s+=v[i]; printf "%.3f", s/(n-2) }' "$1"
}
# Receive-side intra-process rate (Humble has no intra publish tracepoint; RECV carries it).
intra_hz () {  # $1=log $2=topic
  awk -v t="$2" '$1=="RECV" && $2==t {split($4,x,"="); v[n++]=x[2]} END{ if(n<=2){print "nan"; exit} s=0; for(i=1;i<n-1;i++) s+=v[i]; printf "%.3f", s/(n-2) }' "$1"
}
# How many windows carried a publish-side TOPIC line for an intra topic. A pure intra-process
# publisher never reaches rcl_publish, so this is 0 until an out-of-process subscriber (the
# watcher) attaches and serialization begins: the observer switching on a code path.
pub_windows () { awk -v t="$2" '$1=="TOPIC" && $2==t {n++} END{print n+0}' "$1"; }
cpu_of () { awk '/RESULT/{split($3,x,"="); print x[2]}' "$1"; }   # RESULT <tag> cpu_s=<v> msgs=<m>

run_arm () {  # $1=arm  -> appends one CSV row
  local arm="$1"
  rm -f /work/p.log /work/i.log /work/p.res /work/s.res /work/i.res /work/obs.txt /work/obs.times
  env $PROBE ROS_TOPIC_STATS_OUTPUT_FILE=/work/i.log $NODE intrafarm $DUR $NI 2>/work/i.res & local a=$!
  $NODE subfarm $DUR $NL $NH 2>/work/s.res & local b=$!
  sleep 0.5
  env $PROBE ROS_TOPIC_STATS_OUTPUT_FILE=/work/p.log $NODE pubfarm $DUR $NL $NH 2>/work/p.res & local c=$!
  sleep 1
  local cmd=""
  case "$arm" in
    # No ros2cli daemon: its rclpy context dies between arms in a container ("Fault 1:
    # !rclpy.ok()") and `echo`, which asks it for the topic type, then crashes at startup
    # (2026-08-23 run: 9/10 echo trials lost). `ros2 daemon stop` runs before every arm;
    # `hz` then discovers in-process by itself, `echo` needs --no-daemon to do the same.
    hz)    cmd="ros2 topic hz /heavy_0" ;;
    echo)  cmd="ros2 topic echo --no-daemon /heavy_0" ;;
    intra) cmd="ros2 topic hz /intra_0" ;;
  esac
  local obs_cpu="0.000"; local reported="nan"
  if [ -n "$cmd" ]; then
    ros2 daemon stop >/dev/null 2>&1
    # `times` in the wrapper shell reports its children's user+sys: the watcher's own CPU.
    bash -c "timeout $WATCH $cmd > /work/obs.txt 2>&1; times" > /work/obs.times
    obs_cpu=$(tail -1 /work/obs.times | awk '{
      for(i=1;i<=2;i++){ split($i,m,"m"); sub("s","",m[2]); s+=m[1]*60+m[2] } printf "%.3f", s }')
    reported=$(grep -o "average rate: [0-9.]*" /work/obs.txt | tail -1 | awk '{print $3}')
    [ -n "$reported" ] || reported="nan"
  fi
  wait $a $b $c 2>/dev/null
  # Evidence from the last trial of each arm: the probe logs and the watcher's first 2 KB.
  mkdir -p /work/evidence/$arm
  cp /work/p.log /work/evidence/$arm/pubfarm.log; cp /work/i.log /work/evidence/$arm/intrafarm.log
  [ -f /work/obs.txt ] && head -c 2000 /work/obs.txt > /work/evidence/$arm/watcher_head.txt
  printf "%s,%s,%s,%s,%s,%s,%s,%s,%s\n" "$arm" "$(true_hz /work/p.log /heavy_0)" "$reported" \
    "$(cpu_of /work/p.res)" "$(cpu_of /work/s.res)" "$(cpu_of /work/i.res)" "$obs_cpu" \
    "$(intra_hz /work/i.log /intra_0)" "$(pub_windows /work/i.log /intra_0)"
}

ARMS=(none hz echo intra)
CSV=/work/observer_effect.csv
echo "arm,heavy0_true_hz,hz_reported,pub_cpu_s,sub_cpu_s,intra_cpu_s,watcher_cpu_s,intra0_recv_intra_hz,intra0_pub_windows" > $CSV
echo "=== observer effect: N=$N trials x ${#ARMS[@]} arms, DUR=${DUR}s, watcher attached ${WATCH}s, $NL light + $NH heavy + $NI intra topics ==="
for i in $(seq 1 $N); do
  for k in 0 1 2 3; do
    arm=${ARMS[$(( (i + k) % 4 ))]}     # rotate the starting arm each trial
    row=$(run_arm "$arm"); echo "$row" >> $CSV; echo "trial $i  $row"
  done
done

echo
echo "=== mean +/- SEM per arm ==="
printf "%-6s %-18s %-18s %-18s %-18s %-18s %-18s %-18s %-18s\n" arm "heavy0 true Hz" "hz reported" "pub cpu_s" "sub cpu_s" "intra cpu_s" "watcher cpu_s" "intra0 intra Hz" "intra0 pub windows"
for arm in "${ARMS[@]}"; do
  awk -F, -v a="$arm" '$1==a {
      for(c=2;c<=9;c++){ if($c!="nan"){ n[c]++; s[c]+=$c; q[c]+=$c*$c } } }
    END{
      printf "%-6s", a;
      for(c=2;c<=9;c++){
        if(n[c]>0){ m=s[c]/n[c]; v=(n[c]>1)?(q[c]-n[c]*m*m)/(n[c]-1):0; if(v<0)v=0; sem=sqrt(v)/sqrt(n[c]);
          printf " %8.3f +/- %-6.3f", m, sem } else printf " %17s", "n/a" }
      printf "\n" }' $CSV
done
echo
echo "CSV: $CSV   evidence: /work/evidence/<arm>/"; uname -a > /work/platform.txt; lscpu | grep -i "model name\|^CPU(s)" >> /work/platform.txt; echo "distro=$ROS_DISTRO rmw=${RMW_IMPLEMENTATION:-default}" >> /work/platform.txt
echo "=== DONE ==="

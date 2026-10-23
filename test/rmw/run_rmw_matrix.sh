#!/bin/bash
# RMW support matrix for ros2_pulse (ROADMAP R6): prove, not assume, that the probe works
# unmodified on middlewares other than the CI-tested rmw_fastrtps_cpp. The hooks sit above the
# rmw layer (tracetools symbols called by rclcpp/rcl), so any conformant RMW should pass; this
# script is what turns that "should" into evidence, and keeps the README's matrix rows
# re-checkable with one command.
#
# Legs, each = talker+listener in separate processes + an intra-process node, probe preloaded,
# rates asserted at 50 Hz +/-30% by test/rmw/rmw_matrix_check.py (same parser as CI):
#   fastrtps       control leg, the CI-tested default; if THIS fails, blame the harness/host,
#                  not the middleware under test
#   cyclonedds     ros-<distro>-rmw-cyclonedds-cpp, plain (UDP loopback) transport
#   cyclonedds_shm CycloneDDS + iceoryx shared memory: iox-roudi daemon + CYCLONEDDS_URI with
#                  <SharedMemory><Enable>true</>. Also runs the fixed-size UInt64 pair, because
#                  String is not a self-contained type and is NOT SHM-eligible, without the
#                  fixed pair this leg would silently measure loopback UDP. Skipped (reported)
#                  if the distro's libddsc was built without iceoryx.
#   zenoh          ros-<distro>-rmw-zenoh-cpp, no DDS at all. Needs the zenoh router
#                  (rmw_zenohd) up first: rmw_zenoh sessions reach the router at
#                  tcp/localhost:7447 for discovery by default (multicast scouting off).
#   zenoh_shm      rmw_zenoh with Zenoh shared memory on for the router and every session
#                  (ZENOH_CONFIG_OVERRIDE). The SHM message-size threshold drops from 512 to
#                  8 bytes, otherwise the test messages are too small and the leg would
#                  silently measure the network path. Zenoh mlock()s every SHM segment it
#                  creates or opens, so the 48 MiB default pool needs RLIMIT_MEMLOCK above
#                  docker's inherited 8 MiB (the host side passes --ulimit memlock=-1:-1);
#                  without it the ShmProvider fails with ENOMEM and rmw_zenoh quietly falls
#                  back to the network. Proof SHM engaged: no process logs a ShmProvider
#                  error, and a pool-sized segment shows up in /dev/shm. Zenoh also creates
#                  small control segments that appear whether or not data uses SHM (an
#                  intra-only node gets one), so "any new segment" is not evidence.
#
# Host usage:      test/rmw/run_rmw_matrix.sh [distro ...]        (default: jazzy)
# Evidence lands in test/rmw/out/<distro>/<leg>/ (pulse logs, process stderr, daemon logs,
# versions.txt), the excerpts quoted in the README matrix come straight from there.
# Container path: re-invokes itself with --inside <distro> in ros:<distro>; the repo is
# mounted read-only and copied, so a run can never dirty the working tree.
set -u

# ---------------------------------------------------------------------------- container side
if [ "${1:-}" = "--inside" ]; then
  DISTRO="$2"
  OUT=/pulse_out
  # ROS setup scripts reference unset vars; suspend -u only around them.
  set +u; source "/opt/ros/${DISTRO}/setup.bash"; set -u

  echo "==== apt: rmw packages ===="
  apt-get update -qq || exit 2
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    "ros-${DISTRO}-rmw-cyclonedds-cpp" "ros-${DISTRO}-rmw-zenoh-cpp" \
    > "$OUT/apt.log" 2>&1 || { tail -5 "$OUT/apt.log"; exit 2; }
  {
    echo "image=ros:${DISTRO}  run_date=$(date -u +%F)"
    dpkg-query -W -f '${Package} ${Version}\n' \
      "ros-${DISTRO}-rmw-fastrtps-cpp" "ros-${DISTRO}-rmw-cyclonedds-cpp" \
      "ros-${DISTRO}-cyclonedds" "ros-${DISTRO}-rmw-zenoh-cpp" \
      "ros-${DISTRO}-iceoryx-posh" 2>/dev/null
  } | tee "$OUT/versions.txt"

  echo "==== colcon build (probe + test nodes) ===="
  mkdir -p /ws/src && cp -r /pulse_src /ws/src/ros2_pulse
  cd /ws || exit 2
  colcon build --packages-select ros2_pulse --cmake-args -DCMAKE_BUILD_TYPE=Release \
    > "$OUT/build.log" 2>&1 || { tail -30 "$OUT/build.log"; exit 2; }
  set +u; source install/setup.bash; set -u

  CHECK="/ws/src/ros2_pulse/test/rmw/rmw_matrix_check.py"
  FAILED=0

  run_leg() { # <leg> <rmw> [extra rmw_matrix_check args]
    local leg="$1" rmw="$2"; shift 2
    mkdir -p "$OUT/$leg"
    echo "==== leg: $leg (RMW_IMPLEMENTATION=$rmw) ===="
    RMW_IMPLEMENTATION="$rmw" python3 "$CHECK" --leg "$leg" --out "$OUT/$leg" "$@" \
      || FAILED=1
  }

  run_leg fastrtps rmw_fastrtps_cpp
  run_leg cyclonedds rmw_cyclonedds_cpp

  # --- CycloneDDS + iceoryx shared memory ---
  LIBDDSC="$(find "/opt/ros/${DISTRO}" -name 'libddsc.so.*' | head -1)"
  ROUDI="$(command -v iox-roudi || find "/opt/ros/${DISTRO}" -name iox-roudi | head -1)"
  if ldd "$LIBDDSC" | grep -q libiceoryx_posh && [ -n "$ROUDI" ]; then
    mkdir -p "$OUT/cyclonedds_shm"
    ldd "$LIBDDSC" | grep iceoryx > "$OUT/cyclonedds_shm/libddsc_iceoryx_linkage.txt"
    # ${PULSE_ROLE} is expanded by cyclone's config reader per process (the check driver sets
    # it to talker/listener/…), giving each process its own SHM trace file, the trace is the
    # transport-path evidence that samples really moved through iceoryx, not loopback UDP.
    cat > /tmp/cyclonedds_shm.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <SharedMemory>
      <Enable>true</Enable>
      <LogLevel>info</LogLevel>
    </SharedMemory>
    <Tracing>
      <Category>shm</Category>
      <OutputFile>/pulse_out/cyclonedds_shm/cdds-trace-${PULSE_ROLE}.log</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
    "$ROUDI" > "$OUT/cyclonedds_shm/roudi.log" 2>&1 &
    ROUDI_PID=$!
    sleep 2
    if kill -0 "$ROUDI_PID" 2>/dev/null; then
      CYCLONEDDS_URI="file:///tmp/cyclonedds_shm.xml" \
        run_leg cyclonedds_shm rmw_cyclonedds_cpp --fixed
      kill "$ROUDI_PID" 2>/dev/null; wait "$ROUDI_PID" 2>/dev/null
      echo "---- roudi registrations (SHM runtime attach evidence) ----"
      grep -iE "registered|application" "$OUT/cyclonedds_shm/roudi.log" | head -8
      echo "---- cyclone shm trace (data-path evidence) ----"
      grep -ilE "iceoryx|shm" "$OUT"/cyclonedds_shm/cdds-trace-*.log 2>/dev/null | head -4
    else
      echo "FAIL cyclonedds_shm: iox-roudi died at start (see roudi.log, usually /dev/shm" \
           "too small; the host side of this script passes --shm-size=1g for that reason)"
      FAILED=1
    fi
  else
    echo "SKIP cyclonedds_shm: this distro's libddsc has no iceoryx linkage or no iox-roudi"
    echo "no-iceoryx" > "$OUT/cyclonedds_shm.skipped"
  fi

  # --- rmw_zenoh: bring the router up first ---
  # Run rmw_zenohd itself, not through `ros2 run`: before kilted, ros2run does not forward
  # SIGTERM (ros2/ros2cli#899), so killing the wrapper orphaned the router with 7447 still
  # bound, and the next leg's router died with EADDRINUSE while the port probe below happily
  # found the old one.
  ZENOHD="$(ros2 pkg prefix rmw_zenoh_cpp)/lib/rmw_zenoh_cpp/rmw_zenohd"
  start_router() { # <log>; sets ROUTER_PID, returns 0 once OUR router accepts on tcp/7447
    "$ZENOHD" > "$1" 2>&1 &
    ROUTER_PID=$!
    for _ in $(seq 1 40); do
      kill -0 "$ROUTER_PID" 2>/dev/null || return 1
      if (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then return 0; fi
      sleep 0.5
    done
    return 1
  }
  stop_router() { # and wait until 7447 is free, so the next leg cannot reach a stale router
    kill "$ROUTER_PID" 2>/dev/null; wait "$ROUTER_PID" 2>/dev/null
    for _ in $(seq 1 20); do
      (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null || return 0
      sleep 0.25
    done
    echo "WARN: tcp/7447 still open after the router exited"
  }

  mkdir -p "$OUT/zenoh"
  if start_router "$OUT/zenoh/router.log"; then
    run_leg zenoh rmw_zenoh_cpp
  else
    echo "FAIL zenoh: router (rmw_zenohd) never opened tcp/7447, see zenoh/router.log"
    FAILED=1
  fi
  stop_router

  # --- rmw_zenoh + Zenoh shared memory ---
  mkdir -p "$OUT/zenoh_shm"
  ZENOH_SHM_POOL=50331648 # rmw_zenoh's default transport_optimization/pool_size (48 MiB)
  SHM_OPT=transport/shared_memory/transport_optimization
  export ZENOH_CONFIG_OVERRIDE="transport/shared_memory/enabled=true;${SHM_OPT}/message_size_threshold=8;${SHM_OPT}/pool_size=${ZENOH_SHM_POOL}"
  echo "RLIMIT_MEMLOCK: $(ulimit -l)" | tee "$OUT/zenoh_shm/memlock.txt"
  if start_router "$OUT/zenoh_shm/router.log"; then
    # Snapshot after the router is up, so its own segments are not counted as evidence.
    find /dev/shm -maxdepth 1 -name '*.zenoh' -printf '%f\n' | sort \
      > "$OUT/zenoh_shm/dev_shm_before.txt"
    # Segments can go away when the processes exit, so sample /dev/shm (with sizes) while
    # the leg runs.
    ( while :; do find /dev/shm -maxdepth 1 -name '*.zenoh' -printf '%s %f\n'; sleep 0.5; done ) \
      > "$OUT/zenoh_shm/dev_shm_during.txt" 2>&1 &
    SAMPLER_PID=$!
    run_leg zenoh_shm rmw_zenoh_cpp --fixed
    kill "$SAMPLER_PID" 2>/dev/null; wait "$SAMPLER_PID" 2>/dev/null
    awk 'NR == FNR { before[$1]; next } !($2 in before)' \
      "$OUT/zenoh_shm/dev_shm_before.txt" "$OUT/zenoh_shm/dev_shm_during.txt" \
      | sort -u -k2,2 > "$OUT/zenoh_shm/dev_shm_new.txt"
    echo "---- new /dev/shm segments during the leg (bytes name) ----"
    head -8 "$OUT/zenoh_shm/dev_shm_new.txt"
    if grep -lE "Error creating (lazy )?ShmProvider" "$OUT"/zenoh_shm/*.stderr.txt; then
      echo "FAIL zenoh_shm: ShmProvider creation failed in the processes above, data went" \
           "over the network (see memlock.txt; zenoh mlock()s the whole pool)"
      FAILED=1
    elif ! awk -v p="$ZENOH_SHM_POOL" '$1 >= p { f = 1 } END { exit !f }' \
        "$OUT/zenoh_shm/dev_shm_new.txt"; then
      echo "FAIL zenoh_shm: no pool-sized (${ZENOH_SHM_POOL} B) segment appeared, the SHM" \
           "data path never engaged"
      FAILED=1
    fi
  else
    echo "FAIL zenoh_shm: router (rmw_zenohd) never opened tcp/7447, see zenoh_shm/router.log"
    FAILED=1
  fi
  stop_router
  unset ZENOH_CONFIG_OVERRIDE

  # Evidence must belong to the host user, not root, otherwise the next run cannot clean up
  # its own out/ dir. /pulse_src is the host checkout, so its owner IS the host user.
  chown -R "$(stat -c %u:%g /pulse_src)" "$OUT" 2>/dev/null || true

  echo "==== matrix result: $([ "$FAILED" -eq 0 ] && echo PASS || echo FAIL) ===="
  exit "$FAILED"
fi

# ---------------------------------------------------------------------------- host side
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
command -v docker >/dev/null || { echo "docker required"; exit 2; }
DISTROS=("${@:-jazzy}")
RC=0
for d in "${DISTROS[@]}"; do
  OUT_DIR="$REPO_DIR/test/rmw/out/$d"
  mkdir -p "$OUT_DIR"
  echo "############ ros:$d ############"
  # Distinct ROS_DOMAIN_ID per distro: DDS multicast discovery DOES cross containers on the
  # default docker bridge, so concurrent matrix runs (or any domain-0 ROS traffic on the host)
  # would feed a leg's listener from a foreign talker, observed as RECV 100 Hz from two 50 Hz
  # talkers when the humble and kilted runs overlapped. cksum keeps the mapping deterministic
  # (humble=66 jazzy=22 kilted=79 rolling=42) so evidence stays reproducible.
  DOM=$(( ( $(printf '%s' "$d" | cksum | cut -d' ' -f1) % 90 ) + 10 ))
  # --shm-size: iceoryx RouDi's default mempool layout does not fit docker's 64 MB /dev/shm.
  # --ulimit memlock: Zenoh mlock()s its 48 MiB SHM pool, docker's inherited limit is 8 MiB.
  docker run --rm --shm-size=1g --ulimit memlock=-1:-1 -e ROS_DOMAIN_ID="$DOM" \
    -v "$REPO_DIR:/pulse_src:ro" -v "$OUT_DIR:/pulse_out" \
    "ros:$d" bash /pulse_src/test/rmw/run_rmw_matrix.sh --inside "$d" || RC=1
done
exit $RC

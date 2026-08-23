// Every number on screen traces to a committed file. Fill from bench output; never round
// past what the SEM supports.

export const OBSERVER = {
  // bench/out/observer_effect/summary.txt, ros:humble, 8 x ~100 KB @ 50 Hz + 30 light + 15 intra,
  // N=10 rotated arms, watcher attached 8 s. Stock tools were ACCURATE: publisher held 50.000 Hz
  // in every arm. The cost is the watcher's own core share, and on intra-process topics, the
  // serialization the watcher switches on.
  source: 'bench/RESULTS.md · observer effect',
  truePublishHz: '50.0',
  hzReportedHz: '49.8', // 49.845 ± 0.053
  hzWatcherCorePct: 7, // 0.566 s ± 0.005 over 8 s
  echoWatcherCorePct: 31, // 2.504 s ± 0.007 over 8 s, output to /dev/null
  intraCpuPct: 52, // 0.303 -> 0.460 s on the watched process
  intraPubWindowsNone: 0,
  intraPubWindowsWatched: 4, // of 5, in 10/10 trials
};

export const PROBE = {
  hotpathFixedNs: 0.3, // bench/RESULTS.md, x86-64
  hotpathAltNs: '0.6-1.2',
  orinHotpathFixedNs: 0.9, // test/orin/RESULTS.md
  orinHotpathAltNs: 2.1,
  orinJitterNs: 51, // +51.0 ± 0.6 ns/msg, Jetson AGX Orin
  x86JitterNs: 24,
  stressCpuPct: '≈2', // +1.9 % ± 0.7 % pooled, worst-case 4900 msg/s stress
  sockets: 0,
};

export const LINKS = {
  repo: 'github.com/TanayK07/ros2_pulse',
  pip: 'pip install ros2-pulse-top',
  license: 'Apache-2.0',
};

# Overhead and measurements

Every number here comes from a script in the repository, with the raw trials committed next to
the results. Single runs of these workloads swing by ±4 %, so each figure below is a paired,
order-alternated comparison (N = 10) reported with its standard error.

## Cost of the probe

Workload: about 4,900 msg/s across 53 topics in one machine (30 light topics at 100 Hz, 8 heavy
~100 KB topics at 50 Hz inter-process, 15 intra-process topics at 100 Hz), in stock `ros:<distro>`
containers. This is a deliberately harsh stress; real graphs see less.

| Distro | CPU added, paired ± SEM |
|---|---|
| Humble | +2.5 % ± 1.5 % |
| Jazzy | +2.5 % ± 1.6 % |
| Kilted | +1.5 % ± 0.9 % |
| **Pooled** | **+1.9 % ± 0.7 %** |

That is about 1 µs of added work per message against about 51 µs the stack already spends
delivering it. Controlled experiments show where it is not: an empty shim with the same symbols
costs nothing measurable (−0.3 %), and turning flushing off does not change the result. What
remains is the cost of the extra call layer and its cache footprint.

## Hot path

| | x86-64 | Jetson AGX Orin |
|---|---|---|
| Count, one endpoint per thread | 0.3 ns | 0.9 ns |
| Count, alternating across 4 endpoints | 0.6 to 1.2 ns | 2.1 to 2.2 ns |
| Gap tracking clock read (`ROS_TOPIC_STATS_JITTER=1`) | +24 ns | +51 ns |
| For scale: one LTTng-UST tracepoint | about 158 ns | |

On the Orin (L4T R36.4, a 77-node Humble and Cyclone DDS production stack) the probe loaded into
every process on the deployed binaries and opened no sockets.

## Cost of watching with the CLI

The same stress farm on `ros:humble` with Cyclone DDS, one 16-core x86-64 machine, N = 10. The
probe measured the true rates in every arm.

| Watcher | Watcher's own CPU | Effect on the watched system |
|---|---|---|
| nothing | 0 | baseline |
| `ros2 topic hz` on one 100 KB, 50 Hz topic | 7.1 % of a core | none measurable |
| `ros2 topic echo` on the same topic | 31.3 % of a core | none measurable |
| `ros2 topic hz` on an intra-process topic | 8.2 % of a core | publishing process **+52 % CPU**: it starts serializing every message for the new subscriber |

The rate `ros2 topic hz` printed was right (within 0.3 %). The point is the cost, per topic and
for as long as you look, and that an intra-process topic changes behaviour once something
outside the process subscribes.

## Reproduce

```bash
# probe overhead, per distro (paired trials + microbench)
docker run --rm -e ROS_DISTRO=jazzy -v $PWD:/pkg:ro -v /tmp/bench-jazzy:/work ros:jazzy \
  bash /pkg/bench/run_overhead_repeated.sh

# what ros2 topic hz / echo cost
docker run --rm -v $PWD:/pkg -v /tmp/oe:/work ros:humble-ros-base bash /pkg/bench/run_observer_effect.sh
```

Full write-ups: [bench/RESULTS.md](https://github.com/TanayK07/ros2_pulse/blob/main/bench/RESULTS.md),
[test/orin/RESULTS.md](https://github.com/TanayK07/ros2_pulse/blob/main/test/orin/RESULTS.md),
and the blog post
[Watching a ROS 2 topic changes it](https://github.com/TanayK07/ros2_pulse/blob/main/docs/blog/2026-09-18-watching-a-ros2-topic-changes-it.md).

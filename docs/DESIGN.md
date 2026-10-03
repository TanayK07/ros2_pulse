# ros2_pulse design

## Problem

Answer, in production and cheaply: *is each topic flowing at its expected rate, and which nodes are
alive?*, including **intra-process** traffic, which `use_intra_process_comms` routes through the
rclcpp `IntraProcessManager` ring buffer, bypassing the middleware (rmw) and the built-in topic
statistics ([rclcpp#2911](https://github.com/ros2/rclcpp/issues/2911)).

## Why hook the tracetools layer (not rmw)

rclcpp calls the tracetools `ros_trace_*` functions unconditionally on every publish and every
callback, they are exported symbols in `libtracetools.so`, and the LTTng enable-check is *inside*
them, so the call happens even with no tracing session. Hooking here (one layer above the DDS
vendor) buys two things the rmw layer cannot:

1. **Intra-process visibility**: `callback_start(callback, is_intra_process)` fires for every
   subscription callback regardless of transport.
2. **Middleware independence**: works identically on FastRTPS, CycloneDDS, etc.

A prior approach (`rmw_stats_shim`) relied on `RMW_IMPLEMENTATION_WRAPPER`, which does not exist in
stock ROS 2 Humble `rmw_implementation`, so it silently never ran. `ros2_pulse` needs no patched
rmw and no ROS rebuild.

**Related work.** [CARET](https://tier4.github.io/caret_doc/) (Tier IV) independently validates
this exact mechanism at Autoware scale, LD_PRELOAD function hooking over the tracetools layer,
but points it at deep offline latency/chain analysis (LTTng sessions, forked rclcpp, Jupyter
post-processing). ros2_pulse makes the opposite trade: zero dependencies and an always-on,
online Hz file. See `docs/ALTERNATIVES.md` for the full landscape.

## Mechanism

`libros2_pulse.so` is `LD_PRELOAD`ed. It exports the same symbols as `libtracetools.so`; the dynamic
linker binds rclcpp's calls to ours first. Each interposer records a stat, then forwards to the real
function obtained once via `dlsym(RTLD_NEXT, …)` (cached in a function-local `static`, resolving it
per call would cost a symbol-table lookup per message).

Hooked functions:
- Graph (rare): `rcl_node_init`, `rcl_publisher_init`, `rcl_subscription_init`,
  `rclcpp_subscription_init`, `rclcpp_subscription_callback_added`, build the handle→topic maps.
- Publish (inter-process): `rcl_publish` → increment the publisher's topic counter.
- Receive, rclcpp (all transports): `callback_start` → resolve callback→topic, increment
  inter/intra by the `is_intra_process` flag.
- Receive, every other rcl client (rclpy, rclc, ...): `rmw_take(rmw_sub, msg, source_ts, taken)`
  → if `taken`, map rmw handle → rcl handle → topic (from `rcl_subscription_init`) and increment
  inter. A subscription that `rclcpp_subscription_init` named is skipped here, because
  `callback_start` already counts it.

### Why rmw_take and not rcl_take

`rcl_take`'s tracepoint carries only the message pointer, and `rcl_take()` fires it before checking
whether anything was taken, so it can neither be attributed to a topic nor trusted as a receive.
Correlating it with `rmw_take` through a thread-local would work (rcl calls
`rmw_take_with_info()`, which fires `rmw_take`, then fires `rcl_take` on the same thread), but it
adds a second interposer per message and still gets every fact from `rmw_take`. So the probe counts
at `rmw_take` directly. The cost: the hook depends on the rmw emitting it. On Humble,
rmw_cyclonedds_cpp and rmw_fastrtps_cpp emit it for typed takes only (serialized and loaned takes
emit nothing); rmw_connextdds emits it only from Jazzy.

### Receive dedupe: one source per subscription

An rclcpp inter-process delivery fires both `rmw_take` and `callback_start(intra=false)`. The probe
keeps `callback_start` as the rclcpp source, because it also sees the serialized and loaned
deliveries that emit no `rmw_take` on Humble, and the intra-process ones that never touch rmw. It
also avoids a subtle overcount: with intra-process on and an external subscriber, rclcpp's
subscription takes the DDS copy of its own publish and drops it, so `rmw_take` fires for a message
whose callback ran via intra-process. Ownership is decided per subscription, not per topic, so an
rclcpp and an rclpy subscription on one topic in one process each count their own deliveries.
Takes on rmw handles that no `rcl_subscription_init` named (the rmw's internal discovery reader)
and takes with `taken=false` are ignored.

## Core (`core/`, no ROS dependency)

- `TopicRegistry`: owns per-topic `sTopicCounter{atomic pub_inter, recv_inter, recv_intra}` (publish
  vs receive kept in separate buckets so a same-process pub+sub of one topic never collide); pointer-
  keyed maps for publisher_handle→counter and the callback→…→topic chain.
- **Hot path**: a thread-local `{registry-id, key, counter*}` cache serves the common repeated-
  endpoint case with a single relaxed atomic increment; a miss takes a `shared_lock` (concurrent),
  and only the first sighting of a callback takes the `unique_lock` to resolve + cache. No global
  mutex, no per-message string hashing. The take path shares the cache; its entries are tagged
  with the registry id plus a subscription-graph epoch, bumped by every subscription init, so a
  recycled rmw handle or a late rclcpp ownership mark is never served a stale verdict.
- `Timer`: background thread; every window: snapshot + reset counters, compute Hz, append to file.

## Key nuances (learned the hard way)

- **Lazy callback resolution.** For an intra-process subscription, rclcpp creates a separate intra
  waitable whose `callback_added` fires *before* its `sub_handle→topic` chain is populated. Resolve
  at `callback_start` (everything is populated by delivery time), not eagerly.
- **Thread-local cache is scoped by a per-registry id, not `this`.** Otherwise a recycled
  stack/heap address (e.g. across unit tests) can serve a destroyed instance's counter → UAF.
- **Humble has no intra-*publish* tracepoint** (`rclcpp_intra_publish` landed later), so intra rate
  is measured receive-side.
- **Benchmarking:** single short samples carry ~±10% variance; overhead must be measured with
  interleaved trials + averaging (see `bench/`). The clean per-op signal is the microbench.

## Layout

```
include/ros2_pulse/core/   pure C++: registry, timer, types
src/core/                  core impl
src/probe/interposers.cpp  LD_PRELOAD entry points + dlsym(RTLD_NEXT)
test/unit/                 GTest on the core (no ROS)
test/integration/          launch/pytest end-to-end under the real .so
bench/                     bake-off harness + RESULTS + microbench
test/orin/                 on-hardware field-test kit
```

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
- Receive (all transports): `callback_start` → resolve callback→topic, increment inter/intra by the
  `is_intra_process` flag.
- Loaned messages: `rcl_publish_loaned_message` and `rcl_take_loaned_message`, real rcl
  functions rather than tracepoints, see below.

## Loaned messages: wrapping rcl, not a tracepoint

At the tracepoint level a loaned message is indistinguishable from a copied one
([rclcpp#3153](https://github.com/ros2/rclcpp/issues/3153) asks for exactly this introspection):

| distro | `rcl_publish_loaned_message` | `rcl_take_loaned_message` |
|---|---|---|
| Humble | no tracepoint at all | no tracepoint |
| Jazzy | fires the plain `rcl_publish` tracepoint | no tracepoint |
| Rolling | fires the plain `rcl_publish` tracepoint | fires the plain `rcl_take` tracepoint |

So the probe interposes the two rcl functions themselves, by name, through the same preload
mechanism. Both are reached through a PLT: the publish from `rclcpp::Publisher::publish(
LoanedMessage&&)`, a header template compiled into the user's binary, the take from
`rclcpp::Executor::execute_subscription` in `librclcpp.so`. `librcl.so` is not linked with
`-Bsymbolic`, so the preloaded definitions win.

**Why a successful call is a real loan.** rclcpp only reaches these functions when the rmw said
it can loan (line numbers from the `humble` branches, rclcpp 16.0.x):
- `LoanedMessage`'s constructor calls `rcl_borrow_loaned_message` only if `can_loan_messages()`,
  otherwise it allocates locally (`rclcpp/include/rclcpp/loaned_message.hpp:66`).
- `Publisher::publish(LoanedMessage&&)` throws on an intra-process publisher (`publisher.hpp:411`),
  calls `do_loaned_message_publish` and so `rcl_publish_loaned_message` (`publisher.hpp:490`)
  only if `can_loan_messages()` (`publisher.hpp:421`), and otherwise publishes the local message
  through `do_inter_process_publish`, that is plain `rcl_publish`.
- `GenericPublisher::publish_as_loaned_msg` reaches `rcl_publish_loaned_message`
  (`src/rclcpp/generic_publisher.cpp:71`) only after `rcl_borrow_loaned_message` succeeded, and
  it throws when the rmw cannot loan.
- The executor takes through `rcl_take_loaned_message` (`src/rclcpp/executor.cpp:615`) only in the
  `subscription->can_loan_messages()` branch (`executor.cpp:603`); every other subscription is
  taken by copy. Jazzy has the same branch.
- Below rclcpp, `rcl_publish_loaned_message` (`rcl/src/rcl/publisher.c:279`) returns
  `RCL_RET_ERROR` when the rmw fails, and Fast DDS answers `RMW_RET_UNSUPPORTED` for a publisher
  or subscription that cannot loan (`rmw_fastrtps_shared_cpp/src/rmw_publish.cpp:130`,
  `rmw_take.cpp:492`). Fast DDS sets `can_loan_messages` only for data sharing on a plain
  (bounded, self-contained) type (`rmw_fastrtps_cpp/src/publisher.cpp:313`).
- `can_loan_messages()` is the rmw's own flag AND `ROS_DISABLE_LOANED_MESSAGES`: on Humble the
  publisher side is on unless that variable is `1` (`rcl/src/rcl/publisher.c:438`), the
  subscription side is off unless it is `0` (`rcl/src/rcl/subscription.c:734`).

**Counting rules.**
- A publish loan counts when `rcl_publish_loaned_message` returns `RCL_RET_OK`; a take loan when
  `rcl_take_loaned_message` returns `RCL_RET_OK` with a non-null message (`TAKE_FAILED` is a
  spurious wake-up, not a delivery).
- Handles map to topics through the existing `rcl_publisher_init` / `rcl_subscription_init` hooks:
  rclcpp passes the very same `rcl_publisher_t*` / `rcl_subscription_t*`.
- `TOPIC` / `pub_inter_hz` stays the TOTAL publish rate and the loaned rate is a subset. On Jazzy+
  the plain tracepoint inside the loaned call has already counted the publish, on Humble nothing
  has. The probe does not sniff the distro: it learns, from the first successful loaned publish
  in the process, whether the tracepoint fired inside it (a thread-local latch the tracepoint
  interposer marks), caches that per process (one librcl per process), and from then on adds the
  publish to the total only where no tracepoint did. No double count on any distro, and on Humble
  a loaned publisher finally has a `TOPIC` line (it had none before: the probe saw nothing).
- The receive total stays the `callback_start` count, which a loaned delivery also fires; the
  loaned take count is a subset of it.
- Output: an additive `LOAN <topic> pub|recv hz=` line and `pub_loaned_hz` / `recv_loaned_hz`
  jsonl keys, absent when zero, so loan-free logs stay byte-identical.

**Getting a real loan on Humble (how the e2e test does it).** Fast DDS 2.6 loans only with
data sharing, and rmw_fastrtps forces data sharing off unless it reads QoS from XML:

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/datasharing.xml   # FASTDDS_... on Fast DDS 3
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0   # Humble: subscriptions do not loan otherwise
```

```xml
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <data_writer profile_name="default_datawriter" is_default_profile="true">
    <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    <qos><data_sharing><kind>AUTOMATIC</kind></data_sharing></qos>
  </data_writer>
  <data_reader profile_name="default_datareader" is_default_profile="true">
    <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    <qos><data_sharing><kind>AUTOMATIC</kind></data_sharing></qos>
  </data_reader>
</profiles>
```

The message type must be plain (`std_msgs/msg/Float64` or `UInt64` loans, `std_msgs/msg/String`
falls back), and the subscriber should use a const-reference callback (rclcpp warns that loans
are only safe with those). Independently of the probe, the borrowed pointer then lives in a
`/dev/shm/fast_datasharing_*` mapping instead of the heap.

**Hot-path cost.** Every ordinary publish now also runs `noteRclPublishTracepoint()`, a relaxed
load of a global that is nonzero only while a process is learning (its first loaned publish).
Measured on the real `.so` with paired, order-alternated n=10 trials (a stub librcl behind the
probe), across three such runs: at most +0.35 ns per ordinary publish on a ~6 to 7 ns
interposer path (two of the runs could not separate it from zero, e.g. -0.02 +- 0.06 ns at 1
thread and +0.01 +- 0.07 ns at 8), and +5.7 to +8.0 ns per loaned publish for the wrapper
(+6.6 ns at 1 thread, +7.9 ns at 8). Two designs were measured and dropped:
a thread-local check on every publish (+1.4 ns with the default TLS model of a shared object,
+0.45 ns with initial-exec) and a process-wide "loan in flight" counter (one shared RMW per
loaned publish, 340 ns per loaned publish with 8 publishing threads).

**ABI caveat.** `ros_trace_*` is a tracing contract; `rcl_publish_loaned_message(const
rcl_publisher_t*, void*, rmw_publisher_allocation_t*)` and `rcl_take_loaned_message(const
rcl_subscription_t*, void**, rmw_message_info_t*, rmw_subscription_allocation_t*)` are ordinary
API whose signatures are stable within a distro and identical Humble to Rolling today, but are
not promised across distros. The wrappers therefore use only opaque pointers and the `int32_t`
`rcl_ret_t`, forward every argument verbatim, dereference nothing except the take's
out-parameter after success, and are exported by exact name in `src/probe/exports.map` (a
wildcard would let any future `rcl_*` hook in unreviewed). The real function is found with
`dlsym(RTLD_NEXT, ...)`, which searches only the global scope. A plugin that links librcl and is
`dlopen()`ed `RTLD_LOCAL` by a host that does not still binds its call to the preloaded wrapper,
but its librcl is not in the global scope, so the wrapper falls back to
`dlopen("librcl.so", RTLD_NOLOAD)` and looks the symbol up in that already-loaded library
(pinned by `test_wrapper_forwards_when_librcl_is_rtld_local`). Without that fallback the probe
would turn a working loaned publish into an error. If neither lookup finds the function, the
wrapper warns once on stderr and returns `RCL_RET_ERROR` instead of crashing. A future distro
that changes either signature needs a probe release.

## Core (`core/`, no ROS dependency)

- `TopicRegistry`: owns per-topic `sTopicCounter{atomic pub_inter, recv_inter, recv_intra}` (publish
  vs receive kept in separate buckets so a same-process pub+sub of one topic never collide); pointer-
  keyed maps for publisher_handle→counter and the callback→…→topic chain.
- **Hot path**: a thread-local `{registry-id, key, counter*}` cache serves the common repeated-
  endpoint case with a single relaxed atomic increment; a miss takes a `shared_lock` (concurrent),
  and only the first sighting of a callback takes the `unique_lock` to resolve + cache. No global
  mutex, no per-message string hashing.
- `Timer`: background thread; every window: snapshot + reset counters, compute Hz, append to file.

## Key nuances (learned the hard way)

- **Lazy callback resolution.** For an intra-process subscription, rclcpp creates a separate intra
  waitable whose `callback_added` fires *before* its `sub_handle→topic` chain is populated. Resolve
  at `callback_start` (everything is populated by delivery time), not eagerly.
- **Thread-local cache is scoped by a per-registry id, not `this`.** Otherwise a recycled
  stack/heap address (e.g. across unit tests) can serve a destroyed instance's counter → UAF.
- **Humble has no intra-*publish* tracepoint** (`rclcpp_intra_publish` landed later), so intra rate
  is measured receive-side.
- **Humble has no tracepoint in `rcl_publish_loaned_message` either**, so before the loaned wrappers
  a loaned publisher was invisible on Humble (no `TOPIC` line at all).
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

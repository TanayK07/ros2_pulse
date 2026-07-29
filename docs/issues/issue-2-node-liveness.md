# Issue #2 — "node liveness" is monotonic, never decremented

Internal design note for the fix on branch `fix/node-liveness`.
Companion: [KNOWN_ISSUES.md](../KNOWN_ISSUES.md) (#2), [DESIGN.md](../DESIGN.md),
[TESTING_PLAN.md](../TESTING_PLAN.md) (#2 test rows).

## Problem

The README markets **"active-node liveness"** ("which nodes are alive?"), but the data we emit is
really *"every node whose `rcl_node_init` this process ever observed"*:

- `NODE /x` is printed every window forever after a node is initialized — even if that node has
  crashed or shut down.
- A re-`init` of the same node name (rare, but possible in tests / component churn) **duplicates**
  the entry.

## Root cause (file:line)

- `TopicRegistry::onNodeInit` only **appends** to `m_nodes`, with no dedup and no removal:
  `src/core/topic_registry.cpp:68-77` (the `m_nodes.push_back(full)` at `:76`).
- `TopicRegistry::activeNodes()` returns the **entire** `m_nodes` vector unfiltered:
  `src/core/topic_registry.cpp:188-191`.
- The flush prints one `NODE` line per element every window:
  `src/probe/interposers.cpp:94-96`.

There is no shutdown/teardown signal wired in, and the interposer even **drops** the `node_handle`
it already receives (`ros_trace_rcl_node_init(node_handle, rmw_handle, name, ns)` →
`onNodeInit(name, ns)`, `src/probe/interposers.cpp:122`), so a node can't be correlated with its
publishers/subscriptions today.

## Constraint: there is no node-teardown tracepoint on Humble

The obvious fix — hook node destruction and remove the entry — is **not available**. The
`ros2_tracing` / `tracetools` instrumentation layer on Humble exposes **initialization** and
**runtime** tracepoints only; there is **no** `*_fini` / `*_destroy` / shutdown tracepoint for
nodes (or publishers/subscriptions/timers/services). Verified against the Humble header
`ros2/ros2_tracing @ humble : tracetools/include/tracetools/tracetools.h` — all 30 declared
tracepoints are `*_init` (e.g. `rcl_node_init`, `rcl_publisher_init`,
`rclcpp_subscription_init`, ...) or runtime (`rcl_publish`, `callback_start`, `callback_end`,
`rmw_take`, executor events, lifecycle transition). None signal destruction.

Implication: liveness cannot be observed **directly**. It must either be (a) reframed to what we can
truthfully claim, or (b) **inferred** from a runtime signal we *do* see.

## Options considered

### Option A — honest "seen since start" set + rename the claim
Dedup on insert, keep the list monotonic, and change the wording from "liveness" to
"nodes seen since start".

- Pros: trivial, zero risk, 100% accurate *for what it claims*, no hot-path / interposer / ROS
  changes, existing tests untouched.
- Cons: does **not** fix the headline defect — a crashed node still prints `NODE /x` forever. It
  renames the problem rather than solving it, and drops the product's "which nodes are alive"
  promise.

### Option B — activity-based liveness (chosen)
Infer liveness from the runtime signal the probe already collects: **topic traffic**. A node is
*active* in a window if any topic it declared (as publisher or subscriber) carried >=1 message that
window; a node with **zero** observed traffic for **K consecutive windows** is reported *quiet* and
drops out of `activeNodes()`.

- Pros: actually fixes the defect (the list is no longer monotonic — dead/quiet nodes disappear);
  reuses data we already have; keeps the promise honest as *activity*.
- Cons / accepted tradeoffs:
  - **False negatives:** a genuinely-alive node with no topic traffic (pure timer/service node, or
    an event-driven publisher that is idle) reads as *quiet*. This is inherent — without topic
    traffic we have no runtime signal for that node. Documented as a limitation; framed as
    "quiet / no observed traffic", **not** "dead".
  - **Shared-counter attribution (interacts with issue #1):** counters are keyed by topic *name*,
    so two endpoints of the same topic in one process share a counter. If node A and node B both
    touch `/scan` in one process and `/scan` flows, both are marked active. In the common
    pub-here/sub-here case both really *are* active; the only genuine false-positive is two
    *publishers* of the same topic in one process where one is dead — narrow. Fixing #1 (split
    `sTopicCounter` into `{pub_inter, recv_inter, recv_intra}`) would also sharpen this.

### Why B, and why attribute at the window boundary (not on the hot path)
The issue is literally titled *"monotonic, never decremented"*; Option A leaves it monotonic.
Option B is the only one that makes `activeNodes()` shrink when a node goes away.

CONTRIBUTING.md is emphatic that **the hot path stays cheap** ("`onPublish` / `onCallbackStart` run
per message. No allocations, no string hashing, no global locks — per-endpoint atomics + the
thread-local cache only"). So we do **not** add per-node atomics or a second thread-local to the hot
path. Instead we link `node -> its topic counters` at *init* time (low frequency), and age each
node's idle-window counter inside `snapshot()` — which is already the once-per-window boundary that
reads and resets every counter under the write lock. Net hot-path cost of this change: **zero**.

## Chosen approach — mechanics

1. **Thread `node_handle` through** the three init hooks (it is already an argument of the
   tracepoints, just discarded today):
   `onNodeInit(node_handle, name, ns)`, `onPublisherInit(pub_handle, node_handle, topic)`,
   `onSubscriptionInit(sub_handle, node_handle, topic)`. `node_handle` is an opaque `const void*`,
   so the core stays ROS-free (CONTRIBUTING "core stays pure").
2. **Node records, deduped by full name**, owned in insertion order:
   `struct sNode { std::string name; uint32_t idle_windows; std::vector<sTopicCounter*> counters; }`.
   Re-init of the same name reuses the existing `sNode` (dedup) and resets `idle_windows = 0`.
   `node_handle -> sNode*` lets publisher/subscription init append the topic's counter to the
   owning node's `counters`.
3. **Age liveness in `snapshot()`** (the window tick): while exchanging each counter to 0, record
   which counters saw traffic this window (inter+intra > 0, filtered topics included — a node
   spamming `/rosout` is still alive); then for each node, `idle_windows = 0` if any owned counter
   was active else `idle_windows++`.
4. **`activeNodes()`** returns names where `idle_windows < K`, in insertion order. Fresh nodes start
   at `idle_windows = 0` (a grace window of K after init before they can be declared quiet).
5. **K is configurable** via the constructor (`explicit TopicRegistry(uint32_t quiet_windows =
   kDefaultQuietWindows)`, default 3) so tests can use a small K; the probe uses the default.

Well-defined semantic, stated for the tests and docs:
> `activeNodes()` returns the set of distinct initialized nodes that had at least one observed
> publish/receive event within the last K windows. A node silent for K consecutive windows is
> omitted (reported quiet). Re-init of a name does not duplicate; it refreshes the node.

## Test plan (red -> green)
- `DuplicateNodeInitDeduped` — two `onNodeInit` for the same name → `activeNodes().size() == 1`
  (fails today: 2).
- `NodeGoesQuietAfterKWindows` — node + publisher + traffic → active; then K silent `snapshot()`
  windows → node drops out (fails today: sticky forever).
- `NodeReactivatesOnNewTraffic` — a quiet node that publishes again becomes active again.
- `SubscriberNodeCountsAsActivity` — receive-side (callback) traffic keeps a node active.
- Existing `NodeTracking` updated to the new signature and still green; all other 8 unit tests
  unchanged and green. TSan on the suite (snapshot interacts with the concurrent hot path).
- README "Limitations" gains an honest note that liveness is traffic-derived.

## References
- `ros2/ros2_tracing` (humble): `tracetools/include/tracetools/tracetools.h` — full tracepoint list,
  init + runtime only, no teardown tracepoint.
- ROS 2 tracing docs: <https://docs.ros.org/en/humble/p/tracetools/>
- ros2_tracing paper: "ros2_tracing: Multipurpose Low-Overhead Framework for Real-Time Tracing of
  ROS 2" (arXiv:2201.00393) — init vs runtime instrumentation-point taxonomy.
- Internal: KNOWN_ISSUES.md #2 and #1 (shared-counter interaction), CONTRIBUTING.md (hot-path /
  core-purity invariants), TESTING_PLAN.md #2 rows.

# Testing plan

Goal: every item in [KNOWN_ISSUES.md](KNOWN_ISSUES.md) gets a regression test that **fails on the
current code and passes after the fix**, plus coverage the current suite lacks. Layered: pure-C++
unit (fast, no ROS) → ROS integration (real nodes under `LD_PRELOAD`) → sanitizers/stress → CI.

Current suite: `test/unit/test_topic_registry.cpp` (9 core tests), `test/integration/
test_probe_integration.py` (intra + inter, separate processes). Good base — gaps below.

## Testability refactors needed first

Some issues aren't unit-testable against today's shape. Small, behaviour-preserving refactors unlock
them:

1. **Extract env parsing** → `double parsePeriodSeconds(const char* raw, double def) noexcept`
   (issue #5). Lets a unit test hit bad/negative/empty input without spawning a process.
2. **Split counters** in `sTopicCounter` → `{pub_inter, recv_inter, recv_intra}` (issue #1). The
   double-count test asserts against distinct fields.
3. **Expose test hooks** on `TopicRegistry` (compiled only under `BUILD_TESTING`): a counter of
   write-lock acquisitions (or resolution attempts) so issue #3's "no lock storm" is *assertable*,
   and `nodeCount()` for dedup checks.

## Unit tests (core, no ROS) — `test/unit/`

### Issue #1 — double counting (High)
- `SameProcessPubAndSubDoNotDoubleCount`: init a publisher and a resolvable subscriber for the same
  topic in one registry; N publishes + N inter-receives. **Expect** `pub_inter == N` and
  `recv_inter == N` as *separate* values (today they collapse to `inter == 2N`).
- `MultipleSubscribersSameTopic`: one publisher, two resolvable callbacks on the same topic; assert
  receive count reflects deliveries per the documented semantic, publish count untouched.
- `PublishSideIndependentOfReceive`: publishes only (no callbacks) → `recv_inter == 0`; callbacks
  only → `pub_inter == 0`.

### Issue #3 — unresolvable callbacks / lock storm (Medium)
- `TimerCallbackResolvedOnce`: call `onCallbackStart` for a callback never linked to a subscription,
  many times; assert the write-lock/resolution-attempt counter increments **at most once** (negative
  cache), not once per call.
- `NegativeCacheDoesNotLeak`: repeated unknown callbacks don't grow `m_cb_to_counter` unbounded
  beyond one sentinel per distinct callback.

### Issue #2 — node liveness (Medium)
- `DuplicateNodeInitDeduped`: two `onNodeInit` for the same name → `activeNodes()` size 1.
- `NodeLivenessSemantics`: pin down whatever semantic is chosen (seen-set vs activity-window). If
  activity-based: a node silent for K windows drops out; if seen-set: rename and assert it persists
  (and update the README claim).

### Issue #5 — env parsing (Low)
- `ParsePeriod`: table test — `"5.0"→5.0`, `""→def`, `"abc"→def`, `"-1"→def`, `"0"→def`,
  `"1e3"→1000`. Must be `noexcept` (no throw escapes).

### Existing behaviours worth locking down
- Snapshot Hz math across window sizes (have partial coverage).
- Lazy resolution when init arrives out of order (already covered — keep).
- Filtered topics still reset counters (already covered — keep).
- Registry-id scoping: construct a registry, drive the thread-local cache, destroy it, construct a
  new one that may reuse the address → the new one must not read the old counter (the UAF guard;
  add an explicit test, it's the subtlest invariant in the code).

## Integration tests (real ROS under `LD_PRELOAD`) — `test/integration/`

Extend `test_probe_integration.py` (parametrize the existing `stats_probe_test_nodes`):

- **`test_single_process_inter_no_double_count`** (issue #1, the headline): one process running a
  publisher **and** subscriber for a topic, `use_intra_process_comms` **off**. Assert reported rate
  ≈ true publish rate (within 10%), **not** ~2×. This is the test that would have caught the bug.
- **`test_composed_container`**: two nodes in one process via a container; verify per-topic rates are
  correct with intra on *and* off.
- **`test_accuracy_known_rate`**: publisher at exactly R Hz → `|measured − R|/R < 5%` on a full
  window (see [ALTERNATIVES.md](ALTERNATIVES.md#accuracy-benchmark-missing-today--add-it)).
- **`test_per_process_output_file`** (issue #4): two processes with the **default** output path →
  assert no interleaved/garbled window blocks (or, after fix, that each process gets its own pid file).
- **`test_node_death`** (issue #2): start a node, confirm `NODE` appears, kill it, confirm the chosen
  liveness semantic (disappears, or is documented as sticky).
- **`test_probe_survives_bad_env`** (issue #5): launch with `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD=xyz`
  → process must **not** crash; probe falls back to default.
- **`test_timer_heavy_node`** (issue #3): node with a high-rate timer + a subscription → subscription
  Hz still correct and no pathological slowdown (coarse wall-clock guard).

Harden the existing tests: assert on the **last complete** window (skip the partial first/last),
and give explicit failure dumps (they already print the file — keep that).

## Sanitizers & stress

The hot path is concurrent (atomics + `shared_mutex` + thread-local cache) — races won't show in
functional tests. Add sanitizer builds:

- **ThreadSanitizer** job: build the core + unit tests with `-fsanitize=thread`, run
  `ConcurrentPublishExactTotal` and a new multi-threaded mixed publish/receive test. Catches data
  races on the maps and the thread-local/`m_id` interaction.
- **ASan/UBSan** job: catches the UAF class the registry-id guard defends against, and any misuse.
- **Stress**: extend `bench/stress_nodes.cpp` into a soak (many topics, mixed intra/inter, minutes)
  asserting no counter drift, no unbounded map growth, stable RSS.
- **Property/fuzz (optional)**: randomize the order of the five init calls + interleaved
  `callback_start` → resolution must never crash and must converge to correct counts once the chain
  completes.

## CI

`.github/workflows/ci.yml` currently runs `industrial_ci` on Humble. Add:

- A **sanitizer matrix leg** (TSan, ASan/UBSan) on the pure core — fast, no ROS container needed.
- Keep Humble; consider adding a **rolling** leg so behaviour against the post-#3130 built-in stats
  is observed (and so the intra claim's distro scope stays honest).
- Gate merges on the accuracy + double-count integration tests once they land.

## Coverage → issue traceability

| Issue | Unit | Integration | Sanitizer/stress |
|---|---|---|---|
| #1 double count | `SameProcessPubAndSub…`, `PublishSideIndependent…` | `test_single_process_inter_no_double_count`, `test_composed_container`, `test_accuracy_known_rate` | soak drift |
| #2 node liveness | `DuplicateNodeInitDeduped`, `NodeLivenessSemantics` | `test_node_death` | — |
| #3 lock storm | `TimerCallbackResolvedOnce`, `NegativeCacheDoesNotLeak` | `test_timer_heavy_node` | TSan |
| #4 shared file | — | `test_per_process_output_file` | — |
| #5 env parse | `ParsePeriod` | `test_probe_survives_bad_env` | — |
| UAF guard | registry-id scoping test | — | ASan |

# Issue #5 — `std::stod` on a bad env var can call `std::terminate`

Severity: **Low** (robustness). Tracked as item #5 in
[../KNOWN_ISSUES.md](../KNOWN_ISSUES.md).

## Problem

`ros2_pulse` is an `LD_PRELOAD` shim: it exports the same `ros_trace_*` symbols as
`libtracetools.so` and rclcpp calls them unconditionally. The very first tracepoint that fires
constructs the process-wide `ProbeRuntime` Meyers singleton, which reads two environment variables
and parses the publish period out of one of them.

If the operator sets `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` to anything non-numeric, the parse
throws, and the exception unwinds *out of an `extern "C"` tracepoint back into rclcpp*, which is not
compiled to expect it. The result is `std::terminate` — a monitoring probe taking down the process
it is only supposed to observe.

## Root cause (file:line)

`src/probe/interposers.cpp:68` (before fix):

```cpp
ProbeRuntime()
    : m_out_path(getEnv("ROS_TOPIC_STATS_OUTPUT_FILE", "/root/ssd2tb/logs/topic_freq.log")),
      m_period_s(std::stod(getEnv("ROS_TOPIC_STATISTICS_PUBLISH_PERIOD", "5.0"))) {}
```

`std::stod` (`src/probe/interposers.cpp:68`) throws:

- `std::invalid_argument` when no conversion could be performed (e.g. `"abc"`, `""`),
- `std::out_of_range` when the value is out of `double` range.

`getEnv` (`src/probe/interposers.cpp:38-41`) only guards against a missing / empty variable by
substituting the default *string* `"5.0"`; it does no numeric validation, so a set-but-garbage
value flows straight into `std::stod`.

The construction happens inside `ProbeRuntime::instance()`, called from `ros_trace_rcl_node_init`
and every other hot-path interposer (`src/probe/interposers.cpp:121`, `:130`, `:139`, ...). So the
throw crosses the `extern "C"` boundary.

### Why a throw across the tracepoint boundary is fatal

An exception must not propagate across an execution / `extern "C"` boundary. The unwinding logic on
the two sides can be incompatible, and even when it is not, the caller (rclcpp) has no matching
handler, so the runtime calls `std::terminate`. This is codified as **SEI CERT ERR59-CPP** ("Do not
throw an exception across execution boundaries"). For a preload probe the rule is absolute: nothing
we do may be able to abort the host process.

## Options considered

| Option | noexcept? | Allocates? | Locale | Rejects trailing garbage | inf/nan |
|--------|-----------|------------|--------|--------------------------|---------|
| `try/catch` around `std::stod` | only if wrapped | yes (`std::string`) | locale-dependent (calls `strtod`) | no — parses a prefix, `"5abc"` -> 5 | parses them; must reject manually |
| `std::strtod` (C) | yes (no throw) | no | locale-dependent decimal point | via `endptr` check | parses them; must reject manually |
| `std::from_chars` (`<charconv>`, C++17) | **yes, by contract** | **no** | **locale-independent (always `.`)** | via `ptr == last` check | parses them; must reject manually |

Notes on each:

- **`try/catch` `std::stod`** — smallest diff, but leans on exceptions for ordinary control flow,
  heap-allocates a `std::string`, honours the process locale (a comma decimal separator changes
  behaviour), and silently accepts a numeric *prefix* (`"5s"` -> `5`). We would still need explicit
  `<= 0` / `isfinite` checks on top.
- **`std::strtod`** — no exceptions, but locale-sensitive and fiddly: reset `errno`, compare
  `endptr` against the start (no conversion) and against the end (trailing junk), and check `errno
  == ERANGE`.
- **`std::from_chars`** — designed for exactly this: it is `noexcept`, does not allocate, is
  locale-independent, and reports precisely via `{ptr, ec}` whether the parse succeeded and how far
  it got. It does *not* skip leading whitespace and only accepts a leading minus (not plus), so we
  own the whitespace policy explicitly. Floating-point `from_chars` follows the `strtod` pattern, so
  it still parses `"inf"`/`"nan"` as valid — we reject those with `std::isfinite`.

## Chosen approach

Add a small `noexcept` free function to the **core** (no ROS dependency, unit-testable in
isolation):

```cpp
// include/ros2_pulse/core/env_config.hpp
double parsePeriodSeconds(const char* raw, double def) noexcept;
```

Implemented with `std::from_chars` and these rules — anything that fails returns `def`:

1. `raw == nullptr` (variable unset) -> `def`.
2. Trim leading/trailing ASCII whitespace; if nothing remains (`""`, all spaces) -> `def`.
3. `std::from_chars` over the trimmed range; require `ec == std::errc{}` **and** `ptr` at the
   trimmed end (rejects `"5abc"`, `"1,5"`).
4. `std::isfinite(value)` must hold (rejects `nan`, `inf`, `-inf`).
5. `value > 0` (a period `<= 0` is meaningless and would make the flush timer spin) -> else `def`.

Whitespace policy: surrounding whitespace is tolerated (`"  2.5 "` -> `2.5`) because operators set
these via shell exports where a stray space is easy; interior/garbage characters are rejected.

`interposers.cpp` then reads the raw variable with `std::getenv` and hands it straight to the parser:

```cpp
m_period_s(parsePeriodSeconds(std::getenv("ROS_TOPIC_STATISTICS_PUBLISH_PERIOD"), 5.0))
```

Because the function is `noexcept` and `from_chars` never throws, no exception can ever leave it —
the tracepoint boundary is safe by construction, and the parser gets a fast unit-test table.

### Rationale

`from_chars` gives us the strongest guarantee (`noexcept` in the type system, not just by
convention) with no allocation and no locale surprises — the property that actually matters at an
`LD_PRELOAD`/tracepoint boundary. It is available in GCC 11 (Ubuntu 22.04 / ROS 2 Humble's default
toolchain), which is the build target; verified locally on `g++ 11.4.0`.

## References

- SEI CERT C++ — ERR59-CPP, "Do not throw an exception across execution boundaries":
  https://wiki.sei.cmu.edu/confluence/display/cplusplus/ERR59-CPP.+Do+not+throw+an+exception+across+execution+boundaries
- cppreference — `std::from_chars`: https://en.cppreference.com/w/cpp/utility/from_chars
- cppreference — `std::stof/std::stod` (throwing behaviour): https://en.cppreference.com/w/cpp/string/basic_string/stof
- cppreference — `std::strtod` (locale + inf/nan handling): https://en.cppreference.com/w/cpp/string/byte/strtof
- D. Lemire, "Parsing floats in C++: benchmarking strtod vs. from_chars":
  https://lemire.me/blog/2020/09/10/parsing-floats-in-c-benchmarking-strtod-vs-from_chars/
- Microsoft C++ Team Blog, "Exception Boundaries":
  https://devblogs.microsoft.com/cppblog/exception-boundaries/

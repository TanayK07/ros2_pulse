# Issue #14 — Probe .so exports everything; lint deps declared but never wired

Research note for the fix on branch `fix/build-hygiene`.
Addresses [`KNOWN_ISSUES.md` #14](../KNOWN_ISSUES.md) (Low, build).

## Problem

**(a) Symbol pollution.** `libros2_pulse.so` is LD_PRELOADed into *every process on the robot*.
It currently exports **74** dynamic symbols; only the **7** `ros_trace_*` interposers are its
contract. The rest — `TopicRegistry`, `Timer`, `formatWindow`, parser helpers, weak template
instantiations — leak generic C++ names into every process's dynamic symbol namespace. Risks:
ODR/interposition collisions with unrelated code (a preloaded library is precisely the place
this bites), plus a fatter dynamic symbol table on the interposition lookup path.

**(b) Dead lint manifest weight.** `package.xml` declares `ament_lint_auto` /
`ament_lint_common` as test deps, but `CMakeLists.txt` never calls
`ament_lint_auto_find_test_dependencies()` — the lint suite has never run, giving a false sense
of coverage and pulling unused dependencies into test environments.

## Fix

**(a)** Compile the probe and core with `-fvisibility=hidden -fvisibility-inlines-hidden`
(ament lane; the standalone lane builds no shared library) and mark the seven interposers
`__attribute__((visibility("default")))` via a small `ROS2_PULSE_EXPORT` macro. Static-lib
linking of the core into unit tests is unaffected (visibility only matters for the dynamic
symbol table).

**(b)** Drop the two lint deps rather than wiring them: adopting `ament_lint_common` wholesale
would impose uncrustify/cpplint style churn across the codebase for no defect-finding value the
sanitizer lanes + test suite don't already provide. If a linter is adopted later it should be a
deliberate style decision, not a manifest leftover. (KNOWN_ISSUES #14 allowed either direction.)

## Regression test (red on current `main`)

`integration: test_symbols.py::test_only_interposers_exported` — `nm -D --defined-only` on the
installed .so; every exported symbol must match the allowlist (`ros_trace_*` + the linker-script
standards `_init/_fini/_edata/_end/__bss_start`). Red today: 74 exports incl. mangled core
symbols. This also pins the contract so a future source addition can't silently re-leak.

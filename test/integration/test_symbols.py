# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Symbol-surface test (KNOWN_ISSUES #14): the LD_PRELOAD library's dynamic-symbol contract is
# exactly the ros_trace_* interposers, nothing else may leak into every process on the robot.

import os
import re
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_harness as ph  # noqa: E402  # pyright: ignore[reportMissingImports]

# The interposer contract, plus the standard linker-script symbols every .so carries.
ALLOWED = re.compile(r"^(ros_trace_\w+|_init|_fini|_edata|_end|__bss_start)$")


def test_only_interposers_exported():
    so, _ = ph.probe_paths()
    out = subprocess.check_output(["nm", "-D", "--defined-only", so], text=True)
    exported = [line.split()[-1] for line in out.splitlines() if line.strip()]
    assert exported, "nm returned nothing, wrong path?"
    leaked = [s for s in exported if not ALLOWED.match(s)]
    assert not leaked, (
        f"{len(leaked)} non-contract symbols leak from the preload library "
        f"(KNOWN_ISSUES #14), e.g.:\n  " + "\n  ".join(leaked[:10]))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
